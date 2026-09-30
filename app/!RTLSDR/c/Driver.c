/* Driver.c -- shared RTL-SDR device bringup layer, extracted from
   RTLSDR.c (see docs/PLAN.md for the milestone 1-4 diagnostic history
   this code was proven against on real hardware). Every function below
   moved verbatim from RTLSDR.c -- no behaviour change, only relocation,
   so that !RTLSDRView can share it instead of duplicating it.

   C89 only (Norcroft): all declarations at top of block, no //
   comments. */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "kernel.h"
#include "RTLSDR.h"
#include "Driver.h"

#define SCRAP_PATH "<Wimp$ScrapDir>.rtlsdr"

#define SWI_DeviceFS_CallDevice 0x42744
#define USB_CALL_CONTROL_REQUEST ((int)0x80000000u) /* (1<<31) + 0 */

/* ---- register read/write over DeviceFS_CallDevice ----
   Mirrors librtlsdr's rtlsdr_read_reg/write_reg (src/librtlsdr.c), which use
   libusb_control_transfer with:
     read:  bmRequestType=CTRL_IN,  bRequest=0, wValue=addr, wIndex=block<<8
     write: bmRequestType=CTRL_OUT, bRequest=0, wValue=addr, wIndex=(block<<8)|0x10 */

/* device_name e.g. "usb7", as found by find_device() above (caller must
   format "usb" + number). Returns 0 on success, non-zero on error.
   Declared in RTLSDR.h -- shared with c/R82XX.c (the tuner driver), which
   uses it directly rather than duplicating SWI/DeviceFS_CallDevice
   plumbing. */
int usb_ctrl_transfer(const char *device_name, int bm_request_type,
                       int b_request, int w_value, int w_index,
                       int w_length, void *data)
{
    _kernel_swi_regs regs;
    _kernel_oserror *err;

    regs.r[0] = USB_CALL_CONTROL_REQUEST;
    regs.r[1] = (int)device_name;
    regs.r[3] = (bm_request_type & 0xFF) |
                ((b_request & 0xFF) << 8) |
                ((w_value & 0xFFFF) << 16);
    regs.r[4] = (w_index & 0xFFFF) | ((w_length & 0xFFFF) << 16);
    regs.r[5] = (int)data;
    regs.r[6] = 0; /* block until command completion/failure/timeout */

    err = _kernel_swi(SWI_DeviceFS_CallDevice, &regs, &regs);
    if (err != NULL) {
        fprintf(stderr, "usb_ctrl_transfer: %s\n", err->errmess);
        return -1;
    }
    return 0;
}

/* Bytes currently waiting unread in the DeviceFS stream buffer of an open
   USB bulk stream (DeviceCall_USB_BufferSpace, reason 0x80000002: R2 =
   the fileswitch handle from os_find_open(); returns R3 = buffer size,
   R4 = free space). -1 if the call fails. Cheap, moves no data. */
#define USB_CALL_BUFFER_SPACE ((int)0x80000002u)

int usb_stream_used_bytes(const char *device_name, int stream_handle)
{
    _kernel_swi_regs regs;
    _kernel_oserror *err;

    regs.r[0] = USB_CALL_BUFFER_SPACE;
    regs.r[1] = (int)device_name;
    regs.r[2] = stream_handle;
    regs.r[3] = 0;
    regs.r[4] = 0;
    err = _kernel_swi(SWI_DeviceFS_CallDevice, &regs, &regs);
    if (err != NULL) {
        return -1;
    }
    return regs.r[3] - regs.r[4];
}

/* Declared in RTLSDR.h -- also shared with c/R82XX.c, for GPIO control
   (SYS block). */
int rtlsdr_read_reg(const char *dev, int block, int addr, int len)
{
    unsigned char data[2];
    int w_index;

    w_index = block << 8;
    data[0] = 0;
    data[1] = 0;
    if (usb_ctrl_transfer(dev, CTRL_BMREQ_VENDOR_IN, 0, addr, w_index,
                           len, data) != 0) {
        return -1;
    }
    if (len == 1) {
        return data[0];
    }
    return data[0] | (data[1] << 8);
}

int rtlsdr_write_reg(const char *dev, int block, int addr, int val,
                      int len)
{
    unsigned char data[2];
    int w_index;

    w_index = (block << 8) | 0x10;

    /* THE ROOT CAUSE of this project's long-running bulk-IN throughput
       mystery (found 2026-09-29 by comparing raw wire bytes for the very
       first USB-block writes against a real Linux capture, after every
       host-side/driver-side hypothesis had been eliminated).

       A 2-byte write must put the HIGH byte in data[0] and the LOW byte
       in data[1] -- exactly what upstream librtlsdr does:
           if (len == 1) data[0] = val & 0xff; else data[0] = val >> 8;
           data[1] = val & 0xff;
       This function used to pack them the other way round
       (data[0] = low byte, data[1] = high byte), which is the reverse
       of what the RTL2832U expects on the write path. (rtlsdr_demod_
       write_reg() below always packed correctly, which is why every
       demod/tuner register worked and this went unnoticed -- only the
       USB-block registers were affected, and only when len == 2.)

       Effect of the bug on the wire, both confirmed against a real Linux
       capture of the same device:
         USB_EPA_MAXPKT (0x2158) <- 0x0002:
             Linux: 00 02 (= 512, the High-Speed bulk max packet size)
             here:  02 00 (= 2!) -- the bulk-IN endpoint was told its max
                    packet size is TWO BYTES, so every bulk packet the
                    device sent was exactly 2 bytes, i.e. a "short"
                    packet from the host's point of view (512-byte
                    endpoint descriptor), ending each transfer after 2
                    bytes. That single fact explains: actual_len==2 with
                    zero variance across 500 DWC2 completions,
                    independence from chunk size / DeviceFS buffer size /
                    sample rate, the ~30,000x bytes-per-transfer gap vs.
                    Linux, and the original "padding" saga (every read
                    only ever had ~2 genuinely fresh bytes).
         USB_EPA_CTL (0x2148) <- 0x1002:
             Linux: 10 02 (stall + reset EPA)
             here:  02 10 (different, unintended bits set)
       (Values 0x0000 are byte-symmetric, so the FIFO-release write was
       never affected -- which is part of why streaming "worked" at all.) */
    if (len == 1) {
        data[0] = (unsigned char)(val & 0xFF);
    } else {
        data[0] = (unsigned char)((val >> 8) & 0xFF);
    }
    data[1] = (unsigned char)(val & 0xFF);
    return usb_ctrl_transfer(dev, CTRL_BMREQ_VENDOR_OUT, 0, addr, w_index,
                              len, data);
}

/* ---- milestone 1: find the dongle's USBnnn device number ---- */

/* Runs a command with output redirected to a scrap file, and returns a
   malloc'd buffer with the file's contents (caller frees), or NULL if the
   command produced nothing. */
static char *run_and_capture(const char *cmd)
{
    char oscli_buf[256];
    int rc;
    FILE *f;
    long size;
    char *buf;
    size_t got;

    /* _kernel_oscli returns an int, not an error pointer: >=0 success,
       -1 failed/no OS error, _kernel_ERROR (-2) failed with an OS error
       retrievable via _kernel_last_oserror() (not needed here -- NULL
       return is enough for our purposes). */
    sprintf(oscli_buf, "%s { > %s }", cmd, SCRAP_PATH);
    rc = _kernel_oscli(oscli_buf);
    if (rc == _kernel_ERROR) {
        return NULL;
    }

    f = fopen(SCRAP_PATH, "rb");
    if (f == NULL) {
        return NULL;
    }

    fseek(f, 0, SEEK_END);
    size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (size <= 0) {
        fclose(f);
        return NULL;
    }

    buf = (char *)malloc((size_t)size + 1);
    if (buf == NULL) {
        fclose(f);
        return NULL;
    }

    got = fread(buf, 1, (size_t)size, f);
    buf[got] = '\0';
    fclose(f);
    return buf;
}

/* Finds the leading integer in a *USBDevices line, e.g. "  6   1   6  0/ 0
   RTLSDRBlog Blog V4" -> 6. Returns -1 if the line has no leading number
   (e.g. the header row). Mirrors BASIC's VAL(), which skips leading
   whitespace and parses leading digits. */
static int leading_int(const char *line)
{
    while (*line == ' ' || *line == '\t') {
        line++;
    }
    if (*line < '0' || *line > '9') {
        return -1;
    }
    return atoi(line);
}

/* Runs *USBDevices and scans its output for a description matching one of
   our known dongle-description substrings, returning the "No." column
   value of the first match, or -1 if none found. This is the same
   approach tools/probe.bas uses, confirmed against real *USBDevices
   output -- description text, not hex VID/PID, which isn't in this
   command's output at all.

   Prints the raw captured text (or says clearly if capture itself failed,
   vs. capture succeeding but nothing matching) -- this is diagnostic by
   design. */
int find_device(void)
{
    char *text;
    char *line;
    int result;

    text = run_and_capture("USBDevices");
    if (text == NULL) {
        printf("(run_and_capture(\"USBDevices\") returned NULL -- the "
               "OSCLI call or scrap file read/write itself failed, before "
               "any text-matching happened)\n");
        return -1;
    }

    printf("--- raw *USBDevices capture (%lu bytes) ---\n",
           (unsigned long)strlen(text));
    printf("%s", text);
    printf("--- end capture ---\n");

    /* RISC OS text files are CR-terminated, not LF -- splitting on "\n"
       alone found zero delimiters, so the whole buffer became one "line"
       starting with the non-numeric header row ("No. Bus Dev Class
       Description..."), and leading_int() correctly returned -1 for that,
       even though strstr() had already matched "RTLSDR" somewhere deep in
       the blob. Split on both "\r" and "\n" to handle either convention. */
    result = -1;
    line = strtok(text, "\r\n");
    while (line != NULL && result < 0) {
        if (strstr(line, "RTLSDR") != NULL ||
            strstr(line, "RTL2832") != NULL ||
            strstr(line, "Realtek") != NULL) {
            result = leading_int(line);
        }
        line = strtok(NULL, "\r\n");
    }

    free(text);
    return result;
}

/* ---- milestone 3: baseband init + R828D tuner presence check ----
   Ported verbatim (register addresses/values, not just behaviour) from
   the rtlsdr-blog fork's rtlsdr_init_baseband/rtlsdr_demod_read_reg/
   rtlsdr_demod_write_reg/rtlsdr_set_fir in src/librtlsdr.c -- see
   docs/PLAN.md for the fetched source this was transcribed from. */

/* Demod registers use a different address encoding than plain
   rtlsdr_read_reg/write_reg (RTLSDR.h): addr becomes wValue as
   (addr<<8)|0x20, and "page" (not block) becomes wIndex -- 0x10|page for
   writes. */
int rtlsdr_demod_read_reg(const char *dev, int page, int addr, int len)
{
    unsigned char data[2];
    int w_value;

    w_value = (addr << 8) | 0x20;
    data[0] = 0;
    data[1] = 0;
    if (usb_ctrl_transfer(dev, CTRL_BMREQ_VENDOR_IN, 0, w_value, page,
                           len, data) != 0) {
        return -1;
    }
    return (data[1] << 8) | data[0];
}

int rtlsdr_demod_write_reg(const char *dev, int page, int addr, int val,
                            int len)
{
    unsigned char data[2];
    int w_value;
    int w_index;
    int rc;

    w_value = (addr << 8) | 0x20;
    w_index = 0x10 | page;

    if (len == 1) {
        data[0] = (unsigned char)(val & 0xFF);
    } else {
        data[0] = (unsigned char)((val >> 8) & 0xFF);
    }
    data[1] = (unsigned char)(val & 0xFF);

    rc = usb_ctrl_transfer(dev, CTRL_BMREQ_VENDOR_OUT, 0, w_value, w_index,
                            len, data);

    /* Upstream does a dummy read after every demod write (register 0x01
       on page 0x0a) -- reproduced verbatim even though upstream's own
       comment doesn't explain why. */
    rtlsdr_demod_read_reg(dev, 0x0a, 0x01, 1);

    return rc;
}

/* I2C block (block 6) proxies to a downstream I2C device (the tuner or
   the EEPROM chip) rather than an RTL2832U register directly: wValue is
   the I2C target address itself, not a register number. A read is two
   control transfers -- write the target register number, then read the
   response. */
static int rtlsdr_i2c_write_reg(const char *dev, int i2c_addr, int reg,
                                 int val)
{
    unsigned char data[2];
    int w_index;

    w_index = (RTLSDR_BLOCK_I2C << 8) | 0x10;
    data[0] = (unsigned char)(reg & 0xFF);
    data[1] = (unsigned char)(val & 0xFF);
    return usb_ctrl_transfer(dev, CTRL_BMREQ_VENDOR_OUT, 0, i2c_addr,
                              w_index, 2, data);
}

static int rtlsdr_i2c_read_reg(const char *dev, int i2c_addr, int reg)
{
    unsigned char data[1];
    int w_index_out;
    int w_index_in;

    w_index_out = (RTLSDR_BLOCK_I2C << 8) | 0x10;
    w_index_in = RTLSDR_BLOCK_I2C << 8;

    data[0] = (unsigned char)(reg & 0xFF);
    if (usb_ctrl_transfer(dev, CTRL_BMREQ_VENDOR_OUT, 0, i2c_addr,
                           w_index_out, 1, data) != 0) {
        return -1;
    }

    data[0] = 0;
    if (usb_ctrl_transfer(dev, CTRL_BMREQ_VENDOR_IN, 0, i2c_addr,
                           w_index_in, 1, data) != 0) {
        return -1;
    }
    return data[0];
}

/* FIR coefficients (fir_default[] in librtlsdr) and rtlsdr_set_fir()'s
   exact packing: 8 signed 8-bit values, then 8 signed 12-bit values
   packed 2-per-3-bytes, written to demod page 1 registers 0x1c-0x2b. */
static const int rtlsdr_fir_default[16] = {
    -54, -36, -41, -40, -32, -14, 14, 53,
    101, 156, 215, 273, 327, 372, 404, 421
};

static int rtlsdr_set_fir(const char *dev)
{
    unsigned char fir[20];
    int i;
    int val0, val1;

    for (i = 0; i < 8; i++) {
        fir[i] = (unsigned char)(rtlsdr_fir_default[i] & 0xFF);
    }
    for (i = 0; i < 8; i += 2) {
        val0 = rtlsdr_fir_default[8 + i];
        val1 = rtlsdr_fir_default[8 + i + 1];
        fir[8 + i * 3 / 2] = (unsigned char)((val0 >> 4) & 0xFF);
        fir[8 + i * 3 / 2 + 1] =
            (unsigned char)(((val0 << 4) | ((val1 >> 8) & 0x0F)) & 0xFF);
        fir[8 + i * 3 / 2 + 2] = (unsigned char)(val1 & 0xFF);
    }

    for (i = 0; i < 20; i++) {
        if (rtlsdr_demod_write_reg(dev, 1, 0x1c + i, fir[i], 1) != 0) {
            return -1;
        }
    }
    return 0;
}

/* Verbatim port of rtlsdr_init_baseband() -- same sequence regardless of
   which tuner is fitted (R828D vs R820T2 vs anything else); tuner-specific
   setup happens afterwards. */
void rtlsdr_init_baseband(const char *dev)
{
    int i;

    /* initialize USB */
    rtlsdr_write_reg(dev, RTLSDR_BLOCK_USB, USB_SYSCTL, 0x09, 1);
    rtlsdr_write_reg(dev, RTLSDR_BLOCK_USB, USB_EPA_MAXPKT, 0x0002, 2);
    rtlsdr_write_reg(dev, RTLSDR_BLOCK_USB, USB_EPA_CTL, 0x1002, 2);

    /* poweron demod */
    rtlsdr_write_reg(dev, RTLSDR_BLOCK_SYS, DEMOD_CTL_1, 0x22, 1);
    rtlsdr_write_reg(dev, RTLSDR_BLOCK_SYS, DEMOD_CTL, 0xe8, 1);

    /* reset demod (bit 3, soft_rst) */
    rtlsdr_demod_write_reg(dev, 1, 0x01, 0x14, 1);
    rtlsdr_demod_write_reg(dev, 1, 0x01, 0x10, 1);

    /* disable spectrum inversion and adjacent channel rejection */
    rtlsdr_demod_write_reg(dev, 1, 0x15, 0x00, 1);
    rtlsdr_demod_write_reg(dev, 1, 0x16, 0x0000, 2);

    /* clear both DDC shift and IF frequency registers */
    for (i = 0; i < 6; i++) {
        rtlsdr_demod_write_reg(dev, 1, 0x16 + i, 0x00, 1);
    }

    rtlsdr_set_fir(dev);

    /* enable SDR mode, disable DAGC (bit 5) */
    rtlsdr_demod_write_reg(dev, 0, 0x19, 0x05, 1);

    /* init FSM state-holding register */
    rtlsdr_demod_write_reg(dev, 1, 0x93, 0xf0, 1);
    rtlsdr_demod_write_reg(dev, 1, 0x94, 0x0f, 1);

    /* disable AGC (en_dagc, bit 0) (upstream notes: "seems to have no
       effect") */
    rtlsdr_demod_write_reg(dev, 1, 0x11, 0x00, 1);

    /* disable RF and IF AGC loop */
    rtlsdr_demod_write_reg(dev, 1, 0x04, 0x00, 1);

    /* disable PID filter (enable_PID = 0) */
    rtlsdr_demod_write_reg(dev, 0, 0x61, 0x60, 1);

    /* opt_adc_iq = 0, default ADC_I/ADC_Q datapath */
    rtlsdr_demod_write_reg(dev, 0, 0x06, 0x80, 1);

    /* enable Zero-IF mode (en_bbin), DC cancellation (en_dc_est), IQ
       estimation/compensation (en_iq_comp, en_iq_est) */
    rtlsdr_demod_write_reg(dev, 1, 0xb1, 0x1b, 1);

    /* disable 4.096MHz clock output on pin TP_CK0 */
    rtlsdr_demod_write_reg(dev, 0, 0x0d, 0x83, 1);
}

/* Probes for R820T (0x34) then R828D (0x74) at the shared check register/
   value (0x00 -> 0x69), matching rtlsdr_open()'s tuner-detection loop.
   I2C repeater must be enabled first (demod page 1, reg 0x01, 0x18) and
   disabled afterwards (0x10) -- the tuner sits behind the RTL2832U's I2C
   bus, not reachable directly. Returns 1 for R820T, 2 for R828D, 0 if
   neither responded. */
int rtlsdr_probe_tuner(const char *dev)
{
    int reg;
    int result;

    rtlsdr_demod_write_reg(dev, 1, 0x01, 0x18, 1); /* enable I2C repeater */

    result = 0;
    reg = rtlsdr_i2c_read_reg(dev, R820T_I2C_ADDR, R82XX_CHECK_ADDR);
    if (reg == (int)R82XX_CHECK_VAL) {
        result = 1;
    } else {
        reg = rtlsdr_i2c_read_reg(dev, R828D_I2C_ADDR, R82XX_CHECK_ADDR);
        if (reg == (int)R82XX_CHECK_VAL) {
            result = 2;
        }
    }

    rtlsdr_demod_write_reg(dev, 1, 0x01, 0x10, 1); /* disable I2C repeater */
    return result;
}

/* RTL2832U's own crystal; our confirmed hardware (V4) keeps this default
   rather than R828D_XTAL_FREQ -- see h/RTLSDR.h. Needed as a double here
   (matches upstream's TWO_POW(n) macro, literally "(double)(1ULL<<n)") --
   the intermediate products in the functions below (xtal * 2^22) are
   ~1.2x10^14, which overflows 32-bit integer arithmetic by a factor of
   ~28000, so upstream deliberately does this math in floating point, not
   integer. Confirmed by reading the actual macro definition, not assumed. */
#define RTL_XTAL_FREQ_HZ 28800000.0

/* Sets the demod's IF-frequency registers (page 1, 0x19/0x1a/0x1b).
   Verbatim formula from upstream rtlsdr_set_if_freq(). Exposed (not
   static) so callers can re-sync the demod's digital IF after
   r82xx_set_bandwidth() (R82XX.c) updates the tuner's actual analog
   IF -- see Driver.h and docs/PLAN.md for why leaving these out of
   sync was a real, previously-missed bug. */
int rtlsdr_set_if_freq(const char *dev, unsigned long freq)
{
    long if_freq;
    int rc;

    if_freq = (long)(-(((double)freq * 4194304.0) / RTL_XTAL_FREQ_HZ));

    rc = rtlsdr_demod_write_reg(dev, 1, 0x19, (int)((if_freq >> 16) & 0x3f), 1);
    rc |= rtlsdr_demod_write_reg(dev, 1, 0x1a, (int)((if_freq >> 8) & 0xff), 1);
    rc |= rtlsdr_demod_write_reg(dev, 1, 0x1b, (int)(if_freq & 0xff), 1);
    return rc;
}

/* Verbatim port of the R820T/R828D case in rtlsdr_open()'s tuner-type
   switch (librtlsdr.c) -- runs once, right after tuner presence is
   confirmed and before the tuner's own r82xx_init(). Critically,
   this OVERRIDES rtlsdr_init_baseband()'s zero-IF setting: baseband init
   writes demod 1/0xb1=0x1b ("enable zero-IF"), and this immediately
   writes 1/0xb1=0x1a ("disable zero-IF") instead -- R820T/R828D use a
   real 3.57MHz IF, not zero-IF. Missing this step was the leading
   suspect for milestone 4's "zero bytes ever" result: a demod configured
   for the wrong IF mode has no reason to produce valid samples at all.
   (The EEPROM-triggered forced bias-tee step from the same real function
   is deliberately NOT ported here -- it goes through the same GPIO
   mechanism that's caused repeated hangs, see docs/PLAN.md, and isn't
   needed to test whether data flows at all.) */
int rtlsdr_tuner_postinit(const char *dev)
{
    int rc;

    rc = rtlsdr_demod_write_reg(dev, 1, 0xb1, 0x1a, 1); /* disable zero-IF */
    rc |= rtlsdr_demod_write_reg(dev, 0, 0x08, 0x4d, 1); /* I-only ADC input */
    rc |= rtlsdr_set_if_freq(dev, R82XX_IF_FREQ);
    rc |= rtlsdr_demod_write_reg(dev, 1, 0x15, 0x01, 1); /* spectrum inversion */
    return rc;
}

/* Verbatim (ppm=0 case, our only use): writes 0 to both correction
   registers, per rtlsdr_set_sample_freq_correction(). */
static int rtlsdr_set_freq_correction_zero(const char *dev)
{
    int rc;
    rc = rtlsdr_demod_write_reg(dev, 1, 0x3f, 0, 1);
    rc |= rtlsdr_demod_write_reg(dev, 1, 0x3e, 0, 1);
    return rc;
}

/* Verbatim port of rtlsdr_set_sample_rate() (resample-ratio calc, same
   double-arithmetic reasoning as rtlsdr_set_if_freq above), MINUS the
   dev->tuner->set_bw() call -- no r82xx bandwidth-setter has been ported
   (upstream's is itself a no-op/optional for many R82xx configurations;
   not implementing it is a real gap, not assumed-safe, but secondary to
   proving data flows at all). */
int rtlsdr_set_sample_rate(const char *dev, unsigned long samp_rate)
{
    double ratio_d;
    unsigned long rsamp_ratio;
    int rc;

    if (samp_rate <= 225000UL || samp_rate > 3200000UL ||
        (samp_rate > 300000UL && samp_rate <= 900000UL)) {
        return -1;
    }

    ratio_d = (RTL_XTAL_FREQ_HZ * 4194304.0) / (double)samp_rate;
    rsamp_ratio = (unsigned long)ratio_d;
    rsamp_ratio &= 0x0ffffffcUL;

    rc = rtlsdr_demod_write_reg(dev, 1, 0x9f, (int)(rsamp_ratio >> 16), 2);
    rc |= rtlsdr_demod_write_reg(dev, 1, 0xa1, (int)(rsamp_ratio & 0xffffUL), 2);
    rc |= rtlsdr_set_freq_correction_zero(dev);

    /* reset demod (bit 3, soft_rst) */
    rc |= rtlsdr_demod_write_reg(dev, 1, 0x01, 0x14, 1);
    rc |= rtlsdr_demod_write_reg(dev, 1, 0x01, 0x10, 1);

    return rc;
}

/* ---- milestone 4: bulk streaming ----
   Mirrors librtlsdr's rtlsdr_reset_buffer() -- toggle USB_EPA_CTL to
   clear any stale FIFO state before starting a fresh bulk read session. */
void rtlsdr_reset_buffer(const char *dev)
{
    rtlsdr_write_reg(dev, RTLSDR_BLOCK_USB, USB_EPA_CTL, 0x1002, 2);
    rtlsdr_write_reg(dev, RTLSDR_BLOCK_USB, USB_EPA_CTL, 0x0000, 2);
}

#define SWI_OS_GBPB 0x0C
#define SWI_OS_FIND 0x0D
#define SWI_OS_Args 0x09

/* Raw OS_Find/OS_GBPB, bypassing stdio entirely. tools/probe_stream.bas
   proved fopen()/fread() blocks indefinitely against this DeviceFS
   stream even though data is genuinely available (confirmed: raw
   OS_GBPB reason 4 read real bytes where fread() got nothing) -- so this
   is not a "maybe faster" alternative, it's the mechanism now KNOWN to
   work, replacing one proven not to. See docs/PLAN.md. */

int os_find_open(const char *path)
{
    _kernel_swi_regs regs;
    _kernel_oserror *err;

    regs.r[0] = 0x40; /* open input (read-only) */
    regs.r[1] = (int)path;
    regs.r[2] = 0;
    err = _kernel_swi(SWI_OS_FIND, &regs, &regs);
    if (err != NULL) {
        return 0;
    }
    return regs.r[0];
}

void os_find_close(int handle)
{
    _kernel_swi_regs regs;
    regs.r[0] = 0;
    regs.r[1] = handle;
    _kernel_swi(SWI_OS_FIND, &regs, &regs);
}

#define SWI_OS_Module 0x1E

/* Claims/releases a block of the RMA (OS_Module 6 / 7). Used for buffers
   too big to want in a Wimp task's small application slot (the big USB
   read buffer). Returns NULL if the claim fails. */
unsigned char *os_rma_claim(int size)
{
    _kernel_swi_regs regs;
    _kernel_oserror *err;

    regs.r[0] = 6;
    regs.r[3] = size;
    err = _kernel_swi(SWI_OS_Module, &regs, &regs);
    if (err != NULL) {
        return NULL;
    }
    return (unsigned char *)regs.r[2];
}

void os_rma_free(unsigned char *p)
{
    _kernel_swi_regs regs;

    if (p == NULL) {
        return;
    }
    regs.r[0] = 7;
    regs.r[2] = (int)p;
    _kernel_swi(SWI_OS_Module, &regs, &regs);
}

/* The Wimp app (!RTLSDRView) must not write to stderr -- there is no text
   window for it to land in -- so it switches the messages off and reads
   the last error text back instead. */
static int os_quiet_g = 0;
static char os_last_error_g[64] = "";

void os_set_quiet(int quiet)
{
    os_quiet_g = quiet;
}

const char *os_last_error(void)
{
    return os_last_error_g;
}

/* Returns bytes actually read (0..len), or -1 on error. Matches
   tools/probe_stream.bas's proven-working SYS "OS_GBPB",4,... call. */
int os_gbpb_read4(int handle, unsigned char *buf, int len)
{
    _kernel_swi_regs regs;
    _kernel_oserror *err;

    regs.r[0] = 4;
    regs.r[1] = handle;
    regs.r[2] = (int)buf;
    regs.r[3] = len;
    err = _kernel_swi(SWI_OS_GBPB, &regs, &regs);
    if (err != NULL) {
        strncpy(os_last_error_g, err->errmess, sizeof(os_last_error_g) - 1);
        os_last_error_g[sizeof(os_last_error_g) - 1] = '\0';
        if (!os_quiet_g) {
            fprintf(stderr, "os_gbpb_read4(len=%d): %s\n", len, err->errmess);
        }
        return -1;
    }
    return len - regs.r[3];
}

/* Enables (or disables) non-blocking mode on an already-open stream, so
   os_gbpb_read4() reports HONEST short-read counts instead of the
   default blocking mode's silent zero-padding to the requested size --
   see the long comment in Driver.h and docs/PLAN.md milestone 5 for how
   this was found (real DeviceFS source, not guesswork) and confirmed
   on real hardware (sentinel-fill test: touched-byte count matched
   os_gbpb_read4()'s return value exactly once this was enabled, where
   before it always claimed the full requested size regardless of how
   much data genuinely arrived). OS_Args (SWI &09) reason 9 = IOCtl;
   IOCtl group 0xFF (DeviceFS-specific) reason 1 = "Set/Read
   Non-blocking I/O", data word 1=set/0=unset. */
#define OSARGS_REASON_IOCTL    9
#define IOCTL_GROUP_DEVICEFS   ((int)0xFFu << 16)
#define IOCTL_REASON_NONBLOCK  1
#define IOCTL_WRITE_FLAG       ((int)0x80000000u)

int os_args_set_nonblocking(int handle, int enable)
{
    _kernel_swi_regs regs;
    _kernel_oserror *err;
    int block[2];

    block[0] = IOCTL_WRITE_FLAG | IOCTL_GROUP_DEVICEFS | IOCTL_REASON_NONBLOCK;
    block[1] = enable ? 1 : 0;

    regs.r[0] = OSARGS_REASON_IOCTL;
    regs.r[1] = handle;
    regs.r[2] = (int)block;
    err = _kernel_swi(SWI_OS_Args, &regs, &regs);
    if (err != NULL) {
        fprintf(stderr, "os_args_set_nonblocking: %s\n", err->errmess);
        return -1;
    }
    return 0;
}
