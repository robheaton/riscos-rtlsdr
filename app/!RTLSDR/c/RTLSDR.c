/* RTLSDR.c -- milestone 1 (enumerate & identify) and milestone 2 groundwork
   (register read/write over DeviceFS_CallDevice) for the RISC OS RTL-SDR
   driver. See ../../../docs/PLAN.md for the full project plan and the
   register-level grounding this file implements.

   C89 only (Norcroft): all declarations at top of block, no // comments.

   Register-level control transfer code (usb_ctrl_transfer and friends) is
   built from the documented DeviceFS_CallDevice (SWI &42744) "USB Control
   Request" call (RISC OS PRM vol 2 ch 37, and ROOL's build/Doc/USB in the
   USBDriver source) plus librtlsdr's known register map -- not yet run
   against real hardware.

   find_device() below IS confirmed against a real run of tools/probe.bas
   on a real Pi with a dongle attached: *USBDevices lists it by a plain
   description string ("RTLSDRBlog Blog V4" in that run), not hex VID/PID
   (which don't appear in that output at all), with device numbering
   starting at 1 in the "No." column. This matches that. Target dongle is
   an RTL-SDR Blog V4 (R828D tuner + HF upconverter, not plain R820T2) --
   see docs/PLAN.md. */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "kernel.h"
#include "RTLSDR.h"
#include "R82XX.h"

#define SWI_DeviceFS_CallDevice 0x42744
#define USB_CALL_CONTROL_REQUEST ((int)0x80000000u) /* (1<<31) + 0 */

#define SCRAP_PATH "<Wimp$ScrapDir>.rtlsdr"

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
   design: milestone 1 hasn't been confirmed working from the compiled C
   binary yet (only from tools/probe.bas in BASIC), so if this doesn't
   find the device, we need to see why, not just "not found". */
static int find_device(void)
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

/* ---- milestone 2 groundwork: register read/write over DeviceFS_CallDevice ----
   Mirrors librtlsdr's rtlsdr_read_reg/write_reg (src/librtlsdr.c), which use
   libusb_control_transfer with:
     read:  bmRequestType=CTRL_IN,  bRequest=0, wValue=addr, wIndex=block<<8
     write: bmRequestType=CTRL_OUT, bRequest=0, wValue=addr, wIndex=(block<<8)|0x10
   Not yet called from main() -- wired in once milestone 1 is confirmed
   working on real hardware. */

/* device_name e.g. "usb7", as found by find_device() above (caller must
   format "usb" + number). Returns 0 on success, non-zero on error.
   Not static: R82XX.c (the tuner driver) shares this primitive rather
   than duplicating SWI/DeviceFS_CallDevice plumbing -- declared in
   RTLSDR.h. */
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

/* Not static: shared with c/R82XX.c for GPIO control (SYS block). */
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
    data[0] = (unsigned char)(val & 0xFF);
    data[1] = (unsigned char)((val >> 8) & 0xFF);
    return usb_ctrl_transfer(dev, CTRL_BMREQ_VENDOR_OUT, 0, addr, w_index,
                              len, data);
}

/* ---- milestone 3: baseband init + R828D tuner presence check ----
   Ported verbatim (register addresses/values, not just behaviour) from
   the rtlsdr-blog fork's rtlsdr_init_baseband/rtlsdr_demod_read_reg/
   rtlsdr_demod_write_reg/rtlsdr_set_fir in src/librtlsdr.c -- see
   docs/PLAN.md for the fetched source this was transcribed from. */

/* Demod registers use a different address encoding than plain
   rtlsdr_read_reg/write_reg above: addr becomes wValue as (addr<<8)|0x20,
   and "page" (not block) becomes wIndex -- 0x10|page for writes. */
static int rtlsdr_demod_read_reg(const char *dev, int page, int addr,
                                  int len)
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

static int rtlsdr_demod_write_reg(const char *dev, int page, int addr,
                                   int val, int len)
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
static void rtlsdr_init_baseband(const char *dev)
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
static int rtlsdr_probe_tuner(const char *dev)
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
   Verbatim formula from upstream rtlsdr_set_if_freq(). */
static int rtlsdr_set_if_freq(const char *dev, unsigned long freq)
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
static int rtlsdr_tuner_postinit(const char *dev)
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
   proving data flows at all). Without calling this function at all
   (which milestone 4's first attempt didn't), the RTL2832U's resampler
   was left at whatever its power-up-default ratio is -- quite possibly
   not a valid/running configuration, which independently could explain
   zero bytes ever arriving. */
static int rtlsdr_set_sample_rate(const char *dev, unsigned long samp_rate)
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

static int milestone3_baseband_and_tuner(const char *dev)
{
    int tuner;

    printf("\n--- milestone 3: baseband init + tuner probe ---\n");
    rtlsdr_init_baseband(dev);
    printf("Baseband init sequence sent (USB config, demod reset, FIR "
           "coefficients, SDR/zero-IF mode).\n");

    tuner = rtlsdr_probe_tuner(dev);
    if (tuner == 2) {
        printf("Tuner probe: R828D responded at I2C 0x%02X with check "
               "value 0x%02X -- matches this hardware (RTL-SDR Blog V4).\n",
               R828D_I2C_ADDR, R82XX_CHECK_VAL);
        printf("\nMilestone 3 gate: PASS.\n");
        return 0;
    }
    if (tuner == 1) {
        printf("Tuner probe: found R820T, not R828D -- unexpected for "
               "this dongle (docs/PLAN.md assumed R828D from *USBDevInfo). "
               "Milestone 3 gate: FAIL (wrong tuner detected).\n");
        return 1;
    }
    printf("Tuner probe: neither R820T nor R828D responded.\n");
    printf("\nMilestone 3 gate: FAIL -- tuner not found. Check the "
           "baseband init sequence and I2C repeater enable/disable "
           "against docs/PLAN.md.\n");
    return 1;
}

/* ---- milestone 3.5: full R82XX tuner init + tune to a real frequency ----
   Unlike milestone3_baseband_and_tuner()'s self-contained enable-probe-
   disable cycle, the I2C repeater needs to stay enabled across all of
   r82xx_init()+r82xx_set_freq() -- matches real driver behaviour, which
   enables it once early on and leaves it enabled through normal tuner
   use, not just for a one-off presence check. */
static int milestone_tune_test(const char *dev)
{
    r82xx_t tuner;
    int rc;
    unsigned long test_freq_hz;

    printf("\n--- tuner init + tune-to-100MHz test ---\n");
    test_freq_hz = 100000000UL; /* 100MHz, VHF FM broadcast band */

    rtlsdr_demod_write_reg(dev, 1, 0x01, 0x18, 1); /* enable I2C repeater */

    rc = rtlsdr_tuner_postinit(dev);
    if (rc < 0) {
        printf("rtlsdr_tuner_postinit() FAILED (rc=%d)\n", rc);
        rtlsdr_demod_write_reg(dev, 1, 0x01, 0x10, 1);
        return 1;
    }
    printf("Tuner post-init OK (real-IF mode selected, IF freq set, "
           "spectrum inversion enabled).\n");

    rc = r82xx_init(&tuner, dev);
    if (rc < 0) {
        printf("r82xx_init() FAILED (rc=%d)\n", rc);
        rtlsdr_demod_write_reg(dev, 1, 0x01, 0x10, 1);
        return 1;
    }
    printf("r82xx_init() OK (init array written, filter calibration "
           "code=0x%02X).\n", tuner.fil_cal_code);

    rc = r82xx_set_freq(&tuner, test_freq_hz);
    rtlsdr_demod_write_reg(dev, 1, 0x01, 0x10, 1); /* disable I2C repeater */

    if (rc < 0) {
        printf("r82xx_set_freq(100MHz) FAILED (rc=%d)\n", rc);
        printf("\nTune test: FAIL.\n");
        return 1;
    }
    if (!tuner.has_lock) {
        printf("r82xx_set_freq(100MHz) completed but PLL did not lock.\n");
        printf("\nTune test: FAIL (no PLL lock).\n");
        return 1;
    }

    printf("Tuned to 100MHz: PLL locked.\n");
    printf("\nTune test: PASS -- the tuner can be tuned to a real "
           "frequency. Next: milestone 4 (bulk streaming).\n");
    return 0;
}

/* ---- milestone 4: bulk streaming throughput proof ----
   Mirrors librtlsdr's rtlsdr_reset_buffer() -- toggle USB_EPA_CTL to
   clear any stale FIFO state before starting a fresh bulk read session. */
static void rtlsdr_reset_buffer(const char *dev)
{
    rtlsdr_write_reg(dev, RTLSDR_BLOCK_USB, USB_EPA_CTL, 0x1002, 2);
    rtlsdr_write_reg(dev, RTLSDR_BLOCK_USB, USB_EPA_CTL, 0x0000, 2);
}

#define STREAM_CHUNK_BYTES    RTLSDR_DEFAULT_BUF_LEN /* buffer capacity only */
#define STREAM_TARGET_BPS     4800000UL /* 2.4 MSPS * 2 bytes, ~expected --
                                            not measurable yet, see below */

#define SWI_OS_GBPB 0x0C
#define SWI_OS_FIND 0x0D

/* Raw OS_Find/OS_GBPB, bypassing stdio entirely. tools/probe_stream.bas
   proved fopen()/fread() blocks indefinitely against this DeviceFS
   stream even though data is genuinely available (confirmed: raw
   OS_GBPB reason 4 read real bytes where fread() got nothing) -- so this
   is not a "maybe faster" alternative, it's the mechanism now KNOWN to
   work, replacing one proven not to. See docs/PLAN.md. */

static int os_find_open(const char *path)
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

static void os_find_close(int handle)
{
    _kernel_swi_regs regs;
    regs.r[0] = 0;
    regs.r[1] = handle;
    _kernel_swi(SWI_OS_FIND, &regs, &regs);
}

/* Returns bytes actually read (0..len), or -1 on error. Matches
   tools/probe_stream.bas's proven-working SYS "OS_GBPB",4,... call. */
static int os_gbpb_read4(int handle, unsigned char *buf, int len)
{
    _kernel_swi_regs regs;
    _kernel_oserror *err;

    regs.r[0] = 4;
    regs.r[1] = handle;
    regs.r[2] = (int)buf;
    regs.r[3] = len;
    err = _kernel_swi(SWI_OS_GBPB, &regs, &regs);
    if (err != NULL) {
        fprintf(stderr, "os_gbpb_read4(len=%d): %s\n", len, err->errmess);
        return -1;
    }
    return len - regs.r[3];
}

static int milestone4_streaming_test(int device_num)
{
    char dev[16];
    char path[64];
    unsigned char *buf;
    int handle;
    clock_t now, elapsed_ticks;
    unsigned long total_bytes;
    int got;

    printf("\n--- milestone 4: bulk streaming throughput ---\n");

    sprintf(dev, "usb%d", device_num);
    rtlsdr_reset_buffer(dev);

    /* endpoint1: confirmed bulk IN, interface0: confirmed the bulk
       interface, bulk: explicit even though it's the default for this
       endpoint, usbtimeout2000: bound each read to 2s so a stalled
       endpoint shows up as a short/failed read instead of hanging this
       test forever -- see docs/PLAN.md, this is exactly the risk
       milestone 4 exists to check for. */
    sprintf(path, "devices#endpoint%d;interface%d;bulk;usbtimeout2000:%s",
            RTLSDR_BULK_ENDPOINT, RTLSDR_BULK_INTERFACE, dev);
    printf("Opening %s ...\n", path);

    handle = os_find_open(path);
    if (handle == 0) {
        printf("OS_Find FAILED -- could not open the bulk endpoint "
               "stream.\n");
        printf("\nMilestone 4 gate: FAIL.\n");
        return 1;
    }

    buf = (unsigned char *)malloc(STREAM_CHUNK_BYTES);
    if (buf == NULL) {
        printf("malloc(%d) FAILED\n", STREAM_CHUNK_BYTES);
        os_find_close(handle);
        return 1;
    }

    /* Previous run found the exact boundary: 64/128/256/512/1024 bytes
       all fully reliable (5/5 instant attempts each), 2048 blocked ~110s
       before an "Escape" error. Not a USB-max-packet-size story (512 AND
       1024 both worked) -- a real, sharp boundary somewhere in
       (1024,2048), most plausibly a DeviceFS/driver internal buffer
       limit. Re-testing 2048 here would just risk another 100s+ Escape
       recovery for no new information, so the sweep now stops at 1024 --
       already-known-bad sizes aren't repeated. */
    total_bytes = 0;
    {
        static const int sizes[] = { 64, 128, 256, 512, 1024 };
        int size_idx;
        int size_ok;
        int attempt;
        int best_working_size;

        best_working_size = 0;
        for (size_idx = 0; size_idx < 5; size_idx++) {
            printf("--- size %d bytes ---\n", sizes[size_idx]);
            fflush(stdout);
            size_ok = 1;
            for (attempt = 0; attempt < 5; attempt++) {
                now = clock();
                got = os_gbpb_read4(handle, buf, sizes[size_idx]);
                elapsed_ticks = clock() - now;
                if (got < 0) {
                    printf("  attempt %d: ERROR after %.1fs\n", attempt + 1,
                           (double)elapsed_ticks / CLOCKS_PER_SEC);
                    size_ok = 0;
                    break;
                }
                printf("  attempt %d: %d bytes (%.2fs)\n", attempt + 1, got,
                       (double)elapsed_ticks / CLOCKS_PER_SEC);
                fflush(stdout);
                if ((double)elapsed_ticks / CLOCKS_PER_SEC > 1.0) {
                    printf("  (slow -- treating as unreliable at this "
                           "size)\n");
                    size_ok = 0;
                    break;
                }
                total_bytes += (unsigned long)got;
            }
            if (!size_ok) {
                printf("Size %d bytes: NOT reliable.\n", sizes[size_idx]);
                break;
            }
            best_working_size = sizes[size_idx];
            printf("Size %d bytes: all 5 attempts OK.\n", sizes[size_idx]);
        }

        /* Real sustained throughput measurement using the largest
           confirmed-reliable size, over an actual multi-second window --
           the number milestone 4 has been after all along. */
        if (best_working_size > 0) {
            clock_t tstart, tnow, ticks;
            int treads, tgot;
            unsigned long tbytes;
            double tsec, tbps;

            printf("\n--- sustained read at %d bytes/call, up to 5s ---\n",
                   best_working_size);
            fflush(stdout);
            tbytes = 0;
            treads = 0;
            tstart = clock();
            ticks = 0;
            do {
                tgot = os_gbpb_read4(handle, buf, best_working_size);
                treads++;
                tnow = clock();
                ticks = tnow - tstart;
                if (tgot < 0) {
                    printf("  ERROR on read %d after %.1fs\n", treads,
                           (double)ticks / CLOCKS_PER_SEC);
                    break;
                }
                tbytes += (unsigned long)tgot;
                if (treads % 500 == 0) {
                    printf("  ...%d reads, %lu bytes, %.1fs\n", treads,
                           tbytes, (double)ticks / CLOCKS_PER_SEC);
                    fflush(stdout);
                }
            } while (((double)ticks / CLOCKS_PER_SEC) < 5.0);

            tsec = (double)ticks / CLOCKS_PER_SEC;
            tbps = (tsec > 0) ? (tbytes / tsec) : 0;
            printf("\n%lu bytes in %.1fs = %.0f bytes/sec (%d reads)\n",
                   tbytes, tsec, tbps, treads);
            printf("Target (2.4 MSPS x 2 bytes): ~%lu bytes/sec\n",
                   STREAM_TARGET_BPS);
            if (tbps >= (double)STREAM_TARGET_BPS * 0.8) {
                printf("\nMilestone 4 gate: PASS.\n");
            } else {
                printf("\nMilestone 4 gate: FAIL -- throughput below "
                       "target (largest reliable single-call size is "
                       "only %d bytes; may need a fix to read larger "
                       "chunks, not just more small ones, to hit "
                       "target).\n", best_working_size);
            }
        }
    }

    os_find_close(handle);
    free(buf);

    printf("\n(%lu bytes moved during the size sweep itself, separate "
           "from the sustained-read measurement above.)\n", total_bytes);
    return 0;
}

/* ---- milestone 2: prove the control-transfer path works ----
   librtlsdr's EEPROM layout (magic byte, vendor_id/product_id field
   offsets) lives in its separate rtl_eeprom tool, not librtlsdr.c itself,
   and hasn't been pulled in yet -- rather than guess offsets, this reads
   the first 12 raw EEPROM bytes and prints them in hex for eyeballing
   against `rtl_eeprom -d 0` on a Linux box with the same dongle, if one's
   available (that's the plan's original milestone 2 gate). Independent of
   that: it also re-reads the first register twice and checks the two
   reads agree, which alone proves the control-transfer path itself is
   working and stable, without needing to know what the bytes mean. */
static int milestone2_register_probe(const char *dev)
{
    int i;
    int addr;
    int val;
    int val2;
    int all_ok;

    printf("\n--- milestone 2: EEPROM register read-back ---\n");
    all_ok = 1;
    for (i = 0; i < 6; i++) {
        addr = i * 2;
        val = rtlsdr_read_reg(dev, RTLSDR_BLOCK_ROM, addr, 2);
        if (val < 0) {
            printf("EEPROM[0x%02X]: read FAILED\n", addr);
            all_ok = 0;
            continue;
        }
        printf("EEPROM[0x%02X..0x%02X]: %04X\n", addr, addr + 1, val);
    }

    printf("\nConsistency check: reading EEPROM[0x00] twice...\n");
    val = rtlsdr_read_reg(dev, RTLSDR_BLOCK_ROM, 0, 2);
    val2 = rtlsdr_read_reg(dev, RTLSDR_BLOCK_ROM, 0, 2);
    if (val < 0 || val2 < 0) {
        printf("  FAILED (a read errored)\n");
        all_ok = 0;
    } else if (val != val2) {
        printf("  FAILED: %04X then %04X -- control transfers are not "
               "returning consistent data\n", val, val2);
        all_ok = 0;
    } else {
        printf("  PASS: both reads returned %04X\n", val);
    }

    if (all_ok) {
        printf("\nMilestone 2 gate: PASS (control-transfer path is "
               "working and stable).\n");
        printf("If you have Linux + the same dongle, compare the hex "
               "dump above against `rtl_eeprom -d 0` for full "
               "independent confirmation.\n");
        return 0;
    }
    printf("\nMilestone 2 gate: FAIL -- see errors above.\n");
    return 1;
}

/* ---- entry point ---- */

int main(void)
{
    int n;
    char device_name[16];

    printf("RTL-SDR RISC OS driver -- milestone 1 (enumerate & identify)\n");
    printf("Scanning *USBDevices output for a matching description...\n");

    n = find_device();
    if (n < 0) {
        printf("No matching device found.\n");
        printf("Run tools/probe.bas first to confirm the dongle is "
               "enumerated and check its *USBDevices description text.\n");
        return 1;
    }

    sprintf(device_name, "usb%d", n);
    printf("Found dongle: device number %d (DeviceFS name \"%s\")\n",
           n, device_name);
    printf("Milestone 1 gate: PASS (device identified).\n");

    if (milestone2_register_probe(device_name) != 0) {
        return 1;
    }

    if (milestone3_baseband_and_tuner(device_name) != 0) {
        return 1;
    }

    if (milestone_tune_test(device_name) != 0) {
        return 1;
    }

    printf("\nSetting sample rate to 2.4 MSPS...\n");
    if (rtlsdr_set_sample_rate(device_name, 2400000UL) != 0) {
        printf("rtlsdr_set_sample_rate() FAILED.\n");
        return 1;
    }
    printf("Sample rate set. This was never called at all before this "
           "build -- the RTL2832U's resampler was previously left at its "
           "power-up-default ratio, which is a real candidate for why no "
           "streaming data ever arrived.\n");

    return milestone4_streaming_test(n);
}
