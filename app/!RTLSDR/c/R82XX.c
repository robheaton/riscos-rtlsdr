/* R82XX.c -- R820T/R828D tuner driver. Ported from the rtlsdr-blog fork's
   src/tuner_r82xx.c, read from the raw downloaded source (not a
   paraphrase -- WebFetch's summarizer proved unreliable for functions
   this size/precision-sensitive; see docs/PLAN.md). Only the R828D +
   RTL-SDR Blog V4 path is ported; R820T-only and V4L branches from
   upstream are dropped since real hardware (docs/PLAN.md) confirmed V4.

   C89 only (Norcroft): all declarations at top of block, no // comments.

   UNTESTED ON REAL HARDWARE as of writing -- built from verified literal
   source, but "verified against source" isn't the same as "proven on the
   actual dongle". See docs/PLAN.md for what's confirmed vs. not. */

#include <string.h>
#include <stdio.h>
#include <time.h>
#include "RTLSDR.h"
#include "R82XX.h"

/* Pacing experiment: five real-hardware hangs so far have landed at
   different, otherwise-unrelated points in this driver's execution (most
   recently, between two adjacent TRACE() calls with nothing but a closing
   brace between them -- no possible blocking call there at all). That
   pattern -- correct logic, freeze location moving between runs -- points
   away from a line-specific bug and toward something environmental: tuner
   init is by far the densest burst of back-to-back USB control transfers
   anywhere in this project (dozens, zero pacing), and nothing before it
   has ever done that much rapid-fire USB traffic at once. This adds a
   short busy-wait after every I2C transfer to see whether pacing avoids
   the hangs -- a real hypothesis, not proven, but a different class of
   fix than chasing individual call sites has been. Remove if it doesn't
   help. */
static void r82xx_pace(void)
{
    clock_t start;
    start = clock();
    while (((double)(clock() - start) / CLOCKS_PER_SEC) < 0.01) {
        /* busy-wait ~10ms */
    }
}

/* Temporary hang-diagnosis instrumentation: a real-hardware run froze the
   whole machine somewhere inside r82xx_set_freq() with no visible output
   past r82xx_init() -- which doesn't tell us where, since any printf
   after that point may not have been flushed before the freeze. These
   TRACE() calls flush immediately so the LAST line printed to screen
   during a hang is the actual last step that ran, pinpointing the call
   that's stuck. Remove once milestone 4's hang is understood and fixed. */
#define TRACE(msg) do { printf("  [trace] %s\n", msg); fflush(stdout); } while (0)

#define HF_THRESHOLD_HZ   28800000UL   /* 28.8MHz */

/* RTL2832U I2C bridge burst-write limit (reg byte + data), confirmed from
   upstream's rtlsdr_open() tuner config (r82xx_c.max_i2c_msg_len = 8). */
#define R82XX_MAX_I2C_MSG_LEN  8
#define VHF_UPPER_HZ      250000000UL  /* 250MHz */

/* ---- low-level I2C access to the tuner (through the RTL2832U's I2C
   repeater -- caller must have it enabled, see RTLSDR.c's
   rtlsdr_probe_tuner for the enable/disable register writes) ---- */

/* Bit-reversal: the R82xx tuner returns register bytes bit-reversed over
   I2C read (confirmed from upstream's r82xx_bitrev() -- easy to miss,
   would silently break every read-based decision: PLL lock detection,
   VCO fine-tune, filter calibration code). */
static unsigned char r82xx_bitrev(unsigned char byte)
{
    static const unsigned char lut[16] = {
        0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
        0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf
    };
    return (unsigned char)((lut[byte & 0xf] << 4) | lut[byte >> 4]);
}

/* Writes reg, then val[0..len-1] to consecutive registers, as ONE I2C
   transaction (matches upstream r82xx_write -- our transfers are always
   small enough that upstream's max_i2c_msg_len chunking never triggers). */
static int r82xx_i2c_write(const char *dev, int i2c_addr, int reg,
                            const unsigned char *val, int len)
{
    unsigned char buf[R82XX_NUM_REGS + 1];
    int w_index;
    int rc;

    if (len > R82XX_NUM_REGS) {
        return -1;
    }
    buf[0] = (unsigned char)reg;
    memcpy(&buf[1], val, (size_t)len);

    w_index = (RTLSDR_BLOCK_I2C << 8) | 0x10;
    rc = usb_ctrl_transfer(dev, CTRL_BMREQ_VENDOR_OUT, 0, i2c_addr,
                            w_index, len + 1, buf);
    r82xx_pace();
    return rc;
}

/* Reads len bytes starting at reg, bit-reversing each (matches upstream
   r82xx_read). */
static int r82xx_i2c_read(const char *dev, int i2c_addr, int reg,
                           unsigned char *val, int len)
{
    unsigned char sel;
    int w_index_out, w_index_in;
    int i;

    w_index_out = (RTLSDR_BLOCK_I2C << 8) | 0x10;
    w_index_in = RTLSDR_BLOCK_I2C << 8;

    sel = (unsigned char)reg;
    if (usb_ctrl_transfer(dev, CTRL_BMREQ_VENDOR_OUT, 0, i2c_addr,
                           w_index_out, 1, &sel) != 0) {
        r82xx_pace();
        return -1;
    }
    r82xx_pace();
    if (usb_ctrl_transfer(dev, CTRL_BMREQ_VENDOR_IN, 0, i2c_addr,
                           w_index_in, len, val) != 0) {
        r82xx_pace();
        return -1;
    }
    r82xx_pace();
    for (i = 0; i < len; i++) {
        val[i] = r82xx_bitrev(val[i]);
    }
    return 0;
}

/* ---- shadow register cache (upstream: shadow_store/r82xx_read_cache_reg
   -- masked writes read the CACHE, not live hardware, matching upstream
   exactly) ---- */

static void r82xx_shadow_store(r82xx_t *t, int reg, const unsigned char *val,
                                int len)
{
    int r;
    r = reg - R82XX_REG_SHADOW_START;
    if (r < 0) {
        len += r;
        r = 0;
    }
    if (len <= 0) {
        return;
    }
    if (len > R82XX_NUM_REGS - r) {
        len = R82XX_NUM_REGS - r;
    }
    memcpy(&t->regs[r], val, (size_t)len);
}

static int r82xx_read_cache_reg(r82xx_t *t, int reg)
{
    reg -= R82XX_REG_SHADOW_START;
    if (reg >= 0 && reg < R82XX_NUM_REGS) {
        return t->regs[reg];
    }
    return -1;
}

/* The RTL2832U's I2C bridge caps a single burst write at
   R82XX_MAX_I2C_MSG_LEN bytes total (reg + data) -- confirmed from
   upstream's own rtlsdr_open()-time tuner config
   (devt->r82xx_c.max_i2c_msg_len = 8). Chunk exactly like upstream's
   r82xx_write does: consecutive registers, so reg advances by the chunk
   size each iteration. Missing this is what broke the very first call
   this file ever made on real hardware -- the init-array write is 27
   bytes in one shot, the first multi-byte write anywhere in this
   project, and the I2C bridge rejected it outright ("Bad request").
   Everything before this (1-2 byte demod/EEPROM/I2C-select writes) was
   accidentally small enough to never hit the limit. */
static int r82xx_write(r82xx_t *t, int reg, const unsigned char *val,
                        int len)
{
    int pos, size, rc;

    r82xx_shadow_store(t, reg, val, len);

    pos = 0;
    do {
        size = (len - pos > R82XX_MAX_I2C_MSG_LEN - 1)
                   ? R82XX_MAX_I2C_MSG_LEN - 1
                   : len - pos;
        rc = r82xx_i2c_write(t->dev_name, R828D_I2C_ADDR, reg + pos,
                              val + pos, size);
        if (rc != 0) {
            return rc;
        }
        pos += size;
    } while (pos < len);

    return 0;
}

static int r82xx_write_reg(r82xx_t *t, int reg, int val)
{
    unsigned char v;
    v = (unsigned char)val;
    return r82xx_write(t, reg, &v, 1);
}

static int r82xx_write_reg_mask(r82xx_t *t, int reg, int val, int bit_mask)
{
    int cached;
    unsigned char v;

    cached = r82xx_read_cache_reg(t, reg);
    if (cached < 0) {
        return cached;
    }
    v = (unsigned char)((cached & ~bit_mask) | (val & bit_mask));
    return r82xx_write(t, reg, &v, 1);
}

/* ---- GPIO (RTL2832U SYS block -- V4 upconverter switch + bias tee) ---- */

static void rtlsdr_set_gpio_output(const char *dev, int gpio)
{
    int bit, r;
    bit = 1 << gpio;
    TRACE("gpio_output: reading GPD");
    r = rtlsdr_read_reg(dev, RTLSDR_BLOCK_SYS, GPD, 1);
    TRACE("gpio_output: writing GPD");
    rtlsdr_write_reg(dev, RTLSDR_BLOCK_SYS, GPD, r & ~bit, 1);
    TRACE("gpio_output: reading GPOE");
    r = rtlsdr_read_reg(dev, RTLSDR_BLOCK_SYS, GPOE, 1);
    TRACE("gpio_output: writing GPOE");
    rtlsdr_write_reg(dev, RTLSDR_BLOCK_SYS, GPOE, r | bit, 1);
    TRACE("gpio_output: done");
}

static void rtlsdr_set_gpio_bit(const char *dev, int gpio, int val)
{
    int bit, r;
    bit = 1 << gpio;
    TRACE("gpio_bit: reading GPO");
    r = rtlsdr_read_reg(dev, RTLSDR_BLOCK_SYS, GPO, 1);
    r = val ? (r | bit) : (r & ~bit);
    TRACE("gpio_bit: writing GPO");
    rtlsdr_write_reg(dev, RTLSDR_BLOCK_SYS, GPO, r, 1);
    TRACE("gpio_bit: done");
}

static void rtlsdr_set_bias_tee_gpio(const char *dev, int gpio, int on)
{
    rtlsdr_set_gpio_output(dev, gpio);
    rtlsdr_set_gpio_bit(dev, gpio, on);
}

/* ---- init register array (upstream r82xx_init_array, registers
   0x05-0x1F) ---- */

static const unsigned char r82xx_init_array[27] = {
    0x83, 0x30, 0x75,                  /* 05 to 07 */
    0xc0, 0x40, 0xd6, 0x6c,             /* 08 to 0b */
    0xf5, 0x63, 0x75, 0x68,             /* 0c to 0f */
    0x6c, 0x83, 0x80, 0x00,             /* 10 to 13 */
    0x0f, 0x00, 0xc0, 0x30,             /* 14 to 17 */
    0x48, 0xcc, 0x60, 0x00,             /* 18 to 1b */
    0x54, 0xae, 0x4a, 0xc0              /* 1c to 1f */
};

/* ---- frequency range table (upstream freq_ranges[]): freq(MHz), open_d,
   rf_mux_ploy, tf_c, xtal_cap20p, xtal_cap10p, xtal_cap0p ---- */

typedef struct {
    unsigned long freq_mhz;
    unsigned char open_d;
    unsigned char rf_mux_ploy;
    unsigned char tf_c;
    unsigned char xtal_cap20p;
    unsigned char xtal_cap10p;
    unsigned char xtal_cap0p;
} r82xx_freq_range_t;

static const r82xx_freq_range_t freq_ranges[21] = {
    {   0, 0x08, 0x02, 0xdf, 0x02, 0x01, 0x00 },
    {  50, 0x08, 0x02, 0xbe, 0x02, 0x01, 0x00 },
    {  55, 0x08, 0x02, 0x8b, 0x02, 0x01, 0x00 },
    {  60, 0x08, 0x02, 0x7b, 0x02, 0x01, 0x00 },
    {  65, 0x08, 0x02, 0x69, 0x02, 0x01, 0x00 },
    {  70, 0x08, 0x02, 0x58, 0x02, 0x01, 0x00 },
    {  75, 0x00, 0x02, 0x44, 0x02, 0x01, 0x00 },
    {  80, 0x00, 0x02, 0x44, 0x02, 0x01, 0x00 },
    {  90, 0x00, 0x02, 0x34, 0x01, 0x01, 0x00 },
    { 100, 0x00, 0x02, 0x34, 0x01, 0x01, 0x00 },
    { 110, 0x00, 0x02, 0x24, 0x01, 0x01, 0x00 },
    { 120, 0x00, 0x02, 0x24, 0x01, 0x01, 0x00 },
    { 140, 0x00, 0x02, 0x14, 0x01, 0x01, 0x00 },
    { 180, 0x00, 0x02, 0x13, 0x00, 0x00, 0x00 },
    { 220, 0x00, 0x02, 0x13, 0x00, 0x00, 0x00 },
    { 250, 0x00, 0x02, 0x11, 0x00, 0x00, 0x00 },
    { 280, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00 },
    { 310, 0x00, 0x41, 0x00, 0x00, 0x00, 0x00 },
    { 450, 0x00, 0x41, 0x00, 0x00, 0x00, 0x00 },
    { 588, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00 },
    { 650, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00 }
};

/* ---- tuning logic ---- */

static int r82xx_set_mux(r82xx_t *t, unsigned long freq_hz)
{
    const r82xx_freq_range_t *range;
    unsigned long freq_mhz;
    unsigned int i;
    int rc;
    int val;

    TRACE("set_mux: entry");

    freq_mhz = freq_hz / 1000000UL;
    for (i = 0; i < 20; i++) {
        if (freq_mhz < freq_ranges[i + 1].freq_mhz) {
            break;
        }
    }
    range = &freq_ranges[i];

    rc = r82xx_write_reg_mask(t, 0x17, range->open_d, 0x08);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x1a, range->rf_mux_ploy, 0xc3);
    if (rc < 0) return rc;
    rc = r82xx_write_reg(t, 0x1b, range->tf_c);
    if (rc < 0) return rc;
    TRACE("set_mux: wrote 0x17, 0x1a, 0x1b");

    /* xtal_cap_sel is always XTAL_HIGH_CAP_0P in our port (set once in
       r82xx_init and never changed -- upstream only changes it for
       analog-TV standard switching, which we never do). */
    val = range->xtal_cap0p | 0x00;
    rc = r82xx_write_reg_mask(t, 0x10, val, 0x0b);
    if (rc < 0) return rc;

    rc = r82xx_write_reg_mask(t, 0x08, 0x00, 0x3f);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x09, 0x00, 0x3f);
    TRACE("set_mux: done");
    return rc;
}

static int r82xx_set_vga_gain(r82xx_t *t)
{
    int rc;
    TRACE("set_vga_gain: entry");
    rc = r82xx_write_reg_mask(t, 0x0c, 0x08, 0x9f);
    TRACE("set_vga_gain: done");
    return rc;
}

/* The core PLL/VCO frequency synthesis. Verbatim algorithm from upstream
   r82xx_set_pll -- xtal reference is the RTL2832U's 28.8MHz (our only
   supported case; V4 doesn't use R828D_XTAL_FREQ, see h/RTLSDR.h). */
static int r82xx_set_pll(r82xx_t *t, unsigned long freq_hz)
{
    unsigned long pll_ref;
    unsigned long freq_khz, pll_ref_khz;
    unsigned long vco_min, vco_max;
    unsigned long mix_div, div_buf, div_num;
    unsigned long n_sdm, sdm;
    unsigned long vco_freq;
    unsigned long nint, vco_fra;
    unsigned char ni, si, val;
    unsigned char vco_power_ref, vco_fine_tune, refdiv2;
    unsigned char data[5];
    int rc, i;

    TRACE("set_pll: entry");

    pll_ref = 28800000UL;
    freq_khz = (freq_hz + 500) / 1000;
    pll_ref_khz = (pll_ref + 500) / 1000;

    refdiv2 = 0;
    rc = r82xx_write_reg_mask(t, 0x10, refdiv2, 0x10);
    if (rc < 0) return rc;
    TRACE("set_pll: wrote 0x10 (refdiv2)");

    rc = r82xx_write_reg_mask(t, 0x1a, 0x00, 0x0c);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x12, 0x06, 0xff);
    if (rc < 0) return rc;
    TRACE("set_pll: wrote 0x1a, 0x12");

    vco_min = 1770000UL;
    vco_max = vco_min * 2;
    mix_div = 2;
    div_buf = 0;
    div_num = 0;

    while (mix_div <= 64) {
        if ((freq_khz * mix_div) >= vco_min &&
            (freq_khz * mix_div) < vco_max) {
            div_buf = mix_div;
            while (div_buf > 2) {
                div_buf = div_buf >> 1;
                div_num++;
            }
            break;
        }
        mix_div = mix_div << 1;
    }

    TRACE("set_pll: about to i2c_read data[5] (VCO fine-tune)");
    rc = r82xx_i2c_read(t->dev_name, R828D_I2C_ADDR, 0x00, data, 5);
    if (rc < 0) return rc;
    TRACE("set_pll: i2c_read data[5] done");

    /* V4 is always R828D in our port -- upstream's vco_power_ref
       selection collapses to the R828D branch unconditionally. */
    vco_power_ref = 1;

    vco_fine_tune = (unsigned char)((data[4] & 0x30) >> 4);
    if (vco_fine_tune > vco_power_ref) {
        div_num = div_num - 1;
    } else if (vco_fine_tune < vco_power_ref) {
        div_num = div_num + 1;
    }

    rc = r82xx_write_reg_mask(t, 0x10, (int)(div_num << 5), 0xe0);
    if (rc < 0) return rc;
    TRACE("set_pll: wrote 0x10 (div_num)");

    /* upstream uses uint64_t here defensively; not needed (and C89 has no
       guaranteed 64-bit type anyway) -- the mix_div loop above guarantees
       freq_hz*mix_div lands within the VCO's ~1.77-3.54GHz range, which
       always fits in 32-bit unsigned long (max ~4.29G) for every
       frequency this driver's freq_ranges table supports (up to 650MHz,
       mix_div>=4 -> vco_freq<=2.6G; down to ~32MHz HF+IF, mix_div<=64 ->
       vco_freq<=2.1G). Verified by hand, not just assumed. */
    vco_freq = freq_hz * mix_div;
    nint = vco_freq / (2 * pll_ref);
    vco_fra = (vco_freq - 2 * pll_ref * nint) / 1000;

    if (nint > ((128UL / vco_power_ref) - 1)) {
        return -1; /* no valid PLL values for this frequency */
    }

    /* Norcroft warns "lower precision in wider context" here (mixing the
       unsigned char ni with the unsigned long nint) -- benign: nint is
       always ~30-61 in this driver's frequency range (established by the
       mix_div/VCO-range constraint above), so ni ends up ~4-12, nowhere
       near truncating against unsigned char's 0-255 range. */
    ni = (unsigned char)((nint - 13) / 4);
    si = (unsigned char)(nint - 4 * ni - 13);
    rc = r82xx_write_reg(t, 0x14, (int)(ni + (si << 6)));
    if (rc < 0) return rc;

    val = vco_fra ? 0x00 : 0x08;
    rc = r82xx_write_reg_mask(t, 0x12, val, 0x08);
    if (rc < 0) return rc;
    printf("  [trace] set_pll: freq_hz=%lu mix_div=%lu vco_freq=%lu "
           "nint=%lu vco_fra=%lu pll_ref_khz=%lu -- about to enter "
           "sigma-delta loop\n",
           freq_hz, mix_div, vco_freq, nint, vco_fra, pll_ref_khz);
    fflush(stdout);

    n_sdm = 2;
    sdm = 0;
    i = 0;
    while (vco_fra > 1) {
        printf("  [trace]   sigma-delta iter %d: n_sdm=%lu vco_fra=%lu\n",
               i, n_sdm, vco_fra);
        fflush(stdout);
        if (vco_fra > (2 * pll_ref_khz / n_sdm)) {
            sdm = sdm + 32768 / (n_sdm / 2);
            vco_fra = vco_fra - 2 * pll_ref_khz / n_sdm;
            if (n_sdm >= 0x8000) {
                break;
            }
        }
        n_sdm <<= 1;
        /* Defensive: upstream has no bound here beyond n_sdm reaching
           0x8000 inside the if-branch, which this port's own by-hand
           trace (docs/PLAN.md) shows takes ~15 iterations for a real
           100MHz tune -- but a hang was observed somewhere in this
           function on real hardware and the exact cause isn't confirmed
           yet, so don't let this specific loop be a second possible
           culprit while that's being tracked down. 64 is generous (n_sdm
           is unsigned long; this is nowhere near a legitimate need). */
        i++;
        if (i > 64) {
            printf("  [trace] set_pll: sigma-delta loop exceeded 64 "
                   "iterations, aborting (vco_fra=%lu n_sdm=%lu)\n",
                   vco_fra, n_sdm);
            fflush(stdout);
            return -1;
        }
    }
    TRACE("set_pll: sigma-delta loop done");

    rc = r82xx_write_reg(t, 0x16, (int)(sdm >> 8));
    if (rc < 0) return rc;
    rc = r82xx_write_reg(t, 0x15, (int)(sdm & 0xff));
    if (rc < 0) return rc;
    TRACE("set_pll: wrote sdm to 0x15/0x16, entering lock-check loop");

    for (i = 0; i < 2; i++) {
        rc = r82xx_i2c_read(t->dev_name, R828D_I2C_ADDR, 0x00, data, 3);
        if (rc < 0) return rc;
        TRACE("set_pll: lock-check i2c_read done");
        if (data[2] & 0x40) {
            break;
        }
        if (!i) {
            rc = r82xx_write_reg_mask(t, 0x12, 0x06, 0xff);
            if (rc < 0) return rc;
        }
    }

    if (!(data[2] & 0x40)) {
        t->has_lock = 0;
        return 0;
    }
    t->has_lock = 1;

    return r82xx_write_reg_mask(t, 0x1a, 0x08, 0x08);
}

/* Fixed-argument path only: upstream is called as
   r82xx_sysfreq_sel(priv, 0, TUNER_DIGITAL_TV, SYS_DVBT) from r82xx_init,
   always -- the freq==0 branch (default mixer_top/lna_top/etc. below) and
   the "type != TUNER_ANALOG_TV" branch are the only ones we ever take, so
   ported directly rather than as a generic function.
   Assumes cfg->use_predetect == 0 (typical for RTL2832U dongles) -- not
   independently confirmed for this specific unit, see docs/PLAN.md. */
static int r82xx_sysfreq_sel(r82xx_t *t)
{
    int rc;
    unsigned char mixer_top, lna_top, cp_cur, div_buf_cur;
    unsigned char lna_vth_l, mixer_vth_l, air_cable1_in, cable2_in;
    unsigned char lna_discharge, filter_cur;

    mixer_top = 0x24;
    lna_top = 0xe5;
    cp_cur = 0x38;
    div_buf_cur = 0x30;
    lna_vth_l = 0x53;
    mixer_vth_l = 0x75;
    air_cable1_in = 0x00;
    cable2_in = 0x00;
    lna_discharge = 14;
    filter_cur = 0x40;

    rc = r82xx_write_reg_mask(t, 0x1d, lna_top, 0xc7);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x1c, mixer_top, 0xf8);
    if (rc < 0) return rc;
    rc = r82xx_write_reg(t, 0x0d, lna_vth_l);
    if (rc < 0) return rc;
    rc = r82xx_write_reg(t, 0x0e, mixer_vth_l);
    if (rc < 0) return rc;

    t->input = air_cable1_in;

    rc = r82xx_write_reg_mask(t, 0x05, air_cable1_in, 0x60);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x06, cable2_in, 0x08);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x11, cp_cur, 0x38);
    if (rc < 0) return rc;

    /* "RTL-SDR Blog Hack. Improve L-band performance by setting PLL drop
       out to 2.0v" -- upstream overrides div_buf_cur unconditionally. */
    div_buf_cur = 0xa0;
    rc = r82xx_write_reg_mask(t, 0x17, div_buf_cur, 0x30);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x0a, filter_cur, 0x60);
    if (rc < 0) return rc;

    /* type != TUNER_ANALOG_TV branch (always taken) */
    rc = r82xx_write_reg_mask(t, 0x1d, 0, 0x38);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x1c, 0, 0x04);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x06, 0, 0x40);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x1a, 0x30, 0x30);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x1d, 0x18, 0x38);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x1c, mixer_top, 0x04);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x1e, lna_discharge, 0x1f);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x1a, 0x20, 0x30);
    return rc;
}

/* Fixed BW<6MHz path only -- upstream's r82xx_set_tv_standard is called
   from r82xx_init with bw=3 always, and in this fork's simplified form
   the "BW < 6 MHz" constants at the top are set unconditionally (bw isn't
   actually branched on in the body), so ported directly. Includes the
   56MHz filter calibration loop, which itself calls r82xx_set_pll. */
static int r82xx_set_tv_standard(r82xx_t *t)
{
    unsigned char hp_cor, filt_gain, img_r, filt_q, ext_enable;
    unsigned char loop_through, lt_att, flt_ext_widest, polyfil_cur;
    unsigned char data[5];
    unsigned long filt_cal_lo;
    int rc, i;

    filt_cal_lo = 56000000UL;
    filt_gain = 0x30;
    img_r = 0x00;
    filt_q = 0x10;
    hp_cor = 0x6b;
    ext_enable = 0x60;
    loop_through = 0x80;
    lt_att = 0x00;
    flt_ext_widest = 0x00;
    polyfil_cur = 0x60;

    memcpy(t->regs, r82xx_init_array, sizeof(r82xx_init_array));

    rc = r82xx_write_reg_mask(t, 0x0c, 0x00, 0x0f);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x13, R82XX_VER_NUM, 0x3f);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x1d, 0x00, 0x38);
    if (rc < 0) return rc;

    /* Initial analog IF, matching upstream's if_khz=3570 default for
       the "BW < 6MHz" case this port always takes -- r82xx_set_freq()
       uses this field (not a fixed constant) for its LO calculation.
       Corrected later by r82xx_set_bandwidth() once the real capture
       bandwidth is known (see R82XX.h and docs/PLAN.md). */
    t->int_freq = 3570000;

    /* filter calibration -- force_calibration always true in our port
       (upstream: "we call this function only once in rtlsdr, force
       calibration") */
    t->fil_cal_code = 0;
    for (i = 0; i < 2; i++) {
        rc = r82xx_write_reg_mask(t, 0x0b, hp_cor, 0x60);
        if (rc < 0) return rc;
        rc = r82xx_write_reg_mask(t, 0x0f, 0x04, 0x04);
        if (rc < 0) return rc;
        rc = r82xx_write_reg_mask(t, 0x10, 0x00, 0x03);
        if (rc < 0) return rc;

        rc = r82xx_set_pll(t, filt_cal_lo);
        if (rc < 0 || !t->has_lock) {
            return rc < 0 ? rc : -1;
        }

        rc = r82xx_write_reg_mask(t, 0x0b, 0x10, 0x10);
        if (rc < 0) return rc;
        rc = r82xx_write_reg_mask(t, 0x0b, 0x00, 0x10);
        if (rc < 0) return rc;
        rc = r82xx_write_reg_mask(t, 0x0f, 0x00, 0x04);
        if (rc < 0) return rc;

        rc = r82xx_i2c_read(t->dev_name, R828D_I2C_ADDR, 0x00, data, 5);
        if (rc < 0) return rc;

        t->fil_cal_code = data[4] & 0x0f;
        if (t->fil_cal_code && t->fil_cal_code != 0x0f) {
            break;
        }
    }
    if (t->fil_cal_code == 0x0f) {
        t->fil_cal_code = 0;
    }

    rc = r82xx_write_reg_mask(t, 0x0a, filt_q | t->fil_cal_code, 0x1f);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x0b, hp_cor, 0xef);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x07, img_r, 0x80);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x06, filt_gain, 0x30);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x1e, ext_enable, 0x60);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x05, loop_through, 0x80);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x1f, lt_att, 0x80);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x0f, flt_ext_widest, 0x80);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x19, polyfil_cur, 0x60);
    return rc;
}

int r82xx_init(r82xx_t *t, const char *dev_name)
{
    int rc;

    memset(t, 0, sizeof(*t));
    t->dev_name = dev_name;
    t->is_v4 = 1;
    t->input = 0xff; /* force the first set_freq's band-change branch */

    rc = r82xx_write(t, 0x05, r82xx_init_array, sizeof(r82xx_init_array));
    if (rc < 0) return rc;

    rc = r82xx_set_tv_standard(t);
    if (rc < 0) return rc;

    return r82xx_sysfreq_sel(t);
}

int r82xx_set_freq(r82xx_t *t, unsigned long freq_hz)
{
    unsigned long upconvert_freq, lo_freq;
    unsigned char open_d, band, cable_2_in, cable_1_in, air_in;
    int rc;

    upconvert_freq = (freq_hz < HF_THRESHOLD_HZ)
                          ? (freq_hz + HF_THRESHOLD_HZ)
                          : freq_hz;
    /* t->int_freq (NOT the old fixed R82XX_IF_FREQ constant -- see
       R82XX.h and docs/PLAN.md) is the tuner's ACTUAL analog IF: set
       to 3.57MHz by r82xx_set_tv_standard(), then corrected by
       r82xx_set_bandwidth() once main() knows the real capture
       bandwidth. Matches upstream's own priv->int_freq usage here
       exactly -- using a value that could go stale (the old fixed
       constant, never updated after r82xx_set_bandwidth() runs) was a
       real, previously-missed bug: it let the demod's digital IF and
       the tuner's real analog IF disagree by however large the
       bandwidth correction was. */
    lo_freq = upconvert_freq + (unsigned long)t->int_freq;

    TRACE("set_freq: entry, about to call set_mux");
    rc = r82xx_set_mux(t, lo_freq);
    if (rc < 0) return rc;
    TRACE("set_freq: set_mux done, about to call set_vga_gain");
    rc = r82xx_set_vga_gain(t);
    if (rc < 0) return rc;
    TRACE("set_freq: set_vga_gain done, about to call set_pll");
    rc = r82xx_set_pll(t, lo_freq);
    TRACE("set_freq: set_pll returned");
    if (rc < 0 || !t->has_lock) {
        return rc < 0 ? rc : -1;
    }

    open_d = (freq_hz <= 2200000UL ||
              (freq_hz >= 85000000UL && freq_hz <= 112000000UL) ||
              (freq_hz >= 172000000UL && freq_hz <= 242000000UL))
                 ? 0x00 : 0x08;
    rc = r82xx_write_reg_mask(t, 0x17, open_d, 0x08);
    if (rc < 0) return rc;
    TRACE("set_freq: wrote open_d to 0x17");

    if (freq_hz <= HF_THRESHOLD_HZ) {
        band = R82XX_BAND_HF;
    } else if (freq_hz < VHF_UPPER_HZ) {
        band = R82XX_BAND_VHF;
    } else {
        band = R82XX_BAND_UHF;
    }

    if (band == R82XX_BAND_HF) {
        rc = r82xx_write_reg_mask(t, 0x1a, 0x40, 0xc3);
        if (rc < 0) return rc;
        rc = r82xx_write_reg(t, 0x1b, 0x00);
        if (rc < 0) return rc;
    }
    TRACE("set_freq: band determined, checking if band changed");

    if (band != t->input) {
        t->input = band;

        cable_2_in = (band == R82XX_BAND_HF) ? 0x08 : 0x00;
        rc = r82xx_write_reg_mask(t, 0x06, cable_2_in, 0x08);
        if (rc < 0) return rc;
        TRACE("set_freq: wrote cable_2_in to 0x06");

        /* Re-enabled (was disabled through all of phase 1's milestones --
           see docs/PLAN.md). GPIO_UPCONVERT_PIN controls the V4's
           board-level RF switch between the antenna connector and the HF
           upconverter mixer: ON routes through the upconverter (needed
           for HF, which r82xx_set_freq already upconverts before this
           point), OFF bypasses it for direct VHF/UHF sampling. This is
           NOT the same thing as the tuner chip's own internal cable_1_in/
           air_in pins written just below -- those pick which of the
           tuner's own differential inputs is live, but say nothing about
           whether the antenna signal is physically routed through the
           upconverter first. Left disabled through phase 1 because every
           GPIO access (SYS block GPD/GPOE/GPO, RTLSDR_BLOCK_SYS
           0x3001-0x3004) had frozen the whole machine -- but that was
           always under TaskWindow execution, since established (many
           runs, zero hangs since) as the actual freeze cause, not the
           GPIO access itself. Phase 2's !RTLSDRView found a real symptom
           this omission explains: correct tuning, PLL lock, and a real
           antenna, but no visible VHF signal -- consistent with the
           antenna staying routed through the upconverter instead of
           bypassing it. */
        TRACE("set_freq: setting upconvert-bypass GPIO");
        rtlsdr_set_bias_tee_gpio(t->dev_name, GPIO_UPCONVERT_PIN,
                                  (band == R82XX_BAND_HF) ? 1 : 0);
        TRACE("set_freq: upconvert-bypass GPIO done");

        cable_1_in = (band == R82XX_BAND_VHF) ? 0x40 : 0x00;
        rc = r82xx_write_reg_mask(t, 0x05, cable_1_in, 0x40);
        if (rc < 0) return rc;

        air_in = (band == R82XX_BAND_UHF) ? 0x00 : 0x20;
        rc = r82xx_write_reg_mask(t, 0x05, air_in, 0x20);
        if (rc < 0) return rc;
        TRACE("set_freq: band-switch registers done");
    }

    TRACE("set_freq: returning success");
    return 0;
}

/* Reverses the manual-mode auto-off bits below, matching upstream's own
   default (the untouched init array -- see R82XX.h/docs/PLAN.md). */
int r82xx_set_gain_agc(r82xx_t *t)
{
    int rc;

    rc = r82xx_write_reg_mask(t, 0x05, 0x00, 0x10); /* LNA auto on */
    if (rc < 0) return rc;
    return r82xx_write_reg_mask(t, 0x07, 0x10, 0x10); /* Mixer auto on */
}

/* Verbatim register-write shape from upstream r82xx_set_gain()'s
   set_manual_gain branch (real source, fetched and read directly --
   see docs/PLAN.md): LNA auto-off is reg 0x05 bit4=1, Mixer auto-off is
   reg 0x07 bit4=0 (note the OPPOSITE polarity from LNA -- confirmed
   from source, not assumed), then gain index goes in the low 4 bits of
   each register (mask 0x0f). Upstream's r82xx_set_vga_gain() (the fixed
   0x08/16.3dB write, already ported as part of every retune) is called
   as part of the same manual-mode path upstream, so no separate VGA
   write is needed here. index is clamped to 0-15 (the range of both
   r82xx_lna_gain_steps[] and r82xx_mixer_gain_steps[] upstream, which
   aren't themselves ported yet -- this driver doesn't convert an index
   to a real dB value, it just exposes the raw step). */
int r82xx_set_gain_manual(r82xx_t *t, int index)
{
    int rc;

    if (index < 0) {
        index = 0;
    }
    if (index > 15) {
        index = 15;
    }

    rc = r82xx_write_reg_mask(t, 0x05, 0x10, 0x10); /* LNA auto off */
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x07, 0x00, 0x10); /* Mixer auto off */
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x05, index, 0x0f); /* LNA gain index */
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x07, index, 0x0f); /* Mixer gain index */
    return rc;
}

/* Bandwidth contribution by low-pass filter -- verbatim table from
   upstream (real source, fetched and read directly, not summarized;
   see docs/PLAN.md). */
static const int r82xx_if_low_pass_bw_table[10] = {
    1700000, 1600000, 1550000, 1450000, 1200000, 900000, 700000, 550000,
    450000, 350000
};
#define R82XX_FILT_HP_BW1 350000
#define R82XX_FILT_HP_BW2 380000

/* Verbatim port of upstream r82xx_set_bandwidth() -- computes the
   tuner's actual analog IF/filter-bandwidth configuration for a
   requested capture bandwidth, and writes it to registers 0x0a/0x0b.
   Upstream's third parameter (`rate`) is declared but never referenced
   in the function body, so it's dropped here (confirmed from source,
   not assumed -- see docs/PLAN.md for how big a difference "declared
   but unused" vs "actually used" can make when porting from a
   summarizer instead of raw source).

   This was the missing piece behind the flat-spectrum/demod-saturation
   mystery (see docs/PLAN.md): without it, t->int_freq stayed at
   r82xx_set_tv_standard()'s fixed 3.57MHz default forever, no matter
   what capture bandwidth was actually configured, while the tuner's
   OWN analog filter registers were never adjusted for that bandwidth
   either -- for this project's 2.4MHz capture, upstream's real
   computation gives ~1.815MHz, a ~1.75MHz mismatch against a capture
   whose Nyquist limit is only 1.2MHz. A mismatch that large between
   what the demod's digital downconversion assumes and what the
   tuner's analog IF actually centres on aliases real content into
   what looks exactly like unstructured noise -- matching everything
   observed: a flat averaged spectrum, and an FM phase discriminator
   saturating near its theoretical maximum instead of tracking real
   modulation.

   THE CALLER MUST, after this returns successfully: resync the
   demod's digital IF to the returned value (Driver.h's
   rtlsdr_set_if_freq()) and re-tune at the current frequency
   (r82xx_set_freq()) -- matching upstream's r820t_set_bw(), which
   chains all three. Call with the I2C repeater enabled. Returns the
   new t->int_freq (>=0) on success, negative on I2C failure. */
int r82xx_set_bandwidth(r82xx_t *t, int bw_hz)
{
    int rc;
    unsigned int i;
    int real_bw;
    int reg_0a, reg_0b;
    int bw;

    bw = bw_hz;
    real_bw = 0;

    if (bw > 7000000) {
        /* BW: 8 MHz */
        reg_0a = 0x10;
        reg_0b = 0x0b;
        t->int_freq = 4570000;
    } else if (bw > 6000000) {
        /* BW: 7 MHz */
        reg_0a = 0x10;
        reg_0b = 0x2a;
        t->int_freq = 4570000;
    } else if (bw > r82xx_if_low_pass_bw_table[0] +
                        R82XX_FILT_HP_BW1 + R82XX_FILT_HP_BW2) {
        /* BW: 6 MHz */
        reg_0a = 0x10;
        reg_0b = 0x6b;
        t->int_freq = 3570000;
    } else {
        reg_0a = 0x00;
        reg_0b = 0x80;
        t->int_freq = 2300000;

        if (bw > r82xx_if_low_pass_bw_table[0] + R82XX_FILT_HP_BW1) {
            bw -= R82XX_FILT_HP_BW2;
            t->int_freq += R82XX_FILT_HP_BW2;
            real_bw += R82XX_FILT_HP_BW2;
        } else {
            reg_0b |= 0x20;
        }

        if (bw > r82xx_if_low_pass_bw_table[0]) {
            bw -= R82XX_FILT_HP_BW1;
            t->int_freq += R82XX_FILT_HP_BW1;
            real_bw += R82XX_FILT_HP_BW1;
        } else {
            reg_0b |= 0x40;
        }

        /* find low-pass filter */
        for (i = 0; i < 10; i++) {
            if (bw > r82xx_if_low_pass_bw_table[i]) {
                break;
            }
        }
        i--;
        reg_0b |= 15 - (int)i;
        real_bw += r82xx_if_low_pass_bw_table[i];

        t->int_freq -= real_bw / 2;
    }

    rc = r82xx_write_reg_mask(t, 0x0a, reg_0a, 0x10);
    if (rc < 0) return rc;
    rc = r82xx_write_reg_mask(t, 0x0b, reg_0b, 0xef);
    if (rc < 0) return rc;

    return t->int_freq;
}
