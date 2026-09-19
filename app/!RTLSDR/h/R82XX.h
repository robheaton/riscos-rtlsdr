/* R82XX.h -- R820T/R828D tuner driver, ported from the rtlsdr-blog fork's
   src/tuner_r82xx.c + include/tuner_r82xx.h (verbatim register values and
   algorithm, read from the raw source -- not a paraphrase; see
   docs/PLAN.md for how this was researched, including a real hardware
   quirk that would have been silently missed from summarized fetches:
   r82xx_read() bit-reverses every byte it reads back over I2C).

   C89 only (Norcroft): all declarations at top of block, no // comments.

   Only the R828D path matters for v1 (confirmed hardware, see
   docs/PLAN.md) -- R820T-only branches from upstream are not ported. */

#ifndef R82XX_H
#define R82XX_H

#define R82XX_NUM_REGS          30   /* registers 0x05..0x22 */
#define R82XX_REG_SHADOW_START  0x05
#define R82XX_VER_NUM           49   /* 0x31, written to reg 0x13 */

/* Tuner state. dev_name is the "usbN" DeviceFS name from RTLSDR.c's
   find_device(); everything else mirrors struct r82xx_priv. */
typedef struct {
    const char *dev_name;
    unsigned char regs[R82XX_NUM_REGS];
    unsigned char input;      /* current band: R82XX_BAND_HF/VHF/UHF */
    unsigned char fil_cal_code;
    int has_lock;
    int is_v4;                /* always 1 for v1 -- confirmed V4 hardware */
} r82xx_t;

#define R82XX_BAND_HF   0
#define R82XX_BAND_VHF  1
#define R82XX_BAND_UHF  2

/* Returns 0 on success, negative on failure (matches upstream's rc<0
   convention). dev_name must be the DeviceFS name already confirmed by
   RTLSDR.c's find_device() (e.g. "usb6"). */
int r82xx_init(r82xx_t *t, const char *dev_name);

/* Tunes to freq_hz. Below 28.8MHz, V4's HF upconverter is engaged
   automatically (matches upstream: the offset is added transparently).
   Returns 0 on success (check t->has_lock), negative on I2C failure. */
int r82xx_set_freq(r82xx_t *t, unsigned long freq_hz);

#endif /* R82XX_H */
