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
   find_device(); everything else mirrors struct r82xx_priv.
   int_freq is the tuner's ACTUAL analog intermediate frequency (Hz) --
   set to 3.57MHz by r82xx_init() (matching upstream's own TV-standard
   default), then updated by r82xx_set_bandwidth() to whatever the
   tuner's analog filter is really centred on for the requested
   capture bandwidth. r82xx_set_freq() uses this field (not a fixed
   constant) when computing the LO frequency -- see docs/PLAN.md for
   why a stale/mismatched value here was a real, previously-missed bug:
   the demod's digital downconversion must be told the SAME frequency
   the tuner's analog IF is actually centred on, or the two disagree by
   whatever the mismatch is, aliasing real content into what looks
   exactly like unstructured noise once that mismatch is comparable to
   or larger than the capture bandwidth's Nyquist limit. */
typedef struct {
    const char *dev_name;
    unsigned char regs[R82XX_NUM_REGS];
    unsigned char input;      /* current band: R82XX_BAND_HF/VHF/UHF */
    unsigned char fil_cal_code;
    int has_lock;
    int is_v4;                /* always 1 for v1 -- confirmed V4 hardware */
    int int_freq;              /* Hz -- see comment above */
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

/* Re-enables AGC for both LNA and Mixer (reg 0x05 bit4=0, reg 0x07
   bit4=1 -- opposite polarity between the two, confirmed from real
   upstream source, see R82XX.c). This is the untouched init array's
   own default (see docs/PLAN.md), so calling it is only needed to
   return to AGC after a prior r82xx_set_gain_manual() call. The I2C
   repeater must be enabled first (same as r82xx_set_freq()/
   r82xx_set_gain_manual()). Returns 0 on success, negative on I2C
   failure. */
int r82xx_set_gain_agc(r82xx_t *t);

/* Forces manual LNA+Mixer gain at a specific step, matching upstream's
   r82xx_set_gain(priv, 1, <target>) manual-mode register writes
   verbatim (LNA/Mixer auto-off bits, gain index in the low 4 bits of
   each register) but taking a direct step index (0-15, clamped) rather
   than porting upstream's incremental target-dB search loop -- this
   driver doesn't yet have the real upstream gain-step-to-dB tables
   ported, so a raw step index is what the gain control UI exposes.
   Call after r82xx_set_freq() has locked, with the I2C repeater
   enabled. Returns 0 on success, negative on I2C failure. */
int r82xx_set_gain_manual(r82xx_t *t, int index);

/* Computes and applies the tuner's actual analog IF/filter-bandwidth
   configuration for a requested capture bandwidth bw_hz (writes
   registers 0x0a/0x0b, matching upstream r82xx_set_bandwidth()
   verbatim -- note upstream's third parameter, `rate`, is declared but
   never actually used in the function body, so it's dropped here).
   Updates t->int_freq and returns its new value (>=0) on success, or
   negative on I2C failure. THE CALLER MUST THEN sync the demod's
   digital IF to match (Driver.h's rtlsdr_set_if_freq(), with the
   returned value) and re-tune (r82xx_set_freq() at the current
   frequency) -- matching upstream's r820t_set_bw(), which chains all
   three. Skipping either of those leaves the tuner's analog IF and the
   demod's digital downconversion mismatched, exactly the bug this
   function's addition fixes. Call with the I2C repeater enabled. */
int r82xx_set_bandwidth(r82xx_t *t, int bw_hz);

/* The busy-wait pause after every I2C transfer (a leftover from the
   hang hunt in docs/PLAN.md): 10 ms by default, 0 disables it. */
void r82xx_set_pace_ms(int ms);

#endif /* R82XX_H */
