/* Receiver.h -- the receiver's user-facing state (what is tuned, in which
   mode, with which bandwidth, step, squelch and volume), kept apart from
   the windows that show it and the DSP that uses it, plus its persistence
   in a Choices file.

   Per-mode settings (bandwidth, step) are remembered per mode, as SDR++
   does: change the step in AM and it is still what you left in NFM.

   C89 only (Norcroft). */

#ifndef RECEIVER_H
#define RECEIVER_H

#include "Rx.h"

/* Tuning limits. Below 28.8 MHz the V4's up-converter is used (the tuner is
   set 28.8 MHz higher); that path was written but had never been run on
   hardware when the SSB modes were added (docs/ROADMAP.md, Phase 2). */
#define RCV_FREQ_MIN_HZ   100000UL
#define RCV_FREQ_MAX_HZ   1700000000UL

#define RCV_NSTEPS        18

/* SSB and CW tune the dongle this far below the dial frequency (so the
   signal sits off the DC spike) and then move the dial about inside the
   dongle's span without retuning, for as long as the dial stays between
   the two limits above the dongle's frequency. */
#define RCV_OFF_HOME_HZ   45000L
#define RCV_OFF_MIN_HZ    10000L
#define RCV_OFF_MAX_HZ    80000L

typedef struct {
    unsigned long freq_hz;
    int mode;                        /* RX_MODE_... */
    int bw_hz[RX_NMODES];
    int step_idx[RX_NMODES];         /* index into receiver_steps[] */
    int squelch;                     /* 0 = off, 1..100 */
    int volume;                      /* 0..100 */
    int muted;
} receiver_t;

extern receiver_t rcv;
extern const int receiver_steps[RCV_NSTEPS];

/* Sets every field to its default (97.4 MHz WFM, per-mode defaults). */
void receiver_defaults(void);

/* Reads / writes Choices:RTLSDRView.Config (see the source for the path
   rules). Load keeps the defaults for anything missing or out of range.
   Save returns 0 on success, -1 if there is nowhere to write. */
void receiver_load(void);
int receiver_save(void);

const char *receiver_mode_name(int mode);

/* The current mode's step in Hz, and its index. */
int receiver_step_hz(void);
void receiver_step_next(int direction);   /* +1 / -1, clamped */

/* Channel bandwidth of the current mode, its limits and its +/- increment.
   WFM has a fixed channel, so min = max = 0 there. */
int receiver_bw_hz(void);
int receiver_bw_min(int mode);
int receiver_bw_max(int mode);
int receiver_bw_inc(int mode);

unsigned long receiver_clamp_freq(unsigned long hz);

/* The frequency the dongle has to be tuned to for this dial frequency and
   mode, given where it is tuned now (0 if it is not tuned yet). The dial
   itself for the modes that sit in the centre; for USB, LSB and CW, the
   current tuning again if the dial is inside the allowed window above it
   (so moving the dial needs no retune), otherwise a new tuning that puts
   the dial RCV_OFF_HOME_HZ above the dongle's frequency. */
unsigned long receiver_hw_target(unsigned long dial, int mode,
                                 unsigned long current_hw);

/* Formats a value for the window: "12.5k", "150k", "400Hz"... */
void receiver_fmt_hz(char *buf, int buflen, long hz);

/* Pushes the state into the demodulator. apply_mode also resets its
   filters; the others are cheap and do not. */
void receiver_apply_mode(void);
void receiver_apply_bw(void);
void receiver_apply_squelch(void);
void receiver_apply_volume(void);

#endif
