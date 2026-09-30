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

/* Tuning limits. The lower limit is the UI's: the tuner code already
   up-converts below 28.8 MHz, but that path is untested (docs/ROADMAP.md,
   Phase 2). */
#define RCV_FREQ_MIN_HZ   24000000UL
#define RCV_FREQ_MAX_HZ   1700000000UL

#define RCV_NSTEPS        16

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

/* Formats a value for the window: "12.5k", "150k", "100 Hz"... */
void receiver_fmt_hz(char *buf, int buflen, long hz);

/* Pushes the state into the demodulator. apply_mode also resets its
   filters; the others are cheap and do not. */
void receiver_apply_mode(void);
void receiver_apply_bw(void);
void receiver_apply_squelch(void);
void receiver_apply_volume(void);

#endif
