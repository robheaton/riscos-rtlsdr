/* Dsp.h -- integer-only real-time wideband-FM demodulator.

   Why integer-only: this project's compile flags (plan.json:
   `cc -c -za1 -Otime -apcs 3/32bit`, no -fpu option) build every
   `double` operation as emulated floating point, roughly 100x slower
   than integer maths on the same core. The old float demodulator ran
   atan2() on every one of the 2.4 million wideband samples per second
   and managed ~141 frames (36k samples) per second -- about 1.5% of
   real time -- which is why STREAM only ever produced a burst of static
   every couple of seconds. Everything on the per-sample path here is
   plain 32-bit integer arithmetic; floating point is touched only a few
   times per call, for statistics.

   Signal chain:
     unsigned 8-bit I/Q at the input rate (any multiple of 240 kSPS: the
     dongle is normally run at 240 kSPS or 2.4 MSPS)
     -> (input rate > 240 kSPS only) 3rd-order CIC decimator down to
     240 kSPS, which is also the channel filter: at 2.4 MSPS its nulls
     fall on the neighbouring 200 kHz channels. At exactly 240 kSPS
     this stage is skipped: the dongle's own decimator has already done
     the job, and it makes the whole thing ~10x cheaper in both CPU and
     USB traffic.
     -> 240 kSPS complex baseband -> polar discriminator via an integer
     atan2 (phase step between successive samples) -> 50 us de-emphasis
     (single pole) -> integrate-and-dump resampler to the audio rate
     -> signed 16-bit PCM.

   C89 only (Norcroft). */

#ifndef DSP_H
#define DSP_H

/* Rate after the (optional) CIC stage: the discriminator, de-emphasis
   and resampler all run at this rate. */
#define DSP_CHANNEL_RATE_HZ  240000L

/* Default input rate, used if dsp_fm_init() is given a nonsensical one. */
#define DSP_INPUT_RATE_HZ    2400000L

/* Configures the input sample rate (the rate rtlsdr_set_sample_rate()
   was given; should be a multiple of DSP_CHANNEL_RATE_HZ, 1x..16x) and
   the output audio rate (the TimPlayer mixer rate), and resets all
   state. audio_rate_hz must be > 0 and no higher than
   DSP_CHANNEL_RATE_HZ. */
void dsp_fm_init(int audio_rate_hz, long input_rate_hz);

/* Clears filter/discriminator state and statistics but keeps the
   configured rates. Call when streaming starts, so stale state from a
   previous session can't leak into the first samples. */
void dsp_fm_reset(void);

/* Volume, as a Q12 multiplier (4096 = 1.0). At 1.0 a +-75 kHz
   deviation maps to roughly full scale. */
void dsp_fm_set_gain(int gain_q12);

/* Demodulates nbytes of raw interleaved unsigned-8-bit I,Q,I,Q...
   (nbytes must be even; an odd trailing byte is ignored). Writes up to
   max_out signed 16-bit mono samples to out and returns how many were
   written. State carries across calls, so successive chunks of one
   contiguous stream can be passed in any sizes. If out fills, further
   output samples are dropped (the filters keep running so the stream
   stays in step). */
int dsp_fm_process(const unsigned char *iq, int nbytes, short *out,
                   int max_out);

/* Peak and RMS instantaneous frequency deviation (Hz) since the last
   call. Returns 1 and fills the outputs if any demodulated samples
   arrived since then, otherwise returns 0 and leaves them untouched. */
int dsp_fm_take_stats(double *peak_hz, double *rms_hz);

#endif
