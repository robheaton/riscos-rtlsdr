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

   Signal chain (all at the input's 2.4 MSPS unless noted):
     unsigned 8-bit I/Q  ->  3rd-order CIC decimate-by-10 (also the
     channel filter: nulls fall on the neighbouring 200 kHz channels)
     -> 240 kSPS complex baseband -> polar discriminator via an integer
     atan2 (phase step between successive samples) -> 50 us de-emphasis
     (single pole) -> integrate-and-dump resampler to the audio rate
     -> signed 16-bit PCM.

   C89 only (Norcroft). */

#ifndef DSP_H
#define DSP_H

/* Sample rate the demodulator expects at its input, and the rate after
   the CIC stage. The input rate is fixed by rtlsdr_set_sample_rate()
   in SpecView.c's main(). */
#define DSP_INPUT_RATE_HZ    2400000L
#define DSP_CHANNEL_RATE_HZ  240000L

/* Configures the output audio rate (the TimPlayer mixer rate) and
   resets all state. audio_rate_hz must be > 0 and no higher than
   DSP_CHANNEL_RATE_HZ. */
void dsp_fm_init(int audio_rate_hz);

/* Clears filter/discriminator state and statistics but keeps the
   configured audio rate. Call when streaming starts, so stale state
   from a previous session can't leak into the first samples. */
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
