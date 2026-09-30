/* Rx.h -- the receiver's demodulator front: picks the mode (WFM, NFM, AM)
   and runs raw I/Q through to PCM audio at the mixer rate.

   WFM is Dsp.c's validated path (CIC, discriminator, de-emphasis,
   resampler). The narrow-band modes share a front end with it and then
   run: FIR decimation 240 kSPS -> 48 kSPS, a mode-specific channel filter
   at 48 kSPS, the demodulator (FM discriminator or AM envelope), audio
   filters, a squelch gate, volume, and a resampler to the mixer rate.

   Everything on the per-sample path is integer arithmetic (this build has
   no hardware floating point); filter coefficients are designed once, with
   doubles, when a mode or bandwidth changes.

   Why these modes first: NFM covers PMR/marine/amateur VHF-UHF voice and
   data, AM covers airband and broadcast bands (docs/ROADMAP.md).

   C89 only (Norcroft). */

#ifndef RX_H
#define RX_H

#define RX_MODE_NFM  0
#define RX_MODE_WFM  1
#define RX_MODE_AM   2
#define RX_NMODES    3

/* Configures the audio rate (the TimPlayer mixer rate) and the dongle's
   input rate (a multiple of 240 kSPS), designs the fixed filters, selects
   WFM and resets everything. */
void rx_init(int audio_rate_hz, long input_rate_hz);

/* Clears all filter, AGC, squelch and resampler state (keeps the
   configuration). Call after a retune or when streaming (re)starts. */
void rx_reset(void);

/* Selects the mode. Redesigns the channel filter for the current
   bandwidth and resets state. */
void rx_set_mode(int mode);

/* Channel width in Hz for NFM and AM (the passband is +-bw/2); WFM ignores
   it. Clamped to a sane range for the mode. */
void rx_set_bandwidth(int bw_hz);

/* Squelch: 0 = off, 1..100 = increasingly strict. FM modes use noise
   quieting, AM and WFM use signal level. */
void rx_set_squelch(int level);

/* Volume 0..100 (a squared law, 70 is about unity) and mute. */
void rx_set_volume(int percent, int mute);

/* Demodulates nbytes of raw unsigned 8-bit I,Q pairs into up to max_out
   signed 16-bit mono samples at the mixer rate; returns how many. State
   carries across calls, so a stream can be fed in any chunk sizes. */
int rx_process(const unsigned char *iq, int nbytes, short *out, int max_out);

typedef struct {
    int have_dev;        /* peak_hz / rms_hz are valid (FM modes) */
    double peak_hz;
    double rms_hz;
    int level_db10;      /* channel level in dBFS x 10 (0 = full scale) */
    int squelch_open;    /* audio is passing */
} rx_stats;

/* Statistics since the previous call. Returns 1 if any samples have been
   processed since then, else 0 (and leaves *st untouched). */
int rx_take_stats(rx_stats *st);

/* For tests and tuning: the squelch's current noise metric (FM) or carrier
   amplitude (AM). */
long rx_debug_squelch_metric(void);

#endif
