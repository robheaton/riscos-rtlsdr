/* Rx.h -- the receiver's demodulator front: picks the mode (WFM, NFM, AM,
   USB, LSB, CW) and runs raw I/Q through to PCM audio at the mixer rate.

   WFM is Dsp.c's validated path (CIC, discriminator, de-emphasis,
   resampler). The narrow-band modes share a front end with it and then
   run: FIR decimation 240 kSPS -> 48 kSPS, a mode-specific channel filter
   at 48 kSPS, the demodulator (FM discriminator or AM envelope), audio
   filters, a squelch gate, volume, and a resampler to the mixer rate.

   USB, LSB and CW take the same front end, then shift the wanted passband
   to 0 Hz with a software oscillator (so the dongle can be tuned a little
   away from the signal, off its DC spike, and small tuning steps need no
   retune at all), decimate to 12 kSPS, filter with a long, sharp linear-
   phase FIR, shift the result to the audio pitch and take the real part,
   run an AGC, and interpolate back to 48 kSPS for the common output stage.

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
#define RX_MODE_USB  3
#define RX_MODE_LSB  4
#define RX_MODE_CW   5
#define RX_NMODES    6

/* The modes that use the oscillator/12 kSPS path and want the dongle tuned
   a little off the signal (see rx_set_offset). */
#define RX_MODE_IS_SSB(m)  ((m) >= RX_MODE_USB)

/* The pitch a CW carrier at the dial frequency is heard at, in Hz. */
#define RX_CW_PITCH_HZ  700

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

/* Channel width in Hz (NFM and AM: the passband is +-bw/2; USB and LSB: the
   width of the audio passband above 300 Hz; CW: the filter width around the
   dial frequency); WFM ignores it. Clamped to a sane range for the mode. */
void rx_set_bandwidth(int bw_hz);

/* The legal bandwidth range and the default for a mode (all 0 for WFM). */
void rx_bw_limits(int mode, int *min_hz, int *max_hz, int *default_hz);

/* Where the dial frequency (the carrier, for SSB and CW) is relative to the
   frequency the dongle is tuned to: positive = above it. Used by USB, LSB
   and CW; the other modes are always centred. Keep away from 0 (the
   dongle's DC spike). Takes effect immediately, without resetting any
   filter: that is what makes fine tuning instant. */
void rx_set_offset(long hz);

/* 1 if the I/Q stream is mirrored (the signal at +f is seen at -f). Only
   affects the SSB/CW modes and rx_channel(). Default 0. */
void rx_set_mirror(int mirrored);

/* The channel as it appears on the dongle's spectrum display: its centre
   relative to the tuned frequency and its width, in Hz. */
void rx_channel(long *centre_hz, long *width_hz);

/* Squelch: 0 = off, 1..100 = increasingly strict. FM modes use noise
   quieting, AM, WFM and the SSB modes use signal level. */
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
   amplitude (AM); for the SSB modes the AGC gain in Q8. */
long rx_debug_squelch_metric(void);

#endif
