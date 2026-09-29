/* Audio.h -- minimal RISC OS audio output via the TimPlayer module,
   ported from the RISC-OS-native DigitalCD music player's own real,
   working !PlayTone example app (source supplied directly by the
   user, not guessed at or summarized -- see docs/PLAN.md). TimPlayer
   is a standard system module (normally already present, or loadable
   from System:Modules.Audio.Trackers.TimPlayer) that does all the
   actual interrupt-driven DMA/mixing work internally -- this code
   never touches assembler or an interrupt handler itself, it just
   fills a plain memory buffer with PCM samples and hands it to
   TimPlayer via documented SWIs.

   The test tone proved the whole chain works at all (module load, RMA
   buffer allocation, sample registration, looped playback). The
   streaming functions below build on that for real, continuously-
   generated audio (like FM demod output): a second RMA buffer, looped
   the same way, that the caller keeps refilling via
   audio_stream_feed() -- writing only as far "ahead" of wherever
   TimPlayer's playback position is currently estimated to be as is
   safe, so new data never overwrites audio that's about to be played.
   TimPlayer's real playback position isn't queried (no documented
   parameter semantics for TimPlayer_SongPosition were found -- see
   docs/PLAN.md); position is instead estimated from elapsed wall-clock
   time since audio_stream_start(), which needs the caller to actually
   call audio_stream_feed() often enough to keep up in real time (a
   couple of seconds' buffer plus a safety margin gives some slack, but
   this isn't a bulletproof scheduler guarantee).

   C89 only (Norcroft): all declarations at top of block, no //
   comments. */

#ifndef AUDIO_H
#define AUDIO_H

/* Ensures TimPlayer is loaded, claims an RMA buffer, fills it with a
   simple test tone, and registers it with TimPlayer as a loopable
   sample -- but does not start playback (see audio_test_tone_play()).
   Call once at startup. Returns 0 on success, negative on failure (the
   caller should treat failure as non-fatal -- the rest of the app
   doesn't depend on audio working). */
int audio_test_tone_init(void);

/* Starts (play != 0) or stops (play == 0) looped playback of the test
   tone set up by audio_test_tone_init(). Safe to call even if init()
   failed (does nothing). */
void audio_test_tone_play(int play);

/* Stops playback if running and releases everything
   audio_test_tone_init() claimed (the FX handler, the song/sample, and
   the RMA buffer). Call once at shutdown. Safe to call even if init()
   was never called or failed. */
void audio_test_tone_close(void);

/* Claims a second RMA buffer (a few seconds of silence, at the
   mixer's real configured rate) and registers it with TimPlayer as a
   second loopable sample, ready to be fed via audio_stream_feed().
   Requires audio_test_tone_init() to have already succeeded (reuses
   its FX/song handles rather than claiming its own -- this project
   only ever needs one FX handler and one song). Returns 0 on success,
   negative on failure (non-fatal, same as the test tone). */
int audio_stream_init(void);

/* Starts (play != 0) or stops (play == 0) looped playback of the
   streaming buffer, and (re)establishes the wall-clock reference
   audio_stream_feed()'s position estimate is based on. Safe to call
   even if audio_stream_init() failed or wasn't called (does nothing). */
void audio_stream_play(int play);

/* Appends up to n samples (16-bit signed PCM, already at
   audio_stream_rate() Hz -- the caller must decimate/resample its own
   source material to that rate first) into the streaming ring buffer,
   as far as there's safe room ahead of the estimated playback
   position. Returns how many were actually written (0..n) -- callers
   should be prepared for fewer than n (e.g. playback not started yet,
   or temporarily caught up to the play position) and just drop the
   rest rather than block; this is a real-time stream, not a queue with
   backpressure. */
int audio_stream_feed(const short *samples, int n);

/* The streaming buffer's actual sample rate in Hz (the mixer's real
   configured rate, read once during audio_stream_init()), or 0 if
   audio_stream_init() hasn't succeeded. */
int audio_stream_rate(void);

/* How far ahead of the estimated play position the write head is held
   (milliseconds), i.e. how long a stall in the caller the audio can ride
   out before it runs dry. Bigger rides out longer desktop stalls but adds
   that much latency. Takes effect from the next audio_stream_play(1) (or
   the next re-sync). Clamped to a sane range. Default 350. */
void audio_stream_set_lead_ms(int ms);

/* Number of genuine underruns (re-syncs after playback had already
   started) since audio_stream_init(). Each one is an audible pause. */
int audio_stream_underruns(void);

/* Releases the streaming buffer specifically (stops playback if
   running, releases its sample slot and RMA buffer) -- does NOT touch
   the FX/song handles or the test-tone buffer, since those are shared
   with / owned by audio_test_tone_*(). Call audio_test_tone_close()
   (which releases the shared FX/song handles too) once at shutdown;
   call this first if streaming was ever started. Safe to call even if
   audio_stream_init() was never called or failed. */
void audio_stream_close(void);

#endif /* AUDIO_H */
