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

   This first pass proves the whole audio chain works at all (module
   load, RMA buffer allocation, sample registration, looped playback)
   with a simple generated test tone -- NOT the real FM-demodulated
   audio yet. Streaming the real demod output needs periodically
   rewriting portions of the loop buffer ahead of TimPlayer's current
   playback position, which needs more research (e.g.
   TimPlayer_SongPosition) before attempting -- see docs/PLAN.md.

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

#endif /* AUDIO_H */
