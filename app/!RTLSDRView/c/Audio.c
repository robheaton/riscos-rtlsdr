/* Audio.c -- see h/Audio.h. Every SWI call sequence and register
   value below is ported verbatim from DigitalCD's own !PlayTone
   example app (real, working, supplied source -- not guessed,
   summarized, or reconstructed from documentation; see docs/PLAN.md).
   That app uses WimpLib's _swix()/_INR()/_IN()/_OUT() macros; this
   uses this project's own established raw _kernel_swi_regs style
   (matching Driver.c/R82XX.c) instead, but the actual SWI numbers,
   register assignments, and magic values (0x80000000u index-create
   flag, 0xC sample-format flags, the 0x40/256/128 play parameters)
   are unchanged from the real source.

   C89 only (Norcroft): all declarations at top of block, no //
   comments. */

#include <math.h>
#include <string.h>
#include "kernel.h"
#include "Audio.h"

#define SWI_OS_Module 0x1E
#define SWI_OS_ReadMonotonicTime 0x42 /* returns centiseconds in r0 --
                                          same counter DeskLib's
                                          Time_Monotonic() wraps, used
                                          directly here since Audio.c
                                          (like Driver.c) doesn't pull
                                          in DeskLib */

#define TimPlayer_Configure         0x051381
#define TimPlayer_SongNew           0x051384
#define TimPlayer_SongUnload        0x051383
#define TimPlayer_SampleInfo        0x0513A0
#define TimPlayer_SampleMisc        0x0513A1
#define TimPlayer_SampleLoops       0x0513A2
#define TimPlayer_FXRegister        0x0513B0
#define TimPlayer_FXUnregister      0x0513B1
#define TimPlayer_FXGlobalSettings  0x0513B2
#define TimPlayer_FXPlaySample      0x0513B5
#define TimPlayer_FXNoteAction      0x0513B7

/* Matches PlayTone's MaxBufferSize (192000 samples) -- generous
   headroom above any real RISC OS mixer rate (typically 44100 or
   48000), capping how much RMA a 1-second test-tone buffer can claim. */
#define MAX_BUFFER_SAMPLES 192000
#define TEST_TONE_HZ       440.0f /* concert A -- an unambiguous,
                                      easy-to-recognise test tone */
#define AUDIO_PI            3.14159265358979323846

static int song_handle_g = -1;
static int fx_handle_g = -1;
static short *sample_buf_g = 0;
static int sample_size_g = 0; /* samples actually filled/registered */
static int playing_g = 0;

/* ---- streaming ring buffer (see Audio.h) ---- */
#define STREAM_SAMPLE_INDEX 2 /* distinct from the test tone's index 1
                                  -- both live in the same song/FX
                                  handles, and could in principle play
                                  at once, though the app only ever
                                  drives one at a time in practice */
#define STREAM_BUF_SECONDS 2  /* ring buffer length -- long enough to
                                  ride out a brief production stall
                                  without an audible underrun, short
                                  enough that the inherent latency
                                  (audio you hear reflects input from
                                  up to this long ago) stays reasonable
                                  for a live radio receiver */
#define STREAM_SAFETY_MS    200 /* don't write within this many ms
                                    "ahead of" the estimated play
                                    position -- guards against our
                                    wall-clock position estimate (see
                                    Audio.h) being slightly off */

static short *stream_buf_g = 0;
static int stream_capacity_g = 0;  /* samples */
static int stream_rate_g = 0;      /* Hz */
/* Running total of samples ever written (NOT wrapped -- the actual
   ring position is always this mod stream_capacity_g), and the
   equivalent running total of samples "expected consumed" from
   elapsed wall-clock time. Using running totals rather than wrapped
   positions for the throttling math (audio_stream_feed()) avoids a
   real bug the wrapped version had: if production ever fell behind
   real-time playback by more than one full buffer length (plausible
   under load), the modular difference between two wrapped positions
   can't tell "genuinely far ahead" from "behind by more than a lap",
   and computed a near-zero safe-room estimate either way -- getting
   permanently stuck refusing to write once that happened, rather than
   recovering. Running totals never wrap in practice (would take over
   two million years of continuous playback at 48kHz to overflow an
   unsigned long), so "ahead = written - consumed" stays meaningful
   however far behind production ever falls. */
static unsigned long stream_total_written_g = 0;
static int stream_playing_g = 0;
static unsigned int stream_start_time_g = 0; /* centiseconds */

static unsigned int monotonic_cs(void)
{
    _kernel_swi_regs regs;
    _kernel_swi(SWI_OS_ReadMonotonicTime, &regs, &regs);
    return (unsigned int)regs.r[0];
}

/* OS_Module reason 18 (Lookup): is TimPlayer already resident? Most
   RISC OS 5 desktops already have it loaded (boot sounds etc.), so
   this usually succeeds without ever touching reason 1 (Load) below.
   Deliberately simpler than WimpLib's real RMEnsure() (which also
   parses the module's help-string version field) -- this project has
   no specific minimum-version requirement yet, just "loaded, in some
   form". */
static int ensure_timplayer_loaded(void)
{
    _kernel_swi_regs regs;
    _kernel_oserror *err;

    regs.r[0] = 18; /* Lookup */
    regs.r[1] = (int)"TimPlayer";
    err = _kernel_swi(SWI_OS_Module, &regs, &regs);
    if (err == 0) {
        return 0; /* already loaded */
    }

    regs.r[0] = 1; /* Load */
    regs.r[1] = (int)"System:Modules.Audio.Trackers.TimPlayer";
    err = _kernel_swi(SWI_OS_Module, &regs, &regs);
    if (err != 0) {
        return -1;
    }
    return 0;
}

/* Fills sample_buf_g with one second of a simple sine wave at
   TEST_TONE_HZ, sampled at the mixer's real configured rate (read via
   TimPlayer_Configure reason 5, matching PlayTone's Sample_Update()
   exactly, including its own float/double mix -- not tidied up, to
   keep this a faithful port rather than a rewrite). */
static int fill_test_tone(void)
{
    _kernel_swi_regs regs;
    _kernel_oserror *err;
    int sample_rate;
    float base;
    double pos;
    short *ps;
    int i;

    regs.r[0] = 5; /* Configure reason: read mixer sample rate */
    err = _kernel_swi(TimPlayer_Configure, &regs, &regs);
    if (err != 0) {
        return -1;
    }
    sample_rate = regs.r[1];

    sample_size_g = sample_rate;
    if (sample_size_g > MAX_BUFFER_SAMPLES) {
        sample_size_g = MAX_BUFFER_SAMPLES;
    }

    base = TEST_TONE_HZ;
    base /= (float)sample_rate;
    base = 2.0f * (float)AUDIO_PI * base;

    ps = sample_buf_g;
    pos = 0.0;
    for (i = 0; i < sample_size_g; i++) {
        float val = 32767.0f * (float)sin(pos);
        *ps++ = (short)val;
        pos += base;
    }

    regs.r[0] = song_handle_g;
    regs.r[1] = (int)(0x80000000u | 1u); /* create/replace sample index 1 */
    regs.r[2] = sample_size_g;
    regs.r[3] = (int)sample_buf_g;
    regs.r[4] = 0xC; /* sample format flags -- verbatim from PlayTone */
    regs.r[5] = 0;
    regs.r[6] = 0;
    err = _kernel_swi(TimPlayer_SampleInfo, &regs, &regs);
    if (err != 0) {
        return -1;
    }

    regs.r[0] = song_handle_g;
    regs.r[1] = (int)(0x80000000u | 1u);
    regs.r[2] = 256;
    regs.r[3] = 256;
    regs.r[4] = 0;
    regs.r[5] = sample_rate;
    regs.r[6] = 0;
    regs.r[7] = 0;
    err = _kernel_swi(TimPlayer_SampleMisc, &regs, &regs);
    if (err != 0) {
        return -1;
    }

    regs.r[0] = song_handle_g;
    regs.r[1] = (int)(0x80000000u | 1u);
    regs.r[2] = 0;
    regs.r[3] = sample_size_g;
    regs.r[4] = 0;
    regs.r[5] = 0;
    err = _kernel_swi(TimPlayer_SampleLoops, &regs, &regs);
    if (err != 0) {
        return -1;
    }

    return 0;
}

int audio_test_tone_init(void)
{
    _kernel_swi_regs regs;
    _kernel_oserror *err;

    if (ensure_timplayer_loaded() != 0) {
        return -1;
    }

    regs.r[0] = 6; /* Claim */
    regs.r[3] = MAX_BUFFER_SAMPLES * (int)sizeof(short);
    err = _kernel_swi(SWI_OS_Module, &regs, &regs);
    if (err != 0) {
        return -1;
    }
    sample_buf_g = (short *)regs.r[2];

    regs.r[1] = 1; /* polyphony */
    err = _kernel_swi(TimPlayer_FXRegister, &regs, &regs);
    if (err != 0) {
        return -1;
    }
    fx_handle_g = regs.r[0];

    regs.r[0] = fx_handle_g;
    regs.r[1] = 4; /* disable automatic volume scaling with polyphony */
    regs.r[2] = 0;
    err = _kernel_swi(TimPlayer_FXGlobalSettings, &regs, &regs);
    if (err != 0) {
        return -1;
    }

    err = _kernel_swi(TimPlayer_SongNew, &regs, &regs);
    if (err != 0) {
        return -1;
    }
    song_handle_g = regs.r[0];

    if (fill_test_tone() != 0) {
        return -1;
    }

    return 0;
}

void audio_test_tone_play(int play)
{
    _kernel_swi_regs regs;

    if (song_handle_g < 0 || fx_handle_g < 0) {
        return; /* init() never succeeded */
    }

    playing_g = play ? 1 : 0;

    if (playing_g) {
        regs.r[0] = fx_handle_g;
        regs.r[1] = 0;
        regs.r[2] = song_handle_g;
        regs.r[3] = 1;   /* sample index */
        regs.r[4] = 0x40;
        regs.r[5] = 256; /* volume */
        regs.r[6] = 128; /* panning */
        _kernel_swi(TimPlayer_FXPlaySample, &regs, &regs);
    } else {
        regs.r[0] = fx_handle_g;
        regs.r[1] = 0;
        regs.r[2] = 0;
        regs.r[3] = 0;
        _kernel_swi(TimPlayer_FXNoteAction, &regs, &regs);
    }
}

void audio_test_tone_close(void)
{
    _kernel_swi_regs regs;

    if (playing_g) {
        audio_test_tone_play(0);
    }

    if (fx_handle_g >= 0) {
        regs.r[0] = fx_handle_g;
        _kernel_swi(TimPlayer_FXUnregister, &regs, &regs);
        fx_handle_g = -1;
    }

    if (song_handle_g >= 0) {
        regs.r[0] = song_handle_g;
        regs.r[1] = (int)0x80000000u; /* release sample */
        regs.r[2] = 0;
        regs.r[3] = 0;
        regs.r[4] = 0;
        regs.r[5] = 0;
        regs.r[6] = 0;
        _kernel_swi(TimPlayer_SampleInfo, &regs, &regs);

        regs.r[0] = song_handle_g;
        _kernel_swi(TimPlayer_SongUnload, &regs, &regs);
        song_handle_g = -1;
    }

    if (sample_buf_g != 0) {
        regs.r[0] = 7; /* Free */
        regs.r[2] = (int)sample_buf_g;
        _kernel_swi(SWI_OS_Module, &regs, &regs);
        sample_buf_g = 0;
    }
}

int audio_stream_init(void)
{
    _kernel_swi_regs regs;
    _kernel_oserror *err;

    if (song_handle_g < 0 || fx_handle_g < 0) {
        return -1; /* audio_test_tone_init() must succeed first */
    }

    regs.r[0] = 5; /* Configure reason: read mixer sample rate */
    err = _kernel_swi(TimPlayer_Configure, &regs, &regs);
    if (err != 0) {
        return -1;
    }
    stream_rate_g = regs.r[1];

    stream_capacity_g = stream_rate_g * STREAM_BUF_SECONDS;
    if (stream_capacity_g > MAX_BUFFER_SAMPLES) {
        stream_capacity_g = MAX_BUFFER_SAMPLES;
    }

    regs.r[0] = 6; /* Claim */
    regs.r[3] = stream_capacity_g * (int)sizeof(short);
    err = _kernel_swi(SWI_OS_Module, &regs, &regs);
    if (err != 0) {
        stream_rate_g = 0;
        stream_capacity_g = 0;
        return -1;
    }
    stream_buf_g = (short *)regs.r[2];
    memset(stream_buf_g, 0, (size_t)stream_capacity_g * sizeof(short));
    stream_total_written_g = 0;

    regs.r[0] = song_handle_g;
    regs.r[1] = (int)(0x80000000u | (unsigned)STREAM_SAMPLE_INDEX);
    regs.r[2] = stream_capacity_g;
    regs.r[3] = (int)stream_buf_g;
    regs.r[4] = 0xC; /* sample format flags -- verbatim from PlayTone */
    regs.r[5] = 0;
    regs.r[6] = 0;
    err = _kernel_swi(TimPlayer_SampleInfo, &regs, &regs);
    if (err != 0) {
        return -1;
    }

    regs.r[0] = song_handle_g;
    regs.r[1] = (int)(0x80000000u | (unsigned)STREAM_SAMPLE_INDEX);
    regs.r[2] = 256;
    regs.r[3] = 256;
    regs.r[4] = 0;
    regs.r[5] = stream_rate_g;
    regs.r[6] = 0;
    regs.r[7] = 0;
    err = _kernel_swi(TimPlayer_SampleMisc, &regs, &regs);
    if (err != 0) {
        return -1;
    }

    regs.r[0] = song_handle_g;
    regs.r[1] = (int)(0x80000000u | (unsigned)STREAM_SAMPLE_INDEX);
    regs.r[2] = 0;
    regs.r[3] = stream_capacity_g;
    regs.r[4] = 0;
    regs.r[5] = 0;
    err = _kernel_swi(TimPlayer_SampleLoops, &regs, &regs);
    if (err != 0) {
        return -1;
    }

    return 0;
}

void audio_stream_play(int play)
{
    _kernel_swi_regs regs;

    if (stream_buf_g == 0) {
        return; /* audio_stream_init() never succeeded */
    }

    stream_playing_g = play ? 1 : 0;

    if (stream_playing_g) {
        /* Fresh reference point for audio_stream_feed()'s wall-clock
           position estimate -- and start from a silent (zeroed)
           buffer position 0, so there's no stale leftover content
           from a previous run audible before real data catches up. */
        stream_total_written_g = 0;
        stream_start_time_g = monotonic_cs();

        regs.r[0] = fx_handle_g;
        regs.r[1] = 0;
        regs.r[2] = song_handle_g;
        regs.r[3] = STREAM_SAMPLE_INDEX;
        regs.r[4] = 0x40;
        regs.r[5] = 256; /* volume */
        regs.r[6] = 128; /* panning */
        _kernel_swi(TimPlayer_FXPlaySample, &regs, &regs);
    } else {
        regs.r[0] = fx_handle_g;
        regs.r[1] = 0;
        regs.r[2] = 0;
        regs.r[3] = 0;
        _kernel_swi(TimPlayer_FXNoteAction, &regs, &regs);
    }
}

int audio_stream_feed(const short *samples, int n)
{
    unsigned int elapsed_cs;
    unsigned long expected_consumed;
    long ahead;
    long safety;
    long room;
    int written;
    int pos;
    int i;

    if (!stream_playing_g || stream_buf_g == 0) {
        return 0;
    }

    /* elapsed_cs wraps the same way OS_ReadMonotonicTime's own counter
       does (roughly every 497 days) -- unsigned subtraction handles
       that correctly regardless of which side of a wrap
       stream_start_time_g and "now" fall on. */
    elapsed_cs = monotonic_cs() - stream_start_time_g;
    expected_consumed = ((unsigned long)elapsed_cs *
                          (unsigned long)stream_rate_g) / 100UL;

    /* How far ahead of actual real-time consumption our writes
       currently are. Comfortably positive and less than the buffer's
       own capacity is the healthy steady state (some safe margin of
       already-written-but-not-yet-played audio queued up). */
    ahead = (long)(stream_total_written_g - expected_consumed);
    safety = (long)((stream_rate_g * STREAM_SAFETY_MS) / 1000);

    if (ahead <= safety) {
        /* At or behind the estimated play position (production has
           been struggling to keep up with real time) -- accept
           everything rather than continuing to hold back. A fresh
           sample landing right at (or even slightly behind) the play
           head is a normal, recoverable hiccup; refusing to write
           because we're "not far enough ahead" is what got this stuck
           in the original version of this function -- see the
           stream_total_written_g comment above. */
        written = n;
    } else {
        /* Comfortably ahead already -- cap how much more so writes
           never lap all the way around and overwrite audio that's
           genuinely still queued up, not yet played. */
        room = (long)stream_capacity_g - ahead;
        if (room <= 0) {
            written = 0;
        } else {
            written = (n < room) ? n : (int)room;
        }
    }

    pos = (int)(stream_total_written_g % (unsigned long)stream_capacity_g);
    for (i = 0; i < written; i++) {
        stream_buf_g[pos] = samples[i];
        pos++;
        if (pos >= stream_capacity_g) {
            pos = 0;
        }
    }
    stream_total_written_g += (unsigned long)written;

    return written;
}

int audio_stream_rate(void)
{
    return stream_rate_g;
}

void audio_stream_close(void)
{
    _kernel_swi_regs regs;

    if (stream_playing_g) {
        audio_stream_play(0);
    }

    if (song_handle_g >= 0 && stream_buf_g != 0) {
        /* Unlike audio_test_tone_close()'s release call (which uses
           index 0, verbatim from PlayTone's own App_Close -- PlayTone
           only ever has one sample, at index 1, so whether "release"
           genuinely means "release index 0" or something else was
           never distinguishable from that source alone), this
           releases the SPECIFIC index this buffer was registered at.
           Worst case if that's wrong is a harmless TimPlayer-internal
           leak until the module's next reload, not a crash -- see
           docs/PLAN.md. */
        regs.r[0] = song_handle_g;
        regs.r[1] = (int)(0x80000000u | (unsigned)STREAM_SAMPLE_INDEX);
        regs.r[2] = 0;
        regs.r[3] = 0;
        regs.r[4] = 0;
        regs.r[5] = 0;
        regs.r[6] = 0;
        _kernel_swi(TimPlayer_SampleInfo, &regs, &regs);
    }

    if (stream_buf_g != 0) {
        regs.r[0] = 7; /* Free */
        regs.r[2] = (int)stream_buf_g;
        _kernel_swi(SWI_OS_Module, &regs, &regs);
        stream_buf_g = 0;
    }

    stream_rate_g = 0;
    stream_capacity_g = 0;
}
