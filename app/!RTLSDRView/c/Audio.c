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
