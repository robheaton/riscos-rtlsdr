/* SpecView.c -- !RTLSDRView: a live spectrum display for the RTL-SDR
   RISC OS driver. Phase 2, milestone 1 (see ../../../docs/PLAN.md):
   proves the architecture (continuous USB reads inside a Wimp idle
   handler without freezing the desktop, an FFT, and a redraw loop) that
   everything after this builds on. No demodulation, no audio, no tuning
   UI yet -- fixed at 97.4MHz (FM broadcast band), 2.4 MSPS.

   Device bringup (find_device, baseband/tuner init, sample rate, raw
   OS_Find/OS_GBPB stream I/O) reuses c/Driver.c -- the same code proven
   across !RTLSDR's milestones 1-4, not reimplemented here.

   No Templates-editor GUI is available in this environment, so the
   window is built programmatically (a hand-filled window_block passed
   to Wimp_CreateWindow), not loaded from a template file.

   The FFT itself still avoids any libm call (sqrt/sin/cos/log): its
   twiddle table is precomputed offline (matches upstream reasoning for
   avoiding runtime trig -- see R82XX.c's sigma-delta PLL loop comments).
   The display SCALE, though, does call log10() (see compute_spectrum) --
   a first real run with a purely linear/rational power scale showed why
   that's not optional here: RTL-SDR's classic DC-spike artifact sits
   40-60dB above the noise floor (10^4-10^6x in raw power), which no
   single linear-ish curve can show alongside the noise floor without one
   end vanishing or the other saturating (confirmed: it did exactly that,
   twice, with two different linear-family scales -- see docs/PLAN.md).
   log10() was avoided in the very first version purely because runtime
   libm CALLS (distinct from +,-,*,/ on doubles, which were already
   confirmed working via Driver.c's sample-rate math) were unconfirmed
   against this toolchain's link setup -- resolved by just trying it
   through the build service, which would fail at LINK time with a clear
   "undefined symbol" if unavailable, rather than needing a hardware
   round-trip to find out.

   C89 only (Norcroft): all declarations at top of block, no // comments. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "DeskLib:Core.h"
#include "DeskLib:Wimp.h"
#include "DeskLib:WimpSWIs.h"
#include "DeskLib:Window.h"
#include "DeskLib:Event.h"
#include "DeskLib:Handler.h"
#include "DeskLib:GFX.h"
#include "DeskLib:Time.h"
#include "RTLSDR.h"
#include "R82XX.h"
#include "Driver.h"
#include "Audio.h"
#include "Dsp.h"
#include "Rx.h"
#include "Receiver.h"
#include "Ui.h"
#include "Trace.h"

/* ---- FFT / spectrum geometry ----
   256-point complex FFT. Reading exactly one frame (256 complex I,Q byte
   pairs = 512 bytes) per os_gbpb_read4() call matches one of the sizes
   milestone 4's sweep proved fully reliable (64/128/256/512/1024 bytes
   all 5/5 instant; 2048 blocked ~110s) -- deliberately staying inside
   that known-safe boundary rather than picking an arbitrary frame size. */
#define FFT_SIZE      256
#define NUM_BINS      FFT_SIZE   /* one bar per complex output bin --
                                     covers the full -Fs/2..+Fs/2 range,
                                     not just half (unlike a real-input
                                     FFT), since our input is complex I/Q */
#define IQ_BYTES      (FFT_SIZE * 2)

#define WORK_WIDTH    640   /* OS units: room for the control panel's
                               buttons (see Ui.h); the spectrum is
                               stretched to fit */
/* x of the left edge of bin i, in work-area units. */
#define BIN_X(i)      (((i) * WORK_WIDTH) / NUM_BINS)
#define WORK_HEIGHT   480                        /* OS units -- grown from 360 for the
                                                     control panel, and from
                                                     300 to fit a 6th
                                                     diagnostic text line
                                                     without crowding the
                                                     bars below it */

/* Space reserved at the TOP of the window for the diagnostic text
   lines and the gain/freq/demod control icons (create_gain_icons/
   create_freq_icons/create_demod_icon -- their row sits at y -60..-92),
   so a tall bar can never grow up into and visually cover them. Bars
   are Wimp-drawn graphics in the app's own redraw loop, not icons, so
   nothing stops them overlapping icons except keeping their own max
   height below where the icons start. finalize_display() scales into
   BAR_MAX_HEIGHT, not the full WORK_HEIGHT. Grown from 100 to 140 to
   fit a second text line (demod readout) below the icon row.
   (Briefly grown further to 176 for a temporary 3rd line -- idx@/
   nElev -- while re-checking a byte-level finding post-IF-fix; that
   line found and confirmed the actual flat-spectrum root cause, see
   docs/PLAN.md, and has been removed now that it's done its job.)
   Grown again to 180 to fit a second icon row (create_tone_icon) plus
   the demod readout text below it, now that row 1 (AGC through DEM)
   is full. */
#define RESERVED_TOP    280   /* was 180 before the receiver control panel
                                 (Ui.c) added three more rows, a level meter
                                 and a wider readout */
#define REDRAW_TOP      (-208) /* the part of the window that changes at the
                                 display rate: meter, readout, spectrum */
#define BAR_MAX_HEIGHT  (WORK_HEIGHT - RESERVED_TOP)

/* Redraw throttle, separate from how aggressively Null_spectrum drains
   the USB stream (see MAX_READ_ATTEMPTS there) -- redraw only every
   UPDATE_INTERVAL_CS centiseconds so a plain full-window repaint at
   native Wimp_Poll rate doesn't waste CPU/flicker. */
#define UPDATE_INTERVAL_CS 20   /* ~5Hz redraw */

/* finalize_display() auto-scales the (now power-averaged) min/max dB
   range to
   the window height, rather than using a fixed DB_FLOOR/DB_CEIL -- three
   iterations of guessing a fixed range (see docs/PLAN.md) kept failing
   because the right constant depends on absolute signal strength, which
   varies with antenna/location/gain and can't be picked correctly in
   advance. Real spectrum analyzers auto-scale their Y-axis for the same
   reason. DC_EXCLUDE_BINS excludes the bins right around the display
   centre (where the FFT-shift places 0Hz/DC) from that min/max
   calculation: RTL-SDR's DC-spike artifact is a hardware/ADC artifact,
   not received signal, and including it would let it dominate the range
   and crush everything else back toward zero -- exactly what a fixed
   DB_CEIL guess kept doing anyway. The DC bin itself is still PLOTTED
   (just not used to pick the scale), so it'll typically still show as a
   tall/clipped spike, same as real SDR software. */
#define DC_EXCLUDE_BINS 2

/* ---- twiddle table: cos/sin(-2*pi*k/FFT_SIZE), k=0..FFT_SIZE/2-1 ----
   Precomputed offline (Python), not generated by any runtime trig call. */
static const double twiddle_cos[FFT_SIZE / 2] = {
    1.0000000000, 0.9996988187, 0.9987954562, 0.9972904567,
    0.9951847267, 0.9924795346, 0.9891765100, 0.9852776424,
    0.9807852804, 0.9757021300, 0.9700312532, 0.9637760658,
    0.9569403357, 0.9495281806, 0.9415440652, 0.9329927988,
    0.9238795325, 0.9142097557, 0.9039892931, 0.8932243012,
    0.8819212643, 0.8700869911, 0.8577286100, 0.8448535652,
    0.8314696123, 0.8175848132, 0.8032075315, 0.7883464276,
    0.7730104534, 0.7572088465, 0.7409511254, 0.7242470830,
    0.7071067812, 0.6895405447, 0.6715589548, 0.6531728430,
    0.6343932842, 0.6152315906, 0.5956993045, 0.5758081914,
    0.5555702330, 0.5349976199, 0.5141027442, 0.4928981922,
    0.4713967368, 0.4496113297, 0.4275550934, 0.4052413140,
    0.3826834324, 0.3598950365, 0.3368898534, 0.3136817404,
    0.2902846773, 0.2667127575, 0.2429801799, 0.2191012402,
    0.1950903220, 0.1709618888, 0.1467304745, 0.1224106752,
    0.0980171403, 0.0735645636, 0.0490676743, 0.0245412285,
    0.0000000000, -0.0245412285, -0.0490676743, -0.0735645636,
    -0.0980171403, -0.1224106752, -0.1467304745, -0.1709618888,
    -0.1950903220, -0.2191012402, -0.2429801799, -0.2667127575,
    -0.2902846773, -0.3136817404, -0.3368898534, -0.3598950365,
    -0.3826834324, -0.4052413140, -0.4275550934, -0.4496113297,
    -0.4713967368, -0.4928981922, -0.5141027442, -0.5349976199,
    -0.5555702330, -0.5758081914, -0.5956993045, -0.6152315906,
    -0.6343932842, -0.6531728430, -0.6715589548, -0.6895405447,
    -0.7071067812, -0.7242470830, -0.7409511254, -0.7572088465,
    -0.7730104534, -0.7883464276, -0.8032075315, -0.8175848132,
    -0.8314696123, -0.8448535652, -0.8577286100, -0.8700869911,
    -0.8819212643, -0.8932243012, -0.9039892931, -0.9142097557,
    -0.9238795325, -0.9329927988, -0.9415440652, -0.9495281806,
    -0.9569403357, -0.9637760658, -0.9700312532, -0.9757021300,
    -0.9807852804, -0.9852776424, -0.9891765100, -0.9924795346,
    -0.9951847267, -0.9972904567, -0.9987954562, -0.9996988187
};

static const double twiddle_sin[FFT_SIZE / 2] = {
    -0.0000000000, -0.0245412285, -0.0490676743, -0.0735645636,
    -0.0980171403, -0.1224106752, -0.1467304745, -0.1709618888,
    -0.1950903220, -0.2191012402, -0.2429801799, -0.2667127575,
    -0.2902846773, -0.3136817404, -0.3368898534, -0.3598950365,
    -0.3826834324, -0.4052413140, -0.4275550934, -0.4496113297,
    -0.4713967368, -0.4928981922, -0.5141027442, -0.5349976199,
    -0.5555702330, -0.5758081914, -0.5956993045, -0.6152315906,
    -0.6343932842, -0.6531728430, -0.6715589548, -0.6895405447,
    -0.7071067812, -0.7242470830, -0.7409511254, -0.7572088465,
    -0.7730104534, -0.7883464276, -0.8032075315, -0.8175848132,
    -0.8314696123, -0.8448535652, -0.8577286100, -0.8700869911,
    -0.8819212643, -0.8932243012, -0.9039892931, -0.9142097557,
    -0.9238795325, -0.9329927988, -0.9415440652, -0.9495281806,
    -0.9569403357, -0.9637760658, -0.9700312532, -0.9757021300,
    -0.9807852804, -0.9852776424, -0.9891765100, -0.9924795346,
    -0.9951847267, -0.9972904567, -0.9987954562, -0.9996988187,
    -1.0000000000, -0.9996988187, -0.9987954562, -0.9972904567,
    -0.9951847267, -0.9924795346, -0.9891765100, -0.9852776424,
    -0.9807852804, -0.9757021300, -0.9700312532, -0.9637760658,
    -0.9569403357, -0.9495281806, -0.9415440652, -0.9329927988,
    -0.9238795325, -0.9142097557, -0.9039892931, -0.8932243012,
    -0.8819212643, -0.8700869911, -0.8577286100, -0.8448535652,
    -0.8314696123, -0.8175848132, -0.8032075315, -0.7883464276,
    -0.7730104534, -0.7572088465, -0.7409511254, -0.7242470830,
    -0.7071067812, -0.6895405447, -0.6715589548, -0.6531728430,
    -0.6343932842, -0.6152315906, -0.5956993045, -0.5758081914,
    -0.5555702330, -0.5349976199, -0.5141027442, -0.4928981922,
    -0.4713967368, -0.4496113297, -0.4275550934, -0.4052413140,
    -0.3826834324, -0.3598950365, -0.3368898534, -0.3136817404,
    -0.2902846773, -0.2667127575, -0.2429801799, -0.2191012402,
    -0.1950903220, -0.1709618888, -0.1467304745, -0.1224106752,
    -0.0980171403, -0.0735645636, -0.0490676743, -0.0245412285
};

/* Every read request must be one of the round power-of-two sizes
   milestone 4's sweep + milestone 5's sentinel test actually validated
   as getting HONEST non-blocking short-read counts (64/128/256/512
   bytes) -- see docs/PLAN.md. Requesting the shrinking, arbitrary-sized
   REMAINDER (IQ_BYTES - accum_fill_g) on each successive call, as this
   used to do, turned out to resurface the exact padding bug that fix
   was meant to kill for any non-round size: live diagnostics showed
   only the FIRST ~2 bytes of every 512-byte frame were ever genuinely
   written (idx@0/nElev=1 rock-solid, and bytes at the frame's middle
   and end sitting at exactly 0 -- their untouched zero-init value --
   forever), meaning accum_fill_g was reaching IQ_BYTES almost entirely
   from fake reported counts, not real data. Fixed by always requesting
   this SAME fixed, validated chunk size regardless of how much of the
   frame is already filled, carrying any overshoot past the frame
   boundary into the next frame instead of trying to request an exact
   remainder. accum_buf_g needs FIXED_CHUNK_SIZE bytes of headroom past
   IQ_BYTES to safely hold that overshoot.

   256 showed idx@ (the frame's dominant sample) alternating between
   EXACTLY 0 and EXACTLY 128 -- the boundary between that size's two
   256-byte read chunks. Retesting at 512 (one atomic read per frame,
   after also fixing the analog/digital IF mismatch above, which
   confounded an earlier 512-byte test) showed idx@ PERMANENTLY stuck
   at 0 -- i.e. regardless of chunk size, only the very FIRST sample of
   ANY INDIVIDUAL READ CALL is genuinely fresh; the rest of that same
   call's data, despite passing the sentinel test (differs from a 0xAA
   fill), isn't real advancing ADC content. That's consistent with the
   read being padded by REPEATING an earlier byte rather than zero-
   padding -- which "differs from 0xAA" doesn't catch, unlike the
   original zero-padding bug this sentinel test was built to catch.
   If only the first sample or two of any read is genuinely fresh, a
   MUCH SMALLER request size should capture a much higher fraction of
   real data (at the cost of many more individual SWI calls per frame
   -- non-blocking mode means none of them can hang, so this is safe
   to test even though it's a lot more calls). 8 bytes = 4 samples,
   smaller than anything in milestone 4's original size sweep, DID
   fix the spectrum (real multi-peak shape, confirmed on hardware) --
   but real achieved throughput measured only ~15k samples/sec against
   the SDR's actual 2.4M (see app_start_time_g's diagnostic comment in
   Redraw_spectrum), ~160x too slow for any real-time audio, and per-
   call SWI overhead (not USB hardware bandwidth) is the obvious
   suspect: 8 bytes needs 60+ individual calls to fill one 256-sample
   frame. Never tested anything BETWEEN 8 (correct, overhead-bound) and
   256 (fast, corrupted) -- 64 was tried and, on real hardware, broke
   the spectrum again (back to the flat/noisy pattern) while only
   raising achieved throughput to ~40k -- so the honesty threshold
   sits somewhere between 8 and 64, and 40k is still nowhere near
   2.4M anyway. 16 confirmed correct on real hardware (r=23k, up from
   15k at 8). 32 confirmed BROKEN too (r=33k, but the spectrum became a
   comb of evenly-spaced sharp spikes -- the classic signature of a
   periodic artifact repeating at a fixed sample interval, same
   category as the earlier 256-byte idx@0/128-alternating finding,
   just at a different period). So the honesty threshold sat between
   16 and 32 -- UNTIL the real root cause was found by reading
   USBDriver's actual source (gitlab.riscosopen.org): its own
   read_cb() zero-pads the remainder of any request that completes
   short of the full size, a SECOND padding mechanism entirely
   separate from (and not fixed by) the earlier non-blocking-mode fix.
   The stream-open path now includes the real, documented `nopad` flag
   that disables this at the source (see os_find_open()'s call site in
   main()) -- 512 (one atomic read per frame) confirmed CORRECT with
   nopad on real hardware (real multi-peak spectrum), but throughput
   stayed stuck around r=7-9k regardless of size (512, then 64 with
   MAX_FRAMES_PER_TICK raised 8x -- barely moved either time), ruling
   out chunk size and that cap as the bottleneck. Removing the leftover
   0xAA sentinel-fill diagnostic (pure per-attempt overhead once nopad
   fixed correctness at the source) barely moved it either (r=10k).

   Reading USBDriver's actual start_read()/read_cb() source (see
   MAX_READ_ATTEMPTS's got==0 handling below) found the real driver
   model: only ONE bulk transfer is ever in flight, submitted by one
   read() call and completed asynchronously via an interrupt callback
   that a LATER read() call picks up. Changing got==0 from "give up"
   to "retry" (so the loop actually waits out that one transfer
   instead of surrendering to Wimp_Poll after each one) barely moved
   throughput either (r=10-11k) -- meaning per-transfer completion
   latency itself (scheduling + interrupt latency, not our polling
   strategy) is the real fixed cost per read() call, roughly ~25ms
   judging by ~39 frames/sec x 512 bytes matching r=10k. At 512 bytes,
   that wire time is negligible next to 25ms (USB2 HS could carry it
   in microseconds) -- so if that ~25ms really is a fixed per-call
   cost regardless of transfer size, a much bigger request should
   amortize it across far more data per transfer, multiplying
   throughput almost linearly instead of chasing overhead that turned
   out not to matter. Untested until now since every earlier large-
   chunk attempt (256/512 before nopad) was testing the PADDING
   question, not this latency question -- nopad means those old
   correctness failures shouldn't recur at any size. Jumping to 16384
   (32x) to test this aggressively rather than creeping up. */
#define FIXED_CHUNK_SIZE 16384

/* ---- global state ---- */
static char device_name_g[16];
static int stream_handle_g = 0;
static unsigned char iq_frame_g[IQ_BYTES];
static int bin_height_g[NUM_BINS];
static unsigned int next_update_time_g;
static window_handle spectrum_window_g;

/* r82xx_t used to be a local in main() -- the gain-control icon click
   handler (Click_spectrum, below) needs to call r82xx_set_gain_agc()/
   r82xx_set_gain_manual() on it too, so it's file-scope now. Safe: it's
   still only ever initialised once in main() before the window/icons
   are created (so no handler can run before it's valid), and main()
   never returns. */
static r82xx_t tuner_g;

/* ---- gain control UI ---- */
#define GAIN_MODE_AGC     0
#define GAIN_MODE_MANUAL  1
#define GAIN_INDEX_MAX    15   /* r82xx_set_gain_manual()'s step range */
#define GAIN_INDEX_DEFAULT 8   /* first manual step, when switching away
                                   from AGC via +/- rather than a direct
                                   AGC-off click -- mid-range, not max
                                   (max gain was the leading suspect for
                                   the ADC-overload pattern chased
                                   earlier this session -- see
                                   docs/PLAN.md -- even though removing
                                   it made no measurable difference,
                                   starting manual mode at max again by
                                   default isn't well-motivated). */
static int gain_mode_g = GAIN_MODE_AGC;
static int gain_index_g = GAIN_INDEX_DEFAULT;
static icon_handle icon_agc_g = 0;
static icon_handle icon_gaindown_g = 0;
static icon_handle icon_gainup_g = 0;

/* ---- frequency tuning UI ----
   Step buttons rather than a writable text-entry icon: matches the
   gain control's established, already-proven-on-hardware pattern
   (Click_spectrum, create_button_icon) instead of adding keyboard/
   caret handling for a first pass. 100kHz matches typical FM broadcast
   channel spacing (100-200kHz depending on region) -- fine enough to
   walk onto a station, coarse enough that reaching a distant one
   doesn't take forever. Range is a broad, safe bound on what the
   R820T/R828D + this port's freq_ranges table (R82XX.c) can tune to,
   not a precise hardware limit -- r82xx_set_mux()'s range lookup
   degrades gracefully (falls back to the table's last entry) above its
   explicit 650MHz top rather than misbehaving, so this doesn't need to
   match that table exactly. */
#define FREQ_STEP_HZ        100000UL
#define FREQ_MIN_HZ       24000000UL
#define FREQ_MAX_HZ     1700000000UL
#define FREQ_DEFAULT_HZ   97400000UL /* local FM broadcast -- see main() */
static unsigned long tuned_freq_hz_g = FREQ_DEFAULT_HZ;
static icon_handle icon_freqdown_g = 0;
static icon_handle icon_frequp_g = 0;

/* ---- FM demodulation (numeric readout only -- no audio output yet) ----
   Real streaming PCM playback needs RISC OS's interrupt-driven "linear
   handler" sound mechanism or the SharedSound module's streaming API,
   neither of which is grounded in real, fetched documentation the way
   the DeviceFS/tuner work was -- guessing at undocumented SWI numbers
   for something interrupt-driven is a bad idea (see the GPIO/UpCall
   risk this project already backed away from once). This computes and
   displays the actual demodulated deviation instead, as a first,
   lower-risk step: proves the DSP is right before any audio-subsystem
   research/risk is taken on.

   Gated behind a toggle (default OFF): a real phase discriminator is
   one atan2() per sample, up to 255 per frame -- real added CPU cost
   on top of everything already fixed for resource use this session,
   not worth paying for users who don't care about it. */
#define DEMOD_SAMPLE_RATE_HZ  2400000.0 /* matches rtlsdr_set_sample_rate()
                                            in main() */
#define DEMOD_PI              3.14159265358979323846
#define DEMOD_ALPHA           0.3   /* applied once per display update now; faster than the spectrum's
                                        AVG_ALPHA -- a responsive
                                        VU-meter feel, not a slow trace */
static int demod_enabled_g = 0;
static icon_handle icon_demod_g = 0;
static double demod_dev_rms_g = 0.0;  /* Hz, EMA-smoothed */
static double demod_dev_peak_g = 0.0; /* Hz, EMA-smoothed */
static int level_db10_g = -999;       /* channel level, dBFS x 10 */
static int squelch_open_g = 1;

/* ---- audio test tone (see Audio.h) ----
   First-pass proof that RISC OS audio output works at all from this
   app (TimPlayer module load, RMA buffer, looped sample playback) --
   a generated 440Hz tone, NOT the real FM-demodulated audio yet.
   audio_ok_g tracks whether audio_test_tone_init() actually succeeded
   (TimPlayer might not be present/loadable on a given machine) --
   toggling icon_tone_g does nothing if it didn't, rather than trying
   to play into an uninitialised buffer. */
static int audio_ok_g = 0;
static int tone_playing_g = 0;
static icon_handle icon_tone_g = 0;

/* ---- audio streaming (see Audio.h) ----
   Real FM-demodulated audio (Dsp.c: CIC channel filter, integer
   discriminator, de-emphasis) resampled to whatever rate the mixer/
   TimPlayer actually runs at, and fed continuously into a looping ring
   buffer. stream_ok_g mirrors audio_ok_g's role -- tracks whether
   audio_stream_init() succeeded, since it depends on
   audio_test_tone_init() having already claimed the shared FX/song
   handles. Clicking STREAM also turns demod_enabled_g on if it wasn't
   already (streaming needs the same per-sample computation the DEM
   readout uses; no reason to make the user click both). */
static int stream_ok_g = 0;
static int stream_enabled_g = 0;
static icon_handle icon_stream_g = 0;

/* FFT frames folded into the running average since the last display
   update, and a slice counter used to spread them out over the update
   interval. The FFT is the only floating-point-heavy part of the
   per-tick work, so it is budgeted rather than run on every frame; how
   many frames per update, and from how many of the ~16 KB slices of
   incoming data, is set at start-up (see main()). */
#define PCM_BUF_SAMPLES       8192
static int fft_per_update_g = 4;
static int fft_stride_g = 1;
static double avg_alpha_g = 0.04;
static int fft_frames_g = 0;
static unsigned long chunk_seq_g = 0;

/* The USB stream. The dongle's sample rate, the size of each read request
   and the DeviceFS stream buffer are run-time settings (see main()).
   The read request size matters far more than the buffer size: USBDriver
   keeps only ONE bulk transfer in flight per stream, arms the next one
   only in response to a read call, and sizes each transfer to that read
   request (16 KB is ~7 ms of data at 2.4 MSPS). Any gap in this app's
   polling longer than the transfer left the dongle unserviced, and it
   dropped samples -- whatever the buffer size. A big request gives one
   big transfer in flight plus one queued behind it: that much time of
   tolerance for the desktop stalling this app. */
static long rate_hz_g = 2400000L;        /* dongle sample rate */
static unsigned char *rx_buf_g = 0;      /* RMA block the reads land in */
static int rx_buf_size_g = 0;
static int rx_req_bytes_g = 16384;       /* bytes asked for per read */
static int rx_recoveries_g = 0;
static int rx_errors_g = 0;              /* consecutive failed reads */
static unsigned int usb_last_data_cs_g = 0;
static unsigned int last_tick_cs_g = 0;
static int usb_t0_set_g = 0;             /* has any data arrived yet? */

/* How long (centiseconds) the main loop asks the Wimp to sleep between
   polls of this app (Wimp_PollIdle). With big USB transfers there is
   nothing to gain from polling faster than a fraction of one transfer's
   duration, and spinning at 70,000 polls a second (which the log showed)
   burned ~15% of the CPU in empty read calls. 0 = poll flat out, which
   the small-request settings need. */
static unsigned int poll_idle_cs_g = 0;

/* Run-time tunables, read once at start-up from RISC OS system variables
   so different settings can be tried without a rebuild (an Obey launcher
   just *Sets them first). The defaults are the profile that worked best
   on real hardware (240 kSPS audio mode, 192 KB reads, 500 ms lead):
     RTLSDRView$RateK      dongle sample rate, kSPS: 240 (default: FM
                           audio, ~10x less USB traffic and CPU; the
                           spectrum then spans 240 kHz) or 2400 (the whole
                           2.4 MHz, ~4.8 MB/s)
     RTLSDRView$XferKB     read request = USB transfer size, KB (default:
                           ~400 ms of data -- 192 KB at 240 kSPS, 128 KB at
                           2.4 MSPS)
     RTLSDRView$BufKB      DeviceFS stream buffer, KB (default: 4x XferKB
                           rounded up to a power of two)
     RTLSDRView$LeadMS     audio lead, ms (default 500 at 240 kSPS, 350
                           above that)
     RTLSDRView$FFTFrames  spectrum frames per display update (default 4)
     RTLSDRView$PaceMS     0 (default) = no pause after each tuner I2C
                           transfer once running; 10 = the old pause
     RTLSDRView$Log        write a per-second trace to this file on exit
   A bigger transfer rides out longer desktop stalls without losing
   samples; a bigger lead keeps the audio playing through them. Both add
   latency. */
#define SAFE_BUF_KB      128    /* the proven-good stream buffer size... */
#define SAFE_REQ_BYTES   16384  /* ...and read request: the fallback */
#define DEFAULT_LEAD_MS  350   /* above 480 kSPS (see main()) */
static int stream_buf_kb_g = 512;

static int env_int(const char *name, int def, int lo, int hi)
{
    const char *v;
    int n;

    v = getenv(name);
    if (v == NULL || *v == '\0') {
        return def;
    }
    n = atoi(v);
    if (n < lo || n > hi) {
        return def;
    }
    return n;
}
/* Set once in main(), right after Event_Initialise(). */
static unsigned int app_start_time_g = 0;
static double debug_db_min_g = 0.0;
static double debug_db_max_g = 0.0;
static double debug_db_dc_g = 0.0;
static int debug_max_offset_g = 0;
static double debug_raw_min_g = 0.0;
static double debug_raw_max_g = 0.0;


/* ---- in-place iterative radix-2 DIT FFT, fixed N=FFT_SIZE ---- */
static void fft256(double *re, double *im)
{
    unsigned int i, j, k;
    unsigned int len, half, step;
    unsigned int bit;
    double tr, ti, wr, wi, ur, ui;

    /* bit-reversal permutation */
    j = 0;
    for (i = 0; i < FFT_SIZE - 1; i++) {
        if (i < j) {
            tr = re[i]; re[i] = re[j]; re[j] = tr;
            ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
        bit = FFT_SIZE >> 1;
        while (j & bit) {
            j &= ~bit;
            bit >>= 1;
        }
        j |= bit;
    }

    /* iterative Cooley-Tukey, block length doubling each stage */
    for (len = 2; len <= FFT_SIZE; len <<= 1) {
        half = len >> 1;
        step = FFT_SIZE / len; /* stride into the full-size twiddle table */
        for (i = 0; i < FFT_SIZE; i += len) {
            for (k = 0; k < half; k++) {
                wr = twiddle_cos[k * step];
                wi = twiddle_sin[k * step];
                ur = re[i + k];
                ui = im[i + k];
                tr = re[i + k + half] * wr - im[i + k + half] * wi;
                ti = re[i + k + half] * wi + im[i + k + half] * wr;
                re[i + k] = ur + tr;
                im[i + k] = ui + ti;
                re[i + k + half] = ur - tr;
                im[i + k + half] = ui - ti;
            }
        }
    }
}

/* Converts the last-read I/Q frame to power-spectrum bar heights,
   FFT-shifted so the tuned frequency sits in the middle of the display
   (bin 0 of a complex FFT is 0Hz baseband; negative frequencies wrap to
   the upper half, so swapping halves gives the usual centred view).

   Split into two halves: accumulate_frame() FFTs one completed frame
   and folds its power into a running average (called every time
   Null_spectrum finishes a frame, which can happen many times a
   second -- much faster than the ~5Hz display rate); finalize_display()
   converts the current running average into bar heights (called only
   at the throttled display rate). Averaging happens in the POWER
   (linear) domain across many frames before ever taking a log, not by
   averaging dB values -- log of an average is not the same as an
   average of logs, and only power-domain averaging reduces a single
   raw FFT snapshot's high per-bin variance the way real spectrum
   analyzers' "trace averaging" does. A live run found the previous
   single-snapshot-per-display-update approach was detecting a real,
   rock-solid-consistent signal (proving genuine detection, not noise --
   see docs/PLAN.md) but measuring almost no contrast between it and
   the noise floor, which is consistent with exactly this kind of
   un-averaged periodogram variance, not a wiring bug. */

/* Smoothing factor for the exponential power average -- higher weights
   new frames more (faster response, less noise reduction), lower
   weights history more (slower response, more noise reduction).

   With real dB contrast finally showing (after fixing the read
   accumulation bug), a live run showed a dense, "uniformly noisy" bar
   pattern with no visible bump anywhere -- not the clean, localized
   ~40dB peak SDR# shows for this same station. A single un-averaged
   periodogram has high per-bin variance; 0.05's ~20-frame effective
   window is short enough that this variance may still dominate over
   the real, persistent signal. Dropping this by 10x (~200-frame
   window) leans much harder into what averaging is FOR: suppressing
   per-bin noise so a persistent real signal stands out. Frames
   complete fast enough now (many per second) that 200 frames is still
   only a couple of seconds of real time, not an unusably slow response. */
/* (The averaging factor is avg_alpha_g now, set in main() from the
   number of frames per update; 0.02 was tuned for 8 per update.) */
/* (Retuned from 0.005 when the FFT stopped being run on every frame:
   with real full-rate data the display takes only ~8 frames per 200 ms
   update (~40/s) so the DSP can have the CPU, and 0.005's 200-frame
   window would have meant a ~5 s lag after every retune or gain
   change. 0.02 is a ~50-frame window, ~1.2 s -- about what the old
   ~140 frames/s at 0.005 gave.) */

static double avg_power_g[NUM_BINS]; /* zero-initialized; power domain */

/* Retuning to a new frequency (see Click_spectrum's F-/F+ handling)
   without this would leave avg_power_g[] full of the OLD frequency's
   averaged spectrum, which the ~200-frame AVG_ALPHA=0.005 window would
   then blend with the new frequency's real content for a couple of
   seconds -- a confusing, misleading transition rather than a clean
   jump. Zeroing it isn't perfectly clean either (finalize_display()'s
   log10(0+1e-6) floor briefly shows as a very low, flat trace right
   after a retune until real power accumulates again), but that's an
   honest "no data yet" state, not a blend of two different stations. */
static void reset_averaging(void)
{
    int i;
    for (i = 0; i < NUM_BINS; i++) {
        avg_power_g[i] = 0.0;
    }
}


/* FM demodulation and audio decimation live in Dsp.c now (integer-only:
   this project's floating point is compiled as emulated FP, ~100x slower
   per operation, which made the old per-sample float atan2 discriminator
   here manage only ~1.5% of real time). This file only feeds it raw
   I/Q from Null_spectrum() and reads back its peak/RMS deviation
   statistics for the DEM readout. */

static void accumulate_frame(void)
{
    static double re[FFT_SIZE];
    static double im[FFT_SIZE];
    int i, src;
    long isum_re, isum_im;
    double mean_re, mean_im, power;

    /* DC removal: subtract this FRAME'S OWN measured mean, not just the
       fixed assumed centre (127.5) -- see docs/PLAN.md for why a real
       hardware DC bias needs removing at the source (it spectrally
       leaks into nearby bins too, not just the exact centre one), not
       just excluded from the display-scale calculation. Sums are taken
       in integers and converted once: floating point is emulated on
       this platform, so every avoidable FP operation per sample costs
       real time. */
    isum_re = 0;
    isum_im = 0;
    for (i = 0; i < FFT_SIZE; i++) {
        isum_re += (long)iq_frame_g[2 * i];
        isum_im += (long)iq_frame_g[2 * i + 1];
    }
    mean_re = (double)isum_re / (double)FFT_SIZE;
    mean_im = (double)isum_im / (double)FFT_SIZE;
    for (i = 0; i < FFT_SIZE; i++) {
        re[i] = (double)iq_frame_g[2 * i] - mean_re;
        im[i] = (double)iq_frame_g[2 * i + 1] - mean_im;
    }

    fft256(re, im);

    for (i = 0; i < NUM_BINS; i++) {
        src = (i + NUM_BINS / 2) % NUM_BINS;
        power = re[src] * re[src] + im[src] * im[src];
        avg_power_g[i] = avg_power_g[i] * (1.0 - avg_alpha_g) + power * avg_alpha_g;
    }
}

static void finalize_display(void)
{
    static double db[NUM_BINS];
    int i;
    int centre;
    int max_bin;
    int min_bin;
    double scaled;
    double db_min, db_max, range;

    for (i = 0; i < NUM_BINS; i++) {
        /* Epsilon just needs to avoid log10(0) -- see docs/PLAN.md for
           why this must stay small (1e-6) rather than the +1.0 it used
           to be, which was harmless only while DC dominated everything
           by a factor of ~10^9 and became a real dynamic-range-crushing
           bug once that was fixed. */
        db[i] = 10.0 * log10(avg_power_g[i] + 1e-6);
    }

    /* Auto-scale to this frame's own dynamic range (see the
       DC_EXCLUDE_BINS comment above for why the DC bin is skipped
       here). If every non-DC bin happens to read identically (a truly
       flat/silent frame), range collapses to 0 -- floor it to avoid a
       divide-by-zero, which just means that frame draws as flat. */
    centre = NUM_BINS / 2;
    db_min = db[(centre + DC_EXCLUDE_BINS + 1) % NUM_BINS];
    db_max = db_min;
    max_bin = (centre + DC_EXCLUDE_BINS + 1) % NUM_BINS;
    min_bin = max_bin;
    for (i = 0; i < NUM_BINS; i++) {
        if (i >= centre - DC_EXCLUDE_BINS && i <= centre + DC_EXCLUDE_BINS) {
            continue;
        }
        if (db[i] < db_min) {
            db_min = db[i];
            min_bin = i;
        }
        if (db[i] > db_max) {
            db_max = db[i];
            max_bin = i;
        }
    }
    /* Raw linear power (not dB) at the min/max bins -- min=max=48.84dB
       to 2 decimal places across 251 bins is too perfectly uniform to
       be "weak signal"; comparing two very large, similar magnitudes on
       a log scale can make a real linear difference look like nothing
       (log10(1e10) vs log10(1.1e10) differ by only 0.04). Checking the
       actual linear numbers directly, bypassing log compression
       entirely, before guessing at yet another averaging-code change. */
    debug_raw_min_g = avg_power_g[min_bin];
    debug_raw_max_g = avg_power_g[max_bin];
    range = db_max - db_min;
    if (range < 1.0) {
        range = 1.0;
    }

    debug_db_min_g = db_min;
    debug_db_max_g = db_max;
    debug_db_dc_g = db[centre];
    /* How many bins the frame's strongest (non-DC) point sits from the
       tuned frequency (dead centre after the FFT-shift). If this
       consistently lands somewhere within the ~21-bin span a 200kHz-
       wide FM signal should occupy (roughly +-10 of centre, at our
       ~9.4kHz/bin resolution), that's consistent with a real station
       peak; if it's scattered randomly across all 251 non-DC bins
       frame to frame, that points to noise-floor-driven behaviour with
       no real signal peak being captured. See docs/PLAN.md. */
    debug_max_offset_g = max_bin - centre;

    for (i = 0; i < NUM_BINS; i++) {
        scaled = (db[i] - db_min) / range * (double)BAR_MAX_HEIGHT;
        if (scaled > (double)BAR_MAX_HEIGHT) {
            scaled = (double)BAR_MAX_HEIGHT;
        }
        if (scaled < 0.0) {
            scaled = 0.0;
        }
        bin_height_g[i] = (int)scaled;
    }
}

static void report_warning(const char *msg);

static void cleanup_and_exit(void)
{
    trace_event(Time_Monotonic(), "exit");
    receiver_save();          /* quietly does nothing if Choices$Write is
                                  not set */
    audio_stream_close();     /* must precede audio_test_tone_close() --
                                  see Audio.h */
    audio_test_tone_close();
    if (stream_handle_g != 0) {
        os_find_close(stream_handle_g);
        stream_handle_g = 0;
    }
    if (trace_write() != 0) { /* last: a file write to a network share can
                                  take a while, and nothing is running now
                                  that needs servicing */
        report_warning("Could not write the trace log to the file named "
                        "by RTLSDRView$Log.");
    }
    if (rx_buf_g != 0) {
        os_rma_free(rx_buf_g);
        rx_buf_g = 0;
    }
    Event_CloseDown(); /* calls exit(); does not return */
}

static void report_and_die(const char *msg)
{
    os_error err;

    err.errnum = 1;
    strncpy(err.errmess, msg, sizeof(err.errmess) - 1);
    err.errmess[sizeof(err.errmess) - 1] = '\0';
    Wimp_ReportError(&err, 0, "RTLSDRView");
    exit(1);
}

/* Same, but doesn't exit -- used for the audio subsystem (brand new,
   experimental), which shouldn't take down the whole proven spectrum
   display if TimPlayer setup fails on a given machine. */
static void report_warning(const char *msg)
{
    os_error err;

    err.errnum = 1;
    strncpy(err.errmess, msg, sizeof(err.errmess) - 1);
    err.errmess[sizeof(err.errmess) - 1] = '\0';
    Wimp_ReportError(&err, 0, "RTLSDRView");
}

/* Opens the USB bulk stream with a DeviceFS buffer of buf_kb and switches
   it to non-blocking mode. Returns 0 on success (stream_handle_g set). */
static int open_stream(int buf_kb)
{
    char path[112];
    int timeout_ms;

    /* The driver's per-transfer timeout must comfortably outlast one
       transfer (req bytes at rate * 2 bytes/s): at least 2 s. */
    timeout_ms = (int)(((long)rx_req_bytes_g * 3000L) / (rate_hz_g * 2L));
    if (timeout_ms < 2000) {
        timeout_ms = 2000;
    }
    sprintf(path, "devices#endpoint%d;interface%d;bulk;usbtimeout%d;"
                  "nopad;size%d:%s",
            RTLSDR_BULK_ENDPOINT, RTLSDR_BULK_INTERFACE, timeout_ms,
            buf_kb * 1024, device_name_g);
    stream_handle_g = os_find_open(path);
    if (stream_handle_g == 0) {
        return -1;
    }
    if (os_args_set_nonblocking(stream_handle_g, 1) != 0) {
        os_find_close(stream_handle_g);
        stream_handle_g = 0;
        return -1;
    }
    stream_buf_kb_g = buf_kb;
    return 0;
}

static int stream_settings_are_safe(void)
{
    return rx_req_bytes_g == SAFE_REQ_BYTES && stream_buf_kb_g == SAFE_BUF_KB;
}

/* Watchdog for a USB stream the driver can't actually service. The
   transfer size is user-tunable, and each USB transfer gets its own
   physically contiguous DMA buffer, allocated when the transfer starts:
   a very large setting could open fine yet never deliver data if that
   allocation fails. If no data arrives a few seconds after start-up (or
   stops arriving later), reopen the stream with the settings proven to
   work: 16 KB reads and a 128 KB stream buffer. */
static unsigned int first_tick_cs_g = 0;
static int first_tick_set_g = 0;

static void fallback_stream(const char *why)
{
    char msg[80];

    if (stream_handle_g != 0) {
        os_find_close(stream_handle_g);
        stream_handle_g = 0;
    }
    rtlsdr_reset_buffer(device_name_g);
    rx_req_bytes_g = SAFE_REQ_BYTES;
    if (open_stream(SAFE_BUF_KB) != 0) {
        report_and_die("Could not reopen the bulk stream with the "
                        "fallback settings.");
    }
    rx_reset();
    rx_recoveries_g++;
    rx_errors_g = 0;
    usb_last_data_cs_g = Time_Monotonic();
    sprintf(msg, "fallback to safe USB settings: %.20s", why);
    trace_event(Time_Monotonic(), msg);
}

/* Each poll: read whatever the USB stream has buffered (in blocks of up
   to rx_req_bytes_g, non-blocking: see os_args_set_nonblocking), run every
   sample through the demodulator when audio/DEM is wanted (it must see
   the whole stream or the audio would have gaps), feed the audio ring,
   and take a few FFT frames for the display. A wall-clock budget bounds
   the work per tick so the desktop stays responsive.

   got==0 means nothing is buffered right now: yield to Wimp_Poll. Note
   the read call also matters to the USB driver: it is the read call that
   arms the next bulk transfer (see the comment above rx_req_bytes_g), so
   polling often and asking for a big block keeps the dongle serviced. */
#define MAX_READ_ATTEMPTS 64
/* Wall-clock budget per idle tick, in centiseconds (Time_Monotonic()
   resolution): stop processing once this much time has elapsed even if
   data remains, so the Wimp always gets control back promptly. */
#define TICK_BUDGET_CS 2

static BOOL Null_spectrum(event_pollblock *event, void *reference)
{
    int i;
    int got;
    int off;
    int n;
    int npcm;
    int dsp_active;
    int waiting, used;
    unsigned int tick_start_cs, t_prev, t_now;
    static short pcm_g[PCM_BUF_SAMPLES];

    UNUSED_ARG(event);
    UNUSED_ARG(reference);

    tick_start_cs = Time_Monotonic();
    trace_tick_begin(tick_start_cs);
    t_prev = tick_start_cs;

    /* Is the stream delivering? Not a single byte 3 s after start-up, or
       none for 3 s in the middle of a run while this app has been
       polling normally (so it isn't just the desktop having stalled us),
       means the driver could not service the configured transfer size:
       drop back to the settings known to work. */
    if (!usb_t0_set_g) {
        if (!first_tick_set_g) {
            first_tick_cs_g = tick_start_cs;
            first_tick_set_g = 1;
        } else if (tick_start_cs - first_tick_cs_g > 300 &&
                   !stream_settings_are_safe()) {
            fallback_stream("no data 3 s after start-up");
            report_warning("The configured USB transfer size delivered no "
                            "data, so the stream was reopened with 16 KB "
                            "reads and a 128 KB buffer. Try a smaller "
                            "RTLSDRView$XferKB / RTLSDRView$BufKB.");
            first_tick_cs_g = Time_Monotonic();
            last_tick_cs_g = first_tick_cs_g;
            trace_tick_end(first_tick_cs_g);
            return FALSE;
        }
    } else if (tick_start_cs - last_tick_cs_g < 50 &&
               tick_start_cs - usb_last_data_cs_g > 300 &&
               !stream_settings_are_safe()) {
        fallback_stream("no data for 3 s");
        last_tick_cs_g = Time_Monotonic();
        trace_tick_end(last_tick_cs_g);
        return FALSE;
    }

    dsp_active = (demod_enabled_g || stream_enabled_g);
    for (i = 0; i < MAX_READ_ATTEMPTS; i++) {
        got = os_gbpb_read4(stream_handle_g, rx_buf_g, rx_req_bytes_g);
        t_now = Time_Monotonic();
        trace_read(got, t_now - t_prev);
        t_prev = t_now;
        if (got < 0) {
            rx_errors_g++;
            if (rx_errors_g == 1) {
                char msg[TRACE_EVENT_TEXT];
                sprintf(msg, "read error: %.40s", os_last_error());
                trace_event(t_now, msg);
            }
            if (rx_errors_g > 300 && !stream_settings_are_safe()) {
                fallback_stream("repeated read errors");
            }
            break;
        }
        rx_errors_g = 0;
        if (got == 0) {
            break;
        }
        if (got & 1) {
            trace_event(t_now, "odd-length read (I/Q pairing lost)");
        }
        got &= ~1;   /* whole I/Q pairs only */
        if (got == 0) {
            break;
        }
        if (!usb_t0_set_g) {
            usb_t0_set_g = 1;
            trace_event(t_now, "first data");
        }
        usb_last_data_cs_g = t_now;

        /* How much data was waiting (this block plus whatever is still
           buffered behind it): a backlog means the app was stalled and
           the USB pipe kept capturing meanwhile. */
        waiting = got;
        used = usb_stream_used_bytes(device_name_g, stream_handle_g);
        if (used > 0) {
            waiting += used;
        }
        trace_backlog((unsigned long)waiting);

        for (off = 0; off < got; off += FIXED_CHUNK_SIZE) {
            n = got - off;
            if (n > FIXED_CHUNK_SIZE) {
                n = FIXED_CHUNK_SIZE;
            }
            if (dsp_active) {
                npcm = rx_process(rx_buf_g + off, n, pcm_g,
                                      PCM_BUF_SAMPLES);
                if (stream_enabled_g && npcm > 0) {
                    /* Tell the ring how much audio is ready right now
                       (this slice, the rest of the block, and anything
                       still buffered): if it has to re-sync after an
                       underrun, a backlog counts towards the lead. */
                    audio_stream_set_pending_ms(
                        (int)(((long)(waiting - off) * 1000L) /
                              (rate_hz_g * 2L)));
                    /* Return value (samples accepted) deliberately
                       ignored: a real-time stream, not a queue --
                       audio_stream_feed() drops what it can't safely
                       fit. */
                    audio_stream_feed(pcm_g, npcm);
                }
                t_now = Time_Monotonic();
                trace_dsp_cs(t_now - t_prev);
                t_prev = t_now;
            }

            chunk_seq_g++;
            if (n >= IQ_BYTES && fft_frames_g < fft_per_update_g &&
                (chunk_seq_g % (unsigned long)fft_stride_g) == 0) {
                memcpy(iq_frame_g, rx_buf_g + off, IQ_BYTES);
                accumulate_frame();
                fft_frames_g++;
                t_now = Time_Monotonic();
                trace_fft_cs(t_now - t_prev, 1);
                t_prev = t_now;
            }
        }

        if (t_prev - tick_start_cs >= TICK_BUDGET_CS) {
            break;
        }
    }

    if ((fft_frames_g > 0 || fft_per_update_g == 0) &&
        t_prev >= next_update_time_g) {
        if (dsp_active) {
            /* Once per display update rather than per tick: this does a
               square root in emulated floating point. */
            rx_stats rxs;
            if (rx_take_stats(&rxs)) {
                if (rxs.have_dev) {
                    demod_dev_rms_g = demod_dev_rms_g * (1.0 - DEMOD_ALPHA) +
                                       rxs.rms_hz * DEMOD_ALPHA;
                    demod_dev_peak_g = demod_dev_peak_g * (1.0 - DEMOD_ALPHA) +
                                        rxs.peak_hz * DEMOD_ALPHA;
                } else {
                    demod_dev_rms_g = 0.0;
                    demod_dev_peak_g = 0.0;
                }
                level_db10_g = rxs.level_db10;
                squelch_open_g = rxs.squelch_open;
            }
        }
        finalize_display();
        Window_ForceRedraw(spectrum_window_g, 0, -WORK_HEIGHT, WORK_WIDTH,
                           REDRAW_TOP);
        next_update_time_g = Time_Monotonic() + UPDATE_INTERVAL_CS;
        fft_frames_g = 0;
        t_now = Time_Monotonic();
        trace_disp_cs(t_now - t_prev);
        t_prev = t_now;
    }

    last_tick_cs_g = tick_start_cs;
    trace_tick_end(t_prev);
    return FALSE;
}

static BOOL Redraw_spectrum(event_pollblock *event, void *reference)
{
    window_redrawblock r;
    BOOL more;
    int ox, oy;
    int i;

    UNUSED_ARG(event);
    UNUSED_ARG(reference);

    r.window = spectrum_window_g;
    Wimp_RedrawWindow(&r, &more);

    while (more) {
        /* (rect, scroll) are the window's on-screen box and scroll
           offset -- constant for this whole redraw loop, unlike
           cliprect, which changes each iteration. Standard RISC OS
           work-area-to-screen conversion: screen = os_origin + work,
           where os_origin is derived once from rect/scroll. */
        ox = r.rect.min.x - r.scroll.x;
        oy = r.rect.max.y - r.scroll.y;

        Wimp_SetColour((int)colour_BLACK);

        /* Both a fixed 100x100 square AND a fixed 1-wide/WORK_HEIGHT-
           tall rectangle rendered correctly as standalone calls,
           proving drawing/colour/coordinates/thin-width/exact-boundary-
           height are all individually fine. The one thing neither
           isolation test covered: calling GFX_RectangleFill 256 times
           in a tight loop where ~255 of those calls are DEGENERATE
           (bin_height_g[i]==0, since min==max==47 gives near-zero
           height to almost every non-DC bin) -- a zero-height
           rectangle-fill may not be a well-behaved OS_Plot code path,
           and could be corrupting state for later calls in the same
           loop (which would explain why even the one real, non-zero
           bar -- the DC bin -- never showed). Skipping zero-height
           bars costs nothing visually (they're invisible either way)
           and tests this directly. */
        for (i = 0; i < NUM_BINS; i++) {
            if (bin_height_g[i] > 0) {
                GFX_RectangleFill(ox + BIN_X(i), oy - WORK_HEIGHT,
                                   BIN_X(i + 1) - BIN_X(i) - 1,
                                   bin_height_g[i]);
            }
        }

        /* The channel the demodulator is listening to: two red lines
           either side of the centre bin, bandwidth wide (the dongle is
           tuned to the centre). */
        {
            long bin_hz;
            int bw, half_bins;

            bin_hz = rate_hz_g / NUM_BINS;
            bw = (rcv.mode == RX_MODE_WFM) ? 150000 : rcv.bw_hz[rcv.mode];
            half_bins = (int)(((long)bw / 2L) / bin_hz);
            if (half_bins < 1) {
                half_bins = 1;
            }
            if (half_bins > NUM_BINS / 2 - 1) {
                half_bins = NUM_BINS / 2 - 1;
            }
            Wimp_SetColour((int)colour_RED);
            GFX_RectangleFill(ox + BIN_X(NUM_BINS / 2 - half_bins),
                               oy - WORK_HEIGHT, 1, BAR_MAX_HEIGHT);
            GFX_RectangleFill(ox + BIN_X(NUM_BINS / 2 + half_bins),
                               oy - WORK_HEIGHT, 1, BAR_MAX_HEIGHT);
            Wimp_SetColour((int)colour_BLACK);
        }

        /* Status text. VDU 5 switches text output to plot at the graphics
           cursor (GFX_Move'd position) instead of the text cursor; VDU 4
           reverts. Lines must stay short (~32 characters fit the window
           width). */
        {
            char line1[64];
            char bwtxt[12];

            if (rcv.mode == RX_MODE_WFM) {
                strcpy(bwtxt, "wide");
            } else {
                receiver_fmt_hz(bwtxt, (int)sizeof(bwtxt), rcv.bw_hz[rcv.mode]);
            }
            if (gain_mode_g == GAIN_MODE_AGC) {
                sprintf(line1, "%s %s  G:AGC  %ldk", receiver_mode_name(rcv.mode),
                        bwtxt, rate_hz_g / 1000L);
            } else {
                sprintf(line1, "%s %s  G:M%02d  %ldk", receiver_mode_name(rcv.mode),
                        bwtxt, gain_index_g, rate_hz_g / 1000L);
            }
            GFX_VDU(5);
            GFX_Move(ox + 4, oy - 20);
            GFX_Write0(line1);
            GFX_VDU(4);
        }

        /* Signal level meter: -100 dBFS at the left to 0 at the right. The
           bar is green while the audio passes and red while the squelch has
           it muted. */
        {
            int full_w, w;

            full_w = WORK_WIDTH - 16;
            w = ((level_db10_g + 1000) * full_w) / 1000;
            if (w < 0) {
                w = 0;
            }
            if (w > full_w) {
                w = full_w;
            }
            Wimp_SetColour((int)colour_GREY1);
            GFX_RectangleFill(ox + 8, oy - 232, full_w, 16);
            if (w > 0) {
                Wimp_SetColour(squelch_open_g ? (int)colour_GREEN
                                              : (int)colour_RED);
                GFX_RectangleFill(ox + 8, oy - 232, w, 16);
            }
            Wimp_SetColour((int)colour_BLACK);
        }

        /* The readout: r = complex samples/s actually read over the last
           few seconds, in thousands -- it should sit at the dongle's rate
           (240 or 2400); below that, samples are being lost. g / G = the
           longest time the rest of the desktop held the CPU between two
           polls of this app (recent window / ever), ms; u = audible audio
           underruns; pk = FM peak deviation. */
        if (demod_enabled_g) {
            char line2[64];
            double rate_ksps;
            unsigned int gap_ms;

            trace_recent(&rate_ksps, &gap_ms);
            if (rcv.mode == RX_MODE_AM) {
                sprintf(line2, "%.0f dB r=%.0fk g%u G%u u%d",
                        level_db10_g / 10.0, rate_ksps, gap_ms,
                        trace_gap_all_ms(), audio_stream_underruns());
            } else {
                sprintf(line2, "pk=%.0fk r=%.0fk g%u G%u u%d",
                        demod_dev_peak_g / 1000.0, rate_ksps, gap_ms,
                        trace_gap_all_ms(), audio_stream_underruns());
            }
            GFX_VDU(5);
            GFX_Move(ox + 4, oy - 252);
            GFX_Write0(line2);
            GFX_VDU(4);
        }

        Wimp_GetRectangle(&r, &more);
    }
    return TRUE;
}

static BOOL Close_spectrum(event_pollblock *event, void *reference)
{
    UNUSED_ARG(event);
    UNUSED_ARG(reference);
    cleanup_and_exit();
    return TRUE; /* unreached */
}

/* A toggle icon's "pressed in" look tracks whether it's currently
   active, matching the standard RISC OS toggle-button convention.
   Wimp_SetIconState's (value, mask) pair: only bits set in mask are
   touched, and within those, value's bits are the new state -- so
   mask=value=just the `selected` bit sets it, mask=selected/value=0
   clears it. Building both through the icon_flags union rather than a
   hand-computed hex bitmask keeps this correct if the struct layout
   ever changes. Shared by icon_agc_g (AGC on/off) and icon_demod_g
   (demod on/off). */
static void set_icon_selected(icon_handle icon, int selected)
{
    icon_flags mask, value;

    memset(&mask, 0, sizeof(mask));
    memset(&value, 0, sizeof(value));
    mask.data.selected = 1;
    value.data.selected = selected ? 1 : 0;

    Wimp_SetIconState(spectrum_window_g, icon, value.value, mask.value);
}

static void update_agc_icon(void)
{
    set_icon_selected(icon_agc_g, gain_mode_g == GAIN_MODE_AGC);
}

static void update_demod_icon(void)
{
    set_icon_selected(icon_demod_g, demod_enabled_g);
}

static void update_tone_icon(void)
{
    set_icon_selected(icon_tone_g, tone_playing_g);
}

static void update_stream_icon(void)
{
    set_icon_selected(icon_stream_g, stream_enabled_g);
}

/* Reads and discards whatever the DeviceFS stream has buffered right
   now (bounded, non-blocking): after a retune, everything queued was
   captured at the old frequency. */
static void discard_buffered_stream_data(void)
{
    int n;

    for (n = 0; n < 64; n++) {
        if (os_gbpb_read4(stream_handle_g, rx_buf_g, rx_req_bytes_g) <= 0) {
            break;
        }
    }
}

/* Retunes the dongle to new_freq (clamped to the tuning limits). Returns
   non-zero if the tuner locked, in which case the new frequency is
   committed everywhere (tuned_freq_hz_g and rcv.freq_hz); on a failed lock
   nothing changes, so the displayed frequency never disagrees with what
   the tuner is really doing. Also the callback the control panel uses. */
static int app_tune(unsigned long new_freq)
{
    unsigned int retune_t0;
    char evmsg[TRACE_EVENT_TEXT];
    int ok;

    new_freq = receiver_clamp_freq(new_freq);
    if (new_freq == tuned_freq_hz_g) {
        return 1;
    }
    ok = 0;
    rtlsdr_demod_write_reg(device_name_g, 1, 0x01, 0x18, 1); /* I2C repeater on */
    retune_t0 = Time_Monotonic();
    if (r82xx_set_freq(&tuner_g, new_freq) == 0 && tuner_g.has_lock) {
        tuned_freq_hz_g = new_freq;
        rcv.freq_hz = new_freq;
        /* Throw away the stale, pre-retune samples still sitting in the
           DeviceFS buffer, and restart the demodulator so its filters
           don't blend the two frequencies.

           This deliberately does NOT call rtlsdr_reset_buffer() (which
           main() uses once, before the stream is opened): that now
           genuinely stalls and resets the dongle's endpoint (10 02 -- see
           Driver.c's rtlsdr_write_reg() for the byte-order bug that used
           to make it a no-op), and doing that while a bulk transfer is in
           flight would leave the host pipe halted under the driver. */
        discard_buffered_stream_data();
        rx_reset();
        reset_averaging();
        /* The retune kept the app busy and the buffered audio was thrown
           away: rebuild the audio lead (with a moment of silence) rather
           than carry on with a thin one. */
        audio_stream_relead();
        ok = 1;
    }
    rtlsdr_demod_write_reg(device_name_g, 1, 0x01, 0x10, 1); /* I2C repeater off */
    sprintf(evmsg, "retune to %lu Hz took %u cs%s", new_freq,
            Time_Monotonic() - retune_t0, ok ? "" : " FAILED");
    trace_event(Time_Monotonic(), evmsg);
    return ok;
}

/* All three gain buttons need the I2C repeater enabled around the
   register write, same as main()'s own init/tune bracket -- the
   repeater is disabled the rest of the time, and this handler runs
   long after that initial bracket closed. */
static BOOL Click_spectrum(event_pollblock *event, void *reference)
{
    icon_handle icon;
    int is_gain_icon;

    int adjust;

    UNUSED_ARG(reference);

    icon = event->data.mouse.icon;
    adjust = event->data.mouse.button.data.adjust ? 1 : 0;

    /* The control panel (Ui.c) takes Select and Adjust. */
    if ((event->data.mouse.button.data.select || adjust) &&
        ui_click(icon, event->data.mouse.button.data.select ? 1 : 0,
                 adjust)) {
        /* mode, bandwidth and so on appear in the status line at the top */
        Window_ForceRedraw(spectrum_window_g, 0, -40, WORK_WIDTH, 0);
        return TRUE;
    }

    /* F- / F+: tune by the current mode's step (Adjust: ten steps). */
    if ((icon == icon_freqdown_g || icon == icon_frequp_g) &&
        (event->data.mouse.button.data.select || adjust)) {
        unsigned long step, hz;

        step = (unsigned long)receiver_step_hz() * (adjust ? 10UL : 1UL);
        if (icon == icon_frequp_g) {
            hz = tuned_freq_hz_g + step;
        } else {
            hz = (tuned_freq_hz_g > step) ? tuned_freq_hz_g - step : 0UL;
        }
        app_tune(hz);
        ui_refresh();
        return TRUE;
    }

    if (!event->data.mouse.button.data.select) {
        return FALSE; /* only Select clicks drive the older buttons */
    }

    if (icon < 0) {
        /* A click on the spectrum itself (not a button): a marker in the
           trace, to bracket something being tried (a stall test, say). */
        trace_event(Time_Monotonic(), "MARK");
        return TRUE;
    }

    if (icon == icon_demod_g) {
        /* Pure software toggle -- no I2C repeater bracket needed,
           unlike every gain/freq control below. */
        demod_enabled_g = !demod_enabled_g;
        if (demod_enabled_g) {
            rx_reset();
        }
        update_demod_icon();
        trace_event(Time_Monotonic(), demod_enabled_g ? "DEM on" : "DEM off");
        return TRUE;
    }

    if (icon == icon_tone_g) {
        /* Also pure software (TimPlayer, not the tuner) -- no I2C
           repeater bracket. Does nothing if audio_test_tone_init()
           never succeeded (audio_ok_g), rather than toggling a button
           that plays nothing. */
        if (audio_ok_g) {
            tone_playing_g = !tone_playing_g;
            audio_test_tone_play(tone_playing_g);
            update_tone_icon();
        }
        return TRUE;
    }

    if (icon == icon_stream_g) {
        /* Same pattern as TONE -- pure software, does nothing if
           audio_stream_init() never succeeded. Turning STREAM on also
           turns DEM on (streaming needs the same per-sample
           computation the DEM readout uses -- no reason to make the
           user click both), but turning STREAM off leaves DEM as the
           user had it, since they might still want the readout. */
        if (stream_ok_g) {
            stream_enabled_g = !stream_enabled_g;
            if (stream_enabled_g) {
                demod_enabled_g = 1;
                update_demod_icon();
                rx_reset();
            }
            audio_stream_play(stream_enabled_g);
            update_stream_icon();
            trace_event(Time_Monotonic(),
                        stream_enabled_g ? "STREAM on" : "STREAM off");
        }
        return TRUE;
    }

    is_gain_icon = (icon == icon_agc_g || icon == icon_gaindown_g ||
                     icon == icon_gainup_g);
    if (!is_gain_icon) {
        return FALSE;
    }

    rtlsdr_demod_write_reg(device_name_g, 1, 0x01, 0x18, 1); /* enable I2C repeater */

    if (is_gain_icon) {
        if (icon == icon_agc_g) {
            if (gain_mode_g == GAIN_MODE_AGC) {
                gain_mode_g = GAIN_MODE_MANUAL;
                r82xx_set_gain_manual(&tuner_g, gain_index_g);
            } else {
                gain_mode_g = GAIN_MODE_AGC;
                r82xx_set_gain_agc(&tuner_g);
            }
        } else {
            /* +/- while in AGC mode switches to manual first, at the
               default step, rather than doing nothing -- clicking a
               gain button clearly means "I want manual control now". */
            if (gain_mode_g == GAIN_MODE_AGC) {
                gain_mode_g = GAIN_MODE_MANUAL;
                gain_index_g = GAIN_INDEX_DEFAULT;
            } else if (icon == icon_gaindown_g && gain_index_g > 0) {
                gain_index_g--;
            } else if (icon == icon_gainup_g &&
                       gain_index_g < GAIN_INDEX_MAX) {
                gain_index_g++;
            }
            r82xx_set_gain_manual(&tuner_g, gain_index_g);
        }
        update_agc_icon();
    }

    rtlsdr_demod_write_reg(device_name_g, 1, 0x01, 0x10, 1); /* disable I2C repeater */

    return TRUE;
}

/* Key presses with the caret in our window: the frequency field (Return
   tunes, Up/Down step, Escape reverts). Anything the panel doesn't take
   goes to the Wimp, which does the editing in a writable icon. */
static BOOL Key_spectrum(event_pollblock *event, void *reference)
{
    int code;

    UNUSED_ARG(reference);

    code = (int)event->data.key.code;
    if (!ui_key(code, event->data.key.caret.icon)) {
        Wimp_ProcessKey(code);
    }
    return TRUE;
}

static BOOL Quit_message(event_pollblock *event, void *reference)
{
    if (event->data.message.header.action == message_QUIT) {
        cleanup_and_exit();
    }
    return FALSE;
}

/* Mirrors window_block field-for-field (same 88-byte layout Wimp_CreateWindow
   expects -- verified offset by offset against the real block layout: 0
   screenrect, 16 scroll, 24 behind, 28 flags, 32 colours, 40 workarearect,
   56 titleflags, 60 workflags, 64 spritearea, 68 minsize, 72 title, 84
   numicons), but WITHOUT window_block's own `colours` field
   (wimp_colourflags), which mixes `unsigned char` fields with `unsigned
   int` bitfields in the same struct -- a combination where Norcroft can
   pad the union out past its intended 8 bytes to fit the bitfield's
   alignment, silently shifting every field after `colours` relative to
   what the real Wimp SWI expects. Confirmed the hard way: window_block
   crashed Wimp_CreateWindow with a data abort deep in the Wimp's own
   processing of the block, and rewriting only the VALUES (still via
   window_block's .colours field) did not fix it -- same crash, same
   address, because the union's SIZE was the actual problem, not which
   member was used to write into it. See docs/PLAN.md. Every other field
   here reuses window_flags/icon_flags, which are safe: homogeneous
   unsigned-int bitfields matching their own .value alias exactly, with
   no mixed-type padding risk. */
typedef struct {
    wimp_box      screenrect;
    wimp_point    scroll;
    int           behind;
    window_flags  flags;
    unsigned char title_fore, title_back, work_fore, work_back;
    unsigned char scroll_outer, scroll_inner, title_focus, colour_extra;
    wimp_box      workarearect;
    icon_flags    titleflags;
    icon_flags    workflags;
    void          *spritearea;
    unsigned short minsize_x, minsize_y;
    char          title_text[12];
    unsigned int  numicons;
} raw_window_block;

static window_handle create_spectrum_window(void)
{
    raw_window_block rwb;
    window_handle win;
    os_error *err;

    memset(&rwb, 0, sizeof(rwb));

    rwb.screenrect.min.x = 200;
    rwb.screenrect.min.y = 200;
    rwb.screenrect.max.x = 200 + WORK_WIDTH;
    rwb.screenrect.max.y = 200 + WORK_HEIGHT;
    rwb.scroll.x = 0;
    rwb.scroll.y = 0;
    rwb.behind = -1;

    rwb.flags.data.moveable = 1;
    rwb.flags.data.backicon = 1;
    rwb.flags.data.titlebar = 1;
    rwb.flags.data.closeicon = 1;
    rwb.flags.data.newflags = 1;

    rwb.title_fore = colour_BLACK;
    rwb.title_back = colour_LIGHT_BLUE;
    rwb.work_fore = colour_BLACK;
    rwb.work_back = colour_WHITE;
    rwb.scroll_outer = colour_GREY3;
    rwb.scroll_inner = colour_WHITE;
    rwb.title_focus = colour_LIGHT_BLUE;
    rwb.colour_extra = 0;

    rwb.workarearect.min.x = 0;
    rwb.workarearect.min.y = -WORK_HEIGHT;
    rwb.workarearect.max.x = WORK_WIDTH;
    rwb.workarearect.max.y = 0;

    rwb.titleflags.data.text = 1;

    /* Work-area button type 3 (click): without it the Wimp never reports
       clicks on the bare graph (icon -1) -- the trace markers, see
       Click_spectrum. */
    rwb.workflags.data.buttontype = 3;

    rwb.spritearea = NULL;
    rwb.minsize_x = 0;
    rwb.minsize_y = 0;

    strcpy(rwb.title_text, "RTLSDRView");
    rwb.numicons = 0;

    err = Wimp_CreateWindow((window_block *)&rwb, &win);
    if (err != NULL) {
        report_and_die(err->errmess);
    }
    return win;
}

int main(void)
{
    int n;
    int rc;
    int xfer_kb, buf_kb, lead_ms, burst_ms, rate_k, pace_ms;
    char cfg[240];

    Event_Initialise("RTLSDRView");
    app_start_time_g = Time_Monotonic();
    os_set_quiet(1);    /* no stderr in a Wimp task */
    r82xx_set_trace(0); /* ...and no tuner narration on the VDU screen
                           behind the desktop */

    /* What was tuned, and how, last time (defaults if there is nothing
       saved). The frequency is applied by the tuner set-up below; mode,
       bandwidth, squelch and volume once the demodulator exists. */
    receiver_load();
    tuned_freq_hz_g = rcv.freq_hz;

    /* Run-time settings (see the comment above SAFE_BUF_KB). */
    rate_k = env_int("RTLSDRView$RateK", 240, 240, 2880);
    if ((rate_k % 240) != 0 || (rate_k > 300 && rate_k <= 900)) {
        rate_k = 240;    /* not a rate the demodulator/dongle can do */
    }
    rate_hz_g = 1000L * (long)rate_k;

    n = find_device();
    if (n < 0) {
        report_and_die("No RTL-SDR device found. Run !RTLSDR first to "
                        "confirm the dongle is detected.");
    }
    sprintf(device_name_g, "usb%d", n);

    rtlsdr_init_baseband(device_name_g);

    if (rtlsdr_probe_tuner(device_name_g) != 2) {
        report_and_die("R828D tuner not found -- see !RTLSDR's "
                        "diagnostic output for details.");
    }

    rtlsdr_demod_write_reg(device_name_g, 1, 0x01, 0x18, 1); /* enable I2C repeater */
    rc = rtlsdr_tuner_postinit(device_name_g);
    if (rc == 0) {
        rc = r82xx_init(&tuner_g, device_name_g);
    }
    if (rc == 0) {
        rc = r82xx_set_freq(&tuner_g, tuned_freq_hz_g); /* FREQ_DEFAULT_HZ:
                                                             97.4MHz, local
                                                             FM broadcast */
    }
    /* Manual max gain was tested unconditionally here earlier this
       session and judged first "no measurable difference", then (after
       byte-level diagnostics existed) a plausible ADC-overload cause,
       then ruled back out again when removing it made no difference to
       the deeper read-accumulation bug that turned out to be the real
       issue (see docs/PLAN.md). Gain is now a live, user-driven choice
       instead of a fixed startup decision either way -- AGC is only
       the STARTING state (explicitly set here so software state
       (gain_mode_g) is guaranteed to match hardware register state,
       rather than relying on the init array's own default), and the
       gain control icons (create_gain_icons, Click_spectrum) let the
       user switch to manual and step through it live. */
    if (rc == 0) {
        rc = r82xx_set_gain_agc(&tuner_g);
    }
    rtlsdr_demod_write_reg(device_name_g, 1, 0x01, 0x10, 1); /* disable I2C repeater */
    if (rc != 0 || !tuner_g.has_lock) {
        report_and_die("Tuner init/tune to 97.4MHz failed -- see "
                        "!RTLSDR's diagnostic output for details.");
    }

    if (rtlsdr_set_sample_rate(device_name_g, (unsigned long)rate_hz_g) != 0) {
        report_and_die("rtlsdr_set_sample_rate() failed.");
    }

    /* Port of upstream's r820t_set_bw(), which this project's
       rtlsdr_set_sample_rate() never called (a real, previously-missed
       gap -- see docs/PLAN.md): compute the tuner's actual analog IF
       for this capture bandwidth, write its filter registers, resync
       the demod's digital IF to match, then re-tune so
       r82xx_set_freq() picks up the corrected t->int_freq. Without
       this, the demod's digital downconversion and the tuner's real
       analog IF disagreed by ~1.75MHz for this project's 2.4MHz
       capture -- comparable to the capture's own 1.2MHz Nyquist limit,
       enough to alias real signal content into what looks exactly
       like unstructured noise. This is the leading suspect for the
       long-unresolved flat-spectrum/demod-saturation mystery. */
    {
        int new_if;

        rtlsdr_demod_write_reg(device_name_g, 1, 0x01, 0x18, 1); /* enable I2C repeater */
        new_if = r82xx_set_bandwidth(&tuner_g, (int)rate_hz_g);
        if (new_if >= 0) {
            rtlsdr_set_if_freq(device_name_g, (unsigned long)new_if);
            r82xx_set_freq(&tuner_g, tuned_freq_hz_g);
        }
        rtlsdr_demod_write_reg(device_name_g, 1, 0x01, 0x10, 1); /* disable I2C repeater */
        if (new_if < 0 || !tuner_g.has_lock) {
            report_and_die("Bandwidth/IF resync failed -- see "
                            "docs/PLAN.md.");
        }
    }

    /* Tuner bring-up is done (it stays paced: that is where the hangs
       were). Pacing after every I2C transfer is what made a retune take
       200-600 ms; with it off (the default now) a retune takes 40-60 ms
       -- measured over four retunes on the Pi, no hangs, PLL locked each
       time. RTLSDRView$PaceMS 10 restores the pause for runtime retunes
       and gain changes. */
    pace_ms = env_int("RTLSDRView$PaceMS", 0, 0, 100);
    r82xx_set_pace_ms(pace_ms);

    rtlsdr_reset_buffer(device_name_g);

    /* nopad: a REAL, DOCUMENTED USBDriver stream-open flag (real
       fetched source + docs, gitlab.riscosopen.org/RiscOS/Sources/
       HWSupport/USB/USBDriver -- not guessed), found while
       investigating why no chunk size above ~16-31 bytes could stay
       correct. USBDriver's own read_cb() zero-pads the REMAINDER of a
       request whenever a USB transfer completes short of the full
       requested size (its own comment literally says "fill up the
       rest of the request with garbage!") -- a SECOND, separate
       padding mechanism from the DeviceFS-level short-read-count bug
       fixed via the OS_Args non-blocking IOCtl earlier this session.
       Zero-padding still differs from the 0xAA sentinel used to
       "prove" reads were honest, so that fix's own validation never
       caught it. `nopad` disables this at the source -- see
       docs/PLAN.md.

       UPDATE (2026-09-29): the throughput ceiling that the `short` and
       `size131072` experiments were chasing turned out to have a single
       root cause elsewhere -- Driver.c's rtlsdr_write_reg() packed
       2-byte values in the wrong byte order, so USB_EPA_MAXPKT was set
       to 2 instead of 512 (see the comment there). With that fixed,
       reads return full chunks at ~98% of the true 2.4 MSPS rate.
       `short` never gave a real benefit (it is documented as an OUT-
       transfer flag; milestone 10 measured 3.1 MB/s with it vs. 3.5
       without) and has been dropped. `size131072` (a 128 KB DeviceFS
       stream buffer instead of the small default) does help -- ~4.7
       MB/s vs. ~3.5 MB/s in the same investigation -- so it stays. */
    /* Read request size: ~400 ms of data unless told otherwise (see the
       comment above rx_req_bytes_g for why this, not the buffer, is what
       sets the USB stall tolerance). Whole 1 KB steps keep it a multiple
       of the bulk endpoint's 512-byte packets. At 2.4 MSPS that would be
       nearly 2 MB: capped at 128 KB there, the largest size whose DMA
       allocation is known to work at that rate. Never longer than ~1.5 s
       of data, or a transfer would outlast the driver's 2 s timeout. */
    xfer_kb = env_int("RTLSDRView$XferKB", 0, 1, 1024);
    if (xfer_kb == 0) {
        xfer_kb = (int)((rate_hz_g * 4L / 5L) / 1024L);
        xfer_kb = ((xfer_kb + 8) / 16) * 16;
        if (xfer_kb > ((rate_hz_g > 1000000L) ? 128 : 192)) {
            xfer_kb = (rate_hz_g > 1000000L) ? 128 : 192;
        }
        if (xfer_kb < 16) {
            xfer_kb = 16;
        }
    }
    if ((long)xfer_kb * 1024L > rate_hz_g * 3L) {
        xfer_kb = (int)((rate_hz_g * 3L) / 1024L);
    }
    /* The stream buffer has to hold the transfer in flight, the one
       just completed but not yet read, and some slack. */
    buf_kb = env_int("RTLSDRView$BufKB", 0, 32, 8192);
    if (buf_kb == 0) {
        buf_kb = 128;
        while (buf_kb < 4 * xfer_kb) {
            buf_kb *= 2;
        }
    }
    if (buf_kb < 2 * xfer_kb) {
        buf_kb = 2 * xfer_kb;
    }

    rx_req_bytes_g = xfer_kb * 1024;
    rx_buf_size_g = rx_req_bytes_g;
    rx_buf_g = os_rma_claim(rx_buf_size_g);
    if (rx_buf_g == NULL) {
        /* No room in the RMA for the big read buffer: use the small,
           proven-good settings instead. */
        xfer_kb = SAFE_REQ_BYTES / 1024;
        buf_kb = SAFE_BUF_KB;
        rx_req_bytes_g = SAFE_REQ_BYTES;
        rx_buf_size_g = SAFE_REQ_BYTES;
        rx_buf_g = os_rma_claim(rx_buf_size_g);
        if (rx_buf_g == NULL) {
            report_and_die("Out of memory for the USB read buffer.");
        }
    }

    if (open_stream(buf_kb) != 0 && buf_kb != SAFE_BUF_KB) {
        /* The configured buffer couldn't be created (too big for the
           available memory?) -- fall back to the size proven to work
           rather than failing to start. */
        rx_req_bytes_g = SAFE_REQ_BYTES;
        xfer_kb = SAFE_REQ_BYTES / 1024;
        buf_kb = SAFE_BUF_KB;
        open_stream(SAFE_BUF_KB);
    }
    if (stream_handle_g == 0) {
        report_and_die("Could not open the bulk endpoint stream (or "
                        "enable non-blocking mode on it -- see "
                        "docs/PLAN.md milestone 5).");
    }

    /* Spectrum averaging: about 1.25 s of smoothing whatever the number
       of frames per update (0.02 per frame at the original 8). */
    fft_per_update_g = env_int("RTLSDRView$FFTFrames", 4, 0, 16);
    avg_alpha_g = (fft_per_update_g > 0)
        ? 0.8 / (double)(fft_per_update_g * 5) : 0.04;
    fft_stride_g = (rate_hz_g >= 960000L) ? 6 : 1;

    spectrum_window_g = create_spectrum_window();
    ui_create(spectrum_window_g, app_tune);
    icon_stream_g = ui_icon(UI_STREAM);
    icon_tone_g = ui_icon(UI_TONE);
    icon_demod_g = ui_icon(UI_DEM);
    icon_agc_g = ui_icon(UI_AGC);
    icon_gaindown_g = ui_icon(UI_GAIN_DN);
    icon_gainup_g = ui_icon(UI_GAIN_UP);
    icon_freqdown_g = ui_icon(UI_FDN);
    icon_frequp_g = ui_icon(UI_FUP);
    update_agc_icon();    /* reflect gain_mode_g's initial AGC state */
    update_demod_icon();  /* reflect demod_enabled_g's initial OFF state */
    update_tone_icon();   /* reflect tone_playing_g's initial OFF state */
    update_stream_icon(); /* reflect stream_enabled_g's initial OFF state */

    /* Non-fatal: a brand new, experimental subsystem (see Audio.h)
       shouldn't take down the whole proven spectrum display if
       TimPlayer isn't present/loadable on this machine. The TONE
       button simply does nothing if this fails (see Click_spectrum). */
    audio_ok_g = (audio_test_tone_init() == 0);
    if (!audio_ok_g) {
        report_warning("Audio test tone setup failed (TimPlayer module "
                        "not available?) -- the TONE button will do "
                        "nothing. Everything else is unaffected.");
    }

    /* Requires audio_ok_g (audio_stream_init() reuses the FX/song
       handles audio_test_tone_init() claims -- see Audio.h). Same
       non-fatal treatment as the test tone. */
    if (audio_ok_g) {
        stream_ok_g = (audio_stream_init() == 0);
        if (!stream_ok_g) {
            report_warning("Audio streaming setup failed -- the STREAM "
                            "button will do nothing. Everything else "
                            "(including TONE) is unaffected.");
        }
    }

    /* The demodulator runs whether or not audio output is available
       (the DEM readout uses it too), so configure it either way: the
       mixer's real rate if audio came up, else a nominal 48 kHz. */
    lead_ms = env_int("RTLSDRView$LeadMS",
                      (rate_hz_g <= 480000L) ? 500 : DEFAULT_LEAD_MS,
                      100, 1200);
    burst_ms = (int)(((long)rx_req_bytes_g * 1000L) / (rate_hz_g * 2L));
    if (stream_ok_g) {
        audio_stream_set_lead_ms(lead_ms);
        audio_stream_set_burst_ms(burst_ms);
    }
    rx_init((stream_ok_g && audio_stream_rate() > 0)
                    ? audio_stream_rate() : 48000, rate_hz_g);
    receiver_apply_mode();

    sprintf(cfg, "rate=%ld xfer_kb=%d buf_kb=%d burst_ms=%d lead_ms=%d "
                 "fft_frames=%d fft_stride=%d budget_cs=%d audio_rate=%d",
            rate_hz_g, xfer_kb, stream_buf_kb_g, burst_ms, lead_ms,
            fft_per_update_g, fft_stride_g, TICK_BUDGET_CS,
            stream_ok_g ? audio_stream_rate() : 0);
    /* Sleep between polls (up to 20 ms) when a transfer lasts long enough
       that it makes no difference: the pipe needs at least one read call
       per transfer, so keep the sleep under half a transfer. */
    poll_idle_cs_g = (unsigned int)(burst_ms / 20);
    if (poll_idle_cs_g > 2) {
        poll_idle_cs_g = 2;
    }
    sprintf(cfg + strlen(cfg), " poll_idle_cs=%u pace_ms=%d", poll_idle_cs_g,
            pace_ms);
    trace_init(getenv("RTLSDRView$Log"), cfg);
    trace_set_idle_cs(poll_idle_cs_g);
    trace_event(Time_Monotonic(), "started");

    Window_Show(spectrum_window_g, open_CENTERED);

    Event_Claim(event_REDRAW, spectrum_window_g, event_ANY, Redraw_spectrum, NULL);
    Event_Claim(event_OPEN, spectrum_window_g, event_ANY, Handler_OpenWindow, NULL);
    Event_Claim(event_CLOSE, spectrum_window_g, event_ANY, Close_spectrum, NULL);
    Event_Claim(event_CLICK, spectrum_window_g, event_ANY, Click_spectrum, NULL);
    Event_Claim(event_KEY, spectrum_window_g, event_ANY, Key_spectrum, NULL);
    Event_Claim(event_USERMESSAGE, event_ANY, event_ANY, Quit_message, NULL);
    Event_Claim(event_NULL, event_ANY, event_ANY, Null_spectrum, NULL);

    next_update_time_g = Time_Monotonic();

    while (TRUE) {
        Event_PollIdle(poll_idle_cs_g);
    }

    return 0; /* unreached */
}
