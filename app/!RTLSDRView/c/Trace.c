/* Trace.c -- see h/Trace.h.

   C89 only (Norcroft): declarations at the top of each block, no //
   comments. */

#include <stdio.h>
#include <string.h>
#include "Trace.h"
#include "Audio.h"

#define TRACE_MAX_SEC     900     /* 15 minutes of one-second buckets */
#define TRACE_MAX_EVENTS  200
#define TRACE_EVENT_LEN   TRACE_EVENT_TEXT
#define TRACE_RECENT_SEC  4

typedef struct {
    unsigned long  bytes;         /* bytes read off the USB stream */
    unsigned long  maxread;       /* largest single read, bytes */
    unsigned long  backlog;       /* most bytes found waiting unread */
    unsigned short reads;         /* read calls that returned data */
    unsigned short empty;         /* read calls that returned nothing */
    unsigned short ticks;         /* polls of the null-event handler */
    unsigned short tick_cs;       /* time inside the handler */
    unsigned short usb_cs;        /* ...of which in os_gbpb_read4() */
    unsigned short dsp_cs;        /* ...demodulator + audio feed */
    unsigned short fft_cs;        /* ...FFT frames */
    unsigned short disp_cs;       /* ...display update */
    unsigned short fft_frames;
    unsigned short fgap_max_cs;   /* longest time the rest of the desktop
                                     held the CPU between two polls */
    short          ahead_min_ms;  /* audio lead range (see Audio.h) */
    short          ahead_max_ms;
    short          dropped_ms;
    short          feed_gap_ms;
    unsigned char  underruns;
    unsigned char  audio_valid;
} trace_bucket;

typedef struct {
    unsigned int t_cs;            /* since the first tick */
    char text[TRACE_EVENT_LEN];
} trace_event_rec;

#define HIST_N 8
static const char *const hist_label[HIST_N] = {
    "<10ms", "10-20ms", "20-50ms", "50-100ms", "100-250ms", "250-500ms",
    "0.5-1s", ">1s"
};

static char log_path_g[200] = "";
static char config_g[240] = "";

static trace_bucket buckets_g[TRACE_MAX_SEC];
static int nbuckets_g = 0;
static trace_bucket cur_g;
static int cur_sec_g = 0;
static int started_g = 0;
static unsigned int t0_cs_g = 0;

static trace_event_rec events_g[TRACE_MAX_EVENTS];
static int nevents_g = 0;

static unsigned int idle_cs_g = 0;
static unsigned int last_begin_g = 0;
static int last_begin_set_g = 0;
static unsigned int last_busy_g = 0;
static unsigned int tick_begin_g = 0;

static unsigned long hist_n_g[HIST_N];
static unsigned long hist_cs_g[HIST_N];
static unsigned int gap_all_cs_g = 0;
static double total_bytes_g = 0.0;
static unsigned long total_underruns_g = 0;

#define ADD16(field, v) \
    do { \
        unsigned long t_ = (unsigned long)(field) + (unsigned long)(v); \
        (field) = (t_ > 65535UL) ? (unsigned short)65535U \
                                 : (unsigned short)t_; \
    } while (0)

static int hist_class(unsigned int cs)
{
    if (cs < 1) return 0;
    if (cs < 2) return 1;
    if (cs < 5) return 2;
    if (cs < 10) return 3;
    if (cs < 25) return 4;
    if (cs < 50) return 5;
    if (cs < 100) return 6;
    return 7;
}

void trace_init(const char *log_path, const char *config_text)
{
    log_path_g[0] = '\0';
    if (log_path != NULL) {
        strncpy(log_path_g, log_path, sizeof(log_path_g) - 1);
        log_path_g[sizeof(log_path_g) - 1] = '\0';
    }
    config_g[0] = '\0';
    if (config_text != NULL) {
        strncpy(config_g, config_text, sizeof(config_g) - 1);
        config_g[sizeof(config_g) - 1] = '\0';
    }
    memset(&cur_g, 0, sizeof(cur_g));
    nbuckets_g = 0;
    nevents_g = 0;
    started_g = 0;
}

/* Closes the current one-second bucket (folding in the audio ring's
   statistics) and stores it. */
static void push_bucket(void)
{
    int amin, amax, dropped, under, feedgap;

    if (audio_stream_take_interval(&amin, &amax, &dropped, &under,
                                   &feedgap)) {
        cur_g.ahead_min_ms = (short)amin;
        cur_g.ahead_max_ms = (short)amax;
        cur_g.dropped_ms = (short)dropped;
        cur_g.audio_valid = 1;
    }
    cur_g.underruns = (unsigned char)((under > 255) ? 255 : under);
    cur_g.feed_gap_ms = (short)((feedgap > 32000) ? 32000 : feedgap);
    total_underruns_g += (unsigned long)under;
    total_bytes_g += (double)cur_g.bytes;
    if (nbuckets_g < TRACE_MAX_SEC) {
        buckets_g[nbuckets_g] = cur_g;
        nbuckets_g++;
    }
    memset(&cur_g, 0, sizeof(cur_g));
    cur_sec_g++;
}

static void roll(unsigned int now_cs)
{
    unsigned int sec;

    if (!started_g) {
        started_g = 1;
        t0_cs_g = now_cs;
        cur_sec_g = 0;
        memset(&cur_g, 0, sizeof(cur_g));
    }
    sec = (now_cs - t0_cs_g) / 100;
    while ((unsigned int)cur_sec_g < sec) {
        push_bucket();
    }
}

void trace_tick_begin(unsigned int now_cs)
{
    roll(now_cs);
    if (last_begin_set_g) {
        unsigned int gap_cs, fg;
        int c;

        gap_cs = now_cs - last_begin_g;
        fg = (gap_cs > last_busy_g + idle_cs_g)
                 ? (gap_cs - last_busy_g - idle_cs_g) : 0;
        /* The clock ticks every 10 ms and polls can be microseconds
           apart, so two polls either side of a tick boundary differ by
           1 cs however close they really were: a one-tick "gap" is
           quantisation, not the desktop. Only two or more is real. */
        if (fg < 2) {
            fg = 0;
        }
        if (fg > cur_g.fgap_max_cs) {
            cur_g.fgap_max_cs = (unsigned short)((fg > 65535U) ? 65535U : fg);
        }
        if (fg > gap_all_cs_g) {
            gap_all_cs_g = fg;
        }
        c = hist_class(fg);
        hist_n_g[c]++;
        hist_cs_g[c] += fg;
    }
    last_begin_g = now_cs;
    last_begin_set_g = 1;
    tick_begin_g = now_cs;
    ADD16(cur_g.ticks, 1);
}

void trace_tick_end(unsigned int now_cs)
{
    last_busy_g = now_cs - tick_begin_g;
    ADD16(cur_g.tick_cs, last_busy_g);
}

void trace_set_idle_cs(unsigned int idle_cs)
{
    idle_cs_g = idle_cs;
}

void trace_backlog(unsigned long bytes)
{
    if (bytes > cur_g.backlog) {
        cur_g.backlog = bytes;
    }
}

void trace_read(int nbytes, unsigned int usb_cs)
{
    ADD16(cur_g.usb_cs, usb_cs);
    if (nbytes <= 0) {
        ADD16(cur_g.empty, 1);
        return;
    }
    ADD16(cur_g.reads, 1);
    cur_g.bytes += (unsigned long)nbytes;
    if ((unsigned long)nbytes > cur_g.maxread) {
        cur_g.maxread = (unsigned long)nbytes;
    }
}

void trace_dsp_cs(unsigned int cs)
{
    ADD16(cur_g.dsp_cs, cs);
}

void trace_fft_cs(unsigned int cs, int frames)
{
    ADD16(cur_g.fft_cs, cs);
    ADD16(cur_g.fft_frames, frames);
}

void trace_disp_cs(unsigned int cs)
{
    ADD16(cur_g.disp_cs, cs);
}

void trace_event(unsigned int now_cs, const char *text)
{
    trace_event_rec *e;

    if (nevents_g >= TRACE_MAX_EVENTS) {
        return;
    }
    e = &events_g[nevents_g++];
    e->t_cs = started_g ? (now_cs - t0_cs_g) : 0;
    strncpy(e->text, text, TRACE_EVENT_LEN - 1);
    e->text[TRACE_EVENT_LEN - 1] = '\0';
}

void trace_recent(double *ksps, unsigned int *foreign_gap_ms)
{
    int n, i, first;
    double bytes;
    unsigned int gap;

    n = nbuckets_g;
    if (n <= 0) {
        *ksps = 0.0;
        *foreign_gap_ms = 0;
        return;
    }
    first = n - TRACE_RECENT_SEC;
    if (first < 0) {
        first = 0;
    }
    bytes = 0.0;
    gap = 0;
    for (i = first; i < n; i++) {
        bytes += (double)buckets_g[i].bytes;
        if (buckets_g[i].fgap_max_cs > gap) {
            gap = buckets_g[i].fgap_max_cs;
        }
    }
    *ksps = bytes / 2.0 / (double)(n - first) / 1000.0;
    *foreign_gap_ms = gap * 10;
}

unsigned int trace_gap_all_ms(void)
{
    return gap_all_cs_g * 10;
}

int trace_write(void)
{
    FILE *f;
    int i;
    unsigned long total_n;

    if (log_path_g[0] == '\0' || !started_g) {
        return 0;
    }
    push_bucket();          /* the partial last second */

    f = fopen(log_path_g, "w");
    if (f == NULL) {
        return -1;
    }
    fprintf(f, "# RTLSDRView trace v2\n");
    fprintf(f, "# %s\n", config_g);
    fprintf(f, "# columns (one row per second): sec bytes reads empty "
               "ticks tick_cs usb_cs dsp_cs fft_cs disp_cs fft_frames "
               "fgap_max_cs maxread backlog ahead_min_ms ahead_max_ms "
               "dropped_ms feed_gap_ms underruns\n");
    fprintf(f, "# (cs = centiseconds; fgap = longest time the rest of the "
               "desktop held the CPU between two polls of this app; ahead "
               "= how far the audio write head ran ahead of the estimated "
               "play position; -999 = no audio data that second)\n");
    for (i = 0; i < nbuckets_g; i++) {
        const trace_bucket *b = &buckets_g[i];
        fprintf(f, "%d %lu %u %u %u %u %u %u %u %u %u %u %lu %lu %d %d %d "
                   "%d %u\n",
                i, b->bytes, (unsigned int)b->reads,
                (unsigned int)b->empty, (unsigned int)b->ticks,
                (unsigned int)b->tick_cs, (unsigned int)b->usb_cs,
                (unsigned int)b->dsp_cs, (unsigned int)b->fft_cs,
                (unsigned int)b->disp_cs, (unsigned int)b->fft_frames,
                (unsigned int)b->fgap_max_cs, b->maxread, b->backlog,
                b->audio_valid ? (int)b->ahead_min_ms : -999,
                b->audio_valid ? (int)b->ahead_max_ms : -999,
                b->audio_valid ? (int)b->dropped_ms : -999,
                (int)b->feed_gap_ms, (unsigned int)b->underruns);
    }
    fprintf(f, "# events (time in seconds since the first poll)\n");
    for (i = 0; i < nevents_g; i++) {
        fprintf(f, "E %.2f %s\n", (double)events_g[i].t_cs / 100.0,
                events_g[i].text);
    }
    fprintf(f, "# foreign-gap histogram: bucket count total_ms\n");
    total_n = 0;
    for (i = 0; i < HIST_N; i++) {
        total_n += hist_n_g[i];
        fprintf(f, "H %s %lu %lu\n", hist_label[i], hist_n_g[i],
                hist_cs_g[i] * 10UL);
    }
    fprintf(f, "# totals: seconds=%d bytes=%.0f polls=%lu underruns=%lu "
               "longest_foreign_gap_ms=%u\n",
            nbuckets_g, total_bytes_g, total_n, total_underruns_g,
            gap_all_cs_g * 10);
    fclose(f);
    return 0;
}
