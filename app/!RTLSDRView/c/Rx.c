/* Rx.c -- see h/Rx.h.

   Narrow-band signal path (NFM, AM):

     raw I/Q --Dsp front end--> 240 kSPS complex (+-16384)
       --stage A: 63-tap FIR, decimate by 5--> 48 kSPS
       --stage B: channel filter, linear-phase FIR sized to the bandwidth-->
       --demodulator (FM discriminator | AM envelope + carrier AGC)-->
       --audio filters--> squelch gate --> volume --> resample to the mixer.

   Stage A only has to keep anything that would alias into the widest
   channel (+-12.5 kHz) out; stage B does the real selectivity. Both are
   symmetric low-pass FIRs with Q15 taps designed with a Blackman window.

   C89 only (Norcroft): declarations at the top of each block, no //
   comments. All per-sample arithmetic is 32-bit integer. */

#include <math.h>
#include <string.h>
#include "Dsp.h"
#include "Rx.h"

#define PI               3.14159265358979323846

#define DEC              5                 /* 240 kSPS -> 48 kSPS */
#define NA               63                /* stage A taps (odd) */
#define NB_MAX           127               /* stage B taps, at most (odd) */
#define CHUNK_BYTES      16384
#define FE_MAX           (CHUNK_BYTES / 2) /* complex samples per chunk */
#define MID_MAX          (FE_MAX / DEC + 8)

#define FULL_SCALE_POWER 268435456.0       /* 16384^2 */
#define LVL_SHIFT        10                /* level EMA: ~21 ms at 48 kSPS */
#define AM_K             12                /* carrier EMA: ~85 ms */
#define AM_LEVEL         16000             /* audio at 100% modulation */
#define NFM_GAIN_Q12     6112              /* 5 kHz deviation -> 16000 */
#define GATE_STEP        136               /* 5 ms full ramp at 48 kHz */
/* Noise level the NFM squelch metric settles at with no signal. The
   discriminator output of noise is close to uniform phase whatever the
   gain, so this is nearly a constant of the chain (measured in the host
   test harness: 5400-5900 for input noise of 4-8 LSB rms; it is lower,
   ~2900, for very quiet input). */
#define NFM_NOISE_REF    6000.0
#define SQ_TICK          48                /* squelch decides every 1 ms */
#define SQ_HANG_AM       200               /* ticks (ms) the gate stays open
                                              after the carrier drops */
#define SQ_HANG_NFM      40                /* short: a long one lets the
                                              noise burst at the end of a
                                              transmission through */
#define SQ_HANG_WFM      150

/* ---- configuration ---- */
static int audio_rate = 48000;
static int mode = RX_MODE_WFM;
static int bw_hz = 10000;
static int squelch_level = 0;
static int volume_pct = 70;
static int muted = 0;
static int vol_q12 = 4096;

/* ---- stage A ---- */
static short ha[(NA - 1) / 2 + 1];
static short a_hist_i[NA - 1], a_hist_q[NA - 1];
static int a_cnt;
static short fe_i[FE_MAX], fe_q[FE_MAX];
static short wa_i[NA - 1 + FE_MAX], wa_q[NA - 1 + FE_MAX];
static short mid_i[MID_MAX], mid_q[MID_MAX];

/* ---- stage B ---- */
static int nb = 63;
static short hb[(NB_MAX - 1) / 2 + 1];
static short b_hist_i[NB_MAX - 1], b_hist_q[NB_MAX - 1];
static short wb_i[NB_MAX - 1 + MID_MAX], wb_q[NB_MAX - 1 + MID_MAX];
static short ch_i[MID_MAX], ch_q[MID_MAX];

/* ---- biquads (Q13 coefficients, direct form I) ---- */
typedef struct {
    long b0, b1, b2, a1, a2;
    long x1, x2, y1, y2;
} biquad;

static biquad bq_lp;        /* audio low-pass */
static biquad bq_hp;        /* NFM: sub-audible tone (CTCSS) removal */
static biquad bq_nz;        /* NFM: high-pass for the noise squelch */

/* ---- demodulator state ---- */
static long lvl;                     /* smoothed |z|^2 (narrow modes) */
static int nfm_pi, nfm_pq;
static long am_car;                  /* carrier amplitude, Q8 */
static int am_init;
static long dc_x1, dc_y1;            /* AM DC-blocking one-pole */
static long nz;                      /* NFM noise metric */

/* ---- squelch ---- */
static int sq_open = 1;
static int sq_hang;
static int sq_tick;
static long am_thr_open, am_thr_close;
static long nz_thr_open, nz_thr_close;
static double wfm_thr_db = -200.0;
static int gate = 32767;

/* ---- output ---- */
static short *e_out;
static int e_n;
static int e_max;
static long rs_prev;
static long rs_phase;                /* Q12 */
static long rs_step;                 /* Q12 input samples per output sample */

/* ---- statistics ---- */
static unsigned long nf_sumsq;
static unsigned long nf_n;
static int nf_peak;
static double nf_sumsq_d;
static double nf_n_d;
static unsigned long stat_count;

static int clip16(long v)
{
    if (v > 32767) {
        return 32767;
    }
    if (v < -32768) {
        return -32768;
    }
    return (int)v;
}

static unsigned long isqrt32(unsigned long x)
{
    unsigned long res, bit;

    res = 0;
    bit = 1UL << 30;
    while (bit > x) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (x >= res + bit) {
            x -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

/* ---- filter design (doubles are fine here: it runs on a mode or
   bandwidth change, never per sample) ---- */

/* Symmetric low-pass of n taps (odd), cutoff fc in cycles/sample, Blackman
   window, DC gain exactly 1.0 (32768 in Q15). Only the first half plus the
   middle tap is stored. */
static void design_lowpass(short *h_half, int n, double fc)
{
    double hd[NB_MAX];
    double sum, w, x;
    int k, mid;
    long isum;

    mid = (n - 1) / 2;
    sum = 0.0;
    for (k = 0; k < n; k++) {
        x = (double)(k - mid);
        if (k == mid) {
            hd[k] = 2.0 * fc;
        } else {
            hd[k] = sin(2.0 * PI * fc * x) / (PI * x);
        }
        w = 0.42 - 0.5 * cos(2.0 * PI * (double)k / (double)(n - 1)) +
            0.08 * cos(4.0 * PI * (double)k / (double)(n - 1));
        hd[k] *= w;
        sum += hd[k];
    }
    isum = 0;
    for (k = 0; k <= mid; k++) {
        h_half[k] = (short)floor(hd[k] / sum * 32768.0 + 0.5);
        isum += (k == mid) ? (long)h_half[k] : 2L * (long)h_half[k];
    }
    h_half[mid] = (short)(h_half[mid] + (32768L - isum));
}

static void bq_clear(biquad *q)
{
    q->x1 = q->x2 = q->y1 = q->y2 = 0;
}

/* RBJ cookbook low-pass / high-pass, Q = 0.7071. */
static void bq_design(biquad *q, int highpass, double fs, double fc)
{
    double w0, cw, alpha, a0, b0, b1, b2, a1, a2;

    if (fc > 0.45 * fs) {
        fc = 0.45 * fs;
    }
    w0 = 2.0 * PI * fc / fs;
    cw = cos(w0);
    alpha = sin(w0) / (2.0 * 0.70710678);
    a0 = 1.0 + alpha;
    a1 = -2.0 * cw;
    a2 = 1.0 - alpha;
    if (highpass) {
        b0 = (1.0 + cw) / 2.0;
        b1 = -(1.0 + cw);
        b2 = (1.0 + cw) / 2.0;
    } else {
        b0 = (1.0 - cw) / 2.0;
        b1 = 1.0 - cw;
        b2 = (1.0 - cw) / 2.0;
    }
    q->b0 = (long)floor(b0 / a0 * 8192.0 + 0.5);
    q->b1 = (long)floor(b1 / a0 * 8192.0 + 0.5);
    q->b2 = (long)floor(b2 / a0 * 8192.0 + 0.5);
    q->a1 = (long)floor(a1 / a0 * 8192.0 + 0.5);
    q->a2 = (long)floor(a2 / a0 * 8192.0 + 0.5);
}

static int bq_run(biquad *q, int x)
{
    long y;

    y = (q->b0 * x + q->b1 * q->x1 + q->b2 * q->x2 -
         q->a1 * q->y1 - q->a2 * q->y2) >> 13;
    if (y > 32767) {
        y = 32767;
    }
    if (y < -32768) {
        y = -32768;
    }
    q->x2 = q->x1;
    q->x1 = x;
    q->y2 = q->y1;
    q->y1 = y;
    return (int)y;
}

static void design_stage_b(void)
{
    double ntaps;

    if (mode == RX_MODE_WFM) {
        return;
    }
    /* Blackman transition ~ 5.5 / N of the sample rate: aim for a
       transition of 0.4 x the bandwidth. */
    ntaps = 5.5 * (double)48000 / (0.4 * (double)bw_hz);
    nb = (int)ntaps | 1;
    if (nb < 31) {
        nb = 31;
    }
    if (nb > NB_MAX) {
        nb = NB_MAX;
    }
    design_lowpass(hb, nb, ((double)bw_hz / 2.0) / 48000.0);
}

static void design_audio(void)
{
    double fc;

    if (mode == RX_MODE_AM) {
        fc = 0.45 * (double)bw_hz;
        if (fc > 4500.0) {
            fc = 4500.0;
        }
        bq_design(&bq_lp, 0, 48000.0, fc);
    } else if (mode == RX_MODE_NFM) {
        fc = 0.30 * (double)bw_hz;
        if (fc > 3400.0) {
            fc = 3400.0;
        }
        bq_design(&bq_lp, 0, 48000.0, fc);
        bq_design(&bq_hp, 1, 48000.0, 300.0);
        bq_design(&bq_nz, 1, 48000.0, 6000.0);
    }
}

static void update_squelch_thresholds(void)
{
    double s, db;

    s = (double)squelch_level / 100.0;
    /* AM: carrier amplitude threshold, -70 dBFS (level 1) to -20 dBFS (100) */
    db = -70.0 + 0.5 * (double)squelch_level;
    am_thr_open = (long)(16384.0 * pow(10.0, db / 20.0));
    am_thr_close = (am_thr_open * 4) / 5;
    if (am_thr_open < 1) {
        am_thr_open = 1;
    }
    /* NFM: open when the noise metric drops below a fraction of its
       no-signal value; the fraction falls from 0.9 to 0.05 as the level
       goes up. */
    nz_thr_open = (long)(NFM_NOISE_REF * (0.9 - 0.85 * s));
    nz_thr_close = (nz_thr_open * 7) / 5;
    if (nz_thr_close > (long)(NFM_NOISE_REF * 0.97)) {
        nz_thr_close = (long)(NFM_NOISE_REF * 0.97);
    }
    /* WFM: signal level, -50 dBFS to -5 dBFS */
    wfm_thr_db = -50.0 + 0.45 * (double)squelch_level;
}

static void set_volume_gain(void)
{
    long v;

    v = ((long)volume_pct * (long)volume_pct * 8192L) / 10000L;
    vol_q12 = muted ? 0 : (int)v;
    dsp_fm_set_gain(vol_q12);
}

void rx_reset(void)
{
    dsp_fm_reset();
    a_cnt = 0;
    memset(a_hist_i, 0, sizeof(a_hist_i));
    memset(a_hist_q, 0, sizeof(a_hist_q));
    memset(b_hist_i, 0, sizeof(b_hist_i));
    memset(b_hist_q, 0, sizeof(b_hist_q));
    bq_clear(&bq_lp);
    bq_clear(&bq_hp);
    bq_clear(&bq_nz);
    lvl = 0;
    nfm_pi = 0;
    nfm_pq = 0;
    am_car = 0;
    am_init = 0;
    dc_x1 = 0;
    dc_y1 = 0;
    nz = (long)NFM_NOISE_REF;
    sq_open = (squelch_level == 0);
    sq_hang = 0;
    sq_tick = 0;
    gate = sq_open ? 32767 : 0;
    rs_prev = 0;
    rs_phase = 0;
    nf_sumsq = 0;
    nf_n = 0;
    nf_peak = 0;
    nf_sumsq_d = 0.0;
    nf_n_d = 0.0;
    stat_count = 0;
}

void rx_init(int audio_rate_hz, long input_rate_hz)
{
    if (audio_rate_hz < 8000) {
        audio_rate_hz = 8000;
    }
    if (audio_rate_hz > 192000) {
        audio_rate_hz = 192000;
    }
    audio_rate = audio_rate_hz;
    rs_step = ((long)48000 << 12) / (long)audio_rate;
    dsp_fm_init(audio_rate_hz, input_rate_hz);
    design_lowpass(ha, NA, 21000.0 / 240000.0);
    mode = RX_MODE_WFM;
    bw_hz = 10000;
    set_volume_gain();
    update_squelch_thresholds();
    rx_reset();
}

void rx_set_mode(int m)
{
    if (m < 0 || m >= RX_NMODES) {
        m = RX_MODE_WFM;
    }
    mode = m;
    if (mode == RX_MODE_NFM && (bw_hz < 4000 || bw_hz > 40000)) {
        bw_hz = 12500;
    }
    if (mode == RX_MODE_AM && (bw_hz < 2000 || bw_hz > 20000)) {
        bw_hz = 10000;
    }
    design_stage_b();
    design_audio();
    update_squelch_thresholds();
    rx_reset();
}

void rx_set_bandwidth(int hz)
{
    if (mode == RX_MODE_NFM) {
        if (hz < 4000) {
            hz = 4000;
        }
        if (hz > 40000) {
            hz = 40000;
        }
    } else {
        if (hz < 2000) {
            hz = 2000;
        }
        if (hz > 20000) {
            hz = 20000;
        }
    }
    bw_hz = hz;
    design_stage_b();
    design_audio();
}

void rx_set_squelch(int level)
{
    if (level < 0) {
        level = 0;
    }
    if (level > 100) {
        level = 100;
    }
    squelch_level = level;
    update_squelch_thresholds();
    if (squelch_level == 0) {
        sq_open = 1;
    }
}

void rx_set_volume(int percent, int mute)
{
    if (percent < 0) {
        percent = 0;
    }
    if (percent > 100) {
        percent = 100;
    }
    volume_pct = percent;
    muted = mute ? 1 : 0;
    set_volume_gain();
}

/* ---- the output stage shared by every narrow mode ---- */

/* x = one audio sample at 48 kHz, before the squelch gate and volume. */
static void emit(int x)
{
    long y, o;

    if (sq_open) {
        gate += GATE_STEP;
        if (gate > 32767) {
            gate = 32767;
        }
    } else {
        gate -= GATE_STEP;
        if (gate < 0) {
            gate = 0;
        }
    }
    y = ((long)x * (long)gate) >> 15;
    y = (y * (long)vol_q12) >> 12;
    y = clip16(y);

    if (audio_rate == 48000) {
        if (e_n < e_max) {
            e_out[e_n++] = (short)y;
        }
        return;
    }
    /* Linear-interpolating resampler, 48 kHz -> the mixer rate. */
    while (rs_phase < 4096) {
        o = rs_prev + (((y - rs_prev) * rs_phase) >> 12);
        if (e_n < e_max) {
            e_out[e_n++] = (short)o;
        }
        rs_phase += rs_step;
    }
    rs_phase -= 4096;
    rs_prev = y;
}

/* One squelch decision (every SQ_TICK samples): cond = the signal is
   wanted. Hysteresis comes from the caller choosing the threshold by
   sq_open; the hang keeps it open across short gaps. */
static void sq_decide(int cond, int hang_ticks)
{
    if (squelch_level == 0) {
        sq_open = 1;
        return;
    }
    if (cond) {
        sq_open = 1;
        sq_hang = hang_ticks;
    } else if (sq_open) {
        sq_hang--;
        if (sq_hang <= 0) {
            sq_open = 0;
        }
    }
}

/* ---- stages A and B ---- */

static int stage_a(int n)
{
    int t, k, mid, out;
    long si, sq;

    memcpy(wa_i, a_hist_i, (NA - 1) * sizeof(short));
    memcpy(wa_q, a_hist_q, (NA - 1) * sizeof(short));
    memcpy(wa_i + (NA - 1), fe_i, (size_t)n * sizeof(short));
    memcpy(wa_q + (NA - 1), fe_q, (size_t)n * sizeof(short));
    mid = (NA - 1) / 2;
    out = 0;
    for (t = 0; t < n; t++) {
        a_cnt++;
        if (a_cnt >= DEC) {
            a_cnt = 0;
            si = (long)ha[mid] * wa_i[t + mid];
            sq = (long)ha[mid] * wa_q[t + mid];
            for (k = 0; k < mid; k++) {
                si += (long)ha[k] * (wa_i[t + k] + wa_i[t + NA - 1 - k]);
                sq += (long)ha[k] * (wa_q[t + k] + wa_q[t + NA - 1 - k]);
            }
            mid_i[out] = (short)clip16((si + 16384) >> 15);
            mid_q[out] = (short)clip16((sq + 16384) >> 15);
            out++;
        }
    }
    memcpy(a_hist_i, wa_i + n, (NA - 1) * sizeof(short));
    memcpy(a_hist_q, wa_q + n, (NA - 1) * sizeof(short));
    return out;
}

static int stage_b(int n)
{
    int t, k, mid, s;
    long si, sq;

    memcpy(wb_i, b_hist_i, (NB_MAX - 1) * sizeof(short));
    memcpy(wb_q, b_hist_q, (NB_MAX - 1) * sizeof(short));
    memcpy(wb_i + (NB_MAX - 1), mid_i, (size_t)n * sizeof(short));
    memcpy(wb_q + (NB_MAX - 1), mid_q, (size_t)n * sizeof(short));
    mid = (nb - 1) / 2;
    for (t = 0; t < n; t++) {
        s = NB_MAX - nb + t;
        si = (long)hb[mid] * wb_i[s + mid];
        sq = (long)hb[mid] * wb_q[s + mid];
        for (k = 0; k < mid; k++) {
            si += (long)hb[k] * (wb_i[s + k] + wb_i[s + nb - 1 - k]);
            sq += (long)hb[k] * (wb_q[s + k] + wb_q[s + nb - 1 - k]);
        }
        ch_i[t] = (short)clip16((si + 16384) >> 15);
        ch_q[t] = (short)clip16((sq + 16384) >> 15);
    }
    memcpy(b_hist_i, wb_i + n, (NB_MAX - 1) * sizeof(short));
    memcpy(b_hist_q, wb_q + n, (NB_MAX - 1) * sizeof(short));
    return n;
}

/* ---- demodulators ---- */

static void am_block(int n)
{
    int k, zi, zq, env, den, m, x, y;
    long p;

    for (k = 0; k < n; k++) {
        zi = ch_i[k];
        zq = ch_q[k];
        p = (long)zi * zi + (long)zq * zq;
        lvl += (p - lvl) >> LVL_SHIFT;
        env = (int)isqrt32((unsigned long)p);
        if (!am_init) {
            am_car = (long)env << 8;
            am_init = 1;
        }
        am_car += (((long)env << 8) - am_car) >> AM_K;
        den = (int)(am_car >> 8);
        if (den < 1) {
            den = 1;
        }
        /* Modulation index relative to the carrier, Q12: the audio level
           does not depend on how strong the signal is. */
        m = (int)((((long)env - den) * 4096L) / den);
        if (m > 8191) {
            m = 8191;
        }
        if (m < -8191) {
            m = -8191;
        }
        x = (m * AM_LEVEL) >> 12;
        /* One-pole DC block (~60 Hz). */
        y = (int)((long)x - dc_x1 + ((dc_y1 * 32510L) >> 15));
        dc_x1 = x;
        dc_y1 = y;
        y = bq_run(&bq_lp, y);

        sq_tick++;
        if (sq_tick >= SQ_TICK) {
            sq_tick = 0;
            sq_decide((am_car >> 8) > (sq_open ? am_thr_close : am_thr_open),
                      SQ_HANG_AM);
        }
        emit(y);
    }
}

static void nfm_block(int n)
{
    int k, zi, zq, d, ad, x, y, hp, v;
    long cross, dot, p, a;

    for (k = 0; k < n; k++) {
        zi = ch_i[k];
        zq = ch_q[k];
        p = (long)zi * zi + (long)zq * zq;
        lvl += (p - lvl) >> LVL_SHIFT;

        cross = (long)zq * nfm_pi - (long)zi * nfm_pq;
        dot = (long)zi * nfm_pi + (long)zq * nfm_pq;
        nfm_pi = zi;
        nfm_pq = zq;
        d = dsp_atan2_q14((int)cross, (int)dot);

        ad = (d < 0) ? -d : d;
        if (ad > nf_peak) {
            nf_peak = ad;
        }
        v = ad >> 5;
        nf_sumsq += (unsigned long)(v * v);
        nf_n++;
        if (nf_n >= 256) {
            nf_sumsq_d += (double)nf_sumsq;
            nf_n_d += (double)nf_n;
            nf_sumsq = 0;
            nf_n = 0;
        }

        x = clip16(((long)d * NFM_GAIN_Q12) >> 12);

        /* Noise squelch: energy above the voice band. With a signal the
           discriminator is quiet up there; without one it is loud. */
        hp = bq_run(&bq_nz, x);
        a = (hp < 0) ? -hp : hp;
        nz += (a - nz) >> 8;

        y = bq_run(&bq_hp, x);
        y = bq_run(&bq_lp, y);

        sq_tick++;
        if (sq_tick >= SQ_TICK) {
            sq_tick = 0;
            sq_decide(nz < (sq_open ? nz_thr_close : nz_thr_open),
                      SQ_HANG_NFM);
        }
        emit(y);
    }
}

static int process_narrow(const unsigned char *iq, int nbytes, short *out,
                          int max_out)
{
    int n_fe, n_a, n_b;

    e_out = out;
    e_n = 0;
    e_max = max_out;

    n_fe = dsp_front_end(iq, nbytes, fe_i, fe_q, FE_MAX);
    n_a = stage_a(n_fe);
    n_b = stage_b(n_a);
    if (mode == RX_MODE_AM) {
        am_block(n_b);
    } else {
        nfm_block(n_b);
    }
    stat_count += (unsigned long)n_b;
    return e_n;
}

/* WFM: Dsp.c's path, plus a level-based squelch gate over the block. */
static void wfm_gate(short *buf, int n)
{
    int i;
    int want;
    double pwr, db;
    long y;

    if (squelch_level == 0) {
        sq_open = 1;
    } else {
        pwr = (double)dsp_fm_level_power();
        db = (pwr > 1.0) ? 10.0 * log10(pwr / FULL_SCALE_POWER) : -200.0;
        want = db > (sq_open ? wfm_thr_db - 3.0 : wfm_thr_db);
        if (want) {
            sq_open = 1;
            sq_hang = SQ_HANG_WFM;
        } else if (sq_open) {
            /* one decision per block (~30-90 ms): a short hang */
            sq_hang -= 50;
            if (sq_hang <= 0) {
                sq_open = 0;
            }
        }
    }
    for (i = 0; i < n; i++) {
        if (sq_open) {
            gate += GATE_STEP;
            if (gate > 32767) {
                gate = 32767;
            }
        } else {
            gate -= GATE_STEP;
            if (gate < 0) {
                gate = 0;
            }
        }
        if (gate < 32767) {
            y = ((long)buf[i] * (long)gate) >> 15;
            buf[i] = (short)y;
        }
    }
}

int rx_process(const unsigned char *iq, int nbytes, short *out, int max_out)
{
    int total, chunk, got;

    if (mode == RX_MODE_WFM) {
        got = dsp_fm_process(iq, nbytes, out, max_out);
        wfm_gate(out, got);
        stat_count += (unsigned long)got;
        return got;
    }
    total = 0;
    while (nbytes > 0) {
        chunk = (nbytes > CHUNK_BYTES) ? CHUNK_BYTES : nbytes;
        total += process_narrow(iq, chunk, out + total, max_out - total);
        iq += chunk;
        nbytes -= chunk;
    }
    return total;
}

int rx_take_stats(rx_stats *st)
{
    double p, pk, rms;
    const double hz_per_lsb = 48000.0 / (2.0 * PI * 16384.0);

    if (stat_count == 0) {
        return 0;
    }
    stat_count = 0;
    st->have_dev = 0;
    st->peak_hz = 0.0;
    st->rms_hz = 0.0;
    st->squelch_open = sq_open;

    if (mode == RX_MODE_WFM) {
        if (dsp_fm_take_stats(&pk, &rms)) {
            st->have_dev = 1;
            st->peak_hz = pk;
            st->rms_hz = rms;
        }
        p = (double)dsp_fm_level_power();
    } else {
        p = (double)lvl;
        if (mode == RX_MODE_NFM) {
            nf_sumsq_d += (double)nf_sumsq;
            nf_n_d += (double)nf_n;
            nf_sumsq = 0;
            nf_n = 0;
            if (nf_n_d > 0.0) {
                st->have_dev = 1;
                st->rms_hz = sqrt(nf_sumsq_d / nf_n_d) * 32.0 * hz_per_lsb;
                st->peak_hz = (double)nf_peak * hz_per_lsb;
            }
            nf_sumsq_d = 0.0;
            nf_n_d = 0.0;
            nf_peak = 0;
        }
    }
    if (p < 1.0) {
        st->level_db10 = -999;
    } else {
        st->level_db10 = (int)floor(100.0 * log10(p / FULL_SCALE_POWER) +
                                    0.5);
    }
    return 1;
}

long rx_debug_squelch_metric(void)
{
    if (mode == RX_MODE_AM) {
        return am_car >> 8;
    }
    return nz;
}
