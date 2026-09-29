/* Dsp.c -- integer-only real-time wideband-FM demodulator.
   See Dsp.h for the rationale and the signal chain.

   C89 only (Norcroft): declarations at the top of each block, no //
   comments. All per-sample arithmetic is 32-bit integer. CIC integrators
   deliberately wrap modulo 2^32, so they are `unsigned long` (signed
   overflow is undefined in C; unsigned wrap is well defined). */

#include <math.h>
#include "Dsp.h"

#define CIC_R            10      /* 2.4 MSPS / 10 = 240 kSPS */

/* Angles are Q14 radians (16384 = 1 rad) throughout. */
#define Q14_PI           51472
#define Q14_HALF_PI      25736
#define Q14_QUARTER_PI   12868

/* CIC output (up to 128 * 10^3 = 128000, 17 bits signed) is scaled down
   by this many bits before the discriminator's products, so that
   cross/dot terms (sums of two products) stay inside 32 bits:
   16000 * 16000 * 2 = 5.1e8 < 2^31. */
#define CIC_OUT_SHIFT    3

/* De-emphasis time constant, microseconds: 50 in Europe/UK, 75 in the
   Americas/Korea. */
#define DEEMPH_US        50

#define STAT_FLUSH_EVERY 256     /* fold integer sums into doubles this often */

/* ---- state ---- */
static unsigned long a1i, a2i, a3i, a1q, a2q, a3q; /* CIC integrators */
static unsigned long c1i, c2i, c3i, c1q, c2q, c3q; /* CIC comb delays */
static int blk_pos;                /* samples accumulated in current block */
static int prev_i, prev_q;         /* previous 240 kSPS sample (scaled) */
static int deemph_y;               /* de-emphasis filter state, Q14 rad */
static int deemph_a;               /* de-emphasis coefficient, Q16 */
static int audio_rate;
static int gain_q12 = 4096;
static long rs_acc;                /* resampler phase accumulator */
static long rs_sum;                /* running sum inside the current output sample */
static int rs_cnt;

static double stat_sumsq_d;        /* sum over samples of (|d| >> 5)^2 */
static double stat_n_d;
static int stat_peak;              /* max |d| since last take_stats */

/* Q14 atan2 for full-range integer inputs (|x|,|y| up to ~5e8).
   Method: reduce to an octant, form t = min/max in Q12 (after shifting
   both down so the division can't overflow), evaluate
       atan(t) ~ (pi/4) t + t (1 - t) (0.2447 + 0.0663 t)
   (Rajan et al. 2006, max error ~0.0015 rad = 0.09 degrees), then undo
   the octant reduction. */
static int atan2_q14(int y, int x)
{
    int ax, ay, num, den, t, a;

    ax = (x < 0) ? -x : x;
    ay = (y < 0) ? -y : y;
    if (ax == 0 && ay == 0) {
        return 0;
    }
    if (ay <= ax) {
        num = ay;
        den = ax;
    } else {
        num = ax;
        den = ay;
    }
    while (den >= (1 << 18)) {
        den >>= 1;
        num >>= 1;
    }
    t = (num << 12) / den;                           /* Q12, 0..4096 */
    a = ((Q14_QUARTER_PI * t) >> 12) +
        ((t * (((4096 - t) * (4009 + ((1086 * t) >> 12))) >> 12)) >> 12);
    if (ay > ax) {
        a = Q14_HALF_PI - a;
    }
    if (x < 0) {
        a = Q14_PI - a;
    }
    if (y < 0) {
        a = -a;
    }
    return a;
}

void dsp_fm_reset(void)
{
    a1i = a2i = a3i = a1q = a2q = a3q = 0;
    c1i = c2i = c3i = c1q = c2q = c3q = 0;
    blk_pos = 0;
    prev_i = 0;
    prev_q = 0;
    deemph_y = 0;
    rs_acc = 0;
    rs_sum = 0;
    rs_cnt = 0;
    stat_sumsq_d = 0.0;
    stat_n_d = 0.0;
    stat_peak = 0;
}

void dsp_fm_init(int audio_rate_hz)
{
    if (audio_rate_hz < 1) {
        audio_rate_hz = 1;
    }
    if (audio_rate_hz > (int)DSP_CHANNEL_RATE_HZ) {
        audio_rate_hz = (int)DSP_CHANNEL_RATE_HZ;
    }
    audio_rate = audio_rate_hz;

    /* One-pole low-pass, y += a (x - y), a = 1 - exp(-1 / (fs tau)),
       in Q16. Computed once with floating point; never on the per-sample
       path. For tau = 50 us at 240 kSPS this is ~5243, comfortably
       below 8192 so (diff * a) can't overflow 32 bits. */
    deemph_a = (int)(65536.0 *
                     (1.0 - exp(-1.0 / ((double)DSP_CHANNEL_RATE_HZ *
                                        (double)DEEMPH_US * 1.0e-6))) + 0.5);
    dsp_fm_reset();
}

void dsp_fm_set_gain(int g)
{
    if (g < 0) {
        g = 0;
    }
    if (g > 32768) {
        g = 32768;
    }
    gain_q12 = g;
}

int dsp_fm_process(const unsigned char *iq, int nbytes, short *out,
                   int max_out)
{
    int nsamp;
    int nout;
    unsigned long blk_n;
    unsigned long blk_sumsq;
    int blk_peak;

    nsamp = nbytes >> 1;
    nout = 0;
    blk_n = 0;
    blk_sumsq = 0;
    blk_peak = 0;

    while (nsamp > 0) {
        int take;
        int k;
        unsigned long i1, i2, i3, q1, q2, q3;

        take = CIC_R - blk_pos;
        if (take > nsamp) {
            take = nsamp;
        }

        /* Integrator section at the full 2.4 MSPS rate: the only loop
           that runs once per input sample, and it is pure integer adds. */
        i1 = a1i; i2 = a2i; i3 = a3i;
        q1 = a1q; q2 = a2q; q3 = a3q;
        for (k = 0; k < take; k++) {
            unsigned long xi, xq;
            xi = (unsigned long)((int)iq[0] - 128);
            xq = (unsigned long)((int)iq[1] - 128);
            iq += 2;
            i1 += xi; i2 += i1; i3 += i2;
            q1 += xq; q2 += q1; q3 += q2;
        }
        a1i = i1; a2i = i2; a3i = i3;
        a1q = q1; a2q = q2; a3q = q3;

        blk_pos += take;
        nsamp -= take;
        if (blk_pos < CIC_R) {
            break;                   /* block incomplete; carry to next call */
        }
        blk_pos = 0;

        {
            unsigned long y, d1, d2, d3;
            int zi, zq, cross, dot, d, ad, v;

            /* Comb section at the decimated 240 kSPS rate. */
            y = a3i;
            d1 = y - c1i; c1i = y;
            d2 = d1 - c2i; c2i = d1;
            d3 = d2 - c3i; c3i = d2;
            /* d3 holds a small signed value modulo 2^32; recover it
               without an implementation-defined out-of-range cast. */
            zi = ((int)(d3 + 0x40000000UL) - 0x40000000) >> CIC_OUT_SHIFT;

            y = a3q;
            d1 = y - c1q; c1q = y;
            d2 = d1 - c2q; c2q = d1;
            d3 = d2 - c3q; c3q = d2;
            zq = ((int)(d3 + 0x40000000UL) - 0x40000000) >> CIC_OUT_SHIFT;

            /* arg( z[n] * conj(z[n-1]) ): positive frequency deviation
               gives a positive phase step. */
            cross = zq * prev_i - zi * prev_q;
            dot = zi * prev_i + zq * prev_q;
            prev_i = zi;
            prev_q = zq;
            d = atan2_q14(cross, dot);

            ad = (d < 0) ? -d : d;
            if (ad > blk_peak) {
                blk_peak = ad;
            }
            v = ad >> 5;
            blk_sumsq += (unsigned long)(v * v);
            blk_n++;
            if (blk_n >= STAT_FLUSH_EVERY) {
                stat_sumsq_d += (double)blk_sumsq;
                stat_n_d += (double)blk_n;
                blk_sumsq = 0;
                blk_n = 0;
            }

            /* De-emphasis (also the first stage of anti-aliasing for the
               resampler below: it rolls off the 19 kHz stereo pilot and
               the 38 kHz L-R subcarrier). */
            deemph_y += (int)(((long)(d - deemph_y) * deemph_a) >> 16);

            /* Integrate-and-dump resampler: average every 240 kSPS
               sample that falls inside one output-sample interval. The
               Bresenham-style accumulator handles any output rate,
               including ones that don't divide 240 kSPS evenly. */
            rs_sum += deemph_y;
            rs_cnt++;
            rs_acc += audio_rate;
            if (rs_acc >= DSP_CHANNEL_RATE_HZ) {
                rs_acc -= DSP_CHANNEL_RATE_HZ;
                if (nout < max_out) {
                    long s;
                    s = ((rs_sum / rs_cnt) * (long)gain_q12) >> 12;
                    if (s > 32767) {
                        s = 32767;
                    }
                    if (s < -32768) {
                        s = -32768;
                    }
                    out[nout++] = (short)s;
                }
                rs_sum = 0;
                rs_cnt = 0;
            }
        }
    }

    if (blk_n > 0) {
        stat_sumsq_d += (double)blk_sumsq;
        stat_n_d += (double)blk_n;
    }
    if (blk_peak > stat_peak) {
        stat_peak = blk_peak;
    }
    return nout;
}

int dsp_fm_take_stats(double *peak_hz, double *rms_hz)
{
    /* Q14 radians per sample -> Hz at the channel rate. */
    const double hz_per_lsb = (double)DSP_CHANNEL_RATE_HZ /
                              (2.0 * 3.14159265358979323846 * 16384.0);

    if (stat_n_d <= 0.0) {
        return 0;
    }
    *rms_hz = sqrt(stat_sumsq_d / stat_n_d) * 32.0 * hz_per_lsb;
    *peak_hz = (double)stat_peak * hz_per_lsb;
    stat_sumsq_d = 0.0;
    stat_n_d = 0.0;
    stat_peak = 0;
    return 1;
}
