/* Dsp.c -- integer-only real-time wideband-FM demodulator.
   See Dsp.h for the rationale and the signal chain.

   C89 only (Norcroft): declarations at the top of each block, no //
   comments. All per-sample arithmetic is 32-bit integer. CIC integrators
   deliberately wrap modulo 2^32, so they are `unsigned long` (signed
   overflow is undefined in C; unsigned wrap is well defined). */

#include <math.h>
#include "Dsp.h"

#define CIC_R_MAX        16

/* Angles are Q14 radians (16384 = 1 rad) throughout. */
#define Q14_PI           51472
#define Q14_HALF_PI      25736
#define Q14_QUARTER_PI   12868

/* Whatever the input rate, the complex samples handed to the
   discriminator are scaled so their peak magnitude lands in
   (8192, 16384]: the cross/dot terms are sums of two products, and
   2 * 16384 * 16384 = 5.4e8 stays inside 32 bits. (At the original
   2.4 MSPS the 3rd-order CIC output reaches 128 * 10^3 = 128000, which
   is scaled down by 8 bits' worth of shift: 16000.) */
#define DISC_FULL_SCALE  16384

/* De-emphasis time constant, microseconds: 50 in Europe/UK, 75 in the
   Americas/Korea. */
#define DEEMPH_US        50

#define STAT_FLUSH_EVERY 256     /* fold integer sums into doubles this often */

/* ---- configuration (set by dsp_fm_init) ---- */
static long chan_rate = DSP_CHANNEL_RATE_HZ; /* rate after the CIC stage */
static int cic_r = 10;             /* decimation factor; 1 = no CIC at all */
static int cic_shr = 3;            /* right shift applied to the CIC output */
static int cic_mul = 1;            /* ...or a multiplier, for small factors */
static int audio_rate;
static int deemph_a;               /* de-emphasis coefficient, Q16 */
static int gain_q12 = 4096;

/* ---- state ---- */
static unsigned long a1i, a2i, a3i, a1q, a2q, a3q; /* CIC integrators */
static unsigned long c1i, c2i, c3i, c1q, c2q, c3q; /* CIC comb delays */
static int blk_pos;                /* samples accumulated in current block */
static int prev_i, prev_q;         /* previous channel-rate sample (scaled) */
static int deemph_y;               /* de-emphasis filter state, Q14 rad */
static long rs_acc;                /* resampler phase accumulator */
static long rs_sum;                /* running sum inside the current output sample */
static int rs_cnt;

/* Output of the call in progress. */
static short *o_buf;
static int o_max;
static int o_n;

/* Statistics: small integer accumulators, folded into doubles every
   STAT_FLUSH_EVERY samples so they can't overflow. */
static unsigned long b_sumsq;
static unsigned long b_n;
static int b_peak;
static long lvl_ema;               /* smoothed |z|^2 at the channel rate */
static double stat_sumsq_d;        /* sum over samples of (|d| >> 5)^2 */
static double stat_n_d;
static int stat_peak;              /* max |d| since last take_stats */

/* Q14 atan2 for full-range integer inputs (|x|,|y| up to ~5e8).
   Method: reduce to an octant, form t = min/max in Q12 (after shifting
   both down so the division can't overflow), evaluate
       atan(t) ~ (pi/4) t + t (1 - t) (0.2447 + 0.0663 t)
   (Rajan et al. 2006, max error ~0.0015 rad = 0.09 degrees), then undo
   the octant reduction. */
int dsp_atan2_q14(int y, int x)
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
    b_sumsq = 0;
    b_n = 0;
    b_peak = 0;
    lvl_ema = 0;
    stat_sumsq_d = 0.0;
    stat_n_d = 0.0;
    stat_peak = 0;
}

void dsp_fm_init(int audio_rate_hz, long input_rate_hz)
{
    long r;
    long mag;
    int sh;

    if (input_rate_hz < DSP_CHANNEL_RATE_HZ) {
        input_rate_hz = DSP_INPUT_RATE_HZ;
    }
    r = (input_rate_hz + DSP_CHANNEL_RATE_HZ / 2) / DSP_CHANNEL_RATE_HZ;
    if (r < 1) {
        r = 1;
    }
    if (r > CIC_R_MAX) {
        r = CIC_R_MAX;
    }
    cic_r = (int)r;
    chan_rate = input_rate_hz / cic_r;

    /* Peak magnitude out of the CIC is 128 * R^3 (input +-128, gain R^3
       for a 3rd-order, unit-delay CIC); with no CIC it is just 128.
       Scale it into (DISC_FULL_SCALE/2, DISC_FULL_SCALE]. */
    mag = 128L * (long)cic_r * (long)cic_r * (long)cic_r;
    sh = 0;
    while (mag > DISC_FULL_SCALE) {
        mag >>= 1;
        sh++;
    }
    while (mag * 2 <= DISC_FULL_SCALE) {
        mag <<= 1;
        sh--;
    }
    cic_shr = (sh > 0) ? sh : 0;
    cic_mul = (sh < 0) ? (1 << (-sh)) : 1;

    if (audio_rate_hz < 1) {
        audio_rate_hz = 1;
    }
    if (audio_rate_hz > (int)chan_rate) {
        audio_rate_hz = (int)chan_rate;
    }
    audio_rate = audio_rate_hz;

    /* One-pole low-pass, y += a (x - y), a = 1 - exp(-1 / (fs tau)),
       in Q16. Computed once with floating point; never on the per-sample
       path. For tau = 50 us at 240 kSPS this is ~5243, comfortably
       below 8192 so (diff * a) can't overflow 32 bits. */
    deemph_a = (int)(65536.0 *
                     (1.0 - exp(-1.0 / ((double)chan_rate *
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

/* One complex baseband sample at the channel rate: discriminator,
   de-emphasis, resampler, statistics. */
static void demod_sample(int zi, int zq)
{
    int cross, dot, d, ad, v;

    lvl_ema += (((long)zi * zi + (long)zq * zq) - lvl_ema) >> 12;

    /* arg( z[n] * conj(z[n-1]) ): positive frequency deviation gives a
       positive phase step. */
    cross = zq * prev_i - zi * prev_q;
    dot = zi * prev_i + zq * prev_q;
    prev_i = zi;
    prev_q = zq;
    d = dsp_atan2_q14(cross, dot);

    ad = (d < 0) ? -d : d;
    if (ad > b_peak) {
        b_peak = ad;
    }
    v = ad >> 5;
    b_sumsq += (unsigned long)(v * v);
    b_n++;
    if (b_n >= STAT_FLUSH_EVERY) {
        stat_sumsq_d += (double)b_sumsq;
        stat_n_d += (double)b_n;
        b_sumsq = 0;
        b_n = 0;
    }

    /* De-emphasis (also the first stage of anti-aliasing for the
       resampler below: it rolls off the 19 kHz stereo pilot and the
       38 kHz L-R subcarrier). */
    deemph_y += (int)(((long)(d - deemph_y) * deemph_a) >> 16);

    /* Integrate-and-dump resampler: average every channel-rate sample
       that falls inside one output-sample interval. The Bresenham-style
       accumulator handles any output rate, including ones that don't
       divide 240 kSPS evenly. */
    rs_sum += deemph_y;
    rs_cnt++;
    rs_acc += audio_rate;
    if (rs_acc >= chan_rate) {
        rs_acc -= chan_rate;
        if (o_n < o_max) {
            long s;
            s = ((rs_sum / rs_cnt) * (long)gain_q12) >> 12;
            if (s > 32767) {
                s = 32767;
            }
            if (s < -32768) {
                s = -32768;
            }
            o_buf[o_n++] = (short)s;
        }
        rs_sum = 0;
        rs_cnt = 0;
    }
}

/* Advances the front end until it has produced one complex sample at the
   channel rate (recentred, CIC-decimated if the input rate is above
   240 kSPS, scaled into the discriminator's +-16384 range) or has run out
   of input. Returns 1 with the sample in *pzi and *pzq, or 0 when the input is
   exhausted (a partly filled CIC block is carried to the next call).
   *ppiq and *pnsamp are advanced past the consumed input. */
static int cic_next(const unsigned char **ppiq, int *pnsamp, int *pzi,
                    int *pzq)
{
    const unsigned char *iq;
    int nsamp;

    iq = *ppiq;
    nsamp = *pnsamp;

    if (cic_r == 1) {
        /* The dongle is already delivering the channel rate: no
           decimation, just recentre and scale. */
        if (nsamp <= 0) {
            return 0;
        }
        *pzi = ((int)iq[0] - 128) * cic_mul;
        *pzq = ((int)iq[1] - 128) * cic_mul;
        *ppiq = iq + 2;
        *pnsamp = nsamp - 1;
        return 1;
    }

    {
        int take;
        int k;
        unsigned long i1, i2, i3, q1, q2, q3;
        unsigned long y, d1, d2, d3;

        take = cic_r - blk_pos;
        if (take > nsamp) {
            take = nsamp;
        }

        /* Integrator section at the full input rate: the only loop that
           runs once per input sample, and it is pure integer adds. */
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
        *ppiq = iq;
        *pnsamp = nsamp;
        if (blk_pos < cic_r) {
            return 0;                /* block incomplete; carry to next call */
        }
        blk_pos = 0;

        /* Comb section at the decimated rate. */
        y = a3i;
        d1 = y - c1i; c1i = y;
        d2 = d1 - c2i; c2i = d1;
        d3 = d2 - c3i; c3i = d2;
        /* d3 holds a small signed value modulo 2^32; recover it without
           an implementation-defined out-of-range cast. */
        *pzi = (((int)(d3 + 0x40000000UL) - 0x40000000) >> cic_shr) *
               cic_mul;

        y = a3q;
        d1 = y - c1q; c1q = y;
        d2 = d1 - c2q; c2q = d1;
        d3 = d2 - c3q; c3q = d2;
        *pzq = (((int)(d3 + 0x40000000UL) - 0x40000000) >> cic_shr) *
               cic_mul;
        return 1;
    }
}

int dsp_fm_process(const unsigned char *iq, int nbytes, short *out,
                   int max_out)
{
    int nsamp;
    int zi, zq;

    nsamp = nbytes >> 1;
    o_buf = out;
    o_max = max_out;
    o_n = 0;

    while (cic_next(&iq, &nsamp, &zi, &zq)) {
        demod_sample(zi, zq);
    }

    if (b_n > 0) {
        stat_sumsq_d += (double)b_sumsq;
        stat_n_d += (double)b_n;
        b_sumsq = 0;
        b_n = 0;
    }
    if (b_peak > stat_peak) {
        stat_peak = b_peak;
    }
    b_peak = 0;
    return o_n;
}

int dsp_front_end(const unsigned char *iq, int nbytes, short *zi, short *zq,
                  int max)
{
    int nsamp;
    int n;
    int i, q;

    nsamp = nbytes >> 1;
    n = 0;
    while (n < max && cic_next(&iq, &nsamp, &i, &q)) {
        zi[n] = (short)i;
        zq[n] = (short)q;
        n++;
    }
    return n;
}

long dsp_fm_level_power(void)
{
    return lvl_ema;
}

int dsp_fm_take_stats(double *peak_hz, double *rms_hz)
{
    /* Q14 radians per sample -> Hz at the channel rate. */
    const double hz_per_lsb = (double)chan_rate /
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
