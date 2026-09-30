/* Host tests for the SSB / CW modes of Rx.c. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include "Rx.h"

#define TWO_PI 6.283185307179586

static double frand(void) { return (double)rand() / (double)RAND_MAX; }
static double gauss(void)
{
    double u1 = frand() + 1e-12, u2 = frand();
    return sqrt(-2.0 * log(u1)) * cos(TWO_PI * u2);
}

typedef struct { double amp, f; } tone;     /* complex tone at offset f (Hz) */

/* Synthesises n complex samples at fs: sum of tones plus noise, 8-bit. If
   jump_at >= 0 the tones' frequencies are unchanged (the test moves the
   dial instead). */
static void synth(double fs, long n, const tone *t, int nt, double noise,
                  unsigned char *o, double *phase)
{
    long k;
    int j;

    for (k = 0; k < n; k++) {
        double re = 0, im = 0;
        for (j = 0; j < nt; j++) {
            double ph = phase[j];
            re += t[j].amp * cos(ph);
            im += t[j].amp * sin(ph);
            phase[j] = fmod(ph + TWO_PI * t[j].f / fs, TWO_PI);
        }
        re += noise * gauss();
        im += noise * gauss();
        {
            int ii = (int)floor(127.5 + re + 0.5);
            int qq = (int)floor(127.5 + im + 0.5);
            if (ii < 0) ii = 0;
            if (ii > 255) ii = 255;
            if (qq < 0) qq = 0;
            if (qq > 255) qq = 255;
            o[2 * k] = (unsigned char)ii;
            o[2 * k + 1] = (unsigned char)qq;
        }
    }
}

static double tone_amp(const short *y, long n, double fs, double f, long skip)
{
    double sc = 0, ss = 0;
    long k, c = 0;

    for (k = skip; k < n; k++) {
        double a = TWO_PI * f * (double)k / fs;
        sc += y[k] * cos(a);
        ss += y[k] * sin(a);
        c++;
    }
    return 2.0 * sqrt(sc * sc + ss * ss) / (double)c;
}

static double rms(const short *y, long n, long skip)
{
    double t = 0;
    long k;

    for (k = skip; k < n; k++) {
        t += (double)y[k] * y[k];
    }
    return sqrt(t / (double)(n - skip));
}

static double peak(const short *y, long n, long skip)
{
    double m = 0;
    long k;

    for (k = skip; k < n; k++) {
        if (fabs((double)y[k]) > m) {
            m = fabs((double)y[k]);
        }
    }
    return m;
}

#define CAP 400000
static short outbuf[CAP];

/* Runs the receiver on tones. mode, bw, offset as the app would set them. */
static long run(int mode, int bw, long offset, double in_rate, int arate,
                const tone *t, int nt, double noise, double seconds,
                int chunk, int sq)
{
    long nsamp = (long)(in_rate * seconds);
    unsigned char *iq = (unsigned char *)malloc((size_t)nsamp * 2);
    double phase[16];
    long pos = 0, no = 0;

    memset(phase, 0, sizeof(phase));
    srand(12345);
    synth(in_rate, nsamp, t, nt, noise, iq, phase);
    rx_init(arate, (long)in_rate);
    rx_set_mode(mode);
    rx_set_bandwidth(bw);
    rx_set_offset(offset);
    rx_set_squelch(sq);
    rx_set_volume(70, 0);
    while (pos < nsamp * 2) {
        long m = chunk;
        if (pos + m > nsamp * 2) m = nsamp * 2 - pos;
        m &= ~1L;
        no += rx_process(iq + pos, (int)m, outbuf + no, (int)(CAP - no));
        pos += m;
    }
    free(iq);
    return no;
}

static int fails = 0;
static void check(const char *name, int ok, const char *detail)
{
    printf("  [%s] %s  %s\n", ok ? "ok" : "FAIL", name, detail);
    if (!ok) fails++;
}

static double db(double a, double b) { return 20.0 * log10(a / b); }

int main(void)
{
    const double FS = 240000.0;
    const int AR = 48000;
    long n, skip = 24000;       /* skip 0.5 s: filter and AGC settle */
    double a700, a1500, a1000, a200, a3500, r;
    char buf[160];
    tone t[4];

    /* ---- USB ---- */
    printf("USB, dial 25 kHz above the dongle centre, 2.4 kHz filter\n");
    {
        const double D = 25000.0;
        t[0].amp = 20; t[0].f = D + 700;
        t[1].amp = 20; t[1].f = D + 1500;
        n = run(RX_MODE_USB, 2400, (long)D, FS, AR, t, 2, 1.0, 3.0, 16384, 0);
        a700 = tone_amp(outbuf, n, AR, 700, skip);
        a1500 = tone_amp(outbuf, n, AR, 1500, skip);
        sprintf(buf, "700 Hz amp %.0f, 1500 Hz amp %.0f (%.2f dB apart), peak %.0f",
                a700, a1500, db(a700, a1500), peak(outbuf, n, skip));
        check("both tones heard, equal", a700 > 1000 && fabs(db(a700, a1500)) < 1.5, buf);

        /* opposite sideband and out-of-band */
        t[0].amp = 20; t[0].f = D + 1000;    /* wanted reference */
        t[1].amp = 20; t[1].f = D - 1000;    /* opposite sideband */
        t[2].amp = 20; t[2].f = D + 3500;    /* above the passband */
        t[3].amp = 20; t[3].f = D + 150;     /* below the passband */
        n = run(RX_MODE_USB, 2400, (long)D, FS, AR, t, 4, 1.0, 3.0, 16384, 0);
        a1000 = tone_amp(outbuf, n, AR, 1000, skip);
        a200 = tone_amp(outbuf, n, AR, 150, skip);
        a3500 = tone_amp(outbuf, n, AR, 3500, skip);
        sprintf(buf, "1000 Hz %.0f; opposite sideband at 1000 Hz is in the same output bin", a1000);
        printf("  (wanted +1000 and opposite -1000 both map to 1000 Hz in the audio only if leakage: amplitude %.0f)\n", a1000);
        /* separate run: only the opposite sideband */
        t[0].amp = 20; t[0].f = D - 1000;
        n = run(RX_MODE_USB, 2400, (long)D, FS, AR, t, 1, 1.0, 3.0, 16384, 0);
        r = tone_amp(outbuf, n, AR, 1000, skip);
        sprintf(buf, "opposite-sideband tone leaks at %.1f (AGC on noise: output rms %.0f)", r, rms(outbuf, n, skip));
        printf("  %s\n", buf);
        /* wanted alone for the reference level at the same input amplitude */
        t[0].amp = 20; t[0].f = D + 1000;
        n = run(RX_MODE_USB, 2400, (long)D, FS, AR, t, 1, 1.0, 3.0, 16384, 0);
        a1000 = tone_amp(outbuf, n, AR, 1000, skip);
        sprintf(buf, "wanted alone %.0f", a1000);
        printf("  %s\n", buf);
    }

    /* Sideband rejection measured with a constant-gain comparison: put a weak
       opposite-sideband tone with a strong wanted one, so AGC is set by the
       wanted one, and measure the opposite one's residue. */
    printf("USB sideband rejection (wanted +1000 Hz at 40, opposite -1000 Hz at 40)\n");
    {
        const double D = 25000.0;
        double w, u;
        t[0].amp = 40; t[0].f = D + 1000;
        t[1].amp = 40; t[1].f = D - 1300;    /* different audio freq: 1300 */
        n = run(RX_MODE_USB, 2400, (long)D, FS, AR, t, 2, 1.0, 3.0, 16384, 0);
        w = tone_amp(outbuf, n, AR, 1000, skip);
        u = tone_amp(outbuf, n, AR, 1300, skip);
        sprintf(buf, "wanted %.0f, opposite sideband %.2f -> %.1f dB down", w, u, db(w, u + 1e-9));
        check("opposite sideband >= 50 dB down", db(w, u + 1e-9) > 50.0, buf);
        t[1].f = D + 3600;                  /* above the passband: audio 3600 */
        n = run(RX_MODE_USB, 2400, (long)D, FS, AR, t, 2, 1.0, 3.0, 16384, 0);
        w = tone_amp(outbuf, n, AR, 1000, skip);
        u = tone_amp(outbuf, n, AR, 3600, skip);
        sprintf(buf, "wanted %.0f, +3600 Hz %.2f -> %.1f dB down", w, u, db(w, u + 1e-9));
        check("above the passband >= 40 dB down", db(w, u + 1e-9) > 40.0, buf);
        t[1].f = D + 100;                   /* below the 300 Hz edge */
        n = run(RX_MODE_USB, 2400, (long)D, FS, AR, t, 2, 1.0, 3.0, 16384, 0);
        w = tone_amp(outbuf, n, AR, 1000, skip);
        u = tone_amp(outbuf, n, AR, 100, skip);
        sprintf(buf, "wanted %.0f, +100 Hz %.2f -> %.1f dB down", w, u, db(w, u + 1e-9));
        check("below the passband >= 20 dB down", db(w, u + 1e-9) > 20.0, buf);
    }

    /* ---- LSB ---- */
    printf("LSB, dial 25 kHz above the dongle centre\n");
    {
        const double D = 25000.0;
        double w, u;
        t[0].amp = 40; t[0].f = D - 1000;
        t[1].amp = 40; t[1].f = D + 1300;    /* opposite sideband */
        n = run(RX_MODE_LSB, 2400, (long)D, FS, AR, t, 2, 1.0, 3.0, 16384, 0);
        w = tone_amp(outbuf, n, AR, 1000, skip);
        u = tone_amp(outbuf, n, AR, 1300, skip);
        sprintf(buf, "wanted (dial-1000) %.0f at 1000 Hz, opposite %.2f -> %.1f dB down", w, u, db(w, u + 1e-9));
        check("LSB: lower sideband heard, upper rejected", w > 1000 && db(w, u + 1e-9) > 50.0, buf);
    }

    /* ---- CW ---- */
    printf("CW, 400 Hz filter: a carrier at the dial should be heard at 700 Hz\n");
    {
        const double D = 30000.0;
        double w, u, u2;
        t[0].amp = 30; t[0].f = D;
        n = run(RX_MODE_CW, 400, (long)D, FS, AR, t, 1, 1.0, 3.0, 16384, 0);
        w = tone_amp(outbuf, n, AR, 700, skip);
        sprintf(buf, "carrier at dial -> %.0f at 700 Hz, peak %.0f", w, peak(outbuf, n, skip));
        check("CW zero-beat is 700 Hz", w > 3000, buf);

        t[0].f = D + 150;
        n = run(RX_MODE_CW, 400, (long)D, FS, AR, t, 1, 1.0, 3.0, 16384, 0);
        u = tone_amp(outbuf, n, AR, 850, skip);
        sprintf(buf, "carrier +150 Hz -> %.0f at 850 Hz", u);
        check("CW +150 Hz is 850 Hz", u > 3000, buf);

        /* interferer 600 Hz away next to the wanted one */
        t[0].amp = 30; t[0].f = D;
        t[1].amp = 30; t[1].f = D + 600;
        n = run(RX_MODE_CW, 400, (long)D, FS, AR, t, 2, 1.0, 3.0, 16384, 0);
        w = tone_amp(outbuf, n, AR, 700, skip);
        u = tone_amp(outbuf, n, AR, 1300, skip);
        u2 = 0;
        sprintf(buf, "wanted %.0f, +600 Hz interferer %.2f -> %.1f dB down", w, u, db(w, u + 1e-9));
        check("CW interferer 600 Hz away >= 40 dB down", db(w, u + 1e-9) > 40.0, buf);
        t[1].f = D - 600;
        n = run(RX_MODE_CW, 400, (long)D, FS, AR, t, 2, 1.0, 3.0, 16384, 0);
        w = tone_amp(outbuf, n, AR, 700, skip);
        u = tone_amp(outbuf, n, AR, 100, skip);
        sprintf(buf, "wanted %.0f, -600 Hz interferer (100 Hz audio) %.2f -> %.1f dB down", w, u, db(w, u + 1e-9));
        check("CW -600 Hz interferer >= 40 dB down", db(w, u + 1e-9) > 40.0, buf);
        (void)u2;
    }

    /* ---- retune-free tuning: the dial moves, the signal does not ---- */
    printf("Moving the dial without a retune (USB)\n");
    {
        const double D = 25000.0;
        long nsamp = (long)(FS * 4.0);
        unsigned char *iq = (unsigned char *)malloc((size_t)nsamp * 2);
        double phase[4] = {0, 0, 0, 0};
        long pos = 0, no = 0, half;
        double a_before, a_after;

        t[0].amp = 30; t[0].f = D + 1000;
        synth(FS, nsamp, t, 1, 1.0, iq, phase);
        rx_init(AR, (long)FS);
        rx_set_mode(RX_MODE_USB);
        rx_set_bandwidth(2400);
        rx_set_offset((long)D);
        rx_set_volume(70, 0);
        half = nsamp;           /* in bytes: 2 s */
        while (pos < half) {
            no += rx_process(iq + pos, 16384, outbuf + no, CAP - (int)no);
            pos += 16384;
        }
        a_before = tone_amp(outbuf + 24000, no - 24000, AR, 1000, 0);
        {
            long first = no;
            rx_set_offset((long)D + 100);    /* the dial is now 100 Hz higher */
            while (pos + 16384 <= nsamp * 2) {
                no += rx_process(iq + pos, 16384, outbuf + no, CAP - (int)no);
                pos += 16384;
            }
            a_after = tone_amp(outbuf + first + 12000, no - first - 12000, AR, 900, 0);
            sprintf(buf, "before: %.0f at 1000 Hz; after +100 Hz dial: %.0f at 900 Hz", a_before, a_after);
            check("tone moves down by 100 Hz", a_before > 1000 && a_after > 1000, buf);
            /* no click: largest sample-to-sample jump around the change */
            {
                double mx = 0; long k;
                for (k = first - 200; k < first + 200; k++) {
                    double dd = fabs((double)outbuf[k] - (double)outbuf[k - 1]);
                    if (dd > mx) mx = dd;
                }
                sprintf(buf, "largest step around the change: %.0f (signal peak ~%.0f)", mx, peak(outbuf + 24000, first - 24000, 0));
                check("no click at the change", mx < 0.5 * peak(outbuf + 24000, first - 24000, 0) + 1, buf);
            }
        }
        free(iq);
    }

    /* ---- chunk-size independence ---- */
    printf("Chunk independence (USB)\n");
    {
        const double D = 25000.0;
        static short ref[CAP];
        long nr, n2;
        int chunks[4] = {16384, 4096, 1000, 2};
        int c;
        int same = 1;

        t[0].amp = 25; t[0].f = D + 900;
        t[1].amp = 15; t[1].f = D + 2100;
        nr = run(RX_MODE_USB, 2400, (long)D, FS, AR, t, 2, 1.0, 0.5, 16384, 0);
        memcpy(ref, outbuf, (size_t)nr * sizeof(short));
        for (c = 1; c < 4; c++) {
            long k;
            n2 = run(RX_MODE_USB, 2400, (long)D, FS, AR, t, 2, 1.0, 0.5, chunks[c], 0);
            if (n2 != nr) {
                same = 0;
                printf("    chunk %d: %ld samples vs %ld\n", chunks[c], n2, nr);
            } else {
                for (k = 0; k < nr; k++) {
                    if (outbuf[k] != ref[k]) {
                        same = 0;
                        printf("    chunk %d differs at %ld\n", chunks[c], k);
                        break;
                    }
                }
            }
        }
        check("identical output for any chunking", same, "");
    }

    /* ---- AGC ---- */
    printf("AGC: level vs input strength (USB single tone)\n");
    {
        const double D = 25000.0;
        double amps[4] = {0.0, 0.0, 0.0, 0.0};
        double lv[4];
        double in[4] = {4, 12, 40, 100};
        int i;
        for (i = 0; i < 4; i++) {
            t[0].amp = in[i]; t[0].f = D + 1000;
            n = run(RX_MODE_USB, 2400, (long)D, FS, AR, t, 1, 1.0, 3.0, 16384, 0);
            amps[i] = tone_amp(outbuf, n, AR, 1000, skip);
            lv[i] = peak(outbuf, n, skip);
            printf("  input %5.0f -> output tone %6.0f  peak %6.0f\n", in[i], amps[i], lv[i]);
        }
        check("40 and 100 within 1.5 dB", fabs(db(amps[2], amps[3])) < 1.5, "");
        check("12 within 4 dB of 100 (AGC floor)", fabs(db(amps[1], amps[3])) < 4.0, "");
    }

    /* ---- attack: a strong signal after a weak one ---- */
    printf("AGC attack/decay\n");
    {
        const double D = 25000.0;
        long nsamp = (long)(FS * 3.0);
        unsigned char *iq = (unsigned char *)malloc((size_t)nsamp * 2);
        double phase[2] = {0, 0};
        long pos = 0, no = 0, k;
        double mx_before, mx_burst, mx_after;

        t[0].amp = 10; t[0].f = D + 1000;
        synth(FS, nsamp, t, 1, 1.0, iq, phase);
        /* boost a 0.5 s stretch at 1.5 s by replacing it with a stronger tone */
        {
            tone s2; double ph2[2] = {0, 0};
            unsigned char *b = (unsigned char *)malloc((size_t)(FS * 0.5) * 2);
            s2.amp = 100; s2.f = D + 1000;
            synth(FS, (long)(FS * 0.5), &s2, 1, 1.0, b, ph2);
            memcpy(iq + (size_t)(FS * 1.5) * 2, b, (size_t)(FS * 0.5) * 2);
            free(b);
        }
        rx_init(AR, (long)FS);
        rx_set_mode(RX_MODE_USB);
        rx_set_bandwidth(2400);
        rx_set_offset((long)D);
        rx_set_volume(70, 0);
        while (pos < nsamp * 2) {
            long m = 16384;
            if (pos + m > nsamp * 2) m = nsamp * 2 - pos;
            no += rx_process(iq + pos, (int)m, outbuf + no, CAP - (int)no);
            pos += m;
        }
        mx_before = mx_burst = mx_after = 0;
        for (k = 48000; k < 70000; k++) mx_before = fmax(mx_before, fabs((double)outbuf[k]));
        for (k = 72000 + 2400; k < 96000 - 2400; k++) mx_burst = fmax(mx_burst, fabs((double)outbuf[k]));
        for (k = 96000 + 24000; k < 140000; k++) mx_after = fmax(mx_after, fabs((double)outbuf[k]));
        printf("  weak peak %.0f, during the strong burst %.0f, back to weak %.0f\n", mx_before, mx_burst, mx_after);
        {
            double first = 0;
            for (k = 72000; k < 72000 + 480; k++) first = fmax(first, fabs((double)outbuf[k]));
            printf("  first 10 ms of the burst: peak %.0f (overshoot before the AGC catches up)\n", first);
        }
        check("burst level controlled (<= 1.5x the weak level)", mx_burst < 1.5 * mx_before, "");
        check("level recovers after the burst", mx_after > 0.6 * mx_before, "");
        free(iq);
    }

    /* ---- noise only, squelch ---- */
    printf("Noise only\n");
    {
        double r0, r1;
        t[0].amp = 0; t[0].f = 0;
        n = run(RX_MODE_USB, 2400, 25000L, FS, AR, t, 1, 1.5, 3.0, 16384, 0);
        r0 = rms(outbuf, n, skip);
        sprintf(buf, "rms %.0f (AGC on noise), peak %.0f", r0, peak(outbuf, n, skip));
        printf("  squelch off: %s\n", buf);
        n = run(RX_MODE_USB, 2400, 25000L, FS, AR, t, 1, 1.5, 3.0, 16384, 50);
        r1 = rms(outbuf, n, skip);
        printf("  squelch 50: rms %.1f\n", r1);
        check("squelch mutes noise", r1 < 0.05 * r0 + 1, "");
        t[0].amp = 40; t[0].f = 25000 + 1000;
        n = run(RX_MODE_USB, 2400, 25000L, FS, AR, t, 1, 1.5, 3.0, 16384, 50);
        r1 = rms(outbuf, n, skip);
        printf("  squelch 50 with a 40-amplitude signal: rms %.0f\n", r1);
        check("squelch opens on a signal", r1 > 1000, "");
    }

    /* ---- 2.4 MSPS input ---- */
    printf("2.4 MSPS input (USB)\n");
    {
        const double D = 25000.0;
        t[0].amp = 30; t[0].f = D + 1000;
        n = run(RX_MODE_USB, 2400, (long)D, 2400000.0, AR, t, 1, 1.0, 1.0, 16384, 0);
        a1000 = tone_amp(outbuf, n, AR, 1000, 12000);
        sprintf(buf, "%.0f at 1000 Hz", a1000);
        check("works through the CIC", a1000 > 1000, buf);
    }

    /* ---- mirror ---- */
    printf("Mirrored stream (USB)\n");
    {
        const double D = 25000.0;
        double w;
        t[0].amp = 30; t[0].f = -(D + 1000);      /* mirrored: signal at -f */
        rx_set_mirror(1);
        n = run(RX_MODE_USB, 2400, (long)D, FS, AR, t, 1, 1.0, 3.0, 16384, 0);
        w = tone_amp(outbuf, n, AR, 1000, skip);
        sprintf(buf, "%.0f at 1000 Hz", w);
        check("mirror flag undoes a mirrored stream", w > 1000, buf);
        {
            int32_t c, wd;
            rx_channel(&c, &wd);
            sprintf(buf, "channel centre %ld width %ld", (long)c, (long)wd);
            check("rx_channel reports the mirrored display position", c < 0, buf);
        }
        rx_set_mirror(0);
    }

    /* ---- other modes still work with the new limits ---- */
    printf("NFM/AM regression smoke\n");
    {
        int32_t c, wd;
        rx_init(AR, 240000);
        rx_set_mode(RX_MODE_NFM);
        rx_set_bandwidth(12500);
        rx_channel(&c, &wd);
        sprintf(buf, "NFM channel %ld/%ld", (long)c, (long)wd);
        check("NFM centred, 12.5k", c == 0 && wd == 12500, buf);
        rx_set_mode(RX_MODE_CW);
        rx_channel(&c, &wd);
        sprintf(buf, "CW channel %ld/%ld (bw reset from NFM's 12500)", (long)c, (long)wd);
        check("switching to CW resets the bandwidth to 400", wd == 400, buf);
    }

    printf("\n%s (%d failures)\n", fails ? "FAILED" : "all passed", fails);
    return fails ? 1 : 0;
}
