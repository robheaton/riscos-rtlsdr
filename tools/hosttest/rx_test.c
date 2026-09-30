/* Host tests for Rx.c: synthetic AM / NFM at 240 kSPS and 2.4 MSPS. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include "Rx.h"
#include "Dsp.h"

#define TWO_PI 6.283185307179586

static double frand(void) { return (double)rand() / (double)RAND_MAX; }
static double gauss(void) { double u1 = frand() + 1e-12, u2 = frand(); return sqrt(-2.0*log(u1))*cos(TWO_PI*u2); }

typedef struct { int kind; double amp, f0, fm, m_or_dev; } sig;   /* kind 0=AM 1=NFM */

static void synth(double fs, long n, const sig *s, int ns, double noise, unsigned char *o)
{
    long k; int j;
    for (k = 0; k < n; k++) {
        double t = (double)k / fs, re = 0, im = 0;
        for (j = 0; j < ns; j++) {
            double ph, a;
            if (s[j].kind == 0) { a = s[j].amp * (1.0 + s[j].m_or_dev * cos(TWO_PI*s[j].fm*t)); ph = TWO_PI*s[j].f0*t; }
            else { a = s[j].amp; ph = TWO_PI*s[j].f0*t + (s[j].m_or_dev/s[j].fm)*sin(TWO_PI*s[j].fm*t); }
            re += a*cos(ph); im += a*sin(ph);
        }
        re += noise*gauss(); im += noise*gauss();
        { int ii=(int)floor(127.5+re+0.5), qq=(int)floor(127.5+im+0.5);
          if(ii<0)ii=0; if(ii>255)ii=255; if(qq<0)qq=0; if(qq>255)qq=255; o[2*k]=ii; o[2*k+1]=qq; }
    }
}

static long run(int mode, int bw, int sq, long in_rate, int arate, const unsigned char *iq, long nsamp,
                int chunk, short *out, long cap)
{
    long pos = 0, no = 0;
    rx_init(arate, in_rate);
    rx_set_mode(mode);
    rx_set_bandwidth(bw);
    rx_set_squelch(sq);
    rx_set_volume(70, 0);
    while (pos < nsamp*2) { long m = chunk; if (pos+m > nsamp*2) m = nsamp*2-pos; m &= ~1L;
        no += rx_process(iq+pos, (int)m, out+no, (int)(cap-no)); pos += m; }
    return no;
}

static double tone_amp(const short *y, long n, double fs, double f, long skip, double *rest)
{
    double sc=0, ss=0, tot=0; long k, c=0;
    for (k = skip; k < n; k++) { double a = TWO_PI*f*(double)k/fs; sc += y[k]*cos(a); ss += y[k]*sin(a); tot += (double)y[k]*y[k]; c++; }
    { double amp = 2.0*sqrt(sc*sc+ss*ss)/(double)c; double sig = amp*amp/2.0; if (rest) *rest = tot/(double)c - sig; return amp; }
}

static void report(const char *name, double amp, double rest, double expect)
{
    double snr = (rest > 0) ? 10.0*log10((amp*amp/2.0)/rest) : 99.0;
    printf("  %-44s amp=%8.1f  (expect ~%7.1f, %5.1f%%)  SNR=%5.1f dB\n", name, amp, expect, expect>0?100.0*amp/expect:0.0, snr);
}

int main(void)
{
    const long FS = 240000L; const double secs = 1.0; long n = (long)(secs*FS);
    unsigned char *iq = malloc((size_t)n*2); short *out = malloc(400000*sizeof(short));
    long no; double rest, amp; sig s[3];
    srand(4242);

    printf("== AM, 240 kSPS input, 10 kHz channel, 48 kHz audio (vol 70 ~ unity)\n");
    s[0].kind=0; s[0].amp=60; s[0].f0=0; s[0].fm=1000; s[0].m_or_dev=0.5;
    synth(FS,n,s,1,3.0,iq); no = run(RX_MODE_AM,10000,0,FS,48000,iq,n,16384,out,400000);
    amp = tone_amp(out,no,48000,1000,12000,&rest); report("carrier 60, m=0.5, offset 0", amp, rest, 0.5*16000*0.98);
    s[0].amp=25; synth(FS,n,s,1,3.0,iq); no = run(RX_MODE_AM,10000,0,FS,48000,iq,n,16384,out,400000);
    amp = tone_amp(out,no,48000,1000,12000,&rest); report("carrier 25 (weaker), m=0.5", amp, rest, 0.5*16000*0.98);
    s[0].amp=100; s[0].f0=1500; synth(FS,n,s,1,3.0,iq); no = run(RX_MODE_AM,10000,0,FS,48000,iq,n,16384,out,400000);
    amp = tone_amp(out,no,48000,1000,12000,&rest); report("carrier 100, offset +1.5 kHz", amp, rest, 0.5*16000*0.98);
    s[0].f0=-3000; synth(FS,n,s,1,3.0,iq); no = run(RX_MODE_AM,10000,0,FS,48000,iq,n,16384,out,400000);
    amp = tone_amp(out,no,48000,1000,12000,&rest); report("carrier 100, offset -3 kHz", amp, rest, 0.5*16000*0.98);

    printf("== AM frequency response (carrier 60, m=0.5)\n");
    { double fr[] = {100,200,300,1000,2000,3000,4000,4500,5500}; int i;
      s[0].amp=60; s[0].f0=0;
      for (i = 0; i < 9; i++) { char nm[40]; s[0].fm=fr[i]; synth(FS,n,s,1,2.0,iq);
        no = run(RX_MODE_AM,10000,0,FS,48000,iq,n,16384,out,400000); amp = tone_amp(out,no,48000,fr[i],12000,&rest);
        sprintf(nm,"tone %5.0f Hz", fr[i]); report(nm,amp,rest,0.5*16000*0.98); } }

    printf("== AM adjacent-channel: wanted 1 kHz m=0.5 at 0; interferer carrier 60 (5x stronger), 2 kHz m=0.8 at +9 kHz / +20 kHz\n");
    { double offs[] = {9000, 20000}; int i;
      for (i = 0; i < 2; i++) { char nm[60];
        s[0].kind=0; s[0].amp=12; s[0].f0=0; s[0].fm=1000; s[0].m_or_dev=0.5;
        s[1].kind=0; s[1].amp=60; s[1].f0=offs[i]; s[1].fm=2000; s[1].m_or_dev=0.8;
        synth(FS,n,s,2,1.0,iq); no = run(RX_MODE_AM,10000,0,FS,48000,iq,n,16384,out,400000);
        { double a1 = tone_amp(out,no,48000,1000,12000,&rest), a2 = tone_amp(out,no,48000,2000,12000,NULL);
          sprintf(nm,"interferer at +%.0f Hz: 2k leak / 1k wanted = %.1f dB", offs[i], 20*log10((a2+1e-9)/a1)); printf("  %-60s\n", nm); } } }

    printf("== NFM, 240 kSPS input, 12.5 kHz channel: 1 kHz tone, +-3 kHz deviation\n");
    s[0].kind=1; s[0].amp=60; s[0].f0=0; s[0].fm=1000; s[0].m_or_dev=3000;
    synth(FS,n,s,1,3.0,iq); no = run(RX_MODE_NFM,12500,0,FS,48000,iq,n,16384,out,400000);
    amp = tone_amp(out,no,48000,1000,12000,&rest); report("offset 0", amp, rest, 3000.0/5000*16000*0.98);
    s[0].f0=2500; synth(FS,n,s,1,3.0,iq); no = run(RX_MODE_NFM,12500,0,FS,48000,iq,n,16384,out,400000);
    amp = tone_amp(out,no,48000,1000,12000,&rest); report("offset +2.5 kHz", amp, rest, 3000.0/5000*16000*0.98);
    s[0].amp=15; s[0].f0=0; synth(FS,n,s,1,3.0,iq); no = run(RX_MODE_NFM,12500,0,FS,48000,iq,n,16384,out,400000);
    amp = tone_amp(out,no,48000,1000,12000,&rest); report("weak signal (amp 15, noise 3)", amp, rest, 3000.0/5000*16000*0.98);
    printf("== NFM adjacent-channel: wanted 1 kHz +-3 kHz at 0; interferer 5x stronger, 2 kHz +-3 kHz at +12.5 / +25 kHz\n");
    { double offs[] = {12500, 25000}; int i;
      for (i = 0; i < 2; i++) { char nm[70];
        s[0].kind=1; s[0].amp=12; s[0].f0=0; s[0].fm=1000; s[0].m_or_dev=3000;
        s[1].kind=1; s[1].amp=60; s[1].f0=offs[i]; s[1].fm=2000; s[1].m_or_dev=3000;
        synth(FS,n,s,2,1.0,iq); no = run(RX_MODE_NFM,12500,0,FS,48000,iq,n,16384,out,400000);
        { double a1 = tone_amp(out,no,48000,1000,12000,&rest), a2 = tone_amp(out,no,48000,2000,12000,NULL);
          sprintf(nm,"interferer at +%.0f Hz: 2k leak / 1k wanted = %.1f dB", offs[i], 20*log10((a2+1e-9)/a1)); printf("  %-70s\n", nm); } } }

    printf("== squelch metrics (noise only vs signal), NFM 12.5 kHz and AM 10 kHz\n");
    { double nzs[] = {2.0, 4.0, 8.0}; int i;
      for (i = 0; i < 3; i++) {
        s[0].kind=1; s[0].amp=0; s[0].f0=0; s[0].fm=1000; s[0].m_or_dev=3000;
        synth(FS,n,s,1,nzs[i],iq); rx_init(48000,FS); rx_set_mode(RX_MODE_NFM); rx_set_bandwidth(12500);
        { long pos=0; no=0; while (pos < n*2) { long m=16384; if (pos+m>n*2) m=n*2-pos; no+=rx_process(iq+pos,(int)m,out+no,400000-(int)no); pos+=m; } }
        printf("  NFM noise-only sigma=%.0f : nz metric = %ld\n", nzs[i], rx_debug_squelch_metric());
        s[0].amp=40; synth(FS,n,s,1,nzs[i],iq); rx_init(48000,FS); rx_set_mode(RX_MODE_NFM); rx_set_bandwidth(12500);
        { long pos=0; no=0; while (pos < n*2) { long m=16384; if (pos+m>n*2) m=n*2-pos; no+=rx_process(iq+pos,(int)m,out+no,400000-(int)no); pos+=m; } }
        printf("  NFM signal 40, sigma=%.0f : nz metric = %ld\n", nzs[i], rx_debug_squelch_metric()); } }
    { double nzs[] = {2.0, 4.0, 8.0}; int i;
      for (i = 0; i < 3; i++) { rx_stats st;
        s[0].kind=0; s[0].amp=0; synth(FS,n,s,1,nzs[i],iq); no = run(RX_MODE_AM,10000,0,FS,48000,iq,n,16384,out,400000); rx_take_stats(&st);
        printf("  AM noise-only sigma=%.0f : carrier metric = %ld  level=%.1f dBFS\n", nzs[i], rx_debug_squelch_metric(), st.level_db10/10.0);
        s[0].amp=40; s[0].m_or_dev=0.5; synth(FS,n,s,1,nzs[i],iq); no = run(RX_MODE_AM,10000,0,FS,48000,iq,n,16384,out,400000); rx_take_stats(&st);
        printf("  AM carrier 40, sigma=%.0f : carrier metric = %ld  level=%.1f dBFS\n", nzs[i], rx_debug_squelch_metric(), st.level_db10/10.0); } }

    printf("== squelch gating: NFM sq=50, noise then signal then noise (1 s each)\n");
    { long k3 = n*3; unsigned char *big = malloc((size_t)k3*2); short *o2 = malloc(1000000*sizeof(short)); long i;
      s[0].kind=1; s[0].f0=0; s[0].fm=1000; s[0].m_or_dev=3000; s[0].amp=0; synth(FS,n,s,1,4.0,big);
      s[0].amp=50; synth(FS,n,s,1,4.0,big+n*2);
      s[0].amp=0; synth(FS,n,s,1,4.0,big+n*4);
      no = run(RX_MODE_NFM,12500,50,FS,48000,big,k3,16384,o2,1000000);
      { double e[3] = {0,0,0}; long cnt[3] = {0,0,0};
        for (i = 0; i < no; i++) { int seg = (int)(i/48000); if (seg>2) seg=2; e[seg] += (double)o2[i]*o2[i]; cnt[seg]++; }
        printf("  rms audio: noise(1) %.0f  signal(2) %.0f  noise(3) %.0f\n", sqrt(e[0]/cnt[0]), sqrt(e[1]/cnt[1]), sqrt(e[2]/cnt[2])); }
      free(big); free(o2); }

    printf("== chunk-size independence (bit-exact): AM and NFM, chunks 16384 / 2000 / 346 / 2\n");
    { int modes[2] = {RX_MODE_AM, RX_MODE_NFM}; int mi, ci; int chunks[4] = {16384, 2000, 346, 2};
      long small = 30000; 
      for (mi = 0; mi < 2; mi++) {
        s[0].kind = (modes[mi]==RX_MODE_AM)?0:1; s[0].amp=60; s[0].f0=1000; s[0].fm=700; s[0].m_or_dev=(modes[mi]==RX_MODE_AM)?0.6:3000;
        synth(FS,small,s,1,3.0,iq);
        { short *ref = malloc(100000*sizeof(short)); long nref = run(modes[mi],10000,30,FS,44100,iq,small,16384,ref,100000); 
          for (ci = 1; ci < 4; ci++) { long n2 = run(modes[mi],10000,30,FS,44100,iq,small,chunks[ci],out,100000);
            printf("  mode %d chunk %5d: %s (n=%ld vs %ld)\n", modes[mi], chunks[ci], (n2==nref && memcmp(ref,out,nref*sizeof(short))==0)?"identical":"DIFFERENT", n2, nref); }
          free(ref); } } }

    printf("== 2.4 MSPS input (CIC x10), AM and NFM\n");
    { long FS2 = 2400000L; long n2 = (long)(0.5*FS2); unsigned char *iq2 = malloc((size_t)n2*2);
      s[0].kind=0; s[0].amp=60; s[0].f0=0; s[0].fm=1000; s[0].m_or_dev=0.5; synth(FS2,n2,s,1,3.0,iq2);
      no = run(RX_MODE_AM,10000,0,FS2,48000,iq2,n2,16384,out,400000); amp = tone_amp(out,no,48000,1000,6000,&rest); report("AM via CIC", amp, rest, 0.5*16000*0.98);
      s[0].kind=1; s[0].m_or_dev=3000; synth(FS2,n2,s,1,3.0,iq2);
      no = run(RX_MODE_NFM,12500,0,FS2,48000,iq2,n2,16384,out,400000); amp = tone_amp(out,no,48000,1000,6000,&rest); report("NFM via CIC", amp, rest, 3000.0/5000*16000*0.98);
      free(iq2); }

    printf("== other audio rates (NFM tone should stay at 1 kHz)\n");
    { int rates[2] = {44100, 96000}; int i;
      s[0].kind=1; s[0].amp=60; s[0].f0=0; s[0].fm=1000; s[0].m_or_dev=3000; synth(FS,n,s,1,3.0,iq);
      for (i = 0; i < 2; i++) { char nm[40]; no = run(RX_MODE_NFM,12500,0,FS,rates[i],iq,n,16384,out,400000);
        amp = tone_amp(out,no,rates[i],1000,rates[i]/4,&rest); sprintf(nm,"audio rate %d (n=%ld)",rates[i],no); report(nm,amp,rest,3000.0/5000*16000*0.98); } }
    free(iq); free(out);
    return 0;
}
