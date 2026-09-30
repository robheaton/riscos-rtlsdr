/* Receiver.c -- see h/Receiver.h.

   C89 only (Norcroft): declarations at the top of each block, no //
   comments. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "kernel.h"
#include "Receiver.h"

#define SWI_OS_File  0x08

#define DEFAULT_FREQ_HZ  97400000UL

receiver_t rcv;

const int receiver_steps[RCV_NSTEPS] = {
    100, 500, 1000, 2500, 5000, 6250, 8330, 9000,
    10000, 12500, 25000, 50000, 100000, 200000, 500000, 1000000
};

/* Index of a value in receiver_steps[] (nearest at or below). */
static int step_index_of(int hz)
{
    int i, best;

    best = 0;
    for (i = 0; i < RCV_NSTEPS; i++) {
        if (receiver_steps[i] <= hz) {
            best = i;
        }
    }
    return best;
}

void receiver_defaults(void)
{
    rcv.freq_hz = DEFAULT_FREQ_HZ;
    rcv.mode = RX_MODE_WFM;
    rcv.bw_hz[RX_MODE_NFM] = 12500;
    rcv.bw_hz[RX_MODE_WFM] = 150000;
    rcv.bw_hz[RX_MODE_AM] = 10000;
    rcv.step_idx[RX_MODE_NFM] = step_index_of(12500);
    rcv.step_idx[RX_MODE_WFM] = step_index_of(100000);
    rcv.step_idx[RX_MODE_AM] = step_index_of(5000);
    rcv.squelch = 0;
    rcv.volume = 70;
    rcv.muted = 0;
}

const char *receiver_mode_name(int mode)
{
    switch (mode) {
    case RX_MODE_NFM:
        return "NFM";
    case RX_MODE_WFM:
        return "WFM";
    case RX_MODE_AM:
        return "AM";
    }
    return "?";
}

int receiver_step_hz(void)
{
    return receiver_steps[rcv.step_idx[rcv.mode]];
}

void receiver_step_next(int direction)
{
    int i;

    i = rcv.step_idx[rcv.mode] + direction;
    if (i < 0) {
        i = 0;
    }
    if (i >= RCV_NSTEPS) {
        i = RCV_NSTEPS - 1;
    }
    rcv.step_idx[rcv.mode] = i;
}

int receiver_bw_hz(void)
{
    return rcv.bw_hz[rcv.mode];
}

int receiver_bw_min(int mode)
{
    switch (mode) {
    case RX_MODE_NFM:
        return 4000;
    case RX_MODE_AM:
        return 2000;
    }
    return 0;
}

int receiver_bw_max(int mode)
{
    switch (mode) {
    case RX_MODE_NFM:
        return 40000;
    case RX_MODE_AM:
        return 20000;
    }
    return 0;
}

int receiver_bw_inc(int mode)
{
    switch (mode) {
    case RX_MODE_NFM:
        return 2500;
    case RX_MODE_AM:
        return 1000;
    }
    return 0;
}

unsigned long receiver_clamp_freq(unsigned long hz)
{
    if (hz < RCV_FREQ_MIN_HZ) {
        return RCV_FREQ_MIN_HZ;
    }
    if (hz > RCV_FREQ_MAX_HZ) {
        return RCV_FREQ_MAX_HZ;
    }
    return hz;
}

void receiver_fmt_hz(char *buf, int buflen, long hz)
{
    if (hz >= 1000000L && (hz % 1000000L) == 0) {
        sprintf(buf, "%ldM", hz / 1000000L);
    } else if (hz >= 1000L) {
        long k10;

        k10 = (hz + 5L) / 10L;              /* 10 Hz units */
        if ((k10 % 100L) == 0) {
            sprintf(buf, "%ldk", k10 / 100L);
        } else if ((k10 % 10L) == 0) {
            sprintf(buf, "%ld.%ldk", k10 / 100L, (k10 % 100L) / 10L);
        } else {
            sprintf(buf, "%ld.%02ldk", k10 / 100L, k10 % 100L);
        }
    } else {
        sprintf(buf, "%ld", hz);
    }
    buf[buflen - 1] = '\0';
}

void receiver_apply_mode(void)
{
    rx_set_mode(rcv.mode);
    receiver_apply_bw();
    receiver_apply_squelch();
    receiver_apply_volume();
}

void receiver_apply_bw(void)
{
    if (rcv.mode != RX_MODE_WFM) {
        rx_set_bandwidth(rcv.bw_hz[rcv.mode]);
    }
}

void receiver_apply_squelch(void)
{
    rx_set_squelch(rcv.squelch);
}

void receiver_apply_volume(void)
{
    rx_set_volume(rcv.volume, rcv.muted);
}

/* ---- Choices ----

   Reading uses the Choices: path (Choices:RTLSDRView.Config), writing the
   directory named by Choices$Write. If that variable is not set there is
   nowhere to save, and the settings simply do not persist. */

#define CHOICES_LEAF  "RTLSDRView.Config"

static int in_range(long v, long lo, long hi)
{
    return v >= lo && v <= hi;
}

void receiver_load(void)
{
    FILE *f;
    char line[96];
    char key[24];
    long v;
    int i;

    receiver_defaults();
    f = fopen("Choices:" CHOICES_LEAF, "r");
    if (f == NULL) {
        return;
    }
    while (fgets(line, (int)sizeof(line), f) != NULL) {
        if (sscanf(line, "%23s %ld", key, &v) != 2) {
            continue;
        }
        if (strcmp(key, "freq") == 0) {
            if (in_range(v, (long)RCV_FREQ_MIN_HZ, (long)RCV_FREQ_MAX_HZ)) {
                rcv.freq_hz = (unsigned long)v;
            }
        } else if (strcmp(key, "mode") == 0) {
            if (in_range(v, 0, RX_NMODES - 1)) {
                rcv.mode = (int)v;
            }
        } else if (strcmp(key, "squelch") == 0) {
            if (in_range(v, 0, 100)) {
                rcv.squelch = (int)v;
            }
        } else if (strcmp(key, "volume") == 0) {
            if (in_range(v, 0, 100)) {
                rcv.volume = (int)v;
            }
        } else if (strcmp(key, "muted") == 0) {
            rcv.muted = (v != 0);
        } else if (strncmp(key, "bw", 2) == 0 && key[2] >= '0' &&
                   key[2] < '0' + RX_NMODES && key[3] == '\0') {
            i = key[2] - '0';
            if (i != RX_MODE_WFM &&
                in_range(v, receiver_bw_min(i), receiver_bw_max(i))) {
                rcv.bw_hz[i] = (int)v;
            }
        } else if (strncmp(key, "step", 4) == 0 && key[4] >= '0' &&
                   key[4] < '0' + RX_NMODES && key[5] == '\0') {
            i = key[4] - '0';
            if (in_range(v, 0, RCV_NSTEPS - 1)) {
                rcv.step_idx[i] = (int)v;
            }
        }
    }
    fclose(f);
}

int receiver_save(void)
{
    const char *base;
    char path[200];
    FILE *f;
    int i;
    _kernel_swi_regs regs;

    base = getenv("Choices$Write");
    if (base == NULL || *base == '\0' || strlen(base) > 120) {
        return -1;
    }
    /* Make sure the application's directory exists (OS_File 8). */
    sprintf(path, "%s.RTLSDRView", base);
    regs.r[0] = 8;
    regs.r[1] = (int)path;
    regs.r[4] = 0;
    _kernel_swi(SWI_OS_File, &regs, &regs);

    sprintf(path, "%s.%s", base, CHOICES_LEAF);
    f = fopen(path, "w");
    if (f == NULL) {
        return -1;
    }
    fprintf(f, "freq %lu\n", rcv.freq_hz);
    fprintf(f, "mode %d\n", rcv.mode);
    fprintf(f, "squelch %d\n", rcv.squelch);
    fprintf(f, "volume %d\n", rcv.volume);
    fprintf(f, "muted %d\n", rcv.muted);
    for (i = 0; i < RX_NMODES; i++) {
        fprintf(f, "bw%d %d\n", i, rcv.bw_hz[i]);
        fprintf(f, "step%d %d\n", i, rcv.step_idx[i]);
    }
    fclose(f);
    return 0;
}
