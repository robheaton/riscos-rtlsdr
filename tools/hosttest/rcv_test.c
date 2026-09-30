#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "Receiver.h"
static int fails;
static void ck(const char *n, int ok) { printf("  [%s] %s\n", ok ? "ok" : "FAIL", n); if (!ok) fails++; }
int main(void)
{
    char b[32];
    unsigned long hw;
    receiver_defaults();
    ck("default bw NFM 12500", rcv.bw_hz[RX_MODE_NFM] == 12500);
    ck("default bw USB 2400, CW 400", rcv.bw_hz[RX_MODE_USB] == 2400 && rcv.bw_hz[RX_MODE_CW] == 400);
    ck("default steps USB 100, CW 50", receiver_steps[rcv.step_idx[RX_MODE_USB]] == 100 && receiver_steps[rcv.step_idx[RX_MODE_CW]] == 50);
    ck("default step NFM 12.5k, WFM 100k, AM 5k", receiver_steps[rcv.step_idx[0]] == 12500 && receiver_steps[rcv.step_idx[1]] == 100000 && receiver_steps[rcv.step_idx[2]] == 5000);
    ck("step table ascending", ({int i, ok = 1; for (i = 1; i < RCV_NSTEPS; i++) if (receiver_steps[i] <= receiver_steps[i-1]) ok = 0; ok;}));
    ck("bw limits USB 1000..4000", receiver_bw_min(RX_MODE_USB) == 1000 && receiver_bw_max(RX_MODE_USB) == 4000);
    ck("bw limits CW 100..1000", receiver_bw_min(RX_MODE_CW) == 100 && receiver_bw_max(RX_MODE_CW) == 1000);
    ck("bw WFM 0", receiver_bw_min(RX_MODE_WFM) == 0 && receiver_bw_inc(RX_MODE_WFM) == 0);
    /* hw target */
    ck("FM modes: hw = dial", receiver_hw_target(97400000UL, RX_MODE_WFM, 0UL) == 97400000UL && receiver_hw_target(97400000UL, RX_MODE_NFM, 12345UL) == 97400000UL);
    hw = receiver_hw_target(14200000UL, RX_MODE_USB, 0UL);
    ck("USB from cold: 45 kHz below the dial", hw == 14155000UL);
    ck("USB: dial moves 30 kHz up inside the window -> no retune", receiver_hw_target(14185000UL, RX_MODE_USB, hw) == hw);
    ck("USB: +80 kHz edge still no retune", receiver_hw_target(14235000UL, RX_MODE_USB, hw) == hw);
    ck("USB: +81 kHz retunes, dial 45k above the new centre", receiver_hw_target(14236000UL, RX_MODE_USB, hw) == 14191000UL);
    ck("USB: 9 kHz above retunes", receiver_hw_target(14164000UL, RX_MODE_USB, hw) == 14119000UL);
    ck("USB: 10 kHz above stays", receiver_hw_target(14165000UL, RX_MODE_USB, hw) == hw);
    ck("switching from USB to AM at the same dial retunes to the dial", receiver_hw_target(14200000UL, RX_MODE_AM, hw) == 14200000UL);
    ck("AM to USB retunes (dial - 45k)", receiver_hw_target(14200000UL, RX_MODE_USB, 14200000UL) == 14155000UL);
    ck("USB to LSB: no retune", receiver_hw_target(14200000UL, RX_MODE_LSB, hw) == hw);
    ck("lowest dial 100 kHz -> hw 55 kHz", receiver_hw_target(100000UL, RX_MODE_CW, 0UL) == 55000UL);
    ck("clamp low", receiver_clamp_freq(5000UL) == 100000UL);
    receiver_fmt_hz(b, sizeof(b), 400); ck("fmt 400 -> 400Hz", strcmp(b, "400Hz") == 0);
    receiver_fmt_hz(b, sizeof(b), 2400); ck("fmt 2400 -> 2.4k", strcmp(b, "2.4k") == 0);
    receiver_fmt_hz(b, sizeof(b), 12500); ck("fmt 12500 -> 12.5k", strcmp(b, "12.5k") == 0);
    receiver_fmt_hz(b, sizeof(b), 10); ck("fmt 10 -> 10Hz", strcmp(b, "10Hz") == 0);
    receiver_fmt_hz(b, sizeof(b), 150000); ck("fmt 150000 -> 150k", strcmp(b, "150k") == 0);
    ck("mode names", !strcmp(receiver_mode_name(3), "USB") && !strcmp(receiver_mode_name(4), "LSB") && !strcmp(receiver_mode_name(5), "CW"));
    printf("%s\n", fails ? "FAILED" : "all passed");
    return fails != 0;
}
