/* RTLSDR.c -- command-line diagnostic tool: runs milestones 1-4 in
   sequence (enumerate & identify, control-transfer proof, baseband+tuner
   init, bulk streaming throughput) and reports pass/fail for each. See
   ../../../docs/PLAN.md for the full project plan and history.

   Device discovery and bringup (find_device, baseband/tuner init, sample
   rate, raw OS_Find/OS_GBPB stream I/O) live in Driver.c/Driver.h --
   shared with !RTLSDRView, the live spectrum display. This file keeps
   only main() and the milestone-specific diagnostic functions.

   C89 only (Norcroft): all declarations at top of block, no // comments. */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "kernel.h"
#include "RTLSDR.h"
#include "R82XX.h"
#include "Driver.h"

static int milestone3_baseband_and_tuner(const char *dev)
{
    int tuner;

    printf("\n--- milestone 3: baseband init + tuner probe ---\n");
    rtlsdr_init_baseband(dev);
    printf("Baseband init sequence sent (USB config, demod reset, FIR "
           "coefficients, SDR/zero-IF mode).\n");

    tuner = rtlsdr_probe_tuner(dev);
    if (tuner == 2) {
        printf("Tuner probe: R828D responded at I2C 0x%02X with check "
               "value 0x%02X -- matches this hardware (RTL-SDR Blog V4).\n",
               R828D_I2C_ADDR, R82XX_CHECK_VAL);
        printf("\nMilestone 3 gate: PASS.\n");
        return 0;
    }
    if (tuner == 1) {
        printf("Tuner probe: found R820T, not R828D -- unexpected for "
               "this dongle (docs/PLAN.md assumed R828D from *USBDevInfo). "
               "Milestone 3 gate: FAIL (wrong tuner detected).\n");
        return 1;
    }
    printf("Tuner probe: neither R820T nor R828D responded.\n");
    printf("\nMilestone 3 gate: FAIL -- tuner not found. Check the "
           "baseband init sequence and I2C repeater enable/disable "
           "against docs/PLAN.md.\n");
    return 1;
}

/* ---- milestone 3.5: full R82XX tuner init + tune to a real frequency ----
   Unlike milestone3_baseband_and_tuner()'s self-contained enable-probe-
   disable cycle, the I2C repeater needs to stay enabled across all of
   r82xx_init()+r82xx_set_freq() -- matches real driver behaviour, which
   enables it once early on and leaves it enabled through normal tuner
   use, not just for a one-off presence check. */
static int milestone_tune_test(const char *dev)
{
    r82xx_t tuner;
    int rc;
    unsigned long test_freq_hz;

    printf("\n--- tuner init + tune-to-100MHz test ---\n");
    test_freq_hz = 100000000UL; /* 100MHz, VHF FM broadcast band */

    rtlsdr_demod_write_reg(dev, 1, 0x01, 0x18, 1); /* enable I2C repeater */

    rc = rtlsdr_tuner_postinit(dev);
    if (rc < 0) {
        printf("rtlsdr_tuner_postinit() FAILED (rc=%d)\n", rc);
        rtlsdr_demod_write_reg(dev, 1, 0x01, 0x10, 1);
        return 1;
    }
    printf("Tuner post-init OK (real-IF mode selected, IF freq set, "
           "spectrum inversion enabled).\n");

    rc = r82xx_init(&tuner, dev);
    if (rc < 0) {
        printf("r82xx_init() FAILED (rc=%d)\n", rc);
        rtlsdr_demod_write_reg(dev, 1, 0x01, 0x10, 1);
        return 1;
    }
    printf("r82xx_init() OK (init array written, filter calibration "
           "code=0x%02X).\n", tuner.fil_cal_code);

    rc = r82xx_set_freq(&tuner, test_freq_hz);
    rtlsdr_demod_write_reg(dev, 1, 0x01, 0x10, 1); /* disable I2C repeater */

    if (rc < 0) {
        printf("r82xx_set_freq(100MHz) FAILED (rc=%d)\n", rc);
        printf("\nTune test: FAIL.\n");
        return 1;
    }
    if (!tuner.has_lock) {
        printf("r82xx_set_freq(100MHz) completed but PLL did not lock.\n");
        printf("\nTune test: FAIL (no PLL lock).\n");
        return 1;
    }

    printf("Tuned to 100MHz: PLL locked.\n");
    printf("\nTune test: PASS -- the tuner can be tuned to a real "
           "frequency. Next: milestone 4 (bulk streaming).\n");
    return 0;
}

/* ---- milestone 4: bulk streaming throughput proof ---- */

#define STREAM_CHUNK_BYTES    RTLSDR_DEFAULT_BUF_LEN /* buffer capacity only */
#define STREAM_TARGET_BPS     4800000UL /* 2.4 MSPS * 2 bytes, ~expected --
                                            not measurable yet, see below */

static int milestone4_streaming_test(int device_num)
{
    char dev[16];
    char path[64];
    unsigned char *buf;
    int handle;
    clock_t now, elapsed_ticks;
    unsigned long total_bytes;
    int got;

    printf("\n--- milestone 4: bulk streaming throughput ---\n");

    sprintf(dev, "usb%d", device_num);
    rtlsdr_reset_buffer(dev);

    /* endpoint1: confirmed bulk IN, interface0: confirmed the bulk
       interface, bulk: explicit even though it's the default for this
       endpoint, usbtimeout2000: bound each read to 2s so a stalled
       endpoint shows up as a short/failed read instead of hanging this
       test forever -- see docs/PLAN.md, this is exactly the risk
       milestone 4 exists to check for. */
    sprintf(path, "devices#endpoint%d;interface%d;bulk;usbtimeout2000:%s",
            RTLSDR_BULK_ENDPOINT, RTLSDR_BULK_INTERFACE, dev);
    printf("Opening %s ...\n", path);

    handle = os_find_open(path);
    if (handle == 0) {
        printf("OS_Find FAILED -- could not open the bulk endpoint "
               "stream.\n");
        printf("\nMilestone 4 gate: FAIL.\n");
        return 1;
    }

    buf = (unsigned char *)malloc(STREAM_CHUNK_BYTES);
    if (buf == NULL) {
        printf("malloc(%d) FAILED\n", STREAM_CHUNK_BYTES);
        os_find_close(handle);
        return 1;
    }

    /* Previous run found the exact boundary: 64/128/256/512/1024 bytes
       all fully reliable (5/5 instant attempts each), 2048 blocked ~110s
       before an "Escape" error. Not a USB-max-packet-size story (512 AND
       1024 both worked) -- a real, sharp boundary somewhere in
       (1024,2048), most plausibly a DeviceFS/driver internal buffer
       limit. Re-testing 2048 here would just risk another 100s+ Escape
       recovery for no new information, so the sweep now stops at 1024 --
       already-known-bad sizes aren't repeated. */
    total_bytes = 0;
    {
        static const int sizes[] = { 64, 128, 256, 512, 1024 };
        int size_idx;
        int size_ok;
        int attempt;
        int best_working_size;

        best_working_size = 0;
        for (size_idx = 0; size_idx < 5; size_idx++) {
            printf("--- size %d bytes ---\n", sizes[size_idx]);
            fflush(stdout);
            size_ok = 1;
            for (attempt = 0; attempt < 5; attempt++) {
                now = clock();
                got = os_gbpb_read4(handle, buf, sizes[size_idx]);
                elapsed_ticks = clock() - now;
                if (got < 0) {
                    printf("  attempt %d: ERROR after %.1fs\n", attempt + 1,
                           (double)elapsed_ticks / CLOCKS_PER_SEC);
                    size_ok = 0;
                    break;
                }
                printf("  attempt %d: %d bytes (%.2fs)\n", attempt + 1, got,
                       (double)elapsed_ticks / CLOCKS_PER_SEC);
                fflush(stdout);
                if ((double)elapsed_ticks / CLOCKS_PER_SEC > 1.0) {
                    printf("  (slow -- treating as unreliable at this "
                           "size)\n");
                    size_ok = 0;
                    break;
                }
                total_bytes += (unsigned long)got;
            }
            if (!size_ok) {
                printf("Size %d bytes: NOT reliable.\n", sizes[size_idx]);
                break;
            }
            best_working_size = sizes[size_idx];
            printf("Size %d bytes: all 5 attempts OK.\n", sizes[size_idx]);
        }

        /* Real sustained throughput measurement using the largest
           confirmed-reliable size, over an actual multi-second window --
           the number milestone 4 has been after all along. */
        if (best_working_size > 0) {
            clock_t tstart, tnow, ticks;
            int treads, tgot;
            unsigned long tbytes;
            double tsec, tbps;

            printf("\n--- sustained read at %d bytes/call, up to 5s ---\n",
                   best_working_size);
            fflush(stdout);
            tbytes = 0;
            treads = 0;
            tstart = clock();
            ticks = 0;
            do {
                tgot = os_gbpb_read4(handle, buf, best_working_size);
                treads++;
                tnow = clock();
                ticks = tnow - tstart;
                if (tgot < 0) {
                    printf("  ERROR on read %d after %.1fs\n", treads,
                           (double)ticks / CLOCKS_PER_SEC);
                    break;
                }
                tbytes += (unsigned long)tgot;
                if (treads % 500 == 0) {
                    printf("  ...%d reads, %lu bytes, %.1fs\n", treads,
                           tbytes, (double)ticks / CLOCKS_PER_SEC);
                    fflush(stdout);
                }
            } while (((double)ticks / CLOCKS_PER_SEC) < 5.0);

            tsec = (double)ticks / CLOCKS_PER_SEC;
            tbps = (tsec > 0) ? (tbytes / tsec) : 0;
            printf("\n%lu bytes in %.1fs = %.0f bytes/sec (%d reads)\n",
                   tbytes, tsec, tbps, treads);
            printf("Target (2.4 MSPS x 2 bytes): ~%lu bytes/sec\n",
                   STREAM_TARGET_BPS);
            if (tbps >= (double)STREAM_TARGET_BPS * 0.8) {
                printf("\nMilestone 4 gate: PASS.\n");
            } else {
                printf("\nMilestone 4 gate: FAIL -- throughput below "
                       "target (largest reliable single-call size is "
                       "only %d bytes; may need a fix to read larger "
                       "chunks, not just more small ones, to hit "
                       "target).\n", best_working_size);
            }
        }
    }

    os_find_close(handle);
    free(buf);

    printf("\n(%lu bytes moved during the size sweep itself, separate "
           "from the sustained-read measurement above.)\n", total_bytes);
    return 0;
}

/* ---- milestone 2: prove the control-transfer path works ----
   librtlsdr's EEPROM layout (magic byte, vendor_id/product_id field
   offsets) lives in its separate rtl_eeprom tool, not librtlsdr.c itself,
   and hasn't been pulled in yet -- rather than guess offsets, this reads
   the first 12 raw EEPROM bytes and prints them in hex for eyeballing
   against `rtl_eeprom -d 0` on a Linux box with the same dongle, if one's
   available (that's the plan's original milestone 2 gate). Independent of
   that: it also re-reads the first register twice and checks the two
   reads agree, which alone proves the control-transfer path itself is
   working and stable, without needing to know what the bytes mean. */
static int milestone2_register_probe(const char *dev)
{
    int i;
    int addr;
    int val;
    int val2;
    int all_ok;

    printf("\n--- milestone 2: EEPROM register read-back ---\n");
    all_ok = 1;
    for (i = 0; i < 6; i++) {
        addr = i * 2;
        val = rtlsdr_read_reg(dev, RTLSDR_BLOCK_ROM, addr, 2);
        if (val < 0) {
            printf("EEPROM[0x%02X]: read FAILED\n", addr);
            all_ok = 0;
            continue;
        }
        printf("EEPROM[0x%02X..0x%02X]: %04X\n", addr, addr + 1, val);
    }

    printf("\nConsistency check: reading EEPROM[0x00] twice...\n");
    val = rtlsdr_read_reg(dev, RTLSDR_BLOCK_ROM, 0, 2);
    val2 = rtlsdr_read_reg(dev, RTLSDR_BLOCK_ROM, 0, 2);
    if (val < 0 || val2 < 0) {
        printf("  FAILED (a read errored)\n");
        all_ok = 0;
    } else if (val != val2) {
        printf("  FAILED: %04X then %04X -- control transfers are not "
               "returning consistent data\n", val, val2);
        all_ok = 0;
    } else {
        printf("  PASS: both reads returned %04X\n", val);
    }

    if (all_ok) {
        printf("\nMilestone 2 gate: PASS (control-transfer path is "
               "working and stable).\n");
        printf("If you have Linux + the same dongle, compare the hex "
               "dump above against `rtl_eeprom -d 0` for full "
               "independent confirmation.\n");
        return 0;
    }
    printf("\nMilestone 2 gate: FAIL -- see errors above.\n");
    return 1;
}

/* ---- entry point ---- */

int main(void)
{
    int n;
    char device_name[16];

    printf("RTL-SDR RISC OS driver -- milestone 1 (enumerate & identify)\n");
    printf("Scanning *USBDevices output for a matching description...\n");

    n = find_device();
    if (n < 0) {
        printf("No matching device found.\n");
        printf("Run tools/probe.bas first to confirm the dongle is "
               "enumerated and check its *USBDevices description text.\n");
        return 1;
    }

    sprintf(device_name, "usb%d", n);
    printf("Found dongle: device number %d (DeviceFS name \"%s\")\n",
           n, device_name);
    printf("Milestone 1 gate: PASS (device identified).\n");

    if (milestone2_register_probe(device_name) != 0) {
        return 1;
    }

    if (milestone3_baseband_and_tuner(device_name) != 0) {
        return 1;
    }

    if (milestone_tune_test(device_name) != 0) {
        return 1;
    }

    printf("\nSetting sample rate to 2.4 MSPS...\n");
    if (rtlsdr_set_sample_rate(device_name, 2400000UL) != 0) {
        printf("rtlsdr_set_sample_rate() FAILED.\n");
        return 1;
    }
    printf("Sample rate set. This was never called at all before this "
           "build -- the RTL2832U's resampler was previously left at its "
           "power-up-default ratio, which is a real candidate for why no "
           "streaming data ever arrived.\n");

    return milestone4_streaming_test(n);
}
