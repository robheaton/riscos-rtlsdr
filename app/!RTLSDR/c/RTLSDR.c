/* RTLSDR.c -- command-line diagnostic tool: runs milestones 1-4 in
   sequence (enumerate & identify, control-transfer proof, baseband+tuner
   init, bulk streaming throughput) and reports pass/fail for each, then
   milestone 5 (diagnostic, non-gating): verifies whether OS_GBPB reason
   4 actually writes the bytes it claims to transfer, added after phase
   2 found a "successful" 512-byte read that only touched ~2 real bytes.
   See ../../../docs/PLAN.md for the full project plan and history.

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

/* ---- milestone 5 (diagnostic): does OS_GBPB reason 4 actually WRITE
   the bytes it claims to transfer? ----

   RTLSDRView (phase 2) found a 12-byte hex dump of a "successful" 512-
   byte read (got==512, i.e. len-r3==0, every single time -- never a
   short read) showing only the first ~2 bytes looking like real data,
   with the rest exactly 0x00. That's ambiguous on its own: real data
   COULD legitimately be 0x00 sometimes. This test removes that
   ambiguity by pre-filling the destination buffer with a sentinel byte
   (0xAA, never a value 0x00-centred ADC noise would produce as a run)
   before each read, then counting how many bytes actually changed --
   directly measuring how much of the buffer the SWI genuinely writes,
   independent of what value ends up there. Also checks whether content
   changes across consecutive reads at a fixed size (genuine streaming)
   or stays frozen (a stuck/cached single packet being re-read). See
   docs/PLAN.md -- this also bears on whether milestone 4's own 25.6
   MB/s throughput number ever represented real sample content, since it
   only ever checked byte counts. */

#define M5_SENTINEL 0xAAu

/* os_args_set_nonblocking() (the real fix, confirmed on real hardware
   below) now lives in Driver.c/Driver.h, shared with !RTLSDRView --
   see the comment there for how it was found and what it does. */

static void m5_dump_hex(const unsigned char *buf, int n)
{
    int i;
    for (i = 0; i < n; i++) {
        printf("%02X ", buf[i]);
    }
}

static void milestone5_read_primitive_diagnostic(int device_num)
{
    static const int sizes[] = { 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024 };
    char dev[16];
    char path[64];
    unsigned char buf[1024];
    int handle;
    int s, i, got, touched, dump_n;

    printf("\n--- milestone 5 (diagnostic): raw OS_GBPB content "
           "verification ---\n");
    sprintf(dev, "usb%d", device_num);
    rtlsdr_reset_buffer(dev);
    sprintf(path, "devices#endpoint%d;interface%d;bulk;usbtimeout2000:%s",
            RTLSDR_BULK_ENDPOINT, RTLSDR_BULK_INTERFACE, dev);
    handle = os_find_open(path);
    if (handle == 0) {
        printf("OS_Find FAILED -- aborting milestone 5 diagnostic.\n");
        return;
    }

    printf("\n-- sentinel test: pre-fill with 0x%02X, read, count bytes "
           "actually changed vs. what os_gbpb_read4() reports as "
           "transferred --\n", M5_SENTINEL);
    for (s = 0; s < 10; s++) {
        memset(buf, M5_SENTINEL, sizeof(buf));
        got = os_gbpb_read4(handle, buf, sizes[s]);
        touched = 0;
        for (i = 0; i < sizes[s]; i++) {
            if (buf[i] != M5_SENTINEL) {
                touched++;
            }
        }
        dump_n = (sizes[s] < 16) ? sizes[s] : 16;
        printf("size=%4d got=%4d touched=%4d first%2d: ",
               sizes[s], got, touched, dump_n);
        m5_dump_hex(buf, dump_n);
        printf("\n");
        fflush(stdout);
    }

    printf("\n-- repeatability test: 8-byte reads, 10 in a row (checking "
           "for genuinely fresh content each time vs. a frozen/cached "
           "packet) --\n");
    for (i = 0; i < 10; i++) {
        memset(buf, M5_SENTINEL, 8);
        got = os_gbpb_read4(handle, buf, 8);
        printf("read %2d: got=%d bytes: ", i, got);
        m5_dump_hex(buf, 8);
        printf("\n");
        fflush(stdout);
    }

    printf("\n-- non-blocking mode test: enable via OS_Args 9 (IOCtl "
           "group 0xFF reason 1), repeat the sentinel test -- if this "
           "fix works, \"got\" should now honestly report a small "
           "number instead of always claiming the full size --\n");
    if (os_args_set_nonblocking(handle, 1) != 0) {
        printf("Could not enable non-blocking mode -- skipping this "
               "part of the diagnostic.\n");
    } else {
        printf("Non-blocking mode enabled OK.\n");
        for (s = 0; s < 10; s++) {
            memset(buf, M5_SENTINEL, sizeof(buf));
            got = os_gbpb_read4(handle, buf, sizes[s]);
            touched = 0;
            for (i = 0; i < sizes[s]; i++) {
                if (buf[i] != M5_SENTINEL) {
                    touched++;
                }
            }
            dump_n = (sizes[s] < 16) ? sizes[s] : 16;
            printf("size=%4d got=%4d touched=%4d first%2d: ",
                   sizes[s], got, touched, dump_n);
            m5_dump_hex(buf, dump_n);
            printf("\n");
            fflush(stdout);
        }
        os_args_set_nonblocking(handle, 0);
    }

    os_find_close(handle);
    printf("\n--- milestone 5 diagnostic done ---\n");
}

/* ---- milestone 6 (diagnostic): honest sustained throughput, no Wimp/
   GUI involved at all ----

   !RTLSDRView's live spectrum/audio app measures real (non-blocking,
   nopad, honest-short-read) throughput stuck around 7-23k samples/sec
   (~14-46 KB/s) no matter how FIXED_CHUNK_SIZE, MAX_FRAMES_PER_TICK,
   or the read-retry logic are tuned -- roughly 0.5% of the 2.4 MSPS
   target. Every one of those tuning attempts still runs inside its
   Wimp idle-tick loop (Event_Claim(event_NULL, ...), one bounded batch
   of work per Wimp_Poll return), so it's never been possible to tell
   whether that ceiling is a real USB/driver/hardware limit or an
   artifact of how infrequently/cheaply that idle handler gets to run.
   This test removes the GUI entirely: open the same nopad + non-
   blocking stream !RTLSDRView uses, then read in a tight loop bounded
   ONLY by wall-clock time (no per-call attempt cap, no round trips
   through any event loop) and report the real, honest bytes/sec.
   Also worth noting: milestone 4's own 25.6 MB/s figure (this same
   file, above) was later shown by milestone 5's sentinel test to be
   fake -- blocking-mode reads claiming got==512 every time while only
   the first ~2 bytes were genuinely written. So there has never
   actually been a trustworthy measurement of this driver's real
   sustained bulk-read rate until this one. */

#define M6_TEST_SECONDS 3

static void milestone6_honest_throughput(int device_num)
{
    static const int sizes[] = { 1024, 2048, 4096, 8192 };
    char dev[16];
    char path[80];
    unsigned char *buf;
    unsigned char last_good[8];
    int handle;
    int s;
    int got;
    unsigned long total_bytes;
    unsigned long total_reads;
    unsigned long empty_polls;
    clock_t tstart, tnow;
    double tsec;
    double bps;

    printf("\n--- milestone 6 (diagnostic): honest sustained throughput, "
           "no Wimp/GUI involved ---\n");

    sprintf(dev, "usb%d", device_num);

    for (s = 0; s < 4; s++) {
        buf = (unsigned char *)malloc((size_t)sizes[s]);
        if (buf == NULL) {
            printf("size=%d: malloc FAILED, skipping\n", sizes[s]);
            continue;
        }

        rtlsdr_reset_buffer(dev);
        /* First test at size131072 (128KB DeviceFS stream buffer, see
           the size/N doc comment this replaced) found real gains from
           a bigger buffer -- best result was 4096-byte chunks (21.6
           -> 63 KB/s), notably better than either the smallest (512)
           or largest (16384/65536) sizes tested. Pushing the buffer
           bigger still (512KB) and narrowing the chunk-size sweep
           around that 4096 sweet spot (1024/2048/4096/8192) to find
           the real peak instead of the coarse 4x jumps tried so far. */
        sprintf(path, "devices#endpoint%d;interface%d;bulk;usbtimeout2000;"
                      "nopad;size524288:%s",
                RTLSDR_BULK_ENDPOINT, RTLSDR_BULK_INTERFACE, dev);
        handle = os_find_open(path);
        if (handle == 0) {
            printf("size=%d: OS_Find FAILED, skipping\n", sizes[s]);
            free(buf);
            continue;
        }
        if (os_args_set_nonblocking(handle, 1) != 0) {
            printf("size=%d: could not enable non-blocking, skipping\n",
                   sizes[s]);
            os_find_close(handle);
            free(buf);
            continue;
        }

        printf("\n-- size=%d bytes, %d second(s) --\n", sizes[s],
               M6_TEST_SECONDS);
        fflush(stdout);

        total_bytes = 0;
        total_reads = 0;
        empty_polls = 0;
        memset(last_good, 0, sizeof(last_good));
        tstart = clock();
        for (;;) {
            got = os_gbpb_read4(handle, buf, sizes[s]);
            if (got < 0) {
                printf("  ERROR on read after %lu reads\n", total_reads);
                break;
            }
            if (got == 0) {
                empty_polls++;
            } else {
                total_bytes += (unsigned long)got;
                total_reads++;
                memcpy(last_good, buf, sizeof(last_good));
            }
            tnow = clock();
            if (((double)(tnow - tstart) / CLOCKS_PER_SEC) >=
                (double)M6_TEST_SECONDS) {
                break;
            }
        }
        tsec = (double)(clock() - tstart) / CLOCKS_PER_SEC;
        bps = (tsec > 0.0) ? ((double)total_bytes / tsec) : 0.0;

        printf("  %lu bytes in %.1fs = %.0f bytes/sec (%.1f%% of "
               "target)\n", total_bytes, tsec, bps,
               bps / (double)STREAM_TARGET_BPS * 100.0);
        printf("  %lu successful reads, %lu empty (busy) polls, "
               "avg %.0f bytes/successful read\n", total_reads,
               empty_polls, (total_reads > 0) ?
               ((double)total_bytes / (double)total_reads) : 0.0);
        printf("  first bytes of last successful read: %02X %02X %02X "
               "%02X %02X %02X %02X %02X\n", last_good[0], last_good[1],
               last_good[2], last_good[3], last_good[4], last_good[5],
               last_good[6], last_good[7]);
        fflush(stdout);

        os_args_set_nonblocking(handle, 0);
        os_find_close(handle);
        free(buf);
    }

    printf("\n--- milestone 6 diagnostic done ---\n");
}

/* ---- milestone 7 (diagnostic): DeviceCall_USB_TransferInfo ----

   Milestone 6 found "avg bytes/successful read" pinned at almost
   exactly 2, rock solid across every chunk size (512-65536) and every
   DeviceFS `size/N` stream-buffer size tried (default/128KB/512KB),
   while total bytes/sec bounced noisily between runs (e.g. 4096-byte
   chunks: 63 KB/s with a 128KB buffer, 33 KB/s with a 512KB buffer,
   same code, different run) -- the signature of polling far faster
   than real data arrives, catching a small dribble each time, rather
   than a chunk-size-dependent effect. That means total bytes/sec IS
   already close to the true achieved rate, and no read-loop tuning
   parameter tried so far has been able to move it.

   This uses a DIFFERENT diagnostic API, never tried before now: the
   USB doc's "Transfer Info" DeviceFS_CallDevice extension (added in
   USBDriver 0.49), which reports on the ACTUAL underlying USB
   transfer DeviceFS has open right now -- bytes received so far vs.
   total requested, live status (in progress/complete/error), and how
   many bytes were synthetic padding. It needs a USB stream handle
   first, from "Return Handles 2" (reason 7, given the fileswitch
   stream handle from os_find_open). Reading this immediately after a
   read() call should show directly whether DeviceFS is really running
   ONE big transfer per read() (in which case bytes-received should
   climb toward the full requested size over repeated polls) or
   something else -- information the r/got numbers alone can't give
   us. */

#define SWI_DeviceFS_CallDevice 0x42744
#define DEVCALL_RETURN_HANDLES_2 ((int)0x80000007u)
#define DEVCALL_TRANSFER_INFO    ((int)0x80000006u)

static void milestone7_transfer_info_diagnostic(int device_num)
{
    _kernel_swi_regs regs;
    _kernel_oserror *err;
    char dev[16];
    char path[80];
    unsigned char buf[4096];
    int handle;
    int usb_stream_handle;
    int i, got;

    printf("\n--- milestone 7 (diagnostic): DeviceCall_USB_TransferInfo "
           "---\n");

    sprintf(dev, "usb%d", device_num);
    rtlsdr_reset_buffer(dev);
    sprintf(path, "devices#endpoint%d;interface%d;bulk;usbtimeout2000;"
                  "nopad;size131072:%s",
            RTLSDR_BULK_ENDPOINT, RTLSDR_BULK_INTERFACE, dev);
    handle = os_find_open(path);
    if (handle == 0) {
        printf("OS_Find FAILED -- aborting milestone 7.\n");
        return;
    }
    if (os_args_set_nonblocking(handle, 1) != 0) {
        printf("could not enable non-blocking -- aborting milestone 7.\n");
        os_find_close(handle);
        return;
    }

    regs.r[0] = DEVCALL_RETURN_HANDLES_2;
    regs.r[1] = (int)dev;
    regs.r[2] = handle;
    err = _kernel_swi(SWI_DeviceFS_CallDevice, &regs, &regs);
    if (err != NULL) {
        printf("Return Handles 2 FAILED: %s\n", err->errmess);
        os_args_set_nonblocking(handle, 0);
        os_find_close(handle);
        return;
    }
    usb_stream_handle = regs.r[5];
    printf("USB stream handle = 0x%08X\n", (unsigned int)usb_stream_handle);

    printf("\n-- 4096-byte reads, checking TransferInfo right after each "
           "--\n");
    for (i = 0; i < 15; i++) {
        got = os_gbpb_read4(handle, buf, sizeof(buf));

        regs.r[0] = DEVCALL_TRANSFER_INFO;
        regs.r[1] = (int)dev;
        regs.r[2] = usb_stream_handle;
        err = _kernel_swi(SWI_DeviceFS_CallDevice, &regs, &regs);
        if (err != NULL) {
            printf("read %2d: got=%4d  TransferInfo FAILED: %s\n", i, got,
                   err->errmess);
            continue;
        }
        printf("read %2d: got=%4d  xfer: received=%d requested=%d "
               "status=%d padded=%d\n", i, got, regs.r[0], regs.r[1],
               regs.r[3], regs.r[4]);
        fflush(stdout);
    }

    os_args_set_nonblocking(handle, 0);
    os_find_close(handle);
    printf("\n--- milestone 7 diagnostic done ---\n");
}

/* ---- milestone 8 (diagnostic): multiple independent streams on the
   SAME bulk endpoint ----

   USBDriver's start_read() (real source, gitlab.riscosopen.org) keeps
   only one transfer in flight AT A TIME -- but that busy flag lives on
   the per-stream state (the 'str' struct for one DeviceFS OPENIN
   handle), not on the endpoint itself. Every measurement so far (this
   file's milestones 6-7) used exactly one stream, so it could never
   have more than one real transfer in flight regardless of chunk
   size, DeviceFS buffer size, or retry strategy -- matching the flat
   ~15-65 KB/s ceiling seen no matter what got tuned.

   This is untested: does DeviceFS/USBDriver allow the SAME bulk
   endpoint to be opened as several INDEPENDENT streams at once, each
   with its OWN busy flag? If so, round-robin reading across N of them
   should let N real transfers be in flight simultaneously -- genuine
   pipelining, at the application level, needing no driver change --
   and aggregate throughput should scale with N (until some other
   limit appears). If the endpoint can only be opened once, or
   DeviceFS serialises across streams to the same endpoint under the
   hood, aggregate throughput should stay flat -- which would mean the
   one-transfer-in-flight limit is real and not fixable from here. */

#define M8_NSTREAMS 4

static void milestone8_multistream_throughput(int device_num)
{
    char dev[16];
    char path[80];
    unsigned char *bufs[M8_NSTREAMS];
    int handles[M8_NSTREAMS];
    int opened;
    int i, got;
    unsigned long total_bytes;
    unsigned long total_reads;
    unsigned long empty_polls;
    clock_t tstart, tnow;
    double tsec;
    double bps;

    printf("\n--- milestone 8 (diagnostic): %d independent streams on "
           "the same bulk endpoint ---\n", M8_NSTREAMS);

    sprintf(dev, "usb%d", device_num);
    rtlsdr_reset_buffer(dev);
    sprintf(path, "devices#endpoint%d;interface%d;bulk;usbtimeout2000;"
                  "nopad:%s",
            RTLSDR_BULK_ENDPOINT, RTLSDR_BULK_INTERFACE, dev);

    opened = 0;
    for (i = 0; i < M8_NSTREAMS; i++) {
        bufs[i] = (unsigned char *)malloc(4096);
        if (bufs[i] == NULL) {
            printf("stream %d: malloc FAILED\n", i);
            break;
        }
        handles[i] = os_find_open(path);
        if (handles[i] == 0) {
            printf("stream %d: OS_Find FAILED (endpoint may only allow "
                   "one open stream)\n", i);
            free(bufs[i]);
            break;
        }
        if (os_args_set_nonblocking(handles[i], 1) != 0) {
            printf("stream %d: could not enable non-blocking\n", i);
            os_find_close(handles[i]);
            free(bufs[i]);
            break;
        }
        opened++;
        printf("stream %d: opened OK (handle 0x%08X)\n", i,
               (unsigned int)handles[i]);
    }

    if (opened == 0) {
        printf("\n--- milestone 8 diagnostic done (no streams opened) "
               "---\n");
        return;
    }

    printf("\n-- round-robin 4096-byte reads across %d stream(s), %d "
           "second(s) --\n", opened, M6_TEST_SECONDS);
    fflush(stdout);

    total_bytes = 0;
    total_reads = 0;
    empty_polls = 0;
    tstart = clock();
    for (;;) {
        for (i = 0; i < opened; i++) {
            got = os_gbpb_read4(handles[i], bufs[i], 4096);
            if (got < 0) {
                continue;
            }
            if (got == 0) {
                empty_polls++;
            } else {
                total_bytes += (unsigned long)got;
                total_reads++;
            }
        }
        tnow = clock();
        if (((double)(tnow - tstart) / CLOCKS_PER_SEC) >=
            (double)M6_TEST_SECONDS) {
            break;
        }
    }
    tsec = (double)(clock() - tstart) / CLOCKS_PER_SEC;
    bps = (tsec > 0.0) ? ((double)total_bytes / tsec) : 0.0;

    printf("  %lu bytes in %.1fs = %.0f bytes/sec (%.1f%% of target) "
           "across %d stream(s)\n", total_bytes, tsec, bps,
           bps / (double)STREAM_TARGET_BPS * 100.0, opened);
    printf("  %lu successful reads, %lu empty (busy) polls, avg %.0f "
           "bytes/successful read\n", total_reads, empty_polls,
           (total_reads > 0) ?
           ((double)total_bytes / (double)total_reads) : 0.0);
    fflush(stdout);

    for (i = 0; i < opened; i++) {
        os_args_set_nonblocking(handles[i], 0);
        os_find_close(handles[i]);
        free(bufs[i]);
    }

    printf("\n--- milestone 8 diagnostic done ---\n");
}

/* ---- milestone 9 (diagnostic): does the sample-rate register write
   actually hold? ----

   Every real, sustained measurement so far (milestones 6-8) implies an
   actual captured rate of roughly 10-30K IQ samples/sec -- 2-3 orders
   of magnitude below the 2.4 MSPS rtlsdr_set_sample_rate(2400000) is
   supposed to configure. All the obvious software explanations (chunk
   size, DeviceFS buffer size, retry strategy, GUI overhead, one-
   transfer-in-flight, wrong USB controller) have been tested and ruled
   out or found insufficient. rtlsdr_set_sample_rate()'s register math
   was checked against upstream and matches exactly, but that only
   proves the WRITE is correct -- never confirmed the chip actually
   LATCHED it. This reads registers 0x9f/0xa1 (page 1) straight back
   after rtlsdr_set_sample_rate(dev, 2400000) and compares against the
   resample ratio that rate should produce, direct and cheap, before
   concluding the ceiling is unfixable from application code. */

static void milestone9_sample_rate_readback(const char *dev)
{
    unsigned long expect_ratio;
    int hi, lo;
    unsigned long actual_ratio;

    printf("\n--- milestone 9 (diagnostic): sample-rate register "
           "readback ---\n");

    printf("Re-issuing rtlsdr_set_sample_rate(dev, 2400000)...\n");
    rtlsdr_set_sample_rate(dev, 2400000UL);

    expect_ratio = (unsigned long)((28800000.0 * 4194304.0) / 2400000.0);
    expect_ratio &= 0x0ffffffcUL;

    hi = rtlsdr_demod_read_reg(dev, 1, 0x9f, 2);
    lo = rtlsdr_demod_read_reg(dev, 1, 0xa1, 2);

    if (hi < 0 || lo < 0) {
        printf("readback FAILED (hi=%d lo=%d)\n", hi, lo);
        printf("\n--- milestone 9 diagnostic done ---\n");
        return;
    }

    actual_ratio = (((unsigned long)hi & 0xffffUL) << 16) |
                   ((unsigned long)lo & 0xffffUL);

    printf("expected rsamp_ratio for 2.4 MSPS: 0x%08lX\n", expect_ratio);
    printf("register 0x9f (hi 16 bits) = 0x%04X\n", (unsigned int)hi);
    printf("register 0xa1 (lo 16 bits) = 0x%04X\n", (unsigned int)lo);
    printf("readback rsamp_ratio        : 0x%08lX\n", actual_ratio);

    if (actual_ratio == expect_ratio) {
        printf("MATCH -- the register write held. Sample-rate config "
               "is not the cause of the low captured rate.\n");
    } else {
        printf("MISMATCH -- the register did NOT hold the expected "
               "value. This would explain the whole throughput "
               "ceiling: the chip may be running at a much lower real "
               "rate than we think we configured.\n");
    }

    printf("\n--- milestone 9 diagnostic done ---\n");
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

    milestone4_streaming_test(n);

    /* Diagnostic, not gating -- runs regardless of milestone 4's own
       pass/fail so we get the content-verification data either way. */
    milestone5_read_primitive_diagnostic(n);

    /* Diagnostic, not gating -- isolates whether !RTLSDRView's real-
       throughput ceiling is a USB/driver limit or a Wimp idle-tick
       artifact. See the comment above milestone6_honest_throughput(). */
    milestone6_honest_throughput(n);

    /* Diagnostic, not gating -- a different API (Transfer Info) to see
       what DeviceFS's own view of the in-flight transfer looks like,
       since milestone 6 alone couldn't distinguish "genuinely tiny
       transfers" from "polling faster than one big transfer fills". */
    milestone7_transfer_info_diagnostic(n);

    /* Diagnostic, not gating -- tests whether the one-transfer-in-
       flight limit is per-stream (fixable by opening several streams
       from our own app) or a deeper, unfixable-from-here constraint.
       See the comment above milestone8_multistream_throughput(). */
    milestone8_multistream_throughput(n);

    /* Diagnostic, not gating -- sanity-checks that the sample-rate
       register write actually held, since every real measurement so
       far implies a captured rate far below what 2.4 MSPS should give
       us. See the comment above milestone9_sample_rate_readback(). */
    milestone9_sample_rate_readback(device_name);

    return 0;
}
