# Brief: RISC OS USB bulk-IN throughput investigation

## Context

I've been building a native RISC OS driver/app for RTL-SDR USB dongles
(repo: `riscos-rtlsdr`, a real Wimp GUI app called `!RTLSDRView` plus a
CLI diagnostic tool `!RTLSDR`), developed entirely through iterative
real-hardware testing on a physical RISC OS 5.30 system — a **Raspberry
Pi Compute Module 4 on the official CM4 IO Board**, with an RTL-SDR
Blog V4 dongle (USB High-Speed, bulk endpoint 1, 512-byte max packet,
confirmed via `*USBDevInfo`/`*USBConfInfo`).

Everything works: enumeration, control transfers, tuner/baseband init,
PLL lock at a real frequency, a live FFT spectrum display, gain
control, frequency tuning, FM demodulation with a numeric deviation
readout, and a TimPlayer-based audio test tone. The one thing that
doesn't work is **real-time streaming of the actual demodulated audio**
— and after an extremely thorough investigation (documented in that
repo's `docs/PLAN.md`, "Phase 2, follow-on: real audio output,
milestone 3"), the conclusion is that this is blocked by a **structural
limitation in RISC OS's `USBDriver` module**, not a bug in my
application. This new session/project is to investigate whether that
driver-level limitation can actually be fixed.

## The core finding

`USBDriver`'s handling of a `DeviceFS` bulk-IN stream (`start_read()`/
`read_cb()` in `build/c/usbmodule`,
`gitlab.riscosopen.org/RiscOS/Sources/HWSupport/USB/USBDriver`) only
ever keeps **ONE USB transfer in flight at a time, per opened stream**:

```c
if (str->xfer_busy) {
    dprintf (("", "Can't start, xfer still busy\n"));
    goto end;
}
```

A `read()` call made while a transfer is still in progress returns 0
immediately, submitting nothing new. Completion is only noticed (via
an interrupt-driven callback clearing `xfer_busy`) on a *later* `read()`
call — tight re-polling is the intended way to wait for it, and I've
confirmed that changing my own app's read loop to do this genuinely
(from "give up on busy" to "keep retrying") barely moved throughput at
all, which is itself informative: it means the bottleneck isn't in how
the *application* waits, it's in how infrequently real completions
actually happen.

A `DeviceCall_USB_TransferInfo` diagnostic (reason `0x80000006`, needs
a USB stream handle from `Return Handles 2`, reason `0x80000007`) shows
DeviceFS's own live view of the in-flight transfer, and it regularly
reports `status=1` (transfer **successfully completed**) after
receiving only ~2 of tens of thousands of requested bytes — i.e.
genuine short-packet completions happening very frequently, not
timeouts or errors.

Real measured sustained throughput (a GUI-free CLI tight loop, `nopad`
+ non-blocking mode, wall-clock bounded, no per-call attempt cap) sits
around **15-65 KB/s**, regardless of:
- requested chunk size (512 bytes through 65536 bytes — three orders
  of magnitude, no meaningful effect)
- the DeviceFS `size/N` stream-buffer path field (default vs. 128KB vs.
  512KB — some effect, but small and noisy)
- busy-retry vs. give-up strategy in the read loop
- whether the reads happen inside a Wimp idle-tick GUI loop or a plain
  CLI tight loop (ruling out GUI/event-loop overhead as the cause)
- the *configured* SDR sample rate (2.4 MSPS vs. 250 kHz gave
  near-identical achieved bytes/sec in the same test run — ruling out
  "the chip just isn't producing data that fast" as the explanation;
  if it were chip-side, 250 kHz should have been dramatically slower)

Target needed for real-time audio: ~4.8 MB/s (2.4 MSPS × 2 bytes).
Achieved: ~0.3-1.4% of that. This is a ~100-300x gap, far too large for
read-loop tuning to plausibly close.

One genuine, measurable (if insufficient) win: the USB API doc's
`short/S` stream-open path field — "Force a 'short packet' to be sent
at the end of each transfer... even if the data is an exact multiple
[of max packet size]... equivalent to the `USBD_FORCE_SHORT_XFER` flag
used by the core NetBSD code" — gave a real, directly-comparable ~65%
throughput gain in the CLI test (30041 → 49679 bytes/sec, same run).
Applied in the app already; didn't perceptibly change real-world audio
quality given how far short of the target it still is.

## Other things checked and ruled out

- **Multiple independent streams on the same endpoint** (to get
  application-level pipelining without touching driver code): `OS_Find`
  refuses to open the same bulk endpoint a second time while one stream
  is already open. Blocked at the DeviceFS level.
- **Wrong USB controller**: some Pi4-class boards route USB-A ports
  through a separate VL805/XHCI chip instead of the SoC's DWC2, and
  XHCI is reportedly slower under RISC OS. Confirmed via `*Modules`
  that `DWCDriver` is loaded and no XHCI/VL805 module is present at all
  — this hardware genuinely is on the DWC2 path (the better-supported
  one), so that's not the explanation.
- **A simple, obvious driver bug**: cloned the actual `DWCDriver`
  source (`gitlab.riscosopen.org/RiscOS/Sources/HWSupport/USB/
  Controllers/DWCDriver`) — it's a full port of the Synopsys DWC2 HCD
  (44,810 lines total across `dwc/driver/c/*`, essentially the same
  code Linux's `dwc2`/`dwc_otg` driver uses, including the Pi-specific
  `dwc_otg_fiq_fsm` FIQ-based scheduling fix). Nothing obviously wrong
  jumped out from a scan, and a bug in code this mature and widely
  shared seems unlikely — but I did not do a full line-by-line audit;
  that's realistically the job of this new investigation.
- **RTL2832U device-side misconfiguration**: `rtlsdr_init_baseband()`,
  `rtlsdr_set_sample_rate()`, and the R820T/R828D-specific
  `rtlsdr_tuner_postinit()` overrides were all checked line-by-line
  against real upstream `librtlsdr.c` — exact matches. Real,
  correctly-shaped 2-peak FM spectra have been confirmed on hardware
  many times, so the signal chain itself is known-good.

## Relevant prior community history (found via web search)

- [RISC OS Open forum: "USB outbound SLOW on Raspberry Pi"](https://www.riscosopen.org/forum/forums/11/topics/1893)
  (2013, older Pi hardware) — a similar-magnitude (~20KB/s at default
  clocks) USB throughput symptom, described as "some sort of timing
  issue," improved by overclocking. Probably not directly applicable
  (this project's CM4 already runs well above those old clock figures)
  but establishes this class of symptom isn't unprecedented.
- [RISC OS Open forum: "Pi Isochronous ROM"](https://www.riscosopen.org/forum/forums/9/topics/2730)
  (2014) — Colin (a RISC OS Open developer) built a patched ROM adding
  isochronous support and toggling `FORCE_SHORT_TRANSFER`, which
  measurably fixed a different (outbound/upload) USB throughput
  problem for at least one user. Relevant because it shows (a) a ROOL
  developer has form for exactly this kind of DWC2 driver surgery, and
  (b) the `FORCE_SHORT_TRANSFER`/`short` flag has prior history as a
  lever for USB throughput issues on this hardware.
- No prior art found for RTL-SDR specifically on RISC OS — this project
  is pioneering that combination.

## Suggested starting points for this new investigation

1. Clone both repos locally and read `start_read()`/`read_cb()` in
   `USBDriver` alongside the actual transfer-submission path in
   `DWCDriver` (`dwc_otg_hcd.c`, `dwc_otg_hcd_queue.c`,
   `dwc_otg_hcd_intr.c`) to understand exactly what's gating a new
   transfer submission after one completes — is it purely the
   `xfer_busy` software flag, or is there also a hardware-channel or
   scheduling constraint underneath that would need touching too?
2. Check whether `DWCDriver`/the DWC2 HCD hardware actually supports
   multiple simultaneous "channels" (it should — this is standard DWC2
   hardware capability, Linux uses it for concurrent URBs) and whether
   `USBDriver`'s single-transfer-per-stream design is a deliberate
   simplification that could be relaxed, vs. something deeper.
3. Consider whether the fix belongs in `USBDriver` (queue more than one
   transfer per stream) or is achievable via a smarter use of the
   existing `DeviceCall_USB_TransferInfo`/async-callback mechanisms
   already exposed to userspace (the USB API doc mentions an R6-bit1
   callback option for "USB Control Request" transfers specifically —
   worth checking whether an equivalent exists, or could be added, for
   bulk transfers).
4. The full USB API doc (fetched from `gitlab.riscosopen.org/RiscOS/
   Sources/HWSupport/USB/USBDriver/-/raw/master/build/Doc/USB` — note:
   this URL blocks plain `curl`/`WebFetch` with a "suspicious access"
   anti-bot page; fetching it through an actual browser tool worked
   fine) documents every path field and `DeviceCall` extension already
   tried (`nopad`, `short`, `size/N`, `Check Buffer Space`, `Transfer
   Info`, `Get/Set Options`) — worth rereading in full before assuming
   something new needs inventing.
5. Real-hardware testing rule carried over from the app project: **run
   built code directly, never inside a TaskWindow** — TaskWindow
   execution was implicated in repeated full-machine freezes during
   earlier phases of the app work, and this is driver-level code where
   a hang is even higher-stakes.

## How to verify a fix

The `riscos-rtlsdr` repo's `!RTLSDR` CLI tool has a milestone 6
diagnostic (`RTLSDR,ff8`, run directly, no GUI) that measures honest,
wall-clock-bounded sustained bulk-IN throughput against a real RTL-SDR
dongle — the cleanest available real-world regression test for any
driver change. If you get to the point of testing a driver
modification, that binary (or a rebuilt version of it) is the fastest
way to see whether it actually moved the needle, before circling back
to `!RTLSDRView` to check whether real-time audio streaming actually
works now.
