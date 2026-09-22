# riscos-rtlsdr

A native RISC OS driver/app for RTL-SDR (RTL2832U) USB dongles. RISC OS has
no libusb and no existing RTL-SDR support, so this talks to the dongle
directly through RISC OS's own DeviceFS USB interface. See
[`docs/PLAN.md`](docs/PLAN.md) for the full technical plan, API grounding,
and milestones — start there.

## Status

**Phase 1 (milestones 1-4): enumeration through control transfers and
baseband/tuner bring-up are confirmed on real hardware.** An
**RTL-SDR Blog V4** dongle (R828D tuner + upconverter), VID `0BDA` /
PID `2838`, found via `*USBDevices` → `DeviceFS_CallDevice` control
transfers → baseband init → tuner identified and locked → sample rate
configured. No libusb, no existing driver to build on; this talks to
the dongle entirely through RISC OS's own native USB interface.

⚠️ **Milestone 4's original "25.6 MB/s sustained throughput" result was
wrong and should not be treated as settled** — it only ever verified
byte *counts* returned by `OS_GBPB`, never actual byte *content*. A
phase 2 diagnostic (see `docs/PLAN.md`'s milestone 5) proved via a
sentinel-fill test that `OS_GBPB` reason 4 against this bulk endpoint,
in its default blocking mode, reports a full transfer while genuinely
writing only ~2 real bytes per call, zero-padding the rest — a
documented, ~20-year-old RISC OS DeviceFS characteristic (short reads
get silently padded, with no reliable way to learn the true transfer
size from the public API), not a bug introduced by this project. **Now
fixed**: a second, public SWI (`OS_Args` reason 9, IOCtl group `0xFF`
reason 1) enables non-blocking mode on the stream, after which
`OS_GBPB` reports honest short-read counts — confirmed on real hardware
and wired into both `!RTLSDR` (a new milestone 5) and `!RTLSDRView`.
Milestone 4's actual throughput number has not yet been re-measured
with this fix; treat the old number as void until it is.

**Phase 2, milestone 1 (`!RTLSDRView`, a live spectrum display) is
done, confirmed on real hardware: a real, multi-peak spectrum showing
genuine broadcast stations, live gain control (AGC/manual), live
frequency tuning, an FM demodulator (numeric readout only — no
streamed audio yet), and a working audio test tone via the standard
TimPlayer module.** A real Wimp GUI app: continuous idle-driven honest
USB reads, a hand-written FFT, power-domain frame averaging, and a
live-updating bar-graph redraw, all running inside the Wimp event loop
without freezing the desktop. A long list of real bugs was found and
fixed getting here (see `docs/PLAN.md`'s "Phase 2" section for the full
history) — most notably a genuine analog/digital IF mismatch (this
port never called the real upstream bandwidth-configuration step,
leaving the tuner's actual analog IF and the demod's digital
downconversion offset by ~1.75MHz, comparable to the capture's own
Nyquist limit) and, the actual root cause of the long-standing flat/
noisy spectrum: DeviceFS's non-blocking short-read mechanism, for
larger request sizes, returns a buffer where only the very first
sample is genuinely fresh and the rest is padding-by-repetition of an
earlier byte — invisible to the zero-padding-only sentinel test used
to validate the original milestone-5 fix, and worse the larger the
request. Shrinking the read size down to 8 bytes (4 samples) fixed it:
`nElev` (samples per frame carrying real energy, out of 256) jumped
from 1-2 to 256, and the rendered spectrum finally shows real,
distinct station peaks instead of noise.

**Real audio output work has started, but real-time streaming is
blocked on a structural RISC OS USB limitation.** RISC OS's actual
streaming-audio mechanism was researched from real, working source
(DigitalCD's own `!PlayTone` example, user-supplied) rather than
guessed at — it goes through the standard **TimPlayer** module (already
resident on most RISC OS 5 systems, or auto-loaded from
`System:Modules.Audio.Trackers.TimPlayer`), which handles all
interrupt-driven DMA/mixing internally; the app itself never touches
assembler. A "TONE" button in `!RTLSDRView` plays a generated test tone
through it — confirmed working on real hardware, first try. Streaming
the actual FM-demodulated audio was attempted next, and after an
exhaustive investigation (chunk size, DeviceFS buffer size, retry
strategy, GUI-vs-CLI isolation, USB controller identity, multi-stream
pipelining, sample-rate independence — see `docs/PLAN.md`'s "milestone
3" for the full elimination process), achieved throughput tops out
around 10-13k IQ samples/sec against the 2.4M needed — a genuine,
structural limitation in how RISC OS's USBDriver services bulk-IN
endpoints (only one transfer ever in flight per stream, frequent
short-packet completions), not an application-level bug. A follow-up
project investigating the USB driver stack itself (`USBDriver`/
`DWCDriver`) is the planned next step for unblocking this; `!RTLSDRView`
itself keeps the test tone and a numeric FM-deviation readout as where
real-time audio settles for now.

Getting here was a real diagnostic journey — full blow-by-blow in
`docs/PLAN.md`, including several real bugs only found by actually
running this on hardware: an I2C-write chunking limit, a hardware
bit-reversal quirk on I2C reads, TaskWindow execution implicated in
repeated full-machine freezes (fixed by running directly instead — now
the operational rule), `fread()` never working against this DeviceFS
stream at any size for reasons never fully root-caused (worked around
with raw `OS_Find`/`OS_GBPB` instead), a hand-built Wimp window
definition block crashing `Wimp_CreateWindow` with a data abort because
Norcroft can pad a mixed char/bitfield struct past its intended size, a
disabled GPIO call (the V4's antenna-vs-upconverter RF switch) that
turned out safe to re-enable once TaskWindow was ruled out as the real
freeze cause, DeviceFS's read-padding described above, and the
degenerate zero-height rendering bug.

## Layout

- `docs/PLAN.md` — the project plan: why, how, milestones, risks.
- `tools/probe.bas` — no-compiler-needed BASIC script for confirming
  hardware facts (device enumeration, VID/PID, endpoint layout) before
  writing C against them. Already run once; see `docs/PLAN.md`.
- `app/!RTLSDR/c/RTLSDR.c`, `h/RTLSDR.h` — the command-line diagnostic
  tool: milestone-specific tests plus `main()`.
- `app/!RTLSDR/c/Driver.c`, `h/Driver.h` — shared device bringup (device
  discovery, baseband/tuner init, sample rate, raw `OS_Find`/`OS_GBPB`
  stream I/O), used by both `!RTLSDR` and `!RTLSDRView`.
- `app/!RTLSDR/c/R82XX.c`, `h/R82XX.h` — the R820T/R828D tuner driver
  (PLL, filter calibration, V4-specific band-switching), also shared.
- `app/!RTLSDRView/c/SpecView.c` — the live spectrum display Wimp app.
- `build/plan.json` — compile/link plan for `build.riscos.online`.

## Building

`build/build_one_shot.py` builds against the `build.riscos.online` cloud
service (same as `riscos-rdpclient`) in one shot — this project is small
enough that `riscos-rdpclient`'s adaptive chunked `buildapp.py` isn't
needed (only worth pulling in if it grows past what fits the service's
per-request wall-clock cap). It needs the `websocket-client` Python
package; `riscos-rdpclient`'s existing venv already has it:

```
~/Development/riscos-rdpclient/rdpclient/build/.venv/bin/python3 build/build_one_shot.py
```

Output lands at `build/RTLSDR,ff8` and `build/RTLSDRView,ff8` (RISC OS
AIF/Absolute format, tagged with the `,ff8` filetype suffix for
transfer). Gotchas hit and fixed while getting this working, in case the
build script needs touching again: RISC OS leafnames carry no extension
(source must be staged as `c/RTLSDR`, not `c/RTLSDR.c`, inside the
uploaded zip — see `_riscos_leaf()`); the service collects artifacts by
zipping a *directory* you name in `.robuild.yaml`'s `artifacts: path:`,
not a single file — hence `plan.json` copies each linked binary into
`./out/` as a last step; and `!RTLSDRView` links against a prebuilt
32-bit DeskLib reused from `~/Development/riscos-rdpclient`'s
`builddesklib.py` (staged as `desklib/*.h` + `desklib/o/DeskLib` with
its headers keeping their literal dotted names, since they're referenced
via the full pathname `DeskLib:Xyz.h`, not the extension-stripped `-I,C:`
search-path convention the app's own sources use).

Alternative: this machine also has a working local GCCSDK toolchain
(`arm-riscos-gnueabihf-gcc`, proven in `~/Development/hello`) if
Norcroft/DDE proves inconvenient for fast iteration — see that project's
`Makefile` for the invocation and its ELF-not-AIF gotcha.

## Testing

**Requires real RISC OS hardware with the dongle attached** — the
`RISCOSQEMUA72` emulator has no USB passthrough, so it can't be used here.

Copy `build/RTLSDR,ff8` and/or `build/RTLSDRView,ff8` to the Pi and run
them **directly — double-click in the Filer, or `*Run` — not inside a
TaskWindow** (see Status above: TaskWindow execution is implicated in
repeated full-machine freezes; direct execution has not reproduced that
across many runs). `!RTLSDR` is a `printf`-based diagnostic tool, not a
Wimp app; running it directly opens a text output window and shows
"Press SPACE or click mouse to continue" when done, rather than closing
immediately. It runs every milestone in sequence and stops at the first
failure. `!RTLSDRView` is a real Wimp app — it opens a window showing a
live spectrum trace (fixed at 97.4MHz for now, no tuning UI yet); watch
desktop responsiveness while it's running (move another window, click
elsewhere) as the main regression risk to check for.
