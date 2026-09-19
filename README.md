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

⚠️ **Milestone 4's bulk-streaming result is now known to be wrong and
should NOT be treated as settled.** It originally reported "25.6 MB/s
sustained throughput", but that number only ever verified byte
*counts* returned by `OS_GBPB`, never actual byte *content*. A phase 2
diagnostic (see `docs/PLAN.md`'s milestone 5) proved via a sentinel-fill
test that `OS_GBPB` reason 4 against this bulk endpoint reports a full
transfer while genuinely writing only ~2 real bytes per call,
zero-padding the rest — meaning the real sustained data rate was
actually closer to ~50KB/s (about 1% of the ~4.8MB/s target), not 5x
over it. **Root cause found**: this is a documented, ~20-year-old
RISC OS DeviceFS USB characteristic (short reads get silently padded
to the requested size, with no reliable way to learn the true transfer
size from the public API) — not a bug introduced by this project. See
`docs/PLAN.md` for the primary source and full evidence.

**Phase 2, milestone 1 (`!RTLSDRView`, a live spectrum display): the
architecture is confirmed on real hardware** — a real Wimp GUI app,
continuous idle-driven USB reads, a hand-written FFT, and a
live-updating bar-graph redraw, all running inside the Wimp event loop
without freezing the desktop. Given the milestone 4 finding above, the
displayed spectrum should currently be treated as showing whatever
`OS_GBPB`'s zero-padded reads actually produce, not confirmed genuine
RF content — signal-quality work is blocked on the bulk-read
investigation, not just gain/display-scale polish as first thought. See
`docs/PLAN.md`'s "Phase 2" section for the full history.

Getting here was a real diagnostic journey — full blow-by-blow in
`docs/PLAN.md`, including several real bugs only found by actually
running this on hardware: an I2C-write chunking limit, a hardware
bit-reversal quirk on I2C reads, TaskWindow execution implicated in
repeated full-machine freezes (fixed by running directly instead — now
the operational rule), `fread()` never working against this DeviceFS
stream at any size for reasons never fully root-caused (worked around
with raw `OS_Find`/`OS_GBPB` instead, which itself turned out to have
its own reliable-size ceiling), a hand-built Wimp window definition
block crashing `Wimp_CreateWindow` with a data abort because Norcroft
can pad a mixed char/bitfield struct past its intended size, and a
disabled GPIO call (the V4's antenna-vs-upconverter RF switch) that
turned out safe to re-enable once TaskWindow was ruled out as the real
freeze cause.

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
