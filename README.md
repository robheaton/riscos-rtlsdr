# riscos-rtlsdr

A native RISC OS driver/app for RTL-SDR (RTL2832U) USB dongles. RISC OS has
no libusb and no existing RTL-SDR support, so this talks to the dongle
directly through RISC OS's own DeviceFS USB interface. See
[`docs/PLAN.md`](docs/PLAN.md) for the full technical plan, API grounding,
and milestones — start there.

## Status

**All four core milestones are done, confirmed on real hardware.**
RTL-SDR works end-to-end on RISC OS: an **RTL-SDR Blog V4** dongle
(R828D tuner + upconverter), VID `0BDA` / PID `2838`, found via
`*USBDevices` → `DeviceFS_CallDevice` control transfers → baseband init
→ tuner identified and locked at 100MHz → sample rate configured →
**sustained bulk streaming measured at 25.6 MB/s — over 5x the ~4.8 MB/s
a real SDR needs.** No libusb, no existing driver to build on; this talks
to the dongle entirely through RISC OS's own native USB interface.

Getting here was a real diagnostic journey — full blow-by-blow in
`docs/PLAN.md`, including several real bugs only found by actually
running this on hardware: an I2C-write chunking limit, a hardware
bit-reversal quirk on I2C reads, TaskWindow execution implicated in
repeated full-machine freezes (fixed by running directly instead — now
the operational rule), and `fread()` never working against this
DeviceFS stream at any size for reasons never fully root-caused (worked
around with raw `OS_Find`/`OS_GBPB` instead, which itself turned out to
have its own reliable-size ceiling — 1024 bytes good, 2048 blocks for
~110s — found by an empirical sweep, not documentation).

**What exists now is a command-line diagnostic tool, not a usable SDR
application.** Milestone 5 (FFT/waterfall display, demodulation, a real
Wimp app) is unstarted — a substantial new phase, not a continuation of
the current diagnostic loop.

## Layout

- `docs/PLAN.md` — the project plan: why, how, milestones, risks.
- `tools/probe.bas` — no-compiler-needed BASIC script for confirming
  hardware facts (device enumeration, VID/PID, endpoint layout) before
  writing C against them. Already run once; see `docs/PLAN.md`.
- `app/!RTLSDR/c/RTLSDR.c`, `h/RTLSDR.h` — device enumeration, baseband
  init, and the shared USB-control-transfer/register-I/O primitives
  (`usb_ctrl_transfer`, `rtlsdr_read_reg`/`write_reg`) that `R82XX.c` uses.
- `app/!RTLSDR/c/R82XX.c`, `h/R82XX.h` — the R820T/R828D tuner driver
  (PLL, filter calibration, V4-specific band-switching).
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

Output lands at `build/RTLSDR,ff8` (RISC OS AIF/Absolute format, tagged
with the `,ff8` filetype suffix for transfer). Two gotchas hit and fixed
while getting this working, in case the build script needs touching again:
RISC OS leafnames carry no extension (source must be staged as `c/RTLSDR`,
not `c/RTLSDR.c`, inside the uploaded zip — see `_riscos_leaf()`), and the
service collects artifacts by zipping a *directory* you name in
`.robuild.yaml`'s `artifacts: path:`, not a single file — hence `plan.json`
copies the linked binary into `./out/` as its last step.

Alternative: this machine also has a working local GCCSDK toolchain
(`arm-riscos-gnueabihf-gcc`, proven in `~/Development/hello`) if
Norcroft/DDE proves inconvenient for fast iteration — see that project's
`Makefile` for the invocation and its ELF-not-AIF gotcha.

## Testing

**Requires real RISC OS hardware with the dongle attached** — the
`RISCOSQEMUA72` emulator has no USB passthrough, so it can't be used here.

Copy `build/RTLSDR,ff8` to the Pi and run it **directly — double-click it
in the Filer, or `*Run` it — not inside a TaskWindow** (see Status above:
TaskWindow execution is implicated in repeated full-machine freezes;
direct execution has not reproduced that). It's a `printf`-based
diagnostic tool, not yet a Wimp app; running it directly opens a text
output window and shows "Press SPACE or click mouse to continue" when
done, rather than closing immediately. It runs every milestone in
sequence and stops at the first failure — one full run now takes the
dongle from "unidentified USB device" all the way through sustained bulk
streaming at ~25 MB/s, with nothing currently expected to fail.
