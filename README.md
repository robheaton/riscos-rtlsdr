# riscos-rtlsdr

A native RISC OS driver/app for RTL-SDR (RTL2832U) USB dongles. RISC OS has
no libusb and no existing RTL-SDR support, so this talks to the dongle
directly through RISC OS's own DeviceFS USB interface. See
[`docs/PLAN.md`](docs/PLAN.md) for the full technical plan, API grounding,
and milestones — start there.

## Status

**Working on real hardware: a native RISC OS software-defined radio that
plays live broadcast FM.** Everything below is confirmed on a Raspberry Pi
Compute Module 4 running RISC OS 5.30 with an RTL-SDR Blog V4 (R828D tuner
+ upconverter, VID `0BDA` / PID `2838`), talking to the dongle only through
RISC OS's own DeviceFS USB interface — no libusb, no existing driver to
build on.

- **Device bring-up** (`!RTLSDR`, milestones 1-3): enumeration via
  `*USBDevices`, vendor control transfers through `DeviceFS_CallDevice`,
  baseband init, tuner identification, PLL lock at a real frequency,
  sample-rate configuration.
- **Sustained bulk streaming** (`!RTLSDR`, milestones 4-11): **~4.7 MB/s,
  98% of the 2.4 MSPS stream**, tight-loop, no GUI.
- **`!RTLSDRView`**, a Wimp app: live spectrum (hand-written FFT,
  power-domain averaging), AGC/manual gain, frequency tuning (F-/F+), an FM
  deviation readout (DEM), a TimPlayer test tone (TONE), and **real-time
  wideband-FM audio (STREAM)** — the whole 2.4 MSPS stream is demodulated
  live and played through RISC OS's standard **TimPlayer** module.

**How the audio works.** Audio output uses TimPlayer (already resident on
most RISC OS 5 systems, or loaded from
`System:Modules.Audio.Trackers.TimPlayer`), which does all interrupt-driven
DMA/mixing itself; the app never touches assembler. The demodulator
(`Dsp.c`) is **integer-only** — this build has no `-fpu` option, so every
`double` is emulated floating point, roughly 100x slower per operation —
and runs on every sample: a 3rd-order CIC decimate-by-10 (2.4 MSPS → 240
kSPS, which also acts as the channel filter), an integer polar
discriminator, 50 µs de-emphasis, and a resampler to whatever rate the
mixer runs at. It was validated against synthetic FM on a Linux machine
before it ever ran on the Pi. Only the FFT display uses (emulated) floating
point, and it is budgeted to a few frames per update.

**Known limitations.**
- **Audio pauses when other tasks use the CPU.** RISC OS multitasks
  cooperatively: while another task holds the CPU this app isn't polled, so
  nothing drains the USB buffer or refills the audio ring. Mitigated with
  bigger, tunable buffers (see Testing); the readout shows the longest gap
  between polls (`g`, ms) and the count of audible underruns (`u`).
- Mono only (no stereo decode, no RDS); no squelch, so weak stations hiss;
  no volume control yet; the station must be at the centre of the 2.4 MHz
  capture (tune with F-/F+).

**How it got here — the short version** (the full, honest history including
the wrong turns is in [`docs/PLAN.md`](docs/PLAN.md)). Streaming was blocked
for a long time by a ceiling of ~10-30 KB/s against the ~4.8 MB/s a 2.4 MSPS
capture needs, and by a "flat spectrum" mystery. A long investigation —
eventually a two-session effort that eliminated the application,
`USBDriver` and `DWCDriver` layers and even read the DWC2 hardware
registers — turned out to have a single cause in this project's own code:
`rtlsdr_write_reg()` packed 2-byte values low-byte-first instead of
high-byte-first, so the dongle's bulk endpoint max packet size was
programmed as **2 bytes** instead of 512. Every packet, and so every
transfer, was 2 bytes. It was found by diffing the *raw wire bytes* of the
control transfers against a real Linux capture of the same dongle — earlier
comparisons of register *values* and transaction *counts* could not see a
byte-order difference. The next bottleneck was the CPU (emulated floating
point), then the desktop's cooperative scheduling. Along the way: an I2C
write chunking limit, a hardware bit-reversal quirk on I2C reads,
TaskWindow execution implicated in repeated full-machine freezes (the
operational rule is now to run directly, never in a TaskWindow), `fread()`
never working against this DeviceFS stream (raw `OS_Find`/`OS_GBPB` instead),
an analog/digital IF mismatch, and a hand-built Wimp window block that
Norcroft padded past its intended size.

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
- `app/!RTLSDRView/c/SpecView.c` — the Wimp app: spectrum, gain/tuning UI,
  the USB read loop.
- `app/!RTLSDRView/c/Dsp.c`, `h/Dsp.h` — the integer-only real-time FM
  demodulator (CIC decimator, discriminator, de-emphasis, resampler).
- `app/!RTLSDRView/c/Audio.c`, `h/Audio.h` — TimPlayer audio output: the
  test tone and the streaming ring buffer.
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
failure. `!RTLSDRView` is a real Wimp app: it opens a window with the live spectrum
(default 97.4 MHz; F-/F+ retune in 100 kHz steps, AGC and +/- for gain).
Press **DEM** for the FM deviation readout and **STREAM** to listen. Watch
desktop responsiveness while it's running (move another window, click
elsewhere) — that, and audio pauses, are the main things to check.

The readout under the buttons shows `pk`/`rms` (FM deviation, kHz), `r`
(USB read rate in kilo-samples/s; close to 2400k when healthy), `g` (longest gap
between polls, ms) and `u` (audio underruns). Two system variables tune how
much desktop stall the audio can ride out, at the cost of latency; set
them before launching:

```
*Set RTLSDRView$BufKB 512     USB stream buffer, KB   (default 512)
*Set RTLSDRView$LeadMS 350    audio lead, ms          (default 350)
```

The driver sizes each USB transfer to the buffer's free space, so a bigger
buffer means bigger bursts as well as more tolerance. If a very large
`BufKB` delivers no data, the app falls back to a 128 KB buffer after 3
seconds and says so.
