# riscos-rtlsdr

A native RISC OS driver/app for RTL-SDR (RTL2832U) USB dongles. RISC OS has
no libusb and no existing RTL-SDR support, so this talks to the dongle
directly through RISC OS's own DeviceFS USB interface. See
[`docs/PLAN.md`](docs/PLAN.md) for the full technical plan, API grounding,
and milestones — start there. Where it is going next (a proper user
interface, waterfall, more modes and tools, modelled on SDR++) is in
[`docs/ROADMAP.md`](docs/ROADMAP.md).

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
  wideband-FM audio (STREAM)** — the stream (240 kSPS for audio, or the
  full 2.4 MSPS) is demodulated live and played through RISC OS's standard
  **TimPlayer** module.

**How the audio works.** Audio output uses TimPlayer (already resident on
most RISC OS 5 systems, or loaded from
`System:Modules.Audio.Trackers.TimPlayer`), which does all the
interrupt-driven DMA/mixing itself; the app never touches assembler. The
demodulator (`Dsp.c`) is **integer-only** -- this build has no `-fpu`
option, so every `double` is emulated floating point, roughly 100x slower
per operation -- and runs on every sample: an optional 3rd-order CIC
decimator (2.4 MSPS -> 240 kSPS; skipped when the dongle itself is run at
240 kSPS), an integer polar discriminator, 50 us de-emphasis, and a
resampler to whatever rate the mixer runs at. It was validated against
synthetic FM on a Linux machine before it ever ran on the Pi. Only the FFT
display uses (emulated) floating point, and it is budgeted to a few frames
per update.

RISC OS multitasks cooperatively, so any other task can stall this app for
tens or hundreds of milliseconds. The USB side rides that out by asking the
driver for big transfers: USBDriver keeps one bulk transfer in flight plus
one queued, arms the next only when the app makes a read call, and sizes
each to the *read request* -- so the read size (not the stream buffer) is
the stall tolerance. The audio side keeps a lead of a few hundred
milliseconds in its ring. See `docs/PLAN.md` "milestone 7" for the evidence.

**Known limitations.**
- **A stall longer than the audio lead is an audible pause** (RISC OS
  cooperative multitasking; the lead is `RTLSDRView$LeadMS`, default 500 ms
  -- more lead, more latency). The defaults (240 kSPS, 192 KB reads, 500 ms
  lead) are the profile that held up best against real desktop use
  (window dragging, NAS folder windows): a four-minute session on them,
  dragging windows around, had no underruns and lost no data (worst desktop
  gap 240 ms). The price is ~0.9 s between the air and the speaker, so
  tuning feedback is not instant. A stall longer than the lead, which does
  happen occasionally (one log caught three of 1-1.3 s), is still an
  audible pause.
- The default 240 kSPS mode shows a 240 kHz-wide spectrum (one station);
  `RTLSDRView$RateK 2400` gives the whole 2.4 MHz, but a transfer is then
  only ~27 ms of data, so that mode cannot ride out stalls.
- Mono only (no stereo decode, no RDS); no squelch, so weak stations hiss;
  no volume control yet; the station must be at the centre of the capture
  (tune with F-/F+).

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
point), then the desktop's cooperative scheduling -- where buffer-size tuning
did nothing until the driver's source showed that it only arms a USB transfer
per read call, sized to the read request (see `docs/PLAN.md` milestone 7). Along the way: an I2C
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
- `app/!RTLSDRView/c/Trace.c`, `h/Trace.h` — per-second statistics and the
  optional log file (`RTLSDRView$Log`) used to diagnose stalls and USB loss.
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

The readout under the buttons shows `pk` (FM peak deviation, kHz), `r`
(complex samples/s actually read over the last few seconds, thousands --
should sit at the dongle's rate, 240k or 2400k), `g`/`G` (longest time the
rest of the desktop held the CPU between two polls, in the last few seconds
/ ever, ms) and `u` (audible audio underruns). Settings are RISC OS system
variables read at start-up (an Obey launcher just `*Set`s them first):

```
*Set RTLSDRView$RateK 240     dongle sample rate, kSPS: 240 (default) or 2400
*Set RTLSDRView$XferKB 192    read request = USB transfer size, KB
                              (default: ~400 ms of data; 128 KB at 2.4 MSPS)
*Set RTLSDRView$BufKB 1024    DeviceFS stream buffer, KB
                              (default 4x XferKB, power of two)
*Set RTLSDRView$LeadMS 500    audio lead, ms (default 500 at 240 kSPS, 350 above)
*Set RTLSDRView$FFTFrames 4   spectrum frames per display update (default 4)
*Set RTLSDRView$PaceMS 10     pause after each tuner I2C transfer once running
                              (default 0: a retune takes ~50 ms instead of
                              200-600 ms; the pause is a leftover from the
                              hang hunt -- 10 restores it)
*Set RTLSDRView$Log <Obey$Dir>.Log1    write a per-second trace here on exit
```

Ready-made Obey launchers for the profiles compared during the stall work
are in [`tools/launchers/`](tools/launchers/).

If the driver can't service the configured transfer size (no data for 3 s),
the app falls back to 16 KB reads and a 128 KB buffer and says so.
`RTLSDRView$Log` is the way to see what really happened: one row per second
(bytes read, polls, where the time went, the audio lead's range, underruns),
timestamped events (STREAM, each retune and how long it took, and a marker
whenever the graph is clicked), and a histogram of desktop stalls. Examples
from real runs are in [`docs/logs/`](docs/logs/).
