# RTLSDRView roadmap: from spectrum-and-audio demo to a usable SDR receiver

Status: proposal, 2026-09-30. Nothing here is built yet. It is meant to be
cut up, reordered and argued with.

**Why now.** The hard part is done: the USB pipe is lossless, real-time FM
audio runs at ~5% CPU and rides out desktop stalls ([`PLAN.md`](PLAN.md)
milestones 5-7). What is left is that the program is still a *demo*: one
fixed 512x360 window, eight buttons, bars for a 256-point spectrum, FM only,
no volume, no way to type a frequency. This plan looks at what
[SDR++](https://github.com/AlexandreRouma/SDRPlusPlus) gives its users and
asks what a RISC OS-native version of each piece would be, what it would
cost, and in what order to do it.

**Sources.** [SDR++ user guide](https://www.sdrpp.org/manual.pdf) (v1.1,
Dec 2022), the [SDR++ README](https://github.com/AlexandreRouma/SDRPlusPlus)
and its screenshot. SDR++ is GPL-3.0; this plan borrows *ideas* (what the
controls are and how they behave), not code.

## 1. Recommendation in one screen

1. **Do a small foundation step first** (Phase 0): split the 1,900-line
   `SpecView.c` so UI, receiver state and USB/stream control are separate,
   add a Choices file so settings persist, give the app a proper skeleton
   (iconbar icon and menu, `Messages`, generated `Templates`), and add a
   **file source** that plays recorded I/Q through the same pipeline. The
   last one matters more than it looks: it lets us develop and test every
   later feature, including DSP and UI, without the dongle or antenna.
2. **Then make it a radio you can drive** (Phase 1): typed / per-digit /
   keyboard frequency entry, step sizes, volume and mute, squelch, an
   SNR/S-meter, gain in dB, bookmarks, and two cheap new modes (NFM and AM)
   which unlock airband, marine and PMR.
3. **Then the signature SDR++ feature, the waterfall** (Phase 2), with a
   bigger FFT, axes, a draggable VFO band and click-to-tune, zoom/min/max.
4. **Then more modes** (Phase 3: SSB/CW for HF through the V4's built-in
   up-converter, WFM stereo, RDS), **then tools** (Phase 4: recorder,
   scanner, band plans, frequency manager), **then stretch goals** (Phase 5:
   listen inside a wide view, an `rtl_tcp` server so a PC can use the
   dongle over the network).

Phases 2 and 3 are independent; swap them if airband/HF matters more to you
than the waterfall. Decisions I need from you are in section 8.

## 2. Where we are (measured, not guessed)

| area | today |
|---|---|
| Device | RTL-SDR Blog V4 brought up natively over DeviceFS USB: 240 kSPS or 2.4 MSPS, AGC or 16 manual gain steps, tune 24 MHz-1.7 GHz in 100 kHz steps. The tuner code already switches in the V4's HF up-converter below 28.8 MHz, but that path has never been exercised on hardware and the UI stops at 24 MHz |
| Audio | WFM mono, 50 us de-emphasis, TimPlayer output, ~0.9 s air-to-speaker latency at the default profile; no volume control, no squelch |
| Spectrum | 256-point FFT drawn as bars, ~10 frames/s, auto-scaled; no axes, no waterfall, no click-to-tune |
| Window | One fixed 512x360 window built in code (`create_button_icon`): AGC, -, +, F-, F+, DEM, TONE, STREAM and two text lines. The TONE label is clipped to "ONE" |
| Settings | None persisted; everything is a `*Set RTLSDRView$...` variable read at start-up |
| Code | `SpecView.c` 1,870 lines (UI + USB + tuning + display); `Dsp.c`, `Audio.c`, `Trace.c` are already separate |
| CPU | ~5% in steady state at 240 kSPS: demodulator ~2.5%, FFT ~2.6% (256 points x 10/s), USB reads negligible |

Two numbers shape everything below. **Floating point is emulated** (the
build has no `-fpu`): a 256-point FFT costs ~2.6 ms, so a 1024-point one is
~14 ms, which is 14% of the CPU at just 10 frames/s. And **the audio path
must never starve**: any new UI work has to fit inside the per-poll time
budget ([`Trace.c`](../app/!RTLSDRView/c/Trace.c) will show when it does not).

## 3. What SDR++ offers, and what it would take here

Effort: **S** under a session, **M** 1-3 sessions, **L** more. "Session" =
one of our build/test/read-the-log rounds.

### Main screen

| SDR++ feature | what it is for | RISC OS version | effort |
|---|---|---|---|
| Big frequency display; wheel/click/type per digit | fast, precise tuning | writable-digit display; click upper/lower half of a digit = +/-1, arrow keys, typed entry; wheel if the Wimp delivers it (to verify) | M |
| Play/stop, volume slider + mute | basics | STREAM button, slider (DeskLib `Slider`), mute; software gain already exists (`dsp_fm_set_gain`) | S |
| SNR meter | is this signal worth tuning to | bar meter from the demodulator's signal/noise estimate | S |
| Spectrum (FFT) + waterfall, resizable split | see many signals, spot bursts | sprite-based waterfall (8bpp, scroll by memmove, plot with a translation table); integer FFT; resizable window | L |
| Zoom, min, max sliders | contrast and span | vertical sliders beside the plot (DeskLib `Slider` supports vertical) | M |
| VFO band you can drag/resize; click or drag the scale to tune | tuning by eye; bandwidth by dragging the band edge | overlay drawn in the redraw loop; mouse handling in the Wimp click/drag events | M |
| Centre-tuning vs normal tuning | keeps the view stable while tuning | one toggle | S |
| Bookmarks drawn on the spectrum; band-plan strip | orientation | labels from data files drawn above the plot | M |
| Themes, colour maps | taste, contrast | 3-4 palettes for the waterfall (palette swap is cheap with an 8bpp sprite); "themes" = not planned | S |
| Keyboard shortcuts | speed | arrows/PageUp/Home/End equivalents | S |

### Radio module

| SDR++ feature | RISC OS version | effort |
|---|---|---|
| Modes NFM, WFM, AM, DSB, USB, LSB, CW, RAW; per-mode default filter (12.5 k / 150 k / 10 k / 4.6 k / 2.8 k / 2.8 k / 200 Hz) | NCO mixer + decimating FIR channel filter + per-mode demodulator (all integer; the discriminator we have is the NFM/WFM core). NFM and AM are small; SSB/CW need a proper filter chain | M per family |
| Filter width (type, +/-, drag band edge) and **snap interval** per mode | width re-designs the FIR; snap is just a per-mode step | S |
| Squelch | power-based (AM/SSB) and noise-based (FM: energy above the audio band) | S |
| De-emphasis (off/50 us/75 us), low-pass, IF noise reduction | one-pole filters exist; NR is a later experiment | S / M |
| AGC attack/decay, carrier AGC | integer AGC on the audio/IF level | M |
| Noise blanker (impulse noise) | simple threshold blanker on the complex stream | M |
| WFM **stereo** | 19 kHz pilot PLL, 38 kHz L-R; we have 240 kSPS, so the composite is in range | L |
| **RDS** (station name, radio text) | 57 kHz BPSK, differential decode, block sync + CRC, group parsing; text drawn under the frequency | L |
| Multiple VFOs | 2+ channel paths from one stream; the DSP is per-path state, the UI is the hard part | L |

### Source

| SDR++ feature | RISC OS version | effort |
|---|---|---|
| Gain slider in dB, AGC | map our 16 steps to real dB (upstream's LNA/mixer step tables); checkbox + slider | S |
| Sample rate menu | 240 k / 960 k / 1.2 M / 2.4 M with a note on stall tolerance for each | S |
| PPM correction, IQ correction | PPM is one register pair (the V4 has a TCXO, so rarely needed); IQ correction is a small complex-gain stage | S / M |
| Bias-T | the V4's bias-T is a GPIO bit; we disabled GPIO code in phase 1 after hangs, so this needs care | M |
| Offset tuning / up-converter | already implicit: HF is up-converted by the V4 below 28.8 MHz | done |
| Direct sampling | not needed on the V4 (up-converter instead) | not planned |
| **File source** (play recorded baseband) | the same stream interface fed from a file; `.iq`/WAV-16 | M; **high value** |
| Remote sources (SDR++ server, SpyServer) | network client; later | L |

### Tools

| SDR++ feature | RISC OS version | effort |
|---|---|---|
| **Frequency manager**: named lists, bookmarks (name, frequency, mode, width), import/export, shown on the spectrum | text file in Choices, a list window with add/edit/remove, a Bookmarks menu; export = drag the file out | M |
| **Scanner**: range, interval, level, tuning time (250 ms), linger (1 s), resume up/down, status | works off the FFT, so it is cheap; needs the span view and a VFO that moves inside it | M |
| **Recorder**: audio or baseband WAV, level meter, ignore-silence | write from a buffer in chunks (never stall the poll loop on a network share); WAV 16-bit; drag-and-drop save | M |
| Sinks: audio device choice, network audio (TCP/UDP), virtual cable | TimPlayer only for now; a network sink is a stretch goal | L |
| Rig-control server, SDR++ server | a small `rtl_tcp`-compatible server would let a PC's SDR software use this dongle; uses the Internet module's sockets | L |
| Decoders (M17, Meteor, pagers, radiosonde) | not planned; RDS is the one decoder that belongs with FM | no |
| Module system, plugins | not planned; one program, cleanly split source files | no |

## 4. Constraints that shape the RISC OS design

* **Cooperative multitasking and the stall budget.** Every poll we do must
  be short; FFT, waterfall and UI work get a time budget so the USB read
  and audio feed always run first. The per-second trace already breaks the
  poll time into USB / DSP / FFT / display, so every phase below can be
  judged by the log, not by feel.
* **No hardware floating point.** Audio path, filters, AGC, NCO and the
  FFT go integer. Filter coefficients can still be *designed* with `double`
  at start-up (once). Whether VFP is available with the current toolchain
  is Phase 0's first experiment (section 6).
* **The Wimp redraws on demand, not every frame.** The waterfall is a
  sprite we update and invalidate; we control the redraw rate. Sliders
  (DeskLib `Slider`) are drawn in the redraw loop, so the plot window needs
  the usual user-redraw care.
* **Two span modes, because of the USB pipe.** 240 kSPS is the *listening*
  mode (lossless, stall-proof, 0.9 s latency). 960 k-2.4 MSPS are *browse*
  modes (wide view; ~27-100 ms per transfer, so audio is best-effort,
  measured in `docs/logs`). The UI should say which one you are in.
  Listening *inside* a wide view is Phase 5.
* **Tuning latency.** The audio lead makes a retune take ~1 s to be heard.
  Moving the VFO inside the current span needs no retune at all (an NCO
  shift), which is another reason to want a VFO rather than only "tune the
  dongle".
* **Memory.** WimpSlot is 640 KB; big buffers already live in the RMA. A
  1024-2048-bin waterfall history and recording buffers may want a dynamic
  area.
* **No Templates editor in this environment.** Options: keep building
  windows in code (works today), or write the windows as a small text spec
  and have a repo script emit a `Templates` file (reviewable, repeatable;
  [MenuGen](https://github.com/steve-fryatt/menugen) does the same for
  menus), or draw them with FormEd/WinEd on the Pi. I suggest the script.
* **RISC OS conventions to follow:** Select = do it, Adjust = do the
  opposite or keep the window, Menu = context menu; iconbar icon with
  Info/Choices/Quit; Choices via `<Choices$Write>`; interactive help;
  `!Run` sets the WimpSlot and modules; run directly, never in a TaskWindow.

## 5. Proposed user interface

Main window (resizable; top bar is a Wimp pane, plot area fills the rest):

```
+------------------------------------------------------------------+
| RTLSDRView                                                  [][x]|
| [> STREAM] [NFM|WFM|AM|USB|LSB|CW]  97.400.000 Hz   Vol ####.. [M]|
| Squelch ..#......   BW 150 k   Step 100 k   SNR ######....  240k  |
|+-------------------------------------------------------------+ Z |
|| -20                 /\            /\  |VFO band|              | o |
|| -60 __/\_/\__/\____/  \___/\_____/##\_|        |____/\_       | o |
||     96.5      97.0      97.4      97.8      98.2      98.6    | M |
||+-----------------------------------------------------------+ a |
|| waterfall: vertical tracks, newest line at the top          | x |
|| ||   |   ||||    ||   ||||   ||   |||       ||      |       |   |
|+-------------------------------------------------------------+ M |
| lead 500 ms   underruns 0   CPU 6%   tuned 97.400 MHz         | i |
+------------------------------------------------------------------+
```

* **Top pane:** play/stop, mode buttons, the frequency (digits clickable),
  volume + mute, squelch slider, bandwidth, step, meter, span. Everything
  else is one click away in menus and dialogues rather than crammed in.
* **Plot:** Select-click tunes (snapped to the step); drag the band edge to
  change bandwidth; drag the scale to pan; Adjust-click tunes without
  re-centring; wheel/arrows step.
* **Menu-button menu on the window:** Mode >, Bandwidth >, Step >,
  Bookmarks >, Span >, Display..., Source..., Record....
* **Dialogue windows (not one giant side panel):** *Radio* (mode, width,
  squelch, de-emphasis, AGC, stereo, low-pass, RDS), *Source* (gain
  slider + AGC, sample rate, PPM, bias-T, offset), *Display* (FFT size and
  rate, window function, averaging, peak hold, colour map, min/max),
  *Bookmarks* (lists), *Recorder*, *Scanner*.
* **Iconbar:** click opens the receiver; Menu gives Info, Choices..., Quit.

## 6. Phased plan

Every phase ends with something runnable on the Pi and a check against the
trace log, in the way the USB/audio work was done.

### Phase 0 - foundation (2-3 sessions, mostly invisible)

* **0.1 Experiments that decide the toolchain.** (a) Does the build
  service's Norcroft accept `-fpu vfp`, and does a VFP benchmark run on
  the CM4? (b) A host-tested integer (Q15) FFT microbenchmark on the Pi.
  If VFP works without extra modules, float DSP/FFT becomes practical;
  otherwise integer it is (my expectation: a 1024-point integer FFT at
  ~0.1-0.3 ms, to be measured). GCCSDK's hard-float path exists (see
  `~/Development/hello`) but needs the SharedLibs and ARMEABISupport
  modules on the target, so it is a last resort for a program meant to be
  shared.
* **0.2 Split `SpecView.c`** into: receiver state (frequency, mode, width,
  gain, volume, squelch), source/stream control (open, retune, watchdog),
  spectrum (FFT and display data), and UI (windows, icons, events).
  *Done when* the trace of a fixed I/Q replay is identical before and after.
* **0.3 App skeleton:** `!Sprites`, iconbar icon and menu, `Messages`,
  Choices read/write (frequency, mode, width, gain, volume, squelch,
  window position), generated `Templates` (or the code-built equivalent),
  fix the TONE label.
* **0.4 File source + baseband recorder stub:** play a `.iq` file through
  the same read path; record the raw stream to one. *Done when* a recorded
  FM station plays identically from file.

### Phase 1 - a radio you can drive (3-4 sessions)

* Frequency entry: per-digit click/keys, typed entry, step menu, per-mode
  snap interval.
* Volume slider, mute; squelch slider; SNR/S-meter; gain slider with dB;
  status bar (mode, span, lead, underruns, CPU from the trace counters).
* **NFM and AM demodulators** with NCO + FIR channel filter and per-mode
  default widths; host-tested with synthetic signals first, as `Dsp.c` was.
* Bookmarks: a Bookmarks menu over a text list in Choices; add/remove.
* *Done when:* you can type 118.300 and hear AM airband with squelch, tune
  PMR/marine NFM, and the settings come back after a restart.

### Phase 2 - waterfall and a real spectrum (3-4 sessions)

* Integer FFT with 512/1024/2048 points, Nuttall or Blackman-Harris
  window, log-magnitude by table, averaging and peak hold; FFT frame rate
  setting.
* Waterfall sprite + colour maps; plot with translation tables so any
  screen depth works; update only as fast as the budget allows.
* Axes: frequency labels, dB scale; zoom/min/max sliders; resizable window.
* VFO band overlay; click/drag to tune and to set bandwidth; bookmark
  labels above the plot; centre vs normal tuning.
* *Done when:* a 1024-bin waterfall runs at >=10 fps at <=15% total CPU
  with zero underruns (trace), and clicking a FM station tunes it.

### Phase 3 - more radio (3-5 sessions, in this order)

1. SSB/CW/DSB (HF and amateur bands, through the V4's up-converter --
   first prove the HF path at all: lower the UI's 24 MHz limit and look
   for a known AM broadcast or time-signal station), AGC attack/decay,
   noise blanker.
2. WFM stereo and de-emphasis options.
3. RDS: station name and radio text in the window.
4. IF noise reduction, IQ correction, low-pass options.

### Phase 4 - tools (3-4 sessions)

* Recorder (audio WAV; baseband), frequency manager with lists and
  import/export, scanner (range, interval, level, linger), band-plan
  files (UK FM/air/marine/amateur first), full keyboard shortcuts.

### Phase 5 - stretch

* **Listen inside a wide view:** channel extraction from a 1.2-2.4 MSPS
  stream with the NCO path, bigger reads (the DMA allocation limit above
  192 KB is untested) and/or a mode that drops to 240 kSPS for listening.
* **`rtl_tcp`-compatible server** so a PC (SDR++, GQRX) can use the dongle
  over the network; or an audio network sink.
* Packaging (RiscPkg via PackTools), a manual (StrongHelp/HTML via XMLMan),
  a second VFO.

## 7. Risks and things to verify early

| risk | how we find out |
|---|---|
| Mouse wheel may not reach a plain window under the Wimp | a 20-line test on the Pi in Phase 0; fall back to keys + click digits |
| Waterfall plot cost under each screen depth | Phase 2 prototype, measured with the trace's `disp_cs` |
| Integer FFT precision/scaling for 8-bit input | host test against a double FFT first |
| VFP unavailable or needs extra modules | Phase 0.1 |
| DMA allocation for reads above 192 KB (Phase 5) | try 256/384/512 KB launchers, watch for the 3-second fallback |
| Stalls from the network file system (NAS) during recording | record into a RAM buffer, write in chunks; trace will show it |
| Audio latency makes tuning/scanning feel slow | adaptive lead (idea in `PLAN.md`), and VFO moves inside the span without retuning |
| Scope creep | each phase is shippable; stop after any of them |

## 8. Decisions for you

1. **Order:** is the waterfall (Phase 2) or new modes (Phase 3: airband,
   HF) what you want first after Phase 1? Do you care about FM stereo/RDS?
2. **Templates:** generated from a text spec by a repo script (my
   suggestion), or build the windows in code as now, or draw them on the Pi?
3. **Span:** are you happy with "240 kSPS = listen, 2.4 MSPS = browse" as
   two explicit modes for now?
4. **Hang-risk items:** bias-T (GPIO) is the one thing here that touches
   the code path of the old hangs. Want it at all?
5. **Sharing:** is this for you only, or will other people run it (affects
   how much we invest in packaging, help and the manual)?

## 9. Suggested first milestone ("usable radio", ~6 sessions)

Phase 0.2 + 0.3 + the Phase 1 list: new top pane and layout, typed and
per-digit frequency, volume/mute, squelch, meter, NFM + AM, bookmarks,
Choices persistence, iconbar. No waterfall yet; the existing spectrum moves
into the new window unchanged. That is the smallest step that makes the
program feel like a receiver rather than a test harness.
