# RTLSDRView roadmap: from spectrum-and-audio demo to a usable SDR receiver

Status: decisions taken 2026-09-30 (see section 1); Phase 1 done and tagged
`v0.5-nfm-am-ui`; Phase 2 (USB, LSB, CW and the HF path) first build written. It is
meant to be cut up, reordered and argued with.

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

## 1. Decisions taken (2026-09-30) and the resulting order

* **New modes come before the waterfall.** NFM and AM first (airband,
  marine, PMR, amateur VHF/UHF), then SSB/CW/DSB with AGC (HF and amateur
  bands through the V4's up-converter), then the waterfall.
* **Windows: a table-driven UI builder in C on DeskLib**, not a generated
  `Templates` file and not hand-drawn windows. Controls are described in
  one declarative table (kind, position, label, action) and a small
  builder creates the icons; sliders use DeskLib's `Slider`. Reasons: it is
  the approach already proven on the Pi (a hand-built window block is how
  the first window was made, and binary template generation would add a
  format risk that costs hardware round trips), it keeps the layout in
  version-controlled source, and it can be converted to a `Templates` file
  later if packaging or translation ever matters.
* **Two span modes stay:** *listen* (240 kSPS, lossless, stall-proof) and
  *browse* (960 k-2.4 MSPS, wide view, audio best-effort).
* **Bias-T stays on the roadmap, deferred.** It touches the GPIO code path
  of the old hangs; not now.
* **Audience: just the author, for now.** So no investment yet in
  packaging, the manual, interactive help or translation; revisit if that
  changes (Phase 6).

Order: Phase 1 (NFM, AM + the controls to drive them) -> Phase 2 (SSB, CW,
DSB, AGC) -> Phase 3 (drive it well: bookmarks, step list, meters) ->
Phase 4 (waterfall and real spectrum) -> Phase 5 (WFM stereo/RDS, recorder,
file source, scanner) -> Phase 6 (stretch).

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
  is an open experiment (section 7).
* **The Wimp redraws on demand, not every frame.** The waterfall is a
  sprite we update and invalidate; we control the redraw rate. Sliders
  (DeskLib `Slider`) are drawn in the redraw loop, so the plot window needs
  the usual user-redraw care.
* **Two span modes, because of the USB pipe.** 240 kSPS is the *listening*
  mode (lossless, stall-proof, 0.9 s latency). 960 k-2.4 MSPS are *browse*
  modes (wide view; ~27-100 ms per transfer, so audio is best-effort,
  measured in `docs/logs`). The UI should say which one you are in.
  Listening *inside* a wide view is Phase 6.
* **Tuning latency.** The audio lead makes a retune take ~1 s to be heard.
  Moving the VFO inside the current span needs no retune at all (an NCO
  shift), which is another reason to want a VFO rather than only "tune the
  dongle".
* **Memory.** WimpSlot is 640 KB; big buffers already live in the RMA. A
  1024-2048-bin waterfall history and recording buffers may want a dynamic
  area.
* **No Templates editor in this environment.** So windows are built by a
  table-driven builder in C (decision log, section 8): layout lives in one
  declarative table in source, which is reviewable and needs no binary
  resource format. A generated `Templates` file stays possible later.
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
trace log, in the way the USB/audio work was done. DSP is written and
tested on the host first (synthetic signals, as `Dsp.c` was), then run on
the Pi.

### Phase 0 - foundation (folded into Phase 1 as needed)

* A **receiver state** module (frequency, mode, per-mode bandwidth and
  step, squelch, volume) so the window code stops owning it.
* A table-driven control builder and the first **Choices** persistence
  (last frequency, mode, bandwidth, step, squelch, volume).
* Still to do before the waterfall: the `SpecView.c` split, a **file
  source**, and the floating-point/VFP experiment (section 7).

### Phase 1 - NFM and AM, and the controls to drive them (in progress)

* DSP: a shared front end (recentre, optional CIC, FIR decimate to
  48 kSPS), a mode-specific channel filter, NFM discriminator, AM envelope
  with carrier-normalised AGC, audio high/low-pass, squelch (noise-based
  for FM, carrier-level for AM), volume; signal level for a meter.
* Controls: mode buttons, a typed frequency field (MHz) with Up/Down key
  stepping, F-/F+ and a step cycle button, bandwidth -/+, squelch -/+,
  volume -/+ and mute, a signal meter.
* *Done when:* you can type 118.300 and hear AM with squelch, tune NFM
  (PMR/marine/amateur), switch back to WFM, and the trace shows no
  underruns.

**Status (2026-09-30): first build written and on the NAS, not yet run on
the Pi.** Built: `Rx.c` (mode dispatcher and the NFM/AM path),
`Receiver.c` (state, per-mode bandwidth/step, Choices persistence),
`Ui.c` (table-driven control panel), and `Dsp.c` refactored so the WFM
path and the new front end share one CIC (verified bit-identical to the
old WFM output on five reference runs). Host-tested on synthetic signals
at 240 kSPS and 2.4 MSPS: AM tone level within ~8% of ideal whatever the
carrier strength or a +-3 kHz offset, AM audio response 200 Hz-3 kHz flat
within 1 dB (-3 dB at ~4.5 kHz, cut off by the 10 kHz channel), adjacent
AM channel at +9 kHz rejected by 63 dB; NFM tone level within 4% with
adjacent 12.5/25 kHz channels down 32 dB (5x stronger interferer, the FM
capture effect doing part of the work); noise squelch muting pure noise
and opening within ~70 ms of a signal; output identical for any input
chunk size (16384 down to 2 bytes) and for 44.1/48/96 kHz audio.
The per-sample cost estimate on the Pi is ~3% CPU (to be measured with the
trace). Squelch thresholds are calibrated on synthetic noise and will need
adjusting by ear on real signals.

### Phase 2 - SSB, CW, DSB (and the HF path)

* NCO mixer and offset tuning (keeps the signal away from the dongle's DC
  spike and puts a CW tone at a chosen pitch), USB/LSB/DSB/CW channel
  filters, AGC attack/decay, optional noise blanker.
* **Prove the HF path** on hardware: lower the UI's 24 MHz limit and look
  for a known AM broadcast or time-signal station through the V4's
  up-converter (that code exists but has never been exercised).
* Persist settings per mode.

**Status (2026-09-30): USB, LSB and CW built, host-tested, on the NAS; not
yet run on the Pi. DSB and the noise blanker are not done.** How it works:

* The dongle is tuned 45 kHz *below* the dial frequency in these modes
  (`receiver_hw_target()`), so the signal sits off the DC spike. Inside
  `Rx.c` a 2048-entry sine-table oscillator at 240 kSPS shifts the wanted
  passband to 0 Hz (USB: 300 Hz..300 Hz + bandwidth above the carrier; LSB:
  the mirror image; CW: centred on the carrier), then the usual /5 stage,
  a new /4 stage to 12 kSPS, and a real low-pass FIR of up to 511 taps
  (transition 300 Hz for SSB, 150-160 Hz for narrow CW: Blackman, at 12
  kSPS). A second oscillator at 12 kSPS shifts the result up to the audio
  centre (USB/LSB: the middle of the passband; CW: 700 Hz pitch) and only the
  real part is kept, which is what removes the unwanted sideband. Then an
  AGC (fast attack ~1.3 ms, decay ~340 ms SSB / ~170 ms CW, gain limited to
  ~36 dB so empty channels do not roar), a x4 polyphase interpolator back to
  48 kSPS, and the common squelch/volume/resampler stage.
* **Tuning is instant.** The dial moves inside the span (10..80 kHz above
  the dongle's frequency) by changing the first oscillator's rate: no
  retune, no audio gap, phase continuous. A retune only happens when the
  dial leaves that window or the mode changes between "centred" (NFM, WFM,
  AM) and "offset" (USB, LSB, CW). Step sizes now start at 10 Hz;
  defaults USB/LSB 100 Hz, CW 50 Hz. Bandwidths: USB/LSB 1-4 kHz (2.4
  default), CW 100-1000 Hz (400 default, see the limits in the source).
* The red markers on the spectrum are now placed by `rx_channel()`, so they
  show the real passband, off to one side of the centre in SSB/CW.
* The HF path: the lower tuning limit is now 100 kHz (was 24 MHz), using the
  up-converter code in `R82XX.c`. Whether the V4 actually delivers the right
  sideband and signal strength below 28.8 MHz is the thing to check first on
  the Pi. If USB and LSB sound swapped there (or everywhere), set the
  environment variable `RTLSDRView$Mirror` to 1: it flips the I/Q sense in
  the SSB/CW modes and the marker positions.
* Host results (`Rx.c` built with 32-bit `long` and overflow trapping, 240
  kSPS and 2.4 MSPS input): USB/LSB opposite sideband 80+ dB down, tones
  above/below the passband 70+ dB down, CW neighbours 600 Hz away 74+ dB
  down, AGC output within 0.1 dB from input amplitude 12 to 100, 100 Hz dial
  moves produce no click, output bit-identical for any input chunking,
  NFM/AM results unchanged.
* Untested on real signals: how the AGC sounds on speech, the squelch
  thresholds in the SSB modes (level based, -70..-10 dBFS), and the DC-spike
  avoidance in practice.

### Phase 3 - drive it well

* Bookmarks (list in Choices, Bookmarks menu), a proper step list, gain in
  dB, better meters, status bar, keyboard shortcuts, sliders for volume,
  squelch and gain (DeskLib `Slider`).

### Phase 4 - waterfall and a real spectrum

* Integer FFT 512-2048 points, window functions, log-magnitude by table,
  averaging and peak hold; a scrolling sprite waterfall with colour maps;
  axes; zoom/min/max; resizable window; a draggable VFO band with click- and
  drag-to-tune; bookmark labels on the plot; centre vs normal tuning.
* *Done when:* a 1024-bin waterfall runs at >=10 fps at <=15% total CPU
  with zero underruns (trace), and clicking a station tunes it.

### Phase 5 - more radio and tools

* WFM stereo, RDS; IQ file source and baseband/audio recorder; frequency
  manager with lists; scanner; band plans; IF noise reduction, IQ
  correction.

### Phase 6 - stretch

* Listen inside a wide view (channel extraction from a 1.2-2.4 MSPS stream;
  bigger reads; or drop to 240 kSPS for listening); an `rtl_tcp` server;
  bias-T (GPIO, hang-risk path); packaging, manual and help if the audience
  ever grows; a second VFO.

## 7. Risks and things to verify early

| risk | how we find out |
|---|---|
| Mouse wheel may not reach a plain window under the Wimp | a 20-line test on the Pi in Phase 0; fall back to keys + click digits |
| Waterfall plot cost under each screen depth | Phase 2 prototype, measured with the trace's `disp_cs` |
| Integer FFT precision/scaling for 8-bit input | host test against a double FFT first |
| VFP unavailable or needs extra modules | the open VFP experiment (section 7 preamble); integer DSP works without it |
| DMA allocation for reads above 192 KB (Phase 5) | try 256/384/512 KB launchers, watch for the 3-second fallback |
| Stalls from the network file system (NAS) during recording | record into a RAM buffer, write in chunks; trace will show it |
| Audio latency makes tuning/scanning feel slow | adaptive lead (idea in `PLAN.md`), and VFO moves inside the span without retuning |
| Scope creep | each phase is shippable; stop after any of them |

## 8. Decisions log

| date | decision |
|---|---|
| 2026-09-30 | Modes before waterfall |
| 2026-09-30 | Windows built by a table-driven C builder on DeskLib; `Templates` later if ever needed |
| 2026-09-30 | Listen (240 kSPS) and browse (wide) as two explicit span modes |
| 2026-09-30 | Bias-T stays on the roadmap, deferred |
| 2026-09-30 | Personal tool for now: no packaging/manual/help work yet |

| 2026-09-30 | SSB/CW by shifting the passband to 0 Hz and using a real filter (not a phasing filter or FFT), tuned off-centre so fine tuning needs no retune; DSB left for later |

Open: prove the HF path on hardware (Phase 2); whether the waterfall should
wait for a VFP answer; DSB and a noise blanker.
