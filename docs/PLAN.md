# RTL-SDR on RISC OS — project plan

## Context

RISC OS has no RTL-SDR support and no libusb port. RTL2832U dongles enumerate
as a vendor-specific USB interface (class 0xFF), so none of RISC OS's built-in
class drivers (audio/HID/mass-storage/serial) apply — every existing RTL-SDR
tool (`rtl_sdr`, GQRX, etc.) exists only because libusb lets it claim the
device directly, and that path doesn't exist on RISC OS. Confirmed against
RISC OS Open forum history: a 2015–2016 thread ("Software Defined Radio",
riscosopen.org/forum/forums/5/topics/3392) covered the FUNCube Pro+ (which
works via USB Audio + HID) and noted RTL dongles specifically as unsupported
because they need raw bulk/vendor access, which nobody built.

The fix is a native RISC OS app that talks to the dongle's raw USB endpoints
directly through **DeviceFS** (RISC OS's actual generic USB access mechanism
— every enumerated device gets a DeviceFS entry, whether or not any class
driver recognises it) and reimplements the relevant slice of `librtlsdr`'s
device logic against that instead of libusb.

Target hardware for v1: **confirmed as an RTL-SDR Blog V4 dongle** by a
real `tools/probe.bas` run on real hardware — it shows up in `*USBDevices`
as device No. 6, description "RTLSDRBlog Blog V4". V4 boards use the
**R828D** tuner plus a built-in HF upconverter, not plain R820T2 — this
plan originally scoped v1 to generic R820T2 (the most common chipset in
general) but has been retargeted at V4 specifically now that real hardware
is in hand. Mainline `librtlsdr` needs the `rtlsdr-blog` fork's patches for
V4's tuner init/I2C and upconverter mixer-switching; that fork is the
reference for milestone 3, not mainline `osmocom/rtl-sdr`. Other tuner
chips (E4000/FC0012/FC0013/FC2580, plain R820T2) are out of scope for v1.
Target OS: RISC OS 5.30 on a real Raspberry Pi (the same version the
user's `RISCOSQEMUA72` emulator boots, though that emulator cannot be used
for this project — see Testing below).

## Approach

**Standalone Wimp application** (starts as a plain command-line diagnostic
tool for milestones 1–4, becomes a Wimp app for the v2 GUI). Built with the
**Norcroft/DDE toolchain via `build.riscos.online`**, matching the user's
existing `riscos-rdpclient` project rather than standing up GCCSDK: C89
only, `-apcs 3/32bit -za1 -Otime` compile flags, link against `C:o.stubs-32`.
(Note: the user also has a working local GCCSDK toolchain, proven in the
`hello` project at `~/Development/hello` — an option if Norcroft/DDE proves
inconvenient for iterative driver work, but Norcroft/DDE was the deliberate
initial choice to reuse `riscos-rdpclient`'s already-solved toolchain
gotchas.)

## The actual RISC OS USB API (verified against ROOL's own source)

A first pass at this plan used the `riscos-usb.com` "USB Device Drivers"
book (2002, Simtec), which describes USBLib SWIs (`&5635x` range), URBs,
and pipe objects. **That API is for RISC OS's old/podule-specific USB
stack, not what real RISC OS 5 exposes.** The authoritative current doc is
`build/Doc/USB` in ROOL's own USBDriver source
(`gitlab.riscosopen.org/RiscOS/Sources/HWSupport/USB/USBDriver`, versions
0.90–0.95, explicitly "RISC OS 5 and IYONIX pc"), linked directly by a ROOL
core developer (Jeffrey Lee) in a 2018 forum thread, plus the RISC OS PRM's
`DeviceFS_CallDevice` (SWI `&42744`) chapter for the exact register
convention. The real model is simpler than URBs:

- **Every enumerated USB device gets a DeviceFS entry automatically**
  (`USBnnn`, e.g. `USB7`), regardless of class recognition. No "unclaimed
  device pool" or claiming step exists or is needed.
- **Control transfers** go through `DeviceFS_CallDevice` (SWI `&42744`),
  "USB Control Request" reason code:
  - R0 = `(1<<31)+0`
  - R1 = pointer to device name string, e.g. `"usb7"`
  - R3 = `bmRequestType | (bRequest<<8) | (wValue<<16)`
  - R4 = `wIndex | (wLength<<16)`
  - R5 = pointer to data buffer
  - R6 = `0` for simple blocking-until-complete (simplest mode; bit 0/1 of
    R6 select polling or callback modes for later, non-blocking use)

  This is a direct analog of `libusb_control_transfer` — implemented in
  `app/!RTLSDR/c/RTLSDR.c` as `usb_ctrl_transfer()`.
- **Bulk streaming** is opening a stream on `devices#endpointN:usb7`
  (`bulk` special field, `nosleep`/`sleep` to control blocking behaviour)
  with `OPENIN`/`OPENOUT`, then reading with `OS_GBPB` for block reads —
  the same mechanism the existing `SerialUSB:` module already uses for
  USB-serial chips, just pointed at our device's own bulk endpoint. The
  `Check Buffer Space` `DeviceFS_CallDevice` extension lets the app poll
  for available data without blocking a Wimp_Poll loop.
- Device discovery is done by shelling out to `*USBDevices` and
  text-matching the human-readable **description** (e.g. "RTLSDRBlog Blog
  V4"), not hex VID/PID — confirmed by a real `tools/probe.bas` run: VID/PID
  don't even appear in `*USBDevices` output, and device numbering (the
  "No." column) starts at **1**, not 0. `*USBDevInfo`/`*USBConfInfo` (for
  register-level VID/PID and endpoint layout) exist but their exact output
  format is still unconfirmed — probe.bas v2 now calls them defensively
  (`ON ERROR LOCAL`) after identifying the device by description first, so
  a bad call reports an error instead of aborting the whole script (v1 hit
  exactly that: it looped device numbers from 0, RISC OS's numbering
  starts at 1, and the resulting syntax error killed the unguarded script).

## Technical grounding from librtlsdr (osmocom/rtl-sdr)

### Device identification
Known VID/PID table from `librtlsdr.c`'s `known_devices[]` (`0x0bda:0x2832`
/ `0x0bda:0x2838`) turned out not to be how we actually find the device on
RISC OS — see "Device discovery" above, which matches on the `*USBDevices`
description string instead. **Confirmed by a real `*USBDevInfo 6` run:**
Vendor ID `0BDA`, Product ID `2838` — matches `h/RTLSDR.h` exactly. Full
confirmed `*USBDevInfo` field list/format (useful for milestone 2, and for
a stricter check than description-text matching later):
```
USB release   : 0200
Device class  : 00
Device subclass : 00
Device protocol : 00
Max packet size : 40
Vendor ID     : 0BDA
Product ID    : 2838
Device ID     : 0100
# of configs  : 1
Manufacturer  : 'RTLSDRBlog'
Product       : 'Blog V4'
Serial number : '00000001'
Speed         : High
```
`*USBConfInfo` (interface/endpoint layout — needed for milestone 4's bulk
endpoint number) hasn't produced output yet; `tools/probe.bas` v2 had a
bug (see its changelog) that stopped it running before reaching that call.
v3 fixes it — next run should get it.

### Register I/O
`librtlsdr` does all register/I2C access via `libusb_control_transfer()`:
```
read:  bmRequestType=0xC0 (vendor|IN),  bRequest=0, wValue=addr, wIndex=block<<8
write: bmRequestType=0x40 (vendor|OUT), bRequest=0, wValue=addr, wIndex=(block<<8)|0x10
```
Block IDs: `DEMOD=0, USB=1, SYS=2, TUN=3, ROM=4, IR=5, I2C=6` (see
`h/RTLSDR.h`). I2C reg read/write (for the tuner) is `rtlsdr_{read,write}_array`
against the I2C block. `rtlsdr_read_reg`/`rtlsdr_write_reg` are ported as
thin wrappers around `usb_ctrl_transfer()` in `c/RTLSDR.c`.

### Device/baseband init sequence — DONE, ported verbatim
`rtlsdr_init_baseband()`, `rtlsdr_demod_read_reg`/`write_reg`, and
`rtlsdr_set_fir()` (FIR coefficient packing) were fetched as literal
source from the `rtlsdr-blog` fork and transcribed line-for-line into
`c/RTLSDR.c` — not a paraphrase. Demod registers use a different address
encoding than plain `rtlsdr_read_reg`/`write_reg`: `wValue=(addr<<8)|0x20`,
`wIndex=page` (or `0x10|page` for writes), and every demod write is
followed by a dummy read (upstream does this too, unexplained, reproduced
anyway). Sequence: USB config → demod power-on/soft-reset → clear spectrum
inversion/DDC/IF registers → write FIR coefficients (20 bytes, page 1
0x1c-0x2b) → enable SDR mode/zero-IF, disable AGC/PID → I2C-repeater-gated
tuner probe (`rtlsdr_probe_tuner()`): try R820T's I2C address, then
R828D's, at the same check register, matching whichever responds `0x69`.
Built (`build/RTLSDR,ff8`), not yet run on hardware.

**Bug caught before it shipped:** an earlier draft of `h/RTLSDR.h` had
both `R820T_I2C_ADDR` and `R828D_I2C_ADDR` as `0xA0` — wrong, that's
`EEPROM_ADDR` (an unrelated I2C device). Verified against the actual
`#define` lines in the fork's `tuner_r82xx.h`: R820T is `0x34`, R828D is
`0x74` — different 7-bit I2C addresses, same check register/value (`0x00`
→ `0x69`). Fixed before the code was ever run.

### Tuner (R828D) — V4-specific, from the `rtlsdr-blog` fork
Confirmed real hardware is R828D via `rtlsdr_open()`'s own detection
logic: R820T and R828D respond identically at their respective I2C
addresses, so they're told apart only by which one answers `0x69` — not
by anything USB-descriptor-visible. Beyond the shared init array (no
separate R828D register table), V4-specific behaviour pulled from the
fork's `tuner_r82xx.c`/`librtlsdr.c` (not yet implemented, needed for the
tuning/PLL work after milestone 3):

- **Crystal frequency**: `R828D_XTAL_FREQ` = 16MHz is used for *generic*
  R828D boards, but `rtlsdr_open()` specifically skips setting it when
  `rtlsdr_check_dongle_model(dev, "RTLSDRBlog", "Blog V4")` matches — V4
  keeps the RTL2832U's default 28.8MHz xtal instead. Dongle-model
  detection itself is trivial: it just string-compares the USB
  manufacturer/product descriptor strings we already have from
  `*USBDevInfo` (`'RTLSDRBlog'` / `'Blog V4'`).
- **HF upconverter**: below 28.8MHz, the requested frequency gets
  `+28.8MHz` added before the normal PLL tuning math runs — the upconverter
  is otherwise transparent to the tuning code.
- **GPIO band switching** (register-level, SYS block — same
  `GPD`/`GPOE`/`GPO` registers already in `h/RTLSDR.h` from the bias-tee
  work): GPIO 5 selects the upconverter path (inverted — low for HF, high
  otherwise); register `0x06` bit `0x08` and register `0x05` bits `0x40`/
  `0x20` select which of three RF input paths (HF/VHF/UHF) is active,
  written via `r82xx_write_reg_mask`-style read-modify-write on the tuner's
  own I2C registers (not RTL2832U registers — these go through
  `rtlsdr_i2c_write_reg` at the R828D I2C address).
- **Bias tee**: GPIO 0, same `rtlsdr_set_gpio_output`/`set_gpio_bit`
  mechanism. **EEPROM byte 7 bit `0x02`, if clear, forces bias tee on**
  regardless of software setting — our own milestone 2 dump read byte 7 as
  `0x00` (from `EEPROM[0x06..0x07]: 0000`), so this applies to our actual
  unit. Worth remembering once bias-tee control is added: don't be
  surprised if it's already on.
- IF frequency (`R82XX_IF_FREQ` = 3.57MHz) and the full R82xx PLL/gain
  register set (~28 shadow registers, `set_freq`/`set_mux`) are the same
  for R828D as R820T except for the above — not yet ported, this is the
  next real chunk of work after milestone 3's gate passes.

All of the above constants are already in `h/RTLSDR.h`, ready for that
next chunk; only the actual PLL/`set_freq` logic and GPIO writes remain.

### Streaming
`librtlsdr` streams via bulk endpoint `0x81` (libusb), 256KB buffers, 15 in
flight, via libusb's async submit/callback/resubmit loop. **Confirmed
against real hardware** (`*USBConfInfo 6`): interface 0 has "Endpoint 1,
Bulk IN, 512 bytes" — matches `RTLSDR_BULK_ENDPOINT`/`RTLSDR_BULK_MAXPACKET`
in `h/RTLSDR.h`. This dongle exposes two vendor-class (255/255/255)
interfaces (config name `USB2.0-Bulk&Iso`); interface 0 is the bulk one we
want — interface 1's purpose isn't confirmed yet (presumably an unused
isochronous alternate, given the config name, but not verified). On RISC
OS this becomes an `OPENIN` stream on `devices#endpoint1;interface0:usb6`,
read via `OS_GBPB`. Whether RISC OS's real Pi USB stack sustains the
required ~4.8 MB/s (2.4 MSPS × 2 bytes) through this path is still the
main unverified technical risk in the whole project (see milestone 4) —
knowing the endpoint number doesn't tell us the throughput.

## Phased milestones

1. **Enumerate & identify** — confirm the dongle shows up as its own
   `USBnnn` DeviceFS entry, matched by description text (see above).
   **Gate**: correctly finds the device number.
   **DONE.** Confirmed on real hardware, in the actual compiled C binary
   (not just BASIC): `RTLSDR` found device No. 6, DeviceFS name `usb6`,
   printed "Milestone 1 gate: PASS".
2. **Register read-back proof** — `milestone2_register_probe()` dumps the
   first 12 EEPROM bytes via `rtlsdr_read_reg()` and does a read-twice
   consistency check. **DONE.** Confirmed on real hardware: both reads of
   EEPROM[0x00] returned `1A02`, proving `DeviceFS_CallDevice`-based
   control transfers work and return stable, repeatable data. Full dump:
   ```
   EEPROM[0x00..0x01]: 1A02
   EEPROM[0x02..0x03]: 0264
   EEPROM[0x04..0x05]: D814
   EEPROM[0x06..0x07]: 0000
   EEPROM[0x08..0x09]: 0000
   EEPROM[0x0A..0x0B]: 0200
   ```
   Not independently cross-checked against `rtl_eeprom -d 0` on Linux
   (optional — the self-consistency check already proves the transport
   works; field-level meaning of these bytes, e.g. which are vendor/
   product ID, is unconfirmed since that layout lives in librtlsdr's
   separate `rtl_eeprom` tool, not pulled in). Not a blocker for milestone 3.
3. **Baseband + tuner presence** — `rtlsdr_init_baseband()` (verbatim port:
   USB config, demod power-on/reset, FIR coefficients, SDR/zero-IF mode)
   plus an I2C-repeater-gated probe for R820T then R828D. **DONE.**
   Confirmed on real hardware: R828D responded at I2C `0x74` with check
   value `0x69` — exact match. (A `usb_ctrl_transfer: Bad request` line
   appears in the output from the R820T probe at I2C `0x34` — expected,
   not a bug: nothing answers at that address on R828D hardware, so the
   control transfer itself fails, which is exactly what makes the code
   correctly fall through to try R828D next. One error, in exactly that
   position, confirms the probe-and-fallback logic is working as designed.)
   See below for the research this was built from, including a real bug
   caught and fixed *before* it ever ran (wrong I2C addresses).
3.5. **Full R82XX tuner (init + `set_freq`)** — new files
   `app/!RTLSDR/{c,h}/R82XX.c/.h`, ~600 lines: shadow-register cache,
   I2C read/write with bit-reversal, the `freq_ranges[]` mux table, PLL/
   VCO synthesis (`r82xx_set_pll`), 56MHz filter calibration
   (`r82xx_set_tv_standard`), and V4's upconvert + GPIO band-switching
   (`r82xx_set_freq`). All ported from **literal source read directly
   from the downloaded file**, not WebFetch's summarizer — that
   summarizer proved unreliable for functions this size (one attempt
   silently paraphrased a filter-calibration routine into vague prose;
   another flatly refused, citing an internal quoting-length limit). Two
   things the literal source caught that a summary had already gotten
   subtly wrong or would have missed entirely:
   - `r82xx_read()` **bit-reverses every byte** read back over I2C — a
     real hardware quirk. Missing this would have silently broken PLL
     lock detection and VCO fine-tuning (both decoded from bit-reversed
     register data) while looking like ordinary I2C reads.
   - Masked register writes (`r82xx_write_reg_mask`, used throughout) read
     from a **shadow register cache**, not live hardware — so the driver
     needs its own `regs[]` array kept in sync on every write, not just
     passthrough register access.

   **Built and run once already — caught a real bug on the first try.**
   `milestone_tune_test()` in `RTLSDR.c` calls `r82xx_init()` then
   `r82xx_set_freq()` to 100MHz (VHF FM broadcast). First real-hardware
   run: `r82xx_init()` failed immediately (`usb_ctrl_transfer: Bad
   request`) on its very first call — the init-array write, 27 bytes in
   one control transfer. That's the first multi-byte I2C write anywhere
   in this project; everything before it was 1-2 bytes and happened to
   stay under a limit nobody had hit yet. Root cause, confirmed straight
   from upstream's own `rtlsdr_open()` source: the RTL2832U's I2C bridge
   caps a single burst write at **8 bytes total** (reg byte + up to 7
   data bytes) — `r82xx_c.max_i2c_msg_len = 8`, set at tuner-open time.
   Upstream's `r82xx_write()` chunks for exactly this reason; our port
   originally didn't (dismissed as "small transfers never trigger it,"
   true right up until the first 27-byte write). Fixed: `r82xx_write()`
   now chunks into ≤7-byte pieces, advancing the register address between
   chunks, matching upstream exactly. (Confirmed correct in the same
   pass: our `use_predetect = 0` assumption in `r82xx_sysfreq_sel()` —
   also visible directly in upstream's tuner-open code.)

   **DONE — confirmed on real hardware after the fix.** `r82xx_init()`
   OK (filter calibration code `0x09`, a sane non-edge-case value —
   `r82xx_set_tv_standard`'s loop specifically retries if it lands on
   `0x00`/`0x0F`), tuned to 100MHz, **PLL locked**. The full chain —
   enumerate, control transfers, baseband init, tuner presence, tuner
   init, real-frequency tuning — now works end-to-end on real hardware.
4. **Streaming proof** — **DONE. 25,600,205 bytes/sec sustained (25.6
   MB/s), over 5x the ~4.8 MB/s target (2.4 MSPS × 2 bytes), 125,001 reads
   over 5 real seconds, zero errors.** Final mechanism: raw
   `OS_Find`/`OS_GBPB` (reason 4), reading in 1024-byte chunks — the
   largest single-call size found reliable by an empirical sweep (see the
   "exact boundary" section above; 2048 bytes blocks for ~110s, for
   reasons not otherwise diagnosed, most plausibly an internal DeviceFS/
   driver buffer limit). Getting here from the original plan (`fopen`/
   `fread`, 256KB chunks, `usbtimeout` field) took real, hard-won
   discovery: `fread()` never worked on this DeviceFS stream at any size,
   for reasons never root-caused; `OS_GBPB` itself blocks on request size
   rather than returning immediately as first assumed; and the *actual*
   size ceiling had to be found empirically, not documented anywhere.
   None of that matches the plan's original assumptions about a Wimp-app
   non-blocking design (moot — this is a command-line tool with no
   Wimp_Poll loop to protect) or the `usbtimeout` field bounding blocking
   reads (it doesn't, in practice, for this stream type).
5. **Stretch/v2** — FFT + waterfall Wimp window, demod (FM/AM), gain
   control UI, a real 1024-byte-chunked reader feeding a ring buffer
   instead of a diagnostic dump. Not started.

**All of milestones 1 through 4 are now DONE, confirmed on real
hardware.** This is the actual, meaningful proof-of-concept goal: RTL-SDR
demonstrably works end-to-end on RISC OS, from USB enumeration through
sustained bulk sample streaming at well above the rate a real SDR needs.

## Testing / verification

**QEMU (`RISCOSQEMUA72`) cannot be used for hardware testing** — its
`raspi4b` emulation only wires up `usb-kbd`/`usb-tablet`/`usb-net` on an
emulated DWC2 controller; no `usb-host` passthrough is configured, and its
`DESIGN.md` says real USB passthrough was never a goal for that project.

The real loop: build via `build.riscos.online` → run on the **real RISC OS
5.30 Pi with the dongle physically attached**. Each milestone has its own
concrete pass/fail check, so there's no ambiguity about whether a stage
actually works before moving to the next.

**Current status: ALL of milestones 1 through 4 are DONE**, confirmed on
real hardware. The dongle enumerates, control transfers work, baseband
inits, the tuner is identified, initialized, and tunes to a real
frequency with the PLL locked, sample rate is configured, and sustained
bulk streaming measured at 25.6 MB/s — over 5x the required rate. The
detailed run-by-run history below is kept as-is for the record; see the
"Streaming proof" entry in Phased Milestones above and the "Next step"
section at the end of this document for where things stand now.

Bugs caught and fixed along the way: RISC OS text files are CR-terminated
(`find_device()`); `_kernel_oscli()` returns `int`, not an error pointer
(`run_and_capture()`); `h/RTLSDR.h` had both tuner I2C addresses wrong
before any of this ran (`0xA0`/`0xA0` → correct `0x34`/`0x74`);
`R82XX.c`'s `r82xx_write()` didn't chunk multi-byte I2C writes (RTL2832U's
I2C bridge caps a single burst at 8 bytes), caught on its very first
real-hardware call — the 27-byte init-array write.

**Milestone 4's first real run hung the whole machine.** Not the
streaming code itself — it never got there. `r82xx_init()` printed OK,
then the machine froze solid (mouse still moved; everything else,
including other windows, stopped responding) somewhere inside the
following `r82xx_set_freq(100MHz)` call, with no further output. Two
real possibilities, not yet distinguished: a genuine CPU-bound infinite
loop in the port (hand-traced the sigma-delta PLL loop for this exact
frequency and it terminates in ~15 iterations, so if it's this loop,
something differs from that trace in practice, not in the math on
paper), or a RISC OS/hardware-level SWI call that blocked forever despite
requesting the "return on completion, failure, or timeout" mode (R6=0) —
plausible if a bad I2C transaction wedged the RTL2832U's I2C bridge
rather than failing cleanly. GPIO register access (`GPD`/`GPOE`/`GPO`,
for the V4 upconverter switch) is the other prime suspect: it's the one
genuinely new code path in this call that nothing before it in the
project has ever exercised.

A third real possibility: this exact `r82xx_set_freq(100MHz)` call
**already succeeded once**, in the run before milestone 4's code was
added — nothing in `R82XX.c` changed between that success and this hang.
The dongle wasn't power-cycled between attempts, so this run started from
an *already-tuned, already-locked* tuner, not the fresh power-up state
upstream's `rtlsdr_open()` sequence assumes. If some part of the driver
implicitly relies on starting from a known-clean state, repeated runs
without a power cycle could behave differently from run to run — worth
keeping in mind if the instrumented build behaves inconsistently across
attempts.

**Two runs of the instrumented build both localized the hang to the exact
same spot**: entering the sigma-delta loop inside `r82xx_set_pll()`'s
*second* call (the one `set_freq` makes for the real 103.57MHz IF-shifted
target — the *first* call, `set_tv_standard`'s 56MHz filter calibration,
completes fine both times, including its own PLL lock). Printed
"about to enter sigma-delta loop" but never "sigma-delta loop done", and
never the 64-iteration abort message either. That loop has **zero I/O in
it** — pure arithmetic — so a genuine infinite spin there would have to be
either a real logic bug (my by-hand trace, using the same known inputs,
says it terminates in ~15 iterations — so either that trace is wrong, or
the real runtime values differ from what it assumed) or something more
exotic like a division landing on zero (`n_sdm` can only reach zero via
32-bit wraparound after ~31 left-shifts, which the loop's own logic
should prevent by breaking out once `n_sdm>=0x8000` at ~iteration 15 —
but if that break is somehow never reached, this becomes possible, and
divide-by-zero in the C runtime's software division routine could itself
hang rather than error cleanly).

**Response**: rather than reason further from silence, added prints of
the actual computed values (`freq_hz`, `mix_div`, `vco_freq`, `nint`,
`vco_fra`, `pll_ref_khz`) right before the loop, plus a per-iteration
print of `n_sdm`/`vco_fra` inside it — flushed immediately, so even a
genuine hang shows the last real values instead of nothing. Also kept
the 64-iteration hard cap from the previous round. Rebuilt
(`build/RTLSDR,ff8`). Earlier instrumentation (`TRACE()` at every
register/I2C/GPIO step in `set_mux`/`set_vga_gain`/`set_pll`/`set_freq`)
already did its job — it's what localized the hang to this exact loop in
the first place — and stays in place.

**Before trying again: power-cycle the Pi and unplug/replug the dongle.**
If the RTL2832U's own I2C bridge got wedged (not just RISC OS), a plain
reboot of the Pi may not clear the dongle's internal state. This is now
the third hang at the same location — treat it as a real, reproducible
bug, not a fluke.

**Two more runs, and this time the picture changed completely.** With the
extra per-iteration tracing in place, the sigma-delta loop this time
**completed correctly** — all 14 iterations, values matching the by-hand
trace above almost exactly (`vco_fra` 31040→2240→...→2, `n_sdm`
2→4→...→32768). PLL lock-check succeeded too. `set_freq` resumed, wrote
`open_d`, determined the band, wrote `cable_2_in` to `0x06` — then froze
again, this time much later: **right at the first-ever GPIO register
access anywhere in the project** (`rtlsdr_set_gpio_output()`, SYS-block
`GPD`/`GPOE` — registers `0x3004`/`0x3003` — for the V4 upconverter-switch
GPIO via `rtlsdr_set_bias_tee_gpio()`).

Two things matter here. First: **the actual tuning math is now proven
correct** — the sigma-delta loop and PLL lock both work exactly as
designed when they run at all, which retroactively suggests the earlier
two hangs (frozen right at loop *entry*, with identical code) weren't a
logic bug in the loop either — the hang point moving between otherwise
identical runs points at something non-deterministic (hardware/USB-timing
related) rather than a pure C bug. Second: every hang so far has occurred
somewhere in the vicinity of code that's never run in this project before
this point — first the sigma-delta loop's *first-ever real invocation*
(vs. the calibration call, which always worked), now GPIO register access
specifically. That's circumstantial, not proof, but it's a pattern.

**Response**: disabled the `rtlsdr_set_bias_tee_gpio()` call in
`r82xx_set_freq()` (commented out with a full explanation in the code) —
it only selects the V4 upconverter's RF input path, which doesn't affect
whether the PLL locks (already proven, and already succeeds before this
call ever runs) and doesn't affect milestone 4's throughput test (which
only counts USB bytes, doesn't care about RF path correctness). The
remaining band-switch writes (registers `0x05`, via the well-proven I2C
mechanism, not GPIO) still run. This makes forward progress without
retrying the specific code that's hung the machine multiple times, and
turns the GPIO issue into its own separable follow-up rather than a
blocker. Rebuilt (`build/RTLSDR,ff8`).

**With GPIO skipped, the fifth hang landed somewhere new again** — this
time between two adjacent `TRACE()` calls with *nothing* between them but
a closing brace. No function call, no comparison, nothing that could
plausibly block. Combined with the freeze location moving between runs on
otherwise-identical, now-verified-correct logic, this stopped looking like
a bug tied to any specific line. New working theory: tuner init is by far
the densest burst of back-to-back USB control transfers anywhere in this
project — dozens of them, zero pacing between any of them — and nothing
before this point in the codebase has ever generated that much rapid-fire
USB traffic at once. **Response**: added a ~10ms busy-wait
(`r82xx_pace()`) after every I2C transfer in the two central primitives
(`r82xx_i2c_write`/`r82xx_i2c_read`), which every higher-level register
write/read in this driver funnels through. This is a real hypothesis
(RISC OS's USB stack or the RTL2832U itself not tolerating an unpaced
burst), not a proven fix — if it doesn't help, it should be removed rather
than left in as unexplained cargo-cult code. Rebuilt (`build/RTLSDR,ff8`).

### The freezes stopped — but likely not because of the pacing fix

**The very next run went all the way through milestone 3.5 with no
freeze at all** — full trace, "Tuned to 100MHz: PLL locked", "Tune test:
PASS" — and continued into milestone 4. But the user changed something
else at the same time: **they ran the built `RTLSDR` directly (double-
click / `*Run`) instead of inside a TaskWindow**, specifically because the
previous five attempts had all been run in a TaskWindow. This is a
critical, likely-more-important variable than the pacing change, and it
was varied at the same time, so the two can't be cleanly separated from
this one run alone. **Operational rule going forward: run this build
directly, never inside a TaskWindow**, until/unless proven that TaskWindow
itself wasn't the issue. If TaskWindow really was the cause, the earlier
"hang" investigation chasing specific register/GPIO/pacing culprits inside
`R82XX.c` may have been chasing a symptom of the test harness, not a bug
in the driver — worth keeping in mind, though the pacing and GPIO-skip
changes are harmless to leave in regardless.

### Milestone 4's first real result: stream opens, but reads block far longer than the requested timeout

Milestone 4 got its first genuine data: `fopen()` on the bulk endpoint
path succeeded, but the first 256KB-chunk `fread()` **blocked far past**
the `usbtimeout2000` (2 second) special field — recovered only by the
user pressing Escape after roughly a minute, not by a real timeout firing.
Reported "0 bytes in 63.3s" — that number is measuring "time until Escape
was pressed", not a genuine timeout duration. **This means the
`usbtimeout` special field is not bounding the read the way assumed** —
most likely because `fread()`'s C-library buffering retries internally to
fill the entire requested 256KB before giving up, rather than surfacing a
single short underlying transfer. Still unknown: whether the bulk endpoint
has *any* data flowing at all.

**Response**: added a small 64-byte probe read before the full chunked
loop — small enough that it needs at most one underlying transfer to
either succeed or genuinely exhaust the timeout, answering "is data
flowing at all" quickly instead of over another long wait-and-Escape.
Added `fflush()` after every progress line in the loop too, so partial
progress stays visible even if a read blocks again. Note for whoever runs
this next: **if it blocks again, Escape safely recovers to the desktop**
(confirmed) — this is a much lower-risk situation than the earlier full
machine freezes, not the same category of problem. Rebuilt
(`build/RTLSDR,ff8`).

### The probe confirmed zero bytes ever, not just a timeout bug

Ran the probe build directly (not TaskWindow) — no freeze, but still had
to Escape out after about a minute. This time the program handled it
gracefully: "Probe: got 0 of 64 bytes", printed the FAIL message, and
exited cleanly via its normal return path (not just an interrupted hang).
**Real new information**: even a 64-byte read gets *zero* bytes over a
full minute — not a timeout-field bug hiding real throughput, but the
bulk endpoint genuinely never producing any data at all.

Traced back the cause: **`rtlsdr_set_sample_rate()` had never been
called anywhere in this project.** Real `rtl_sdr`/`rtlsdr_open()` always
calls it before reading — it programs the RTL2832U's resample-ratio
registers (demod page 1, `0x9f`/`0xa1`); without it, the resampler is left
at whatever its power-up-default state is, quite plausibly not a valid,
running configuration. Reading the actual `rtlsdr_open()` source also
turned up a second, more fundamental gap: **a tuner-specific post-init
step, run once right after tuner detection and *before* `r82xx_init()`,
that this port had entirely skipped.** For R820T/R828D it does four
things — and the first one matters a lot: it **overrides**
`rtlsdr_init_baseband()`'s zero-IF setting (demod `1/0xb1`, `0x1b` →
`0x1a`) back to real-IF mode, since R82xx tuners use a genuine 3.57MHz IF,
not zero-IF. A demod left configured for the wrong IF mode has no reason
to ever produce valid samples — a strong independent candidate for "zero
bytes, always," on top of the missing sample-rate setup.

Both gaps needed the actual source, not memory or a summary, to fix
correctly — `rtlsdr_set_sample_rate()`'s resample-ratio math overflows
32-bit integer arithmetic by a factor of ~28000 unless done in `double`
(confirmed from upstream's own `TWO_POW(n)` macro, literally
`(double)(1ULL<<n)` — easy to miss and port as plain integer math, which
would have silently produced a wrong ratio rather than an obvious error).

**Response**: ported both pieces verbatim — `rtlsdr_tuner_postinit()`
(the four-step override block, minus the EEPROM-triggered forced
bias-tee, which goes through the same GPIO mechanism already disabled for
hang risk) now runs right after the I2C repeater is enabled and before
`r82xx_init()`; `rtlsdr_set_sample_rate(dev, 2400000)` now runs once
tuning succeeds, before milestone 4 opens the stream. Rebuilt
(`build/RTLSDR,ff8`). This is a real, sourced fix for a concrete gap, not
a guess — but "correctly transcribed" isn't "confirmed on hardware" until
the next run actually shows non-zero bytes.

### Still zero bytes after the sample-rate/tuner-postinit fix

Ran the fixed build — no freeze, "Setting sample rate to 2.4 MSPS... Sample
rate set" printed correctly, milestone 4 opened the stream fine, but the
64-byte probe still returned **0 bytes**. So the device-init gaps that
got fixed were real and worth fixing, but they weren't the (whole) answer
to "why no data" — or there's another gap of the same kind still missing.

Rather than keep guessing at more `rtlsdr_open()` init-sequence gaps one
at a time, `tools/probe_stream.bas` now tests a genuinely different
hypothesis: is this specific to the C/stdio `fopen`/`fread` path, or does
the same read via raw `OS_GBPB` (reason 4, matching the DeviceFS doc's
own BASIC example — the exact mechanism `tools/probe.bas` already proved
works for `*USBDevices`/`*USBDevInfo` output, just not yet tried against
a live bulk stream) also get 0 bytes? If raw `OS_GBPB` also gets nothing,
the problem is upstream of stdio entirely (device or USB stack) and more
init-sequence work is the right next move. If it gets real data where
`fread()` didn't, the bug is specific to how the C code opens/reads the
stream, not the device's actual state — a very different, much narrower
fix. No rebuild needed for this one, just run it on the Pi (after running
`RTLSDR` first, so the dongle is already tuned and sample-rate-configured
from that run).

**v1 of that script had its own bug**, caught immediately on the first
run: `SYS "OS_GBPB",... TO ,,unread%` only has two commas, which
positionally captures R2 (the updated buffer pointer — a raw memory
address) into `unread%`, not R3 (the real "bytes not transferred" count)
— needs a third comma. The reported "got -39096 bytes" was that bug, not
a real result; ignore it. One genuinely useful thing survived the bug
though: the call returned in ~80ms, nowhere near `fread()`'s ~1 minute
block — suggesting `OS_GBPB` reason 4 may not wait for data to arrive the
way `fread()` does, and might just report what's available at the instant
it's called. **v2** fixes the register capture and polls in a loop for up
to 3 seconds instead of trying once, so data gets a real chance to show
up if it's going to arrive at all, without conflating "didn't wait long
enough" with "never arrives".

### Confirmed: real data flows. The bug was in fread(), not the device.

`probe_stream.bas` v2's very first attempt got **16 of 16 bytes**, hex
values including repeated `0x7F` — exactly the DC-centred value expected
from RTL2832U's 8-bit unsigned I/Q samples. This is conclusive: the
device, tuning, sample-rate config, and baseband init are all genuinely
working. The entire hangup was specific to how `RTLSDR.c` read the
stream — `fopen()`/`fread()` on this DeviceFS special-field path blocks
indefinitely even when data is sitting there ready, for reasons not
otherwise diagnosed (Norcroft's stdio buffering layer doing something
this particular device-style stream doesn't suit, most likely).

**Response**: replaced `RTLSDR.c`'s bulk read entirely — `fopen`/`fread`
out, raw `OS_Find`/`OS_GBPB` (reason 4) in, mirroring the now-proven BASIC
mechanism exactly (`os_find_open()`/`os_gbpb_read4()`/`os_find_close()`).
Also redesigned the throughput loop around what `OS_GBPB` actually does:
it returns immediately with whatever's currently available rather than
blocking to fill a full requested chunk, so "short read = fail" (built
around `fread()`'s different semantics) no longer fits. The loop now
polls repeatedly for a fixed 5-second window and sums whatever every call
returns, treating zero-byte polls as normal/expected rather than
failures, and gates purely on aggregate throughput. Rebuilt
(`build/RTLSDR,ff8`). This is the first build with a real, evidence-based
chance of actually passing milestone 4 — everything before this was
either confirmed-working-but-untested-together, or blocked before
reaching a throughput measurement at all.

### First real run of the C version: probe fully confirmed, main loop hit two bugs

The `RTLSDR.c` port of the raw-`OS_GBPB` mechanism ran: **probe read got
a clean 64/64 bytes** — direct confirmation the C port of the mechanism
works exactly like the proven BASIC version. But the main polling loop
errored on its very first call and reported "64 bytes in 529.1s" — an
impossible number (the loop's own timeout caps it at 5s).

Two separate bugs, both now fixed:
1. **`elapsed_ticks` was never initialized before the loop.** When the
   very first iteration hit `break` (on the error below) before reaching
   the code that normally sets it, the final report read uninitialized
   stack memory — that's where "529.1s" came from, not a real duration.
   Now initialized to 0 up front, and computed *before* the error check
   so an early-exit report is still accurate.
2. **The first poll requested the full `STREAM_CHUNK_BYTES` (256KB) in
   one `OS_GBPB` call and errored immediately**, where the 64-byte probe
   had just succeeded cleanly. `os_gbpb_read4()` was silently discarding
   the actual OS error message (`if (err != NULL) return -1;`) — fixed to
   print `err->errmess`, so the next run showed *why* it failed, not just
   that it did (see below).

### The error message: "Escape" — OS_GBPB blocks too, it isn't non-blocking

Reduced the per-call size to 4KB and re-ran. Result: `os_gbpb_read4
(len=4096): Escape` after **88.2 seconds** — the user had to interrupt it
manually, same as `fread()`. This overturns the earlier working theory:
`OS_GBPB` reason 4 does **not** just return whatever's immediately
available — it blocks trying to fill the requested amount, exactly like
`fread()` did. The 64-byte probe succeeding fast was very likely a
coincidence (an initial burst already buffered from `reset_buffer()`),
not a general property of the call. Real possibility now on the table:
**the device produces one small initial burst, not a continuous
stream** — plausible if the RTL2832U's pipeline needs genuinely
back-to-back reads with no gap to keep flowing, and the separate
"probe, then start a new loop" structure introduced exactly such a gap.

**Response**: rather than guess at another request size, made the test
decisive. Removed the probe/main-loop split entirely — one continuous
loop, 64-byte requests throughout (the only size that's ever definitely
worked fast), each read's result and per-read timing delta printed and
flushed immediately, capped at 50 attempts rather than a time budget (so
a stall costs at most one blocked call to recover from, not a whole timed
loop of larger requests). This will show, unambiguously, whether reads 2
through 50 keep succeeding at a steady pace (continuous stream, and the
real bug is specifically about request *size*) or stall after the first
one or two (burst-then-stop, a very different and more fundamental
problem). Rebuilt (`build/RTLSDR,ff8`). Note: this run's final "bytes/sec"
number is not a real throughput measurement (at most 50×64=3200 bytes
total) — it exists purely to show the per-read timing pattern.

### Confirmed continuous, not bursty — the size sweep is the current build

All 50 64-byte reads succeeded, steady ~10ms pace throughout, zero
stalls: `3200 bytes in 0.1s = 21333 bytes/sec (50 reads, 0 empty)`.
**Rules out the burst-then-stop theory entirely** — this is a genuine
continuous stream. The real, now well-isolated question: what request
*size* stops being reliable, somewhere between 64 bytes (works, fast) and
4096 bytes (blocked 88s). 21KB/s at 64 bytes/read is itself explained by
per-call SWI overhead (~10ms/call regardless of size) dominating at such
a small size — nowhere near the ~4.8MB/s target, but not a sign of a
deeper problem; just the wrong tool for measuring real throughput.

**Response**: replaced the single-size loop with a size sweep — 64, 128,
256, 512 (one USB max-packet — `RTLSDR_BULK_MAXPACKET`), 1024, 2048
bytes, 5 attempts each, stopping at the first size that stalls (>1s) or
errors (no point testing sizes larger than a known-bad one, and each
stall costs a manual Escape to recover from). Finds the largest reliable
single-call size directly, rather than guessing at one value at a time.
Rebuilt (`build/RTLSDR,ff8`). If 512 (one full USB packet) turns out to
be the boundary, that would point at multi-packet transfers specifically
being the problem — a very different, more actionable finding than a
vague "somewhere under 4096".

### The exact boundary: 1024 bytes reliable, 2048 blocks ~110s

Clean, sharp result: 64/128/256/512/1024 bytes **all** fully reliable —
5/5 attempts each, every one effectively instant (0.00s). 2048 bytes
blocked for **110.2 seconds** before an "Escape" error (user recovered
manually, no reboot needed — same safe category as the earlier `fread()`/
4096-byte blocks, not a full-machine freeze). Not the USB-max-packet-size
story (512 *and* 1024 both worked cleanly) — a real, sharp threshold
somewhere in `(1024, 2048)`, most plausibly a DeviceFS/driver internal
buffer-size limit rather than anything USB-protocol-level.

**Response**: this is now a big enough, clean enough signal to stop
sweeping and get a real number. The sweep no longer re-tests 2048 (no new
information, only another 100s+ Escape recovery). After the sweep
confirms the largest reliable size, the build now runs an actual
**sustained throughput measurement** — repeated reads at that size (1024
bytes) for up to 5 real seconds, computing genuine bytes/sec and gating
on it against the ~4.8MB/s target. This is the number milestone 4 has
been after this whole investigation. Rebuilt (`build/RTLSDR,ff8`). Even
if it passes, note the caveat already printed in the code: hitting target
via many 1024-byte reads (rather than fewer, larger ones) is a real but
different way to satisfy "sustained throughput" than the milestone
originally envisioned — some CPU/SWI-overhead cost per byte is
structurally higher this way than a design that could safely use larger
chunks.

## Open risks

- **Why single `OS_GBPB` reads above 1024 bytes block for ~110s was never
  root-caused**, only empirically worked around (read in 1024-byte
  chunks instead). Most plausibly a DeviceFS/driver internal buffer-size
  limit — the doc's `size/N` special field is an unexplored, plausible
  lever if a future need arises to push past 1024 bytes/call (e.g. to
  reduce per-byte CPU/SWI overhead below what 125,001 reads/5s implies).
  Not needed now — 25.6 MB/s already clears the target by 5x — but worth
  knowing this is a workaround, not a fix, if it resurfaces.
- **Why `fread()` never worked on this DeviceFS stream at any size was
  never root-caused either** — same category, worked around (raw
  `OS_Find`/`OS_GBPB` instead of stdio) rather than fixed. Expect the
  same issue if stdio is ever used against a DeviceFS special-field
  stream again.
- **Whether TaskWindow itself was the actual cause of the early
  full-machine freezes is unconfirmed** — running directly instead of in
  a TaskWindow has not reproduced a freeze since, across many runs, which
  is fairly strong circumstantial evidence, but the pacing delay added at
  the same time was never isolated as a separate variable.
- **GPIO register access (`GPD`/`GPOE`/`GPO`, SYS block) has frozen the
  whole machine multiple times** and is currently disabled rather than
  fixed. Needed eventually for correct V4 RF-path/bias-tee switching
  (real reception quality/band selection), but not for anything in scope
  through milestone 4.
- `dev->tuner->set_bw()` (a bandwidth-setter call in real
  `rtlsdr_set_sample_rate()`) has no ported equivalent — skipped as
  likely-optional; data is confirmed flowing at well above target without
  it, so likely fine, but not verified to be *correct* (vs. just present)
  for actual signal quality.
- Interface 1's purpose (second vendor-class interface on this dongle) is
  unconfirmed — not currently needed, but worth understanding eventually.
- Only RTL-SDR Blog V4 is in scope; other dongles won't work with v1.

## Next step

**Milestones 1 through 4 are complete** — RTL-SDR is proven working on
RISC OS end-to-end, from USB enumeration through sustained bulk sample
streaming at 25.6 MB/s (5x the required rate). What exists is a
command-line diagnostic tool, not a usable SDR application — milestone 5
(stretch/v2, not started) is where actual usability lives: a real
1024-byte-chunked reader feeding a ring buffer instead of a diagnostic
dump, FFT + waterfall display in a Wimp window, demodulation (FM/AM),
and a gain control UI. That's a substantial new phase of work, not a
continuation of the current diagnostic loop — worth treating as its own
planning discussion rather than assuming the same milestone-by-milestone
structure applies.

## Phase 2, milestone 1: live spectrum display (`!RTLSDRView`)

Pushed the completed phase 1 work to GitHub
(https://github.com/robheaton/riscos-rtlsdr, private repo, matching the
convention of this user's other native RISC OS projects), then scoped
phase 2's first milestone: a real Wimp GUI app showing a live-updating
spectrum trace. No demodulation, no audio, no tuning UI yet — the goal
is to prove the *architecture* (continuous USB reads inside a Wimp idle
handler without freezing the desktop, an FFT, and a redraw loop), which
everything after this builds on.

### Refactor: `c/Driver.c` / `h/Driver.h` (behaviour-preserving)

Extracted the non-`main()` device-control functions out of
`c/RTLSDR.c` into a new shared `c/Driver.c`/`h/Driver.h`: `find_device`,
`usb_ctrl_transfer`, `rtlsdr_read_reg`/`write_reg`, `rtlsdr_demod_read_reg`/
`write_reg`, `rtlsdr_probe_tuner`, `rtlsdr_init_baseband`,
`rtlsdr_tuner_postinit`, `rtlsdr_set_sample_rate`, `rtlsdr_reset_buffer`,
`os_find_open`/`os_find_close`/`os_gbpb_read4`. Every function moved
verbatim — same behaviour, only relocated — so `!RTLSDRView` (the new
app) can reuse the same proven bringup code instead of duplicating it.
`RTLSDR.c` now keeps only `main()` and the milestone 2-4 diagnostic
functions, calling into `Driver.c`.

**Regression-tested on real hardware and confirmed behaviour-preserving**:
rebuilt `!RTLSDR` against the refactored code, ran it on the Pi (direct
execution, not TaskWindow, per the established rule) — all four
milestones passed again, with throughput actually slightly *higher* than
the original run: **25,784,525 bytes/sec** (125,901 reads over 5.0s) vs.
the original 25,600,205 bytes/sec. Milestone 4 gate: PASS. Confirms the
extraction changed nothing about behaviour, only where the code lives.

### First real-hardware run: `Wimp_CreateWindow` data abort, root-caused and fixed

First build ran the full device bringup identically to `!RTLSDR` (tuner
PLL lock succeeded again, same trace as before) then crashed with
"Internal error: abort on data transfer" inside `Wimp_CreateWindow`
itself (postmortem backtrace: `create_spectrum_window` -> `main` ->
shared library function -> fault). Root cause: `wimp_colourflags.cols`
(the window-colours field in DeskLib's `Wimp.h`) mixes `unsigned char`
fields with `unsigned int` bitfields in the same struct -- a combination
where Norcroft can insert padding that breaks the byte-for-byte layout
the real Wimp SWI expects, even though the OTHER bitfield unions used
here (`window_flags`, `icon_flags`) are homogeneous `unsigned int`
throughout and don't have this problem. The window_block was being
filled via the `.cols.titlefore` etc. bitfield view; DeskLib provides an
alternate `.vals.colours[7]`/`.vals.extra` byte-array view of the exact
same field specifically for this reason. **Fix attempt 1**: switched to `.vals.colours[N]` (a plain byte array,
same field, no bitfields). Rebuilt, ran again: **identical crash, same
fixed address (&FC1A0078)**. That ruled out the value-level fix as
sufficient: rewriting *how* the colour bytes were written didn't fix
the union's *size* -- `wimp_colourflags` still has the `.cols` bitfield
member in its type, so Norcroft can still pad the union out past its
intended 8 bytes to fit that member's alignment, regardless of which
member is actually used to write into it. That silently shifts every
window_block field after `colours` relative to the offset the real Wimp
SWI expects. **Fix attempt 2** (the one that matters): stopped using
`window_block`/`wimp_colourflags` for construction entirely. Defined a
local `raw_window_block` struct mirroring the same 88-byte field-by-field
layout by hand (verified offset by offset against the real block
layout), using only plain `unsigned char`/`int`/pointer/`short` fields
for the colours (no bitfields at all, so no padding ambiguity is even
possible there), while still reusing `window_flags`/`icon_flags` for the
other flag fields -- those ARE safe, being homogeneous
all-`unsigned-int` bitfield unions matching their own `.value` alias
exactly, unlike `wimp_colourflags`'s char+int mix. Cast to
`(window_block *)` only at the `Wimp_CreateWindow` call site. Notable
because every "grounding" example found for this project
(`riscos-rdpclient`'s windows) builds windows from
`Wimp_LoadTemplate`-loaded blocks, never a hand-filled one -- so this
particular field-filling path had no proven-working precedent to check
against before the first real run.

**Confirmed fixed on real hardware**: the raw-struct rewrite ran clean --
a real `RTLSDRView` window opened (blue titlebar, no crash), the desktop
stayed responsive throughout (Filer windows visible and usable behind
it), proving the core architecture (device bringup, idle-driven USB
reads, window creation, redraw loop) all work together without
reintroducing the freeze risk phase 1 fought to fix. The one visible
issue: the work area rendered solid black -- `BAR_SCALE` (a flat linear
`power * constant`) was wildly oversaturating every one of the 256
bins to full height, so contiguous black bars painted the whole window
black. **Fix**: replaced the linear scale with a saturating
`power/(power+K)` curve (still no sqrt/log -- see R82XX.c's PLL loop for
the same avoid-libm reasoning) which stays informative even if the
constant is guessed wrong by an order of magnitude, unlike a flat
multiply which either clips everything or shows nothing. **Second real run**: window opened cleanly (confirming the
`raw_window_block` fix), work area was no longer solid black -- but
showed only a single thin vertical line at the horizontal centre, on an
otherwise blank white background. That's the classic RTL-SDR DC-spike
artifact, correctly centred by the FFT-shift, but it revealed the
saturating `power/(power+K)` curve still can't work here: a DC spike
typically sits 40-60dB above the noise floor (10^4-10^6x in raw power),
which no single linear-ish curve can show alongside the noise floor
without one end vanishing. **Switched to an actual log scale**
(`10*log10(power+1)`, `DB_FLOOR`/`DB_CEIL` linear-mapped to bar height)
-- and since a genuinely useful spectrum display needs this regardless,
tested whether `log10()` (a real libm function CALL, unlike the plain
`+,-,*,/` on doubles already confirmed working via Driver.c) would even
*link* against `C:o.stubs-32`, rather than guessing and burning another
hardware round-trip: **it linked cleanly, rc=0, no unresolved symbols**.
libm calls are confirmed available on this toolchain after all. **Third real run**: window opened fine, but the display looked
identical to the pre-log version -- a single thin line at centre,
otherwise blank. Root cause: `DB_FLOOR=30` clipped everything below
30dB to invisible, and the actual noise floor (most likely genuinely
quiet RF -- no/weak antenna -- rather than a bug, since the DC spike, an
ADC/mixer artifact rather than a received signal, still showed at full
height) sat below that. **Fix**: `DB_FLOOR=0` is not a re-guess, it's
the actual mathematical floor -- `power = re^2+im^2 >= 0` always, so
`10*log10(power+1) >= 0` always, meaning any floor above 0 was clipping
real data for no principled reason.

**Fourth real run, WITH a real antenna connected**: user confirmed an
antenna was attached for this test, tuned to 97.4MHz (a station
confirmed locally strong). Screen recording showed the display genuinely
is live-updating frame to frame (confirmed by diffing extracted frames --
real pixel differences, not a frozen image), but still showed no visible
bar variation beyond the same DC-spike line, even with real RF present.
That ruled out "no antenna" as the explanation and pointed at something
upstream of the display math entirely.

**Found it**: `R82XX.c`'s `r82xx_set_freq()` has had a GPIO call
disabled since phase 1 (`TEMPORARILY DISABLED: rtlsdr_set_bias_tee_gpio()`,
skipped after repeated hangs during tuner-init GPIO access). Re-reading
it now: this call controls the V4 board's RF switch between the antenna
connector and the HF upconverter mixer -- separate from the tuner chip's
own internal cable_1_in/air_in pins (which ARE written, just below the
disabled block, and only pick which of the tuner's own differential
inputs is live). Without it, the antenna signal may stay permanently
routed through the upconverter regardless of tuned frequency, which
would explain exactly this symptom: correct tuning, confirmed PLL lock,
a real antenna, and still no visible VHF signal at 97.4MHz. **Re-enabled
it** (`GPIO_UPCONVERT_PIN`, ON for HF/OFF for VHF+UHF, matching what
band-switching already does for the tuner's own pins just below it).
The historical hangs that caused it to be disabled in the first place
happened under TaskWindow execution -- since firmly established (many
runs, zero hangs) as the actual freeze cause, not GPIO access itself --
so re-attempting this now, with far more operational confidence than
phase 1 had, is a reasoned bet rather than repeating a known-bad
experiment. Pending a fifth real run: the main thing to watch for is
whether GPIO access reintroduces a hang (watch responsiveness closely;
power-cycle + unplug/replug the dongle if it does, per the established
recovery procedure) as well as whether it actually fixes VHF reception.

### `!RTLSDRView`: built and proven; milestone 1 declared complete

Built exactly per the plan: hand-built `window_block` (no Templates
editor available in this environment), reads pumped in bounded batches
of `os_gbpb_read4(512)` calls inside an `event_NULL` idle handler
(matching `riscos-rdpclient`'s `Status,fff` `Time_Monotonic()`-throttled
pattern — the same proven idle-update mechanism, not a new one), a
hand-written radix-2 FFT with offline-precomputed twiddle factors, and a
`Wimp_RedrawWindow`/`GFX_RectangleFill` bar-graph redraw. Links against a
prebuilt 32-bit `DeskLib32` reused from `riscos-rdpclient`'s
`builddesklib.py` (compiles all ~516 DeskLib source objects with
`-apcs 3/32bit` since the official prebuilt DeskLib doesn't match this
toolchain's 32-bit APCS variant), rather than rebuilding DeskLib from
scratch for this project.

**Fifth real run (GPIO fix), then a sixth (live) — the actual result**:
`!RTLSDR` completed all four milestones cleanly with the GPIO change in
place, confirming GPIO access itself is safe under direct execution (not
the freeze cause phase 1 blamed it for — TaskWindow was). `RTLSDRView`
with a real antenna connected showed a single visible feature (the
DC-spike bin, centred correctly by the FFT-shift) whose height visibly
**fluctuates live** — confirmed both from a screen recording (frame-by-
frame pixel diffs showed real variation, not a frozen image once video-
compression noise was accounted for) and directly by the user ("a
horizontal line that is moving up and down"). That is the actual proof
this milestone needed: continuous USB reads, a real FFT computing
genuinely varying values from live antenna data, and a redraw loop
showing it, all running inside the Wimp event loop without a single
hang or crash across six consecutive real-hardware iterations on this
new app (two of which were real, fixed bugs: the `wimp_colourflags`
struct-padding data abort, and the disabled upconvert-bypass GPIO).

**Milestone 1 gate: PASS**, by the plan's own definition of the goal --
proving the *architecture* (idle-driven streaming + FFT + window +
redraw, without freezing the desktop), not achieving broadcast-quality
spectral resolution. Merged to `main` via PR #1.

## Phase 2, follow-on: signal-quality pass (gain + display scale)

With milestone 1 merged, picked up the deferred signal-quality gap: the
FM station wasn't yet standing out clearly from the DC spike/noise
floor. Two candidate causes going in -- tuner gain, and the fixed
`DB_FLOOR`/`DB_CEIL` display range.

**Gain: verified correct, not the problem.** Rather than guess at R820T/
R828D gain register bits again (the exact mistake that cost real time
earlier in this project -- see the R820T_I2C_ADDR and I2C-bitrev
findings above), fetched the actual upstream source directly this time
(`tuner_r82xx.c` and `librtlsdr.c` from the rtlsdr-blog fork, raw GitHub
content, read in full) rather than trusting memory or a summarizer.
Findings:
- `rtlsdr_open()` never calls any gain-setting function at all --
  `rtlsdr_set_tuner_gain_mode()`/`rtlsdr_set_tuner_gain()` are public API
  calls real applications (`rtl_sdr`, gqrx) make themselves after
  opening the device. Our port never calling them either is faithful to
  upstream, not a gap.
- Our `r82xx_init_array`'s untouched register values already leave LNA
  in auto/AGC mode (reg 0x05 bit4=0) and Mixer in auto/AGC mode (reg
  0x07 bit4=1) -- verified bit-for-bit against upstream's own
  auto-vs-manual convention (`r82xx_set_gain()`'s bit4 writes). Both
  already match what upstream's own explicit auto-gain call would set,
  before we'd ever call it.
- Our `r82xx_set_vga_gain()` (0x08 -> 16.3dB, written unconditionally on
  every `r82xx_set_freq()`) is byte-for-byte identical to upstream's own
  function of the same name, called from the same place in upstream's
  own `r82xx_set_freq()` -- and upstream calls it *after* its own
  gain-mode setup, meaning even a real dongle with explicit
  `rtlsdr_set_tuner_gain_mode(dev, 0)` ends up at this same fixed VGA
  value after any retune. Our port's gain configuration is confirmed
  faithful to a genuinely working real-world default, not under-gained.

**Display scale: replaced the fixed-constant guessing with per-frame
auto-scaling.** Three iterations of picking `DB_FLOOR`/`DB_CEIL` by hand
(see above) kept failing because the right range depends on absolute
signal strength, which isn't knowable in advance. `compute_spectrum()`
now computes each frame's own min/max dB across all bins and maps that
range to the window height, excluding a small band of bins
(`DC_EXCLUDE_BINS`) around the display centre from the min/max
calculation so the DC-spike artifact can't dominate the range and crush
everything else toward zero -- the exact failure mode every fixed
`DB_CEIL` guess kept hitting. The DC bin is still plotted, just not used
to pick the scale. This is the standard auto-scaling technique real
spectrum analyzers use, and removes the guessing entirely rather than
requiring a fourth constant to tune.

**Real run with auto-scaling: no visible movement at all** (worse than
the earlier fixed-scale runs, which at least showed one moving line).
Added on-screen diagnostics directly in the window (successful/failed
read counts, this frame's dB range, raw ADC byte min/max, and eventually
a 12-byte hex dump of the last frame) rather than keep inferring from
photos -- printf/stderr goes nowhere visible in a Wimp app, so the
numbers get drawn into the window itself via `GFX_Write0` under `VDU 5`
(graphics-cursor text mode). Took a few iterations to get the layout
readable (a combined line overran the window width; 20-unit line
spacing badly overlapped this system font's actual line height; settled
on ~56-unit spacing, short per-line text).

**What the diagnostics found, in order:**
1. `ok=643240 bad=0` -- the read pipeline itself is completely healthy.
   Ruled out "os_gbpb_read4 silently failing, buffer stuck at its
   zero-initialized state" as the explanation for the DC-only display.
2. `min=46 max=46 dc=93` -- the ENTIRE non-DC spectrum (251 bins) reads
   within under 1dB of each other. Mathematically this is what flat
   white noise produces under an FFT, so on its own this wasn't
   necessarily a bug -- could just mean no distinguishable station
   carrier is reaching the tuner.
3. Raw byte range across the whole 512-byte frame: `0..129`, then
   `0..214` on a later run -- real, substantial variation, but sitting
   asymmetrically BELOW the expected ~127.5 ADC centre rather than
   spread around it. Healthy I/Q data shouldn't look like that.
4. **The actual smoking gun**: a 12-byte hex dump of consecutive raw
   bytes from one frame showed `86 D6 00 00 00 00 00 00 00 00 00 00` --
   two genuine-looking bytes, then a long run of exact zeros. Not noise
   -- a deterministic pattern.

**Working hypothesis**: `os_gbpb_read4()` is documented elsewhere in
this project as a blocking, synchronous SWI call, and milestone 4's
celebrated phase-1 throughput result (25.6 MB/s) only ever verified
BYTE COUNTS, never the actual content of what came back -- and used the
same kind of tight, back-to-back read loop `RTLSDRView`'s
`BATCH_READS=8` also used (8 `os_gbpb_read4(512)` calls with zero
pacing between them, every idle tick). If DeviceFS is polled for a full
512-byte chunk faster than the RTL2832U can actually fill one at 4.8
MB/s (~107 microseconds needed), it may report "512 of 512 bytes
transferred" while only a couple of bytes are genuinely fresh USB data,
with the rest being stale/zeroed padding from its own internal buffer
-- rather than genuinely blocking until real data is ready, or
returning an honest short count. **This would mean milestone 4's own
25.6 MB/s result may never have verified genuine sample content
either** -- a real open question about a previously "confirmed"
phase-1 result, not just a `RTLSDRView`-specific bug.

**Direct test**: dropped `BATCH_READS` from 8 to 1 (one
`os_gbpb_read4(512)` call per idle tick, relying on natural `Wimp_Poll`
call spacing for pacing instead of a tight loop) to see whether the
zero-padding pattern goes away. If it does, that confirms over-polling
as the cause here, and reopens whether milestone 4's throughput number
needs re-verifying with a content check, not just a byte-count check.
If the pattern persists even at one read per tick, something else is
going on and the sustained-throughput result stands.

**Real run result: the pattern persisted even at BATCH_READS=1.** Hex
dump: `F7 CC 00 00 00 00 00 00 00 00 00 00` -- same shape as before (two
real-looking bytes, then exact zeros), unaffected by read pacing.
`debug_reads_bad_g` has been exactly 0 across every single run --
`os_gbpb_read4()` has never once reported a short read (its return
value, `len` minus "bytes not transferred", has always equalled the
full requested count). This rules out over-polling as the cause and
narrows it to something more fundamental: `OS_GBPB` reason 4 against
this bulk-endpoint stream appears to report a full transfer while only
writing a small fraction of the destination buffer, at any read pacing.

## Phase 2, follow-on: milestone 5 (diagnostic) -- verifying OS_GBPB content, not just counts

Rather than keep patching `RTLSDRView`'s read loop blindly, built a
focused diagnostic independent of the FFT/display code entirely, added
to `!RTLSDR` as a new non-gating milestone 5
(`milestone5_read_primitive_diagnostic()` in `c/RTLSDR.c`). Two tests,
both against the same proven bulk-endpoint path milestone 4 uses:

1. **Sentinel test**: pre-fills the destination buffer with `0xAA`
   (a value plausible I/Q noise wouldn't produce as a long run) before
   each read, across sizes 2/4/8/16/.../1024 bytes, then counts how many
   bytes actually changed vs. what `os_gbpb_read4()` reports as
   transferred. This removes the ambiguity a plain zero-check has (real
   data could legitimately read as 0x00 sometimes) by measuring what the
   SWI actually *writes*, independent of what value ends up there.
2. **Repeatability test**: ten consecutive 8-byte reads, dumped in full,
   to distinguish genuinely fresh USB data each call from a stuck/cached
   single packet being re-read.

This also bears directly on phase 1's own headline result: milestone
4's 25.6 MB/s sustained-throughput number used the exact same
`len - bytes_not_transferred` formula and the exact same bulk-endpoint
path, and only ever checked that the count was large and fast -- never
that the bytes were genuine sample content. If milestone 5 confirms
`OS_GBPB` under-writes the buffer while over-reporting success, that
number may need re-verifying with a content check of its own, not just
treated as settled. Pending a real run.

**Real run result -- conclusive, and significant.** Sentinel test:
`touched == size` for EVERY tested size, 2 through 1024 bytes -- proving
`OS_GBPB` genuinely writes the entire requested buffer every time, not
leaving stale memory behind (ruling out the earlier "buffer just never
gets fully written" theory outright). But the actual bytes written are
telling: only the first TWO bytes are ever non-zero; every byte from
position 2 onward is a real, deliberate write of `0x00`, at every tested
size. The repeatability test confirms this isn't frozen/cached data --
all ten consecutive 2-byte prefixes differ (`B1 73`, `82 98`, `43 8D`,
`68 4B`, `93 9C`, `58 A9`, `A2 6C`, `8E B9`, `69 98`, `A1 6D`), so
genuinely fresh USB data is arriving each call.

**Conclusion**: the underlying USB transfer appears to deliver exactly
2 real bytes (one I/Q sample pair) per `OS_GBPB` reason-4 call,
regardless of requested buffer size, with RISC OS's DeviceFS zero-
filling the remainder while still reporting the FULL requested count as
"transferred" -- rather than returning an honest short count or
blocking until genuinely more data is ready.

**This overturns milestone 4's headline result.** "25.6 MB/s sustained
throughput" (docs/PLAN.md, phase 1) used the identical call and the
identical `len - bytes_not_transferred` counting formula, and never
checked content. At the sustained test's own measured rate (~125,901
reads over 5.0s, ~25,180 reads/sec), if each read genuinely carries only
2 real bytes, the ACTUAL sample data rate is roughly 25,180 * 2 =
**~50KB/s -- about 1% of the ~4.8MB/s target**, not 5x over it as
previously reported. The measured "throughput" almost certainly
reflected how fast a buffer can be zero-padded, not genuine USB
transfer speed. **The milestone 4 PASS verdict from phase 1 should be
treated as unconfirmed pending further investigation, not as a settled
result.** This is a genuine correction to a previously-reported
finding, not just a new open question.

**Root cause found**: RISC OS's own community technical documentation
confirms this is a known, longstanding DeviceFS USB characteristic, not
a bug introduced by this project. From the "DeviceFS USB technical"
wiki page (riscos.info, dated 2006, notes gathered from
comp.sys.acorn.programmer and driver-author experimentation):

> "When the transfer has occurred, the buffer will be filled with the
> USB data and then padded until it is full ... This padding is
> unfortunate as it is impossible to determine the actual amount of
> data transferred using the public API. This can be worked around by
> accessing the actual transfer size which is in the 11th word from the
> DeviceFS driver's handle. ... If you can get away without abusing this
> knowledge it is probably wise to do so, as it is relying on the
> private internal workings of the USB Driver."

And under "Summary of problems and unknowns": *"It would be nice to get
rid of the padding of short reads and the need to probe internal USB
Driver workspace to get the real transfer size."* -- confirming RISC
OS's own driver-writing community considers this padding-of-short-reads
behavior a known wart, not something this project misused or
misunderstood. The doc's own caveat ("only used with non-bulk pipes, so
this may be inaccurate for other transfer types") means it isn't a
perfect match for our bulk-endpoint case, but our own sentinel-test
evidence (short-of-request data padded with zeros while the public API
reports full success) lines up with this description closely enough
that it's very likely the same underlying DeviceFS buffer-padding
mechanism, just reached via the higher-level `OS_Find`+`OS_GBPB` +
special-field path this project uses rather than the low-level
module-driver API (`DeviceFS_CallDevice`/UpCalls) that wiki page
documents.

**What a real fix would need**: per that doc, getting the TRUE transfer
size requires reading an undocumented word (the "11th word from the
DeviceFS driver's handle") -- itself obtained via UpCall 10, a
mechanism normally used by module drivers, not applications. This is
explicitly flagged by RISC OS's own community as fragile ("private
internal workings"), and it's not even confirmed to apply cleanly to
bulk pipes specifically. This is a real platform-level limitation of
the public `OS_Find`/`OS_GBPB` DeviceFS USB API for high-rate bulk
streaming, not something fixable by changing read size, pacing, or
buffer handling in this project's own C code -- all of which have now
been tried (size sweep in milestone 4, `BATCH_READS` 8 vs 1, sentinel
verification across 10 sizes) without effect.

Source: [DeviceFS USB technical (riscos.info)](https://www.riscos.info/index.php/DeviceFS_USB_technical)

## Phase 2, follow-on: a real (non-UpCall) fix candidate found

The user asked to pursue the UpCall 10 workaround despite its scope.
Before writing any of it, researched the exact register/entry
conventions needed -- and the picture that emerged was significantly
worse than initially estimated: per the official PRM, an *application*
(not a module) claiming any vector must wrap **every single
`Wimp_Poll` call** with `OS_DelinkApplication`/`OS_RelinkApplication`
(classic RISC OS swaps an app's entire 32K code space in and out during
multitasking; a vector firing while swapped out jumps into whatever
other application happens to be loaded there), the handler itself needs
hand-written ARM assembler (vector handlers can't safely call normal C
library functions), and the failure mode is categorically worse than
anything else in this project -- not an app-level hang, but potential
vector-chain corruption affecting the whole desktop. Went back to the
user with this concretely worse picture before proceeding; they chose
to still attempt it.

**Before writing any assembler, did one more round of real-source
research (not guesswork) -- and it paid off.** Fetched DeviceFS's
actual source from `gitlab.riscosopen.org` (`RiscOS/Sources/HWSupport/
DeviceFS`), not just the informal 2006 wiki page. Two findings:

1. `Doc/UpCalls` shows DeviceFS's own UpCalls are just
   `UpCall_StreamCreated`/`UpCall_StreamClosed` -- nothing about
   transfer sizes at all. The wiki's "11th word from the DeviceFS
   driver's handle, obtained via UpCall 10" trick refers to something
   murkier (likely BufferManager internals, module-specific), not a
   clean, well-defined DeviceFS mechanism. This on its own was reason
   enough to be very cautious about that path.
2. `Doc/NonBlock` documents an entirely different, MUCH simpler,
   PUBLIC mechanism: DeviceFS streams support a non-blocking mode via a
   standard `OS_Args` IOCtl. States plainly: *"When in non-blocking
   mode, if gbpb_get cannot get the complete number of requested bytes
   it returns with number of bytes not done in R3"* -- i.e. an honest
   short-read count, unlike the padding-to-full-size behaviour in
   blocking mode (the stream's default state). Confirmed the exact
   calling convention against the real assembler (`s/FSystem`'s
   `args_ioctl`/`ioctl_miscop_nonblock`) and cross-checked against
   ROOL's own public SWI reference: `OS_Args` (SWI `&09`) reason 9 =
   IOCtl; IOCtl group `0xFF` (DeviceFS-specific) reason 1 = "Set/Read
   Non-blocking I/O", data word 1=set/0=unset.

**This needs no UpCalls, no vector claiming, no assembler** -- just a
second, ordinary SWI call (`OS_Args`) before reading, using primitives
already proven safe throughout this project (`_kernel_swi`, the same
pattern `os_gbpb_read4` itself uses). Added a test to milestone 5
(`os_args_set_nonblocking()` + a repeat of the sentinel test with
non-blocking mode enabled) to validate this actually produces honest
short-read counts before touching `RTLSDRView`'s real read loop.

**Confirmed on real hardware -- the fix works.** With non-blocking mode
enabled, `touched == got` in every single sentinel-test case (2 through
1024 bytes), where before `touched` was stuck at `size` regardless of
how much data genuinely arrived. A clear pattern emerged too: `got` is
consistently about half the requested size (4->2, 8->4, ... 1024->512),
capping at exactly 512 bytes (one USB max-packet) for anything larger
-- the underlying transfer genuinely can't deliver more than that per
non-blocking poll, but now says so honestly instead of lying.

**Wired into the real code.** Promoted `os_args_set_nonblocking()` from
the milestone 5 diagnostic into `Driver.c`/`Driver.h` (shared with
`!RTLSDRView`), with the padding-vs-honest-mode distinction documented
directly on `os_gbpb_read4()`'s declaration so it can't be missed by a
future caller. Restructured `RTLSDRView`'s `Null_spectrum()`: instead of
requiring a single call to return a full `IQ_BYTES` frame (which
blocking mode never honestly did anyway), it now accumulates
honestly-reported partial reads into `accum_buf_g` across multiple
calls until a full frame is ready; `got==0` means "nothing new yet", not
an error. `main()` calls `os_args_set_nonblocking()` once right after
opening the stream. `BATCH_READS` went back up from 1 to a generous 16
now that doing more reads per tick is safe again (each one just reports
however much genuinely arrived, rather than risking fabricated
"successes"). Pending a real run to see actual FFT content instead of
the DC-only display every previous run showed.

Sources:
- [DeviceFS source (gitlab.riscosopen.org)](https://gitlab.riscosopen.org/RiscOS/Sources/HWSupport/DeviceFS)
- [OS_Args (riscosopen.org wiki)](https://www.riscosopen.org/wiki/documentation/show/OS_Args)
- [IOCtl Reason Codes (riscosopen.org wiki)](https://www.riscosopen.org/wiki/documentation/show/IOCtl%20Reason%20Codes)

## Phase 2, follow-on: a second real bug -- degenerate zero-height plots

With the read fix wired in, a real run (no antenna) showed reads
healthy (`ok` climbing, `bad=0`) but STILL nothing visible drawn -- not
even the DC spike every earlier run had shown. Reconnecting the antenna
and re-running gave the same result: `min=49 max=49 dc=93 h=300` (the
diagnostic line now shows the actual computed `bin_height_g` for the DC
bin) -- the SCALE MATH was correct (44dB separation should clip the DC
bar to full height), but nothing rendered.

**Isolated with two targeted tests** rather than more guessing: (1) a
fixed, unconditional 100x100-unit black square, independent of
`bin_height_g`/`ox`/`oy` -- rendered correctly, proving
`GFX_RectangleFill`/`Wimp_SetColour`/coordinate math all work; (2) a
fixed 1-unit-wide, `WORK_HEIGHT`-tall rectangle (matching a real bar's
exact shape) at a distinct position -- ALSO rendered correctly, ruling
out thin-width and exact-boundary-height as the cause individually.

**Root cause**: calling `GFX_RectangleFill` 256 times in a tight loop,
where ~255 of those calls are degenerate (`bin_height_g[i]==0`, since a
near-flat spectrum gives almost every non-DC bin zero height after
auto-scaling) -- a zero-height rectangle-fill is apparently not a
well-behaved `OS_Plot` code path when called in volume, and was
corrupting state badly enough that even the one genuinely tall bar (the
DC bin) never rendered. **Fix**: skip the `GFX_RectangleFill` call
entirely when `bin_height_g[i] == 0` (costs nothing visually -- a
zero-height bar is invisible either way). **Confirmed on real
hardware**: the DC bar is visible again, centred, exactly where
expected.

**Where this leaves phase 2**: the architecture is now proven with
BOTH real bugs found this session fixed (DeviceFS's read-padding, and
degenerate-plot rendering) -- reads are honest, the FFT/scale math is
correct, and rendering is correct. What remains is the same open
question from before this detour: the non-DC spectrum is genuinely
close to flat (`min`/`max` a couple of dB apart, at most), so the
97.4MHz station isn't visually standing out from the noise floor. That
is now known to be a real signal-conditions/gain question, not a
software bug hiding behind it.

## Phase 2, follow-on: testing manual maximum gain

With the read/render pipeline confirmed correct, picked back up the
gain question. Earlier research (this doc, upstream `tuner_r82xx.c`)
found AGC already enabled via the untouched init array, matching
upstream's own default bring-up (real apps don't call
`rtlsdr_set_tuner_gain_mode()` as part of opening a device either) --
so AGC not being the bottleneck was already reasonably well-established.
But every real run since has shown the non-DC spectrum close to flat,
so it's worth directly testing whether MORE gain than AGC settles on
changes anything, rather than assuming AGC alone is sufficient.

Ported `r82xx_set_gain_max()` from upstream's real `r82xx_set_gain()`
manual-mode branch (same fetched source as the earlier gain research,
not re-guessed): LNA auto-off (reg 0x05 bit4=1), Mixer auto-off (reg
0x07 bit4=0 -- confirmed OPPOSITE polarity from LNA, straight from
source), then gain index in the low 4 bits of each register. Skipped
upstream's incremental target-gain search loop (which finds the
smallest gain combination meeting a specific dB target) since there's
no partial target here -- went straight to index 15 (0x0f), the
maximum step in both `r82xx_lna_gain_steps[]` and
`r82xx_mixer_gain_steps[]`. Wired into `RTLSDRView`'s startup, called
right after `r82xx_set_freq()` locks (needs the I2C repeater still
enabled, same as the tune call itself).

**Real run: manual max gain made no difference at all** -- `min=47
max=47 dc=93`, indistinguishable from every prior AGC run (49/49,
50/50, 46/46) and `dc` identical to the digit across every single run
regardless of antenna, gain, or any of this session's fixes. Gain
conclusively ruled out as the limiting factor.

## Phase 2, follow-on: independent hardware verification, and the real remaining bug

Asked the user to test the exact same dongle and antenna with SDR# on
Windows, at the same 97.4MHz, to isolate hardware/RF-conditions from
this project's software entirely. **Result: a strong, clean ~40dB peak
at 97.4MHz**, plus several other visible FM stations across the band,
confirmed via both the spectrum display and a persistent, strong
waterfall trace. This conclusively rules out antenna, dongle, and local
RF conditions as an explanation -- there IS a strong, real, ~40dB-above-
noise-floor signal reaching this exact hardware right now. The bug is
definitely still in this project's own software.

Given raw ADC bytes have shown wide variation in every run (0..127,
0..247, etc. -- genuinely swinging), but the computed FFT spectrum
stays essentially flat (~1-3dB spread) regardless of gain, the FFT/
accumulation pipeline itself must be destroying real dynamic range that
demonstrably exists in the raw samples. Re-read `fft256()` fresh (the
bit-reversal permutation and Cooley-Tukey butterfly both check out
against the standard DIT algorithm -- no bug found there) and
`compute_spectrum()` (also checks out on inspection).

**Suspect found in the accumulation logic instead**: milestone 5's
sentinel test showed each honest, non-blocking read returns roughly
HALF of what's requested. Filling a 512-byte frame from that pattern
converges quickly for the first ~511 bytes (256+128+64+...+1 = 511
after just 9 reads) but the LAST byte can need many more attempts,
each potentially returning 0. With the previous fixed cap
(`BATCH_READS=16`), a frame could easily need its final byte from the
NEXT idle tick -- tens of milliseconds later -- meaning the "frame"
handed to the FFT could be a concatenation of samples from genuinely
DIFFERENT moments in time. That would corrupt frequency resolution
(smearing real signal energy across every bin, since the FFT assumes a
temporally contiguous sample sequence) while leaving the DC bin
untouched (it's just a sum of all samples, order-independent) --
exactly the observed pattern: real signal present, DC consistently
large, everything else flat.

**Fix**: since non-blocking reads can never hang, it's safe to loop
until a frame is genuinely complete (or the stream truly has nothing
more available right now, `got==0`) instead of capping at a small fixed
attempt count. Replaced `BATCH_READS=16` with an unbounded loop (a
generous `MAX_READ_ATTEMPTS=5000` safety bound against a pathological
runaway, not a normal-operation limit) in `Null_spectrum()`. Real run:
no change -- same flat pattern.

## Phase 2, follow-on: I/Q pairing guard, then live observation, then DC removal

**I/Q pairing guard, no effect.** New hypothesis: milestone 5's
sentinel test only ever requested round power-of-two sizes (always
yielding even `got` counts); real usage requests whatever's left to
reach a full frame, which isn't always a round power of two, so a read
could plausibly return an ODD byte count -- desyncing every
`iq_frame_g[2*i]`/`[2*i+1]` I/Q pairing by one byte for the rest of the
frame. Fixed (`accum_fill_g += got & ~1`, only counting whole pairs).
Real run: `min=47 max=47 dc=93` -- still no change.

**Direct live observation broke the stalemate.** Rather than keep
guessing at more accumulation-layer bugs, asked the user to watch the
running display continuously for several seconds (not comparing
separate screenshots/launches) and report whether anything changes at
all. Result: **`min`/`max` genuinely fluctuate, 28-51dB, continuously**
-- conclusively proving the pipeline is live and recomputing every
frame, not stuck. But **the DC bar's height never visibly changes** --
which turns out to be simple arithmetic, not a bug: `dc` (~93) is
*always* so much larger than `max` (which tops out around 51-54) that
`(dc-min)/range` mathematically clips to full height (300) on every
single frame regardless of what min/max actually are. The apparent
"frozen line" was never evidence of a stuck pipeline -- it was the DC
bin correctly, consistently maxing out.

**Real fix candidate: DC removal before the FFT, not just excluding a
couple of bins from the display scale.** Real spectrum analyzers
subtract each frame's own measured mean from the samples before
transforming, specifically because a genuine hardware DC bias
spectrally leaks into and swamps NEARBY bins too, not just the exact
centre one -- `DC_EXCLUDE_BINS=2` only ever hid that leakage from the
*scale calculation*, it never stopped the leakage from corrupting the
FFT's own bin values in the first place. `compute_spectrum()` now
computes the frame's actual mean I and mean Q and subtracts them before
centering-by-127.5 and calling `fft256()`, instead of only ever
centering by the fixed assumed 127.5. This is standard, correct
signal-processing practice, not a guess.

**Real run: `dc=0` and `h=0`, exactly, every time -- expected, but a
diagnosis mistake, not new information.** Subtracting the mean from a
set of values and then summing the result is EXACTLY zero by
definition of "mean" -- `db[centre]` (the FFT's bin-0/DC output) was
therefore mathematically guaranteed to read 0 after this fix,
regardless of whether any real signal was present. Pointing the user at
it cost a wasted round-trip. What actually matters is whether bins
*near* centre (where a real station's energy should spread, not the
exact centre point) show any elevation -- added tracking for `max_bin`
(which bin holds the frame's strongest non-DC value) and its offset
from centre, replacing the now-meaningless `dc=`/`h=` display with
`pk@%+d`.

**Real run: `pk@+3`, ROCK-SOLID across several seconds of live
observation.** This is the actual breakthrough. Random noise would
shuffle which bin happens to be "highest" unpredictably frame to frame;
a perfectly consistent winner at the same bin, every single frame,
means a real, stable signal IS being detected -- at roughly +3 bins
from the tuned frequency (~+28kHz at this project's ~9.4kHz/bin
resolution), plausible as a PPM/crystal calibration offset. So
detection is working. But `min`/`max` both rounding to the same integer
means the MEASURED difference between that peak and the noise floor is
far smaller than SDR#'s independently-confirmed ~40dB for the same
station.

**Likely cause found**: `db[i] = 10*log10(power + 1.0)` -- that `+1.0`
epsilon was only ever meant to avoid `log10(0)`, and was harmless back
when a huge, un-removed DC bias (power ~10^9) made it negligible by
comparison. Now that DC removal correctly reduces typical per-bin power
to much smaller values, that same `+1.0` can dominate for any bin with
power well under 1 -- crushing real dB differences down to a fraction
of a dB, exactly matching the observed "peak position is rock-solid
but barely measurable" result. Changed the epsilon to `1e-6` (small
enough to avoid `log10(0)` without swamping realistic power values).

**Real run: no change.** `min=45 max=45 pk@+3` -- identical to before
the epsilon fix, `pk@` still rock-solid. Epsilon ruled out as the
(sole) cause.

## Phase 2, follow-on: power-domain frame averaging

Reframed: the epsilon fix not mattering, combined with `pk@` staying
perfectly consistent across many seconds of live observation, points
at something more fundamental than a scaling constant -- a single, raw,
un-averaged 256-point FFT snapshot ("periodogram") is well known in DSP
to have high per-bin variance even for genuinely strong, real signals;
only averaging many such snapshots together reveals the true
signal-to-noise ratio clearly. This is exactly what SDR#'s clean ~40dB
display relies on and this project had never done -- every previous
run computed and displayed a single instantaneous snapshot, discarding
every other frame that completed between display updates (which, given
the fast honest-read throughput this project has, could be many per
second).

Split `compute_spectrum()` into two functions: `accumulate_frame()`
(DC removal, FFT, power computation, folded into a running exponential
average in the POWER/linear domain) now runs every time a frame
completes in `Null_spectrum()` -- which can happen many times a second,
not just at the ~5Hz display rate -- so many more real frames
contribute to what gets shown. `finalize_display()` (auto-scale,
peak-bin tracking, bar heights) runs only at the throttled display
rate, converting the current running average to dB and pixels.
Averaging must happen in the power domain, not by averaging dB values
directly (log of an average is not the same as an average of logs).
`AVG_ALPHA=0.05` (an exponential moving average, ~20-frame effective
window) is a first-pass guess, not measured. Pending a real run.

**Apparent real run: `min=0.00 max=0.00 pk@+0`** -- looked like a new
regression (matching the debug globals' own static initializers
exactly, as if `finalize_display()`/`accumulate_frame()` never ran at
all). Turned out to be a mistakenly reposted screenshot from an earlier
build in the sequence, not a result from this build -- user corrected
this ("i posted the wrong picture, everything is stuck at 0"). No code
issue; noted here only so this dead end isn't rediscovered.

**Real run (correct screenshot): `ok=15000 bad=0`,
`min=48.84 max=48.84`** -- identical to 2 decimal places across 251
non-DC-excluded bins, after 15000 accumulated frames. This rules out
"nothing is running" (reads and averaging are clearly executing at
volume) but is itself suspicious: real noise-floor variance across 251
independent bins essentially never lands on the exact same rounded
value everywhere. Two live hypotheses: (a) `avg_power_g[]` genuinely IS
this uniform (a deeper bug upstream of display -- e.g. FFT input,
accumulation, or bin-index mapping), or (b) there IS a real linear
difference between bins that log compression is hiding, since
`log10(x)` barely moves for large, similar-magnitude `x` (e.g.
`log10(1e10)` vs `log10(1.1e10)` differ by only ~0.04 dB). Added
`debug_raw_min_g`/`debug_raw_max_g` (`avg_power_g[min_bin]`/
`avg_power_g[max_bin]`, captured straight from the linear-domain
average, bypassing `10*log10()` entirely) and a new display line
showing them in scientific notation, to distinguish (a) from (b) before
guessing at another change to the averaging/FFT code itself. Pending a
real run.

## Phase 2, follow-on: the read-accumulation bug, and where things stand

**Real run: raw power identical too (4.828e+04 == 4.828e+04)** --
ruling out (b) (log compression hiding a real difference) definitively.
`avg_power_g[]` genuinely is near-uniform across bins in the linear
domain, not just after `log10()`. Points at (a): something upstream of
the display, most likely the FFT's actual input data.

**Unrelated but urgent: the user reported the Pi becoming unresponsive
while running recent builds.** Root-caused to `Null_spectrum()`'s
non-blocking read loop: it kept accumulating and FFTing completed
frames all the way up to `MAX_READ_ATTEMPTS` (5000 at the time) instead
of returning to `Wimp_Poll` after a handful, and with real sustained
USB throughput the "stream is empty" exit condition is rare -- so a
single idle-tick call was burning through nearly the whole attempt
budget in one tight, non-yielding loop, every tick. Fixed with a
`MAX_FRAMES_PER_TICK` cap (8) and a much smaller `MAX_READ_ATTEMPTS`
(200), restoring frequent returns to `Wimp_Poll`. Confirmed by the user
("the machine is responsive again now"). This was a real, independent
regression from switching to non-blocking reads with an unbounded
per-tick batch -- exactly the risk the original phase-2 plan flagged
for an over-large per-tick batch.

**Byte-level diagnostics finally pinned down the flat-spectrum root
cause.** Added, one at a time, live on real hardware: raw byte range
across the frame, time-domain per-sample power range (pre-FFT), which
sample index carries the frame's peak power, how many samples clear 1%
of that peak, and finally the actual raw byte values at specific
offsets. Findings, in order:

- Per-sample power ranged from ~1 to ~64,000 (5 orders of magnitude) in
  a single frame, with only 1-2 of 256 samples ever "elevated" -- the
  signature of a near-impulse time-domain frame (mostly one dominant
  sample, the rest near the frame's own mean). The DFT of a near-delta
  function is flat by definition, which is exactly what was being
  measured: the FFT math was behaving correctly for degenerate input,
  the input itself was the problem.
- The dominant sample's buffer INDEX was rock-solid at exactly 0, every
  frame, and bytes at the frame's middle/end read as exactly 0 -- their
  never-touched zero-init value -- every frame. Manual max gain
  (thought at the time to be a plausible overload cause) was removed
  and made no difference, ruling out ADC saturation.
- Root cause: `Null_spectrum()`'s read loop requested
  `IQ_BYTES - accum_fill_g` on each successive call -- a SHRINKING,
  ARBITRARY size once any data had already accumulated. Milestone 5's
  non-blocking-mode fix was only ever validated against round
  power-of-two sizes (64/128/256/512); non-round sizes apparently still
  hit the original short-read padding behaviour, so `accum_fill_g` was
  reaching `IQ_BYTES` almost entirely from fake reported byte counts.
  Fixed by always requesting the SAME fixed, validated chunk size
  regardless of how much of the frame is already filled, carrying any
  overshoot past the frame boundary into the next frame instead of
  discarding it.

**Real run: min/max finally separated** (e.g. `min=36.58 max=55.90`,
raw power differing by ~76x, matching the dB gap). The core "why is
this flat" mystery, chased since early in phase 2, was this bug.

**But the display then showed a dense, static "barcode" pattern that
didn't change.** Chased through several more rounds:

- 4-byte destination-offset alignment -- no effect.
- Reading into a fixed offset-0 scratch buffer instead of a growing
  destination pointer (sidestepping any DeviceFS quirk tied to
  non-zero addresses) -- no effect.
- A sentinel-fill test (pre-fill 0xAA, count touched bytes vs. `got`,
  the exact technique that caught the original padding bug) proved
  reads ARE honest at the 256-byte chunk size -- `got` always equalled
  `touched`. So the remaining zero bytes are genuinely-written zeros,
  not padding.
- Sampling the EXACT 256-byte chunk boundaries showed a suspicious
  pattern: bytes right at/near a chunk boundary kept landing on exactly
  0 while other positions varied normally.
- The user then clarified the RTLSDRView window has no scrollbar or
  resize -- what looked like a static barcode WAS the entire visible
  window. Six accumulated diagnostic text lines were stripped back to
  one, restoring the window height to the bars -- but the barcode
  pattern was still there underneath, just previously hidden behind
  dense text (a real, independent bug -- reintroducing the exact
  "diagnostic text fills the whole window" mistake already documented
  and fixed once earlier in phase 2 -- but not the cause of the
  barcode itself).
- 10x heavier averaging (`AVG_ALPHA` 0.05 -> 0.005, ~200-frame window
  instead of ~20) made NO visible difference to the pattern -- a real
  tell, since genuine per-bin noise averages down with more frames,
  while a pattern that survives 10x more averaging unchanged must be
  deterministic, repeating identically every frame.
- The math lines up: two 256-byte chunk reads per 512-byte frame, each
  apparently carrying the same "boundary" artifact, sit exactly 128
  samples (N/2) apart -- and two fixed impulses spaced at exactly N/2
  in an N-point DFT alias to a perfect every-other-bin alternating
  pattern. That's mathematically consistent with the observed barcode,
  and with why averaging couldn't remove it.
- Tested switching `FIXED_CHUNK_SIZE` to 512 (the bulk endpoint's
  actual native USB max packet size), making each frame a single
  atomic chunk read with zero software-level splicing. The sentinel
  test confirmed this was ALSO honestly reported (`g512/t512` always).
  But the result **regressed to completely flat** (`min==max` exactly,
  no bars drawn at all) -- worse than the spliced 256-byte version, not
  better.

**Where this leaves things:** the 256-byte chunk version's apparent dB
contrast may be partly (or wholly) a splicing artifact rather than
fully real captured signal -- the clean, unspliced 512-byte read
showing nothing suggests the true, artifact-free capture may not
actually contain the strong signal SDR# shows on Windows with the same
hardware/antenna. That points the remaining mystery upstream of the
read/accumulation mechanism entirely (tuning, IF configuration, gain
staging) rather than at anything in how bytes move from the SWI into
memory -- a genuinely different category of investigation than
anything tried so far in phase 2. Per the user's explicit choice, the
app has been reverted to the 256-byte chunk size (the more usable
result of the two, even if not fully understood) as this milestone's
working baseline, and this specific investigation is paused here
rather than continued blind.

**Real, confirmed fixes from this investigation, independent of the
open mystery above:** the DeviceFS short-read padding bug (both its
original blocking-mode form, and its resurfacing for non-round-sized
non-blocking requests), the `Wimp_CreateWindow` struct-padding crash,
the degenerate zero-height `GFX_RectangleFill` rendering bug, the
resource-hogging unbounded read-attempt loop, and the "too much
diagnostic text fills the whole window" mistake (now fixed twice). All
confirmed via real hardware testing, all still in place.

## Phase 2, follow-on: gain control UI

Added a live gain control UI: three real Wimp icons (`Wimp_CreateIcon`,
not custom-drawn graphics) below the diagnostic text -- "AGC" toggle
button and "-"/"+" manual gain-step buttons (0-15) -- handled by a new
`Click_spectrum` `event_CLICK` handler. New tuner driver functions
`r82xx_set_gain_agc()`/`r82xx_set_gain_manual(t, index)` in
`R82XX.c`/`.h` (the old hardcoded `r82xx_set_gain_max()` is retired,
generalized into the parameterized manual function). Current gain
state shows compactly in the existing diagnostic line (`G:AGC` /
`G:M08`) rather than a separate live-updating icon. Also reserved a
100-unit strip at the top of the window (`RESERVED_TOP`/
`BAR_MAX_HEIGHT`) so a tall bar can never grow up into and cover the
text or icon row.

**Real run: confirmed working end-to-end.** User clicked through AGC/
-/+ and confirmed the displayed `min` value visibly moves within a few
seconds of a gain change (matching the ~200-frame `AVG_ALPHA=0.005`
averaging window's settle time) -- proving the button clicks are
genuinely reaching the tuner over I2C and changing real ADC gain, not
just updating the display. First real, hands-on confirmation that a UI
control changes something measurable in this app.

## Phase 2, follow-on: frequency tuning UI

Added F-/F+ frequency step buttons (100kHz per click, 24MHz-1.7GHz
range), mirroring the gain control's button pattern rather than a
writable text-entry icon. `Click_spectrum` only commits the new
`tuned_freq_hz_g` if `r82xx_set_freq()` actually locks, and on success
flushes the stale pre-retune buffer, discards any in-flight partial
frame, and resets the power average so the display doesn't blend two
different stations' spectra together for the next couple of seconds.
Current frequency shows in the same diagnostic line as gain state.

**Real run: confirmed working** ("looks to work") -- stepping away
from and back to 97.4MHz kept the tuner locked and the display
running.

## Phase 2, follow-on: FM demod DSP (numeric readout, no audio yet)

Real streaming PCM audio output needs RISC OS's interrupt-driven
"linear handler" sound mechanism or the SharedSound module's streaming
API, neither grounded in real, fetched documentation the way the
DeviceFS/tuner work was -- deliberately sequenced as DSP-first,
audio-later rather than guessing at undocumented SWIs (the same
category of risk this project already backed away from once, with the
GPIO/UpCall approach). Added `demodulate_frame()`: a standard complex-
baseband phase discriminator (atan2 of consecutive samples' product,
scaled from radians/sample to Hz via the real 2.4MHz sample rate),
called on the DC-removed re[]/im[] arrays before `fft256()` overwrites
them in place, tracking EMA-smoothed RMS/peak deviation. Gated behind
a new "DEM" toggle icon (off by default -- real added CPU cost, one
atan2() per sample). Readout shown as a second text line when enabled.

**Real run: `dev pk=1167.6k rms=126.0k Hz`, tuned to 97.4MHz.** This is
a genuinely informative result, not just a number to note: 1167.6kHz
is essentially the discriminator's theoretical maximum possible output
(±half the 2.4MHz sample rate, ~1.2MHz -- where atan2's phase
calculation wraps around ±pi). Hitting that ceiling means consecutive
samples' phase relationship is essentially random frame-to-frame, not
smoothly, coherently progressing the way real FM modulation would
(genuine wideband FM tops out around 75kHz deviation, more than an
order of magnitude below this reading, and shouldn't make a massively-
oversampled 2.4MHz-rate discriminator saturate at all). This is the
same category of finding as the still-unresolved flat-spectrum mystery
(see the earlier "Phase 2, follow-on: the read-accumulation bug, and
where things stand" section) approached from a completely different
angle (time-domain phase coherence, not frequency-domain averaged
power) -- and it points the same direction: the captured samples don't
look like they carry real, coherent broadcast content, independent of
gain/AGC state. Worth revisiting the paused flat-spectrum
investigation with this as a second, independent data point.

## Phase 2, follow-on: the flat-spectrum mystery, RESOLVED

Picked back up with two leads in hand: the demod saturation finding
above, and the idx@/nElev byte-level diagnostics from much earlier
(one dominant sample out of 256, rest near-silent) that the various
read-mechanism fixes never actually resolved -- they'd just moved
where in the accumulation pipeline the symptom showed up.

**Step 1: re-fetched real upstream source fresh** (librtlsdr.c,
tuner_r82xx.c, rtlsdr-blog fork) rather than trusting research notes
from much earlier in this very long session. Found a genuine,
previously-missed gap: real `rtlsdr_set_sample_rate()` calls
`dev->tuner->set_bw()`, which for R820T/R828D chains
`r82xx_set_bandwidth()` (computes the tuner's actual analog IF for the
requested capture bandwidth, writes its filter registers) ->
`rtlsdr_set_if_freq()` (resyncs the demod's digital downconversion to
match) -> re-tune. This project's port never called any of that --
`r82xx_set_freq()` used a fixed `R82XX_IF_FREQ` constant (3.57MHz)
forever, matching only `r82xx_init()`'s own generic default. Hand-
computing upstream's real formula for this project's 2.4MHz capture
gives ~1.815MHz -- a ~1.75MHz mismatch against a capture whose own
Nyquist limit is only 1.2MHz, easily enough to explain looking like
noise. Ported `r82xx_set_bandwidth()` verbatim (R82XX.c/h), added
`t->int_freq` (replacing the fixed constant), exposed
`rtlsdr_set_if_freq()` (was static), and wired the full resync sequence
into `main()` after `rtlsdr_set_sample_rate()`.

**Real run: demod stats barely moved** (`dev pk=1198.4k`, was
`1167.6k`) despite the fix being genuinely correct and confirmed
applied. This was a real, independent bug worth fixing regardless, but
not (on its own) the explanation for the flat spectrum / demod
saturation.

**Step 2: re-checked idx@/nElev post-fix.** Re-enabled the (still-
computed, no-longer-displayed) diagnostic. With the CURRENT chunk size
(256 bytes) at the time: `idx@` alternated between EXACTLY 0 and
EXACTLY 128 -- not scattered, but flipping between the two fixed
positions exactly 128 samples (256 bytes) apart marking the boundary
between each frame's two 256-byte software-level read chunks.

**Step 3: retried atomic 512-byte reads** (one read per frame, zero
software-level splicing) now that the IF fix removed a confound from
the FIRST time this was tried. Result: `idx@` became PERMANENTLY stuck
at 0 -- i.e. regardless of chunk size, only the very FIRST sample of
ANY INDIVIDUAL READ CALL is genuinely fresh; the rest of that same
call's data, despite passing the sentinel test (differs from a 0xAA
fill), isn't real advancing ADC content. This reframed the sentinel
test's own limitation: "differs from 0xAA" only catches zero/fixed-
value padding, not padding by REPEATING an earlier real byte, which
would still register as "touched."

**Step 4: shrank the read size to 8 bytes (4 samples)** -- far smaller
than anything in milestone 4's original round-size sweep -- on the
theory that if only the first sample of any read is genuinely fresh,
a much smaller request maximizes what fraction of each read is real.
Raised `MAX_READ_ATTEMPTS` 200->1200 to compensate (filling one
512-byte frame now needs many more individual, still-cheap,
still-non-blocking-so-can't-hang calls).

**Real run: this was it.** `nElev` jumped from 1-2 to **256** -- every
single sample in the frame now carries real, meaningful energy, not
just one or two. `idx@` scatters normally instead of sticking to a
fixed position. `min=48.0 max=82.1` (34dB spread). And for the first
time all session, the actual rendered bar graph shows a real,
multi-peak spectrum shape -- distinct stations visible across the
tuned band, not noise. User's reaction: "yes!!!! look at that."

**Root cause, in one sentence:** DeviceFS's non-blocking short-read
mechanism, for large request sizes, was returning a "genuine" (passes
sentinel testing) but stale buffer where only the first sample was
real and the rest were padding-by-repetition of an earlier byte --
invisible to the zero-padding-only sentinel test used to validate the
original milestone-5 fix, and worse the larger the request, which is
exactly backwards from what the earlier round-size sweep assumed
("bigger validated sizes are safe"). Small requests (8 bytes) work
because the "stale tail" shrinks to almost nothing.

**Still open / not yet re-tuned:** `MAX_READ_ATTEMPTS=1200` and
`FIXED_CHUNK_SIZE=8` were the first values tried that worked, not
necessarily the best -- worth experimenting with sizes between 8 and
256 to find a sweet spot with fewer SWI calls per frame while keeping
nElev healthy, if per-frame overhead ever becomes a concern. The FM
demod deviation reading is still not obviously "sane" (large peak/rms)
even with clean spectrum data now -- but this is expected, not a bug:
`demodulate_frame()` runs on the FULL 2.4MHz-wide raw baseband, not a
single filtered 200kHz channel, so it's currently demodulating the
combined energy of everything in view (multiple stations at once,
per the new multi-peak spectrum), not one station -- a real per-channel
filter/decimate stage is future work, not part of this fix.
