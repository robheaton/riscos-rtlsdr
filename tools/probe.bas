REM >Probe
REM
REM RTL-SDR hardware probe for RISC OS. Run this FIRST, before any C code,
REM on the real Pi with the dongle plugged in. Answers the open questions
REM in ../docs/PLAN.md: does the dongle show up in *USBDevices, what number
REM does it get, and what do *USBDevInfo/*USBConfInfo print for it.
REM
REM No compiler needed -- just load this into !Edit/BASIC and RUN it, or
REM `*Basic` then `INSTALL"Probe"` from a command line.
REM
REM v2: fixed from the first real run against a real dongle --
REM   - *USBDevices alone identifies the device by description text (no
REM     need to hunt for hex VID/PID in *USBDevInfo output, which isn't
REM     even in the *USBDevices dump).
REM   - *USBDevInfo/*USBConfInfo calls are now wrapped in ON ERROR LOCAL,
REM     since an OSCLI() call that hits a RISC OS error (e.g. a bad device
REM     number) raises a real BASIC error and aborts the whole script if
REM     unguarded -- that's what killed v1 (it looped device numbers from
REM     0, but real numbering starts at 1, and the resulting syntax error
REM     stopped the script dead after the very first call).
REM   - Device numbering in *USBDevices starts at 1 (the "No." column),
REM     not 0.
REM
REM v3: fixed from the second real run -- *USBDevInfo 6 worked fine (no
REM error), but the unconditional "RESTORE ERROR" that ran right after it
REM regardless of whether an error occurred is invalid when nothing is on
REM the error-handler stack to restore ("Error control status not found on
REM stack"). RESTORE ERROR now only appears inside the ON ERROR LOCAL
REM handler bodies themselves, where it belongs, killing the script before
REM it ever reached *USBConfInfo. Confirmed from that run: VID 0BDA,
REM PID 2838, matching h/RTLSDR.h already.

10 PRINT "Listing all USB devices (*USBDevices)..."
20 PRINT
30 OSCLI("USBDevices { > <Wimp$ScrapDir>.rtlprobe1 }")
40 SYS "OS_File", 17, "<Wimp$ScrapDir>.rtlprobe1" TO ,,,size%
50 IF size% = 0 THEN PRINT "*USBDevices produced no output -- is a device attached?": END
60 f% = OPENIN("<Wimp$ScrapDir>.rtlprobe1")
70 found% = -1
80 WHILE NOT EOF#f%
90   line$ = GET$#f%
100   PRINT line$
110   IF INSTR(line$, "RTLSDR") > 0 OR INSTR(line$, "RTL2832") > 0 OR INSTR(line$, "Realtek") > 0 THEN
120     found% = VAL(line$)
130   ENDIF
140 ENDWHILE
150 CLOSE#f%
160 PRINT
170 IF found% < 0 THEN
180   PRINT "No device description matched RTLSDR/RTL2832/Realtek in the"
190   PRINT "*USBDevices dump above -- check it by eye; the dongle may report"
200   PRINT "a different description string than expected."
210   END
220 ENDIF
230 PRINT "Found dongle at device No. "; found%
240 PRINT
250 PRINT "=== *USBDevInfo "; found%; " ==="
260 ON ERROR LOCAL PRINT "  (error: "; REPORT$; ")": RESTORE ERROR
270 OSCLI("USBDevInfo " + STR$(found%) + " { > <Wimp$ScrapDir>.rtlprobe2 }")
280 SYS "OS_File", 17, "<Wimp$ScrapDir>.rtlprobe2" TO ,,,size2%
290 IF size2% > 0 THEN
300   g% = OPENIN("<Wimp$ScrapDir>.rtlprobe2")
310   WHILE NOT EOF#g%
320     PRINT GET$#g%
330   ENDWHILE
340   CLOSE#g%
350 ENDIF
370 PRINT
380 PRINT "=== *USBConfInfo "; found%; " ==="
390 ON ERROR LOCAL PRINT "  (error: "; REPORT$; ")": RESTORE ERROR
400 OSCLI("USBConfInfo " + STR$(found%) + " { > <Wimp$ScrapDir>.rtlprobe3 }")
410 SYS "OS_File", 17, "<Wimp$ScrapDir>.rtlprobe3" TO ,,,size3%
420 IF size3% > 0 THEN
430   h% = OPENIN("<Wimp$ScrapDir>.rtlprobe3")
440   WHILE NOT EOF#h%
450     PRINT GET$#h%
460   ENDWHILE
470   CLOSE#h%
480 ENDIF
500 PRINT
510 PRINT "Paste all of the above back for the next step -- it tells us the"
520 PRINT "DeviceFS name to use (probably USB"; STR$(found%); ") and, from"
530 PRINT "*USBConfInfo, which endpoint number is the bulk IN endpoint"
540 PRINT "(0x81 in libusb terms) for the streaming test later."
