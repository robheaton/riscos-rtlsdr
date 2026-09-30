/* Driver.h -- shared RTL-SDR device bringup layer: device discovery,
   baseband/tuner init, sample-rate config, and raw OS_Find/OS_GBPB bulk
   stream primitives. Extracted from RTLSDR.c (the milestone 1-4
   diagnostic tool) so !RTLSDRView (the live spectrum display) can reuse
   the same proven bringup code without duplicating it. See
   docs/PLAN.md for how each piece was derived/confirmed on real
   hardware. C89 only (Norcroft). */

#ifndef DRIVER_H
#define DRIVER_H

/* ---- device discovery ----
   Scans *USBDevices output for a matching dongle description, returning
   its "No." column value (for building the "usbN" DeviceFS name), or -1
   if none found. */
int find_device(void);

/* ---- demod register access ----
   Page-addressed, distinct from the block-addressed rtlsdr_read_reg/
   write_reg in RTLSDR.h. Exposed here (not static) because
   milestone_tune_test in RTLSDR.c toggles the I2C repeater (page 1,
   reg 0x01) directly around tuner init/tune calls. */
int rtlsdr_demod_read_reg(const char *dev, int page, int addr, int len);
int rtlsdr_demod_write_reg(const char *dev, int page, int addr, int val,
                            int len);

/* ---- baseband/tuner bringup ---- */
void rtlsdr_init_baseband(const char *dev);
int rtlsdr_probe_tuner(const char *dev);
int rtlsdr_tuner_postinit(const char *dev);
int rtlsdr_set_sample_rate(const char *dev, unsigned long samp_rate);
void rtlsdr_reset_buffer(const char *dev);

/* Sets the demod's digital IF-frequency registers to freq (Hz).
   Exposed for callers to re-sync after R82XX.c's
   r82xx_set_bandwidth() updates the tuner's actual analog IF -- see
   docs/PLAN.md for why the two must be kept in agreement. Call with
   the I2C repeater enabled. Returns 0 on success, negative on I2C
   failure. */
int rtlsdr_set_if_freq(const char *dev, unsigned long freq);

/* ---- raw bulk stream I/O ----
   OS_Find/OS_GBPB, bypassing stdio entirely -- fopen()/fread() never
   worked against this DeviceFS stream at any size, for reasons never
   root-caused (see docs/PLAN.md). os_gbpb_read4 returns bytes actually
   read (0..len), or -1 on error. Reads above 1024 bytes are known to
   block for ~110s before erroring -- callers must chunk accordingly.

   IMPORTANT (found in phase 2, see docs/PLAN.md milestone 5): in the
   DEFAULT blocking mode, a stream opened this way pads short reads to
   the requested size instead of returning an honest short count --
   os_gbpb_read4() will claim "len bytes transferred" even when only a
   couple of bytes of genuine USB data actually arrived, silently
   zero-filling the rest. This is a real, confirmed RISC OS DeviceFS
   USB characteristic (see the "DeviceFS USB technical" wiki page and
   DeviceFS's own Doc/NonBlock), not a bug in this code. Call
   os_args_set_nonblocking(handle, 1) once after opening the stream to
   get HONEST short-read counts (confirmed on real hardware: with
   non-blocking mode enabled, os_gbpb_read4's return value always
   matches how many bytes were genuinely written, verified via a
   sentinel-fill test) -- callers MUST then handle got < len as normal
   (accumulate across multiple calls) and got == 0 as "nothing new yet",
   not as an error. */
int os_find_open(const char *path);
void os_find_close(int handle);
int os_gbpb_read4(int handle, unsigned char *buf, int len);
/* Silences os_gbpb_read4()'s stderr messages (for Wimp apps), and returns
   the text of its most recent error. */
void os_set_quiet(int quiet);
const char *os_last_error(void);

/* Bytes waiting unread in an open USB stream's DeviceFS buffer, or -1. */
int usb_stream_used_bytes(const char *device_name, int stream_handle);

/* RMA block claim/release (OS_Module 6/7); NULL on failure. */
unsigned char *os_rma_claim(int size);
void os_rma_free(unsigned char *p);
int os_args_set_nonblocking(int handle, int enable);

#endif /* DRIVER_H */
