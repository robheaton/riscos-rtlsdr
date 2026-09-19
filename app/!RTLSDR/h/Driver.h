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

/* ---- raw bulk stream I/O ----
   OS_Find/OS_GBPB, bypassing stdio entirely -- fopen()/fread() never
   worked against this DeviceFS stream at any size, for reasons never
   root-caused (see docs/PLAN.md). os_gbpb_read4 returns bytes actually
   read (0..len), or -1 on error. Reads above 1024 bytes are known to
   block for ~110s before erroring -- callers must chunk accordingly. */
int os_find_open(const char *path);
void os_find_close(int handle);
int os_gbpb_read4(int handle, unsigned char *buf, int len);

#endif /* DRIVER_H */
