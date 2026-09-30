/* Trace.h -- run-time statistics and an optional log file for !RTLSDRView.

   Everything is bucketed per second of wall-clock time (centisecond
   clock): how many bytes came off the USB stream, how many read calls
   returned data, how long the app was busy vs. how long the rest of the
   desktop held the CPU between two polls of this app (a "foreign gap"),
   where the time went (USB reads, demodulation, FFT, redraw), and what
   the audio ring was doing (how far ahead of the play position it ran,
   underruns). A stall shows up in the log as a row with few or no bytes
   and a big foreign gap; a starved USB pipe as bytes/second below the
   nominal rate while the gaps are small.

   The recent figures also drive the on-screen readout, so a screenshot
   taken at any moment says how things have been going for the last few
   seconds (not just what the peak-held maximum was).

   The log is only written when the app quits (a file write to a network
   share can itself stall the desktop for a while), to the path in the
   RTLSDRView$Log system variable if that is set.

   C89 only (Norcroft). */

#ifndef TRACE_H
#define TRACE_H

/* Room for one event line (see trace_event()), including the NUL. */
#define TRACE_EVENT_TEXT 56

/* log_path may be NULL or empty (statistics only, no file). config_text
   is copied into the log header. Call once at start-up. */
void trace_init(const char *log_path, const char *config_text);

/* Call at the very start and end of every poll of the app's null-event
   handler, with the current centisecond clock. */
void trace_tick_begin(unsigned int now_cs);
void trace_tick_end(unsigned int now_cs);

/* One os_gbpb_read4() call: bytes returned (0 = nothing buffered) and
   how many centiseconds the call itself took. */
void trace_read(int nbytes, unsigned int usb_cs);

/* Time spent in the demodulator + audio feed, and in FFT frames (and how
   many), and in the display update/redraw preparation. */
void trace_dsp_cs(unsigned int cs);
void trace_fft_cs(unsigned int cs, int frames);
void trace_disp_cs(unsigned int cs);

/* A timestamped one-line note (a click, a retune and how long it took,
   a stream recovery...). text is truncated to fit. */
void trace_event(unsigned int now_cs, const char *text);

/* Sample rate (thousands of complex samples per second) actually read
   over the last few complete seconds, and the longest foreign gap in that
   window (ms). Both 0 until at least one second has completed. */
void trace_recent(double *ksps, unsigned int *foreign_gap_ms);

/* Longest foreign gap seen since start-up, ms. */
unsigned int trace_gap_all_ms(void);

/* Writes the log (if a path was given). Returns 0 on success or if
   there is nothing to write, -1 if the file could not be written. */
int trace_write(void);

#endif
