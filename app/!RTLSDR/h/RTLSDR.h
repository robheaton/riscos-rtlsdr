/* RTLSDR.h -- constants ported from osmocom librtlsdr (src/librtlsdr.c,
   src/tuner_r82xx.c). C89: no // comments, no mid-block declarations
   assumed by callers. */

#ifndef RTLSDR_H
#define RTLSDR_H

/* Known VID/PID for generic RTL2832U dongles. */
#define RTL_VENDOR_ID_BDA    0x0BDAu
#define RTL_PRODUCT_ID_2832  0x2832u
#define RTL_PRODUCT_ID_2838  0x2838u

/* Register blocks (librtlsdr enum blocks). Packed into the USB control
   request's wIndex high byte: read wIndex = block<<8, write wIndex =
   (block<<8)|0x10. */
#define RTLSDR_BLOCK_DEMOD  0
#define RTLSDR_BLOCK_USB    1
#define RTLSDR_BLOCK_SYS    2
#define RTLSDR_BLOCK_TUN    3
#define RTLSDR_BLOCK_ROM    4
#define RTLSDR_BLOCK_IR     5
#define RTLSDR_BLOCK_I2C    6

/* USB block registers used during init (see rtlsdr_init_baseband). */
#define USB_SYSCTL       0x2000u
#define USB_CTRL         0x2010u
#define USB_STAT         0x2014u
#define USB_EPA_CFG      0x2144u
#define USB_EPA_CTL      0x2148u
#define USB_EPA_MAXPKT   0x2158u
#define USB_EPA_MAXPKT_2 0x215Au
#define USB_EPA_FIFO_CFG 0x2160u

/* SYS block registers. */
#define DEMOD_CTL   0x3000u
#define GPO         0x3001u
#define GPI         0x3002u
#define GPOE        0x3003u
#define GPD         0x3004u
#define SYSINTE     0x3005u
#define SYSINTS     0x3006u
#define GP_CFG0     0x3007u
#define GP_CFG1     0x3008u
#define DEMOD_CTL_1 0x300Bu

/* I2C addresses of known tuner chips, and their "are you there" check
   register + expected value. Confirmed real hardware (see docs/PLAN.md)
   is an RTL-SDR Blog V4, which uses R828D (not plain R820T2) plus a
   built-in HF upconverter -- that's now the v1 target.

   CORRECTION: an earlier version of this file had both at 0xA0 -- wrong,
   that's EEPROM_ADDR (a completely different I2C device, the EEPROM
   chip). Verified against the actual rtlsdr-blog fork's
   include/tuner_r82xx.h #define lines: R820T and R828D are DIFFERENT
   7-bit I2C addresses. Both share the same check register/value though
   (rtlsdr_open() tries R820T's address first, then R828D's, at the same
   check register -- whichever responds with 0x69 identifies the chip). */
#define R820T_I2C_ADDR      0x34u
#define R828D_I2C_ADDR      0x74u
#define R82XX_CHECK_ADDR    0x00u
#define R82XX_CHECK_VAL     0x69u

/* V4-specific tuner constants, from the rtlsdr-blog fork (mainline
   librtlsdr doesn't have V4 support at all). Needed for milestone 3+. */
#define R828D_XTAL_FREQ     16000000  /* used for non-V4 R828D boards; the
                                          V4 itself keeps the RTL2832U's
                                          default 28.8MHz xtal instead --
                                          see rtlsdr_open()'s tuner-type
                                          switch, only sets this when NOT
                                          detected as "RTLSDRBlog"/"Blog V4" */
#define R82XX_IF_FREQ       3570000  /* 3.57MHz intermediate frequency,
                                         set via rtlsdr_set_if_freq() after
                                         baseband init, both R820T/R828D */
#define V4_UPCONVERT_THRESHOLD_HZ  28800000  /* 28.8MHz -- below this, the
                                                 V4 upconverts (adds this
                                                 offset) before PLL tuning */

/* GPIO control (SYS block), used for the V4 upconverter switch (GPIO 5)
   and bias tee (GPIO 0). rtlsdr_set_gpio_output()/rtlsdr_set_gpio_bit()
   from the rtlsdr-blog fork operate on these three SYS-block registers:
   clear the bit in GPD (direction/mode), set it in GPOE (output enable),
   then read-modify-write GPO to the desired level. */
#define GPIO_UPCONVERT_PIN  5
#define GPIO_BIAS_TEE_PIN   0

/* EEPROM byte 7, bit 1 (0x02): if CLEAR, the dongle's bias tee is forced
   on in software regardless of user setting (rtlsdr_open() checks this
   and calls rtlsdr_set_bias_tee(dev, 1) if so). Our own milestone 2
   EEPROM dump read byte 7 as 0x00 (from "EEPROM[0x06..0x07]: 0000") on
   this specific unit -- bit clear, so force-on applies here. */
#define EEPROM_FORCE_BT_BYTE_OFFSET  7
#define EEPROM_FORCE_BT_BIT          0x02u

/* USB control request direction/type, matching librtlsdr's CTRL_IN/CTRL_OUT
   (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN/OUT). RISC OS's
   DeviceFS "USB Control Request" call takes the raw bmRequestType byte
   directly -- see rtlsdr_ctrl_read/write in c/RTLSDR.c. */
#define CTRL_BMREQ_VENDOR_IN   0xC0u  /* 0x80 (device-to-host) | 0x40 (vendor) */
#define CTRL_BMREQ_VENDOR_OUT  0x40u  /* 0x00 (host-to-device) | 0x40 (vendor) */

/* Shared USB control-transfer primitive, defined in c/RTLSDR.c, used by
   both RTLSDR.c itself and c/R82XX.c (the tuner driver) -- avoids
   duplicating the SWI/DeviceFS_CallDevice plumbing in two files. */
int usb_ctrl_transfer(const char *device_name, int bm_request_type,
                       int b_request, int w_value, int w_index,
                       int w_length, void *data);

/* Also shared with R82XX.c, for GPIO control (SYS block registers
   GPD/GPOE/GPO above) -- the V4 upconverter switch and bias tee. */
int rtlsdr_read_reg(const char *dev, int block, int addr, int len);
int rtlsdr_write_reg(const char *dev, int block, int addr, int val,
                      int len);

/* Streaming (librtlsdr defaults). RTLSDR_BULK_ENDPOINT confirmed against
   real hardware via *USBConfInfo (see docs/PLAN.md): interface 0 has
   "Endpoint 1, Bulk IN, 512 bytes" -- matches. Interface 0 is the one to
   select explicitly (special field "interface/0") since this dongle
   exposes two interfaces (config name "USB2.0-Bulk&Iso"; interface 1's
   role isn't confirmed yet, presumably an unused iso alternate). */
#define RTLSDR_BULK_ENDPOINT      1      /* libusb ep 0x81 = IN, endpoint 1 */
#define RTLSDR_BULK_INTERFACE     0
#define RTLSDR_BULK_MAXPACKET     512    /* confirmed: High Speed, 512B */
#define RTLSDR_DEFAULT_BUF_LEN    (16 * 32 * 512)  /* 262144 bytes */
#define RTLSDR_DEFAULT_BUF_COUNT  15

#endif /* RTLSDR_H */
