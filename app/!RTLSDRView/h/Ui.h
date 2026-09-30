/* Ui.h -- the receiver's control panel: every button, field and display
   text in the main window's top area, built from one table (Ui.c).
   Clicks and keys come in from SpecView.c; the panel reads and writes the
   receiver state in Receiver.h, and calls back into SpecView.c to retune
   the hardware.

   Layout rules (button text is in the proportional desktop font, about 22
   OS units for a capital and less for lower case, and an icon's text is
   clipped to the icon less its border): a label needs about
   22 x characters + ~32 units of width.

   C89 only (Norcroft). */

#ifndef UI_H
#define UI_H

#include "DeskLib:Wimp.h"

/* Control ids. The first UI_N_LEGACY are acted on by SpecView.c (stream,
   demod, AGC, gain, frequency step); the rest by the panel itself. */
enum {
    UI_STREAM, UI_DEM, UI_AGC, UI_GAIN_DN, UI_GAIN_UP,
    UI_FDN, UI_FUP,
    UI_N_LEGACY,
    UI_MODE_NFM = UI_N_LEGACY, UI_MODE_WFM, UI_MODE_AM,
    UI_FREQ, UI_FREQ_UNIT,
    UI_STEP,
    UI_BW_DN, UI_BW_VAL, UI_BW_UP,
    UI_SQ_DN, UI_SQ_VAL, UI_SQ_UP,
    UI_VOL_DN, UI_VOL_VAL, UI_VOL_UP, UI_MUTE,
    UI_COUNT
};

/* Work-area y of the bottom of the last row of controls (negative). */
#define UI_PANEL_BOTTOM  (-200)

/* Creates all the controls in win. tune(hz) must retune the dongle and
   return non-zero if it locked (having updated rcv.freq_hz itself). */
void ui_create(window_handle win, int (*tune)(unsigned long hz));

/* The icon handle of a control (for the legacy ids SpecView.c drives). */
icon_handle ui_icon(int id);

/* Handles a mouse click on an icon. Returns 1 if the icon belongs to the
   panel proper (clicks on it should not go any further); 0 for any other
   icon, including the legacy controls. */
int ui_click(icon_handle icon, int select, int adjust);

/* Handles a key press with the caret in icon. Returns 1 if the key was
   consumed; 0 means the caller should pass it to Wimp_ProcessKey(). Keys
   are only interpreted while the caret is in the frequency field. */
int ui_key(int code, icon_handle caret_icon);

/* Re-reads the receiver state into every control. */
void ui_refresh(void);

#endif
