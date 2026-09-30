/* Ui.h -- the receiver's control panel: mode buttons, the typed frequency
   field, step, bandwidth, squelch and volume controls. Built into the main
   window from a table (Ui.c), driven by clicks and keys that SpecView.c
   passes in, and reading/writing the receiver state in Receiver.h.

   The panel does not know how to tune the hardware: SpecView.c gives it a
   callback for that.

   C89 only (Norcroft). */

#ifndef UI_H
#define UI_H

#include "DeskLib:Wimp.h"

/* Work-area position of the panel's first row (top of the first icon row,
   negative, OS units) and the number of rows it occupies. */
#define UI_PANEL_TOP     (-140)
#define UI_PANEL_ROWS    3
#define UI_PANEL_BOTTOM  (UI_PANEL_TOP - 40 * UI_PANEL_ROWS + 8)

/* Creates the controls in win. tune(hz) must retune the dongle and return
   non-zero if it locked (and then has updated rcv.freq_hz itself). */
void ui_create(window_handle win, int (*tune)(unsigned long hz));

/* Handles a mouse click on an icon. select/adjust say which button.
   Returns 1 if the icon belonged to the panel (clicks on it should not go
   any further), else 0. */
int ui_click(icon_handle icon, int select, int adjust);

/* Handles a key press with the caret in icon. Returns 1 if the key was
   consumed; 0 means the caller should pass it to Wimp_ProcessKey(). Keys
   are only interpreted while the caret is in the frequency field. */
int ui_key(int code, icon_handle caret_icon);

/* Re-reads the receiver state into every control (after a retune, a mode
   change, a load...). */
void ui_refresh(void);

#endif
