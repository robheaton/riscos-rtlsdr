/* Ui.c -- see h/Ui.h.

   The controls are described in one table (ui_items[]): kind, column range,
   row, label or text buffer. ui_create() walks it and creates the icons;
   ui_click() and ui_refresh() work from the same ids. Adding a control is
   a line in the table, an enum entry, and a case in ui_click().

   Mouse conventions: Select = the normal action, Adjust = the fine one
   (a fifth of the step) for bandwidth, squelch and volume, and the
   opposite end of the step list for the step button.

   C89 only (Norcroft): declarations at the top of each block, no //
   comments. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "DeskLib:Core.h"
#include "DeskLib:Wimp.h"
#include "DeskLib:WimpSWIs.h"
#include "DeskLib:Icon.h"
#include "Receiver.h"
#include "Ui.h"

#define PANEL_TOP   (-48)    /* top of the first row; the status text sits
                                above it */
#define ROW_PITCH   40
#define ICON_H      32
#define TXT_LEN     20

#define KEY_RETURN  13
#define KEY_ESCAPE  27
#define KEY_DOWN    0x18E
#define KEY_UP      0x18F

/* what sort of icon */
#define K_BUTTON      0   /* clickable, fixed label */
#define K_BUTTON_IND  1   /* clickable, text changes (indirected) */
#define K_VALUE       2   /* display only, text changes (indirected) */
#define K_LABEL       3   /* display only, fixed text, no border */
#define K_FIELD       4   /* writable (indirected) */

typedef struct {
    int id;
    int kind;
    int x0, x1;
    int row;                 /* 0 = UI_PANEL_TOP, 1 = one pitch lower... */
    const char *label;       /* fixed text (initial text for indirected) */
} ui_item;

/* Widths: the button text is in the proportional desktop font, where capital
   letters run to about 22 units: allow that x characters + 32. The window is
   640 units wide. */
static const ui_item ui_items[] = {
    { UI_STREAM,    K_BUTTON,      8, 176, 0, "STREAM" },
    { UI_DEM,       K_BUTTON,    184, 272, 0, "DEM" },
    { UI_AGC,       K_BUTTON,    288, 372, 0, "AGC" },
    { UI_GAIN_DN,   K_BUTTON,    380, 420, 0, "-" },
    { UI_GAIN_UP,   K_BUTTON,    428, 468, 0, "+" },

    { UI_MODE_NFM,  K_BUTTON,      8,  92, 1, "NFM" },
    { UI_MODE_WFM,  K_BUTTON,    100, 184, 1, "WFM" },
    { UI_MODE_AM,   K_BUTTON,    192, 256, 1, "AM" },
    { UI_MODE_USB,  K_BUTTON,    264, 348, 1, "USB" },
    { UI_MODE_LSB,  K_BUTTON,    356, 440, 1, "LSB" },
    { UI_MODE_CW,   K_BUTTON,    448, 512, 1, "CW" },

    { UI_FREQ,      K_FIELD,       8, 192, 2, "97.400000" },
    { UI_FREQ_UNIT, K_LABEL,     196, 260, 2, "MHz" },
    { UI_FDN,       K_BUTTON,    268, 316, 2, "F-" },
    { UI_FUP,       K_BUTTON,    324, 372, 2, "F+" },

    { UI_STEP,      K_BUTTON_IND,  8, 184, 3, "Step 100k" },
    { UI_BW_DN,     K_BUTTON,    200, 240, 3, "-" },
    { UI_BW_VAL,    K_VALUE,     240, 384, 3, "BW 150k" },
    { UI_BW_UP,     K_BUTTON,    384, 424, 3, "+" },

    { UI_SQ_DN,     K_BUTTON,      8,  48, 4, "-" },
    { UI_SQ_VAL,    K_VALUE,      48, 176, 4, "Sq off" },
    { UI_SQ_UP,     K_BUTTON,    176, 216, 4, "+" },
    { UI_VOL_DN,    K_BUTTON,    240, 280, 4, "-" },
    { UI_VOL_VAL,   K_VALUE,     280, 424, 4, "Vol 100" },
    { UI_VOL_UP,    K_BUTTON,    424, 464, 4, "+" },
    { UI_MUTE,      K_BUTTON,    480, 576, 4, "Mute" }
};

#define N_ITEMS ((int)(sizeof(ui_items) / sizeof(ui_items[0])))

static window_handle win_g;
static icon_handle icon_g[UI_COUNT];
static char text_g[UI_COUNT][TXT_LEN];     /* indirected text buffers */
static int (*tune_g)(unsigned long hz);

static void make_icon(const ui_item *it)
{
    icon_createblock cb;
    icon_handle icon;
    os_error *err;
    int y1;

    y1 = PANEL_TOP - ROW_PITCH * it->row;
    memset(&cb, 0, sizeof(cb));
    cb.window = win_g;
    cb.icondata.workarearect.min.x = it->x0;
    cb.icondata.workarearect.min.y = y1 - ICON_H;
    cb.icondata.workarearect.max.x = it->x1;
    cb.icondata.workarearect.max.y = y1;

    cb.icondata.flags.data.text = 1;
    cb.icondata.flags.data.vcentre = 1;
    cb.icondata.flags.data.foreground = colour_BLACK;
    cb.icondata.flags.data.background = colour_GREY1;

    switch (it->kind) {
    case K_BUTTON:
    case K_BUTTON_IND:
        cb.icondata.flags.data.border = 1;
        cb.icondata.flags.data.hcentre = 1;
        cb.icondata.flags.data.filled = 1;
        cb.icondata.flags.data.buttontype = 3;     /* Click */
        break;
    case K_VALUE:
        cb.icondata.flags.data.border = 1;
        cb.icondata.flags.data.hcentre = 1;
        cb.icondata.flags.data.filled = 1;
        cb.icondata.flags.data.buttontype = 0;     /* ignores clicks */
        cb.icondata.flags.data.background = colour_WHITE;
        break;
    case K_LABEL:
        cb.icondata.flags.data.hcentre = 1;
        cb.icondata.flags.data.buttontype = 0;
        break;
    case K_FIELD:
        cb.icondata.flags.data.border = 1;
        cb.icondata.flags.data.filled = 1;
        cb.icondata.flags.data.buttontype = 15;    /* writable */
        cb.icondata.flags.data.background = colour_WHITE;
        break;
    }

    if (it->kind == K_BUTTON || it->kind == K_LABEL) {
        strncpy(cb.icondata.data.text, it->label, wimp_MAXNAME - 1);
        cb.icondata.data.text[wimp_MAXNAME - 1] = '\0';
    } else {
        strncpy(text_g[it->id], it->label, TXT_LEN - 1);
        text_g[it->id][TXT_LEN - 1] = '\0';
        cb.icondata.flags.data.indirected = 1;
        cb.icondata.data.indirecttext.buffer = text_g[it->id];
        cb.icondata.data.indirecttext.validstring =
            (it->kind == K_FIELD) ? (char *)"A0-9." : (char *)-1;
        cb.icondata.data.indirecttext.bufflen = TXT_LEN;
    }

    err = Wimp_CreateIcon(&cb, &icon);
    if (err != NULL) {
        /* A failure here is a programming error (bad flags); there is no
           useful recovery, and a half-built panel is worse than none. */
        Wimp_ReportError(err, 0, "RTLSDRView");
        exit(1);
    }
    icon_g[it->id] = icon;
}

static void set_selected(int id, int selected)
{
    if (selected) {
        Icon_Select(win_g, icon_g[id]);
    } else {
        Icon_Deselect(win_g, icon_g[id]);
    }
}

/* Sets an indirected icon's text only if it changed (avoids needless
   redraws). Formats into a scratch buffer first: Icon_SetText() copies into
   the icon's own buffer. */
static void set_text(int id, const char *s)
{
    if (strcmp(text_g[id], s) != 0) {
        Icon_SetText(win_g, icon_g[id], s);
    }
}

static int caret_in_freq_field(void)
{
    caret_block caret;

    if (Wimp_GetCaretPosition(&caret) != NULL) {
        return 0;
    }
    return caret.window == win_g && caret.icon == icon_g[UI_FREQ];
}

static void refresh_freq_text(int force)
{
    char b[TXT_LEN];

    if (!force && caret_in_freq_field()) {
        return;              /* do not stomp on a half-typed frequency */
    }
    sprintf(b, "%lu.%06lu", rcv.freq_hz / 1000000UL,
            rcv.freq_hz % 1000000UL);
    set_text(UI_FREQ, b);
}

void ui_refresh(void)
{
    char b[TXT_LEN];
    char v[12];
    int wfm, m;

    for (m = 0; m < RX_NMODES; m++) {
        set_selected(UI_MODE_NFM + m, rcv.mode == m);
    }

    refresh_freq_text(0);

    receiver_fmt_hz(v, (int)sizeof(v), receiver_step_hz());
    sprintf(b, "Step %s", v);
    set_text(UI_STEP, b);

    wfm = (rcv.mode == RX_MODE_WFM);
    receiver_fmt_hz(v, (int)sizeof(v), receiver_bw_hz());
    sprintf(b, "BW %s", v);
    set_text(UI_BW_VAL, b);
    Icon_SetShade(win_g, icon_g[UI_BW_DN], wfm);
    Icon_SetShade(win_g, icon_g[UI_BW_UP], wfm);

    if (rcv.squelch == 0) {
        strcpy(b, "Sq off");
    } else {
        sprintf(b, "Sq %d", rcv.squelch);
    }
    set_text(UI_SQ_VAL, b);

    sprintf(b, "Vol %d", rcv.volume);
    set_text(UI_VOL_VAL, b);
    set_selected(UI_MUTE, rcv.muted);
}

icon_handle ui_icon(int id)
{
    return icon_g[id];
}

void ui_create(window_handle win, int (*tune)(unsigned long hz))
{
    int i;

    win_g = win;
    tune_g = tune;
    for (i = 0; i < N_ITEMS; i++) {
        make_icon(&ui_items[i]);
    }
    ui_refresh();
}

static int clamp(int v, int lo, int hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static void set_mode(int m)
{
    if (m == rcv.mode) {
        return;
    }
    rcv.mode = m;
    receiver_apply_mode();
    /* The SSB modes want the dongle tuned a little off the dial frequency and
       the others do not, so a change between the two kinds retunes. */
    if (tune_g != NULL) {
        tune_g(rcv.freq_hz);
    }
    ui_refresh();
}

/* Applies the text in the frequency field. Anything unparsable or out of
   range is clamped or reverted; the field always ends up showing the
   frequency the receiver is really on. */
static void apply_freq_text(void)
{
    double mhz;
    unsigned long hz;

    mhz = atof(text_g[UI_FREQ]);
    if (mhz > 0.0) {
        hz = receiver_clamp_freq((unsigned long)(mhz * 1.0e6 + 0.5));
        if (hz != rcv.freq_hz && tune_g != NULL) {
            tune_g(hz);
        }
    }
    refresh_freq_text(1);
}

static void step_frequency(int direction)
{
    unsigned long hz;
    unsigned long step;

    step = (unsigned long)receiver_step_hz();
    if (direction > 0) {
        hz = receiver_clamp_freq(rcv.freq_hz + step);
    } else {
        hz = (rcv.freq_hz > step) ? rcv.freq_hz - step : 0UL;
        hz = receiver_clamp_freq(hz);
    }
    if (hz != rcv.freq_hz && tune_g != NULL) {
        tune_g(hz);
    }
    refresh_freq_text(1);
}

int ui_click(icon_handle icon, int select, int adjust)
{
    int id, fine, inc, bw;

    (void)select;
    for (id = 0; id < UI_COUNT; id++) {
        if (icon_g[id] == icon) {
            break;
        }
    }
    if (id >= UI_COUNT) {
        return 0;
    }
    if (id < UI_N_LEGACY) {
        return 0;     /* stream, demod, gain, F-/F+: SpecView.c's */
    }
    fine = adjust ? 1 : 0;

    switch (id) {
    case UI_MODE_NFM:
    case UI_MODE_WFM:
    case UI_MODE_AM:
    case UI_MODE_USB:
    case UI_MODE_LSB:
    case UI_MODE_CW:
        set_mode(id - UI_MODE_NFM);
        break;
    case UI_STEP:
        receiver_step_next(fine ? -1 : 1);
        ui_refresh();
        break;
    case UI_BW_DN:
    case UI_BW_UP:
        inc = receiver_bw_inc(rcv.mode);
        if (inc > 0) {
            if (fine) {
                inc /= 5;
            }
            bw = rcv.bw_hz[rcv.mode] + ((id == UI_BW_UP) ? inc : -inc);
            rcv.bw_hz[rcv.mode] = clamp(bw, receiver_bw_min(rcv.mode),
                                        receiver_bw_max(rcv.mode));
            receiver_apply_bw();
            ui_refresh();
        }
        break;
    case UI_SQ_DN:
    case UI_SQ_UP:
        inc = fine ? 1 : 5;
        rcv.squelch = clamp(rcv.squelch + ((id == UI_SQ_UP) ? inc : -inc),
                            0, 100);
        receiver_apply_squelch();
        ui_refresh();
        break;
    case UI_VOL_DN:
    case UI_VOL_UP:
        inc = fine ? 1 : 5;
        rcv.volume = clamp(rcv.volume + ((id == UI_VOL_UP) ? inc : -inc),
                           0, 100);
        receiver_apply_volume();
        ui_refresh();
        break;
    case UI_MUTE:
        rcv.muted = !rcv.muted;
        receiver_apply_volume();
        ui_refresh();
        break;
    default:
        break;        /* the frequency field and the labels: nothing to do */
    }
    return 1;
}

int ui_key(int code, icon_handle caret_icon)
{
    if (caret_icon != icon_g[UI_FREQ]) {
        return 0;
    }
    switch (code) {
    case KEY_RETURN:
        apply_freq_text();
        return 1;
    case KEY_ESCAPE:
        refresh_freq_text(1);
        return 1;
    case KEY_UP:
        step_frequency(1);
        return 1;
    case KEY_DOWN:
        step_frequency(-1);
        return 1;
    default:
        return 0;
    }
}
