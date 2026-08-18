/*
 * Copyright (c) 2026 The zmk-joycon2 Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Bridges decoded Joy-Con 2 state onto the gamepad HID report that ZMK
 * sends to the host (CONFIG_ZMK_GAMEPAD -- report ID 4 inside ZMK's own
 * single HID interface).
 *
 * An earlier attempt used zmk-hid-io, which registers a SECOND
 * HID-over-GATT service. That broke BLE HID on Android outright: the
 * keyboard stopped pairing entirely while USB still worked. Two HIDS
 * primary services in one GATT database is evidently more than Android's
 * HID host accepts. Adding a report to the existing service is both the
 * conventional way composite HID devices work and the way ZMK already
 * carries keyboard, consumer and mouse together.
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/endpoints.h>
#include <zmk/hid.h>

#include <zmk/joycon2/buttons.h>
#include <zmk/joycon2/gamepad.h>
#if IS_ENABLED(CONFIG_ZMK_JOYCON2_MOUSE)
#include <zmk/joycon2/mouse.h>
#endif
#include <zmk/joycon2/zmk_compat.h>

LOG_MODULE_REGISTER(joycon2_gamepad, CONFIG_ZMK_LOG_LEVEL);


/* Matches ZMK_HID_GAMEPAD_NUM_BUTTONS. Indices below are 0-based; a
 * gamepad tester shows them as buttons 1-32. */
#define JOYCON2_GAMEPAD_NUM_BUTTONS ZMK_HID_GAMEPAD_NUM_BUTTONS

/* Logical slots.
 *
 * The index values are NOT consecutive, and the order past button 10 is not
 * the obvious one. Android's HID driver follows the Linux BTN_* gamepad
 * ordering, which contains two vestigial entries no app reads and puts the
 * stick clicks at the very end:
 *
 *   HID button: 1  2  3  4  5  6  7   8   9   10  11      12     13    14      15
 *   Android:    A  B  C  X  Y  Z  L1  R1  L2  R2  Select  Start  Mode  ThumbL  ThumbR
 *   keycode:    96 97 98 99 100 101 102 103 104 105 109   108    110   106     107
 *
 * So the face buttons are 1, 2, 4, 5 (skipping C) and the shoulders start at
 * 7 (skipping Z). All of this was derived by reading back the keycodes a
 * gamepad tester reported for known inputs. Values below are 0-based, so
 * index = HID button - 1.
 */
enum joycon2_pad_button {
    PAD_FACE_DOWN = 0,  /* A      */
    PAD_FACE_RIGHT = 1, /* B      */
    PAD_FACE_LEFT = 3,  /* X      */
    PAD_FACE_UP = 4,    /* Y      */
    /* BUTTON_C and BUTTON_Z are vestigial: no ordinary game reads them,
     * which makes them the safe place to put bindings for a specific app,
     * since nothing standard can collide with them. */
    PAD_C = 2,
    PAD_Z = 5,
    PAD_L1 = 6,
    PAD_R1 = 7,
    PAD_L2 = 8,
    PAD_R2 = 9,
    PAD_SELECT = 10,
    PAD_START = 11,
    PAD_MODE = 12,
    PAD_THUMBL = 13, /* left stick click, "L3"  */
    PAD_THUMBR = 14, /* right stick click, "R3" */
    /* Past ThumbR, Android exposes BUTTON_1..16 (keycodes 188+), which are
     * generic and not D-pad keycodes -- the D-pad goes through the report's
     * hat switch instead (see dpad_to_hat below). */
};

/* Index = HID button number - 1. 0 means "nothing mapped at this index". */
struct joycon2_profile_map {
    uint32_t buttons[JOYCON2_GAMEPAD_NUM_BUTTONS];
    /* Send this half's D-pad through the hat switch. Only duo does: in solo
     * the D-pad stands in for the missing face cluster instead. */
    bool dpad_to_hat;
};

/* SOLO: a single Joy-Con held UPRIGHT in one hand.
 *
 * Its own shoulder pair keeps its natural side, and the rail buttons stand in
 * for the pair the missing half would have provided -- so on the right half
 * SR/SL become L1/L2, and on the left half they become R1/R2. The half's
 * single stick acts as the left stick, so its click is ThumbL.
 *
 * The system buttons are deliberately not the conventional assignment: Home
 * (right) and Capture (left) are Start, while Plus/Minus are Mode. With only
 * one half in hand there is just one menu button to go around, and this is
 * what testing settled on.
 */
static const struct joycon2_profile_map solo_left = {
    .buttons =
        {
            /* The D-pad serves as the face cluster -- there is no second half
             * to provide one, and it falls under the thumb. Positions match
             * the face buttons they stand in for. */
            [PAD_FACE_DOWN] = JC2_DOWN,
            [PAD_FACE_RIGHT] = JC2_RIGHT,
            [PAD_FACE_LEFT] = JC2_LEFT,
            [PAD_FACE_UP] = JC2_UP,
            [PAD_L1] = JC2_L,
            [PAD_L2] = JC2_ZL,
            /* Swapped relative to the right half: held upright, the left
             * half's rail is mirrored, so SL is the one that falls under
             * the finger first. */
            [PAD_R1] = JC2_SL_L,
            [PAD_R2] = JC2_SR_L,
            [PAD_START] = JC2_CAPTURE,
            [PAD_MODE] = JC2_MINUS,
            [PAD_THUMBL] = JC2_LSTK,
        },
};

static const struct joycon2_profile_map solo_right = {
    .buttons =
        {
            /* Nintendo's face buttons by position, not by letter: Nintendo B
             * is the bottom button so it becomes A, Nintendo A is on the
             * right so it becomes B, and likewise Y->X and X->Y. */
            [PAD_FACE_DOWN] = JC2_B,
            [PAD_FACE_RIGHT] = JC2_A,
            [PAD_FACE_LEFT] = JC2_Y,
            [PAD_FACE_UP] = JC2_X,
            [PAD_L1] = JC2_SR_R,
            [PAD_L2] = JC2_SL_R,
            [PAD_R1] = JC2_R,
            [PAD_R2] = JC2_ZR,
            [PAD_SELECT] = JC2_C,
            [PAD_START] = JC2_HOME,
            [PAD_MODE] = JC2_PLUS,
            [PAD_THUMBL] = JC2_RSTK,
        },
};

/* DUO: both halves held upright as one pad, so every control keeps its
 * natural role -- and with all the system buttons available, they take their
 * conventional assignments rather than the solo compromises above. The two
 * tables map disjoint physical buttons and each half only writes its own
 * indices, so the host sees one merged pad. The rail buttons are free here,
 * since both real shoulder pairs are present. */
static const struct joycon2_profile_map duo_left = {
    .buttons =
        {
            [PAD_L1] = JC2_L,
            [PAD_L2] = JC2_ZL,
            /* Minus goes to the app-usable Z rather than Select, by request:
             * it is reachable but ignored by ordinary games. Capture takes
             * Select in its place. */
            [PAD_Z] = JC2_MINUS,
            [PAD_SELECT] = JC2_CAPTURE,
            [PAD_THUMBL] = JC2_LSTK,
        },
    .dpad_to_hat = true,
};

static const struct joycon2_profile_map duo_right = {
    .buttons =
        {
            [PAD_FACE_DOWN] = JC2_B,
            [PAD_FACE_RIGHT] = JC2_A,
            [PAD_FACE_LEFT] = JC2_Y,
            [PAD_FACE_UP] = JC2_X,
            [PAD_R1] = JC2_R,
            [PAD_R2] = JC2_ZR,
            /* One of the two slots ordinary games ignore, kept for Matt's own
             * app; Minus takes the other (Z) on the left half. */
            [PAD_C] = JC2_C,
            /* Home is Start and Plus is Mode, matching solo, so the same
             * button does the same thing in either profile. */
            [PAD_START] = JC2_HOME,
            [PAD_MODE] = JC2_PLUS,
            [PAD_THUMBR] = JC2_RSTK,
        },
};

/* Per-side, because in duo both halves report independently: sharing this
 * would make each half's report suppress the other's. */
struct joycon2_side_state {
    bool have_last;
    uint32_t last_buttons;
    int8_t last_x;
    int8_t last_y;
};

static struct joycon2_side_state side_state[2];

static enum zmk_joycon2_profile current_profile = ZMK_JOYCON2_PROFILE_SOLO;

static const struct joycon2_profile_map *map_for(enum zmk_joycon2_side side) {
    bool right = (side == ZMK_JOYCON2_SIDE_RIGHT);

    if (current_profile == ZMK_JOYCON2_PROFILE_DUO) {
        return right ? &duo_right : &duo_left;
    }
    return right ? &solo_right : &solo_left;
}

void zmk_joycon2_gamepad_set_connected_count(uint8_t count) {
    enum zmk_joycon2_profile profile =
        (count >= 2) ? ZMK_JOYCON2_PROFILE_DUO : ZMK_JOYCON2_PROFILE_SOLO;

    if (profile == current_profile) {
        return;
    }

    current_profile = profile;
    LOG_INF("joycon2: gamepad profile -> %s", profile == ZMK_JOYCON2_PROFILE_DUO ? "DUO" : "SOLO");

    /* Release everything so a button held under the old mapping can't
     * stay stuck down at a HID index the new mapping never touches. */
    zmk_hid_gamepad_clear();
    memset(side_state, 0, sizeof(side_state));
    ZMK_JOYCON2_SEND_GAMEPAD_REPORT();
}

/* Used until the controller's own calibration arrives. Deliberately a
 * plausible deflection rather than the full 12-bit half-span: a stick only
 * travels a fraction of the raw range, and assuming otherwise under-reports
 * badly. */
#define JOYCON2_STICK_DEFAULT_CENTRE 2048
/* Measured on a real left stick, which read centre ~2050 and travel ~1200. */
#define JOYCON2_STICK_DEFAULT_RANGE 1200

/* A stick's centre must sit near the middle of the 12-bit span and its travel
 * must be a believable fraction of it. Erased flash reads as 0xFFF, which
 * without this check is taken as centre 4095 and pegs both axes hard over --
 * seen on a controller whose calibration region was unwritten. */
#define JOYCON2_CAL_CENTRE_MIN 1024
#define JOYCON2_CAL_CENTRE_MAX 3072
#define JOYCON2_CAL_TRAVEL_MIN 200
#define JOYCON2_CAL_TRAVEL_MAX 2048

/* Deadzone as a fraction of each axis's measured travel, so it scales with
 * whatever range the controller reports. The sticks rest slightly off centre
 * and jitter by a few counts; without this the host sees constant drift and
 * a report goes out for every input packet. */
#define JOYCON2_STICK_DEADZONE_NUM 1
#define JOYCON2_STICK_DEADZONE_DEN 8

/* Report only when an axis moves at least this far (in final int8 units),
 * so a resting-but-noisy stick doesn't flood the HID endpoint. */
#define JOYCON2_AXIS_CHANGE_THRESHOLD 3

static struct zmk_joycon2_stick_calib stick_calib[2] = {
    [0] = {.center_x = JOYCON2_STICK_DEFAULT_CENTRE,
           .center_y = JOYCON2_STICK_DEFAULT_CENTRE,
           .max_x = JOYCON2_STICK_DEFAULT_RANGE,
           .max_y = JOYCON2_STICK_DEFAULT_RANGE,
           .min_x = JOYCON2_STICK_DEFAULT_RANGE,
           .min_y = JOYCON2_STICK_DEFAULT_RANGE},
    [1] = {.center_x = JOYCON2_STICK_DEFAULT_CENTRE,
           .center_y = JOYCON2_STICK_DEFAULT_CENTRE,
           .max_x = JOYCON2_STICK_DEFAULT_RANGE,
           .max_y = JOYCON2_STICK_DEFAULT_RANGE,
           .min_x = JOYCON2_STICK_DEFAULT_RANGE,
           .min_y = JOYCON2_STICK_DEFAULT_RANGE},
};

#if IS_ENABLED(CONFIG_ZMK_JOYCON2_MOUSE)
/* Mirrored between the halves -- see the note in joycon2_mouse.c. */
static uint32_t mouse_click_masks(enum zmk_joycon2_side side) {
    return side == ZMK_JOYCON2_SIDE_RIGHT ? (JC2_R | JC2_ZR) : (JC2_L | JC2_ZL);
}
#endif

static size_t side_index(enum zmk_joycon2_side side) {
    return side == ZMK_JOYCON2_SIDE_RIGHT ? 1 : 0;
}

void zmk_joycon2_gamepad_set_calibration(enum zmk_joycon2_side side,
                                          const struct zmk_joycon2_stick_calib *calib) {
    if (!IN_RANGE(calib->center_x, JOYCON2_CAL_CENTRE_MIN, JOYCON2_CAL_CENTRE_MAX) ||
        !IN_RANGE(calib->center_y, JOYCON2_CAL_CENTRE_MIN, JOYCON2_CAL_CENTRE_MAX) ||
        !IN_RANGE(calib->max_x, JOYCON2_CAL_TRAVEL_MIN, JOYCON2_CAL_TRAVEL_MAX) ||
        !IN_RANGE(calib->max_y, JOYCON2_CAL_TRAVEL_MIN, JOYCON2_CAL_TRAVEL_MAX) ||
        !IN_RANGE(calib->min_x, JOYCON2_CAL_TRAVEL_MIN, JOYCON2_CAL_TRAVEL_MAX) ||
        !IN_RANGE(calib->min_y, JOYCON2_CAL_TRAVEL_MIN, JOYCON2_CAL_TRAVEL_MAX)) {
        LOG_WRN("joycon2: implausible stick calibration, keeping defaults");
        return;
    }

    stick_calib[side_index(side)] = *calib;
    LOG_INF("joycon2: stick calibration centre=%u,%u max=%u,%u min=%u,%u", calib->center_x,
            calib->center_y, calib->max_x, calib->max_y, calib->min_x, calib->min_y);
}

/* travel_pos/travel_neg are this axis's measured deflection either side of
 * centre, so full tilt maps to full scale whatever the stick's actual span. */
static int8_t scale_axis(uint16_t raw, uint16_t centre, uint16_t travel_pos, uint16_t travel_neg) {
    int32_t centred = (int32_t)raw - (int32_t)centre;
    int32_t travel = centred >= 0 ? travel_pos : travel_neg;
    int32_t deadzone = (travel * JOYCON2_STICK_DEADZONE_NUM) / JOYCON2_STICK_DEADZONE_DEN;

    if (centred > -deadzone && centred < deadzone) {
        return 0;
    }

    /* Map the travel beyond the deadzone onto -127..127. */
    if (centred > 0) {
        centred -= deadzone;
    } else {
        centred += deadzone;
    }

    int32_t span = travel - deadzone;
    if (span <= 0) {
        return 0;
    }

    return (int8_t)CLAMP((centred * 127) / span, -127, 127);
}


void zmk_joycon2_gamepad_scale_stick(enum zmk_joycon2_side side, uint16_t raw_x, uint16_t raw_y,
                                      int8_t *out_x, int8_t *out_y) {
    const struct zmk_joycon2_stick_calib *cal = &stick_calib[side_index(side)];

    *out_x = scale_axis(raw_x, cal->center_x, cal->max_x, cal->min_x);
    /* HID Y grows downwards, the stick's raw Y grows upwards. */
    *out_y = -scale_axis(raw_y, cal->center_y, cal->max_y, cal->min_y);
}

void zmk_joycon2_gamepad_update(enum zmk_joycon2_side side, uint32_t buttons, uint16_t stick_x_raw,
                                 uint16_t stick_y_raw) {
    struct joycon2_side_state *st = &side_state[side == ZMK_JOYCON2_SIDE_RIGHT ? 1 : 0];
    const struct joycon2_profile_map *map = map_for(side);

    int8_t x;
    int8_t y;
    zmk_joycon2_gamepad_scale_stick(side, stick_x_raw, stick_y_raw, &x, &y);

#if IS_ENABLED(CONFIG_ZMK_JOYCON2_MOUSE)
    /* While this half drives the pointer its stick is the scroll wheel, so
     * report it centred here rather than leaving it deflected. */
    if (zmk_joycon2_mouse_owns(side)) {
        x = 0;
        y = 0;
    }
#endif

    bool buttons_changed = !st->have_last || buttons != st->last_buttons;
    bool axes_changed = !st->have_last || abs(x - st->last_x) >= JOYCON2_AXIS_CHANGE_THRESHOLD ||
                        abs(y - st->last_y) >= JOYCON2_AXIS_CHANGE_THRESHOLD ||
                        /* Always report a return to exact centre, however
                         * small the final step, so the stick can't stick
                         * just shy of neutral. */
                        ((x == 0 || y == 0) && (st->last_x != x || st->last_y != y));

    if (!buttons_changed && !axes_changed) {
        return;
    }

    if (buttons_changed) {
        for (uint8_t i = 0; i < JOYCON2_GAMEPAD_NUM_BUTTONS; i++) {
            uint32_t mask = map->buttons[i];
            if (!mask) {
                continue;
            }
#if IS_ENABLED(CONFIG_ZMK_JOYCON2_MOUSE)
            /* While this half is driving the pointer, its shoulder pair is
             * the mouse's clicks, so it must not also fire as R1/R2 (or
             * L1/L2). */
            if ((mask & mouse_click_masks(side)) && zmk_joycon2_mouse_owns(side)) {
                continue;
            }
#endif
            bool now_pressed = (buttons & mask) != 0;
            bool was_pressed = st->have_last && (st->last_buttons & mask) != 0;
            if (now_pressed == was_pressed) {
                continue;
            }
            if (now_pressed) {
                zmk_hid_gamepad_button_press(i);
            } else {
                zmk_hid_gamepad_button_release(i);
            }
        }
    }

    if (map->dpad_to_hat) {
        zmk_hid_gamepad_dpad_set(buttons & JC2_UP, buttons & JC2_DOWN, buttons & JC2_LEFT,
                                  buttons & JC2_RIGHT);
    }

    /* Absolute stick position, not a mouse-style delta. A lone Joy-Con is
     * the left stick whichever half it is; paired up, each half drives its
     * own side. */
    if (current_profile == ZMK_JOYCON2_PROFILE_DUO && side == ZMK_JOYCON2_SIDE_RIGHT) {
        zmk_hid_gamepad_right_stick_set(x, y);
    } else {
        zmk_hid_gamepad_left_stick_set(x, y);
    }

    st->have_last = true;
    st->last_buttons = buttons;
    st->last_x = x;
    st->last_y = y;

    int err = ZMK_JOYCON2_SEND_GAMEPAD_REPORT();
    if (err) {
        LOG_WRN("joycon2: gamepad report send failed (%d)", err);
    }
}
