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

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/endpoints.h>
#include <zmk/hid.h>

#include <zmk/joycon2/gamepad.h>
#include <zmk/joycon2/zmk_compat.h>

LOG_MODULE_REGISTER(joycon2_gamepad, CONFIG_ZMK_LOG_LEVEL);

/* Joy-Con 2 button bits -- see the input-report layout notes in
 * joycon2_connection.c. Left-half and right-half buttons occupy distinct
 * bits in the same 32-bit word, so one table covers both. */
#define JC2_Y 0x00000001U
#define JC2_X 0x00000002U
#define JC2_B 0x00000004U
#define JC2_A 0x00000008U
#define JC2_SR_R 0x00000010U
#define JC2_SL_R 0x00000020U
#define JC2_R 0x00000040U
#define JC2_ZR 0x00000080U
#define JC2_MINUS 0x00000100U
#define JC2_PLUS 0x00000200U
#define JC2_RSTK 0x00000400U
#define JC2_LSTK 0x00000800U
#define JC2_HOME 0x00001000U
#define JC2_CAPTURE 0x00002000U
#define JC2_C 0x00004000U
#define JC2_DOWN 0x00010000U
#define JC2_UP 0x00020000U
#define JC2_RIGHT 0x00040000U
#define JC2_LEFT 0x00080000U
#define JC2_SR_L 0x00100000U
#define JC2_SL_L 0x00200000U
#define JC2_L 0x00400000U
#define JC2_ZL 0x00800000U
#define JC2_GR 0x01000000U
#define JC2_GL 0x02000000U

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
     * generic and NOT D-pad keycodes. A real D-pad needs a hat switch in the
     * report descriptor; until then DUO's D-pad lands on these, which apps
     * can at least bind individually. */
    PAD_GENERIC_1 = 15,
    PAD_GENERIC_2 = 16,
    PAD_GENERIC_3 = 17,
    PAD_GENERIC_4 = 18,
};

/* Index = HID button number - 1. 0 means "nothing mapped at this index". */
struct joycon2_profile_map {
    uint32_t buttons[JOYCON2_GAMEPAD_NUM_BUTTONS];
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
            [PAD_R1] = JC2_SR_L,
            [PAD_R2] = JC2_SL_L,
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
            [PAD_SELECT] = JC2_MINUS,
            [PAD_THUMBL] = JC2_LSTK,
            [PAD_GENERIC_1] = JC2_UP,
            [PAD_GENERIC_2] = JC2_DOWN,
            [PAD_GENERIC_3] = JC2_LEFT,
            [PAD_GENERIC_4] = JC2_RIGHT,
        },
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
            [PAD_START] = JC2_PLUS,
            [PAD_MODE] = JC2_HOME,
            [PAD_THUMBR] = JC2_RSTK,
        },
};

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
    ZMK_JOYCON2_SEND_GAMEPAD_REPORT();
}

#define JOYCON2_STICK_CENTRE 2048
/* Deadzone in raw 12-bit units. The sticks rest a little off centre and
 * jitter by a few counts; without this the host sees constant drift and a
 * report goes out for every input packet. */
#define JOYCON2_STICK_DEADZONE 240
/* Report only when an axis moves at least this far (in final int8 units),
 * so a resting-but-noisy stick doesn't flood the HID endpoint. */
#define JOYCON2_AXIS_CHANGE_THRESHOLD 3

static int8_t scale_axis(uint16_t raw) {
    int32_t centred = (int32_t)raw - JOYCON2_STICK_CENTRE;

    if (centred > -JOYCON2_STICK_DEADZONE && centred < JOYCON2_STICK_DEADZONE) {
        return 0;
    }

    /* Map the usable travel either side of the deadzone onto -127..127. */
    if (centred > 0) {
        centred -= JOYCON2_STICK_DEADZONE;
    } else {
        centred += JOYCON2_STICK_DEADZONE;
    }

    int32_t span = JOYCON2_STICK_CENTRE - JOYCON2_STICK_DEADZONE;

    return (int8_t)CLAMP((centred * 127) / span, -127, 127);
}

void zmk_joycon2_gamepad_update(enum zmk_joycon2_side side, uint32_t buttons, uint16_t stick_x_raw,
                                 uint16_t stick_y_raw) {
    static bool have_last;
    static uint32_t last_buttons;
    static int8_t last_x;
    static int8_t last_y;

    const struct joycon2_profile_map *map = map_for(side);

    int8_t x = scale_axis(stick_x_raw);
    /* HID Y grows downwards, the stick's raw Y grows upwards. */
    int8_t y = -scale_axis(stick_y_raw);

    bool buttons_changed = !have_last || buttons != last_buttons;
    bool axes_changed = !have_last || abs(x - last_x) >= JOYCON2_AXIS_CHANGE_THRESHOLD ||
                        abs(y - last_y) >= JOYCON2_AXIS_CHANGE_THRESHOLD ||
                        /* Always report a return to exact centre, however
                         * small the final step, so the stick can't stick
                         * just shy of neutral. */
                        ((x == 0 || y == 0) && (last_x != x || last_y != y));

    if (!buttons_changed && !axes_changed) {
        return;
    }

    if (buttons_changed) {
        for (uint8_t i = 0; i < JOYCON2_GAMEPAD_NUM_BUTTONS; i++) {
            uint32_t mask = map->buttons[i];
            if (!mask) {
                continue;
            }
            bool now_pressed = (buttons & mask) != 0;
            bool was_pressed = have_last && (last_buttons & mask) != 0;
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

    /* Absolute stick position, not a mouse-style delta. A lone Joy-Con is
     * the left stick whichever half it is; paired up, each half drives its
     * own side. */
    if (current_profile == ZMK_JOYCON2_PROFILE_DUO && side == ZMK_JOYCON2_SIDE_RIGHT) {
        zmk_hid_gamepad_right_stick_set(x, y);
    } else {
        zmk_hid_gamepad_left_stick_set(x, y);
    }

    have_last = true;
    last_buttons = buttons;
    last_x = x;
    last_y = y;

    int err = ZMK_JOYCON2_SEND_GAMEPAD_REPORT();
    if (err) {
        LOG_WRN("joycon2: gamepad report send failed (%d)", err);
    }
}
