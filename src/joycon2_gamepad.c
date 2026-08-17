/*
 * Copyright (c) 2026 The zmk-joycon2 Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Bridges decoded Joy-Con 2 state onto a gamepad HID report sent to the
 * host. ZMK's own HID report descriptor is a static const array in its
 * app headers and cannot be extended from an out-of-tree module, so the
 * gamepad is exposed by zmk-hid-io, which registers a second, independent
 * HID-over-GATT service declaring a Generic Desktop Joystick.
 */

#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/hid-io/endpoints.h>
#include <zmk/hid-io/hid_joystick.h>

#include <zmk/joycon2/gamepad.h>

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

/* Our zmk-hid-io fork widens the joystick report to 32 buttons (upstream
 * has 8, too few for a two-Joy-Con pad). Indices below are 0-based; a
 * gamepad tester shows them as buttons 1-32. */
#define JOYCON2_GAMEPAD_NUM_BUTTONS 32

/* Logical slots, laid out so DUO and either SOLO half all land on the same
 * indices -- a game configured against one profile keeps working in the
 * others as far as the shared controls allow. */
enum joycon2_pad_button {
    PAD_FACE_DOWN = 0, /* A / Cross      */
    PAD_FACE_RIGHT,    /* B / Circle     */
    PAD_FACE_LEFT,     /* X / Square     */
    PAD_FACE_UP,       /* Y / Triangle   */
    PAD_L1,
    PAD_L2,
    PAD_R1,
    PAD_R2,
    PAD_SELECT, /* Minus  */
    PAD_START,  /* Plus   */
    PAD_L3,     /* Left stick click  */
    PAD_R3,     /* Right stick click */
    PAD_HOME,
    PAD_CAPTURE,
    /* A real D-pad, separate from the face cluster. Only DUO uses these:
     * in SOLO the single half's D-pad (or ABXY) has to serve as the face
     * cluster, since there is no second half to provide one. */
    PAD_DPAD_UP,
    PAD_DPAD_DOWN,
    PAD_DPAD_LEFT,
    PAD_DPAD_RIGHT,
};

/* Index = HID button number - 1. 0 means "nothing mapped at this index". */
struct joycon2_profile_map {
    uint32_t buttons[JOYCON2_GAMEPAD_NUM_BUTTONS];
};

/* SOLO: a single Joy-Con held UPRIGHT in one hand.
 *
 * The half's own shoulder pair keeps its natural side (the left half's
 * L/ZL really are L1/L2), and the rail buttons stand in for the pair the
 * missing half would have provided -- so on the left half SL/SR become
 * R1/R2, and on the right half they become L1/L2. No stick rotation,
 * since the controller is not turned.
 */
static const struct joycon2_profile_map solo_left = {
    .buttons =
        {
            [PAD_FACE_DOWN] = JC2_DOWN,
            [PAD_FACE_RIGHT] = JC2_RIGHT,
            [PAD_FACE_LEFT] = JC2_LEFT,
            [PAD_FACE_UP] = JC2_UP,
            [PAD_L1] = JC2_L,
            [PAD_L2] = JC2_ZL,
            [PAD_R1] = JC2_SL_L,
            [PAD_R2] = JC2_SR_L,
            [PAD_SELECT] = JC2_MINUS,
            [PAD_L3] = JC2_LSTK,
            [PAD_CAPTURE] = JC2_CAPTURE,
        },
};

static const struct joycon2_profile_map solo_right = {
    .buttons =
        {
            [PAD_FACE_DOWN] = JC2_B,
            [PAD_FACE_RIGHT] = JC2_A,
            [PAD_FACE_LEFT] = JC2_Y,
            [PAD_FACE_UP] = JC2_X,
            [PAD_L1] = JC2_SL_R,
            [PAD_L2] = JC2_SR_R,
            [PAD_R1] = JC2_R,
            [PAD_R2] = JC2_ZR,
            [PAD_START] = JC2_PLUS,
            [PAD_R3] = JC2_RSTK,
            [PAD_HOME] = JC2_HOME,
        },
};

/* DUO: both halves held upright as one pad, so every control keeps its
 * natural role. The two tables map disjoint physical buttons, and each
 * half only ever writes its own indices, so the host sees one merged pad.
 * The rail buttons are left unmapped here -- both real shoulder pairs are
 * present, so SL/SR are free for future remapping. */
static const struct joycon2_profile_map duo_left = {
    .buttons =
        {
            [PAD_L1] = JC2_L,
            [PAD_L2] = JC2_ZL,
            [PAD_SELECT] = JC2_MINUS,
            [PAD_L3] = JC2_LSTK,
            [PAD_CAPTURE] = JC2_CAPTURE,
            /* A true D-pad here, distinct from the face cluster that the
             * right half contributes in this profile. */
            [PAD_DPAD_UP] = JC2_UP,
            [PAD_DPAD_DOWN] = JC2_DOWN,
            [PAD_DPAD_LEFT] = JC2_LEFT,
            [PAD_DPAD_RIGHT] = JC2_RIGHT,
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
            [PAD_R3] = JC2_RSTK,
            [PAD_HOME] = JC2_HOME,
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
    zmk_hid_joy2_clear();
    zmk_endpoints_send_joystick_report_alt();
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
                zmk_hid_joy2_button_press(i);
            } else {
                zmk_hid_joy2_button_release(i);
            }
        }
    }

    /* movement_set replaces the axis values outright rather than
     * accumulating deltas like a mouse, which is what an absolute stick
     * position needs. */
    zmk_hid_joy2_movement_set(x, y);

    have_last = true;
    last_buttons = buttons;
    last_x = x;
    last_y = y;

    int err = zmk_endpoints_send_joystick_report_alt();
    if (err) {
        LOG_WRN("joycon2: gamepad report send failed (%d)", err);
    }
}
