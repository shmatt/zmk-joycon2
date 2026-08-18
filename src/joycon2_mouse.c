/*
 * Copyright (c) 2026 The zmk-joycon2 Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Drives the host's mouse pointer from a Joy-Con 2's optical sensor.
 *
 * Nothing here needs a firmware patch: ZMK already carries a mouse report,
 * and the sensor is already running because the handshake enables every
 * feature. The work is only in interpreting the report correctly.
 */

#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <dt-bindings/zmk/mouse.h>
#include <zmk/endpoints.h>
#include <zmk/hid.h>

#include <zmk/joycon2/buttons.h>
#include <zmk/joycon2/gamepad.h>
#include <zmk/joycon2/mouse.h>
#include <zmk/joycon2/zmk_compat.h>

LOG_MODULE_REGISTER(joycon2_mouse, CONFIG_ZMK_LOG_LEVEL);

/* Byte 0x17 of the input report, and it means the opposite of what the name
 * suggests: 0 is contact, higher values are airborne (around 12 in the air).
 * Confirmed on hardware by the JoyCon2Mac authors. */
#define JOYCON2_SURFACE_TOUCHING 0

/* ZMK's mouse report carries signed 8-bit deltas. */
#define JOYCON2_MOUSE_DELTA_MIN (-127)
#define JOYCON2_MOUSE_DELTA_MAX 127

/* A jump larger than this is not a hand movement -- it is the sensor
 * re-acquiring the surface, or a dropped report. Emitting it would throw the
 * pointer across the screen, so it is dropped and the position resynced. */
#define JOYCON2_MOUSE_GLITCH_THRESHOLD 4096

/* Stick deflection is accumulated and spent in whole wheel steps: emitting a
 * step per report would scroll absurdly fast at 60-120Hz, and rounding each
 * report to an integer would drop slow scrolling entirely. */
#define JOYCON2_SCROLL_STEP CONFIG_ZMK_JOYCON2_MOUSE_SCROLL_STEP

struct joycon2_mouse_side {
    bool have_last;
    int32_t scroll_acc_x;
    int32_t scroll_acc_y;
    int16_t last_x;
    int16_t last_y;
    bool on_surface;
    /* Click state, so a release is only sent for a click actually held. */
    bool left_held;
    bool right_held;
};

static struct joycon2_mouse_side sides[2];
static enum zmk_joycon2_side owner = ZMK_JOYCON2_SIDE_UNKNOWN;

static size_t side_index(enum zmk_joycon2_side side) {
    return side == ZMK_JOYCON2_SIDE_RIGHT ? 1 : 0;
}

bool zmk_joycon2_mouse_owns(enum zmk_joycon2_side side) {
    return owner != ZMK_JOYCON2_SIDE_UNKNOWN && owner == side &&
           sides[side_index(side)].on_surface;
}

/* Each half's clicks come from its own shoulder pair, mirrored between them:
 * resting a half face-down puts a different one of the two under the finger
 * that naturally left-clicks. */
static uint32_t left_click_mask(enum zmk_joycon2_side side) {
    return side == ZMK_JOYCON2_SIDE_RIGHT ? JC2_R : JC2_ZL;
}

static uint32_t right_click_mask(enum zmk_joycon2_side side) {
    return side == ZMK_JOYCON2_SIDE_RIGHT ? JC2_ZR : JC2_L;
}

/* Uses the mask-taking plural form on purpose: the singular
 * zmk_hid_mouse_button_press() takes a bit INDEX, not a mask, and passing
 * MB1/MB2 to it would silently press the wrong buttons -- the same trap as
 * zmk_hid_register_mod versus register_mods. */
static bool release_clicks(struct joycon2_mouse_side *s) {
    bool changed = false;

    if (s->left_held) {
        zmk_hid_mouse_buttons_release(MB1);
        s->left_held = false;
        changed = true;
    }
    if (s->right_held) {
        zmk_hid_mouse_buttons_release(MB2);
        s->right_held = false;
        changed = true;
    }
    return changed;
}

void zmk_joycon2_mouse_reset(enum zmk_joycon2_side side) {
    struct joycon2_mouse_side *s = &sides[side_index(side)];

    if (release_clicks(s)) {
        zmk_hid_mouse_movement_set(0, 0);
        ZMK_JOYCON2_SEND_MOUSE_REPORT();
    }

    s->have_last = false;
    s->on_surface = false;
    s->scroll_acc_x = 0;
    s->scroll_acc_y = 0;

    if (owner == side) {
        owner = ZMK_JOYCON2_SIDE_UNKNOWN;
    }
}

/* Hands the pointer to whichever half is face-down. If both are, the current
 * owner keeps it: swapping on every report would make the pointer jitter
 * between two sensors. */
static void update_owner(enum zmk_joycon2_side side, bool on_surface) {
    if (on_surface) {
        if (owner == ZMK_JOYCON2_SIDE_UNKNOWN || !sides[side_index(owner)].on_surface) {
            if (owner != side) {
                LOG_INF("joycon2: mouse now driven by the %s half",
                        side == ZMK_JOYCON2_SIDE_RIGHT ? "right" : "left");
            }
            owner = side;
        }
        return;
    }

    if (owner == side) {
        owner = ZMK_JOYCON2_SIDE_UNKNOWN;
    }
}

void zmk_joycon2_mouse_update(enum zmk_joycon2_side side, int16_t raw_x, int16_t raw_y,
                               uint8_t distance, uint32_t buttons, uint16_t stick_x_raw,
                               uint16_t stick_y_raw) {
    struct joycon2_mouse_side *s = &sides[side_index(side)];
    bool on_surface = (distance == JOYCON2_SURFACE_TOUCHING);

    s->on_surface = on_surface;
    update_owner(side, on_surface);

    if (!on_surface) {
        /* Airborne: forget the position so the first sample after landing
         * produces no movement, and drop any clicks still held. */
        s->have_last = false;
        s->scroll_acc_x = 0;
        s->scroll_acc_y = 0;
        if (release_clicks(s)) {
            zmk_hid_mouse_movement_set(0, 0);
            ZMK_JOYCON2_SEND_MOUSE_REPORT();
        }
        return;
    }

    if (owner != side) {
        return;
    }

    int16_t dx = 0;
    int16_t dy = 0;
    if (s->have_last) {
        int32_t delta_x = (int32_t)raw_x - s->last_x;
        int32_t delta_y = (int32_t)raw_y - s->last_y;

        if (abs(delta_x) < JOYCON2_MOUSE_GLITCH_THRESHOLD &&
            abs(delta_y) < JOYCON2_MOUSE_GLITCH_THRESHOLD) {
            dx = CLAMP(delta_x / CONFIG_ZMK_JOYCON2_MOUSE_DIVISOR, JOYCON2_MOUSE_DELTA_MIN,
                       JOYCON2_MOUSE_DELTA_MAX);
            dy = CLAMP(delta_y / CONFIG_ZMK_JOYCON2_MOUSE_DIVISOR, JOYCON2_MOUSE_DELTA_MIN,
                       JOYCON2_MOUSE_DELTA_MAX);
        }
    }

    s->last_x = raw_x;
    s->last_y = raw_y;
    s->have_last = true;

    /* The stick scrolls while this half drives the pointer. Reusing the
     * gamepad's scaling keeps one copy of the calibration and deadzone. */
    int8_t stick_x;
    int8_t stick_y;
    zmk_joycon2_gamepad_scale_stick(side, stick_x_raw, stick_y_raw, &stick_x, &stick_y);

    s->scroll_acc_x += stick_x;
    /* Wheel-up is positive, and the scaled stick Y is positive downwards. */
    s->scroll_acc_y -= stick_y;

    int8_t scroll_x = (int8_t)CLAMP(s->scroll_acc_x / JOYCON2_SCROLL_STEP, -127, 127);
    int8_t scroll_y = (int8_t)CLAMP(s->scroll_acc_y / JOYCON2_SCROLL_STEP, -127, 127);
    s->scroll_acc_x -= scroll_x * JOYCON2_SCROLL_STEP;
    s->scroll_acc_y -= scroll_y * JOYCON2_SCROLL_STEP;

    bool buttons_changed = false;
    bool want_left = (buttons & left_click_mask(side)) != 0;
    bool want_right = (buttons & right_click_mask(side)) != 0;

    if (want_left != s->left_held) {
        if (want_left) {
            zmk_hid_mouse_buttons_press(MB1);
        } else {
            zmk_hid_mouse_buttons_release(MB1);
        }
        s->left_held = want_left;
        buttons_changed = true;
    }
    if (want_right != s->right_held) {
        if (want_right) {
            zmk_hid_mouse_buttons_press(MB2);
        } else {
            zmk_hid_mouse_buttons_release(MB2);
        }
        s->right_held = want_right;
        buttons_changed = true;
    }

    if (dx == 0 && dy == 0 && scroll_x == 0 && scroll_y == 0 && !buttons_changed) {
        return;
    }

    /* Set rather than accumulate: this is one report's worth of movement. */
    zmk_hid_mouse_movement_set(dx, dy);
    zmk_hid_mouse_scroll_set(scroll_x, scroll_y);
    ZMK_JOYCON2_SEND_MOUSE_REPORT();
    /* Both are relative, so leave the report zeroed for next time --
     * otherwise a report sent for a click alone would repeat the last
     * movement and scroll. */
    zmk_hid_mouse_movement_set(0, 0);
    zmk_hid_mouse_scroll_set(0, 0);
}
