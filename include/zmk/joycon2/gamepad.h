/*
 * Copyright (c) 2026 The zmk-joycon2 Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>

enum zmk_joycon2_side {
    ZMK_JOYCON2_SIDE_UNKNOWN = 0,
    ZMK_JOYCON2_SIDE_LEFT,
    ZMK_JOYCON2_SIDE_RIGHT,
};

/* Button mapping profile.
 *
 * SOLO: a single Joy-Con used on its own, held UPRIGHT in one hand. Its
 * own shoulder pair keeps its natural side (the left half's L/ZL really
 * are L1/L2) and the rail buttons stand in for the pair the missing half
 * would have provided -- so SL/SR become R1/R2 on the left half and
 * L1/L2 on the right. The half's D-pad (left) or ABXY cluster (right)
 * serves as the face buttons, since there is no second half to supply
 * one.
 *
 * DUO: both halves used together as one pad, each held upright, so every
 * control keeps its natural role: the right half provides the face
 * cluster and R1/R2, the left half a true D-pad and L1/L2.
 */
enum zmk_joycon2_profile {
    ZMK_JOYCON2_PROFILE_SOLO = 0,
    ZMK_JOYCON2_PROFILE_DUO,
};

/* A stick's measured range, read from the controller itself. max/min are
 * offsets either side of centre, not absolute positions.
 *
 * This matters more than it looks: a Joy-Con stick only swings over a
 * fraction of the raw 12-bit span, so scaling as though it used the whole
 * range leaves the axes reporting roughly half of what they should at full
 * tilt. */
struct zmk_joycon2_stick_calib {
    uint16_t center_x;
    uint16_t center_y;
    uint16_t max_x;
    uint16_t max_y;
    uint16_t min_x;
    uint16_t min_y;
};

/* Ignores implausible values and keeps the defaults, so a controller whose
 * calibration region is unwritten does not end up with a pegged stick. */
void zmk_joycon2_gamepad_set_calibration(enum zmk_joycon2_side side,
                                          const struct zmk_joycon2_stick_calib *calib);

/* Selects the mapping profile from how many controllers are currently
 * connected: 1 -> SOLO, 2 or more -> DUO. Safe to call on every
 * connect/disconnect; it only acts on a change. */
void zmk_joycon2_gamepad_set_connected_count(uint8_t count);

/* Feed one decoded Joy-Con 2 input report into the gamepad HID report sent
 * to the host. Call for every report; change detection and rate limiting
 * happen inside. Stick values are the raw 12-bit values from the report
 * (centre ~2048), not yet calibrated. */
void zmk_joycon2_gamepad_update(enum zmk_joycon2_side side, uint32_t buttons, uint16_t stick_x_raw,
                                 uint16_t stick_y_raw);
