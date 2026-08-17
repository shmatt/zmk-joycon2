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
 * SOLO: a single Joy-Con used on its own, held sideways -- the rail
 * (SL/SR) becomes the shoulder buttons, the D-pad (left half) or face
 * buttons (right half) become the face cluster, and the stick is rotated
 * to match the rotated grip.
 *
 * DUO: both halves used together as one pad, each held upright, so every
 * control keeps its natural orientation and no rotation is applied.
 */
enum zmk_joycon2_profile {
    ZMK_JOYCON2_PROFILE_SOLO = 0,
    ZMK_JOYCON2_PROFILE_DUO,
};

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
