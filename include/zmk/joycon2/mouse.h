/*
 * Copyright (c) 2026 The zmk-joycon2 Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <zmk/joycon2/gamepad.h>

/* Optical mouse, switched on by resting a Joy-Con on a surface rather than by
 * a button: whichever half is face-down drives the pointer, and lifting it
 * stops the pointer the way lifting a real mouse does. Either half can do it.
 *
 * While a half owns the pointer its own shoulder pair becomes the clicks --
 * R/ZR on the right, L/ZL on the left -- so the gamepad mapping asks about
 * this before emitting those two.
 */
bool zmk_joycon2_mouse_owns(enum zmk_joycon2_side side);

/* Feed one half's optical sensor and buttons.
 *
 * raw_x/raw_y are the sensor's own accumulated position, NOT per-frame
 * deltas: movement is the difference between consecutive samples. distance is
 * inverted from what its name suggests -- 0 means the controller is TOUCHING
 * a surface, anything higher means airborne (typically around 12).
 */
void zmk_joycon2_mouse_update(enum zmk_joycon2_side side, int16_t raw_x, int16_t raw_y,
                               uint8_t distance, uint32_t buttons);

/* Drops any held clicks and forgets the sensor position, so a disconnect
 * cannot leave a click stuck down or fling the pointer on reconnect. */
void zmk_joycon2_mouse_reset(enum zmk_joycon2_side side);
