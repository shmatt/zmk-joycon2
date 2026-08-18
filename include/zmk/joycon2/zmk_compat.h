/*
 * Copyright (c) 2026 The zmk-joycon2 Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * ZMK renamed several endpoint functions from plural to singular
 * (zmk_endpoints_send_report -> zmk_endpoint_send_report, and likewise for
 * the per-report senders). This module targets both the shmatt/zmk
 * feat/mouse-keys-3.2 fork, which predates the rename, and current upstream
 * ZMK, so route the calls through names of our own.
 *
 * Symptom if this is wrong: the call is implicitly declared as returning int
 * and the build fails on an implicit-declaration warning-as-error, or on a
 * struct initialisation reported as "invalid initializer".
 */

#pragma once

#if IS_ENABLED(CONFIG_ZMK_JOYCON2_LEGACY_ENDPOINT_API)
#define ZMK_JOYCON2_SEND_REPORT zmk_endpoints_send_report
#define ZMK_JOYCON2_SEND_GAMEPAD_REPORT zmk_endpoints_send_gamepad_report
#define ZMK_JOYCON2_SEND_MOUSE_REPORT zmk_endpoints_send_mouse_report
#else
#define ZMK_JOYCON2_SEND_REPORT zmk_endpoint_send_report
#define ZMK_JOYCON2_SEND_GAMEPAD_REPORT zmk_endpoint_send_gamepad_report
#define ZMK_JOYCON2_SEND_MOUSE_REPORT zmk_endpoint_send_mouse_report
#endif
