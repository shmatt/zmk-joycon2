#pragma once

#include <stdbool.h>

/**
 * Types `str` into whatever field currently has focus on the host, one
 * character at a time, via ZMK's existing HID keyboard report path. A
 * trailing space is appended so consecutive messages stay visually
 * separated when typed back-to-back.
 *
 * Non-blocking: queues the string and returns immediately. Up to
 * JOYCON2_DEBUG_PRINT_QUEUE_DEPTH messages may be pending in addition to the
 * one currently printing; returns -ENOSPC if that backlog is full (dropped,
 * logged via LOG_WRN), 0 otherwise. Unsupported characters print as '?'.
 */
int zmk_joycon2_debug_print(const char *str);

/**
 * All output through this channel is OFF by default, because it types into
 * whatever field has focus on the host: with it on, a gamepad tester or any
 * game is unusable, since every press also types text. Nothing prints --
 * not connection progress, not failures -- until it is switched on.
 *
 * Toggled at runtime so switching between "using the gamepad" and
 * "debugging" never needs a reflash.
 */
bool zmk_joycon2_debug_logging_enabled(void);

/** Flips logging and announces the new state, which always prints. */
void zmk_joycon2_debug_logging_toggle(void);
