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
 * Per-input-report logging (button state changes) is OFF by default,
 * because this channel types into whatever field has focus on the host --
 * with it on, playing a game or using a gamepad tester is impossible since
 * every press also types text. Connection-lifecycle messages are
 * unaffected: they fire a handful of times per connection, not per input
 * report.
 *
 * Toggled at runtime so switching between "testing the gamepad" and
 * "debugging the decoder" doesn't need a reflash.
 */
bool zmk_joycon2_debug_input_logging_enabled(void);

/** Flips input logging and reports the new state through the same channel. */
void zmk_joycon2_debug_input_logging_toggle(void);
