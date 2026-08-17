#pragma once

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
