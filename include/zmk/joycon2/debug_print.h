#pragma once

/**
 * Types `str` into whatever field currently has focus on the host, one
 * character at a time, via ZMK's existing HID keyboard report path.
 *
 * Non-blocking: queues the string and returns immediately. Returns -EBUSY if
 * a previous call is still in progress (only one message may be in flight at
 * a time); returns 0 on success. Unsupported characters print as '?'.
 */
int zmk_joycon2_debug_print(const char *str);
