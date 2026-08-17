#pragma once

/**
 * Stage 1 sub-stage 4: scans for a Joy-Con 2 (filtered by its main BLE
 * service UUID, since it advertises with no device name), connects,
 * discovers the input/command/response characteristics, subscribes to
 * input+response notifications, runs the IMU-enable + MAC-binding command
 * handshake (using this keyboard's own Bluetooth identity address), then
 * relays raw input notification payloads as hex via the debug-print helper.
 * No decoding yet -- see zmk_joycon2_debug_print() for how output reaches
 * the host.
 *
 * Returns -EBUSY if already connecting/connected.
 */
int zmk_joycon2_connection_start(void);
