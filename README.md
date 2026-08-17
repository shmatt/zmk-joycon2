# zmk-joycon2

Nintendo Switch 2 Joy-Con 2 controller support for ZMK split keyboards, as a
third BLE central role alongside ZMK's existing host and split-half links.

**Status: early proof-of-concept.** Right now this module contains only the
HID debug-print helper (sub-stage 1 of the project's Stage 1 POC) — a way to
get firmware debug output onto the phone screen with no PC, no serial
console, and no extra hardware, by literally typing it through the keyboard's
existing HID keyboard connection. BLE central scanning/connection to a Joy-Con
2 and packet decoding are not implemented yet.

## Why this exists

Joy-Con 2 (Switch 2) uses Bluetooth LE, unlike the original Switch 1 Joy-Con
(classic Bluetooth HID) — this is why tools built for the original Joy-Con
(JoyCon Droid, JoyDroid) don't apply here. The goal of this module is to let
a ZMK split keyboard's central half connect to Joy-Con 2 controllers directly
over BLE and expose them as a gamepad to whatever host the keyboard is
already paired with.

## What's here today: the HID debug-print helper

`zmk_joycon2_debug_print(const char *str)` (declared in
`include/zmk/joycon2/debug_print.h`) types `str` into whatever field has
focus on the host, one character at a time, via ZMK's existing
`zmk_hid_keyboard_press`/`release` + `zmk_endpoints_send_report` path — the
same path ZMK uses for normal keypresses. It's non-blocking (driven by a
`k_work_delayable`, ~8ms between HID reports) so it doesn't stall the main
thread while the split and host BLE links are also active.

A companion behavior, `zmk,behavior-joycon2-debug-print`, lets you trigger a
canned debug message from the keymap (`&joycon2_debug_print 0`). This exists
purely to test the debug channel itself before any BLE central code is added.

## Installation

Add this module to your ZMK config's build via the `modules` build option
(the mechanism upstream ZMK's GitHub Actions workflows and
[manna-harbour/miryoku_zmk](https://github.com/manna-harbour/miryoku_zmk)-based
build pipelines already support) — no ZMK fork required:

```yaml
modules: '["shmatt/zmk-joycon2/main"]'
```

Or, in a `west.yml` manifest:

```yaml
manifest:
  remotes:
    - name: shmatt
      url-base: https://github.com/shmatt
  projects:
    - name: zmk-joycon2
      remote: shmatt
      revision: main
```

### Wiring the debug-print test behavior into a keymap

1. `#include <joycon2.dtsi>` in your keymap/overlay to pull in the
   `joycon2_debug_print` behavior label.
2. Bind it to something you can trigger deliberately — a combo works well
   since it doesn't cost you a keymap position. Example (adjust
   `key-positions` to your own layout's physical position numbering):

   ```dts
   / {
       combos {
           compatible = "zmk,combos";
           combo_joycon2_debug {
               timeout-ms = <50>;
               key-positions = <37 38>;
               bindings = <&joycon2_debug_print 0>;
           };
       };
   };
   ```

## Kconfig

- `CONFIG_ZMK_JOYCON2` (default `y`) — top-level module gate.
- `CONFIG_ZMK_JOYCON2_DEBUG_PRINT` (default `y`) — the debug-print helper
  above.

## Roadmap

- BLE central scan/connect (proving 3 concurrent BLE roles work on
  nRF52840 + ZMK's Bluetooth stack — the core open risk this project is
  built around).
- Joy-Con 2 packet decoding (buttons, D-pad, analog sticks).
- HID gamepad report integration.
- `CONFIG_BT_MAX_CONN`/`CONFIG_BT_MAX_PAIRED` guidance, once proven.

Out of scope for the base module: optical-sensor mouse mode, amiibo/NFC,
rumble/haptics, IMU motion data.

## Credit

The packet-decoding approach (byte-offset layout of Joy-Con 2 BLE input
reports) is conceptually informed by
[OZORDI/JoyCon2Mac](https://github.com/OZORDI/JoyCon2Mac), a macOS DriverKit
driver for Joy-Con 2 — none of its code is reused directly (different
platform, different HID stack), but its `JoyConDecoder` design is a useful
reference for what the byte layout looks like.

## License

MIT — see [LICENSE](LICENSE).
