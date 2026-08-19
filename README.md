# zmk-joycon2

Use Nintendo Switch 2 Joy-Con controllers with a ZMK keyboard, and let the
keyboard republish them to its host as a gamepad and a mouse.

Allows an existing keyboard to act as a bridge for controllers that cannot 
pair themselves, or can be used to create a ZMK-powered hardware bridge to 
allow Joy-Con 2 input to any host that supports a gamepad and a mouse.

The keyboard becomes a BLE **central** for the controllers — an extra role
alongside the host HID link and the split-half link it already maintains — so
the host only ever sees one device: your keyboard. Nothing is installed on the
host, and it works with hosts that cannot pair Joy-Cons themselves.

Confirmed working on a Corne-style split (nice!nano v2, nRF52840) with both
halves of a Joy-Con 2 pair connected at once, against an Android host.

## What it does

- **Connects one or both Joy-Con 2 halves** over BLE, on demand from a keymap
  binding.
- **Gamepad output**: face buttons, shoulders, a real D-pad (via a HID hat
  switch), stick clicks and system buttons, with sticks scaled by each
  controller's own factory calibration.
- **Two mapping profiles, chosen automatically**: one half held upright on its
  own, or both halves together as a single pad.
- **Optical mouse**: rest a half face-down on a desk and it drives the pointer;
  lift it and the pointer stops, the way lifting a real mouse does. Its
  shoulder pair becomes left/right click and its stick becomes the scroll
  wheel, for as long as it is face-down.
- **Debug output without a serial port**, by typing it through the keyboard's
  own HID connection — off by default.

Not implemented: rumble, IMU/motion, NFC/amiibo, analog triggers (Joy-Cons
report ZL/ZR digitally), and reconnecting without holding SYNC (see
[Limitations](#limitations)).

## Requirements

A ZMK tree with the gamepad HID report, because ZMK's report descriptor is a
`static const` array in its own headers and **cannot** be extended from an
out-of-tree module. Two ready branches:

| ZMK base | branch | notes |
|---|---|---|
| current upstream ZMK | [`shmatt/zmk` `feat/gamepad-hid`](https://github.com/shmatt/zmk/tree/feat/gamepad-hid) | use this unless you need the older base |
| ZMK ~3.5-era forks | [`shmatt/zmk` `feat/gamepad-hid-3.2`](https://github.com/shmatt/zmk/tree/feat/gamepad-hid-3.2) | set `CONFIG_ZMK_JOYCON2_LEGACY_ENDPOINT_API=y` |

Each adds an optional `CONFIG_ZMK_GAMEPAD` — a Generic Desktop Gamepad report
(two sticks, 32 buttons, a hat switch) inside ZMK's existing HID interface. The
patch is additive and off by default, so builds that don't select it are
unchanged. Mouse output needs no patch: it uses ZMK's existing mouse report
(`CONFIG_ZMK_POINTING`, or `CONFIG_ZMK_MOUSE` on older trees).

Also required, because Joy-Con 2 input reports are 63 bytes:

```
CONFIG_BT_L2CAP_TX_MTU=250
CONFIG_BT_BUF_ACL_RX_SIZE=254
CONFIG_BT_CTLR_DATA_LENGTH_MAX=251
```

**The MTU line is not optional.** ZMK's default ATT MTU of 65 caps a
notification at 62 bytes — one byte short — and the controller responds by
sending *nothing at all*, while still acknowledging every command. That failure
looks exactly like a broken handshake and is miserable to diagnose.

## Installation

With a `zmk-config` built by GitHub Actions, add the module and the ZMK branch
to your build matrix. Several modules for one build go in a **single**
space-separated string:

```yaml
modules: '["shmatt/zmk-joycon2/main"]'
branches: '["shmatt/zmk/feat/gamepad-hid"]'
```

Or as a west manifest entry:

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

### Wiring up the keymap

Include the behaviours and bind `&joycon2_connect` to something deliberate. A
combo works well, since it costs no keymap position:

```dts
#include <joycon2.dtsi>

/ {
    combos {
        compatible = "zmk,combos";

        // Starts a scan; hold SYNC on a controller to connect it.
        combo_joycon2_connect {
            timeout-ms = <50>;
            key-positions = <36 37>;   // your own layout's positions
            bindings = <&joycon2_connect>;
        };

        // Optional: toggles debug output on and off at runtime.
        combo_joycon2_log {
            timeout-ms = <50>;
            key-positions = <37 38>;
            bindings = <&joycon2_debug_print 0>;
        };
    };
};
```

### Connecting

1. Trigger the connect binding. The keyboard scans for about 20 seconds.
2. Hold **SYNC** on a controller until it connects.
3. For a pair, hold SYNC on the second one — the same scan picks it up, and the
   mapping switches to the two-half profile automatically.

If a controller stops being found after several quick attempts, wait a couple
of minutes: the controllers have their own connection cooldown.

**On Android, forget and re-pair the keyboard after the first flash — and
after any change to ZMK's HID configuration.** Android caches the HID report
descriptor per bonded device and does not re-read it while the bond survives,
so until you re-pair, the host parses new reports using the old layout. Clear
it from both ends: forget the keyboard in Android's Bluetooth settings, and
run `&bt BT_CLR` on the active profile from the keymap. Two failure modes,
both of which look exactly like broken firmware:

- Before the first re-pair, the host has no idea the gamepad exists.
- If the *mouse* report changes shape, every field is read from the wrong
  offset. Moving from a tree with 8-bit mouse axes to one with 16-bit axes
  (`CONFIG_ZMK_POINTING`) makes X roughly work, bleeds X's high byte into Y,
  and lands Y in the scroll wheel, while real scroll falls outside the report
  the host thinks it is reading. This hits ZMK's own mouse keys as well as
  this module — that both break identically is the tell that the descriptor
  is stale rather than the code being wrong.

## Mapping

Held upright, a lone half's own shoulders keep their side and its rail buttons
stand in for the pair the missing half would have provided.

| | left half alone | right half alone | both halves |
|---|---|---|---|
| Face buttons | D-pad | B/A/Y/X → A/B/X/Y | right half's B/A/Y/X |
| L1 / L2 | L / ZL | SR / SL | left half's L / ZL |
| R1 / R2 | SL / SR | R / ZR | right half's R / ZR |
| D-pad | *(is the face cluster)* | — | left half's D-pad (hat) |
| Start | Capture | Home | Home |
| Select | — | C | Capture |
| Mode | Minus | Plus | Plus |
| Stick click | ThumbL | ThumbL | ThumbL / ThumbR |

Face buttons map by **position, not letter**: Nintendo's B is the bottom
button, so it becomes A, and Nintendo's A is on the right, so it becomes B.
Nintendo X/Y are likewise swapped relative to the Xbox-style convention hosts
expect.

While a half is face-down driving the pointer, its shoulder pair and stick are
the mouse's and are withheld from the gamepad: **R** = left click and **ZR** =
right click on the right half, mirrored as **ZL** = left and **L** = right on
the left half.

Edit the tables at the top of `src/joycon2_gamepad.c` to change any of this.
Note the button indices are deliberately **not** consecutive — see the comment
there, and [Findings](#findings) below.

## Configuration

| option | default | purpose |
|---|---|---|
| `ZMK_JOYCON2` | `y` | module gate |
| `ZMK_JOYCON2_CONNECT` | `y` | BLE central: scan, connect, decode |
| `ZMK_JOYCON2_GAMEPAD` | `y` | gamepad output (needs `ZMK_GAMEPAD`) |
| `ZMK_JOYCON2_MOUSE` | `y` | optical mouse (needs ZMK's mouse report) |
| `ZMK_JOYCON2_MOUSE_DIVISOR` | `1` | raise to slow the pointer |
| `ZMK_JOYCON2_MOUSE_SCROLL_STEP` | `600` | raise to slow stick scrolling |
| `ZMK_JOYCON2_DEBUG_PRINT` | `y` | debug channel (silent until toggled) |
| `ZMK_JOYCON2_BOND` | `y` | store this host on the controller |
| `ZMK_JOYCON2_AUTO_RECONNECT` | `n` | rescan after a disconnect — read below |
| `ZMK_JOYCON2_RIGHT_C_CONSUMER_USAGE` | `0` | send a Consumer usage from C instead of a button |
| `ZMK_JOYCON2_LEGACY_ENDPOINT_API` | `n` | for pre-rename ZMK trees |

`ZMK_JOYCON2_RIGHT_C_CONSUMER_USAGE` diverts the right half's C button to the
HID Consumer page, reaching host functions a gamepad report cannot express.
`0x0D8` is the standard dictation key, which Android maps to `DICTATE`; note
that `0x0CF` ("Voice Command") is a different thing and only launches the
assistant. C stops acting as a gamepad button while this is set.

If a consumer usage does nothing on the host, suspect ZMK's report type before
suspecting the usage. `choice ZMK_HID_CONSUMER_REPORT_USAGES` has no `default`,
so builds silently get `..._USAGES_FULL` (16-bit ids), which ZMK's own help
warns "has compatibility issues with some host OSes" -- on Android that showed
up as media keys working only sometimes, for years, and dictation never.
`CONFIG_ZMK_HID_CONSUMER_REPORT_USAGES_BASIC=y` fixed both. Everything here
fits under its `0xFF` ceiling. It changes the report descriptor, so re-pair
afterwards.

`ZMK_JOYCON2_AUTO_RECONNECT` is off deliberately. Zephyr allows one scanner
with one callback, and ZMK's split central scans whenever a peripheral is
missing; while this module holds the scanner, ZMK's request fails but its
internal "scanning" flag stays set, so **the other keyboard half can stay
disconnected until a reset**. Brief on-demand scans make that unlikely, a
persistent scan would not. A controller also only advertises while awake, so on
an always-powered keyboard an idle controller that wakes in a bag can be
reconnected to and quietly drained.

## Limitations

- **Reconnect needs SYNC held.** The bonding sequence is sent and
  acknowledged, but no wake gesture has been found that makes a bonded
  controller advertise for us, so `JC2 REMEMBERS US` has never been observed.
- **GATT handles are hardcoded.** This device never answers enumeration-style
  ATT requests (`READ_BY_TYPE`, `READ_BY_GROUP_TYPE`) from us, in every
  configuration tried, though Android enumerates it in under a second. The
  handles below were recovered from Android's cached GATT database and verified
  on two controllers, but they may not hold across firmware revisions.
- Analog triggers, rumble, IMU and NFC are not implemented.
- Tested only on nRF52840 (nice!nano v2) against Android.

## Findings

Details that cost real time to establish, in case they save someone else:

**Input reports are 63 bytes**, which needs ATT MTU ≥ 66. At a smaller MTU the
controller sends nothing rather than truncating.

**Report layout** (little-endian): `[0:4]` timestamp, `[4:8]` buttons — the top
three bits are always set, mask with `0x03FFFFFF` — `[10:13]` left stick and
`[13:16]` right stick as 12-bit X/Y packed in three bytes, `[0x10]`/`[0x12]`
optical sensor position, `[0x17]` surface state, `[0x1F:0x21]` battery
millivolts, `[0x30:0x36]` accelerometer, `[0x36:0x3C]` gyroscope. **Each half
reports its own stick in its own field** and pins the other at `0x7FF`.

**The optical sensor reports accumulated position, not deltas** — movement is
the difference between consecutive samples — and `[0x17]` is inverted from its
name: `0` means *touching* a surface, higher means airborne. Discard the first
sample after landing, or the stale-versus-fresh difference throws the pointer
across the screen.

**Stick calibration lives at `0x000130A8` for both halves.** A stick travels
only about ±1200 of the raw 12-bit span, so scaling by the full half-span
reports roughly half scale at full tilt. Reference implementations read a
second address for a "stick 2" because Pro Controllers have two sticks; a right
Joy-Con returns *erased flash* there. Sanity-check what you read: all-`0xFFF`
taken literally means centre 4095, which pins both axes into one corner.

**A second HID service breaks BLE HID on Android.** Exposing the gamepad
through its own HID-over-GATT service stopped the keyboard pairing at all,
while USB HID still worked. The gamepad has to be another report inside the
existing service.

**Android maps HID buttons to a fixed keycode order** that includes two
vestigial entries and puts the stick clicks last: `A B C X Y Z L1 R1 L2 R2
Select Start Mode ThumbL ThumbR`. Nothing reads `C` or `Z`, so face buttons are
1, 2, 4, 5 and shoulders start at 7 — and those two spare slots are a good
place for bindings only your own software should see. A **D-pad must be a hat
switch**; four buttons produce no D-pad keycodes at all.

## Credits

This would not have been possible without prior reverse-engineering work.
None of their code is reused — different platforms and HID stacks — but their
findings are what made this tractable:

- **[trevlars/switch2-controllers-linux](https://github.com/trevlars/switch2-controllers-linux)**
  — the closest analogue to firmware, driving the controllers over a raw L2CAP
  ATT socket. The authority here for command framing, the initialisation order
  (features enabled *before* subscribing to input), the memory-read command,
  calibration structure, and for showing that bonding is optional for input.
- **[OZORDI/JoyCon2Mac](https://github.com/OZORDI/JoyCon2Mac)** — a macOS
  DriverKit driver. Source of the service and characteristic UUIDs, the command
  byte sequences, the input-report offsets, and the hardware-confirmed meaning
  of the surface byte and the pointer-jump problem it causes.
- **[Misaka10571/joycon2-connector](https://github.com/Misaka10571/joycon2-connector)**
  and the **joycon2cpp**/**joycon2py** lineage by
  [TheFrano](https://github.com/TheFrano) — feature-flag values, and the
  controller-side connection cooldown that explains controllers going quiet
  after repeated attempts.
- **[TiernanDeFranco/JoyConPlusPlus](https://github.com/TiernanDeFranco/JoyConPlusPlus)**
  — documents getting this far and *not* receiving input notifications, which
  is precisely the MTU trap above.

Built on **[ZMK](https://zmk.dev)**, and developed against
**[manna-harbour/miryoku_zmk](https://github.com/manna-harbour/miryoku_zmk)**.

## License

MIT — see [LICENSE](LICENSE).
