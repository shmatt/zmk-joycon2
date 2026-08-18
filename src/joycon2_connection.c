/*
 * Copyright (c) 2026 The zmk-joycon2 Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <zmk/joycon2/connection.h>
#include <zmk/joycon2/gamepad.h>
#if IS_ENABLED(CONFIG_ZMK_JOYCON2_MOUSE)
#include <zmk/joycon2/mouse.h>
#endif
#include <zmk/joycon2/debug_print.h>

LOG_MODULE_REGISTER(joycon2_connection, CONFIG_ZMK_LOG_LEVEL);

/* Exact ATT handles recovered via `adb shell dumpsys bluetooth_manager`'s
 * cached GATT database, from a successful Android/nRF Connect discovery
 * against a real Joy-Con 2 (see project notes). GATT discovery
 * (ATT_READ_BY_TYPE / ATT_READ_BY_GROUP_TYPE -- i.e. anything that
 * enumerates rather than looks up one specific thing) never gets a
 * response from this device's firmware, in every configuration tried
 * (full/scoped handle ranges, targeted/broad service lookup, slow/fast
 * connection interval, with/without MTU exchange, with/without an
 * inter-request delay) -- yet Android fully enumerates this same device in
 * under a second. Root cause unresolved; hardcoding the handles sidesteps
 * the entire class of problem rather than continuing to chase it.
 *
 * Caveat: these handles are almost certainly NOT portable across Joy-Con 2
 * firmware revisions or units -- a real fix (or a scan-time
 * re-verification step) is needed before this can be part of the eventual
 * shareable Stage 2 module.
 */
#define JOYCON2_INPUT_VALUE_HANDLE 0x000a
#define JOYCON2_INPUT_CCC_HANDLE 0x000b
#define JOYCON2_COMMAND_VALUE_HANDLE 0x0014
#define JOYCON2_RESPONSE_VALUE_HANDLE 0x001a
#define JOYCON2_RESPONSE_CCC_HANDLE 0x001b

#define JOYCON2_SCAN_TIMEOUT_SEC 20
#define JOYCON2_CONNECT_TIMEOUT_SEC 15
#define JOYCON2_HANDSHAKE_STEP_DELAY_MS 500
#define JOYCON2_SUBSCRIBE_TO_HANDSHAKE_DELAY_MS 500

/* Sequence and ordering match switch2-controllers-linux (the only known
 * working implementation at the raw-ATT level): subscribe RESPONSE ->
 * LED -> vibration preset -> feature-init -> feature-enable -> subscribe
 * INPUT **last**. In that implementation the input CCCD write is the
 * final step after features are enabled -- if it doubles as the "start
 * streaming" trigger, writing it first (as we did before) arms nothing.
 * MAC-binding ("bond") is optional reconnect persistence there, not part
 * of init, so it is dropped from the handshake entirely for now. */
enum handshake_step {
    HANDSHAKE_LED,
    HANDSHAKE_PAIRING_VIBRATION,
    HANDSHAKE_IMU_1,
    HANDSHAKE_IMU_2,
    HANDSHAKE_READ_STICK_CALIB,
    HANDSHAKE_SUBSCRIBE_INPUT,
    /* Bonding runs last, after input is already streaming, so that if it
     * ever disturbs the stream the cause is unambiguous. It only affects
     * reconnection, never this session. */
    HANDSHAKE_BOND_SET_MAC,
    HANDSHAKE_BOND_LTK1,
    HANDSHAKE_BOND_LTK2,
    HANDSHAKE_BOND_FINISH,
    HANDSHAKE_DONE,
};

/* Both halves can be connected at once, each with its own handshake,
 * subscriptions and side, so all of that lives per controller rather than in
 * file statics. Callbacks arrive keyed by bt_conn, subscribe_params or work
 * item, so each is mapped back to its owner below. */
#define JOYCON2_MAX_CONTROLLERS 2

struct joycon2_ctrl {
    struct bt_conn *conn;
    bool connecting;
    /* Which physical half this is, taken from the advertisement (see
     * eir_parse_cb). The gamepad mapping depends on it. */
    enum zmk_joycon2_side side;
    enum handshake_step step;
    struct bt_gatt_subscribe_params input_params;
    struct bt_gatt_subscribe_params response_params;
    struct bt_gatt_exchange_params mtu_params;
    struct k_work_delayable handshake_work;
    struct k_work_delayable connect_timeout_work;
    /* Button-change detection for the debug log, per controller: one half's
     * reports must not suppress the other's. */
    bool have_last_buttons;
    uint32_t last_buttons;
    /* Address of the outstanding memory read, so its reply can be told from
     * the command ACKs that share the response characteristic. */
    uint32_t pending_read_addr;
    bool calib_alt_tried;
};

static struct joycon2_ctrl controllers[JOYCON2_MAX_CONTROLLERS];
static bool scanning;

static struct k_work_delayable scan_timeout_work;

static struct joycon2_ctrl *ctrl_for_conn(struct bt_conn *conn) {
    for (size_t i = 0; i < ARRAY_SIZE(controllers); i++) {
        if (controllers[i].conn == conn) {
            return &controllers[i];
        }
    }
    return NULL;
}

static struct joycon2_ctrl *ctrl_free_slot(void) {
    for (size_t i = 0; i < ARRAY_SIZE(controllers); i++) {
        if (controllers[i].conn == NULL && !controllers[i].connecting) {
            return &controllers[i];
        }
    }
    return NULL;
}

static uint8_t ctrl_connected_count(void) {
    uint8_t n = 0;
    for (size_t i = 0; i < ARRAY_SIZE(controllers); i++) {
        if (controllers[i].conn != NULL) {
            n++;
        }
    }
    return n;
}

static const char *side_tag(enum zmk_joycon2_side side) {
    switch (side) {
    case ZMK_JOYCON2_SIDE_LEFT:
        return "L";
    case ZMK_JOYCON2_SIDE_RIGHT:
        return "R";
    default:
        return "?";
    }
}


static void hex_encode_and_print(const char *prefix, const uint8_t *data, uint16_t length) {
    /* Sized so a full 63-byte input report (126 hex chars) plus prefix
     * fits without truncation; debug_print's own buffer is 256. */
    char msg[160];
    int n = snprintf(msg, sizeof(msg), "%s ", prefix);
    for (uint16_t i = 0; i < length && n >= 0 && (size_t)n < sizeof(msg) - 3; i++) {
        n += snprintf(msg + n, sizeof(msg) - n, "%02X", data[i]);
    }
    zmk_joycon2_debug_print(msg);
}

/* Nintendo's Bluetooth SIG company identifier (little-endian in the AD
 * payload: 53 05). The Joy-Con 2's advertisement carries only Flags +
 * Manufacturer Specific Data -- confirmed by capturing its raw AD bytes via
 * nRF Connect (0201061BFF53050100037E0566200001000000000000000F...) -- no
 * service UUID list is broadcast at all, so filtering on the GATT service
 * UUID (only discoverable post-connection) never matches. */
#define JOYCON2_NINTENDO_COMPANY_ID 0x0553

struct eir_parse_ctx {
    bool found;
    enum zmk_joycon2_side side;
    /* The host this controller is bonded to, taken straight from the
     * advertisement; all-zero means it is in pairing mode. Same byte order
     * as Zephyr's bt_addr_le_t, so it compares directly against a.val. */
    bool have_bond_addr;
    uint8_t bond_addr[6];
};

/* Byte 5 of the manufacturer payload (after the 2-byte company ID)
 * identifies the half; values cross-checked against JoyConPlusPlus and
 * against both of our own units. */
#define JOYCON2_AD_SIDE_OFFSET 5
#define JOYCON2_AD_SIDE_RIGHT 0x66
#define JOYCON2_AD_SIDE_LEFT 0x67

/* Bytes 10..15 of the manufacturer payload carry the address of the host the
 * controller has been bonded to, and are all-zero while it is in pairing
 * mode. Confirmed against a capture taken with SYNC held, which read as all
 * zeroes. This is what makes a sync-free reconnect possible: we can tell
 * "waiting for us" from "bonded to something else" before connecting. */
#define JOYCON2_AD_BOND_ADDR_OFFSET 10
#define JOYCON2_AD_BOND_ADDR_LEN 6

static bool eir_parse_cb(struct bt_data *data, void *user_data) {
    struct eir_parse_ctx *ctx = user_data;

    if (data->type != BT_DATA_MANUFACTURER_DATA) {
        return true;
    }
    if (data->data_len < 2) {
        return true;
    }

    uint16_t company_id = sys_get_le16(data->data);
    if (company_id != JOYCON2_NINTENDO_COMPANY_ID) {
        return true;
    }

    ctx->found = true;

    const uint8_t *mfg = &data->data[2];
    uint8_t mfg_len = data->data_len - 2;

    if (mfg_len >= JOYCON2_AD_BOND_ADDR_OFFSET + JOYCON2_AD_BOND_ADDR_LEN) {
        memcpy(ctx->bond_addr, &mfg[JOYCON2_AD_BOND_ADDR_OFFSET], JOYCON2_AD_BOND_ADDR_LEN);
        ctx->have_bond_addr = true;
    }

    if (mfg_len > JOYCON2_AD_SIDE_OFFSET) {
        switch (mfg[JOYCON2_AD_SIDE_OFFSET]) {
        case JOYCON2_AD_SIDE_LEFT:
            ctx->side = ZMK_JOYCON2_SIDE_LEFT;
            break;
        case JOYCON2_AD_SIDE_RIGHT:
            ctx->side = ZMK_JOYCON2_SIDE_RIGHT;
            break;
        default:
            break;
        }
    }
    return false;
}

/* Input report layout, confirmed byte-for-byte against real captures from
 * this hardware (battery read back as a plausible 3176mV, accel Z ~1g and
 * gyro ~0 while at rest, sticks centred at ~2048, and the L button
 * appearing exactly where predicted). Offsets match
 * trevlars/switch2-controllers-linux's InputReport.parse. Reports are 63
 * bytes -- which needs ATT MTU >= 66, the reason nothing ever arrived
 * while the MTU sat at its 65-byte default. */
/* Factory stick-calibration addresses in controller memory. Each half keeps
 * its own stick's calibration at the address for the report field it uses. */
#define JOYCON2_CALIB_ADDR_STICK_LEFT 0x000130A8
#define JOYCON2_CALIB_ADDR_STICK_RIGHT 0x000130E8
#define JOYCON2_CALIB_READ_LEN 0x0B

/* A memory-read reply is the 8-byte command header, then the echoed length,
 * three fixed bytes and the echoed address, and only then the data. */
#define JOYCON2_READ_REPLY_ADDR_OFFSET 12
#define JOYCON2_READ_REPLY_DATA_OFFSET 16

/* 12-bit X and Y packed into three bytes, the same packing the sticks use. */
static void unpack_xy(const uint8_t *d, uint16_t *x, uint16_t *y) {
    *x = d[0] | ((uint16_t)(d[1] & 0x0F) << 8);
    *y = (d[1] >> 4) | ((uint16_t)d[2] << 4);
}

/* BOTH halves keep their stick's calibration at the "stick 1" address,
 * confirmed by reading real hardware: a right Joy-Con returned erased flash
 * from the "stick 2" address and valid data from stick 1. That makes sense --
 * each Joy-Con has one stick, so it is stick 1 from its own point of view;
 * the second address only means anything on a two-stick controller like the
 * Pro Controller. The reference implementations read both because they also
 * support those, which is what led to the wrong guess here.
 *
 * The second address is still tried as a fallback, so a two-stick controller
 * would work if support for one is ever added.
 */
static uint32_t stick_calib_addr(enum zmk_joycon2_side side, bool alternate) {
    ARG_UNUSED(side);
    return alternate ? JOYCON2_CALIB_ADDR_STICK_RIGHT : JOYCON2_CALIB_ADDR_STICK_LEFT;
}

static int jc_write_command(struct joycon2_ctrl *c, const uint8_t *data, size_t len);

static void read_stick_calibration(struct joycon2_ctrl *c, uint32_t addr) {
    uint8_t cmd[16] = {0x02, 0x91, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00,
                        JOYCON2_CALIB_READ_LEN, 0x7E, 0x00, 0x00};
    sys_put_le32(addr, &cmd[12]);

    c->pending_read_addr = addr;
    int err = jc_write_command(c, cmd, sizeof(cmd));
    LOG_INF("joycon2: read stick calibration at %08x (%d)", addr, err);
}

/* Returns true if this response was our memory read rather than a command
 * ACK, so the caller knows not to treat it as one. */
static bool handle_memory_read_reply(struct joycon2_ctrl *c, const uint8_t *data,
                                      uint16_t length) {
    if (c->pending_read_addr == 0 || length < JOYCON2_READ_REPLY_DATA_OFFSET + 9) {
        return false;
    }
    /* Command 0x02 is the memory read; anything else is an ACK. */
    if (data[0] != 0x02) {
        return false;
    }
    if (sys_get_le32(&data[JOYCON2_READ_REPLY_ADDR_OFFSET]) != c->pending_read_addr) {
        return false;
    }

    const uint8_t *d = &data[JOYCON2_READ_REPLY_DATA_OFFSET];
    struct zmk_joycon2_stick_calib calib;
    /* Centre first, then the travel either side of it. */
    unpack_xy(&d[0], &calib.center_x, &calib.center_y);
    unpack_xy(&d[3], &calib.max_x, &calib.max_y);
    unpack_xy(&d[6], &calib.min_x, &calib.min_y);

    c->pending_read_addr = 0;

    /* Erased flash reads as all-ones. Seen on a real right Joy-Con at the
     * address the reference implied, so try the other one before settling
     * for defaults. */
    bool erased = (calib.center_x == 0x0FFF && calib.center_y == 0x0FFF);
    if (erased && !c->calib_alt_tried) {
        c->calib_alt_tried = true;
        zmk_joycon2_debug_print("JC2 CAL RETRY");
        read_stick_calibration(c, stick_calib_addr(c->side, true));
        return true;
    }

#if IS_ENABLED(CONFIG_ZMK_JOYCON2_GAMEPAD)
    zmk_joycon2_gamepad_set_calibration(c->side, &calib);
#endif

    char msg[80];
    snprintf(msg, sizeof(msg), "JC2 CAL-%s c=%u,%u +%u,%u -%u,%u", side_tag(c->side),
             calib.center_x, calib.center_y, calib.max_x, calib.max_y, calib.min_x, calib.min_y);
    zmk_joycon2_debug_print(msg);
    return true;
}

#define JOYCON2_REPORT_MIN_LEN 8
#define JOYCON2_REPORT_BUTTONS_OFFSET 4
#define JOYCON2_REPORT_BATTERY_OFFSET 0x1F
/* Each half reports its own stick in its own field, and leaves the other
 * field at a constant 0x7FF (dead centre). Reading the wrong one gives a
 * stick that never moves -- which is exactly what a right Joy-Con did while
 * this always read the left field. */
#define JOYCON2_REPORT_LEFT_STICK_OFFSET 10
#define JOYCON2_REPORT_RIGHT_STICK_OFFSET 13

/* Optical mouse sensor: an accumulated position rather than per-frame deltas,
 * plus a surface reading at 0x17 where 0 means touching. */
#define JOYCON2_REPORT_MOUSE_X_OFFSET 0x10
#define JOYCON2_REPORT_MOUSE_Y_OFFSET 0x12
#define JOYCON2_REPORT_SURFACE_OFFSET 0x17

/* The top three bits of the button word are always set on this hardware
 * (some always-on status flags, not buttons) -- mask to the documented
 * button bits so they don't read as phantom presses. */
#define JOYCON2_BUTTON_MASK 0x03FFFFFFU

struct joycon2_button {
    uint32_t mask;
    const char *name;
};

/* Left-half buttons first since that is the unit under test; the right
 * half's bits are included so the same decode works for either. */
static const struct joycon2_button joycon2_buttons[] = {
    {0x00010000, "DN"},   {0x00020000, "UP"},   {0x00040000, "RT"},
    {0x00080000, "LF"},   {0x00100000, "SR-L"}, {0x00200000, "SL-L"},
    {0x00400000, "L"},    {0x00800000, "ZL"},   {0x02000000, "GL"},
    {0x00000100, "MINUS"},{0x00000800, "LSTK"}, {0x00002000, "CAP"},
    {0x00000001, "Y"},    {0x00000002, "X"},    {0x00000004, "B"},
    {0x00000008, "A"},    {0x00000010, "SR-R"}, {0x00000020, "SL-R"},
    {0x00000040, "R"},    {0x00000080, "ZR"},   {0x00000200, "PLUS"},
    {0x00000400, "RSTK"}, {0x00001000, "HOME"}, {0x00004000, "C"},
    {0x01000000, "GR"},
};

/* Reports stream at ~60-120Hz but the HID-typing debug channel manages
 * only a few characters per report period, so print on button-state
 * CHANGE only -- that is both what proves the decode and the only rate a
 * human-readable channel can sustain. Even then it is off unless input
 * logging is toggled on, since typing into the focused window makes the
 * gamepad unusable for actually playing anything. */
static void decode_input_report(struct joycon2_ctrl *c, const uint8_t *data, uint16_t length) {
    if (length < JOYCON2_REPORT_MIN_LEN) {
        return;
    }

    uint32_t buttons = sys_get_le32(&data[JOYCON2_REPORT_BUTTONS_OFFSET]) & JOYCON2_BUTTON_MASK;

    /* Each half reports its own stick, so pick the field for this side and
     * parse it once: the gamepad and the mouse's scroll wheel both want it. */
    uint16_t stick_x = 0;
    uint16_t stick_y = 0;
    uint16_t stick_offset = (c->side == ZMK_JOYCON2_SIDE_RIGHT)
                                ? JOYCON2_REPORT_RIGHT_STICK_OFFSET
                                : JOYCON2_REPORT_LEFT_STICK_OFFSET;
    if (length >= stick_offset + 3) {
        /* 12-bit X and Y packed into three bytes. */
        const uint8_t *st = &data[stick_offset];
        stick_x = st[0] | ((uint16_t)(st[1] & 0x0F) << 8);
        stick_y = (st[1] >> 4) | ((uint16_t)st[2] << 4);
    }

#if IS_ENABLED(CONFIG_ZMK_JOYCON2_MOUSE)
    /* Before the gamepad, so that when this half takes over the pointer the
     * gamepad sees it on this same report rather than one report later. */
    if (length > JOYCON2_REPORT_SURFACE_OFFSET) {
        zmk_joycon2_mouse_update(c->side,
                                  (int16_t)sys_get_le16(&data[JOYCON2_REPORT_MOUSE_X_OFFSET]),
                                  (int16_t)sys_get_le16(&data[JOYCON2_REPORT_MOUSE_Y_OFFSET]),
                                  data[JOYCON2_REPORT_SURFACE_OFFSET], buttons, stick_x, stick_y);
    }
#endif

#if IS_ENABLED(CONFIG_ZMK_JOYCON2_GAMEPAD)
    /* Runs for every report, not just on button change, so stick movement is
     * continuous; it does its own change detection and rate limiting. */
    zmk_joycon2_gamepad_update(c->side, buttons, stick_x, stick_y);
#endif

    if (c->have_last_buttons && buttons == c->last_buttons) {
        return;
    }
    c->have_last_buttons = true;
    c->last_buttons = buttons;

    /* Redundant for correctness -- debug_print gates the whole channel --
     * but formatting a 128-byte string at 60-120Hz is not free. */
    if (!zmk_joycon2_debug_logging_enabled()) {
        return;
    }

    char msg[128];
    int n = snprintf(msg, sizeof(msg), "JC2 BTN-%s", side_tag(c->side));
    if (buttons == 0) {
        n += snprintf(msg + n, sizeof(msg) - n, " --");
    } else {
        for (size_t i = 0; i < ARRAY_SIZE(joycon2_buttons) && n > 0 && (size_t)n < sizeof(msg); i++) {
            if (buttons & joycon2_buttons[i].mask) {
                n += snprintf(msg + n, sizeof(msg) - n, " %s", joycon2_buttons[i].name);
            }
        }
    }

    /* Battery is cheap to include and doubles as a sanity check that the
     * whole report is still being framed correctly. */
    if (length >= JOYCON2_REPORT_BATTERY_OFFSET + 2) {
        uint16_t mv = sys_get_le16(&data[JOYCON2_REPORT_BATTERY_OFFSET]);
        snprintf(msg + n, sizeof(msg) - n, " %umV", mv);
    }

    zmk_joycon2_debug_print(msg);
}

static void start_handshake(struct joycon2_ctrl *c) {
    c->step = HANDSHAKE_LED;
    k_work_schedule(&c->handshake_work, K_MSEC(JOYCON2_HANDSHAKE_STEP_DELAY_MS));
}

/* Notify and subscribe callbacks identify their controller by which of its
 * two params structs was passed. */
static struct joycon2_ctrl *ctrl_for_params(struct bt_gatt_subscribe_params *params) {
    for (size_t i = 0; i < ARRAY_SIZE(controllers); i++) {
        if (params == &controllers[i].input_params || params == &controllers[i].response_params) {
            return &controllers[i];
        }
    }
    return NULL;
}

static uint8_t input_notify_func(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
                                  const void *data, uint16_t length) {
    ARG_UNUSED(conn);

    if (!data) {
        LOG_INF("joycon2: input unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    struct joycon2_ctrl *c = ctrl_for_params(params);
    if (c == NULL) {
        return BT_GATT_ITER_CONTINUE;
    }

    decode_input_report(c, data, length);
    return BT_GATT_ITER_CONTINUE;
}

static uint8_t response_notify_func(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
                                     const void *data, uint16_t length) {
    ARG_UNUSED(conn);

    if (!data) {
        LOG_INF("joycon2: response unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    struct joycon2_ctrl *c = ctrl_for_params(params);
    if (c != NULL && handle_memory_read_reply(c, data, length)) {
        return BT_GATT_ITER_CONTINUE;
    }

    hex_encode_and_print("JC2 ACK", data, length);
    return BT_GATT_ITER_CONTINUE;
}

static int jc_write_command(struct joycon2_ctrl *c, const uint8_t *data, size_t len) {
    if (c->conn == NULL) {
        return -ENOTCONN;
    }
    /* sign=false: this controller flatly rejects standard BLE bonding (no
     * CSRK is ever established), unlike ZMK's own split link which is
     * bonded and can use signed writes. */
    return bt_gatt_write_without_response(c->conn, JOYCON2_COMMAND_VALUE_HANDLE, data, len, false);
}

static void subscribe_input(struct joycon2_ctrl *c);

static void handshake_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct joycon2_ctrl *c = CONTAINER_OF(dwork, struct joycon2_ctrl, handshake_work);
    int err;

    if (c->conn == NULL) {
        return;
    }

    switch (c->step) {
    case HANDSHAKE_LED: {
        /* ledMask 0x01 = player-1 pattern. */
        static const uint8_t cmd[] = {0x09, 0x91, 0x01, 0x07, 0x00, 0x08, 0x00, 0x00,
                                       0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        err = jc_write_command(c, cmd, sizeof(cmd));
        LOG_INF("joycon2: set player LED (%d)", err);
        break;
    }
    case HANDSHAKE_PAIRING_VIBRATION: {
        /* "Play vibration preset 3" -- the connected/pairing buzz. */
        static const uint8_t cmd[] = {0x0A, 0x91, 0x01, 0x02, 0x00, 0x04,
                                       0x00, 0x00, 0x03, 0x00, 0x00, 0x00};
        err = jc_write_command(c, cmd, sizeof(cmd));
        LOG_INF("joycon2: pairing vibration (%d)", err);
        break;
    }
    case HANDSHAKE_IMU_1: {
        /* Feature-init, flags 0xFF = all features (0x0C/0x02). */
        static const uint8_t cmd[] = {0x0C, 0x91, 0x01, 0x02, 0x00, 0x04,
                                       0x00, 0x00, 0xFF, 0x00, 0x00, 0x00};
        err = jc_write_command(c, cmd, sizeof(cmd));
        LOG_INF("joycon2: feature init (%d)", err);
        break;
    }
    case HANDSHAKE_IMU_2: {
        /* Feature-enable, same flags (0x0C/0x04). */
        static const uint8_t cmd[] = {0x0C, 0x91, 0x01, 0x04, 0x00, 0x04,
                                       0x00, 0x00, 0xFF, 0x00, 0x00, 0x00};
        err = jc_write_command(c, cmd, sizeof(cmd));
        LOG_INF("joycon2: feature enable (%d)", err);
        break;
    }
    case HANDSHAKE_READ_STICK_CALIB: {
        read_stick_calibration(c, stick_calib_addr(c->side, false));
        break;
    }
    case HANDSHAKE_SUBSCRIBE_INPUT:
        subscribe_input(c);
        zmk_joycon2_debug_print("JC2 HANDSHAKE SENT");
        if (!IS_ENABLED(CONFIG_ZMK_JOYCON2_BOND)) {
            c->step = HANDSHAKE_DONE;
            return;
        }
        break;

#if IS_ENABLED(CONFIG_ZMK_JOYCON2_BOND)
    case HANDSHAKE_BOND_SET_MAC: {
        /* Stores this host's address on the controller so it will wake and
         * reconnect to us on a button press instead of needing SYNC held.
         * The reference sends the host address twice, little-endian --
         * which is exactly Zephyr's bt_addr_le_t byte order, so a.val goes
         * out as-is. (An earlier attempt reversed it, i.e. sent it
         * backwards, which would explain why bonding never took.) */
        bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
        size_t count = ARRAY_SIZE(addrs);

        bt_id_get(addrs, &count);
        if (count == 0) {
            LOG_ERR("joycon2: no local BT identity address");
            zmk_joycon2_debug_print("JC2 NO LOCAL BT ADDR");
            c->step = HANDSHAKE_DONE;
            return;
        }

        uint8_t buf[22] = {0x15, 0x91, 0x01, 0x01, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x02};
        memcpy(&buf[10], addrs[0].a.val, 6);
        memcpy(&buf[16], addrs[0].a.val, 6);

        err = jc_write_command(c, buf, sizeof(buf));
        LOG_INF("joycon2: bond set-mac (%d)", err);
        break;
    }
    case HANDSHAKE_BOND_LTK1: {
        /* Opaque 16-byte key the host chooses; the reference implementations
         * each ship a different constant, which is how we know these are
         * not captured console secrets. */
        static const uint8_t cmd[] = {0x15, 0x91, 0x01, 0x04, 0x00, 0x11, 0x00, 0x00, 0x00,
                                       0xEA, 0xBD, 0x47, 0x13, 0x89, 0x35, 0x42, 0xC6, 0x79,
                                       0xEE, 0x07, 0xF2, 0x53, 0x2C, 0x6C, 0x31};
        err = jc_write_command(c, cmd, sizeof(cmd));
        LOG_INF("joycon2: bond ltk1 (%d)", err);
        break;
    }
    case HANDSHAKE_BOND_LTK2: {
        static const uint8_t cmd[] = {0x15, 0x91, 0x01, 0x02, 0x00, 0x11, 0x00, 0x00, 0x00,
                                       0x40, 0xB0, 0x8A, 0x5F, 0xCD, 0x1F, 0x9B, 0x41, 0x12,
                                       0x5C, 0xAC, 0xC6, 0x3F, 0x38, 0xA0, 0x73};
        err = jc_write_command(c, cmd, sizeof(cmd));
        LOG_INF("joycon2: bond ltk2 (%d)", err);
        break;
    }
    case HANDSHAKE_BOND_FINISH: {
        static const uint8_t cmd[] = {0x15, 0x91, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00};
        err = jc_write_command(c, cmd, sizeof(cmd));
        LOG_INF("joycon2: bond commit (%d)", err);
        zmk_joycon2_debug_print("JC2 BOND SENT");
        c->step = HANDSHAKE_DONE;
        return;
    }
#endif // IS_ENABLED(CONFIG_ZMK_JOYCON2_BOND)

    default:
        return;
    }

    c->step++;
    k_work_schedule(&c->handshake_work, K_MSEC(JOYCON2_HANDSHAKE_STEP_DELAY_MS));
}

/* struct bt_gatt_subscribe_params.subscribe reports the CCC WRITE's actual
 * ATT-level result -- separate from .notify, which only ever fires for
 * real value notifications once subscribed. Never having set this means
 * a silently-failed CCCD write on one specific characteristic (while
 * others succeed) would look identical to "no data ever sent" -- exactly
 * the symptom seen (RESPONSE notifies fine, INPUT never does). */
static void subscribe_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_subscribe_params *params) {
    ARG_UNUSED(conn);

    struct joycon2_ctrl *c = ctrl_for_params(params);
    const char *name = (c != NULL && params == &c->input_params) ? "IN" : "RESP";

    char msg[48];
    snprintf(msg, sizeof(msg), "JC2 CCC-%s %s err=%u", side_tag(c ? c->side : 0), name, err);
    zmk_joycon2_debug_print(msg);
}

/* Deliberately called LAST, after feature-enable, matching the working
 * Linux implementation's ordering -- see the handshake_step comment. */
static void subscribe_input(struct joycon2_ctrl *c) {
    if (c->conn == NULL) {
        return;
    }

    c->input_params.value_handle = JOYCON2_INPUT_VALUE_HANDLE;
    c->input_params.ccc_handle = JOYCON2_INPUT_CCC_HANDLE;
    c->input_params.notify = input_notify_func;
    c->input_params.subscribe = subscribe_cb;
    c->input_params.value = BT_GATT_CCC_NOTIFY;
    int err = bt_gatt_subscribe(c->conn, &c->input_params);
    if (err && err != -EALREADY) {
        LOG_ERR("joycon2: input subscribe failed (%d)", err);
        zmk_joycon2_debug_print("JC2 INPUT SUBSCRIBE FAILED");
    }
}

static void start_subscriptions(struct joycon2_ctrl *c) {
    /* Explicit ccc_handle (rather than .disc_params) skips CCC
     * auto-discovery entirely -- that path also uses a READ_BY_TYPE-style
     * op internally and would likely hit the same wall as characteristic
     * discovery did. Only RESPONSE is subscribed here; INPUT is subscribed
     * as the handshake's final step. */
    int err;

    c->response_params.value_handle = JOYCON2_RESPONSE_VALUE_HANDLE;
    c->response_params.ccc_handle = JOYCON2_RESPONSE_CCC_HANDLE;
    c->response_params.notify = response_notify_func;
    c->response_params.subscribe = subscribe_cb;
    c->response_params.value = BT_GATT_CCC_NOTIFY;
    err = bt_gatt_subscribe(c->conn, &c->response_params);
    if (err && err != -EALREADY) {
        LOG_ERR("joycon2: response subscribe failed (%d)", err);
        zmk_joycon2_debug_print("JC2 RESPONSE SUBSCRIBE FAILED");
    }

    zmk_joycon2_debug_print("JC2 SUBSCRIBED");
    /* start_handshake resets the step -- without it a reconnect would find
     * it still at HANDSHAKE_DONE and never handshake. */
    start_handshake(c);
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
                             struct bt_gatt_exchange_params *params) {
    ARG_UNUSED(params);

    struct joycon2_ctrl *c = ctrl_for_conn(conn);
    if (c == NULL) {
        return;
    }

    /* JoyConDecoder (the reference driver) needs a report of at least 62
     * bytes to hold buttons+sticks+motion+battery -- if the negotiated ATT
     * MTU can't fit that (MTU - 3 byte header must be >= 62), that alone
     * could fully explain zero input notifications ever arriving despite
     * a successful handshake. Surfacing this via debug-print since the
     * previous LOG_INF was never actually visible without serial. */
    char msg[32];
    if (err) {
        LOG_ERR("joycon2: MTU exchange failed (err %u)", err);
        snprintf(msg, sizeof(msg), "JC2 MTU EXCHANGE FAILED %u", err);
        zmk_joycon2_debug_print(msg);
    } else {
        uint16_t mtu = bt_gatt_get_mtu(conn);
        LOG_INF("joycon2: MTU exchange succeeded, ATT MTU=%u", mtu);
        /* An MTU below 66 cannot carry a 63-byte input report at all. */
        snprintf(msg, sizeof(msg), "JC2 MTU=%u", mtu);
        zmk_joycon2_debug_print(msg);
    }

    start_subscriptions(c);
}

static void scan_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                        struct net_buf_simple *ad);

/* Restart the scan when a slot is still free, so one combo press can pick up
 * both halves as they are synced in turn. */
static void resume_scan_if_slot_free(void) {
    if (scanning || ctrl_free_slot() == NULL) {
        return;
    }

    int err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, scan_found);
    if (err) {
        LOG_WRN("joycon2: scan resume failed (%d)", err);
        return;
    }

    scanning = true;
    k_work_schedule(&scan_timeout_work, K_SECONDS(JOYCON2_SCAN_TIMEOUT_SEC));
}

static void scan_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                        struct net_buf_simple *ad) {
    ARG_UNUSED(rssi);

    if (!scanning) {
        return;
    }
    if (type != BT_GAP_ADV_TYPE_ADV_IND) {
        return;
    }

    struct joycon2_ctrl *c = ctrl_free_slot();
    if (c == NULL) {
        return;
    }

    struct eir_parse_ctx ctx = {.found = false, .side = ZMK_JOYCON2_SIDE_UNKNOWN};
    bt_data_parse(ad, eir_parse_cb, &ctx);
    if (!ctx.found) {
        return;
    }

    /* Skip controllers bonded to a different host: they are advertising for
     * that host, not for us, and grabbing them would steal them from it.
     * A controller in pairing mode advertises an all-zero address and is
     * fair game, as is one already bonded to us. */
    if (ctx.have_bond_addr) {
        bt_addr_le_t self[CONFIG_BT_ID_MAX];
        size_t count = ARRAY_SIZE(self);
        static const uint8_t no_bond[JOYCON2_AD_BOND_ADDR_LEN] = {0};

        bt_id_get(self, &count);

        bool pairing_mode = memcmp(ctx.bond_addr, no_bond, sizeof(no_bond)) == 0;
        bool bonded_to_us =
            count > 0 && memcmp(ctx.bond_addr, self[0].a.val, JOYCON2_AD_BOND_ADDR_LEN) == 0;

        if (!pairing_mode && !bonded_to_us) {
            hex_encode_and_print("JC2 SKIP BONDED", ctx.bond_addr, sizeof(ctx.bond_addr));
            return;
        }

        zmk_joycon2_debug_print(pairing_mode ? "JC2 PAIRMODE" : "JC2 REMEMBERS US");
    }

    struct bt_conn *existing = bt_conn_lookup_addr_le(BT_ID_DEFAULT, addr);
    if (existing != NULL) {
        bt_conn_unref(existing);
        return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    LOG_INF("joycon2: found %s, connecting", addr_str);

    /* One connection can be set up at a time: bt_conn_le_create needs the
     * initiator, which the scanner is using. Scanning resumes from
     * jc_connected if a slot is still free. */
    bt_le_scan_stop();
    scanning = false;
    k_work_cancel_delayable(&scan_timeout_work);

    c->side = ctx.side;
    c->connecting = true;

    /* 7.5-15ms: fast enough for a 120Hz+ input stream. The previous
     * 30-50ms request may have been too slow for the device to bother
     * streaming (matches Android's initial 7.5ms connection). */
    struct bt_le_conn_param *param = BT_LE_CONN_PARAM(0x0006, 0x000C, 0, 400);
    int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, param, &c->conn);
    if (err) {
        LOG_ERR("joycon2: create conn failed (%d)", err);
        c->connecting = false;
        zmk_joycon2_debug_print("JC2 CREATE CONN FAILED");
        return;
    }

    zmk_joycon2_debug_print("JC2 FOUND CONNECTING");
    /* Independent watchdog: don't rely solely on whatever internal timeout
     * bt_conn_le_create()'s default create params use -- if neither
     * jc_connected nor jc_disconnected ever fires, this guarantees we
     * eventually report *something* instead of hanging silently. */
    k_work_schedule(&c->connect_timeout_work, K_SECONDS(JOYCON2_CONNECT_TIMEOUT_SEC));
}

static void scan_timeout_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (!scanning) {
        return;
    }

    bt_le_scan_stop();
    scanning = false;
    /* Not an error when one controller is already connected: the scan was
     * just waiting to see whether a second would turn up. */
    zmk_joycon2_debug_print(ctrl_connected_count() > 0 ? "JC2 SCAN DONE"
                                                       : "JC2 SCAN TIMEOUT NOT FOUND");
}

static void connect_timeout_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct joycon2_ctrl *c = CONTAINER_OF(dwork, struct joycon2_ctrl, connect_timeout_work);

    if (c->conn == NULL) {
        return;
    }

    LOG_ERR("joycon2: connect attempt timed out");
    bt_conn_disconnect(c->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    bt_conn_unref(c->conn);
    c->conn = NULL;
    c->connecting = false;
    zmk_joycon2_debug_print("JC2 CONNECT TIMEOUT");
}

static void jc_connected(struct bt_conn *conn, uint8_t err) {
    struct joycon2_ctrl *c = ctrl_for_conn(conn);
    if (c == NULL) {
        /* Not one of ours -- the host link or the other keyboard half. */
        return;
    }

    k_work_cancel_delayable(&c->connect_timeout_work);
    c->connecting = false;

    if (err) {
        LOG_ERR("joycon2: connect failed (err %d)", err);
        bt_conn_unref(c->conn);
        c->conn = NULL;
        zmk_joycon2_debug_print("JC2 CONNECT FAILED");
        resume_scan_if_slot_free();
        return;
    }

    LOG_INF("joycon2: connected, subscribing");
    char msg[32];
    snprintf(msg, sizeof(msg), "JC2 CONNECTED %s", side_tag(c->side));
    zmk_joycon2_debug_print(msg);

#if IS_ENABLED(CONFIG_ZMK_JOYCON2_GAMEPAD)
    /* One controller selects the solo mapping, two the duo mapping. */
    zmk_joycon2_gamepad_set_connected_count(ctrl_connected_count());
#endif

    /* Keep looking for the other half while this one finishes its
     * handshake, so a single combo press can pick up both. */
    resume_scan_if_slot_free();

    /* Not required before subscribing works, but the default 23-byte ATT
     * MTU would truncate longer input-report notifications later, and no
     * known-working client against this device skips it. Subscriptions
     * start from mtu_exchange_cb once this actually completes. */
    c->mtu_params.func = mtu_exchange_cb;
    int mtu_err = bt_gatt_exchange_mtu(conn, &c->mtu_params);
    if (mtu_err) {
        LOG_ERR("joycon2: MTU exchange request failed (%d)", mtu_err);
        /* No callback will ever fire in this case -- proceed directly. */
        start_subscriptions(c);
    }
}

static void jc_disconnected(struct bt_conn *conn, uint8_t reason) {
    struct joycon2_ctrl *c = ctrl_for_conn(conn);
    if (c == NULL) {
        return;
    }

    LOG_INF("joycon2: disconnected (reason %d)", reason);
    k_work_cancel_delayable(&c->connect_timeout_work);
    k_work_cancel_delayable(&c->handshake_work);
    bt_conn_unref(c->conn);
    c->conn = NULL;
    c->connecting = false;
    c->step = HANDSHAKE_DONE;
    c->have_last_buttons = false;
    c->pending_read_addr = 0;
    c->calib_alt_tried = false;
#if IS_ENABLED(CONFIG_ZMK_JOYCON2_MOUSE)
    zmk_joycon2_mouse_reset(c->side);
#endif

    char msg[48];
    snprintf(msg, sizeof(msg), "JC2 DISCONNECTED %s reason=0x%02x", side_tag(c->side), reason);
    c->side = ZMK_JOYCON2_SIDE_UNKNOWN;

#if IS_ENABLED(CONFIG_ZMK_JOYCON2_GAMEPAD)
    zmk_joycon2_gamepad_set_connected_count(ctrl_connected_count());
#endif

    zmk_joycon2_debug_print(msg);

#if IS_ENABLED(CONFIG_ZMK_JOYCON2_AUTO_RECONNECT)
    /* A bonded controller advertises for this host when it wakes, so a
     * bounded scan gives it a chance to come back without the combo. Bounded
     * on purpose: see the Kconfig note about sharing the scanner with ZMK's
     * split central. */
    if (!scanning && ctrl_free_slot() != NULL) {
        int err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, scan_found);
        if (err) {
            /* -EALREADY means ZMK's split central owns the scanner; it needs
             * it more than we do, so leave it alone. */
            LOG_WRN("joycon2: reconnect scan not started (%d)", err);
        } else {
            scanning = true;
            k_work_schedule(&scan_timeout_work,
                            K_SECONDS(CONFIG_ZMK_JOYCON2_AUTO_RECONNECT_WINDOW_SEC));
        }
    }
#endif
}

static void jc_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
                                 uint16_t timeout) {
    if (ctrl_for_conn(conn) == NULL) {
        return;
    }

    /* Report what interval the link ACTUALLY runs at -- the request in
     * bt_conn_le_create is only a request, and the peripheral can
     * renegotiate afterwards (Android's log showed this device moving
     * 7.5ms -> 30ms shortly after connecting). Interval unit: 1.25ms. */
    char msg[48];
    snprintf(msg, sizeof(msg), "JC2 CI %u.%02ums lat=%u", (interval * 125) / 100,
             (unsigned)((interval * 125) % 100), latency);
    zmk_joycon2_debug_print(msg);
}

#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
static void jc_le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info) {
    if (ctrl_for_conn(conn) == NULL) {
        return;
    }

    /* Direct observation of whether Data Length Extension actually
     * negotiated on this link (tx/rx = max Link Layer payload bytes;
     * 27 = no DLE, 251 = full). A 62-byte input report needs 70 bytes
     * on-air, and the hypothesis is Nintendo's input hot path refuses
     * to fragment across 27-byte PDUs. */
    char msg[48];
    snprintf(msg, sizeof(msg), "JC2 DLE tx=%u rx=%u", info->tx_max_len, info->rx_max_len);
    zmk_joycon2_debug_print(msg);
}
#endif

static struct bt_conn_cb jc_conn_callbacks = {
    .connected = jc_connected,
    .disconnected = jc_disconnected,
    .le_param_updated = jc_le_param_updated,
#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
    .le_data_len_updated = jc_le_data_len_updated,
#endif
};

int zmk_joycon2_connection_start(void) {
    if (scanning) {
        return -EBUSY;
    }

    /* Deliberately keeps whatever is already connected: pressing the combo
     * again is how a second half gets added, so tearing down the first would
     * defeat the point. */
    if (ctrl_free_slot() == NULL) {
        zmk_joycon2_debug_print("JC2 BOTH CONNECTED");
        return -ENOSPC;
    }

    /* Immediate feedback that the combo registered at all, decoupled from
     * whatever the scan eventually finds (which can take up to
     * JOYCON2_SCAN_TIMEOUT_SEC to resolve one way or the other). Includes
     * this build's git hash so a fast-iterating build/flash loop can't
     * accidentally leave stale firmware ambiguity. */
#ifndef ZMK_JOYCON2_GIT_HASH
#define ZMK_JOYCON2_GIT_HASH "unknown"
#endif
    zmk_joycon2_debug_print("JC2 SCANNING " ZMK_JOYCON2_GIT_HASH);

    int err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, scan_found);
    if (err) {
        LOG_ERR("joycon2: scan start failed (%d)", err);
        zmk_joycon2_debug_print("JC2 SCAN START FAILED");
        return err;
    }

    scanning = true;
    k_work_schedule(&scan_timeout_work, K_SECONDS(JOYCON2_SCAN_TIMEOUT_SEC));
    return 0;
}

static int joycon2_connection_init(void) {
    k_work_init_delayable(&scan_timeout_work, scan_timeout_work_handler);
    for (size_t i = 0; i < ARRAY_SIZE(controllers); i++) {
        controllers[i].step = HANDSHAKE_DONE;
        k_work_init_delayable(&controllers[i].handshake_work, handshake_work_handler);
        k_work_init_delayable(&controllers[i].connect_timeout_work, connect_timeout_work_handler);
    }
    bt_conn_cb_register(&jc_conn_callbacks);
    return 0;
}

SYS_INIT(joycon2_connection_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
