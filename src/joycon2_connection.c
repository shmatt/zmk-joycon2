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

/* The handshake completed successfully (confirmed via structured ACKs on
 * RESPONSE), but no data ever arrived on INPUT even after pressing
 * buttons. Subscribing to every other notify-capable characteristic in
 * the service too, to find out which one (if any) actually carries live
 * button/stick state -- the two "groups" of notify characteristics seen
 * during manual nRF Connect exploration (grouped by shared vendor
 * descriptor UUID) may carry different kinds of data. */
#define JOYCON2_ALT1_VALUE_HANDLE 0x000e /* d5a9e01e-2ffc-4cca-b20c-8b67142bf442 */
#define JOYCON2_ALT1_CCC_HANDLE 0x000f
#define JOYCON2_ALT2_VALUE_HANDLE 0x001e /* 640ca58e-0e88-410c-a7f3-426faf2b690b */
#define JOYCON2_ALT2_CCC_HANDLE 0x001f
#define JOYCON2_ALT3_VALUE_HANDLE 0x0022 /* d3bd69d2-841c-4241-ab15-f86f406d2a80 */
#define JOYCON2_ALT3_CCC_HANDLE 0x0023
#define JOYCON2_ALT4_VALUE_HANDLE 0x0026 /* ab7de9be-89fe-49ad-828f-118f09df7fde */
#define JOYCON2_ALT4_CCC_HANDLE 0x0027

#define JOYCON2_SCAN_TIMEOUT_SEC 20
#define JOYCON2_CONNECT_TIMEOUT_SEC 15
#define JOYCON2_HANDSHAKE_STEP_DELAY_MS 500
#define JOYCON2_SUBSCRIBE_TO_HANDSHAKE_DELAY_MS 500

static struct bt_conn *jc_conn;
static bool jc_connecting;

static struct bt_gatt_subscribe_params input_subscribe_params;
static struct bt_gatt_subscribe_params response_subscribe_params;
static struct bt_gatt_subscribe_params alt1_subscribe_params;
static struct bt_gatt_subscribe_params alt2_subscribe_params;
static struct bt_gatt_subscribe_params alt3_subscribe_params;
static struct bt_gatt_subscribe_params alt4_subscribe_params;
static struct bt_gatt_exchange_params mtu_exchange_params;

static struct k_work_delayable scan_timeout_work;
static struct k_work_delayable connect_timeout_work;
static struct k_work_delayable handshake_work;

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
    HANDSHAKE_SUBSCRIBE_INPUT,
    HANDSHAKE_DONE,
};

static enum handshake_step handshake_step;

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
};

static bool eir_parse_cb(struct bt_data *data, void *user_data) {
    struct eir_parse_ctx *ctx = user_data;

    if (data->type != BT_DATA_MANUFACTURER_DATA) {
        return true;
    }
    if (data->data_len < 2) {
        return true;
    }

    uint16_t company_id = sys_get_le16(data->data);
    if (company_id == JOYCON2_NINTENDO_COMPANY_ID) {
        ctx->found = true;
        return false;
    }
    return true;
}

static void start_handshake(void) {
    handshake_step = HANDSHAKE_LED;
    k_work_schedule(&handshake_work, K_MSEC(JOYCON2_HANDSHAKE_STEP_DELAY_MS));
}

static uint8_t input_notify_func(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
                                  const void *data, uint16_t length) {
    ARG_UNUSED(conn);

    if (!data) {
        LOG_INF("joycon2: input unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    /* Input reports may stream at 120Hz+; the HID-typing debug channel
     * types a few chars per report period at best, so print only the
     * first few packets and then a sparse heartbeat with the count. */
    static uint32_t input_count;
    input_count++;
    if (input_count <= 5 || (input_count % 256) == 0) {
        char prefix[24];
        snprintf(prefix, sizeof(prefix), "JC2 IN #%u", (unsigned)input_count);
        hex_encode_and_print(prefix, data, length);
    }
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

    hex_encode_and_print("JC2 ACK", data, length);
    return BT_GATT_ITER_CONTINUE;
}

#define JOYCON2_ALT_NOTIFY_FUNC(name, prefix)                                                     \
    static __maybe_unused uint8_t name(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, \
                         const void *data, uint16_t length) {                                      \
        ARG_UNUSED(conn);                                                                           \
        if (!data) {                                                                                \
            params->value_handle = 0;                                                               \
            return BT_GATT_ITER_STOP;                                                               \
        }                                                                                            \
        hex_encode_and_print(prefix, data, length);                                                 \
        return BT_GATT_ITER_CONTINUE;                                                               \
    }

JOYCON2_ALT_NOTIFY_FUNC(alt1_notify_func, "JC2 ALT1")
JOYCON2_ALT_NOTIFY_FUNC(alt2_notify_func, "JC2 ALT2")
JOYCON2_ALT_NOTIFY_FUNC(alt3_notify_func, "JC2 ALT3")
JOYCON2_ALT_NOTIFY_FUNC(alt4_notify_func, "JC2 ALT4")

/* Catch-all notification sniffer.
 *
 * Five of the service's characteristics carry the Notify property but no
 * CCC descriptor (per the nRF Connect service dump). A hand-rolled GATT
 * server can send notifications on such handles unconditionally -- and
 * both Android (setCharacteristicNotification local filter) and Zephyr
 * (bt_gatt_notification drops handles with no matching subscription)
 * would silently discard them. So the input stream may already be
 * flowing on a handle nobody has ever listened to.
 *
 * bt_gatt_resubscribe() registers a local subscription entry WITHOUT
 * writing any CCC (no on-air traffic), so covering every handle in the
 * service is free and makes any notification from any handle visible.
 */
#define JOYCON2_SNIFF_FIRST_HANDLE 0x0001
#define JOYCON2_SNIFF_LAST_HANDLE 0x0030
#define JOYCON2_SNIFF_COUNT (JOYCON2_SNIFF_LAST_HANDLE - JOYCON2_SNIFF_FIRST_HANDLE + 1)

static struct bt_gatt_subscribe_params sniff_params[JOYCON2_SNIFF_COUNT];
static uint32_t sniff_counts[JOYCON2_SNIFF_COUNT];

static uint8_t sniff_notify_func(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
                                  const void *data, uint16_t length) {
    ARG_UNUSED(conn);

    if (!data) {
        return BT_GATT_ITER_STOP;
    }

    uint16_t idx = params->value_handle - JOYCON2_SNIFF_FIRST_HANDLE;
    uint32_t count = ++sniff_counts[idx];
    if (count <= 3 || (count % 256) == 0) {
        char prefix[32];
        snprintf(prefix, sizeof(prefix), "JC2 N%04X #%u", params->value_handle,
                 (unsigned)count);
        hex_encode_and_print(prefix, data, length);
    }
    return BT_GATT_ITER_CONTINUE;
}

static void register_sniffer(struct bt_conn *conn) {
    const bt_addr_le_t *peer = bt_conn_get_dst(conn);
    int registered = 0;

    for (int i = 0; i < JOYCON2_SNIFF_COUNT; i++) {
        uint16_t handle = JOYCON2_SNIFF_FIRST_HANDLE + i;

        /* These two have real subscriptions with their own callbacks --
         * a sniffer entry would double-print every packet. */
        if (handle == JOYCON2_INPUT_VALUE_HANDLE || handle == JOYCON2_RESPONSE_VALUE_HANDLE) {
            continue;
        }

        sniff_counts[i] = 0;
        struct bt_gatt_subscribe_params *p = &sniff_params[i];
        p->value_handle = handle;
        /* Never used (no CCC write happens), but bt_gatt_resubscribe
         * asserts it is non-zero. */
        p->ccc_handle = 0xffff;
        p->value = BT_GATT_CCC_NOTIFY;
        p->notify = sniff_notify_func;
        int err = bt_gatt_resubscribe(0, peer, p);
        if (err == 0) {
            registered++;
        }
    }

    char msg[32];
    snprintf(msg, sizeof(msg), "JC2 SNIFF %d", registered);
    zmk_joycon2_debug_print(msg);
}

static int jc_write_command(const uint8_t *data, size_t len) {
    if (jc_conn == NULL) {
        return -ENOTCONN;
    }
    /* sign=false: this controller flatly rejects standard BLE bonding (no
     * CSRK is ever established), unlike ZMK's own split link which is
     * bonded and can use signed writes. */
    return bt_gatt_write_without_response(jc_conn, JOYCON2_COMMAND_VALUE_HANDLE, data, len, false);
}

static void subscribe_input(void);

static void handshake_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    int err;

    switch (handshake_step) {
    case HANDSHAKE_LED: {
        /* ledMask 0x01 = player-1 pattern. */
        static const uint8_t cmd[] = {0x09, 0x91, 0x01, 0x07, 0x00, 0x08, 0x00, 0x00,
                                       0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        err = jc_write_command(cmd, sizeof(cmd));
        LOG_INF("joycon2: set player LED (%d)", err);
        break;
    }
    case HANDSHAKE_PAIRING_VIBRATION: {
        /* "Play vibration preset 3" -- the connected/pairing buzz. */
        static const uint8_t cmd[] = {0x0A, 0x91, 0x01, 0x02, 0x00, 0x04,
                                       0x00, 0x00, 0x03, 0x00, 0x00, 0x00};
        err = jc_write_command(cmd, sizeof(cmd));
        LOG_INF("joycon2: pairing vibration (%d)", err);
        break;
    }
    case HANDSHAKE_IMU_1: {
        /* Feature-init, flags 0xFF = all features (0x0C/0x02). */
        static const uint8_t cmd[] = {0x0C, 0x91, 0x01, 0x02, 0x00, 0x04,
                                       0x00, 0x00, 0xFF, 0x00, 0x00, 0x00};
        err = jc_write_command(cmd, sizeof(cmd));
        LOG_INF("joycon2: feature init (%d)", err);
        break;
    }
    case HANDSHAKE_IMU_2: {
        /* Feature-enable, same flags (0x0C/0x04). */
        static const uint8_t cmd[] = {0x0C, 0x91, 0x01, 0x04, 0x00, 0x04,
                                       0x00, 0x00, 0xFF, 0x00, 0x00, 0x00};
        err = jc_write_command(cmd, sizeof(cmd));
        LOG_INF("joycon2: feature enable (%d)", err);
        break;
    }
    case HANDSHAKE_SUBSCRIBE_INPUT:
        subscribe_input();
        zmk_joycon2_debug_print("JC2 HANDSHAKE SENT");
        handshake_step = HANDSHAKE_DONE;
        return;
    default:
        return;
    }

    handshake_step++;
    k_work_schedule(&handshake_work, K_MSEC(JOYCON2_HANDSHAKE_STEP_DELAY_MS));
}

/* struct bt_gatt_subscribe_params.subscribe reports the CCC WRITE's actual
 * ATT-level result -- separate from .notify, which only ever fires for
 * real value notifications once subscribed. Never having set this means
 * a silently-failed CCCD write on one specific characteristic (while
 * others succeed) would look identical to "no data ever sent" -- exactly
 * the symptom seen (RESPONSE notifies fine, INPUT never does). */
static void subscribe_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_subscribe_params *params) {
    ARG_UNUSED(conn);

    const char *name = "?";
    if (params == &input_subscribe_params) {
        name = "IN";
    } else if (params == &response_subscribe_params) {
        name = "RESP";
    } else if (params == &alt1_subscribe_params) {
        name = "ALT1";
    } else if (params == &alt2_subscribe_params) {
        name = "ALT2";
    } else if (params == &alt3_subscribe_params) {
        name = "ALT3";
    } else if (params == &alt4_subscribe_params) {
        name = "ALT4";
    }

    char msg[48];
    snprintf(msg, sizeof(msg), "JC2 CCC %s err=%u", name, err);
    zmk_joycon2_debug_print(msg);
}

/* Deliberately called LAST, after feature-enable, matching the working
 * Linux implementation's ordering -- see the handshake_step comment. */
static void subscribe_input(void) {
    if (!jc_conn) {
        return;
    }

    input_subscribe_params.value_handle = JOYCON2_INPUT_VALUE_HANDLE;
    input_subscribe_params.ccc_handle = JOYCON2_INPUT_CCC_HANDLE;
    input_subscribe_params.notify = input_notify_func;
    input_subscribe_params.subscribe = subscribe_cb;
    input_subscribe_params.value = BT_GATT_CCC_NOTIFY;
    int err = bt_gatt_subscribe(jc_conn, &input_subscribe_params);
    if (err && err != -EALREADY) {
        LOG_ERR("joycon2: input subscribe failed (%d)", err);
        zmk_joycon2_debug_print("JC2 INPUT SUBSCRIBE FAILED");
    }
}

static void start_subscriptions(struct bt_conn *conn) {
    /* Explicit ccc_handle (rather than .disc_params) skips CCC
     * auto-discovery entirely -- that path also uses a READ_BY_TYPE-style
     * op internally and would likely hit the same wall as characteristic
     * discovery did. Only RESPONSE is subscribed here; INPUT is subscribed
     * as the handshake's final step. */
    int err;

    response_subscribe_params.value_handle = JOYCON2_RESPONSE_VALUE_HANDLE;
    response_subscribe_params.ccc_handle = JOYCON2_RESPONSE_CCC_HANDLE;
    response_subscribe_params.notify = response_notify_func;
    response_subscribe_params.subscribe = subscribe_cb;
    response_subscribe_params.value = BT_GATT_CCC_NOTIFY;
    err = bt_gatt_subscribe(conn, &response_subscribe_params);
    if (err && err != -EALREADY) {
        LOG_ERR("joycon2: response subscribe failed (%d)", err);
        zmk_joycon2_debug_print("JC2 RESPONSE SUBSCRIBE FAILED");
    }

    /* The reference implementation (JoyCon2Mac's BLEManager.mm) only ever
     * calls setNotifyValue on the input and response characteristics --
     * it never subscribes to these four "ALT" channels at all. Skipping
     * them here to match that exactly, in case subscribing to extras the
     * device doesn't expect is interfering with input notifications. */
#if 0
    alt1_subscribe_params.value_handle = JOYCON2_ALT1_VALUE_HANDLE;
    alt1_subscribe_params.ccc_handle = JOYCON2_ALT1_CCC_HANDLE;
    alt1_subscribe_params.notify = alt1_notify_func;
    alt1_subscribe_params.subscribe = subscribe_cb;
    alt1_subscribe_params.value = BT_GATT_CCC_NOTIFY;
    bt_gatt_subscribe(conn, &alt1_subscribe_params);

    alt2_subscribe_params.value_handle = JOYCON2_ALT2_VALUE_HANDLE;
    alt2_subscribe_params.ccc_handle = JOYCON2_ALT2_CCC_HANDLE;
    alt2_subscribe_params.notify = alt2_notify_func;
    alt2_subscribe_params.subscribe = subscribe_cb;
    alt2_subscribe_params.value = BT_GATT_CCC_NOTIFY;
    bt_gatt_subscribe(conn, &alt2_subscribe_params);

    alt3_subscribe_params.value_handle = JOYCON2_ALT3_VALUE_HANDLE;
    alt3_subscribe_params.ccc_handle = JOYCON2_ALT3_CCC_HANDLE;
    alt3_subscribe_params.notify = alt3_notify_func;
    alt3_subscribe_params.subscribe = subscribe_cb;
    alt3_subscribe_params.value = BT_GATT_CCC_NOTIFY;
    bt_gatt_subscribe(conn, &alt3_subscribe_params);

    alt4_subscribe_params.value_handle = JOYCON2_ALT4_VALUE_HANDLE;
    alt4_subscribe_params.ccc_handle = JOYCON2_ALT4_CCC_HANDLE;
    alt4_subscribe_params.notify = alt4_notify_func;
    alt4_subscribe_params.subscribe = subscribe_cb;
    alt4_subscribe_params.value = BT_GATT_CCC_NOTIFY;
    bt_gatt_subscribe(conn, &alt4_subscribe_params);
#endif

    zmk_joycon2_debug_print("JC2 SUBSCRIBED");
    /* start_handshake resets handshake_step -- without it a reconnect
     * would find the step still at HANDSHAKE_DONE and never handshake. */
    start_handshake();
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
                             struct bt_gatt_exchange_params *params) {
    ARG_UNUSED(params);

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
    } else {
        uint16_t mtu = bt_gatt_get_mtu(conn);
        LOG_INF("joycon2: MTU exchange succeeded, ATT MTU=%u", mtu);
        snprintf(msg, sizeof(msg), "JC2 MTU=%u", mtu);
    }
    zmk_joycon2_debug_print(msg);

    start_subscriptions(conn);
}

static void scan_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                        struct net_buf_simple *ad) {
    ARG_UNUSED(rssi);

    if (!jc_connecting || jc_conn != NULL) {
        return;
    }
    if (type != BT_GAP_ADV_TYPE_ADV_IND) {
        return;
    }

    struct eir_parse_ctx ctx = {.found = false};
    bt_data_parse(ad, eir_parse_cb, &ctx);
    if (!ctx.found) {
        return;
    }

    struct bt_conn *existing = bt_conn_lookup_addr_le(BT_ID_DEFAULT, addr);
    if (existing != NULL) {
        bt_conn_unref(existing);
        return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    LOG_INF("joycon2: found %s, connecting", addr_str);

    bt_le_scan_stop();
    k_work_cancel_delayable(&scan_timeout_work);

    /* 7.5-15ms: fast enough for a 120Hz+ input stream. The previous
     * 30-50ms request may have been too slow for the device to bother
     * streaming (matches Android's initial 7.5ms connection). */
    struct bt_le_conn_param *param = BT_LE_CONN_PARAM(0x0006, 0x000C, 0, 400);
    int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, param, &jc_conn);
    if (err) {
        LOG_ERR("joycon2: create conn failed (%d)", err);
        jc_connecting = false;
        zmk_joycon2_debug_print("JC2 CREATE CONN FAILED");
        return;
    }

    zmk_joycon2_debug_print("JC2 FOUND CONNECTING");
    /* Independent watchdog: don't rely solely on whatever internal timeout
     * bt_conn_le_create()'s default create params use -- if neither
     * jc_connected nor jc_disconnected ever fires, this guarantees we
     * eventually report *something* instead of hanging silently. */
    k_work_schedule(&connect_timeout_work, K_SECONDS(JOYCON2_CONNECT_TIMEOUT_SEC));
}

static void scan_timeout_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (!jc_connecting || jc_conn != NULL) {
        return;
    }

    bt_le_scan_stop();
    zmk_joycon2_debug_print("JC2 SCAN TIMEOUT NOT FOUND");
    jc_connecting = false;
}

static void connect_timeout_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (jc_conn == NULL) {
        return;
    }

    LOG_ERR("joycon2: connect attempt timed out");
    bt_conn_disconnect(jc_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    bt_conn_unref(jc_conn);
    jc_conn = NULL;
    jc_connecting = false;
    zmk_joycon2_debug_print("JC2 CONNECT TIMEOUT");
}

static void jc_connected(struct bt_conn *conn, uint8_t err) {
    if (conn != jc_conn) {
        return;
    }

    k_work_cancel_delayable(&connect_timeout_work);

    if (err) {
        LOG_ERR("joycon2: connect failed (err %d)", err);
        bt_conn_unref(jc_conn);
        jc_conn = NULL;
        jc_connecting = false;
        zmk_joycon2_debug_print("JC2 CONNECT FAILED");
        return;
    }

    LOG_INF("joycon2: connected, subscribing");
    zmk_joycon2_debug_print("JC2 CONNECTED");
    jc_connecting = false;

    /* Purely local (no on-air traffic) -- catch notifications from ANY
     * handle in the service, registered before anything else so even
     * immediate post-connect traffic is visible. */
    register_sniffer(conn);

    /* Not required before subscribing works, but the default 23-byte ATT
     * MTU would truncate longer input-report notifications later, and no
     * known-working client against this device skips it. Subscriptions
     * start from mtu_exchange_cb once this actually completes. */
    mtu_exchange_params.func = mtu_exchange_cb;
    int mtu_err = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
    if (mtu_err) {
        LOG_ERR("joycon2: MTU exchange request failed (%d)", mtu_err);
        /* No callback will ever fire in this case -- proceed directly. */
        start_subscriptions(conn);
    }
}

static void jc_disconnected(struct bt_conn *conn, uint8_t reason) {
    if (conn != jc_conn) {
        return;
    }

    LOG_INF("joycon2: disconnected (reason %d)", reason);
    k_work_cancel_delayable(&connect_timeout_work);
    k_work_cancel_delayable(&handshake_work);
    bt_conn_unref(jc_conn);
    jc_conn = NULL;
    jc_connecting = false;

    char msg[48];
    snprintf(msg, sizeof(msg), "JC2 DISCONNECTED reason=0x%02x", reason);
    zmk_joycon2_debug_print(msg);
}

static void jc_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
                                 uint16_t timeout) {
    if (conn != jc_conn) {
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
    if (conn != jc_conn) {
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
    if (jc_connecting) {
        return -EBUSY;
    }

    if (jc_conn != NULL) {
        bt_conn_disconnect(jc_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        bt_conn_unref(jc_conn);
        jc_conn = NULL;
    }

    jc_connecting = true;

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
        jc_connecting = false;
        zmk_joycon2_debug_print("JC2 SCAN START FAILED");
        return err;
    }

    k_work_schedule(&scan_timeout_work, K_SECONDS(JOYCON2_SCAN_TIMEOUT_SEC));
    return 0;
}

static int joycon2_connection_init(void) {
    k_work_init_delayable(&scan_timeout_work, scan_timeout_work_handler);
    k_work_init_delayable(&connect_timeout_work, connect_timeout_work_handler);
    k_work_init_delayable(&handshake_work, handshake_work_handler);
    bt_conn_cb_register(&jc_conn_callbacks);
    return 0;
}

SYS_INIT(joycon2_connection_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
