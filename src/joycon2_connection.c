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
#include <zephyr/bluetooth/uuid.h>

#include <zmk/joycon2/connection.h>
#include <zmk/joycon2/debug_print.h>

LOG_MODULE_REGISTER(joycon2_connection, CONFIG_ZMK_LOG_LEVEL);

/* GATT layout confirmed by hand via nRF Connect against a real Joy-Con 2,
 * cross-checked against github.com/OZORDI/JoyCon2Mac's BLEManager.mm (which
 * uses the identical UUIDs). These are only visible via discovery *after*
 * connecting -- the device's advertisement carries no service UUID at all,
 * so the scan filter below matches on manufacturer data instead (see
 * JOYCON2_NINTENDO_COMPANY_ID below). */
#define JOYCON2_UUID_SERVICE_VAL BT_UUID_128_ENCODE(0xab7de9be, 0x89fe, 0x49ad, 0x828f, 0x118f09df7fd0)
#define JOYCON2_UUID_INPUT_VAL BT_UUID_128_ENCODE(0xab7de9be, 0x89fe, 0x49ad, 0x828f, 0x118f09df7fd2)
#define JOYCON2_UUID_COMMAND_VAL BT_UUID_128_ENCODE(0x649d4ac9, 0x8eb7, 0x4e6c, 0xaf44, 0x1ea54fe5f005)
#define JOYCON2_UUID_RESPONSE_VAL BT_UUID_128_ENCODE(0xc765a961, 0xd9d8, 0x4d36, 0xa20a, 0x5315b111836a)

static const struct bt_uuid_128 joycon2_uuid_service = BT_UUID_INIT_128(JOYCON2_UUID_SERVICE_VAL);
static const struct bt_uuid_128 joycon2_uuid_input = BT_UUID_INIT_128(JOYCON2_UUID_INPUT_VAL);
static const struct bt_uuid_128 joycon2_uuid_command = BT_UUID_INIT_128(JOYCON2_UUID_COMMAND_VAL);
static const struct bt_uuid_128 joycon2_uuid_response = BT_UUID_INIT_128(JOYCON2_UUID_RESPONSE_VAL);

#define JOYCON2_SCAN_TIMEOUT_SEC 20
#define JOYCON2_CONNECT_TIMEOUT_SEC 15
#define JOYCON2_DISCOVER_TIMEOUT_SEC 10
#define JOYCON2_HANDSHAKE_STEP_DELAY_MS 500

static struct bt_conn *jc_conn;
static bool jc_connecting;
static uint16_t command_value_handle;

static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_discover_params input_ccc_disc_params;
static struct bt_gatt_discover_params response_ccc_disc_params;
static struct bt_gatt_subscribe_params input_subscribe_params;
static struct bt_gatt_subscribe_params response_subscribe_params;
static struct bt_gatt_exchange_params mtu_exchange_params;

static struct k_work_delayable scan_timeout_work;
static struct k_work_delayable connect_timeout_work;
static struct k_work_delayable discover_timeout_work;
static struct k_work_delayable handshake_work;

enum handshake_step {
    HANDSHAKE_IMU_1,
    HANDSHAKE_IMU_2,
    HANDSHAKE_MAC_1,
    HANDSHAKE_MAC_2,
    HANDSHAKE_MAC_3,
    HANDSHAKE_MAC_4,
    HANDSHAKE_DONE,
};

static enum handshake_step handshake_step;

static void hex_encode_and_print(const char *prefix, const uint8_t *data, uint16_t length) {
    char msg[128];
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

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
                             struct bt_gatt_exchange_params *params) {
    ARG_UNUSED(params);

    if (err) {
        LOG_ERR("joycon2: MTU exchange failed (err %u)", err);
    } else {
        LOG_INF("joycon2: MTU exchange succeeded, ATT MTU=%u", bt_gatt_get_mtu(conn));
    }
}

static void start_handshake(void) {
    handshake_step = HANDSHAKE_IMU_1;
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

    hex_encode_and_print("JC2 IN", data, length);
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

static uint8_t chrc_discovery_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    struct bt_gatt_discover_params *params) {
    ARG_UNUSED(conn);

    if (!attr) {
        LOG_INF("joycon2: characteristic discovery complete");
        k_work_cancel_delayable(&discover_timeout_work);
        memset(params, 0, sizeof(*params));

        char msg[64];
        snprintf(msg, sizeof(msg), "JC2 CHRC in=%u cmd=%u resp=%u",
                 input_subscribe_params.value_handle, command_value_handle,
                 response_subscribe_params.value_handle);
        zmk_joycon2_debug_print(msg);

        if (command_value_handle) {
            start_handshake();
        }
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = attr->user_data;

    if (bt_uuid_cmp(chrc->uuid, &joycon2_uuid_input.uuid) == 0) {
        input_subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);
        input_subscribe_params.notify = input_notify_func;
        input_subscribe_params.value = BT_GATT_CCC_NOTIFY;
        input_subscribe_params.disc_params = &input_ccc_disc_params;
        input_subscribe_params.end_handle = discover_params.end_handle;
        int err = bt_gatt_subscribe(conn, &input_subscribe_params);
        if (err && err != -EALREADY) {
            LOG_ERR("joycon2: input subscribe failed (%d)", err);
        }
    } else if (bt_uuid_cmp(chrc->uuid, &joycon2_uuid_command.uuid) == 0) {
        command_value_handle = bt_gatt_attr_value_handle(attr);
    } else if (bt_uuid_cmp(chrc->uuid, &joycon2_uuid_response.uuid) == 0) {
        response_subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);
        response_subscribe_params.notify = response_notify_func;
        response_subscribe_params.value = BT_GATT_CCC_NOTIFY;
        response_subscribe_params.disc_params = &response_ccc_disc_params;
        response_subscribe_params.end_handle = discover_params.end_handle;
        int err = bt_gatt_subscribe(conn, &response_subscribe_params);
        if (err && err != -EALREADY) {
            LOG_ERR("joycon2: response subscribe failed (%d)", err);
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t service_discovery_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                       struct bt_gatt_discover_params *params) {
    ARG_UNUSED(params);

    if (!attr) {
        LOG_ERR("joycon2: service not found");
        zmk_joycon2_debug_print("JC2 SERVICE NOT FOUND");
        memset(&discover_params, 0, sizeof(discover_params));
        return BT_GATT_ITER_STOP;
    }

    /* Reverting to a targeted UUID-filtered lookup (ATT_FIND_BY_TYPE_VALUE)
     * -- this reliably worked twice; switching to "discover all services"
     * (ATT_READ_BY_GROUP_TYPE) made things WORSE (this call never got a
     * response at all). Combined with characteristic discovery
     * (ATT_READ_BY_TYPE) also never getting a response, the pattern is:
     * this device answers FIND_BY_TYPE_VALUE reliably but not
     * READ_BY_TYPE/READ_BY_GROUP_TYPE -- yet Android's nRF Connect *did*
     * fully enumerate this same device using (presumably) the same
     * standard ATT operations. The one concrete difference spotted in that
     * working session's own connection log: Android negotiated a much
     * faster interval (7.5ms) than we request below -- worth testing
     * whether this device's firmware has tight timing margins for
     * preparing enumeration-style responses. */
    k_work_cancel_delayable(&discover_timeout_work);

    struct bt_gatt_service_val *service_val = attr->user_data;
    uint16_t chrc_start_handle = attr->handle + 1;
    uint16_t chrc_end_handle = service_val->end_handle;

    LOG_INF("joycon2: service found, discovering characteristics");
    zmk_joycon2_debug_print("JC2 SERVICE FOUND");

    memset(&discover_params, 0, sizeof(discover_params));
    discover_params.uuid = NULL;
    discover_params.func = chrc_discovery_func;
    discover_params.start_handle = chrc_start_handle;
    discover_params.end_handle = chrc_end_handle;
    discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(conn, &discover_params);
    if (err) {
        LOG_ERR("joycon2: chrc discover failed (%d)", err);
        zmk_joycon2_debug_print("JC2 CHRC DISCOVER FAILED");
        return BT_GATT_ITER_STOP;
    }

    k_work_schedule(&discover_timeout_work, K_SECONDS(JOYCON2_DISCOVER_TIMEOUT_SEC));
    return BT_GATT_ITER_STOP;
}

static int jc_write_command(const uint8_t *data, size_t len) {
    if (jc_conn == NULL || command_value_handle == 0) {
        return -ENOTCONN;
    }
    /* sign=false: this controller flatly rejects standard BLE bonding (no
     * CSRK is ever established), unlike ZMK's own split link which is
     * bonded and can use signed writes. */
    return bt_gatt_write_without_response(jc_conn, command_value_handle, data, len, false);
}

static void handshake_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    static uint8_t mac1[6];
    static uint8_t mac2[6];
    uint8_t buf[22];
    int err;

    switch (handshake_step) {
    case HANDSHAKE_IMU_1: {
        static const uint8_t cmd[] = {0x0C, 0x91, 0x01, 0x02, 0x00, 0x04,
                                       0x00, 0x00, 0xFF, 0x00, 0x00, 0x00};
        err = jc_write_command(cmd, sizeof(cmd));
        LOG_INF("joycon2: IMU enable step1 (%d)", err);
        break;
    }
    case HANDSHAKE_IMU_2: {
        static const uint8_t cmd[] = {0x0C, 0x91, 0x01, 0x04, 0x00, 0x04,
                                       0x00, 0x00, 0xFF, 0x00, 0x00, 0x00};
        err = jc_write_command(cmd, sizeof(cmd));
        LOG_INF("joycon2: IMU enable step2 (%d)", err);
        break;
    }
    case HANDSHAKE_MAC_1: {
        bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
        size_t count = ARRAY_SIZE(addrs);

        bt_id_get(addrs, &count);
        if (count == 0) {
            LOG_ERR("joycon2: no local BT identity address");
            zmk_joycon2_debug_print("JC2 NO LOCAL BT ADDR");
            handshake_step = HANDSHAKE_DONE;
            return;
        }

        /* MAC-binding step 1 wants the host's own address, byte-reversed;
         * MAC2 is the same reversed address with its first byte
         * decremented by one (the "no second address supplied" fallback
         * JoyCon2Mac itself always hits in practice). */
        for (int i = 0; i < 6; i++) {
            mac1[i] = addrs[0].a.val[5 - i];
        }
        memcpy(mac2, mac1, sizeof(mac2));
        mac2[0] = mac1[0] - 1;

        buf[0] = 0x15;
        buf[1] = 0x91;
        buf[2] = 0x01;
        buf[3] = 0x01;
        buf[4] = 0x00;
        buf[5] = 0x0E;
        buf[6] = 0x00;
        buf[7] = 0x00;
        buf[8] = 0x00;
        buf[9] = 0x02;
        memcpy(&buf[10], mac1, sizeof(mac1));
        memcpy(&buf[16], mac2, sizeof(mac2));

        err = jc_write_command(buf, sizeof(buf));
        LOG_INF("joycon2: MAC-bind step1 (%d)", err);
        break;
    }
    case HANDSHAKE_MAC_2: {
        static const uint8_t cmd[] = {0x15, 0x91, 0x01, 0x04, 0x00, 0x11, 0x00, 0x00, 0x00,
                                       0x08, 0x06, 0x5A, 0x60, 0xE9, 0x02, 0xE4, 0xE1, 0x02,
                                       0x02, 0x9E, 0x3F, 0xA3, 0x9A, 0x78, 0xD1};
        err = jc_write_command(cmd, sizeof(cmd));
        LOG_INF("joycon2: MAC-bind step2 (%d)", err);
        break;
    }
    case HANDSHAKE_MAC_3: {
        static const uint8_t cmd[] = {0x15, 0x91, 0x01, 0x02, 0x00, 0x11, 0x00, 0x00, 0x00,
                                       0x93, 0x4E, 0x58, 0x0F, 0x16, 0x3A, 0xEE, 0xCF, 0xB5,
                                       0x75, 0xFC, 0x91, 0x36, 0xB2, 0x2F, 0xBB};
        err = jc_write_command(cmd, sizeof(cmd));
        LOG_INF("joycon2: MAC-bind step3 (%d)", err);
        break;
    }
    case HANDSHAKE_MAC_4: {
        static const uint8_t cmd[] = {0x15, 0x91, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00};
        err = jc_write_command(cmd, sizeof(cmd));
        LOG_INF("joycon2: MAC-bind step4 (%d)", err);
        zmk_joycon2_debug_print("JC2 HANDSHAKE SENT");
        handshake_step = HANDSHAKE_DONE;
        return;
    }
    default:
        return;
    }

    handshake_step++;
    k_work_schedule(&handshake_work, K_MSEC(JOYCON2_HANDSHAKE_STEP_DELAY_MS));
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

    /* 0x0006 = 7.5ms, the fastest interval BLE allows -- matches what
     * Android's own BLE stack (nRF Connect) negotiated with this exact
     * device in a session where full GATT enumeration worked. Previously
     * requested 30-50ms; testing whether this device's firmware has tight
     * timing margins for preparing enumeration-style responses. Timeout
     * 500*10ms=5000ms also matches Android's observed supervision timeout. */
    struct bt_le_conn_param *param = BT_LE_CONN_PARAM(0x0006, 0x0006, 0, 500);
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

static void discover_timeout_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (jc_conn == NULL) {
        return;
    }

    LOG_ERR("joycon2: discovery timed out");
    memset(&discover_params, 0, sizeof(discover_params));
    zmk_joycon2_debug_print("JC2 DISCOVER TIMEOUT");
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

    LOG_INF("joycon2: connected, discovering service");
    zmk_joycon2_debug_print("JC2 CONNECTED");

    /* Fire-and-forget: not required before discovery works, but the default
     * 23-byte ATT MTU would truncate longer input-report notifications
     * later, and no known-working client against this device skips it. */
    mtu_exchange_params.func = mtu_exchange_cb;
    int mtu_err = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
    if (mtu_err) {
        LOG_ERR("joycon2: MTU exchange request failed (%d)", mtu_err);
    }

    memset(&discover_params, 0, sizeof(discover_params));
    discover_params.uuid = &joycon2_uuid_service.uuid;
    discover_params.func = service_discovery_func;
    discover_params.start_handle = 0x0001;
    discover_params.end_handle = 0xffff;
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    int rc = bt_gatt_discover(conn, &discover_params);
    if (rc) {
        LOG_ERR("joycon2: discover failed (%d)", rc);
        zmk_joycon2_debug_print("JC2 DISCOVER FAILED");
        return;
    }

    k_work_schedule(&discover_timeout_work, K_SECONDS(JOYCON2_DISCOVER_TIMEOUT_SEC));
}

static void jc_disconnected(struct bt_conn *conn, uint8_t reason) {
    if (conn != jc_conn) {
        return;
    }

    LOG_INF("joycon2: disconnected (reason %d)", reason);
    k_work_cancel_delayable(&connect_timeout_work);
    k_work_cancel_delayable(&discover_timeout_work);
    bt_conn_unref(jc_conn);
    jc_conn = NULL;
    jc_connecting = false;
    command_value_handle = 0;
    k_work_cancel_delayable(&handshake_work);

    char msg[48];
    snprintf(msg, sizeof(msg), "JC2 DISCONNECTED reason=0x%02x", reason);
    zmk_joycon2_debug_print(msg);
}

static struct bt_conn_cb jc_conn_callbacks = {
    .connected = jc_connected,
    .disconnected = jc_disconnected,
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

    command_value_handle = 0;
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
    k_work_init_delayable(&discover_timeout_work, discover_timeout_work_handler);
    k_work_init_delayable(&handshake_work, handshake_work_handler);
    bt_conn_cb_register(&jc_conn_callbacks);
    return 0;
}

SYS_INIT(joycon2_connection_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
