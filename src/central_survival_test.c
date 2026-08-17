/*
 * Copyright (c) 2026 The zmk-joycon2 Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>
#include <stdio.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>

#include <zmk/joycon2/central_test.h>
#include <zmk/joycon2/debug_print.h>

LOG_MODULE_REGISTER(joycon2_central_test, CONFIG_ZMK_LOG_LEVEL);

#define JOYCON2_CENTRAL_TEST_TIMEOUT_SEC 15
#define JOYCON2_CENTRAL_TEST_STILL_ALIVE_DELAY_SEC 3

static struct bt_conn *test_conn;
static bool test_in_progress;

static struct k_work_delayable test_timeout_work;
static struct k_work_delayable test_still_alive_work;

struct conn_count_ctx {
    int total;
    int central;
    int peripheral;
};

static void count_conn_cb(struct bt_conn *conn, void *data) {
    struct conn_count_ctx *ctx = data;
    struct bt_conn_info info;

    if (bt_conn_get_info(conn, &info) != 0) {
        return;
    }

    ctx->total++;
    if (info.role == BT_CONN_ROLE_CENTRAL) {
        ctx->central++;
    } else if (info.role == BT_CONN_ROLE_PERIPHERAL) {
        ctx->peripheral++;
    }
}

/* Builds "total=%d central=%d periph=%d" into msg, appended after whatever
 * prefix the caller already wrote (via snprintf's return value). */
static void append_conn_counts(char *msg, size_t msg_size, size_t prefix_len) {
    struct conn_count_ctx ctx = {0};
    bt_conn_foreach(BT_CONN_TYPE_LE, count_conn_cb, &ctx);

    if (prefix_len >= msg_size) {
        return;
    }
    snprintf(msg + prefix_len, msg_size - prefix_len, "total=%d central=%d periph=%d", ctx.total,
             ctx.central, ctx.peripheral);
}

static void test_still_alive_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (test_conn == NULL) {
        /* already disconnected -- test_disconnected() already reported this */
        return;
    }

    char msg[96];
    int n = snprintf(msg, sizeof(msg), "BLE3 STILL CONNECTED ");
    append_conn_counts(msg, sizeof(msg), n);
    zmk_joycon2_debug_print(msg);

    test_in_progress = false;
}

static void scan_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                        struct net_buf_simple *ad) {
    ARG_UNUSED(rssi);
    ARG_UNUSED(ad);

    if (!test_in_progress || test_conn != NULL) {
        return;
    }

    /* Only connectable advertisements can actually be connected to. */
    if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    LOG_INF("central test: found %s, connecting", addr_str);

    bt_le_scan_stop();

    struct bt_le_conn_param *param = BT_LE_CONN_PARAM(0x0018, 0x0028, 0, 400);
    int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, param, &test_conn);
    if (err) {
        LOG_ERR("central test: create conn failed (%d)", err);
        k_work_cancel_delayable(&test_timeout_work);

        char msg[48];
        snprintf(msg, sizeof(msg), "BLE3 CREATE CONN FAILED err=%d", err);
        zmk_joycon2_debug_print(msg);
        test_in_progress = false;
    }
}

static void test_connected(struct bt_conn *conn, uint8_t err) {
    if (conn != test_conn) {
        return;
    }

    k_work_cancel_delayable(&test_timeout_work);

    if (err) {
        LOG_ERR("central test: connect failed (err %d)", err);
        bt_conn_unref(test_conn);
        test_conn = NULL;

        char msg[48];
        snprintf(msg, sizeof(msg), "BLE3 CONNECT FAILED err=%d", err);
        zmk_joycon2_debug_print(msg);
        test_in_progress = false;
        return;
    }

    LOG_INF("central test: connected");

    /* Report immediately -- this is the instant all three roles are
     * provably alive at once, before the peer (or anything else) gets a
     * chance to tear the new link back down. */
    char msg[96];
    int n = snprintf(msg, sizeof(msg), "BLE3 CONNECTED ");
    append_conn_counts(msg, sizeof(msg), n);
    zmk_joycon2_debug_print(msg);

    k_work_reschedule(&test_still_alive_work, K_SECONDS(JOYCON2_CENTRAL_TEST_STILL_ALIVE_DELAY_SEC));
}

static void test_disconnected(struct bt_conn *conn, uint8_t reason) {
    if (conn != test_conn) {
        return;
    }

    LOG_INF("central test: disconnected (reason %d)", reason);
    bt_conn_unref(test_conn);
    test_conn = NULL;

    k_work_cancel_delayable(&test_still_alive_work);

    char msg[64];
    snprintf(msg, sizeof(msg), "BLE3 TEST-DEV DISCONNECTED reason=0x%02x", reason);
    zmk_joycon2_debug_print(msg);
    test_in_progress = false;
}

static struct bt_conn_cb test_conn_callbacks = {
    .connected = test_connected,
    .disconnected = test_disconnected,
};

static void test_timeout_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (!test_in_progress || test_conn != NULL) {
        /* a connect attempt is already in flight; let it resolve on its own */
        return;
    }

    bt_le_scan_stop();
    zmk_joycon2_debug_print("BLE3 TEST TIMEOUT NO DEVICE FOUND");
    test_in_progress = false;
}

int zmk_joycon2_central_test_start(void) {
    if (test_in_progress) {
        return -EBUSY;
    }

    if (test_conn != NULL) {
        /* Previous test connection never got torn down (e.g. re-triggered
         * before the peer disconnected) -- release it properly so repeated
         * presses can't leak connection slots. */
        k_work_cancel_delayable(&test_still_alive_work);
        bt_conn_disconnect(test_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        bt_conn_unref(test_conn);
        test_conn = NULL;
    }

    test_in_progress = true;

    int err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, scan_found);
    if (err) {
        LOG_ERR("central test: scan start failed (%d)", err);
        test_in_progress = false;

        char msg[48];
        snprintf(msg, sizeof(msg), "BLE3 TEST SCAN START FAILED %d", err);
        zmk_joycon2_debug_print(msg);
        return err;
    }

    k_work_schedule(&test_timeout_work, K_SECONDS(JOYCON2_CENTRAL_TEST_TIMEOUT_SEC));
    return 0;
}

static int joycon2_central_test_init(void) {
    k_work_init_delayable(&test_timeout_work, test_timeout_work_handler);
    k_work_init_delayable(&test_still_alive_work, test_still_alive_work_handler);
    bt_conn_cb_register(&test_conn_callbacks);
    return 0;
}

SYS_INIT(joycon2_central_test_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
