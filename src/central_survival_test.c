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

#include <zmk/joycon2/central_test.h>
#include <zmk/joycon2/debug_print.h>

LOG_MODULE_REGISTER(joycon2_central_test, CONFIG_ZMK_LOG_LEVEL);

#define JOYCON2_CENTRAL_TEST_TIMEOUT_SEC 15
#define JOYCON2_CENTRAL_TEST_REPORT_DELAY_SEC 2

static struct bt_conn *test_conn;
static bool test_in_progress;
static bool test_connect_ok;

static struct k_work_delayable test_timeout_work;
static struct k_work_delayable test_report_work;

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

static void test_report_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    struct conn_count_ctx ctx = {0};
    bt_conn_foreach(BT_CONN_TYPE_LE, count_conn_cb, &ctx);

    char msg[96];
    snprintf(msg, sizeof(msg), "BLE3 TEST connect=%s total=%d central=%d periph=%d",
             test_connect_ok ? "OK" : "FAIL", ctx.total, ctx.central, ctx.peripheral);
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
        test_connect_ok = false;
        k_work_cancel_delayable(&test_timeout_work);
        k_work_reschedule(&test_report_work, K_NO_WAIT);
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
        test_connect_ok = false;
    } else {
        LOG_INF("central test: connected");
        test_connect_ok = true;
    }

    k_work_reschedule(&test_report_work, K_SECONDS(JOYCON2_CENTRAL_TEST_REPORT_DELAY_SEC));
}

static void test_disconnected(struct bt_conn *conn, uint8_t reason) {
    if (conn != test_conn) {
        return;
    }

    LOG_INF("central test: disconnected (reason %d)", reason);
    bt_conn_unref(test_conn);
    test_conn = NULL;
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

    test_in_progress = true;
    test_connect_ok = false;
    test_conn = NULL;

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
    k_work_init_delayable(&test_report_work, test_report_work_handler);
    bt_conn_cb_register(&test_conn_callbacks);
    return 0;
}

SYS_INIT(joycon2_central_test_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
