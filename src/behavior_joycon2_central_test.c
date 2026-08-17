/*
 * Copyright (c) 2026 The zmk-joycon2 Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_joycon2_central_test

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>

#include <zmk/joycon2/central_test.h>

LOG_MODULE_DECLARE(joycon2_central_test, CONFIG_ZMK_LOG_LEVEL);

static int behavior_joycon2_central_test_init(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return zmk_joycon2_central_test_start();
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                       struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return 0;
}

static const struct behavior_driver_api behavior_joycon2_central_test_driver_api = {
    .binding_pressed = on_keymap_binding_pressed, .binding_released = on_keymap_binding_released};

#define JOYCON2_CENTRAL_TEST_INST(n)                                                              \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_joycon2_central_test_init, NULL, NULL, NULL, POST_KERNEL, \
                             CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                  \
                             &behavior_joycon2_central_test_driver_api);

DT_INST_FOREACH_STATUS_OKAY(JOYCON2_CENTRAL_TEST_INST)
