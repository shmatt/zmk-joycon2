/*
 * Copyright (c) 2026 The zmk-joycon2 Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_joycon2_debug_print

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>

#include <zmk/joycon2/debug_print.h>

LOG_MODULE_DECLARE(joycon2_debug_print, CONFIG_ZMK_LOG_LEVEL);

/* param1 selects which canned message to type; add more as later sub-stages need them. */
static const char *const joycon2_debug_print_messages[] = {
    "JOYCON2 DEBUG PRINT OK 12345",
};

static int behavior_joycon2_debug_print_init(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);

    if (binding->param1 >= ARRAY_SIZE(joycon2_debug_print_messages)) {
        LOG_WRN("no debug_print message at index %d", binding->param1);
        return -ENOTSUP;
    }

    return zmk_joycon2_debug_print(joycon2_debug_print_messages[binding->param1]);
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                       struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return 0;
}

static const struct behavior_driver_api behavior_joycon2_debug_print_driver_api = {
    .binding_pressed = on_keymap_binding_pressed, .binding_released = on_keymap_binding_released};

#define JOYCON2_DEBUG_PRINT_INST(n)                                                               \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_joycon2_debug_print_init, NULL, NULL, NULL, POST_KERNEL,  \
                             CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                 \
                             &behavior_joycon2_debug_print_driver_api);

DT_INST_FOREACH_STATUS_OKAY(JOYCON2_DEBUG_PRINT_INST)
