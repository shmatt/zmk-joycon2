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

/* param1 selects the action:
 *   0 -- toggle per-input-report (button) logging
 *   1 -- type a fixed self-test string
 *
 * 0 used to type the self-test string, which was the point during Stage 1;
 * now that the decoder is proven, the useful thing to have on a combo is
 * the logging toggle, since logging types into the focused window and has
 * to be off to actually use the gamepad. */
#define JOYCON2_DEBUG_ACTION_TOGGLE_INPUT_LOG 0
#define JOYCON2_DEBUG_ACTION_SELF_TEST 1

static int behavior_joycon2_debug_print_init(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);

    switch (binding->param1) {
    case JOYCON2_DEBUG_ACTION_TOGGLE_INPUT_LOG:
        zmk_joycon2_debug_logging_toggle();
        return 0;
    case JOYCON2_DEBUG_ACTION_SELF_TEST:
        return zmk_joycon2_debug_print("JOYCON2 DEBUG PRINT OK 12345");
    default:
        LOG_WRN("no debug_print action %d", binding->param1);
        return -ENOTSUP;
    }
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
