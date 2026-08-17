/*
 * Copyright (c) 2026 The zmk-joycon2 Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>
#include <string.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <dt-bindings/zmk/hid_usage_pages.h>
#include <dt-bindings/zmk/modifiers.h>

#include <zmk/joycon2/debug_print.h>

LOG_MODULE_REGISTER(joycon2_debug_print, CONFIG_ZMK_LOG_LEVEL);

#define JOYCON2_DEBUG_PRINT_BUF_SIZE 256
#define JOYCON2_DEBUG_PRINT_STEP_DELAY_MS 8

/*
 * ASCII -> USB HID Usage Tables 1.12, page 0x07 (Keyboard/Keypad), US layout.
 * Values are the raw (unshifted-page) usage ids zmk_hid_keyboard_press/release
 * expect; letters and digits are derived, everything else is a fixed table so
 * there's no dependency on this ZMK branch's exact keycode macro names.
 */
static bool ascii_to_hid(char c, uint8_t *usage, bool *shift) {
    *shift = false;

    if (c >= 'a' && c <= 'z') {
        *usage = 0x04 + (uint8_t)(c - 'a');
        return true;
    }
    if (c >= 'A' && c <= 'Z') {
        *usage = 0x04 + (uint8_t)(c - 'A');
        *shift = true;
        return true;
    }

    switch (c) {
    case '1': *usage = 0x1E; return true;
    case '!': *usage = 0x1E; *shift = true; return true;
    case '2': *usage = 0x1F; return true;
    case '@': *usage = 0x1F; *shift = true; return true;
    case '3': *usage = 0x20; return true;
    case '#': *usage = 0x20; *shift = true; return true;
    case '4': *usage = 0x21; return true;
    case '$': *usage = 0x21; *shift = true; return true;
    case '5': *usage = 0x22; return true;
    case '%': *usage = 0x22; *shift = true; return true;
    case '6': *usage = 0x23; return true;
    case '^': *usage = 0x23; *shift = true; return true;
    case '7': *usage = 0x24; return true;
    case '&': *usage = 0x24; *shift = true; return true;
    case '8': *usage = 0x25; return true;
    case '*': *usage = 0x25; *shift = true; return true;
    case '9': *usage = 0x26; return true;
    case '(': *usage = 0x26; *shift = true; return true;
    case '0': *usage = 0x27; return true;
    case ')': *usage = 0x27; *shift = true; return true;
    case ' ': *usage = 0x2C; return true;
    case '-': *usage = 0x2D; return true;
    case '_': *usage = 0x2D; *shift = true; return true;
    case '=': *usage = 0x2E; return true;
    case '+': *usage = 0x2E; *shift = true; return true;
    case '[': *usage = 0x2F; return true;
    case '{': *usage = 0x2F; *shift = true; return true;
    case ']': *usage = 0x30; return true;
    case '}': *usage = 0x30; *shift = true; return true;
    case '\\': *usage = 0x31; return true;
    case '|': *usage = 0x31; *shift = true; return true;
    case ';': *usage = 0x33; return true;
    case ':': *usage = 0x33; *shift = true; return true;
    case '\'': *usage = 0x34; return true;
    case '"': *usage = 0x34; *shift = true; return true;
    case '`': *usage = 0x35; return true;
    case '~': *usage = 0x35; *shift = true; return true;
    case ',': *usage = 0x36; return true;
    case '<': *usage = 0x36; *shift = true; return true;
    case '.': *usage = 0x37; return true;
    case '>': *usage = 0x37; *shift = true; return true;
    case '/': *usage = 0x38; return true;
    case '?': *usage = 0x38; *shift = true; return true;
    default:
        return false;
    }
}

enum joycon2_debug_print_phase {
    JOYCON2_DEBUG_PRINT_PHASE_PRESS,
    JOYCON2_DEBUG_PRINT_PHASE_RELEASE,
};

static char print_buf[JOYCON2_DEBUG_PRINT_BUF_SIZE];
static size_t print_len;
static size_t print_pos;
static bool print_busy;

static uint8_t cur_usage;
static bool cur_shift;
static enum joycon2_debug_print_phase print_phase;

static struct k_work_delayable print_work;

static void joycon2_debug_print_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (print_phase == JOYCON2_DEBUG_PRINT_PHASE_PRESS) {
        if (!ascii_to_hid(print_buf[print_pos], &cur_usage, &cur_shift)) {
            /* unsupported character prints as '?' */
            cur_usage = 0x38;
            cur_shift = true;
        }

        if (cur_shift) {
            /* zmk_hid_register_mod() takes a bit *index* (0-7), not the
             * pre-shifted MOD_* bitmask -- zmk_hid_register_mods() is the
             * bitmask-taking variant and does the index conversion itself. */
            zmk_hid_register_mods(MOD_LSFT);
        }
        zmk_hid_keyboard_press(cur_usage);
        zmk_endpoints_send_report(HID_USAGE_KEY);

        print_phase = JOYCON2_DEBUG_PRINT_PHASE_RELEASE;
        k_work_schedule(&print_work, K_MSEC(JOYCON2_DEBUG_PRINT_STEP_DELAY_MS));
        return;
    }

    zmk_hid_keyboard_release(cur_usage);
    if (cur_shift) {
        zmk_hid_unregister_mods(MOD_LSFT);
    }
    zmk_endpoints_send_report(HID_USAGE_KEY);

    print_pos++;
    if (print_pos >= print_len) {
        print_len = 0;
        print_pos = 0;
        print_busy = false;
        return;
    }

    print_phase = JOYCON2_DEBUG_PRINT_PHASE_PRESS;
    k_work_schedule(&print_work, K_MSEC(JOYCON2_DEBUG_PRINT_STEP_DELAY_MS));
}

int zmk_joycon2_debug_print(const char *str) {
    if (print_busy) {
        LOG_WRN("debug_print busy, dropping message: %s", str);
        return -EBUSY;
    }

    size_t len = strlen(str);
    if (len == 0) {
        return 0;
    }
    if (len >= JOYCON2_DEBUG_PRINT_BUF_SIZE) {
        len = JOYCON2_DEBUG_PRINT_BUF_SIZE - 1;
    }

    memcpy(print_buf, str, len);
    print_len = len;
    print_pos = 0;
    print_phase = JOYCON2_DEBUG_PRINT_PHASE_PRESS;
    print_busy = true;

    k_work_schedule(&print_work, K_NO_WAIT);
    return 0;
}

static int joycon2_debug_print_init(void) {
    k_work_init_delayable(&print_work, joycon2_debug_print_work_handler);
    return 0;
}

SYS_INIT(joycon2_debug_print_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
