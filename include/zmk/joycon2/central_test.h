#pragma once

/**
 * Stage 1 sub-stage 2 test: starts a one-shot BLE central scan, connects to
 * the first connectable advertiser found, and a couple of seconds after
 * connecting (or on timeout / failure) reports the outcome via
 * zmk_joycon2_debug_print() -- including how many BLE LE connections are
 * alive in each role, so the existing host + split-half links can be
 * confirmed to have survived alongside the new one.
 *
 * Returns -EBUSY if a test is already in progress.
 */
int zmk_joycon2_central_test_start(void);
