/*
 * Copyright (c) 2026 The zmk-joycon2 Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Joy-Con 2 button bits, as they appear in the input report's 32-bit button
 * word (report bytes 4..7, little-endian). Left-half and right-half buttons
 * occupy distinct bits in the same word, so one set of names covers both.
 */

#pragma once

#define JC2_Y 0x00000001U
#define JC2_X 0x00000002U
#define JC2_B 0x00000004U
#define JC2_A 0x00000008U
#define JC2_SR_R 0x00000010U
#define JC2_SL_R 0x00000020U
#define JC2_R 0x00000040U
#define JC2_ZR 0x00000080U
#define JC2_MINUS 0x00000100U
#define JC2_PLUS 0x00000200U
#define JC2_RSTK 0x00000400U
#define JC2_LSTK 0x00000800U
#define JC2_HOME 0x00001000U
#define JC2_CAPTURE 0x00002000U
#define JC2_C 0x00004000U
#define JC2_DOWN 0x00010000U
#define JC2_UP 0x00020000U
#define JC2_RIGHT 0x00040000U
#define JC2_LEFT 0x00080000U
#define JC2_SR_L 0x00100000U
#define JC2_SL_L 0x00200000U
#define JC2_L 0x00400000U
#define JC2_ZL 0x00800000U
#define JC2_GR 0x01000000U
#define JC2_GL 0x02000000U
