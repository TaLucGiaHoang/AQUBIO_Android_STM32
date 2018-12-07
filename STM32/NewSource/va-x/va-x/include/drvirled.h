/*
 * VA-X IR-LEDドライバ
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/18 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>

/*
 * 公開関数
 */

/* ペリフェラル初期化 */
void drvirled_initialize_peripherals();

/* ドライバ初期化 */
void drvirled_initialize();

/* LED 点灯/消灯 */
void drvirled_set_state(bool_t on);

/* デューティー比変更(デバッグ用) */
void drvirled_set_duty(uint32_t ledno, uint32_t duty_percent);

