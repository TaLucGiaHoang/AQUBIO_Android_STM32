/*
 * VA-X UIアプリケーション
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/01/25 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>

#include "aplevt.h"
#include "aplicc.h"
#include "aplui.h"

/*
 * 型定義
 */

/*
 * 定数定義
 */

// メッセージID
enum {
    APLUI_EVT_XXX = 0,
};

/*
 * 共通初期化API(APLEVT_INITIALIZE_FUNC_T)
 */
int32_t aplui_initialize(APLEVT_EVENT_RECEIVER_FUNC_T receiver_func);

/*
 * 共通イベント受信API(APLEVT_EVENT_RECEIVER_FUNC_T)
 */
int32_t aplui_event(APLEVT_EVENT_T* event);

/*
 * 内部関数
 */

// UIタスク
void aplui_task(intptr_t exinf);

// LEDタスク
void aplui_led_task(intptr_t exinf);

// ブザータスク
void aplui_buz_task(intptr_t exinf);

// 【デバッグ用】LEDパターン変更
void aplui_dbg_set_led_pattern(int led, int pattern);

// 【デバッグ用】LED ALL DELETE
void aplui_dbg_led_pattern_off();

// 【デバッグ用】ブザーパターン変更
void aplui_dbg_set_buz_pattern(int pattern);

void led_turn_off_all();
void led_set_pattern(int led, int pattern);
