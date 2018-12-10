/*
 * VA-X 共用タイマータスク
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/04/24 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */
#pragma once

#include <t_stddef.h>

// タイマーID定義
enum {
    CMNTIMER_TIMER_APLMAIN = 0,
    CMNTIMER_TIMER_APLLDOROPN,
    CMNTIMER_TIMER_APLDOROPN,
    CMNTIMER_TIMER_APLLLOCK,
    CMNTIMER_TIMER_APLLONEINT,
    CMNTIMER_TIMER_APLICC,
    CMNTIMER_NUM_TIMERS,
};

// タイマー種別定義
enum {
    CMNTIMER_TYPE_NONE = 0,
    CMNTIMER_TYPE_ONESHOT,
    CMNTIMER_TYPE_CYCLIC,
};

// コールバック
typedef void (*CMNTIMER_CALLBACK_T)(int timer_id, intptr_t otp1);

// 初期化
void cmntimer_initialize();

// タイマー設定
void cmntimer_set_timer(int timer_id, int type, int interval, CMNTIMER_CALLBACK_T callback, intptr_t opt);

// タイマー設定 (非タスクコンテキスト用)
void isr_cmntimer_set_timer(int timer_id, int type, int interval, CMNTIMER_CALLBACK_T callback, intptr_t opt);

// タイマーキャンセル
void cmntimer_cancell_timer(int timer_id);

// タイマーキャンセル (非タスクコンテキスト用)
void isr_cmntimer_cancell_timer(int timer_id);

// タイマータスク
void cmntimer_task(intptr_t exinf);

