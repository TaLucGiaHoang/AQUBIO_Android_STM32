/*
 * VA-X 生体センサードライバ
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/16 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>

/* コールバック関数型 */
typedef void (*DRVTS_CALLBACK_T)(int32_t evt);

/* 定数 */
enum {
    DRVTS_EVT_INITE = 0,
    DRVTS_EVT_TOUCH,
    DRVTS_EVT_RELEASE,
    DRVTS_EVT_INTERMEDIATE,
    DRVTS_EVT_INITIALIZE_COMPLETE,
    DRVTS_EVT_FACTORY_RESET_COMPLETE,
};

/*
 * 公開関数
 */

void drvts_initialize_peripherals();

int32_t drvts_initialize();

/* 生体センサー開始 */
int32_t drvts_start(DRVTS_CALLBACK_T callback);

/* 生体センサー停止 */
int32_t drvts_stop();

/*
 * 内部関数
 */

/* 割込みサービスルーチン */
int32_t drvts_exti_isr(uint32_t pinno);

/* 割込み処理タスク */
void drvts_task(intptr_t exinf);
