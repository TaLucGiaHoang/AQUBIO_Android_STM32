/*
 * VA-X
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/13 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>

/* コールバック関数型 */
typedef void (*DRVPBT_CBFUNC_T)(uint32_t button, int32_t evt);

/* 定数 */
#define DRVPBT_EVT_PRESS		1
#define DRVPBT_EVT_RELEASE		2

/*
 * 公開関数
 */


/* ペリフェラル初期化 */
void drvpbt_initialize_peripherals();

/* ドライバ初期化 */
void drvpbt_initialize(DRVPBT_CBFUNC_T callback);

/*
 * 内部関数
 */

/* 割込みサービスルーチン */
int32_t drvpbt_pushbutton_isr(uint32_t pinno);

/* 割込み処理タスク */
void drvpbt_task(intptr_t exinf);

