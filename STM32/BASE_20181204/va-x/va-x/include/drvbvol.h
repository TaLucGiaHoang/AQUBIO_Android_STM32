/*
 * VA-X バッテリー電圧計ドライバ
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/11/20 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>

/* コールバック関数型 */
typedef void (*DRVBVOL_CBFUNC_T)(uint32_t voltage_mv);


/*
 * 公開関数
 */

/* ペリフェラル初期化 */
void drvbvol_initialize_peripherals();

/* ドライバ初期化 */
void drvbvol_initialize();

/* 電圧取得 */
void drvbvol_get_voltage(DRVBVOL_CBFUNC_T callback);

/*
 * 内部関数
 */

/* 割込みサービスルーチン */
void drvbvol_adc_isr(intptr_t exinf);

/* タスク */
void drvbvol_task(intptr_t exinf);

