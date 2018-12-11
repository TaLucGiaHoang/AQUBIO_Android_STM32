#pragma once
#include <t_stddef.h>

/*
 * 定数
 */


/*
 * 公開関数
 */

/* 初期化 */
void drvbuz_initialize_peripherals();

/* 初期化 */
void drvbuz_initialize();

/* ブザー開始 */
void drvbuz_start(int freq_x10);

/* ブザー停止 */
void drvbuz_stop();

/* 割込み処理タスク */
void drvbuz_task(intptr_t exinf);

