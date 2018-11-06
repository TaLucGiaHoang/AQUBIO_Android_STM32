/*
 * VA-X Wifiミドル
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/03/29 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>

/*
 * マクロ定数
 */


/*
 * 型定義
 */

/* コールバック関数型 */
typedef void (*MDLWIFI_CALLBACK_T)(int event, intptr_t opt1, intptr_t opt2);


/*
 * 公開関数
 */

// 初期化
void mdlwifi_initialize(MDLWIFI_CALLBACK_T callback);

/*
 * 内部関数
 */
// タスク
void mdlwifi_task(intptr_t exinf);


