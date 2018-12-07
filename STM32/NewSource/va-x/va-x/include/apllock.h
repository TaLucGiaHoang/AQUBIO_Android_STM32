/*
 * VA-X 錠制御アプリケーション
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/04/24 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>

#include "aplevt.h"

/*
 * 型定義
 */

/*
 * 定数定義
 */


/*
 * 共通初期化API(APLEVT_INITIALIZE_FUNC_T)
 */
int32_t apllock_initialize(APLEVT_EVENT_RECEIVER_FUNC_T receiver_func);

/*
 * 共通イベント受信API(APLEVT_EVENT_RECEIVER_FUNC_T)
 */
int32_t apllock_event(APLEVT_EVENT_T* event);

/*
 * 内部関数
 */

// UIタスク
void apllock_task(intptr_t exinf);

