/*
 * VA-X シリアルコンソール
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/04/11 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */
#pragma once

#include <t_stddef.h>

#include "aplevt.h"

/*********************************************************************
 * API
*********************************************************************/

// 共通初期化API(APLEVT_INITIALIZE_FUNC_T)
int32_t aplcon_initialize(APLEVT_EVENT_RECEIVER_FUNC_T receiver_func);

// 共通イベント受信API(APLEVT_EVENT_RECEIVER_FUNC_T)
int32_t aplcon_event(APLEVT_EVENT_T* event);

/*********************************************************************
 * 内部関数
*********************************************************************/

// タスク
void aplcon_task(intptr_t exinf);

