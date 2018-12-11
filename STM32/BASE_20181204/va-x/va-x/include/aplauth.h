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
 * API
 */

/*
 * 共通初期化API(APLEVT_INITIALIZE_FUNC_T)
 */
int32_t aplauth_initialize(APLEVT_EVENT_RECEIVER_FUNC_T receiver_func);

/*
 * 共通イベント受信API(APLEVT_EVENT_RECEIVER_FUNC_T)
 */
int32_t aplauth_event(APLEVT_EVENT_T* event);

/*
 * タスク
 */
void aplauth_task(intptr_t exinf);



