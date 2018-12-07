/*
 * VA-X カードアプリケーション
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/09/01 : 新規作成
 */

#pragma once

#include <t_stddef.h>

#include "aplevt.h"
#include "drvicc.h"

/*
 * 型定義
 */

/*
 * 定数定義
 */
#define APLICC_VACANCY_MAX 20
#define APLICC_DROOM_MAX 10

/* 券種 */
enum {
    APLICC_CARD_CONSTRUCTION = 0,
    APLICC_CARD_VACANCY_REGIST,
    APLICC_CARD_VACANCY,
    APLICC_CARD_REGIST,
    APLICC_CARD_DROOM1,
    APLICC_CARD_DROOM2,
    APLICC_NUM_CARD,
};

/* エラー */
enum {
    APLICC_ERROR_READ_CARD = 1,
    APLICC_ERROR_AUTH,
    APLICC_ERROR_IDENTIFY_CODE,
    APLICC_ERROR_WRITE_CARD,
    APLICC_ERROR_NO_REGIST_CARD,
    APLICC_ERROR_TIMEOUT,
    APLICC_ERROR_MAX_CNT,
    APLICC_ERROR_DRVICC,
};

/*
 * 共通初期化API(APLEVT_INITIALIZE_FUNC_T)
 */
int32_t aplicc_initialize(APLEVT_EVENT_RECEIVER_FUNC_T receiver_func);

/*
 * 共通イベント受信API(APLEVT_EVENT_RECEIVER_FUNC_T)
 */
int32_t aplicc_event(APLEVT_EVENT_T* event);

/*
 * 内部関数
 */

// タスク
void aplicc_task(intptr_t exinf);

