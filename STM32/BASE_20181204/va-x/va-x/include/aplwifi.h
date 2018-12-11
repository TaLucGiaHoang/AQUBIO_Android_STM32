/*
 * VA-X WiFiアプリケーション
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 */

#pragma once

#include <t_stddef.h>

#include "aplevt.h"
#include "drvwifi.h"

/*
 * 型定義
 */

/*
 * 定数定義
 */

/* エラー */
enum {
    APLWIFI_ERROR_INITIALIZE = 1,
    APLWIFI_ERROR_AP_CONNECT,
    APLWIFI_ERROR_HTTPS_CONNECT,
    APLWIFI_ERROR_HTTPS_POST,
    APLWIFI_ERROR_TIMEOUT,
    APLWIFI_ERROR_READ_INFO,
    APLWIFI_ERROR_WRITE_ERR_LOG,
    APLWIFI_ERROR_WRITE_OPE_LOG,
    APLWIFI_ERROR_WRITE_CARD,
    APLWIFI_ERROR_HTTPS_DISCONNECT,
    APLWIFI_ERROR_AP_DISCONNECT,
};

/*
 * 共通初期化API(APLEVT_INITIALIZE_FUNC_T)
 */
int32_t aplwifi_initialize(APLEVT_EVENT_RECEIVER_FUNC_T receiver_func);

/*
 * 共通イベント受信API(APLEVT_EVENT_RECEIVER_FUNC_T)
 */
int32_t aplwifi_event(APLEVT_EVENT_T* event);

/*
 * 内部関数
 */

// タスク
void aplwifi_task(intptr_t exinf);

