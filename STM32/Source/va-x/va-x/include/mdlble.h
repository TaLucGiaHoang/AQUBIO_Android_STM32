/*
 * VA-X BLEミドル
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/01/05 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>
#include "va-x_def.h"

/*
 * マクロ定数
 */

/* イベント種別 */
enum {
    MDLBLE_EVT_NONE = 0,
    MDLBLE_EVT_INITIALIZE_COMPLETE,
    MDLBLE_EVT_START_COMPLETE,
    MDLBLE_EVT_STOP_COMPLETE,
    MDLBLE_EVT_RESTART_COMPLETE,
    MDLBLE_EVT_CONNECTED,
    MDLBLE_EVT_DISCONNECTED,
    MDLBLE_EVT_VALIDATED,
    MDLBLE_EVT_VALIDATION_FAILED,
    MDLBLE_EVT_DATA_RECEIVED,
    MDLBLE_EVT_SEND_COMPLETE,
};

/* エラーコード */
#define MDLBLE_ERROR_NONE	0
#define MDLBLE_ERROR_OTHER	1

/* メモリプールブロック長(内部用) */
#define MDLBLE_MPFBLK_SIZE	20
#define MDLBLE_DTQ_SIZE		10
#define MDLBLE_MAX_DATA_LEN	256
#define MDLBLE_NUM_DATA_MPF	2

#define MDLBLE_PKG_FORMAT_REQ_ID       (0x40) // "@"
#define MDLBLE_PKG_FORMAT_RES_ID       (0x2A) // "*"
#define MDLBLE_PKG_FORMAT_ENDCODE      (0x0A) // "LF"

#define MDLBLE_CMD_AUTHENTICATE        (0x3030U) // "00"
#define MDLBLE_CMD_SET_DB_WIFI         (0x3131U) // "11"
#define MDLBLE_CMD_SET_PHONE_WIFI      (0x3132U) // "12"
#define MDLBLE_CMD_SET_DEVICE_INFO     (0x3133U) // "13"
#define MDLBLE_CMD_SET_COUNTER_INFO    (0x3134U) // "14"
#define MDLBLE_CMD_SET_ICCARD_INFO1    (0x3135U) // "15"
#define MDLBLE_CMD_UPDATE_FIRMWARE     (0x3145U) // "1E"
#define MDLBLE_CMD_INITILIZE_DATA      (0x3146U) // "1F"

#define MDLBLE_CMD_GET_VERSION         (0x3231U) // "21"
#define MDLBLE_CMD_GET_SERIAL          (0x3232U) // "22"
#define MDLBLE_CMD_GET_BLE_MAC         (0x3233U) // "23"
#define MDLBLE_CMD_GET_ROOM_INFO       (0x3234U) // "24"
#define MDLBLE_CMD_GET_DB_WIFI_INFO    (0x3235U) // "25"
#define MDLBLE_CMD_GET_ERROR_LOG       (0x3236U) // "26"
#define MDLBLE_CMD_GET_ICCARD_INFO1    (0x3237U) // "27"
#define MDLBLE_CMD_GET_ICCARD_INFO2    (0x3238U) // "28"
#define MDLBLE_CMD_GET_PARAMETERS      (0x3239U) // "29"
#define MDLBLE_CMD_GET_DIAGNOSTIC      (0x3241U) // "2A"

#define MDLBLE_CMD_CHECK_ICCARD1_SIZE  (0x3337U) // "37"
#define MDLBLE_CMD_CHECK_ICCARD2_SIZE  (0x3338U) // "38"

/*
 * 型定義
 */

/* コールバック関数型 */
typedef void (*MDLBLE_CALLBACK_T)(int event, intptr_t opt1, intptr_t opt2);

/* 受信データ */
typedef struct {
    uint8_t size;
    uint16_t command;
    uint8_t body[MDLBLE_MAX_DATA_LEN];
} MDLBLE_DATA_T;

/*
 * 公開関数
 */

// 初期化
void mdlble_initialize(MDLBLE_CALLBACK_T callback, const BNVA_DEVICE_ID_T* serialno);

// 開始
void mdlble_start();

// 停止
void mdlble_stop();

// 再起動
void mdlble_restart();

// 送信
void mdlble_send(uint16_t command, const uint8_t* data, size_t length);

// 受信バッファ返却
void mdlble_return_buffer(const uint8_t* data);

/*
 * 内部関数
 */
// タスク
void mdlble_task(intptr_t exinf);


