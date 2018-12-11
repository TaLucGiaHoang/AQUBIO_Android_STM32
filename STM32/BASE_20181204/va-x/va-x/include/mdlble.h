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
#define MDLBLE_MAX_DATA_LEN	4096
#define MDLBLE_NUM_DATA_MPF	2

/*
 * 型定義
 */

/* コールバック関数型 */
typedef void (*MDLBLE_CALLBACK_T)(int event, intptr_t opt1, intptr_t opt2);

/* 受信データ */
typedef struct {
    uint8_t service;
    uint16_t length;
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
void mdlble_send(uint8_t service, const uint8_t* data, size_t length);

// 受信バッファ返却
void mdlble_return_buffer(const uint8_t* data);

/*
 * 内部関数
 */
// タスク
void mdlble_task(intptr_t exinf);


