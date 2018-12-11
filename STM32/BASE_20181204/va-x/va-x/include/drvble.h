/*
 * VA-X BLEドライバ
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
 * マクロ定義
 */

/* コールバック種別 */
#define DRVBLE_EVT_INITIALIZE_COMPLETE		1
#define DRVBLE_EVT_FACTORY_RESET_COMPLETE	2
#define DRVBLE_EVT_GET_READY_COMPLETE		3
#define DRVBLE_EVT_POWER_OFF_COMPLETE		4
#define DRVBLE_EVT_SEND_COMPLETE			5
#define DRVBLE_EVT_RECEIVE_COMPLETE			6

/* エラー種別 */
#define DRVBLE_ERROR_NONE			0
#define DRVBLE_ERROR_TIMEOUT		1
#define DRVBLE_ERROR_CANCELLED		2

/* メモリプールブロック長(内部用) */
#define DRVBLE_MPFBLK_SIZE	20
#define DRVBLE_DTQ_SIZE		10

/*
 * 型定義
 */
/* コールバック関数型 */
typedef void (*DRVBLE_CALLBACK_T)(int32_t evt, int32_t error, intptr_t opt);

/*
 * 公開関数
 */

/* ペリフェラル初期化 */
void drvble_initialize_peripherals();

/* ペリフェラル初期化 */
void drvble_initialize(DRVBLE_CALLBACK_T callback);

/* BLEモジュール出荷初期化 */
void drvble_factory_reset(DRVBLE_CALLBACK_T callback, const BNVA_DEVICE_ID_T* device_id);

/* 送受信可能にする */
void drvble_get_ready(DRVBLE_CALLBACK_T callback);

/* 電源OFF */
void drvble_power_off(DRVBLE_CALLBACK_T callback);

/* 送信 */
void drvble_send(const uint8_t* data, size_t size, int32_t timeout, DRVBLE_CALLBACK_T callback);

/* 受信 */
void drvble_receive(uint8_t* data, size_t size, int32_t timeout, DRVBLE_CALLBACK_T callback);

/* 受信キャンセル */
void drvble_cancel_receive();

/*
 * 内部関数
 */
// タスク
void drvble_task(intptr_t exinf);

// 受信タスク
void drvble_rx_task(intptr_t exinf);

