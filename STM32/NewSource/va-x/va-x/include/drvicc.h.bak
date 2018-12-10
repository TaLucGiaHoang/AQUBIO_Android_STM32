/*
 * VA-X ICカードドライバ (ARI3030)
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/04/04 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>

/*
 * マクロ
 */


/* エラー種別 */
#define DRVICC_ERROR_NONE			0
#define DRVICC_ERROR_TIMEOUT		1

/* メモリプールブロック長(内部用) */
#define DRVICC_MPFBLK_SIZE		16
#define DRVICC_DTQ_SIZE		10

/*
 * 型定義
 */
/* コールバック関数型 */
typedef void (*DRVICC_CALLBACK_T)(int32_t evt, int32_t error, intptr_t data);

/* Felica IDm */
#define DRVICC_FELICA_IDM_LEN 8
typedef struct {
    uint8_t idm[DRVICC_FELICA_IDM_LEN];
} DRVICC_FELICA_IDM_T;

/* Felica Block Descriptor */
typedef struct {
    uint8_t idm[DRVICC_FELICA_IDM_LEN];
    uint16_t sys;
    uint16_t service;
    uint8_t block;
//    uint8_t nblocks;
} DRVICC_FELICA_BLOCK_DESC_T;

/* Block */
#define DRVICC_FELICA_BLOCK_LEN 16
typedef struct {
    uint8_t block[DRVICC_FELICA_BLOCK_LEN];
} DRVICC_FELICA_BLOCK_T;

/*
 * 定数
 */
/* コールバック種別 */
enum {
    DRVICC_EVT_NONE = 0,
    DRVICC_EVT_INITIALIZE_COMPLETE,
    DRVICC_EVT_POLLING_COMPLETE,
    DRVICC_EVT_CANCEL_POLLING_COMPLETE,
    DRVICC_EVT_READ_COMPLETE,
    DRVICC_EVT_WRITE_COMPLETE,
};



/*
 * 公開関数
 */

/* ペリフェラル初期化 */
void drvicc_initialize_peripherals();

/* ドライバ初期化 */
void drvicc_initialize(DRVICC_CALLBACK_T callback);

/* ポーリング開始 */
void drvicc_start_polling(DRVICC_CALLBACK_T callback, DRVICC_FELICA_IDM_T* idm);

/* ポーリングキャンセル */
void drvicc_cancel_polling(DRVICC_CALLBACK_T callback);

/* カード読み取り */
void drvicc_read(DRVICC_CALLBACK_T callback, DRVICC_FELICA_BLOCK_T* blocks, const DRVICC_FELICA_BLOCK_DESC_T* desc);

/* カード書込み */
void drvicc_write(DRVICC_CALLBACK_T callback, const DRVICC_FELICA_BLOCK_T* blocks, const DRVICC_FELICA_BLOCK_DESC_T* desc);

/*
 * 内部関数
 */
// タスク
void drvicc_task(intptr_t exinf);

// 受信タスク
void drvicc_rx_task(intptr_t exinf);

