/*
 * VA-X 外部フラッシュドライバ
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/02/07 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>

/*
 * マクロ定義
 */


/* フラッシュメモリ特性 */
#define DRVFLX_FLASH_MEMORY_SIZE	(2 * 1024 * 1024)	// 2MByte
#define DRVFLX_ERASE_BLOCK_SIZE		(4 * 1024)			// 4KByte

/* メモリプールブロック長(内部用) */
#define DRVFLX_MPFBLK_SIZE	20
#define DRVFLX_DTQ_SIZE		10

/*
 * 型定義
 */

/* コールバック関数型 */
typedef void (*DRVFLX_CALLBACK_T)(int evt, int error, intptr_t opt);

/*
 * 定数
 */

/* コールバック種別 */
enum {
    DRVFLX_EVT_NONE = 0,
    DRVFLX_EVT_INITIALIZE_COMPLETE,
    DRVFLX_EVT_READ_COMPLETE,
    DRVFLX_EVT_WRITE_COMPLETE,
    DRVFLX_EVT_ERASE_COMPLETE,
    DRVFLX_EVT_MEMMAP_STARTED,
    DRVFLX_EVT_MEMMAP_END,
};

/*
 * 公開関数
 */

/* ペリフェラル初期化 */
void drvqflx_initialize_peripherals();

/* ドライバ初期化 */
void drvqflx_initialize();

/* 読み込み */
void drvqflx_read(void* dest, intptr_t src, size_t length);

/* メモリマップ開始 */
void drvqflx_start_memmap(DRVFLX_CALLBACK_T callback);

/* メモリマップ終了 */
void drvqflx_end_memmap(DRVFLX_CALLBACK_T callback);

/* 書き込み */
void drvqflx_write(intptr_t dest, const void* src, size_t length);

/* 消去 */
void drvqflx_erase(intptr_t addr, size_t length);

// ISR
void drvqflx_qspi_isr(intptr_t exinf);


