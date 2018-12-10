/*
 * VA-X DMA共通処理
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/20 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>
#include <sil.h>

/*
 * マクロ定義
 */

/**
 * Priority level (PL)
 */
#define DRVCMN_DMA_PL_LOW		0x0
#define DRVCMN_DMA_PL_MEDIUM	0x1
#define DRVCMN_DMA_PL_HIGH		0x2
#define DRVCMN_DMA_PL_VERY_HIGH	0x3

/**
 * Memory/Peripheral data size (MSIZE/PSIZE)
 */
#define DRVCMN_DMA_DATASIZE_8BIT	0x0
#define DRVCMN_DMA_DATASIZE_16BIT	0x1
#define DRVCMN_DMA_DATASIZE_32BIT	0x2

/**
 * Data transfer direction (DIR)
 */
#define DRVCMN_DMA_DIR_P2M	0x0
#define DRVCMN_DMA_DIR_M2P	0x1
#define DRVCMN_DMA_DIR_M2M	0x2


/*
 * 型定義
 */

/**
 * DMA ISRコールバック関数
 */
typedef void (*DRVCMN_DMA_ISR_FUNC_T)();

/**
 * DMAストリーム
 */
typedef struct {
    int32_t dmano;	///< DMA(1 or 2)
    int32_t strno;	///< Stream(0~7)
} DRVCMN_DMA_STR_T;

/**
 * DMAストリーム設定
 */
typedef struct {
    uint32_t chsel;		///< Channel selection (DMA_SxCR CHSEL) (0~7)
    bool_t dbm;			///< Double buffer mode (DMA_SxCR DBM)
    uint32_t pl;		///< Priority level (DMA_SxCR PL)
    //uint32_t msize;		// Memory data size
    //uint32_t psize;		// Peripheral data size
    uint32_t data_size;	///< Memory/Peripheral data size
    bool_t minc;		///< Memory increment mode (DMA_SxCR MINC)
    bool_t pinc;		///< Peripheral increment mode (DMA_SxCR PINC)
    uint32_t dir;		///< Data transfer direction (DMA_SxCR DIR)
} DRVCMN_DMA_SETTING_T;

/**
 * DMA転送設定
 */
typedef struct {
    uint32_t dir;				///< Data transfer direction (DMA_SxCR DIR)
    uintptr_t src;				///< 転送元
    uintptr_t dest;				///< 転送先
    size_t nbytes;				///< 転送バイト数(DBMの場合は1回の転送量)
    size_t nxfer;				///< 転送回数(DBM時のみ)
    bool_t use_fifo;			///< true: FIFO使用, false: Direct Mode
    DRVCMN_DMA_ISR_FUNC_T isr;	///< ISR
} DRVCMN_DMA_XFER_SETTING_T;

/*
 * 定数定義
 */


/*
 * 公開関数
 */

/**
 * DMAストリーム初期化
 * @param stream DMAストリーム
 * @param setting DMAストリーム設定
 */
void drvcmn_dma_initialize(const DRVCMN_DMA_STR_T* stream, const DRVCMN_DMA_SETTING_T* setting);

/**
 * DMAストリーム有効化
 * @param DMAストリーム
 * @param xfer_setting DMA転送設定
 */
void drvcmn_dma_transfer_enable(const DRVCMN_DMA_STR_T* stream, const DRVCMN_DMA_XFER_SETTING_T* xfer_setting);

/**
 * DMAストリーム無効化
 * @param stream DMAストリーム
 */
void drvcmn_dma_transfer_disable(const DRVCMN_DMA_STR_T* stream);

/*
 * 内部関数
 */
void drvcmn_dma_isr(intptr_t exinf);

