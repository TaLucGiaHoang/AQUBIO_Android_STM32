/*
 * VA-X カメラドライバ
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/25 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once
#include <t_stddef.h>



/*
 * マクロ定義
 */

// コールバック種別
#define DRVCAM_CALLBACK_INITIALIZE	1
#define DRVCAM_CALLBACK_POWERON		2
#define DRVCAM_CALLBACK_POWEROFF	3
#define DRVCAM_CALLBACK_PREPARE		4
#define DRVCAM_CALLBACK_CAPTURE		5

// カメラパラメタ種別
#define DRVCAM_PARAM_EXPOSURE		1

// 露出プリセット
#define DRVCAM_EXPOSURE_NORMAL		0
#define DRVCAM_EXPOSURE_HIGH		1
#define DRVCAM_EXPOSURE_LOW			2

// 必要バッファサイズ
#define DRVCAM_CAPTURE_BUFFER_SIZE	((640 * 480) + (1024 * 4))	// 640x480 Y8 + マージン(4kB)

/*
 * 型定義
 */

/* コールバック関数型 */
typedef void (*DRVCAM_CALLBACK_T)(int32_t type, int32_t error);

/* カメラレジスタのアドレスと値 */
typedef struct {
    uint8_t addr;
    uint16_t	value;
} DRVCAM_REGVAL_T;

/* 撮影パラメタ */
typedef struct {
    int32_t exposure;			// 露出設定
    DRVCAM_REGVAL_T* regval;	// カメラレジスタ設定値(デバッグ用)
    size_t num_regval;			// カメラレジスタ設定値の数
} DRVCAM_CAPTURE_PARAM_T;


/*
 * 定数
*/

/*
 * 公開関数
 */

/* ペリフェラル初期化 */
void drvcam_initialize_peripherals();

/* ドライバ初期化 */
void drvcam_initialize(DRVCAM_CALLBACK_T callback);

/* カメラ電源ON */
void drvcam_power_on(DRVCAM_CALLBACK_T callback);

/* カメラ電源OFF */
void drvcam_power_off(DRVCAM_CALLBACK_T callback);

/* 撮影準備 */
void drvcam_prepare_capture(DRVCAM_CALLBACK_T callback, const DRVCAM_CAPTURE_PARAM_T* param);

/* 撮影 */
void drvcam_capture(DRVCAM_CALLBACK_T callback, uint8_t* output, size_t size);

/*
 * 内部関数
 */

/* 割込みサービスルーチン */
void drvcam_dcmi_isr(intptr_t exinf);

/* 割込み処理タスク */
void drvcam_task(intptr_t exinf);

