/*
 * VA-X 認証アルゴリズム
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/01/31 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>

/*
 * マクロ定数
 */

//#define MDLAUTHLIB_DATA_CAPTURED_SIZE			16500
#define MDLAUTHLIB_DATA_CAPTURED_SIZE			29700
#define MDLAUTHLIB_DATA_REGISTERED_SIZE			12800
#define MDLAUTHLIB_DATA_REGISTERED_SMALL_SIZE	400

/*
 * 型定義
 */
#define SIZE_INPUT_X	640		//撮影サイズ(X)
#define SIZE_INPUT_Y	480		//撮影サイズ(Y)
#define FIN_CUT_SIZE_X	80
#define FIN_CUT_SIZE_Y	480
#define FIN_BLOCK_SIZE_X 1
#define FIN_BLOCK_SIZE_Y 10
#define FIN_INTERVAL_X 20
#define X_Number FIN_CUT_SIZE_X / FIN_INTERVAL_X
#define Y_Number FIN_CUT_SIZE_Y / FIN_BLOCK_SIZE_Y
#define FIN_X 0
#define FIN_Y 0

// 画像データ
typedef struct {
    uint8_t*	pixels;	// 画像データ
    size_t		length;	// データ長
    int 		width;	// 幅
    int			height;	// 高さ
} MDLAUTHLIB_IMAGE_T;

// 矩形
typedef struct {
    int	left;	// 左上X
    int	top;	// 左上Y
    int	width;	// 幅
    int	height;	// 高さ
} MDLAUTHLIB_RECT_T;

// 登録データ (フラッシュから読込み)
typedef struct {
    uint8_t data[MDLAUTHLIB_DATA_REGISTERED_SIZE];
} MDLAUTHLIB_DATA_REGISTERED_T;

// 登録データ (小サイズ) (フラッシュから読込み)
typedef struct {
    uint8_t data[MDLAUTHLIB_DATA_REGISTERED_SMALL_SIZE];
} MDLAUTHLIB_DATA_REGISTERED_SMALL_T;

// 認証データ (撮影データから生成)
typedef struct {
    uint8_t data[MDLAUTHLIB_DATA_CAPTURED_SIZE];
} MDLAUTHLIB_DATA_CAPTURED_T;

typedef struct {
    bool_t match;	// 判定: {true: OK, false: NG}
    int offset;		// オフセット
    int score;		// スコア
} MDLAUTHLIB_MATCH_RESULT_T;

/*
 * 公開関数
 */
int mdlauthlib_finger_search(const MDLAUTHLIB_IMAGE_T* image, const MDLAUTHLIB_RECT_T* rect);
/*
 * 画像の明るさ(輝度平均値)を算出
 */
int mdlauthlib_calc_brightness(const MDLAUTHLIB_IMAGE_T* image, const MDLAUTHLIB_RECT_T* rect);

/*
 * 画像処理(撮影画像から認証データを生成)
 */
int mdlauthlib_process_image(MDLAUTHLIB_DATA_CAPTURED_T* data, MDLAUTHLIB_IMAGE_T* input);

/*
 * 極小画像のマッチング処理を行いスコアを返す
 */
int mdlauthlib_match_small(const MDLAUTHLIB_DATA_CAPTURED_T* captured_data, const MDLAUTHLIB_DATA_REGISTERED_SMALL_T* registered_data);

/*
 * 画像のマッチング処理を行う
 */
void mdlauthlib_match(MDLAUTHLIB_MATCH_RESULT_T* result, const MDLAUTHLIB_DATA_CAPTURED_T* captured_data, const MDLAUTHLIB_DATA_REGISTERED_T* registered_data);

/*
 * 撮影データから登録用のデータを生成する
 */
int mdlauthlib_create_data_registered(MDLAUTHLIB_DATA_REGISTERED_T* registered_data, MDLAUTHLIB_DATA_REGISTERED_SMALL_T* registered_data_small, const MDLAUTHLIB_DATA_CAPTURED_T* captured_data, int offset);

/*
 * 登録時1回目処理
 */
int mdlauthlib_registered_first_image(MDLAUTHLIB_DATA_CAPTURED_T* a_data, MDLAUTHLIB_IMAGE_T* input);

/*
 * 登録時2回目処理
 */
int mdlauthlib_registered_second_image(MDLAUTHLIB_DATA_CAPTURED_T* a_data, MDLAUTHLIB_IMAGE_T* input);

/*
 * 登録画像処理(撮影画像から認証データを生成)
 */
int mdlauthlib_regprocess_image(MDLAUTHLIB_DATA_CAPTURED_T* a_data, MDLAUTHLIB_IMAGE_T* input);

/*
 * FLASH読み込み（CenterTrimming用）
 */
void center_flash_read();

