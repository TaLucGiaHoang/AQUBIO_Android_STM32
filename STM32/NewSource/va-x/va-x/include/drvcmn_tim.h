/*
 * VA-X
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/16 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>
#include <sil.h>

/*
 * マクロ定義
 */

// 更新フラグ
#define DRVCMN_TIM_UPDATE_PSC	(0x1 << 0)
#define DRVCMN_TIM_UPDATE_ARR	(0x1 << 1)

// 更新フラグ(チャネル)
#define DRVCMN_TIM_UPDATE_CCR	(0x1 << 0)

/*
 * 型定義
 */
typedef struct {
    uint32_t psc;		// TIMx_PSC
    uint32_t arr;		// TIMx_ARR
} DRVCMN_TIM_SETTING_T;

typedef struct {
    uint32_t ccr;		// TIMx_CCRx
} DRVCMN_TIM_CH_SETTING_T;

/*
 * 公開関数
 */

// 初期化
void drvcmn_tim_initialize(int tim, const DRVCMN_TIM_SETTING_T* setting);

// チャネル初期化
void drvcmn_tim_ch_initialize(int tim, int ch, const DRVCMN_TIM_CH_SETTING_T* setting);

// カウンター有効/無効設定
void drvcmn_tim_set_counter_enable(int tim, bool_t enable);

// 出力有効/無効
void drvcmn_tim_set_output_enable(int tim, int ch, bool_t enable);

// 設定を変更
void drvcmn_tim_update_setting(int tim, uint32_t flags, const DRVCMN_TIM_SETTING_T* setting);

// チャネル設定を変更
void drvcmn_tim_update_ch_setting(int tim, int ch, uint32_t flags, const DRVCMN_TIM_CH_SETTING_T* setting);

// 

/*
 * 内部関数
 */

