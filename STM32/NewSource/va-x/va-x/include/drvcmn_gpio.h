/*
 * VA-X
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/13 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>
#include <sil.h>

/*
 * 定数定義
 */

/**
 * GPIOポート(A~K)
 */
#define DRVCMN_GPIO_PORT_A 0
#define DRVCMN_GPIO_PORT_B 1
#define DRVCMN_GPIO_PORT_C 2
#define DRVCMN_GPIO_PORT_D 3
#define DRVCMN_GPIO_PORT_E 4
#define DRVCMN_GPIO_PORT_F 5
#define DRVCMN_GPIO_PORT_G 6
#define DRVCMN_GPIO_PORT_H 7
#define DRVCMN_GPIO_PORT_I 8
#define DRVCMN_GPIO_PORT_J 9
#define DRVCMN_GPIO_PORT_K 10

/**
 * GPIO モード (GPIOx_MODER)
 */
#define DRVCMN_GPIO_MODE_INPUT	0
#define DRVCMN_GPIO_MODE_GPOUT	1
#define DRVCMN_GPIO_MODE_AF		2
#define DRVCMN_GPIO_MODE_ANALOG	3

/**
 * GPIO 出力タイプ (GPIOx_OTYPER)
 */
#define DRVCMN_GPIO_OTYPE_PP	0
#define DRVCMN_GPIO_OTYPE_OD	1

/**
 * GPIO 出力スピード (GPIOx_OSPEEDR)
 */
#define DRVCMN_GPIO_OSPEED_LOW		0
#define DRVCMN_GPIO_OSPEED_MEDIUM	1
#define DRVCMN_GPIO_OSPEED_HIGH		2
#define DRVCMN_GPIO_OSPEED_VERYHIGH	3

/** 
 * GPIO プルアップ/プルダウン (GPIOx_PUPDR)
 */
#define DRVCMN_GPIO_PUPD_NONE		0
#define DRVCMN_GPIO_PUPD_PULLUP		1
#define DRVCMN_GPIO_PUPD_PULLDOWN	2

/**
 * GPIO EXTI 設定
 */
#define DRVCMN_GPIO_EXTI_NONE		0	///< EXTIなし
#define DRVCMN_GPIO_EXTI_RTRIG		1	///< Risingエッジトリガ
#define DRVCMN_GPIO_EXTI_FTRIG		2	///< Fallingエッジトリガ
#define DRVCMN_GPIO_EXTI_BOTHTRIG	(DRVCMN_GPIO_EXTI_RTRIG | DRVCMN_GPIO_EXTI_FTRIG)	///< 両方向エッジトリガ

/*
 * 型定義
 */

/**
 * ISRコールバック関数
 * @param pinno ピン番号
 * @return 未使用
 */
typedef int32_t (*DRVCMN_EXTI_ISR_T)(uint32_t pinno);

/**
 * GPIO ピン
 */
typedef struct {
    uint32_t port;	///< ポート(DRVCMN_GPIO_PORT_X)
    uint32_t pinno;	///< ピン番号(0~15)
} DRVCMN_GPIO_PIN_T;

/**
 * GPIO ピン初期化設定
 */
typedef struct {
    uint32_t mode;				///< GPIOx_MODER
    uint32_t otype;				///< GPIOx_OTYPER
    uint32_t ospeed;			///< GPIOx_OSPEEDR
    uint32_t pupd;				///< GPIOx_PUPDR
    uint32_t afno;				///< AF番号(0~15)
    uint32_t exti;				///< EXTI設定
    DRVCMN_EXTI_ISR_T exti_isr;	///< ISRコールバック関数
} DRVCMN_GPIO_SETTING_T;

/*
 * 公開関数
 */

/**
 * GPIO のピンを初期化
 * @param pin ピン
 * @param setting 設定
 */
void drvcmn_gpio_pin_initialize(const DRVCMN_GPIO_PIN_T* pin, const DRVCMN_GPIO_SETTING_T* setting);

/**
 * GPIO のピンの出力をセット
 * @param pin ピン
 * @param set ピンの出力(0,1)
 */
void drvcmn_gpio_pin_set(const DRVCMN_GPIO_PIN_T* pin, uint32_t set);

/**
 * GPIO のピンの状態を取得
 * @param pin ピン
 * @return ピンの状態(0,1)
 */
uint32_t drvcmn_gpio_pin_get(const DRVCMN_GPIO_PIN_T* pin);

/**
 * EXTI の割込みマスクを設定
 * @param ピン番号
 * @param mask (true: マスクあり, false: マスクなし)
 */
void drvcmn_gpio_set_exti_mask(uint32_t pinno, bool_t mask);

/**
 * EXTI の PR レジスタをクリア
 * @param pinno ピン番号
 */
void drvcmn_gpio_clear_exti_pr(uint32_t pinno);

/*
 * 内部関数
 */
/* 割込みサービスルーチン */
void drvcmn_gpio_exti_isr(intptr_t exinf);

