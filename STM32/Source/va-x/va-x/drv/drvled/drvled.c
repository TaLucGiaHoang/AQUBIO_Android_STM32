/*
 * VA-X LEDドライバ
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/13 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "drvled.h"

#include <kernel.h>
#include <sil.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "kernel_cfg.h"
#include "drvcmn_gpio.h"

/*
 * 型定義
 */

/*
 * 定数定義
 */

// GPIOピン
static const DRVCMN_GPIO_PIN_T GPIO_PINS[] = {
    { DRVCMN_GPIO_PORT_C, 0,},	// ERR
    { DRVCMN_GPIO_PORT_A, 11,},	// CER
    { DRVCMN_GPIO_PORT_C, 2,},	// PWR
    { DRVCMN_GPIO_PORT_C, 3,},	// BLE
    { DRVCMN_GPIO_PORT_B, 1,},	// REAR
};

/* ピン設定 */
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING = {
    .mode = DRVCMN_GPIO_MODE_GPOUT,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLDOWN,
};

/*
 * 内部変数
 */


/*
 * 公開関数
 */

/* 
 * ペリフェラル初期化
 */
void drvled_initialize_peripherals()
{
    for (int i = 0; i < sizeof(GPIO_PINS) / sizeof(GPIO_PINS[0]); i++) {
        drvcmn_gpio_pin_initialize(&(GPIO_PINS[i]), &GPIO_SETTING);	// ピン設定
        drvcmn_gpio_pin_set(&(GPIO_PINS[i]), false);				// Lowにする
    }
}

/* ドライバ初期化 */
void drvled_initialize()
{
    // 特に処理なし
}

/* LED 点灯/消灯 */
void drvled_set_state(int led_no, bool_t led_on)
{
    assert((led_no >= 0) && (led_no < (sizeof(GPIO_PINS) / sizeof(GPIO_PINS[0]))));
    drvcmn_gpio_pin_set(&(GPIO_PINS[led_no]), led_on);
}

