/*
 * VA-X IR-LEDドライバ
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/18 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "drvirled.h"

#include <kernel.h>
#include <sil.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "stm32f7xx.h"

#include "drvcmn_gpio.h"
#include "drvcmn_tim.h"

/*
 * マクロ定義
 */

// サービスコールのエラーのログ出力
static inline void svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}
#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))

/*
 * 型定義
*/

typedef struct {
    DRVCMN_GPIO_PIN_T gpio_pin;			/* GPIOピン */
    uint32_t gpio_afno;					/* AF番号 */
    uint32_t timno;						/* TIM番号*/
    uint32_t tim_chno;					/* CH番号 */
    DRVCMN_TIM_SETTING_T tim_setting;	/* TIM設定 */
    uint32_t duty_percent;				/* DUTY比(0~100)*/
} TIM_SETTING_T;

/*
 * 定数定義
 */

#if defined(TOPPERS_STM32F7_BNVA)	// VA-X基板
// GPIO,TIM設定
static const TIM_SETTING_T TIM_SETTINGS[] = {
    {
        .gpio_pin = {DRVCMN_GPIO_PORT_A, 7},	/* PA7 */
        .gpio_afno = 2,							/* TIM3_CH2 */
        .timno = 3,
        .tim_chno = 2,
        .tim_setting = {
            .psc = 0xff,
            .arr = 0xff,
        },
        .duty_percent = 60,
    },
};
#elif defined(TOPPERS_STM32F7_DISCOVERY)	// Discoveryボード
// GPIO,TIM設定
static const TIM_SETTING_T TIM_SETTINGS[] = {
    {
        .gpio_pin = {DRVCMN_GPIO_PORT_I, 2},	/* PI2 (Disco D8) */
        .gpio_afno = 3,							/* TIM8_CH4 */
        .timno = 8,
        .tim_chno = 4,
        .tim_setting = {
            .psc = 0xff,
            .arr = 0xff,
        },
        .duty_percent = 50,
    },
    {
        .gpio_pin = {DRVCMN_GPIO_PORT_H, 6},	/* PH6 (Disco D6) */
        .gpio_afno = 9,							/* TIM12_CH1 */
        .timno = 12,
        .tim_chno = 1,
        .tim_setting = {
            .psc = 0xff,
            .arr = 0xff,
        },
        .duty_percent = 50,
    },
};
#endif

/*
 * ペリフェラル初期化
 */
void drvirled_initialize_peripherals()
{
	/* GPIO ピンの設定 */
    DRVCMN_GPIO_SETTING_T gpio_setting = {
        .mode = DRVCMN_GPIO_MODE_AF,
        .otype = DRVCMN_GPIO_OTYPE_PP,
        .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
        .pupd = DRVCMN_GPIO_PUPD_PULLDOWN,
    };

    DRVCMN_TIM_CH_SETTING_T tim_ch_setting = {
        .ccr = 0,
    };

    for (int i = 0; i < sizeof(TIM_SETTINGS) / sizeof(TIM_SETTINGS[0]); i++) {
        /* GPIO初期化 */
        gpio_setting.afno = TIM_SETTINGS[i].gpio_afno;
        drvcmn_gpio_pin_initialize(&(TIM_SETTINGS[i].gpio_pin), &gpio_setting);

        /* TIM 初期化 */
        drvcmn_tim_initialize(TIM_SETTINGS[i].timno, &(TIM_SETTINGS[i].tim_setting));

        /* TIM チャネル初期化 */
        tim_ch_setting.ccr = (TIM_SETTINGS[i].tim_setting.arr * (100 - TIM_SETTINGS[i].duty_percent)) / 100;	/* 反転 */
        drvcmn_tim_ch_initialize(TIM_SETTINGS[i].timno, TIM_SETTINGS[i].tim_chno, &tim_ch_setting);
    }

#if 1	// TODO: 電源管理側で管理する
    {
        const DRVCMN_GPIO_PIN_T	SVCC_ON_PIN = { DRVCMN_GPIO_PORT_D, 2 };
        const DRVCMN_GPIO_SETTING_T SVCC_ON_SETTING = {
            .mode = DRVCMN_GPIO_MODE_GPOUT,
            .otype = DRVCMN_GPIO_OTYPE_PP,
            .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
            .pupd = DRVCMN_GPIO_PUPD_PULLDOWN,
        };

        drvcmn_gpio_pin_initialize(&SVCC_ON_PIN, &SVCC_ON_SETTING);
        drvcmn_gpio_pin_set(&SVCC_ON_PIN, true);
    }
#endif
}

/*
 * ドライバ初期化
 */
void drvirled_initialize()
{
    // 処理なし
}

/*
 * LED 点灯/消灯
 */
void drvirled_set_state(bool_t on)
{
    for (int i = 0; i < sizeof(TIM_SETTINGS) / sizeof(TIM_SETTINGS[0]); i++) {
        drvcmn_tim_set_output_enable(TIM_SETTINGS[i].timno, TIM_SETTINGS[i].tim_chno, on);
        drvcmn_tim_set_counter_enable(TIM_SETTINGS[i].timno, on);
    }
}

/* デューティー比変更(デバッグ用) */
void drvirled_set_duty(uint32_t ledno, uint32_t duty_percent)
{
    assert(ledno < sizeof(TIM_SETTINGS) / sizeof(TIM_SETTINGS[0]));
    assert((duty_percent >= 0) && (duty_percent <= 100));
           
    DRVCMN_TIM_CH_SETTING_T tim_ch_setting = {
        .ccr = (TIM_SETTINGS[ledno].tim_setting.arr * (100 - duty_percent)) / 100,	/* 反転 */
    };

    drvcmn_tim_update_ch_setting(TIM_SETTINGS[ledno].timno, TIM_SETTINGS[ledno].tim_chno,
                                 DRVCMN_TIM_UPDATE_CCR, &tim_ch_setting);
}

