#include "drvbuz.h"

#include <kernel.h>
#include <sil.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "stm32f7xx.h"

#include "kernel_cfg.h"
#include "drvcmn.h"
#include "drvcmn_gpio.h"
#include "drvcmn_tim.h"

/* マクロ */
#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))

/*
 *  サービスコールのエラーのログ出力
 */
inline void svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}
#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))

// タイマークロックソース
#define CLK_SOURCE_HZ	((uint32_t)216000000)	// 216MHz

// プリスケーラ
#define TIM_PSC	0xff

/*
 * 型
 */

/* 
 * 定数
 */

// GPIOピン
static const DRVCMN_GPIO_PIN_T BUZZER_GPIO_PIN = { DRVCMN_GPIO_PORT_A, 5 };

// GPIO ピンの設定
static const DRVCMN_GPIO_SETTING_T BUZZER_GPIO_SETTING = {
    .mode = DRVCMN_GPIO_MODE_AF,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLDOWN,
    .afno = 1,
};

// TIM番号
static const int BUZZER_TIMNO = 2;

// TIM チャネル番号
static const int BUZZER_CHNO = 1;

// TIM設定
static const DRVCMN_TIM_SETTING_T BUZZER_TIM_SETTING = {
    .psc = TIM_PSC,
    .arr = (CLK_SOURCE_HZ / TIM_PSC) / 440,
};

// TIM チャネル設定
static const DRVCMN_TIM_CH_SETTING_T BUZZER_TIM_CH_SETTING = {
    .ccr = ((CLK_SOURCE_HZ / TIM_PSC) / 440) / 2,
};


/*
 * 内部変数
 */

/*
 * ペリフェラル初期化
 */
void drvbuz_initialize_peripherals()
{
    /* GPIO初期化 */
    drvcmn_gpio_pin_initialize(&BUZZER_GPIO_PIN, &BUZZER_GPIO_SETTING);

    /* TIM 初期化 */
    drvcmn_tim_initialize(BUZZER_TIMNO, &BUZZER_TIM_SETTING);

    /* TIM チャネル初期化 */
    drvcmn_tim_ch_initialize(BUZZER_TIMNO, BUZZER_CHNO, &BUZZER_TIM_CH_SETTING);
}

/*
 * 初期化
 */
void drvbuz_initialize()
{
    // 特に処理なし
}

/*
 * ブザー開始
 */
void drvbuz_start(int freq_x10)
{
    // TIM設定
    DRVCMN_TIM_SETTING_T tim_setting = {
        .arr = (((CLK_SOURCE_HZ * 10) / TIM_PSC) / freq_x10) / 10,
    };

    // TIM チャネル設定
    DRVCMN_TIM_CH_SETTING_T tim_ch_setting = {
        .ccr = tim_setting.arr / 2,
    };

    // 周波数に合わせて設定変更
    drvcmn_tim_update_setting(BUZZER_TIMNO, DRVCMN_TIM_UPDATE_ARR, &tim_setting);
    drvcmn_tim_update_ch_setting(BUZZER_TIMNO, BUZZER_CHNO, DRVCMN_TIM_UPDATE_CCR, &tim_ch_setting);

    // PWM出力開始
    drvcmn_tim_set_output_enable(BUZZER_TIMNO, BUZZER_CHNO, true);
    drvcmn_tim_set_counter_enable(BUZZER_TIMNO, true);
}

/*
 * ブザー停止
 */
void drvbuz_stop()
{
    // PWM出力停止
    drvcmn_tim_set_output_enable(BUZZER_TIMNO, BUZZER_CHNO, false);
    drvcmn_tim_set_counter_enable(BUZZER_TIMNO, false);
}


