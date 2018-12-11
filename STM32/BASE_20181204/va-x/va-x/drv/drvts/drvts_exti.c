/*
 * VA-X 生体センサードライバ
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/13 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "drvts.h"

#include <kernel.h>
#include <sil.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "stm32f7xx.h"

#include "kernel_cfg.h"
#include "drvcmn_gpio.h"

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

/*
 * 定数定義
 */

#if defined(TOPPERS_STM32F7_BNVA)	// VA-X基板

/* GPIOポート1(手前側) */
static const DRVCMN_GPIO_PIN_T GPIO_PIN1 = {	/* B5 */
    DRVCMN_GPIO_PORT_B, 5,
};

/* GPIOポート2(奥側) */
static const DRVCMN_GPIO_PIN_T GPIO_PIN2 = {	/* B8 */
    DRVCMN_GPIO_PORT_B, 8,
};

#elif defined(TOPPERS_STM32F7_DISCOVERY)	// Discoveryボード

/* GPIOポート1(手前側) */
static const DRVCMN_GPIO_PIN_T GPIO_PIN1 = {	/* I0 */
    DRVCMN_GPIO_PORT_I, 0,
};

/* GPIOポート2(奥側) */
static const DRVCMN_GPIO_PIN_T GPIO_PIN2 = {	/* H6 */
    DRVCMN_GPIO_PORT_H, 6,
};

#endif

/* チャタリング時間[ms] */
static const TMO CHATTER_TIMEOUT = 50;

/* GPIO ピンの設定 */
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING = {
    .mode = DRVCMN_GPIO_MODE_INPUT,
    .pupd = DRVCMN_GPIO_PUPD_PULLDOWN,
    .exti = DRVCMN_GPIO_EXTI_BOTHTRIG,
    .exti_isr = drvts_exti_isr,
};

/*
 * 内部変数
 */
static DRVTS_CALLBACK_T s_callback = NULL;

/*
 * 初期化コード
 */
int32_t drvts_initialize()
{
    drvcmn_gpio_pin_initialize(&GPIO_PIN1, &GPIO_SETTING);
    drvcmn_gpio_pin_initialize(&GPIO_PIN2, &GPIO_SETTING);

    return 0;
}

/* 生体センサー開始 */
int32_t drvts_start(DRVTS_CALLBACK_T callback)
{
    assert(callback);
    
    /* コールバックを設定 */
    s_callback = callback;

    return 0;
}

/* 生体センサー停止 */
int32_t drvts_stop()
{
    s_callback = NULL;

    return 0;
}

/*
 * 割込みサービスルーチン
 */
int32_t drvts_exti_isr(uint32_t pinno)
{
	syslog(LOG_NOTICE, "drvts_exti_isr(%d)", pinno);

    /* 割込みが多いので一旦マスクする */
    drvcmn_gpio_set_exti_mask(pinno, true);

    /* ピンの番号のみ通知する(ピンの状態は後で取得) */
    SVC_PERROR(ipsnd_dtq(DTQ_DRVTS, pinno));

    return 0;
}

/*
 * 割込み処理タスク
 */
void drvts_task(intptr_t exinf)
{
	syslog(LOG_NOTICE, "drvts_task() starts .");

    bool_t pending_event = false;
    int32_t last_event = -1;
    while (true) {
        TMO timeout = 0;
        if (pending_event) {
            timeout = CHATTER_TIMEOUT;
        } else {
            timeout = TMO_FEVR;
        }

        intptr_t msg = 0;
        ER er_rcv = trcv_dtq(DTQ_DRVTS, (intptr_t*)&msg, timeout);
        assert((er_rcv == E_OK) || (er_rcv == E_TMOUT));

        if (er_rcv == E_TMOUT) {
            /* タイムアウト */
            int pin1 = drvcmn_gpio_pin_get(&GPIO_PIN1);
            int pin2 = drvcmn_gpio_pin_get(&GPIO_PIN2);
            int32_t event = 0;
            if (pin1 && pin2) {
                event = DRVTS_EVT_TOUCH;
            } else if (!pin1 && !pin2) {
                event = DRVTS_EVT_RELEASE;
            } else {
                /* NOP */
            }

            syslog(LOG_NOTICE, "drvts_task() TMOUT pin1=%d, pin2=%d", pin1, pin2);

            if (event != last_event) {
                if (s_callback) {
                    syslog(LOG_NOTICE, "drvts_task() callback %d", event);
                    s_callback(event);
                }
            }
            last_event = event;
            pending_event = false;
        } else {
            /* ここでは状態を保持するだけでコールバックを行わない */
            pending_event = true;

            dly_tsk(5);	/* 少し待ってから割込みマクスを解除する */
            drvcmn_gpio_set_exti_mask(msg, false);
        }
    }
}

