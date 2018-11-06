/*
 * VA-X
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/13 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "drvpbt.h"

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

// デバッグログ
#if 0
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[DRVPBT]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[DRVPBT]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[DRVPBT]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[DRVPBT]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

#define NUM_BUTTONS 1

/*
 * 型定義
*/

typedef struct {
    int last_event;
    int prev_state;
    SYSTIM timeout;
} DRVPBT_CONTEXT_T;

/*
 * 定数定義
 */

/* GPIOポート */
static const DRVCMN_GPIO_PIN_T GPIO_PINS[] = {
    { DRVCMN_GPIO_PORT_B,	9,	},	/* PB9 */
};
static_assert(sizeof(GPIO_PINS) / sizeof(GPIO_PINS[0]) == NUM_BUTTONS, "NUM_BUTTONS");

/* ピン設定 */
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING = {
    .mode = DRVCMN_GPIO_MODE_INPUT,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLUP,
    .exti = DRVCMN_GPIO_EXTI_BOTHTRIG,
    .exti_isr = drvpbt_pushbutton_isr,
};

/* チャタリング時間[ms] */
static const TMO CHATTER_TIMEOUT = 50;

/*
 * 内部変数
 */
static DRVPBT_CBFUNC_T s_callback = NULL;
//static DRVPBT_CONTEXT_T s_contexts[NUM_BUTTONS] = {0};

/*
 * ペリフェラル初期化
 */
void drvpbt_initialize_peripherals()
{
    /* GPIO ピンの設定 */
    for (int i = 0; i < sizeof(GPIO_PINS) / sizeof(GPIO_PINS[0]); i++) {
        drvcmn_gpio_pin_initialize(&(GPIO_PINS[i]), &GPIO_SETTING);
    }
}

/*
 * ドライバ初期化
 */
void drvpbt_initialize(DRVPBT_CBFUNC_T callback)
{
    assert(callback);
    
    /* コールバックを設定 */
    s_callback = callback;

    /* 割込み処理タスクを起動 */
    ER er = E_OK;
    er = act_tsk(TSK_DRVPBT);
    assert(er == E_OK);
}

/*
 * 割込みサービスルーチン
 */
int32_t drvpbt_pushbutton_isr(uint32_t pinno)
{
    DBGLOG1("isr %d", pinno);

    if (!s_callback) {
        return 0;
    }

    int32_t msg = 0;
    if (drvcmn_gpio_pin_get(&GPIO_PINS[0])) {
        msg = DRVPBT_EVT_PRESS;
    } else {
        msg = DRVPBT_EVT_RELEASE;
    }

    ER er = ipsnd_dtq(DTQ_DRVPBT, msg);
    assert(er == E_OK);

    return 0;
}

/*
 * 割込み処理タスク
 */
void drvpbt_task(intptr_t exinf)
{
	DBGLOG0("drvpbt_task() starts.");

    int pending_event = 0;
    while (true) {
        TMO timeout = 0;
        if (pending_event != 0) {
            timeout = CHATTER_TIMEOUT;
        } else {
            timeout = TMO_FEVR;
        }

        int32_t msg = 0;
        ER er_rcv = trcv_dtq(DTQ_DRVPBT, (intptr_t*)&msg, timeout);
        assert((er_rcv == E_OK) || (er_rcv == E_TMOUT));

        if (er_rcv == E_TMOUT) {
            /* 最後の割込み発生後一定時間経過してからピンの状態を調べて上位側に通知する(チャタリング対策) */
            int event = 0;
            if (drvcmn_gpio_pin_get(&GPIO_PINS[0])) {
                event = DRVPBT_EVT_PRESS;
            } else {
                event = DRVPBT_EVT_RELEASE;
            }

            if (s_callback) {
                s_callback(0, pending_event);
                if (event != pending_event) {
                    s_callback(0, event);
                }
            }
            pending_event = 0;
        } else {
            /* ここでは状態を保持するだけでコールバックを行わない(チャタリング対策) */
            switch (msg) {
            case DRVPBT_EVT_PRESS:		/* プレス */
            case DRVPBT_EVT_RELEASE:	/* リリース */
                pending_event = msg;
                break;
            default:
                assert(false);
                break;
            }
        }
    }
}

