/*
 * VA-X 生体センサードライバ 
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/11/24 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 * - 2018/04/06 Takashi Otsuka <otsuka-takashi@bionics-k.co.jp>: CY8CMBR3102用への追加変更
 * - 2018/04/16 Takashi Otsuka <otsuka-takashi@bionics-k.co.jp>: 追加変更
 */

#include "drvts.h"

#include <string.h>
#include <kernel.h>
#include <sil.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "stm32f7xx.h"

#include "kernel_cfg.h"
#include "drvcmn_gpio.h"
#include "drvcmn_i2c.h"

/*
 * マクロ定義
 */

// デバッグログ
#if 0
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[DRVTS]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[DRVTS]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[DRVTS]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[DRVTS]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

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

/* GPIOポート1(TS1) */
static const DRVCMN_GPIO_PIN_T GPIO_TS1 = {	/* B10 */
    DRVCMN_GPIO_PORT_B, 10,
};

/* GPIOポート2(TS2) */
static const DRVCMN_GPIO_PIN_T GPIO_TS2 = {	/* B11 */
    DRVCMN_GPIO_PORT_B, 11,
};

/* チャタリング時間[ms] */
static const TMO CHATTER_TIMEOUT = 50;

/* GPIO ピンの設定 */
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING = {
    .mode = DRVCMN_GPIO_MODE_INPUT,
    .pupd = DRVCMN_GPIO_PUPD_PULLUP,
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
void drvts_initialize_peripherals()
{
    drvcmn_gpio_pin_initialize(&GPIO_TS1, &GPIO_SETTING);
    drvcmn_gpio_pin_initialize(&GPIO_TS2, &GPIO_SETTING);

    return;
}

/* 生体センサー開始 ドライバタスク起動*/
int32_t drvts_initialize()
{
    DBGLOG0("drvts_initialize .");
/*
    ER er = 0;
    er = act_tsk(TSK_DRVTS);
    assert(er == E_OK);
*/
    return 0;
}

/* 生体センサー開始 ドライバタスク起動*/
int32_t drvts_start(DRVTS_CALLBACK_T callback)
{
    assert(callback);
    DBGLOG0("drvts_starts .");
    /* コールバックを設定 */
    s_callback = callback;

    ER er = 0;
    er = act_tsk(TSK_DRVTS);
    assert(er == E_OK);
    
    return 0;
}

/* 生体センサー停止 */
int32_t drvts_stop()
{
    s_callback = NULL;
    DBGLOG0("drvts_stop .");
    
    ER er = 0;
    er = ter_tsk(TSK_DRVTS);

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
//	syslog(LOG_NOTICE, "drvts_task() starts .");
    DBGLOG0("drvts_task() starts .");
    
    bool_t pending_event = false;
    int32_t last_event = -1;
    int32_t event = -1;

    while (true) {
        TMO timeout = 0;
        if (pending_event) {
            timeout = CHATTER_TIMEOUT;
        } else {
            timeout = TMO_FEVR;
        }
        DBGLOG2("1_last_event=%d, event=%d", last_event, event);
        intptr_t msg = 0;
        ER er_rcv = trcv_dtq(DTQ_DRVTS, (intptr_t*)&msg, timeout);
        assert((er_rcv == E_OK) || (er_rcv == E_TMOUT));

        if (er_rcv == E_TMOUT) {
            /* タイムアウト */
            int pin1 = drvcmn_gpio_pin_get(&GPIO_TS1);
            int pin2 = drvcmn_gpio_pin_get(&GPIO_TS2);
            pin1 = pin1 ^ 1;
            pin2 = pin2 ^ 1;
            DBGLOG2("drvts_task() TMOUT pin1=%d, pin2=%d", pin1, pin2);
            
#if 0		//debug用
            if(pin1 && pin2){				//消える
				aplui_dbg_set_led_pattern(1, 1);	//B
				aplui_dbg_set_led_pattern(2, 1);	//G
				aplui_dbg_set_led_pattern(0, 1); //R
            }else if(!pin1 && !pin2){	//緑　on
				aplui_dbg_set_led_pattern(1, 1);
				aplui_dbg_set_led_pattern(2, 2);
				aplui_dbg_set_led_pattern(0, 1);
            }else if(pin1){				//青　on
				aplui_dbg_set_led_pattern(1, 2);
				aplui_dbg_set_led_pattern(2, 1);
				aplui_dbg_set_led_pattern(0, 1);
            }else if(pin2){				//赤　on
				aplui_dbg_set_led_pattern(1, 1);
				aplui_dbg_set_led_pattern(2, 1);
				aplui_dbg_set_led_pattern(0, 2);
            }else{
            	/* NOP */
            }
#endif
            if (pin1 && pin2) {
                event = DRVTS_EVT_TOUCH;
                DBGLOG0("DRVTS_EVT_TOUCH .");
            } else if (!pin1 && !pin2) {
                event = DRVTS_EVT_RELEASE;
                DBGLOG0("DRVTS_EVT_RELEASE .");
            } else {
                /* NOP */
            }
            DBGLOG2("2_last_event=%d, event=%d", last_event, event);
//            syslog(LOG_NOTICE, "drvts_task() TMOUT pin1=%d, pin2=%d", pin1, pin2);
            DBGLOG2("drvts_task() TMOUT pin1=%d, pin2=%d", pin1, pin2);
            
            if (event != last_event) {
                if (s_callback) {
//                    syslog(LOG_NOTICE, "drvts_task() callback %d", event);
                    DBGLOG1("drvts_task() callback %d", event);
                    s_callback(event);
                }
            }
            last_event = event;
            DBGLOG2("3_last_event=%d, event=%d", last_event, event);
            pending_event = false;
        } else {
            /* ここでは状態を保持するだけでコールバックを行わない */
            pending_event = true;

            dly_tsk(5);	/* 少し待ってから割込みマクスを解除する */
            drvcmn_gpio_set_exti_mask(msg, false);
        }
    }
}

