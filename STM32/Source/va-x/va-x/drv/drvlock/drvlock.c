/*
 * VA-X 錠ドライバ
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
  * - 2018/04/20 Takashi Otsuka <otsuka-takashi@bionics-k.co.jp>: 新規作成
 */

#include "drvlock.h"

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
#if 1
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[DRVLOCK]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[DRVLOCK]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[DRVLOCK]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[DRVLOCK]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

/*
 * 型定義
*/

/*
 * 定数定義
 */

/* GPIOポート 入力側 CS基板時設定*/
#if 1
static const DRVCMN_GPIO_PIN_T GPIO_PINS_INPUT[] = {
    { DRVCMN_GPIO_PORT_A,	0,	},	/* PA0  DOOR HALL */
    { DRVCMN_GPIO_PORT_C,	1,	},	/* PC1  LOCKING */
    { DRVCMN_GPIO_PORT_A,	2,	},	/* PA2  OPEN LOCK */
    { DRVCMN_GPIO_PORT_C,	4,	},	/* PC4  NEUTRAL */
};
#else
static const DRVCMN_GPIO_PIN_T GPIO_PINS_INPUT[] = {
    { DRVCMN_GPIO_PORT_E,	11,	},// Door sensor
    { DRVCMN_GPIO_PORT_E,	12,	},// locking
    { DRVCMN_GPIO_PORT_E,	13,	},// open lock
    { DRVCMN_GPIO_PORT_E,	14,	},// neutral
};
#endif

/* ピン設定 入力側*/
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_INPUT = {
    .mode = DRVCMN_GPIO_MODE_INPUT,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLUP,
    .exti = DRVCMN_GPIO_EXTI_BOTHTRIG,
    .exti_isr = drvlock_sensor_isr
};

/* GPIOポート 出力側 CS基板時設定*/
#if 1
static const DRVCMN_GPIO_PIN_T GPIO_PINS_GPOUT[] = {
    { DRVCMN_GPIO_PORT_D,	4,	},	/* PD4 MTR_ON*/
    { DRVCMN_GPIO_PORT_D,	5,	},	/* PD5 MTR_PLS*/
    { DRVCMN_GPIO_PORT_D,	6,	},	/* PD6 MTR_MNS*/
};
#else
static const DRVCMN_GPIO_PIN_T GPIO_PINS_GPOUT[] = {
    { DRVCMN_GPIO_PORT_E,	7,	},
    { DRVCMN_GPIO_PORT_E,	8,	},
    { DRVCMN_GPIO_PORT_D,	12,	},
};
#endif
/* ピン設定 出力側*/
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_GPOUT = {
    .mode = DRVCMN_GPIO_MODE_GPOUT,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLDOWN,
};

/* 割込み PIN番号 */
enum {
    INTRPT_DRSNS = 0,   //ドアセンサ
    INTRPT_UNLK = 1,   //解錠
    INTRPT_LOCK = 2,   //施錠
    INTRPT_NUM3 = 5,
    INTRPT_NUET = 4    //鍵ニュートラル
    
};
// メッセージ番号
enum {
    MSG_GETSTATE = 1,
    MSG_OPENLOCK,
    MSG_LOCKING,
};

/* GPIO ステータス*/
enum {
    LOW = 0,
    HIGH
};

/*
 * 内部関数
 */
static void lock_centor_seq(uint8_t lock);

/*
 * 内部変数
 */
static DRVLOCK_CBFUNC_T s_callback = NULL;
static uint8_t pre_event = -1;
static int evt_sts = OKEY_EVTS;

// イベントフラグ
static const FLGPTN FLGPTN_LOCK_REQ =			(0x1 << 0);
static const FLGPTN FLGPTN_UNLOCK_REQ =			(0x1 << 1);
static const FLGPTN FLGPTN_NEUT_REQ =			(0x1 << 2);
static const FLGPTN FLGPTN_DOOR_REQ =			(0x1 << 3);

/* ウェイトタイム[ms] */
static const TMO LC_WATING_INTERVAL = 2000;

/*
 * ペリフェラル初期化
 */
void drvlock_initialize_peripherals()
{
    DBGLOG0("drvlock_ini_per_state .");
    /* GPIO 入力ピンの設定 */
    for (int i = 0; i < sizeof(GPIO_PINS_INPUT) / sizeof(GPIO_PINS_INPUT[0]); i++) {
        drvcmn_gpio_pin_initialize(&(GPIO_PINS_INPUT[i]), &GPIO_SETTING_INPUT);
    }

    /* GPIO 出力ピンの設定 */
    for (int i = 0; i < sizeof(GPIO_PINS_GPOUT) / sizeof(GPIO_PINS_GPOUT[0]); i++) {
        drvcmn_gpio_pin_initialize(&(GPIO_PINS_GPOUT[i]), &GPIO_SETTING_GPOUT);
        drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[i]), false);				// Lowにする
    }
}

/*
 * ドライバ初期化
 */
void drvlock_initialize(DRVLOCK_CBFUNC_T callback)
{
    assert(callback);
    
    /* コールバックを設定 */
    s_callback = callback;

    /* 割込み処理タスクを起動 */
    ER er = E_OK;
    er = act_tsk(TSK_DRVLOCK);
    assert(er == E_OK);
}

/*
 * 鍵錠 状態検出
 */
void drvlock_get_state()
{
    
    DBGLOG0("drvlock_get_state .");

    ER er = snd_dtq(DTQ_DRVLOCK, (intptr_t)MSG_GETSTATE);
    assert(er == E_OK);
}

/*
 * 鍵錠 解除要求
 */
void drvlock_open()
{
    
    DBGLOG0("drvlock_open .");

    ER er = snd_dtq(DTQ_DRVLOCK, (intptr_t)MSG_OPENLOCK);
    assert(er == E_OK);
}

/*
 * 鍵錠 施錠要求
 */
void drvlock_close()
{
    
    DBGLOG0("drvlock_close .");

    ER er = snd_dtq(DTQ_DRVLOCK, (intptr_t)MSG_LOCKING);
    assert(er == E_OK);
}
/*
 * 鍵錠GPIO 割込みサービスルーチン
 */
int32_t drvlock_sensor_isr(uint32_t pinno)
{
    DBGLOG1("sensor_isr %d", pinno);

    if (!s_callback) {
        return 0;
    }
    int32_t intrpnum = 0; 
    intrpnum = pinno;

    //解錠、施錠信号割込み待ち
    if((intrpnum == INTRPT_UNLK) || (intrpnum == INTRPT_LOCK)){
        if ((drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[1]) == LOW) &&    //施錠
            (drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[2]) == HIGH)){
            DBGLOG0("sensor_isr_locking. ");
            ER ercd = iset_flg(FLG_DRVLOCK, FLGPTN_LOCK_REQ);
            assert(ercd == E_OK);
                
        }else if((drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[1]) == HIGH) &&    //解錠
            (drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[2]) == LOW)){
            DBGLOG0("sensor_isr_opnlck. ");
            ER ercd = iset_flg(FLG_DRVLOCK, FLGPTN_UNLOCK_REQ);
            assert(ercd == E_OK);
        }
    //ニュートラル信号割込み待ち
    }else if(intrpnum == INTRPT_NUET){
        if (!(drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[3]))) {
            DBGLOG0("sensor_isr_neutral. ");
            ER ercd = iset_flg(FLG_DRVLOCK, FLGPTN_NEUT_REQ);
            assert(ercd == E_OK);
        }
    //ドアセンサ割込み待ち
    }else  if(intrpnum == INTRPT_DRSNS){
        if (!(drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[0]))) {
            DBGLOG0("sensor_isr_doorsns. ");
            ER ercd = iset_flg(FLG_DRVLOCK, FLGPTN_DOOR_REQ);
            assert(ercd == E_OK);
        }
    }
    ER er = ipsnd_dtq(DTQ_DRVLOCK, (intptr_t)MSG_GETSTATE);
    assert(er == E_OK);
    return 0;
}

/*
 * 割込み処理タスク
 */
void drvlock_task(intptr_t exinf)
{
	DBGLOG0("drvlock_task() starts.");
    int event = 0;
    int opn_sts = 0;
    
    while (true) {
        intptr_t msg = 0;
        ER er_rcv = trcv_dtq(DTQ_DRVLOCK, (intptr_t*)&msg, TMO_FEVR);
        assert((er_rcv == E_OK) || (er_rcv == E_TMOUT));
        DBGLOG1("drvlock_task() msg=%d", msg);
        
        switch (msg) {
        case MSG_GETSTATE:
        {   //施錠
            if ((drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[1]) == LOW) &&    //施錠
            (drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[2]) == HIGH) &&
            (drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[0]) == LOW)){
                DBGLOG0("EVT_LOCK.");
                event = DRVLOCK_EVT_LOCK;
            //解錠
            }else if((drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[1]) == HIGH) &&    //解錠
            (drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[2]) == LOW) &&
            (drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[0]) == LOW)){
                DBGLOG0("EVT_UNLOCK.");
                event = DRVLOCK_EVT_UNLOCK;
                if(pre_event == DRVLOCK_EVT_DOOROPN){
                    evt_sts = OPN_EVTS;
                    pre_event = DRVLOCK_EVT_UNLOCK;
                }
            //Door open
            }else if((drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[1]) == HIGH) &&
            (drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[2]) == LOW) &&
            (drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[0]) == HIGH)){
                event = DRVLOCK_EVT_DOOROPN;
                pre_event = DRVLOCK_EVT_DOOROPN;
            //Door error
            }else if((drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[1]) == LOW) &&
            (drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[2]) == HIGH) &&
            (drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[0]) == HIGH)){
                event = DRVLOCK_EVT_DOORERR;
#if 0
                //ニュートラル
            }else if(drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[3]) == LOW){
                event = DRVLOCK_EVT_NEUTRAL;
            //インターミディエート

            }else if((drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[1]) == LOW) &&
            (drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[2]) == LOW)){
                event = DRVLOCK_EVT_INTMDET;
#endif
            }
            
            s_callback(event, evt_sts);
            DBGLOG1("drvlock_sts_call() msg=%d", event);
//            s_callback = NULL;
            break;
        }
        case MSG_OPENLOCK:
        {
            //Motor駆動
            if(event == DRVLOCK_EVT_LOCK){
                drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[1]), false);				// MTR_MNS Off
                drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[2]), false);				// MTR_MNS On
                DBGLOG0("MTR_ON_UNLOCK. ");
                drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[0]), true);				// MTR_ON PowerOn
                dly_tsk(1);														// 動作間状態待ちディレイ
                drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[1]), true);				// MTR_PLS On
                drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[2]), false);				// MTR_MNS Off

                dly_tsk(1);														// 動作間状態待ちディレイ
                // 割込みによる停止フラグが立つかタイムアウトするまで
                FLGPTN flgptn = 0;
                ER ercd = clr_flg(FLG_DRVLOCK, ~FLGPTN_UNLOCK_REQ);
                assert(ercd == E_OK);
                ER er_wai_flg = twai_flg(FLG_DRVLOCK, FLGPTN_UNLOCK_REQ, TWF_ANDW, &flgptn, LC_WATING_INTERVAL);
                assert((er_wai_flg == E_OK) || (er_wai_flg == E_TMOUT));

                if (er_wai_flg == E_TMOUT) {    //モータ動作エラー処理
                    DBGLOG0("Error Sequence. ");
                    evt_sts = DOR_ERR;
                    lock_centor_seq(1);												//センター位置に移動
                } else if (er_wai_flg == E_OK) {
                    DBGLOG0("Brake start. ");
                    //一時惰性モード
                    drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[2]), false);				// MTR_MNS Off
                    dly_tsk(10);													// 動作間状態待ちディレイ
                    //ブレーキ
                    drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[1]), true);				// MTR_PLS On
                    drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[2]), true);				// MTR_MNS On
                    dly_tsk(100);													//ブレーキ状態待ち
                    lock_centor_seq(1);												//センター位置に移動
                } else {
                    assert(false);
                }
            }
            //for debug
            s_callback(event, evt_sts);
            evt_sts = OKEY_EVTS;
            DBGLOG1("drvlock_opn_call() msg=%d", event);
            break;
        }
        case MSG_LOCKING:
        {
            //Motor駆動
            if(event == DRVLOCK_EVT_UNLOCK){
                drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[1]), false);				// MTR_MNS Off
                drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[2]), false);				// MTR_MNS On
                DBGLOG0("MTR_ON. ");
                drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[0]), true);				// MTR_ON PowerOn
                dly_tsk(1);
                drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[1]), false);				// MTR_PLS Off
                drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[2]), true);				// MTR_MNS On
                dly_tsk(1);
                // 割込みによる停止フラグが立つかタイムアウトするまで
                FLGPTN flgptn = 0;
                ER ercd = clr_flg(FLG_DRVLOCK, ~FLGPTN_LOCK_REQ);
                assert(ercd == E_OK);
                ER er_wai_flg = twai_flg(FLG_DRVLOCK, FLGPTN_LOCK_REQ, TWF_ANDW, &flgptn, LC_WATING_INTERVAL);
                assert((er_wai_flg == E_OK) || (er_wai_flg == E_TMOUT));

                if (er_wai_flg == E_TMOUT) {    //モータ動作エラー処理
                    DBGLOG0("Error Sequence. ");
                    evt_sts = DOR_ERR;
                    lock_centor_seq(0);												//センター位置に移動
                } else if (er_wai_flg == E_OK) {
                    DBGLOG0("Brake start. ");
                    //一時惰性モード
                    drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[1]), false);				// MTR_MNS Off
                    dly_tsk(10);
                    //ブレーキ
                    drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[1]), true);				// MTR_PLS On
                    dly_tsk(1);
                    drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[2]), true);				// MTR_MNS On
                    dly_tsk(1);
                    dly_tsk(100);													//ブレーキ状態待ち
                    lock_centor_seq(0);												//センター位置に移動
                } else {
                    assert(false);
                }
            
            }
            //for debug
            s_callback(event, evt_sts);
            DBGLOG1("drvlock_cls_call() msg=%d", event);
            evt_sts = OKEY_EVTS;
            break;
        }
        default:
            assert(false);
            break;
        }
    }
}
/*
 * 電気錠のモータをセンター位置に移動
 */
void lock_centor_seq(uint8_t lock)
{
    DBGLOG0("lock_centor_seq. ");
    if(lock)
    {   //解錠時
        drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[1]), false);				// MTR_PLS On
        dly_tsk(1);
        drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[2]), true);				// MTR_MNS On
        dly_tsk(1);
    }else{
        drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[1]), true);				// MTR_PLS On
        dly_tsk(1);
        drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[2]), false);				// MTR_MNS On
        dly_tsk(1);
    }
    // 割込みによる停止フラグが立つかタイムアウトするまで
    FLGPTN flgptn = 0;
    ER ercd = clr_flg(FLG_DRVLOCK, ~FLGPTN_NEUT_REQ);
    assert(ercd == E_OK);
    ER er_wai_flg = twai_flg(FLG_DRVLOCK, FLGPTN_NEUT_REQ, TWF_ANDW, &flgptn, LC_WATING_INTERVAL);
    assert((er_wai_flg == E_OK) || (er_wai_flg == E_TMOUT));

    if (er_wai_flg == E_TMOUT) {    //モータ動作エラー処理
        DBGLOG0("Motor Error. ");
        evt_sts = MTR_ERR;
    } else if (er_wai_flg == E_OK) {
    
        //一時惰性モード
        if(lock)
        {   //解錠時
            drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[1]), false);				// MTR_PLS Off
        }else{
            drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[2]), false);				// MTR_MNS Off
        }
        dly_tsk(10);
        //ブレーキ
        drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[1]), true);				// MTR_PLS On
        drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[2]), true);				// MTR_MNS On
        dly_tsk(100);													//ブレーキ状態待ち
        
        drvcmn_gpio_pin_set(&(GPIO_PINS_GPOUT[0]), false);				// MTR_ON PowerOff
        dly_tsk(1);
    } else {
        assert(false);
    }
}

