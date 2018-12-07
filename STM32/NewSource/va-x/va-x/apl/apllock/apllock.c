/*
 * VA-X 錠制御アプリケーション
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/05/07 Takashi Otsuka <otsuka-takashi@bionics-k.co.jp>: 新規作成
 */

#include "apllock.h"

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"

#include "cmntimer.h"
#include "drvlock.h"

// デバッグログ
#if 0
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[APLLOCK]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[APLLOCK]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[APLLOCK]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[APLLOCK]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

/*
 * 定数
 */

/*
 * 内部関数プロトタイプ
 */
static int process_event(APLEVT_EVENT_T* event);

/*
 * 定数定義
 */
static const int OPEN30S  = 30000;
static const int CLOSE30S = 30000;
static const int INTVAL1S = 1000;
/*
 * 内部変数
 */

/* TIM ステータス*/
enum {
    STOP = 0,
    START
};
static int pre_tim = 0;
/*
 * 内部関数プロトタイプ
 */

// イベント送信先
static APLEVT_EVENT_RECEIVER_FUNC_T s_event_dest = NULL;

/* 電気錠 コールバック */
static void drvlock_callback(int event, int opn_sts);

/* タイムアウト */
static void timeout_callback(int timer_id, intptr_t otp1);

/* 処理エラータイム[ms] */
static const TMO LC_ERROR_INTERVAL = 4000;     //暫定的

// イベントフラグ

static const FLGPTN FLGPTN_DRVLOCK_EVT_LOCK =	(0x1 << 0);
static const FLGPTN FLGPTN_DRVLOCK_EVT_UNLOCK =	(0x1 << 1);
static const FLGPTN FLGPTN_DRVLOCK_EVT_DOOROPN =	(0x1 << 2);
static const FLGPTN FLGPTN_DRVLOCK_EVT_DOORERR =	(0x1 << 3);
static const FLGPTN FLGPTN_DRVLOCK_EVT_NEUTRAL =	(0x1 << 4);
static const FLGPTN FLGPTN_DRVLOCK_EVT_INTMDET =	(0x1 << 5);

/* ロックイベント待ち フラグ */
#define FLGPTN_DRVLOCK_EVT_ALL (FLGPTN_DRVLOCK_EVT_LOCK | FLGPTN_DRVLOCK_EVT_UNLOCK | FLGPTN_DRVLOCK_EVT_DOOROPN | \
    FLGPTN_DRVLOCK_EVT_DOORERR | FLGPTN_DRVLOCK_EVT_NEUTRAL | FLGPTN_DRVLOCK_EVT_INTMDET)
/* 施錠タイマカウント */
static const int LOCK_CNT = 1;
static int tim_cnt = 0;
/*********************************************************************
 * 公開関数
 ********************************************************************/

/*
 * 共通初期化API(APLEVT_INITIALIZE_FUNC_T)
 */
int32_t apllock_initialize(APLEVT_EVENT_RECEIVER_FUNC_T receiver_func)
{
    assert(!s_event_dest);
    assert(receiver_func);

    DBGLOG0("APLlock_ini() starts.");
    
    // イベント送信先を登録
    s_event_dest = receiver_func;

    // タスクを起動
    ER er = E_OK;
    er = act_tsk(TSK_APLLOCK);
    assert(er == E_OK);

    return 0;
}

/*
 * 共通イベント受信API(APLEVT_EVENT_RECEIVER_FUNC_T)
 */
int32_t apllock_event(APLEVT_EVENT_T* event)
{
    DBGLOG0("apllock_event.");
    ER er = aplevt_queue_event(DTQ_APLLOCK, event, 0);
    assert(er == E_OK);

    return 0;
}

/*
 * メインタスク
 */
void apllock_task(intptr_t exinf)
{
	DBGLOG0("apllock_task() starts.");
    // 鍵ドライバを初期化
    drvlock_initialize(drvlock_callback);

    TMO timeout = TMO_FEVR;
    while (true) {
        APLEVT_EVENT_T* event = NULL;
        ER er_rcv = aplevt_receive_event(DTQ_APLLOCK, &event, timeout);
        assert(er_rcv == E_OK);
        syslog(LOG_NOTICE, "aplui_task() received: (msg = %d).", event->code);

        // イベントを処理
        process_event(event);

        // イベントを返却
        aplevt_return_event(event);
        event = NULL;
    }
}

/*********************************************************************
 * 内部関数
 ********************************************************************/

/*
 * イベント処理
 */
int process_event(APLEVT_EVENT_T* event)
{
    int result = 0;

    switch(event->code) {
    case APLEVT_AUTH_COMPLETE:
    case APLEVT_BLE_OPEN_LOCK:
        // 認証OKによる解錠
        drvlock_get_state();        //現状取得
        //現状取得待ち
        FLGPTN flgptn = 0;
        ER ercd = clr_flg(FLG_APLLOCK, ~FLGPTN_DRVLOCK_EVT_ALL);
        assert(ercd == E_OK);
        ER er_wai_flg = twai_flg(FLG_APLLOCK, FLGPTN_DRVLOCK_EVT_ALL, TWF_ORW, &flgptn, LC_ERROR_INTERVAL);
        assert((er_wai_flg == E_OK) || (er_wai_flg == E_TMOUT));
        
        if (er_wai_flg == E_TMOUT) {
        // 処理タイムアウト
            break;
        } else if ((er_wai_flg == E_OK) || (flgptn == FLGPTN_DRVLOCK_EVT_LOCK)) {	// 実行フラグが立った
        // TODO: 認証のタイプ(解錠のための認証か？)と結果(認証成功か？)を見て解錠するか判断
            if (event->error == 0) {
                drvlock_open();
                DBGLOG0("Open_Flag .");
            }
            break;
        } else {
            assert(false);
        }
        break;
    default:
        result = -1;
        break;
    }

    return result;
}

/*
 * 電気錠コールバック
 *///for debug
void drvlock_callback(int event, int evt_sts)
{
    DBGLOG2("drvlock_callback: evt=%d sts=%d", event, evt_sts);
    
    switch(event) {
    case DRVLOCK_EVT_LOCK:
        if((evt_sts == DOR_ERR)||(evt_sts == MTR_ERR)){
            aplevt_send_event(APLEVT_LOCK_ERROR, 3, NULL, s_event_dest);
        }
        pre_tim = STOP;
        set_flg(FLG_APLLOCK, FLGPTN_DRVLOCK_EVT_LOCK);
        break;
    case DRVLOCK_EVT_UNLOCK://Door close
        DBGLOG0("drvlock_callback: DRVLOCK_EVT_UNLOCK");
        //施錠タイマスタート(30sec) 解錠状態維持
        if(pre_tim == STOP){
            tim_cnt = 0;
            cmntimer_cancell_timer(CMNTIMER_TIMER_APLLLOCK);
            cmntimer_set_timer(CMNTIMER_TIMER_APLLLOCK, CMNTIMER_TYPE_ONESHOT, OPEN30S, timeout_callback, 0);
            pre_tim = START;
        }
        if(evt_sts == OPN_EVTS){
            //1secINTVL タイマスタート
            cmntimer_cancell_timer(CMNTIMER_TIMER_APLLONEINT);
            cmntimer_set_timer(CMNTIMER_TIMER_APLLONEINT, CMNTIMER_TYPE_ONESHOT, INTVAL1S, timeout_callback, 0);
            //Door開放タイマキャンセル
            cmntimer_cancell_timer(CMNTIMER_TIMER_APLLDOROPN);
        }else if((evt_sts == DOR_ERR)||(evt_sts == MTR_ERR)){
            cmntimer_cancell_timer(CMNTIMER_TIMER_APLLLOCK);
            aplevt_send_event(APLEVT_LOCK_ERROR, 3, NULL, s_event_dest);
        }
        break;
    case DRVLOCK_EVT_DOOROPN:
            DBGLOG0("drvlock_callback: DRVLOCK_EVT_DOOROPN");
            //Door開放タイマスタート(30sec)
            cmntimer_cancell_timer(CMNTIMER_TIMER_APLLDOROPN);
            cmntimer_set_timer(CMNTIMER_TIMER_APLLDOROPN, CMNTIMER_TYPE_ONESHOT, CLOSE30S, timeout_callback, 0);
            //施錠タイマキャンセル
            cmntimer_cancell_timer(CMNTIMER_TIMER_APLLLOCK);
        break;
#if 0
        case DRVLOCK_EVT_DOORCLS:
        DBGLOG0("drvlock_callback: DRVLOCK_EVT_DOORCLS");
        break;
#endif
    case DRVLOCK_EVT_DOORERR:
        DBGLOG0("drvlock_callback: DRVLOCK_EVT_DOORERR");
        aplevt_send_event(APLEVT_LOCK_ERROR, 2, NULL, s_event_dest);
        break;
    case DRVLOCK_EVT_NEUTRAL:
        DBGLOG0("drvlock_callback: DRVLOCK_EVT_NEUTRAL");
        break;
    case DRVLOCK_EVT_INTMDET:
        DBGLOG0("drvlock_callback: DRVLOCK_EVT_INTMDET");
        break;
        default:
        assert(false);
        break;
    }

}

/*
 * タイムアウト
 */
void timeout_callback(int timer_id, intptr_t otp1)
{
    // 解錠後、10秒ドアを開けなかった
    // ドアを閉めたあと1秒INTVL
    //   施錠
    DBGLOG2("timeout_callback: e=%d cnt=%d", timer_id, tim_cnt);
    if((timer_id == CMNTIMER_TIMER_APLLLOCK) || (timer_id == CMNTIMER_TIMER_APLLONEINT)){
        if(timer_id == CMNTIMER_TIMER_APLLONEINT){
            cmntimer_cancell_timer(CMNTIMER_TIMER_APLLONEINT);
            drvlock_close();
        }else if (timer_id == CMNTIMER_TIMER_APLLLOCK){
            if(tim_cnt >= LOCK_CNT){
                cmntimer_cancell_timer(CMNTIMER_TIMER_APLLLOCK);
                tim_cnt = 0;
            }else{
                drvlock_close();
                tim_cnt++;
                pre_tim = STOP;
            }
        }
    }else if(timer_id == CMNTIMER_TIMER_APLLDOROPN){
        //ここに扉過開放処理追加
        DBGLOG0("timeout_callback: DRVLOCK_EVT_DOOROPN");
        aplevt_send_event(APLEVT_LOCK_ERROR, 1, NULL, s_event_dest);
    }
    
    return;
}

