/*
 * VA-X 錠制御アプリケーション
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/05/07 Takashi Otsuka <otsuka-takashi@bionics-k.co.jp>: 新規作成
 */

#include "aplilock.h"

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"

#include "cmntimer.h"
#include "drvlock.h"
#include "drvllock.h"

// デバッグログ
#if 1
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[APLiLOCK]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[APLiLOCK]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[APLiLOCK]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[APLiLOCK]" msg, arg1, arg2, arg3)
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
static const int CLOSE30S = 5000;
//static const int INTVAL1S = 1000;
/*
 * 内部関数プロトタイプ
 */

// イベント送信先
static APLEVT_EVENT_RECEIVER_FUNC_T s_event_dest = NULL;

/* Linkey電気錠 コールバック */
static void drvlilock_callback(int32_t evt, int32_t error);

/* タイムアウト */
static void timeout_callback(int timer_id, intptr_t otp1);

/* 処理エラータイム[ms] */
static const TMO LC_ERROR_INTERVAL = 10000;     //暫定的

// イベントフラグ
static const FLGPTN FLGPTN_DRVLLOCK_EVT_OPEN =	(0x1 << 0);
static const FLGPTN FLGPTN_DRVLLOCK_EVT_CLOSE =	(0x1 << 1);

/* ロックイベント待ち フラグ */
#define FLGPTN_DRVLLOCK_EVT_ALL (FLGPTN_DRVLLOCK_EVT_OPEN | FLGPTN_DRVLLOCK_EVT_CLOSE)
/* 施錠タイマカウント */
static const int LOCK_CNT = 1;
static int tim_cnt = 0;
// 施錠タイムアウト
static const int32_t LOCK_TIMEOUT = 4000;

static uint8_t l_header_buf[LNKY_HEADER_SIZE];
/*********************************************************************
 * 公開関数
 ********************************************************************/

/*
 * 共通初期化API(APLEVT_INITIALIZE_FUNC_T)
 */
int32_t aplilock_initialize(APLEVT_EVENT_RECEIVER_FUNC_T receiver_func)
{
    assert(!s_event_dest);
    assert(receiver_func);

    DBGLOG0("APLlilock_ini() starts.");
    
    // イベント送信先を登録
    s_event_dest = receiver_func;

    // タスクを起動
    ER er = E_OK;
    er = act_tsk(TSK_APLILOCK);
    assert(er == E_OK);

    return 0;
}

/*
 * 共通イベント受信API(APLEVT_EVENT_RECEIVER_FUNC_T)
 */
int32_t aplilock_event(APLEVT_EVENT_T* event)
{
    DBGLOG0("aplilock_event.");
    ER er = aplevt_queue_event(DTQ_APLILOCK, event, 0);
    assert(er == E_OK);

    return 0;
}

/*
 * メインタスク
 */
void aplilock_task(intptr_t exinf)
{
	DBGLOG0("aplilock_task() starts.");
    // Linkey制御ドライバをタスク起動
    drvllock_initialize(drvlilock_callback);
    //リセット→Linkey接続
    reset_cmd(drvlilock_callback, l_header_buf, LNKY_HEADER_SIZE);
    
    TMO timeout = TMO_FEVR;
    while (true) {
        APLEVT_EVENT_T* event = NULL;
        ER er_rcv = aplevt_receive_event(DTQ_APLILOCK, &event, timeout);
        assert(er_rcv == E_OK);
        syslog(LOG_NOTICE, "aplilock_task() received: (msg = %d).", event->code);

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
        //ドア状態取得
        DBGLOG0("status_mgnt_cmd .");
        status_mgnt_command(drvlilock_callback, l_header_buf, LNKY_HEADER_SIZE);
        //現状取得待ち
        FLGPTN flgptn = 0;

        ER er_wai_flg = twai_flg(FLG_APLILOCK, FLGPTN_DRVLLOCK_EVT_ALL, TWF_ORW, &flgptn, LC_ERROR_INTERVAL);
        assert((er_wai_flg == E_OK) || (er_wai_flg == E_TMOUT));
        DBGLOG0("Open_Flag00 .");
        if (er_wai_flg == E_TMOUT) {
            DBGLOG0("Open_Flag_err .");
            // 処理タイムアウト
            break;
        } else if ((er_wai_flg == E_OK) || (flgptn == FLGPTN_DRVLLOCK_EVT_OPEN)) {	// 実行フラグが立った
        // TODO: 認証のタイプ(解錠のための認証か？)と結果(認証成功か？)を見て解錠するか判断
            if (event->error == 0) {
                DBGLOG0("Open_Flag01 .");
                open_command(drvlilock_callback, l_header_buf, LNKY_HEADER_SIZE);
//                dly_tsk(5000);
//                status_dedvlt_command(l_header_buf, LNKY_HEADER_SIZE);
                //dly_tsk(6000);
                status_chek_command(drvlilock_callback, l_header_buf, LNKY_HEADER_SIZE);
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
void drvlilock_callback(int32_t evt, int32_t error)
{
    DBGLOG0("drvlilock_callback:");
    switch(evt) {
    case DRVLLOCK_EVT_CONECT_COMP:
        DBGLOG0("linkey_callback: DRVLLOCK_EVT_CONNECT_COMPLETE");
        break;
    case DRVLLOCK_EVT_OPEN_STATUS:
        DBGLOG1("error:%d", error);
        DBGLOG0("linkey_callback: DRVLLOCK_EVT_OPEN_COMPLETE");
        set_flg(FLG_APLILOCK, FLGPTN_DRVLLOCK_EVT_OPEN);
        break;
    case DRVLLOCK_EVT_CLOSE_STATUS:
        DBGLOG0("linkey_callback: DRVLLOCK_EVT_CLOSE_COMPLETE");
        set_flg(FLG_APLILOCK, FLGPTN_DRVLLOCK_EVT_CLOSE);
        break;
    case DRVLLOCK_EVT_OPEN_ERROR:
        DBGLOG0("linkey_callback: DRVLLOCK_EVT_OPEN_ERROR");
//        set_flg(FLG_APLILOCK, FLGPTN_DRVLLOCK_EVT_CLOSE);
        break;
    case DRVLLOCK_EVT_SUMTURN_OCCUR:
        DBGLOG0("linkey_callback: DRVLLOCK_EVT_SUMTURN_OCCUR");
//        set_flg(FLG_APLILOCK, FLGPTN_DRVLLOCK_EVT_CLOSE);
        break;
    case DRVLLOCK_EVT_BATERRY_ERROR:
        DBGLOG0("linkey_callback: DRVLLOCK_EVT_BATERRY_ERROR");
        break;
    case DRVLLOCK_EVT_DEDVOLT_OPEN:
        DBGLOG0("linkey_callback: DRVLLOCK_EVT_DEDVOLT_OPEN");
        break;
    case DRVLLOCK_EVT_DEDVOLT_CLOSE:
        DBGLOG0("linkey_callback: DRVLLOCK_EVT_DEDVOLT_ERROR");
        break;
    case DRVLLOCK_EVT_DOOR_OPENISR:
        DBGLOG0("linkey_callback: DRVLLOCK_EVT_DOOR_OPENISR");
        //Door開放タイマスタート(30sec)
//        cmntimer_cancell_timer(CMNTIMER_TIMER_APLLDOROPN);
        isr_cmntimer_set_timer(CMNTIMER_TIMER_APLDOROPN, CMNTIMER_TYPE_ONESHOT, CLOSE30S, timeout_callback, 0);
        break;
    case DRVLLOCK_EVT_DOOR_CLOSEISR:
        DBGLOG0("linkey_callback: DRVLLOCK_EVT_DOOR_CLOSEISR");
        //Door開放タイマキャンセル(30sec)
        isr_cmntimer_cancell_timer(CMNTIMER_TIMER_APLDOROPN);
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
    DBGLOG2("timeout_callback: e=%d cnt=%d", timer_id, tim_cnt);

    if(timer_id == CMNTIMER_TIMER_APLDOROPN){
        //ここに扉過開放処理追加
        DBGLOG0("timeout_callback: DRVLLOCK_EVT_DOOR_OPENISR");
        aplevt_send_event(APLEVT_LOCK_ERROR, 1, NULL, s_event_dest);
    }
    
    return;
}

