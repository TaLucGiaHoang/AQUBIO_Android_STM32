/*
 * VA-X メインタスク
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/01/19 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "aplmain.h"

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"

#include "drvcmn_initialize.h"
#include "mdlauth.h"
#include "mdlstrg.h"
#include "aplauth.h"
#include "aplble.h"
#include "aplui.h"
#include "apllock.h"
#include "aplilock.h"
#include "aplcon.h"
#include "cmntimer.h"
#include "drvpwr.h"

//
// マクロ
//

// デバッグログ
#if 1
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[APLMAIN]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[APLMAIN]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[APLMAIN]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[APLMAIN]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif


//
// 型
//

typedef struct {
    int apl_id;
    APLEVT_INITIALIZE_FUNC_T initialize_func;
    APLEVT_EVENT_RECEIVER_FUNC_T event_func;
    APLEVT_EVENT_RECEIVER_FUNC_T event_func_incoming;
} APLTBL_T;

typedef int (*ACTION_FUNC_T)(intptr_t opt);

typedef struct {
    int event;
    ACTION_FUNC_T action;
    intptr_t opt;
} EVTTBL_T;

/*
 * 内部関数プロトタイプ
 */
static int deliver_event(APLEVT_EVENT_T* event);
static int32_t event_from_auth(APLEVT_EVENT_T* event);
static int32_t aplevent(int32_t apl, APLEVT_EVENT_T* event);
static void start_apps();
static int32_t event_auth(APLEVT_EVENT_T* event);
static int32_t event_ble(APLEVT_EVENT_T* event);
static int32_t event_ui(APLEVT_EVENT_T* event);
static int32_t event_con(APLEVT_EVENT_T* event);
static int32_t event_lock(APLEVT_EVENT_T* event);
static int32_t event_lilock(APLEVT_EVENT_T* event);
static void mdlstrg_callback(int event, intptr_t opt1, intptr_t opt2);

//
// 定数
//

// アプリケーション
enum {
    APL_MAIN = 0,
    APL_AUTH,
    APL_BLE,
    APL_UI,
    APL_LOCK,
    APL_LI_LOCK,
    APL_CON,
    NUM_APL,
};

// アプリケーション定義
static const APLTBL_T APLTBL[] = {
    /* apl_id,	initialize_func,	event_func,		event_func_incoming	*/
    { APL_MAIN,	NULL,				NULL,			NULL,				},
    { APL_AUTH,	aplauth_initialize, aplauth_event,	event_auth,			},
    { APL_BLE,	aplble_initialize, 	aplble_event,	event_ble,			},
    { APL_UI,	aplui_initialize, 	aplui_event,	event_ui,			},
//    { APL_LOCK,	apllock_initialize, apllock_event,	event_lock,			},
    { APL_LI_LOCK,	aplilock_initialize, aplilock_event,	event_lilock,},
    { APL_CON,	aplcon_initialize, 	aplcon_event,	event_con,			},
};


// イベント配信先フラグ
#define DEST_FLG(apl_id)	((uint_t)(0x1 << apl_id))
#define DEST_NONE			((uint_t)0)

typedef struct  {
    int	event_code;
    uint_t dest_flag;
} EVENT_DEST_T;

// イベント配信先定義
static const EVENT_DEST_T EVENT_DEST_TBL[] = {
    { APLEVT_AUTH_PREPARE_REQ,				DEST_NONE			},
    { APLEVT_AUTH_READY,					DEST_FLG(APL_UI)	},
    { APLEVT_AUTH_PROCESSING,				DEST_FLG(APL_UI)	},
//    { APLEVT_AUTH_COMPLETE,					DEST_FLG(APL_UI)|DEST_FLG(APL_LOCK)	},
    { APLEVT_AUTH_COMPLETE,					DEST_FLG(APL_UI)|DEST_FLG(APL_LI_LOCK)	},
    { APLEVT_AUTH_RETRYING,					DEST_FLG(APL_UI)	},
    { APLEVT_REGISTRATION_PREPARE_REQ,		DEST_FLG(APL_AUTH)	},
    { APLEVT_REGISTRATION_READY,			DEST_FLG(APL_UI)|DEST_FLG(APL_BLE)	},
    { APLEVT_REGISTRATION_PROCESSING,		DEST_FLG(APL_UI)|DEST_FLG(APL_BLE)	},
    { APLEVT_REGISTRATION_COMPLETE,			DEST_FLG(APL_UI)|DEST_FLG(APL_BLE)	},
    { APLEVT_STORAGE_PROCESSING,			DEST_FLG(APL_UI)	},
    { APLEVT_STORAGE_COMPLETE,				DEST_FLG(APL_UI)	},
    { APLEVT_BLE_ENABLE,					DEST_FLG(APL_UI)	},
    { APLEVT_BLE_CONNECTION_ESTABLISHED,	DEST_FLG(APL_UI)	},
    { APLEVT_BLE_START_REMOTEOP,			DEST_FLG(APL_UI)	},
    { APLEVT_BLE_END_REMOTEOP,				DEST_FLG(APL_UI)	},
    { APLEVT_BLE_DISCONNECTED,				DEST_FLG(APL_UI)	},
    { APLEVT_BLE_OPEN_LOCK,					DEST_FLG(APL_UI)	},
    { APLEVT_USER_BLE_ENABLE,				DEST_FLG(APL_UI)	},
    { APLEVT_BATTERY_LOW,					DEST_FLG(APL_UI)	},
    { APLEVT_LOCK_ERROR,					DEST_FLG(APL_UI)	},
    { APLEVT_VAREG_REQ,						DEST_FLG(APL_AUTH)	},
};

/*
 *  メインタスク
 */
void aplmain_task(intptr_t exinf)
{
	DBGLOG0("main_task() starts.");

    // マイコンのペリフェラル初期化
//    drvcmn_initialize_peripherals();

    // できるだけ早く認証可能な状態にするため, 認証機能を先に開始
    mdlauth_trigger_start();

    // 他アプリから起動されない機能をここで起動
    cmntimer_initialize();	// 共用タイマータスク
//    mdlstrg_initialize(mdlstrg_callback);	// ストレージミドル

    // 各アプリを起動
    start_apps();

    // メッセージループ
    while (true) {
        APLEVT_EVENT_T* event = NULL;
#if 0	// TODO: デモ用. 30秒でスタンバイ遷移
        ER er_rcv = aplevt_receive_event(DTQ_APLMAIN, &event, 60 * 1000);
#else
        ER er_rcv = aplevt_receive_event(DTQ_APLMAIN, &event, TMO_FEVR);
#endif
        assert((er_rcv == E_OK) || (er_rcv == E_TMOUT));

#if 1	//TODO: デモ用.
        if (er_rcv == E_TMOUT) {
            DBGLOG0("entering standby...");
            dly_tsk(10);
            drvpwr_enable_wakeup(DRVPWR_WUPIN_4_PC13, DRVPWR_WUPOL_FALLING);
//            drvpwr_enable_wakeup(DRVPWR_WUPIN_1_PA0, DRVPWR_WUPOL_FALLING);
            drvpwr_enter_standby_mode();
        }
#endif
        syslog(LOG_NOTICE, "main_task() received: (msg = %d).", (int_t) event->code);

        // APLMAINで処理するイベントはここで処理
        switch (event->code) {
#if 1	// TODO: デモ用: 認証状態に戻る
        case APLEVT_AUTH_COMPLETE:
        case APLEVT_REGISTRATION_COMPLETE:
            aplevt_send_event(APLEVT_AUTH_PREPARE_REQ, 0, NULL, aplauth_event);
            break;
#endif
        default:
            break;
        }

        // 他アプリへのイベント配信(中継)
        deliver_event(event);

        // 返却
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
int deliver_event(APLEVT_EVENT_T* event)
{
    assert(event);

    // イベント宛先情報
    const EVENT_DEST_T* event_dest = NULL;
    for (int i = 0; i < sizeof(EVENT_DEST_TBL)/sizeof(EVENT_DEST_TBL[0]); i++) {
        if (EVENT_DEST_TBL[i].event_code == event->code) {
            event_dest = &(EVENT_DEST_TBL[i]);
            break;
        }
    }
    assert(event_dest);
    if (!event_dest) {
        goto end;
    }

    // イベント配信
    for (int i = 0; i < sizeof(APLTBL)/sizeof(APLTBL[0]); i++) {
        if (APLTBL[i].apl_id == APL_MAIN) {
            continue;	// APL_MAIN は対象外
        }
        if (event_dest->dest_flag & DEST_FLG(APLTBL[i].apl_id)) {
            APLTBL[i].event_func(event);
        }
    }
    
end:
    return 0;
}

/*
 * アプリ
 */
int32_t aplevent(int32_t apl, APLEVT_EVENT_T* event)
{
    assert(event);

    ER ercd = E_OK;
    switch (event->code) {
    case APLEVT_AUTH_READY:
        ercd = aplevt_queue_event(DTQ_APLMAIN, event, TMO_POL);
        assert(ercd == E_OK);
        break;
    default:
        ercd = aplevt_queue_event(DTQ_APLMAIN, event, TMO_POL);
        assert(ercd == E_OK);
        break;
    }
    
    return 0;
}

/*
 * 認証アプリからのイベント
 */
int32_t event_auth(APLEVT_EVENT_T* event)
{
    aplevent(APL_AUTH, event);

    return 0;
}

/*
 * BLEアプリからのイベント
 */
int32_t event_ble(APLEVT_EVENT_T* event)
{
    aplevent(APL_BLE, event);

    return 0;
}

/*
 * UIアプリからのイベント
 */
int32_t event_ui(APLEVT_EVENT_T* event)
{
    aplevent(APL_UI, event);

    return 0;
}

/*
 * 錠制御アプリからのイベント
 */
int32_t event_lock(APLEVT_EVENT_T* event)
{
    aplevent(APL_LOCK, event);

    return 0;
}

/*
 * Linkey錠制御アプリからのイベント
 */
int32_t event_lilock(APLEVT_EVENT_T* event)
{
    aplevent(APL_LI_LOCK, event);

    return 0;
}

/*
 * コンソールアプリからのイベント
 */
int32_t event_con(APLEVT_EVENT_T* event)
{
    aplevent(APL_CON, event);

    return 0;
}

/*
 * アプリ起動
 */
void start_apps()
{
    
    // テーブルに登録しているアプリを順番に起動する
    for (int i = 0; i < (sizeof(APLTBL) / sizeof(APLTBL[0])); i++) {
        if (APLTBL[i].initialize_func) {
            APLTBL[i].initialize_func(APLTBL[i].event_func_incoming);
        }
    }
}

/*
 * ストレージ
 */
void mdlstrg_callback(int event, intptr_t opt1, intptr_t opt2)
{
    DBGLOG1("mdlstrg_callback: e=%d", event);
    
}

