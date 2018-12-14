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
#include "aplicc.h"
#include "aplwifi.h"
#include "cmntimer.h"
#include "drvpwr.h"
#include "drvllock.h"
#include "cmndbg.h"
#include "mdlble.h"
#include "mdlstrg.h"
#include "mdlstrg_data.h"

//
// マクロ
//

// デバッグログ
#if 0
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
static int32_t event_icc(APLEVT_EVENT_T* event);
static int32_t event_wifi(APLEVT_EVENT_T* event);
static void mdlstrg_callback(int event, intptr_t opt1, intptr_t opt2);

static void print_now_time(APLEVT_EVENT_T* event);

static const FLGPTN FLGPTN_MDLSTRG_REQUEST_COMPLETE =			(0x1 << 0);

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
    APL_ICC,
    APL_WIFI,
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
    { APL_ICC,	aplicc_initialize, 	aplicc_event,	event_icc,			},
//    { APL_WIFI, aplwifi_initialize, aplwifi_event,  event_wifi,         },
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
    { APLEVT_AUTH_ERROR,					DEST_FLG(APL_UI)	},
    { APLEVT_TS_DETECTED,					DEST_FLG(APL_UI)	},
    { APLEVT_UI_PBT_STOP,					DEST_FLG(APL_UI)	},
    { APLEVT_REGISTRATION_PREPARE_REQ,		DEST_FLG(APL_UI)|DEST_FLG(APL_AUTH)	},
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
    { APLEVT_LINKEY_READY,					DEST_FLG(APL_UI)	},
    { APLEVT_LINKEY_COMPLETE,				DEST_FLG(APL_UI)	},
    { APLEVT_REG_CARD_REG_VACANCY_READY,	DEST_FLG(APL_UI)	},
    { APLEVT_REG_CARD_REG_VACANCY_COMPLETE,	DEST_FLG(APL_UI)	},
    { APLEVT_REG_CARD_VACANCY_READY,	    DEST_FLG(APL_UI)	},
    { APLEVT_REG_CARD_VACANCY_COMPLETE,	    DEST_FLG(APL_UI)	},
    { APLEVT_REREG_CARD_REG_VACANCY_READY,  DEST_FLG(APL_UI)	},
    { APLEVT_REREG_CARD_REG_VACANCY_COMPLETE,DEST_FLG(APL_UI)	},
    { APLEVT_VAREG_REQ,						DEST_FLG(APL_AUTH)	},
    { APLEVT_ICC_START_REQ,					DEST_FLG(APL_ICC)	},
    { APLEVT_ICC_START_COMPLETE,			DEST_FLG(APL_MAIN)	},
    { APLEVT_ICC_STOP_REQ,					DEST_FLG(APL_ICC)	},
    { APLEVT_ICC_STOP_COMPLETE,				DEST_FLG(APL_MAIN)	},
    { APLEVT_ICC_AUTH_REQ,					DEST_FLG(APL_ICC)	},
    { APLEVT_ICC_AUTH_COMPLETE,				DEST_FLG(APL_BLE)|(APL_UI)|DEST_FLG(APL_LI_LOCK)	},
    { APLEVT_ICC_REGAUTH_COMPLETE,			DEST_FLG(APL_BLE)|(APL_UI)},
    { APLEVT_ICC_REGIST_REQ,				DEST_FLG(APL_ICC)	},
    { APLEVT_ICC_REGIST_COMPLETE,			DEST_FLG(APL_BLE)	},
    { APLEVT_ICC_DELETE_REQ,				DEST_FLG(APL_ICC)	},
    { APLEVT_ICC_DELETE_COMPLETE,			DEST_FLG(APL_MAIN)	},
    { APLEVT_ICC_DEPARTURE_REQ,				DEST_FLG(APL_ICC)	},
    { APLEVT_ICC_DEPARTURE_COMPLETE,		DEST_FLG(APL_MAIN)	},
    { APLEVT_ICC_REREGIST_REQ,				DEST_FLG(APL_ICC)	},
    { APLEVT_ICC_REREGIST_COMPLETE,			DEST_FLG(APL_MAIN)	},
    { APLEVT_ICC_INITIALIZE,			    DEST_FLG(APL_ICC)	},
    { APLEVT_ICC_POLLING_COMPLETE,			DEST_FLG(APL_MAIN)	},
    { APLEVT_ICC_INTERNAL_POLLING,			DEST_FLG(APL_ICC)	},
//    { APLEVT_ICC_AUTH_COMPLETE,				DEST_FLG(APL_UI)|DEST_FLG(APL_LI_LOCK)	},
    { APLEVT_WIFI_START_REQ,                DEST_FLG(APL_WIFI)  },
    { APLEVT_WIFI_READY_COMPLETE,           DEST_FLG(APL_MAIN)  },
    { APLEVT_WIFI_STOP_REQ,                 DEST_FLG(APL_WIFI)  },
    { APLEVT_WIFI_STOP_COMPLETE,            DEST_FLG(APL_MAIN)  },
    { APLEVT_WIFI_CHECK_SERVERTASK_REQ,     DEST_FLG(APL_WIFI)  },
    { APLEVT_WIFI_CHECK_SERVERTASK_COMPLETE,DEST_FLG(APL_MAIN)  },
    { APLEVT_WIFI_UPLOAD_OPE_LOG_REQ,       DEST_FLG(APL_WIFI)  },
    { APLEVT_WIFI_UPLOAD_OPE_LOG_COMPLETE,  DEST_FLG(APL_MAIN)  },
    { APLEVT_WIFI_UPLOAD_ERR_LOG_REQ,       DEST_FLG(APL_WIFI)  },
    { APLEVT_WIFI_UPLOAD_ERR_LOG_COMPLETE,  DEST_FLG(APL_MAIN)  },
    { APLEVT_WIFI_DOWNLOAD_VACANCY_CARD_REQ,DEST_FLG(APL_WIFI)  },
    { APLEVT_WIFI_DOWNLOAD_VACANCY_CARD_COMPLETE,DEST_FLG(APL_MAIN)  },
};

// 日付データ表示用バッファ
static uint8_t extra_date_buff[APLEVT_EXTRA_DATE_SIZE+1];

//タスク間フラグ
//カードタスクフラグ
static int icc_tsk = 0;

//BLE(スマホ)タスクフラグ
static int ble_tsk = 0;

//認証タスクフラグ
static int auth_tsk = 0;

/* BLE実行中フラグ */
static int ble_process;
/*
//工事 ステータスフラグ 初期値：1
static int cst_sts = 0; 

//空室 ステータスフラグ
static int vac_sts = 0;

//Droom ステータスフラグ
static int drm_sts = 1;
*/
/* ROOM識別情報 */
//static MDLSTRG_DATA_ROOM_STS_T room_sts = {0};
static MDLSTRG_DATA_ROOM_STS_T room_sts;

static const int STANDBY_TIME = 60;
/*
 *  メインタスク
 */
void aplmain_task(intptr_t exinf)
{
	DBGLOG0("main_task() starts.");
    ER er = E_OK;
    ER er_rcv = E_OK;
    // マイコンのペリフェラル初期化
    drvcmn_initialize_peripherals();

    // できるだけ早く認証可能な状態にするため, 認証機能を先に開始
//    mdlauth_trigger_start();

    // 他アプリから起動されない機能をここで起動
    cmntimer_initialize();	// 共用タイマータスク
    mdlstrg_initialize(mdlstrg_callback);	// ストレージミドル

    // 各アプリを起動
    start_apps();

    // 部屋ステータスフラグ読み込み
    MDLSTRG_REQUEST_T ROOM_STS_REQ = {
        .data_type = MDLSTRG_DATA_TYPE_ROOM_TYPE,        // 
        .request_type = MDLSTRG_REQ_TYPE_READ,
        .data = (intptr_t)&room_sts,
    };
    clr_flg(FLG_APLMAIN, ~FLGPTN_MDLSTRG_REQUEST_COMPLETE);
    mdlstrg_request(&ROOM_STS_REQ, mdlstrg_callback);
    er = twai_flg(FLG_APLMAIN, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
    assert(er == E_OK);

//    cmndbg_hexdump(&room_sts.cst_sts, sizeof(room_sts), "device_sts_read");
    
    // メッセージループ
    while (true) {
        DBGLOG0("before entering standby...");
        DBGLOG1("ble_process：0x%x", ble_process);
        APLEVT_EVENT_T* event = NULL;
// TODO: デモ用. 30秒でスタンバイ遷移
        if(ble_process == true){
            er_rcv = aplevt_receive_event(DTQ_APLMAIN, &event, TMO_FEVR);
            assert((er_rcv == E_OK) || (er_rcv == E_TMOUT));
            DBGLOG0("entering standby...TMO_FEVR");

}else{
            er_rcv = aplevt_receive_event(DTQ_APLMAIN, &event, STANDBY_TIME * 1000);
            assert((er_rcv == E_OK) || (er_rcv == E_TMOUT));
            DBGLOG0("entering standby...STANDBY_TIME");
        }

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
        case APLEVT_AUTH_ERROR:
            DBGLOG0("Main to APLEVT_AUTH_ERROR");
            aplevt_send_event(APLEVT_AUTH_PREPARE_REQ, 0, NULL, aplauth_event);
            break;
        case APLEVT_REGISTRATION_COMPLETE:
            DBGLOG0("Main to APLEVT_AUTH_PREPARE_REQ");
            aplevt_send_event(APLEVT_AUTH_PREPARE_REQ, 0, NULL, aplauth_event);
            break;
        case APLEVT_ICC_POLLING_COMPLETE:
            DBGLOG0("Main to APLEVT_AUTH_CANCEL");
            aplevt_send_event(APLEVT_AUTH_CANCEL, 0, NULL, aplauth_event);
            aplevt_send_event(APLEVT_UI_PBT_STOP, 0, NULL, aplui_event);
            break;
        case APLEVT_AUTH_PROCESSING:
            DBGLOG0("Main to APLEVT_AUTH_PROCESSING");
            aplevt_send_event(APLEVT_ICC_STOP_REQ, 0, NULL, aplicc_event);
            aplevt_send_event(APLEVT_UI_PBT_STOP, 0, NULL, aplui_event);
            break;
        case APLEVT_BLE_ENABLE:
            DBGLOG0("Main to APLEVT_BLE_ENABLE");
            aplevt_send_event(APLEVT_ICC_STOP_REQ, 0, NULL, aplicc_event);
            aplevt_send_event(APLEVT_AUTH_CANCEL, 0, NULL, aplauth_event);
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
        DBGLOG0("!event_dest");
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
    ER er = E_OK;
    
    APLEVT_EXTRA_DATA_T extra_data;
    //タスク間フラグチェックシーケンス

#if 0	// 工事ステータスフラグセット
    {
        room_sts.cst_sts = false;
        DBGLOG1("room_sts: 0x%x", room_sts.cst_sts);
        MDLSTRG_REQUEST_T ROOM_STS_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_ROOM_TYPE,
            .request_type = MDLSTRG_REQ_TYPE_WRITE,
            .data = (intptr_t)&room_sts,
            .size = sizeof(room_sts.cst_sts),
        };
        mdlstrg_request(&ROOM_STS_REQ, mdlstrg_callback);
        er = twai_flg(FLG_APLMAIN, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 2000);
        assert(er == E_OK);
        
        cmndbg_hexdump(&room_sts.cst_sts, sizeof(room_sts.cst_sts), "cst_sts_write");
    }
#endif
    
    //タスク間フラグセットシーケンス
    switch (event->code) {
        case APLEVT_AUTH_PROCESSING:
        DBGLOG0("case_APLEVT_AUTH_PROCESSING");
        DBGLOG1("event->extra_data.reg_prep_req.index：0x%x", event->extra_data.reg_prep_req.index);
        ble_process = event->extra_data.reg_prep_req.index;
        //血流フラグ以外を"1"の禁止状態へ
        if((auth_tsk == false) || (ble_process == true)){
            DBGLOG0("case_fls_APLEVT_AUTH_PROCESSING");
            aplevt_send_event(APLEVT_AUTH_PROCESSING, 0, NULL, aplauth_event);
            icc_tsk  = true;
            ble_tsk  = true;
            auth_tsk  = false;
            
        }else{
            DBGLOG0("case_tre_APLEVT_AUTH_PROCESSING");
            goto end;
        }
        break;

        case APLEVT_ICC_POLLING_COMPLETE:
            //カードフラグ以外を"1"の禁止状態へ
        DBGLOG0("APLEVT_ICC_POLLING_COMPLETE_tsk_flag_on");
        if(icc_tsk == false){
            auth_tsk = true;
            ble_tsk  = true;
            
        }else{
            goto end;
        }
        break;

        case APLEVT_BLE_ENABLE:
            //BLEフラグ以外を"1"の禁止状態へ
        DBGLOG0("case_APLEVT_BLE_ENABLE");
        DBGLOG1("event->extra_data.reg_prep_req.index：0x%x", event->extra_data.reg_prep_req.index);
        ble_process = event->extra_data.reg_prep_req.index;
        if(ble_tsk == false){
            aplevt_send_event(APLEVT_BLE_ENABLE, 0, NULL, aplble_event);
            auth_tsk = true;
            icc_tsk  = true;
            
        }else{
            goto end;
        }
        break;

        case APLEVT_REG_CARD_VACANCY_READY:
            DBGLOG0("case_tre_APLEVT_REG_CARD_VACANCY_READY");
            //空室カード登録
            room_sts.vac_sts = true;
            room_sts.drm_sts = false;
            
            DBGLOG1("room_sts: 0x%x", room_sts.drm_sts);
            MDLSTRG_REQUEST_T ROOM_STS_WREQ = {
                .data_type = MDLSTRG_DATA_TYPE_ROOM_TYPE,
                .request_type = MDLSTRG_REQ_TYPE_WRITE,
                .data = (intptr_t)&room_sts,
                .size = sizeof(room_sts),
            };
            mdlstrg_request(&ROOM_STS_WREQ, mdlstrg_callback);
            er = twai_flg(FLG_APLMAIN, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 2000);
            assert(er == E_OK);
        break;

        default:
        break;
    }
    
    switch (apl) {
        
    case APL_ICC: {
        // アプリイベント受信（カード機能動作確認用）
        switch (event->code) {
        case APLEVT_ICC_START_COMPLETE: {
            switch (event->error) {
            case 0:
                DBGLOG0("APLEVT_ICC_START_COMPLETE, OK");
                break;
            default:
                DBGLOG1("APLEVT_ICC_START_COMPLETE : error = %d", event->error);
                break;
            }
            break;
            }
        case APLEVT_ICC_STOP_COMPLETE: {
            switch (event->error) {
            case 0:
                DBGLOG0("APLEVT_ICC_STOP_COMPLETE, OK");
                break;
            default:
                DBGLOG1("APLEVT_ICC_STOP_COMPLETE : error = %d", event->error);
                break;
            }
            break;
            }
        case APLEVT_ICC_AUTH_COMPLETE: {
            switch (event->error) {
            case 0:
                DBGLOG0("APLEVT_ICC_AUTH_COMPLETE, OK");
                break;
            case APLICC_ERROR_READ_CARD:
                DBGLOG0("APLEVT_ICC_AUTH_COMPLETE, APLICC_ERROR_READ_CARD");
                break;
            case APLICC_ERROR_AUTH:
                DBGLOG0("APLEVT_ICC_AUTH_COMPLETE, APLICC_ERROR_AUTH");
                break;
            case APLICC_ERROR_IDENTIFY_CODE:
                DBGLOG0("APLEVT_ICC_AUTH_COMPLETE, APLICC_ERROR_IDENTIFY_CODE");
                break;
            case APLICC_ERROR_WRITE_CARD:
                DBGLOG0("APLEVT_ICC_AUTH_COMPLETE, APLICC_ERROR_WRITE_CARD");
                break;
            case APLICC_ERROR_NO_REGIST_CARD:
                DBGLOG0("APLEVT_ICC_AUTH_COMPLETE, APLICC_ERROR_NOREGIST_CARD");
                break;
            case APLICC_ERROR_TIMEOUT:
                DBGLOG0("APLEVT_ICC_AUTH_COMPLETE, APLICC_ERROR_TIMEOUT");
                break;
            case APLICC_ERROR_DRVICC:
                DBGLOG0("APLEVT_ICC_AUTH_COMPLETE, APLICC_ERROR_DRVICC");
                break;
            default:
                DBGLOG1("APLEVT_ICC_AUTH_COMPLETE : error = %d", event->error);
                break;
            }
            break;
            }
        case APLEVT_ICC_REGIST_COMPLETE: {
            switch (event->error) {
            case 0:
                DBGLOG0("APLEVT_ICC_REGIST_COMPLETE, OK ForDroom");
                room_sts.vac_sts = false;
                room_sts.drm_sts = true;
                
                DBGLOG1("room_sts: 0x%x", room_sts.drm_sts);
                MDLSTRG_REQUEST_T ROOM_STS_WREQ = {
                    .data_type = MDLSTRG_DATA_TYPE_ROOM_TYPE,
                    .request_type = MDLSTRG_REQ_TYPE_WRITE,
                    .data = (intptr_t)&room_sts,
                    .size = sizeof(room_sts),
                };
                mdlstrg_request(&ROOM_STS_WREQ, mdlstrg_callback);
                er = twai_flg(FLG_APLMAIN, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 2000);
                assert(er == E_OK);
                //cmndbg_hexdump(event->extra_data.reg_prep_req.member_num, sizeof(event->extra_data.reg_prep_req.member_num), "member_num");
                break;
            case APLICC_ERROR_READ_CARD:
                DBGLOG0("APLEVT_ICC_REGIST_COMPLETE, APLICC_ERROR_READ_CARD");
                break;
            case APLICC_ERROR_WRITE_CARD:
                DBGLOG0("APLEVT_ICC_REGIST_COMPLETE, APLICC_ERROR_WRITE_CARD");
                break;
            case APLICC_ERROR_TIMEOUT:
                DBGLOG0("APLEVT_ICC_REGIST_COMPLETE, APLICC_ERROR_TIMEOUT");
                break;
            case APLICC_ERROR_MAX_CNT:
                DBGLOG0("APLEVT_ICC_REGIST_COMPLETE, APLICC_ERROR_MAX_CNT");
                break;
            case APLICC_ERROR_DRVICC:
                DBGLOG0("APLEVT_ICC_REGIST_COMPLETE, APLICC_ERROR_DRVICC");
                break;
            default:
                DBGLOG1("APLEVT_ICC_REGIST_COMPLETE : error = %d", event->error);
                break;
            }
//            cmndbg_hexdump(event->extra_data.reg_prep_req.member_num, sizeof(event->extra_data.reg_prep_req.member_num), "member_num");
            // 確認用：LED（緑）
            aplevt_send_event(APLEVT_AUTH_READY, 0, NULL, event_icc);
            break;
            }
        case APLEVT_ICC_DELETE_COMPLETE: {
            switch (event->error) {
            case 0:
                DBGLOG0("APLEVT_ICC_DELETE_COMPLETE, OK");
                break;
            default:
                DBGLOG1("APLEVT_ICC_DELETE_COMPLETE : error = %d", event->error);
                break;
            }
            break;
            }
        case APLEVT_ICC_DEPARTURE_COMPLETE: {
            switch (event->error) {
            case 0:
                DBGLOG0("APLEVT_ICC_DEPARTURE_COMPLETE, OK");
                break;
            default:
                DBGLOG1("APLEVT_ICC_DEPARTURE_COMPLETE : error = %d", event->error);
                break;
            }
            break;
            }
        case APLEVT_ICC_REREGIST_COMPLETE: {
            switch (event->error) {
            case 0:
                DBGLOG0("APLEVT_ICC_REREGIST_COMPLETE, OK");
//                cmndbg_hexdump(event->extra_data.reg_prep_req.member_num, sizeof(event->extra_data.reg_prep_req.member_num), "member_num");
                break;
            case APLICC_ERROR_READ_CARD:
                DBGLOG0("APLEVT_ICC_REREGIST_COMPLETE, APLICC_ERROR_READ_CARD");
                break;
            case APLICC_ERROR_WRITE_CARD:
                DBGLOG0("APLEVT_ICC_REREGIST_COMPLETE, APLICC_ERROR_WRITE_CARD");
                break;
            case APLICC_ERROR_TIMEOUT:
                DBGLOG0("APLEVT_ICC_REREGIST_COMPLETE, APLICC_ERROR_TIMEOUT");
                break;
            case APLICC_ERROR_DRVICC:
                DBGLOG0("APLEVT_ICC_REREGIST_COMPLETE, APLICC_ERROR_DRVICC");
                break;
            default:
                DBGLOG1("APLEVT_ICC_REREGIST_COMPLETE : error = %d", event->error);
                break;
            }
            // 確認用：LED（緑）
            aplevt_send_event(APLEVT_AUTH_READY, 0, NULL, event_icc);
            break;
            }
        case APLEVT_ICC_POLLING_COMPLETE: {
            switch (event->error) {
            case 0:
                DBGLOG0("APLEVT_ICC_POLLING_COMPLETE, OK");
                if(icc_tsk == false){
//                    extra_data.reg_prep_req.index = APLICC_CARD_DROOM1;
//                    aplevt_send_event(APLEVT_ICC_AUTH_REQ, 0, &extra_data, aplicc_event);
                    DBGLOG2("APLEVT_ICC_POLLING_COMPLETE cst:%d, vac:%d:", room_sts.cst_sts, room_sts.vac_sts);
                    if(room_sts.cst_sts == true){
                        DBGLOG0("APLICC_CARD_CONSTRUCTION_AUTH_REQ");
                        extra_data.reg_prep_req.index = APLICC_CARD_CONSTRUCTION;
                        aplevt_send_event(APLEVT_ICC_AUTH_REQ, 0, &extra_data, aplicc_event);
                    }else if(room_sts.vac_sts == true){
                        DBGLOG0("APLICC_CARD_VACANCY_AUTH_REQ");
                        extra_data.reg_prep_req.index = APLICC_CARD_VACANCY;
                        aplevt_send_event(APLEVT_ICC_AUTH_REQ, 0, &extra_data, aplicc_event);
                    }else if(room_sts.drm_sts == true){
                        DBGLOG0("APLICC_CARD_DROOM1_AUTH_REQ");
                        extra_data.reg_prep_req.index = APLICC_CARD_DROOM1;
                        aplevt_send_event(APLEVT_ICC_AUTH_REQ, 0, &extra_data, aplicc_event);
                    }

//                aplevt_send_event(APLEVT_AUTH_CANCEL, 0, NULL, aplauth_event);
                }
                break;
            default:
                DBGLOG1("APLEVT_ICC_POLLING_COMPLETE : error = %d", event->error);
                break;
            }
            break;
            }
        }
        break;
    }
    case APL_WIFI:
        switch (event->code) {
        case APLEVT_WIFI_READY_COMPLETE: {
            switch (event->error) {
            case 0:
                DBGLOG0("APLEVT_WIFI_READY_COMPLETE, OK");
                break;
            default:
                DBGLOG1("APLEVT_WIFI_READY_COMPLETE : error = %d", event->error);
                break;
            }
            break;
        }
        case APLEVT_WIFI_STOP_COMPLETE: {
            switch (event->error) {
            case 0:
                DBGLOG0("APLEVT_WIFI_STOP_COMPLETE, OK");
                break;
            default:
                DBGLOG1("APLEVT_WIFI_STOP_COMPLETE : error = %d", event->error);
                break;
            }
            break;
        }
        case APLEVT_WIFI_CHECK_SERVERTASK_COMPLETE: {
            print_now_time(event);
            switch (event->error) {
            case 0:
                DBGLOG0("APLEVT_WIFI_CHECK_SERVERTASK_COMPLETE, OK");
                break;
            default:
                DBGLOG1("APLEVT_WIFI_CHECK_SERVERTASK_COMPLETE : error = %d", event->error);
                break;
            }
            break;
        }
        case APLEVT_WIFI_UPLOAD_ERR_LOG_COMPLETE: {
            print_now_time(event);
            switch (event->error) {
            case 0:
                DBGLOG0("APLEVT_WIFI_UPLOAD_ERR_LOG_COMPLETE, OK");
                break;
            default:
                DBGLOG1("APLEVT_WIFI_UPLOAD_ERR_LOG_COMPLETE : error = %d", event->error);
                break;
            }
            break;
        }
        case APLEVT_WIFI_UPLOAD_OPE_LOG_COMPLETE: {
            print_now_time(event);
            switch (event->error) {
            case 0:
                DBGLOG0("APLEVT_WIFI_UPLOAD_OPE_LOG_COMPLETE, OK");
                break;
            default:
                DBGLOG1("APLEVT_WIFI_UPLOAD_OPE_LOG_COMPLETE : error = %d", event->error);
                break;
            }
            break;
        }
        default:
            break;
        }
        break;
    }

    switch (event->code) {
    case APLEVT_AUTH_READY:
        DBGLOG0("APLEVT_AUTH_READY");
        ercd = aplevt_queue_event(DTQ_APLMAIN, event, TMO_POL);
        assert(ercd == E_OK);
        break;
    default:
        DBGLOG0("NONE_APLEVT_AUTH_READY");
        ercd = aplevt_queue_event(DTQ_APLMAIN, event, TMO_POL);
        assert(ercd == E_OK);
        DBGLOG1("event->extra_data.reg_prep_req.index：0x%x", event->extra_data.reg_prep_req.index);
        ble_process = event->extra_data.reg_prep_req.index;
        break;
    }
    
//    cmndbg_hexdump(&room_sts.cst_sts, sizeof(room_sts), "cst_sts_write");
//フラグ判定での終了処理
end:
    
    return 0;
}

/*
 * サーバー取得日時を表示
 */
static void print_now_time(APLEVT_EVENT_T* event)
{
    memcpy(extra_date_buff, event->extra_data.wifi_data.date, APLEVT_EXTRA_DATE_SIZE);
    extra_date_buff[APLEVT_EXTRA_DATE_SIZE] = '\0';
    DBGLOG1("NOW_TIME : %s", extra_date_buff);
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
 * カード制御アプリからのイベント
 */
int32_t event_icc(APLEVT_EVENT_T* event)
{
    aplevent(APL_ICC, event);

    return 0;
}

/*
 * WIFI制御アプリからのイベント
 */
int32_t event_wifi(APLEVT_EVENT_T* event)
{
    aplevent(APL_WIFI, event);

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
 * ストレージミドルコールバック
 */
void mdlstrg_callback(int event, intptr_t opt1, intptr_t opt2)
{
    DBGLOG1("mdlstrg_callback_main: e=%d", event);
    switch(event) {
    case MDLSTRG_EVT_REQUEST_COMPLETE:
        DBGLOG0("mdlstrg_callback: MDLSTRG_EVT_REQUEST_COMPLETE");
        set_flg(FLG_APLMAIN, FLGPTN_MDLSTRG_REQUEST_COMPLETE);
        break;
    default:
        DBGLOG1("mdlstrg_callback: e=%d", event);
        break;
    }
}
