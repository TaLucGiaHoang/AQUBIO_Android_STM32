/*
 * VA-X カードアプリケーション
 *
 * Copyright (C) 2018 Bionics co.ltd.
 *
 * - 2018/09/01 新規作成
 */

#include "aplicc.h"

#include <string.h>
#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include <stdio.h>

#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"

#include "cmntimer.h"
#include "drvicc.h"
#include "cmndbg.h"
#include "drvrng.h"

#include "mdlstrg.h"
#include "mdlstrg_data.h"

// デバッグログ
#if 0
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[APLICC]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[APLICC]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[APLICC]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[APLICC]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

/*
 * 型定義
 */
/* 識別コード管理データ */
typedef struct {
    MDLSTRG_DATA_IDCODE_T id_code;
    uint32_t address;
} APLICC_IDENTIFY_CODE_T;


/*
 * 定数
 */
static const FLGPTN FLGPTN_DRVICC_INITIALIZE_COMPLETE =	(0x1 << 0);
static const FLGPTN FLGPTN_DRVICC_FINALIZE_COMPLETE =	(0x1 << 1);
static const FLGPTN FLGPTN_DRVICC_WRITE_COMPLETE =		(0x1 << 2);
static const FLGPTN FLGPTN_MDLSTRG_REQ_COMPLETE =		(0x1 << 3);

/*
 * 内部関数プロトタイプ
 */
static int process_event(APLEVT_EVENT_T* event);
static void aplicc_start();
static void aplicc_stop();
static void aplicc_start_polling(APLEVT_EVENT_T* event);
static void aplicc_restart_polling();
static void aplicc_timeout_callback(int timer_id, intptr_t otp1);
static int polling_complete_event();
static void aplicc_read_flash();
static void aplicc_read_card();
static int aplicc_auth_card();
static int aplicc_check_card();
static void set_status_flg(uint8_t *status_flg);
static int aplicc_regist_card();
static void aplicc_delete_card();
static void aplicc_departure();
static void aplicc_regist_vacancy_card(APLEVT_EVENT_T* event);
static void aplicc_reregist_vacancy_card(APLEVT_EVENT_T* event);
static void aplicc_all_delete_vacancy_card();
static void aplicc_retry_polling();
static void aplicc_self_aplicc_stop();
static int32_t aplicc_stop_and_send_event(uint32_t code, int32_t error, const APLEVT_EXTRA_DATA_T* extra_data, APLEVT_EVENT_RECEIVER_FUNC_T receiver);

/*
 * 定数定義
 */
#define ICC_SERVICE_CODE_CNT 3
/* Service Code */
static const uint16_t ICC_SERVICE_CODE[ICC_SERVICE_CODE_CNT] = {0x110B, 0x2009, 0x300B};
/* 券種 */
static const uint8_t ICC_CARD_TYPE_CODE[APLICC_NUM_CARD][2] = {{0x39, 0x39}, {0x30, 0x35}, {0x30, 0x34}, {0x30, 0x33}, {0x30, 0x31}, {0x30, 0x32}};

/* 工事カード カード種別 */
static const uint8_t ICC_CONSTRUCTION_CODE = 0x39;

/* 識別コード管理データフラグ */
static const uint16_t IDENTIFY_CODE_NOUSE = 0x0000;
static const uint16_t IDENTIFY_CODE_INIT = 0xFFFF;
static const uint16_t IDENTIFY_CODE_NEW = 0x7FFF;
static const uint16_t IDENTIFY_CODE_OLD = 0x4FFF;

/* 識別コード管理データ */
#define ICC_NUM_ID_CODE_MNG 10
static MDLSTRG_DATA_IDENTIFY_CODE_MNG_T id_code_mng[ICC_NUM_ID_CODE_MNG];

/* D-roomカード認証時の新しい識別コード */
static MDLSTRG_DATA_IDCODE_T new_id_code;

/*
 * 内部変数
 */
enum {
    STOP = 0,
    START,
    STOP_RECV,
};
/* カードRW ステータス*/
static int icc_status = 0;
/* ポーリング ステータス*/
static int polling_status = 0;
/* カード書き込み ステータス*/
static int write_status = 0;

/* Service Code読込回数 */
static int icc_read_cnt = 0;
/* ポーリング タイムアウト時間 */
static int polling_timeout = 0;
/* ポーリング中 イベント */
static int polling_event_code = 0;

/* カード情報 */
static DRVICC_FELICA_IDM_T idm;
static DRVICC_FELICA_BLOCK_DESC_T desc = {
    .sys = 0x8CCE,
    .service = 0x2009,
    .block = 0,
};

/* カード読込回数 */
// D-roomカードは6回読み込む、それ以外のカードは2回読み込む
#define ICC_DROOM_READ_CNT 6
#define ICC_NO_DROOM_READ_CNT 2

/* カードから読み込んだ情報 */
// D-roomカード     0:カード種別, 1-4:識別コード, 5:会員番号
// D-roomカード以外 0:カード種別, 1:会員番号
static DRVICC_FELICA_BLOCK_T block[ICC_DROOM_READ_CNT];

/* D-roomカード登録時のカードINDEX */
static int card_index;

/* D-roomカード登録時の名称（使用者） */
static uint8_t user_name[APLEVT_EXTRA_USER_NAME_SIZE];

/* 一括登録カード */
static MDLSTRG_DATA_CARD_T info_vacancy_regist;

/* 空室カード */
static MDLSTRG_DATA_CARD_T info_vacancy[APLICC_VACANCY_MAX];

/* 登録・抹消カード */
static MDLSTRG_DATA_CARD_T info_regist;

/* D-roomカード */
static MDLSTRG_DATA_DROOM_CARD_T info_droom[APLICC_DROOM_MAX];

/* カードドライバからのエラー */
static int32_t drvicc_error = 0;

/* ミドルストレージのリクエストステート */
static intptr_t s_strgmdl_req_state = 0;

/* 読込中の券種 */
static int reading_type;

/* REQイベントの券種 */
static int reading_type_req;

/* 一括登録カードが登録済か */
static bool_t exists_vacancy_regist = false;

/* 空室カード登録枚数 */
static int vacancy_card_cnt = 0;

/* 内部処理ののカード制御中止 */
static bool_t self_drv_stop = false;

/* 初期処理中か */
static bool_t icc_initializing = false;

/* メインに返却するパラメータ */
static APLEVT_EXTRA_DATA_T res_extra_data;

/*
 * 内部関数プロトタイプ
 */

// イベント送信先
static APLEVT_EVENT_RECEIVER_FUNC_T s_event_dest = NULL;

/* カードドライバ コールバック */
static void drvicc_callback(int32_t event, int32_t error, intptr_t data);

/* ミドルストレージ コールバック */
static void strg_callback(int event, intptr_t opt1, intptr_t opt2);

/*********************************************************************
 * 公開関数
 ********************************************************************/

/*
 * 共通初期化API(APLEVT_INITIALIZE_FUNC_T)
 */
int32_t aplicc_initialize(APLEVT_EVENT_RECEIVER_FUNC_T receiver_func)
{
    assert(!s_event_dest);
    assert(receiver_func);

    DBGLOG0("APLicc_ini() starts.");
    
    // イベント送信先を登録
    s_event_dest = receiver_func;

    // タスクを起動
    ER er = E_OK;
    er = act_tsk(TSK_APLICC);
    assert(er == E_OK);

    return 0;
}

/*
 * 共通イベント受信API(APLEVT_EVENT_RECEIVER_FUNC_T)
 */
int32_t aplicc_event(APLEVT_EVENT_T* event)
{
    DBGLOG0("aplicc_event.");
    ER er = aplevt_queue_event(DTQ_APLICC, event, 0);
    assert(er == E_OK);

    return 0;
}

/*
 * メインタスク
 */
void aplicc_task(intptr_t exinf)
{
    DBGLOG0("aplicc_task() starts.");

    icc_initializing = true;
    ER er = clr_flg(FLG_APLICC, ~FLGPTN_DRVICC_INITIALIZE_COMPLETE);
    assert(er == E_OK);
    // カードドライバを初期化(カードドライバタスクを起動)
    drvicc_initialize(drvicc_callback);
    FLGPTN flgptn = 0;
    er = twai_flg(FLG_APLICC, FLGPTN_DRVICC_INITIALIZE_COMPLETE, TWF_ANDW, &flgptn, 2000);
    assert(er == E_OK);
    
    if (drvicc_error == 0) {
        APLEVT_EVENT_T* event = NULL;
        aplevt_create_event(&event, APLEVT_ICC_INITIALIZE, 0, NULL);
        // ポーリング
        aplicc_start_polling(event);
    }

    icc_initializing = false;
    TMO timeout = TMO_FEVR;
    while (true) {
        APLEVT_EVENT_T* event = NULL;
        ER er_rcv = aplevt_receive_event(DTQ_APLICC, &event, timeout);
        assert(er_rcv == E_OK);
        DBGLOG2("aplicc_task() received: (msg = %d) (index = %d).", event->code, event->extra_data.reg_prep_req.index);
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
    bool_t exists = false;
    ER er = E_OK;

    switch(event->code) {
    case APLEVT_ICC_START_REQ:
        DBGLOG0("APLEVT_ICC_START_REQ");
        aplicc_start();
        break;
    case APLEVT_ICC_STOP_REQ:
        DBGLOG0("APLEVT_ICC_STOP_REQ");
        aplicc_stop();
        break;
    case APLEVT_ICC_AUTH_REQ:
        aplicc_start();             // カードRWがOFFのときはカード起動
        if (drvicc_error != 0) {
            aplevt_send_event(APLEVT_ICC_AUTH_COMPLETE, APLICC_ERROR_DRVICC, NULL, s_event_dest);
            break;
        }
        
        reading_type_req = event->extra_data.reg_prep_req.index;
        reading_type = reading_type_req;
        polling_timeout = 0;
        if (reading_type == APLICC_CARD_REGIST) { // 登録・抹消カード
            polling_timeout = event->extra_data.reg_prep_req.timeout;
        }
        DBGLOG2("APLEVT_ICC_AUTH_REQ: index:%d timeout:%d", reading_type, polling_timeout);
        
        if (reading_type != APLICC_CARD_CONSTRUCTION) {
            // 工事カード以外の場合、フラッシュからカードデータ取得
            aplicc_read_flash();
            if (reading_type == APLICC_CARD_REGIST) { // 登録・抹消カード
                exists = s_strgmdl_req_state;
                
                DBGLOG1("REG_CARD_READ_REQ exists: %d", exists);
                if (!exists) {
                    // 登録・抹消カードが登録されてない場合は登録なしをメインに返す
                    aplevt_send_event(APLEVT_ICC_AUTH_COMPLETE, APLICC_ERROR_NO_REGIST_CARD, NULL, s_event_dest);
                    break;
                }
            }
        }

        if (reading_type == APLICC_CARD_DROOM1 || reading_type == APLICC_CARD_DROOM2) {
            memset(id_code_mng, 0, sizeof(id_code_mng));
            // 入居の場合、識別コード管理データ作成
            MDLSTRG_REQUEST_T CREATE_ID_CODE_MNG_DATA_REQ = {
                .data_type = MDLSTRG_DATA_TYPE_IDCODE,
                .request_type = MDLSTRG_REQ_TYPE_READ,
                .data = (intptr_t)&id_code_mng,
            };
            mdlstrg_request(&CREATE_ID_CODE_MNG_DATA_REQ, strg_callback);
            er = twai_flg(FLG_APLICC, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
            assert(er == E_OK);
            
            // debug用
            DBGLOG0("CREATE_ID_CODE_MNG_DATA_REQ");
            for(int i=0; i < 10; i++){
                cmndbg_hexdump(&id_code_mng[i], sizeof(id_code_mng[i]), "id_code_mng");
            }
        }
        // ポーリング
        aplicc_start_polling(event);
        break;
    case APLEVT_ICC_REGIST_REQ:
        aplicc_start();             // カードRWがOFFのときはカード起動
        if (drvicc_error != 0) {
            aplevt_send_event(APLEVT_ICC_REGIST_COMPLETE, APLICC_ERROR_DRVICC, NULL, s_event_dest);
            break;
        }
        
        reading_type_req = event->extra_data.reg_prep_req.index;
        reading_type = reading_type_req;
        polling_timeout = event->extra_data.reg_prep_req.timeout;
        DBGLOG2("APLEVT_ICC_REGIST_REQ: index:%d timeout:%d", reading_type, polling_timeout);
        
        if (reading_type == APLICC_CARD_VACANCY) {
            // 空室カード登録
            aplicc_regist_vacancy_card(event);
            break;
        }

        // D-roomカード登録のパラメータ設定
        card_index = event->extra_data.reg_prep_req.card_index;
        memset(user_name, 0, sizeof(user_name));
        memcpy(user_name, event->extra_data.reg_prep_req.user_name, sizeof(user_name));
        // D-roomカード/登録・抹消カード ポーリング
        aplicc_start_polling(event);
        break;
    case APLEVT_ICC_DELETE_REQ:
        card_index = event->extra_data.reg_prep_req.card_index;
        DBGLOG1("APLEVT_ICC_DELETE_REQ: index:%d", card_index);
        aplicc_delete_card();
        break;
    case APLEVT_ICC_DEPARTURE_REQ:
        DBGLOG0("APLEVT_ICC_DEPARTURE_REQ");
        aplicc_departure();
        break;
    case APLEVT_ICC_REREGIST_REQ:
        aplicc_start();             // カードRWがOFFのときはカード起動
        if (drvicc_error != 0) {
            aplevt_send_event(APLEVT_ICC_REREGIST_COMPLETE, APLICC_ERROR_DRVICC, NULL, s_event_dest);
            break;
        }
        
        DBGLOG0("APLEVT_ICC_REREGIST_REQ");
        reading_type_req = APLICC_CARD_VACANCY;
        polling_timeout = event->extra_data.reg_prep_req.timeout;
        aplicc_reregist_vacancy_card(event);
        break;
    case APLEVT_ICC_INTERNAL_POLLING:
        // 内部処理：ドライバコールバックからのポーリング
        DBGLOG0("APLEVT_ICC_INTERNAL_POLLING");
        aplicc_restart_polling();
        break;
    default:
        result = -1;
        break;
    }

    return result;
}

/*
 * カードRW起動
 */
void aplicc_start()
{
    DBGLOG1("aplicc_start icc_status:%d", icc_status);
    drvicc_error = 0;
    
    if (icc_status == STOP) {
        ER er = clr_flg(FLG_APLICC, ~FLGPTN_DRVICC_INITIALIZE_COMPLETE);
        assert(er == E_OK);
        // カードドライバを初期化
        drvicc_start(drvicc_callback);
        FLGPTN flgptn = 0;
        er = twai_flg(FLG_APLICC, FLGPTN_DRVICC_INITIALIZE_COMPLETE, TWF_ANDW, &flgptn, 2000);
        assert(er == E_OK);
    }
    else {
        aplevt_send_event(APLEVT_ICC_START_COMPLETE, 0, NULL, s_event_dest);
    }
}

/*
 * カード制御中止
 */
void aplicc_stop()
{
    DBGLOG0("aplicc_stop");

    icc_status = STOP_RECV;
    // ポーリング中ならばポーリング停止
    if (polling_status == START) {
        drvicc_cancel_polling(drvicc_callback);
    }

    FLGPTN flgptn = 0;
    ER er = 0;
    // カード書き込み中ならば書き込み終了を待つ
    if (write_status == START) {
        er = twai_flg(FLG_APLICC, FLGPTN_DRVICC_WRITE_COMPLETE, TWF_ANDW, &flgptn, -1);
        assert(er == E_OK);
    }

    clr_flg(FLG_APLICC, ~FLGPTN_DRVICC_FINALIZE_COMPLETE);
    // カードRWのストップ処理
    if (self_drv_stop) {
        drvicc_power_off_nomsg();
        icc_status = STOP;
    }
    else {
        // メインからカード制御中止
        clr_flg(FLG_APLICC, ~FLGPTN_DRVICC_FINALIZE_COMPLETE);
        drvicc_power_off(drvicc_callback);
        er = twai_flg(FLG_APLICC, FLGPTN_DRVICC_FINALIZE_COMPLETE, TWF_ANDW, &flgptn, 2000);
        assert(er == E_OK);

        // ドライバが異常となりdrvicc_poser_off()がタイムアウトした場合に
        // アプリ側に通知する手段がないため、念のためここでエラー通知を行う。（コールバックが呼ばれない可能性を考慮）
        if (er != E_OK) {
            aplevt_send_event(APLEVT_ICC_STOP_COMPLETE, er, NULL, s_event_dest);
        }
    }
    self_drv_stop = false;
}

/*
 * カード読込（ポーリング開始）
 */
void aplicc_start_polling(APLEVT_EVENT_T* event)
{
    DBGLOG1("aplicc_start_polling polling_timeout:%d", polling_timeout);
    
    polling_event_code = event->code;
    aplicc_restart_polling();
}

/*
 * カード再読込（ポーリング開始）
 */
void aplicc_restart_polling()
{
    // 読み込み回数初期化
    icc_read_cnt = 0;
    drvicc_start_polling(drvicc_callback, &idm);
    polling_status = START;

    cmntimer_cancell_timer(CMNTIMER_TIMER_APLICC);
    if (0 < polling_timeout) {
        cmntimer_set_timer(CMNTIMER_TIMER_APLICC, CMNTIMER_TYPE_ONESHOT, polling_timeout, aplicc_timeout_callback, 0);
    }
}

/*
 * ポーリングタイムアウト
 */
void aplicc_timeout_callback(int timer_id, intptr_t otp1)
{
    DBGLOG0("aplicc_timeout_callback");
    // カード制御中止してポーリングしていたイベントに対応した送信イベントをメインに返す
    aplicc_stop_and_send_event(polling_complete_event(), APLICC_ERROR_TIMEOUT, NULL, s_event_dest);
}

/*
 * ポーリングタイムアウト/エラー時の送信イベント
 */
int polling_complete_event()
{
    int complete_event = APLEVT_INVALID;
    switch (polling_event_code) {
    case APLEVT_ICC_AUTH_REQ:
        complete_event = APLEVT_ICC_AUTH_COMPLETE;
        break;
    case APLEVT_ICC_REGIST_REQ:
        complete_event = APLEVT_ICC_REGIST_COMPLETE;
        break;
    case APLEVT_ICC_REREGIST_REQ:
        complete_event = APLEVT_ICC_REREGIST_COMPLETE;
        break;
    case APLEVT_ICC_INITIALIZE:
        complete_event = APLEVT_ICC_POLLING_COMPLETE;
        break;
    default:
        break;
    }
    return complete_event;
}

/*
 *  フラッシュのカード情報読込
 */
void aplicc_read_flash()
{
    ER er = E_OK;
    
    if (reading_type == APLICC_CARD_VACANCY_REGIST) {
        MDLSTRG_REQUEST_T VACANCY_REG_CARD_READ_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_VACANCY_REG_CARD,  // 一括登録カード情報
            .request_type = MDLSTRG_REQ_TYPE_READ,
            .data = (intptr_t)&info_vacancy_regist,
        };
        mdlstrg_request(&VACANCY_REG_CARD_READ_REQ, strg_callback);
        er = twai_flg(FLG_APLICC, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        
        DBGLOG1("VACANCY_REG_CARD_READ_REQ : er = %d", er);
        //cmndbg_hexdump(&info_vacancy_regist, sizeof(info_vacancy_regist), "VACANCY_REG_CARD flash");
    }
    else if (reading_type == APLICC_CARD_VACANCY) {
        MDLSTRG_REQUEST_T VACANCY_CARD_READ_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_VACANCY_CARD,    // 空室カード情報
            .request_type = MDLSTRG_REQ_TYPE_READ,
            .data = (intptr_t)&info_vacancy,
        };
        mdlstrg_request(&VACANCY_CARD_READ_REQ, strg_callback);
        er = twai_flg(FLG_APLICC, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);

        DBGLOG1("VACANCY_CARD_READ_REQ : er = %d", er);
        //cmndbg_hexdump(&info_vacancy, sizeof(info_vacancy), "VACANCY_CARD flash");
        /* debug用 */
        //char title[32];
        //for (int i = 0; i < APLICC_VACANCY_MAX; i++) {
        //    sprintf(title, "VACANCY_CARD[%d] flash", i);
        //    cmndbg_hexdump(&info_vacancy[i], sizeof(info_vacancy[i]), title);
        //}
        /* */
    }
    else if (reading_type == APLICC_CARD_REGIST) {
        MDLSTRG_REQUEST_T REG_CARD_READ_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_REG_CARD,        // 登録・抹消カード情報
            .request_type = MDLSTRG_REQ_TYPE_READ,
            .data = (intptr_t)&info_regist,
        };
        mdlstrg_request(&REG_CARD_READ_REQ, strg_callback);
        er = twai_flg(FLG_APLICC, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);

        //cmndbg_hexdump(&info_regist, sizeof(info_regist), "REG_CARD flash");
    }
    else if (reading_type == APLICC_CARD_DROOM1 || reading_type == APLICC_CARD_DROOM2) {
        MDLSTRG_REQUEST_T DROOM_CARD_READ_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_DROOM_CARD,      // Droomカード情報
            .request_type = MDLSTRG_REQ_TYPE_READ,
            .data = (intptr_t)&info_droom,
        };
        mdlstrg_request(&DROOM_CARD_READ_REQ, strg_callback);
        er = twai_flg(FLG_APLICC, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);

        DBGLOG1("DROOM_CARD_READ_REQ : er = %d", er);
        
        /* debug用 */
        //char title[32];
        //for (int i = 0; i < APLICC_DROOM_MAX; i++) {
        //    sprintf(title, "DROOM_CARD[%d] flash", i);
        //    cmndbg_hexdump(&info_droom[i], sizeof(info_droom[i]), title);
        //}
        /* */
    }
}

/*
 *  カード(Serivce code)読込
 */
void aplicc_read_card()
{
    memcpy(desc.idm, idm.idm, sizeof(desc.idm));
    
    int idx;
    if (reading_type == APLICC_CARD_DROOM1 || reading_type == APLICC_CARD_DROOM2) {
        // D-roomカードの場合: カード種別->識別コード4つ->会員番号を読み込み
        // Service Codeの設定
        idx = 0;
        if (0 < icc_read_cnt && icc_read_cnt < 5) idx = 1;
        else if (icc_read_cnt == 5) idx = 2;
        desc.service = ICC_SERVICE_CODE[idx];
        // 読み込みblockの設定
        desc.block = 0;
        if (0 < icc_read_cnt && icc_read_cnt < 5) {
            desc.block = icc_read_cnt;
        }
    }
    else {
        // D-roomカード以外の場合: カード種別->会員番号を読み込み
        // Service Codeの設定
        idx = 0;
        if (0 < icc_read_cnt) idx = 2;
        desc.service = ICC_SERVICE_CODE[idx];
        // 読み込みblockの設定
        desc.block = 0;
    }
    DBGLOG2("aplicc_read_card service:%02x block:%d", desc.service, desc.block);

    drvicc_read(drvicc_callback, &block[icc_read_cnt], &desc);
}

/*
 *  認証
 */
int aplicc_auth_card()
{
    DBGLOG1("aplicc_auth_card: card=%d", reading_type);

    switch(reading_type) {
    case APLICC_CARD_CONSTRUCTION:
        for (int i = 0; i < sizeof(block[0].block); i++) {
            if (block[0].block[i] != ICC_CONSTRUCTION_CODE) {
                return -1;
            }
        }
        return 0;
        break;
    case APLICC_CARD_VACANCY_REGIST:
        if (memcmp(&(block[0].block), &(ICC_CARD_TYPE_CODE[APLICC_CARD_VACANCY_REGIST][0]), 2) == 0
            && memcmp(&(info_vacancy_regist.card_type), &(block[0].block), sizeof(info_vacancy_regist.card_type)) == 0
            && memcmp(&(info_vacancy_regist.idm), &(desc.idm), sizeof(desc.idm)) == 0) {
                return 0;
        }
        break;
    case APLICC_CARD_VACANCY:
        for (int i = 0; i < APLICC_VACANCY_MAX; i++) {
            if (memcmp(&(block[0].block), &(ICC_CARD_TYPE_CODE[APLICC_CARD_VACANCY][0]), 2) == 0
            && memcmp(&(info_vacancy[i].card_type), &(block[0].block), sizeof(info_vacancy[i].card_type)) == 0
            && memcmp(&(info_vacancy[i].idm), &(desc.idm), sizeof(desc.idm)) == 0) {
                return 0;
            }
        }
        break;
    case APLICC_CARD_REGIST:
        if (memcmp(&(block[0].block), &(ICC_CARD_TYPE_CODE[APLICC_CARD_REGIST][0]), 2) == 0
            && memcmp(&(info_regist.card_type), &(block[0].block), sizeof(info_regist.card_type)) == 0
            && memcmp(&(info_regist.idm), &(desc.idm), sizeof(desc.idm)) == 0) {
                return 0;
        }
        break;
    case APLICC_CARD_DROOM1:
    case APLICC_CARD_DROOM2:
        {
        int i = 0;
        for (i = 0; i < APLICC_DROOM_MAX; i++) {
            if (info_droom[i].status_flg[0] == 0xFF) {
                continue;
            }
            if (memcmp(&(info_droom[i].idm), &(desc.idm), sizeof(desc.idm)) == 0) {
                break;
            }
        }
        if (APLICC_DROOM_MAX <= i) {
            DBGLOG0("auth droom: no idm");
            return -1;
        }
        uint16_t card_idx = i + 1; // 1起算, 1～10 
        DBGLOG1("auth droom: idm match index: %d", card_idx);

        bool_t auth1 = false;
        if ((memcmp(block[0].block, &(ICC_CARD_TYPE_CODE[APLICC_CARD_DROOM1][0]), 2) == 0
             || memcmp(block[0].block, &(ICC_CARD_TYPE_CODE[APLICC_CARD_DROOM2][0]), 2) == 0)
            && memcmp(info_droom[i].card_type, block[0].block, sizeof(info_droom[i].card_type)) == 0
            && memcmp(info_droom[i].member_num, block[5].block, sizeof(info_droom[i].member_num)) == 0) {
                auth1 = true;
        }
        if (!auth1) {
            DBGLOG0("auth droom: auth1 NG");
            return -1;
        }
        DBGLOG0("auth droom: auth1 OK");

        // D-roomカード未使用状態か？
        int j = 0;
        for (j = 0; j < ICC_NUM_ID_CODE_MNG; j++) {
            // 識別コードの書込みブロックが0以外ならば未使用でない
            if (id_code_mng[j].id_code.idx == card_idx && id_code_mng[j].id_code.block != 0) {
                //DBGLOG2("idx:%d block:%d", card_idx, id_code_mng[j].id_code.block);
                break;
            }
        }
        int id_code_mng_idx = j;
        if (id_code_mng_idx < ICC_NUM_ID_CODE_MNG) {
            DBGLOG1("auth droom: id_code_mng_idx:%d", id_code_mng_idx);
            DBGLOG2("auth droom: idx:%d block:%d", card_idx, id_code_mng[id_code_mng_idx].id_code.block);
        }
        else {
            DBGLOG0("auth droom: new card");
        }

        // 識別コードのチェック
        bool_t auth2 = false;
        int block_idx = 1;
        // 未使用状態（識別コードが書き込まれていない）はチェックしない
        if (id_code_mng_idx < ICC_NUM_ID_CODE_MNG) {
            // debug用
            //for (int k = 1; k <= 4; k++) {
            //    cmndbg_hexdump(block[k].block, sizeof(block[k].block), "card id_code");
            //}

            //cmndbg_hexdump(&(id_code_mng[id_code_mng_idx].id_code.block), sizeof(id_code_mng[id_code_mng_idx].id_code.block), "flash block");
            
            block_idx = id_code_mng[id_code_mng_idx].id_code.block;
            DBGLOG1("auth droom: flash block:%d", block_idx);
            
            //cmndbg_hexdump(id_code_mng[id_code_mng_idx].id_code.code, sizeof(id_code_mng[id_code_mng_idx].id_code.code), "flash pswd");
            if (memcmp(&(block[block_idx].block[8]), id_code_mng[id_code_mng_idx].id_code.code, 8) == 0) {
                auth2 = true;
                // 書き込みブロック（1～4のリング形式）
                block_idx++;
                if (4 < block_idx) {
                    block_idx = 1;
                }
                
                DBGLOG1("auth droom: next block_idx:%d", block_idx);
            }

            if (!auth2) {
                DBGLOG0("auth droom: auth2 NG");
                return -2;
            }

            DBGLOG0("auth droom: auth2 OK");
        }
        
        // 新しい識別コード作成
        drvrng_get_random(new_id_code.code, 8);
        
        //cmndbg_hexdump(new_id_code.code, sizeof(new_id_code.code), "new_pswd");
        new_id_code.flg = 0x7FFF;
        new_id_code.idx = card_idx;
        new_id_code.block = block_idx;
        //DBGLOG1("new_id_code idx:%d", new_id_code.idx);
        //DBGLOG1("new_id_code block:%d", new_id_code.block);

        return 0;
        break;
        }
    }
    return -1;
}

/*
 *  識別コードの書き込み
 */
void aplicc_write_identify_code() {
    write_status = START;
    // カード書き込みフラグリセット
    ER er = clr_flg(FLG_APLICC, ~FLGPTN_DRVICC_WRITE_COMPLETE);
    assert(er == E_OK);
    
    int block_idx =  new_id_code.block;
    memcpy(&(block[block_idx].block[0]), &new_id_code.flg, 2);
    memcpy(&(block[block_idx].block[2]), &new_id_code.idx, 2);
    block[block_idx].block[4] = block_idx;
    memcpy(&(block[block_idx].block[8]), new_id_code.code, 8);
    cmndbg_hexdump(block[block_idx].block, sizeof(block[block_idx].block), "block");

    memcpy(desc.idm, idm.idm, sizeof(desc.idm));
    desc.service = ICC_SERVICE_CODE[1];
    desc.block = block_idx;

    DBGLOG2("aplicc_write_identify_code service:%02x block:%d", desc.service, desc.block);
    // カード書き込み
    drvicc_write(drvicc_callback, &block[block_idx], &desc);
}

/*
 *  カード種別チェック
 */
int aplicc_check_card()
{
    DBGLOG1("aplicc_check_card: card=%d", reading_type);

    switch(reading_type) {
    case APLICC_CARD_CONSTRUCTION:
        for (int i = 0; i < sizeof(block[0].block); i++) {
            if (block[0].block[i] != ICC_CONSTRUCTION_CODE) {
                return -1;
            }
        }
        return 0;
        break;
    case APLICC_CARD_VACANCY_REGIST:
        if (memcmp(&(block[0].block), &(ICC_CARD_TYPE_CODE[APLICC_CARD_VACANCY_REGIST][0]), 2) == 0) {
            return 0;
        }
        break;
    case APLICC_CARD_VACANCY:
        if (memcmp(&(block[0].block), &(ICC_CARD_TYPE_CODE[APLICC_CARD_VACANCY][0]), 2) == 0) {
            return 0;
        }
        break;
    case APLICC_CARD_REGIST:
        if (memcmp(&(block[0].block), &(ICC_CARD_TYPE_CODE[APLICC_CARD_REGIST][0]), 2) == 0) {
            return 0;
        }
        break;
    case APLICC_CARD_DROOM1:
    case APLICC_CARD_DROOM2:
        if (memcmp(block[0].block, &(ICC_CARD_TYPE_CODE[APLICC_CARD_DROOM1][0]), 2) == 0
            || memcmp(block[0].block, &(ICC_CARD_TYPE_CODE[APLICC_CARD_DROOM2][0]), 2) == 0) {
            return 0;
        }
        break;
    }
    return -1;
}

/*
 *  カード登録時のstatus_flgを設定する
 */
void set_status_flg(uint8_t *status_flg)
{
    status_flg[0] = 0x7F;
    for (int i = 1; i < sizeof(info_regist.status_flg); i++) {
        status_flg[i] = 0xFF;
    }
}

/*
 *  カード登録
 */
int aplicc_regist_card()
{
    DBGLOG1("aplicc_regist_card: card=%d", reading_type);
    ER er = E_OK;

    switch(reading_type) {
    case APLICC_CARD_VACANCY_REGIST:
        {
            // 一括登録カード保存
            set_status_flg(info_vacancy_regist.status_flg);
            memcpy(info_vacancy_regist.idm, idm.idm, sizeof(info_vacancy_regist.idm));
            memcpy(info_vacancy_regist.card_type, block[0].block, sizeof(info_vacancy_regist.card_type));
            memcpy(info_vacancy_regist.member_num, block[1].block, sizeof(info_vacancy_regist.member_num));

            MDLSTRG_REQUEST_T VACANCY_REG_CARD_WRITE_REQ = {
                .data_type = MDLSTRG_DATA_TYPE_VACANCY_REG_CARD,
                .request_type = MDLSTRG_REQ_TYPE_WRITE,
                .data = (intptr_t)&info_vacancy_regist,
            };
            mdlstrg_request(&VACANCY_REG_CARD_WRITE_REQ, strg_callback);
            er = twai_flg(FLG_APLICC, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
            assert(er == E_OK);
            
            if (s_strgmdl_req_state != 0) {
                return -1;
            }
        break;
        }
    case APLICC_CARD_VACANCY:
        {
            // 空室カード保存
            int idx = vacancy_card_cnt;
            set_status_flg(info_vacancy[idx].status_flg);
            memcpy(info_vacancy[idx].idm, idm.idm, sizeof(info_vacancy[idx].idm));
            memcpy(info_vacancy[idx].card_type, block[0].block, sizeof(info_vacancy[idx].card_type));
            memcpy(info_vacancy[idx].member_num, block[1].block, sizeof(info_vacancy[idx].member_num));
            // 
            MDLSTRG_REQUEST_T VACANCT_CARD_WRITE_REQ = {
                .data_type = MDLSTRG_DATA_TYPE_VACANCY_CARD,
                .request_type = MDLSTRG_REQ_TYPE_APPEND,
                .data = (intptr_t)&info_vacancy[idx],
            };
            mdlstrg_request(&VACANCT_CARD_WRITE_REQ, strg_callback);
            er = twai_flg(FLG_APLICC, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
            assert(er == E_OK);
            
            if (s_strgmdl_req_state != 0) {
                return -1;
            }
        break;
        }
    case APLICC_CARD_REGIST:
        {
        	// ※以下の空室カード初期化は行わない
            // 空室カード初期化
            //MDLSTRG_REQUEST_T VACANCY_CARD_DELETE_REQ = {
            //    .data_type = MDLSTRG_DATA_TYPE_VACANCY_CARD,
            //    .request_type = MDLSTRG_REQ_TYPE_ALL_DELETE,
            //    .data = (intptr_t)NULL,
            //};
            //mdlstrg_request(&VACANCY_CARD_DELETE_REQ, strg_callback);
            //er = twai_flg(FLG_APLICC, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
            //assert(er == E_OK);
            //
            //if (s_strgmdl_req_state != 0) {
            //    return -1;
            //}

            // 登録・抹消カード保存
            set_status_flg(info_regist.status_flg);
            memcpy(info_regist.idm, idm.idm, sizeof(info_regist.idm));
            memcpy(info_regist.card_type, block[0].block, sizeof(info_regist.card_type));
            memcpy(info_regist.member_num, block[1].block, sizeof(info_regist.member_num));

            MDLSTRG_REQUEST_T REG_CARD_WRITE_REQ = {
                .data_type = MDLSTRG_DATA_TYPE_REG_CARD,
                .request_type = MDLSTRG_REQ_TYPE_WRITE,
                .data = (intptr_t)&info_regist,
            };
            mdlstrg_request(&REG_CARD_WRITE_REQ, strg_callback);
            er = twai_flg(FLG_APLICC, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
            assert(er == E_OK);
            
            if (s_strgmdl_req_state != 0) {
                return -1;
            }
            
            // 識別コード管理データ初期化
            MDLSTRG_REQUEST_T ID_CODE_INIT_REQ = {
                .data_type = MDLSTRG_DATA_TYPE_IDCODE,
                .request_type = MDLSTRG_REQ_TYPE_CREATE,
            };
            mdlstrg_request(&ID_CODE_INIT_REQ, strg_callback);
            er = twai_flg(FLG_APLICC, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
            assert(er == E_OK);
            
            if (s_strgmdl_req_state != 0) {
                return -1;
            }
        break;
        }
    case APLICC_CARD_DROOM1:
    case APLICC_CARD_DROOM2:
        {
            // D-roomカード保存
            int idx = card_index - 1;
            set_status_flg(info_droom[idx].status_flg);
            memcpy(info_droom[idx].idm, idm.idm, sizeof(info_droom[idx].idm));
            memcpy(info_droom[idx].card_type, block[0].block, sizeof(info_droom[idx].card_type));
            memcpy(info_droom[idx].member_num, block[5].block, sizeof(info_droom[idx].member_num));
            memcpy(info_droom[idx].user_name, user_name, sizeof(info_droom[idx].user_name));

            MDLSTRG_REQUEST_T DROOM_CARD_WRITE_REQ = {
                .data_type = MDLSTRG_DATA_TYPE_DROOM_CARD,
                .request_type = MDLSTRG_REQ_TYPE_WRITE,
                .data = (intptr_t)&info_droom[idx],
                .opt1 = idx,
            };
            mdlstrg_request(&DROOM_CARD_WRITE_REQ, strg_callback);
            er = twai_flg(FLG_APLICC, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
            assert(er == E_OK);
            
            if (s_strgmdl_req_state != 0) {
                return -1;
            }
        break;
        }
    default:
        break;
    }
    return 0;
}

/*
 *  カード削除（D-roomカード）
 */
void aplicc_delete_card()
{
    int err = 0;
    
    // D-roomカード情報を削除
    int idx = card_index - 1;
    MDLSTRG_REQUEST_T DROOM_CARD_DELETE_REQ = {
        .data_type = MDLSTRG_DATA_TYPE_DROOM_CARD,
        .request_type = MDLSTRG_REQ_TYPE_DELETE,
        .opt1 = idx,
    };
    mdlstrg_request(&DROOM_CARD_DELETE_REQ, strg_callback);
    ER er = twai_flg(FLG_APLICC, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
    assert(er == E_OK);
    
    if (s_strgmdl_req_state != 0) {
        err = (int)s_strgmdl_req_state;
        aplevt_send_event(APLEVT_ICC_DELETE_COMPLETE, err, NULL, s_event_dest);
        return;
    }
    
    // 識別コードに初期データを書き込む
    new_id_code.flg = 0x7FFF;
    new_id_code.idx = card_index;
    new_id_code.block = 0;
    memset(new_id_code.code, 0, sizeof(new_id_code.code));
    cmndbg_hexdump(&new_id_code, sizeof(new_id_code), "new_id_code");
    
    MDLSTRG_REQUEST_T ID_CODE_WRITE_REQ = {
        .data_type = MDLSTRG_DATA_TYPE_IDCODE,
        .request_type = MDLSTRG_REQ_TYPE_WRITE,
        .data = (intptr_t)&new_id_code,
    };
    mdlstrg_request(&ID_CODE_WRITE_REQ, strg_callback);
    er = twai_flg(FLG_APLICC, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
    assert(er == E_OK);
    
    err = (int)s_strgmdl_req_state;
    aplevt_send_event(APLEVT_ICC_DELETE_COMPLETE, err, NULL, s_event_dest);
}

/*
 *  退去
 */
void aplicc_departure()
{
    int err = 0;
    // D-roomカード情報および登録・抹消カード情報を削除
    MDLSTRG_REQUEST_T DROOM_CARD_ALL_DELETE_REQ = {
        .data_type = MDLSTRG_DATA_TYPE_DROOM_CARD,
        .request_type = MDLSTRG_REQ_TYPE_ALL_DELETE,
        .data = (intptr_t)NULL,
    };
    mdlstrg_request(&DROOM_CARD_ALL_DELETE_REQ, strg_callback);
    ER er = twai_flg(FLG_APLICC, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
    assert(er == E_OK);
    
    err = (int)s_strgmdl_req_state;
    if (err != 0) {
        aplevt_send_event(APLEVT_ICC_DEPARTURE_COMPLETE, err, NULL, s_event_dest);
        return;
    }
    
    // 識別コード削除
    MDLSTRG_REQUEST_T ID_CODE_ALL_DELETE_REQ = {
        .data_type = MDLSTRG_DATA_TYPE_IDCODE,
        .request_type = MDLSTRG_REQ_TYPE_ALL_DELETE,
    };
    mdlstrg_request(&ID_CODE_ALL_DELETE_REQ, strg_callback);
    er = twai_flg(FLG_APLICC, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
    assert(er == E_OK);
    
    err = (int)s_strgmdl_req_state;
    aplevt_send_event(APLEVT_ICC_DEPARTURE_COMPLETE, err, NULL, s_event_dest);
}

/*
 *  空室カード登録
 */
void aplicc_regist_vacancy_card(APLEVT_EVENT_T* event)
{
    exists_vacancy_regist = false;

    // LED点滅（黄白）
    aplevt_send_event(APLEVT_REG_CARD_REG_VACANCY_READY, 0, NULL, s_event_dest);
    
    // flash カードデータ取得（一括登録カード）
    reading_type = APLICC_CARD_VACANCY_REGIST;
    aplicc_read_flash();
    exists_vacancy_regist = s_strgmdl_req_state;

    DBGLOG1("VACANCY_REG_CARD_READ_REQ exists: %d", exists_vacancy_regist);
    
    // flash カードデータ取得（空室カード）
    reading_type = APLICC_CARD_VACANCY;
    aplicc_read_flash();
    
    int i = 0;
    for (i = 0; i < APLICC_VACANCY_MAX; i++) {
        if (info_vacancy[i].status_flg[0] == 0xFF) {
            break;
        }
    }
    vacancy_card_cnt = i;
    DBGLOG1("vacancy_card_cnt: %d", vacancy_card_cnt);
    
    // 20枚登録済の場合、メインに返信
    if (APLICC_VACANCY_MAX <= vacancy_card_cnt) {
        aplevt_send_event(APLEVT_ICC_REGIST_COMPLETE, APLICC_ERROR_MAX_CNT, NULL, s_event_dest);
        return;
    }
    
    // 一括登録カード ポーリング
    reading_type = APLICC_CARD_VACANCY_REGIST;
    aplicc_start_polling(event);
}

/*
 *  空室カード再登録
 */
void aplicc_reregist_vacancy_card(APLEVT_EVENT_T* event)
{
    // LED点滅（赤白）
    aplevt_send_event(APLEVT_REREG_CARD_REG_VACANCY_READY, 0, NULL, s_event_dest);
    
    // 一括登録カードなしにして強制的に登録する
    exists_vacancy_regist = false;
    // 一括登録カード ポーリング
    reading_type = APLICC_CARD_VACANCY_REGIST;
    aplicc_start_polling(event);
}

/*
 *  空室カード初期化
 */
void aplicc_all_delete_vacancy_card()
{
    // 空室カード情報および一括登録カード情報を削除
    MDLSTRG_REQUEST_T VACANYCY_CARD_ALL_DELETE_REQ = {
        .data_type = MDLSTRG_DATA_TYPE_VACANCY_CARD,
        .request_type = MDLSTRG_REQ_TYPE_ALL_DELETE,
    };
    mdlstrg_request(&VACANYCY_CARD_ALL_DELETE_REQ, strg_callback);
    ER er = twai_flg(FLG_APLICC, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
    assert(er == E_OK);
    
    memset(info_vacancy, 0, sizeof(info_vacancy));
}

/*
 *  エラー時の再ポーリング
 */
void aplicc_retry_polling()
{
    if (icc_status == START) {
        dly_tsk(1000);
        aplevt_send_event(APLEVT_ICC_INTERNAL_POLLING, 0, NULL, s_event_dest);
    }
}

/*
 *  内部からのカード制御中止
 */
void aplicc_self_aplicc_stop()
{
    if (icc_status != STOP) {
        self_drv_stop = true;
        // カード制御中止
        aplicc_stop();
    }
}

/*
 *  カード制御中止してメインにイベントを返却する
 */
int32_t aplicc_stop_and_send_event(uint32_t code, int32_t error, const APLEVT_EXTRA_DATA_T* extra_data, APLEVT_EVENT_RECEIVER_FUNC_T receiver)
{
    // カード制御中止
    aplicc_self_aplicc_stop();
    //aplevt_send_event(APLEVT_ICC_STOP_REQ, 0, NULL, s_event_dest);
    // メインにイベントを返却
    int32_t ret = aplevt_send_event(code, error, extra_data, receiver);
    return ret;
}

/*
 * カードドライバコールバック
 */
void drvicc_callback(int32_t event, int32_t error, intptr_t data)
{
    DBGLOG2("drvicc_callback: evt=%d error=%d", event, error);
    int ret = 0;
    int32_t err = 0;
    ER er = E_OK;
    drvicc_error = error;
    
    switch(event) {
    case DRVICC_EVT_NONE:
        break;
    case DRVICC_EVT_INITIALIZE_COMPLETE:
        DBGLOG0("[CALLBACK]DRVICC_EVT_INITIALIZE_COMPLETE");
        if (error == 0) {
            icc_status = START;
        }
        set_flg(FLG_APLICC, FLGPTN_DRVICC_INITIALIZE_COMPLETE);
        // 初期処理中で正常の場合はメインに返さない（ポーリング検出で返す）
        if (!icc_initializing) {
            aplevt_send_event(APLEVT_ICC_START_COMPLETE, error, NULL, s_event_dest);
        }
        else if (error != 0) {
            // 初期処理中のエラー
            aplevt_send_event(APLEVT_ICC_POLLING_COMPLETE, error, NULL, s_event_dest);
        }
        break;
    case DRVICC_EVT_FINALIZE_COMPLETE:
        DBGLOG0("[CALLBACK]DRVICC_EVT_FINALIZE_COMPLETE");
        icc_status = STOP;
        set_flg(FLG_APLICC, FLGPTN_DRVICC_FINALIZE_COMPLETE);
        aplevt_send_event(APLEVT_ICC_STOP_COMPLETE, error, NULL, s_event_dest);
        break;
    case DRVICC_EVT_POLLING_COMPLETE:
        DBGLOG0("[CALLBACK]DRVICC_EVT_POLLING_COMPLETE");
        polling_status = STOP;
        cmntimer_cancell_timer(CMNTIMER_TIMER_APLICC);
        if (polling_event_code == APLEVT_ICC_INITIALIZE) {
            // 初期処理のポーリングは結果(error)をメインに返して処理終了
            aplevt_send_event(APLEVT_ICC_POLLING_COMPLETE, error, NULL, s_event_dest);
            return;
        }
        if (error != 0 && icc_status != STOP_RECV) {
            if (polling_event_code == APLEVT_ICC_AUTH_REQ
                && (reading_type == APLICC_CARD_CONSTRUCTION || reading_type == APLICC_CARD_VACANCY
                    || reading_type == APLICC_CARD_DROOM1 || reading_type == APLICC_CARD_DROOM2)) {
                // カード認証時はカード制御中止してエラーを返す
                // ポーリングしていたイベントに対応した送信イベントをメインに返す
                aplicc_stop_and_send_event(polling_complete_event(), APLICC_ERROR_READ_CARD, NULL, s_event_dest);
            }
            else {
                DBGLOG0("polling error -> retry polling");
                // カード登録・空室カード再登録時は再度ポーリング
                aplicc_retry_polling();
            }
            return;
        }

        // Serivce code1回目読込
        aplicc_read_card();

        break;
    case DRVICC_EVT_CANCEL_POLLING_COMPLETE:
        DBGLOG0("[CALLBACK]DRVICC_EVT_CANCEL_POLLING_COMPLETE");
        polling_status = STOP;
        break;
    case DRVICC_EVT_READ_COMPLETE:
        DBGLOG1("[CALLBACK]DRVICC_EVT_READ_COMPLETE %d", error);
        if (error != 0 && icc_status != STOP_RECV) {
            if (polling_event_code == APLEVT_ICC_AUTH_REQ && reading_type != APLICC_CARD_REGIST) {
                // カード認証（登録・抹消カード以外）はカード制御中止してエラーを返す
                aplicc_stop_and_send_event(polling_complete_event(), APLICC_ERROR_READ_CARD, NULL, s_event_dest);
            }
            else {
                DBGLOG0("read error -> retry polling");
                // カード登録・空室カード再登録時は再度ポーリング
                aplicc_retry_polling();
            }
            return;
        }
        
        if (icc_read_cnt == 0) {
            cmndbg_hexdump(idm.idm, sizeof(idm.idm), "Felica IDm");
        }
        char title[32];
        sprintf(title, "service_%04x", desc.service);
        cmndbg_hexdump(block[icc_read_cnt].block, sizeof(block[0].block), title);

        int read_max_cnt = ICC_NO_DROOM_READ_CNT;
        if (reading_type == APLICC_CARD_DROOM1 || reading_type == APLICC_CARD_DROOM2) {
            read_max_cnt = ICC_DROOM_READ_CNT;
        }
        //DBGLOG1("read_max_cnt %d", read_max_cnt);
        
        // D-roomカードの場合は6回読込、それ以外は2回読込
        icc_read_cnt++;
        if (icc_read_cnt < read_max_cnt) {
            aplicc_read_card();
            return;
        }
        
        switch (polling_event_code) {
        case APLEVT_ICC_AUTH_REQ:
            {
            // --- カード認証時の処理
            // 認証
            ret = aplicc_auth_card();
            DBGLOG1("aplicc_auth_card ret(%d)", ret);
            err = 0;
            if (ret == -2) {
                err = APLICC_ERROR_IDENTIFY_CODE;
            }
            else if (ret != 0) {
                err = APLICC_ERROR_AUTH;
            }

            // カード認証（登録・抹消カード）の認証エラーは再度ポーリング
            if (err != 0 && reading_type == APLICC_CARD_REGIST) {
                DBGLOG0("auth error -> retry polling");
                aplicc_retry_polling();
                break;
            }
            // カード制御中止してエラー応答を返す（メインからカード制御中止した場合は認証結果は返さない）
            if (err != 0 && icc_status != STOP_RECV) {
                aplicc_stop_and_send_event(APLEVT_ICC_AUTH_COMPLETE, err, NULL, s_event_dest);
                break;
            }

            if (reading_type == APLICC_CARD_DROOM1 || reading_type ==  APLICC_CARD_DROOM2) {
                 // D-roomカードの場合は識別コードを書き込む
                aplicc_write_identify_code();
            }
            else if(reading_type == APLICC_CARD_REGIST){
                // D-roomカード以外はカード制御中止してメインに応答
                aplicc_stop_and_send_event(APLEVT_ICC_REGAUTH_COMPLETE, 0, NULL, s_event_dest);
            }
            break;
            }
        case APLEVT_ICC_REGIST_REQ:
        case APLEVT_ICC_REREGIST_REQ:
            {
            // --- カード登録時、空室カード登録、空室カード再登録の処理
            // カード種別チェック
            ret = aplicc_check_card();
            DBGLOG1("aplicc_check_card ret(%d)", ret);
            if (ret != 0) {
                DBGLOG0("card type mismatch -> retry polling");
                // 再度ポーリング
                aplicc_retry_polling();
                break;
            }

            //DBGLOG1("reading_type %d", reading_type);
            if (reading_type == APLICC_CARD_REGIST
                || reading_type == APLICC_CARD_DROOM1 || reading_type == APLICC_CARD_DROOM2) {
                // -- カード登録（登録・抹消カード/D-roomカード）
                // カード制御中止
                aplicc_self_aplicc_stop();

                // カード登録処理（登録・抹消カード/D-roomカード）

                ret = aplicc_regist_card();
                err = 0;
                if (ret != 0) {
                    err = APLICC_ERROR_WRITE_CARD;
                }

                memset(&res_extra_data, 0, sizeof(res_extra_data));
                if (reading_type == APLICC_CARD_DROOM1 || reading_type == APLICC_CARD_DROOM2) {
                    // D-roomカードは会員番号を返す
                    memcpy(res_extra_data.reg_prep_req.member_num, block[5].block, sizeof(res_extra_data.reg_prep_req.member_num));
                    //cmndbg_hexdump(res_extra_data.reg_prep_req.member_num, sizeof(res_extra_data.reg_prep_req.member_num), "member_num");
                }
                // メインに応答
                aplevt_send_event(APLEVT_ICC_REGIST_COMPLETE, err, &res_extra_data, s_event_dest);
            }
            else if (reading_type == APLICC_CARD_VACANCY_REGIST){
                // -- 空室カード登録、空室カード再登録時の処理（一括登録カード処理）
                if (polling_event_code == APLEVT_ICC_REREGIST_REQ) {
                    // 空室カード再登録時は空室カード初期化
                    aplicc_all_delete_vacancy_card();
                    vacancy_card_cnt = 0;
                }
                
                if (!exists_vacancy_regist) {
                    // 一括登録カード登録なしならば一括登録カード登録
                    ret = aplicc_regist_card();
                    if (ret != 0) {
                        err = APLICC_ERROR_WRITE_CARD;
                        aplevt_send_event(polling_complete_event(), err, NULL, s_event_dest);
                        break;
                    }
                }
                else {
                    // 一括登録カード認証
                    ret = aplicc_auth_card();
                    DBGLOG1("aplicc_auth_card ret(%d)", ret);
                    if (ret != 0) {
                        DBGLOG0("vacancy_regist_card auth err -> retry polling");
                        // 再度ポーリング
                        aplicc_retry_polling();
                        break;
                    }
                }
                
                if (polling_event_code == APLEVT_ICC_REREGIST_REQ) {
                    // LED点灯（赤） 空室カード再登録時
                    aplevt_send_event(APLEVT_REREG_CARD_REG_VACANCY_COMPLETE, 0, NULL, s_event_dest);
                }
                else {
                    // LED点灯（黄）
                    aplevt_send_event(APLEVT_REG_CARD_REG_VACANCY_COMPLETE, 0, NULL, s_event_dest);
                }
                
                dly_tsk(2000);
                
                // LED点滅（青白）
                aplevt_send_event(APLEVT_REG_CARD_VACANCY_READY, 0, NULL, s_event_dest);
                
                // 空室カード ポーリング
                reading_type = APLICC_CARD_VACANCY;
                aplevt_send_event(APLEVT_ICC_INTERNAL_POLLING, 0, NULL, s_event_dest);
            }
            else if (reading_type == APLICC_CARD_VACANCY){
                // -- 空室カード登録、空室カード再登録時の処理（空室カード処理）

                // idmが登録済ならば再度ポーリング
                /* ※20枚登録を確認する時はここをコメントにする */
                for (int i = 0; i < APLICC_VACANCY_MAX; i++) {
                    if (memcmp(&(info_vacancy[i].idm), &(desc.idm), sizeof(desc.idm)) == 0
                        && info_vacancy[i].status_flg[0] != 0xFF) {
                        DBGLOG0("exsits_idm -> retry polling");
                        // 再度ポーリング
                        aplicc_retry_polling();
                        return;
                    }
                }
                /* */

                // カード登録処理（空室カード）
                ret = aplicc_regist_card();
                if (ret != 0) {
                    err = APLICC_ERROR_WRITE_CARD;
                    aplevt_send_event(polling_complete_event(), err, NULL, s_event_dest);
                    break;
                }
                vacancy_card_cnt++;
                DBGLOG1("vacancy_card_cnt %d", vacancy_card_cnt);
                
                // LED点灯（青）
                aplevt_send_event(APLEVT_REG_CARD_VACANCY_COMPLETE, 0, NULL, s_event_dest);
                
                dly_tsk(2000);
                
                // 20枚登録なら終了
                if (APLICC_VACANCY_MAX <= vacancy_card_cnt) {
                    aplevt_send_event(polling_complete_event(), 0, NULL, s_event_dest);
                    break;
                }
                
                // LED点滅（青白）
                aplevt_send_event(APLEVT_REG_CARD_VACANCY_READY, 0, NULL, s_event_dest);
                
                // 空室カード ポーリング
                reading_type = APLICC_CARD_VACANCY;
                aplevt_send_event(APLEVT_ICC_INTERNAL_POLLING, 0, NULL, s_event_dest);
            }
                
            break;
            }
        default:
            break;
        }
        break;

    case DRVICC_EVT_WRITE_COMPLETE:
        // 識別コードをカードに書き込み終わった後の処理
        DBGLOG1("[CALLBACK]DRVICC_EVT_WRITE_COMPLETE %d", error);
        
        // フラグセット前のicc_status
        int flg_icc_status = icc_status;
        
        // カード書き込みフラグセット
        write_status = STOP;
        er = set_flg(FLG_APLICC, FLGPTN_DRVICC_WRITE_COMPLETE);
        assert(er == E_OK);
        
        if (error != 0 && icc_status != STOP_RECV && flg_icc_status != STOP_RECV) {
            aplicc_stop_and_send_event(APLEVT_ICC_AUTH_COMPLETE, APLICC_ERROR_WRITE_CARD, NULL, s_event_dest);
            break;
        }
        
        cmndbg_hexdump(&new_id_code, sizeof(new_id_code), "new_id_code");

        // 正常ならばフラッシュに識別コードの書き込み
        MDLSTRG_REQUEST_T ID_CODE_WRITE_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_IDCODE,
            .request_type = MDLSTRG_REQ_TYPE_WRITE,
            .data = (intptr_t)&new_id_code,
        };
        mdlstrg_request(&ID_CODE_WRITE_REQ, strg_callback);
        er = twai_flg(FLG_APLICC, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        
        if (s_strgmdl_req_state != 0 && icc_status != STOP_RECV && flg_icc_status != STOP_RECV) {
            aplicc_stop_and_send_event(APLEVT_ICC_AUTH_COMPLETE, APLICC_ERROR_WRITE_CARD, NULL, s_event_dest);
            break;
        }
        
        // メインからカード制御中止した場合は認証結果は返さない
        if (icc_status != STOP_RECV && flg_icc_status != STOP_RECV) {
            aplicc_stop_and_send_event(APLEVT_ICC_AUTH_COMPLETE, 0, NULL, s_event_dest);
        }
        break;
    default:
        break;
    }

}

/*
 * ミドルストレージ コールバック
 */
void strg_callback(int event, intptr_t opt1, intptr_t opt2)
{
    DBGLOG1("STRG CALLBACK(APLICC): %d", event);

    ER er = E_OK;

    switch (event) {
    case MDLSTRG_EVT_REQUEST_COMPLETE:
        s_strgmdl_req_state = opt1;
        er = set_flg(FLG_APLICC, FLGPTN_MDLSTRG_REQ_COMPLETE);
        assert(er == E_OK);
        break;
    default:
        break;
    }
}

