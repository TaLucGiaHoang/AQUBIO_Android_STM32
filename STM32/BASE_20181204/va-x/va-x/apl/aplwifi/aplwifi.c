/*
 * VA-X WiFiアプリケーション
 *
 * Copyright (C) 2018 Bionics co.    ltd.
 *
 */

#include "aplwifi.h"

#include <string.h>
#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include <stdio.h>

#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"

#include "cmntimer.h"
//#include "drvwifi.h"
#include "cmndbg.h"
#include "drvrng.h"
#include "mdlstrg_data.h"
#include "mdlstrg.h"
#include "mdlwifi.h"

// デバッグログ
#if 0
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[APLWIFI]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[APLWIFI]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[APLWIFI]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[APLWIFI]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

/*
 * 定数定義
 */
static const FLGPTN FLGPTN_MDLWIFI_INITIALIZE_COMPLETE              = (0x1 << 0);
static const FLGPTN FLGPTN_MDLWIFI_READY_COMPLETE                   = (0x1 << 1);
static const FLGPTN FLGPTN_MDLWIFI_STOP_COMPLETE                    = (0x1 << 2);
static const FLGPTN FLGPTN_MDLWIFI_CHECK_SERVERTASK_COMPLETE        = (0x1 << 3);
static const FLGPTN FLGPTN_MDLWIFI_DOWNLOAD_VACANCY_CARD_DOWNLOAD   = (0x1 << 4);
static const FLGPTN FLGPTN_MDLWIFI_UPLOAD_OPE_LOG_COMPLETE          = (0x1 << 5);
static const FLGPTN FLGPTN_MDLWIFI_UPLOAD_ERR_LOG_COMPLETE          = (0x1 << 6);

static const FLGPTN FLGPTN_MDLSTRG_REQ_COMPLETE =   	    (0x1 << 6);


#define TASK_CODE_NONE                  "00"
#define TASK_CODE_VACANCY_CARD          "01"

#define APLWIFI_VACANCY_MAX 20

#define APLWIFI_OPE_POST_STR_LENGTH 22
#define APLWIFI_ERR_POST_STR_LENGTH 22

// デバッグ用コード有効
//#define ENABLE_DEBUG_CODE

// 空室カードダウンロード時にFlashに書き込むカードタイプ
uint8_t download_card_type[16] = { 0x30, 0x34, 0x31, 0x38, 0x30, 0x39, 0x30, 0x39, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00 };
// 空室カードダウンロード時にFlashに書き込むメンバーナンバー(会員番号)
uint8_t download_member_num[16] = { 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20 };

// 空室カードダウンロード時にデータが入っていないときのIDmの値(All 0x00)
uint8_t empty_card_data[8] = {0};

// レスポンスキー種別
enum {
    RES_KEY_RESULT,
    RES_KEY_NOW_DATE,
    RES_KEY_TASK_CODE1,
    RES_KEY_TASK_CODE2,
    RES_KEY_TASK_CODE3,
    RES_KEY_TASK_CODE4,
    RES_KEY_DOWNLOAD_DATA,
};

// 書き込む操作ログの種類
enum {
    LOG_WRITE_OPE_LOG_UPLOAD,
    LOG_WRITE_ERR_LOG_UPLOAD,
    LOG_WRITE_VACANCY_CARD_DOWNLOAD,
};

// エラーログのパターン
typedef struct {
    unsigned short err_code;
    uint8_t err_code_char[4];
} APLWIFI_ERR_LOG_DEF_T;

/*
 * 内部変数
 */

/* ポーリング タイムアウト時間 */
static int polling_timeout = 0;

/* WIFI接続パラメータ */
static MDLSTRG_DATA_WIFI_PARAM_T wifi_param = {0};

/* 機器識別情報 */
static BNVA_DEVICE_ID_T device_id = {0};

/* アプリ間でやり取りするデータ */
static APLEVT_EXTRA_DATA_T extra_data;


struct {
    intptr_t opt1;
    intptr_t opt2;
} strg_result;

// struct {
//     intptr_t opt1;
//     intptr_t opt2;
// } mdlwifi_result;

/* 共通レスポンス情報 */
static MDLWIFI_RES_COMMON_T         res_common;

/* サーバータスクレスポンス情報 */
static MDLWIFI_RES_SERVERTASK_T     res_server_task;

/* 空室カードレスポンス情報 */
static MDLWIFI_RES_VACANCY_CARD_T   res_vecancy_card;

/* エラーコード */
static int32_t apl_error_code = 0;

static const APLWIFI_ERR_LOG_DEF_T APLWIFI_ERR_LOG_DEFS[] = {
    // unsigned short       err_code;                       // エラーコード
    // uint8_t              err_code_char[4];               // 電文ログコード
    { 0x1001,               "1001" },                // 電池残量エラー
    { 0x1011,               "1011" },                // ハード接続エラー
    { 0x1012,               "1012" },                // カメラ異常
    { 0x1013,               "1013" },                // 撮影異常（指以外の画像等）
    { 0x1014,               "1014" },                // 指エッジエラー
    { 0x1021,               "1021" },                // 施解錠異常
    { 0x1022,               "1022" },                // 扉過開放
    { 0x1023,               "1023" },                // 扉こじ開け
    { 0x1024,               "1024" },                // 電気錠電圧異常
    { 0x1031,               "1031" },                // スタンバイ遷移異常
    { 0x1032,               "1032" },                // 電源起動異常
    { 0x1033,               "1033" },                // 電流電圧異常
    { 0x1051,               "1051" },                // ハード接続エラー
    { 0x1052,               "1052" },                // BLE通信タイムアウト
    { 0x1053,               "1053" },                // BLE通信異常
    { 0x1054,               "1054" },                // 接続スマホ違反
    { 0x1061,               "1061" },                // フラッシュリードエラー
    { 0x1062,               "1062" },                // フラッシュライトエラー
    { 0x1071,               "1071" },                // ハード接続エラー
    { 0x1072,               "1072" },                // カードリードエラー
    { 0x1073,               "1073" },                // カードライトエラー
    { 0x1074,               "1074" },                // 1TimePassword認証エラー
    { 0x1081,               "1081" },                // ハード接続エラー
    { 0xFFFF,               "FFFF" },                // 終端データ
};

#ifdef ENABLE_DEBUG_CODE
// デバッグ出力用操作ログ退避変数
static MDLSTRG_DATA_OPE_LOG_T ope_log_for_debug;
static MDLSTRG_DATA_STRACT_DATE_T date_for_debug;
static int8_t date_str_for_debug[13];
#endif

/*
 * 内部関数プロトタイプ
 */

// イベント送信先
static APLEVT_EVENT_RECEIVER_FUNC_T s_event_dest = NULL;

/*
 * 内部関数プロトタイプ
 */

// イベント処理
static void process_event(APLEVT_EVENT_T* event);

// WIFI開始
static void aplwifi_start();

// パラメータ読込
static bool_t aplwifi_read_parameter();

// WiFi制御中止
static void aplwifi_stop();

// サーバタスク確認処理
static void aplwifi_check_servertask();

// サーバタスク実行
static void aplwifi_run_servertask(uint8_t* task_code);

// エラーログアップロード処理
static void aplwifi_upload_err_log(APLEVT_EXTRA_DATA_T* extra_data);

// エラーログ電文生成処理
static bool_t aplwifi_generate_err_log(unsigned short log_err_code, uint8_t* buffer, uint8_t battery);

// エラーコード追記
static bool_t aplwifi_add_err_log(MDLSTRG_DATA_ERR_LOG_T* log, unsigned short log_err_code);

// 操作ログアップロード処理
static void aplwifi_upload_ope_log(APLEVT_EXTRA_DATA_T* extra_data);

// 操作ログ電文生成処理
static bool_t aplwifi_generate_ope_log(MDLSTRG_DATA_OPE_LOG_T* log, uint8_t* buffer, uint8_t battery);

// 2桁までの数値を2桁の文字列に変換
static void aplwifi_cvt_num_to_str(uint8_t* pos, uint8_t num);

// 操作ログ書込み
static void aplwifi_write_ope_log(MDLSTRG_DATA_OPE_LOG_T* data);

// 空室カードダウンロード処理
static void aplwifi_download_vacancy_card();

// ミドルウェアコールバック
static void mdlwifi_callback(int event, intptr_t error, intptr_t opt1, intptr_t opt2);

// ストレージミドルコールバック
static void mdlstrg_callback(int event, intptr_t opt1, intptr_t opt2);

// YYYYMMDDhhmm形式の文字列を日付構造体に変換
static void aplwifi_cvt_date_str_to_struct(uint8_t* str_date, MDLSTRG_DATA_STRACT_DATE_T* output);

// 日付構造体をYYYYMMDDhhmm形式の文字列に変換
static void aplwifi_cvt_date_struct_to_str(MDLSTRG_DATA_STRACT_DATE_T* date, uint8_t* output_buf);

/*********************************************************************
 * 公開関数
 ********************************************************************/

/*
 * 共通初期化API(APLEVT_INITIALIZE_FUNC_T)
 */
int32_t aplwifi_initialize(APLEVT_EVENT_RECEIVER_FUNC_T receiver_func)
{
    assert(!s_event_dest);
    assert(receiver_func);

    DBGLOG0("APLwifi_ini() starts.");
    
    // イベント送信先を登録
    s_event_dest = receiver_func;

#ifdef ENABLE_DEBUG_CODE
    // 関数の単体動作確認
    aplwifi_cvt_date_str_to_struct("201812232028", &date_for_debug);
//    cmndbg_hexdump(&date_for_debug, sizeof(MDLSTRG_DATA_STRACT_DATE_T), "cvt_date_str_to_struct");

    MDLSTRG_DATA_STRACT_DATE_T date;
    date.year = 2018;
    date.month = 12;
    date.day = 23;
    date.hour = 20;
    date.min = 28;
    aplwifi_cvt_date_struct_to_str(&date, date_str_for_debug);
    date_str_for_debug[12] = '\0';
    DBGLOG1("cvt_date_struct_to_str : %s", date_str_for_debug);
#endif

    // タスクを起動
    ER er = E_OK;
    er = act_tsk(TSK_APLWIFI);
    assert(er == E_OK);

    return 0;
}

/*
 * 共通イベント受信API(APLEVT_EVENT_RECEIVER_FUNC_T)
 */
int32_t aplwifi_event(APLEVT_EVENT_T* event)
{
    DBGLOG0("aplwifi_event.");
    ER er = aplevt_queue_event(DTQ_APLWIFI, event, 0);
    assert(er == E_OK);

    return 0;
}

/*
 * メインタスク
 */
void aplwifi_task(intptr_t exinf)
{
    DBGLOG0("aplwifi_task() starts.");

    FLGPTN flgptn = 0;
    ER er = E_OK;

    // WiFiミドル起動（ここではWiFiモジュールをONにしない）
    clr_flg(FLG_APLWIFI, ~FLGPTN_MDLWIFI_INITIALIZE_COMPLETE);
    mdlwifi_initialize(mdlwifi_callback);
    er = twai_flg(FLG_APLWIFI, FLGPTN_MDLWIFI_INITIALIZE_COMPLETE, TWF_ANDW, &flgptn, 2000);
    assert(er == E_OK);

    TMO timeout = TMO_FEVR;
    while (true) {
        APLEVT_EVENT_T* event = NULL;
        ER er_rcv = aplevt_receive_event(DTQ_APLWIFI, &event, timeout);
        assert(er_rcv == E_OK);
        DBGLOG2("aplwifi_task() received: (msg = %d) (param = %d).", event->code, event->extra_data.reg_prep_req.index);
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
void process_event(APLEVT_EVENT_T* event)
{
    switch(event->code) {
    case APLEVT_WIFI_START_REQ:
        aplwifi_start();
        break;
    case APLEVT_WIFI_STOP_REQ:
        aplwifi_stop();
        break;
    case APLEVT_WIFI_CHECK_SERVERTASK_REQ:
        aplwifi_check_servertask();
        aplevt_send_event(APLEVT_WIFI_CHECK_SERVERTASK_COMPLETE, apl_error_code, &extra_data, s_event_dest);
        break;
    case APLEVT_WIFI_UPLOAD_OPE_LOG_REQ:
        aplwifi_upload_ope_log(&(event->extra_data));
        aplevt_send_event(APLEVT_WIFI_UPLOAD_OPE_LOG_COMPLETE, apl_error_code, &extra_data, s_event_dest);
        break;
    case APLEVT_WIFI_UPLOAD_ERR_LOG_REQ:
        aplwifi_upload_err_log(&(event->extra_data));
        aplevt_send_event(APLEVT_WIFI_UPLOAD_ERR_LOG_COMPLETE, apl_error_code, &extra_data, s_event_dest);
        break;
    case APLEVT_WIFI_DOWNLOAD_VACANCY_CARD_REQ : {
        // 単体で呼ばれた場合のみ（通常Serverタスクとして実行されるため、ここから実行されることはない）
        aplwifi_download_vacancy_card();

        // アプリに完了を通知
        memcpy(&extra_data.wifi_data.date, &res_common.now_date, APLEVT_EXTRA_DATE_SIZE); // 現在時刻を渡す
        aplevt_send_event(APLEVT_WIFI_CHECK_SERVERTASK_COMPLETE, apl_error_code, &extra_data, s_event_dest);
        break;
    }
    default:
        assert(false);
        break;
    }
}

/*
 * WiFi開始
 */
void aplwifi_start()
{
    ER er = E_OK;

    if (aplwifi_read_parameter() == false ) {
        // パラメータ読み込みエラーの場合、処理停止
        return;
    }

    // ミドル層を呼び出し
    clr_flg(FLG_APLWIFI, ~FLGPTN_MDLWIFI_READY_COMPLETE);
    mdlwifi_start(&wifi_param, sizeof(wifi_param), &device_id);
    er = twai_flg(FLG_APLWIFI, FLGPTN_MDLWIFI_READY_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 60000);
    assert(er == E_OK || er == E_TMOUT);

}

/*
 * パラメータ読み込み
 */
bool_t aplwifi_read_parameter() {
    ER er = E_OK;

    // 必要なデータを取得
    MDLSTRG_REQUEST_T WIFI_PARAM_READ_REQ = {
        .data_type = MDLSTRG_DATA_TYPE_WIFI,        // WiFi接続情報取得
        .request_type = MDLSTRG_REQ_TYPE_READ,
        .data = &wifi_param,
    };
    clr_flg(FLG_APLWIFI, ~FLGPTN_MDLSTRG_REQ_COMPLETE);
    mdlstrg_request(&WIFI_PARAM_READ_REQ, mdlstrg_callback);
    er = twai_flg(FLG_APLWIFI, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
    assert(er == E_OK || er == E_TMOUT);

    // フラッシュリードエラーならこの時点でAplmainに通知する
    if (strg_result.opt1 != true) {
        DBGLOG0("MDLSTRG_REQ : WIFI_PARAM_READ Error");
        aplevt_send_event(APLEVT_WIFI_READY_COMPLETE, APLWIFI_ERROR_READ_INFO, NULL, s_event_dest);
        return false;
    }

//    cmndbg_hexdump(wifi_param.essid, sizeof(wifi_param.essid), "SSID");
//    cmndbg_hexdump(wifi_param.passphrase, sizeof(wifi_param.passphrase), "pass");
    DBGLOG2("SSID_LEN %d, PASSPHRASE_LEN %d", wifi_param.essid_len, wifi_param.passphrase_len);

    // 機器識別情報を読み出し
    MDLSTRG_REQUEST_T strg_req = {
        .data_type = MDLSTRG_DATA_TYPE_DEVICE_ID,
        .request_type = MDLSTRG_REQ_TYPE_READ,
        .data = (intptr_t)&(device_id),
        .size = sizeof(device_id),
    };
    clr_flg(FLG_APLWIFI, ~FLGPTN_MDLSTRG_REQ_COMPLETE);
    mdlstrg_request(&strg_req, mdlstrg_callback);
    er = twai_flg(FLG_APLWIFI, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 2000);
    assert(er == E_OK);

    // 機器識別情報はエラーを返さないのでエラーチェックしない

    DBGLOG1("device_id: %s", device_id.serialno);

    return true;
}

/*
 * WIFI制御中止
 */
void aplwifi_stop()
{
    ER er = E_OK;

    clr_flg(FLG_APLWIFI, ~FLGPTN_MDLWIFI_STOP_COMPLETE);
    mdlwifi_stop();
    er = twai_flg(FLG_APLWIFI, FLGPTN_MDLWIFI_STOP_COMPLETE, TWF_ANDW, &(FLGPTN){0}, TMO_FEVR);
    assert(er == E_OK || er == E_TMOUT);
}

/*
 * サーバタスク確認処理
 */
void aplwifi_check_servertask()
{
    ER er = E_OK;

    // すでに読み出し済み
    //aplwifi_read_parameter();

    // 必要ならWIFI開始を呼び出すがミドル層で対応する。
    clr_flg(FLG_APLWIFI, ~FLGPTN_MDLWIFI_CHECK_SERVERTASK_COMPLETE);
    mdlwifi_check_servertask(&wifi_param, sizeof(wifi_param));
    er = twai_flg(FLG_APLWIFI, FLGPTN_MDLWIFI_CHECK_SERVERTASK_COMPLETE, TWF_ANDW, &(FLGPTN){0}, TMO_FEVR);
    assert(er == E_OK || er == E_TMOUT);

    // エラーがあれば中断
    if (apl_error_code != 0) {
        DBGLOG0("apl_error_code != 0");
        return;             // エラー内容はapl_error_codeに記録
    }

    // タスクのありなしをチェックし、ありならタスクを実行する

//    cmndbg_hexdump(&res_server_task.task_code1, 8, "taskcode");

    if (apl_error_code == 0) aplwifi_run_servertask(res_server_task.task_code1);
    if (apl_error_code == 0) aplwifi_run_servertask(res_server_task.task_code2);
    if (apl_error_code == 0) aplwifi_run_servertask(res_server_task.task_code3);
    if (apl_error_code == 0) aplwifi_run_servertask(res_server_task.task_code4);
}

/* 
 * サーバタスク実行
 */
void aplwifi_run_servertask(uint8_t* task_code) {
    ER er = 0;
    int result = 0;

    // タスクが登録されていたらタスクを実施
    if (memcmp(task_code, TASK_CODE_NONE,  2) == 0) {
        // タスク登録なし
        DBGLOG0("TASK_CODE : NONE");
    } else if (memcmp(task_code, TASK_CODE_VACANCY_CARD, 2) == 0) {
        // 空室カード登録を実行
        DBGLOG0("TASK_CODE : Download vacancy_card");
        aplwifi_download_vacancy_card();

        DBGLOG0("aplwifi_write_ope_log_start");

        // 空室カードダウンロードの結果を操作ログに保存
        MDLSTRG_DATA_OPE_LOG_T ope_log = {0};

        // TODO : ログ発生日時を追加(内部時計)　→　内部時計のAPIはTBD

        // ログ発生日時を追加(サーバ取得データのため文字列から数値に変換)
        aplwifi_cvt_date_str_to_struct(res_common.now_date, &(ope_log.log_date[1]));

        // 待機ログ
        ope_log.log[0].state = 1;           // 待機

        // 準備ログ
        ope_log.log[1].state = 2;           // 準備

        // 処理ログ
        ope_log.log[2].state = 3;           // 処理（処理内容なし）

        // 電気錠
        ope_log.log[3].state = 4;           // 電気錠（処理内容なし）

        // 終了ログ
        ope_log.log[4].state = 5;           // 終了
        ope_log.log[4].processing = 1;      // 終了処理
        ope_log.log[4].detail01 = 0x01;     // 空室カード
        if (apl_error_code == 0) {
            ope_log.log[4].detail02 = 2;    // OK
        } else {
            ope_log.log[4].detail02 = 1;    // NG
        }
        ope_log.log[4].detail03 = 0;        // (追加データなし))

#ifdef ENABLE_DEBUG_CODE
        memcpy(&ope_log_for_debug, &ope_log, sizeof(MDLSTRG_DATA_OPE_LOG_T));
//        cmndbg_hexdump(&ope_log_for_debug, sizeof(MDLSTRG_DATA_OPE_LOG_T), "ope_log_for_debug");
#endif
        aplwifi_write_ope_log(&ope_log);
    } else {
        DBGLOG0("TASK_CODE : UNDEFINED");
    }
}

/*
 * エラーログアップロード処理
 */
void aplwifi_upload_err_log(APLEVT_EXTRA_DATA_T* extra_data)
{
    DBGLOG0("aplwifi_upload_err_log");
    ER er = E_OK;

    MDLSTRG_DATA_ERR_LOG_T* data = (MDLSTRG_DATA_ERR_LOG_T*)(extra_data->wifi_data.err_log);

    for (int i = 0; i < MDLSTRG_MAX_ERR_LOG; i++) {
        DBGLOG1("Error log post start : index = %d", i);
        uint8_t post_str[APLWIFI_ERR_POST_STR_LENGTH];

        // エラーログ情報からサーバー電文を作成
        if (aplwifi_generate_err_log(data->err[i], post_str, extra_data->wifi_data.battery) == false) {
            // エラーコードの記載がなかった / または未定義だった場合は次のログを処理する
            DBGLOG1("aplwifi_generate_err_log is false. undefined code or no err : index = %d", i);
            continue;
        }

        // 付帯情報1/2を追記
        memcpy(post_str+4, "0000", 4);
        // エラーログ情報のログ発生日時を追加
        aplwifi_cvt_date_struct_to_str(&(data->log_date[0]), post_str+8);

        // バッテリー情報を追記
        aplwifi_cvt_num_to_str(post_str+20, extra_data->wifi_data.battery);

//        cmndbg_hexdump(post_str, 22, "Error log (for server)");

        // ミドル層呼び出し
        clr_flg(FLG_APLWIFI, ~FLGPTN_MDLWIFI_UPLOAD_ERR_LOG_COMPLETE);
        mdlwifi_upload_err_log(post_str, APLWIFI_ERR_POST_STR_LENGTH);
        er = twai_flg(FLG_APLWIFI, FLGPTN_MDLWIFI_UPLOAD_ERR_LOG_COMPLETE, TWF_ANDW, &(FLGPTN){0}, TMO_FEVR);
        assert(er == E_OK);

        if (apl_error_code != 0) {
            // エラーが発生したら、最後にエラー情報を追加する
            // TODO：仮でエラーコード0x1081(ハード接続エラー)を設定(本来は通信エラーだがエラーログ仕様上未定義)
            aplwifi_add_err_log(data, 0x1081);
            break;
        }
    }

    // エラーログ読み込みカウンタを更新
    MDLSTRG_REQUEST_T ERR_LOG_INCREMENT_REQ = {
        .data_type = MDLSTRG_DATA_TYPE_ERR_LOG,        // エラーログ
        .request_type = MDLSTRG_REQ_TYPE_INCREMENT
    };
    clr_flg(FLG_APLWIFI, ~FLGPTN_MDLSTRG_REQ_COMPLETE);
    mdlstrg_request(&ERR_LOG_INCREMENT_REQ, mdlstrg_callback);
    er = twai_flg(FLG_APLWIFI, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
    assert(er == E_OK);

    if (strg_result.opt1 != E_OK) {
        DBGLOG0("ERR_LOG_INCREMENT_REQ write error.");
        apl_error_code = APLWIFI_ERROR_WRITE_ERR_LOG;
        return;
    }

    // エラー構造体のサーバ日時を更新
    if (apl_error_code == 0) {
        aplwifi_cvt_date_str_to_struct(res_common.now_date, &(data->log_date[1]));
    }
    
    // アップロードしたエラー情報をFlashに保存する
    MDLSTRG_REQUEST_T ERR_LOG_WRITE_REQ = {
        .data_type = MDLSTRG_DATA_TYPE_ERR_LOG,        // エラーログ
        .request_type = MDLSTRG_REQ_TYPE_WRITE,        // 書込み
        .data = data,
    };
    clr_flg(FLG_APLWIFI, ~FLGPTN_MDLSTRG_REQ_COMPLETE);
    mdlstrg_request(&ERR_LOG_WRITE_REQ, mdlstrg_callback);
    er = twai_flg(FLG_APLWIFI, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
    assert(er == E_OK);

    if (strg_result.opt1 != E_OK) {
        DBGLOG0("MDLSTRG_REQ_TYPE_WRITE write error.");
        apl_error_code = APLWIFI_ERROR_WRITE_ERR_LOG;
    }

}

/*
 * エラーログ電文生成処理
 */
bool_t aplwifi_generate_err_log(unsigned short log_err_code, uint8_t* buffer, uint8_t battery) {
    // 生成するデータは、エラーコード＋付帯情報1＋付帯情報2＋発生日時＋電池残量(計22byte))
    int i = 0;
    while (1) {
        if (APLWIFI_ERR_LOG_DEFS[i].err_code == log_err_code) {
            // 電文エラーコードをコピー
            memcpy(buffer, APLWIFI_ERR_LOG_DEFS[i].err_code_char, 4);
            //DBGLOG1("aplwifi_generate_err_log : 0x%04x", log_err_code);
            break;
        } else if (APLWIFI_ERR_LOG_DEFS[i].err_code == 0xFFFF) {
            // 終端コードまでいった場合は、対象外のため抜ける
            //DBGLOG1("aplwifi_generate_err_log fail. 0x%04x", log_err_code);
            return false;
        }
        i++;            // カウンターインクリメント
    }
    return true;
}

/*
 * エラーコードを追記
 */
bool_t aplwifi_add_err_log(MDLSTRG_DATA_ERR_LOG_T* log, unsigned short log_err_code) {
    for (int i = 0; i < MDLSTRG_MAX_ERR_LOG; i++) {
        if (log->err[i] == 0) {
            // データが何も入っていない領域があれば、そこにエラーコードを追記する
            log->err[i] = log_err_code;
            return true;
        }
    }
    return false;           // 全て埋まっていて追記できなかった
}

/*
 * 操作ログアップロード処理
 */
void aplwifi_upload_ope_log(APLEVT_EXTRA_DATA_T* extra_data)
{
    ER er = E_OK;

    MDLSTRG_DATA_OPE_LOG_T* data_array = (MDLSTRG_DATA_OPE_LOG_T*)(extra_data->wifi_data.ope_log_array);

    for (int i = 0; i < extra_data->wifi_data.ope_log_num; i++) {
        uint8_t post_str[APLWIFI_OPE_POST_STR_LENGTH] = {0};
        // 操作ログ情報からサーバ電文を作成
        if (aplwifi_generate_ope_log(&(data_array[i]), post_str, extra_data->wifi_data.battery) == false) {
            // アップロード対象のログではなかった
            DBGLOG0("log code was not eligible for upload.");
            // Flashにのみ保存し、次のログを処理する
            aplwifi_write_ope_log(&data_array[i]);

            continue;
        }

        // ミドル層実行
        clr_flg(FLG_APLWIFI, ~FLGPTN_MDLWIFI_UPLOAD_OPE_LOG_COMPLETE);
        mdlwifi_upload_ope_log(post_str, APLWIFI_OPE_POST_STR_LENGTH);
        er = twai_flg(FLG_APLWIFI, FLGPTN_MDLWIFI_UPLOAD_OPE_LOG_COMPLETE, TWF_ANDW, &(FLGPTN){0}, TMO_FEVR);
        assert(er == E_OK);

        // アップロード結果を操作ログ構造体(終了ログ)に追記(OK/NG)
        data_array[i].log[4].state = 5;                 // 終了
        data_array[i].log[4].processing = 1;            // 終了処理
        data_array[i].log[4].detail01 = extra_data->wifi_data.battery;
        if (apl_error_code == 0) {
            data_array[i].log[4].detail02 = 2;          // アップロードOK
        } else {
            data_array[i].log[4].detail02 = 1;          // アップロードNG
        }
        get_tim((SYSTIM*)&(data_array[i].log[4].detail03));     // 起動時間

        // 操作ログ読み込みカウンタを更新
        MDLSTRG_REQUEST_T OPE_LOG_INCREMENT_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_OPE_LOG,        // 操作ログ
            .request_type = MDLSTRG_REQ_TYPE_INCREMENT
        };

        clr_flg(FLG_APLWIFI, ~FLGPTN_MDLSTRG_REQ_COMPLETE);
        mdlstrg_request(&OPE_LOG_INCREMENT_REQ, mdlstrg_callback);
        er = twai_flg(FLG_APLWIFI, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        if (strg_result.opt1 != E_OK) {
            DBGLOG0("OPE_LOG_INCREMENT_REQ write error.");
            apl_error_code = APLWIFI_ERROR_WRITE_ERR_LOG;
        }

        // 操作ログ構造体のサーバ日時を更新
        if (apl_error_code == 0) {
            aplwifi_cvt_date_str_to_struct(res_common.now_date, &(data_array[i].log_date[1]));
        }

        // 操作ログ保存
        aplwifi_write_ope_log(&data_array[i]);

        // エラーが発生していたら打ち切る
        if (apl_error_code != 0) {
            break;
        }
    }
}

/*
 * 操作ログ電文生成処理
 */
bool_t aplwifi_generate_ope_log(MDLSTRG_DATA_OPE_LOG_T* log, uint8_t* buffer, uint8_t battery) {
    // 生成するデータは、ログコード＋付帯情報1＋付帯情報2＋発生日時＋電池残量(計22byte))
    bool_t result = true;
    
    // 状態③の大分類・中分類で分類
    if (log->log[2].state == 0x03 && log->log[2].processing == 0x00) {
        // 電源関係
        if (log->log[3].state == 0x04 && log->log[3].processing == 0x00) {
            // 起動のみ
            memcpy(buffer, "00010000", 8);
        } else {
            result = false;
        }
    } else if (log->log[2].state == 0x03 && log->log[2].processing == 0x01) {
        // 認証関連
        if (log->log[2].detail01 == 0x00 && log->log[2].detail02 == 0x02) {
            // 血流認証成功
            memcpy(buffer, "0011", 4);

            // 付帯情報1
            uint8_t num = log->log[2].detail03 >> 24;
            aplwifi_cvt_num_to_str(buffer+4, num);

            // 付帯情報2
            uint8_t cnt = log->log[2].detail03 & 0x00ffffff;
            aplwifi_cvt_num_to_str(buffer+6, cnt);

        } else if (log->log[2].detail01 == 0x00 && log->log[2].detail02 == 0x01) {
            // 血流認証失敗
            memcpy(buffer, "001200", 6);

            // 付帯情報2
            aplwifi_cvt_num_to_str(buffer+6, log->log[2].detail03);
        } else {
            result = false;
        }
    } else if (log->log[2].state == 0x03 && log->log[2].processing == 0x02) {
        // 認証関連
        if (log->log[2].detail01 == 0x00 && log->log[2].detail02 == 0x02) {
            // カード認証成功
            memcpy(buffer, "0013", 4);

            // 付帯情報1
            uint8_t cnt = log->log[2].detail03 >> 24;
            aplwifi_cvt_num_to_str(buffer+4, cnt);

            // 付帯情報2
            memcpy(buffer+6, "00", 2);
        } else if (log->log[2].detail01 == 0x00 && log->log[2].detail02 == 0x01) {
            // カード認証失敗
            memcpy(buffer, "00140000", 8);
        } else {
            result = false;
        }
    } else if (log->log[2].state == 0x03 && log->log[2].processing == 0x03) {
        if (log->log[2].detail01 == 0x10 && log->log[2].detail02 == 0x02) {
            // 初期接続
            memcpy(buffer, "0031", 4);
        } else if (log->log[2].detail01 == 0x20 && log->log[2].detail02 == 0x02) {
            // 血流登録
            memcpy(buffer, "0032", 4);
        } else if (log->log[2].detail01 == 0x21 && log->log[2].detail02 == 0x02) {
            // 血流削除
            memcpy(buffer, "0033", 4);
        } else if (log->log[2].detail01 == 0x30 && log->log[2].detail02 == 0x02) {
            // カード登録
            memcpy(buffer, "0034", 4);
        } else if (log->log[2].detail01 == 0x31 && log->log[2].detail02 == 0x02) {
            // カード削除
            memcpy(buffer, "0035", 4);
        } else if (log->log[2].detail01 == 0x41 && log->log[2].detail02 == 0x02) {
            // ユーザ削除
            memcpy(buffer, "0036", 4);
        } else if (log->log[2].detail01 == 0x50 && log->log[2].detail02 == 0x02) {
            // 緊急解錠（管理者）
            memcpy(buffer, "0037", 4);
        } else if (log->log[2].detail01 == 0x51 && log->log[2].detail02 == 0x02) {
            // 緊急解錠（一般者）
            memcpy(buffer, "0038", 4);
        } else if (log->log[2].detail01 == 0x61 && log->log[2].detail02 == 0x02) {
            // AQUBIO機器設定
            memcpy(buffer, "0039", 4);
        } else if (log->log[2].detail01 == 0xF0 && log->log[2].detail02 == 0x02) {
            // 代理スマホ接続
            memcpy(buffer, "003A", 4);
        } else {
            result = false;
        }

        // 付帯情報1
        uint8_t cnt = log->log[2].detail03 >> 24;
        aplwifi_cvt_num_to_str(buffer+4, cnt);

        // 付帯情報2
        memcpy(buffer+6, "00", 2);
    } else {
        result = false;
    }

    if (result == false && log->log[3].state == 0x04 && log->log[3].processing == 0x01) {
        result = true;
        // 電気錠関係のみ、状態④のみでログコードが決まる
        if (log->log[3].detail01 == 0x03 && log->log[3].detail02 == 0x02 && log->log[3].detail03 == 0x03) {
            // 電気錠解錠＋施錠
            memcpy(buffer, "00210000", 8);
        } else {
            result = false;
        }
    }

    if (result == false) {
        // 各種ログコードに当てはまらない場合中断する
        return false;
    }

    // 操作ログ構造体のログ発生日時を追加
    aplwifi_cvt_date_struct_to_str(&(log->log_date[0]), buffer+8);

    // バッテリー情報を追記
    aplwifi_cvt_num_to_str(buffer+20, battery);

//    cmndbg_hexdump(buffer, 22, "Operation log (for server)");

    return true;
}

/*
 * 2桁までの数値を2桁の文字列に変換
 */
void aplwifi_cvt_num_to_str(uint8_t* pos, uint8_t num) {
    assert(num < 100);      // 2桁以下の数字であること
    char str_cnt[3];
    itoa(num, str_cnt, 10);
    if (num < 10 ) {
        // 10以下ならパディング
        memcpy(pos, "0", 1);
        memcpy(pos+1, str_cnt, 1);
    } else {
        memcpy(pos, str_cnt, 2);
    }
}

/*
 * 操作ログ書き込み
 */
void aplwifi_write_ope_log(MDLSTRG_DATA_OPE_LOG_T* data) {
    DBGLOG0("aplwifi_write_ope_log");

    ER er = E_OK;

    MDLSTRG_REQUEST_T OPE_LOG_WRITE_REQ = {
        .data_type = MDLSTRG_DATA_TYPE_OPE_LOG,        // 操作ログ
        .request_type = MDLSTRG_REQ_TYPE_WRITE,        // 書込み
        .data = data,
        .opt1 = 1
    };

    clr_flg(FLG_APLWIFI, ~FLGPTN_MDLSTRG_REQ_COMPLETE);
    mdlstrg_request(&OPE_LOG_WRITE_REQ, mdlstrg_callback);
    er = twai_flg(FLG_APLWIFI, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
    assert(er == E_OK);

    if (strg_result.opt1 != E_OK) {
        DBGLOG0("OPE_LOG_WRITE_REQ Write error.");
        apl_error_code = APLWIFI_ERROR_WRITE_ERR_LOG;
    }
}

/*
 * 空室カードダウンロード処理
 */
void aplwifi_download_vacancy_card()
{
    ER er = E_OK;

    // ミドル層実行
    clr_flg(FLG_APLWIFI, ~FLGPTN_MDLWIFI_DOWNLOAD_VACANCY_CARD_DOWNLOAD);
    mdlwifi_download_vacancy_card();
    er = twai_flg(FLG_APLWIFI, FLGPTN_MDLWIFI_DOWNLOAD_VACANCY_CARD_DOWNLOAD, TWF_ANDW, &(FLGPTN){0}, TMO_FEVR);
    assert(er == E_OK);

    // エラーがあれば中断
    if (apl_error_code != 0) {
        DBGLOG0("apl_error_code != 0");
        return;
    }

    // 空室カード初期化
    DBGLOG0("VACANCY_CARD_ALL_DELETE_REQ");
    MDLSTRG_REQUEST_T VACENCY_CARD_ALL_DELETE_REQ = {
        .data_type = MDLSTRG_DATA_TYPE_VACANCY_CARD,
        .request_type = MDLSTRG_REQ_TYPE_ALL_DELETE,
    };
    clr_flg(FLG_APLWIFI, ~FLGPTN_MDLSTRG_REQ_COMPLETE);
    mdlstrg_request(&VACENCY_CARD_ALL_DELETE_REQ, mdlstrg_callback);
    er = twai_flg(FLG_APLWIFI, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
    assert(er == E_OK);

    // 取得したカードの枚数分カード保存
    MDLSTRG_DATA_CARD_T data_card;
    memcpy(data_card.card_type, download_card_type, 16);
    memcpy(data_card.member_num, download_member_num, 16);

    for (int i = 0; i < APLWIFI_VACANCY_MAX; i++) {
        // TEST CODE
        // if (i == 6) {
        //     memcpy(res_vecancy_card.idm[i], empty_card_data, 8);
//             cmndbg_hexdump(&(res_vecancy_card.idm[i]), 8, "res_vecancy_card idm");
        // }

        if (memcmp(res_vecancy_card.idm[i], empty_card_data, 8) == 0) {
            // All 0x00の場合、データが入っていないため、追加処理をスキップする
            DBGLOG1("VACANCY_CARD_EMPTY_DATA : index = %d", i);
            continue;
        }
        memcpy(data_card.idm, res_vecancy_card.idm[i], 8);

        DBGLOG1("VACANCY_CARD_WRITE_REQ : index = %d", i);
        MDLSTRG_REQUEST_T VACANCY_CARD_WRITE_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_VACANCY_CARD,
            .request_type = MDLSTRG_REQ_TYPE_APPEND,
            .data = &data_card,
        };
        clr_flg(FLG_APLWIFI, ~FLGPTN_MDLSTRG_REQ_COMPLETE);
        mdlstrg_request(&VACANCY_CARD_WRITE_REQ, mdlstrg_callback);
        er = twai_flg(FLG_APLWIFI, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        
        if (strg_result.opt1 != E_OK) {
            // MDLSTRG 書き込みエラー
            DBGLOG0("VACANCY_CARD_WRITE_REQ : ERROR");
            apl_error_code = APLWIFI_ERROR_WRITE_CARD;
            break;
        }
    }
}

/*
 * YYYYMMDDhhmm形式の文字列を日付構造体に変換
 */
void aplwifi_cvt_date_str_to_struct(uint8_t* str_date, MDLSTRG_DATA_STRACT_DATE_T* output) {
    uint8_t buff[5] = {0};
    memcpy(buff, str_date, 4);
    buff[4] = '\0';
    output->year  = atoi(buff);
    memcpy(buff, str_date+4, 2);
    buff[2] = '\0';
    output->month = atoi(buff);
    memcpy(buff, str_date+6, 2);
    buff[2] = '\0';
    output->day   = atoi(buff);
    memcpy(buff, str_date+8, 2);
    buff[2] = '\0';
    output->hour  = atoi(buff);
    memcpy(buff, str_date+10, 2);
    buff[2] = '\0';
    output->min   = atoi(buff);
    output->dmy   = 0;
}

/*
 * 日付構造体をYYYYMMDDhhmm形式の文字列に変換
 */
void aplwifi_cvt_date_struct_to_str(MDLSTRG_DATA_STRACT_DATE_T* date, uint8_t* output_buf) {
    uint8_t str[5];
    sprintf(str, "%04d", date->year);
    memcpy(output_buf, str, 4);

    sprintf(str, "%02d", date->month);
    memcpy(output_buf+4, str, 2);
    sprintf(str, "%02d", date->day);
    memcpy(output_buf+6, str, 2);
    sprintf(str, "%02d", date->hour);
    memcpy(output_buf+8, str, 2);
    sprintf(str, "%02d", date->min);
    memcpy(output_buf+10, str, 2);
}

/*
 * WIFIミドルウェアコールバック
 */
void mdlwifi_callback(int event, intptr_t error, intptr_t opt1, intptr_t opt2)
{
    ER er = 0;

    DBGLOG2("mdlwifi_callback: evt=%d, drverror=%d", event, error);

    // エラーコード生成
    switch (error) {
    case MDLWIFI_ERR_NONE:
        apl_error_code = 0;
        break;
    case MDLWIFI_ERR_INITIALIZE:
        apl_error_code = APLWIFI_ERROR_INITIALIZE;
        break;
    case MDLWIFI_ERR_AP_CONNECT:
        apl_error_code = APLWIFI_ERROR_AP_CONNECT;
        break;
    case MDLWIFI_ERR_HTTP_CONNECT:
        apl_error_code = APLWIFI_ERROR_HTTPS_CONNECT;
        break;
    case MDLWIFI_ERR_HTTP_POST:
        apl_error_code = APLWIFI_ERROR_HTTPS_POST;
        break;
    case MDLWIFI_ERR_AP_DISCONNECT:
        apl_error_code = APLWIFI_ERROR_AP_DISCONNECT;
        break;
    case MDLWIFI_ERR_HTTP_DISCONNECT:
        apl_error_code = APLWIFI_ERROR_HTTPS_DISCONNECT;
        break;
    case MDLWIFI_ERR_POWER_OFF:
        // POWER OFFはエラーにしない
        //apl_error_code = APLWIFI_ERROR_POWER_OFF;
        break;
    default:
        assert(false);
        break;
    }

    if (opt1 != NULL) {
        memcpy(&res_common, opt1, sizeof(MDLWIFI_RES_COMMON_T));
    }

    switch(event) {
    case MDLWIFI_EVT_INITIALIZE_COMPLETE:
        // ミドル初期化完了
        set_flg(FLG_APLWIFI, FLGPTN_MDLWIFI_INITIALIZE_COMPLETE);
        DBGLOG0("[CALLBACK]MDLWIFI_EVT_INITIALIZE_COMPLETE");

        break;
    case MDLWIFI_EVT_READY_COMPLETE:
        // WIFI開始完了
        set_flg(FLG_APLWIFI, FLGPTN_MDLWIFI_READY_COMPLETE);
        DBGLOG1("[CALLBACK]MDLWIFI_EVT_READY_COMPLETE : %d", apl_error_code);
        aplevt_send_event(APLEVT_WIFI_READY_COMPLETE, apl_error_code, NULL, s_event_dest);
        break;
    case MDLWIFI_EVT_STOP_COMPLETE:
        // WIFI制御中止完了
        set_flg(FLG_APLWIFI, FLGPTN_MDLWIFI_STOP_COMPLETE);
        DBGLOG1("[CALLBACK]MDLWIFI_EVT_STOP_COMPLETE : %d", apl_error_code);
        aplevt_send_event(APLEVT_WIFI_STOP_COMPLETE, apl_error_code, NULL, s_event_dest);
        break;
    case MDLWIFI_EVT_SERVERTASK_COMPLETE:
        // サーバタスク確認
        DBGLOG1("[CALLBACK]MDLWIFI_EVT_SERVERTASK_COMPLETE : %d", apl_error_code);
        memcpy(extra_data.wifi_data.date, ((MDLWIFI_RES_COMMON_T*)opt1)->now_date, APLEVT_EXTRA_DATE_SIZE); // 現在時刻を渡す

        if (apl_error_code != 0) {
            // もしエラーなら終了
            aplevt_send_event(APLEVT_WIFI_CHECK_SERVERTASK_COMPLETE, apl_error_code, &extra_data, s_event_dest);
        } else {
            // サーバタスク情報をコピー
            memcpy(&res_server_task, opt2, sizeof(MDLWIFI_RES_SERVERTASK_T));
        }
        set_flg(FLG_APLWIFI, FLGPTN_MDLWIFI_CHECK_SERVERTASK_COMPLETE);
        break;
    case MDLWIFI_EVT_DOWNLOAD_CARD_COMPLETE:
        // 空室カードダウンロード完了
        DBGLOG1("[CALLBACK]MDLWIFI_EVT_DOWNLOAD_CARD_COMPLETE : %d", apl_error_code);
        if (apl_error_code == 0) {
            // 空室カード情報をコピー
            memcpy(&res_vecancy_card, opt2, sizeof(MDLWIFI_RES_VACANCY_CARD_T));
        }
        // if (apl_error_code != 0) {
        //     // もしエラーなら抜ける（空室カードダウンロードは、サーバタスク確認処理の１機能）
        //     //aplevt_send_event(APLEVT_WIFI_CHECK_SERVERTASK_COMPLETE, apl_error_code, &extra_data, s_event_dest);
        // } else {
        //     // 空室カード情報をコピー
        //     memcpy(&res_vecancy_card, opt2, sizeof(MDLWIFI_RES_VACANCY_CARD_T));
        // }
        memcpy(extra_data.wifi_data.date, ((MDLWIFI_RES_COMMON_T*)opt1)->now_date, APLEVT_EXTRA_DATE_SIZE); // 現在時刻を渡す
        // cmndbg_hexdump(extra_data.wifi_data.date, APLEVT_EXTRA_DATE_SIZE, "wifi_data.date");
//         cmndbg_hexdump(((MDLWIFI_RES_COMMON_T*)opt1)->now_date, APLEVT_EXTRA_DATE_SIZE, "opt1->now_date");
        set_flg(FLG_APLWIFI, FLGPTN_MDLWIFI_DOWNLOAD_VACANCY_CARD_DOWNLOAD);
        break;
    case MDLWIFI_EVT_UPLOAD_OPE_LOG_COMPLETE:
        // 操作ログアップロード完了
        DBGLOG1("[CALLBACK]MDLWIFI_EVT_UPLOAD_OPE_LOG_COMPLETE : %d", apl_error_code);
        memcpy(extra_data.wifi_data.date, ((MDLWIFI_RES_COMMON_T*)opt1)->now_date, APLEVT_EXTRA_DATE_SIZE); // 現在時刻を渡す
        set_flg(FLG_APLWIFI, FLGPTN_MDLWIFI_UPLOAD_OPE_LOG_COMPLETE);
        break;
    case MDLWIFI_EVT_UPLOAD_ERR_LOG_COMPLETE:
        // エラーログアップロード完了
        DBGLOG1("[CALLBACK]MDLWIFI_EVT_UPLOAD_ERR_LOG_COMPLETE : %d", apl_error_code);
        memcpy(extra_data.wifi_data.date, ((MDLWIFI_RES_COMMON_T*)opt1)->now_date, APLEVT_EXTRA_DATE_SIZE); // 現在時刻を渡す
        set_flg(FLG_APLWIFI, FLGPTN_MDLWIFI_UPLOAD_ERR_LOG_COMPLETE);
        break;
    default:
        assert(false);
        break;
    }
}

/*
 * ストレージミドルコールバック
 */
void mdlstrg_callback(int event, intptr_t opt1, intptr_t opt2)
{
    DBGLOG1("mdlstrg_callback: event=%d", event);

    strg_result.opt1 = opt1;
    strg_result.opt2 = opt2;

    switch(event) {
    case MDLSTRG_EVT_REQUEST_COMPLETE:
        DBGLOG1("mdlstrg_callback: MDLSTRG_EVT_REQUEST_COMPLETE : opt1 = %d", opt1);
        set_flg(FLG_APLWIFI, FLGPTN_MDLSTRG_REQ_COMPLETE);
        break;
    default:
        assert(false);
        break;
    }

}
