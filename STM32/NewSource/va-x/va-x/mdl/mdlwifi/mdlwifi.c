/*
 * VA-X Wifiミドル
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/03/29 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "mdlwifi.h"

#include <string.h>

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "kernel_cfg.h"
#include "mdlstrg_data.h"
#include "drvwifi.h"
#include "drvrng.h"


/*
 * マクロ定義
 */

// デバッグログ
#if 0
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[MDLWIFI]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[MDLWIFI]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[MDLWIFI]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[MDLWIFI]" msg, arg1, arg2, arg3)
#define DBGLOG4(msg, arg1, arg2, arg3, arg4)	syslog(LOG_NOTICE, "[MDLWIFI]" msg, arg1, arg2, arg3, arg4)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#define DBGLOG4(msg, arg1, arg2, arg3, arg4)
#endif



// バッファサイズ
#define RECEIVE_POST_SIZE	(2048)

// サービスコールのエラーのログ出力
static inline void svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}
#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))


#define RES_KEY_STR_TASK_CODE1              "\"task_code1\""
#define RES_KEY_STR_TASK_CODE2              "\"task_code2\""
#define RES_KEY_STR_TASK_CODE3              "\"task_code3\""
#define RES_KEY_STR_TASK_CODE4              "\"task_code4\""
#define RES_KEY_STR_NOW_DATE                "\"now_date\""
#define RES_KEY_STR_DOWNLOAD_DATA           "\"download_data\""
#define RES_KEY_STR_RESULT                  "\"result\""
#define RES_KEY_STR_MESSAGE                 "\"message\""

#define TASK_CODE_NONE                  "00"
#define TASK_CODE_VACANCY_CARD          "01"

#define SERIAL_NO_LENGTH 16

// デバッグ用コード有効
#define ENABLE_DEBUG_CODE


/*
 * 型定義
 */
// メモリプールのブロック
typedef struct {
    int msg;
    uint8_t service;
    intptr_t data;
    size_t length;
    intptr_t opt1;
} MDLWIFI_MPFBLK_T;

// MDLWIFI_MPFBLK_SIZE は sizeof(MDLWIFI_MPFBKL) に一致させること
static_assert(sizeof(MDLWIFI_MPFBLK_T) == MDLWIFI_MPFBLK_SIZE, "MPF size");

// コンテキスト
typedef struct {
    MDLWIFI_CALLBACK_T callback;
    BNVA_DEVICE_ID_T device_id;
    int status;
    int req_kind;
    uint8_t receive_data[RECEIVE_POST_SIZE];
    MDLWIFI_RES_COMMON_T res_common;
    MDLWIFI_RES_SERVERTASK_T res_servertask;
    MDLWIFI_RES_VACANCY_CARD_T res_vacancy_card;
} MDLWIFI_CONTEXT_T;

// レスポンス定義構造体
typedef struct {
    int res_kind;
    unsigned char* key_str;
    int data_length;
} MDLWIFI_RES_DEF_T;

// 空室ダウンロードデータ用構造体
typedef struct {
    uint8_t key[6];                 // 文字列のため文字数+1
    uint8_t rnd[14];
    uint8_t task_num[3];            // 文字列のため文字数+1
} MDLWIFI_DOWNLOAD_VACANCY_POST_DATA_T;

/*
 * 内部関数プロトタイプ
 */
// メッセージ送信
static void mpf_send(int msg, uint8_t service, intptr_t data, size_t length, intptr_t opt1);

// WIFIドライバのコールバック
static void drvwifi_callback(int32_t evt, int32_t error, intptr_t opt);

// WiFi初期化処理（ドライバタスク起動）
static void mwifi_initialize();

// WiFi開始処理
static void mwifi_start(MDLSTRG_DATA_WIFI_PARAM_T* data);

// サーバタスク確認処理
static void mwifi_check_servertask(MDLSTRG_DATA_WIFI_PARAM_T* data);

// ログアップロード(操作ログ、エラーログ共通処理)
static void mwifi_upload_log(uint8_t* data, int length);

// 空室カードダウンロード
static void mwifi_download_vacancy_card();

// HTTPS接続処理
static void mwifi_connect_https();

// WIFIレスポンス解析
static bool_t mwifi_response_parser();

// WIFIレスポンス解析（１データ分）
static void mwifi_response_parser_data(char* pos, int length);

// WIFIレスポンス解析動作確認用
static void mwifi_test_response_parser();

/*
 * 定数定義
 */

// メッセージ番号
enum {
    MSG_INITIALIZE = 1,
    MSG_START,
    MSG_STOP,
    MSG_CHECK_TASK,
    MSG_UPLOAD_ERR_LOG,
    MSG_UPLOAD_OPE_LOG,
    MSG_DOWNLOAD_CARD,
};

// WIFIミドルの状態
enum {
    STATUS_INITIALIZED = 1,     /// 初期状態
    //STATUS_STARTED,           /// 開始状態
    STATUS_CONNECTED,           /// WIFI接続状態
    STATUS_DISCONNECTED,        /// 切断状態
};

// Postで送信したリクエストの種類
enum {
    REQ_SERVERTASK = 1,
    REQ_UPLOAD_ERR_LOG,
    REQ_UPLOAD_OPE_LOG,
    REQ_DOWNLOAD_CARD,
};

// パーサーの内部状態
enum {
    PARSE_START,
    PARSE_DATA,
    PARSE_END,
    PARSE_ERROR,
};

// イベントフラグ
static const FLGPTN FLGPTN_DRVWIFI_INITIALIZE_COMPLETE =		(0x1 << 0);
static const FLGPTN FLGPTN_DRVWIFI_READY_COMPLETE =             (0x1 << 1);
static const FLGPTN FLGPTN_DRVWIFI_HTTPS_CONNECT_COMPLETE =     (0x1 << 2);
static const FLGPTN FLGPTN_DRVWIFI_HTTPS_DISCONNECT_COMPLETE =  (0x1 << 3);
static const FLGPTN FLGPTN_DRVWIFI_AP_DISCONNECT_COMPLETE =     (0x1 << 4);
static const FLGPTN FLGPTN_DRVWIFI_POWEROFF_COMPLETE =          (0x1 << 5);
static const FLGPTN FLGPTN_DRVWIFI_HTTPS_POST_TASK_COMPLETE =   (0x1 << 6);


// データ受信タイムアウト
static const int32_t DTQTIMEOUT = 100;

static const TMO DRV_CALLBACK_TIMEOUT	=	3000;

// パーサー動作確認用文字列(デバッグ用)
// 20181016受領 テストサーバにあわせたテストデータ
static unsigned char str_test_task[] = "{\"result\":true,\"now_date\":\"201810121401\",\"task_code1\":\"01\",\"task_code2\":\"02\",\"task_code3\":\"00\",\"task_code4\":\"00\"}";
static unsigned char str_test_ok[] = "{\"result\":true,\"now_date\":\"201809211234\"}";
static unsigned char str_test_err[] = "{\"result\":false,\"now_date\":\"201809211234\"}";
static unsigned char str_test_card_base64[] = "{\"result\":true,\"now_date\":\"201810121411\",\"download_data\":\"mjS+crHc1H5o6pdbEbBMDW9c2WQVdeyGKt6B5LSGpC2rWE9MZPRxaWfpOqa6+of\\/PuisxGmikDvw\\njP41aYA7cgV6bBsGSimtWDjjpeh3QeqjIXsB0iSWVS86n2xdNuxm7B3MQL7G5Zri2x7Sbo94Xj2H\\nlHYAhd\\/f82J7+mX6mrcjmZSDm3h++ywvUsQ88fCsLeSeVnuR6ohCXPKnUR5j4Q==\",\"message\":\"card data\"}";
static unsigned char str_test_card[] = "{\"result\":true,\"now_date\":\"201810121411\",\"download_data\":\"00000000111111112222222233333333444444445555555566666666777777778888888899999999AAAAAAAABBBBBBBBCCCCCCCCDDDDDDDDEEEEEEEE00000000aaaaaaaabbbbbbbbccccccccdddddddd\",\"message\":\"card data\"}";

// TODO 暗号化後のポストデータ（デバッグ用）
//static unsigned char str_test_download_vacancy_post_data[] = "wa3X5hxILOMpO2zl7ldFSYM/9LZFLZ8VdCUquTp6d2w=";
static unsigned char str_test_download_vacancy_post_data[] = "+naua4dNMf2h1meN9tEGTXR/24iqcOhiokCcyPEsyKMbWGn6QtrqdXeGmlT4LKQy";
static unsigned char str_test_update_log_post_data[] = "BMQp2GCtw9/IUXT0S6SvmZcSzPZXoPCbC/rDctk+0jfQqccEYa5rFf4GqlFVxkUTNtsQOK5Ypsxgq2TPzPHnhA==";
static unsigned char str_test_check_task_post_data[] = "BMQp2GCtw9/IUXT0S6SvmaYO9bVU+8qDiOwjCm7HptNnuYVk6FSgcdCnwiJSdPN0pPrejJO0BU7ITm2eMZZf1g==";

/*
 * 内部変数
 */
static MDLWIFI_CONTEXT_T s_context = {0};

static int drvwifi_result = E_OK;

// WIFI接続設定
static DRVWIFI_CONFIG conf = {0};

// HTTPS接続設定
static DRVWIFI_HTTPS_CONFIG hconf = {0};

// HTTPSリクエスト設定
static DRVWIFI_HTTPS_REQ req = {0};

// 操作ログ/エラーログアップロード Postデータ用バッファ(TODO:セキュリティが入った場合、バッファ容量が変わる可能性あり)
static uint8_t upload_log_post_data_buff[100];

// 空室ダウンロード用データ
static MDLWIFI_DOWNLOAD_VACANCY_POST_DATA_T download_vacancy_post_data;

// 空室ダウンロード Postデータ用バッファ（TODO:セキュリティが入った場合、バッファ容量が変わる可能性あり）
static uint8_t download_vacancy_post_data_buff[100];

// レスポンスキー種別
enum {
    RES_KEY_RESULT,
    RES_KEY_NOW_DATE,
    RES_KEY_TASK_CODE1,
    RES_KEY_TASK_CODE2,
    RES_KEY_TASK_CODE3,
    RES_KEY_TASK_CODE4,
    RES_KEY_DOWNLOAD_DATA,
    RES_KEY_MESSAGE,
};

// サーバアドレス
#ifdef HTTP_LOCALSERVER
static uint8_t hnm[] = {"219.75.137.238"};            // テストサーバ(HTTP)
#else
//static uint8_t hnm[] = {"v4.aqubio.jp"};              // テストサーバ(HTTPS)
static uint8_t hnm[] = {"d4.aqubio.jp"};              // テストサーバ(HTTPS)
#endif

// ログアップロードパス
static uint8_t ph_log_up[] = {"/dapi/v1/device_log/add"};

// カードダウンロードパス
static uint8_t ph_download_card[] = {"/dapi/v1/device_log/get"};

static const MDLWIFI_RES_DEF_T MDLWIFI_RES_DEFS[] = {
    //      int             res_kind;               // レスポンスキー種別
    //      unsigned char*  key_str;                // キー文字列
    //      int             data_length;            // データ文字列長

    // res_kind                 key_str                     data_lendgth
    { RES_KEY_RESULT,           RES_KEY_STR_RESULT,             0,      },
    { RES_KEY_NOW_DATE,         RES_KEY_STR_NOW_DATE,          12,      },
    { RES_KEY_TASK_CODE1,       RES_KEY_STR_TASK_CODE1,         2,      },
    { RES_KEY_TASK_CODE2,       RES_KEY_STR_TASK_CODE2,         2,      },
    { RES_KEY_TASK_CODE3,       RES_KEY_STR_TASK_CODE3,         2,      },
    { RES_KEY_TASK_CODE4,       RES_KEY_STR_TASK_CODE4,         2,      },
    { RES_KEY_DOWNLOAD_DATA,    RES_KEY_STR_DOWNLOAD_DATA,    160,      },
    { RES_KEY_MESSAGE,          RES_KEY_STR_MESSAGE,            0,      },
};

#ifdef ENABLE_DEBUG_CODE
// デバッグ出力用操作ログ退避変数
static int8_t log_data_buffer_for_debug[48];
#endif

/*********************************************************************
 * 公開関数
 ********************************************************************/

/*
 * 初期化
 */
void mdlwifi_initialize(MDLWIFI_CALLBACK_T callback)
{
    assert(!s_context.callback);

    // コールバック関数を保持
    s_context.callback = callback;

    // タスクを起動
    ER er = E_OK;
    er = act_tsk(TSK_MDLWIFI);
    assert(er == E_OK);

    // 初期化処理
    mpf_send(MSG_INITIALIZE, 0, (intptr_t)NULL, 0, NULL);
}

/*
 * 開始
 */
void mdlwifi_start(intptr_t data, size_t length, const BNVA_DEVICE_ID_T* serialno)
{
    // 機器識別情報を保持
    memcpy(s_context.device_id.serialno, serialno->serialno, SERIAL_NO_LENGTH);

    mpf_send(MSG_START, 0, data, length, NULL);
}

/*
 * 制御中止
 */
void mdlwifi_stop()
{
    mpf_send(MSG_STOP, 0, NULL, 0, 0);
}

/*
 * サーバタスク確認
 */
void mdlwifi_check_servertask(intptr_t data, size_t length)
{
    mpf_send(MSG_CHECK_TASK, 0, data, length, 0);
}

/*
 * エラーログアップロード処理
 */
void mdlwifi_upload_err_log(intptr_t data, size_t length)
{
    mpf_send(MSG_UPLOAD_ERR_LOG, 0, data, length, 0);
}

/*
 * 操作ログアップロード処理
 */
void mdlwifi_upload_ope_log(intptr_t data, size_t length)
{
    mpf_send(MSG_UPLOAD_OPE_LOG, 0, data, length, 0);
}

/*
 * 空室カードダウンロード処理
 */
void mdlwifi_download_vacancy_card()
{
    mpf_send(MSG_DOWNLOAD_CARD, 0, NULL, 0, 0);
}


/*
 * タスク
 */
void mdlwifi_task(intptr_t exinf)
{
    syslog(LOG_NOTICE, "mdlwifi_task() starts .");

#if 0
    // 内部関数動作確認
    mwifi_test_response_parser();
    //mwifi_download_vacancy_card();
#endif
    int result = MDLWIFI_ERR_NONE;

    while (true) {
        MDLWIFI_MPFBLK_T* blk = NULL;
        ER er = 0;
        er = trcv_dtq(DTQ_MDLWIFI, (intptr_t*)&blk, TMO_FEVR);
        assert(er == E_OK);
        int error = 0;
        switch (blk->msg) {
        case MSG_INITIALIZE:
            mwifi_initialize();
            break;
        case MSG_START: {
            mwifi_start((MDLSTRG_DATA_WIFI_PARAM_T*)blk->data);
            // コールバック
            s_context.callback(MDLWIFI_EVT_READY_COMPLETE, drvwifi_result, NULL, NULL);
            break;
        }
        case MSG_STOP: {
            FLGPTN flgptn = 0;

            // 接続中なら切断する
            if (s_context.status == STATUS_CONNECTED) {

                // 各シーケンスでDISCONNECTしているためコメントアウト
                // clr_flg(FLG_MDLWIFI, ~FLGPTN_DRVWIFI_HTTPS_DISCONNECT_COMPLETE);
                // drvwifi_https_disconnect(drvwifi_callback);
                // er = twai_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_HTTPS_DISCONNECT_COMPLETE, TWF_ANDW, &flgptn, TMO_FEVR);
                // assert((er == E_OK) || (er == E_TMOUT));

                clr_flg(FLG_MDLWIFI, ~FLGPTN_DRVWIFI_AP_DISCONNECT_COMPLETE);
                drvwifi_ap_disconnect(drvwifi_callback);
                er = twai_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_AP_DISCONNECT_COMPLETE, TWF_ANDW, &flgptn,  TMO_FEVR);
                assert((er == E_OK) || (er == E_TMOUT));
            }

            s_context.status = STATUS_DISCONNECTED;

            // 途中でエラーが発生しても、電源は落とす
            clr_flg(FLG_MDLWIFI, ~FLGPTN_DRVWIFI_POWEROFF_COMPLETE);
            drvwifi_power_off(drvwifi_callback);
            er = twai_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_POWEROFF_COMPLETE, TWF_ANDW, &flgptn,  TMO_FEVR);

            // コールバック
            s_context.callback(MDLWIFI_EVT_STOP_COMPLETE, DRVWIFI_ERROR_NONE, NULL, NULL);
            break;
        }
        case MSG_CHECK_TASK: {
            mwifi_check_servertask((MDLSTRG_DATA_WIFI_PARAM_T*)blk->data);

            // コールバック
            s_context.callback(MDLWIFI_EVT_SERVERTASK_COMPLETE, drvwifi_result , &(s_context.res_common), &(s_context.res_servertask));
            break;
        }
        case MSG_UPLOAD_ERR_LOG:
            s_context.req_kind = REQ_UPLOAD_ERR_LOG;
            mwifi_upload_log(blk->data, blk->length);
            s_context.callback(MDLWIFI_EVT_UPLOAD_ERR_LOG_COMPLETE, drvwifi_result, &(s_context.res_common), NULL);
            break;
        case MSG_UPLOAD_OPE_LOG: {
            s_context.req_kind = REQ_UPLOAD_OPE_LOG;
            mwifi_upload_log(blk->data, blk->length);
            s_context.callback(MDLWIFI_EVT_UPLOAD_OPE_LOG_COMPLETE, drvwifi_result, &(s_context.res_common), NULL);
            break;
        }
        case MSG_DOWNLOAD_CARD:
            mwifi_download_vacancy_card();
            s_context.callback(MDLWIFI_EVT_DOWNLOAD_CARD_COMPLETE, drvwifi_result , &(s_context.res_common), &(s_context.res_vacancy_card));
            break;
        default:
            assert(false);
            break;
        }

        er = rel_mpf(MPF_MDLWIFI, blk);
        assert(er == E_OK);
    }
}

/*********************************************************************
 * 内部関数
 ********************************************************************/
/*
 * メッセージ送信
 */
void mpf_send(int msg, uint8_t service, intptr_t data, size_t length, intptr_t opt1)
{
    MDLWIFI_MPFBLK_T* blk = NULL;
    ER er = 0;
    er = tget_mpf(MPF_MDLWIFI, (void**)&blk, DTQTIMEOUT);
    assert(er == E_OK);

    blk->msg = msg;
    blk->service = service;
    blk->data = data;
    blk->length = length;
    blk->opt1 = opt1;
    er = tsnd_dtq(DTQ_MDLWIFI, (intptr_t)blk, DTQTIMEOUT);
    assert(er == E_OK);
}

/*
 * 初期化
 */
void mwifi_initialize()
{
    // ドライバ初期化
    drvwifi_initialize(drvwifi_callback);

    // 状態変更
    s_context.status = STATUS_INITIALIZED;

    // 完了コールバック
    s_context.callback(MDLWIFI_EVT_INITIALIZE_COMPLETE, 0, NULL, NULL);
}

/*
 * WIFI開始処理
 */
void mwifi_start(MDLSTRG_DATA_WIFI_PARAM_T* data)
{
    DBGLOG0("mwifi_start start");

    int result = MDLWIFI_ERR_NONE;

    // WIFI開始
    clr_flg(FLG_MDLWIFI, ~FLGPTN_DRVWIFI_INITIALIZE_COMPLETE);
    drvwifi_start(drvwifi_callback);
    FLGPTN flgptn = 0;
    ER er = twai_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_INITIALIZE_COMPLETE, TWF_ANDW, &flgptn, TMO_FEVR);
    assert((er == E_OK) || (er == E_TMOUT));

    // ここでエラーが発生した場合は、キャンセルする
    if (drvwifi_result != MDLWIFI_ERR_NONE) {
        DBGLOG0("drvwifi_start() : error");
        return;
    }

    // WIFI接続設定を実行
    #if 1
    conf.essid_len = data->essid_len;
    conf.passphrase_len = data->passphrase_len;
    conf.essid = data->essid;
    conf.passphrase = data->passphrase;
    conf.wpaver = DRVWIFI_SECURITY_WPA2RNS;
    conf.cipher = DRVWIFI_SECURITY_CCMP;
    #else
    conf.essid_len = 8;
    conf.passphrase_len = 8;
    memcpy(conf.essid, "ZMI_90D1", conf.essid_len);
    memcpy(conf.passphrase, "11622752", conf.passphrase_len);
    conf.wpaver = DRVWIFI_SECURITY_WPA2RNS;
    conf.cipher = DRVWIFI_SECURITY_CCMP;
    #endif

    clr_flg(FLG_MDLWIFI, ~FLGPTN_DRVWIFI_READY_COMPLETE);
    drvwifi_ap_connect(drvwifi_callback, &conf);
    er = twai_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_READY_COMPLETE, TWF_ANDW, &flgptn, TMO_FEVR);
    assert((er == E_OK) || (er == E_TMOUT));

    if (drvwifi_result == MDLWIFI_ERR_NONE) {
        // 状態更新
        DBGLOG0("DRVWIFI is CONNECTED");
        s_context.status = STATUS_CONNECTED;
    }
    return;
}

/*
 * サーバタスク確認処理
 */
void mwifi_check_servertask(MDLSTRG_DATA_WIFI_PARAM_T* data)
{
    DBGLOG0("mwifi_check_servertask : start");
    ER er = 0;
    FLGPTN flgptn = 0;

    // 電文生成
    uint8_t data_buffer[48];                        // 暗号化の対象となるlogの値
    drvrng_get_random(data_buffer, 10);
    memcpy(&(data_buffer[10]), s_context.device_id.serialno, SERIAL_NO_LENGTH);
    // ログコードを'0000'にした場合、ログを記録せずにタスクの情報のみ返却
    // ログコード、付帯情報、発生日時、電池残量を全て'0'で埋める
    memcpy(&(data_buffer[26]), "0000000000000000000000", 22);

    // TODO：テストサーバ向け対応（暗号化文字列に置き換え）
    // 本来であれば、data_bufferの中身を暗号化する
    memcpy(upload_log_post_data_buff, "log=", 4);
    memcpy(&(upload_log_post_data_buff[4]), str_test_check_task_post_data, 89);
    DBGLOG1("Upload_log_post_data(for check task) = %s", upload_log_post_data_buff);

    // WIFIが接続状態でなければ接続する
    if (s_context.status != STATUS_CONNECTED) {
        mwifi_start(data);
        if (drvwifi_result != MDLWIFI_ERR_NONE) {
            // 初期化時にエラーが発生したら中断する
            DBGLOG0("drvwifi_start() : error");
            return;
        }
    }

    mwifi_connect_https();

    if (drvwifi_result != MDLWIFI_ERR_NONE) {
        // HTTPS接続でエラーが発生したら、ここで抜ける
        DBGLOG0("mwifi_connect_https() : error");
        return;
    }

    // Post通信リクエスト設定
    req.path = "/dapi/v1/device_log/add";
	req.path_len = 23;		// 文字数を設定すること
    req.body = upload_log_post_data_buff;
    req.body_len = 93;       // 文字数を設定すること

    s_context.req_kind = REQ_SERVERTASK;

    // Post通信
    clr_flg(FLG_MDLWIFI, ~FLGPTN_DRVWIFI_HTTPS_POST_TASK_COMPLETE);
    drvwifi_https_post(drvwifi_callback, &req);
    er = twai_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_HTTPS_POST_TASK_COMPLETE, TWF_ANDW, &flgptn, TMO_FEVR);
    assert(er == E_OK);

#if 0
    if (drvwifi_result != MDLWIFI_ERR_NONE) {
        // Post通信エラーで中断
        return;
    }
#else
    // TODO : サーバからタスクが返送されないためデバッグ用にテストデータで上書きする。
    DBGLOG0("FOR DEBUG : error cancel, and replace receive_data");
    DBGLOG1("str = %s", str_test_task);
    drvwifi_result = MDLWIFI_ERR_NONE;                          // エラーコードを削除
    strcpy(&s_context.receive_data, str_test_task);
    //memcpy(&s_context.receive_data, str_test_task, 114);        // テストデータで上書き
#endif

    // HTTPSクライアント切断
    clr_flg(FLG_MDLWIFI, ~FLGPTN_DRVWIFI_HTTPS_DISCONNECT_COMPLETE);
    drvwifi_https_disconnect(drvwifi_callback);
    er = twai_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_HTTPS_DISCONNECT_COMPLETE, TWF_ANDW, &flgptn, TMO_FEVR);
    assert(er == E_OK);

    // 電文解析
    if (mwifi_response_parser() != E_OK) {
        drvwifi_result = MDLWIFI_ERR_HTTP_POST; // 解析できない電文はPOSTのエラーとして処理
    }

    // 電文の中身をダンプ（デバッグ用）
    DBGLOG0("--- Response Data ----");
    DBGLOG1("result %d", s_context.res_common.res_result);
    cmndbg_hexdump(s_context.res_common.now_date, 12, "now_date");
    cmndbg_hexdump(s_context.res_servertask.task_code1, 2, "task_code1");
    cmndbg_hexdump(s_context.res_servertask.task_code2, 2, "task_code2");
    cmndbg_hexdump(s_context.res_servertask.task_code3, 2, "task_code3");
    cmndbg_hexdump(s_context.res_servertask.task_code4, 2, "task_code4");

    if (s_context.res_common.res_result == false) {
        DBGLOG0("Response result is false.");
        drvwifi_result = MDLWIFI_ERR_HTTP_POST; // サーバからfalseが返ってきたらPOSTのエラーとして処理
    }
}

/*
 * ログアップロード（操作ログ、エラーログ共通処理）
 * 呼び出す前に、s_context.req_kindに呼び出し元を入れること
 */
void mwifi_upload_log(uint8_t* data, int length)
{
    DBGLOG1("mwifi_upload_log (req_kind : %d)", s_context.req_kind);

    ER er = E_OK;
    FLGPTN flgptn = 0;

    // 電文生成
    uint8_t data_buffer[48];                        // 暗号化の対象となるlogの値
    drvrng_get_random(data_buffer, 10);
    memcpy(&(data_buffer[10]), s_context.device_id.serialno, SERIAL_NO_LENGTH);
    memcpy(&(data_buffer[26]), data, length);

#ifdef ENABLE_DEBUG_CODE
    // デバッグ用変数に退避
    memcpy(log_data_buffer_for_debug, data_buffer, 48);
    cmndbg_hexdump(log_data_buffer_for_debug, 48, "upload_log_data_buffer");
#endif

    // TODO：テストサーバ向け対応（暗号化文字列に置き換え）
    // 本来であれば、data_bufferの中身を暗号化する
    memcpy(upload_log_post_data_buff, "log=", 4);
    memcpy(&(upload_log_post_data_buff[4]), str_test_update_log_post_data, 89);
    DBGLOG1("Upload_log_post_data = %s", upload_log_post_data_buff);

    // HTTPS接続
    mwifi_connect_https();

    if (drvwifi_result != MDLWIFI_ERR_NONE) {
        // HTTPS接続でエラーが発生したら、ここで抜ける
        return;
    }

    // Post通信リクエスト設定
    req.path = "/dapi/v1/device_log/add";
	req.path_len = 23;		// 文字数を設定すること
    req.body = upload_log_post_data_buff;
    req.body_len = 93;       // 文字数を設定すること

    s_context.req_kind = REQ_UPLOAD_OPE_LOG;

    // Post通信
    clr_flg(FLG_MDLWIFI, ~FLGPTN_DRVWIFI_HTTPS_POST_TASK_COMPLETE);
    drvwifi_https_post(drvwifi_callback, &req);
    er = twai_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_HTTPS_POST_TASK_COMPLETE, TWF_ANDW, &flgptn, TMO_FEVR);
    assert(er == E_OK);

#if 0
    if (drvwifi_result != MDLWIFI_ERR_NONE) {
        // Post通信エラーで中断
        return;
    }
#else
    // TODO : サーバからタスクが返送されないためデバッグ用にテストデータで上書きする。
    DBGLOG0("FOR DEBUG : error cancel, and replace receive_data");
    DBGLOG1("str = %s", str_test_ok);
    drvwifi_result = MDLWIFI_ERR_NONE;                          // エラーコードを削除
    strcpy(&s_context.receive_data, str_test_ok);
    //memcpy(&s_context.receive_data, str_test_ok, 42);
#endif

    // HTTPSクライアント切断
    clr_flg(FLG_MDLWIFI, ~FLGPTN_DRVWIFI_HTTPS_DISCONNECT_COMPLETE);
    drvwifi_https_disconnect(drvwifi_callback);
    er = twai_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_HTTPS_DISCONNECT_COMPLETE, TWF_ANDW, &flgptn, TMO_FEVR);
    assert(er == E_OK);

    // 電文解析
    if (mwifi_response_parser() != E_OK) {
        drvwifi_result = MDLWIFI_ERR_HTTP_POST; // 解析できない電文はPOSTのエラーとして処理
    }

    if (s_context.res_common.res_result == false) {
        DBGLOG0("Response result is false.");
        drvwifi_result = MDLWIFI_ERR_HTTP_POST; // サーバからfalseが返ってきたらPOSTのエラーとして処理
    }

}

/*
 * 空室カードダウンロード
 */
void mwifi_download_vacancy_card()
{
    DBGLOG0("mwifi_download_vacancy_card : start");

    ER er = E_OK;
    FLGPTN flgptn = 0;

    // 電文生成
    uint8_t data_buffer[32];                        // 暗号化の対象となるtaskの値
    drvrng_get_random(data_buffer, 14);
    memcpy(data_buffer[14], s_context.device_id.serialno, 16);
    memcpy(data_buffer[14+16], TASK_CODE_VACANCY_CARD, 2);

    cmndbg_hexdump(data_buffer, sizeof(data_buffer), "vacancy_databuffer");

    // TODO : テストサーバ向け対応(暗号化文字列に置き換え)
    // 本来であれば、data_bufferの中身を暗号化する
    memcpy(download_vacancy_post_data_buff, "task=", 5);
    memcpy(&(download_vacancy_post_data_buff[5]), str_test_download_vacancy_post_data, 65);
    DBGLOG1("Download_vacancy_post_data = %s", download_vacancy_post_data_buff);

    mwifi_connect_https();

    if (drvwifi_result != MDLWIFI_ERR_NONE) {
        // HTTPS接続でエラーが発生したら、ここで抜ける
        return;
    }

    // Post通信リクエスト設定
    req.path = "/dapi/v1/device_log/get";
	req.path_len = 23;		// 文字数を設定すること
    req.body = download_vacancy_post_data_buff;
    req.body_len = 65;       // 文字数を設定すること

    s_context.req_kind = REQ_DOWNLOAD_CARD;

    // Post通信
    clr_flg(FLG_MDLWIFI, ~FLGPTN_DRVWIFI_HTTPS_POST_TASK_COMPLETE);
    drvwifi_https_post(drvwifi_callback, &req);
    er = twai_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_HTTPS_POST_TASK_COMPLETE, TWF_ANDW, &flgptn, TMO_FEVR);
    assert(er == E_OK);

#if 0
    if (drvwifi_result != MDLWIFI_ERR_NONE) {
        // Post通信エラーで中断
        return;
    }
#else
    // TODO : サーバからタスクが返送されないためデバッグ用にテストデータで上書きする。
    DBGLOG0("FOR DEBUG : error cancel, and replace receive_data");
    DBGLOG1("str = %s", str_test_card);
    drvwifi_result = MDLWIFI_ERR_NONE;                          // エラーコードを削除
    strcpy(&s_context.receive_data, str_test_card);
    //memcpy(&s_context.receive_data, str_test_card, 221);
#endif

    // HTTPSクライアント切断
    clr_flg(FLG_MDLWIFI, ~FLGPTN_DRVWIFI_HTTPS_DISCONNECT_COMPLETE);
    drvwifi_https_disconnect(drvwifi_callback);
    er = twai_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_HTTPS_DISCONNECT_COMPLETE, TWF_ANDW, &flgptn, TMO_FEVR);
    assert(er == E_OK);

    // 電文解析
    if (mwifi_response_parser() != E_OK) {
        drvwifi_result = MDLWIFI_ERR_HTTP_POST; // 解析できない電文はPOSTのエラーとして処理
    }

    if (s_context.res_common.res_result == false) {
        DBGLOG0("Response result is false.");
        drvwifi_result = MDLWIFI_ERR_HTTP_POST; // サーバからfalseが返ってきたらPOSTのエラーとして処理
    }

}

/*
 * HTTPS接続
 */
void mwifi_connect_https()
{
    ER er = 0;
    FLGPTN flgptn = 0;

    // HTTPS接続
	hconf.hostname = hnm;
    hconf.hostname_len = 12;        // 文字数を設定すること
	hconf.exhdrname[0] = "\0";
	hconf.exhdrname_len = 0;		//文字数を設定すること
	hconf.exhdrvalue[0] = "\0";
	hconf.exhdrval_len = 0;		//文字数を設定すること
#ifdef HTTP_LOCALSERVER
    hconf.schema = DRVWIFI_SCHEMA_HTTP;
#else
//	hconf.schema = DRVWIFI_SCHEMA_HTTPS;	//0:http, 1:https
	hconf.schema = DRVWIFI_SCHEMA_HTTP;	//0:http, 1:https
    #endif
    clr_flg(FLG_MDLWIFI, ~FLGPTN_DRVWIFI_HTTPS_CONNECT_COMPLETE);
    drvwifi_https_connect(drvwifi_callback, &hconf);
    er = twai_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_HTTPS_CONNECT_COMPLETE, TWF_ANDW, &flgptn, TMO_FEVR);
    assert((er == E_OK) || (er == E_TMOUT));
}

/*
 * Wifiドライバコールバック
 */
void drvwifi_callback(int32_t evt, int32_t error, intptr_t opt)
{
    ER er = 0;
    drvwifi_result = MDLWIFI_ERR_NONE;

    DBGLOG3("drvwifi_callback(evt=%d,error=%d,opt=0x%08x)", evt, error, opt);

    switch(evt) {
    case DRVWIFI_EVT_INITIALIZE_COMPLETE:
        if (error != 0) {
            drvwifi_result = MDLWIFI_ERR_INITIALIZE;     // モジュール初期化エラー
        }
        er = set_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_INITIALIZE_COMPLETE);
        assert(er == E_OK);
        break;
    case DRVWIFI_EVT_AP_CONNECT_COMPLETE:
        if (error != 0) {
            drvwifi_result = MDLWIFI_ERR_AP_CONNECT;     // AP接続エラー
        }
        er = set_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_READY_COMPLETE);
        assert(er == E_OK);
        break;
    case DRVWIFI_EVT_AP_DISCONNECTED:
        if (error != 0) {
            drvwifi_result = MDLWIFI_ERR_AP_DISCONNECT;       // AP切断エラー
        } else {
            s_context.status = STATUS_DISCONNECTED;
        }
        er = set_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_AP_DISCONNECT_COMPLETE);
        assert(er == E_OK);
        break;
    case DRVWIFI_EVT_HTTP_CONNECT_COMPLETE:
        if (error != 0) {
            drvwifi_result = MDLWIFI_ERR_HTTP_CONNECT;
        }
        er = set_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_HTTPS_CONNECT_COMPLETE);
        assert(er == E_OK);
        break;
    case DRVWIFI_EVT_HTTP_DISCONNECT_COMPLETE:
        if (error != 0) {
            drvwifi_result = MDLWIFI_ERR_NONE;
        }
        er = set_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_HTTPS_DISCONNECT_COMPLETE);
        assert(er == E_OK);
        break;
    case DRVWIFI_EVT_POWER_OFF_COMPLETE:
        if (error != 0) {
            drvwifi_result = MDLWIFI_ERR_POWER_OFF;
        }
        er = set_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_POWEROFF_COMPLETE);
        assert(er == E_OK);
        break;
    case DRVWIFI_EVT_HTTP_POST_COMPLETE: {
        // 電文をコピー
        memcpy(&s_context.receive_data, opt, RECEIVE_POST_SIZE);

        // どのイベントからの返信かチェックする（TODO:分類する必要なし？）
        switch (s_context.req_kind) {
        case REQ_SERVERTASK:
        case REQ_DOWNLOAD_CARD:
            if (error != 0) {
                drvwifi_result = MDLWIFI_ERR_HTTP_POST;
            }
            er = set_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_HTTPS_POST_TASK_COMPLETE);
            assert(er == E_OK);
            break;
        case REQ_UPLOAD_ERR_LOG:
        case REQ_UPLOAD_OPE_LOG:
            if (error != 0) {
                drvwifi_result = MDLWIFI_ERR_HTTP_POST;
            }
            er = set_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_HTTPS_POST_TASK_COMPLETE);
            assert(er == E_OK);
            break;
        default:
            assert(false);
            break;
        }
        break;
    }
    default:
        assert(false);
        break;
    }

}

/*
 * レスポンスデータ解析処理
 */
bool_t mwifi_response_parser()
{
    int state = PARSE_START;
    char* p_st;
    char* p_en;
    char c;

    p_st = &s_context.receive_data;

    while ((state != PARSE_END) && (state != PARSE_ERROR)) {
        //DBGLOG1("PARSE : state = %d", state);
        switch (state) {
        case PARSE_START: {
            DBGLOG0("PARSE_START start.");
            p_st = strchr(p_st, '{');

            if (p_st == NULL) {
                DBGLOG0("[PARSER]PARSE_START : p_st = NULL -> PARSE_ERROR");
                state = PARSE_ERROR;
            } else {
                state = PARSE_DATA;
                p_st++;         // 先頭データへ移動
                //DBGLOG2("[PARSER]PARSE_START : p_st = 0x%08x, data_st = 0x%08x", p_st, &s_context.receive_data);
            }
            break;
        }
        case PARSE_DATA: {
            DBGLOG0("PARSE_DATA start.");
            p_en = strchr(p_st, ',');

            if (p_en == NULL) {
                DBGLOG0("[PARSER:]PARSE_DATA : p_en = NULL");
                p_en = strchr(p_st, '}');
                if (p_en == NULL) {
                    // エラー
                    DBGLOG0("[PARSER]PARSE_DATA(}) : p_en = NULL -> PARSE_ERROR");
                    state = PARSE_ERROR;
                } else {
                    // 最後のデータを取得
                    DBGLOG3("[PARSER]PARSE_DATA : p_st = 0x%08x, p_en - p_st = %d, %s", p_st, p_en - p_st, p_st);
                    mwifi_response_parser_data(p_st, p_en - p_st);
                    state = PARSE_END;
                }
            } else {
                // データ１個分の解析を実施
                DBGLOG3("[PARSER]PARSE_DATA : p_st = 0x%08x, p_en - p_st = %d, %s", p_st, p_en - p_st, p_st);
                mwifi_response_parser_data(p_st, p_en - p_st);

                // 次のデータの先頭へ
                p_st = p_en + 1;
            }
            break;
        }
        default:
            assert(false);
            state = PARSE_ERROR;
            break;
        }
    }
    DBGLOG0("mwifi_response_parser() is end.");

    if (state == PARSE_ERROR) {
        return -1;
    }

    return 0;
}

/*
 * レスポンスデータ解析処理（データ分類/保存）
 */
void mwifi_response_parser_data(char* pos, int length) {
    char key[50] = {0};
    char value[200] = {0};
    // キーとデータを取得
    for (int i = 0; i < length; i++) {
        if (*(pos+i) == ':') {
            // キーをコピー
            memcpy(key, pos, i);
            // valueをコピー
            memcpy(value, pos+i+1, length-i-1);

            //DBGLOG4("[PARSER] KEY = %s, VALUE = %s (pos=0x%08x, i=%d)", key, value, pos, i);
            break;
        }
    }

    // コンテキストにデータを保存
    if (strcmp(key, MDLWIFI_RES_DEFS[RES_KEY_RESULT].key_str) == 0) {
        if (strcmp(value, "true") == 0) {
            s_context.res_common.res_result = true;
        } else {
            s_context.res_common.res_result = false;
        }
        DBGLOG1("Key is RES_KEY_RESULT : %d", s_context.res_common.res_result);
    }
    else if (strcmp(key, MDLWIFI_RES_DEFS[RES_KEY_NOW_DATE].key_str) == 0) {
        DBGLOG1("RES_KEY_NOW_DATE data_length %d", MDLWIFI_RES_DEFS[RES_KEY_NOW_DATE].data_length);
        memcpy(s_context.res_common.now_date, &(value[1]), MDLWIFI_RES_DEFS[RES_KEY_NOW_DATE].data_length);
        cmndbg_hexdump(s_context.res_common.now_date, MDLWIFI_RES_DEFS[RES_KEY_NOW_DATE].data_length, "Now_Date");
        //DBGLOG1("Key is RES_KEY_NOW_DATE : %s", s_context.res_common.now_date);
    }
    else if (strcmp(key, MDLWIFI_RES_DEFS[RES_KEY_TASK_CODE1].key_str) == 0) {
        memcpy(s_context.res_servertask.task_code1, &(value[1]), MDLWIFI_RES_DEFS[RES_KEY_TASK_CODE1].data_length);
        cmndbg_hexdump(s_context.res_servertask.task_code1, MDLWIFI_RES_DEFS[RES_KEY_TASK_CODE1].data_length, "task_code1");
        //DBGLOG1("Key is RES_KEY_TASK_CODE1 : %s", s_context.res_servertask.task_code1);
    }
    else if (strcmp(key, MDLWIFI_RES_DEFS[RES_KEY_TASK_CODE2].key_str) == 0) {
        memcpy(s_context.res_servertask.task_code2, &(value[1]), MDLWIFI_RES_DEFS[RES_KEY_TASK_CODE2].data_length);
        //DBGLOG1("Key is RES_KEY_TASK_CODE2 : %s", s_context.res_servertask.task_code2);
    }
    else if (strcmp(key, MDLWIFI_RES_DEFS[RES_KEY_TASK_CODE3].key_str) == 0) {
        memcpy(s_context.res_servertask.task_code3, &(value[1]), MDLWIFI_RES_DEFS[RES_KEY_TASK_CODE3].data_length);
        //DBGLOG1("Key is RES_KEY_TASK_CODE3 : %s", s_context.res_servertask.task_code3);
    }
    else if (strcmp(key, MDLWIFI_RES_DEFS[RES_KEY_TASK_CODE4].key_str) == 0) {
        memcpy(s_context.res_servertask.task_code4, &(value[1]), MDLWIFI_RES_DEFS[RES_KEY_TASK_CODE4].data_length);
        //DBGLOG1("Key is RES_KEY_TASK_CODE4 : %s", s_context.res_servertask.task_code4);
    }
    else if (strcmp(key, MDLWIFI_RES_DEFS[RES_KEY_DOWNLOAD_DATA].key_str) == 0) {
        memcpy(&s_context.res_vacancy_card, &(value[1]), MDLWIFI_RES_DEFS[RES_KEY_DOWNLOAD_DATA].data_length);
        //DBGLOG1("Key is RES_KEY_DOWNLOAD_DATA : %s", s_context.res_vacancy_card);
    }
    else if (strcmp(key, MDLWIFI_RES_DEFS[RES_KEY_MESSAGE].key_str) == 0) {
        DBGLOG0("Key is RES_KEY_MESSAGE -> Invalid Data");          // メッセージは無視する
    }
    else {
        assert(false);
    }
}

void mwifi_test_response_parser()
{
    // 動作確認用
    memcpy(&s_context.receive_data, str_test_task, 116);
    //memcpy(&s_context.receive_data, str_test_err, 44);
    //memcpy(&s_context.receive_data, str_test_card, 222);

    mwifi_response_parser();
}
