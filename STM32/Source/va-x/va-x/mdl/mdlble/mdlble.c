/*
 * VA-X BLEミドル
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/01/05 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "mdlble.h"

#include <string.h>

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "kernel_cfg.h"

#include "drvble.h"

/*
 * マクロ定義
 */

#define MDLBLE_ALWAYS_FACTORY_RESET

// サービスコールのエラーのログ出力
static inline void svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}
#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))

#define BLE_HEADER_SIZE			4	/* サービス番号(1) + データ長(2) + チェックサム(1)*/
#define BLE_RESPONSE_DATA_SIZE	2	/* サービス番号(1) + 応答コード(1) */

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
} MDLBLE_MPFBLK_T;

// MDLBLE_MPFBLK_SIZE は sizeof(MDLBLE_MPFBKL) に一致させること
static_assert(sizeof(MDLBLE_MPFBLK_T) == MDLBLE_MPFBLK_SIZE, "MPF size");

// ヘッダ情報
typedef struct {
    uint8_t service;
    uint16_t length;
    uint8_t checksum;
} MDLBLE_HEADER_INFO_T;

// コンテキスト
typedef struct {
    MDLBLE_CALLBACK_T callback;
    BNVA_DEVICE_ID_T device_id;
    int status;
    struct {
        int get_ready;
        int send;
    } drv_err;
    struct {
        int status;
        bool_t busy;
        MDLBLE_HEADER_INFO_T header;
        MDLBLE_DATA_T* data;
    } receiving;
    struct {
        int status;
        bool_t busy;
        MDLBLE_HEADER_INFO_T header;
        uint8_t resp_code;
        int resp_error;
        //MDLBLE_DATA_T* data;
    } sending;
} MDLBLE_CONTEXT_T;

/*
 * 内部関数プロトタイプ
 */
//メッセージ送信
static void mpf_send(int msg, uint8_t service, intptr_t data, size_t length, intptr_t opt1);

// 初期化
static void mble_initialize(MDLBLE_CALLBACK_T callback, const BNVA_DEVICE_ID_T* device_id);

// 開始
static int mble_start();

// 停止
static int mble_stop();

// 送信
static void mble_send(uint8_t service, const uint8_t* data, size_t length);

// 受信バッファ返却
static void mble_return_buffer(const uint8_t* data);

// BLEドライバのコールバック
static void drvble_callback(int32_t evt, int32_t error, intptr_t opt);

// BLEドライバのコールバック(レスポンス送信完了)
static void drvble_callback_resp(int32_t evt, int32_t error, intptr_t opt);

// 受信時の処理
static void mble_on_receive(int32_t error);

// チェックサム計算
static uint8_t mble_calc_crc(const MDLBLE_DATA_T* data);

// レスポンス送信
static void mble_send_response(uint8_t service, uint8_t response);

// サービス0を処理
static void process_service_0(const uint8_t* data);

// CRC計算
static uint8_t crc8_itu(uint8_t crc, const void *buf, size_t size);

/*
 * 定数定義
 */

// メッセージ番号
enum {
    MSG_INITIALIZE = 1,
    MSG_START,
    MSG_STOP,
    MSG_RESTART,
	MSG_SEND,
	MSG_RETURN_BUFFER,
};

// BLEミドルの状態
enum {
    STATUS_INITIALIZED = 1,	///< 初期状態
    STATUS_STARTED,			///< 開始状態
    STATUS_CONNECTED,		///< BLE接続状態
    STATUS_VALIDATED,		///< 通信相手認証済み状態
    STATUS_DISCONNECTED,	///< 切断状態
};

// 応答種別
enum {
    RESPONSE_NONE = 0,			// 使用しない
    RESPONSE_ACK,				// 正常応答
    RESPONSE_CHECKSUM,			// チェックサムエラー
    RESPONSE_BUSY,				// ビジー
    RESPONSE_AVAILABLE,			// ビジー解除
    RESPONSE_SEQUENCE_BREAK,	// 通信シーケンス
};

// 受信状態
enum {
    RECEIVING_STATUS_IDLE = 0,		// アイドル
    RECEIVING_STATUS_HEADER,		// ヘッダ待機中
    RECEIVING_STATUS_DATA,			// データ待機中
    RECEIVING_STATUS_DISCARDING,	// 読み捨て中
};

// 送信状態
enum {
    SENDING_STATUS_IDLE = 0,	// アイドル
    SENDING_STATUS_DATA,		// データ送信中
    SENDING_STATUS_WAITING_FOR_RESPONSE,	// 応答待ち
};

// 認証種別

// イベントフラグ
static const FLGPTN FLGPTN_DRVBLE_INITIALIZE_COMPLETE =		(0x1 << 0);
static const FLGPTN FLGPTN_DRVBLE_FACTORY_RESET_COMPLETE =	(0x1 << 1);
static const FLGPTN FLGPTN_DRVBLE_GET_READY_COMPLETE =		(0x1 << 2);
static const FLGPTN FLGPTN_DRVBLE_SEND_COMPLETE =			(0x1 << 3);
static const FLGPTN FLGPTN_RESPONSE_RECEIVED = 				(0x1 << 4);
static const FLGPTN FLGPTN_DRVBLE_POWER_OFF_COMPLETE =		(0x1 << 5);

// データ受信タイムアウト
static const int32_t DTQTIMEOUT = 100;
static const int32_t DATA_TIMEOUT = -1;
static const int32_t BLE_SEND_RETRY_COUNT = 3;

/*
 * 内部変数
 */
static MDLBLE_CONTEXT_T s_context = {0};

static uint8_t s_header_buf[BLE_HEADER_SIZE];
static uint8_t s_response_buf[BLE_HEADER_SIZE + BLE_RESPONSE_DATA_SIZE];
//static uint8_t s_send_header_buf[BLE_HEADER_SIZE];
static uint8_t s_send_buf[BLE_HEADER_SIZE + MDLBLE_MAX_DATA_LEN];

/*
 * 初期化
 */
void mdlble_initialize(MDLBLE_CALLBACK_T callback, const BNVA_DEVICE_ID_T* device_id)
{
    // タスク起動
    ER er = act_tsk(TSK_MDLBLE);
    assert(er == E_OK);

    mpf_send(MSG_INITIALIZE, 0, (intptr_t)callback, 0, (intptr_t)device_id);
}

/*
 * 開始
 */
void mdlble_start()
{
    mpf_send(MSG_START, 0, 0, 0, 0);
}

/*
 * 停止
 */
void mdlble_stop()
{
    mpf_send(MSG_STOP, 0, 0, 0, 0);
}

/*
 * 再起動
 */
void mdlble_restart()
{
    mpf_send(MSG_RESTART, 0, 0, 0, 0);
}

/*
 * 送信
 */
void mdlble_send(uint8_t service, const uint8_t* data, size_t length)
{
    mpf_send(MSG_SEND, service, (intptr_t)data, length, 0);
}

/*
 * 受信バッファ返却
 */
void mdlble_return_buffer(const uint8_t* data)
{
    mpf_send(MSG_RETURN_BUFFER, 0, (intptr_t)data, 0, 0);
}

/*
 * タスク
 */
void mdlble_task(intptr_t exinf)
{
    syslog(LOG_NOTICE, "mdlble_task() starts .");

    while (true) {
        MDLBLE_MPFBLK_T* blk = NULL;
        ER er = 0;
        er = trcv_dtq(DTQ_MDLBLE, (intptr_t*)&blk, TMO_FEVR);
        assert(er == E_OK);
        int error = 0;
        switch (blk->msg) {
        case MSG_INITIALIZE:
            mble_initialize((MDLBLE_CALLBACK_T)(blk->data), (const BNVA_DEVICE_ID_T*)(blk->opt1));
            break;
        case MSG_START:
            error = mble_start();

            // コールバック
            s_context.callback(MDLBLE_EVT_START_COMPLETE, error, 0);
            break;
        case MSG_STOP:
            error = mble_stop();

            // コールバック
            s_context.callback(MDLBLE_EVT_STOP_COMPLETE, error, 0);
            break;
        case MSG_RESTART:
            mble_stop();
            mble_start();

            // コールバック
            s_context.callback(MDLBLE_EVT_RESTART_COMPLETE, 0, 0);
            break;
        case MSG_RETURN_BUFFER:
            mble_return_buffer((const uint8_t*)(blk->data));
            break;
        case MSG_SEND:
            mble_send(blk->service, (const uint8_t*)(blk->data), blk->length);
            break;
//        case MSG_POWER_OFF:
//            ble_power_off(blk->callback);
//            break;
        default:
            assert(false);
            break;
        }

        er = rel_mpf(MPF_MDLBLE, blk);
        assert(er == E_OK);
    }

}

/*
 * 内部関数
 */

/*
 * メッセージ送信
 */
void mpf_send(int msg, uint8_t service, intptr_t data, size_t length, intptr_t opt1)
{
    MDLBLE_MPFBLK_T* blk = NULL;
    ER er = 0;
    er = tget_mpf(MPF_MDLBLE, (void**)&blk, DTQTIMEOUT);
    assert(er == E_OK);

    blk->msg = msg;
    blk->service = service;
    blk->data = data;
    blk->length = length;
    blk->opt1 = opt1;
    er = tsnd_dtq(DTQ_MDLBLE, (intptr_t)blk, DTQTIMEOUT);
    assert(er == E_OK);
}

/*
 * 初期化
 */
void mble_initialize(MDLBLE_CALLBACK_T callback, const BNVA_DEVICE_ID_T* opt1)
{
    assert(!s_context.callback);

    // コールバック関数を保持
    s_context.callback = callback;

    // 機器識別情報を保持
    memcpy(&(s_context.device_id), opt1, sizeof(BNVA_DEVICE_ID_T));

    // ドライバ初期化
    clr_flg(FLG_MDLBLE, ~FLGPTN_DRVBLE_INITIALIZE_COMPLETE);
    drvble_initialize(drvble_callback);
    FLGPTN flgptn = 0;
    ER er = twai_flg(FLG_MDLBLE, FLGPTN_DRVBLE_INITIALIZE_COMPLETE, TWF_ANDW, &flgptn, 2000);
    assert(er == E_OK);

#ifdef MDLBLE_ALWAYS_FACTORY_RESET
    // 工場初期化
    {
        clr_flg(FLG_MDLBLE, ~FLGPTN_DRVBLE_FACTORY_RESET_COMPLETE);
        drvble_factory_reset(drvble_callback, &(s_context.device_id));
        FLGPTN flgptn = 0;
        ER er = twai_flg(FLG_MDLBLE, FLGPTN_DRVBLE_FACTORY_RESET_COMPLETE, TWF_ANDW, &flgptn, 2000);
        assert(er == E_OK);
    }
#endif

    // 状態変更
    s_context.status = STATUS_INITIALIZED;

    // 完了コールバック
    s_context.callback(MDLBLE_EVT_INITIALIZE_COMPLETE, 0, 0);
}

/*
 * 開始
 */
int mble_start()
{
    assert(s_context.status == STATUS_INITIALIZED);
    assert(s_context.callback);

    // BLEモジュールを動作状態へ
    clr_flg(FLG_MDLBLE, ~FLGPTN_DRVBLE_GET_READY_COMPLETE);
    drvble_get_ready(drvble_callback);
    FLGPTN flgptn = 0;
    ER er = twai_flg(FLG_MDLBLE, FLGPTN_DRVBLE_GET_READY_COMPLETE, TWF_ANDW, &flgptn, 2000);
    assert(er == E_OK);

    int error = MDLBLE_ERROR_NONE;
    if (s_context.drv_err.get_ready != DRVBLE_ERROR_NONE) {
        error = MDLBLE_ERROR_OTHER;
        goto end;
    }

    // ヘッダ受信待機
    s_context.receiving.status = RECEIVING_STATUS_HEADER;
    drvble_receive(s_header_buf, BLE_HEADER_SIZE, -1, drvble_callback);

    // ステータス更新
    s_context.status = STATUS_STARTED;

end:
    return error;
}

/*
 * 停止
 */
int mble_stop()
{
    // 受信停止
    drvble_cancel_receive();

    // BLE 電源OFF
    drvble_power_off(drvble_callback);
    ER er = twai_flg(FLG_MDLBLE, FLGPTN_DRVBLE_POWER_OFF_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 1000);
    assert(er == E_OK);

    s_context.status = STATUS_INITIALIZED;
    s_context.receiving.status = RECEIVING_STATUS_IDLE;

    return 0;
}

/*
 * 送信
 */
void mble_send(uint8_t service, const uint8_t* data, size_t length)
{
    assert(service > 0);
    assert(data);
    assert(length > 0);

    syslog(LOG_NOTICE, "mble_send(s=%d, d=0x%08x, l=%d)", service, data, length);

    s_send_buf[0] = service;	// [0]service number
    s_send_buf[1] = ((uint8_t*)&(length))[0];		// [1]data length
    s_send_buf[2] = ((uint8_t*)&(length))[1];		// [2]data length
    s_send_buf[3] = 0xff;		// [3]checksum: 0xff を入れた状態でCRCを算出する
    memcpy(&(s_send_buf[4]), data, length);	// [4-]data

    // CRC計算
    s_send_buf[3] = crc8_itu(0, s_send_buf, BLE_HEADER_SIZE + length);

    bool_t sent = false;
    int send_error = 0;
    for (int retry = 0; retry < BLE_SEND_RETRY_COUNT; retry++) {
        FLGPTN flgptn = 0;
        ER er = E_OK;

        // フラグクリア
        er = clr_flg(FLG_MDLBLE, ~(FLGPTN_DRVBLE_GET_READY_COMPLETE | FLGPTN_RESPONSE_RECEIVED));
        assert(er == E_OK);

        // 送信
        drvble_send(s_send_buf, BLE_HEADER_SIZE + length, DATA_TIMEOUT, drvble_callback);

        // 送信完了待ち
        er = twai_flg(FLG_MDLBLE, FLGPTN_DRVBLE_SEND_COMPLETE, TWF_ANDW, &flgptn, 1000);
        assert((er == E_OK) || (er == E_TMOUT));

        // レスポンス受信待ち
        s_context.sending.status = SENDING_STATUS_WAITING_FOR_RESPONSE;
        er = twai_flg(FLG_MDLBLE, FLGPTN_RESPONSE_RECEIVED, TWF_ANDW, &flgptn, 1000);
        assert((er == E_OK) || (er == E_TMOUT));

        if (s_context.sending.resp_code != RESPONSE_ACK) {
            send_error = s_context.sending.resp_code;
        }

        // チェックサムエラー以外はリトライしない
        if (s_context.sending.resp_code != RESPONSE_CHECKSUM) {
            break;
        }
    }

    // コールバック
    s_context.callback(MDLBLE_EVT_SEND_COMPLETE, send_error, 0);
}

// 受信バッファ返却
void mble_return_buffer(const uint8_t* data)
{
    assert(data);

    ER er = rel_mpf(MPF_MDLBLE_DATA, (void**)data);
    assert(er == E_OK);

    // AVEILABLEメッセージを投げる
    if (s_context.receiving.busy) {
        mble_send_response(0, RESPONSE_AVAILABLE);
        // ビジーフラグはここでは落とさない
    }
}

/*
 * BLEドライバのコールバック
 */
void drvble_callback(int32_t evt, int32_t error, intptr_t opt)
{
    syslog(LOG_NOTICE, "drvble_callback(evt=%d, err=%d)", evt, error);

    switch (evt) {
    case DRVBLE_EVT_INITIALIZE_COMPLETE:
        set_flg(FLG_MDLBLE, FLGPTN_DRVBLE_INITIALIZE_COMPLETE);
        break;
    case DRVBLE_EVT_FACTORY_RESET_COMPLETE:
        set_flg(FLG_MDLBLE, FLGPTN_DRVBLE_FACTORY_RESET_COMPLETE);
        break;
    case DRVBLE_EVT_GET_READY_COMPLETE:
        s_context.drv_err.get_ready = error;
        set_flg(FLG_MDLBLE, FLGPTN_DRVBLE_GET_READY_COMPLETE);
        break;
    case DRVBLE_EVT_POWER_OFF_COMPLETE:
        set_flg(FLG_MDLBLE, FLGPTN_DRVBLE_POWER_OFF_COMPLETE);
        break;
    case DRVBLE_EVT_SEND_COMPLETE:
        s_context.drv_err.send = error;
        set_flg(FLG_MDLBLE, FLGPTN_DRVBLE_SEND_COMPLETE);
        break;
    case DRVBLE_EVT_RECEIVE_COMPLETE:
        if (error != DRVBLE_ERROR_CANCELLED) {
            mble_on_receive(error);
        }
        break;
    default:
        assert(false);
        break;
    }
}

/*
 * BLEドライバのコールバック(レスポンス用)
 */
void drvble_callback_resp(int32_t evt, int32_t error, intptr_t opt)
{
    syslog(LOG_NOTICE, "drvble_callback_resp(evt=%d, err=%d)", evt, error);

    switch (evt) {
    case DRVBLE_EVT_SEND_COMPLETE:
        break;
    default:
        assert(false);
        break;
    }
}

/*
 * 受信時の処理
 */
void mble_on_receive(int32_t error)
{
    switch (s_context.receiving.status) {
    case RECEIVING_STATUS_HEADER:	// ヘッダ受信中
    {
        // ヘッダ情報を保持
        s_context.receiving.header.service = s_header_buf[0];
        ((uint8_t*)&(s_context.receiving.header.length))[0] = s_header_buf[1];
        ((uint8_t*)&(s_context.receiving.header.length))[1] = s_header_buf[2];
        s_context.receiving.header.checksum = s_header_buf[3];
        syslog(LOG_NOTICE, "drvble_callback: header{0x%02x, 0x%02x, 0x%02x}", s_context.receiving.header.service, s_context.receiving.header.length, s_context.receiving.header.checksum);

        // メモリプールから受信バッファを取得する
        MDLBLE_DATA_T* data = NULL;
        ER er = 0;
        er = tget_mpf(MPF_MDLBLE_DATA, (void**)&data, 0);
        assert(er == E_OK || er == E_TMOUT);
        if (er == E_OK) {
            s_context.receiving.busy = false;
            s_context.receiving.data = data;
        } else if (er == E_TMOUT) {
            // 使用できるバッファが無い場合はビジー状態にする
            s_context.receiving.busy = true;
            s_context.receiving.data = NULL;
        }

        // データ受信開始
        if (!s_context.receiving.busy) {
            s_context.receiving.status = RECEIVING_STATUS_DATA;
            drvble_receive(s_context.receiving.data->body, s_context.receiving.header.length, DATA_TIMEOUT, drvble_callback);
        } else {
            s_context.receiving.status = RECEIVING_STATUS_DISCARDING;
            drvble_receive(NULL, s_context.receiving.header.length, DATA_TIMEOUT, drvble_callback);
        }
    }
    break;
    case RECEIVING_STATUS_DATA:	// データ受信中
    {
        assert(!s_context.receiving.busy);	// ビジーの場合はここに到達してはいけない

        uint8_t response = 0;
        // チェックサムを確認
        uint8_t header_buf[BLE_HEADER_SIZE];
        header_buf[0] = s_context.receiving.header.service;
        header_buf[1] = ((uint8_t*)&(s_context.receiving.header.length))[0];
        header_buf[2] = ((uint8_t*)&(s_context.receiving.header.length))[1];
        header_buf[3] = 0xff;
        uint8_t crc = 0;
        crc = crc8_itu(crc, header_buf, BLE_HEADER_SIZE);
        crc = crc8_itu(crc, s_context.receiving.data->body, s_context.receiving.header.length);
        //syslog(LOG_NOTICE, "checksum: 0x%02x, 0x%02x", s_context.receiving.header.checksum, crc);
        if (crc == s_context.receiving.header.checksum) {
            response = RESPONSE_ACK;
        } else {
            response = RESPONSE_CHECKSUM;
        }

        if (s_context.receiving.header.service != 0) {	// サービス0(受信応答)以外の場合
            // 応答を返す
            mble_send_response(s_context.receiving.header.service, response);
        }

        // 受信データチェック
        if (response == RESPONSE_ACK) {
            s_context.receiving.data->service = s_context.receiving.header.service;
            s_context.receiving.data->length = s_context.receiving.header.length;

            if (s_context.receiving.header.service == 0) {
                // サービス0(受信応答)はBLEミドル内で処理する
                process_service_0(s_context.receiving.data->body);
            } else {
                // 呼出側へコールバック
                s_context.callback(MDLBLE_EVT_DATA_RECEIVED, (intptr_t)s_context.receiving.data, 0);
                s_context.receiving.data = NULL;	// バッファはコールバック先で管理する
            }
        }

        // バッファ返却
        if (s_context.receiving.data) {
            ER er = rel_mpf(MPF_MDLBLE_DATA, (void**)s_context.receiving.data);
            s_context.receiving.data = NULL;
            assert(er == E_OK);
        }

        // 次のヘッダ受信
        s_context.receiving.status = RECEIVING_STATUS_HEADER;
        drvble_receive(s_header_buf, BLE_HEADER_SIZE, -1, drvble_callback);
    }
    break;
    case RECEIVING_STATUS_DISCARDING:	// データ読み捨て中
    {
        assert(s_context.receiving.busy);	// ビジーの場合しかここに到達しない

        uint8_t response = 0;
        response = RESPONSE_BUSY;

        if (s_context.receiving.header.service != 0) {	// サービス0(受信応答)以外の場合
            // 応答を返す
            mble_send_response(s_context.receiving.header.service, response);
        }

        // 次のヘッダ受信
        s_context.receiving.status = RECEIVING_STATUS_HEADER;
        drvble_receive(s_header_buf, BLE_HEADER_SIZE, -1, drvble_callback);
    }
    break;
    case RECEIVING_STATUS_IDLE:	// 停止中
        // データを無視する
        break;
    default:
        assert(false);
        break;
    }
}

// チェックサム計算
uint8_t mble_calc_crc(const MDLBLE_DATA_T* data)
{

    return 0;
}

// レスポンス送信
void mble_send_response(uint8_t service, uint8_t response)
{
    uint16_t length16 = BLE_RESPONSE_DATA_SIZE;

    static_assert(sizeof(s_response_buf) >= (BLE_HEADER_SIZE + BLE_RESPONSE_DATA_SIZE), "sizeof(s_response_buf)");
    s_response_buf[0] = 0;						// service: 0
    s_response_buf[1] = ((uint8_t*)&(length16))[0];	// length: 2
    s_response_buf[2] = ((uint8_t*)&(length16))[1];	// length: 2
    s_response_buf[3] = 0xff;					// crc: 0xff を入れた状態で計算する
    s_response_buf[4] = service;				// service: 
    s_response_buf[5] = response;				// response:

    // CRC計算
    s_response_buf[3] = crc8_itu(0, s_response_buf, BLE_HEADER_SIZE + BLE_RESPONSE_DATA_SIZE);

    // 送信
    drvble_send(s_response_buf, sizeof(s_response_buf), DATA_TIMEOUT, drvble_callback_resp);
}

// サービス0を処理
void process_service_0(const uint8_t* data)
{
    assert(data);

    uint8_t service = data[0];
    uint8_t resp_code = data[1];

    s_context.sending.resp_code = resp_code;
    ER er = set_flg(FLG_MDLBLE, FLGPTN_RESPONSE_RECEIVED);
    assert(er == E_OK);

#if 0
    if ((s_context.sending.status == SENDING_STATUS_WAITING_FOR_RESPONSE) &&
        (s_context.sending.header.service == service)) {
        s_context.sending.resp_code = resp_code;
        ER er = set_flg(FLG_MDLBLE, FLGPTN_RESPONSE_RECEIVED);
        assert(er == E_OK);
    }
#endif
}

/* crc計算 */
uint8_t crc8_itu(uint8_t crc, const void *buf, size_t size)
{
    static const uint8_t TABLE[] = {
        0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
        0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
        0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
        0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
        0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
        0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
        0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
        0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
        0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
        0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
        0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
        0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
        0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
        0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
        0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
        0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3,
    };

	const uint8_t *p;

	p = buf;
	crc = crc ^ ~0U;

	while (size--)
		crc = TABLE[(crc ^ *p++)];

	return crc ^ ~0U;
}

