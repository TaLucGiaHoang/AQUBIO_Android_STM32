/*
 * VA-X 認証ミドル
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/01/23 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "mdlauth.h"

#include <string.h>
#include <stdlib.h>

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "kernel_cfg.h"

#include "drvcam.h"
#include "drvirled.h"
#include "mdlstrg.h"
#include "mdlstrg_data.h"
#include "drvts.h"
#include "mdlauthlib.h"
#include "cmnbuf.h"

/*
 * マクロ定義
 */

// デバッグログ
#if 1
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[MDLAUTH]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[MDLAUTH]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[MDLAUTH]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[MDLAUTH]" msg, arg1, arg2, arg3)
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

// メモリプールのブロック
typedef struct {
    int msg;
    intptr_t opt1;
    intptr_t opt2;
} MDLAUTH_MPFBLK_T;

//
typedef struct {
    int score;
    int data_type;
    int index;
} AUTH_SMALL_RESULT_T;

// バッファ構成(認証用1)
typedef struct {
    MDLAUTHLIB_DATA_CAPTURED_T captured;
} BUF_AUTHDATA1_T;

// バッファ構成(認証用2)
typedef struct {
    MDLSTRG_DATA_AUTH_SMALL_T small;
    AUTH_SMALL_RESULT_T results_array[MDLSTRG_MAX_NDATA_AUTH];
    MDLAUTHLIB_DATA_REGISTERED_T registered;
    MDLAUTHLIB_DATA_REGISTERED_T tobe_registered;
    MDLAUTHLIB_DATA_REGISTERED_SMALL_T tobe_registered_small;
} BUF_AUTHDATA2_T;

// コンテキスト
typedef struct {
    int state;
    int mode;
    int reg_index;
    struct {
        int prepare;
    } drv_err;
    struct {
        intptr_t arg1;
        intptr_t arg2;
        intptr_t arg3;
        intptr_t retval;
    } worker_params;
    struct {
        uint8_t* camera;
        BUF_AUTHDATA1_T* auth1;
        BUF_AUTHDATA2_T* auth2;
    } buffers;
    MDLAUTH_CALLBACK_T callback;
    bool_t tsk_started;
    int retry_count;
    bool_t release_detected;
} MDLAUTH_CONTEXT_T;

// MDLBLE_MPFBLK_SIZE は sizeof(MDLAUTH_MPFBKL) に一致させること
static_assert(sizeof(MDLAUTH_MPFBLK_T) == MDLAUTH_MPFBLK_SIZE, "MPF size");

/*
 * 内部関数プロトタイプ
 */
//メッセージ送信
static void mpf_send(int msg, intptr_t opt1, intptr_t opt2);

// 初期化
static void initialize();

// 認証準備
static void prepare_auth();

// カメラドライバからのコールバック
static void drvcam_callback(int32_t type, int32_t error);

// ストレージミドルコールバック
static void mdlstrg_callback(int event, intptr_t opt1, intptr_t opt2);

// タッチセンサーコールバック
static void drvts_callback(int event);

// 認証処理実行
static int perform_auth();

// 登録処理実行
static int perform_registration(int index);

// 画像取得
static void capture_image(MDLAUTHLIB_IMAGE_T* image);

// 停止
static void stop();

// スコア比較関数(qsort用)
static int score_compar(const void* a_p1, const void* a_p2);

// リトライ
static void prepare_retry();

/*
 * 定数定義
 */

// 認証の試行回数(極小データのマッチ率の高い順に何件やるか)
static const int AUTH_ATTEMPT_LIMIT = 4;

// 認証リトライ回数
static const int AUTH_RETRY_LIMIT = 0;

// 状態
enum {
    STATE_NONE = 0,
    STATE_INITIALIZED,	// 初期化済み
    STATE_READY,		// 認証準備完了
};

// メッセージ番号
enum {
    MSG_NONE = 0,
    MSG_TRIGGER_START,
    MSG_INITIALIZE,
    MSG_PREPARE,
    MSG_CANCEL,
    MSG_TS_DETECTED,
};

// ワーカータスクの要求
enum {
    WORKER_NONE = 0,
    WORKER_PROCESS_IMAGE,
    WORKER_MATCH_SMALL,
    WORKER_MATCH,
};

static const FLGPTN FLGPTN_DRVCAM_INITIALIZE_COMPLETE =		(0x1 << 0);
static const FLGPTN FLGPTN_DRVCAM_POWERON_COMPLETE =		(0x1 << 1);
static const FLGPTN FLGPTN_DRVCAM_POWEROFF_COMPLETE =		(0x1 << 2);
static const FLGPTN FLGPTN_DRVCAM_PREPARE_COMPLETE =		(0x1 << 3);
static const FLGPTN FLGPTN_DRVCAM_CAPTURE_COMPLETE =		(0x1 << 4);
static const FLGPTN FLGPTN_MDLSTRG_REQUEST_COMPLETE =		(0x1 << 5);
static const FLGPTN FLGPTN_DRVTS_INITIALIZE_COMPLETE =		(0x1 << 6);
static const FLGPTN FLGPTN_DRVTS_DETECTED =					(0x1 << 7);
static const FLGPTN FLGPTN_CANCEL_REQUESTED =				(0x1 << 8);
static const FLGPTN FLGPTN_CANCEL_COMPLETE =				(0x1 << 9);

static const FLGPTN FLGPTN_WORKER_READY =		(0x1 << 0);
static const FLGPTN FLGPTN_WORKER_REQUEST =		(0x1 << 1);
static const FLGPTN FLGPTN_WORKER_COMPLETE =	(0x1 << 2);

/*
 * 内部変数
 */
static MDLAUTH_CONTEXT_T s_context = {0};

/*
 * 事前開始処理
 */
void mdlauth_trigger_start()
{
    assert(!s_context.callback);
    assert(s_context.state == 0);
    assert(!s_context.tsk_started);

    ER er = act_tsk(TSK_MDLAUTH);
    s_context.tsk_started = true;
    assert(er == E_OK);

    mpf_send(MSG_TRIGGER_START, 0, 0);
}

/*
 * 初期化
 */
void mdlauth_initialize(MDLAUTH_CALLBACK_T callback)
{
    ER er = 0;

    // タスク起動がまだなら起動する
    if (!s_context.tsk_started) {
        er = act_tsk(TSK_MDLAUTH);
        assert(er == E_OK);
        s_context.tsk_started = true;
        //SVC_PERROR(er);
    }

    mpf_send(MSG_INITIALIZE, (intptr_t)callback, 0);
}

/*
 * 認証準備
 */
int mdlauth_prepare(int mode, int index)
{
    mpf_send(MSG_PREPARE, (intptr_t)mode, (intptr_t)index);

    return 0;
}

/*
 * 認証・登録をキャンセルし、初期状態に戻す
 */
int mdlauth_cancel()
{
    mpf_send(MSG_CANCEL, 0, 0);

    return 0;
}

/*
 * タスク
 */
void mdlauth_task(intptr_t exinf)
{
    DBGLOG0("mdlauth_task() starts .");

    int callback_type = 0;
    intptr_t result = 0;

    while (true) {
        MDLAUTH_MPFBLK_T* blk = NULL;
        ER er = 0;
        er = trcv_dtq(DTQ_MDLAUTH, (intptr_t*)&blk, TMO_FEVR);
        assert(er == E_OK);
        switch (blk->msg) {
        case MSG_TRIGGER_START: {	// 先行開始処理
            assert(s_context.state == STATE_NONE);
            initialize();
            prepare_auth(MDLAUTH_MODE_AUTH);
            break;
        }
        case MSG_INITIALIZE: {	// 初期化
            assert((s_context.state == STATE_NONE) || (s_context.state == STATE_READY));
            assert(!s_context.callback);

            s_context.callback = (MDLAUTH_CALLBACK_T)blk->opt1;

            // mdlauth_trigger_start()が呼ばれていない場合のみ
            if (s_context.state == STATE_NONE) {
                initialize();
            }

            s_context.callback(MDLAUTH_EVT_INITIALIZE_COMPLETE, 0, 0);
            break;
        }
        case MSG_PREPARE: {	// 認証準備
            assert((s_context.state == STATE_INITIALIZED) || (s_context.state == STATE_READY));
            assert(s_context.callback);

            s_context.retry_count = 0;

            // mdlauth_trigger_start()が呼ばれていない場合のみ
            if (s_context.state == STATE_INITIALIZED) {
                prepare_auth((int)blk->opt1);
            }

            s_context.mode = (int)blk->opt1;
            s_context.reg_index = (int)blk->opt2;
            s_context.callback(MDLAUTH_EVT_PREPARE_COMPLETE, 0, 0);
            break;
        }
        case MSG_TS_DETECTED: {	// タッチセンサー検出
            assert(s_context.state == STATE_READY);
            assert(s_context.callback);

            s_context.callback(MDLAUTH_EVT_TS_DETECT, 0, 0);

            if (s_context.mode == MDLAUTH_MODE_REGISTRATION) {
                result = perform_registration(s_context.reg_index);
                callback_type = MDLAUTH_EVT_REGISTRATION_COMPLETE;
            } else {
                result = perform_auth();
                if (result == 0) {
                    callback_type = MDLAUTH_EVT_AUTH_COMPLETE;
                    stop();
                } else {
                    if (s_context.retry_count < AUTH_RETRY_LIMIT) {
                        s_context.retry_count++;
                        prepare_retry();
                        callback_type = MDLAUTH_EVT_AUTH_RETRYING;
                    } else {
                        s_context.retry_count = 0;
                        callback_type = MDLAUTH_EVT_AUTH_COMPLETE;
                        stop();
                    }
                }
            }

            s_context.callback(callback_type, result, 0);
            break;
        }
        case MSG_CANCEL: {
            stop();
            s_context.callback(MDLAUTH_EVT_CANCEL_COMPLETE, result, 0);
            break;
        }
        default:
            assert(false);
            break;
        }

        er = rel_mpf(MPF_MDLAUTH, blk);
        assert(er == E_OK);
    }
}

/*
 * 認証処理のワーカータスク(優先度低)
 */
void mdlauth_worker_task(intptr_t exinf)
{
    DBGLOG0("mdlauth_worker_task() starts .");

    while (true) {
        ER ercd = 0;
        FLGPTN flgptn = 0;

        ercd = set_flg(FLG_MDLAUTH_WORKER, FLGPTN_WORKER_READY);	// READYフラグをセット
        assert(ercd == E_OK);

        ercd = twai_flg(FLG_MDLAUTH_WORKER, FLGPTN_WORKER_REQUEST, TWF_ANDW, &flgptn, TMO_FEVR);	// REQUESTフラグを待つ
        assert(ercd == E_OK);

        ercd = clr_flg(FLG_MDLAUTH_WORKER, ~FLGPTN_WORKER_READY);	// READYフラグをクリア
        assert(ercd == E_OK);

        switch (exinf) {
        case WORKER_PROCESS_IMAGE:
            //mdlauthlib_process_image((MDLAUTHLIB_DATA_CAPTURED_T*)s_context.worker_params.arg1, (const MDLAUTHLIB_IMAGE_T*)s_context.worker_params.arg2);
            break;
        case WORKER_MATCH_SMALL:
            //mdlauthlib_match_small((const MDLAUTHLIB_DATA_CAPTURED_T*)s_context.worker_params.arg1, (const MDLAUTHLIB_DATA_REGISTERED_SMALL_T*)s_context.worker_params.arg2);
            break;
        case WORKER_MATCH:
            //mdlauthlib_match((const MDLAUTHLIB_DATA_CAPTURED_T*)s_context.worker_params.arg1, (const MDLAUTHLIB_DATA_REGISTERED_T*)s_context.worker_params.arg2);
            break;
        default:
            break;
        }

        ercd = set_flg(FLG_MDLAUTH_WORKER, FLGPTN_WORKER_COMPLETE);	// COMPLETEフラグをセット
        assert(ercd == E_OK);
    }
    return;
}

/*
 * 内部関数
 */

/*
 * メッセージ送信
 */
void mpf_send(int msg, intptr_t opt1, intptr_t opt2)
{
    MDLAUTH_MPFBLK_T* blk = NULL;
    ER er = 0;
    er = tget_mpf(MPF_MDLAUTH, (void**)&blk, TMO_FEVR);
    assert(er == E_OK);

    blk->msg = msg;
    blk->opt1 = opt1;
    blk->opt2 = opt2;
    er = tsnd_dtq(DTQ_MDLAUTH, (intptr_t)blk, TMO_FEVR);
    assert(er == E_OK);
}

void initialize()
{
    assert(s_context.state == STATE_NONE);

    ER ercd = 0;
    FLGPTN flgptn = 0;

    SYSTIM systim1 = 0;
    SYSTIM systim2 = 0;

    /* カメラドライバ初期化 */
    get_tim(&systim1);
    DBGLOG0("call drvcam_initialize()");
    drvcam_initialize(drvcam_callback);
    ercd = twai_flg(FLG_MDLAUTH, FLGPTN_DRVCAM_INITIALIZE_COMPLETE, TWF_ANDW, &flgptn, -1);
    assert(ercd == E_OK);
    get_tim(&systim2);
    DBGLOG1("complete drvcam_initialize (%dms)", systim2 - systim1);

    /* IR-LEDドライバ初期化 */
    DBGLOG0("call drvirled_initialize()");
    drvirled_initialize();
    DBGLOG0("complete drvirled_initialize()");

    /* タッチセンサードライバ初期化 */
    drvts_initialize();
//    ercd = twai_flg(FLG_MDLAUTH, FLGPTN_DRVTS_INITIALIZE_COMPLETE, TWF_ANDW, &flgptn, -1);
//    assert(ercd == E_OK);

    s_context.state = STATE_INITIALIZED;
    return;
}

/*
 * 認証準備
 */
void prepare_auth(int mode)
{
    assert(s_context.state == STATE_INITIALIZED);

    ER ercd = 0;
    FLGPTN flgptn = 0;

    SYSTIM systim1 = 0;
    SYSTIM systim2 = 0;

#if 0
    // データ読み出し要求
    static const MDLSTRG_REQUEST_T READ_REQ = {
        .data_type = MDLSTRG_DATA_TYPE_AUTH_TEMPLATE_SMALL,
        .request_type = MDLSTRG_REQ_TYPE_READ,
    };
    mdlstrg_request(NULL, mdlstrg_callback);
#endif

    // IRLED点灯
    drvirled_set_state(true);

    // カメラ電源ON
    get_tim(&systim1);
    DBGLOG0("call drvcam_power_on()");
    drvcam_power_on(drvcam_callback);
    ercd = twai_flg(FLG_MDLAUTH, FLGPTN_DRVCAM_POWERON_COMPLETE, TWF_ANDW, &flgptn, -1);
    assert(ercd == E_OK);
    get_tim(&systim2);
    DBGLOG1("complete drvcam_power_on() (%dms)", systim2 - systim1);

    // カメラ撮影準備
    get_tim(&systim1);
    DBGLOG0("call drvcam_prepare_capture()");
    DRVCAM_CAPTURE_PARAM_T capture_param = {
        .exposure = 0,						// 露出設定
        .regval = NULL,						// カメラレジスタ設定値(デバッグ用)
        .num_regval = 0,					// カメラレジスタ設定値の数
    };
    drvcam_prepare_capture(drvcam_callback, &capture_param);
    //s_context.buffers.camera = camera_buf;
    ercd = twai_flg(FLG_MDLAUTH, FLGPTN_DRVCAM_PREPARE_COMPLETE, TWF_ANDW, &flgptn, -1);
    assert(ercd == E_OK);
    get_tim(&systim2);
    DBGLOG1("complete drvcam_prepare_capture() (%dms)", systim2 - systim1);

    // タッチセンサー待機
    s_context.release_detected = false;
    drvts_start(drvts_callback);
    s_context.state = STATE_READY;

//end:
#if 0
    if (cancel){
        // 初期状態に戻す
        // todo:
    }
#endif

    return;
}

/*
 * 認証実行
 */
int perform_auth()
{
    int error = 0;
    FLGPTN flgptn = 0;
    ER ercd = E_OK;
    SYSTIM tim1 = 0, tim2 = 0;

	// バッファ取得
	void* buf = NULL;
    size_t buffer_size = 0;
	buffer_size = cmnbuf_aquire_buffer(&buf, CMNBUF_USE_AUTHDATA_1);	// 認証用1
    assert(buf);
    assert(buffer_size >= sizeof(BUF_AUTHDATA1_T));
	s_context.buffers.auth1 = buf;
	buf = NULL;

	buffer_size = cmnbuf_aquire_buffer(&buf, CMNBUF_USE_CAMERA_CAPTURE);	// キャプチャ用
    assert(buf);
    assert(buffer_size >= DRVCAM_CAPTURE_BUFFER_SIZE);
	s_context.buffers.camera = buf;
	buf = NULL;

    // 画像取得
    MDLAUTHLIB_IMAGE_T image = {0};
    get_tim(&tim1);
    capture_image(&image);
    get_tim(&tim2);
    DBGLOG1("capture_image: %ums", tim2-tim1);

    // 画像処理実行
    get_tim(&tim1);
    mdlauthlib_process_image(&(s_context.buffers.auth1->captured), &image);
    get_tim(&tim2);
    DBGLOG1("process_image: %ums", tim2-tim1);

    // 撮影バッファ開放
    cmnbuf_release_buffer(s_context.buffers.camera, CMNBUF_USE_CAMERA_CAPTURE);
    s_context.buffers.camera = NULL;

    // バッファ取得
	buf = NULL;
	buffer_size = cmnbuf_aquire_buffer(&buf, CMNBUF_USE_AUTHDATA_2);	// 認証用2
    assert(buf);
    assert(buffer_size >= sizeof(BUF_AUTHDATA2_T));
	s_context.buffers.auth2 = buf;
	buf = NULL;

    // 極小データ読み出し
    get_tim(&tim1);
    MDLSTRG_REQUEST_T request = {
        .data_type = MDLSTRG_DATA_TYPE_AUTH_SMALL,
        .request_type = MDLSTRG_REQ_TYPE_READ,
        .data = (intptr_t)&(s_context.buffers.auth2->small),
    };
    mdlstrg_request(&request, mdlstrg_callback);
    ercd = twai_flg(FLG_MDLAUTH, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &flgptn, 3000);
    assert(ercd == E_OK);
    get_tim(&tim2);
    DBGLOG1("flash_read(small): %ums", tim2-tim1);

    AUTH_SMALL_RESULT_T* results_array = s_context.buffers.auth2->results_array;
    // 無効値で初期化
    for (int i = 0; i < MDLSTRG_MAX_NDATA_AUTH; i++) {
        results_array[i].data_type = -1;
        results_array[i].index = -1;
    }

    get_tim(&tim1);
    MDLSTRG_DATA_AUTH_SMALL_T* small = &(s_context.buffers.auth2->small);

    DBGLOG1("small->count: %d", small->count);
    for (int i = 0; i < small->count; i++) {
        // 極小サイズデータでスコア算出
        int score = mdlauthlib_match_small(&(s_context.buffers.auth1->captured), &(small->array[i].data));
        assert(score >= 0);

        results_array[i].data_type = small->array[i].data_type;
        results_array[i].index = small->array[i].index;
        results_array[i].score = score;
    }
    get_tim(&tim2);
    DBGLOG2("match_small(%d): %ums", small->count, tim2-tim1);

    // スコアが高い順に処理を行う
    qsort(results_array, small->count, sizeof(AUTH_SMALL_RESULT_T), score_compar);	// スコア順にソート
    MDLAUTHLIB_MATCH_RESULT_T result = {0};
    int matched_index = -1;
    for (int i = 0; (i < AUTH_ATTEMPT_LIMIT) && (results_array[i].index != -1) ; i++) {	// 極小サイズデータでのスコアが高い順に試行
        // データを読み出し
        MDLSTRG_REQUEST_T request = {
            .data_type = results_array[i].data_type,
            .request_type = MDLSTRG_REQ_TYPE_READ,
            .data = (intptr_t)&(s_context.buffers.auth2->registered),
            .opt1 = results_array[i].index,
        };
        get_tim(&tim1);
        mdlstrg_request(&request, mdlstrg_callback);
        ercd = twai_flg(FLG_MDLAUTH, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &flgptn, 3000);
        assert(ercd == E_OK);
        get_tim(&tim2);
        DBGLOG1("flash_read: %ums", tim2-tim1);

        // マッチング試行
        get_tim(&tim1);
        mdlauthlib_match(&result, &(s_context.buffers.auth1->captured), &(s_context.buffers.auth2->registered));
        get_tim(&tim2);
        DBGLOG1("match: %ums", tim2-tim1);
        if (result.match) {
            matched_index = results_array[i].index;
            break;
        }
    }

#if 0	// TODO: 学習機能をOFFにする
    if (result.match) {
        // マッチングOKのデータから登録用データを生成
        get_tim(&tim1);
        mdlauthlib_create_data_registered(&(s_context.buffers.auth2->tobe_registered),
                                          &(s_context.buffers.auth2->tobe_registered_small),
                                          &(s_context.buffers.auth1->captured),
                                          result.offset);
        get_tim(&tim2);
        DBGLOG1("create_data: %ums", tim2-tim1);

        // 学習データとして保存
        MDLSTRG_REQUEST_T request = {
            .data_type = MDLSTRG_DATA_TYPE_AUTH_LEARNED,
            .request_type = MDLSTRG_REQ_TYPE_WRITE,
            .data = (intptr_t)&(void*[]){&(s_context.buffers.auth2->tobe_registered),
                                         &(s_context.buffers.auth2->tobe_registered_small)},
            .opt1 = matched_index,
        };
        get_tim(&tim1);
        mdlstrg_request(&request, mdlstrg_callback);
        ercd = twai_flg(FLG_MDLAUTH, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &flgptn, 3000);
        assert(ercd == E_OK);
        get_tim(&tim2);
        DBGLOG1("flash_write: %ums", tim2-tim1);
    }
#endif

    // 認証NGの場合はエラーを返す
    if (!result.match) {
        error = -1;
    }

    // 作業バッファ開放
    cmnbuf_release_buffer(s_context.buffers.auth1, CMNBUF_USE_AUTHDATA_1);
    s_context.buffers.auth1 = NULL;

    cmnbuf_release_buffer(s_context.buffers.auth2, CMNBUF_USE_AUTHDATA_2);
    s_context.buffers.auth2 = NULL;

    return error;
}

/*
 * 登録処理実行
 */
int perform_registration(int index)
{
    int error = 0;
    FLGPTN flgptn = 0;
    ER ercd = E_OK;

	// バッファ取得
	void* buf = NULL;
    size_t buffer_size = 0;
	buffer_size = cmnbuf_aquire_buffer(&buf, CMNBUF_USE_AUTHDATA_1);	// 認証用1
    assert(buf);
    assert(buffer_size >= sizeof(BUF_AUTHDATA1_T));
	s_context.buffers.auth1 = buf;
	buf = NULL;

	buffer_size = cmnbuf_aquire_buffer(&buf, CMNBUF_USE_CAMERA_CAPTURE);	// キャプチャ用
    assert(buf);
    assert(buffer_size >= DRVCAM_CAPTURE_BUFFER_SIZE);
	s_context.buffers.camera = buf;
	buf = NULL;

    // 画像取得
    MDLAUTHLIB_IMAGE_T image = {0};
    capture_image(&image);

    // 画像処理実行
    mdlauthlib_process_image(&(s_context.buffers.auth1->captured), &image);

    // 撮影バッファ開放
    cmnbuf_release_buffer(s_context.buffers.camera, CMNBUF_USE_CAMERA_CAPTURE);
    s_context.buffers.camera = NULL;

    // 作業バッファ取得
	buffer_size = cmnbuf_aquire_buffer(&buf, CMNBUF_USE_AUTHDATA_2);	// 認証用2
    assert(buf);
    assert(buffer_size >= sizeof(BUF_AUTHDATA2_T));
	s_context.buffers.auth2 = buf;
	buf = NULL;

    // 登録用データを生成
    mdlauthlib_create_data_registered(&(s_context.buffers.auth2->tobe_registered),
                                      &(s_context.buffers.auth2->tobe_registered_small),
                                      &(s_context.buffers.auth1->captured),
                                      0);

    SYSTIM systim1, systim2;
    get_tim(&systim1);
    // 認証データを書き込み
    MDLSTRG_REQUEST_T request = {
        .data_type = MDLSTRG_DATA_TYPE_AUTH,
        .request_type = MDLSTRG_REQ_TYPE_WRITE,
        .data = (intptr_t)&(void*[]){&(s_context.buffers.auth2->tobe_registered),
                                     &(s_context.buffers.auth2->tobe_registered_small)},
        .opt1 = index,
    };
    mdlstrg_request(&request, mdlstrg_callback);
    ercd = twai_flg(FLG_MDLAUTH, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &flgptn, 3000);
    assert(ercd == E_OK);
    get_tim(&systim2);
    DBGLOG1("write in %dms", systim2 - systim1);

    // 作業バッファ開放
    cmnbuf_release_buffer(s_context.buffers.auth1, CMNBUF_USE_AUTHDATA_1);
    s_context.buffers.auth1 = NULL;

    cmnbuf_release_buffer(s_context.buffers.auth2, CMNBUF_USE_AUTHDATA_2);
    s_context.buffers.auth2 = NULL;

    return error;
}

/*
 * 撮影
 */
void capture_image(MDLAUTHLIB_IMAGE_T* image)
{
    assert(image);

    // 撮影実行
    DBGLOG0("capture");
	SYSTIM systim1 = 0, systim2 = 0;
    get_tim(&systim1);
    drvcam_capture(drvcam_callback, s_context.buffers.camera, DRVCAM_CAPTURE_BUFFER_SIZE);
    ER ercd = twai_flg(FLG_MDLAUTH, FLGPTN_DRVCAM_CAPTURE_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 1000);
    assert(ercd == E_OK);
    get_tim(&systim2);
    DBGLOG1("capture ok (%dms)", systim2 - systim1);

    // 撮影画像から各種認証データ生成(画像処理)
	image->pixels = s_context.buffers.camera;
	image->length = 640*480;
	image->width = 640;
	image->height = 480;
}

/*
 * 停止
 */
void stop()
{
    if (s_context.state == STATE_READY) {
        // タッチセンサー停止
        drvts_stop();

        // カメラ停止
        DBGLOG0("call drvcam_power_on()");
        drvcam_power_off(drvcam_callback);
        ER ercd = twai_flg(FLG_MDLAUTH, FLGPTN_DRVCAM_POWEROFF_COMPLETE, TWF_ANDW, &(FLGPTN){0}, -1);
        assert(ercd == E_OK);
        DBGLOG0("complete drvcam_power_off()");

        // IR-LED消灯
        drvirled_set_state(false);

        // 各バッファの開放
        if (s_context.buffers.camera) {
            cmnbuf_release_buffer(s_context.buffers.camera, CMNBUF_USE_CAMERA_CAPTURE);
            s_context.buffers.camera = NULL;
        }

        if (s_context.buffers.auth1) {
            cmnbuf_release_buffer(s_context.buffers.auth1, CMNBUF_USE_AUTHDATA_1);
            s_context.buffers.auth1 = NULL;
        }

        if (s_context.buffers.auth2) {
            cmnbuf_release_buffer(s_context.buffers.auth2, CMNBUF_USE_AUTHDATA_2);
            s_context.buffers.auth2 = NULL;
        }

        s_context.state = STATE_INITIALIZED;
    }

    return;
}

/*
 * リトライ
 */
void prepare_retry()
{
    assert(s_context.state == STATE_READY);

    // タッチセンサー待機
    s_context.release_detected = false;
    drvts_start(drvts_callback);

    return;
}

/*
 * カメラドライバコールバック
 */
void drvcam_callback(int32_t type, int32_t error)
{
    switch (type) {
    case DRVCAM_CALLBACK_INITIALIZE:
        DBGLOG0("drvcam_callback: DRVCAM_CALLBACK_INITIALIZE");
        set_flg(FLG_MDLAUTH, FLGPTN_DRVCAM_INITIALIZE_COMPLETE);
        break;
    case DRVCAM_CALLBACK_POWERON:
        DBGLOG0("drvcam_callback: DRVCAM_CALLBACK_POWERON");
        set_flg(FLG_MDLAUTH, FLGPTN_DRVCAM_POWERON_COMPLETE);
        break;
    case DRVCAM_CALLBACK_POWEROFF:
        DBGLOG0("drvcam_callback: DRVCAM_CALLBACK_POWEROFF");
        set_flg(FLG_MDLAUTH, FLGPTN_DRVCAM_POWEROFF_COMPLETE);
        break;
    case DRVCAM_CALLBACK_PREPARE:
        DBGLOG0("drvcam_callback: DRVCAM_CALLBACK_PREPARE");
        set_flg(FLG_MDLAUTH, FLGPTN_DRVCAM_PREPARE_COMPLETE);
        break;
    case DRVCAM_CALLBACK_CAPTURE:
        DBGLOG0("drvcam_callback: DRVCAM_CALLBACK_CAPTURE");
        set_flg(FLG_MDLAUTH, FLGPTN_DRVCAM_CAPTURE_COMPLETE);
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
    DBGLOG1("mdlstrg_callback: e=%d", event);
    switch(event) {
    case MDLSTRG_EVT_REQUEST_COMPLETE:
        DBGLOG0("mdlstrg_callback: MDLSTRG_EVT_REQUEST_COMPLETE");
        set_flg(FLG_MDLAUTH, FLGPTN_MDLSTRG_REQUEST_COMPLETE);
        break;
    default:
        assert(false);
        break;
    }
}

/*
 * タッチセンサーコールバック
 */
void drvts_callback(int event)
{
    DBGLOG1("drvts_callback: e=%d", event);
    switch(event) {
    case DRVTS_EVT_INITE:
        DBGLOG0("DRVTS_EVT_RELEASE");	// 初期
        break;
    case DRVTS_EVT_TOUCH:
        if (s_context.release_detected) {
            DBGLOG0("DRVTS_EVT_TOUCH");
            drvts_stop();
            mpf_send(MSG_TS_DETECTED, 0, 0);
        }
        break;
    case DRVTS_EVT_RELEASE:
        DBGLOG0("DRVTS_EVT_RELEASE");	// 離れた
        s_context.release_detected = true;
        break;
    default:
        assert(false);
        break;
    }
}

// スコア比較関数(qsort用)
int score_compar(const void* a_p1, const void* a_p2)
{
    const AUTH_SMALL_RESULT_T* p1 = (const AUTH_SMALL_RESULT_T*)a_p1;
    const AUTH_SMALL_RESULT_T* p2 = (const AUTH_SMALL_RESULT_T*)a_p2;

    return p2->score - p1->score;
}


