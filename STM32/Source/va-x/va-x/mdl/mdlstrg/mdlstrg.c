/*
 * VA-X 保存データ管理ミドル
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/01/25 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "mdlstrg.h"

#include <string.h>

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "kernel_cfg.h"

#include "va-x_def.h"
#include "mdlstrg_data.h"
#include "drvflx.h"

/*
 * マクロ定義
 */

// デバッグログ
#if 1
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[MDLSTRG]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[MDLSTRG]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[MDLSTRG]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[MDLSTRG]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

#define MEMBER_SIZEOF(type, member)	sizeof(((type*)0)->member)

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
    MDLSTRG_REQUEST_T request;
    MDLSTRG_CALLBACK_T callback;
} MDLSTRG_MPFBLK_T;
// MDLBLE_MPFBLK_SIZE は sizeof(MDLSTRG_MPFBKL) に一致させること
static_assert(sizeof(MDLSTRG_MPFBLK_T) == MDLSTRG_MPFBLK_SIZE, "MPF size");

// コンテキスト
typedef struct {
    int state;
} MDLSTRG_CONTEXT_T;

// データブロックヘッダ
typedef struct {
    uint8_t state;		// データ状態
    uint8_t id;			// データID
} FLASH_DATA_HEADER_T;

// フラッシュ領域定義
typedef struct {
    int			area;			// データ種別
    intptr_t	addr;			// 先頭アドレス
    size_t		leb_size;		// LEBサイズ(ヘッダ込み, 物理イレースブロックサイズ*n)
    size_t		num_lebs;		// LEB数
    int			strategy;		// 管理方式
    size_t		data_size;		// データサイズ(ヘッダ除く)
    int			num_dbs;		// データブロック個数
} FLASH_AREA_DEF_T;

// 論理イレースブロックヘッダ
typedef struct {
    uint32_t	leb_state;	// 論理イレースブロック状態
} LEB_HEADER_T;

// データブロックヘッダ
typedef struct {
    uint32_t	data_id;	// データID
    size_t		data_len;	// データ長
    uint32_t	db_state;	// データブロック状態
} DB_HEADER_T;

// DBカーソル
typedef struct {
    const FLASH_AREA_DEF_T*	area_def;
    int						leb_index;
    intptr_t				db_addr;
    uint32_t				db_state;
} DB_CURSOR_T;

// 認証データ(フラッシュ保存形式)
typedef struct {
    MDLAUTHLIB_DATA_REGISTERED_T auth;
    MDLAUTHLIB_DATA_REGISTERED_SMALL_T auth_small;
} FLASH_DATA_AUTH_T;


/*====================================================================
 * 内部関数プロトタイプ
 *==================================================================*/
// メッセージ送信
static void mpf_send(int msg, const MDLSTRG_REQUEST_T* request, MDLSTRG_CALLBACK_T callback);

// フラッシュドライバコールバック
static void drvflx_callback(int evt, int error, intptr_t opt);

// -------------------------------------------------------------------
// フラッシュ read/write
// -------------------------------------------------------------------
// 読込み
static void flash_read(void* dest, intptr_t src, size_t length);

// 書込み
static void flash_write(intptr_t dest, const void* src, size_t length);

// 消去
static void flash_erase(intptr_t dest, size_t length);

// コピー
static void flash_copy(intptr_t dest, intptr_t src, size_t length);

// -------------------------------------------------------------------
// データブロック(DB)操作
// -------------------------------------------------------------------
// データブロックの先頭アドレス取得
static bool_t get_db_addr(intptr_t* a_addr, size_t* a_size, int area, uint32_t data_id, int search_for);

// データのアドレス取得
static bool_t db_get_data_addr(intptr_t* a_addr, size_t* a_size, DB_CURSOR_T* cursor, int area, uint32_t data_id);

// STRATEGY_1DATA_1LEB_INDEXED 用処理
static bool_t db_get_data_addr_1d1li(intptr_t* a_addr, size_t* a_size, DB_CURSOR_T* cursor, const FLASH_AREA_DEF_T* area_def, uint32_t data_id, int search_for);

// STRATEGY_INCREMENTAL_FIXED 用の処理
static bool_t db_get_data_addr_incf(intptr_t* a_addr, size_t* a_size, DB_CURSOR_T* cursor, const FLASH_AREA_DEF_T* area_def, uint32_t data_id, int search_for);

// 新しいDBを割り当ててアドレス取得
static bool_t db_new_data(intptr_t* a_addr, size_t* a_size, DB_CURSOR_T* cursor, int area, uint32_t data_id);

// ブロック状態フラグ更新
static void db_update_block_state(const DB_CURSOR_T* cursor, uint32_t state);

// ブロック無効化
static void db_set_invalid(const DB_CURSOR_T* cursor);

// LEB消去
static void leb_erase(const FLASH_AREA_DEF_T* area_def, int leb_index);

// -------------------------------------------------------------------
// 各種データ操作
// -------------------------------------------------------------------
// 認証データ読み出し
static bool_t read_auth(MDLAUTHLIB_DATA_REGISTERED_T* dest, int index);

// 認証データ書込み
static void write_auth(const MDLAUTHLIB_DATA_REGISTERED_T* data, const MDLAUTHLIB_DATA_REGISTERED_SMALL_T* data_small, int index);

// 認証データ削除
static void delete_auth(int index);

// 認証データ存在確認
static bool_t exists_auth(int index);

// 認証メタデータ読み出し
static bool_t read_auth_meta(MDLSTRG_DATA_AUTH_META_T* dest, int index);

// 認証メタデータ書込み
static void write_auth_meta(const MDLSTRG_DATA_AUTH_META_T* data, int index);

// 認証メタデータ削除
static void delete_auth_meta(int index);

// 認証学習データ読み出し
static bool_t read_auth_learned(MDLAUTHLIB_DATA_REGISTERED_T* dest, int index);

// 認証学習データ書込み
static void write_auth_learned(const MDLAUTHLIB_DATA_REGISTERED_T* data, const MDLAUTHLIB_DATA_REGISTERED_SMALL_T* data_small, int index);

// 認証学習データ削除
static void delete_auth_learned(int index);

// 認証学習データ存在確認
static bool_t exists_auth_learned(int index);

// 極小データ読み出し
static void read_auth_small(MDLSTRG_DATA_AUTH_SMALL_T* dest);

// アプリ登録情報読み出し
static bool_t read_appkey(MDLSTRG_DATA_APPKEY_T* dest, int index);

// 認証データ書込み
static void write_appkey(const MDLSTRG_DATA_APPKEY_T* data, int index);

// 機器識別情報読み出し
static bool_t read_device_id(BNVA_DEVICE_ID_T* dest);

// 機器識別情報書込み
static void write_device_id(const BNVA_DEVICE_ID_T* device_id);

//////////////////////
// Store Program memory
static bool_t read_store_program(void* data, intptr_t address, size_t size, int index);
static void write_store_program(void* data, intptr_t address, size_t size, int index);
static void delete_store_program(size_t size, int index);
static bool_t exists_store_program(size_t* size, int index);
//////////////////////

/*
 * 定数定義
 */

// 状態
enum {
    STATE_NONE = 0,
    STATE_READY,		// 準備完了
};

// メッセージ番号
enum {
    MSG_NONE = 0,
    MSG_INITIALIZE,
    MSG_REQUEST,
};

// 検索方式 (get_data_block)
enum {
    SEARCH_FOR_VALID_DATA = 0,
    SEARCH_FOR_WRITE_DEST,
};

// イベントフラグ
static const FLGPTN FLGPTN_DRVFLX_INITIALIZE_COMPLETE =	(0x1 << 0);
static const FLGPTN FLGPTN_DRVFLX_READ_COMPLETE =		(0x1 << 1);
static const FLGPTN FLGPTN_DRVFLX_WRITE_COMPLETE =		(0x1 << 2);
static const FLGPTN FLGPTN_DRVFLX_ERASE_COMPLETE =		(0x1 << 3);

// フラッシュの領域種別
enum {
    FLASH_AREA_NONE = 0,			// なし
    FLASH_AREA_DEVICE_ID,			// デバイス識別情報
    FLASH_AREA_APPKEY,				// 鍵など
    FLASH_AREA_AUTH_META,			// 登録メタ情報(名前など)
    FLASH_AREA_AUTH,				// 登録情報(テンプレート)
    FLASH_AREA_AUTH_LEARNED,		// 学習情報(テンプレート)
    FLASH_AREA_AUTH_LEARNED_INC,	// 学習情報(追記領域)
    // D-roomカード
    // ログ何件か
    // Wifiの接続情報
    // カウンター
    //   認証OK/NGなど
    //   電気錠の開閉回数
    // 
    ///////////////////////////
    FLASH_AREA_STORE_PROGRAM,		// Store Program
    ///////////////////////////
};

// フラッシュの保存方法
enum {
    STRATEGY_NONE = 0,
    STRATEGY_1DATA_1LEB,
    STRATEGY_1DATA_1LEB_INDEXED,
    STRATEGY_INCREMENTAL_FIXED,
    STRATEGY_INCREMENTAL_VARIABLE,
    STRATEGY_INCREMENTAL_COUNTER,
};

// フラッシュの領域定義
static const FLASH_AREA_DEF_T FLASH_AREA_DEFS[] = {
    //    int			area;			// データ種別
    //    intptr_t		addr;			// 先頭アドレス
    //    size_t		leb_size;		// LEBサイズ(ヘッダ込み, 物理イレースブロックサイズ*n)
    //    size_t		num_lebs;		// LEB数
    //    int			strategy;		// 管理方式
    //    size_t		db_size;		// データブロックサイズ(ヘッダ除く)
    //    int			num_dbs;		// データブロック個数

    // area,						addr,		leb_size,	num_lebs,	strategy,						db_size,					num_dbs,
    { FLASH_AREA_NONE,				0x000000,	0,			0,			STRATEGY_NONE,					0,							0,		},
    { FLASH_AREA_DEVICE_ID,			0x000000,	4096,		1,			STRATEGY_1DATA_1LEB_INDEXED,	0,							0,		},
    { FLASH_AREA_APPKEY,			0x001000,	4096,		10,			STRATEGY_1DATA_1LEB_INDEXED,	0,							0,		},
    { FLASH_AREA_AUTH_META,			0x00B000,	4096,		20,			STRATEGY_1DATA_1LEB_INDEXED,	0,							0,		},
    { FLASH_AREA_AUTH,				0x01F000,	16384,		20,			STRATEGY_1DATA_1LEB_INDEXED,	0,							0,		},
    { FLASH_AREA_AUTH_LEARNED,		0x06F000,	16384,		20,			STRATEGY_1DATA_1LEB_INDEXED,	0,							0,		},
    { FLASH_AREA_AUTH_LEARNED_INC,	0x0BF000,	135168,		1,			STRATEGY_INCREMENTAL_FIXED,		sizeof(FLASH_DATA_AUTH_T) /*13200bytes */,	10,		},
	//////////////////
    { FLASH_AREA_STORE_PROGRAM,     0x130000,	720896,		1,			STRATEGY_1DATA_1LEB_INDEXED,	0,                          0,		}, // Store Program
//  { FLASH_AREA_STORE_PROGRAM,     0x130000,	4096,		176,		STRATEGY_1DATA_1LEB_INDEXED,	0,	                        0,		}, // Store Program
	//////////////////
    // 次: 0x0E0000
    // 2M: -0x1FFFFF
};

// ブロック状態
static const uint32_t BLOCK_STATE_ERASED		=	~(uint32_t)0x0;	// 初期状態
static const uint32_t BLOCK_STATE_EMPTY			=	~(uint32_t)0x1;	// 空
static const uint32_t BLOCK_STATE_VALID			=	~(uint32_t)0x3;	// 有効データあり
static const uint32_t BLOCK_STATE_FULL			=	~(uint32_t)0x7;	// データフル
static const uint32_t BLOCK_STATE_INVALID		=	~(uint32_t)0xF;	// 無効(削除待ち)

/*
 * 内部変数
 */
static MDLSTRG_CONTEXT_T s_context = {0};

#define COPY_BUF_SIZE	1024
static uint8_t s_copy_buf[COPY_BUF_SIZE];

/*
 * 初期化
 */
void mdlstrg_initialize(MDLSTRG_CALLBACK_T callback)
{
    ER er = act_tsk(TSK_MDLSTRG);
    assert(er == E_OK);

    mpf_send(MSG_INITIALIZE, NULL, callback);
}

/*
 * ストレージ要求
 */
void mdlstrg_request(const MDLSTRG_REQUEST_T* request, MDLSTRG_CALLBACK_T callback)
{
    mpf_send(MSG_REQUEST, request, callback);
}

/*
 * タスク
 */
void mdlstrg_task(intptr_t exinf)
{
    DBGLOG0("mdlstrg_task() starts .");

#if 1
    // テーブルチェック
    dly_tsk(100);
    intptr_t addr = 0;
    for (int i = 0; i < sizeof(FLASH_AREA_DEFS) / sizeof(FLASH_AREA_DEFS[0]); i++) {
        assert(FLASH_AREA_DEFS[i].addr % DRVFLX_ERASE_BLOCK_SIZE == 0);
        assert(addr <= FLASH_AREA_DEFS[i].addr);
        assert(FLASH_AREA_DEFS[i].area == i);
        assert(FLASH_AREA_DEFS[i].leb_size % DRVFLX_ERASE_BLOCK_SIZE == 0);
        addr += FLASH_AREA_DEFS[i].leb_size * FLASH_AREA_DEFS[i].num_lebs;
        dly_tsk(10);
    }
    assert(addr < 2 * 1024 * 1024);
#endif

    while (true) {
        intptr_t opt1 = 0;
        intptr_t opt2 = 0;
        MDLSTRG_MPFBLK_T* blk = NULL;
        ER er = 0;
        er = trcv_dtq(DTQ_MDLSTRG, (intptr_t*)&blk, TMO_FEVR);
        assert(er == E_OK);
        assert(blk);
        switch (blk->msg) {
        case MSG_INITIALIZE:
        {
            DBGLOG0("mdlstrg_task: MSG_INITIALIZE");
            assert(s_context.state == STATE_NONE);
            assert(blk->callback);

            // フラッシュドライバ初期化
            drvflx_initialize(drvflx_callback);

            blk->callback(MDLSTRG_EVT_INITIALIZE_COMPLETE, 0, 0);
            s_context.state = STATE_READY;

            break;
        }
        case MSG_REQUEST:
        {
            DBGLOG0("mdlstrg_task: MSG_REQUEST");
            assert(s_context.state == STATE_READY);
            assert(blk->callback);

            switch (blk->request.data_type) {
            case MDLSTRG_DATA_TYPE_NONE: {
                switch (blk->request.request_type) {
                case MDLSTRG_REQ_TYPE_FACTORY_RESET:
                    flash_erase(0, DRVFLX_FLASH_MEMORY_SIZE);
                    break;
                default:
                    break;
                }
                break;
            }
            case MDLSTRG_DATA_TYPE_DEVICE_ID: {	// 機器識別情報
                switch (blk->request.request_type) {
                case MDLSTRG_REQ_TYPE_READ:
                    read_device_id((BNVA_DEVICE_ID_T*)blk->request.data);
                    break;
                case MDLSTRG_REQ_TYPE_WRITE:
                    write_device_id((const BNVA_DEVICE_ID_T*)blk->request.data);
                    break;
                default:
                    assert(false);
                    break;
                }
                break;
            }
            case MDLSTRG_DATA_TYPE_AUTH: {
                switch (blk->request.request_type) {
                case MDLSTRG_REQ_TYPE_READ:
                    read_auth((MDLAUTHLIB_DATA_REGISTERED_T*)blk->request.data, blk->request.opt1);
                    break;
                case MDLSTRG_REQ_TYPE_WRITE: {
                    void** ptrs = (void**)(blk->request.data);
                    write_auth((const MDLAUTHLIB_DATA_REGISTERED_T*)ptrs[0],
                               (const MDLAUTHLIB_DATA_REGISTERED_SMALL_T*)ptrs[1], blk->request.opt1);
                    break;
                }
                case MDLSTRG_REQ_TYPE_DELETE:
                    delete_auth(blk->request.opt1);
                    break;
                case MDLSTRG_REQ_TYPE_EXISTS:
                    opt1 = exists_auth(blk->request.opt1);
                    break;
                default:
                    assert(false);
                    break;
                }
                break;
            }
            case MDLSTRG_DATA_TYPE_AUTH_META: {
                switch (blk->request.request_type) {
                case MDLSTRG_REQ_TYPE_READ:
                    opt1 = read_auth_meta((MDLSTRG_DATA_AUTH_META_T*)blk->request.data, blk->request.opt1);
                    break;
                case MDLSTRG_REQ_TYPE_WRITE: {
                    write_auth_meta((const MDLSTRG_DATA_AUTH_META_T*)blk->request.data, blk->request.opt1);
                    break;
                }
                case MDLSTRG_REQ_TYPE_DELETE:
                    delete_auth_meta(blk->request.opt1);
                    break;
                default:
                    assert(false);
                    break;
                }
                break;
            }
            case MDLSTRG_DATA_TYPE_AUTH_LEARNED: {
                switch (blk->request.request_type) {
                case MDLSTRG_REQ_TYPE_READ:
                    read_auth_learned((MDLAUTHLIB_DATA_REGISTERED_T*)blk->request.data, blk->request.opt1);
                    break;
                case MDLSTRG_REQ_TYPE_WRITE: {
                    void** ptrs = (void**)(blk->request.data);
                    write_auth_learned((const MDLAUTHLIB_DATA_REGISTERED_T*)ptrs[0],
                                       (const MDLAUTHLIB_DATA_REGISTERED_SMALL_T*)ptrs[1], blk->request.opt1);
                    break;
                }
                case MDLSTRG_REQ_TYPE_DELETE:
                    delete_auth_learned(blk->request.opt1);
                    break;
                case MDLSTRG_REQ_TYPE_EXISTS:
                    opt1 = exists_auth_learned(blk->request.opt1);
                    break;
                default:
                    assert(false);
                    break;
                }
                break;
            }
            case MDLSTRG_DATA_TYPE_AUTH_SMALL: {
                switch (blk->request.request_type) {
                case MDLSTRG_REQ_TYPE_READ:
                    read_auth_small((MDLSTRG_DATA_AUTH_SMALL_T*)blk->request.data);
                    break;
                case MDLSTRG_REQ_TYPE_WRITE:
                    assert(false);
                    break;
                case MDLSTRG_REQ_TYPE_DELETE:
                    break;
                default:
                    assert(false);
                    break;
                }
                break;
            }
            case MDLSTRG_DATA_TYPE_BLE_APPKEY: {	// アプリ鍵
                switch (blk->request.request_type) {
                case MDLSTRG_REQ_TYPE_READ:
                    read_appkey((MDLSTRG_DATA_APPKEY_T*)blk->request.data, blk->request.opt1);
                    break;
                case MDLSTRG_REQ_TYPE_WRITE:
                    write_appkey((const MDLSTRG_DATA_APPKEY_T*)blk->request.data, blk->request.opt1);
                    break;
                case MDLSTRG_REQ_TYPE_DELETE:
                    assert(false);
                    break;
                default:
                    assert(false);
                    break;
                }
                break;
            }
            ////////////////////////////////
            case MDLSTRG_DATA_TYPE_STORE_PROGRAM: {	// Store program
                switch (blk->request.request_type) {
                case MDLSTRG_REQ_TYPE_READ:
                    read_store_program((void*)blk->request.data, blk->request.opt2, blk->request.size, blk->request.opt1);
                    break;
                case MDLSTRG_REQ_TYPE_WRITE:
                    write_store_program((void*)blk->request.data, blk->request.opt2, blk->request.size, blk->request.opt1);
                    break;
                case MDLSTRG_REQ_TYPE_DELETE:
                    delete_store_program(blk->request.size, blk->request.opt1);
                    break;
                case MDLSTRG_REQ_TYPE_EXISTS:
                    opt1 = exists_store_program((size_t*)blk->request.opt2, blk->request.opt1);
                    break;
                default:
                    assert(false);
                    break;
                }
                break;
            }
            ////////////////////////////////
            default:
                assert(false);
                break;
            }

            blk->callback(MDLSTRG_EVT_REQUEST_COMPLETE, opt1, opt2);
            break;
        }
        default:
            assert(false);
            break;
        }

        er = rel_mpf(MPF_MDLSTRG, blk);
        assert(er == E_OK);
    }
}

/*********************************************************************
 * 内部関数
 ********************************************************************/

/*
 * メッセージ送信
 */
void mpf_send(int msg, const MDLSTRG_REQUEST_T* request, MDLSTRG_CALLBACK_T callback)
{
    MDLSTRG_MPFBLK_T* blk = NULL;
    ER er = 0;
    er = tget_mpf(MPF_MDLSTRG, (void**)&blk, TMO_FEVR);
    assert(er == E_OK);

    blk->msg = msg;
    if (request) {
        memcpy(&(blk->request), request, sizeof(MDLSTRG_REQUEST_T));
    } else {
        memset(&(blk->request), 0, sizeof(MDLSTRG_REQUEST_T));
    }
    blk->callback = callback;

    er = tsnd_dtq(DTQ_MDLSTRG, (intptr_t)blk, TMO_FEVR);
    assert(er == E_OK);
}

/*
 * フラッシュドライバのコールバック
 */
void drvflx_callback(int evt, int error, intptr_t opt)
{
    //DBGLOG3("drvflx_callback evt=%d, err=%d, opt=%d", evt, error, opt);

    ER er = E_OK;

    switch (evt) {
    case DRVFLX_EVT_INITIALIZE_COMPLETE:
        er = set_flg(FLG_MDLSTRG, FLGPTN_DRVFLX_INITIALIZE_COMPLETE);
        assert(er == E_OK);
        break;
    case DRVFLX_EVT_READ_COMPLETE:
        er = set_flg(FLG_MDLSTRG, FLGPTN_DRVFLX_READ_COMPLETE);
        assert(er == E_OK);
        break;
    case DRVFLX_EVT_WRITE_COMPLETE:
        er = set_flg(FLG_MDLSTRG, FLGPTN_DRVFLX_WRITE_COMPLETE);
        assert(er == E_OK);
        break;
    case DRVFLX_EVT_MEMMAP_STARTED:
        assert(!"NOT IMPLEMENTED");	// 非サポート
        break;
    case DRVFLX_EVT_MEMMAP_END:
        assert(!"NOT IMPLEMENTED");	// 非サポート
        break;
    case DRVFLX_EVT_ERASE_COMPLETE:
        er = set_flg(FLG_MDLSTRG, FLGPTN_DRVFLX_ERASE_COMPLETE);
        assert(er == E_OK);
        break;
    default:
        break;
    }
}

/*
 * 読込み
 */
void flash_read(void* dest, intptr_t src, size_t length)
{
    DBGLOG3("flash_read: FLASH:0x%08x => RAM:0x%08x (%dbytes)", src, dest, length);
    drvflx_read(dest, src, length, drvflx_callback);
    ER er = twai_flg(FLG_MDLSTRG, FLGPTN_DRVFLX_READ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
    assert(er == E_OK);
}

/*
 * 書込み
 */
void flash_write(intptr_t dest, const void* src, size_t length)
{
    DBGLOG3("flash_write: FLASH:0x%08x <= RAM:0x%08x (%dbytes)", dest, src, length);
    drvflx_write(dest, src, length, drvflx_callback);
    ER er = twai_flg(FLG_MDLSTRG, FLGPTN_DRVFLX_WRITE_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
    assert(er == E_OK);
}

/*
 * 消去
 */
void flash_erase(intptr_t dest, size_t length)
{
    DBGLOG2("flash_erase: FLASH:0x%08x (%dbytes)", dest, length);
    drvflx_erase(dest, length, drvflx_callback);
    ER er = twai_flg(FLG_MDLSTRG, FLGPTN_DRVFLX_ERASE_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 10000);
    assert(er == E_OK);
}

/*
 * コピー
 */
void flash_copy(intptr_t dest, intptr_t src, size_t length)
{
    DBGLOG3("flash_copy: FLASH:0x%08x <= FLASH:0x%08x (%dbytes)", dest, src, length);

    ER er = E_OK;
    size_t rest = length;
    size_t next = 0;
    size_t copied = 0;

    // 全て書き込むまで繰り返し
    while (rest > 0) {
        next = (rest <= sizeof(s_copy_buf)) ? (rest) : (sizeof(s_copy_buf));

        drvflx_read(s_copy_buf, src + copied, next, drvflx_callback);
        er = twai_flg(FLG_MDLSTRG, FLGPTN_DRVFLX_READ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);

        drvflx_write(dest + copied, s_copy_buf, next, drvflx_callback);
        er = twai_flg(FLG_MDLSTRG, FLGPTN_DRVFLX_WRITE_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);

        copied += next;
        rest -= next;
    }
}

/*********************************************************************
 * LEB, DB 操作関数
 ********************************************************************/

/*
 * データアドレス取得
 */
bool_t db_get_data_addr(intptr_t* a_addr, size_t* a_size, DB_CURSOR_T* cursor, int area, uint32_t data_id)
{
    assert(a_addr);
    assert(a_size);
    assert(cursor);
    assert(area > 0 && sizeof(FLASH_AREA_DEFS)/sizeof(FLASH_AREA_DEFS[0]));

    // 領域情報取得
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);

    bool_t found = false;
    switch (area_def->strategy) {
    case STRATEGY_1DATA_1LEB_INDEXED:
    {
        found = db_get_data_addr_1d1li(a_addr, a_size, cursor, area_def, data_id, SEARCH_FOR_VALID_DATA);
        break;
    }
    case STRATEGY_INCREMENTAL_FIXED:
    {
        found = db_get_data_addr_incf(a_addr, a_size, cursor, area_def, data_id, SEARCH_FOR_VALID_DATA);
        break;
    }
    default:
        assert(false);
        break;
    }
    return found;
    
}

/*
 * 新しいDBを割り当ててアドレス取得
 */
bool_t db_new_data(intptr_t* a_addr, size_t* a_size, DB_CURSOR_T* cursor, int area, uint32_t data_id)
{
    assert(a_addr);
    assert(a_size);
    assert(cursor);
    assert(area > 0 && sizeof(FLASH_AREA_DEFS)/sizeof(FLASH_AREA_DEFS[0]));

    // 領域情報取得
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);

    bool_t found = false;
    switch (area_def->strategy) {
    case STRATEGY_1DATA_1LEB_INDEXED:
    {
        found = db_get_data_addr_1d1li(a_addr, a_size, cursor, area_def, data_id, SEARCH_FOR_WRITE_DEST);
        break;
    }
    case STRATEGY_INCREMENTAL_FIXED:
    {
        found = db_get_data_addr_incf(a_addr, a_size, cursor, area_def, data_id, SEARCH_FOR_WRITE_DEST);
        break;
    }
    default:
        assert(false);
        break;
    }
    // 見付からない場合は終了
    if (!found) {
        goto end;
    }

    // ブロック消去
    switch (area_def->strategy) {
    case STRATEGY_1DATA_1LEB_INDEXED:
    {
        // LEB消去
        leb_erase(cursor->area_def, cursor->leb_index);
        break;
    }
    case STRATEGY_INCREMENTAL_FIXED:
    {
        // 既に存在するデータを無効化
        DB_CURSOR_T cur_exists = {0};
        bool_t data_exists = false;
        data_exists = db_get_data_addr_incf(&(intptr_t){0}, &(size_t){0}, &cur_exists, area_def, data_id, SEARCH_FOR_VALID_DATA);
        if (data_exists) {
            db_update_block_state(&cur_exists, BLOCK_STATE_INVALID);
        }
        break;
    }
    default:
        assert(false);
        break;
    }
    
end:
    return found;
}

/*
 * ブロックのヘッダにデータIDと状態フラグ(有効)を設定
 */
void db_set_valid(const DB_CURSOR_T* cursor, uint32_t data_id, size_t data_len)
{
    assert(cursor);

    DB_HEADER_T header = {0};
    flash_read(&header, cursor->db_addr, sizeof(DB_HEADER_T));
    assert(header.db_state == BLOCK_STATE_ERASED);

    header.data_id = data_id;
    header.data_len = data_len;
    header.db_state = BLOCK_STATE_VALID;

    flash_write(cursor->db_addr, &(header), sizeof(header));

    return;
}

/*
 * ブロックを無効にする
 */
void db_set_invalid(const DB_CURSOR_T* cursor)
{
    db_update_block_state(cursor, BLOCK_STATE_INVALID);
}

/*
 * ブロックの状態を更新する
 */
void db_update_block_state(const DB_CURSOR_T* cursor, uint32_t state)
{
    assert(cursor);

    DB_HEADER_T header = {0};
    flash_read(&header, cursor->db_addr, sizeof(DB_HEADER_T));
    assert(header.db_state & ~state);	// ビットを落とす操作のみ可
    flash_write(cursor->db_addr + offsetof(DB_HEADER_T, db_state), &state, sizeof(uint32_t));

    return;
}

/*
 * LEB消去
 */
void leb_erase(const FLASH_AREA_DEF_T* area_def, int leb_index)
{
    assert(area_def);

    // LEBアドレス
    intptr_t leb_addr = area_def->addr + (area_def->leb_size * leb_index);
    flash_erase(leb_addr, area_def->leb_size);

    // LEBヘッダ書込み
    LEB_HEADER_T leb_header = {0};
    leb_header.leb_state = BLOCK_STATE_EMPTY;
    flash_write(leb_addr, &leb_header, sizeof(LEB_HEADER_T));

    return;
}


bool_t db_get_data_addr_1d1li(intptr_t* a_addr, size_t* a_size, DB_CURSOR_T* cursor, const FLASH_AREA_DEF_T* area_def, uint32_t data_id, int search_for)
{
    assert(a_addr);
    assert(a_size);
    assert(cursor);
    assert(area_def);

    intptr_t leb_addr = 0;
    intptr_t db_addr = 0;
    DB_HEADER_T db_header = {0};
    bool_t found = false;

    assert(data_id < area_def->num_lebs);

    // LEBアドレス(内容は見ない)
    leb_addr = area_def->addr + (area_def->leb_size * data_id);

    // DBヘッダの読み出し
    db_addr = leb_addr + sizeof(LEB_HEADER_T);
    flash_read(&db_header, db_addr, sizeof(DB_HEADER_T));

    // データが有効かをチェック
    if (search_for == SEARCH_FOR_VALID_DATA) {
        if (db_header.db_state == BLOCK_STATE_VALID) {
            found = true;
        }
    } else if (search_for == SEARCH_FOR_WRITE_DEST) {
        found = true;
    } else {
        assert(false);
    }
    if (found) {
        *a_addr = db_addr + sizeof(DB_HEADER_T);
        *a_size = db_header.data_len;
        cursor->area_def = area_def;
        cursor->leb_index = data_id;
        cursor->db_addr = db_addr;
        cursor->db_state = db_header.db_state;
    }
    
    return found;
}

bool_t db_get_data_addr_incf(intptr_t* a_addr, size_t* a_size, DB_CURSOR_T* cursor, const FLASH_AREA_DEF_T* area_def, uint32_t data_id, int search_for)
{
    assert(a_addr);
    assert(a_size);
    assert(cursor);
    assert(area_def);

    intptr_t leb_addr = 0;
    intptr_t db_addr = 0;
    LEB_HEADER_T leb_header = {0};
    DB_HEADER_T db_header = {0};
    bool_t found = false;

    // 全LEB
    for (int i = 0; i < area_def->num_lebs; i++) {
        // LEBヘッダの読み出し
        leb_addr = area_def->addr + (area_def->leb_size * i);
        flash_read(&leb_header, leb_addr, sizeof(LEB_HEADER_T));

#if 0	// LEBヘッダは使わない
        // LEBが有効かをチェック
        found = false;
        if (search_for == SEARCH_FOR_VALID_DATA) {
            if (leb_header.leb_state == BLOCK_STATE_VALID) {
                found = true;
            }
        } else if (search_for == SEARCH_FOR_WRITE_DEST) {
            if ((leb_header.leb_state == BLOCK_STATE_ERASED) || (leb_header.leb_state == BLOCK_STATE_EMPTY) || (leb_header.leb_state == BLOCK_STATE_VALID)) {
                found = true;
            }
        } else {
            assert(false);
        }
        if (!found) {
            continue;
        }
#endif

        found = false;
        db_addr = leb_addr + sizeof(LEB_HEADER_T);
        for (int j = 0; j < area_def->num_dbs; j++) {
            // DBヘッダの読み出し
            flash_read(&db_header, db_addr, sizeof(DB_HEADER_T));
            // データが有効かをチェック
            if (search_for == SEARCH_FOR_VALID_DATA) {
                if ((db_header.db_state == BLOCK_STATE_VALID) && (db_header.data_id == data_id)) {
                    found = true;
                }
            } else if (search_for == SEARCH_FOR_WRITE_DEST) {
                if ((db_header.db_state == BLOCK_STATE_ERASED) || (db_header.db_state == BLOCK_STATE_EMPTY)) {
                    found = true;
                }
            }
            if (found) {
                *a_addr = db_addr + sizeof(DB_HEADER_T);
                *a_size = area_def->data_size;
                cursor->area_def = area_def;
                cursor->leb_index = i;
                cursor->db_addr = db_addr;
                cursor->db_state = db_header.db_state;
                break;
            }
            db_addr += (sizeof(DB_HEADER_T) + area_def->data_size);
        }	// END for j
        if (found) {
            break;
        }
    }	// END for i

    return found;
}

/*********************************************************************
 * データ種別毎
 ********************************************************************/

/*
 * 認証データ読み出し
 */
bool_t read_auth(MDLAUTHLIB_DATA_REGISTERED_T* dest, int index)
{
    assert(dest);
    assert(index >= 0 && index < MDLSTRG_MAX_NDATA_AUTH);

    bool_t found = false;
    intptr_t addr = 0;
    size_t size = 0;
    DB_CURSOR_T cursor = {0};

    // 登録データ
    found = db_get_data_addr(&addr, &size, &cursor, FLASH_AREA_AUTH, index);
    if (!found) {
        goto end;	// 見付からない場合
    }

    // データ取得
    assert(size == sizeof(FLASH_DATA_AUTH_T));
    flash_read(&(dest->data), addr + offsetof(FLASH_DATA_AUTH_T, auth), sizeof(FLASH_DATA_AUTH_T));

end:
    return found;
}

/*
 * 認証データ書込み
 */
void write_auth(const MDLAUTHLIB_DATA_REGISTERED_T* data, const MDLAUTHLIB_DATA_REGISTERED_SMALL_T* data_small, int index)
{
    assert(data);
    assert(index >= 0 && index < MDLSTRG_MAX_NDATA_AUTH);

    bool_t found = false;
    intptr_t addr = 0;
    size_t size = 0;
    DB_CURSOR_T cursor = {0};
    size_t offset = 0;

    // 書き込み先を取得
    found = db_new_data(&addr, &size, &cursor, FLASH_AREA_AUTH, index);
    assert(found);

    // data
    offset = offsetof(FLASH_DATA_AUTH_T, auth);
    flash_write(addr + offset, data, sizeof(MDLAUTHLIB_DATA_REGISTERED_T));

    // small
    offset = offsetof(FLASH_DATA_AUTH_T, auth_small);
    flash_write(addr + offset, data_small, sizeof(MDLAUTHLIB_DATA_REGISTERED_SMALL_T));

    // ヘッダを更新
    db_set_valid(&cursor, index, sizeof(FLASH_DATA_AUTH_T));

    // 学習データを削除
    delete_auth_learned(index);

    return;
}

/*
 * 認証データ削除
 */
void delete_auth(int index)
{
    assert(index >= 0 && index < MDLSTRG_MAX_NDATA_AUTH);

    bool_t found = false;
    intptr_t addr = 0;
    size_t size = 0;
    DB_CURSOR_T cursor = {0};

    // 登録情報を削除
    found = db_get_data_addr(&addr, &size, &cursor, FLASH_AREA_AUTH, index);
    if (found) {
        DBGLOG1("delete: %d", cursor.leb_index);
        db_set_invalid(&cursor);
    }

    // 登録メタ情報を削除
    delete_auth_meta(index);

    // 学習情報を削除
    delete_auth_learned(index);

    return;
}

/*
 * 認証データ存在確認
 */
bool_t exists_auth(int index)
{
    bool_t found = false;
    intptr_t addr = 0;
    size_t size = 0;
    DB_CURSOR_T cursor = {0};

    // 登録情報があるか確認
    found = db_get_data_addr(&addr, &size, &cursor, FLASH_AREA_AUTH, index);

    return found;
}

/*
 * 認証データ読み出し
 */
bool_t read_auth_meta(MDLSTRG_DATA_AUTH_META_T* dest, int index)
{
    assert(dest);
    assert(index >= 0 && index < MDLSTRG_MAX_NDATA_AUTH);

    bool_t found = false;
    intptr_t addr = 0;
    size_t size = 0;
    DB_CURSOR_T cursor = {0};

    // 登録データ
    found = db_get_data_addr(&addr, &size, &cursor, FLASH_AREA_AUTH_META, index);
    if (!found) {
        goto end;	// 見付からない場合
    }

    // データ取得
    assert(size == sizeof(MDLSTRG_DATA_AUTH_META_T));
    flash_read(dest, addr, sizeof(MDLSTRG_DATA_AUTH_META_T));

end:
    return found;
}

/*
 * 認証データ書込み
 */
void write_auth_meta(const MDLSTRG_DATA_AUTH_META_T* data, int index)
{
    assert(data);
    assert(index >= 0 && index < MDLSTRG_MAX_NDATA_AUTH);

    bool_t found = false;
    intptr_t addr = 0;
    size_t size = 0;
    DB_CURSOR_T cursor = {0};

    // 書き込み先を取得
    found = db_new_data(&addr, &size, &cursor, FLASH_AREA_AUTH_META, index);
    assert(found);

    // data
    flash_write(addr, data, sizeof(MDLSTRG_DATA_AUTH_META_T));

    // ヘッダを更新
    db_set_valid(&cursor, index, sizeof(MDLSTRG_DATA_AUTH_META_T));

    return;
}

/*
 * 認証データ削除
 */
void delete_auth_meta(int index)
{
    assert(index >= 0 && index < MDLSTRG_MAX_NDATA_AUTH);

    bool_t found = false;
    intptr_t addr = 0;
    size_t size = 0;
    DB_CURSOR_T cursor = {0};

    // 登録情報を削除
    found = db_get_data_addr(&addr, &size, &cursor, FLASH_AREA_AUTH_META, index);
    if (found) {
        db_set_invalid(&cursor);
    }

    return;
}

/*
 * 認証学習データ読み出し
 */
bool_t read_auth_learned(MDLAUTHLIB_DATA_REGISTERED_T* dest, int index)
{
    bool_t found = false;
    intptr_t addr = 0;
    size_t size = 0;
    DB_CURSOR_T cursor = {0};

    // 学習データ
    found = db_get_data_addr(&addr, &size, &cursor, FLASH_AREA_AUTH_LEARNED_INC, index);
    if (!found) {
        // 学習データ(追記領域)
        found = db_get_data_addr(&addr, &size, &cursor, FLASH_AREA_AUTH_LEARNED, index);
    }
    if (!found) {
        goto end;
    }

    // データ取得
    assert(size == sizeof(FLASH_DATA_AUTH_T));
    flash_read(&(dest->data), addr + offsetof(FLASH_DATA_AUTH_T, auth), sizeof(FLASH_DATA_AUTH_T));
    DBGLOG1("read_auth_learned 0x%08x", addr + offsetof(FLASH_DATA_AUTH_T, auth));

end:
    return found;
}

/*
 * 認証学習データ書込み
 */
void write_auth_learned(const MDLAUTHLIB_DATA_REGISTERED_T* data, const MDLAUTHLIB_DATA_REGISTERED_SMALL_T* data_small, int index)
{
    bool_t found = false;
    intptr_t addr = 0;
    size_t size = 0;
    DB_CURSOR_T cursor = {0};

    // 追記領域から書き込み先を取得
    found = db_new_data(&addr, &size, &cursor, FLASH_AREA_AUTH_LEARNED_INC, index);

    if (!found) {
        // 追記領域フルの場合, 全件反映処理
        for (int i = 0; i < MDLSTRG_MAX_NDATA_AUTH; i++) {
            found = db_get_data_addr(&addr, &size, &cursor, FLASH_AREA_AUTH_LEARNED_INC, i);
            if (found) {
                found = db_new_data(&addr, &size, &cursor, FLASH_AREA_AUTH_LEARNED, i);
                assert(found);
                flash_copy(addr, addr, sizeof(MDLAUTHLIB_DATA_REGISTERED_T));
            }
        }

        // 追記領域を消去
        leb_erase(&(FLASH_AREA_DEFS[FLASH_AREA_AUTH_LEARNED_INC]), 0);
        found = db_new_data(&addr, &size, &cursor, FLASH_AREA_AUTH_LEARNED_INC, index);
    }
    assert(found);

    // data
    size_t offset = 0;
    offset = offsetof(FLASH_DATA_AUTH_T, auth);
    flash_write(addr + offset, data, sizeof(MDLAUTHLIB_DATA_REGISTERED_T));

    // small
    offset = offsetof(FLASH_DATA_AUTH_T, auth_small);
    flash_write(addr + offset, data_small, sizeof(MDLAUTHLIB_DATA_REGISTERED_SMALL_T));

    // ヘッダを更新
    db_set_valid(&cursor, index, sizeof(FLASH_DATA_AUTH_T));
    DBGLOG1("write_auth_learned 0x%08x", addr + offsetof(FLASH_DATA_AUTH_T, auth));

    return;
}

/*
 * 認証学習データ削除
 */
void delete_auth_learned(int index)
{
    bool_t found = false;
    intptr_t addr = 0;
    size_t size = 0;
    DB_CURSOR_T cursor = {0};

    // 学習データ(追記領域)
    found = db_get_data_addr(&addr, &size, &cursor, FLASH_AREA_AUTH_LEARNED_INC, index);
    if (found) {
        db_set_invalid(&cursor);
    }

    // 学習データ
    found = db_get_data_addr(&addr, &size, &cursor, FLASH_AREA_AUTH_LEARNED, index);
    if (found) {
        db_set_invalid(&cursor);
    }

    return;
}

/*
 * 認証学習データ存在確認
 */
bool_t exists_auth_learned(int index)
{
    bool_t found = false;
    intptr_t addr = 0;
    size_t size = 0;
    DB_CURSOR_T cursor = {0};

    // 登録情報があるか確認
    found = db_get_data_addr(&addr, &size, &cursor, FLASH_AREA_AUTH_LEARNED_INC, index);
    if (!found) {
        found = db_get_data_addr(&addr, &size, &cursor, FLASH_AREA_AUTH_LEARNED, index);
    }

    return found;
}


/*
 * 極小データ読み出し
 */
void read_auth_small(MDLSTRG_DATA_AUTH_SMALL_T* dest)
{
    assert(dest);

    // フラッシュエリア情報取得
    intptr_t base = 0;
    size_t size = 0;
    DB_CURSOR_T cursor = {0};

    // フラッシュエリア
    dest->count = 0;
    size_t offset = 0;
    for (int i = 0; i < MDLSTRG_MAX_NDATA_AUTH; i++) {
        bool_t found = false;

        // 登録データ
        found = db_get_data_addr(&base, &size, &cursor, FLASH_AREA_AUTH, i);
        if (found) {
            // data_type, index
            dest->array[dest->count].data_type = MDLSTRG_DATA_TYPE_AUTH;
            dest->array[dest->count].index = i;

            // small data
            offset = offsetof(FLASH_DATA_AUTH_T, auth_small);
            flash_read(&(dest->array[dest->count].data), base + offset, sizeof(MDLAUTHLIB_DATA_REGISTERED_SMALL_T));

            dest->count++;
        }

        // 学習データ
        found = db_get_data_addr(&base, &size, &cursor, FLASH_AREA_AUTH_LEARNED_INC, i);
        if (!found) {
            found = db_get_data_addr(&base, &size, &cursor, FLASH_AREA_AUTH_LEARNED, i);
        }

        if (found) {
            // data_type, index
            dest->array[dest->count].data_type = MDLSTRG_DATA_TYPE_AUTH_LEARNED;
            dest->array[dest->count].index = i;

            // small data
            offset = offsetof(FLASH_DATA_AUTH_T, auth_small);
            flash_read(&(dest->array[dest->count].data), base + offset, sizeof(MDLAUTHLIB_DATA_REGISTERED_SMALL_T));

            dest->count++;
        }
    }

    return;
}

/*
 * アプリ鍵情報読み出し
 */
bool_t read_appkey(MDLSTRG_DATA_APPKEY_T* dest, int index)
{
    assert(dest);
    assert(index >= 0 && index < 10);

    bool_t found = false;
    intptr_t addr = 0;
    size_t size = 0;
    DB_CURSOR_T cursor = {0};

    // 機器識別情報
    found = db_get_data_addr(&addr, &size, &cursor, FLASH_AREA_APPKEY, index);
    if (!found) {
        goto end;	// 見付からない場合
    }

    // フラッシュ読込み
    flash_read(dest, addr, sizeof(MDLSTRG_DATA_APPKEY_T));

end:
    return found;
}

/*
 * アプリ登録情報書込み
 */
void write_appkey(const MDLSTRG_DATA_APPKEY_T* data, int index)
{
    assert(data);
    assert(index >= 0 && index < 10);

    bool_t found = false;
    intptr_t addr = 0;
    size_t size = 0;
    DB_CURSOR_T cursor = {0};

    // 書き込み先を取得
    found = db_new_data(&addr, &size, &cursor, FLASH_AREA_APPKEY, index);
    assert(found);

    // フラッシュ書込み
    flash_write(addr, data, sizeof(MDLSTRG_DATA_APPKEY_T));

    // ヘッダを更新
    db_set_valid(&cursor, 0, sizeof(MDLSTRG_DATA_APPKEY_T));

    return;
}

/*
 * 機器識別情報読込み
 */
bool_t read_device_id(BNVA_DEVICE_ID_T* dest)
{
    assert(dest);

    bool_t found = false;
    intptr_t addr = 0;
    size_t size = 0;
    DB_CURSOR_T cursor = {0};

    // 機器識別情報
    found = db_get_data_addr(&addr, &size, &cursor, FLASH_AREA_DEVICE_ID, 0);
    if (!found) {
        goto end;	// 見付からない場合
    }

    // フラッシュ読込み
    flash_read(dest, addr, sizeof(BNVA_DEVICE_ID_T));

end:
    return found;
}

/*
 * 機器識別情報書込み
 */
void write_device_id(const BNVA_DEVICE_ID_T* device_id)
{
    assert(device_id);

    bool_t found = false;
    intptr_t addr = 0;
    size_t size = 0;
    DB_CURSOR_T cursor = {0};

    // 書き込み先を取得
    found = db_new_data(&addr, &size, &cursor, FLASH_AREA_DEVICE_ID, 0);
    assert(found);

    // フラッシュ書込み
    flash_write(addr, device_id, sizeof(BNVA_DEVICE_ID_T));

    // ヘッダを更新
    db_set_valid(&cursor, 0, sizeof(BNVA_DEVICE_ID_T));

    return;
}

////////////////////////

/*
 * Read from Store Program memory
 */
bool_t read_store_program(void* data, intptr_t address, size_t size, int index)
{
    assert(data);
    assert(index >= 0 && index < 176);
	
    bool_t found = false;
    intptr_t db_addr = 0;
    size_t block_size = 0;
    DB_CURSOR_T cursor = {0};

    // Get Store Program address
    found = db_get_data_addr(&db_addr, &block_size, &cursor, FLASH_AREA_STORE_PROGRAM, index);
    if (!found) {
        goto end;
    }

    // Read Store Program data
    flash_read(data, db_addr + address, size);

end:
    return found;
}
/*
 * Write to Store Program memory
 */
void write_store_program(void* data, intptr_t address, size_t size, int index)
{
    assert(data);
    assert(index >= 0 && index < 176);

    bool_t found = false;
    intptr_t db_addr = 0;
    size_t block_size = 0;
    DB_CURSOR_T cursor = {0};

    // Get write destination
    found = db_get_data_addr_1d1li(&db_addr, &block_size, &cursor, &(FLASH_AREA_DEFS[FLASH_AREA_STORE_PROGRAM]), index, SEARCH_FOR_WRITE_DEST);
    assert(found);

    // Write to Store Program to flash
    flash_write(db_addr + address, data, size);
}

/*
 * Delete existing Store Program
 */
void delete_store_program(size_t size, int index)
{
    bool_t found = false;
    intptr_t db_addr = 0;
    size_t block_size = 0;
    DB_CURSOR_T cursor = {0};

    // Get Store Program address
    found = db_get_data_addr_1d1li(&db_addr, &block_size, &cursor, &(FLASH_AREA_DEFS[FLASH_AREA_STORE_PROGRAM]), index, SEARCH_FOR_WRITE_DEST);
    //found = db_get_data_addr(&addr, &block_size, &cursor, FLASH_AREA_STORE_PROGRAM, index);
    if (found) {
        if (size == 0) {
            db_set_invalid(&cursor);
        } else {
            intptr_t leb_addr = cursor.area_def->addr + (cursor.area_def->leb_size * index);
            size_t erase_size = size + sizeof(LEB_HEADER_T) + sizeof(DB_HEADER_T);
            erase_size = (((erase_size - 1) / DRVFLX_ERASE_BLOCK_SIZE) + 1) * DRVFLX_ERASE_BLOCK_SIZE;
            DBGLOG2("Erasing %08x, size %d", leb_addr, erase_size);
            flash_erase(leb_addr, erase_size);

            // Write LEB header
            LEB_HEADER_T leb_header = {0};
            leb_header.leb_state = BLOCK_STATE_EMPTY;
            flash_write(leb_addr, &leb_header, sizeof(LEB_HEADER_T));

            // Write DB header
            DB_HEADER_T header = {0};
            header.data_id = index;
            header.data_len = size;
            header.db_state = BLOCK_STATE_VALID;
            flash_write(cursor.db_addr, &(header), sizeof(header));
        }
    }

    return;
}

/*
 * Check existence of Store Program
 */
bool_t exists_store_program(size_t* size, int index)
{
    bool_t found = false;
    intptr_t addr = 0;
    DB_CURSOR_T cursor = {0};

    // Check if registered
    found = db_get_data_addr(&addr, size, &cursor, FLASH_AREA_STORE_PROGRAM, index);

    return found;
}
///////////////////////
