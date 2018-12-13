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
#include "mdlstrg.h"
#include "drvflx.h"

/*
 * マクロ定義
 */

// デバッグログ
#if 0
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

// #define CARD_STATUS_NO_DATA             0xFFFFFFFFFFFFFFFF
// #define CARD_STATUS_EXISTS              0x7FFFFFFFFFFFFFFF
#define CARD_STATUS_NO_DATA                0xFF
#define ID_CODE_NO_DATA                    0xFFFF
#define LOG_NO_DATA                        0xFFFF
#define ID_CODE_INIT                       0xF
#define LOG_CNT_NO_DATA                    0xFFFFFFFFFFFFFFFF

#define MAX_NUM_VACANCY_CARD                10
#define DEFAULT_NUM_ID_CODE                 10
#define MAX_NUM_LOG_SECTOR                  16
#define LOG_SECTOR_SIZE                     4096
#define MAX_NUM_OPE_LOG                     72
#define MAX_NUM_ERR_LOG                    100

#define LATEST_ID_CODE                     0x7FFF
#define OLD_ID_CODE                        0x4FFF

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
/*
typedef struct {
    uint32_t	data_id;	// データID
    size_t		data_len;	// データ長
    uint32_t	db_state;	// データブロック状態
} DB_HEADER_T;
*/
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

// STRATEGY_1DATA_1LEB_INDEXED 用処理
static bool_t c_db_get_data_addr_1d1li(intptr_t* a_addr, size_t* a_size, DB_CURSOR_T* cursor, const FLASH_AREA_DEF_T* area_def, uint32_t data_id, int search_for);

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
static void init_store_program();
static bool_t read_store_program(void* data, intptr_t address, size_t size, int index);
static void write_store_program(void* data, intptr_t address, size_t size, int index);
static void delete_store_program(size_t size, int index);
static bool_t exists_store_program(size_t* size, int index);
//////////////////////

// 識別コードのフラグ比較
bool_t compare_id_code_flg(intptr_t addr, uint16_t flg);

// 識別コードのINDEX比較
bool_t compare_id_code_index(intptr_t addr, uint16_t index);

// ログカウンタ更新
static void update_log_cnt(intptr_t addr);

// 空のログセクターを探す
static int search_current_sector(intptr_t addr);

// 書き込みカウンタ確認
static int check_write_cnt(intptr_t addr);

// ログ読込カウンタ更新(操作/エラー共通)
static void increment_read_log_cnt(int flash_area);

// ログR/Wカウンタ取得(操作/エラー共通)
static bool_t get_read_write_cnt(MDLSTRG_DATA_RW_COUNTER_T* data, int flash_area);

// 最新ログ取得(操作/エラー共通)
static bool_t read_latest_log(void* data, int flash_area);

// ログカウンタベリファイ
ER verify_log_cnt(unsigned long long before[2], intptr_t addr_flash, int size);

// -------------------------------------------------------------------
// カードデータ操作
// -------------------------------------------------------------------
// カード登録状況確認（存在確認）
bool_t exists_card(intptr_t addr);

// ベリファイ
ER verify(intptr_t addr_ram, intptr_t addr_flash, int size);

// 一部データ更新（RAMへデータ退避後に更新）
bool_t update_flash(intptr_t addr_flash_area, intptr_t addr_flash_pos, size_t leb_size, intptr_t addr_ram, size_t size);

// -------------------------------------------------------------------
// カード機能リクエスト処理関数
// -------------------------------------------------------------------

// 空室カード存在確認
static bool_t exists_vacancy_card(int index);

// 空室カード追加
static bool_t append_vacancy_card(const MDLSTRG_DATA_CARD_T* data);

// 空室カード読込
static void read_vacancy_card(MDLSTRG_DATA_CARD_T* data_array);

// 空室カード情報削除
static void delete_vacancy_card();

// D-roomカードデータ書き込み
static bool_t write_droom_card(const MDLSTRG_DATA_DROOM_CARD_T* data, int index);

// D-roomカードデータ削除（セクター単位）
static void all_delete_droom_card();

// 識別コード管理データ作成
static bool_t create_id_code_mng(MDLSTRG_DATA_IDENTIFY_CODE_MNG_T* data_array);

// 識別コード追加
static ER append_id_code(const MDLSTRG_DATA_IDCODE_T* data);

// 識別コード削除
static void delete_id_code();

// 識別コード初期化
static bool_t init_id_code();

// 登録・抹消カード存在確認
static bool_t exists_reg_card();

// 登録・抹消カード読み出し
static bool_t read_reg_card(MDLSTRG_DATA_CARD_T* data);

// 登録・抹消カード書き込み
static bool_t write_reg_card(const MDLSTRG_DATA_CARD_T* data);

// 一括登録カードデータ存在確認
static bool_t exists_vacancy_reg_card();

// 一括登録カードデータ読み出し
static bool_t read_vacancy_reg_card(MDLSTRG_DATA_CARD_T* data);

// 一括登録カードデータ書き込み
static bool_t write_vacancy_reg_card(const MDLSTRG_DATA_CARD_T* data);

// WIFI接続情報存在確認
static bool_t exists_wifi_param();

// WIFI接続情報読み出し
static bool_t read_wifi_param(MDLSTRG_DATA_WIFI_PARAM_T* data);

// WIFI接続情報書き込み
static bool_t write_wifi_param(const MDLSTRG_DATA_WIFI_PARAM_T* data);

// 操作ログ読込カウンタ更新
static void increment_ope_log();

// 操作ログ書き込み
static ER write_ope_log(const MDLSTRG_DATA_OPE_LOG_T* data, intptr_t data_cnt);

// 操作ログ読込み
static bool_t read_ope_log(MDLSTRG_DATA_OPE_LOG_T* data);

// 操作ログR/Wカウンタ読込
static bool_t read_ope_log_cnt(MDLSTRG_DATA_RW_COUNTER_T* data);

// エラーログ読込カウンタ更新
static void  increment_err_log();

// エラーログ書き込み
static ER write_err_log(const MDLSTRG_DATA_ERR_LOG_T* data);

// エラーログ読込み
static bool_t read_err_log(MDLSTRG_DATA_ERR_LOG_T* data);

// エラーログR/Wカウンタ読込
static bool_t read_err_log_cnt(MDLSTRG_DATA_RW_COUNTER_T* data);

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
    FLASH_AREA_ROOM_STS,            // カードによる部屋の状態フラグ
    FLASH_AREA_CARD_META,			// カード：登録メタ情報(名前など)
    FLASH_AREA_VACANCY_CARD,        // 空室カード
    FLASH_AREA_DROOM_CARD,          // D-roomカード
    FLASH_AREA_IDCODE,              // 識別コード
    FLASH_AREA_WIFI_PARAM,          // WIFI接続情報
    FLASH_AREA_STORE_PROGRAM,       // Store Program
    FLASH_AREA_OPE_LOG,             // 操作ログ
    FLASH_AREA_ERR_LOG,             // エラーログ
    // D-roomカード
    // ログ何件か
    // Wifiの接続情報
    // カウンター
    //   認証OK/NGなど
    //   電気錠の開閉回数
    // 
};

// フラッシュの保存方法
enum {
    STRATEGY_NONE = 0,
    STRATEGY_1DATA_1LEB,
    STRATEGY_1DATA_1LEB_INDEXED,
    STRATEGY_NDATA_1LEB_INDEXED,
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
    { FLASH_AREA_NONE,				0x000000,	0,			0,			STRATEGY_NONE,					0,							    0,		},
    { FLASH_AREA_DEVICE_ID,			0x000000,	4096,		1,			STRATEGY_1DATA_1LEB_INDEXED,	0,							    0,		},
    { FLASH_AREA_APPKEY,			0x001000,	4096,		10,			STRATEGY_1DATA_1LEB_INDEXED,	0,							    0,		},
    { FLASH_AREA_AUTH_META,			0x00B000,	4096,		20,			STRATEGY_1DATA_1LEB_INDEXED,	0,							    0,		},
    { FLASH_AREA_AUTH,				0x01F000,	16384,		20,			STRATEGY_1DATA_1LEB_INDEXED,	0,							    0,		},
    { FLASH_AREA_AUTH_LEARNED,		0x06F000,	16384,		20,			STRATEGY_1DATA_1LEB_INDEXED,	0,							    0,		},
    { FLASH_AREA_AUTH_LEARNED_INC,	0x0BF000,	135168,		1,			STRATEGY_INCREMENTAL_FIXED,		sizeof(FLASH_DATA_AUTH_T),	    10,		},
    // 次: 0x0E0000
    { FLASH_AREA_ROOM_STS,          0x110000,   4096,       1,          STRATEGY_1DATA_1LEB_INDEXED,    0,                              0,      },
    { FLASH_AREA_CARD_META,			0x111000,	4096,		1,			STRATEGY_NDATA_1LEB_INDEXED,	0,							    0,		},
    // 次: 0x110000
    { FLASH_AREA_VACANCY_CARD,      0x120000,   4096,       1,          STRATEGY_NONE,                  sizeof(MDLSTRG_DATA_CARD_T),                              0,      },
    { FLASH_AREA_DROOM_CARD,        0x121000,   4096,       1,          STRATEGY_NONE,                  0,                              0,      },
    { FLASH_AREA_IDCODE,            0x122000,   4096,       2,          STRATEGY_NONE,                  sizeof(MDLSTRG_DATA_IDCODE_T),    0,      },
    { FLASH_AREA_WIFI_PARAM,        0x124000,   4096,       1,          STRATEGY_NONE,                  0,                              0,      },
    { FLASH_AREA_STORE_PROGRAM,     0x130000,   720896,     1,          STRATEGY_1DATA_1LEB_INDEXED,    0,                              0,      }, // Store Program
    { FLASH_AREA_OPE_LOG,           0x1E0000,   4096,       16,         STRATEGY_INCREMENTAL_FIXED,     0/*sizeof(FLASH_DATA_OPE_LOG_T)*/,   0,      },
    { FLASH_AREA_ERR_LOG,           0x1F0000,   4096,       16,         STRATEGY_INCREMENTAL_FIXED,     0/*sizeof(FLASH_DATA_ERR_LOG_T)*/,   0,      },
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
MDLSTRG_DATA_IDENTIFY_CODE_MNG_T id_code_mng[DEFAULT_NUM_ID_CODE];
int* ptr_id_code_mng = id_code_mng;

#define COPY_BUF_SIZE	1024
static uint8_t s_copy_buf[COPY_BUF_SIZE];

// カード情報一時退避領域
#define FLASH_UPDATE_BUF_SIZE 1808      // Droomカード領域のバイト数
static uint8_t s_update_buf[FLASH_UPDATE_BUF_SIZE];

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

            ER er = twai_flg(FLG_MDLSTRG, FLGPTN_DRVFLX_INITIALIZE_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
            assert(er == E_OK);

            init_store_program(0);

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
            case MDLSTRG_DATA_TYPE_CARD_META: {
                switch (blk->request.request_type) {
                case MDLSTRG_REQ_TYPE_READ:
                    opt1 = read_card_meta((MDLSTRG_DATA_CHASH_META_T*)blk->request.data, blk->request.opt1);
                    break;

                case MDLSTRG_REQ_TYPE_WRITE: {
                    write_card_meta((const MDLSTRG_DATA_CHASH_META_T*)blk->request.data, blk->request.opt1);
                    break;
                }
/*
                case MDLSTRG_REQ_TYPE_DELETE:
                    delete_auth_meta(blk->request.opt1);
                    break;
*/
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
            case MDLSTRG_DATA_TYPE_VACANCY_CARD: {          // 空室カード情報
                switch (blk->request.request_type) {
                // case MDLSTRG_REQ_TYPE_EXISTS:               // データ存在（空室カードのデータ存在確認はなし）
                //     break;
                case MDLSTRG_REQ_TYPE_APPEND: {             // 追加
                    DBGLOG0("MDLSTRG_REQ_TYPE_APPEND");
                    opt1 = append_vacancy_card((const MDLSTRG_DATA_CARD_T*)blk->request.data);
                    break;
                }    
                case MDLSTRG_REQ_TYPE_READ: {               // 読込み
                    read_vacancy_card((MDLSTRG_DATA_CARD_T*)blk->request.data);
                    break;
                }    
                case MDLSTRG_REQ_TYPE_ALL_DELETE: {         // 削除（セクター単位）
                    delete_vacancy_card();
                    break;
                }
                default:
                    DBGLOG1("MDLSTRG_REQ_TYPE %d", blk->request.request_type);
                    assert(false);
                    break;
                }
                break;
            }
            case MDLSTRG_DATA_TYPE_DROOM_CARD: {            // D-roomカード
                switch (blk->request.request_type) {
                case MDLSTRG_REQ_TYPE_EXISTS:               // データ存在
                    DBGLOG0("No Implement");
                    break;
                case MDLSTRG_REQ_TYPE_WRITE:                // 書き込み
                    opt1 = write_droom_card((const MDLSTRG_DATA_DROOM_CARD_T*)blk->request.data, blk->request.opt1);
                    break;
                case MDLSTRG_REQ_TYPE_READ:                 // 読込み
                    read_droom_card((MDLSTRG_DATA_DROOM_CARD_T*)blk->request.data);
                    break;
                case MDLSTRG_REQ_TYPE_DELETE:               // データ削除
                    opt1 = delete_droom_card(blk->request.opt1);
                    break;
                case MDLSTRG_REQ_TYPE_ALL_DELETE:           // データ削除（セクター単位）
                    all_delete_droom_card();
                    break;
                default:
                    assert(false);
                    break;
                }
                break;
            }
            case MDLSTRG_DATA_TYPE_IDCODE: {                // 識別コード(D-roomカード)
                switch (blk->request.request_type) {
                case MDLSTRG_REQ_TYPE_READ:                 // 識別コード管理データ作成
                    opt1 = create_id_code_mng((MDLSTRG_DATA_IDENTIFY_CODE_MNG_T*)blk->request.data);
                    break;
                case MDLSTRG_REQ_TYPE_WRITE:                // 識別コード追加
                    opt1 = append_id_code((const MDLSTRG_DATA_IDCODE_T*)blk->request.data);
                    break;
                case MDLSTRG_REQ_TYPE_ALL_DELETE:           // 識別コード削除（セクター単位）
                    delete_id_code();
                	break;
                case MDLSTRG_REQ_TYPE_CREATE:               // 識別コード初期化
                    opt1 = init_id_code();
                    break;
                default:
                    assert(false);
                    break;
                }
                break;
            }
            case MDLSTRG_DATA_TYPE_VACANCY_REG_CARD: {      // 一括登録カード
                switch (blk->request.request_type) {
                case MDLSTRG_REQ_TYPE_EXISTS:               // データ存在
                    opt1 = exists_vacancy_reg_card();
                    break;
                case MDLSTRG_REQ_TYPE_WRITE: {              // 書込み
                    opt1 = write_vacancy_reg_card((const MDLSTRG_DATA_CARD_T*)blk->request.data);
                    break;
                }    
                case MDLSTRG_REQ_TYPE_READ: {              // 読込み
                    opt1 = read_vacancy_reg_card((MDLSTRG_DATA_CARD_T*)blk->request.data);
                    break;
                }    
                default:
                    assert(false);
                    break;
                }
                break;
            }
            case MDLSTRG_DATA_TYPE_REG_CARD: {              // 登録・抹消カード
                switch (blk->request.request_type) {
                case MDLSTRG_REQ_TYPE_EXISTS:               // データ存在
                    opt1 = exists_reg_card();
                    break;
                case MDLSTRG_REQ_TYPE_WRITE: {              // 書き込み
                    opt1 = write_reg_card((const MDLSTRG_DATA_CARD_T*)blk->request.data);
                    break;
                }
                case MDLSTRG_REQ_TYPE_READ: {               // 読込み
                    opt1 = read_reg_card((MDLSTRG_DATA_CARD_T*)blk->request.data);
                    break;
                }
                default:
                    assert(false);
                    break;
                }
                break;
            }
            
            case MDLSTRG_DATA_TYPE_WIFI: {                  // WIFI接続情報
                switch (blk->request.request_type) {
                case MDLSTRG_REQ_TYPE_EXISTS: {             // データ存在
                    opt1 = exists_wifi_param();
                    break;
                }
                case MDLSTRG_REQ_TYPE_WRITE: {              // 書き込み
                    opt1 = write_wifi_param((const MDLSTRG_DATA_WIFI_PARAM_T*)blk->request.data);
                    break;
                }
                case MDLSTRG_REQ_TYPE_READ: {               // 読込み
                    opt1 = read_wifi_param((MDLSTRG_DATA_WIFI_PARAM_T*)blk->request.data);
                    break;
                }
                default:
                    assert(false);
                    break;
                }
                break;
            }

            case MDLSTRG_DATA_TYPE_OPE_LOG: {              // 操作ログ
                switch (blk->request.request_type) {
                case MDLSTRG_REQ_TYPE_WRITE:               // 書き込み
                    opt1 = write_ope_log((const MDLSTRG_DATA_CARD_T*)blk->request.data, blk->request.opt1);
                    break;
                case MDLSTRG_REQ_TYPE_READ: {              // 読込み
                    opt1 = read_ope_log((MDLSTRG_DATA_CARD_T*)blk->request.data);
                    break;
                }
                case MDLSTRG_REQ_TYPE_INCREMENT: {         // 読込みカウンタ更新
                    increment_ope_log();
                    break;
                }
                case MDLSTRG_REQ_TYPE_CNT_READ: {          // カウンタ読込
                    opt1 = read_ope_log_cnt((MDLSTRG_DATA_CARD_T*)blk->request.data);
                    break;
                }

                default:
                    assert(false);
                    break;
                }
                break;
            }

            case MDLSTRG_DATA_TYPE_ERR_LOG: {              // エラーログ
                switch (blk->request.request_type) {
                case MDLSTRG_REQ_TYPE_WRITE:               // 書き込み
                    opt1 = write_err_log((const MDLSTRG_DATA_CARD_T*)blk->request.data);
                    break;
                case MDLSTRG_REQ_TYPE_READ: {              // 読込み
                    opt1 = read_err_log((MDLSTRG_DATA_CARD_T*)blk->request.data);
                    break;
                }
                case MDLSTRG_REQ_TYPE_INCREMENT: {         // 読込みカウンタ更新
                    increment_err_log();
                    break;
                }
                case MDLSTRG_REQ_TYPE_CNT_READ: {          // カウンタ読込
                    opt1 = read_err_log_cnt((MDLSTRG_DATA_CARD_T*)blk->request.data);
                    break;
                }

                default:
                    assert(false);
                    break;
                }
                break;
            }

            case MDLSTRG_DATA_TYPE_ROOM_TYPE: {
                switch (blk->request.request_type) {
                case MDLSTRG_REQ_TYPE_READ:
//                    DBGLOG0("CASE_ MDLSTRG_REQ_TYPE_READ");
                    read_room_sts((MDLSTRG_DATA_ROOM_STS_T*)blk->request.data);
                    break;
                case MDLSTRG_REQ_TYPE_WRITE:
                    write_room_sts((MDLSTRG_DATA_ROOM_STS_T*)blk->request.data);
                    break;
                default:
                    assert(false);
                    break;
                }
                break;
            }
            ////////////////////////////////
            case MDLSTRG_DATA_TYPE_STORE_PROGRAM: {	// Store program
                DBGLOG0("MDLSTRG_DATA_TYPE_STORE_PROGRAM.");
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

    DBGLOG1("db_get_data %d", area);
    
//    DBGLOG0("db_get_data_addr");
    
    // 領域情報取得
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);

    bool_t found = false;
    switch (area_def->strategy) {
//    DBGLOG1("strategy_data %d", area_def->strategy);
    case STRATEGY_NDATA_1LEB_INDEXED:
    {
        found = c_db_get_data_addr_1d1li(a_addr, a_size, cursor, area_def, data_id, SEARCH_FOR_VALID_DATA);
        if (!found) {
            DBGLOG0("Not_fnd_db_get_data_addr_1d1li");
        }
        
        break;
    }
    case STRATEGY_1DATA_1LEB_INDEXED:
    {
        found = db_get_data_addr_1d1li(a_addr, a_size, cursor, area_def, data_id, SEARCH_FOR_VALID_DATA);
        if (!found) {
            DBGLOG0("Not_fnd_db_get_data_addr_1d1li");
        }
        
        break;
    }
    case STRATEGY_INCREMENTAL_FIXED:
    {
        found = db_get_data_addr_incf(a_addr, a_size, cursor, area_def, data_id, SEARCH_FOR_VALID_DATA);
        if (!found) {
            DBGLOG0("Not_fnd_STRATEGY_INCREMENTAL_FIXED");
        }
        
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

    DBGLOG0("db_new_data");
    
    // 領域情報取得
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);

    bool_t found = false;
    switch (area_def->strategy) {
    case STRATEGY_NDATA_1LEB_INDEXED:
    {
        found = c_db_get_data_addr_1d1li(a_addr, a_size, cursor, area_def, data_id, SEARCH_FOR_WRITE_DEST);
        break;
    }
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

    DBGLOG0("write_blk_erase");
    // ブロック消去
    switch (area_def->strategy) {
    case STRATEGY_NDATA_1LEB_INDEXED:
    {
        // LEB消去
        leb_erase(cursor->area_def, cursor->leb_index);
        break;
    }
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

    DBGLOG0("db_set_valid");
    DB_HEADER_T header = {0};
    DBGLOG2("db_addr：0x%x, data_len：%d ", cursor->db_addr, data_len);
    DBGLOG1("DB_HEADER_T：%d ", sizeof(DB_HEADER_T));
    flash_read(&header, cursor->db_addr, sizeof(DB_HEADER_T));
//    assert(header.db_state == BLOCK_STATE_ERASED);

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

//    assert(data_id < area_def->num_lebs);

    // LEBアドレス(内容は見ない)
    leb_addr = area_def->addr + (area_def->leb_size * data_id);
    DBGLOG1("leb_addr：0x%x", leb_addr);
    
    // DBヘッダの読み出し
    db_addr = leb_addr + sizeof(LEB_HEADER_T);
    DBGLOG1("db_addr：0x%x", db_addr);
    
    flash_read(&db_header, db_addr, sizeof(DB_HEADER_T));

    // データが有効かをチェック
    if (search_for == SEARCH_FOR_VALID_DATA) {
        if (db_header.db_state == BLOCK_STATE_VALID) {
            DBGLOG0("BLOCK_STATE_VALID");
            found = true;
        }
    } else if (search_for == SEARCH_FOR_WRITE_DEST) {
        DBGLOG0("SEARCH_FOR_WRITE_DEST");
        found = true;
    } else {
        DBGLOG0("NOT_SEARCH");
        assert(false);
    }
    DBGLOG1("found:%d", found);
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

bool_t c_db_get_data_addr_1d1li(intptr_t* a_addr, size_t* a_size, DB_CURSOR_T* cursor, const FLASH_AREA_DEF_T* area_def, uint32_t data_id, int search_for)
{
    assert(a_addr);
    assert(a_size);
    assert(cursor);
    assert(area_def);

    intptr_t leb_addr = 0;
    intptr_t db_addr = 0;
    DB_HEADER_T db_header = {0};
    bool_t found = false;
    intptr_t addr_pos;
//    assert(data_id < area_def->num_lebs);

    // 対象のDroomカードのアドレス（先頭に登録抹消カードがあるためその分シフト）
//    leb_addr = area_def->addr + (area_def->leb_size * data_id);

//    DBGLOG1("data_id : %d", data_id);
//    DBGLOG1("leb_addr : 0x%x", leb_addr);
//    DBGLOG2("data_id :%d, area_def->num_lebs：%d", data_id, area_def->num_lebs);
    
    // DBヘッダの読み出し
    addr_pos = sizeof(MDLSTRG_DATA_CHASH_META_T) * data_id;
    DBGLOG1("addr_pos : %d", addr_pos);
    leb_addr = area_def->addr + addr_pos;
    
    db_addr = leb_addr + sizeof(LEB_HEADER_T) + addr_pos;

    DBGLOG2("db_addr：0x%x, LEB_HEADER_T : %d", db_addr, sizeof(LEB_HEADER_T));
    flash_read(&db_header, db_addr, sizeof(DB_HEADER_T));
/*
    for(int i = 0; i <= data_id; i++){
        cmndbg_hexdump(&db_header, sizeof(db_header), "db_header");
    }
*/
    // データが有効かをチェック
    if (search_for == SEARCH_FOR_VALID_DATA) {
        if (db_header.db_state == BLOCK_STATE_VALID) {
            DBGLOG0("BLOCK_STATE_VALID");
            found = true;
        }
    } else if (search_for == SEARCH_FOR_WRITE_DEST) {
        DBGLOG0("SEARCH_FOR_WRITE_DEST");
        found = true;
    } else {
        DBGLOG0("NOT_SEARCH");
        assert(false);
    }
    DBGLOG1("found:%d", found);
    if (found) {
        *a_addr = db_addr + sizeof(DB_HEADER_T) + sizeof(DB_HEADER_T);//データアドレス + 12byte
        *a_size = db_header.data_len;
        cursor->area_def = area_def;
//        cursor->leb_index = data_id;
        cursor->leb_index = 0;  //書き込みセクタ　1セクタのみ
        cursor->db_addr = db_addr;
        cursor->db_state = db_header.db_state;
    }
    DBGLOG1("*a_addr:%x", *a_addr);
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
 * 認証データ読み出し
 */
bool_t read_card_meta(MDLSTRG_DATA_CHASH_META_T* dest, int index)
{
    DBGLOG1("index_read: 0x%x", index);
    assert(dest);
    assert(index >= 0 && index < MDLSTRG_MAX_NDATA_CARD);

    bool_t found = false;
    intptr_t addr = 0;
    size_t size = 0;
    DB_CURSOR_T cursor = {0};

    // 登録データ
    found = db_get_data_addr(&addr, &size, &cursor, FLASH_AREA_CARD_META, index);
    if (!found) {
        goto end;	// 見付からない場合
    }

    // データ取得
    assert(size == sizeof(MDLSTRG_DATA_CHASH_META_T));
    flash_read(dest, addr, sizeof(MDLSTRG_DATA_CHASH_META_T));

end:
    return found;
}


/*
 * カードデータ書込み
 */
void write_card_meta(const MDLSTRG_DATA_CHASH_META_T* data, int index)
{
    DBGLOG1("index_write: 0x%x", index);
    DBGLOG0("write_card_meta");
    assert(data);
    assert(index >= 0 && index < MDLSTRG_MAX_NDATA_CARD);

    bool_t found = false;
    intptr_t addr = 0;
    size_t size = 0;
    DB_CURSOR_T cursor[MDLSTRG_MAX_NDATA_CARD] = {0};

    // 書き込み先を取得
    found = db_new_data(&addr, &size, cursor, FLASH_AREA_CARD_META, index);
    assert(found);
    DBGLOG1("a_addr 0x%x", addr);
    // data
    flash_write(addr, data, sizeof(MDLSTRG_DATA_CHASH_META_T));

    // ヘッダを更新
    db_set_valid(&cursor, index, sizeof(MDLSTRG_DATA_CHASH_META_T));

//    cmndbg_hexdump(&cursor, sizeof(cursor), "db_set_valid");
    
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

    DBGLOG0("read_device_id ");
    
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

/*
 * カードの存在確認
 */
bool_t exists_card(intptr_t addr) {
    bool_t found = false;
    uint8_t status_flg[8];

    flash_read(&status_flg, addr, 8);

//    cmndbg_hexdump(status_flg, sizeof(uint8_t) * 8, "status_flg");

    for (int i = 0; i < 8; i++) {
        if (status_flg[i] != CARD_STATUS_NO_DATA) {
            found = true;
            break;
        }
    }
    return found;
}

/*
 * 識別コードのフラグ比較
 */
bool_t compare_id_code_flg(intptr_t addr, uint16_t flg) {
    bool_t found = false;
    uint16_t status_flg;

    flash_read(&status_flg, addr, sizeof(uint16_t));
    if (status_flg == flg) {
        found = true;
    }
    return found;
}

/*
 * 識別コードのINDEX比較
 */
bool_t compare_id_code_index(intptr_t addr, uint16_t index) {
    bool_t found = false;
    uint16_t id_code_index;
	intptr_t src = addr + sizeof(uint16_t);

    flash_read(&id_code_index, src, sizeof(uint16_t));

    if (id_code_index == index) {
        found = true;
    }
    return found;
}

/*
 * RAMとFlashのベリファイ
 * 書き込み検証用
 */
ER verify(intptr_t addr_ram, intptr_t addr_flash, int size) {
    bool_t res = E_OK;
    int8_t* flash_data;

    flash_data = (int8_t *)malloc(size * sizeof(int8_t));       // flash内の情報を読み出すバッファを確保
    flash_read(flash_data, addr_flash, size);
    if (memcmp(flash_data, addr_ram, size) != 0) {
        res = E_SYS;
    }
    free(flash_data);                                           // 確保したバッファを解放
    return res;
}

/*
 * 一部データ更新（RAMへデータ退避後に更新）
 * addr_flash_area : セクター先頭アドレス
 * addr_flash_pos  : セクター先頭から書き換え範囲の先頭アドレス
 * leb_size        : セクターサイズ（消去サイズ）
 * addr_ram        : 書き換えデータの先頭アドレス
 * size            : 書き換えデータのサイズ
 */
bool_t update_flash(intptr_t addr_flash_area, intptr_t addr_flash_pos, size_t leb_size, intptr_t addr_ram, size_t size) {
    bool_t res = E_SYS;

    DBGLOG1("addr_flash_pos : 0x%08x", addr_flash_pos);
    DBGLOG1("update size : %d", size);

    // Flash上のデータを退避
    flash_read(s_update_buf, addr_flash_area, FLASH_UPDATE_BUF_SIZE);

    // 退避データを書き換え
    memcpy(s_update_buf + addr_flash_pos, addr_ram, size);

    // 対象セクターを消去
    flash_erase(addr_flash_area, leb_size);

    // 退避データを書き込み
    flash_write(addr_flash_area, s_update_buf, FLASH_UPDATE_BUF_SIZE);
    
    return verify(s_update_buf, addr_flash_area, FLASH_UPDATE_BUF_SIZE);
}

/*
 * ログカウンタ更新
 */
void update_log_cnt(intptr_t addr) {
	unsigned long long log_flg[2] = {0};

    flash_read(log_flg, addr, sizeof(unsigned long long)*2);
//    cmndbg_hexdump(log_flg, sizeof(log_flg), "update_log_cnt() read");

    //右に1bitシフトして、上位に0を詰める
	if(log_flg[0] > 0){
		log_flg[0] = log_flg[0] >> 1;
	}
	else if(log_flg[1] > 0){
		log_flg[1] = log_flg[1] >> 1;
	}else{
		//すべて0の場合、全bit反転
        log_flg[0] = ~log_flg[0];
		log_flg[1] = ~log_flg[1];
	}

    flash_write(addr, &log_flg, sizeof(log_flg));
//    cmndbg_hexdump(log_flg, sizeof(log_flg), "update_log_cnt() write");

    return;
}

/*
 * 現在のログセクターを探す
 */
int search_current_sector(intptr_t addr) {
    int res = 0;
	unsigned long long log_cnt[2] = {0};
    intptr_t search_addr = addr;

    //セクターの先頭アドレスが空か確認
    DBGLOG0("search_current_sector");
    //DBGLOG1("search_current_sector [search_addr][addr] : %p", addr);
	for(res = 0; res < MAX_NUM_LOG_SECTOR; res++){
		flash_read(log_cnt, search_addr, sizeof(unsigned long long)*2);
//		cmndbg_hexdump(log_cnt, sizeof(unsigned long long)*2, "search_current_sector head address of sector");
		if(log_cnt[0] == LOG_CNT_NO_DATA && log_cnt[1] == LOG_CNT_NO_DATA){
            DBGLOG1("search_current_sector res : %d", res);
			break;
		}
		search_addr = search_addr + LOG_SECTOR_SIZE;
        //DBGLOG1("search_current_sector [update search_addr] : %p", search_addr);
	}

	// 初期化されているセクターが見つからなかった場合はエラー
	if(res >= MAX_NUM_LOG_SECTOR){
		res = -1;
        DBGLOG1("search_current_sector res fail : %d", res);
	}
	else if(res != 0){
		//現在のセクターは、空のセクターの一つ手前のためデクリメント
        res--;
        DBGLOG1("search_current_sector res decrement : %d", res);
	}else{
        //セクター1が空の場合、セクター2も空ならセクター1からログを格納する
        flash_read(log_cnt, addr + LOG_SECTOR_SIZE, sizeof(unsigned long long)*2);
        cmndbg_hexdump(log_cnt, sizeof(unsigned long long)*2, "search_current_sector sector2");
        //DBGLOG1("search_current_sector [search_current_sector sector2 addr] : %p", search_addr);

        if(log_cnt[0] == LOG_CNT_NO_DATA && log_cnt[1] == LOG_CNT_NO_DATA){
		    res = 0;
            DBGLOG0("search_current_sector sector2 is empty");
		}else{
            //セクター2が空でない場合、セクター1の一つ手前のセクター16からログを格納する
            res = 15;
            DBGLOG0("search_current_sector start from sector16");
        }
    }

    return res;
}

/*
 * ログ書き込み回数取得
 */
int check_write_cnt(intptr_t addr) {
	unsigned long long write_cnt = 0;
    unsigned long long check_cnt[2] = {0};
    unsigned long long comp_bit = 0x8000000000000000;
    int bit_cnt = 0;

    flash_read(check_cnt, addr, sizeof(unsigned long long)*2);
//    cmndbg_hexdump(&check_cnt, sizeof(unsigned long long)*2, "check_write_cnt() read");

    
    if(check_cnt[0] == LOG_CNT_NO_DATA){
        return 0;
    }else if(check_cnt[0] != LOG_CNT_NO_DATA && check_cnt[0] != 0){
        write_cnt = check_cnt[0];
    }else if(check_cnt[0] == 0 && check_cnt[1] != 0){
        write_cnt = check_cnt[1];
        bit_cnt = sizeof(unsigned long long)*8;
    }else{
        DBGLOG0("check_write_cnt cnt = 0");
        return 128;
    }
    while((write_cnt & comp_bit) == 0){
        //0x8000000000000000と＆し、何桁目で1になるか
        comp_bit = comp_bit >> 1;
        bit_cnt++;
    }
    DBGLOG1("check_write_cnt bit_cnt : %d", bit_cnt);
    return bit_cnt;
}

/*
 * ログ読込カウンタ更新(操作/エラー共通)
 */
void increment_read_log_cnt(int flash_area){
	int current_sector = 0;
    intptr_t	addr;
    // 領域情報取得
    int area = flash_area;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);

    // どのセクターを更新するか判断
	current_sector = search_current_sector(area_def->addr);
    addr = area_def->addr + area_def->leb_size *current_sector + sizeof(unsigned long long)*2;
    update_log_cnt(addr);

    return;
}

/*
 * ログR/Wカウンタ取得(操作/エラー共通)
 */
bool_t get_read_write_cnt(MDLSTRG_DATA_RW_COUNTER_T* data, int flash_area){
	// 領域情報取得
    int area = flash_area;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);
    
    int current_sector = 0;
    intptr_t current_sector_addr = 0;

    // 使用中のセクターを取得
	current_sector = search_current_sector(area_def->addr);
	// 初期化されているセクターが見つからなかった場合はエラー
	if(current_sector == -1){
        DBGLOG0("blank sector not found");
		return false;
	}
    current_sector_addr = area_def->addr + area_def->leb_size * current_sector;

    flash_read(data, current_sector_addr, sizeof(MDLSTRG_DATA_RW_COUNTER_T));
    
    if(data->bitsw_rd[0] == LOG_CNT_NO_DATA && data->bitsw_rd[1] == LOG_CNT_NO_DATA
    && data->bitsw_wt[0] == LOG_CNT_NO_DATA && data->bitsw_wt[1] == LOG_CNT_NO_DATA){
        DBGLOG0("get_read_write_cnt() Read,Write Counter no data.");
        return false;
    }
    return true;
}

/*
 * 最新ログ取得(操作/エラー共通)
 */
bool_t read_latest_log(void* data, int flash_area){
	// 領域情報取得
    int area = flash_area;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);

    int current_sector = 0;
    int write_cnt = 0;
    int log_size = 0;
    intptr_t current_sector_addr = 0;
    intptr_t latest_log_addr = 0;

    if(flash_area == FLASH_AREA_OPE_LOG){
        log_size = sizeof(MDLSTRG_DATA_OPE_LOG_T);
    }else if (flash_area == FLASH_AREA_ERR_LOG){
        log_size = sizeof(MDLSTRG_DATA_ERR_LOG_T);
    }

    // 使用中のセクターを取得
	current_sector = search_current_sector(area_def->addr);

	// 初期化されているセクターが見つからなかった場合はエラー
	if(current_sector == -1){
        DBGLOG0("blank sector not found");
		return false;
	}
    current_sector_addr = area_def->addr + area_def->leb_size * current_sector;

    // 書き込みカウンタ確認
    write_cnt = check_write_cnt(current_sector_addr);
	if(write_cnt == 0){
        write_cnt = 1;
	}
    latest_log_addr = current_sector_addr + sizeof(MDLSTRG_DATA_RW_COUNTER_T) + log_size * (write_cnt-1);

    flash_read(data, latest_log_addr, log_size);
//    cmndbg_hexdump(data, log_size, "read log");

    return true;
}

/*
 * ログカウンタのベリファイ
 * 書き込み検証用
 */
ER verify_log_cnt(unsigned long long before[2], intptr_t addr_flash, int size) {
    ER res = E_OK;
    int8_t* flash_data;

    flash_data = (int8_t *)malloc(size * sizeof(int8_t));       // flash内の情報を読み出すバッファを確保
    flash_read(flash_data, addr_flash, size);
    //右に1bitシフトして、上位に0を詰める
	if(before[0] > 0){
		before[0] = before[0] >> 1;
	}else if(before[1] > 0){
		before[1] = before[1] >> 1;
	}else{
		//すべて0の場合、全bit反転
        before[0] = ~before[0];
		before[1] = ~before[1];
	}
    if (memcmp(flash_data, before, size) != 0) {
        res = E_SYS;
//        cmndbg_hexdump(before, size,     "before");
//        cmndbg_hexdump(flash_data, size, "after  (flash_data)");
    }
    free(flash_data);                                           // 確保したバッファを解放
    return res;
}

/*
 * 空室カード存在確認（不要？）
 */
bool_t exists_vacancy_card(int index) {
    // 領域情報取得
    int area = FLASH_AREA_VACANCY_CARD;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);
    assert(index < MDLSTRG_MAX_VACANCY_CARD);

    // インデックス移動（先頭１つ目は空室一括登録カード）
    intptr_t addr = area_def->addr + (sizeof(MDLSTRG_DATA_CARD_T) * (index + 1));

    DBGLOG1("addr : 0x%08x", addr);

    // カードの存在確認
    return exists_card(addr);
}

/*
 * 空室カード読み出し
 */
void read_vacancy_card(MDLSTRG_DATA_CARD_T* data_array) {
    // 領域情報取得
    int area = FLASH_AREA_VACANCY_CARD;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);

    flash_read(data_array, area_def->addr + sizeof(MDLSTRG_DATA_CARD_T), sizeof(MDLSTRG_DATA_CARD_T) * MDLSTRG_MAX_VACANCY_CARD);
}

/*
 * 空室カード追加
 */
bool_t append_vacancy_card(const MDLSTRG_DATA_CARD_T* data) {
    bool_t res = E_SYS;
    bool_t found = false;
    intptr_t addr;

    // 領域情報取得
    int area = FLASH_AREA_VACANCY_CARD;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);

    // 空いているインデックスを探す
    for (int i = 0; i < MDLSTRG_MAX_VACANCY_CARD; i++) {

        // インデックス移動（先頭１つ目は空室一括登録カード）
        addr = area_def->addr + sizeof(MDLSTRG_DATA_CARD_T) * (i + 1);

        DBGLOG1("addr : 0x%08x", addr);
        found = exists_card(addr);

        if (found == false) {

            flash_write(addr, data, sizeof(MDLSTRG_DATA_CARD_T));

            res = verify(data, addr, sizeof(MDLSTRG_DATA_CARD_T));
            break;
        }
    }

    return res;
}


/*
 * 空室カード削除
 */
void delete_vacancy_card() {
    // 領域情報取得
    int area = FLASH_AREA_VACANCY_CARD;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);

    //flash_erase(area_def->addr, sizeof(MDLSTRG_DATA_CARD_T) * (MAX_NUM_VACANCY_CARD + 1));
    flash_erase(area_def->addr, area_def->leb_size);
}

/*
 * D-roomカード読込
 */
void read_droom_card(MDLSTRG_DATA_CARD_T* data_array) {
    // 領域情報取得
    int area = FLASH_AREA_DROOM_CARD;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);

    flash_read(data_array, area_def->addr + sizeof(MDLSTRG_DATA_CARD_T), sizeof(MDLSTRG_DATA_DROOM_CARD_T) * MDLSTRG_MAX_DROOM_CARD);
}

/*
 * D-roomカード書き込み（更新）
 */
bool_t write_droom_card(const MDLSTRG_DATA_DROOM_CARD_T* data, int index) {
    bool_t res = E_SYS;
    bool_t found = false;
    intptr_t addr;
    intptr_t addr_pos;

    // 領域情報取得
    int area = FLASH_AREA_DROOM_CARD;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);
    assert(index < MDLSTRG_MAX_DROOM_CARD);

    // 対象のDroomカードのアドレス（先頭に登録抹消カードがあるためその分シフト）
    addr_pos = sizeof(MDLSTRG_DATA_CARD_T) + sizeof(MDLSTRG_DATA_DROOM_CARD_T) * index;
    addr = area_def->addr + addr_pos;
    DBGLOG1("index : %d", index);
    DBGLOG1("addr : 0x%08x", addr);
    found = exists_card(addr);

    if (found == true) {
        // すでにデータが存在しているため、特定の部分のみ更新
        DBGLOG1("UPDATE Droom Card : (%d)", index);
        res = update_flash(area_def->addr, addr_pos, area_def->leb_size, data, sizeof(MDLSTRG_DATA_DROOM_CARD_T));
    } else {
        // 空のデータのため、そのまま書き込み
        DBGLOG1("Write Droom Card : (%d)", index);
        flash_write(addr, data, sizeof(MDLSTRG_DATA_DROOM_CARD_T));
        res = verify(data, addr, sizeof(MDLSTRG_DATA_DROOM_CARD_T));
    }

    return res;
}

/*
 * D-roomカード削除（カード単位）
 */
bool_t delete_droom_card(int index) {
    // 領域情報取得
    int area = FLASH_AREA_DROOM_CARD;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);

    // 削除後のデータを作成(All 0xFF)
    uint8_t temp[sizeof(MDLSTRG_DATA_DROOM_CARD_T)];
    memset(temp, 0xFF, sizeof(MDLSTRG_DATA_DROOM_CARD_T));

    return update_flash(area_def->addr, sizeof(MDLSTRG_DATA_CARD_T) + sizeof(MDLSTRG_DATA_DROOM_CARD_T) * index ,area_def->leb_size, temp, sizeof(MDLSTRG_DATA_DROOM_CARD_T));
}

/*
 * D-roomカード削除（セクター単位）
 * 登録・抹消カードも削除
 */
void all_delete_droom_card() {
    // 領域情報取得
    int area = FLASH_AREA_DROOM_CARD;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);

    flash_erase(area_def->addr, area_def->leb_size);
}

/*
 * 識別コード管理データ作成
 */
static bool_t create_id_code_mng(MDLSTRG_DATA_IDENTIFY_CODE_MNG_T* data_array) {
    bool_t found = false;
    intptr_t addr;

    // 領域情報取得
    int area = FLASH_AREA_IDCODE;
    int index_count = 0;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);
    addr = area_def->addr;

    //識別コードが格納されているセクターを判定
    if (compare_id_code_flg(addr, ID_CODE_NO_DATA) == true)
    {
        addr = addr + area_def->leb_size;
    }

    // 最新の識別コードをINDEX10個分探す
    for (int i = 0; i < MDLSTRG_MAX_ID_CODE; i++) {     // セクター内を探索
        found = compare_id_code_flg(addr, LATEST_ID_CODE);
        if (found == true)
        {
            // 最新のデータを取得
            DBGLOG2("Found latest id code : 0x%08x (index = %d)", addr, i);
            flash_read(&(data_array[index_count]), addr, sizeof(MDLSTRG_DATA_IDCODE_T));
            data_array[index_count].address = addr;
            index_count++;
        }
        //10個分見つかったら抜ける
        if(index_count >= DEFAULT_NUM_ID_CODE){
            break;
        }
        // インデックス移動
        addr = addr + sizeof(MDLSTRG_DATA_IDCODE_T);
    }
    // 10個見つからなかったらエラー
    if (index_count < DEFAULT_NUM_ID_CODE) {
        found = false;
    }

    return found;
}

/*
 * 識別コード追加
 */
ER append_id_code(const MDLSTRG_DATA_IDCODE_T* data) {
    ER res = E_SYS;
    bool_t found = false;
    intptr_t addr = 0;
    intptr_t verify_addr = 0;
    intptr_t empty_sector_addr = 0;
    intptr_t used_sector_addr = 0;
    intptr_t write_address = 0;
    intptr_t found_addr = 0;
    intptr_t old_flg_addr = 0;

    // 領域情報取得
    int area = FLASH_AREA_IDCODE;
    const uint16_t old_flg = OLD_ID_CODE;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);
    addr = area_def->addr;

    //識別コードが格納されているセクターを判定
    if (compare_id_code_flg(addr, ID_CODE_NO_DATA) == true)
    {
        //DBGLOG0("append_id_code() sector 2 used");
        used_sector_addr  = addr + area_def->leb_size;
        empty_sector_addr = addr;
    }else{
        //DBGLOG0("append_id_code() sector 1 used");
        used_sector_addr  = addr;
        empty_sector_addr = addr + area_def->leb_size;
    }

    // 追記のために空いている領域を探す
    for (int i = 0; i < MDLSTRG_MAX_ID_CODE; i++) {
        // インデックス移動
        addr = used_sector_addr + sizeof(MDLSTRG_DATA_IDCODE_T) * i;
        found = compare_id_code_flg(addr, (uint16_t)ID_CODE_NO_DATA);

        //フラグが最新か確認
        if(compare_id_code_flg(addr, (uint16_t)LATEST_ID_CODE)){
            //インデックスが今回認証されたカードと同じか確認
            if(compare_id_code_index(addr, data->idx)){
                old_flg_addr = addr;
            }
        }

        if (found == true) {
            //DBGLOG0("append_id_code() found");
            found_addr = addr;
            break;
        }
    }

    //空いているインデックスがあった場合
    if (found == true) {
    	
        if (old_flg_addr != 0) {
            //フラグが最新かつ、インデックスが今回認証されたカードと同じレコードは、
            //フラグを古いものに書き換える
            flash_write(old_flg_addr, &old_flg, sizeof(uint16_t));
        }

        //新しいレコードを追記する
#if 0
        DBGLOG0("NEW Data");
        DBGLOG1("FLG 0x%04x", data->flg);
        DBGLOG1("IDX 0x%04x", data->idx);
        DBGLOG1("BLK 0x%08x", data->block);
        //DBGLOG1("CODE 0x%08x", test_id_code_mng[i].id_code.code);
        cmndbg_hexdump(data->code, sizeof(uint8_t)*8, "ID Code");
#endif
        flash_write(found_addr, data, sizeof(MDLSTRG_DATA_IDCODE_T));
        res = (bool_t)verify((intptr_t)data, found_addr, sizeof(MDLSTRG_DATA_IDCODE_T));

    //空いているインデックスがなかった場合、セクターを変更する
    } else {
        //DBGLOG0("append_id_code() not found");
 
        // 管理データ作成
        create_id_code_mng(id_code_mng);

        // 空のセクターに移動
        addr = empty_sector_addr;

        // 最新の識別コードを管理データからコピーする
        for (int i = 0; i < DEFAULT_NUM_ID_CODE; i++) {
            //今回認証されたデータとINDEXが同じ場合には、管理データではなく今回の認証データを格納する
            if(data->idx == id_code_mng[i].id_code.idx){
                flash_write(addr, data, sizeof(MDLSTRG_DATA_IDCODE_T));
                verify_addr = addr;
            }else{
                flash_write(addr, &id_code_mng[i], sizeof(MDLSTRG_DATA_IDCODE_T));
            }
            addr = addr + sizeof(MDLSTRG_DATA_IDCODE_T);
       }
        res = (bool_t)verify((intptr_t)data, verify_addr, sizeof(MDLSTRG_DATA_IDCODE_T));

        //元のセクターを初期状態にする
        flash_erase(used_sector_addr, area_def->leb_size);
   }

    return res;

}

/*
 * 識別コード削除
 */
void delete_id_code() {
    // 領域情報取得
    int area = FLASH_AREA_IDCODE;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);

    //2つのセクターを削除
    flash_erase(area_def->addr, area_def->leb_size);
    flash_erase(area_def->addr + area_def->leb_size, area_def->leb_size);
}

/*
 * 識別コード初期化
 */
bool_t init_id_code() {
    // 領域情報取得
    bool_t res = E_OK;
    int area = FLASH_AREA_IDCODE;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    uint8_t  id_code[8] = {0};
    int      index = 0;
    MDLSTRG_DATA_IDCODE_T default_id_code;
    default_id_code.flg = LATEST_ID_CODE;
    default_id_code.idx = index;
    default_id_code.block = 0x0000;
    memcpy(default_id_code.code, id_code, 8);
    intptr_t addr;
    assert(area_def->area == area);

    delete_id_code();
    addr = area_def->addr;

    //1セクター目の先頭から、INDEX10個分の初期識別コードを書き込む
    for(index = 0; index < DEFAULT_NUM_ID_CODE; index++)
    {
        // Flashに書き込むindexは1から10
        default_id_code.idx = index + 1;
        flash_write(addr, &default_id_code, sizeof(MDLSTRG_DATA_IDCODE_T));

        if (verify(&default_id_code, addr, sizeof(MDLSTRG_DATA_IDCODE_T)) != E_OK) {
            res = E_SYS;
            assert(false);
        }

        addr = addr + sizeof(MDLSTRG_DATA_IDCODE_T);
    }

    return res;
}

/*
 * 登録・抹消カード存在確認
 */
bool_t exists_reg_card() {
    bool_t found = false;

    // 領域情報取得
    int area = FLASH_AREA_DROOM_CARD;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);

    // カードの存在確認
    return exists_card(area_def->addr);
}

/*
 * 登録・抹消カード読み出し
 */
bool_t read_reg_card(MDLSTRG_DATA_CARD_T* data) {
    bool_t found = false;

    // 存在確認
    found = exists_reg_card();
    if (found == true) {
        // 領域情報取得
        int area = FLASH_AREA_DROOM_CARD;
        const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
        assert(area_def->area == area);

        flash_read(data, area_def->addr, sizeof(MDLSTRG_DATA_CARD_T));
    }
    return found;
}

/*
 * 登録・抹消カード書き込み
 */
bool_t write_reg_card(const MDLSTRG_DATA_CARD_T* data) {
    // 領域情報取得
    int area = FLASH_AREA_DROOM_CARD;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);

    DBGLOG2("addr = 0x%08%x, size = %d", area_def->addr, sizeof(MDLSTRG_DATA_CARD_T));
    flash_write(area_def->addr, data, sizeof(MDLSTRG_DATA_CARD_T));

    return verify(data, area_def->addr, sizeof(MDLSTRG_DATA_CARD_T));
}

/*
 * 空室一括登録カード存在確認
 */
bool_t exists_vacancy_reg_card() {
    bool_t found = false;
    uint8_t status_flg[8];

    // 領域情報取得
    int area = FLASH_AREA_VACANCY_CARD;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);

    // カードの存在確認
    return exists_card(area_def->addr);

/*
    flash_read(&status_flg, area_def->addr, 8);

    for (int i = 0; i < 8; i++) {
        if (status_flg[i] != CARD_STATUS_NO_DATA) {
            found = true;
            break;
        }
    }
    return found;
*/
}

/*
 * 空室一括登録カード読込み
 */
bool_t read_vacancy_reg_card(MDLSTRG_DATA_CARD_T* data) {
    bool_t found = false;

    // 存在確認
    found = exists_vacancy_reg_card();
    if (found == true) {
        // 領域情報取得
        int area = FLASH_AREA_VACANCY_CARD;
        const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
        assert(area_def->area == area);

        flash_read(data, area_def->addr, sizeof(MDLSTRG_DATA_CARD_T));
    }
    return found;
}

/*
 * 空室一括登録カード書き込み
 */
bool_t write_vacancy_reg_card(const MDLSTRG_DATA_CARD_T* data) {
    // 領域情報取得
    int area = FLASH_AREA_VACANCY_CARD;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);

    DBGLOG2("addr = 0x08%d, size = %d", area_def->addr, sizeof(MDLSTRG_DATA_CARD_T));
    flash_write(area_def->addr, data, sizeof(MDLSTRG_DATA_CARD_T));

    // ベリファイエラー動作確認
    //*((int8_t*)data+3) = 0x11;

    // ベリファイ
    return verify(data, area_def->addr, sizeof(MDLSTRG_DATA_CARD_T));
}

/*
 * WiFi接続情報存在確認
 */
bool_t exists_wifi_param() {
    bool_t found = false;
    uint8_t data;

    // 領域情報取得
    int area = FLASH_AREA_WIFI_PARAM;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);

    flash_read(&data, area_def->addr, 1);

    // 最初の1byteに文字列が入っていれば、データありとみなす
    if (data != 0xFF) {
        found = true;
    }
    return found;
}

/*
 * WiFi接続情報読み出し
 */
bool_t read_wifi_param(MDLSTRG_DATA_WIFI_PARAM_T* data) {
    bool_t found = false;

    // 存在確認
    found = exists_wifi_param();
    if (found == true) {
        // 領域情報取得
        int area = FLASH_AREA_WIFI_PARAM;
        const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
        assert(area_def->area == area);

        flash_read(data, area_def->addr, sizeof(MDLSTRG_DATA_WIFI_PARAM_T));
    }
    return found;
}

/*
 * WiFi接続情報書き込み
 */
bool_t write_wifi_param(const MDLSTRG_DATA_WIFI_PARAM_T* data) {
    bool_t found = false;

    // 領域情報取得
    int area = FLASH_AREA_WIFI_PARAM;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);

    // すでにデータがあったら削除する
    found = exists_wifi_param();
    if (found == true) {
        flash_erase(area_def->addr, area_def->leb_size);
    }

    DBGLOG2("addr = 0x08%d, size = %d", area_def->addr, sizeof(MDLSTRG_DATA_WIFI_PARAM_T));
    flash_write(area_def->addr, data, sizeof(MDLSTRG_DATA_WIFI_PARAM_T));

    // ベリファイエラー動作確認
    //*((int8_t*)data+3) = 0x11;

    // ベリファイ
    return verify(data, area_def->addr, sizeof(MDLSTRG_DATA_WIFI_PARAM_T));
}

/*
 * 操作ログ読込カウンタ更新
 */
void increment_ope_log() {
    increment_read_log_cnt(FLASH_AREA_OPE_LOG);
    return;
}

/*
 * 操作ログ書き込み
 */
ER write_ope_log(const MDLSTRG_DATA_OPE_LOG_T* data, intptr_t data_cnt) {
    // 領域情報取得
    int area = FLASH_AREA_OPE_LOG;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);
	
	int current_sector = 0;
    int write_cnt = 0;
	unsigned long long log_cnt[2] = {0};
    unsigned long long log_cnt_verify[2] = {0};
    intptr_t current_sector_addr = 0;
    intptr_t latest_log_addr = 0;
    intptr_t erase_sector_addr = 0;
    ER verify_res = E_SYS;

	// どのセクターに書き込むか判断
	current_sector = search_current_sector(area_def->addr);
	// 初期化されているセクターが見つからなかった場合はエラー
	if(current_sector == -1){
        DBGLOG0("blank sector not found");
		return E_SYS;
	}
    current_sector_addr = area_def->addr + area_def->leb_size * current_sector;

    for(int i = 0; i < data_cnt; i++){
        latest_log_addr = current_sector_addr + sizeof(MDLSTRG_DATA_RW_COUNTER_T);
        // 書き込みカウンタ確認
        write_cnt = check_write_cnt(current_sector_addr);
#if 1
        DBGLOG1("write_cnt:%d", write_cnt);
#else
        DBGLOG1("write_cnt:%d before", write_cnt);
#endif
        // セクターが満杯の場合、次のセクターの先頭に追記する
        if(write_cnt >= MAX_NUM_OPE_LOG){
            // セクター15までは順次 次のセクターに進む
            if(current_sector < (MAX_NUM_LOG_SECTOR - 1)){
                DBGLOG0("sector is less than 15");
                current_sector_addr = current_sector_addr + area_def->leb_size;
                current_sector++;
            }else{
                DBGLOG0("sector is 15");
                current_sector_addr = area_def->addr;
                current_sector = 0;
            }
            DBGLOG1("current sector : %d", current_sector);
            latest_log_addr = current_sector_addr + sizeof(MDLSTRG_DATA_RW_COUNTER_T);

            // 新しく書き込むセクターのもう一つ先のセクターが空でない場合、そのセクターを初期化する
            if((current_sector + 1) < MAX_NUM_LOG_SECTOR){
                erase_sector_addr = current_sector_addr + area_def->leb_size;
            }else{
                erase_sector_addr = area_def->addr;
                DBGLOG2("current sector : %d  erase sector addr: 0x%08x", current_sector, erase_sector_addr);
             }
            flash_read(log_cnt, erase_sector_addr, sizeof(unsigned long long)*2);

            if(log_cnt[0] != LOG_CNT_NO_DATA || log_cnt[1] != LOG_CNT_NO_DATA){
                flash_erase(erase_sector_addr, area_def->leb_size);
            }
        }else{
            latest_log_addr = current_sector_addr + sizeof(MDLSTRG_DATA_RW_COUNTER_T) + sizeof(MDLSTRG_DATA_OPE_LOG_T) * (write_cnt);
            DBGLOG1("sector is not full latest_log_addr: 0x%08x", latest_log_addr);
        }

        // ログ追記
        flash_write(latest_log_addr, data, sizeof(MDLSTRG_DATA_OPE_LOG_T));
        //MDLSTRG_DATA_OPE_LOG_T* test_ope_log;
        //flash_read(&test_ope_log, latest_log_addr, sizeof(MDLSTRG_DATA_OPE_LOG_T));
        //cmndbg_hexdump(&test_ope_log, sizeof(MDLSTRG_DATA_OPE_LOG_T), "write ope log");

        // 書き込み確認に使用するため、書き換え前の状態を保存
        flash_read(log_cnt_verify, current_sector_addr, sizeof(unsigned long long)*2);
        // 書込みカウンタ更新
        update_log_cnt(current_sector_addr);
        //MDLSTRG_DATA_RW_COUNTER_T* test_ope_log_cnt;
        //flash_read(&test_ope_log_cnt, current_sector_addr, sizeof(MDLSTRG_DATA_RW_COUNTER_T));
        //cmndbg_hexdump(&test_ope_log_cnt, sizeof(MDLSTRG_DATA_RW_COUNTER_T), "write ope log cnt");

        // 書き込み確認
        verify_res = verify_log_cnt(log_cnt_verify, current_sector_addr, sizeof(unsigned long long)*2);
        if(verify_res != E_OK){
            DBGLOG0("verify_log_cnt err");
            return verify_res;
        }
#if 0
        write_cnt = check_write_cnt(current_sector_addr);
        DBGLOG1("write_cnt:%d after", write_cnt);
#endif
    }
   verify_res = verify(data, latest_log_addr, sizeof(MDLSTRG_DATA_OPE_LOG_T));
   DBGLOG1("verify: %d", verify_res);

   return verify_res;
}

/*
 * 操作ログ読込み
 */
bool_t read_ope_log(MDLSTRG_DATA_OPE_LOG_T* data) {
    bool_t res = false;
    res = read_latest_log(data, FLASH_AREA_OPE_LOG);

    if(data->log_date->year == LOG_NO_DATA){
        res = false;
    }
    return res;
}

/*
 * 操作ログR/Wカウンタ読込
 */
bool_t read_ope_log_cnt(MDLSTRG_DATA_RW_COUNTER_T* data) {
    return (get_read_write_cnt(data, FLASH_AREA_OPE_LOG));
}

/*
 * エラーログ読込カウンタ更新
 */
void increment_err_log() {
    increment_read_log_cnt(FLASH_AREA_ERR_LOG);
    return;
}

/*
 * エラーログ書き込み
 */
ER write_err_log(const MDLSTRG_DATA_ERR_LOG_T* data) {
    // 領域情報取得
    int area = FLASH_AREA_ERR_LOG;
    const FLASH_AREA_DEF_T* area_def = &(FLASH_AREA_DEFS[area]);
    assert(area_def->area == area);
	
	int current_sector = 0;
    int write_cnt = 0;
	unsigned long long log_cnt[2] = {0};
    unsigned long long log_cnt_verify[2] = {0};
    intptr_t current_sector_addr = 0;
    intptr_t latest_log_addr = 0;
    intptr_t erase_sector_addr = 0;
    ER verify_res = E_SYS;

	// どのセクターに書き込むか判断
	current_sector = search_current_sector(area_def->addr);
	// 初期化されているセクターが見つからなかった場合はエラー
	if(current_sector == -1){
        DBGLOG0("blank sector not found");
		return E_SYS;
	}
    current_sector_addr = area_def->addr + area_def->leb_size * current_sector;

    // 書き込みカウンタ確認
    write_cnt = check_write_cnt(current_sector_addr);

    // セクターが満杯の場合、次のセクターの先頭に追記する
    if(write_cnt >= MAX_NUM_ERR_LOG){
        // セクター15までは順次 次のセクターに進む
        if(current_sector < (MAX_NUM_LOG_SECTOR - 1)){
            DBGLOG0("sector is less than 15");
            current_sector_addr = current_sector_addr + area_def->leb_size;
            current_sector++;
        }else{
            DBGLOG0("sector is 15");
            current_sector_addr = area_def->addr;
            current_sector = 0;
        }
        latest_log_addr = current_sector_addr + sizeof(MDLSTRG_DATA_RW_COUNTER_T);

        // 新しく書き込むセクターのもう一つ先のセクターが空でない場合、そのセクターを初期化する
        if((current_sector + 1) < MAX_NUM_LOG_SECTOR){
            erase_sector_addr = current_sector_addr + area_def->leb_size;
        }else{
            erase_sector_addr = area_def->addr;
            DBGLOG2("current sector : %d  erase sector addr: 0x%08x", current_sector, erase_sector_addr);
        }
        flash_read(log_cnt, erase_sector_addr, sizeof(unsigned long long)*2);
        if(log_cnt[0] != LOG_CNT_NO_DATA || log_cnt[1] != LOG_CNT_NO_DATA){
            flash_erase(erase_sector_addr, area_def->leb_size);
        }
    }else{
        latest_log_addr = current_sector_addr + sizeof(MDLSTRG_DATA_RW_COUNTER_T) + sizeof(MDLSTRG_DATA_ERR_LOG_T) * (write_cnt);
        DBGLOG1("sector is not full latest_log_addr: 0x%08x", latest_log_addr);
    }

    // ログ追記
    flash_write(latest_log_addr, data, sizeof(MDLSTRG_DATA_ERR_LOG_T));

    // 書き込み確認に使用するため、書き換え前の状態を保存
    flash_read(&log_cnt_verify, current_sector_addr, sizeof(unsigned long long)*2);
    // 書込みカウンタ更新
    update_log_cnt(current_sector_addr);

    // ベリファイ
    verify_res = verify(data, latest_log_addr, sizeof(MDLSTRG_DATA_ERR_LOG_T));
    DBGLOG1("verify: %d", verify_res);
 
    return verify_res;
}

/*
 * エラーログ読込み
 */
bool_t read_err_log(MDLSTRG_DATA_ERR_LOG_T* data) {
    bool_t res = false;
    res = read_latest_log(data, FLASH_AREA_ERR_LOG);

    if(data->log_date->year == LOG_NO_DATA){
        res = false;
    }
    return res;
}

/*
 * エラーログR/Wカウンタ読込
 */
bool_t read_err_log_cnt(MDLSTRG_DATA_RW_COUNTER_T* data) {
    return (get_read_write_cnt(data , FLASH_AREA_ERR_LOG));
}

/*
 * Room ステータス読込
 */
bool_t read_room_sts(MDLSTRG_DATA_ROOM_STS_T* room_data)
{
    assert(room_data);

    DBGLOG0("read_room_sts 00");
    
    bool_t found = false;
    intptr_t addr = 0;
    size_t size = 0;
    DB_CURSOR_T cursor = {0};

    // 機器識別情報
    found = db_get_data_addr(&addr, &size, &cursor, FLASH_AREA_ROOM_STS, 0);
    if (!found) {
        DBGLOG0("read_room_sts not found");
        goto end;	// 見付からない場合
    }
    DBGLOG0("read_room_sts 01");
    // フラッシュ読込み
    flash_read(room_data, addr, sizeof(MDLSTRG_DATA_ROOM_STS_T));

end:
    return found;
}

/*
 * Room ステータス書き込み
 */
void write_room_sts(const MDLSTRG_DATA_ROOM_STS_T* room_data)
{
    assert(room_data);

    bool_t found = false;
    intptr_t addr = 0;
    size_t size = 0;
    DB_CURSOR_T cursor = {0};

    // 書き込み先を取得
    found = db_new_data(&addr, &size, &cursor, FLASH_AREA_ROOM_STS, 0);
    assert(found);

    // フラッシュ書込み
    flash_write(addr, room_data, sizeof(MDLSTRG_DATA_ROOM_STS_T));

    // ヘッダを更新
    db_set_valid(&cursor, 0, sizeof(MDLSTRG_DATA_ROOM_STS_T));
    
    return;
}

////////////////////////

void init_store_program(int index)
{
    bool_t found = false;
    intptr_t db_addr = 0;
    size_t block_size = 0;
    DB_CURSOR_T cursor = {0};

    // Get write destination
    found = db_get_data_addr_1d1li(&db_addr, &block_size, &cursor, &(FLASH_AREA_DEFS[FLASH_AREA_STORE_PROGRAM]), index, SEARCH_FOR_WRITE_DEST);
    assert(found);

    // Erase Store Program Area if db_state is BLOCK_STATE_INVALID
    if (cursor.db_state == BLOCK_STATE_INVALID) {
        delete_store_program(0, index);
    }
}

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
            intptr_t leb_addr = cursor.area_def->addr + (cursor.area_def->leb_size * index);
            size_t erase_size = size + sizeof(LEB_HEADER_T) + sizeof(DB_HEADER_T);
            erase_size = cursor.area_def->leb_size;
            DBGLOG2("Erasing %08x, size %d", leb_addr, erase_size);
            flash_erase(leb_addr, erase_size);
//            db_set_invalid(&cursor);
        } else {
            intptr_t leb_addr = cursor.area_def->addr + (cursor.area_def->leb_size * index);
//            size_t erase_size = size + sizeof(LEB_HEADER_T) + sizeof(DB_HEADER_T);
//            erase_size = (((erase_size - 1) / DRVFLX_ERASE_BLOCK_SIZE) + 1) * DRVFLX_ERASE_BLOCK_SIZE;
//            DBGLOG2("Erasing %08x, size %d", leb_addr, erase_size);
//            flash_erase(leb_addr, erase_size);

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
