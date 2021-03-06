/*
 * VA-X 保存データ管理ミドル
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/02/20 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>
/*
 * マクロ定数
 */

// データ種別
enum {
    MDLSTRG_DATA_TYPE_NONE = 0,		// なし
    MDLSTRG_DATA_TYPE_AUTH,			// 認証データ
    MDLSTRG_DATA_TYPE_AUTH_LEARNED,	// 学習データ
    MDLSTRG_DATA_TYPE_AUTH_SMALL,	// 極小データ(全件読み出し)
    MDLSTRG_DATA_TYPE_AUTH_META,	// 認証データメタ情報
    MDLSTRG_DATA_TYPE_CARD_META,	// カードデータメタ情報
    MDLSTRG_DATA_TYPE_BLE_APPKEY,	// BLEアプリ鍵
    MDLSTRG_DATA_TYPE_DEVICE_ID,	// 機器識別情報
    MDLSTRG_DATA_TYPE_VACANCY_CARD, // 空室カード情報
    MDLSTRG_DATA_TYPE_DROOM_CARD,   // D-roomカード情報
    MDLSTRG_DATA_TYPE_IDCODE,       // 識別コード(D-roomカード)
    MDLSTRG_DATA_TYPE_REG_CARD,     // 登録・抹消カード情報
    MDLSTRG_DATA_TYPE_VACANCY_REG_CARD, // 一括登録カード情報
    MDLSTRG_DATA_TYPE_WIFI,         // WiFi情報
    MDLSTRG_DATA_TYPE_STORE_PROGRAM, // Store program
    MDLSTRG_DATA_TYPE_OPE_LOG,      // 操作ログ
    MDLSTRG_DATA_TYPE_ERR_LOG,      // エラーログ
    MDLSTRG_DATA_TYPE_ROOM_TYPE,    // カードによる部屋の状態フラグ
    MDLSTRG_DATA_TYPE_NUM,			// 末尾
};

enum {
    MDLSTRG_REQ_TYPE_NONE = 0,		// なし
	MDLSTRG_REQ_TYPE_CREATE,		// 新規作成
    MDLSTRG_REQ_TYPE_EXISTS,		// データ存在
    MDLSTRG_REQ_TYPE_READ,			// 読込み
    MDLSTRG_REQ_TYPE_WRITE,			// 書込み
    MDLSTRG_REQ_TYPE_APPEND,		// 追加
    MDLSTRG_REQ_TYPE_DELETE,		// 削除
    MDLSTRG_REQ_TYPE_ALL_DELETE,    // 削除（セクター単位）
    MDLSTRG_REQ_TYPE_INCREMENT,		// カウンターインクリメント
    MDLSTRG_REQ_TYPE_FACTORY_RESET,	// 工場初期化
	MDLSTRG_REQ_TYPE_CNT_READ       // ログ読込み/書込みカウンタ取得
};

// イベント種別
enum {
    MDLSTRG_EVT_NONE = 0,
    MDLSTRG_EVT_INITIALIZE_COMPLETE,	// 初期化完了
    MDLSTRG_EVT_REQUEST_COMPLETE,		// 処理完了
};


/* メモリプールブロック長(内部用) */
#define MDLSTRG_MPFBLK_SIZE		32
#define MDLSTRG_DTQ_SIZE		10

/*
 * 型定義
 */

// 要求
typedef struct {
    int data_type;
    int request_type;
    intptr_t data;
    size_t size;
    intptr_t opt1;
    intptr_t opt2;
} MDLSTRG_REQUEST_T;

// データブロックヘッダ

typedef struct {
    uint32_t	data_id;	// データID
    size_t		data_len;	// データ長
    uint32_t	db_state;	// データブロック状態
} DB_HEADER_T;

/* コールバック関数型 */
typedef void (*MDLSTRG_CALLBACK_T)(int event, intptr_t opt1, intptr_t opt2);

/*
 * 公開関数
 */

// 初期化
void mdlstrg_initialize(MDLSTRG_CALLBACK_T callback);

// ストレージ要求
void mdlstrg_request(const MDLSTRG_REQUEST_T* request, MDLSTRG_CALLBACK_T callback);

/*
 * 内部関数
 */

// タスク
void mdlstrg_task(intptr_t exinf);

