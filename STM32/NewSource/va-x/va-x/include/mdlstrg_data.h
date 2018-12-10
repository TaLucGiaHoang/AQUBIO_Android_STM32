/*
 * VA-X 保存データ管理ミドル データ定義
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/02/16 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>

#include <mdlauthlib.h>

/*
 * マクロ定数
 */

#define MDLSTRG_MAX_NDATA_AUTH	20
#define MDLSTRG_MAX_NDATA_CARD	1
#define MDLSTRG_MAX_VACANCY_CARD 20
#define MDLSTRG_MAX_DROOM_CARD 10
#define MDLSTRG_MAX_ID_CODE    256

#define MDLSTRG_MAX_ESSID_LEN           32
#define MDLSTRG_MAX_PASSPHRASE_LEN      64

#define MDLSTRG_MAX_ERR_LOG             12

/*
 * 型定義
 */

// 登録メタ情報
typedef struct {
    uint8_t		slotno;			// 登録スロット名
    uint8_t		user_type;		// ユーザータイプ
    uint8_t		finger_type;	// 指種別
    uint8_t		name_len;		// 名前長さ
    uint32_t	name[32];		// 名前(UTF-32-LE)
} MDLSTRG_DATA_AUTH_META_T;

// 登録メタ情報(カード)

typedef struct {
    uint8_t		slotno;			// 登録スロット名
    uint8_t		name_len;		// 名前長さ
    uint32_t	name[32];		// 名前(UTF-32-LE)
    uint8_t 	card[16];		// サービスコード300B(ASCII-8BIT)
} MDLSTRG_DATA_CARD_META_T;

// キャッシュメタ情報(カード)

typedef struct {
    uint32_t	data_id;	// データID
    size_t		data_len;	// データ長
    uint32_t	db_state;	// データブロック状態
    uint8_t		slotno;			// 登録スロット名
    uint8_t		name_len;		// 名前長さ
//    uint8_t		dummy_one;		// dummyデータ1
//    uint8_t		dummy_two;		// dummyデータ2
    uint32_t	name[32];		// 名前(UTF-32-LE)
    uint8_t 	card[16];		// サービスコード300B(ASCII-8BIT)
} MDLSTRG_DATA_CHASH_META_T;

// 極小データ
typedef struct {
    int count;
    struct {
        int data_type;
        int index;
        MDLAUTHLIB_DATA_REGISTERED_SMALL_T data;
    } array[MDLSTRG_MAX_NDATA_AUTH];
} MDLSTRG_DATA_AUTH_SMALL_T;

// アプリ登録情報
typedef struct {
    uint32_t name[16];		// 登録名
    uint8_t pubkey[294];	// 公開鍵
} MDLSTRG_DATA_APPKEY_T;

// カード情報（一括登録カード、空室カード、登録・抹消カード共通）
typedef struct {
    uint8_t status_flg[8];
    //uint64_t status_flg;
    uint8_t idm[8];
    uint8_t card_type[16];
    uint8_t member_num[16];
} MDLSTRG_DATA_CARD_T;

// D-roomカード情報
typedef struct {
    uint8_t		slotno;			// 登録スロット名
    uint8_t		name_len;		// 名前長さ
    uint8_t status_flg[8];      //登録状況（全bit1の場合未登録）
//    uint64_t status_flg;
    uint8_t idm[8];             //idm
    uint8_t card_type[16];      //カード種別
    uint8_t member_num[16];     //(サービスコード300Bの情報)(ASCII-8BIT)
    uint8_t user_name[128];     //名前(UTF-32-LE)??
} MDLSTRG_DATA_DROOM_CARD_T;

// 識別コード情報
typedef struct {
    uint16_t flg;
    uint16_t idx;                                       // ここのインデックスは1起算
    uint32_t block;
    uint8_t  code[8];
} MDLSTRG_DATA_IDCODE_T;

/* 識別コード管理データ */
typedef struct {
    MDLSTRG_DATA_IDCODE_T id_code;
    uint32_t address;
} MDLSTRG_DATA_IDENTIFY_CODE_MNG_T;

typedef struct {
    uint8_t essid[MDLSTRG_MAX_ESSID_LEN];                // ssid（1～32文字）
    size_t essid_len;
    uint8_t passphrase[MDLSTRG_MAX_PASSPHRASE_LEN];     // passphrase
    size_t passphrase_len;
} MDLSTRG_DATA_WIFI_PARAM_T;

// ログのRead/Writeカウンター
typedef struct {
    unsigned long long bitsw_wt[2];	//ログ書込みカウンター
	unsigned long long bitsw_rd[2];	//ログ読込みカウンター
} MDLSTRG_DATA_RW_COUNTER_T;

// ログ発生日時
typedef struct {
    unsigned short year;
	unsigned char  month;
	unsigned char  day;
	unsigned char  hour;
	unsigned char  min;
    unsigned short dmy;     // 予備エリア
} MDLSTRG_DATA_STRACT_DATE_T;

// 状態ログ
typedef struct {
    unsigned char state;        // 大分類 状態
    unsigned char processing;   // 中分類 処理内容
    unsigned char detail01;     // 小分類 処理詳細
    unsigned char detail02;     // 小分類 処理詳細
	unsigned int  detail03;     // 小分類 処理詳細
} MDLSTRG_DATA_STRACT_LOG_T;

// 操作ログ情報
typedef struct {
    MDLSTRG_DATA_STRACT_DATE_T log_date[2];	// ログ発生日時
	MDLSTRG_DATA_STRACT_LOG_T  log[5];       // 状態のログ
} MDLSTRG_DATA_OPE_LOG_T;

// エラーログ情報
typedef struct {
    MDLSTRG_DATA_STRACT_DATE_T log_date[2];	// ログ発生日時
	unsigned short             err[MDLSTRG_MAX_ERR_LOG];     // エラーログ
} MDLSTRG_DATA_ERR_LOG_T;

// AQUBIOデバイスステータス
typedef struct {
    uint8_t cst_sts;      //工事  ステータスフラグ 初期値：1
    uint8_t vac_sts;      //空室  ステータスフラグ
    uint8_t drm_sts;      //Droom ステータスフラグ
    uint16_t ofs1_vlu;    //認証オフセット値1
    uint16_t ofs2_vlu;    //認証オフセット値2
} MDLSTRG_DATA_ROOM_STS_T;

/*
 * 公開関数
 */


/*
 * 内部関数
 */


