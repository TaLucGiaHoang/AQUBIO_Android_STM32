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

/*
 * 公開関数
 */


/*
 * 内部関数
 */


