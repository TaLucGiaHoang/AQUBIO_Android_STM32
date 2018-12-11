/*
 * VA-X バッファ管理
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/02/07 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>

// バッファの用途定義
enum {
    CMNBUF_USE_NONE = 0,		// なし
    CMNBUF_USE_CAMERA_CAPTURE,	// カメラキャプチャ
    CMNBUF_USE_AUTHDATA_1,		// 認証1(カメラキャプチャ領域と共存可)
    CMNBUF_USE_AUTHDATA_2,		// 認証2(カメラキャプチャ領域と共存不可)
    CMNBUF_NUM_USE,
};

/*
 * バッファの取得
 */
size_t cmnbuf_aquire_buffer(void** buffer, int use);

/*
 * バッファの開放
 */
void cmnbuf_release_buffer(void* buffer, int use);

/*
 * 
 */

