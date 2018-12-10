/*
 * VA-X デバッグ機能
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/02/23 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>
#include <sil.h>

/*
 * 定数定義
 */


/*
 * 型定義
 */


/*
 * 関数プロトタイプ
 */

// シリアルに16進ダンプ
void cmndbg_hexdump(const void* buf, size_t nbytes, const char* filename);

// バイナリ形式のフレームをシリアルに出力
void cmndbg_binframe(const void* abuf, uint16_t type, uint16_t sizex, uint16_t sizey);

/*
 * インライン関数
 */

