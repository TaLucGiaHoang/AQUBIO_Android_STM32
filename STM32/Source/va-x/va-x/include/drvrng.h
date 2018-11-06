/*
 * VA-X 乱数ジェネレータドライバ
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/03/29 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>

/*
 * マクロ
 */


/*
 * 公開関数
 */

/* 乱数取得 */
void drvrng_get_random(uint8_t* buf, size_t size);

