/*
 * VA-X 暗号アルゴリズム (libtomcrypt)
 * libtomcrypt用のmallocシリーズ
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/03/08 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

//#include <t_stddef.h>

void* cmncrypt_malloc(size_t a_size);
void* cmncrypt_realloc(void *p, size_t n);
void* cmncrypt_calloc(size_t n, size_t s);
void cmncrypt_free(void *p);
void cmncrypt_init_malloc();

