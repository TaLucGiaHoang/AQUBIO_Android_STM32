/*
 * VA-X 動的メモリ
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/03/08 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

//#include <stddef.h>
#include <t_stddef.h>

typedef struct {
    void* area;
} CMNMALC_HANDLE_T;

void* cmnmalc_alloc(CMNMALC_HANDLE_T* h, size_t a_size);
void* cmnmalc_realloc(CMNMALC_HANDLE_T* h, void *p, size_t n);
void* cmnmalc_calloc(CMNMALC_HANDLE_T* h, size_t n, size_t s);
void cmnmalc_free(CMNMALC_HANDLE_T* h, void *p);
void cmnmalc_init(CMNMALC_HANDLE_T* h, void* area, size_t size);
void cmnmalc_dump(CMNMALC_HANDLE_T* h);

