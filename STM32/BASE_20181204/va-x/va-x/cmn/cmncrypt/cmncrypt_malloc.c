/*
 * VA-X 暗号アルゴリズム (libtomcrypt)
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/03/08 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include <t_stddef.h>
#include "cmncrypt_malloc.h"

#include <string.h>
#include <t_syslog.h>

#include "cmnmalc.h"

// デバッグログ
#if 0
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[CMNCRYPT]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[CMNCRYPT]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[CMNCRYPT]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[CMNCRYPT]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

// 内部変数
#define AREASIZE	(1024 * 16)
static uint8_t s_area[AREASIZE];
static CMNMALC_HANDLE_T s_handle;

void* cmncrypt_malloc(size_t a_size)
{
    void* ptr = cmnmalc_alloc(&s_handle, a_size);
    assert(ptr);
    return ptr;
}

void* cmncrypt_realloc(void *p, size_t n)
{
    void* ptr = cmnmalc_realloc(&s_handle, p, n);
    assert(ptr);
    return ptr;
}

void* cmncrypt_calloc(size_t n, size_t s)
{
    void* ptr = cmnmalc_calloc(&s_handle, n, s);
    assert(ptr);
    return ptr;
}

void cmncrypt_free(void *p)
{
    cmnmalc_free(&s_handle, p);
}

void cmncrypt_init_malloc()
{
    cmnmalc_init(&s_handle, s_area, AREASIZE);
}

