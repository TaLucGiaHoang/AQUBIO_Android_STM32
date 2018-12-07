/*
 * VA-X 動的メモリ
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/03/08 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "cmnmalc.h"

#if 0	// 標準C
#include <stdint.h>
#include <limits.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#else	// toppers
#include <string.h>
#include <t_stddef.h>
#include <t_syslog.h>
#endif

#define SIZE_T_MAX	((size_t)(-1))

// デバッグログ
#if 0
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[CMNMALC]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[CMNMALC]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[CMNMALC]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[CMNMALC]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

// メモリセル
typedef struct CMNMALC_CELL_T_TAG CMNMALC_CELL_T;
struct CMNMALC_CELL_T_TAG {
    CMNMALC_CELL_T* list_next;
    size_t size;
};

// 領域
typedef struct {
    CMNMALC_CELL_T free_first;
    CMNMALC_CELL_T free_last;
    size_t area_size;
} CMNMALC_AREA_T;

// オフセットを指定してオブジェクトに格納されているアドレスを取得
static inline void* get_ptr(void* obj, size_t offset)
{
    return *(void**)((intptr_t)obj + offset);
}

// オフセットを指定してオブジェクトに格納されているアドレスを設定
static inline void set_ptr(void* obj, size_t offset, void* ptr)
{
    *(void**)((intptr_t)obj + offset) = ptr;
}

// プロトタイプ
static void* list_find_prev(void* first, size_t ptr_offset, void* a_obj);
static void list_insert(void* prev_obj, size_t ptr_offset, void* obj);
static void list_remove(void* prev_obj, size_t ptr_offset);
static CMNMALC_CELL_T* cell_divide(CMNMALC_CELL_T* cell, size_t size);
static void dump_cell(CMNMALC_CELL_T* cell);

// 内部変数
//static CMNMALC_AREA_T* s_area = NULL;

// malloc
void* cmnmalc_alloc(CMNMALC_HANDLE_T* h, size_t a_size)
{
    assert(h);

    CMNMALC_AREA_T* area = (CMNMALC_AREA_T*)h->area;

    size_t size = a_size;
    if (size % sizeof(intptr_t) != 0) {
        size += (sizeof(intptr_t) - (size % sizeof(intptr_t)));
    }

    void* ptr = NULL;

    CMNMALC_CELL_T* free = NULL;
    for (CMNMALC_CELL_T* cell = area->free_first.list_next; cell; cell = cell->list_next) {
        if (cell->size >= size && cell->size != SIZE_T_MAX) {
            free = cell;
            break;
        }
    }
    if (!free) {
        goto end;
    }

    void* rest = cell_divide(free, size);
    if (rest) {
        list_insert(free, offsetof(CMNMALC_CELL_T, list_next), rest);
    }
    
    CMNMALC_CELL_T* free_prev = NULL;
    free_prev = list_find_prev(&(area->free_first), offsetof(CMNMALC_CELL_T, list_next), free);
    assert(free_prev);
    list_remove(free_prev, offsetof(CMNMALC_CELL_T, list_next));
    ptr = (void*)((intptr_t)free + sizeof(CMNMALC_CELL_T));

end:
    DBGLOG2("cmnmalc_alloc(%d) => %p\n", a_size, ptr);
    assert(ptr);
    return ptr;
}

// realloc
void* cmnmalc_realloc(CMNMALC_HANDLE_T* h, void *p, size_t n)
{
    assert(h);

    void* new_ptr = NULL;

    if (n > 0) {
        new_ptr = cmnmalc_alloc(h, n);
        if (!new_ptr) {
            goto end;
        }
    }

    if (p && n > 0) {
        CMNMALC_CELL_T* old_cell = (CMNMALC_CELL_T*)((intptr_t)p - sizeof(CMNMALC_CELL_T));
        CMNMALC_CELL_T* new_cell = (CMNMALC_CELL_T*)((intptr_t)new_ptr - sizeof(CMNMALC_CELL_T));

        size_t copy_size = old_cell->size > new_cell->size ? new_cell->size : old_cell->size;
        memcpy(new_ptr, p, copy_size);
    }
    cmnmalc_free(h, p);

end:
    return new_ptr;
}

// calloc
void* cmnmalc_calloc(CMNMALC_HANDLE_T* h, size_t n, size_t s)
{
    assert(h);

    void* ptr = NULL;
    ptr = cmnmalc_alloc(h, n * s);

    if (ptr) {
        memset(ptr, 0, n * s);
    }

    return ptr;
}

// free
void cmnmalc_free(CMNMALC_HANDLE_T* h, void *p)
{
    assert(h);
    DBGLOG1("cmnmalc_free(%p)\n", p);

    CMNMALC_AREA_T* area = (CMNMALC_AREA_T*)h->area;

    if (!p) {
        goto end;
    }

    CMNMALC_CELL_T* cell = (CMNMALC_CELL_T*)((intptr_t)p - sizeof(CMNMALC_CELL_T));
    CMNMALC_CELL_T* last_prev = list_find_prev(&(area->free_first), offsetof(CMNMALC_CELL_T, list_next), &(area->free_last));
    assert(last_prev);
    list_insert(last_prev, offsetof(CMNMALC_CELL_T, list_next), cell);

end:
    return;
}

// 領域を初期化する
void cmnmalc_init(CMNMALC_HANDLE_T* h, void* area, size_t size)
{
    assert(h);
    assert(area);
    assert(size > 0);

    h->area = area;
    CMNMALC_AREA_T* cmnmalc_area = (CMNMALC_AREA_T*)h->area;
    memset(area, 0, sizeof(CMNMALC_AREA_T));
    cmnmalc_area->area_size = size - sizeof(CMNMALC_AREA_T);

    cmnmalc_area->free_first.size = SIZE_T_MAX;
    cmnmalc_area->free_last.size = SIZE_T_MAX;

    list_insert(&(cmnmalc_area->free_first), offsetof(CMNMALC_CELL_T, list_next), &(cmnmalc_area->free_last));

    CMNMALC_CELL_T* first_cell = (CMNMALC_CELL_T*)((intptr_t)area + sizeof(CMNMALC_AREA_T));
    memset(first_cell, 0, sizeof(CMNMALC_CELL_T));
    first_cell->size = cmnmalc_area->area_size - sizeof(CMNMALC_CELL_T);

    list_insert(&(cmnmalc_area->free_first), offsetof(CMNMALC_CELL_T, list_next), first_cell);
}

// 全セルをダンプする
void cmnmalc_dump(CMNMALC_HANDLE_T* h)
{
    assert(h);

    CMNMALC_AREA_T* area = (CMNMALC_AREA_T*)h->area;

    DBGLOG0("---------- cmnmalc_dump ----------\n");
    DBGLOG0("ALL CELLS:\n");
    CMNMALC_CELL_T* area_end = (CMNMALC_CELL_T*)((intptr_t)area + sizeof(CMNMALC_AREA_T) + area->area_size);
    CMNMALC_CELL_T* next = NULL;
    CMNMALC_CELL_T* first_cell = (CMNMALC_CELL_T*)((intptr_t)area + sizeof(CMNMALC_AREA_T));
    for (CMNMALC_CELL_T* cell = first_cell; cell < area_end; cell = next) {
        dump_cell(cell);
        next = (CMNMALC_CELL_T*)((intptr_t)cell + sizeof(CMNMALC_CELL_T) + cell->size);
        if (next > area_end) {
            DBGLOG3("overflow: area_end: %p, last_cell: %p, last_size: %u\n", area_end, cell, cell->size);
            break;
        }
    }

    DBGLOG0("free_first:\n");
    dump_cell(&(area->free_first));
    DBGLOG0("free_last:\n");
    dump_cell(&(area->free_last));
    DBGLOG0("ALL FREE CELLS:\n");
    for (CMNMALC_CELL_T* cell = &(area->free_first); cell; cell = cell->list_next) {
        dump_cell(cell);
    }
    DBGLOG0("--------------------------------------\n");
}

// -------------------------------------------------------------------
// 内部関数関数
// -------------------------------------------------------------------

// セルを分割する
CMNMALC_CELL_T* cell_divide(CMNMALC_CELL_T* cell, size_t size)
{
    assert(size % sizeof(intptr_t) == 0);

    CMNMALC_CELL_T* divided = NULL;

    if (cell->size < size + sizeof(CMNMALC_CELL_T)) {
        goto end;
    }

    divided = (CMNMALC_CELL_T*)((intptr_t)cell + (sizeof(CMNMALC_CELL_T) + size));
    divided->size = cell->size - (sizeof(CMNMALC_CELL_T) + size);
    cell->size = size;

end:
    return divided;
}

void cell_merge(CMNMALC_CELL_T* prev_cell)
{
    
}

// 与えられたオブジェクトを指しているオブジェクトを取得
void* list_find_prev(void* first, size_t ptr_offset, void* a_obj)
{
    void* found = NULL;
    for (void* obj = first; obj; obj = get_ptr(obj, ptr_offset)) {
        if (a_obj == get_ptr(obj, ptr_offset)) {
            found = obj;
            break;
        }
    }

    return found;
}

// 与えられたオブジェクトの次の位置にオブジェクトを挿入
void list_insert(void* prev_obj, size_t ptr_offset, void* obj)
{
    void* next_obj = get_ptr(prev_obj, ptr_offset);
    set_ptr(prev_obj, ptr_offset, obj);
    set_ptr(obj, ptr_offset, next_obj);
}

// 与えられたオブジェクトの次のオブジェクトをリストから削除
void list_remove(void* prev_obj, size_t ptr_offset)
{
    void* obj = get_ptr(prev_obj, ptr_offset);
    void* next_obj = get_ptr(obj, ptr_offset);
    set_ptr(obj, ptr_offset, NULL);
    set_ptr(prev_obj, ptr_offset, next_obj);
}

// セルのダンプ
void dump_cell(CMNMALC_CELL_T* cell)
{
    DBGLOG3("[%p]list_next=%p, size=%d\n", cell, cell->list_next, cell->size);
}

