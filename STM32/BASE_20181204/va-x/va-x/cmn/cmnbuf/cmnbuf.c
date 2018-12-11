/*
 * VA-X バッファ管理
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/02/07 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "cmnbuf.h"

#include <string.h>

#include <kernel.h>
#include <kernel_cfg.h>
#include <t_syslog.h>
#include <t_stdlib.h>

/*
 * マクロ定義
 */

// サービスコールのエラーのログ出力
static inline void svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}
#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))

#define BLOCK_SIZE	(4 * 1024)	// バッファブロックサイズ(4kB)
#define NUM_BLOCKS	92			// バッファブロック数

/*
 * 型定義
 */

typedef struct {
    uint8_t data[BLOCK_SIZE];
} BUFFER_BLOCK_T;

typedef struct {
    int use;	// 使用中の用途
} BUFFER_STATUS_T;

typedef struct {
    int use;		// 用途
    int head;		// 先頭ブロック
    int nblocks;	// ブロック数
} BUFFER_DEF_T;

/*
 * 内部関数プロトタイプ
 */


/*
 * 定数定義
 */

// バッファ定義
static const BUFFER_DEF_T BUFFER_DEF[] = {
    /* 用途,						先頭ブロック,	ブロック数	*/
    { CMNBUF_USE_NONE,				0,				0	},	// なし
    { CMNBUF_USE_CAMERA_CAPTURE,	8,				76	},	// カメラキャプチャ用 (8-83, 300kB + 4kB (76blocks))
    { CMNBUF_USE_AUTHDATA_1,		0,				8	},	// 認証用1 (0-7, 32kB (8blocks)))
    { CMNBUF_USE_AUTHDATA_2,		8,				76	},	// 認証用2 (8-83, 304kB (76blocks)) (カメラ用とオーバーラップ)
};


/*
 * 内部変数
 */

// バッファ本体
__attribute__ ((section (".largebuf"))) static BUFFER_BLOCK_T s_buffers[NUM_BLOCKS];

// バッファ状態
static BUFFER_STATUS_T s_buffer_status[NUM_BLOCKS] = {0};


/*
 * バッファの取得
 */
size_t cmnbuf_aquire_buffer(void** buffer, int use)
{
    ER er = twai_sem(SEM_CMNBUF, 100);
    assert(er == E_OK);

    assert(buffer);
    assert((use > CMNBUF_USE_NONE) && (use < CMNBUF_NUM_USE));

    assert(BUFFER_DEF[use].use == use);

    int head = BUFFER_DEF[use].head;
    void* head_addr = s_buffers[head].data;

    for (int i = 0; i < BUFFER_DEF[use].nblocks; i++) {
        assert(s_buffer_status[head + i].use == CMNBUF_USE_NONE);
        s_buffer_status[head + i].use = use;
    }

    *buffer = head_addr;
    sig_sem(SEM_CMNBUF);
    return BLOCK_SIZE * BUFFER_DEF[use].nblocks;
}

/*
 * バッファの開放
 */
void cmnbuf_release_buffer(void* buffer, int use)
{
    ER er = twai_sem(SEM_CMNBUF, 100);
    assert(er == E_OK);

    assert(buffer);
    assert((use > CMNBUF_USE_NONE) && (use < CMNBUF_NUM_USE));
    assert(BUFFER_DEF[use].use == use);

    int head = BUFFER_DEF[use].head;
    void* head_addr = s_buffers[head].data;
    assert(buffer == head_addr);

    for (int i = 0; i < BUFFER_DEF[use].nblocks; i++) {
        assert(s_buffer_status[head + i].use == use);
        s_buffer_status[head + i].use = CMNBUF_USE_NONE;
    }

    sig_sem(SEM_CMNBUF);
    return;
}



