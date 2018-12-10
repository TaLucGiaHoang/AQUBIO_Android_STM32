/*
 * VA-X 乱数ジェネレータドライバ
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/03/29 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "drvrng.h"

#include <kernel.h>
#include <sil.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "kernel_cfg.h"
#include "stm32f7xx.h"
#include "drvcmn.h"

/*
 * マクロ定義
 */

// デバッグログ
#if 1
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[DRVRNG]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[DRVRNG]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[DRVRNG]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[DRVRNG]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

/*
 * 型定義
 */

/*
 * 内部関数
 */
static uint32_t get_random32();

/*
 * 定数定義
 */


/*
 * 内部変数
 */


/* ===================================================================
 * 公開関数
 * ================================================================ */
/* 
 * 乱数取得
 */
void drvrng_get_random(uint8_t* buf, size_t size)
{
    // RCC
    drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_AHB2ENR, 0, RCC_AHB2ENR_RNGEN, RCC_AHB2ENR_RNGEN);

    // RNG_CR
    drvcmn_setreg32(TADR_RNG_BASE + TOFF_RNG_CR, 2, 0x1, 0x1);	// [2]RNGEN

    SYSTIM systim1, systim2;
    get_tim(&systim1);

    size_t count = 0;
    uint32_t random32 = 0;
    for (int i = 0; i < (size + 4) / 4; i++) {
        random32 = get_random32();

        for (int j = 0; j < 4; j++) {
            buf[count] = ((uint8_t*)&random32)[j];
            count++;

            if (count >= size) {
                goto loop_end;
            }
        }
    }
loop_end:

    get_tim(&systim2);
    DBGLOG1("time: %u", systim2 - systim1);
    
    drvcmn_setreg32(TADR_RNG_BASE + TOFF_RNG_CR, 2, 0x1, 0x0);	// [2]RNGEN
    drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_AHB2ENR, 0, RCC_AHB2ENR_RNGEN, 0);

    return;
}

/* ===================================================================
 * 内部関数
 * ================================================================ */
/* 
 * 乱数取得
 */
uint32_t get_random32()
{
    uint32_t data_ready = 0;
    int count = 0;
    uint32_t random = 0;
    for (int i = 0; i < 10000; i++) {
        data_ready = drvcmn_getreg32(TADR_RNG_BASE + TOFF_RNG_SR, 0, 0x1);	// [0]DRDY
        if (data_ready) {
            random  = drvcmn_getreg32(TADR_RNG_BASE + TOFF_RNG_DR, 0, ~(uint32_t)0);
            count = i;
            break;
        }
    }

    if (!data_ready) {
        assert(false);
    }

    DBGLOG2("random: 0x%08x, loop: %u", random, count);
    return random;
}
