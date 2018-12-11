/*
 * VA-X TIM 共通処理
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/18 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成(PWM mode 1 のみ)
 */

#include "drvcmn_tim.h"

#include <t_syslog.h>
#include <sil.h>
#include <stm32f7xx.h>
#include <kernel.h>

#include "drvcmn.h"

/*
 * マクロ定義
 */

// レジスタダンプ
#if 0
  #define DUMP_REG32(name, addr) syslog(LOG_NOTICE, name " (0x%08p): => 0x%08x", addr, drvcmn_getreg32(addr, 0, UINT32_MAX));
  #define DUMP_REG16(name, addr) syslog(LOG_NOTICE, name " (0x%08p): => 0x%08x", addr, drvcmn_getreg16(addr, 0, UINT16_MAX));
#else
  #define DUMP_REG32(name, addr) ;
  #define DUMP_REG16(name, addr) ;
#endif


/*
 * 型定義
 */

// タイマー別のレジスタ情報
typedef struct {
    intptr_t regbase;
    intptr_t rccreg;
    uint32_t rccenbit;
} TIMX_ADRS_T;

// チャネル別のレジスタ情報
typedef struct {
    uint32_t ccmr_offset;
    uint32_t ccr_offset;
} TIMX_CH_REG_T;

/*
 * 定数定義
 */

// タイマー別のレジスタ情報
static const TIMX_ADRS_T TIMX_ADRS[] = {
    /* レジスタベース,	RCC ENレジススタ,               ENビット       */
    { 0,				0,								0					},	/* [0]なし */
    { TADR_TIM1_BASE,	TADR_RCC_BASE+TOFF_RCC_APB2ENR,	RCC_APB2ENR_TIM1EN	},	/* [1]TIM1 */
    { TADR_TIM2_BASE,	TADR_RCC_BASE+TOFF_RCC_APB1ENR,	RCC_APB1ENR_TIM2EN	},	/* [2]TIM2 */
    { TADR_TIM3_BASE,	TADR_RCC_BASE+TOFF_RCC_APB1ENR,	RCC_APB1ENR_TIM3EN	},	/* [3]TIM3 */
    { TADR_TIM4_BASE,	TADR_RCC_BASE+TOFF_RCC_APB1ENR,	RCC_APB1ENR_TIM4EN	},	/* [4]TIM4 */
    { TADR_TIM5_BASE,	TADR_RCC_BASE+TOFF_RCC_APB1ENR,	RCC_APB1ENR_TIM5EN	},	/* [5]TIM5 */
    { TADR_TIM6_BASE,	TADR_RCC_BASE+TOFF_RCC_APB1ENR,	RCC_APB1ENR_TIM6EN	},	/* [6]TIM6 */
    { TADR_TIM7_BASE,	TADR_RCC_BASE+TOFF_RCC_APB1ENR,	RCC_APB1ENR_TIM7EN	},	/* [7]TIM7 */
    { TADR_TIM8_BASE,	TADR_RCC_BASE+TOFF_RCC_APB2ENR,	RCC_APB2ENR_TIM8EN	},	/* [8]TIM8 */
    { TADR_TIM9_BASE,	TADR_RCC_BASE+TOFF_RCC_APB2ENR,	RCC_APB2ENR_TIM9EN	},	/* [9]TIM9 */
    { TADR_TIM10_BASE,	TADR_RCC_BASE+TOFF_RCC_APB2ENR,	RCC_APB2ENR_TIM10EN	},	/* [10]TIM10 */
    { TADR_TIM11_BASE,	TADR_RCC_BASE+TOFF_RCC_APB2ENR,	RCC_APB2ENR_TIM11EN	},	/* [11]TIM11 */
    { TADR_TIM12_BASE,	TADR_RCC_BASE+TOFF_RCC_APB1ENR,	RCC_APB1ENR_TIM12EN	},	/* [12]TIM12 */
    { TADR_TIM13_BASE,	TADR_RCC_BASE+TOFF_RCC_APB1ENR,	RCC_APB1ENR_TIM13EN	},	/* [13]TIM13 */
    { TADR_TIM14_BASE,	TADR_RCC_BASE+TOFF_RCC_APB1ENR,	RCC_APB1ENR_TIM14EN	},	/* [14]TIM14 */
};

// チャネル別のレジスタ情報
static const TIMX_CH_REG_T TIMX_CH_REGS[] = {
    { 0,				0,				},	/* [0]なし */
    { TOFF_TIM_CCMR1,	TOFF_TIM_CCR1,	},	/* [1]CH1 */
    { TOFF_TIM_CCMR1,	TOFF_TIM_CCR2,	},	/* [2]CH2 */
    { TOFF_TIM_CCMR2,	TOFF_TIM_CCR3,	},	/* [3]CH3 */
    { TOFF_TIM_CCMR2,	TOFF_TIM_CCR4,	},	/* [4]CH4 */
    { TOFF_TIM_CCMR3,	TOFF_TIM_CCR5,	},	/* [5]CH5 */
    { TOFF_TIM_CCMR3,	TOFF_TIM_CCR6,	},	/* [6]CH6 */
};

/*
 * 内部変数
 */


/*
 * 内部関数のプロトタイプ
 */


/*
 * 公開関数
 */

/* 初期化 */
void drvcmn_tim_initialize(int tim, const DRVCMN_TIM_SETTING_T* setting)
{
    assert((tim >= 1) && (tim <= 14));
    assert(setting);

    intptr_t timx_base = TIMX_ADRS[tim].regbase;

    /* RCC_APBxENR */
    drvcmn_setreg32(TIMX_ADRS[tim].rccreg, 0, TIMX_ADRS[tim].rccenbit, TIMX_ADRS[tim].rccenbit);
    DUMP_REG32("RCC_APBxENR", TIMX_ADRS[tim].rccreg);

    /* TIMx_PSC */
    drvcmn_setreg16(timx_base + TOFF_TIM_PSC, 0, UINT16_MAX, setting->psc);
    DUMP_REG16("TIMx_PSC", timx_base + TOFF_TIM_PSC);

    /* TIMx_ARR */
    drvcmn_setreg16(timx_base + TOFF_TIM_ARR, 0, UINT16_MAX, setting->arr);
    DUMP_REG16("TIMx_ARR", timx_base + TOFF_TIM_ARR);

#if 0
    /* TIMx_EGR */
    drvcmn_setreg16(timx_base + TOFF_TIM_EGR, 0, TIM_EGR_UG, TIM_EGR_UG);
    DUMP_REG16("TIMx_EGR", timx_base + TOFF_TIM_EGR);
#endif

    /* TIMx_CR1 */
    drvcmn_setreg16(timx_base + TOFF_TIM_CR1, 0, TIM_CR1_ARPE, TIM_CR1_ARPE);
    DUMP_REG16("TIMx_CR1", timx_base + TOFF_TIM_CR1);
}

/* チャネル初期化 */
void drvcmn_tim_ch_initialize(int tim, int ch, const DRVCMN_TIM_CH_SETTING_T* setting)
{
    assert((tim >= 1) && (tim <= 14));
    assert((ch >= 1) && (ch <= 6));
    assert(setting);

    intptr_t timx_base = TIMX_ADRS[tim].regbase;

    /* TIMx_CCMRx */
    /* OCx => ‘0110’ (PWM mode 1)*/
    drvcmn_setreg32(timx_base + TIMX_CH_REGS[ch].ccmr_offset,
                    ((ch - 1) % 2) * 8,
                    TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE,
                    (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1) | TIM_CCMR1_OC1PE);
    DUMP_REG32("TIMx_CCMRx", timx_base + TIMX_CH_REGS[ch].ccmr_offset);

    /* TIMx_CCRx */
    drvcmn_setreg16(timx_base + TIMX_CH_REGS[ch].ccr_offset, 0, UINT16_MAX, setting->ccr);
    DUMP_REG16("TIMx_CCRx", timx_base + TIMX_CH_REGS[ch].ccr_offset);

    /* TIMx_CR2 */
    if (tim == 1 || tim == 8) {	/* Advanced-control timers (TIM1,TIM8) のみ*/
        drvcmn_setreg32(timx_base + TOFF_TIM_CR2, (ch - 1) * 2, TIM_CR2_OIS1, TIM_CR2_OIS1);
        DUMP_REG32("TIMx_CR2", timx_base + TOFF_TIM_CR2);

        drvcmn_setreg32(timx_base + TOFF_TIM_BDTR, 0, TIM_BDTR_MOE, TIM_BDTR_MOE);
        DUMP_REG32("TIMx_BDTR", timx_base + TOFF_TIM_BDTR);
    }
}

// 有効/無効設定
void drvcmn_tim_set_counter_enable(int tim, bool_t enable)
{
    assert((tim >= 1) && (tim <= 14));

    intptr_t timx_base = TIMX_ADRS[tim].regbase;

    /* TIMx_CR1 */
    drvcmn_setreg16(timx_base + TOFF_TIM_CR1, 0, TIM_CR1_CEN, (enable ? TIM_CR1_CEN : 0));
    DUMP_REG16("TIMx_CR1", TIMX_ADRS[tim].regbase + TOFF_TIM_CR1);
}

// 有効/無効設定
void drvcmn_tim_set_output_enable(int tim, int ch, bool_t enable)
{
    assert((tim >= 1) && (tim <= 14));
    assert((ch >= 1) && (ch <= 6));

    intptr_t timx_base = TIMX_ADRS[tim].regbase;

    /* TIMx_CCER */
    drvcmn_setreg16(timx_base + TOFF_TIM_CCER, (ch - 1) * 4, TIM_CCER_CC1E, (enable ? TIM_CCER_CC1E : 0));
    DUMP_REG16("TIMx_CCER", timx_base + TOFF_TIM_CCER);
}

// 設定を変更
void drvcmn_tim_update_setting(int tim, uint32_t flags, const DRVCMN_TIM_SETTING_T* setting)
{
    assert((tim >= 1) && (tim <= 14));
    assert(setting);
    
    intptr_t timx_base = TIMX_ADRS[tim].regbase;

    if (flags & DRVCMN_TIM_UPDATE_PSC) {
        /* TIMx_PSC */
        drvcmn_setreg16(timx_base + TOFF_TIM_PSC, 0, UINT16_MAX, setting->psc);
        DUMP_REG16("TIMx_PSC", timx_base + TOFF_TIM_PSC);
    }

    if (flags & DRVCMN_TIM_UPDATE_ARR) {
        /* TIMx_ARR */
        drvcmn_setreg16(timx_base + TOFF_TIM_ARR, 0, UINT16_MAX, setting->arr);
        DUMP_REG16("TIMx_ARR", timx_base + TOFF_TIM_ARR);
    }
}

// チャネル設定を変更
void drvcmn_tim_update_ch_setting(int tim, int ch, uint32_t flags, const DRVCMN_TIM_CH_SETTING_T* setting)
{
    assert((tim >= 1) && (tim <= 14));
    assert((ch >= 1) && (ch <= 6));
    assert(setting);

    intptr_t timx_base = TIMX_ADRS[tim].regbase;

    if (flags & DRVCMN_TIM_UPDATE_CCR) {
        /* TIMx_CCRx */
        drvcmn_setreg16(timx_base + TIMX_CH_REGS[ch].ccr_offset, 0, UINT16_MAX, setting->ccr);
        DUMP_REG16("TIMx_CCRx", timx_base + TIMX_CH_REGS[ch].ccr_offset);
    }
}

