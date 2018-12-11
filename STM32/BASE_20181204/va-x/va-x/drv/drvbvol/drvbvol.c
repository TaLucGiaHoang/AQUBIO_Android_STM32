/*
 * VA-X バッテリー電圧計ドライバ
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/11/20 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "drvbvol.h"

#include <kernel.h>
#include <sil.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "stm32f7xx.h"

#include "kernel_cfg.h"
#include "drvcmn.h"
#include "drvcmn_gpio.h"


/* マクロ */

// レジスタダンプ
#if 0
  #define DUMP_REG32(name, addr) syslog(LOG_NOTICE, name " (0x%08p): => 0x%08x", addr, drvcmn_getreg32(addr, 0, UINT32_MAX));
#else
  #define DUMP_REG32(name, addr)
#endif

#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))

// サービスコールのエラーのログ出力
static inline void svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}

/*
 * 型定義
 */

/*
 * 定数
 */

/* GPIOポート(ADC入力有効) */
static const DRVCMN_GPIO_PIN_T GPIO_PIN_ENABLE = {
    DRVCMN_GPIO_PORT_E, 11,	// E11
};

/* GPIOポート(ADC入力ポート) */
static const DRVCMN_GPIO_PIN_T GPIO_PIN_ADCIN = {
    DRVCMN_GPIO_PORT_A, 3,	// PA3 => ADC1_IN3
};

#if 1	// TODO: 電源管理側で管理する
static const DRVCMN_GPIO_PIN_T GPIO_PIN_3V3_ON = {
    DRVCMN_GPIO_PORT_E, 15,	// 3V3_ON (Active H)
};
#endif

/* GPIOピン設定(ADC電圧印加) */
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_ENABLE = {
    .mode = DRVCMN_GPIO_MODE_GPOUT,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLDOWN,
};

/* GPIOピン設定(ADC入力ポート) */
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_ADCIN = {
    .mode = DRVCMN_GPIO_MODE_ANALOG,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLDOWN,
};

/* ADCレジスタベース */
static const intptr_t ADC_BASE = TADR_ADC1_BASE;

/* VREF 電圧 (mv) */
static const uint32_t VREF_MV_2V8 = 2813;	// 2812.5 mv
static const uint32_t VREF_MV_3V3 = 3255;	// 3255 mv

/* 初期化待ち時間 (ms) */
static const uint32_t INITIAL_WAIT = 200;

/* メッセージ */
enum {
    MSG_GET_VOLTAGE = 1,
    MSG_EOC,
    MSG_INITIAL_WAIT,
};

/*
 * 内部変数
 */
static DRVBVOL_CBFUNC_T s_callback = NULL;

/*
 * 内部関数プロトタイプ
 */
void get_voltage();
void conversion_complete(uint32_t data);

/*
 * ペリフェラル初期化
 */
void drvbvol_initialize_peripherals()
{
    /* GPIO ピンの設定 */
    drvcmn_gpio_pin_initialize(&GPIO_PIN_ENABLE, &GPIO_SETTING_ENABLE);	/* ADC電圧印加 */
    drvcmn_gpio_pin_initialize(&GPIO_PIN_ADCIN, &GPIO_SETTING_ADCIN);	/* ADC入力ポート */

    /* ADC電圧印加 */
    drvcmn_gpio_pin_set(&GPIO_PIN_ENABLE, true);

    /* RCC_APB2ENR */
    drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_APB2ENR, 0, RCC_APB2ENR_ADC1EN, ~(uint32_t)0);	// ADC1EN
    DUMP_REG32("RCC_APB2ENR", TADR_RCC_BASE + TOFF_RCC_APB2ENR);

    /* ADC_SQR1 */
    drvcmn_setreg32(ADC_BASE + TOFF_ADC_SQR1, 0, ADC_SQR1_L, 0);	// [23:20]L
    DUMP_REG32("ADC_SQR1", ADC_BASE + TOFF_ADC_SQR1);

    /* ADC_SQR3 */
    drvcmn_setreg32(ADC_BASE + TOFF_ADC_SQR3, 0, ADC_SQR3_SQ1, 3);	// [4:0]SQ1
    DUMP_REG32("ADC_SQR3", ADC_BASE + TOFF_ADC_SQR3);

    /* ADC_CR1 */
    drvcmn_setreg32(ADC_BASE + TOFF_ADC_CR1, 0, ADC_CR1_RES, 0);	// [25:24]RES
    drvcmn_setreg32(ADC_BASE + TOFF_ADC_CR1, 0, ADC_CR1_EOCIE, ~(uint32_t)0);	// [5]EOCIE
    DUMP_REG32("ADC_CR1", ADC_BASE + TOFF_ADC_CR1);
}

/*
 * 初期化
 */
void drvbvol_initialize()
{
    ER er = 0;

    // タスク起動
    er = act_tsk(TSK_DRVBVOL);
    assert(er == E_OK);

    /* 測定前に待ちを入れておく */
    er = snd_dtq(DTQ_DRVBVOL, MSG_INITIAL_WAIT);
    assert(er == E_OK);
}

/*
 * 電圧取得
 */
void drvbvol_get_voltage(DRVBVOL_CBFUNC_T callback)
{
    assert(!s_callback);
    assert(callback);

    /* コールバック関数を設定 */
    s_callback = callback;

    ER ercd = snd_dtq(DTQ_DRVBVOL, MSG_GET_VOLTAGE);
    assert(ercd == E_OK);

}

/* 割込みサービスルーチン */
void drvbvol_adc_isr(intptr_t exinf)
{
    /* TOFF_ADC_DR */
    uint32_t data = drvcmn_getreg32(ADC_BASE + TOFF_ADC_DR, 0, 0x0fff);	/* 12bit */

    uint32_t msg = MSG_EOC;
    msg |= (data << 16);

    SVC_PERROR(ipsnd_dtq(DTQ_DRVBVOL, msg));
}

/* タスク */
void drvbvol_task(intptr_t exinf)
{
	syslog(LOG_NOTICE, "drvbvol_task() starts .");

    while (true) {
        uint32_t msg = 0;
        ER er_rcv = rcv_dtq(DTQ_DRVBVOL, (intptr_t*)&msg);
        assert(er_rcv == E_OK);

        uint32_t msg_id = msg & 0x0000ffff;
        uint32_t ad_data = (msg & 0xffff0000) >> 16;

        /* 長押し */
        switch (msg_id) {
        case MSG_GET_VOLTAGE:	/* 電圧取得 */
            get_voltage();
            break;
        case MSG_EOC:	/* AD変換完了 */
            conversion_complete(ad_data);
            break;
        case MSG_INITIAL_WAIT:	/* 電圧安定待ち */
            dly_tsk(INITIAL_WAIT);
            break;
        default:
            assert(false);
            break;
        }
    }
}

/*
 * 内部関数
 */

/*
 * 電圧取得
 */
void get_voltage()
{
    assert(s_callback);

    /* ADC_CR2 */
    drvcmn_setreg32(ADC_BASE + TOFF_ADC_CR2, 0, ADC_CR2_ADON, ~(uint32_t)0);	// ADON
    DUMP_REG32("ADC_CR2", ADC_BASE + TOFF_ADC_CR2);

    dly_tsk(10);	/* ADC安定待ち */

    drvcmn_setreg32(ADC_BASE + TOFF_ADC_CR2, 0, ADC_CR2_SWSTART, ~(uint32_t)0);	// [30]SWSTART
    DUMP_REG32("ADC_CR2", ADC_BASE + TOFF_ADC_CR2);
}

/*
 * ADC変換完了
 */
void conversion_complete(uint32_t data)
{
    assert(s_callback);

    /* ADC_CR2 */
    drvcmn_setreg32(ADC_BASE + TOFF_ADC_CR2, 0, ADC_CR2_ADON, 0);	// ADON
    DUMP_REG32("ADC_CR2", ADC_BASE + TOFF_ADC_CR2);

#if 1	// TODO: 電源管理で管理
    /* VREF */
    uint32_t vref = drvcmn_gpio_pin_get(&GPIO_PIN_3V3_ON) ? VREF_MV_3V3 : VREF_MV_2V8;
#endif

    /* mv に変換 */
    uint32_t voltage = (data * vref) / 0xfff;

    /* コールバック */
    s_callback(voltage);
    s_callback = NULL;
}

