#include "drvadc.h"

#include <kernel.h>
#include <sil.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "stm32f7xx.h"

#include "kernel_cfg.h"
#include "drvcmn.h"

#include "drvled.h"

/* マクロ */
#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))

/*
 *  サービスコールのエラーのログ出力
 */
static inline void svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}
#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))

/* GPIOポート */
#define DRVADC_GPIO_PORT DRVCMN_GPIO_PORT_A
#define DRVADC_GPIO_PIN  0

/* 型 */
typedef struct {
    uint32_t count;
    uint32_t prev_count;
    bool_t state;
} HB_STATE_T;

/* 定数 */
#define HB_THRESHOLD ((0x0FFF * 8) / 10)

/* 内部変数 */
static DRVADC_CBFUNC_T s_callback = NULL;
static HB_STATE_T s_hb_state = {0};

/* */
void step_hb_state(uint32_t value);

/*
 * 初期化コード
 */
void drvadc_initialize(intptr_t exinf)
{
    /* GPIO ピンの設定 */
    drvcmn_gpio_pin_initialize(DRVADC_GPIO_PORT, DRVADC_GPIO_PIN, DRVCMN_GPIO_TYPE_AINPUT, NULL);

    uint32_t* addr = NULL;
    uint32_t data = 0;

    /* RCC_APB2ENR */
    addr = (uint32_t*)(TADR_RCC_BASE + TOFF_RCC_APB2ENR);
    data = sil_rew_mem(addr);
    data = data | RCC_APB2ENR_ADC3EN;
    sil_wrw_mem(addr, data);
    syslog(LOG_NOTICE, "RCC_APB2ENR: %p => 0x%08x", addr, data);

    /* ADON */
    addr = (uint32_t*)(TADR_ADC3_BASE + TOFF_ADC_CR2);
    data = sil_rew_mem(addr);
    data = data | ADC_CR2_ADON;
    sil_wrw_mem(addr, data);
    syslog(LOG_NOTICE, "ADC_CR2: %p => 0x%08x", addr, data);

    /* EOCIE */
    addr = (uint32_t*)(TADR_ADC3_BASE + TOFF_ADC_CR1);
    data = sil_rew_mem(addr);
    data = data | ADC_CR1_EOCIE;
    sil_wrw_mem(addr, data);
    syslog(LOG_NOTICE, "ADC_CR1: %p => 0x%08x", addr, data);

    /* ADC_SQR1 */
    addr = (uint32_t*)(TADR_ADC3_BASE + TOFF_ADC_SQR1);
    data = sil_rew_mem(addr);
    /* ADC_SQR1_L => 0 (設定数1) */
    sil_wrw_mem(addr, data);
    syslog(LOG_NOTICE, "ADC_SQR1: %p => 0x%08x", addr, data);

    /* ADC_SQR3 */
    addr = (uint32_t*)(TADR_ADC3_BASE + TOFF_ADC_SQR3);
    data = sil_rew_mem(addr);
    /* ADC_SQR3_SQ1 => 0 (チャネル0) */
    sil_wrw_mem(addr, data);
    syslog(LOG_NOTICE, "ADC_SQR3: %p => 0x%08x", addr, data);

}

/*
 * コールバック登録
 */
void drvadc_set_callback(DRVADC_CBFUNC_T callback)
{
    assert(!s_callback);
    assert(callback);

    s_callback = callback;
}

/*
 * ADC開始
 */
void drvadc_start()
{
    snd_dtq(DRVADC_DTQID, DRVADC_EVT_START);
}

/*
 * ADC停止
 */
void drvadc_stop()
{
    snd_dtq(DRVADC_DTQID, DRVADC_EVT_STOP);
}

/*
 * 割込みサービスルーチン
 */
void drvadc_isr(intptr_t exinf)
{
    /* TOFF_ADC_DR */
    uint32_t data = sil_rew_mem((uint32_t*)(TADR_ADC3_BASE + TOFF_ADC_DR));
    uint32_t addata = (data & 0x0fff);

    uint32_t msg = DRVADC_EVT_EOC;
    msg |= (addata << 16);

    SVC_PERROR(ipsnd_dtq(DRVADC_DTQID, msg));
}

/*
 * 割込み処理タスク
 */
void drvadc_task(intptr_t exinf)
{
	syslog(LOG_NOTICE, "drvadc_task() starts .");

    while (true) {
        uint32_t msg = 0;
        ER er_rcv = rcv_dtq(DRVADC_DTQID, (intptr_t*)&msg);
        assert(er_rcv == E_OK);

        uint32_t msg_id = msg & 0x0000ffff;
        uint32_t ad_data = (msg & 0xffff0000) >> 16;

        /* 長押し */
        switch (msg_id) {
        case DRVADC_EVT_START:	/* スタート */
            syslog(LOG_NOTICE, "drvadc_task() DRVADC_EVT_START");

            sta_cyc(DRVADC_CYC);
            break;
        case DRVADC_EVT_STOP:	/* ストップ */
            syslog(LOG_NOTICE, "drvadc_task() DRVADC_EVT_STOP");
            stp_cyc(DRVADC_CYC);
            break;
        case DRVADC_EVT_EOC:	/* AD変換完了 */
            //syslog(LOG_NOTICE, "drvadc_task() DRVADC_EVT_EOC (%d)", ad_data);
            step_hb_state(ad_data);
            break;
        default:
            assert(false);
            break;
        }
    }
}

/* 周期ハンドラ */
void drvadc_cychdr(intptr_t exinf)
{
	//syslog(LOG_NOTICE, "drvadc_cychdr() starts .");

    /* ADON */
    uint32_t* addr = (uint32_t*)(TADR_ADC3_BASE + TOFF_ADC_CR2);
    uint32_t data = sil_rew_mem(addr);
    data = (data | ADC_CR2_SWSTART);
    sil_wrw_mem(addr, data);
}

/*
 * 内部関数
 */
void step_hb_state(uint32_t value)
{
    bool_t next_state;
    if (value > HB_THRESHOLD) {
        next_state = true;
    } else {
        next_state = false;
    }

    drvled_set(0, next_state);

    uint32_t timediff = 0;
    if ((!s_hb_state.state) && (next_state)) {	/* low => high に変化 */
        timediff = (s_hb_state.count - s_hb_state.prev_count) * DRVADC_SAMPLING_INTERVAL;
        syslog(LOG_NOTICE, "HEART RATE: interval: %d(ms), bpm: %d", timediff, (60 * 1000) / timediff);

        s_hb_state.prev_count = s_hb_state.count;
    }

    s_hb_state.state = next_state;
    s_hb_state.count++;
}

