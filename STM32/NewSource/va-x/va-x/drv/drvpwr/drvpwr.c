/*
 * VA-X 電源関係共通処理
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/06/27 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "drvpwr.h"

#include <kernel.h>
#include <sil.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "kernel_cfg.h"
#include "stm32f7xx.h"
#include "drvcmn.h"
#include "drvcmn_gpio.h"

/*
 * マクロ定義
 */

// デバッグログ
#if 0
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[DRVPWR]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[DRVPWR]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[DRVPWR]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[DRVPWR]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

/*
 * 型定義
 */
// バックアップSRAM構成
typedef union {
    struct {
        ;
    } data;
    uint32_t padding[4*1024/sizeof(uint32_t)];
} BKPSRAM_T;
static_assert(sizeof(BKPSRAM_T) == 4*1024, "sizeof(BKPSRAM_T)");

/*
 * 内部関数
 */
// バックアップSRAMアドレス
static BKPSRAM_T* const s_bkpsram = (BKPSRAM_T*)BKPSRAM_BASE;

/*
 * 定数定義
 */

#define RTC_CLOCK_HZ ((uint32_t)0x8000)		// RTCクロック
#define RTC_CK_APRE_HZ ((uint32_t)0x100)	// CK_APRE
#define RTC_WUT_PRES ((uint32_t)16)			// WakeUpTimerのプリスケーラ(注: WUCKSELの設定による)

// 3V3_ON (Active H)
static const DRVCMN_GPIO_PIN_T GPIO_PIN_3V3_ON = { DRVCMN_GPIO_PORT_E, 15 };
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_3V3_ON = {
    .mode = DRVCMN_GPIO_MODE_GPOUT,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLDOWN,
};

// SVCC_ON (Active H)
static const DRVCMN_GPIO_PIN_T	GPIO_PIN_SVCC_ON = { DRVCMN_GPIO_PORT_D, 2 };
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_SVCC_ON = {
    .mode = DRVCMN_GPIO_MODE_GPOUT,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLDOWN,
};

// TS & FLASH_ON (Active L)
static const DRVCMN_GPIO_PIN_T	GPIO_PIN_TSVC_ON = { DRVCMN_GPIO_PORT_D, 8 };
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_TSVC_ON = {
    .mode = DRVCMN_GPIO_MODE_GPOUT,
    .otype = DRVCMN_GPIO_OTYPE_OD,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLUP,
};

/* GPIOポート(ADC入力有効) */
static const DRVCMN_GPIO_PIN_T GPIO_PIN_ENABLE = { DRVCMN_GPIO_PORT_E, 11 };
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_ENABLE = {
    .mode = DRVCMN_GPIO_MODE_GPOUT,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLDOWN,
};


/* GPIOポート(ADC入力ポート) */
static const DRVCMN_GPIO_PIN_T GPIO_PIN_ADCIN = { DRVCMN_GPIO_PORT_A, 3 };
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_ADCIN = {
    .mode = DRVCMN_GPIO_MODE_ANALOG,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLDOWN,
};

// Wakeupピン設定
typedef struct {
    int			wakeup_pin;
    uint32_t	wupp_bit;	// Polality
    uint32_t	cwupf_bit;	// Clear
    uint32_t	ewup_bit;	// Enable
    uint32_t	wupf_bit;	// Flag
} WAKEUP_PIN_REG_T;
const static WAKEUP_PIN_REG_T WAKEUP_PIN_REGS[] = {
    // wakeup_pin,			wupp_bit,		cwupf_bit,		ewup_bit,		wupf_bit
    { DRVPWR_WUPIN_NONE,	0,				0,				0,				0,				},
    { DRVPWR_WUPIN_1_PA0,	PWR_CR2_WUPP1,	PWR_CR2_CWUPF1,	PWR_CSR2_EWUP1,	PWR_CSR2_WUPF1,	},
    { DRVPWR_WUPIN_2_PA2,	PWR_CR2_WUPP2,	PWR_CR2_CWUPF2,	PWR_CSR2_EWUP2,	PWR_CSR2_WUPF2,	},
    { DRVPWR_WUPIN_3_PC1,	PWR_CR2_WUPP3,	PWR_CR2_CWUPF3,	PWR_CSR2_EWUP3,	PWR_CSR2_WUPF3,	},
    { DRVPWR_WUPIN_4_PC13,	PWR_CR2_WUPP4,	PWR_CR2_CWUPF4,	PWR_CSR2_EWUP4,	PWR_CSR2_WUPF4,	},
    { DRVPWR_WUPIN_5_PI8,	PWR_CR2_WUPP5,	PWR_CR2_CWUPF5,	PWR_CSR2_EWUP5,	PWR_CSR2_WUPF5,	},
    { DRVPWR_WUPIN_6_PI11,	PWR_CR2_WUPP6,	PWR_CR2_CWUPF6,	PWR_CSR2_EWUP6,	PWR_CSR2_WUPF6,	},
};

// 動作電圧
enum {
    REQV_NONE = 0,	// 両方可
    REQV_2V8,		// 2.8vのみ可
    REQV_3V3,		// 3.3vのみ可
};

// デバイスの使用条件
typedef struct {
    int device;		// デバイス
    bool_t svcc;	// 要SVCC
    int mcu_reqv;	// マイコン電圧
} DEVICE_REQUIREMENT_T;

static const DEVICE_REQUIREMENT_T DEVICE_REQUIREMENT[] = {
    // device,			svcc,		mcu_reqv
    { DRVPWR_CAM,		false,		REQV_2V8,	},
    { DRVPWR_IRLED,		true,		REQV_NONE,	},
    { DRVPWR_ICC,		true,		REQV_3V3,	},
    { DRVPWR_WIFI,		true,		REQV_3V3,	},
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
    MSG_NONE = 0,
    MSG_GET_VOLTAGE,
    MSG_EOC,
};

/*
 * 内部変数
 */

// コンテキスト
static struct {
    bool_t in_use[DRVPWR_NUM_DEVS];
    bool_t wakeup_flags[DRVPWR_WUPIN_NUM];
    bool_t rtcwakeup_flag;
    bool_t use_2v8;
    DRVPWR_CBFUNC_T callback;
} s_context;

/*
 * 内部関数
 */
static void initialize_peripherals_adc();
static void initialize_backup_domain();
static void get_bat_voltage();
static void conversion_complete(uint32_t data);

/* ===================================================================
 * 公開関数
 * ================================================================ */
/* 
 * ペリフェラル初期化
 */
void drvpwr_initialize_peripherals()
{
    // バックアップドメイン初期化
    initialize_backup_domain();

    // Wakeup 要因を取得
    for (int i = 1 /* 1はじまり */; i < DRVPWR_WUPIN_NUM; i++) {
        s_context.wakeup_flags[i] = (drvcmn_getreg32(TADR_PWR_BASE + TOFF_PWR_CSR2, 0, WAKEUP_PIN_REGS[i].wupf_bit) ? true : false);
        // Wakeup ピン無効
        drvcmn_setreg32(TADR_PWR_BASE + TOFF_PWR_CSR2, 0, WAKEUP_PIN_REGS[i].ewup_bit, 0);	// EWUPn

        // レジスタクリア
        drvcmn_setreg32(TADR_PWR_BASE + TOFF_PWR_CR2, 0, WAKEUP_PIN_REGS[i].cwupf_bit, UINT32_MAX);
    }

    // Wakeup 要因(RTC)を取得
    s_context.rtcwakeup_flag = (drvcmn_getreg32(TADR_RTC_BASE + TOFF_RTC_ISR, 0, RTC_ISR_WUTF) ? true : false);

    // RTCレジスタのライトプロテクト解除
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_WPR, 0, UINT32_MAX, 0xCA);
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_WPR, 0, UINT32_MAX, 0x53);

    // WUTF フラグをクリア
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_ISR, 0, RTC_ISR_WUTF, 0);
    
    // WakeUpTimer無効化
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_CR, 0, RTC_CR_WUTE, 0);

    // ライトプロテクト解除を戻す
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_WPR, 0, UINT32_MAX, 0x0);

    // 3V3_ON と SVCC_ON のピン設定
    drvcmn_gpio_pin_initialize(&GPIO_PIN_3V3_ON, &GPIO_SETTING_3V3_ON);
    drvcmn_gpio_pin_initialize(&GPIO_PIN_SVCC_ON, &GPIO_SETTING_SVCC_ON);
    drvcmn_gpio_pin_initialize(&GPIO_PIN_TSVC_ON, &GPIO_SETTING_TSVC_ON);
    
    // 3V3_ON と SVCC_ON を初期状態に設定
    drvcmn_gpio_pin_set(&GPIO_PIN_3V3_ON, true);
    s_context.use_2v8 = false;
    drvcmn_gpio_pin_set(&GPIO_PIN_SVCC_ON, true);	// 常にActive
    drvcmn_gpio_pin_set(&GPIO_PIN_TSVC_ON, false);	// 常にActive "L"
    
    // バッテリー電圧取得用ADC初期化
    initialize_peripherals_adc();
}

/*
 * ドライバ初期化
 */
void drvpwr_initialize()
{
    ER er = 0;

    // タスク起動
    er = act_tsk(TSK_DRVPWR);
    assert(er == E_OK);
}

/*
 * Wakeup要因取得
 */
int drvpwr_get_wakeup_cause()
{
    int cause = DRVPWR_WUPIN_NONE;
    // Wakeup ピン
    for (int i = 1 /* 1はじまり */; i < DRVPWR_WUPIN_NUM; i++) {
        if (s_context.wakeup_flags[i]) {
            cause = i;
            goto end;
        }
    }

    // RTC
    if (s_context.rtcwakeup_flag) {
        cause = DRVPWR_WURTC;
        goto end;
    }

end:
    return cause;
}

/*
 * Wakeup ピン有効
 */
void drvpwr_enable_wakeup(int pin, int polarity)
{
    DBGLOG0("Entering enable_wakeup...");
    assert((pin > DRVPWR_WUPIN_NONE) && (pin < DRVPWR_WUPIN_NUM));

    // WAKEUP ピン有効
    drvcmn_setreg32(TADR_PWR_BASE + TOFF_PWR_CSR2, 0, WAKEUP_PIN_REGS[pin].ewup_bit, UINT32_MAX);	// EWUPn
    while (drvcmn_getreg32(TADR_PWR_BASE + TOFF_PWR_CSR2, 0, WAKEUP_PIN_REGS[pin].ewup_bit) == 0);	// 反映されるまで待つ

    DBGLOG0("wakeup...set1");
    // Polarity 設定
    if (polarity) {
        drvcmn_setreg32(TADR_PWR_BASE + TOFF_PWR_CR2, 0, WAKEUP_PIN_REGS[pin].wupp_bit, UINT32_MAX);	// WUPPn
        while (drvcmn_getreg32(TADR_PWR_BASE + TOFF_PWR_CR2, 0, WAKEUP_PIN_REGS[pin].wupp_bit) == 0);	// 反映されるまで待つ
    } else {
        drvcmn_setreg32(TADR_PWR_BASE + TOFF_PWR_CR2, 0, WAKEUP_PIN_REGS[pin].wupp_bit, 0);	// WUPPn
        while (drvcmn_getreg32(TADR_PWR_BASE + TOFF_PWR_CR2, 0, WAKEUP_PIN_REGS[pin].wupp_bit) != 0);	// 反映されるまで待つ
    }
    DBGLOG0("wakeup...set2");
    // WAKEUP フラグをクリア
    drvcmn_setreg32(TADR_PWR_BASE + TOFF_PWR_CR2, 0, WAKEUP_PIN_REGS[pin].cwupf_bit, UINT32_MAX);	// CWUPFn
    while (drvcmn_getreg32(TADR_PWR_BASE + TOFF_PWR_CR2, 0, WAKEUP_PIN_REGS[pin].cwupf_bit) != 0);	// 反映されるまで待つ
    DBGLOG0("wakeup...set3");
}

/* 
 * Standbyモード遷移
 */
void drvpwr_enter_standby_mode()
{
    DBGLOG0("Entering standby...");

    // SLEEPDEEP => 1 (ARM System Control Register)
    drvcmn_setreg32(0xE000ED10 /* SCR */, 2, 0x1, 0x1);	// [2]SLEEPDEEP

    // PDDS => 1 (PWR_CR1)
    drvcmn_setreg32(TADR_PWR_BASE + TOFF_PWR_CR1, 0, PWR_CR1_PDDS, UINT32_MAX);	// [1]PDDS

    while (1) {
        // WFI
        __asm volatile ("wfi");
    }

}

/*
 * RTC 時刻取得
 */
void drvpwr_rtc_get(DRVPWR_RTC_DATETIME_T* datetime)
{
    assert(datetime);
    
    // RTC_SSR
    uint32_t ssr = drvcmn_getreg32(TADR_RTC_BASE + TOFF_RTC_SSR, 0, 0xFFFF);
    // RTC_TR
    uint32_t tr = drvcmn_getreg32(TADR_RTC_BASE + TOFF_RTC_TR, 0, UINT32_MAX);
    // RTC_DR
    uint32_t dr = drvcmn_getreg32(TADR_RTC_BASE + TOFF_RTC_DR, 0, UINT32_MAX);

    datetime->year = ((dr >> 20) & 0xf) * 10 + ((dr >> 16) & 0xf);	// YT * 10 + YU
    datetime->month = ((dr >> 12) & 0x1) * 10 + ((dr >> 8) & 0xf);	// MT * 10 + MU
    datetime->day = ((dr >> 4) & 0x3) * 10 + ((dr >> 0) & 0xf);		// DT * 10 + DU
    datetime->weekday = ((dr >> 13) & 0x7);							// WDU

    datetime->hour = ((tr >> 20) & 0x3) * 10 + ((tr >> 16) & 0xf);	// HT * 10 + HU
    datetime->minute = ((tr >> 12) & 0x7) * 10 + ((tr >> 8) & 0xf);	// MNT * 10 + MNU
    datetime->second = ((tr >> 4) & 0x7) * 10 + ((tr >> 0) & 0xf);	// ST * 10 + SU;

    datetime->msecond = ((RTC_CK_APRE_HZ - ssr) * 1000) / RTC_CK_APRE_HZ;
}

/*
 * RTC 時刻設定
 */
void drvpwr_rtc_set(const DRVPWR_RTC_DATETIME_T* datetime)
{
    assert(datetime);

    // RTCレジスタのライトプロテクト解除
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_WPR, 0, UINT32_MAX, 0xCA);
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_WPR, 0, UINT32_MAX, 0x53);

    // シャドウレジスタをバイパスする(注意: RTCのシャドウレジスタ関係でエラッタあり)
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_CR, 0, RTC_CR_BYPSHAD, UINT32_MAX);

    // INIT ビットを立てて初期化モードに設定
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_ISR, 0, RTC_ISR_INIT, UINT32_MAX);

    // 初期化モードに入るのを待つ(INITF ビット)
    while (drvcmn_getreg32(TADR_RTC_BASE + TOFF_RTC_ISR, 0, RTC_ISR_INITF) == 0);

    // 時刻を設定
    // RTC_DR
    uint32_t regval = 0;
    regval =
        ((((datetime->year % 100) / 10) & 0xf) << 20)	|	// YT
        (((datetime->year % 10) & 0xf) << 16)			|	// YU
        ((((datetime->month % 100) / 10) & 0x1) << 12)	|	// MT
        (((datetime->month % 10) & 0xf) << 8)			|	// MU
        ((((datetime->day % 100) / 10) & 0x3) << 4)		|	// DT
        (((datetime->day % 10) & 0xf) << 0);				// DU
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_DR, 0, 0xffff3f, regval);

    // RTC_TR
    regval =
        ((((datetime->hour % 100) / 10) & 0x3) << 20)	|	// HT
        (((datetime->hour % 10) & 0xf) << 16)			|	// HU
        ((((datetime->minute % 100) / 10) & 0x7) << 12)	|	// MNT
        (((datetime->minute % 10) & 0xf) << 8)			|	// MNU
        ((((datetime->second % 100) / 10) & 0x7) << 4)	|	// ST
        (((datetime->second % 10) & 0xf) << 0);				// SU
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_TR, 0, 0x7f7f7f, regval);

    // 初期化モードを解除
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_ISR, 0, RTC_ISR_INIT, 0);

    // ライトプロテクトを有効に戻す
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_WPR, 0, UINT32_MAX, 0x0);
}

/*
 * RTC WakeUpTimer(WUT)設定
 */
void drvpwr_rtc_set_wakeup(uint32_t wakeup_after_ms)
{
    // 32secが最長
    assert((wakeup_after_ms * RTC_CLOCK_HZ) / (1000 * RTC_WUT_PRES) <= 0xffff);

    // RTCレジスタのライトプロテクト解除
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_WPR, 0, UINT32_MAX, 0xCA);
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_WPR, 0, UINT32_MAX, 0x53);
    
    // WUT無効化
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_CR, 0, RTC_CR_WUTE, 0);

    // レジスタが書込み可能になるのを待つ
    while (drvcmn_getreg32(TADR_RTC_BASE + TOFF_RTC_ISR, 0, RTC_ISR_WUTWF) == 0);
    
    // プリスケーラを16に設定
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_CR, 0, RTC_CR_WUCKSEL, 0);

    // RTC_WUTR
    uint32_t wakeup_count = (wakeup_after_ms * RTC_CLOCK_HZ) / (1000 * RTC_WUT_PRES);
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_WUTR, 0, 0xffff, wakeup_count);

    // WUTF フラグをクリア
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_ISR, 0, RTC_ISR_WUTF, 0);

    // 割込み有効
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_CR, 0, RTC_CR_WUTIE, UINT32_MAX);

    // WUT有効化
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_CR, 0, RTC_CR_WUTE, UINT32_MAX);

    // ライトプロテクト解除を戻す
    drvcmn_setreg32(TADR_RTC_BASE + TOFF_RTC_WPR, 0, UINT32_MAX, 0x0);
}

/*
 * デバイス使用設定
 * (各デバイス専用の電源ON/OFFはそれぞれのデバイスドライバで制御する)
 */
void drvpwr_use_device(int device, bool_t use)
{
    assert(device >= 0 && device < DRVPWR_NUM_DEVS);
    assert(DEVICE_REQUIREMENT[device].device == device);

    // 2.8Vと3.3Vが同時使用になっていないかチェックする
    if (use) {
        int mcu_reqv = DEVICE_REQUIREMENT[device].mcu_reqv;
        if (mcu_reqv != REQV_NONE) {
            for (int i = 0; i < DRVPWR_NUM_DEVS; i++) {
                if (s_context.in_use[i] &&
                    DEVICE_REQUIREMENT[i].mcu_reqv != REQV_NONE &&
                    DEVICE_REQUIREMENT[i].mcu_reqv != DEVICE_REQUIREMENT[device].mcu_reqv) {
                    // 同時には使えないデバイスを使おうとしている!
                    DBGLOG2("Illegal Device Use (%d and %d)", i, device);
                    assert(!"Illegal Device Use");
                }
            }
        }
    }

    // 使用設定
    s_context.in_use[device] = use;

    // マイコンの2.8v/3.3vを設定
    bool_t use_2v8 = false;
    for (int i = 0; i < DRVPWR_NUM_DEVS; i++) {
        if (s_context.in_use[i]) {
            if (DEVICE_REQUIREMENT[i].mcu_reqv == REQV_2V8) {
                use_2v8 = true;
            }
        }
    }

    // 3V3設定(Active H)
    DBGLOG1("3V3 => %d", (use_2v8 ? false : true));
    drvcmn_gpio_pin_set(&GPIO_PIN_3V3_ON, (use_2v8 ? false : true));
    s_context.use_2v8 = use_2v8;
}

/*
 * 電圧取得
 */
void drvpwr_get_bat_voltage(DRVPWR_CBFUNC_T callback)
{
    assert(!s_context.callback);
    assert(callback);

    /* コールバック関数を設定 */
    s_context.callback = callback;

    ER ercd = snd_dtq(DTQ_DRVPWR, MSG_GET_VOLTAGE);
    assert(ercd == E_OK);
}

/*
 * ADC割込みサービスルーチン
 */
void drvpwr_adc_isr(intptr_t exinf)
{
    /* TOFF_ADC_DR */
    uint32_t data = drvcmn_getreg32(ADC_BASE + TOFF_ADC_DR, 0, 0x0fff);	/* 12bit */

    uint32_t msg = MSG_EOC;
    msg |= (data << 16);

    ER er = ipsnd_dtq(DTQ_DRVPWR, msg);
    assert(er == E_OK);
}

/*
 * タスク
 */
void drvpwr_task(intptr_t exinf)
{
    while (true) {
        uint32_t msg = 0;
        ER er_rcv = rcv_dtq(DTQ_DRVPWR, (intptr_t*)&msg);
        assert(er_rcv == E_OK);

        uint32_t msg_id = msg & 0x0000ffff;
        uint32_t ad_data = (msg & 0xffff0000) >> 16;

        /* 長押し */
        switch (msg_id) {
        case MSG_GET_VOLTAGE:	/* 電圧取得 */
            get_bat_voltage();
            break;
        case MSG_EOC:	/* AD変換完了 */
            conversion_complete(ad_data);
            break;
        default:
            assert(false);
            break;
        }
    }
}

/* ===================================================================
 * 内部関数
 * ================================================================ */

/*
 * ADCペリフェラル初期化
 */
void initialize_peripherals_adc()
{
    /* GPIO ピンの設定 */
    drvcmn_gpio_pin_initialize(&GPIO_PIN_ENABLE, &GPIO_SETTING_ENABLE);	/* ADC電圧印加 */
    drvcmn_gpio_pin_initialize(&GPIO_PIN_ADCIN, &GPIO_SETTING_ADCIN);	/* ADC入力ポート */

    /* ADC入力有効 */
    drvcmn_gpio_pin_set(&GPIO_PIN_ENABLE, true);

    /* RCC_APB2ENR */
    drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_APB2ENR, 0, RCC_APB2ENR_ADC1EN, UINT32_MAX);	// ADC1EN

    /* ADC_SQR1 */
    drvcmn_setreg32(ADC_BASE + TOFF_ADC_SQR1, 0, ADC_SQR1_L, 0);	// [23:20]L

    /* ADC_SQR3 */
    drvcmn_setreg32(ADC_BASE + TOFF_ADC_SQR3, 0, ADC_SQR3_SQ1, 3);	// [4:0]SQ1

    /* ADC_CR1 */
    drvcmn_setreg32(ADC_BASE + TOFF_ADC_CR1, 0, ADC_CR1_RES, 0);	// [25:24]RES
    drvcmn_setreg32(ADC_BASE + TOFF_ADC_CR1, 0, ADC_CR1_EOCIE, UINT32_MAX);	// [5]EOCIE
}

/*
 * バックアップドメイン初期化
 */
void initialize_backup_domain()
{
    drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_APB1ENR, 0, RCC_APB1ENR_PWREN, UINT32_MAX);

    drvcmn_setreg32(TADR_PWR_BASE + TOFF_PWR_CR1, 0, PWR_CR1_DBP, UINT32_MAX);

    // BKPSRAM のインターフェースを有効にする
    drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_AHB1ENR, 0, RCC_AHB1ENR_BKPSRAMEN, UINT32_MAX);

    uint32_t bdcr = drvcmn_getreg32(TADR_RCC_BASE + TOFF_RCC_BDCR, 0, RCC_BDCR_RTCSEL | RCC_BDCR_LSEON | RCC_BDCR_RTCEN);
    if (bdcr != (RCC_BDCR_RTCSEL_0 | RCC_BDCR_LSEON | RCC_BDCR_RTCEN)) {

        drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_BDCR, 0, RCC_BDCR_BDRST, RCC_BDCR_BDRST);
        drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_BDCR, 0, RCC_BDCR_BDRST, 0);
        drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_BDCR, 0, RCC_BDCR_RTCSEL, RCC_BDCR_RTCSEL_0);
        drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_BDCR, 0, RCC_BDCR_LSEON, RCC_BDCR_LSEON);
        drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_BDCR, 0, RCC_BDCR_RTCEN, RCC_BDCR_RTCEN);
    }
}

/*
 * 電圧取得
 */
void get_bat_voltage()
{
    /* ADC_CR2 */
    drvcmn_setreg32(ADC_BASE + TOFF_ADC_CR2, 0, ADC_CR2_ADON, UINT32_MAX);	// ADON => 1
    dly_tsk(10);	/* ADC安定待ち */

    drvcmn_setreg32(ADC_BASE + TOFF_ADC_CR2, 0, ADC_CR2_SWSTART, UINT32_MAX);	// [30]SWSTART
}

/*
 * ADC変換完了
 */
void conversion_complete(uint32_t data)
{
    /* ADC_CR2 */
    drvcmn_setreg32(ADC_BASE + TOFF_ADC_CR2, 0, ADC_CR2_ADON, 0);	// ADON => 0

    /* VREF */
    uint32_t vref = s_context.use_2v8 ? VREF_MV_2V8 : VREF_MV_3V3 ;

    /* mv に変換 */
    uint32_t voltage = (data * vref) / 0xfff;

    /* コールバック */
    s_context.callback(DRVPWR_EVT_BAT_VOLTAGE, voltage);
    s_context.callback = NULL;
}

