/*
 * VA-X
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/13 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "drvcmn_gpio.h"

#include <t_syslog.h>
#include <sil.h>
#include <stm32f7xx.h>

#include "drvcmn.h"

/*
 * マクロ定義
 */

// レジスタダンプ
#if 0
  #define DUMP_REG32(name, addr) syslog(LOG_NOTICE, name " (0x%08p): => 0x%08x", addr, drvcmn_getreg32(addr, 0, UINT32_MAX));
#else
  #define DUMP_REG32(name, addr)
#endif


/*
 * 型定義
 */



/*
 * 定数定義
 */

// GPIOXENのポート毎のビットパターン
static const intptr_t GPIOXEN_PATTERNS[] = {
    RCC_AHB1ENR_GPIOAEN,	// [0]
    RCC_AHB1ENR_GPIOBEN,	// [1]
    RCC_AHB1ENR_GPIOCEN,	// [2]
    RCC_AHB1ENR_GPIODEN,	// [3]
    RCC_AHB1ENR_GPIOEEN,	// [4]
    RCC_AHB1ENR_GPIOFEN,	// [5]
    RCC_AHB1ENR_GPIOGEN,	// [6]
    RCC_AHB1ENR_GPIOHEN,	// [7]
    RCC_AHB1ENR_GPIOIEN,	// [8]
    RCC_AHB1ENR_GPIOJEN,	// [9]
    RCC_AHB1ENR_GPIOKEN,	// [10]
};

// ポート毎のレジスタアドレス
static const intptr_t GPIOX_BASE_ADRS[] = {
    TADR_GPIOA_BASE,	// [0]
    TADR_GPIOB_BASE,	// [1]
    TADR_GPIOC_BASE,	// [2]
    TADR_GPIOD_BASE,	// [3]
    TADR_GPIOE_BASE,	// [4]
    TADR_GPIOF_BASE,	// [5]
    TADR_GPIOG_BASE,	// [6]
    TADR_GPIOH_BASE,	// [7]
    TADR_GPIOI_BASE,	// [8]
    TADR_GPIOJ_BASE,	// [9]
    TADR_GPIOK_BASE,	// [10]
};

/*
 * 内部変数
 */

// EXTI のピン毎の ISR
static DRVCMN_EXTI_ISR_T s_exti_isrs[16];

/*
 * 内部関数のプロトタイプ
 */

// exti割り当て
void drvcmn_gpio_assign_exti(const DRVCMN_GPIO_PIN_T* pin, bool_t r_trig, bool_t f_trig, DRVCMN_EXTI_ISR_T exti_isr);

#if 0
/* GPIO のポートを有効化 */
void drvcmn_gpio_port_enable(uint32_t port)
{
    assert(port < (sizeof(GPIOXEN_PATTERNS) / sizeof(GPIOXEN_PATTERNS[0])));
    
    /* RCC_AHB1ENR をセット */
    uint32_t* addr = (uint32_t*)(TADR_RCC_BASE + TOFF_RCC_AHB1ENR);
    uint32_t data = sil_rew_mem(addr);
    sil_wrw_mem(addr, data | GPIOXEN_PATTERNS[port]);

    return;
}
#endif

/* GPIO のピンを初期化 */
void drvcmn_gpio_pin_initialize(const DRVCMN_GPIO_PIN_T* pin, const DRVCMN_GPIO_SETTING_T* setting)
{
    assert(pin && setting);
    assert(pin->port < (sizeof(GPIOX_BASE_ADRS) / sizeof(GPIOX_BASE_ADRS[0])));
    assert(pin->pinno < 16);

    intptr_t base_addr = GPIOX_BASE_ADRS[pin->port];

    /* RCC_AHB1ENR */
    drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_AHB1ENR, 0, GPIOXEN_PATTERNS[pin->port], GPIOXEN_PATTERNS[pin->port]);
    DUMP_REG32("RCC_AHBIENR", TADR_RCC_BASE + TOFF_RCC_AHB1ENR);

    /* GPIOx_MODER */
    drvcmn_setreg32(base_addr + TOFF_GPIO_MODER, pin->pinno * 2, 0x3, setting->mode);
    DUMP_REG32("GPIO_MODER", base_addr + TOFF_GPIO_MODER);

    /* GPIOx_OTYPER */
    drvcmn_setreg32(base_addr + TOFF_GPIO_OTYPER, pin->pinno, 0x1, setting->otype);
    DUMP_REG32("GPIO_OTYPER", base_addr + TOFF_GPIO_OTYPER);

    /* GPIOx_OSPEEDR */
    drvcmn_setreg32(base_addr + TOFF_GPIO_OSPEEDR, pin->pinno * 2, 0x3, setting->ospeed);
    DUMP_REG32("GPIO_OSPEEDR", base_addr + TOFF_GPIO_OSPEEDR);

    /* GPIOx_PUPDR */
    drvcmn_setreg32(base_addr + TOFF_GPIO_PUPDR, pin->pinno * 2, 0x3, setting->pupd);
    DUMP_REG32("GPIO_PUPDR", base_addr + TOFF_GPIO_PUPDR);

    /* EXTI 割当て */
    if (setting->exti != DRVCMN_GPIO_EXTI_NONE) {
        drvcmn_gpio_assign_exti(pin, setting->exti & DRVCMN_GPIO_EXTI_RTRIG, setting->exti & DRVCMN_GPIO_EXTI_FTRIG, setting->exti_isr);
    }

    /* AF 設定 */
    if (setting->mode == DRVCMN_GPIO_MODE_AF) {

        const uint32_t AFR_OFFSETS[] = {
            TOFF_GPIO_AFR0,
            TOFF_GPIO_AFR1,
        };

        drvcmn_setreg32(base_addr + AFR_OFFSETS[pin->pinno / 8], (4 * (pin->pinno % 8)), 0xf, setting->afno);
        DUMP_REG32("GPIO_AFR", base_addr + AFR_OFFSETS[pin->pinno / 8]);
    }

    return;
}

/* GPIO のピンの出力をセット */
void drvcmn_gpio_pin_set(const DRVCMN_GPIO_PIN_T* pin, uint32_t set)
{
    assert(pin);
    assert(pin->port < (sizeof(GPIOX_BASE_ADRS) / sizeof(GPIOX_BASE_ADRS[0])));
    assert(pin->pinno < 16);

    int bit_offset = pin->pinno + (set ? 0 : 16);

    /* GPIOx_BSRR */
    sil_wrw_mem((uint32_t*)(GPIOX_BASE_ADRS[pin->port] + TOFF_GPIO_BSRR), 0x1 << bit_offset);

    return;
}

/* GPIO のピンの入力を取得 */
uint32_t drvcmn_gpio_pin_get(const DRVCMN_GPIO_PIN_T* pin)
{
    assert(pin);
    assert(pin->port < (sizeof(GPIOX_BASE_ADRS) / sizeof(GPIOX_BASE_ADRS[0])));
    assert(pin->pinno < 16);

    /* GPIOx_IDR */
    return drvcmn_getreg32(GPIOX_BASE_ADRS[pin->port] + TOFF_GPIO_IDR, pin->pinno, 0x1);
}

/* EXTI の割込みマスクを設定 */
void drvcmn_gpio_set_exti_mask(uint32_t pinno, bool_t mask)
{
    assert(pinno < 16);

    drvcmn_setreg32(TADR_EXTI_BASE + TOFF_EXTI_IMR, pinno, 0x1, (mask ? 0x0 : 0x1));

    return;
}

/* EXTI の PR をクリア */
void drvcmn_gpio_clear_exti_pr(uint32_t pinno)
{
    assert(pinno < 16);

    sil_wrw_mem((uint32_t*)(TADR_EXTI_BASE + TOFF_EXTI_PR), 0x1 << pinno);

    return;
}

/* 同じピン番号の GPIO は重複して割り当てる事ができない (例: PA0 と PB0 は片方しか使用できない) */
/* 割り込み番号はピンによって決まるため, 自由に設定はできない. */
void drvcmn_gpio_assign_exti(const DRVCMN_GPIO_PIN_T* pin, bool_t r_trig, bool_t f_trig, DRVCMN_EXTI_ISR_T exti_isr)
{
    assert(pin);
    assert(pin->port < (sizeof(GPIOX_BASE_ADRS) / sizeof(GPIOX_BASE_ADRS[0])));
    assert(pin->pinno < 16);

    const intptr_t EXTICR_OFFSETS[] = {
        TOFF_SYSCFG_EXTICR0,
        TOFF_SYSCFG_EXTICR1,
        TOFF_SYSCFG_EXTICR2,
        TOFF_SYSCFG_EXTICR3,
    };

    /* RCC_APB2ENR */
    drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_APB2ENR, 0, RCC_APB2ENR_SYSCFGEN, RCC_APB2ENR_SYSCFGEN);
    DUMP_REG32("RCC_APB2ENR", TADR_RCC_BASE + TOFF_RCC_APB2ENR);

    /* SYSCFG_EXTICRx */
    drvcmn_setreg32(TADR_SYSCFG_BASE + EXTICR_OFFSETS[pin->pinno / 4], ((pin->pinno % 4) * 4), 0xF, pin->port);
    DUMP_REG32("SYSCFG_EXTICRx", TADR_SYSCFG_BASE + EXTICR_OFFSETS[pin->pinno / 4]);

    /* EXTI_IMR */
    drvcmn_setreg32(TADR_EXTI_BASE + TOFF_EXTI_IMR, pin->pinno, 0x1, 0x1);
    DUMP_REG32("EXTI_IMR", TADR_EXTI_BASE + TOFF_EXTI_IMR);

    /* EXTI_RTSR */
    drvcmn_setreg32(TADR_EXTI_BASE + TOFF_EXTI_RTSR, pin->pinno, 0x1, (r_trig ? 1 : 0));
    DUMP_REG32("EXTI_RTSR", TADR_EXTI_BASE + TOFF_EXTI_RTSR);

    /* EXTI_FTSR */
    drvcmn_setreg32(TADR_EXTI_BASE + TOFF_EXTI_FTSR, pin->pinno, 0x1, (f_trig ? 1 : 0));
    DUMP_REG32("EXTI_FTSR", TADR_EXTI_BASE + TOFF_EXTI_FTSR);

    /* ISR を登録 */
    s_exti_isrs[pin->pinno] = exti_isr;

#if 0
    /* 割り込み番号をチェックする */
    uint32_t int_no_check = 0;
    if (pin == 0) {
        int_no_check = IRQ_VECTOR_EXTI0;
    } else if (pin == 1) {
        int_no_check = IRQ_VECTOR_EXTI1;
    } else if (pin == 2) {
        int_no_check = IRQ_VECTOR_EXTI2;
    } else if (pin == 3) {
        int_no_check = IRQ_VECTOR_EXTI3;
    } else if (pin == 4) {
        int_no_check = IRQ_VECTOR_EXTI4;
    } else if (pin <= 9) {
        int_no_check = IRQ_VECTOR_EXTI9_5;
    } else {
        int_no_check = IRQ_VECTOR_EXTI15_10;
    }
    assert(int_no == int_no_check);
#endif

    return;
}

/*
 * 割込みサービスルーチン
 */
void drvcmn_gpio_exti_isr(intptr_t exinf)
{
    int pinno = 0;
    if (exinf >= 0 && exinf <= 4) {	/* 0-4 */
        pinno = exinf;
    } else if (exinf == 5 || exinf == 10) {
        /* 5-9,10-15は割込ベクタが共通なのでEXTI_PRの値でピンを識別する */
        uint32_t exti_pr = drvcmn_getreg32(TADR_EXTI_BASE + TOFF_EXTI_PR, 0, UINT32_MAX);
        int32_t i = 0;
        int32_t end = 0;
        if (exinf == 5) {
            i = 5;
            end = 9;
        } else {
            i = 10;
            end = 16;
        }
        for (; i <= end ; i++) {
            if (exti_pr & (1 << i)) {
                pinno = i;
                break;
            }
        }
    } else {
        assert(false);
    }

    /* ISR呼出 */
    assert(s_exti_isrs[pinno]);
    s_exti_isrs[pinno](pinno);

    /* EXTI_PRビットをクリア */
    drvcmn_gpio_clear_exti_pr(pinno);
}


