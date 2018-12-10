/*
 * VA-X UART共通処理
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/11/10 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "drvcmn_uart.h"

#include <string.h>

#include <t_syslog.h>
#include <sil.h>
#include <kernel.h>
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

#define NUM_UART 8

/*
 * 型定義
 */

// UART個別のレジスタ
typedef struct {
    intptr_t rccenr_off;
    uint32_t rccen_bit;
    intptr_t uart_base;
    int dckcfgr_bitoff;
    int irqnum;
} UART_REG_T;

// コンテキスト
typedef struct {
    DRVCMN_UART_ISR_CALLBACK_T	callback;	// コールバック
    const uint8_t*				tx_buf;		// 送信バッファ
    size_t						tx_size;	// 送信サイズ
    size_t						tx_count;	// 送信済みカウント
    uint8_t*					rx_buf;		// 受信バッファ
    size_t						rx_size;	// 受信サイズ
    size_t						rx_count;	// 受信済みカウント
} UART_CONTEXT_T;

/*
 * 定数定義
 */

// UARTレジスタ
static const UART_REG_T UART_REGS[] = {
    { 0,				0,						0,					0,	0					},	// [0]なし
    { TOFF_RCC_APB2ENR,	RCC_APB2ENR_USART1EN,	TADR_USART1_BASE,	0,	IRQ_VECTOR_USART1	},	// [1]USART1
    { TOFF_RCC_APB1ENR,	RCC_APB1ENR_USART2EN,	TADR_USART2_BASE,	2,	IRQ_VECTOR_USART2	},	// [2]USART2
    { TOFF_RCC_APB1ENR,	RCC_APB1ENR_USART3EN,	TADR_USART3_BASE,	4,	IRQ_VECTOR_USART3	},	// [3]USART3
    { TOFF_RCC_APB1ENR,	RCC_APB1ENR_UART4EN,	TADR_UART4_BASE,	6,	IRQ_VECTOR_UART4	},	// [4]UART4
    { TOFF_RCC_APB1ENR,	RCC_APB1ENR_UART5EN,	TADR_UART5_BASE,	8,	IRQ_VECTOR_UART5	},	// [5]UART5
    { TOFF_RCC_APB2ENR,	RCC_APB2ENR_USART6EN,	TADR_USART6_BASE,	10,	IRQ_VECTOR_USART6	},	// [6]USART6
    { TOFF_RCC_APB1ENR,	RCC_APB1ENR_UART7EN,	TADR_UART7_BASE,	12,	IRQ_VECTOR_UART7	},	// [7]UART7
    { TOFF_RCC_APB1ENR,	RCC_APB1ENR_UART8EN,	TADR_UART8_BASE,	14,	IRQ_VECTOR_UART8	},	// [8]UART8
};
static_assert(sizeof(UART_REGS)/sizeof(UART_REGS[0]) == NUM_UART + 1, "size UART_REGS");

/*
 * 内部変数
 */

// コンテキスト
static UART_CONTEXT_T s_contexts[NUM_UART + 1];


/*
 * 内部関数のプロトタイプ
 */


/*
 * 公開関数
 */


/*
 * ペリフェラル初期化
 */
void drvcmn_uart_initialize(int uart, const DRVCMN_UART_SETTING_T* setting)
{
    assert((uart >= 1) && (uart <= NUM_UART));
    assert(setting);

    // コンテキストをクリア
    UART_CONTEXT_T* context = &(s_contexts[uart]);
    memset(context, 0, sizeof(UART_CONTEXT_T));

    // コールバックを保持
    context->callback = setting->callback;

    // RCC ENR
    drvcmn_setreg32(TADR_RCC_BASE + UART_REGS[uart].rccenr_off, 0, UART_REGS[uart].rccen_bit, ~(uint32_t)0);
    DUMP_REG32("RCC_APBNENR", TADR_RCC_BASE + UART_REGS[uart].rccenr_off);

    // DCKCFGR2
    drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_DCKCFGR2, UART_REGS[uart].dckcfgr_bitoff, 0x3, setting->clocksel);
    DUMP_REG32("RCC_DCKCFGR2", TADR_RCC_BASE + TOFF_RCC_DCKCFGR2);

    // UARTレジスタベースアドレス
    intptr_t uart_base = UART_REGS[uart].uart_base;

    // USART_CR1
    uint32_t cr1_m = ((setting->wordlen & 0x1) ? USART_CR1_M_0 : 0) | ((setting->wordlen & 0x2) ? USART_CR1_M_1 : 0);
    drvcmn_setreg32(uart_base + TOFF_USART_CR1, 0, USART_CR1_M, cr1_m);	// [28]M1, [12]M0
    drvcmn_setreg32(uart_base + TOFF_USART_CR1, 9, 0x3, setting->parity);	// [10]PCE, [9]PS
    drvcmn_setreg32(uart_base + TOFF_USART_CR1, 0, (USART_CR1_RE | USART_CR1_TE), ~(uint32_t)0);	// [3]TE, [2]RE

    DUMP_REG32("USART_CR1", uart_base + TOFF_USART_CR1);

    // USART_CR2
    drvcmn_setreg32(uart_base + TOFF_USART_CR2, 12, 0x3, setting->stop);	// [13:12]STOP
    DUMP_REG32("USART_CR2", uart_base + TOFF_USART_CR2);
    
    // USART_CR3
    drvcmn_setreg32(uart_base + TOFF_USART_CR3, 8, 0x3, (setting->hwflow ? ~(uint32_t)0 : 0 ));	// [9]CTSE,[8]RTSE
    DUMP_REG32("USART_CR3", uart_base + TOFF_USART_CR3);

    // USART_BRR
    uint32_t brr = 0;
    switch (setting->clocksel) {
    case DRVCMN_UART_CLOCKSEL_APB:
        if ((uart == 1) || (uart == 6)) {	// APB2 (PCLK2)
            brr = 108000000 / setting->baudrate;
        } else {							// APB1 (PCLK1)
            brr = 54000000 / setting->baudrate;
        }
        break;
    case DRVCMN_UART_CLOCKSEL_SYS:
        brr = 216000000 / setting->baudrate;
        break;
    case DRVCMN_UART_CLOCKSEL_HSI:
        brr = 16000000 / setting->baudrate;
        break;
    case DRVCMN_UART_CLOCKSEL_LSE:
        brr = 32768 / setting->baudrate;
        break;
    default:
        assert(false);
        break;
    }
    assert(brr <= 0xffff);
    drvcmn_setreg32(uart_base + TOFF_USART_BRR, 0, 0xffff, brr);
    DUMP_REG32("USART_BRR", uart_base + TOFF_USART_BRR);

    // 割込み有効
    drvcmn_setreg32(uart_base + TOFF_USART_ICR, 0, USART_ICR_ORECF | USART_ICR_NCF | USART_ICR_FECF | USART_ICR_PECF, ~(uint32_t)0);
    ER er = ena_int(UART_REGS[uart].irqnum);
    assert(er == E_OK);

    // UART有効
    drvcmn_setreg32(uart_base + TOFF_USART_CR1, 0, USART_CR1_UE, ~(uint32_t)0);	// [0]UE
    DUMP_REG32("USART_CR1", uart_base + TOFF_USART_CR1);
}

void drvcmn_uart_send(int uart, const uint8_t* data, size_t size)
{
    assert((uart >= 1) && (uart <= NUM_UART));
    assert(data);
    assert(size > 0);

    UART_CONTEXT_T* context = &(s_contexts[uart]);
    assert(!context->tx_buf);
    assert(context->tx_size == 0);

    // 送信データとサイズを保持
    context->tx_buf = data;
    context->tx_size = size;
    context->tx_count = 0;

    // 割込を有効化
    drvcmn_setreg32(UART_REGS[uart].uart_base + TOFF_USART_CR1, 0, USART_CR1_TXEIE, ~(uint32_t)0);
}

void drvcmn_uart_receive(int uart, uint8_t* buf, size_t size)
{
    assert((uart >= 1) && (uart <= NUM_UART));
    //assert(buf);
    assert(size > 0);

    UART_CONTEXT_T* context = &(s_contexts[uart]);
    assert(!context->rx_buf);
    assert(context->rx_size == 0);

    // 送信データとサイズを保持
    context->rx_buf = buf;
    context->rx_size = size;
    context->rx_count = 0;

    // 割込を有効化
    drvcmn_setreg32(UART_REGS[uart].uart_base + TOFF_USART_CR3, 0, USART_CR3_EIE, ~(uint32_t)0);
    drvcmn_setreg32(UART_REGS[uart].uart_base + TOFF_USART_CR1, 0, USART_CR1_PEIE | USART_CR1_RXNEIE, ~(uint32_t)0);
 }

size_t drvcmn_uart_cancel_send(int uart)
{
    assert((uart >= 1) && (uart <= NUM_UART));

    // 割込を無効化
    drvcmn_setreg32(UART_REGS[uart].uart_base + TOFF_USART_CR1, 0, USART_CR1_TXEIE | USART_CR1_TCIE, 0);

    // 送信済みサイズを返却
    UART_CONTEXT_T* context = &(s_contexts[uart]);
    size_t tx_count = context->tx_count;

    context->tx_buf = NULL;
    context->tx_size = 0;
    context->tx_count = 0;

    return tx_count;
}

size_t drvcmn_uart_cancel_receive(int uart)
{
    assert((uart >= 1) && (uart <= NUM_UART));

    // 割込を無効化
    drvcmn_setreg32(UART_REGS[uart].uart_base + TOFF_USART_CR3, 0, USART_CR3_EIE, 0);
    drvcmn_setreg32(UART_REGS[uart].uart_base + TOFF_USART_CR1, 0, USART_CR1_PEIE | USART_CR1_RXNEIE, 0);

    // 受信済みサイズを返却
    UART_CONTEXT_T* context = &(s_contexts[uart]);
    size_t rx_count = context->rx_count;

    context->rx_buf = NULL;
    context->rx_size = 0;
    context->rx_count = 0;

    return rx_count;
}

void drvcmn_uart_send_pol(int uart, uint32_t data)
{
    assert((uart >= 1) && (uart <= NUM_UART));

    // ベースアドレス
    intptr_t uart_base = UART_REGS[uart].uart_base;

    while (drvcmn_getreg32(uart_base + TOFF_USART_ISR, 0, USART_ISR_TXE) == 0);

    sil_wrw_mem((uint32_t*)(uart_base + TOFF_USART_TDR), (uint32_t)data);

    while (drvcmn_getreg32(uart_base + TOFF_USART_ISR, 0, USART_ISR_TC) == 0);
}

void drvcmn_uart_receive_pol(int uart, uint32_t* data)
{
    assert((uart >= 1) && (uart <= NUM_UART));
    assert(data);

    // ベースアドレス
    intptr_t uart_base = UART_REGS[uart].uart_base;

    while (drvcmn_getreg32(uart_base + TOFF_USART_ISR, 0, USART_ISR_RXNE) == 0);

    *data = drvcmn_getreg32(uart_base + TOFF_USART_RDR, 0, ~(uint32_t)0);

    //syslog(LOG_NOTICE, "ISR: 0x%08x", drvcmn_getreg32(uart_base + TOFF_USART_ISR, 0, ~(uint32_t)0));
}

void drvcmn_uart_isr(intptr_t exinf)
{
    assert((exinf >= 1) && (exinf <= NUM_UART));

    // ベースアドレス
    intptr_t uart_base = UART_REGS[exinf].uart_base;

    // コンテキスト
    UART_CONTEXT_T* context = &(s_contexts[exinf]);

    // ISRレジスタ
    uint32_t isr = drvcmn_getreg32(uart_base + TOFF_USART_ISR, 0, ~(uint32_t)0);

    // CR1レジスタ
    uint32_t cr1 = drvcmn_getreg32(uart_base + TOFF_USART_CR1, 0, ~(uint32_t)0);

    if ((cr1 & USART_CR1_TXEIE) && (isr & USART_ISR_TXE)) {
        // 最後の送信の場合, TXE割込みを無効にし, TC割込みを有効にする
        if ((context->tx_count + 1) == context->tx_size) {
            drvcmn_setreg32(uart_base + TOFF_USART_ICR, 0, USART_ICR_TCCF, ~(uint32_t)0);	// TCをクリア
            drvcmn_setreg32(uart_base + TOFF_USART_CR1, 0, USART_CR1_TXEIE | USART_CR1_TCIE, USART_CR1_TCIE);
        }

        // 送信レジスタにデータをセット
        drvcmn_setreg32(uart_base + TOFF_USART_TDR, 0, 0xff, context->tx_buf[context->tx_count]);
        context->tx_count++;

    } else if ((cr1 & USART_CR1_TCIE) && (isr & USART_ISR_TC)) {
        drvcmn_setreg32(uart_base + TOFF_USART_ICR, 0, USART_ICR_TCCF, ~(uint32_t)0);	// TCをクリア
        drvcmn_setreg32(uart_base + TOFF_USART_CR1, 0, USART_CR1_TCIE, 0);	// TC割込み無効

        // コンテキストクリア
        context->tx_buf = NULL;
        context->tx_size = 0;
        context->tx_count = 0;

        // 送信完了コールバック
        if (context->callback) {
            context->callback(DRVCMN_UART_TXCOMPLETE, 0);
        }

    } else if ((cr1 & USART_CR1_RXNEIE) && (isr & USART_ISR_RXNE)) {
        // 最後の受信の場合, RXNE割込みを無効にする
        if ((context->rx_count + 1) == context->rx_size) {
            drvcmn_setreg32(uart_base + TOFF_USART_CR3, 0, USART_CR3_EIE, 0);
            drvcmn_setreg32(uart_base + TOFF_USART_CR1, 0, USART_CR1_PEIE | USART_CR1_RXNEIE, 0);
        }

        // 受信レジスタからデータを取得
        if (context->rx_buf) {
            context->rx_buf[context->rx_count] = drvcmn_getreg32(uart_base + TOFF_USART_RDR, 0, 0x00ff);
        } else {
            // 読み捨て
            uint8_t dummybuf = drvcmn_getreg32(uart_base + TOFF_USART_RDR, 0, 0x00ff);
            (void)dummybuf;
        }
        context->rx_count++;

        if ((context->rx_count) == context->rx_size) {
            // コンテキストクリア
            context->rx_buf = NULL;
            context->rx_size = 0;
            context->rx_count = 0;

            if (context->callback) {
                // 受信完了コールバック
                context->callback(DRVCMN_UART_RXCOMPLETE, 0);
            }
        }
    } else {
        drvcmn_setreg32(uart_base + TOFF_USART_ICR, 0, USART_ICR_ORECF | USART_ICR_NCF | USART_ICR_FECF | USART_ICR_PECF, ~(uint32_t)0);
        context->callback(DRVCMN_UART_ERROR, isr);
        //assert(false);
    }
}


