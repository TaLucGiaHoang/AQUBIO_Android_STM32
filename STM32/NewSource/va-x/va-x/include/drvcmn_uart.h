/*
 * VA-X UART共通処理
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/11/10 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>
#include <sil.h>

/*
 * マクロ定義
 */

#define DRVCMN_UART_CLOCKSEL_APB	0x0
#define DRVCMN_UART_CLOCKSEL_SYS	0x1
#define DRVCMN_UART_CLOCKSEL_HSI	0x2
#define DRVCMN_UART_CLOCKSEL_LSE	0x3

#define DRVCMN_UART_WORD_8BIT	0x0
#define DRVCMN_UART_WORD_9BIT	0x1
#define DRVCMN_UART_WORD_7BIT	0x2

#define DRVCMN_UART_PARITY_NONE	0x0	// パリティなし
#define DRVCMN_UART_PARITY_EVEN	0x2	// 偶数
#define DRVCMN_UART_PARITY_ODD	0x3	// 奇数

#define DRVCMN_UART_STOP_1	0x0	// 1bit
#define DRVCMN_UART_STOP_05	0x1	// 0.5bit
#define DRVCMN_UART_STOP_2	0x2	// 2bits
#define DRVCMN_UART_STOP_15	0x3	// 1.5bits

// UARTコールバック
#define DRVCMN_UART_TXCOMPLETE	1	// 送信完了
#define DRVCMN_UART_RXCOMPLETE	2	// 受信完了
#define DRVCMN_UART_ERROR		3	// エラー

/*
 * 型定義
 */

/* ISRコールバック関数 */
typedef bool_t (*DRVCMN_UART_ISR_CALLBACK_T)(int type, uint32_t opt);

/* UART設定 */
typedef struct {
    uint32_t clocksel;
    uint32_t baudrate;
    uint32_t wordlen;
    uint32_t parity;
    uint32_t stop;
    bool_t hwflow;
    DRVCMN_UART_ISR_CALLBACK_T callback;
} DRVCMN_UART_SETTING_T;


/*
 * 定数定義
 */

/*
 * 公開関数
 */

// UART初期化
void drvcmn_uart_initialize(int uart, const DRVCMN_UART_SETTING_T* setting);

// 送信
void drvcmn_uart_send(int uart, const uint8_t* data, size_t size);

// 受信
void drvcmn_uart_receive(int uart, uint8_t* buf, size_t size);

// 送信キャンセル
size_t drvcmn_uart_cancel_send(int uart);

// 受信キャンセル
size_t drvcmn_uart_cancel_receive(int uart);

// 1文字送信 (ポーリング方式)
void drvcmn_uart_send_pol(int uart, uint32_t data);

// 1文字受信 (ポーリング方式)
void drvcmn_uart_receive_pol(int uart, uint32_t* data);

/*
 * 内部関数
 */

// UART ISR
void drvcmn_uart_isr(intptr_t exinf);

