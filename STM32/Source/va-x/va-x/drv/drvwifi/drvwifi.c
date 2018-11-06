/*
 * VA-X Wifiドライバ (SX-ULPGN)
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/03/22 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "drvwifi.h"

#include <string.h>

#include <kernel.h>
#include <sil.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "kernel_cfg.h"
#include "stm32f7xx.h"

#include "drvcmn_gpio.h"
#include "drvcmn_uart.h"

/*
 * マクロ定義
 */

// デバッグログ
#if 1
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[DRVWIFI]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[DRVWIFI]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[DRVWIFI]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[DRVWIFI]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

// サービスコールのエラーのログ出力
static inline void svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}
#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))

/*
 * 型定義
 */

// メモリプールブロック
typedef struct {
    uint32_t msg;
    intptr_t data;
    size_t size;
} DRVWIFI_MPFBLK_T;

// DRVWIFI_MPFBLK_SIZE は sizeof(DRVWIFI_MPFBKL) に一致させること
static_assert(sizeof(DRVWIFI_MPFBLK_T) == DRVWIFI_MPFBLK_SIZE, "MPF size");


/*
 * 内部関数プロトタイプ
 */
// メモリプールブロックを使ってメッセージ送信
static void mpf_send(int msg, intptr_t data, size_t size);

// wifiモジュール初期化
static void process_msg_initialize(DRVWIFI_CALLBACK_T callback);

// UART送信
static int uart_send_at(const uint8_t* data, size_t data_len, bool_t discard_echo);

// UART受信
static int uart_receive(uint8_t* data, size_t data_len);

// UART1行受信(\r\nまで読み込み)
static int uart_receive_1line(uint8_t* data, size_t data_limit);

// UART ISRコールバック
static bool_t uart_isr_callback(int type, uint32_t opt);

/*
 * 定数定義
 */

// GPIO(UART)
static const DRVCMN_GPIO_PIN_T GPIO_PINS_UART[] = {
    { DRVCMN_GPIO_PORT_E, 7 },	// AF8: UART7_RX
    { DRVCMN_GPIO_PORT_E, 8 },	// AF8: UART7_TX
    { DRVCMN_GPIO_PORT_E, 9 },	// AF8: UART7_RTS
    { DRVCMN_GPIO_PORT_E, 10 },	// AF8: UART7_CTS
};

// GPIO設定(UART)
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_UART = {
    .mode = DRVCMN_GPIO_MODE_AF,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLUP,
    .afno = 8,
};

// UART設定
static const int WIFI_UART = 7;	// UART7
static const DRVCMN_UART_SETTING_T UART_SETTING = {
    .clocksel = DRVCMN_UART_CLOCKSEL_SYS,
    .baudrate = 115200,
    .wordlen = DRVCMN_UART_WORD_8BIT,
    .parity = DRVCMN_UART_PARITY_NONE,
    .stop = DRVCMN_UART_STOP_1,
    .hwflow = true,
    .callback = uart_isr_callback,
};

// WIFI_ON
static const DRVCMN_GPIO_PIN_T GPIO_PIN_WIFI_ON = { DRVCMN_GPIO_PORT_D, 10 };	// PD10: WIFI_ON (Active Low)

// GPIO設定(CARD_ON)
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_WIFI_ON = {
    .mode = DRVCMN_GPIO_MODE_GPOUT,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLUP,
    .afno = 0,
};

// メッセージ番号
enum {
    MSG_INITIALIZE = 1,
};

// イベントフラグ
static const FLGPTN FLGPTN_UART_TXCOMPLETE = (0x1 << 0);
static const FLGPTN FLGPTN_UART_RXCOMPLETE = (0x1 << 1);

// タイムアウト
static const int32_t SYSCALL_TIMEOUT = 1000;
static const int32_t UART_TIMEOUT = 3000;

// ATコマンド
#define CR				"\r"	// CR
#define CRLF			"\r\n"	// CRLF
#define ATCMD_OK		"OK"	// OK
#define ATCMD_RESET		"ATZ"	// ソフトウェアリセット
#define ATCMD_ECHO_OFF	"ATE0"	// コマンドエコーOFF

// バッファサイズ
#define RECEIVE_BUF_SIZE	(1024)

/*
 * 内部変数
 */
static DRVWIFI_CALLBACK_T s_callback = NULL;

// コンテキスト情報
static struct {
    uint8_t receive_buf[RECEIVE_BUF_SIZE];
} s_context;


/*********************************************************************
 * 公開関数
 ********************************************************************/

/*
 * ペリフェラル初期化
 */
void drvwifi_peripheral_initialize()
{
    // GPIO初期化(UART)
    for (int i = 0; i < sizeof(GPIO_PINS_UART) / sizeof(GPIO_PINS_UART[0]); i++) {
        drvcmn_gpio_pin_initialize(&(GPIO_PINS_UART[i]), &GPIO_SETTING_UART);
    }

    drvcmn_gpio_pin_initialize(&GPIO_PIN_WIFI_ON, &GPIO_SETTING_WIFI_ON);
    drvcmn_gpio_pin_set(&GPIO_PIN_WIFI_ON, true);


}

/*
 * ドライバ初期化
 */
void drvwifi_initialize(DRVWIFI_CALLBACK_T callback)
{
    ER er = 0;

    // ドライバタスク起動
    act_tsk(TSK_DRVWIFI);
    assert(er == E_OK);

    // 初期化メッセージを送信
    mpf_send(MSG_INITIALIZE, (intptr_t)callback, 0);
}

/*
 * タスク
 */
void drvwifi_task(intptr_t exinf)
{
    DBGLOG0("drvwifi_task() starts .");

    while (true) {
        DRVWIFI_MPFBLK_T* blk = NULL;
        ER er = 0;
        er = trcv_dtq(DTQ_DRVWIFI, (intptr_t*)&blk, TMO_FEVR);
        assert(er == E_OK);
        switch (blk->msg) {
        case MSG_INITIALIZE:
        {
            process_msg_initialize((DRVWIFI_CALLBACK_T)blk->data);
            break;
        }
        default:
            assert(false);
            break;
        }

        er = rel_mpf(MPF_DRVWIFI, blk);
        assert(er == E_OK);
    }
}

/*
 * 受信タスク
 */
void drvwifi_rx_task(intptr_t exinf)
{
    
}

/*********************************************************************
 * 内部関数
 ********************************************************************/

/*
 * メッセージ送信
 */
void mpf_send(int msg, intptr_t data, size_t size)
{
    DRVWIFI_MPFBLK_T* blk = NULL;
    ER er = 0;
    er = tget_mpf(MPF_DRVWIFI, (void**)&blk, SYSCALL_TIMEOUT);
    assert(er == E_OK);

    blk->msg = msg;
    blk->data = data;
    blk->size = size;
    er = tsnd_dtq(DTQ_DRVWIFI, (intptr_t)blk, SYSCALL_TIMEOUT);
    assert(er == E_OK);
}

/*
 * wifiモジュール初期化
 */
void process_msg_initialize(DRVWIFI_CALLBACK_T callback)
{
    assert(!s_callback);
    assert(callback);
    
    // コールバック関数を保持
    s_callback = callback;

    // WIFIモジュールのリセット(電源OFF->ON)
    drvcmn_gpio_pin_set(&GPIO_PIN_WIFI_ON, true);
    dly_tsk(10);
    drvcmn_gpio_pin_set(&GPIO_PIN_WIFI_ON, false);

    // UART初期化
    drvcmn_uart_initialize(WIFI_UART, &UART_SETTING);

    // UART読み捨て
    drvcmn_uart_receive(WIFI_UART, NULL, 1024);
    dly_tsk(100);	// 100ms待つ
    drvcmn_uart_cancel_receive(WIFI_UART);

    // WIFIモジュールに初期化コマンド("ATZ")を送る
    DBGLOG1("SEND: \"%s\"", ATCMD_RESET);
    uart_send_at((const uint8_t*)ATCMD_RESET, sizeof(ATCMD_RESET) - 1, true);

    // 応答5行受信
    int count = 0;
    for (int i = 0; i < 5; i++) {
        count = uart_receive_1line(s_context.receive_buf, RECEIVE_BUF_SIZE);
    }
    DBGLOG1("result: \"%s\"", s_context.receive_buf);

    // コマンドエコーオフ("ATE0")
    DBGLOG1("SEND: \"%s\"", ATCMD_ECHO_OFF);
    uart_send_at((const uint8_t*)ATCMD_ECHO_OFF, sizeof(ATCMD_ECHO_OFF) - 1, true);

    // 応答2行受信
    for (int i = 0; i < 2; i++) {
        uart_receive_1line(s_context.receive_buf, RECEIVE_BUF_SIZE);
    }
    DBGLOG1("result: \"%s\"", s_context.receive_buf);

    // コールバック
    callback(DRVWIFI_EVT_INITIALIZE_COMPLETE, 0, 0);
}

/*
 * UARTでATコマンド送信(CRLFを付加して送信)
 */
int uart_send_at(const uint8_t* data, size_t data_len, bool_t discard_echo)
{
    assert(data);
    assert(data_len > 0);

    FLGPTN flgptn = 0;
    ER er = 0;

    if (discard_echo) {
        // コマンドエコー受信待機(読み捨て)
        drvcmn_uart_receive(WIFI_UART, NULL, data_len + (sizeof(CR) - 1));
    }

    // コマンド送信
    drvcmn_uart_send(WIFI_UART, data, data_len);

    // 送信完了を待つ
    er = twai_flg(FLG_DRVWIFI, FLGPTN_UART_TXCOMPLETE, TWF_ANDW, &flgptn, UART_TIMEOUT);
    assert(er == E_OK);

    // CR送信
    drvcmn_uart_send(WIFI_UART, (const uint8_t*)CR, (sizeof(CR) - 1));

    // 送信完了を待つ
    er = twai_flg(FLG_DRVWIFI, FLGPTN_UART_TXCOMPLETE, TWF_ANDW, &flgptn, UART_TIMEOUT);
    assert(er == E_OK);

    if (discard_echo) {
        // コマンドエコー受信を完了を待つ)
        er = twai_flg(FLG_DRVWIFI, FLGPTN_UART_RXCOMPLETE, TWF_ANDW, &flgptn, UART_TIMEOUT);
        assert(er == E_OK);
    }

    return 0;
}

/*
 * UART受信(サイズ指定)
 */
int uart_receive(uint8_t* data, size_t data_len)
{
    // 受信
    drvcmn_uart_receive(WIFI_UART, data, data_len);

    // 送信完了を待つ
    ER er = twai_flg(FLG_DRVWIFI, FLGPTN_UART_RXCOMPLETE, TWF_ANDW, &(FLGPTN){0}, UART_TIMEOUT);
    assert(er == E_OK);

    return 0;
}

/*
 * UART1行受信(\r\nまで読み込み)
 */
int uart_receive_1line(uint8_t* data, size_t data_limit)
{
    const uint8_t* PATTERN = (const uint8_t*)CRLF;
    const size_t PATTERN_LEN = (sizeof(CRLF) - 1);	// 終端文字を除く

    size_t received = 0;
    size_t match_count = 0;
    for (received = 0; received < data_limit; received++) {
        // 1文字受信
        drvcmn_uart_receive(WIFI_UART, &(data[received]), 1);

        // 送信完了を待つ(コマンドエコーを読み捨てる場合は受信完了も待つ)
        ER er = twai_flg(FLG_DRVWIFI, FLGPTN_UART_RXCOMPLETE, TWF_ANDW, &(FLGPTN){0}, UART_TIMEOUT);
        assert(er == E_OK);
        if (er != E_OK) {
            break;
        }

        // 比較
        if (data[received] == PATTERN[match_count]) {
            match_count++;
            if (match_count >= PATTERN_LEN) {
                DBGLOG1("match!(%d)", received+1);
                break;
            }
        } else {
            match_count = 0;
        }
    }

    return received;
}

/*
 * UART ISRコールバック
 */
bool_t uart_isr_callback(int type, uint32_t opt)
{
    ER er = E_OK;
    switch (type) {
    case DRVCMN_UART_TXCOMPLETE:
        er = iset_flg(FLG_DRVWIFI, FLGPTN_UART_TXCOMPLETE);
        assert(er == E_OK);
        break;
    case DRVCMN_UART_RXCOMPLETE:
        er = iset_flg(FLG_DRVWIFI, FLGPTN_UART_RXCOMPLETE);
        assert(er == E_OK);
        break;
    case DRVCMN_UART_ERROR:
        // TODO: [暫定]Framing Error を無視する
        //DBGLOG1("UARTERROR: 0x%08x", opt);
        break;
    default:
        assert(false);
        break;
    }

    return false;
}
