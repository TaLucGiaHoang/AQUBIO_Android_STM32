/*
 * VA-X Wifiドライバ (SX-ULPGN)
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/03/22 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "drvwifi.h"

#include <stdio.h>
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
static void mpf_send(int dtq, int msg, intptr_t data, size_t size);

// wifiモジュール初期化
static void process_msg_initialize(DRVWIFI_CALLBACK_T callback);

static void wifi_ap_connect(const uint8_t* essid, const uint8_t* passphrase);

static void wifi_tcp_client(const uint8_t* ip_addr, const uint16_t port);

static void wifi_tcp_server(const uint16_t port);

static int wifi_get_result();

static void wifi_send(const uint8_t* data, size_t size);

static void wifi_receive(uint8_t* data, size_t size);

static int uart_send(const uint8_t* data, size_t size);

// UART送信
static int uart_send_at(const uint8_t* data, size_t data_len, bool_t discard_echo);

// UART受信
static int uart_receive(size_t* count, uint8_t* data, size_t data_len, TMO timeout);

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
    MSG_AP_CONNECT,
    MSG_TCP_CONNECT,
    MSG_TCP_SERVER,
    MSG_SEND,
    MSG_RECEIVE,
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
#define ATCMD_CREATE_SOCKET         "ATNSOCK=0" // Create socket 0
#define ATCMD_TCP_CLIENT            "ATNCTCP=%s,%d" // Connect to TCP server: ip, port
#define ATCMD_TCP_CLIENT_TIMEOUT    "ATS103=0" // Set TCP connect timeout to 30s
#define ATCMD_TCP_SERVER            "ATNSTCP=%d" // Listen to TCP port
#define ATCMD_TCP_SERVER_TIMEOUT    "ATS104=0" // Set TCP connect timeout to 30s
#define ATCMD_QUERY_IP4_SETTING     "ATNSET=?"
#define ATCMD_CLOSE_SOCKET          "ATNCLOSE"
#define ATCMD_DISCONNECT            "ATWD"

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

static DRVWIFI_CONFIG s_wifi_config;
static DRVWIFI_TCP_CONFIG s_tcp_config;
static char* s_ip_address = NULL;

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

    act_tsk(TSK_DRVWIFI_RX);
    assert(er == E_OK);

    // 初期化メッセージを送信
    mpf_send(DTQ_DRVWIFI, MSG_INITIALIZE, (intptr_t)callback, 0);
}

void drvwifi_ap_connect(const uint8_t* essid, const uint8_t* pass)
{
    strcpy((char*)s_wifi_config.essid, (char*)essid);
    strcpy((char*)s_wifi_config.passphrase, (char*)pass);

    mpf_send(DTQ_DRVWIFI, MSG_AP_CONNECT, (intptr_t)&s_wifi_config, 0);
}

void drvwifi_tcp_connect(const uint8_t* ip_address, uint16_t port)
{
    strcpy((char*)s_tcp_config.ip_address, (char*)ip_address);
    s_tcp_config.port = port;

    mpf_send(DTQ_DRVWIFI, MSG_TCP_CONNECT, (intptr_t)&s_tcp_config, 0);
}

void drvwifi_tcp_server(uint16_t port)
{
    s_tcp_config.port = port;

    mpf_send(DTQ_DRVWIFI, MSG_TCP_SERVER, (intptr_t)&s_tcp_config, 0);
}

void drvwifi_send(const uint8_t* data, size_t size)
{
    mpf_send(DTQ_DRVWIFI, MSG_SEND, (intptr_t)data, size);
}

void drvwifi_receive(uint8_t* data, size_t size)
{
    DBGLOG0("drvwifi_receive");
    mpf_send(DTQ_DRVWIFI_RX, MSG_RECEIVE, (intptr_t)data, size);
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
        case MSG_AP_CONNECT:
        {
            DRVWIFI_CONFIG *wifi_config = (DRVWIFI_CONFIG*)blk->data;
            wifi_ap_connect(wifi_config->essid, wifi_config->passphrase);
            break;
        }
        case MSG_TCP_CONNECT:
        {
            DRVWIFI_TCP_CONFIG *wifi_tcp_config = (DRVWIFI_TCP_CONFIG*)blk->data;
            wifi_tcp_client(wifi_tcp_config->ip_address, wifi_tcp_config->port);
            break;
        }
        case MSG_TCP_SERVER:
        {
            DRVWIFI_TCP_CONFIG *wifi_tcp_config = (DRVWIFI_TCP_CONFIG*) blk->data;
            wifi_tcp_server(wifi_tcp_config->port);
            break;
        }
        case MSG_SEND:
        {
            wifi_send((uint8_t*) blk->data, blk->size);
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
    syslog(LOG_NOTICE, "drvwifi_rx_task() starts .");

    while (true) {
        DRVWIFI_MPFBLK_T* blk = NULL;
        ER er = 0;
        er = trcv_dtq(DTQ_DRVWIFI_RX, (intptr_t*) &blk, TMO_FEVR);
        assert(er == E_OK);
        switch (blk->msg) {
        case MSG_RECEIVE:
            wifi_receive((uint8_t*) blk->data, blk->size);
            break;
        default:
            assert(false);
            break;
        }

        er = rel_mpf(MPF_DRVWIFI, blk);
        assert(er == E_OK);
    }
}

/*********************************************************************
 * 内部関数
 ********************************************************************/

/*
 * メッセージ送信
 */
void mpf_send(int dtq, int msg, intptr_t data, size_t size)
{
    DRVWIFI_MPFBLK_T* blk = NULL;
    ER er = 0;
    er = tget_mpf(MPF_DRVWIFI, (void**)&blk, SYSCALL_TIMEOUT);
    assert(er == E_OK);

    blk->msg = msg;
    blk->data = data;
    blk->size = size;
    er = tsnd_dtq(dtq, (intptr_t)blk, SYSCALL_TIMEOUT);
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
//    int count = 0;
//    for (int i = 0; i < 5; i++) {
//        count = uart_receive_1line(s_context.receive_buf, RECEIVE_BUF_SIZE);
//        DBGLOG2("result (%d): \"%s\"", count, s_context.receive_buf);
//        if (count == 0) break;
//    }
    wifi_get_result();

    // コマンドエコーオフ("ATE0")
    DBGLOG1("SEND: \"%s\"", ATCMD_ECHO_OFF);
    uart_send_at((const uint8_t*)ATCMD_ECHO_OFF, sizeof(ATCMD_ECHO_OFF) - 1, true);

    // 応答2行受信
//    for (int i = 0; i < 2; i++) {
//        uart_receive_1line(s_context.receive_buf, RECEIVE_BUF_SIZE);
//        DBGLOG1("result: \"%s\"", s_context.receive_buf);
//    }
    if (wifi_get_result() == DRVWIFI_RESULT_OK) {
        callback(DRVWIFI_EVT_INITIALIZE_COMPLETE, 0, 0);
    }
}

#if 1
char* wifi_get_ip_address()
{
    DBGLOG1("SEND: \"%s\"", ATCMD_QUERY_IP4_SETTING);
    uart_send_at((const uint8_t*) ATCMD_QUERY_IP4_SETTING, sizeof(ATCMD_QUERY_IP4_SETTING) - 1, false);
    wifi_get_result();
    return s_ip_address;
}
#endif

void wifi_ap_connect(const uint8_t* essid, const uint8_t* passphrase)
{
    uint8_t cmd[100];
    int result;
    DBGLOG0("drvwifi_ap_connect() called.....\n");

    assert(essid);
    assert(passphrase);

    // Disconnect from currently connected Access Point
    DBGLOG1("SEND: \"%s\"", ATCMD_DISCONNECT);
    uart_send_at((const uint8_t*) ATCMD_DISCONNECT, sizeof(ATCMD_DISCONNECT) - 1, false);
    result = wifi_get_result();

    // Connect to AP
    // "ATWAWPA=<ssid>,<ver>,<ucipher>,<mcipher>,<passphrase>"
    sprintf((char*)cmd, "%s=%s,%d,%d,%d,%s", "ATWAWPA", essid, 2, 1, 0, passphrase);

    // Send command to the WIFI module
    DBGLOG1("SEND: \"%s\"", cmd);
    uart_send_at((const uint8_t*)cmd, sizeof(cmd) - 1, false);
    result = wifi_get_result();

    // Wait for getting IP Address from DHCP
    dly_tsk(3000);

    // Check IP address
    char* ip_addr = NULL;
    for (int i = 0; i < 10; i++) {
        ip_addr = wifi_get_ip_address();
        if (ip_addr != NULL) {
            DBGLOG1("IP: \"%s\"", ip_addr);
            break;
        }
        dly_tsk(1000);
    }

    if (s_callback) {
        s_callback(DRVWIFI_EVT_AP_CONNECT_COMPLETE, ip_addr, 0);
    }
}

void wifi_tcp_client(const uint8_t* ip_addr, const uint16_t port)
{
    assert(ip_addr);
    assert(port);

    DBGLOG0("wifi_tcp_client()\n");
    int result;

    // Close all sockets
//    DBGLOG1("SEND: \"%s\"", ATCMD_CLOSE_SOCKET);
//    uart_send_at((const uint8_t*) ATCMD_CLOSE_SOCKET, sizeof(ATCMD_CLOSE_SOCKET) - 1, false);
//    result = wifi_get_result();

    // Create TCP socket
    DBGLOG1("SEND: \"%s\"", ATCMD_CREATE_SOCKET);
    uart_send_at((const uint8_t*)ATCMD_CREATE_SOCKET, sizeof(ATCMD_CREATE_SOCKET) - 1, false);
    result = wifi_get_result();

    // Set timeout
    DBGLOG1("SEND: \"%s\"", ATCMD_TCP_CLIENT_TIMEOUT);
    uart_send_at((const uint8_t*) ATCMD_TCP_CLIENT_TIMEOUT, strlen(ATCMD_TCP_CLIENT_TIMEOUT) - 1, false);
    result = wifi_get_result();

    // Connect to TCP server
    uint8_t cmd[100];
    sprintf((char*)cmd, ATCMD_TCP_CLIENT, ip_addr, port);

    // Send command to the WIFI module
    DBGLOG2("SEND (%d): \"%s\"", strlen((char*)cmd), cmd);
    uart_send_at(cmd, strlen((char*)cmd), false);
    result = wifi_get_result();

    if (s_callback && result == DRVWIFI_RESULT_CONNECT) {
        s_callback(DRVWIFI_EVT_TCP_CONNECT_COMPLETE, 0, 0);
    }
}

void wifi_tcp_server(const uint16_t port)
{
    assert(port);

    DBGLOG0("wifi_tcp_server()\n");
    int result;

    // Send command to the WIFI module
    DBGLOG1("SEND: \"%s\"", ATCMD_CREATE_SOCKET);
    uart_send_at((const uint8_t*) ATCMD_CREATE_SOCKET, sizeof(ATCMD_CREATE_SOCKET) - 1, false);
    result = wifi_get_result();

    DBGLOG1("SEND: \"%s\"", ATCMD_TCP_SERVER_TIMEOUT);
    uart_send_at((const uint8_t*) ATCMD_TCP_SERVER_TIMEOUT, strlen(ATCMD_TCP_SERVER_TIMEOUT) - 1, false);
    result = wifi_get_result();

    uint8_t cmd[100];
    sprintf((char*) cmd, ATCMD_TCP_SERVER, port);

    // Send command to the WIFI module
    DBGLOG2("SEND (%d): \"%s\"", strlen((char* )cmd), cmd);
    uart_send_at(cmd, strlen((char*) cmd), false);

    result = wifi_get_result();

    if (s_callback && result == DRVWIFI_RESULT_CONNECT) {
        s_callback(DRVWIFI_EVT_TCP_SERVER_COMPLETE, 0, 0);
    }
}


int wifi_get_result()
{
    // Parse AT response to get the result
    int count;
    do {
        count = uart_receive_1line(s_context.receive_buf, RECEIVE_BUF_SIZE);
#if DEBUG
        char* tmp = strdup((char*)s_context.receive_buf);
        if (count >= 2) tmp[count - 2] = 0;
        DBGLOG2("Rcv (%d): %s", count, tmp);
#endif

        if (memcmp(s_context.receive_buf, "OK", 2) == 0) {
            DBGLOG0("DRVWIFI_RESULT_OK");
            return DRVWIFI_RESULT_OK;
        } else if (memcmp(s_context.receive_buf, "ERROR", 5) == 0) {
            DBGLOG0("DRVWIFI_RESULT_ERROR");
            return DRVWIFI_RESULT_ERROR;
        } else if (memcmp(s_context.receive_buf, "BUSY", 4) == 0) {
            DBGLOG0("DRVWIFI_RESULT_BUSY");
            return DRVWIFI_RESULT_BUSY;
        } else if (memcmp(s_context.receive_buf, "NO CARRIER", 10) == 0) {
            DBGLOG0("DRVWIFI_RESULT_NO_CARRIER");
            return DRVWIFI_RESULT_NO_CARRIER;
        } else if (memcmp(s_context.receive_buf, "CONNECT", 7) == 0) {
            DBGLOG0("DRVWIFI_RESULT_CONNECT");
            return DRVWIFI_RESULT_CONNECT;
        } else if (memcmp(s_context.receive_buf, "IP:0.0.0.0", 10) == 0) {

        } else if (memcmp(s_context.receive_buf, "IP:", 3) == 0) {
            char* c = (char*)s_context.receive_buf;
            while (*++c != ',') {}
            *c = 0;
            s_ip_address = strdup((char*)&s_context.receive_buf[3]);
        }
    } while (count > 0);

    return DRVWIFI_RESULT_OK;
}

void wifi_send(const uint8_t* data, size_t size)
{
    assert(data);
    assert(size > 0);

    int error = uart_send(data, size);

    if (s_callback) {
        s_callback(DRVWIFI_EVT_TCP_SEND_COMPLETE, error, 0);
    }
}

void wifi_receive(uint8_t* data, size_t size)
{
    DBGLOG0("wifi_receive");
    assert(size > 0);

    size_t count = 0;
    int error = uart_receive(&count, data, size, TMO_FEVR);

    DBGLOG0("Receive completed");

    if (s_callback) {
        DBGLOG0("Receive completed callback");
        s_callback(DRVWIFI_EVT_TCP_RECEIVE_COMPLETE, error, count);
    }
}

int uart_send(const uint8_t* data, size_t size)
{
    assert(data);
    assert(size > 0);

    ER er = 0;
    // イベントフラグをクリア
    er = clr_flg(FLG_DRVWIFI, ~FLGPTN_UART_TXCOMPLETE);
    assert(er == E_OK);

    // UART送信
    drvcmn_uart_send(WIFI_UART, data, size);

    // 完了待ち
    FLGPTN flgptn = 0;
    er = twai_flg(FLG_DRVWIFI, FLGPTN_UART_TXCOMPLETE, TWF_ANDW, &flgptn, UART_TIMEOUT);
    assert((er == E_OK) || (er == E_TMOUT));

    // タイムアウトの場合は中断処理
    if (er == E_TMOUT) {
        drvcmn_uart_cancel_send(WIFI_UART);
        return -1;
    }

    return 0;
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

    er = clr_flg(FLG_DRVWIFI, ~FLGPTN_UART_TXCOMPLETE);
    assert(er == E_OK);

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
int uart_receive(size_t* count, uint8_t* data, size_t data_len, TMO timeout)
{
    assert(count);
    assert(data_len > 0);

    int error = DRVWIFI_ERROR_NONE;
    ER er = 0;

    er = clr_flg(FLG_DRVWIFI, ~FLGPTN_UART_RXCOMPLETE);
    assert(er == E_OK);

    // 受信
    drvcmn_uart_receive(WIFI_UART, data, data_len);

    // 送信完了を待つ
    FLGPTN flgptn = 0;
    er = twai_flg(FLG_DRVWIFI, FLGPTN_UART_RXCOMPLETE, TWF_ORW, &flgptn, timeout);
    *count = drvcmn_uart_cancel_receive(WIFI_UART);
    assert(er == E_OK);

    if (er == E_TMOUT) {
        error = DRVWIFI_ERROR_TIMEOUT;
//        *count = drvcmn_uart_cancel_receive(WIFI_UART);
    }

    return error;
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
    ER er = 0;
    data[0] = 0;
    for (received = 0; received < data_limit; received++) {
        er = clr_flg(FLG_DRVWIFI, ~FLGPTN_UART_RXCOMPLETE);
        assert(er == E_OK);

        // 1文字受信
        drvcmn_uart_receive(WIFI_UART, (uint8_t*)&data[received], 1);

        // 送信完了を待つ(コマンドエコーを読み捨てる場合は受信完了も待つ)
        FLGPTN flgptn = 0;
        ER er = twai_flg(FLG_DRVWIFI, FLGPTN_UART_RXCOMPLETE, TWF_ORW, &flgptn, UART_TIMEOUT);
        drvcmn_uart_cancel_receive(WIFI_UART);
        if (er != E_OK) {
            DBGLOG0("TIMEOUT");
            break;
        }

        // 比較
        if (data[received] == PATTERN[match_count]) {
            match_count++;
            if (match_count >= PATTERN_LEN) {
                received++;
                data[received] = 0;
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
