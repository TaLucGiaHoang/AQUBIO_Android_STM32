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
#if 0
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
    DRVWIFI_CALLBACK_T callback;
    intptr_t data;
    size_t size;
} DRVWIFI_MPFBLK_T;

// DRVWIFI_MPFBLK_SIZE は sizeof(DRVWIFI_MPFBKL) に一致させること
static_assert(sizeof(DRVWIFI_MPFBLK_T) == DRVWIFI_MPFBLK_SIZE, "MPF size");


/*
 * 内部関数プロトタイプ
 */
// メモリプールブロックを使ってメッセージ送信
static void mpf_send(int msg, DRVWIFI_CALLBACK_T callback, intptr_t data, size_t size);

// wifiモジュール初期化
static void process_msg_initialize(DRVWIFI_CALLBACK_T callback);
static void process_msg_initialize2(DRVWIFI_CALLBACK_T callback);	// SHC function
// wifiモジュール電源Off
static void process_msg_power_off(DRVWIFI_CALLBACK_T callback);
// APに接続
static void process_msg_ap_connect(DRVWIFI_CALLBACK_T callback, DRVWIFI_CONFIG* cfg);
static void process_msg_ap_connect2(DRVWIFI_CALLBACK_T callback, DRVWIFI_CONFIG *cfg);	// SHC function
// WPSでAPに接続
static void process_msg_ap_connect_wps(DRVWIFI_CALLBACK_T callback);
// AP切断
static void process_msg_ap_disconnect(DRVWIFI_CALLBACK_T callback);
// HTTPS 接続
static void process_msg_https_connect(DRVWIFI_CALLBACK_T callback, DRVWIFI_HTTPS_CONFIG* httpcfg);
// HTTPS POST
static void process_msg_https_post(DRVWIFI_CALLBACK_T callback, DRVWIFI_HTTPS_REQ* hreq);
// HTTPS 切断
static void process_msg_https_disconnect(DRVWIFI_CALLBACK_T callback);
// PING送信
static void process_msg_ping(DRVWIFI_CALLBACK_T callback, DRVWIFI_PING* ping);

static void wifi_tcp_client(DRVWIFI_CALLBACK_T callback, DRVWIFI_TCP_CONFIG* tcpcfg);

static void wifi_tcp_server(DRVWIFI_CALLBACK_T callback, DRVWIFI_TCP_CONFIG* tcpcfg);

static int wifi_get_result(void);

static void wifi_send(DRVWIFI_CALLBACK_T callback, const uint8_t* data, size_t size);

static void wifi_receive(DRVWIFI_CALLBACK_T callback, uint8_t* data, size_t size);

static int uart_send(const uint8_t* data, size_t size);

// UART送信
static int uart_send_at(const uint8_t* data, size_t data_len, bool_t discard_echo);

// UART受信
static int uart_receive(uint8_t* data, size_t data_len);

// UART受信+ wait
static int uart_receive_wait(size_t* count, uint8_t* data, size_t data_len, TMO timeout);

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
    MSG_POWER_OFF,
    MSG_AP_CONNECT,
    MSG_AP_CONNECT_WPS,
    MSG_AP_DISCONNECT,
    MSG_HTTP_CONNECT,
    MSG_HTTP_POST,
    MSG_HTTP_DISCONNECT,
	MSG_PING,
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
//static const int32_t UART_TIMEOUT = 3000;
static const int32_t UART_TIMEOUT = 20000;

// ATコマンド 結果コード
#define CR							"\r"		// CR
#define CRLF						"\r\n"	    // CRLF
#define ATRET_OK					"OK"
#define ATRET_CONNECT		        "CONNECT"
#define ATRET_RING				    "RING"
#define ATRET_NOCARRIER	            "NO CARRIER"
#define ATRET_ERROR			        "ERROR"
#define ATRET_NODIALTONE	        "NO DIALTONE"
#define ATRET_BUSY				    "BUSY"
#define ATRET_NOANSWER	            "NO ANSWER"

// ATコマンド
#define ATCMD_RESET					"ATZ"			// ソフトウェアリセット
#define ATCMD_ECHO_OFF				"ATE0"			// コマンドエコーOFF
#define ATCMD_ECHO_ON				"ATE1"			// コマンドエコーON
#define ATCMD_APCONNECT			    "ATWAWPA"		// APへの接続
#define ATCMD_APCONNECT_WPS	        "ATWAWPS"		// APへの接続(WPS)
#define ATCMD_APDISCONNECT		    "ATWD"			// 無線切断
#define ATCMD_HTTPS_TLSINIT		    "ATNHTTPCSSL"	// TSLコンテキスト初期化
#define ATCMD_HTTPS_CONN			"ATNHTTPCCON"	// HTTP接続
#define ATCMD_HTTPS_EXHDR		    "ATNHTTPCHDR"	// 拡張HTTPヘッダ設定
#define ATCMD_HTTPS_BODY			"ATNHTTPCBODY"	// HTTP本文設定
#define ATCMD_HTTPS_POST			"ATNHTTPCPOST"	// HTTP送信
#define ATCMD_HTTPS_DISCONN	        "ATNHTTPCDISC"	// HTTP切断
#define ATCMD_PING				        "ATNPING"				// Debug用pingコマンド
#define AT_ESC_SEQUENCE			    "+++"			// 透過モードOffエスケープシーケンス
#define ATCMD_CREATE_SOCKET         "ATNSOCK=0" // Create socket 0
#define ATCMD_TCP_CLIENT            "ATNCTCP=%s,%d" // Connect to TCP server: ip, port
#define ATCMD_TCP_CLIENT_TIMEOUT    "ATS103=0" // Set TCP connect timeout to 30s
#define ATCMD_TCP_SERVER            "ATNSTCP=%d" // Listen to TCP port
#define ATCMD_TCP_SERVER_TIMEOUT    "ATS104=0" // Set TCP connect timeout to 30s
#define ATCMD_QUERY_IP4_SETTING     "ATNSET=?"
#define ATCMD_CLOSE_SOCKET          "ATNCLOSE"
#define ATCMD_DISCONNECT            ATCMD_APDISCONNECT //"ATWD"

// バッファサイズ
#define RECEIVE_BUF_SIZE	(1024)
#define RECEIVE_POST_SIZE	(2048)

/*
 * 内部変数
 */
// コンテキスト情報
static struct {
    uint8_t receive_buf[RECEIVE_BUF_SIZE];
    uint8_t receive_post[RECEIVE_POST_SIZE];
} s_context;


// static DRVWIFI_TCP_CONFIG s_tcp_config;
static char* s_ip_address = NULL;

/*********************************************************************
 * 公開関数
 ********************************************************************/

/*
 * ペリフェラル初期化
 */
void drvwifi_peripheral_initialize(void)
{
    // GPIO初期化(UART)
    for (int i = 0; i < sizeof(GPIO_PINS_UART) / sizeof(GPIO_PINS_UART[0]); i++) {
        drvcmn_gpio_pin_initialize(&(GPIO_PINS_UART[i]), &GPIO_SETTING_UART);
    }

    drvcmn_gpio_pin_initialize(&GPIO_PIN_WIFI_ON, &GPIO_SETTING_WIFI_ON);
    drvcmn_gpio_pin_set(&GPIO_PIN_WIFI_ON, true);  //Power Off
}

/*
 * ドライバ初期化
 */
void drvwifi_initialize(DRVWIFI_CALLBACK_T callback)
{
    ER er = 0;

    // ドライバタスク起動
    er = act_tsk(TSK_DRVWIFI);
    assert(er == E_OK);

    act_tsk(TSK_DRVWIFI_RX);
    assert(er == E_OK);

    // 起動直後はWiFiモジュールをONしない。サーバアクセス時のみ起動する
    // 初期化メッセージを送信
    mpf_send(MSG_INITIALIZE, callback, 0, 0);
}

/*
 * wifiモジュールのリセット
 */
void drvwifi_start(DRVWIFI_CALLBACK_T callback)
{
	// メッセージを送信
    mpf_send(MSG_INITIALIZE, callback, 0, 0);	
}

/*
 * wifiモジュール電源Off
 */
void drvwifi_power_off(DRVWIFI_CALLBACK_T callback)
{
    // 終了メッセージを送信
    mpf_send(MSG_POWER_OFF, callback, 0, 0);	
}

/* 
 *APに接続
 */
void drvwifi_ap_connect(DRVWIFI_CALLBACK_T callback, DRVWIFI_CONFIG* ap)
{
	// メッセージを送信
    mpf_send(MSG_AP_CONNECT, callback, (intptr_t)ap, 0);	
}

/*
 * WPSでAPに接続
 */
void drvwifi_ap_connect_wps(DRVWIFI_CALLBACK_T callback)
{
	// メッセージを送信
    mpf_send(MSG_AP_CONNECT_WPS, callback, 0, 0);	
}

/* 
 *AP切断
 */
void drvwifi_ap_disconnect(DRVWIFI_CALLBACK_T callback)
{
	// メッセージを送信
    mpf_send(MSG_AP_DISCONNECT, callback, 0, 0);	
}

void drvwifi_tcp_connect(DRVWIFI_CALLBACK_T callback, DRVWIFI_TCP_CONFIG* tcpcfg)
{
	mpf_send(MSG_TCP_CONNECT, callback, (intptr_t)tcpcfg, 0);
}

void drvwifi_tcp_server(DRVWIFI_CALLBACK_T callback, DRVWIFI_TCP_CONFIG* tcpcfg)
{
	mpf_send(MSG_TCP_SERVER, callback, (intptr_t)tcpcfg, 0);
}

void drvwifi_send(DRVWIFI_CALLBACK_T callback, const uint8_t* data, size_t size)
{
	mpf_send(MSG_SEND, callback, (intptr_t)data, size);
}

void drvwifi_receive(DRVWIFI_CALLBACK_T callback, uint8_t* data, size_t size)
{
	mpf_send_rx(MSG_RECEIVE, callback, (intptr_t)data, size);
}

/*
 * HTTPS 接続
 */
void drvwifi_https_connect(DRVWIFI_CALLBACK_T callback, DRVWIFI_HTTPS_CONFIG* httpcfg)
{
	// メッセージを送信
    mpf_send(MSG_HTTP_CONNECT, callback, (intptr_t)httpcfg, 0);	
}

/*
 * HTTPS POST
 */
void drvwifi_https_post(DRVWIFI_CALLBACK_T callback, DRVWIFI_HTTPS_REQ* req)
{
	// メッセージを送信
    mpf_send(MSG_HTTP_POST, callback, (intptr_t)req, 0);	
}

/*
 * HTTPS 切断
 */
void drvwifi_https_disconnect(DRVWIFI_CALLBACK_T callback)
{
	// メッセージを送信
    mpf_send(MSG_HTTP_DISCONNECT, callback, 0, 0);	
}

/* 
 * PING
 */
void drvwifi_ping(DRVWIFI_CALLBACK_T callback, DRVWIFI_PING* ping)
{
	// メッセージを送信
    mpf_send(MSG_PING, callback, (intptr_t)ping, 0);	
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
            process_msg_initialize2(blk->callback);    // SHC function
            break;
        }
        case MSG_POWER_OFF:
        {
            process_msg_power_off(blk->callback);
            break;
        }
        case MSG_AP_CONNECT:
        {
            process_msg_ap_connect2(blk->callback, (DRVWIFI_CONFIG*)blk->data);	// SHC function
            break;
        }
        case MSG_AP_CONNECT_WPS:
        {
            process_msg_ap_connect_wps(blk->callback);
            break;
        }
		case MSG_AP_DISCONNECT:
        {
            process_msg_ap_disconnect(blk->callback);
            break;
        }
        case MSG_TCP_CONNECT:
        {
            wifi_tcp_client(blk->callback, (DRVWIFI_TCP_CONFIG*)blk->data);
            break;
        }
        case MSG_TCP_SERVER:
        {
            wifi_tcp_server(blk->callback, (DRVWIFI_TCP_CONFIG*)blk->data);
            break;
        }
        case MSG_SEND:
        {
            wifi_send(blk->callback, (uint8_t*) blk->data, blk->size);
            break;
        }
		case MSG_HTTP_CONNECT:
        {
            process_msg_https_connect(blk->callback, (DRVWIFI_HTTPS_CONFIG*)blk->data);
            break;
        }
		case MSG_HTTP_POST:
        {
            process_msg_https_post(blk->callback, (DRVWIFI_HTTPS_REQ*)blk->data);
            break;
        }
		case MSG_HTTP_DISCONNECT:
        {
            process_msg_https_disconnect(blk->callback);
            break;
        }
		case MSG_PING:
        {
            process_msg_ping(blk->callback, (DRVWIFI_PING*)blk->data);
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
    DBGLOG0("drvwifi_rx_task() starts .");

    while (true) {
        DRVWIFI_MPFBLK_T* blk = NULL;
        ER er = 0;
        er = trcv_dtq(DTQ_DRVWIFI_RX, (intptr_t*) &blk, TMO_FEVR);
        assert(er == E_OK);
        switch (blk->msg) {
        case MSG_RECEIVE:
            wifi_receive(blk->callback, (uint8_t*) blk->data, blk->size);
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
void mpf_send(int msg, DRVWIFI_CALLBACK_T callback, intptr_t data, size_t size)
{
    DRVWIFI_MPFBLK_T* blk = NULL;
    ER er = 0;
    er = tget_mpf(MPF_DRVWIFI, (void**)&blk, SYSCALL_TIMEOUT);
    assert(er == E_OK);

    blk->msg = msg;
    blk->callback = callback;
    blk->data = data;
    blk->size = size;
    er = tsnd_dtq(DTQ_DRVWIFI, (intptr_t)blk, SYSCALL_TIMEOUT);
    assert(er == E_OK);
}

/*
 * mpf_send_rx
 */
void mpf_send_rx(int msg, DRVWIFI_CALLBACK_T callback, intptr_t data, size_t size)
{
    DRVWIFI_MPFBLK_T* blk = NULL;
    ER er = 0;
    er = tget_mpf(MPF_DRVWIFI, (void**)&blk, SYSCALL_TIMEOUT);
    assert(er == E_OK);

    blk->msg = msg;
    blk->callback = callback;
    blk->data = data;
    blk->size = size;
    er = tsnd_dtq(DTQ_DRVWIFI_RX, (intptr_t)blk, SYSCALL_TIMEOUT);
    assert(er == E_OK);
}

/*
 * wifiモジュール初期化
 */
void process_msg_initialize(DRVWIFI_CALLBACK_T callback)
{
    assert(callback);
    
	int recvcnt;
    ER er = 0;
    FLGPTN flgptn = 0;

    // WIFIモジュールのリセット(電源OFF->ON)
    drvcmn_gpio_pin_set(&GPIO_PIN_WIFI_ON, true);
    dly_tsk(10); // 10ms待つ
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
    // 応答受信
	recvcnt = 0;
    for (int i = 0; i < 5; i++) {
        recvcnt += uart_receive_1line(s_context.receive_buf, RECEIVE_BUF_SIZE);
  }
    DBGLOG1("recvcnt = %d", recvcnt);
    DBGLOG1("result: \"%s\"", s_context.receive_buf);
	s_context.receive_buf[0]='\0';
	
    // コマンドエコーOFF("ATE0")
    DBGLOG1("SEND: \"%s\"", ATCMD_ECHO_OFF);
    uart_send_at((const uint8_t*)ATCMD_ECHO_OFF, sizeof(ATCMD_ECHO_OFF) - 1, true);
    // 応答受信
	recvcnt = 0;
    for (int i = 0; i < 2; i++) {
		recvcnt += uart_receive_1line(s_context.receive_buf, RECEIVE_BUF_SIZE);
		if(strncmp(ATRET_OK, s_context.receive_buf ,sizeof(ATRET_OK)-1) == 0) {
			//OKなら1行受信のため、抜ける
			break;
		}
	}
    DBGLOG1("recvcnt = %d", recvcnt);
	DBGLOG1("result: \"%s\"", s_context.receive_buf);
	s_context.receive_buf[0]='\0';

    // コールバック
    callback(DRVWIFI_EVT_INITIALIZE_COMPLETE, 0, 0);
}

/*
 * wifiモジュール初期化
 */
void process_msg_initialize2(DRVWIFI_CALLBACK_T callback)
{
    assert(callback);

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

/*
 *  wifiモジュール電源Off
 */
void process_msg_power_off(DRVWIFI_CALLBACK_T callback)
{
    assert(callback);
	
	// CARD_OFF
    drvcmn_gpio_pin_set(&GPIO_PIN_WIFI_ON, false);	// WIFI_ON => L (ON)
    dly_tsk(10);
    drvcmn_gpio_pin_set(&GPIO_PIN_WIFI_ON, true);	// WIFI_ON => H (OFF)
    DBGLOG1("GPIO: \"%s\"", "WIFI OFF");
	
	// コールバック
    callback(DRVWIFI_EVT_POWER_OFF_COMPLETE, 0, 0);
}

/*
 *  APに接続
 */
void process_msg_ap_connect(DRVWIFI_CALLBACK_T callback, DRVWIFI_CONFIG* cfg)
{
    assert(callback);
    assert(cfg);

	char	buf[sizeof(ATCMD_APCONNECT)+16+DRVWIFI_MAX_PASSPHRASE_LEN];
	int		cnt, recvcnt;
    ER er = 0;
    FLGPTN flgptn = 0;
	int32_t	ret=0;


    // WIFIモジュールに接続コマンドを送る
	cnt = sizeof(ATCMD_APCONNECT)-1;
	memcpy(buf, ATCMD_APCONNECT, cnt);									//コマンド文字
	buf[cnt] = '=';
	cnt += 1;
	memcpy(&(buf[cnt]), cfg->essid, cfg->essid_len);						//SSID
	cnt += cfg->essid_len;
	if(cfg->wpaver == DRVWIFI_SECURITY_WPA) {
		memcpy(&(buf[cnt]), ",1", 2);										//WPA ver
		cnt += 2;
	} else {
		memcpy(&(buf[cnt]), ",2", 2);										//WPA ver
		cnt += 2;
	}
	if(cfg->cipher == DRVWIFI_SECURITY_TKIP) {
		memcpy(&(buf[cnt]), ",TKIP,TKIP,", 11);								//暗号指定
		cnt += 11;
	} else {
		memcpy(&(buf[cnt]), ",CCMP,CCMP,", 11);							//暗号指定
		cnt += 11;
	}
	memcpy(&(buf[cnt]), cfg->passphrase, cfg->passphrase_len);		    //パスワード
	cnt += cfg->passphrase_len;
	buf[cnt] = '\0';

	// コマンド発行
    DBGLOG2("SEND: \"%s\"  >num=%d", buf, cnt);
    uart_send_at((const uint8_t*)buf, cnt, false);
	
    // 応答2行受信
	recvcnt = 0;
    for (int i = 0; i < 2; i++) {
        recvcnt += uart_receive_1line(s_context.receive_buf, RECEIVE_BUF_SIZE);
		if(strncmp(ATRET_OK, s_context.receive_buf ,sizeof(ATRET_OK)-1) == 0) {
			//OKなら1行受信のため、抜ける
			ret = (int32_t)0;
			break;
		} else {
			ret = (int32_t)-1;
		}
    }
    DBGLOG1("recvcnt = %d", recvcnt);
    DBGLOG1("result: \"%s\"", s_context.receive_buf);
	s_context.receive_buf[0]='\0';

	// コールバック
    callback(DRVWIFI_EVT_AP_CONNECT_COMPLETE, ret, 0);
}

/*
 * WPSでAPに接続
 */
void process_msg_ap_connect_wps(DRVWIFI_CALLBACK_T callback)
{
    assert(callback);

	char	buf[sizeof(ATCMD_APCONNECT_WPS)+8];
	int		cnt, recvcnt;
    ER er = 0;
    FLGPTN flgptn = 0;
	int32_t	ret=0;

    // WIFIモジュールに接続コマンドを送る
	cnt = sizeof(ATCMD_APCONNECT_WPS)-1;
	memcpy(buf, ATCMD_APCONNECT_WPS, cnt);				//コマンド文字
	buf[cnt] = '=';
	cnt += 1;
	memcpy(&buf[cnt], "1,PUSH", 6);								//パラメタ
	cnt += 6;
	buf[cnt] = '\0';

	// コマンド発行
	DBGLOG2("SEND: \"%s\"  >num=%d", buf, cnt);
	uart_send_at((const uint8_t*)buf, cnt, false);
	
    // 応答2行受信
	recvcnt = 0;
    for (int i = 0; i < 2; i++) {
        recvcnt += uart_receive_1line(s_context.receive_buf, RECEIVE_BUF_SIZE);
		if(strncmp(ATRET_OK, s_context.receive_buf ,sizeof(ATRET_OK)-1) == 0) {
			//OKなら1行受信のため、抜ける
			ret = (int32_t)0;
		break;
		} else {
			ret = (int32_t)-1;
		}
    }
    DBGLOG1("recvcnt = %d", recvcnt);
    DBGLOG1("result: \"%s\"", s_context.receive_buf);
	s_context.receive_buf[0]='\0';

	// コールバック
    callback(DRVWIFI_EVT_AP_CONNECT_WPS_COMPLETE, ret, 0);
}

/*
 * AP切断
 */
void process_msg_ap_disconnect(DRVWIFI_CALLBACK_T callback)
{
    assert(callback);
	
	int recvcnt;
    ER er = 0;
    FLGPTN flgptn = 0;
	int32_t	ret=0;

	// 切断コマンドを送る
    DBGLOG1("SEND: \"%s\"", ATCMD_APDISCONNECT);
    uart_send_at((const uint8_t*)ATCMD_APDISCONNECT, sizeof(ATCMD_APDISCONNECT) - 1, false);	
	
    // 応答2行受信
	recvcnt = 0;
    for (int i = 0; i <2; i++) {
        recvcnt += uart_receive_1line(s_context.receive_buf, RECEIVE_BUF_SIZE);
		if(strncmp(ATRET_OK, s_context.receive_buf ,sizeof(ATRET_OK)-1) == 0) {
			//OKなら1行受信のため、抜ける
			ret = (int32_t)0;
			break;
		} else {
			ret = (int32_t)-1;
		}
    }
    DBGLOG1("recvcnt = %d", recvcnt);
    DBGLOG1("result: \"%s\"", s_context.receive_buf);
	s_context.receive_buf[0]='\0';

	// コールバック
    callback(DRVWIFI_EVT_AP_DISCONNECTED, ret, 0);
}


/*
 * HTTPS 接続
 */
void process_msg_https_connect(DRVWIFI_CALLBACK_T callback, DRVWIFI_HTTPS_CONFIG* httpcfg)
{
    assert(callback);
    assert(httpcfg);
	
	char	buf[128];
	int		cnt, recvcnt;
    ER er = 0;
    FLGPTN flgptn = 0;
	int32_t	ret=0;

    /* TLSコンテキスト初期化 */
	cnt = sizeof(ATCMD_HTTPS_TLSINIT)-1;
	memcpy(buf, ATCMD_HTTPS_TLSINIT, cnt);			//コマンド文字
	memcpy(&buf[cnt], "=1", 2);								//パラメタ
	cnt += 2;
	buf[cnt] = '\0';
	// コマンド発行
    //dly_tsk(1000);
	DBGLOG2("SEND: \"%s\"  >num=%d", buf, cnt);
    uart_send_at((const uint8_t*)buf, cnt, false);
    // 応答2行受信
	recvcnt = 0;
    for (int i = 0; i < 2; i++) {
        recvcnt += uart_receive_1line(s_context.receive_buf, RECEIVE_BUF_SIZE);
		if(strncmp(ATRET_OK, s_context.receive_buf ,sizeof(ATRET_OK)-1) == 0) {
			//OKなら1行受信のため、抜ける
			ret = (int32_t)0;
			break;
		} else {
			ret = (int32_t)-1;
		}
    }
    DBGLOG1("recvcnt = %d", recvcnt);
    DBGLOG1("result: \"%s\"", s_context.receive_buf);
	s_context.receive_buf[0]='\0';

	
	/* HTTPSクライアント接続 */
	cnt = sizeof(ATCMD_HTTPS_CONN)-1;
	memcpy(buf, ATCMD_HTTPS_CONN, cnt);			//コマンド文字
	buf[cnt] = '=';
	cnt += 1;
	if(httpcfg->schema == DRVWIFI_SCHEMA_HTTPS) {
		memcpy(&buf[cnt], "https://", 8);				//https指定
		cnt += 8;
	}
	memcpy(&buf[cnt], httpcfg->hostname, httpcfg->hostname_len);			//パラメタ：ホスト名
	cnt += httpcfg->hostname_len;
	buf[cnt] = '\0';
	// コマンド発行
	DBGLOG2("SEND: \"%s\"  >num=%d", buf, cnt);
    uart_send_at((const uint8_t*)buf, cnt, false);
    // 応答2行受信
	recvcnt = 0;
    for (int i = 0; i < 2; i++) {
        recvcnt += uart_receive_1line(s_context.receive_buf, RECEIVE_BUF_SIZE);
 		if(strncmp(ATRET_OK, s_context.receive_buf ,sizeof(ATRET_OK)-1) == 0) {
			//OKなら1行受信のため、抜ける
			ret = (int32_t)0;
			break;
		} else {
			ret = (int32_t)-1;
		}
   }
    DBGLOG1("recvcnt = %d", recvcnt);
    DBGLOG1("result: \"%s\"", s_context.receive_buf);
	s_context.receive_buf[0]='\0';

	
	/* 拡張HTTPSヘッダー設定 */
	if((httpcfg->exhdrname_len != 0) || (httpcfg->exhdrval_len != 0)) {
		cnt = sizeof(ATCMD_HTTPS_EXHDR)-1;
		memcpy(buf, ATCMD_HTTPS_EXHDR, cnt);			//コマンド文字
		memcpy(&buf[cnt], "=", 1);
		cnt += 1;
		memcpy(&buf[cnt], httpcfg->exhdrname, httpcfg->exhdrname_len);		//拡張ヘッダname
		cnt += httpcfg->exhdrname_len;
		memcpy(&buf[cnt], ":", 1);
		cnt += 1;
		memcpy(&buf[cnt], httpcfg->exhdrvalue, httpcfg->exhdrval_len);		//拡張ヘッダvalue
		cnt += httpcfg->exhdrval_len;
		buf[cnt] = '\0';
		// コマンド発行
		DBGLOG2("SEND: \"%s\"  >num=%d", buf, cnt);
		uart_send_at((const uint8_t*)buf, cnt, false);
		// 応答2行受信
		recvcnt = 0;
		for (int i = 0; i < 2; i++) {
			recvcnt += uart_receive_1line(s_context.receive_buf, RECEIVE_BUF_SIZE);
			if(strncmp(ATRET_OK, s_context.receive_buf ,sizeof(ATRET_OK)-1) == 0) {
			//OKなら1行受信のため、抜ける
				ret = (int32_t)0;
				break;
			} else {
				ret = (int32_t)-1;
			}
		}
		DBGLOG1("recvcnt = %d", recvcnt);
		DBGLOG1("result: \"%s\"", s_context.receive_buf);
		s_context.receive_buf[0]='\0';
	}

	
	// コールバック
    callback(DRVWIFI_EVT_HTTP_CONNECT_COMPLETE, ret, 0);
}

/*
 * HTTPS POST
 */
void process_msg_https_post(DRVWIFI_CALLBACK_T callback, DRVWIFI_HTTPS_REQ* hreq)
{
    assert(callback);
    assert(hreq);

	char	buf[DRVWIFI_MAX_BODY_LEN+sizeof(ATCMD_HTTPS_BODY)+1];
	int		cnt, recvsz, recvcnt;
	int32_t	ret=0;
    ER er = 0;
    FLGPTN flgptn = 0;

    /* 本文設定 */
	if(hreq->body_len != 0) {
		cnt = sizeof(ATCMD_HTTPS_BODY)-1;
		memcpy(buf, ATCMD_HTTPS_BODY, cnt);			//コマンド文字
		memcpy(&buf[cnt], "=", 1);
		cnt += 1;
		memcpy(&buf[cnt], hreq->body, hreq->body_len);	//パラメタ：本文
		cnt += hreq->body_len;
		buf[cnt] = '\0';
		// コマンド発行
		DBGLOG2("SEND: \"%s\"  >num=%d", buf, cnt);
		uart_send_at((const uint8_t*)buf, cnt, false);
		//結果コード取得
		recvcnt = 0;
		for (int i = 0; i < 2; i++) {
			recvcnt += uart_receive_1line(s_context.receive_buf, RECEIVE_BUF_SIZE);
			if(strncmp(ATRET_OK, s_context.receive_buf ,sizeof(ATRET_OK)-1) == 0) {
				//OKなら1行受信のため、抜ける
				ret = (int32_t)0;
				break;
			} else {
				ret = (int32_t)-1;
			}
		}
		DBGLOG1("recvcnt = %d", recvcnt);
		DBGLOG1("result: \"%s\"", s_context.receive_buf);
		s_context.receive_buf[0]='\0';
	}
	
	/* HTTPS POST送信 */
	cnt = sizeof(ATCMD_HTTPS_POST)-1;
	memcpy(buf, ATCMD_HTTPS_POST, cnt);			//コマンド文字
	memcpy(&buf[cnt], "=", 1);
	cnt += 1;
	memcpy(&buf[cnt], hreq->path, hreq->path_len);	//パラメタ：パス
	cnt += hreq->path_len;
	buf[cnt] = '\0';
	// コマンド発行
	DBGLOG2("SEND: \"%s\"  >num=%d", buf, cnt);
    uart_send_at((const uint8_t*)buf, cnt, false);
	// 応答受信

//SilexのUART仕様と実際が異なり、コンテンツの長さが応答パラメタにないため、以下のコードをコメントアウト
	//まず2行取得し、受信サイズを獲得
	recvcnt = 0;
    for (int i = 0; i < 2; i++) {
		recvcnt += uart_receive_1line(s_context.receive_buf, RECEIVE_BUF_SIZE);
	}
    DBGLOG1("recvcnt = %d", recvcnt);
    DBGLOG1("recv 1line: \"%s\"", s_context.receive_buf);
	for(int i=0; i<RECEIVE_BUF_SIZE; i++) {
		//カンマの次から改行までが受信サイズ
		if(s_context.receive_buf[i] == ',') {
			recvsz = atoi(&(s_context.receive_buf[i+1]));
			break;
		}
	}
	DBGLOG1("recvsz = %d", recvsz);
	//応答コンテンツ取得
	s_context.receive_post[0] = '\0';
    if(recvsz > RECEIVE_POST_SIZE) {
        recvsz = RECEIVE_POST_SIZE;
    }
	if(recvsz > 0)	{
		uart_receive(s_context.receive_post, recvsz);
		DBGLOG1("recv left: \"%s\"", s_context.receive_post);
	}
	recvcnt = 0;
    for (int i = 0; i < 2; i++) {
		recvcnt += uart_receive_1line(s_context.receive_buf, RECEIVE_BUF_SIZE);
		if(strncmp(ATRET_OK, s_context.receive_buf ,sizeof(ATRET_OK)-1) == 0) {
			//OKなら1行受信のため、抜ける
			ret = (int32_t)0;
			break;
		} else {
			ret = (int32_t)-1;
		}
	}
    DBGLOG1("recvcnt = %d", recvcnt);
    DBGLOG1("result: \"%s\"", s_context.receive_buf);
	s_context.receive_buf[0]='\0';

	// コールバック
    callback(DRVWIFI_EVT_HTTP_POST_COMPLETE, ret, (intptr_t)s_context.receive_post);
}

/*
 * HTTPS 切断
 */
void process_msg_https_disconnect(DRVWIFI_CALLBACK_T callback)
{
    assert(callback);
	
	int recvcnt;
    ER er = 0;
    FLGPTN flgptn = 0;
	int32_t	ret=0;

	// 切断コマンドを送る
    DBGLOG1("SEND: \"%s\"", ATCMD_HTTPS_DISCONN);
    uart_send_at((const uint8_t*)ATCMD_HTTPS_DISCONN, sizeof(ATCMD_HTTPS_DISCONN) - 1, false);	
	
    // 応答2行受信
	recvcnt = 0;
    for (int i = 0; i <2; i++) {
        recvcnt += uart_receive_1line(s_context.receive_buf, RECEIVE_BUF_SIZE);
		if(strncmp(ATRET_OK, s_context.receive_buf ,sizeof(ATRET_OK)-1) == 0) {
			//OKなら1行受信のため、抜ける
			ret = (int32_t)0;
			break;
		} else {
			ret = (int32_t)-1;
		}
    }
    DBGLOG1("recvcnt = %d", recvcnt);
    DBGLOG1("result: \"%s\"", s_context.receive_buf);
	s_context.receive_buf[0]='\0';

	// コールバック
    callback(DRVWIFI_EVT_HTTP_DISCONNECT_COMPLETE, ret, 0);
}

/*
 * PING
 */
void process_msg_ping(DRVWIFI_CALLBACK_T callback, DRVWIFI_PING* ping)
{
    assert(callback);

	char	buf[DRVWIFI_MAX_IPADR_LEN+sizeof(ATCMD_PING)+1];
	int		cnt, recvcnt;
	int32_t	ret=0;
    ER er = 0;
    FLGPTN flgptn = 0;
	
	/* PING送信 */
	cnt = sizeof(ATCMD_PING)-1;
	memcpy(buf, ATCMD_PING, cnt);			//コマンド文字
	memcpy(&buf[cnt], "=", 1);
	cnt += 1;
	memcpy(&buf[cnt], ping->ip_address, ping->ipadr_len);	//パラメタ：ip adr
	cnt += ping->ipadr_len;
	buf[cnt] = '\0';
	// コマンド発行
	DBGLOG2("SEND: \"%s\"  >num=%d", buf, cnt);
    uart_send_at((const uint8_t*)buf, cnt, false);
	
    // 応答受信
	recvcnt = 0;
    for (int i = 0; i <3; i++) {
        recvcnt += uart_receive_1line(s_context.receive_buf, RECEIVE_BUF_SIZE);
		if(strncmp(ATRET_OK, s_context.receive_buf ,sizeof(ATRET_OK)-1) == 0) {
			//OKなら1行受信のため、抜ける
			ret = (int32_t)0;
			break;
		} else {
			ret = (int32_t)-1;
		}
    }
    DBGLOG1("recvcnt = %d", recvcnt);
    DBGLOG1("result: \"%s\"", s_context.receive_buf);
	s_context.receive_buf[0]='\0';

	// コールバック
    callback(DRVWIFI_EVT_PING_COMPLETE, ret, 0);
}

#if 1
char* wifi_get_ip_address(void)
{
    DBGLOG1("SEND: \"%s\"", ATCMD_QUERY_IP4_SETTING);
    uart_send_at((const uint8_t*) ATCMD_QUERY_IP4_SETTING, sizeof(ATCMD_QUERY_IP4_SETTING) - 1, false);
    wifi_get_result();
    return s_ip_address;
}
#endif

void process_msg_ap_connect2(DRVWIFI_CALLBACK_T callback, DRVWIFI_CONFIG *cfg)
{
    assert(callback);
    assert(cfg);

    uint8_t cmd[100];
    int result;
    DBGLOG0("drvwifi_ap_connect() called.....\n");

    uint8_t essid[DRVWIFI_MAX_ESSID_LEN];
    uint8_t passphrase[DRVWIFI_MAX_PASSPHRASE_LEN];

    memcpy(essid, cfg->essid, cfg->essid_len);
    memcpy(passphrase, cfg->passphrase, cfg->passphrase_len);

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

    if (callback) {
        callback(DRVWIFI_EVT_AP_CONNECT_COMPLETE, ip_addr, 0);
    }
}

void wifi_tcp_client(DRVWIFI_CALLBACK_T callback, DRVWIFI_TCP_CONFIG* tcpcfg)
{
    assert(callback);
    assert(tcpcfg);

    DBGLOG0("wifi_tcp_client()\n");
    int result;

    uint8_t ip_addr[DRVWIFI_MAX_IPADR_LEN];
    uint16_t port;

    memcpy(ip_addr, tcpcfg->ip_address, tcpcfg->ipadr_len);
    port = tcpcfg->port;

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

	// strcpy((char*)s_tcp_config.ip_address, (char*)ip_address);
	// s_tcp_config.port = port;
    // Connect to TCP server
    uint8_t cmd[100];
    sprintf((char*)cmd, ATCMD_TCP_CLIENT, ip_addr, port);

    // Send command to the WIFI module
    DBGLOG2("SEND (%d): \"%s\"", strlen((char*)cmd), cmd);
    uart_send_at(cmd, strlen((char*)cmd), false);
    result = wifi_get_result();

    if (callback && result == DRVWIFI_RESULT_CONNECT) {
        callback(DRVWIFI_EVT_TCP_CONNECT_COMPLETE, 0, 0);
    }
}

void wifi_tcp_server(DRVWIFI_CALLBACK_T callback, DRVWIFI_TCP_CONFIG* tcpcfg)
{
    assert(callback);
    assert(tcpcfg);

    DBGLOG0("wifi_tcp_server()\n");
    int result;

    uint16_t port;

    port = tcpcfg->port;

    // Send command to the WIFI module
    DBGLOG1("SEND: \"%s\"", ATCMD_CREATE_SOCKET);
    uart_send_at((const uint8_t*) ATCMD_CREATE_SOCKET, sizeof(ATCMD_CREATE_SOCKET) - 1, false);
    result = wifi_get_result();

    DBGLOG1("SEND: \"%s\"", ATCMD_TCP_SERVER_TIMEOUT);
    uart_send_at((const uint8_t*) ATCMD_TCP_SERVER_TIMEOUT, strlen(ATCMD_TCP_SERVER_TIMEOUT) - 1, false);
    result = wifi_get_result();

	// s_tcp_config.port = port;
    uint8_t cmd[100];
    sprintf((char*) cmd, ATCMD_TCP_SERVER, port);

    // Send command to the WIFI module
    DBGLOG2("SEND (%d): \"%s\"", strlen((char* )cmd), cmd);
    uart_send_at(cmd, strlen((char*) cmd), false);

    result = wifi_get_result();

    if (callback && result == DRVWIFI_RESULT_CONNECT) {
        callback(DRVWIFI_EVT_TCP_SERVER_COMPLETE, 0, 0);
    }
}


int wifi_get_result(void)
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

void wifi_send(DRVWIFI_CALLBACK_T callback, const uint8_t* data, size_t size)
{
    assert(data);
    assert(size > 0);

    int error = uart_send(data, size);

    if (callback) {
        callback(DRVWIFI_EVT_TCP_SEND_COMPLETE, error, 0);
    }
}

void wifi_receive(DRVWIFI_CALLBACK_T callback, uint8_t* data, size_t size)
{
    assert(size > 0);

    size_t count = 0;
    int error = uart_receive_wait(&count, data, size, TMO_FEVR);

    if (callback) {
        callback(DRVWIFI_EVT_TCP_RECEIVE_COMPLETE, error, count);
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
 * UARTでATコマンド送信(CRを付加して送信)
 */
int uart_send_at(const uint8_t* data, size_t data_len, bool_t discard_echo)
{
    assert(data);
    assert(data_len > 0);

    FLGPTN flgptn = 0;
    ER er = 0;
	uint8_t buf[1024];

    // イベントフラグをクリア
    er = clr_flg(FLG_DRVWIFI, ~FLGPTN_UART_TXCOMPLETE);
    assert(er == E_OK);


    // コマンド送信
	memcpy(buf, data, data_len);
	buf[data_len] = '\r'; //CR追加
//	buf[data_len+1] = '\n'; //LF追加
	DBGLOG2("cmd>%s    cmd_num>%d", buf, data_len+1);
    drvcmn_uart_send(WIFI_UART, buf, data_len+1);

    // 送信完了を待つ
    er = twai_flg(FLG_DRVWIFI, FLGPTN_UART_TXCOMPLETE, TWF_ANDW, &flgptn, UART_TIMEOUT);
    assert((er == E_OK) || (er == E_TMOUT));

	// タイムアウトの場合は中断処理
    if (er == E_TMOUT) {
        drvcmn_uart_cancel_send(WIFI_UART);
    }

    if (discard_echo) {
        // コマンドエコー受信(読み捨て)完了を待つ
		uart_receive(NULL, data_len + 1);
    }

    return 0;
}

/*
 * UART受信(サイズ指定)
 */
int uart_receive(uint8_t* data, size_t data_len)
{
    ER er = 0;
    FLGPTN flgptn = 0;

    // イベントフラグをクリア
    er = clr_flg(FLG_DRVWIFI, ~FLGPTN_UART_RXCOMPLETE);
    assert(er == E_OK);

    // 受信
    drvcmn_uart_receive(WIFI_UART, data, data_len);

    // 受信完了を待つ
    er = twai_flg(FLG_DRVWIFI, FLGPTN_UART_RXCOMPLETE, TWF_ANDW, &flgptn, UART_TIMEOUT);
    assert((er == E_OK) || (er == E_TMOUT));

    // タイムアウトの場合は中断処理
    if (er == E_TMOUT) {
        drvcmn_uart_cancel_receive(WIFI_UART);
    }

    return 0;
}

/*
 * UART受信(サイズ指定) + wait
 */
int uart_receive_wait(size_t* count, uint8_t* data, size_t data_len, TMO timeout)
{
    assert(count);
    assert(data_len > 0);

    int error = DRVWIFI_ERROR_NONE;
    ER er = 0;
    FLGPTN flgptn = 0;

    // イベントフラグをクリア
    er = clr_flg(FLG_DRVWIFI, ~FLGPTN_UART_RXCOMPLETE);
    assert(er == E_OK);

    // 受信
    drvcmn_uart_receive(WIFI_UART, data, data_len);

    // 送信完了を待つ
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
    const size_t PATTERN_LEN = (sizeof(CRLF)-1);	// 終端文字を除く
    ER er = 0;
    FLGPTN flgptn = 0;

	// イベントフラグをクリア
    er = clr_flg(FLG_DRVWIFI, ~FLGPTN_UART_RXCOMPLETE);
    assert(er == E_OK);
	
    size_t received = 0;
    size_t match_count = 0;
    for (received = 0; received < data_limit; received++) {
        // 1文字受信
        drvcmn_uart_receive(WIFI_UART, &(data[received]), 1);
	
        // 送信完了を待つ(コマンドエコーを読み捨てる場合は受信完了も待つ)
        er = twai_flg(FLG_DRVWIFI, FLGPTN_UART_RXCOMPLETE, TWF_ANDW, &flgptn, UART_TIMEOUT);
        assert((er == E_OK)  || (er == E_TMOUT));
        if (er != E_OK) {
    // タイムアウトの場合は中断処理
			if (er == E_TMOUT) {
				drvcmn_uart_cancel_receive(WIFI_UART);
			}
            break;
        }

        // 比較
//        if ((data[received] == PATTERN[0]) && (data[received+1] == PATTERN[1])) {
//                DBGLOG2("string>%s, recv_num>%d", data, received+1);
//                break;
//		}

		if (data[received] == PATTERN[match_count]) {
            match_count++;
            if (match_count >= PATTERN_LEN) {
//                DBGLOG1("match!(%d)", received+1);
//                DBGLOG2("string>%s    recv_num>%d", data, received+1);
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
