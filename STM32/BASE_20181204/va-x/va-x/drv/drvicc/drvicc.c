/*
 * VA-X ICカードリーダードライバ (ARI3030)
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/03/22 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "drvicc.h"

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
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[DRVICC]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[DRVICC]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[DRVICC]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[DRVICC]" msg, arg1, arg2, arg3)
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
    DRVICC_CALLBACK_T callback;
    intptr_t opt1;
    intptr_t opt2;
} DRVICC_MPFBLK_T;

// DRVICC_MPFBLK_SIZE は sizeof(DRVICC_MPFBKL) に一致させること
static_assert(sizeof(DRVICC_MPFBLK_T) == DRVICC_MPFBLK_SIZE, "MPF size");

/*
 * 内部関数プロトタイプ
 */
// メモリプールブロックを使ってメッセージ送信
static void mpf_send(int msg, DRVICC_CALLBACK_T callback, intptr_t opt1, intptr_t opt2);

// ICCモジュール初期化
static void process_msg_initialize(DRVICC_CALLBACK_T callback);

// ICCモジュール電源OFF
static void process_msg_power_off(DRVICC_CALLBACK_T callback);

// ポーリング
static void process_msg_start_polling(DRVICC_CALLBACK_T callback, DRVICC_FELICA_IDM_T* idm);

/* カード読み取り */
static void process_msg_read(DRVICC_CALLBACK_T callback, DRVICC_FELICA_BLOCK_T* blocks, const DRVICC_FELICA_BLOCK_DESC_T* desc);

/* カード書込み */
static void process_msg_write(DRVICC_CALLBACK_T callback, const DRVICC_FELICA_BLOCK_T* blocks, const DRVICC_FELICA_BLOCK_DESC_T* desc);

// UART送信
static int uart_send(const uint8_t* data, size_t size);

// UART受信
static int uart_receive(uint8_t* data, size_t data_len, TMO timeout);

// コマンド組み立て
static size_t build_command_frame(uint8_t* output, uint8_t command_code, const uint8_t* optional_data, size_t data_len);

// 1フレーム読み込み
static int read_inf_frame(uint8_t* data, size_t max_len);

// ACKフレーム待ち
static int wait_ack_frame();

// PN532 コマンド送受信
static int pn532_send_receive(uint8_t command_code, const uint8_t* optional_data, size_t data_len);

// UART ISRコールバック
static bool_t uart_isr_callback(int type, uint32_t opt);

/*
 * 定数定義
 */

// GPIO(UART)
static const DRVCMN_GPIO_PIN_T GPIO_PINS_UART[] = {
    { DRVCMN_GPIO_PORT_B, 12 },	// AF8: UART5_RX
    { DRVCMN_GPIO_PORT_B, 13 },	// AF8: UART5_TX
};

// GPIO設定(UART)
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_UART = {
    .mode = DRVCMN_GPIO_MODE_AF,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLUP,
    .afno = 8,
};

// CARD_ON
static const DRVCMN_GPIO_PIN_T GPIO_PIN_CARD_ON = { DRVCMN_GPIO_PORT_E, 12 };	// E12: CARD_ON

// GPIO設定(CARD_ON)
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_CARD_ON = {
    .mode = DRVCMN_GPIO_MODE_GPOUT,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLUP,
    .afno = 0,
};

// UART設定
static const int ICC_UART = 5;	// UART5
static const DRVCMN_UART_SETTING_T UART_SETTING = {
    .clocksel = DRVCMN_UART_CLOCKSEL_SYS,
    .baudrate = 115200,
    .wordlen = DRVCMN_UART_WORD_8BIT,
    .parity = DRVCMN_UART_PARITY_NONE,
    .stop = DRVCMN_UART_STOP_1,
    .hwflow = false,
    .callback = uart_isr_callback,
};

// ポーリング周期[msec]
static const int POLLING_PERIOD = 250;	// 250ms

// メッセージ番号
enum {
    MSG_INITIALIZE = 1,
    MSG_POWER_OFF,
    MSG_START_POLLING,
    MSG_CANCEL_POLLING,
    MSG_READ,
    MSG_WRITE,
};

// イベントフラグ
static const FLGPTN FLGPTN_UART_TXCOMPLETE = (0x1 << 0);
static const FLGPTN FLGPTN_UART_RXCOMPLETE = (0x1 << 1);
static const FLGPTN FLGPTN_CANCEL_POLLING = (0x1 << 2);

// タイムアウト
static const int32_t SYSCALL_TIMEOUT = 1000;
static const int32_t RESPONSE_TIMEOUT = 1000;
static const int32_t ACK_TIMEOUT = 10;

// バッファサイズ
#define RX_BUF_SIZE	(1024)
#define TX_BUF_SIZE	(64)

// 応答パケット
static const uint8_t PN532_ACK_FRAME[] = {0x00, 0x00, 0xff, 0x00, 0xff, 0x00};	// ACK
static const uint8_t PN532_NACK_FRAME[] = {0x00, 0x00, 0xff, 0xff, 0x00, 0x00};	// NACK

// テストデータ
static const uint8_t TEST_DATA[] = {0x00, 'T', 'E', 'S', 'T'};

// フレームフォーマット
static const size_t FRAME_OFFSET_DATA = 6;
static const size_t FRAME_OVERHEAD_SIZE = 8;

/*
 * 内部変数
 */
// コンテキスト情報
static struct {
    uint8_t rx_buf[RX_BUF_SIZE];
    size_t rx_buf_pos;
    uint8_t tx_buf[TX_BUF_SIZE];
} s_context;


/*********************************************************************
 * 公開関数
 ********************************************************************/

/*
 * ペリフェラル初期化
 */
void drvicc_initialize_peripherals()
{
    // GPIO初期化(UART)
    for (int i = 0; i < sizeof(GPIO_PINS_UART) / sizeof(GPIO_PINS_UART[0]); i++) {
        drvcmn_gpio_pin_initialize(&(GPIO_PINS_UART[i]), &GPIO_SETTING_UART);
    }

    // CARD_ONピン
    drvcmn_gpio_pin_initialize(&GPIO_PIN_CARD_ON, &GPIO_SETTING_CARD_ON);
    drvcmn_gpio_pin_set(&GPIO_PIN_CARD_ON, true);	// CARD_ON => H (OFF)
}

/*
 * ドライバ初期化
 */
void drvicc_initialize(DRVICC_CALLBACK_T callback)
{
    ER er = 0;

    // ドライバタスク起動
    act_tsk(TSK_DRVICC);
    assert(er == E_OK);

    // 初期化メッセージを送信
    mpf_send(MSG_INITIALIZE, callback, 0, 0);
}

/*
 * ドライバ初期化（タスク起動済み）
 */
 void drvicc_start(DRVICC_CALLBACK_T callback)
{
    ER er = 0;

    // 初期化メッセージを送信
    mpf_send(MSG_INITIALIZE, callback, 0, 0);
}

/*
 * ドライバ終了
 */
void drvicc_power_off(DRVICC_CALLBACK_T callback)
{
    // 終了メッセージを送信
    mpf_send(MSG_POWER_OFF, callback, 0, 0);
}

/*
 * メッセージを送らず直接ドライバ終了
 */
void drvicc_power_off_nomsg()
{
    // 直接CARD_OFF
    drvcmn_gpio_pin_set(&GPIO_PIN_CARD_ON, false);	// CARD_ON => L (ON)
    dly_tsk(10);
    drvcmn_gpio_pin_set(&GPIO_PIN_CARD_ON, true);	// CARD_ON => H (OFF)
    dly_tsk(10);
}

/*
 * ポーリング開始
 */
void drvicc_start_polling(DRVICC_CALLBACK_T callback, DRVICC_FELICA_IDM_T* idm)
{
    // キャンセルフラグリセット
    ER er = clr_flg(FLG_DRVICC, ~FLGPTN_CANCEL_POLLING);
    assert(er == E_OK);

    // メッセージを送信
    mpf_send(MSG_START_POLLING, callback, (intptr_t)idm, 0);
}

/*
 * ポーリングキャンセル
 */
void drvicc_cancel_polling(DRVICC_CALLBACK_T callback)
{
    // キャンセルフラグセット
    ER er = set_flg(FLG_DRVICC, FLGPTN_CANCEL_POLLING);
    assert(er == E_OK);

    // メッセージを送信
    mpf_send(MSG_CANCEL_POLLING, callback, 0, 0);
}

/* カード読み取り */
void drvicc_read(DRVICC_CALLBACK_T callback, DRVICC_FELICA_BLOCK_T* blocks, const DRVICC_FELICA_BLOCK_DESC_T* desc)
{
    // メッセージを送信
    mpf_send(MSG_READ, callback, (intptr_t)blocks, (intptr_t)desc);
}

/* カード書込み */
void drvicc_write(DRVICC_CALLBACK_T callback, const DRVICC_FELICA_BLOCK_T* blocks, const DRVICC_FELICA_BLOCK_DESC_T* desc)
{
    // メッセージを送信
    mpf_send(MSG_WRITE, callback, (intptr_t)blocks, (intptr_t)desc);
}

/*
 * タスク
 */
void drvicc_task(intptr_t exinf)
{
    DBGLOG0("drvicc_task() starts .");
    bool_t finalize = false;

    while (true) {
        DRVICC_MPFBLK_T* blk = NULL;
        ER er = 0;
        er = trcv_dtq(DTQ_DRVICC, (intptr_t*)&blk, TMO_FEVR);
        assert(er == E_OK);
        DBGLOG1("drvicc_task(): msg=%d", blk->msg);
        switch (blk->msg) {
        case MSG_INITIALIZE:
        {
            process_msg_initialize(blk->callback);
            break;
        }
        case MSG_POWER_OFF:
        {
            process_msg_power_off(blk->callback);
            finalize = true;
            break;
        }
        case MSG_START_POLLING:
        {
            process_msg_start_polling(blk->callback, (DRVICC_FELICA_IDM_T*)blk->opt1);
            break;
        }
        case MSG_CANCEL_POLLING:
        {
            // コールバックのみ
            assert(blk->callback);
            blk->callback(DRVICC_EVT_CANCEL_POLLING_COMPLETE, 0, 0);
            break;
        }
        case MSG_READ:
        {
            process_msg_read(blk->callback, (DRVICC_FELICA_BLOCK_T*)blk->opt1, (const DRVICC_FELICA_BLOCK_DESC_T*)blk->opt2);
            break;
        }
        case MSG_WRITE:
        {
            process_msg_write(blk->callback, (const DRVICC_FELICA_BLOCK_T*)blk->opt1, (const DRVICC_FELICA_BLOCK_DESC_T*)blk->opt2);
            break;
        }
        default:
            assert(false);
            break;
        }

        er = rel_mpf(MPF_DRVICC, blk);
        assert(er == E_OK);

        if (finalize) {
            // ドライバタスク終了
            ter_tsk(TSK_DRVICC);
        }
    }
}

/*
 * 受信タスク
 */
void drvicc_rx_task(intptr_t exinf)
{
    DBGLOG0("drvicc_rx_task() starts .");
    
}

/*********************************************************************
 * 内部関数
 ********************************************************************/

/*
 * メッセージ送信
 */
void mpf_send(int msg, DRVICC_CALLBACK_T callback, intptr_t opt1, intptr_t opt2)
{
    DRVICC_MPFBLK_T* blk = NULL;
    ER er = 0;
    er = tget_mpf(MPF_DRVICC, (void**)&blk, SYSCALL_TIMEOUT);
    assert(er == E_OK);

    blk->msg = msg;
    blk->callback = callback;
    blk->opt1 = opt1;
    blk->opt2 = opt2;
    er = tsnd_dtq(DTQ_DRVICC, (intptr_t)blk, SYSCALL_TIMEOUT);
    assert(er == E_OK);
}

/*
 * ドライバ初期化
 */
void process_msg_initialize(DRVICC_CALLBACK_T callback)
{
    assert(callback);
    int error = 0;
    int frame_size = 0;

    // UART初期化
    drvcmn_uart_initialize(ICC_UART, &UART_SETTING);

    // CARD_ON
    drvcmn_gpio_pin_set(&GPIO_PIN_CARD_ON, true);	// CARD_ON => H (OFF)
    dly_tsk(10);
    drvcmn_gpio_pin_set(&GPIO_PIN_CARD_ON, false);	// CARD_ON => L (ON)
    dly_tsk(10);

    int packet_len = 0;
    pn532_send_receive(0x00, TEST_DATA, sizeof(TEST_DATA));

    DBGLOG2("inf_frame(%d) \"%s\"", frame_size, s_context.rx_buf);
    if (memcmp(&(s_context.rx_buf[FRAME_OFFSET_DATA+2]), "TEST", 4) == 0) {
        DBGLOG0("UART TEST OK");
    } else {
        DBGLOG0("UART TEST NG");
        error = 1;
        goto end;
    }

    // MaxRetries
    static const uint8_t CFG_MAXRETRIES[] = {0x05, 0x02, 0x02, 0x05};
    pn532_send_receive(0x32, CFG_MAXRETRIES, sizeof(CFG_MAXRETRIES));

    // Analog
    static const uint8_t CFG_ANALOG[] = {0x0B, 0x5F, 0xFF, 0x3F, 0x11, 0x4D, 0x85, 0x61, 0x6F};
    pn532_send_receive(0x32, CFG_ANALOG, sizeof(CFG_ANALOG));

    // RF
    static const uint8_t CFG_RFFIELD[] = {0x01, 0x01};
    pn532_send_receive(0x32, CFG_RFFIELD, sizeof(CFG_RFFIELD));

    // SAM
    static const uint8_t CMD_SAMCONFIG[] = {0x01, 0x00};
    pn532_send_receive(0x14, CMD_SAMCONFIG, sizeof(CMD_SAMCONFIG));

end:
    // コールバック
    callback(DRVICC_EVT_INITIALIZE_COMPLETE, error, 0);
}

/*
 * パワーオフ
 */
void process_msg_power_off(DRVICC_CALLBACK_T callback)
{
    assert(callback);

    int error = 0;

    // CARD_OFF
    drvcmn_gpio_pin_set(&GPIO_PIN_CARD_ON, false);	// CARD_ON => L (ON)
    dly_tsk(10);
    drvcmn_gpio_pin_set(&GPIO_PIN_CARD_ON, true);	// CARD_ON => H (OFF)
    dly_tsk(10);

    // コールバック
    callback(DRVICC_EVT_FINALIZE_COMPLETE, error, 0);
}

/*
 * ポーリング開始
 */
void process_msg_start_polling(DRVICC_CALLBACK_T callback, DRVICC_FELICA_IDM_T* idm)
{
    assert(callback);
    assert(idm);

    int error = 0;
    ER er = 0;
    
    bool_t cancelled = false;
    while (true) {
        SYSTIM next = 0;
        get_tim(&next);
        next += POLLING_PERIOD;

        int resp_len = 0;
        static const uint8_t FELICA_POLLCMD[] = {0x01, 0x01, 0x00, 0xFF, 0xFF, 0x00, 0x00};
        resp_len = pn532_send_receive(0x4a, FELICA_POLLCMD, sizeof(FELICA_POLLCMD));
        if (s_context.rx_buf[7] > 0) {
            DBGLOG0("CARD FOUND");
            memcpy(idm->idm, &(s_context.rx_buf[11]), sizeof(idm->idm));
            break;
        }

        // 次の周期までの時間を計算
        SYSTIM now = 0;
        TMO interval = 0;
        get_tim(&now);
        interval = next - now;
        if ((interval > POLLING_PERIOD) || (interval < 0)) {
            interval = POLLING_PERIOD;
        }

        // 次の周期またはキャンセルまで待つ
        er = twai_flg(FLG_DRVICC, FLGPTN_CANCEL_POLLING, TWF_ANDW, &(FLGPTN){0}, interval);
        if (er == E_TMOUT) {		// 時間経過
            // 繰り返し
        } else if (er == E_OK) {	// キャンセル要求
            DBGLOG0("CANCELLED");
            cancelled = true;
            break;
        } else {
            DBGLOG1("er = %d", er);
            assert(false);
        }
    }

end:
    if (!cancelled) {
        // コールバック
        callback(DRVICC_EVT_POLLING_COMPLETE, error, 0);
    }
}

/*
 * カード読み取り
 */
void process_msg_read(DRVICC_CALLBACK_T callback, DRVICC_FELICA_BLOCK_T* blocks, const DRVICC_FELICA_BLOCK_DESC_T* desc)
{
    assert(callback);
    assert(blocks);
    assert(desc);

    DBGLOG0("process_msg_read");

    int error = 0;

    int resp_len = 0;
    static const uint8_t FELICA_READ_WO_ENCRYPTION[] = { 0x01,									// [0]Target
                                                         0x10,									// [1]Len (Cmd + IDm + )
                                                         0x06,									// [2]Cmd (Read Without Encryption)
                                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// [3]IDm
                                                         0x01,									// [11]Number of service
                                                         0x00, 0x00,								// [12]Service code
                                                         0x01,									// [14]Number of blocks
                                                         0x80, 0x00,								// [15]Block list
    };
    static uint8_t cmd[sizeof(FELICA_READ_WO_ENCRYPTION)];
    memcpy(cmd, FELICA_READ_WO_ENCRYPTION, sizeof(cmd));

    // InDataExchange
    memcpy(&(cmd[3]), desc->idm, sizeof(desc->idm));
    memcpy(&(cmd[12]), &(desc->service), sizeof(desc->service));
    cmd[16] = desc->block;
    resp_len = pn532_send_receive(0x40, cmd, sizeof(cmd));

    uint8_t resp_code = s_context.rx_buf[7];
    if (resp_code != 0) {
        DBGLOG0("READ ERROR");
        error = 1;
        goto end;
    }

    // データをコピー
    memcpy(blocks->block, &(s_context.rx_buf[21]), sizeof(blocks->block));

end:
    // コールバック
    callback(DRVICC_EVT_READ_COMPLETE, error, 0);
}

/*
 * カード書込み
 */
void process_msg_write(DRVICC_CALLBACK_T callback, const DRVICC_FELICA_BLOCK_T* blocks, const DRVICC_FELICA_BLOCK_DESC_T* desc)
{
    assert(callback);
    assert(blocks);
    assert(desc);

    DBGLOG0("process_msg_write");

    int error = 0;
    
    int resp_len = 0;
    static const uint8_t FELICA_WRITE_WO_ENCRYPTION[] = { 0x01,									// [0]Target
                                                          0x20,									// [1]Len (Cmd + IDm + )
                                                          0x08,									// [2]Cmd (Write Without Encryption)
                                                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// [3]IDm
                                                          0x01,									// [11]Number of service
                                                          0x00, 0x00,								// [12]Service code
                                                          0x01,									// [14]Number of blocks
                                                          0x80, 0x00,								// [15]Block list
                                                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// [17]Data
                                                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                    
    };
    static uint8_t cmd[sizeof(FELICA_WRITE_WO_ENCRYPTION)];
    memcpy(cmd, FELICA_WRITE_WO_ENCRYPTION, sizeof(cmd));

    // InDataExchange
    memcpy(&(cmd[3]), desc->idm, sizeof(desc->idm));
    memcpy(&(cmd[12]), &(desc->service), sizeof(desc->service));
    cmd[16] = desc->block;
    memcpy(&(cmd[17]), blocks->block, sizeof(blocks->block));
    resp_len = pn532_send_receive(0x40, cmd, sizeof(cmd));

    uint8_t resp_code = s_context.rx_buf[7];
    if (resp_code != 0) {
        DBGLOG0("WRITE ERROR");
        error = 1;
        goto end;
    }

end:
    // コールバック
    callback(DRVICC_EVT_WRITE_COMPLETE, error, 0);
}

/*
 * UART送信
 */
int uart_send(const uint8_t* data, size_t size)
{
    assert(data);
    assert(size > 0);

    ER er = 0;
    int error = 0;

    // イベントフラグをクリア
    er = clr_flg(FLG_DRVICC, ~FLGPTN_UART_TXCOMPLETE);
    assert(er == E_OK);

    // UART送信
    drvcmn_uart_send(ICC_UART, data, size);

    // 完了待ち
    FLGPTN flgptn = 0;
    er = twai_flg(FLG_DRVICC, FLGPTN_UART_TXCOMPLETE, TWF_ANDW, &flgptn, RESPONSE_TIMEOUT);
    assert((er == E_OK) || (er == E_TMOUT));

    // タイムアウトの場合は中断処理
    if (er == E_TMOUT) {
        error = DRVICC_ERROR_TIMEOUT;
        drvcmn_uart_cancel_send(ICC_UART);
    }

    return error;
}

/*
 * UART受信
 */
int uart_receive(uint8_t* data, size_t data_len, TMO timeout)
{
    assert(data);
    assert(data_len > 0);

    int error = 0;
    // 受信
    drvcmn_uart_receive(ICC_UART, data, data_len);

    // 受信完了を待つ
    ER er = twai_flg(FLG_DRVICC, FLGPTN_UART_RXCOMPLETE, TWF_ANDW, &(FLGPTN){0}, timeout);
    assert((er == E_OK) || (er == E_TMOUT));

    // タイムアウトの場合は中断処理
    if (er == E_TMOUT) {
        error = DRVICC_ERROR_TIMEOUT;
        drvcmn_uart_cancel_receive(ICC_UART);
    }

    return error;
}

/*
 * PN532 コマンド組み立て
 */
size_t build_command_frame(uint8_t* output, uint8_t command_code, const uint8_t* optional_data, size_t data_len)
{
    assert(output);

    size_t pos = 0;

    static const uint8_t CMD_WAKEUP[] = {0x55, 0x55, 0x55, 0x55, 0x00, 0x00, 0x00, 0x00};
    memcpy(&(output[pos]), CMD_WAKEUP, sizeof(CMD_WAKEUP));
    pos += sizeof(CMD_WAKEUP);

    output[pos] = 0x00;	// PREAMBLE
    pos++;
    
    output[pos] = 0x00;	// START CODE[0]
    pos++;

    output[pos] = 0xff;	// START CODE[1]
    pos++;

    output[pos] = 0xff;	// NORMAL PACKET LEN
    pos++;

    output[pos] = 0xff;	// NORMAL PACKET LEN CHECKSUM
    pos++;

    uint8_t len = 2 + data_len;
    output[pos] = 0;	// LEN[0] = 0
    pos++;

    output[pos] = len;	// LEN[1] (TFI(1) + command_code(1) + data_len)
    pos++;

    output[pos] = ((0 - len) & 0xff);	// LCS (LEN + LCS == 0x00)
    pos++;

    uint8_t tfi = 0xd4;
    output[pos] = tfi;	// TFI (0xd4: host to PN532)
    pos++;
    uint8_t sum = tfi;

    output[pos] = command_code;	// PD0 (command_code)
    pos++;
    sum += command_code;

    assert(data_len == 0 || optional_data);

    // PDn (optional_data)
    for (int i = 0; i < data_len; i++) {
        output[pos] = optional_data[i];
        pos++;
        sum += optional_data[i];
    }

    output[pos] = ((0 - sum) & 0xff);	// DCS
    pos++;

    output[pos] = 0x00;	// POSTAMBLE
    pos++;

    return pos;
}

// Normal information frame 読み込み
int read_inf_frame(uint8_t* data, size_t max_len)
{
    assert(data);
    assert(max_len > 7);

    int result = 0;
    int ret = 0;
    ret = uart_receive(&(data[0]), 5, RESPONSE_TIMEOUT);	// 5バイト
    if (ret != 0) {
        DBGLOG0("in read_inf_frame(): header timeout");
        result = -1;
        goto end;
    }
    uint8_t len = data[3];	// LEN取得

    ret = uart_receive(&(data[5]), len + 2, RESPONSE_TIMEOUT);	// 残り(DATA(len) + DCS(1) + Postamble(1))
    if (ret != 0) {
        DBGLOG0("in read_inf_frame(): data timeout");
        result = -1;
        goto end;
    }

    result = 5 + len + 2;

#if 0
    cmndbg_hexdump(data, result, "inf_frame");
#endif
end:
    return result;
}

// ACK / NACK frame を待つ
int wait_ack_frame()
{
    int error = 0;
    int ret = 0;
    uint8_t data[6];

    ret = uart_receive(&(data[0]), 6, ACK_TIMEOUT);	// 6バイト
    if (ret != 0) {
        error = 2;
        goto end;
    }

    uint8_t* code = &(data[3]);	// LEN取得

    if ((code[0] == 0x00) && (code[1] == 0xff)) {
        // ACK
        error = 0;
    } else if ((code[0] == 0xff) && (code[1] == 0xff)) {
        // NACK
        error = 1;
    } else {
        // ?
        error = 3;
    }

end:
    return error;
}

/*
 * 
 */
int pn532_send_receive(uint8_t command_code, const uint8_t* optional_data, size_t data_len)
{
    int error = 0;

    // パケット組み立て
    size_t packet_len = build_command_frame(s_context.tx_buf, command_code, optional_data, data_len);

    for (int i = 0; i < 3; i++) {
        // パケット送信
        error = uart_send(s_context.tx_buf, packet_len);
        if (error) {
            DBGLOG1("pn532_send_receive: uart_send() error(%d)", error);
            goto end;
        }

        // ACK待ち
        error = wait_ack_frame();
        if (error) {
            DBGLOG1("pn532_send_receive: wait_ack_frame() error(%d)", error);
        }

        if (!error) {
            break;
        }
        // ACKを受信できていない場合は再送
    }

    // レスポンス待ち
    int resp_size = read_inf_frame(s_context.rx_buf, sizeof(s_context.rx_buf));
    if (resp_size <= 0) {
        DBGLOG1("pn532_send_receive: read_inf_frame() error(%d)", resp_size);
        error = 1;
        goto end;
    }

    // ACK送信
    error = uart_send(PN532_ACK_FRAME, sizeof(PN532_ACK_FRAME));
    if (error) {
        DBGLOG1("pn532_send_receive: uart_send() error(%d)", error);
        goto end;
    }

end:
    return (error ? 0 : resp_size);
}

/*
 * UART ISRコールバック
 */
bool_t uart_isr_callback(int type, uint32_t opt)
{
    ER er = E_OK;
    switch (type) {
    case DRVCMN_UART_TXCOMPLETE:
        er = iset_flg(FLG_DRVICC, FLGPTN_UART_TXCOMPLETE);
        assert(er == E_OK);
        break;
    case DRVCMN_UART_RXCOMPLETE:
        er = iset_flg(FLG_DRVICC, FLGPTN_UART_RXCOMPLETE);
        assert(er == E_OK);
        break;
    case DRVCMN_UART_ERROR:
        DBGLOG1("uart_isr_callback() error (%08x)", opt);
        break;
    default:
        assert(false);
        break;
    }

    return false;
}

