/*
 * VA-X 鍵錠ドライバ (Linkey対応)
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/07/16 Takakashi Otsuka <otsuka-takashi@bionics-k.co.jp>: 新規作成
 */

#include "drvllock.h"
//#include "drvllockl.h"

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
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[DRVllock]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[DRVllock]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[DRVllock]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[DRVllock]" msg, arg1, arg2, arg3)
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

//#define LNKY_HEADER_SIZE			8	/* パケットスタート(1) + データ長(2) + チェックサム(1)*/

/*
 * 型定義
 */

// 送信データメモリブロック
typedef struct {
    uint32_t msg;
    intptr_t data;
    size_t   size;
    DRVLLOCK_CALLBACK_T callback;
} DRVLLOCK_MPFBLK_T;

// DRVLLOCK_MPFBLK_SIZE は sizeof(DRVLLOCK_MPFBKL) に一致させること
static_assert(sizeof(DRVLLOCK_MPFBLK_T) == DRVLLOCK_MPFBLK_SIZE, "MPF size");

/*
    内部変数
*/
//static DRVLOCK_CBFUNC_T s_callback = NULL;

/*
 * 内部関数プロトタイプ
 */
// メモリプールブロックを使ってメッセージ送信
static void mpf_send(int msg, DRVLLOCK_CALLBACK_T callback, intptr_t data, size_t size);

// Linkeyモジュール初期化
static void process_msg_initialize(DRVLLOCK_CALLBACK_T callback);

// LinkeyOPENコマンド送信タスク
static void process_msg_opencmd(uint8_t* data, size_t size, DRVLLOCK_CALLBACK_T callback);

// Linkeyドアセンサコマンド送信タスク
static void process_msg_mgntcmd(uint8_t* data, size_t size, DRVLLOCK_CALLBACK_T callback);

// Linkeyデッドボルトコマンド送信タスク
static void process_msg_dvltcmd(uint8_t* data, size_t size);

// Linkeyバッテリー確認コマンド送信タスク
static void process_msg_chkcmd(uint8_t* data, size_t size, DRVLLOCK_CALLBACK_T callback);

// UART送信
static int uart_send(const uint8_t* data, size_t size);

// UART ISRコールバック
static bool_t uart_isr_callback(int type, uint32_t opt);

//Linkeyへリセット発行
//static void reset_cmd(DRVLLOCK_CALLBACK_T callback, uint8_t* data, size_t size);

//Linkeyへ解錠コマンド発行
//static void open_command(DRVLLOCK_CALLBACK_T callback, const uint8_t* data, size_t size);

//static void status_mgnt_command(DRVLLOCK_CALLBACK_T callback, const uint8_t* data, size_t size);
/*
 * 定数定義
 */

// GPIO(UART)
static const DRVCMN_GPIO_PIN_T GPIO_PINS_UART[] = {
    { DRVCMN_GPIO_PORT_A,  9 },	// AF7: USART1_TX
    { DRVCMN_GPIO_PORT_A, 10 },	// AF7: USART1_RX
};

// GPIO設定(UART)
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_UART = {
    .mode = DRVCMN_GPIO_MODE_AF,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLUP,
    .afno = 7,
};

// GPIO(BTS-01)
static const DRVCMN_GPIO_PIN_T GPIO_PIN_RST = 		{ DRVCMN_GPIO_PORT_E,	14	};	// RST

// GPIO設定(BTS-01, OD)
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_LOCK_OD = {
    .mode = DRVCMN_GPIO_MODE_GPOUT,
    .otype = DRVCMN_GPIO_OTYPE_OD,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLDOWN,
};

static const DRVCMN_GPIO_PIN_T GPIO_PINS_INPUT[] = {
    { DRVCMN_GPIO_PORT_A,	0,	},	/* PA0  DOOR HALL */

};

/* ピン設定 入力側*/
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_INPUT = {
    .mode = DRVCMN_GPIO_MODE_INPUT,
    .otype = DRVCMN_GPIO_OTYPE_OD,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_NONE,
    .exti = DRVCMN_GPIO_EXTI_BOTHTRIG,
    .exti_isr = drvlilock_sensor_isr
};

static const struct {
    const DRVCMN_GPIO_PIN_T* pin;
    const DRVCMN_GPIO_SETTING_T* setting;
    int initial_value;
} GPIO_PINS_LOCK[] = {
    { &GPIO_PIN_RST,	&GPIO_SETTING_LOCK_OD,	1	},	// RST (OD, Active L)
};

// UART設定
static const int LLOCK_UART = 1;	// USART1
static const DRVCMN_UART_SETTING_T UART_SETTING = {
    .clocksel = DRVCMN_UART_CLOCKSEL_SYS,
    .baudrate = 38400,
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
    MSG_OPCMD,
    MSG_MGNTCMD,
    MSG_DEDVLTCMD,
    MSG_CHKCMD,
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

// 内部変数
//static uint8_t l_header_buf[LNKY_HEADER_SIZE];

// 応答パケット
static const uint8_t PN532_ACK_FRAME[] = {0x00, 0x00, 0xff, 0x00, 0xff, 0x00};	// ACK
static const uint8_t PN532_NACK_FRAME[] = {0x00, 0x00, 0xff, 0xff, 0x00, 0x00};	// NACK

// テストデータ
static const uint8_t TEST_DATA[] = {0x00, 'T', 'E', 'S', 'T'};
static const uint8_t CMD_FAKE_FRAME[] = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01};	// FAKE
static const uint8_t CMD_OPEN_FRAME[] = {0x02, 0x02, 0x02, 0x01, 0x03, 0x03};	// OPEN
static const uint8_t CMD_CLOSE_FRAME[] = {0x02, 0x02, 0x02, 0x02, 0x00, 0x03};	// CLOSE
static const uint8_t CMD_CNACK_FRAME[] = {0x02, 0x03, 0x41, 0x05, 0x03, 0x47, 0x03};	// OK_ACK
static const uint8_t CMD_STS_FRAME[]   = {0x02, 0x02, 0x04, 0x02, 0x06, 0x03};	// STATUS
static const uint8_t CMD_MGNET_FRAME[] = {0x02, 0x02, 0x04, 0x03, 0x07, 0x03};	// MGNET SENSOR
static const uint8_t CMD_ATLOCK_FRAME[] = {0x02, 0x02, 0x04, 0x06, 0x02, 0x03};	// AUTOLOCK MODE
static const uint8_t CMD_SECURE_FRAME[] = {0x02, 0x02, 0x04, 0x07, 0x03, 0x03};	// SEKURITY
static const uint8_t CMD_ABSENCE_FRAME[] = {0x02, 0x02, 0x04, 0x07, 0x03, 0x03};// Absence RT MODE
static const uint8_t CMD_ABSENCM_FRAME[] = {0x02, 0x03, 0x02, 0x07, 0x01, 0x04, 0x03};// Absence MODE
static const uint8_t CMD_DEDVLT_FRAME[] = {0x02, 0x02, 0x04, 0x02, 0x06, 0x03};	// DEDVOLT MODE

// フレームフォーマット
static const size_t FRAME_OFFSET_DATA = 6;
static const size_t FRAME_OVERHEAD_SIZE = 8;

/*
 * 内部変数
 */
static DRVLLOCK_CALLBACK_T s_callback = NULL;

/* 割込み PIN番号 */
enum {
    INTRPT_DRSNS = 0,   //ドアセンサ

};

// リセットタイムアウト
static const int32_t RESET_TIMEOUT = -1;
// ドアセンサタイムアウト
static const int32_t MGNT_TIMEOUT = 2000;
// 施錠タイムアウト
static const int32_t LOCK_TIMEOUT = -1;
// バッテリー確認タイムアウト
static const int32_t CHEK_TIMEOUT = -1;

/*********************************************************************
 * 公開関数
 ********************************************************************/

/*
 * ペリフェラル初期化
 */
void drvllock_initialize_peripherals()
{
    // GPIO初期化(USART)
    for (int i = 0; i < sizeof(GPIO_PINS_UART) / sizeof(GPIO_PINS_UART[0]); i++) {
        drvcmn_gpio_pin_initialize(&(GPIO_PINS_UART[i]), &GPIO_SETTING_UART);
    }
    
    // GPIO初期化
    for (int i = 0; i < sizeof(GPIO_PINS_LOCK) / sizeof(GPIO_PINS_LOCK[0]); i++) {
        drvcmn_gpio_pin_initialize(GPIO_PINS_LOCK[i].pin, GPIO_PINS_LOCK[i].setting);
        if (GPIO_PINS_LOCK[i].initial_value >= 0) {
            drvcmn_gpio_pin_set(GPIO_PINS_LOCK[i].pin, GPIO_PINS_LOCK[i].initial_value);
        }
    }

    /* GPIO 入力ピンの設定 */
    for (int i = 0; i < sizeof(GPIO_PINS_INPUT) / sizeof(GPIO_PINS_INPUT[0]); i++) {
        drvcmn_gpio_pin_initialize(&(GPIO_PINS_INPUT[i]), &GPIO_SETTING_INPUT);
    }

}

/*
 * ドライバ初期化
 */
void drvllock_initialize(DRVLLOCK_CALLBACK_T callback)
{
    assert(callback);
    /* コールバックを設定 */
    s_callback = callback;
    
    ER er = 0;

    // ドライバタスク起動
    act_tsk(TSK_DRVLLOCK);
    assert(er == E_OK);

    // 初期化メッセージを送信
    mpf_send(MSG_INITIALIZE, callback, 0, 0);

}

/*
 * タスク
 */
void drvllock_task(intptr_t exinf)
{
    DBGLOG0("drvllock_task() starts .");

    while (true) {
        DRVLLOCK_MPFBLK_T* blk = NULL;
        ER er = 0;
        er = trcv_dtq(DTQ_DRVLLOCK, (intptr_t*)&blk, TMO_FEVR);
        assert(er == E_OK);
        DBGLOG1("drvllock_task(): msg=%d", blk->msg);
        switch (blk->msg) {
        case MSG_INITIALIZE:
            process_msg_initialize(blk->callback);
            break;
        case MSG_MGNTCMD:
            process_msg_mgntcmd((uint8_t*)(blk->data), blk->size, blk->callback);
            break;
        case MSG_OPCMD:
            process_msg_opencmd((uint8_t*)(blk->data), blk->size, blk->callback);
            break;
        case MSG_DEDVLTCMD:
            process_msg_dvltcmd((uint8_t*)(blk->data), blk->size);
            break;
        case MSG_CHKCMD:
            process_msg_chkcmd((uint8_t*)(blk->data), blk->size, blk->callback);
            break;
        default:
            assert(false);
            break;
        }
        er = rel_mpf(MPF_DRVLLOCK, blk);
        assert(er == E_OK);
    }
}

/*
 * 鍵錠GPIO 割込みサービスルーチン
 */

int32_t drvlilock_sensor_isr(uint32_t pinno)
{
    DBGLOG1("sensor_isr %d", pinno);

    if (!s_callback) {
        return 0;
    }

    int32_t intrpnum = 0; 
    intrpnum = pinno;

    if(intrpnum == INTRPT_DRSNS){
        if (drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[0])) {
            DBGLOG0("sensor_isr_doorcls. ");
            s_callback(DRVLLOCK_EVT_DOOR_CLOSEISR, 0);
        }else{
            DBGLOG0("sensor_isr_dooropn. ");
            s_callback(DRVLLOCK_EVT_DOOR_OPENISR, 0);
        }
    }
    return 0;
}

/*
 * 受信タスク
 */
void drvllock_rx_task(intptr_t exinf)
{
    DBGLOG0("drvllock_rx_task() starts .");
    
}

/*********************************************************************
 * 内部関数
 ********************************************************************/

/*
 * メッセージ送信
 */
void mpf_send(int msg, DRVLLOCK_CALLBACK_T callback, intptr_t data, size_t size)
{
    DBGLOG0("mpf_send .");
    
    DRVLLOCK_MPFBLK_T* blk = NULL;
    ER er = 0;
    er = tget_mpf(MPF_DRVLLOCK, (void**)&blk, SYSCALL_TIMEOUT);
    assert(er == E_OK);

    blk->msg = msg;
    blk->callback = callback;
    blk->data = data;
    blk->size = size;
    er = tsnd_dtq(DTQ_DRVLLOCK, (intptr_t)blk, SYSCALL_TIMEOUT);
    assert(er == E_OK);
}

/*
 * ドライバ初期化
 */
void process_msg_initialize(DRVLLOCK_CALLBACK_T callback)
{
    assert(callback);
    
    // UART初期化
    drvcmn_uart_initialize(LLOCK_UART, &UART_SETTING);
    
}

void reset_cmd(DRVLLOCK_CALLBACK_T callback, uint8_t* data, size_t size)
{
    int error = 0;
    
    //リセット送信
    drvcmn_gpio_pin_set(&GPIO_PIN_RST, false);		// RESET => L (リセット)
    dly_tsk(2000);

    drvcmn_gpio_pin_set(&GPIO_PIN_RST, true);		// RESET => H (リセット解除)

    //接続確認受信
    DBGLOG0("uart_recieve_rst .");

    for (int i = 0; i < size; i++) {
        data[i] = 0x00;
//        syslog(LOG_NOTICE, "receive 0x%02x '%c'", l_header_buf[i], l_header_buf[i]);
    }
    //LinkeyからDEV Module連結確認コマンド受信待ち
    uart_receive(data, size, RESET_TIMEOUT);

    for (int i = 0; i < size; i++) {
        syslog(LOG_NOTICE, "receive_rst 0x%02x '%c'", data[i], data[i]);
    }

    DBGLOG1("heder_buf_rst: 0x%02x", data[0]);

    for(int i = 0; i < 2; i++){
        if(data[i] == 0xaa){
            if(i == 0){
                if((data[3] == 0x05) && (data[4] == 0x03)){
                    error = uart_send(CMD_CNACK_FRAME, sizeof(CMD_CNACK_FRAME));
                    if (error) {
                        DBGLOG1("msg_initialize: uart_send() error(%d)", error);
                    }
                    DBGLOG0("msg_initialize: uart_send starts .");
                    
                }
            }else{
                if((data[4] == 0x05) && (data[5] == 0x03)){
                    error = uart_send(CMD_CNACK_FRAME, sizeof(CMD_CNACK_FRAME));
                    if (error) {
                        DBGLOG1("msg_initialize: uart_send() error(%d)", error);
                    }
                    DBGLOG0("msg_initialize: uart_send starts .");
                }
            }
        }
    }

    if(error){  //Linkey連結Error発生
        callback(DRVLLOCK_EVT_CONECT_FALE, error);
    }else{      //連結完了
        callback(DRVLLOCK_EVT_CONECT_COMP, error);
    }
}

void open_command(DRVLLOCK_CALLBACK_T callback, const uint8_t* data, size_t size)
{
    mpf_send(MSG_OPCMD, callback, (intptr_t)data, size);
}

void process_msg_opencmd(uint8_t* data, size_t size, DRVLLOCK_CALLBACK_T callback)
{
    assert(callback);
    
    int error = 0;
    
    // LinkeyへOPENコマンド送信
    error = uart_send(CMD_FAKE_FRAME, sizeof(CMD_FAKE_FRAME));
    dly_tsk(1000);
    error = uart_send(CMD_OPEN_FRAME, sizeof(CMD_OPEN_FRAME));
    if (error) {
        DBGLOG1("msg_initialize: uart_send() error(%d)", error);
    }
    DBGLOG0("msg_initialize: uart_send starts .");
    //LinkeyからACK待ち
    uart_receive(data, size, LOCK_TIMEOUT);
    for (int i = 0; i < size; i++) {
        syslog(LOG_NOTICE, "receive 0x%02x '%c'", data[i], data[i]);
    }

    for(int i = 0; i < 2; i++){
        if(data[i] == 0xaa){
            //ACK判定
            if((data[3] == 0x05) && (data[4] == 0x03)){
                // サムターン割込み待ち
                error = uart_err_receive(data, size, LOCK_TIMEOUT);
                if(error){
                    DBGLOG0("sumturn intrrupt timeout: .");
                    //施解錠異常判定
                    //close時、施解錠異常発生
                    if(data[4] == 0x03){
                        callback(DRVLLOCK_EVT_OPEN_ERROR, error);
                        break;
                    }else{
                        //異常無し
                        break;
                    }
                }else{
                    DBGLOG0("sumturn intrrupt recieved: .");
                    //サムターンイベント発生
                    callback(DRVLLOCK_EVT_SUMTURN_OCCUR, error);
                    break;
                }
            }
        }
    }

}

void close_command(DRVLLOCK_CBFUNC_T callback)
{
    assert(callback);
    
    int error = 0;

    // OPN送信
    error = uart_send(CMD_FAKE_FRAME, sizeof(CMD_FAKE_FRAME));
    dly_tsk(1000);
    error = uart_send(CMD_CLOSE_FRAME, sizeof(CMD_CLOSE_FRAME));
    if (error) {
        DBGLOG1("msg_initialize: uart_send() error(%d)", error);
    }
    DBGLOG0("msg_initialize: uart_send starts .");
    
}

//Linkey Battery 確認
//
void status_chek_command(DRVLLOCK_CALLBACK_T callback, const uint8_t* data, size_t size)
{
    mpf_send(MSG_CHKCMD, callback, (intptr_t)data, size);
}

void process_msg_chkcmd(uint8_t* data, size_t size, DRVLLOCK_CALLBACK_T callback)
{
    assert(callback);
    
    DBGLOG0("get_check_status: uart_recieve starts .");

    //EVNT受信
    uart_receive(data, size, CHEK_TIMEOUT);

    for (int i = 0; i < size; i++) {
        syslog(LOG_NOTICE, "receive 0x%02x '%c'", data[i], data[i]);
    }
    
    for(int i = 0; i < 3; i++){
        if(data[i] == 0xaa){
            DBGLOG1("i: 0x%02x", i);
                if((data[5] == 0x03) && (data[6] == 0x07)){

                    callback(DRVLLOCK_EVT_BATERRY_ERROR, 0);
                }
        }
    }

}

//Door Status  取得
//
void status_mgnt_command(DRVLLOCK_CALLBACK_T callback, const uint8_t* data, size_t size)
{
    mpf_send(MSG_MGNTCMD, callback, (intptr_t)data, size);
}

void process_msg_mgntcmd(uint8_t* data, size_t size, DRVLLOCK_CALLBACK_T callback)
{
    assert(callback);
    
    int error = 0;
    
    error = uart_send(CMD_FAKE_FRAME, sizeof(CMD_FAKE_FRAME));
    dly_tsk(1000);
    error = uart_send(CMD_MGNET_FRAME, sizeof(CMD_MGNET_FRAME));
    if (error) {
        DBGLOG1("get_mgnt_status: uart_send() error(%d)", error);
    }
    DBGLOG0("get_mgnt_status: uart_send starts .");

    //ACK受信
    error = uart_receive(data, size, MGNT_TIMEOUT);
    if (error) {
        DBGLOG1("get_mgnt_status: uart_receiv() error(%d)", error);
    }
    
    for (int i = 0; i < size; i++) {
        syslog(LOG_NOTICE, "receive 0x%02x '%c'", data[i], data[i]);
    }

    DBGLOG1("heder_buf: 0x%02x", data[0]);

    for(int i = 0; i < 2; i++){
        if(data[i] == 0xaa){
            DBGLOG1("i: 0x%02x", i);
//            if(i == 0){
                if((data[5] == 0x04) && (data[6] == 0x03)){
                    if(data[7] == 0x01){
                        DBGLOG0("door_open  .");
                        //error:送信状態返信
                        callback(DRVLLOCK_EVT_OPEN_STATUS, error);
                    }else{
                        DBGLOG0("door_close  .");
                        callback(DRVLLOCK_EVT_CLOSE_STATUS, error);
                    }
                }
//            }
        }
    }
    
}

void absence_rq_command(DRVLLOCK_CBFUNC_T callback)
{
    assert(callback);
    
    int error = 0;

    error = uart_send(CMD_FAKE_FRAME, sizeof(CMD_FAKE_FRAME));
    dly_tsk(1000);
    error = uart_send(CMD_ABSENCE_FRAME, sizeof(CMD_ABSENCE_FRAME));
    if (error) {
        DBGLOG1("msg_absence_rq: uart_send() error(%d)", error);
    }
    DBGLOG0("msg_absence_rq: uart_send starts .");
    
}

//Dedvolt Status  取得
//
void status_dedvlt_command(const uint8_t* data, size_t size)
{
    DBGLOG0("sts_dvlt_cmd .");
    DRVLLOCK_CALLBACK_T callback;
    callback = NULL;
    
    mpf_send(MSG_DEDVLTCMD, callback, (intptr_t)data, size);
}

void process_msg_dvltcmd(uint8_t* data, size_t size)
{
    
    int error = 0;
    
    error = uart_send(CMD_FAKE_FRAME, sizeof(CMD_FAKE_FRAME));
    dly_tsk(1000);
    error = uart_send(CMD_DEDVLT_FRAME, sizeof(CMD_DEDVLT_FRAME));
    if (error) {
        DBGLOG1("dedvlt: uart_send() error(%d)", error);
    }
    DBGLOG0("dedvlt: uart_send starts .");
    
    uart_receive(data, size, LOCK_TIMEOUT);
    for (int i = 0; i < size; i++) {
        syslog(LOG_NOTICE, "receive 0x%02x '%c'", data[i], data[i]);
    }
    for(int i = 0; i < 2; i++){
        if(data[i] == 0xaa){
            DBGLOG1("i: 0x%02x", i);
                if((data[5] == 0x04) && (data[6] == 0x02)){
                    if(data[7] == 0x01){
                        DBGLOG0("dedvolt_open  .");
                        s_callback(DRVLLOCK_EVT_DEDVOLT_OPEN, error);
                    }else{
                        DBGLOG0("dedvolt_close  .");
                        s_callback(DRVLLOCK_EVT_DEDVOLT_CLOSE, error);
                    }
                }
        }
    }
}

void absence_command(DRVLLOCK_CBFUNC_T callback)
{
    assert(callback);
    
    int error = 0;

    error = uart_send(CMD_FAKE_FRAME, sizeof(CMD_FAKE_FRAME));
    dly_tsk(1000);
    error = uart_send(CMD_ABSENCM_FRAME, sizeof(CMD_ABSENCM_FRAME));
    if (error) {
        DBGLOG1("msg_initialize: uart_send() error(%d)", error);
    }
    DBGLOG0("msg_initialize: uart_send starts .");
    
}

void command_recieve(DRVLLOCK_CBFUNC_T callback)
{
    assert(callback);
    
    int error = 0;

    error = uart_send(CMD_FAKE_FRAME, sizeof(CMD_FAKE_FRAME));
    dly_tsk(1000);
    error = uart_send(CMD_ABSENCM_FRAME, sizeof(CMD_ABSENCM_FRAME));
    if (error) {
        DBGLOG1("msg_initialize: uart_send() error(%d)", error);
    }
    DBGLOG0("msg_initialize: uart_send starts .");
    
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
    er = clr_flg(FLG_DRVLLOCK, ~FLGPTN_UART_TXCOMPLETE);
    assert(er == E_OK);

    // UART送信
    drvcmn_uart_send(LLOCK_UART, data, size);

    // 完了待ち
    FLGPTN flgptn = 0;
    er = twai_flg(FLG_DRVLLOCK, FLGPTN_UART_TXCOMPLETE, TWF_ANDW, &flgptn, RESPONSE_TIMEOUT);
    assert((er == E_OK) || (er == E_TMOUT));

    // タイムアウトの場合は中断処理
    if (er == E_TMOUT) {
        error = DRVLLOCK_ERROR_TIMEOUT;
        drvcmn_uart_cancel_send(LLOCK_UART);
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
    drvcmn_uart_receive(LLOCK_UART, data, data_len);
    // 受信完了を待つ
    ER er = twai_flg(FLG_DRVLLOCK, FLGPTN_UART_RXCOMPLETE, TWF_ANDW, &(FLGPTN){0}, timeout);
    assert((er == E_OK) || (er == E_TMOUT));

    // タイムアウトの場合は中断処理
    if (er == E_TMOUT) {
        error = DRVLLOCK_ERROR_TIMEOUT;
        drvcmn_uart_cancel_receive(LLOCK_UART);
    }

    return error;
}

/*施錠解錠エラー待ち*/
int uart_err_receive(uint8_t* data, size_t size, int32_t timeout)
{
    
    assert(data);
    assert(size > 0);

    int error = 0;
    // 受信
    drvcmn_uart_receive(LLOCK_UART, data, size);
    // 受信完了を待つ
    ER er = twai_flg(FLG_DRVLLOCK, FLGPTN_UART_RXCOMPLETE, TWF_ANDW, &(FLGPTN){0}, timeout);
    assert((er == E_OK) || (er == E_TMOUT));

    // タイムアウトの場合は中断処理
    if (er == E_TMOUT) {
        error = DRVLLOCK_ERROR_TIMEOUT;
        drvcmn_uart_cancel_receive(LLOCK_UART);
    }
    
    return error;
}
/*
 * UART ISRコールバック
 */
bool_t uart_isr_callback(int type, uint32_t opt)
{
    ER er = E_OK;
    switch (type) {
    case DRVCMN_UART_TXCOMPLETE:
        er = iset_flg(FLG_DRVLLOCK, FLGPTN_UART_TXCOMPLETE);
        DBGLOG1("uart_isr_callback() tx (%08x)", opt);
        assert(er == E_OK);
        break;
    case DRVCMN_UART_RXCOMPLETE:
        er = iset_flg(FLG_DRVLLOCK, FLGPTN_UART_RXCOMPLETE);
        DBGLOG1("uart_isr_callback() rx (%08x)", opt);
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

