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
#include "aplui.h"
#include "drvled.h"
/*
 * マクロ定義
 */

// デバッグログ
#if 0
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
static void process_msg_initialize();

// Linkeyリセットコマンド
void reset_cmd();

// LinkeyOPENコマンド送信タスク
//static void process_msg_opencmd();

// Linkeyドアセンサコマンド送信タスク
static void process_msg_mgntcmd();

// Linkeyバッテリー確認コマンド送信タスク
static void process_msg_chkcmd();

// Linkeyチェックサムシーケンス
static int check_sum_seq(uint8_t* cdata, uint8_t cmd_d1, uint8_t cmd_d2, uint8_t cmd_d3, int l_num);

// UART送信
static int uart_send(const uint8_t* data, size_t size);

// UART ISRコールバック
static bool_t uart_isr_callback(int type, uint32_t opt);

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
    { DRVCMN_GPIO_PORT_B,	0,	},	/* PB0  */
    
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
    MSG_RESET,
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

// Linkey command data
static const uint8_t TEST_DATA[] = {0x00, 'T', 'E', 'S', 'T'};
static const uint8_t CMD_FAKE_FRAME[] = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01};	// FAKE
static const uint8_t CMD_OPEN_FRAME[] = {0x02, 0x02, 0x02, 0x01, 0x03, 0x03};	// OPEN
static const uint8_t CMD_OPACK_FRAME[] = {0x02, 0x04, 0x41, 0x02, 0x01, 0x01, 0x43, 0x03};	// OK_ACK
static const uint8_t CMD_CLOSE_FRAME[] = {0x02, 0x02, 0x02, 0x02, 0x00, 0x03};	// CLOSE
static const uint8_t CMD_CNACK_FRAME[] = {0x02, 0x03, 0x41, 0x05, 0x03, 0x47, 0x03};	// OK_ACK
static const uint8_t CMD_MGACK_FRAME[] = {0x02, 0x04, 0x41, 0x04, 0x03, 0x01, 0x47, 0x03};	// OK_ACK
static const uint8_t CMD_BTREQ_FRAME[] = {0x02, 0x02, 0x03, 0x07, 0x04, 0x03};	// OK_ACK
static const uint8_t CMD_STS_FRAME[]   = {0x02, 0x02, 0x04, 0x02, 0x06, 0x03};	// STATUS
static const uint8_t CMD_MGNET_FRAME[] = {0x02, 0x02, 0x04, 0x03, 0x07, 0x03};	// MGNET SENSOR
static const uint8_t CMD_ATLOCK_FRAME[] = {0x02, 0x02, 0x04, 0x06, 0x02, 0x03};	// AUTOLOCK MODE
static const uint8_t CMD_SECURE_FRAME[] = {0x02, 0x02, 0x04, 0x07, 0x03, 0x03};	// SEKURITY
static const uint8_t CMD_ABSENCE_FRAME[] = {0x02, 0x02, 0x04, 0x07, 0x03, 0x03};// Absence RT MODE
static const uint8_t CMD_ABSENCM_FRAME[] = {0x02, 0x03, 0x02, 0x07, 0x01, 0x04, 0x03};// Absence MODE
static const uint8_t CMD_DEDVLT_FRAME[] = {0x02, 0x02, 0x04, 0x02, 0x06, 0x03};	// DEDVOLT MODE
static const uint8_t CMD_DEVMOD_FRAME[] = {0x02, 0x02, 0x05, 0x03, 0x06, 0x03};	// CONFIRM DEVICE MODULE

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
static const int32_t MGNT_TIMEOUT = 500;    //波形より実測値
// 施錠タイムアウト
static const int32_t LOCK_TIMEOUT = -1;
// バッテリー確認タイムアウト
static const int32_t CHEK_TIMEOUT = -1;

static uint8_t l_header_buf[LNKY_HEADER_SIZE];
/*********************************************************************
 * 公開関数
 ********************************************************************/


/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Linkey Peripheral初期化シーケンス GPIO初期化
戻り値：
引数：
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
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
 * ドライバ初期化(API)
 */

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Linkey デバイスドライバ初期化(API) ドライバメインタスク起動 
　UART初期化メッセージ送信
戻り値：
引数：callback：aplilockで指定
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
void drvllock_initialize(DRVLLOCK_CALLBACK_T callback)
{
    assert(callback);
    /* コールバック(GLOBAL)を設定 */
    s_callback = callback;
    
    ER er = 0;

    // ドライバタスク起動
    act_tsk(TSK_DRVLLOCK);
    assert(er == E_OK);

    // 初期化メッセージを送信
    er = snd_dtq(DTQ_DRVLLOCK, (intptr_t)MSG_INITIALIZE);
    assert(er == E_OK);
    
}

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Linkey デバイスリセット(API)
　リセットコマンドメッセージ送信
戻り値：
引数：
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
void reset_cmd_msg()
{
    DBGLOG0("reset_cmd_msg .");
    
    ER er = snd_dtq(DTQ_DRVLLOCK, (intptr_t)MSG_RESET);
    assert(er == E_OK);
    
}

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Linkey ドアセンサ状態取得(API)
　ドアセンサコマンドメッセージ送信
戻り値：
引数：
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
void status_mgnt_command()
{
    DBGLOG0("status_mgnt_msg .");
    
    ER er = snd_dtq(DTQ_DRVLLOCK, (intptr_t)MSG_MGNTCMD);
    assert(er == E_OK);
}

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Linkey 解錠コマンド発行(API)
　ドアセンサコマンドメッセージ送信
戻り値：
引数：
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
void open_command_msg()
{
    DBGLOG0("open_command_msg .");
    
    ER er = snd_dtq(DTQ_DRVLLOCK, (intptr_t)MSG_OPCMD);
    assert(er == E_OK);
}

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Linkey バッテリ状態受信(API)
　バッテリ状態受信メッセージ送信
戻り値：
引数：
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
void status_chek_command()
{
    DBGLOG0("chek_command_msg .");
    
    ER er = snd_dtq(DTQ_DRVLLOCK, (intptr_t)MSG_CHKCMD);
    assert(er == E_OK);
    
}

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Linkey ドライバメインタスク(データキューメッセージ受信)
　API関数からのデータキューメッセージ待ち
戻り値：
引数：
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
void drvllock_task(intptr_t exinf)
{
    DBGLOG0("drvllock_task() starts .");

    while (true) {
        DBGLOG0("drvllock_task() while .");
        intptr_t msg = 0;
        ER er_rcv = trcv_dtq(DTQ_DRVLLOCK, (intptr_t*)&msg, TMO_FEVR);
        assert((er_rcv == E_OK) || (er_rcv == E_TMOUT));
        DBGLOG1("drvllock_task() msg=%d", msg);

        switch (msg) {
        case MSG_INITIALIZE:
            process_msg_initialize();
            break;
        case MSG_RESET:
            DBGLOG0("drvllock_task() MSG_RESET .");
            dly_tsk(500);
            if (drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[1])) {
                DBGLOG0("POWER_ON MSG_RESET .");
                reset_cmd();
            }

            break;
        case MSG_MGNTCMD:
            process_msg_mgntcmd();
            break;
        case MSG_OPCMD:
            process_msg_opencmd();
            break;
//        case MSG_DEDVLTCMD:
//            process_msg_dvltcmd((uint8_t*)(blk->data), blk->size);
//            break;
        case MSG_CHKCMD:
            process_msg_chkcmd();
            break;
        default:
            assert(false);
            break;
        }
    }
    s_callback = NULL;
}

/*
 * 鍵錠GPIO 割込みサービスルーチン
 */
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Linkey 鍵錠GPIO 割込みサービスルーチン
　ドアセンサからのハード割込み受信
戻り値：0
引数：割込みPINナンバー
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
int32_t drvlilock_sensor_isr(uint32_t pinno)
{
    DBGLOG1("sensor_isr %d", pinno);

    if (!s_callback) {
        return 0;
    }

    int32_t intrpnum = 0; 
    intrpnum = pinno;

    if(intrpnum == INTRPT_DRSNS){
        if (!(drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[0]))) {
            DBGLOG0("sensor_isr_doorcls. ");
//            s_callback(DRVLLOCK_EVT_DOOR_CLOSEISR, 0);
        }
        /*
        else if(drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[0])) {
                DBGLOG0("sensor_isr_dooropn. ");
            ER er = ipsnd_dtq(DTQ_DRVLLOCK, (intptr_t)MSG_MGNTCMD);
            assert(er == E_OK);
            s_callback(DRVLLOCK_EVT_DOOR_OPENISR, 0);
        }
        */
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
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Linkey デバイスドライバ(Device)
　UART通信初期化設定
戻り値：
引数：
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
void process_msg_initialize()
{
    
    // UART初期化
    drvcmn_uart_initialize(LLOCK_UART, &UART_SETTING);

    DBGLOG0("drvllock_task() MSG_RESET .");
    dly_tsk(500);
    if (drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[1])) {
        led_turn_off_all();
        led_set_pattern(DRVLED_LED_FRONT_GREEN, LEDPATTERN_BLINK_2HZ);
    }
}
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Linkey リセットコマンド(Device)
　GPIOPINによるリセット　LinkeyからのModule連結確認コマンド受信待ち　コマンドチェックサム確認
戻り値：
引数：
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
void reset_cmd()
{
    int error = 0;
   
    uint8_t* data;
    size_t size;
    
    data = l_header_buf;
    DBGLOG1("msg_rst %d", (drvcmn_gpio_pin_get(&GPIO_PINS_INPUT[1])));
    size = LNKY_HEADER_SIZE;
    
    //リセット送信
    drvcmn_gpio_pin_set(&GPIO_PIN_RST, false);		// RESET => L (リセット)
    dly_tsk(100);

    drvcmn_gpio_pin_set(&GPIO_PIN_RST, true);		// RESET => H (リセット解除)

    //接続確認受信
    DBGLOG0("uart_recieve_rst .");

    for (int i = 0; i < size; i++) {
        data[i] = 0x00;
    }
    //LinkeyからDEV Module連結確認コマンド受信待ち
    uart_receive(data, size, RESET_TIMEOUT);

//    for (int i = 0; i < size; i++) {
//        syslog(LOG_NOTICE, "receive_rst 0x%02x '%c'", data[i], data[i]);
//    }

    DBGLOG1("heder_buf_rst: 0x%02x", data[0]);

    error = check_sum_seq(data, CMD_DEVMOD_FRAME[2], CMD_DEVMOD_FRAME[3], 0, 0);
    if (error) {
        DBGLOG0("check_sum_reset: OK! .");
    }else{
        DBGLOG1("check_sum_reset: error(%d)", error);
    }
    
    error = uart_send(CMD_CNACK_FRAME, sizeof(CMD_CNACK_FRAME));
    
    if(error){  //Linkey連結Error発生
        s_callback(DRVLLOCK_EVT_CONECT_FALE, error);
    }else{      //連結完了
        s_callback(DRVLLOCK_EVT_CONECT_COMP, error);
    }

}
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Linkey 解錠コマンド発行(Device)
　LinkeyへOPENコマンド送信　LinkeyからのACKコマンド受信待ち　コマンドチェックサム確認
戻り値：
引数：
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
void process_msg_opencmd()
{
    
    int error = 0;

    uint8_t* data;
    size_t size;
    
    data = l_header_buf;
    
    size = LNKY_HEADER_SIZE;
        
    // LinkeyへOPENコマンド送信
    error = uart_send(CMD_FAKE_FRAME, sizeof(CMD_FAKE_FRAME));
    dly_tsk(10);
    error = uart_send(CMD_OPEN_FRAME, sizeof(CMD_OPEN_FRAME));
    if (error) {
        DBGLOG1("msg_open: uart_send() error(%d)", error);
    }
    DBGLOG0("msg_open: uart_send starts .");
    //LinkeyからACK待ち
    uart_receive(data, size, LOCK_TIMEOUT);
//    for (int i = 0; i < size; i++) {
//        syslog(LOG_NOTICE, "receive 0x%02x '%c'", data[i], data[i]);
//    }
    
    //ACK CHECK
    error = check_sum_seq(data, CMD_OPACK_FRAME[3], CMD_OPACK_FRAME[4], CMD_OPACK_FRAME[5], 0);
    if (error) {
        DBGLOG1("check_sum_opcmd: error(%d)", error);
    }else{
        DBGLOG0("check_sum_opcmd: OK! .");
    }
    if(error){
        s_callback(DRVLLOCK_EVT_OPEN_ERROR, error);
    }else{
        s_callback(DRVLLOCK_EVT_OPEN_STATUS, error);
    }
    

}

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Linkey バッテリ状態受信(Device)
　前回のデータ受信で起動コマンドが受信されているか確認　Linkeyからのバッテリイベント待ち
　コマンドチェックサム確認
戻り値：
引数：
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
void process_msg_chkcmd()
{
    int fake_num = 8;
    int error = 0;
    int l;
    uint8_t* data;
    size_t size;
    
    data = l_header_buf;
    
    size = LNKY_HEADER_SIZE;
    
    DBGLOG0("get_check_status: uart_recieve starts .");
    //前回のデータ受信で起動コマンドが受信されているか確認
    for(l = 0; l < fake_num; l++){
        //Fake Bit Search
        if(data[l] == 0xaa){
            DBGLOG2("fake bit: 0x%x, i:0%d ", data[l], l);
            l++;
            break;
        }else if(l == fake_num -1){
            DBGLOG0("fake bit get error .");
            goto end;
        }
    }
    
    for (int i = 0; i < size; i++) {
        data[i] = 0x00;
    }
    //Linkeyからのバッテリイベント待ち
    uart_receive(data, size, CHEK_TIMEOUT);

//    for (int i = 0; i < size; i++) {
//        syslog(LOG_NOTICE, "receive 0x%02x '%c'", data[i], data[i]);
//    }
    //コマンド受信チェックサムシーケンス
    error = check_sum_seq(data, CMD_BTREQ_FRAME[2], CMD_BTREQ_FRAME[3], 0 ,l);
    if (error) {
        DBGLOG1("check_sum_bttry: error(%d)", error);
    }else{
        DBGLOG0("check_sum_bttry: OK! .");
    }
    
    if(error){
        s_callback(DRVLLOCK_EVT_BATERRY_ERROR, 0);
    }else{
        s_callback(DRVLLOCK_EVT_OPEN_STATUS, 0);
    }
end: ;
}
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Linkey ドアセンサ状態確認コマンド発行(Device)
　ドアセンサ状態確認コマンド発行　タイムアウト"00"データ受信判定確認　LinkeyからのACK待ち
戻り値：
引数：
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
void process_msg_mgntcmd()
{
    int error = 0;
    int cnt0 = 0;
    uint8_t* data;
    size_t size;
    
    data = l_header_buf;
    
    size = LNKY_HEADER_SIZE;
    //ドアセンサ状態コマンド発行
    error = uart_send(CMD_FAKE_FRAME, sizeof(CMD_FAKE_FRAME));
    dly_tsk(500);
    error = uart_send(CMD_MGNET_FRAME, sizeof(CMD_MGNET_FRAME));
    if (error) {
        DBGLOG1("get_mgnt_status: uart_send() error(%d)", error);
    }
    DBGLOG0("get_mgnt_status: uart_send starts .");

    for (int i = 0; i < size; i++) {
        data[i] = 0x00;
    }
    
    //ACK受信
    error = uart_receive(data, size, MGNT_TIMEOUT);
    DBGLOG1("size(%d)", size);
    //タイムアウト"00"データ確認
    for (int i = 0; i < size; i++) {
//        DBGLOG1("receive 0x%02x", data[i]);
        if(data[i] == 0x00){
            cnt0++;
//            DBGLOG1("cnt0(0x%x)", cnt0);
        }
    }
    //タイムアウト"00"データ受信判定
    if((cnt0 > 1)&&(cnt0 < 4)){
        //受信データが2、3個"00"(途中からデータ受信できず)
        error = false;
        DBGLOG1("error(%d)", error);
    }else{
        //受信データがALL"00"(データ受信できず)
        error = true;
        DBGLOG1("error(%d)", error);
    }
    //リトライ処理
    if(error){
        //送信リトライ
        error = uart_send(CMD_MGNET_FRAME, sizeof(CMD_MGNET_FRAME));

        for (int i = 0; i < size; i++) {
            data[i] = 0x00;
        }
        error = uart_receive(data, size, MGNT_TIMEOUT);
    }
    

    DBGLOG1("heder_buf: 0x%02x", data[0]);

    error = check_sum_seq(data, CMD_MGACK_FRAME[3], CMD_MGACK_FRAME[4], CMD_MGACK_FRAME[5], 0);
    if (error) {
        DBGLOG0("check_sum_mgn: OK! .");
    }else{
        DBGLOG1("check_sum_mgn: error(%d)", error);
    }
    if(error){
        s_callback(DRVLLOCK_EVT_OPEN_STATUS, error);
    }else{
        s_callback(DRVLLOCK_EVT_CLOSE_STATUS, error);
    }
    
}

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Linkey UART送信処理(Device)
　引数のデータ、サイズのUART送信　UART割込み待ち
戻り値：タイムアウトエラー
引数：送信データ、送信サイズ
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
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

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Linkey UART受信処理(Device)
　引数のデータ、サイズのUART受信　UART割込み待ち
戻り値：タイムアウトエラー
引数：送信データ、送信サイズ
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
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

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Linkey コマンド受信チェックサムシーケンス
戻り値：error:uart 送信エラー
引数：cdata:受信データ  cmd_d1:データ照合用1  cmd_d2:データ照合用2  cmd_d3:データ照合用3  
l_num:前回データカウント受け継ぎ
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
int check_sum_seq(uint8_t* cdata, uint8_t cmd_d1, uint8_t cmd_d2, uint8_t cmd_d3, int l_num)
{
    int error = 0;
    int fake_num = 8;
    int chksum = 0;
    int data_length = 0;
    int m, k, l;
    int data_stsbit = 2;
    m = 0; k = 0;
    
    l = l_num;
    //前回のデータ受信で起動コマンドの有無を確認
    if(!l){
        //コマンド チェック シーケンス
        for(l = 0; l < fake_num; l++){
            //Fake Bit Search
            if(cdata[l] == 0xaa){
                DBGLOG2("fake bit: 0x%x, i:0%d ", cdata[l], l);
                break;
            }else if(l == fake_num -1){
                DBGLOG0("fake bit get error .");
                goto end;
            }
        }
    }
    //スタートビットチェック
    for(; l < fake_num; l++){
        //Start Bit Search
        if(cdata[l] == 0x02){
            DBGLOG2("stx bit: 0x%0x, l:0%d ", cdata[l], l);
            break;
        }else if(l == fake_num -1){
            DBGLOG0("stx bit get error .");
            goto end;
        }
    }
    //データレングス保存
    data_length = cdata[l + 1];

    //チェックサム計算
    while(m < data_length){
        chksum = chksum ^ cdata[l + data_stsbit + k];
        k++;
        m++;
    }
//    DBGLOG2("chksum: 0x%0x, l: 0x%0x", chksum, l);
    
    //チェックサム照合
    if(chksum == cdata[l + data_stsbit + data_length]){
        if(l_num){
            l--;
        }
        //データ照合
        if(!l_num){
            if((cdata[l + data_stsbit +1] == cmd_d1) && (cdata[l + data_stsbit + data_length -1] == cmd_d2)){
                if(!cmd_d3){
                    DBGLOG0("CMD_NONE");
                    error = false;
                }else if(cdata[l + data_stsbit + data_length -1] == cmd_d3){
                    DBGLOG0("OPEN");
                    error = false;
                }else{
                    DBGLOG0("OPEN_ERR");
                    error = false;
                }
            }else{
                DBGLOG0("CMD_OK");
                error = true;
            }
        }else{
            if((cdata[l + data_stsbit +1] == cmd_d1) && (cdata[l + data_stsbit + data_length] == cmd_d2)){
                if(!cmd_d3){
                    DBGLOG0("CMD_NONE_l");
                    error = true;
                }else if(cdata[l + data_stsbit + data_length -1] == cmd_d3){
                    DBGLOG0("OPEN_l");
                    error = true;
                }else{
                    DBGLOG0("OPEN_ERR_l");
                    error = true;
                }
            }else{
                DBGLOG0("CMD_OK_l");
                error = false;
            }
            
        }
    }
end:
    return error;
}

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Linkey UART割込みコールバッグ(Device)
　UART割込み待ち
戻り値：
引数：割込み種別
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
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

