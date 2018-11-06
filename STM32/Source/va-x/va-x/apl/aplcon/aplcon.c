/*
 * VA-X シリアルコンソール
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/04/11 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "aplcon.h"

#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
#include "chip_serial.h"

#include "aplauth.h"
#include "aplui.h"
#include "cmntimer.h"
#include "mdlstrg.h"
#include "mdlstrg_data.h"

#include "drvcmn_gpio.h"
#include "drvlock.h"
#include "drvllock.h"
#include "drvts.h"
#include "drvbvol.h"
#include "drvicc.h"
#include "drvwifi.h"

#include "mdlauth.h"
#include "drvcam.h"
#include "drvirled.h"
#include "cmnbuf.h"
#include "cmndbg.h"
#include "drvble.h"
#include "drvpwr.h"

//
// マクロ
//

// デバッグログ
#if 1
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[APLCON]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[APLCON]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[APLCON]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[APLCON]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

// マクロ定数
#define SERIAL_PORT_ID	1	// シリアルポートのID

//
// 型
//

// コマンド関数
typedef int (*APLCON_CMDFUNC_T)(const char* opt1, const char* opt2, const char* opt3);

// ステート
#define LINE_LEN	64
typedef struct {
    uint8_t input_buf[LINE_LEN];
    size_t input_len;
    uint8_t cmd[LINE_LEN];
    size_t cmd_len;
} CMD_STATE_T;

// コマンド
typedef struct {
    int cmd_type;
    const char* cmd_name;
    size_t cmd_len;
    APLCON_CMDFUNC_T cmd_func;
    const char* help;
} CMD_DEF_T;

// 関数プロトタイプ
static int handle_key_event(CMD_STATE_T* state, uint8_t key);
static int process_cmd(uint8_t* cmd, size_t cmd_len);
static int cmdfunc_help(const char* opt1, const char* opt2, const char* opt3);
static int cmdfunc_vareg(const char* opt1, const char* opt2, const char* opt3);
static int cmdfunc_vadel(const char* opt1, const char* opt2, const char* opt3);
static int cmdfunc_valist(const char* opt1, const char* opt2, const char* opt3);
static int cmdfunc_lock(const char* opt1, const char* opt2, const char* opt3);
static int cmdfunc_llock(const char* opt1, const char* opt2, const char* opt3);
static int cmdfunc_led(const char* opt1, const char* opt2, const char* opt3);
static int cmdfunc_buz(const char* opt1, const char* opt2, const char* opt3);
static int cmdfunc_tim(const char* opt1, const char* opt2, const char* opt3);
static int cmdfunc_tstest(const char* opt1, const char* opt2, const char* opt3);
static int cmdfunc_bvol(const char* opt1, const char* opt2, const char* opt3);
static int cmdfunc_icc(const char* opt1, const char* opt2, const char* opt3);
static int cmdfunc_wifi(const char* opt1, const char* opt2, const char* opt3);
static int cmdfunc_pwr(const char* opt1, const char* opt2, const char* opt3);
static int cmdfunc_freset(const char* opt1, const char* opt2, const char* opt3);
static int cmdfunc_cam(const char* opt1, const char* opt2, const char* opt3);
static void tstest_callback(int32_t evt);
static void drvlock_callback_aplcn(int event, int opn_sts);
static void drvllock_callback_aplcn(int32_t evt, int32_t error);
static void drvbvol_callback(uint32_t voltage_mv);
static void drvpwr_callback(int evt, uint32_t voltage_mv);
static void drvicc_callback(int32_t evt, int32_t error, intptr_t data);
static void drvwifi_callback(int32_t evt, int32_t error, intptr_t opt);
static int cmdfunc_freset(const char* opt1, const char* opt2, const char* opt3);
static void strg_callback(int event, intptr_t opt1, intptr_t opt2);	// ストレージミドルコールバック
static void drvcam_callback(int32_t type, int32_t error);
static void drvts_callback(int event);

//
// 定数
//

// フラグ
static const FLGPTN FLGPTN_MDLSTRG_REQ_COMPLETE =			(0x1 << 0);
static const FLGPTN FLGPTN_DRVCAM_INITIALIZE_COMPLETE =		(0x1 << 1);
static const FLGPTN FLGPTN_DRVCAM_POWERON_COMPLETE =		(0x1 << 2);
static const FLGPTN FLGPTN_DRVCAM_POWEROFF_COMPLETE =		(0x1 << 3);
static const FLGPTN FLGPTN_DRVCAM_PREPARE_COMPLETE =		(0x1 << 4);
static const FLGPTN FLGPTN_DRVCAM_CAPTURE_COMPLETE =		(0x1 << 5);
static const FLGPTN FLGPTN_DRVTS_TOUCH =					(0x1 << 6);
static const FLGPTN FLGPTN_DRVTS_RELEASE =					(0x1 << 7);

// コマンド定義
enum {
    CMD_NONE = 0,
    CMD_HELP,
    CMD_VAREG,
    CMD_VADEL,
    CMD_VALIST,
    CMD_LOCK,
    CMD_LLOCK,
    CMD_LED,
    CMD_BUZ,
    CMD_TIM,
    CMD_TSTEST,
    CMD_BVOL,
    CMD_ICC,
    CMD_WIFI,
    CMD_PWR,
    CMD_FRESET,
    CMD_CAM,
};

#define CMD_DEF_ITEM(type, name, cmd_func, help)\
    {type, name, sizeof(name) - 1, cmd_func, help}

static const CMD_DEF_T CMD_DEF[] = {
    CMD_DEF_ITEM(CMD_HELP, "HELP", cmdfunc_help, "HELP"),
    CMD_DEF_ITEM(CMD_VAREG, "VAREG", cmdfunc_vareg, "VAREG <index>"),
    CMD_DEF_ITEM(CMD_VADEL, "VADEL", cmdfunc_vadel, "VADEL <index>"),
    CMD_DEF_ITEM(CMD_VALIST, "VALIST", cmdfunc_valist, "VALIST"),
    CMD_DEF_ITEM(CMD_LOCK, "LOCK", cmdfunc_lock,"LOCK <index>"),
    CMD_DEF_ITEM(CMD_LLOCK, "LLOCK", cmdfunc_llock,"LLOCK <index>"),
    CMD_DEF_ITEM(CMD_LED, "LED", cmdfunc_led, "LED <led> <pattern>"),
    CMD_DEF_ITEM(CMD_BUZ, "BUZ", cmdfunc_buz, "BUZ <pattern>"),
    CMD_DEF_ITEM(CMD_TIM, "TIM", cmdfunc_tim, "TIM <id> <type> <interval>"),
    CMD_DEF_ITEM(CMD_TSTEST, "TSTEST", cmdfunc_tstest, "TSTEST <cmd>"),
    CMD_DEF_ITEM(CMD_BVOL, "BVOL", cmdfunc_bvol, "BVOL <cmd>"),
    CMD_DEF_ITEM(CMD_ICC, "ICC", cmdfunc_icc, "ICC <cmd>"),
    CMD_DEF_ITEM(CMD_WIFI, "WIFI", cmdfunc_wifi, "WIFI <type>"),
    CMD_DEF_ITEM(CMD_PWR, "PWR", cmdfunc_pwr, "PWR <cmd>"),
    CMD_DEF_ITEM(CMD_BUZ, "FRESET", cmdfunc_freset, "FRESET <type>"),
    CMD_DEF_ITEM(CMD_CAM, "CAM", cmdfunc_cam, "CAM <cmd>"),
};

//
// 内部変数
//
static CMD_STATE_T s_cmd_state = {0};
static struct {
    APLEVT_EXTRA_DATA_T vareg_ext;
} s_events;
static APLEVT_EVENT_RECEIVER_FUNC_T s_receiver_func = NULL;
static intptr_t s_strgmdl_req_state = 0;

static uint8_t test_buf[8];

static uint8_t test_buf2[8];

/*********************************************************************
 * 公開関数
*********************************************************************/

/*
 * 共通初期化API(APLEVT_INITIALIZE_FUNC_T)
 */
int32_t aplcon_initialize(APLEVT_EVENT_RECEIVER_FUNC_T receiver_func)
{
    assert(receiver_func);
    assert(!s_receiver_func);

    s_receiver_func = receiver_func;

    // タスクを起動
    ER er = act_tsk(TSK_APLCON);
    assert(er == E_OK);

    return 0;
}

/*
 * 共通イベント受信API(APLEVT_EVENT_RECEIVER_FUNC_T)
 */
int32_t aplcon_event(APLEVT_EVENT_T* event)
{
    assert(!"NOT IMPLEMENTED");
    return 0;
}

/*
 *  メインタスク
 */
void aplcon_task(intptr_t exinf)
{
	DBGLOG0("aplcon_task() starts.");

    ER ercd = 0;

    dly_tsk(100);	// 少し遅らせる

    // シリアルポートの初期化
	ercd = serial_opn_por(SERIAL_PORT_ID);
    assert(ercd == E_OK);
	serial_ctl_por(SERIAL_PORT_ID, IOCTL_CRLF);

    /*
 	 * メインループ
	 */
    uint8_t c = 0;
	while (true) {
		serial_rea_dat(SERIAL_PORT_ID, (char*)&c, 1);

        uint8_t print = 0;
        print = handle_key_event(&s_cmd_state, c);

        // エコー
        if (print != '\0') {
            if (print == '\n') {
                sio_pol_snd_chr('\r', SERIAL_PORT_ID);
            }
            sio_pol_snd_chr(c, SERIAL_PORT_ID);
        }

        //DBGLOG3("%s(%d), 0x%02x", itron_strerror(ercd), ercd, c);
        if (s_cmd_state.cmd_len > 0) {
            // コマンドあり
            DBGLOG2("cmd: %d \"%s\"", s_cmd_state.cmd_len, s_cmd_state.cmd);

            process_cmd(s_cmd_state.cmd, s_cmd_state.cmd_len);

            s_cmd_state.cmd_len = 0;
        }
	}

}

/*********************************************************************
 * 内部関数
 ********************************************************************/

// キーイベント
int handle_key_event(CMD_STATE_T* state, uint8_t key)
{
    assert(state);

    int print = 0;

    if ((state->input_len >= sizeof(state->input_buf) - 1) &&
        (key != '\n') && (key != '\b')) {
        goto end;
    }

    if (key == '\n') {	// LF
        if (state->input_len > 0) {
            // コマンド領域にコピー
            memcpy(state->cmd, state->input_buf, state->input_len);
            state->cmd_len = state->input_len;
            state->cmd[state->cmd_len] = '\0';
            state->input_len = 0;
        }
        print = key;
    } else if (key == '\b') {	// BS
        if (state->input_len > 0) {
            state->input_len--;
            print = key;
        }
    } else if (key >= 0x20 && key <= 0x7e) {	// 文字
        state->input_buf[state->input_len] = key;
        (state->input_len)++;
        print = key;
    } else {
        // 無視する
        print = 0;
    }

    //
end:
    return print;
}

int process_cmd(uint8_t* cmd, size_t cmd_len)
{
    assert(cmd);
    assert(cmd_len > 0);

    // 切り出し
    char* tmp_ptr = NULL;
    strtok_r((char*)cmd, " ", &tmp_ptr);
    size_t first_len = strlen((char*)cmd);

    // 大文字に変換
    for (int i = 0; i < first_len; i++) {
        cmd[i] = toupper(cmd[i]);
    }

    // コマンド検索
    int cmd_type = CMD_NONE;
    APLCON_CMDFUNC_T cmd_func = NULL;
    for (int i = 0; i < sizeof(CMD_DEF)/sizeof(CMD_DEF[0]); i++) {
        if ((CMD_DEF[i].cmd_len == first_len) &&
            (memcmp(CMD_DEF[i].cmd_name, cmd, first_len) == 0)) {
            cmd_type = CMD_DEF[i].cmd_type;
            cmd_func = CMD_DEF[i].cmd_func;
            break;
        }
    }

    // 有効なコマンドでない場合
    if (cmd_type == CMD_NONE) {
        goto end;
    }

    // オプション切り出し
    char* opts[3] = {0};
    for (int i = 0; i < 3; i++) {
        opts[i] = strtok_r(NULL, " ", &tmp_ptr);
    }

    // 関数
    if (cmd_func) {
        cmd_func(opts[0], opts[1], opts[2]);
    }

end:
    return 0;
}

int cmdfunc_help(const char* opt1, const char* opt2, const char* opt3)
{
    DBGLOG0("CMD_HELP");

    syslog(LOG_NOTICE, "Available commands ==============================");
    for (int i = 0; i < sizeof(CMD_DEF)/sizeof(CMD_DEF[0]); i++) {
        syslog(LOG_NOTICE, CMD_DEF[i].help);
    }
    syslog(LOG_NOTICE, "==============================");

    return 0;
}

int cmdfunc_vareg(const char* opt1, const char* opt2, const char* opt3)
{
    DBGLOG0("CMD_VAREG");
    if (!opt1) {
        DBGLOG0("invalid option");
        goto end;
    }

    int index = atoi(opt1);
    if ((index < 0) || (index >= 20)) {
        DBGLOG0("invalid option");
        goto end;
    }

    // イベント生成, 送信
    APLEVT_EVENT_T* event = NULL;
    s_events.vareg_ext.reg_prep_req.index = index;
    aplevt_create_event(&event, APLEVT_VAREG_REQ, 0, &(s_events.vareg_ext));
    s_receiver_func(event);
    aplevt_return_event(event);
    event = NULL;

end:
    return 0;
}

// ストレージミドルコールバック
void strg_callback(int event, intptr_t opt1, intptr_t opt2)
{
    DBGLOG1("STRG CALLBACK: %d", event);

    ER er = E_OK;

    switch (event) {
    case MDLSTRG_EVT_REQUEST_COMPLETE:
        s_strgmdl_req_state = opt1;
        er = set_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE);
        assert(er == E_OK);
        break;
    default:
        break;
    }
}

int cmdfunc_vadel(const char* opt1, const char* opt2, const char* opt3)
{
    DBGLOG0("CMD_VADEL");

    int index = atoi(opt1);	// インデックス
    ER er = E_OK;

    // 読み出し
    MDLSTRG_REQUEST_T FRESET_REQ = {
        .data_type = MDLSTRG_DATA_TYPE_AUTH,
        .request_type = MDLSTRG_REQ_TYPE_DELETE,
        .opt1 = index,
    };
    mdlstrg_request(&FRESET_REQ, strg_callback);
    er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
    assert(er == E_OK);

    return 0;
}

int cmdfunc_valist(const char* opt1, const char* opt2, const char* opt3)
{
    DBGLOG0("CMD_VALIST");

    ER er = E_OK;
    int exists_reg = 0;
    int exists_learned = 0;

    for (int i = 0; i < MDLSTRG_MAX_NDATA_AUTH; i++) {
        // 登録データ確認
        MDLSTRG_REQUEST_T FRESET_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_AUTH,
            .request_type = MDLSTRG_REQ_TYPE_EXISTS,
            .opt1 = i,
        };
        mdlstrg_request(&FRESET_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        exists_reg = s_strgmdl_req_state;

        // 学習データ確認
        MDLSTRG_REQUEST_T FRESET_REQ_LEARNED = {
            .data_type = MDLSTRG_DATA_TYPE_AUTH_LEARNED,
            .request_type = MDLSTRG_REQ_TYPE_EXISTS,
            .opt1 = i,
        };
        mdlstrg_request(&FRESET_REQ_LEARNED, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        exists_learned = s_strgmdl_req_state;

        DBGLOG3("[%d]exists: %d, %d", i, exists_reg, exists_learned);
        dly_tsk(10);
    }

    return 0;
}

int cmdfunc_lock(const char* opt1, const char* opt2, const char* opt3)
{
    DBGLOG0("CMD_LOCK");

    int index = atoi(opt1);
    if ((index < 0) || (index >= 6)) {
        DBGLOG0("invalid option");
        goto end;
    }
    DBGLOG1("CMD_LOCK: index = 0x%02x", index);
    if(index == 0x01)
    {
        DBGLOG0("initialize");
        drvlock_initialize(drvlock_callback_aplcn);
    }else if(index == 0x02){
        DBGLOG0("get_state");
        drvlock_get_state();
    }else if(index == 0x03){
        DBGLOG0("open_lock");
        drvlock_open();
    }else if(index == 0x04){
        DBGLOG0("locking");
        drvlock_close();
    }
    //for apli
    if(index == 0x05){

            // イベント生成, 送信
    APLEVT_EVENT_T* event = NULL;
    aplevt_create_event(&event, APLEVT_AUTH_COMPLETE, 0, NULL);
    s_receiver_func(event);
    aplevt_return_event(event);
    event = NULL;
        
        
    }
    
end:
    DBGLOG0("CMD_END");
    return 0;
}

int cmdfunc_llock(const char* opt1, const char* opt2, const char* opt3)
{
    DBGLOG0("CMD_LLOCK");

    int index = atoi(opt1);
    if ((index < 0) || (index >= 15)) {
        DBGLOG0("invalid option");
        goto end;
    }

//    int uart_receive(uint8_t* data, size_t data_len, TMO timeout)
    
//    uint8_t* id = l_header_buf;		// id
    size_t type = 8;		// type
    
    DBGLOG1("CMD_LOCK: index = 0x%02x", index);
    if(index == 0x01)
    {
        DBGLOG0("linkey_initialize_pheriphrals");
        drvllock_initialize_peripherals();
    }else if(index == 0x02){
        DBGLOG0("linkey_initialize");
        drvllock_initialize(drvllock_callback_aplcn);
    }else if(index == 0x03){
        DBGLOG0("linkey_reset");
        reset_cmd(drvllock_callback_aplcn, test_buf, type);
    }else if(index == 0x04){
        DBGLOG0("linkey_open");
        open_command(drvllock_callback_aplcn, test_buf, type);
    }else if(index == 0x05){
        DBGLOG0("linkey_recieve");
        uart_receive(test_buf, type, -1);

        for (int i = 0; i < type; i++) {
        syslog(LOG_NOTICE, "receive 0x%02x '%c'", test_buf[i], test_buf[i]);
        }
        
    }else if(index == 0x06){
        DBGLOG0("linkey_status_dedvlt");
//        status_dedvlt_command(drvllock_callback_aplcn, test_buf, type);
    }else if(index == 0x07){
        DBGLOG0("linkey_status_mgnt");
        status_mgnt_command(drvllock_callback_aplcn, test_buf, type);
    }else if(index == 0x08){
        DBGLOG0("linkey_rq_mgnet");
//        mgnet_rq_command(drvllock_callback_aplcn);
    }else if(index == 0x09){
        DBGLOG0("linkey_reset");
//        rst_command(drvllock_callback_aplcn);
    }

end:
    DBGLOG0("CMD_END");
    return 0;
}

/*
 * 電気錠コールバック
 *///for debug
void drvlock_callback_aplcn(int event, int opn_sts)
{
    DBGLOG1("drvlock_callback_aplcn: e=%d", event);
}

void drvllock_callback_aplcn(int32_t evt, int32_t error)
{
    DBGLOG1("drvllock_callback_aplcn: e=%d", evt);
}

// LEDコマンド
int cmdfunc_led(const char* opt1, const char* opt2, const char* opt3)
{
    DBGLOG0("CMD_LED");

    if (!opt1 || !opt2) {
        DBGLOG0("invalid option");
        goto end;
    }

    int led = atoi(opt1);	// LED番号
    int pattern = atoi(opt2);	// パターン番号

    // LEDのパターンを設定
    aplui_dbg_set_led_pattern(led, pattern);

end:
    return 0;
}

// BUZコマンド
int cmdfunc_buz(const char* opt1, const char* opt2, const char* opt3)
{
    DBGLOG0("CMD_BUZ");

    if (!opt1) {
        DBGLOG0("invalid option");
        goto end;
    }

    int pattern = atoi(opt1);	// パターン番号

    // ブザーのパターンを設定
    aplui_dbg_set_buz_pattern(pattern);

end:
    return 0;
}

// TIMコマンド
static void timer_callback(int timer_id, intptr_t otp1)
{
    DBGLOG1("TIMER CALLBACK: %d", timer_id);
}

int cmdfunc_tim(const char* opt1, const char* opt2, const char* opt3)
{
    DBGLOG0("CMD_TIM");

    if (!opt1 || !opt2 || !opt3) {
        DBGLOG0("invalid option");
        goto end;
    }

    int id = atoi(opt1);		// id
    int type = atoi(opt2);		// type
    int interval = atoi(opt3);	// interval

    // ブザーのパターンを設定
    cmntimer_set_timer(id, type, interval, timer_callback, 0);


end:
    return 0;
}

// TSTEST
int cmdfunc_tstest(const char* opt1, const char* opt2, const char* opt3)
{
    DBGLOG0("CMD_TSTEST");

    if (!opt1) {
        DBGLOG0("invalid option");
        goto end;
    }

    int test = atoi(opt1);		// test

    switch (test) {
    case 0:	// drvts_start
        DBGLOG0("[2]drvts_start()");
        drvts_start(tstest_callback);
        break;
    case 1:	// drvts_stop
        DBGLOG0("[3]drvts_stop()");
        drvts_stop(tstest_callback);
        break;
    default:
        break;
    }

end:
    return 0;
}

void tstest_callback(int32_t evt)
{
    DBGLOG1("tstest_callback(evt=%d)", evt);
}

// BVOL
int cmdfunc_bvol(const char* opt1, const char* opt2, const char* opt3)
{
#if 0
#if 1	// TODO: 電源管理側で管理する
    static const DRVCMN_GPIO_PIN_T GPIO_PIN_3V3_ON = {
        DRVCMN_GPIO_PORT_E, 15,	// 3V3_ON (Active H)
    };

    /* GPIOピン設定(ADC電圧印加) */
    static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_ENABLE = {
        .mode = DRVCMN_GPIO_MODE_GPOUT,
        .otype = DRVCMN_GPIO_OTYPE_PP,
        .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
        .pupd = DRVCMN_GPIO_PUPD_PULLDOWN,
    };

#endif

    DBGLOG0("CMD_BVOL");

    if (!opt1) {
        DBGLOG0("invalid option");
        goto end;
    }

    int test = atoi(opt1);		// test

    switch (test) {
    case 0:	// initialize
        DBGLOG0("drvbvol_initialize()");
        drvbvol_initialize();

#if 1	// TODO: 電源管理側で管理
        drvcmn_gpio_pin_initialize(&GPIO_PIN_3V3_ON, &GPIO_SETTING_ENABLE);	/* 3V3 */
#endif
        break;
    case 1:
        DBGLOG0("drvbvol_get_voltage()");
        drvbvol_get_voltage(drvbvol_callback);
        break;
    case 2:
        DBGLOG0("3V3ON => L");
#if 1	// TODO: 電源管理側で管理
        drvcmn_gpio_pin_set(&GPIO_PIN_3V3_ON, false);	/* 3V3 => L (2.8v) */
#endif
        break;
    case 3:
        DBGLOG0("3V3ON => H");
#if 1	// TODO: 電源管理側で管理
        drvcmn_gpio_pin_set(&GPIO_PIN_3V3_ON, true);	/* 3V3 => H (3.3v) */
#endif
        break;
    default:
        break;
    }

end:
#endif
    return 0;
}

void drvpwr_callback(int evt, uint32_t voltage_mv)
{
    DBGLOG1("Battery: %d mv", voltage_mv);
}

int cmdfunc_icc(const char* opt1, const char* opt2, const char* opt3)
{
    DBGLOG0("CMD_ICC");

    if (!opt1) {
        DBGLOG0("invalid option");
        goto end;
    }

    int cmd = atoi(opt1);		// cmd
    static DRVICC_FELICA_IDM_T idm;
    const static DRVICC_FELICA_BLOCK_T sample = { .block = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F }};
    static DRVICC_FELICA_BLOCK_T block;
    static DRVICC_FELICA_BLOCK_DESC_T desc = {
        .sys = 0x8CCE,
        .service = 0x2009,
        .block = 0,
//        .nblocks = 1,
    };

    switch (cmd) {
    case 0:
        drvicc_initialize(drvicc_callback);
        break;
    case 1:
        drvicc_start_polling(drvicc_callback, &idm);
        break;
    case 2:
        drvicc_cancel_polling(drvicc_callback);
        break;
    case 3:
        memcpy(desc.idm, idm.idm, sizeof(desc.idm));
        drvicc_read(drvicc_callback, &block, &desc);
        break;
    case 4:
        drvicc_write(drvicc_callback, &sample, &desc);
        break;
    case 9:
        cmndbg_hexdump(idm.idm, sizeof(idm.idm), "Felica IDm");
        cmndbg_hexdump(block.block, sizeof(block.block), "service_2009");
        break;
    default:
        break;
    }

end:
    return 0;
}

void drvicc_callback(int32_t evt, int32_t error, intptr_t data)
{
    DBGLOG3("drvicc_callback(e=%d, error=0x%08x, data=0x%08x", evt, error, data);
}

int cmdfunc_wifi(const char* opt1, const char* opt2, const char* opt3)
{
    DBGLOG0("CMD_WIFI");

    if (!opt1) {
        DBGLOG0("invalid option");
        goto end;
    }

    int cmd = atoi(opt1);		// cmd

    switch (cmd) {
    case 0:
        drvwifi_initialize(drvwifi_callback);
        break;
    case 1:
        
        break;
    default:
        break;
    }

end:
    return 0;
}

/*
 * Wifiドライバコールバック
 */
void drvwifi_callback(int32_t evt, int32_t error, intptr_t opt)
{
    DBGLOG3("drvwifi_callback(evt=%d,error=%d,opt=0x%08x)", evt, error, opt);


}

int cmdfunc_pwr(const char* opt1, const char* opt2, const char* opt3)
{
    DBGLOG0("CMD_PWR");

    if (!opt1) {
        DBGLOG0("invalid option");
        goto end;
    }

    int cmd = atoi(opt1);		// cmd

    DRVPWR_RTC_DATETIME_T datetime = {0};

#if 1
    static bool_t initialized = false;
    if (!initialized) {
        drvpwr_initialize();
        initialized = true;
    }
#endif

    switch (cmd) {
    case 0:	{
//          drvpwr_enable_wakeup(DRVPWR_WUPIN_4_PC13, DRVPWR_WUPOL_FALLING);
        drvpwr_enable_wakeup(DRVPWR_WUPIN_1_PA0, DRVPWR_WUPOL_FALLING);
        DBGLOG0("enable_wakeup...OK");
        drvpwr_enter_standby_mode();
        break;
    }
    case 1: {
        if (!opt2) {
            DBGLOG0("invalid option");
            goto end;
        }
        drvpwr_rtc_set_wakeup(atoi(opt2));
        drvpwr_enter_standby_mode();
        break;
    }
    case 2: {
        DBGLOG1("Wakeup Cause: %d", drvpwr_get_wakeup_cause());
        break;
    }
    case 3: {
        drvpwr_rtc_get(&datetime);
        DBGLOG3("%d/%d/%d", datetime.year, datetime.month, datetime.day);
        DBGLOG3("%d:%d:%d", datetime.hour, datetime.minute, datetime.second);
        DBGLOG2("weekday: %d, ss: %d", datetime.weekday, datetime.msecond);
        break;
    }
    case 4: {
        if (!opt2) {
            DBGLOG0("invalid option");
            goto end;
        }
        datetime.year = (opt2[2] - '0') * 10 + (opt2[3] - '0');
        datetime.month = (opt2[4] - '0') * 10 + (opt2[5] - '0');
        datetime.day = (opt2[6] - '0') * 10 + (opt2[7] - '0');
        datetime.hour = (opt2[8] - '0') * 10 + (opt2[9] - '0');
        datetime.minute = (opt2[10] - '0') * 10 + (opt2[11] - '0');
        datetime.second = (opt2[12] - '0') * 10 + (opt2[13] - '0');
        drvpwr_rtc_set(&datetime);
        break;
    }
    case 5: {
        if (!opt2 || !opt3) {
            DBGLOG0("invalid option");
            goto end;
        }
        drvpwr_use_device(atoi(opt2), atoi(opt3));
        break;
    }
    case 6: {
        drvpwr_get_bat_voltage(drvpwr_callback);
        break;
    }
    default:
        break;
    }

end:
    return 0;
}

int cmdfunc_freset(const char* opt1, const char* opt2, const char* opt3)
{
    DBGLOG0("CMD_FRESET");
    ER er = E_OK;

    if (!opt1 || !opt2) {
        DBGLOG0("invalid option");
        goto end;
    }

    int type = atoi(opt1);		// type

    if (strlen(opt2) != 16) {
        DBGLOG0("invalid serialnumber");
        goto end;
    }
    BNVA_DEVICE_ID_T device_id = {0};
    memcpy(&device_id.serialno, opt2, strlen(opt2));

    bool_t flash_reset = false;
    bool_t serial_reset = false;
    switch (type) {
    case 0: {	// 
        DBGLOG0("[0]ALL RESET");
        flash_reset = true;
        serial_reset = true;
        break;
    }
    default:
        break;
    }

    MDLSTRG_REQUEST_T strg_req = {0};
    FLGPTN flgptn = {0};
    // フラッシュ初期化
    if (flash_reset) {
        memset(&strg_req, 0, sizeof(strg_req));
        strg_req.data_type = MDLSTRG_DATA_TYPE_NONE;
        strg_req.request_type = MDLSTRG_REQ_TYPE_FACTORY_RESET;
        strg_req.opt1 = type;

        mdlstrg_request(&strg_req, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &flgptn, 15000);
        assert(er == E_OK);
    }

    // シリアル再設定
    if (serial_reset) {
        memset(&strg_req, 0, sizeof(strg_req));
        strg_req.data_type = MDLSTRG_DATA_TYPE_DEVICE_ID;
        strg_req.request_type = MDLSTRG_REQ_TYPE_WRITE;
        strg_req.data = (intptr_t)&(device_id);
        strg_req.size = sizeof(device_id);

        mdlstrg_request(&strg_req, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &flgptn, 1000);
        assert(er == E_OK);
    }

end:
    return 0;
}

int cmdfunc_cam(const char* opt1, const char* opt2, const char* opt3)
{
    static void* cambuf = NULL;

    DBGLOG0("CMD_CAM");

    if (!opt1) {
        DBGLOG0("invalid option");
        goto end;
    }

    int cmd = atoi(opt1);		// cmd

    SYSTIM systim1 = 0;
    SYSTIM systim2 = 0;
    ER ercd = 0;
    FLGPTN flgptn = 0;
    switch (cmd) {
    case 0: {
        // 認証ミドル停止
        mdlauth_cancel();
        dly_tsk(100);

        /* カメラドライバ初期化 */
        get_tim(&systim1);
        DBGLOG0("call drvcam_initialize()");
        drvcam_initialize(drvcam_callback);
        ercd = twai_flg(FLG_APLCON, FLGPTN_DRVCAM_INITIALIZE_COMPLETE, TWF_ANDW, &flgptn, -1);
        assert(ercd == E_OK);
        get_tim(&systim2);
        DBGLOG1("complete drvcam_initialize (%dms)", systim2 - systim1);

        /* IR-LEDドライバ初期化 */
        DBGLOG0("call drvirled_initialize()");
        drvirled_initialize();
        DBGLOG0("complete drvirled_initialize()");

        // カメラ電源ON
        get_tim(&systim1);
        DBGLOG0("call drvcam_power_on()");
        drvpwr_use_device(DRVPWR_CAM, true);
        drvcam_power_on(drvcam_callback);
        ercd = twai_flg(FLG_APLCON, FLGPTN_DRVCAM_POWERON_COMPLETE, TWF_ANDW, &flgptn, -1);
        assert(ercd == E_OK);
        get_tim(&systim2);
        DBGLOG1("complete drvcam_power_on() (%dms)", systim2 - systim1);

        // カメラ撮影準備
        get_tim(&systim1);
        DBGLOG0("call drvcam_prepare_capture()");
        DRVCAM_CAPTURE_PARAM_T capture_param = {
            .exposure = 0,						// 露出設定
            .regval = NULL,						// カメラレジスタ設定値(デバッグ用)
            .num_regval = 0,					// カメラレジスタ設定値の数
        };
        drvcam_prepare_capture(drvcam_callback, &capture_param);
        ercd = twai_flg(FLG_APLCON, FLGPTN_DRVCAM_PREPARE_COMPLETE, TWF_ANDW, &flgptn, -1);
        assert(ercd == E_OK);
        get_tim(&systim2);
        DBGLOG1("complete drvcam_prepare_capture() (%dms)", systim2 - systim1);

        /* IR-LED 点灯 */
        drvpwr_use_device(DRVPWR_IRLED, true);
        drvirled_set_state(true);

        // バッファ取得
        void* buf = NULL;
        size_t buffer_size = 0;
        buffer_size = cmnbuf_aquire_buffer(&buf, CMNBUF_USE_CAMERA_CAPTURE);	// キャプチャ用
        assert(buf);
        assert(buffer_size >= DRVCAM_CAPTURE_BUFFER_SIZE);
        cambuf = buf;
        buf = NULL;

        break;
    }
    case 1: {
        // 即時キャプチャ→hexダンプ
        DBGLOG0("capture");
        get_tim(&systim1);
        drvcam_capture(drvcam_callback, cambuf, DRVCAM_CAPTURE_BUFFER_SIZE);
        ercd = twai_flg(FLG_APLCON, FLGPTN_DRVCAM_CAPTURE_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 1000);
        assert(ercd == E_OK);
        get_tim(&systim2);
        DBGLOG1("capture ok (%dms)", systim2 - systim1);

        static int logcount = 0;
        char fname[32];
        sprintf(fname, "camcapture_%04d.raw", logcount);
        cmndbg_hexdump(cambuf, 640*480, fname);
        logcount++; 
        break;
    }
    case 2: {
        while (true) {
            // 生体センサでキャプチャ→バイナリダンプ(データ取得ツール用)
            // 注) このモードに入ると電源OFFまで抜けられない
            memset(cambuf, 0, DRVCAM_CAPTURE_BUFFER_SIZE);

//            ercd = clr_flg(FLG_APLCON, ~FLGPTN_DRVTS_RELEASE);
//            assert(ercd == E_OK);
            drvts_start(drvts_callback);
//            ercd = twai_flg(FLG_APLCON, FLGPTN_DRVTS_RELEASE, TWF_ANDW, &(FLGPTN){0}, -1);
//            assert(ercd == E_OK);

            ercd = clr_flg(FLG_APLCON, ~FLGPTN_DRVTS_TOUCH);
            ercd = twai_flg(FLG_APLCON, FLGPTN_DRVTS_TOUCH, TWF_ANDW, &(FLGPTN){0}, -1);
            assert(ercd == E_OK);
            drvts_stop();

            drvcam_capture(drvcam_callback, cambuf, DRVCAM_CAPTURE_BUFFER_SIZE);
            ercd = twai_flg(FLG_APLCON, FLGPTN_DRVCAM_CAPTURE_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 1000);
            assert(ercd == E_OK);

            // ダンプ
            dly_tsk(100);
            cmndbg_binframe(cambuf, 0, 640, 480);
        }
        break;
    }
    default:
        break;
    }

end:
    return 0;
}

/*
 * カメラドライバコールバック
 */
void drvcam_callback(int32_t type, int32_t error)
{
    switch (type) {
    case DRVCAM_CALLBACK_INITIALIZE:
        DBGLOG0("drvcam_callback: DRVCAM_CALLBACK_INITIALIZE");
        set_flg(FLG_APLCON, FLGPTN_DRVCAM_INITIALIZE_COMPLETE);
        break;
    case DRVCAM_CALLBACK_POWERON:
        DBGLOG0("drvcam_callback: DRVCAM_CALLBACK_POWERON");
        set_flg(FLG_APLCON, FLGPTN_DRVCAM_POWERON_COMPLETE);
        break;
    case DRVCAM_CALLBACK_POWEROFF:
        DBGLOG0("drvcam_callback: DRVCAM_CALLBACK_POWEROFF");
        set_flg(FLG_APLCON, FLGPTN_DRVCAM_POWEROFF_COMPLETE);
        break;
    case DRVCAM_CALLBACK_PREPARE:
        DBGLOG0("drvcam_callback: DRVCAM_CALLBACK_PREPARE");
        set_flg(FLG_APLCON, FLGPTN_DRVCAM_PREPARE_COMPLETE);
        break;
    case DRVCAM_CALLBACK_CAPTURE:
        DBGLOG0("drvcam_callback: DRVCAM_CALLBACK_CAPTURE");
        set_flg(FLG_APLCON, FLGPTN_DRVCAM_CAPTURE_COMPLETE);
        break;
    default:
        assert(false);
        break;
    }
} 

/*
 * タッチセンサーコールバック
 */
void drvts_callback(int event)
{
    DBGLOG1("drvts_callback: e=%d", event);
    switch(event) {
    case DRVTS_EVT_TOUCH:
        DBGLOG0("DRVTS_EVT_TOUCH");
        set_flg(FLG_APLCON, FLGPTN_DRVTS_TOUCH);
        break;
    case DRVTS_EVT_INTERMEDIATE:
        DBGLOG0("DRVTS_EVT_INTERMEDIATE");	// 中間状態: 何もしない
        break;
    case DRVTS_EVT_RELEASE:
        DBGLOG0("DRVTS_EVT_RELEASE");	// 離れた
        set_flg(FLG_APLCON, FLGPTN_DRVTS_RELEASE);
        break;
    default:
        assert(false);
        break;
    }
}
