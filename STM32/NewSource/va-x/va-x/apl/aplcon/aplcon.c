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

#include "aplicc.h"
#include "aplwifi.h"

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
#define LOG_NO_DATA                        0xFFFF
#define LOG_CNT_NO_DATA                    0xFFFFFFFFFFFFFFFF

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

static int cmdfunc_aplicc(const char* opt1, const char* opt2, const char* opt3);
static int cmdfunc_aplwifi(const char* opt1, const char* opt2, const char* opt3);
static int cmdfunc_mdlstrg(const char* opt1, const char* opt2, const char* opt3);
static int cmdfunc_authoffset(const char* opt1, const char* opt2);

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
static void base64dec(char *instr, char *outstr, int len);
static char charconv(char c);

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
    CMD_AUOFFSET,

    CMD_APLICC,
    CMD_APLWIFI,
    CMD_MDLSTRG,
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
    CMD_DEF_ITEM(CMD_AUOFFSET, "AUOFFSET", cmdfunc_authoffset, "AUOFFSET <val> <val>"),
    
    CMD_DEF_ITEM(CMD_APLICC, "APLICC", cmdfunc_aplicc, "APLICC <cmd>"),
    CMD_DEF_ITEM(CMD_APLWIFI, "APLWIFI", cmdfunc_aplwifi, "APLWIFI <cmd>"),
    CMD_DEF_ITEM(CMD_MDLSTRG, "MDLSTRG", cmdfunc_mdlstrg, "MDLSTRG <cmd>"),
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

static APLEVT_EXTRA_DATA_T aplevt_extra_data;
#define OPE_LOG_NUM 2
static MDLSTRG_DATA_OPE_LOG_T ope_log[OPE_LOG_NUM];
static MDLSTRG_DATA_ERR_LOG_T err_log;

// TODO テスト用仮データ（後日アプリ層に定義される変数に切り替える）
// #define MAX_VACANCY_CARD_NUM    20
// #define MAX_DROOM_CARD_NUM      10
static MDLSTRG_DATA_CARD_T test_vacancy_reg_card_data;
static MDLSTRG_DATA_CARD_T test_vacancy_card_data[MDLSTRG_MAX_VACANCY_CARD];
static MDLSTRG_DATA_CARD_T test_reg_card_data;
static MDLSTRG_DATA_DROOM_CARD_T test_droom_card_data[MDLSTRG_MAX_DROOM_CARD];
static MDLSTRG_DATA_IDENTIFY_CODE_MNG_T test_id_code_mng[10];
static MDLSTRG_DATA_IDCODE_T test_id_code_data = {0x7FFF,1,0,{0,0,0,0,0,0,0,0}};
static MDLSTRG_DATA_WIFI_PARAM_T test_wifi_param_data;

//static MDLSTRG_DATA_CARD_T test_card;
/////																				/////

// mdlstrg テストデータカウンター（登録）
static int mdlstrg_count_emp = 0;
static int mdlstrg_count_drm = 0;
static mdlstrg_mem_emp = 21;
//static mdlstrg_mem_drm = 10;

static MDLSTRG_DATA_ROOM_STS_T authu_value;

/////																				/////

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
	
	DBGLOG1("cmd_type: %d", cmd_type);

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
//drvllock_initialize(drvllock_callback_aplcn);
    }else if(index == 0x03){
        DBGLOG0("linkey_reset");
//reset_cmd(drvllock_callback_aplcn, test_buf, type);
    }else if(index == 0x04){
        DBGLOG0("linkey_open");
//open_command(drvllock_callback_aplcn, test_buf, type);
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
        status_mgnt_command();
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

// WiFi test関数
static DRVWIFI_CONFIG conf;
static uint8_t sid[] = {"ZMI_90D1"};
static uint8_t pp[] = {"11622752"};
static DRVWIFI_HTTPS_CONFIG hconf;
static uint8_t hnm[] = {"219.75.137.238"};
//static uint8_t hnm[] = {"v4.aqubio.jp"};
static uint8_t exhn[] = {"\0"};
static uint8_t exhv[] = {"\0"};
static DRVWIFI_HTTPS_REQ req;
static uint8_t bdy[] = {"log=BMQp2GCtw9/IUXT0S6SvmZcSzPZXoPCbC/rDctk+0jfQqccEYa5rFf4GqlFVxkUTNtsQOK5Ypsxgq2TPzPHnhA=="};
static uint8_t ph[] = {"/dapi/v1/device_log/add"};
static DRVWIFI_PING ping;
static uint8_t ipa1[] = {"192.168.21.1"};
static uint8_t ipa2[] = {"219.75.137.238"};
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
        DBGLOG0("Called drvwifi_initialize()");
        drvwifi_initialize(drvwifi_callback);
        break;
    case 1:
        DBGLOG0("Called drvwifi_start()");
        drvwifi_start(drvwifi_callback);
        break;
    case 2:
        DBGLOG0("Called drvwifi_power_off()");
        drvwifi_power_off(drvwifi_callback);
        break;
    case 3: {
        DBGLOG0("Called drvwifi_ap_connect()");
        // TODO : DRVWIFI_CONFIGに入力する値を決める
		
		conf.essid = sid;
		conf.essid_len = 8;		//文字数を設定すること
		conf.passphrase = pp;
		conf.passphrase_len = 8;		//文字数を設定すること
		conf.wpaver = DRVWIFI_SECURITY_WPA2RNS;	//1:WPA, 2:WPA2/RNS
		conf.cipher =DRVWIFI_SECURITY_CCMP;		//0:TKIP, 1:CCMP
        drvwifi_ap_connect(drvwifi_callback, &conf);
        break;
    }
    case 4:
        DBGLOG0("Called drvwifi_ap_connect_wps()");
        drvwifi_ap_connect_wps(drvwifi_callback);
        break;
    case 5:
        DBGLOG0("Called drvwifi_ap_disconnect()");
        drvwifi_ap_disconnect(drvwifi_callback);
        break;
    case 6: {
        DBGLOG0("Called drvwifi_https_connect()");
        // TODO : DRVWIFI_HTTPS_CONFIGに入力する値を決める

		hconf.hostname = hnm;
		hconf.hostname_len = 14;		//文字数を設定すること
//		hconf.hostname_len = 12;		//文字数を設定すること
		hconf.exhdrname = exhn;
		hconf.exhdrname_len = 0;		//文字数を設定すること
		hconf.exhdrvalue = exhv;
		hconf.exhdrval_len = 0;		//文字数を設定すること
		hconf.schema = DRVWIFI_SCHEMA_HTTP;	//0:http, 1:https
        drvwifi_https_connect(drvwifi_callback, &hconf);
        break;
    }
    case 7: {
        DBGLOG0("Called drvwifi_https_post()");
        // TODO : DRVWIFI_HTTPS_REQに入力する値を決める
	
		req.path = ph;
		req.path_len = 23;		//phの文字数を設定すること
		req.body = bdy;
		req.body_len = 92;		//bdyの文字数を設定すること
        drvwifi_https_post(drvwifi_callback, &req);
        break;
    }
    case 8:
        DBGLOG0("Called drvwifi_https_disconnect()");
        drvwifi_https_disconnect(drvwifi_callback);
        break;
    case 9: {
        DBGLOG0("Called drvwifi_ping() -> router");
		
		ping.ip_address = ipa1;    // ルータ
		ping.ipadr_len = 12;		//ipa1の文字数を設定すること
        drvwifi_ping(drvwifi_callback, &ping);
        break;
    }
    case 10: {
        DBGLOG0("Called drvwifi_ping() -> test server");
		
		ping.ip_address = ipa2;  // テストサーバ
		ping.ipadr_len = 14;		//ipa2の文字数を設定すること
        drvwifi_ping(drvwifi_callback, &ping);
        break;
    }
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


// 認証用オフセット値 入力
int cmdfunc_authoffset(const char* opt1, const char* opt2)
{
    DBGLOG0("CMD_AUOFFSET");
    ER er = E_OK;
    
    if (!opt1 || !opt2) {
        DBGLOG0("invalid option");
        goto end;
    }

    MDLSTRG_REQUEST_T ROOM_STS_REQ = {
        .data_type = MDLSTRG_DATA_TYPE_ROOM_TYPE,        // 
        .request_type = MDLSTRG_REQ_TYPE_READ,
        .data = (intptr_t)&authu_value,
    };
    clr_flg(FLG_APLWIFI, ~FLGPTN_MDLSTRG_REQ_COMPLETE);
    mdlstrg_request(&ROOM_STS_REQ, strg_callback);
    er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
    assert(er == E_OK);

    authu_value.ofs1_vlu = atoi(opt1);
    authu_value.ofs2_vlu = atoi(opt2);
    
    cmndbg_hexdump(&authu_value.cst_sts, sizeof(authu_value), "cst_sts_read");
    
    DBGLOG1("authu_value1: 0x%x", authu_value.ofs1_vlu);
    MDLSTRG_REQUEST_T ROOM_STS_WREQ = {
        .data_type = MDLSTRG_DATA_TYPE_ROOM_TYPE,
        .request_type = MDLSTRG_REQ_TYPE_WRITE,
        .data = (intptr_t)&authu_value,
        .size = sizeof(authu_value),
    };
    mdlstrg_request(&ROOM_STS_WREQ, strg_callback);
    er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 2000);
    
    assert(er == E_OK);
    
end:
    return 0;
}


// TODO: aplicc確認用に変更したので、必要であればこのファイル全体を最初のソースファイルに戻す
int cmdfunc_aplicc(const char* opt1, const char* opt2, const char* opt3)
{
    DBGLOG0("CMD_APLICC");

    if (!opt1) {
        DBGLOG0("invalid option");
        goto end;
    }

    int cmd = atoi(opt1);		// cmd
	APLEVT_EXTRA_DATA_T extra_data;

    switch (cmd) {
    case 0: // カード起動
        aplevt_send_event(APLEVT_ICC_START_REQ, 0, NULL, s_receiver_func);
        break;
    case 1: // カード制御中止
        aplevt_send_event(APLEVT_ICC_STOP_REQ, 0, NULL, s_receiver_func);
        break;
    case 2: // カード認証（D-ROOMカード）
        extra_data.reg_prep_req.index = APLICC_CARD_DROOM1;
        aplevt_send_event(APLEVT_ICC_AUTH_REQ, 0, &extra_data, s_receiver_func);
        break;
    case 3: // カード認証（空室カード）
        extra_data.reg_prep_req.index = APLICC_CARD_VACANCY;
        aplevt_send_event(APLEVT_ICC_AUTH_REQ, 0, &extra_data, s_receiver_func);
        break;
    case 4: // カード認証（工事カード）
        extra_data.reg_prep_req.index = APLICC_CARD_CONSTRUCTION;
        aplevt_send_event(APLEVT_ICC_AUTH_REQ, 0, &extra_data, s_receiver_func);
        break;
    case 5: // カード認証（登録・抹消カード）
        extra_data.reg_prep_req.index = APLICC_CARD_REGIST;
        extra_data.reg_prep_req.timeout = 30000;
        DBGLOG2("extra_data %d %d", extra_data.reg_prep_req.index, extra_data.reg_prep_req.timeout);
        aplevt_send_event(APLEVT_ICC_AUTH_REQ, 0, &extra_data, s_receiver_func);
        break;
    case 6: // カード登録（登録・抹消カード）
        extra_data.reg_prep_req.index = APLICC_CARD_REGIST;
        extra_data.reg_prep_req.timeout = 30000;
        DBGLOG2("extra_data %d %d", extra_data.reg_prep_req.index, extra_data.reg_prep_req.timeout);
        aplevt_send_event(APLEVT_ICC_REGIST_REQ, 0, &extra_data, s_receiver_func);
        break;
    case 7: // カード登録（D-roomカード）
        extra_data.reg_prep_req.index = APLICC_CARD_DROOM1;
        extra_data.reg_prep_req.timeout = 30000;
        extra_data.reg_prep_req.card_index = 1;
        memcpy(extra_data.reg_prep_req.user_name, "山田太郎", sizeof(extra_data.reg_prep_req.user_name));
        DBGLOG2("extra_data %d %d", extra_data.reg_prep_req.index, extra_data.reg_prep_req.timeout);
        DBGLOG2("extra_data %d %s", extra_data.reg_prep_req.card_index, extra_data.reg_prep_req.user_name);
        aplevt_send_event(APLEVT_ICC_REGIST_REQ, 0, &extra_data, s_receiver_func);
        break;
    case 8: // カード削除（D-roomカード）
        extra_data.reg_prep_req.card_index = 1;
        aplevt_send_event(APLEVT_ICC_DELETE_REQ, 0, &extra_data, s_receiver_func);
        break;
    case 9: // 退去
        aplevt_send_event(APLEVT_ICC_DEPARTURE_REQ, 0, NULL, s_receiver_func);
        break;
    case 10: // 空室カード登録
        extra_data.reg_prep_req.index = APLICC_CARD_VACANCY;
        extra_data.reg_prep_req.timeout = 30000;
        DBGLOG2("extra_data %d %d", extra_data.reg_prep_req.index, extra_data.reg_prep_req.timeout);
        aplevt_send_event(APLEVT_ICC_REGIST_REQ, 0, &extra_data, s_receiver_func);
        break;
    case 11: // 空室カード再登録
        extra_data.reg_prep_req.timeout = 30000;
        DBGLOG1("extra_data %d", extra_data.reg_prep_req.timeout);
        aplevt_send_event(APLEVT_ICC_REREGIST_REQ, 0, &extra_data, s_receiver_func);
        break;
        
    case 80: // カード登録（D-roomカード）２枚目(card_index=2)
        extra_data.reg_prep_req.index = APLICC_CARD_DROOM1;
        extra_data.reg_prep_req.timeout = 30000;
        extra_data.reg_prep_req.card_index = 2;
        memcpy(extra_data.reg_prep_req.user_name, "ABCDEFGHIJKL", sizeof(extra_data.reg_prep_req.user_name));
        DBGLOG2("extra_data %d %d", extra_data.reg_prep_req.index, extra_data.reg_prep_req.timeout);
        DBGLOG2("extra_data %d %s", extra_data.reg_prep_req.card_index, extra_data.reg_prep_req.user_name);
        aplevt_send_event(APLEVT_ICC_REGIST_REQ, 0, &extra_data, s_receiver_func);
        break;
    default:
        break;
    }

end:
    return 0;
}

static int cmdfunc_aplwifi(const char* opt1, const char* opt2, const char* opt3)
{
    DBGLOG0("CMD_APLWIFI");

    if (!opt1) {
        DBGLOG0("invalid option");
        goto end;
    }

    int cmd = atoi(opt1);		// cmd
	//APLEVT_EXTRA_DATA_T extra_data;

    switch (cmd) {
    case 0: // WIFI開始
        DBGLOG0("APLEVT_WIFI_START_REQ");
        aplevt_send_event(APLEVT_WIFI_START_REQ, 0, NULL, s_receiver_func);
        break;
    case 1: // サーバータスク確認
        DBGLOG0("APLEVT_WIFI_CHECK_SERVERTASK_REQ");
        aplevt_send_event(APLEVT_WIFI_CHECK_SERVERTASK_REQ, 0, NULL, s_receiver_func);
        break;
    case 2: { // エラーログアップロード
        DBGLOG0("APLEVT_WIFI_UPLOAD_ERR_LOG_REQ");
        // エラーログ構造体と電池残量を渡す必要がある
        for (int i = 0; i < 2; i++) {
            err_log.log_date[0].year = 2018;
            err_log.log_date[0].month = 10;
            err_log.log_date[0].day = 10;
            err_log.log_date[0].hour = 7;
            err_log.log_date[0].min = 30;
            err_log.log_date[0].dmy = 0;
        }

#if 1
        err_log.err[0] = 0x1001;            // メインタスク：電池残量エラー
        err_log.err[1] = 0x1011;            // 血流認証タスク：ハード接続エラー
#endif

#if 0
        err_log.err[0] = 0x1001;            // メインタスク：電池残量エラー
        err_log.err[1] = 0x1011;            // 血流認証タスク：ハード接続エラー
        err_log.err[2] = 0x1012;            // 血流認証タスク：カメラ異常
        err_log.err[3] = 0x1013;            // 血流認証タスク：撮影異常
        err_log.err[4] = 0x1014;            // 血流認証タスク：指エッジエラー
        err_log.err[5] = 0x1021;            // 錠制御タスク：施錠解錠異常
        err_log.err[6] = 0x1022;            // 錠制御タスク：扉過開放
        err_log.err[7] = 0x1023;            // 錠制御タスク：扉こじ開け
        err_log.err[8] = 0x1024;            // 錠制御タスク：電源電圧異常
        err_log.err[9] = 0x1031;            // 電源制御タスク：スタンバイ遷移異常
        err_log.err[10] = 0x1032;           // 電源制御タスク：電源起動異常
        err_log.err[11] = 0x1033;           // 電源制御タスク：電源電圧異常
#endif

#if 0
        err_log.err[0]  = 0x1051;            // BLE制御タスク：ハード接続エラー
        err_log.err[1]  = 0x1052;            // BLE制御タスク：BLE通信タイムアウト
        err_log.err[2]  = 0x1053;            // BLE制御タスク：BLE通信異常
        err_log.err[3]  = 0x1054;            // BLE制御タスク：接続スマホ違反
        err_log.err[4]  = 0x1061;            // データ管理タスク：フラッシュリードエラー
        err_log.err[5]  = 0x1062;            // データ管理タスク：フラッシュライトエラー
        err_log.err[6]  = 0x1071;            // カード制御タスク：ハード接続エラー
        err_log.err[7]  = 0x1072;            // カード制御タスク：カードリードエラー
        err_log.err[8]  = 0x1073;            // カード制御タスク：カードライトエラー
        err_log.err[9]  = 0x1074;            // カード制御タスク：1TimePassword認証エラー
        err_log.err[10] = 0x1081;            // WiFi制御タスク：ハード接続エラー
#endif

        aplevt_extra_data.wifi_data.err_log = &err_log;
        aplevt_extra_data.wifi_data.battery = 2;

        aplevt_send_event(APLEVT_WIFI_UPLOAD_ERR_LOG_REQ, 0, &aplevt_extra_data, s_receiver_func);
        break;
    }
    case 3: { // 操作ログアップロード
        DBGLOG0("APLEVT_WIFI_UPLOAD_OPE_LOG_REQ");

        // １つ目：血流認証成功
        for (int i = 0; i < 2; i++) {
            ope_log[0].log_date[i].year = 2018;
            ope_log[0].log_date[i].month = 12;
            ope_log[0].log_date[i].day = 1;
            ope_log[0].log_date[i].hour = 12;
            ope_log[0].log_date[i].min = 10;
            ope_log[0].log_date[i].dmy = 0;
        }
        ope_log[0].log[0].state = 1;
        ope_log[0].log[0].processing = 1;
        ope_log[0].log[0].detail01 = 0;
        ope_log[0].log[0].detail02 = 0;
        ope_log[0].log[0].detail03 = 0;
        ope_log[0].log[1].state = 2;
        ope_log[0].log[1].processing = 1;
        ope_log[0].log[1].detail01 = 0;
        ope_log[0].log[1].detail02 = 0;
        ope_log[0].log[1].detail03 = 0;
        ope_log[0].log[2].state = 3;
        ope_log[0].log[2].processing = 1;
        ope_log[0].log[2].detail01 = 0;
        ope_log[0].log[2].detail02 = 2;
        ope_log[0].log[2].detail03 = 0x02000003;
        ope_log[0].log[3].state = 4;
        ope_log[0].log[3].processing = 0;
        ope_log[0].log[3].detail01 = 0;
        ope_log[0].log[3].detail02 = 0;
        ope_log[0].log[3].detail03 = 0;

        // ２つ目：カード認証成功
        for (int i = 0; i < 2; i++) {
            ope_log[1].log_date[i].year = 2018;
            ope_log[1].log_date[i].month = 12;
            ope_log[1].log_date[i].day = 1;
            ope_log[1].log_date[i].hour = 12;
            ope_log[1].log_date[i].min = 10;
            ope_log[1].log_date[i].dmy = 0;
        }
        ope_log[1].log[0].state = 1;
        ope_log[1].log[0].processing = 1;
        ope_log[1].log[0].detail01 = 0;
        ope_log[1].log[0].detail02 = 0;
        ope_log[1].log[0].detail03 = 0;
        ope_log[1].log[1].state = 2;
        ope_log[1].log[1].processing = 1;
        ope_log[1].log[1].detail01 = 0;
        ope_log[1].log[1].detail02 = 0;
        ope_log[1].log[1].detail03 = 0;
        ope_log[1].log[2].state = 3;
        ope_log[1].log[2].processing = 2;
        ope_log[1].log[2].detail01 = 0;
        ope_log[1].log[2].detail02 = 2;
        ope_log[1].log[2].detail03 = 0x0A000000;
        ope_log[1].log[3].state = 4;
        ope_log[1].log[3].processing = 0;
        ope_log[1].log[3].detail01 = 0;
        ope_log[1].log[3].detail02 = 0;
        ope_log[1].log[3].detail03 = 0;

        // 操作ログ構造体の配列とデータ数、電池残量を渡す
        aplevt_extra_data.wifi_data.ope_log_array = (void*)ope_log;
        aplevt_extra_data.wifi_data.ope_log_num = 2;
        //aplevt_extra_data.wifi_data.ope_log_num = OPE_LOG_NUM;
        aplevt_extra_data.wifi_data.battery = 2;
        aplevt_send_event(APLEVT_WIFI_UPLOAD_OPE_LOG_REQ, 0, &aplevt_extra_data, s_receiver_func);
        break;
    }
    case 4: // 空室カードダウンロード
        // 本来は内部機能のため、アプリ間イベントでは動作しない
        DBGLOG0("APLEVT_WIFI_DOWNLOAD_VACANCY_CARD_REQ");
        aplevt_send_event(APLEVT_WIFI_DOWNLOAD_VACANCY_CARD_REQ, 0, NULL, s_receiver_func);
        break;
    case 5: // WiFi制御中止
        DBGLOG0("APLEVT_WIFI_STOP_REQ");
        aplevt_send_event(APLEVT_WIFI_STOP_REQ, 0, NULL, s_receiver_func);
        break;
    case 6: { // 操作ログアップロード（opt2の値により、送信電文を切替）
        int index = 0;

        DBGLOG0("APLEVT_WIFI_UPLOAD_OPE_LOG_REQ(index)");
        if (!opt2) {
            DBGLOG0("Error : opt2 is empty.");
            break;
        }

        index = atoi(opt2);

        for (int i = 0; i < 2; i++) {
            ope_log[0].log_date[i].year = 2018;
            ope_log[0].log_date[i].month = 12;
            ope_log[0].log_date[i].day = 1;
            ope_log[0].log_date[i].hour = 12;
            ope_log[0].log_date[i].min = 10;
            ope_log[0].log_date[i].dmy = 0;
        }
        ope_log[0].log[0].state = 1;
        ope_log[0].log[0].processing = 1;
        ope_log[0].log[0].detail01 = 0;
        ope_log[0].log[0].detail02 = 0;
        ope_log[0].log[0].detail03 = 0;
        ope_log[0].log[1].state = 2;
        ope_log[0].log[1].processing = 1;
        ope_log[0].log[1].detail01 = 0;
        ope_log[0].log[1].detail02 = 0;
        ope_log[0].log[1].detail03 = 0;

        if (index == 0) {           // 電源関係：起動のみ
            ope_log[0].log[2].state = 3;
            ope_log[0].log[2].processing = 0;
            ope_log[0].log[2].detail01 = 0;
            ope_log[0].log[2].detail02 = 0;
            ope_log[0].log[2].detail03 = 0x00000000;
            ope_log[0].log[3].state = 4;
            ope_log[0].log[3].processing = 0;
            ope_log[0].log[3].detail01 = 0;
            ope_log[0].log[3].detail02 = 0;
            ope_log[0].log[3].detail03 = 0;
        } else if (index == 1) {    // 認証関係：血流認証成功
            ope_log[0].log[2].state = 3;
            ope_log[0].log[2].processing = 1;
            ope_log[0].log[2].detail01 = 0;
            ope_log[0].log[2].detail02 = 2;
            ope_log[0].log[2].detail03 = 0x02000003;
            ope_log[0].log[3].state = 4;
        } else if (index == 2) {    // 認証関係：血流認証失敗
            ope_log[0].log[2].state = 3;
            ope_log[0].log[2].processing = 1;
            ope_log[0].log[2].detail01 = 0;
            ope_log[0].log[2].detail02 = 1;
            ope_log[0].log[2].detail03 = 0x00000007;
            ope_log[0].log[3].state = 4;
        } else if (index == 3) {    // 認証関係：カード認証成功
            ope_log[0].log[2].state = 3;
            ope_log[0].log[2].processing = 2;
            ope_log[0].log[2].detail01 = 0;
            ope_log[0].log[2].detail02 = 2;
            ope_log[0].log[2].detail03 = 0x04000000;
            ope_log[0].log[3].state = 4;
        } else if (index == 4) {    // 認証関係：カード認証失敗
            ope_log[0].log[2].state = 3;
            ope_log[0].log[2].processing = 2;
            ope_log[0].log[2].detail01 = 0;
            ope_log[0].log[2].detail02 = 1;
            ope_log[0].log[2].detail03 = 0x00000000;
            ope_log[0].log[3].state = 4;
        } else if (index == 5) {    // 電気錠関係：電気錠解錠
            ope_log[0].log[2].state = 3;
            ope_log[0].log[2].processing = 0;
            ope_log[0].log[3].state = 4;
            ope_log[0].log[3].processing = 1;
            ope_log[0].log[3].detail01 = 3;
            ope_log[0].log[3].detail02 = 2;
            ope_log[0].log[3].detail03 = 0x00000003;
        } else if (index == 6) {    // スマホ関係：初期接続
            ope_log[0].log[2].state = 3;
            ope_log[0].log[2].processing = 3;
            ope_log[0].log[2].detail01 = 0x10;
            ope_log[0].log[2].detail02 = 0x02;
            ope_log[0].log[2].detail03 = 0x06000000;
            ope_log[0].log[3].state = 4;
        } else if (index == 7) {    // スマホ関係：血流登録
            ope_log[0].log[2].state = 3;
            ope_log[0].log[2].processing = 3;
            ope_log[0].log[2].detail01 = 0x20;
            ope_log[0].log[2].detail02 = 0x02;
            ope_log[0].log[2].detail03 = 0x07000000;
            ope_log[0].log[3].state = 4;
        } else if (index == 8) {    // スマホ関係：血流削除
            ope_log[0].log[2].state = 3;
            ope_log[0].log[2].processing = 3;
            ope_log[0].log[2].detail01 = 0x21;
            ope_log[0].log[2].detail02 = 0x02;
            ope_log[0].log[2].detail03 = 0x08000000;
            ope_log[0].log[3].state = 4;
        } else if (index == 9) {    // スマホ関係：カード登録
            ope_log[0].log[2].state = 3;
            ope_log[0].log[2].processing = 3;
            ope_log[0].log[2].detail01 = 0x30;
            ope_log[0].log[2].detail02 = 0x02;
            ope_log[0].log[2].detail03 = 0x09000000;
            ope_log[0].log[3].state = 4;
        } else if (index == 10) {    // スマホ関係：カード削除
            ope_log[0].log[2].state = 3;
            ope_log[0].log[2].processing = 3;
            ope_log[0].log[2].detail01 = 0x31;
            ope_log[0].log[2].detail02 = 0x02;
            ope_log[0].log[2].detail03 = 0x0a000000;
            ope_log[0].log[3].state = 4;
        } else if (index == 11) {    // スマホ関係：ユーザー削除
            ope_log[0].log[2].state = 3;
            ope_log[0].log[2].processing = 3;
            ope_log[0].log[2].detail01 = 0x41;
            ope_log[0].log[2].detail02 = 0x02;
            ope_log[0].log[2].detail03 = 0x0b000000;
            ope_log[0].log[3].state = 4;
        } else if (index == 12) {    // スマホ関係：緊急解錠（管理者）
            ope_log[0].log[2].state = 3;
            ope_log[0].log[2].processing = 3;
            ope_log[0].log[2].detail01 = 0x50;
            ope_log[0].log[2].detail02 = 0x02;
            ope_log[0].log[2].detail03 = 0x0c000000;
            ope_log[0].log[3].state = 4;
        } else if (index == 13) {    // スマホ関係：緊急解錠（一般者）
            ope_log[0].log[2].state = 3;
            ope_log[0].log[2].processing = 3;
            ope_log[0].log[2].detail01 = 0x51;
            ope_log[0].log[2].detail02 = 0x02;
            ope_log[0].log[2].detail03 = 0x0d000000;
            ope_log[0].log[3].state = 4;
        } else if (index == 14) {    // スマホ関係：AQUBIO機器設定
            ope_log[0].log[2].state = 3;
            ope_log[0].log[2].processing = 3;
            ope_log[0].log[2].detail01 = 0x61;
            ope_log[0].log[2].detail02 = 0x02;
            ope_log[0].log[2].detail03 = 0x0e000000;
            ope_log[0].log[3].state = 4;
        } else if (index == 15) {    // スマホ関係：代理スマホ接続
            ope_log[0].log[2].state = 3;
            ope_log[0].log[2].processing = 3;
            ope_log[0].log[2].detail01 = 0xF0;
            ope_log[0].log[2].detail02 = 0x02;
            ope_log[0].log[2].detail03 = 0x0f000000;
            ope_log[0].log[3].state = 4;
        } else {
            DBGLOG1("undefined index : %d ", index);
        }

        // 操作ログ構造体の配列とデータ数、電池残量を渡す
        aplevt_extra_data.wifi_data.ope_log_array = (void*)ope_log;
        aplevt_extra_data.wifi_data.ope_log_num = 1;
        aplevt_extra_data.wifi_data.battery = 1;
        aplevt_send_event(APLEVT_WIFI_UPLOAD_OPE_LOG_REQ, 0, &aplevt_extra_data, s_receiver_func);

        break;
    }
    case 7: // base64文字列をデコード
        DBGLOG0("APLEVT_WIFI_BASE64_DECODE");
        const char *instr = "QUJDREVGRw==";
        static char  outstr[9] = {0};
        base64dec(instr, outstr, 12);
        cmndbg_hexdump(outstr, 9, "APLEVT_WIFI_BASE64_DECODE dump outstr");
        break;
    default:
        break;
    }

end:
    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////
/*
 * base64の文字列 instr をデコードしてバイナリ値に変換し outstr に格納
 * lenはbase64の文字数(4の倍数)
 * 
 * outstrの配列の大きさは (len÷4)×3
 * 
 */
static void base64dec(char *instr, char *outstr, int len)
{
    int		i, iR;
    char	a1, a2, a3, a4;

    i = 0;
    iR = 0;
    while (1) {
        if (i >= len)
            break;
        a1 = charconv(instr[i]);
        a2 = charconv(instr[i+1]);
        a3 = charconv(instr[i+2]);
        a4 = charconv(instr[i+3]);
        outstr[iR] = (a1 << 2) + (a2 >>4);
        outstr[iR + 1] = (a2 << 4) + (a3 >>2);
        outstr[iR + 2] = (a3 << 6) + a4;
        i += 4;
        iR += 3;
    }
    return;
}

/*
 * base64の1文字を6bitの値に変換する
 */
static char charconv(char c)
{
    if (c >= 'A' && c <= 'Z')
        return (c - 'A');
    if (c >= 'a' && c <= 'z')
        return (c - 'a' + 0x1a);
    if (c >= '0' && c <= '9')
        return (c - '0' + 0x34);
    if (c == '+')
        return 0x3e;
    if (c == '/')
        return 0x3f;
    if (c == '=')
        return '\0';

	return '\0';
}
/////////////////////////////////////////////////////////////////////////////////////////////


// mdlstrg動作確認用
int cmdfunc_mdlstrg(const char* opt1, const char* opt2, const char* opt3)
{
    DBGLOG0("CMD_MDLSTRG");

    if (!opt1) {
        DBGLOG0("invalid option");
        goto end;
    }

    int cmd = atoi(opt1);		// cmd
	APLEVT_EXTRA_DATA_T extra_data;

    ER er = E_OK;
    bool_t exists = false;
    bool_t result = false;

    switch (cmd) {
    case 0: // デバッグ用（特定部分の読み出し）
        // opt2 : address

        DBGLOG0("NO IMPLEMENT");

        

        /*
        // 読み出し
        MDLSTRG_REQUEST_T FRESET_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_AUTH,
            .request_type = MDLSTRG_REQ_TYPE_DELETE,
            .opt1 = index,
        };
        mdlstrg_request(&FRESET_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        */
        break;
    case 1: { // 空室カードデータ取得（20枚分のデータを一括読込）
        DBGLOG0("VACANCY_CARD_READ_REQ");
        MDLSTRG_REQUEST_T VACANCY_CARD_READ_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_VACANCY_CARD,
            .request_type = MDLSTRG_REQ_TYPE_READ,
            .data = &test_vacancy_card_data,
        };
        mdlstrg_request(&VACANCY_CARD_READ_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);

        for (int i = 0; i < MDLSTRG_MAX_VACANCY_CARD; i++) {
            DBGLOG1("VACANCY_CARD index = %d", i);
            cmndbg_hexdump(test_vacancy_card_data[i].status_flg, sizeof(test_vacancy_card_data[i].status_flg), "Flg");
            cmndbg_hexdump(test_vacancy_card_data[i].idm, sizeof(test_vacancy_card_data[i].idm), "IDm");
        }

        break;
    }
    case 2: { // 空室カード保存（追加）
        DBGLOG0("VACANCY_CARD_APPEND_REQ");
        // TODO テストデータ
        MDLSTRG_DATA_CARD_T test_card;
        test_card.status_flg[0] = 0x7F;
        for (int i = 1; i < 8; i++) {
            test_card.status_flg[i] = 0xFF;
        }
		/////																				/////
		int indexlp = 0;

		if (!opt2) {// 保存
			indexlp = 1;
		}else {// 連続保存
			indexlp = atoi(opt2);		// 繰り返し回数
		}
		if (indexlp >= mdlstrg_mem_emp && indexlp <= 0) {// 範囲外(21~/~0)
			DBGLOG0("KEYINPUT_OPTION 2_BETWEEN_1~20");
			goto end;
		}

		mdlstrg_count_emp = mdlstrg_count_emp % mdlstrg_mem_emp;// 20以内+1（1~21）
		mdlstrg_count_emp++;

		uint8_t idm[8] = { (0x00 + (mdlstrg_count_emp)), 0x14, 0xa4, 0x8a, 0x50, 0x03, 0x93, (0x49 + (mdlstrg_count_emp * 2)) };// (0x01~0x14<+1>, 0x4b~0x71<+2>)
		uint8_t card_type[16] = { 0x30, 0x34, 0x31, 0x38, 0x30, 0x39, 0x30, 0x39, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00 };

//        uint8_t idm[8] = {0x01,0x14,0xc4,0xab,0x53,0x11,0x01,0x04}; // 040002916
//        uint8_t card_type[16] = {0x30, 0x34, 0x31, 0x32, 0x30, 0x33, 0x31, 0x32, 0x30, 0x30, 0x30, 0x32, 0x39, 0x31, 0x31, 0x30}; // 040002916


		uint8_t member_num[16] = { 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20 };
		/////																				/////
        //uint8_t idm[8] = {0x01,0x14,0xa4,0x8a,0x50,0x03,0x93,0x49};
        //uint8_t card_type[16] = {0x30, 0x34, 0x31, 0x38, 0x30, 0x39, 0x30, 0x39, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00};
        //uint8_t member_num[16] = {0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20};
        memcpy(test_card.idm, idm, 8);
        memcpy(test_card.card_type, card_type, 16);
        memcpy(test_card.member_num, member_num, 16);

        MDLSTRG_REQUEST_T VACANCY_CARD_APPEND_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_VACANCY_CARD,        // 空室カード
            .request_type = MDLSTRG_REQ_TYPE_APPEND,
            .data = &test_card,
        };
        mdlstrg_request(&VACANCY_CARD_APPEND_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        result = s_strgmdl_req_state;

        if (result != E_OK) {
            DBGLOG0("REG_CARD_WRITE_REQ : Error / Verify Error or Max num Error");
        }
		/////																				/////
		for (int lpi = 1; lpi < indexlp; lpi++) {
			mdlstrg_count_emp = mdlstrg_count_emp % mdlstrg_mem_emp;// 20以内+1（1~21）
			mdlstrg_count_emp++;

			uint8_t idm[8] = { (0x00 + (mdlstrg_count_emp)), 0x14, 0xa4, 0x8a, 0x50, 0x03, 0x93, (0x49 + (mdlstrg_count_emp * 2)) };// (0x01~0x14<+1>, 0x4b~0x71<+2>)
			uint8_t card_type[16] = { 0x30, 0x34, 0x31, 0x38, 0x30, 0x39, 0x30, 0x39, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00 };
			uint8_t member_num[16] = { 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20 };

			memcpy(test_card.idm, idm, 8);
			memcpy(test_card.card_type, card_type, 16);
			memcpy(test_card.member_num, member_num, 16);

			MDLSTRG_REQUEST_T VACANCY_CARD_APPEND_REQ = {
				.data_type = MDLSTRG_DATA_TYPE_VACANCY_CARD,        // 空室カード
				.request_type = MDLSTRG_REQ_TYPE_APPEND,
				.data = &test_card,
			};
			mdlstrg_request(&VACANCY_CARD_APPEND_REQ, strg_callback);
			er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
			assert(er == E_OK);
			result = s_strgmdl_req_state;

			if (result != E_OK) {
				DBGLOG0("REG_CARD_WRITE_REQ : Error / Verify Error or Max num Error");
			}
		}
		/////																				/////
/*
        cmndbg_hexdump(test_reg_card_data.status_flg, sizeof(test_reg_card_data.status_flg), "Status");
        cmndbg_hexdump(test_reg_card_data.idm, sizeof(test_reg_card_data.idm), "IDm");
        cmndbg_hexdump(test_reg_card_data.card_type, sizeof(test_reg_card_data.card_type), "Card Type");
        cmndbg_hexdump(test_reg_card_data.member_num, sizeof(test_reg_card_data.member_num), "Member Num");
*/
        break;
    }
    case 3: { // 空室カード初期化
        // 空室カード領域を一括削除する
        DBGLOG0("VACENCY_CARD_ALL_DELETE_REQ");
        MDLSTRG_REQUEST_T VACENCY_CARD_ALL_DELETE_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_VACANCY_CARD,
            .request_type = MDLSTRG_REQ_TYPE_ALL_DELETE,
        };
        mdlstrg_request(&VACENCY_CARD_ALL_DELETE_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
		/////																				/////
		mdlstrg_count_emp = 0;//	リセット
		/////																				/////
        break;
    }
    case 4: // D-roomカードデータ取得
        DBGLOG0("DROOM_CARD_READ_REQ");
        MDLSTRG_REQUEST_T DROOM_CARD_READ_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_DROOM_CARD,
            .request_type = MDLSTRG_REQ_TYPE_READ,
            .data = &test_droom_card_data,
        };
        mdlstrg_request(&DROOM_CARD_READ_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);

        for (int i = 0; i < MDLSTRG_MAX_DROOM_CARD; i++) {
            DBGLOG1("DROOM_CARD index = %d", i);
            cmndbg_hexdump(test_droom_card_data[i].status_flg, sizeof(test_droom_card_data[i].status_flg), "Flg");
            cmndbg_hexdump(test_droom_card_data[i].idm, sizeof(test_droom_card_data[i].idm), "IDm");
            cmndbg_hexdump(test_droom_card_data[i].card_type, sizeof(test_droom_card_data[i].card_type), "Card_Type");
            cmndbg_hexdump(test_droom_card_data[i].member_num, sizeof(test_droom_card_data[i].member_num), "Mem_Num");
            cmndbg_hexdump(test_droom_card_data[i].user_name, sizeof(test_droom_card_data[i].user_name), "UserName");
        }

        break;
    case 5: { // D-roomカード保存
        DBGLOG0("DROOM_CARD_WRITE_REQ");

        if (!opt2) {
            DBGLOG0("invalid option 2 (for index)");
            goto end;
        }

        int index = atoi(opt2);		// cmd
        DBGLOG1("index = %d", index);

        // テストデータ
        MDLSTRG_DATA_DROOM_CARD_T test_droom_card;
        test_droom_card.status_flg[0] = 0x7F;
        for (int i = 1; i < 8; i++) {
            test_droom_card.status_flg[i] = 0xFF;
        }
		/////																				/////

		mdlstrg_count_drm++;

		uint8_t idm[8] = {(0x01 + (index)), 0x14, 0xa4, 0x8a, 0x50, 0x03, 0x93, (0x00 + (mdlstrg_count_drm))};
		uint8_t card_type[16] = {0x30, 0x31, 0x31, 0x38, 0x30, 0x39, 0x30, 0x39, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00};
		uint8_t member_num[16] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x20, 0x20, 0x20};

		/////																				/////
        //uint8_t idm[8] = {0x01,0x14,0xa4,0x8a,0x50,0x03,0x93,0x49};
        ////uint8_t idm[8] = {0x21,0x14,0xa4,0x8a,0x50,0x03,0x93,0x49};   // 書き換え確認用
        //uint8_t card_type[16] = {0x30, 0x31, 0x31, 0x38, 0x30, 0x39, 0x30, 0x39, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00};
        //uint8_t member_num[16] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x2f, 0x20};
        for (int i = 0; i < 128; i++) {
            test_droom_card.user_name[i] = i;
        }
        memcpy(test_droom_card.idm, idm, 8);
        memcpy(test_droom_card.card_type, card_type, 16);
        memcpy(test_droom_card.member_num, member_num, 16);

        MDLSTRG_REQUEST_T DROOM_CARD_WRITE_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_DROOM_CARD,
            .request_type = MDLSTRG_REQ_TYPE_WRITE,
            .data = &test_droom_card,
            .opt1 = index,
        };
        mdlstrg_request(&DROOM_CARD_WRITE_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        break;
    }
    case 6: { // D-roomカード削除
        DBGLOG0("DROOM_CARD_DELETE_REQ");

        if (!opt2) {
            DBGLOG0("invalid option 2 (for index)");
            goto end;
        }

        int index = atoi(opt2);		// cmd
        DBGLOG1("index = %d", index);

        MDLSTRG_REQUEST_T DROOM_CARD_DELETE_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_DROOM_CARD,
            .request_type = MDLSTRG_REQ_TYPE_DELETE,
            .opt1 = index,
        };
        mdlstrg_request(&DROOM_CARD_DELETE_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);

        break;
    }
    case 7: { // D-roomカード初期化
        DBGLOG0("DROOM_CARD_ALL_DELETE_REQ");
        MDLSTRG_REQUEST_T DROOM_CARD_ALL_DELETE_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_DROOM_CARD,
            .request_type = MDLSTRG_REQ_TYPE_ALL_DELETE,
        };
        mdlstrg_request(&DROOM_CARD_ALL_DELETE_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
		/////																				/////
		mdlstrg_count_emp = 0;//	リセット
		/////																				/////
        break;
    }
    case 8: {// 識別コード管理データ作成
        DBGLOG0("CREATE_ID_CODE_MNG_DATA_REQ");
        MDLSTRG_REQUEST_T CREATE_ID_CODE_MNG_DATA_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_IDCODE,        // 識別コード(D-roomカード)
            .request_type = MDLSTRG_REQ_TYPE_READ,
            .data = &test_id_code_mng,
        };
        mdlstrg_request(&CREATE_ID_CODE_MNG_DATA_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);


        DBGLOG0("CREATE_ID_CODE_MNG_DATA_REQ finish");

    
    	for(int i=0; i < 10; i++){
        /*
            cmndbg_hexdump(test_id_code_mng[i].id_code.flg, sizeof(test_id_code_mng[i].id_code.flg), "Flg");
            cmndbg_hexdump(test_id_code_mng[i].id_code.idx, sizeof(test_id_code_mng[i].id_code.idx), "INDEX");
            cmndbg_hexdump(test_id_code_mng[i].id_code.block, sizeof(test_id_code_mng[i].id_code.block), "Block");
            cmndbg_hexdump(test_id_code_mng[i].id_code.code, sizeof(test_id_code_mng[i].id_code.code), "ID Code");
            cmndbg_hexdump(test_id_code_mng[i].address, sizeof(test_id_code_mng[i].address), "address");
        */
            DBGLOG1("-- INDEX %d ---------------------------------------------", i);
            DBGLOG1("FLG 0x%04x", test_id_code_mng[i].id_code.flg);
            DBGLOG1("IDX 0x%04x", test_id_code_mng[i].id_code.idx);
            DBGLOG1("BLK 0x%08x", test_id_code_mng[i].id_code.block);
            //DBGLOG1("CODE 0x%08x", test_id_code_mng[i].id_code.code);
            cmndbg_hexdump(test_id_code_mng[i].id_code.code, sizeof(uint8_t)*8, "ID Code");
            DBGLOG1("ADD 0x%08x", test_id_code_mng[i].address);
    	}
        break;
    }
    case 9: {// 識別コード書き込み
        DBGLOG0("ID_CODE_WRITE_REQ");

        // TODO テストデータ
        //test_id_code_data.flg = 0x7FFF;
    	//test_id_code_data.idx = 1;
    	//test_id_code_data.block = 0;
        static uint8_t code[8] = {0x01,0x24,0xc4,0x11,0x70,0x23,0xb3,0x01};

        MDLSTRG_REQUEST_T ID_CODE_WRITE_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_IDCODE,        // 識別コード(D-roomカード)
            .request_type = MDLSTRG_REQ_TYPE_WRITE,
            .data = &test_id_code_data,
        };

        // 引数(opt2)で指定された書き込み数を保存
        int loopCntMax;
        if (!opt2) {
            loopCntMax = 1;
        }else{
            loopCntMax = atoi(opt2);		// cmd
        }
        DBGLOG1("loopCntMax = %d", loopCntMax);

        // 連続して書き込む
        int loopCnt = 0;
        for(loopCnt=0; loopCnt < loopCntMax; loopCnt++){
            printf("=== Loop Count %d ===\n", loopCnt + 1);
        	
            mdlstrg_request(&ID_CODE_WRITE_REQ, strg_callback);
            er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
            assert(er == E_OK);
            result = s_strgmdl_req_state;
    
            if (result != E_OK) {
                DBGLOG0("ID_CODE_WRITE_REQ : Verify Error");
            }
            
            test_id_code_data.idx++;
            //test_id_code_data.idx++;
            if (test_id_code_data.idx > 10) {
                test_id_code_data.idx = 1;
            }
            (code[0])++;
            (code[7])++;
            memcpy(test_id_code_data.code, code, 8);
        }
        break;
    }
    case 10: {// 識別コード削除
        DBGLOG0("ID_CODE_ALL_DELETE_REQ");
        MDLSTRG_REQUEST_T ID_CODE_ALL_DELETE_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_IDCODE,
            .request_type = MDLSTRG_REQ_TYPE_ALL_DELETE,
        };
        mdlstrg_request(&ID_CODE_ALL_DELETE_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        break;
    }
    case 23: {// 識別コード初期化
        DBGLOG0("ID_CODE_INIT_REQ");
        MDLSTRG_REQUEST_T ID_CODE_INIT_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_IDCODE,
            .request_type = MDLSTRG_REQ_TYPE_CREATE,
        };
        mdlstrg_request(&ID_CODE_INIT_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);

        result = s_strgmdl_req_state;

        if (result != E_OK) {
            DBGLOG0("ID_CODE_INIT_REQ : Verify Error (Write error)");
        }
        break;
    }
    case 11: { // 登録・抹消カードデータ存在確認
        DBGLOG0("REG_CARD_EXIST_REQ");
        MDLSTRG_REQUEST_T REG_CARD_EXIST_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_REG_CARD,              // 登録・抹消カード
            .request_type = MDLSTRG_REQ_TYPE_EXISTS,
        };
        mdlstrg_request(&REG_CARD_EXIST_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        exists = s_strgmdl_req_state;

        DBGLOG1("REG_CARD exists: %d", exists);

        break;
    }
    case 12: { // 登録・抹消カードデータ取得
        DBGLOG0("REG_CARD_READ_REQ");
        MDLSTRG_REQUEST_T REG_CARD_READ_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_REG_CARD,        // 登録・抹消カードデータ
            .request_type = MDLSTRG_REQ_TYPE_READ,
            .data = &test_reg_card_data,
        };
        mdlstrg_request(&REG_CARD_READ_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        exists = s_strgmdl_req_state;

        DBGLOG1("REG_CARD_READ_REQ exists: %d", exists);
        if (exists == true) {
            cmndbg_hexdump(test_reg_card_data.idm, sizeof(test_reg_card_data.idm), "IDm");
            cmndbg_hexdump(test_reg_card_data.card_type, sizeof(test_reg_card_data.card_type), "Card Type");
            cmndbg_hexdump(test_reg_card_data.member_num, sizeof(test_reg_card_data.member_num), "Member Num");
        }

        break;
    }
    case 13: {// 登録・抹消カード保存
        DBGLOG0("REG_CARD_WRITE_REQ");

        // TODO テストデータ
        test_reg_card_data.status_flg[0] = 0x7F;
        for (int i = 1; i < 8; i++) {
            test_reg_card_data.status_flg[i] = 0xFF;
        }

        uint8_t idm[8] = {0x11,0x24,0xc4,0xba,0x70,0x23,0xb3,0x69};
        uint8_t card_type[16] = {0x30, 0x33, 0x31, 0x38, 0x30, 0x39, 0x30, 0x39, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00};
        uint8_t member_num[16] = {0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20};

        memcpy(test_reg_card_data.idm, idm, 8);
        memcpy(test_reg_card_data.card_type, card_type, 16);
        memcpy(test_reg_card_data.member_num, member_num, 16);

        MDLSTRG_REQUEST_T REG_CARD_WRITE_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_REG_CARD,        // 空室一括登録カード
            .request_type = MDLSTRG_REQ_TYPE_WRITE,
            .data = &test_reg_card_data,
        };
        mdlstrg_request(&REG_CARD_WRITE_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        result = s_strgmdl_req_state;

        if (result != E_OK) {
            DBGLOG0("REG_CARD_WRITE_REQ : Verify Error");
        }

        cmndbg_hexdump(test_reg_card_data.status_flg, sizeof(test_reg_card_data.status_flg), "Status");
        cmndbg_hexdump(test_reg_card_data.idm, sizeof(test_reg_card_data.idm), "IDm");
        cmndbg_hexdump(test_reg_card_data.card_type, sizeof(test_reg_card_data.card_type), "Card Type");
        cmndbg_hexdump(test_reg_card_data.member_num, sizeof(test_reg_card_data.member_num), "Member Num");

        break;
    }
    case 14: // 登録・抹消カード削除
        // TODO : 不要？
        DBGLOG0("NO IMPLEMENT");
        break;
    case 15: { // 一括登録カードデータ存在確認
        DBGLOG0("VACENCY_REG_CARD_EXIST_REQ");
        MDLSTRG_REQUEST_T VACENCY_REG_CARD_EXIST_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_VACANCY_REG_CARD,        // 空室一括登録カード
            .request_type = MDLSTRG_REQ_TYPE_EXISTS,
        };
        mdlstrg_request(&VACENCY_REG_CARD_EXIST_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        exists = s_strgmdl_req_state;

        DBGLOG1("VACANCY_REG_CARD exists: %d", exists);

        break;
    }
    case 16: { // 一括登録カードデータ取得
        DBGLOG0("VACENCY_REG_CARD_READ_REQ");
        MDLSTRG_REQUEST_T VACENCY_REG_CARD_READ_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_VACANCY_REG_CARD,        // 空室一括登録カード
            .request_type = MDLSTRG_REQ_TYPE_READ,
            .data = &test_vacancy_reg_card_data,
        };
        mdlstrg_request(&VACENCY_REG_CARD_READ_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);

        cmndbg_hexdump(test_vacancy_reg_card_data.status_flg, sizeof(test_vacancy_reg_card_data.status_flg), "Status Flg");
        cmndbg_hexdump(test_vacancy_reg_card_data.idm, sizeof(test_vacancy_reg_card_data.idm), "IDm");
        cmndbg_hexdump(test_vacancy_reg_card_data.card_type, sizeof(test_vacancy_reg_card_data.card_type), "Card Type");
        cmndbg_hexdump(test_vacancy_reg_card_data.member_num, sizeof(test_vacancy_reg_card_data.member_num), "Member Num");

        break;
    }
    case 17: {// 一括登録カードデータ保存
        DBGLOG0("VACENCY_REG_CARD_WRITE_REQ");

        // TODO テストデータ
//        test_vacancy_reg_card_data.status_flg = 0x7FFFFFFFFFFFFFFF;
        test_vacancy_reg_card_data.status_flg[0] = 0x7F;
        for (int i = 1; i < 8; i++) {
            test_vacancy_reg_card_data.status_flg[i] = 0xFF;
        }
//        uint8_t idm[8] = {0xF1,0x14,0xb4,0xaa,0x60,0x13,0xa3,0xF9};
        uint8_t idm[8] = {0x01,0x14,0xb4,0xaa,0x60,0x13,0xa3,0x59}; // 050002468
//        uint8_t card_type[16] = {0x30, 0x35, 0x31, 0x38, 0x30, 0x39, 0x30, 0x39, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00};
        uint8_t card_type[16] = {0x30, 0x35, 0x31, 0x33, 0x30, 0x38, 0x30, 0x36, 0x30, 0x30, 0x30, 0x30, 0x39, 0x36, 0x38, 0x38}; // 050002468
        uint8_t member_num[16] = {0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20};

        memcpy(test_vacancy_reg_card_data.idm, idm, 8);
        memcpy(test_vacancy_reg_card_data.card_type, card_type, 16);
        memcpy(test_vacancy_reg_card_data.member_num, member_num, 16);

        MDLSTRG_REQUEST_T VACENCY_REG_CARD_WRITE_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_VACANCY_REG_CARD,        // 空室一括登録カード
            .request_type = MDLSTRG_REQ_TYPE_WRITE,
            .data = &test_vacancy_reg_card_data,
        };
        mdlstrg_request(&VACENCY_REG_CARD_WRITE_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        result = s_strgmdl_req_state;

        if (result != E_OK) {
            DBGLOG0("VACANCY_REG_CARD write error.");
        }

        cmndbg_hexdump(test_vacancy_reg_card_data.status_flg, sizeof(test_vacancy_reg_card_data.status_flg), "Status Flg");
        cmndbg_hexdump(test_vacancy_reg_card_data.idm, sizeof(test_vacancy_reg_card_data.idm), "IDm");
        cmndbg_hexdump(test_vacancy_reg_card_data.card_type, sizeof(test_vacancy_reg_card_data.card_type), "Card Type");
        cmndbg_hexdump(test_vacancy_reg_card_data.member_num, sizeof(test_vacancy_reg_card_data.member_num), "Member Num");
        break;
    }
    case 18: // WiFi接続情報存在確認
        DBGLOG0("WIFI_PARAM_EXISTS_REQ");
        MDLSTRG_REQUEST_T WIFI_PARAM_EXISTS_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_WIFI,        // WIFI接続情報
            .request_type = MDLSTRG_REQ_TYPE_EXISTS,
        };
        mdlstrg_request(&WIFI_PARAM_EXISTS_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        exists = s_strgmdl_req_state;

        DBGLOG1("WIFI_PARAM exists: %d", exists);

        break;
    case 19: // WiFi接続情報取得
        DBGLOG0("WIFI_PARAM_READ_REQ");
        MDLSTRG_REQUEST_T WIFI_PARAM_READ_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_WIFI,        // WiFi接続情報取得
            .request_type = MDLSTRG_REQ_TYPE_READ,
            .data = &test_wifi_param_data,
        };
        mdlstrg_request(&WIFI_PARAM_READ_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);

        cmndbg_hexdump(test_wifi_param_data.essid, sizeof(test_wifi_param_data.essid), "SSID");
        cmndbg_hexdump(test_wifi_param_data.passphrase, sizeof(test_wifi_param_data.passphrase), "pass");
        DBGLOG2("SSID_LEN %d, PASSPHRASE_LEN %d", test_wifi_param_data.essid_len, test_wifi_param_data.passphrase_len);

        break;
    case 20: {// WiFi接続情報保存
        DBGLOG0("WIFI_PARAM_WRITE_REQ");

        // TODO テストデータ
        // uint8_t ssid[32] = {0x30, 0x35, 0x31, 0x38, 0x30, 0x39, 0x30, 0x39, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00,
        //                     0x30, 0x32, 0x31, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x40, 0x41, 0x42, 0x43, 0x44, 0xFF};
        // uint8_t passphrase[64];
        // for (int i = 0; i < 64; i++ ) {
        //     passphrase[i] = i;
        // }
/*
        test_wifi_param_data.essid_len = 8;
        test_wifi_param_data.passphrase_len = 8;
        memcpy(test_wifi_param_data.essid, "ZMI_90D1", test_wifi_param_data.essid_len);
        memcpy(test_wifi_param_data.passphrase, "11622752", test_wifi_param_data.passphrase_len);
*/
        test_wifi_param_data.essid_len = 11;
        test_wifi_param_data.passphrase_len = 13;
//        memcpy(test_wifi_param_data.essid, "N01J-fa1c7c", test_wifi_param_data.essid_len);
//        memcpy(test_wifi_param_data.passphrase, "4850b15d8c731", test_wifi_param_data.passphrase_len);
        memcpy(test_wifi_param_data.essid, "Buffalo-G-2099", test_wifi_param_data.essid_len);
        memcpy(test_wifi_param_data.passphrase, "7r7kn53ckc7dh", test_wifi_param_data.passphrase_len);
        
        MDLSTRG_REQUEST_T WIFI_PARAM_WRITE_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_WIFI,        // WIFI接続情報
            .request_type = MDLSTRG_REQ_TYPE_WRITE,
            .data = &test_wifi_param_data,
        };
        mdlstrg_request(&WIFI_PARAM_WRITE_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        result = s_strgmdl_req_state;

        if (result != E_OK) {
            DBGLOG0("WIFI_PARAM_WRITE_REQ write error.");
        }

        cmndbg_hexdump(test_wifi_param_data.essid, sizeof(test_wifi_param_data.essid), "SSID");
        cmndbg_hexdump(test_wifi_param_data.passphrase, sizeof(test_wifi_param_data.passphrase), "pass");
        break;
    }
    case 21: {// 操作ログ保存
        DBGLOG0("MDLSTRG_REQ_TYPE_WRITE");

        MDLSTRG_DATA_OPE_LOG_T test_ooe_log[2] = {0};
        MDLSTRG_DATA_STRACT_LOG_T test_log_data1 = {1, 1, 0, 0, 0};
        MDLSTRG_DATA_STRACT_LOG_T test_log_data2 = {2, 1, 0, 0, 0};

        test_ooe_log[0].log_date[0].year  = 2018;
        test_ooe_log[0].log_date[0].month = 9;
        test_ooe_log[0].log_date[0].day   = 28;
        test_ooe_log[0].log_date[0].hour  = 11;
        test_ooe_log[0].log_date[0].min   = 7;
        test_ooe_log[0].log_date[0].dmy   = 8;
        test_ooe_log[0].log_date[1].year  = 2018;
        test_ooe_log[0].log_date[1].month = 9;
        test_ooe_log[0].log_date[1].day   = 28;
        test_ooe_log[0].log_date[1].hour  = 11;
        test_ooe_log[0].log_date[1].min   = 8;
        test_ooe_log[0].log_date[1].dmy   = 9;
        memcpy(&test_ooe_log[0].log[0], &test_log_data1, sizeof(MDLSTRG_DATA_STRACT_LOG_T));
        memcpy(&test_ooe_log[0].log[1], &test_log_data2, sizeof(MDLSTRG_DATA_STRACT_LOG_T));
        test_ooe_log[0].log[2].state      = 0x03;
        test_ooe_log[0].log[2].processing = 0;
        test_ooe_log[0].log[2].detail01   = 0;
        test_ooe_log[0].log[2].detail02   = 0;
        test_ooe_log[0].log[2].detail03   = 0;
        test_ooe_log[0].log[3].state      = 0x04;
        test_ooe_log[0].log[3].processing = 0;
        test_ooe_log[0].log[3].detail01   = 0;
        test_ooe_log[0].log[3].detail02   = 0;
        test_ooe_log[0].log[3].detail03   = 0;
        memcpy(&test_ooe_log[0].log[4], &test_log_data1, sizeof(MDLSTRG_DATA_STRACT_LOG_T));
        test_ooe_log[1].log_date[0].year  = 2019;
        test_ooe_log[1].log_date[0].month = 10;
        test_ooe_log[1].log_date[0].day   = 29;
        test_ooe_log[1].log_date[0].hour  = 12;
        test_ooe_log[1].log_date[0].min   = 24;
        test_ooe_log[1].log_date[0].dmy   = 9;
        test_ooe_log[1].log_date[1].year  = 2019;
        test_ooe_log[1].log_date[1].month = 10;
        test_ooe_log[1].log_date[1].day   = 29;
        test_ooe_log[1].log_date[1].hour  = 12;
        test_ooe_log[1].log_date[1].min   = 25;
        test_ooe_log[1].log_date[1].dmy   = 10;
        memcpy(&test_ooe_log[1].log[0], &test_log_data2, sizeof(MDLSTRG_DATA_STRACT_LOG_T));
        memcpy(&test_ooe_log[1].log[1], &test_log_data1, sizeof(MDLSTRG_DATA_STRACT_LOG_T));
        test_ooe_log[1].log[2].state      = 0x30;
        test_ooe_log[1].log[2].processing = 0x30;
        test_ooe_log[1].log[2].detail01   = 0x30;
        test_ooe_log[1].log[2].detail02   = 0x30;
        test_ooe_log[1].log[2].detail03   = 0x31;
        test_ooe_log[1].log[3].state      = 0x30;
        test_ooe_log[1].log[3].processing = 0x30;
        test_ooe_log[1].log[3].detail01   = 0x32;
        test_ooe_log[1].log[3].detail02   = 0x30;
        test_ooe_log[1].log[3].detail03   = 0x32;
        memcpy(&test_ooe_log[1].log[4], &test_log_data2, sizeof(MDLSTRG_DATA_STRACT_LOG_T));

        MDLSTRG_REQUEST_T OPE_LOG_WRITE_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_OPE_LOG,        // 操作ログ
            .request_type = MDLSTRG_REQ_TYPE_WRITE,        // 書込み
            .data = &test_ooe_log,
            .opt1 = 2
        };
        mdlstrg_request(&OPE_LOG_WRITE_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        result = s_strgmdl_req_state;

        if (result != E_OK) {
            DBGLOG0("MDLSTRG_REQ_TYPE_WRITE write error.");
        }

        cmndbg_hexdump(test_ooe_log[0].log, sizeof(MDLSTRG_DATA_OPE_LOG_T), "log1");
        cmndbg_hexdump(test_ooe_log[1].log, sizeof(MDLSTRG_DATA_OPE_LOG_T), "log2");
        break;
    }
    case 22: {// エラーログ保存
        DBGLOG0("MDLSTRG_REQ_TYPE_WRITE");

        MDLSTRG_DATA_ERR_LOG_T test_err_log = {0};

        test_err_log.log_date[0].year  = 2018;
        test_err_log.log_date[0].month = 9;
        test_err_log.log_date[0].day   = 28;
        test_err_log.log_date[0].hour  = 11;
        test_err_log.log_date[0].min   = 7;
        test_err_log.log_date[0].dmy   = 8;
        test_err_log.log_date[1].year  = 2018;
        test_err_log.log_date[1].month = 9;
        test_err_log.log_date[1].day   = 28;
        test_err_log.log_date[1].hour  = 11;
        test_err_log.log_date[1].min   = 8;
        test_err_log.log_date[1].dmy   = 9;
        test_err_log.err[0]            = 0x3431;
        test_err_log.err[1]            = 0x3030;
        test_err_log.err[2]            = 0x3230;
        test_err_log.err[3]            = 0x3138;
        test_err_log.err[4]            = 0x3132;
        test_err_log.err[5]            = 0x3331;
        test_err_log.err[6]            = 0x3030;
        test_err_log.err[7]            = 0x3031;
        test_err_log.err[8]            = 0x3130;
        test_err_log.err[9]            = 0x3031;
        test_err_log.err[10]           = 0x3030;
        test_err_log.err[11]           = 0x3030;
 
        MDLSTRG_REQUEST_T ERR_LOG_WRITE_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_ERR_LOG,        // エラーログ
            .request_type = MDLSTRG_REQ_TYPE_WRITE,        // 書込み
            .data = &test_err_log,
        };

        mdlstrg_request(&ERR_LOG_WRITE_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        result = s_strgmdl_req_state;

        if (result != E_OK) {
            DBGLOG0("MDLSTRG_REQ_TYPE_WRITE write error.");
        }

        cmndbg_hexdump(test_err_log.err, sizeof(test_err_log.err), "ERROR");
        break;
    }
//    case23:	//※※※case23はすでに使用しています※※※
//        DBGLOG0("NO IMPLEMENT");
//        break;
    case 24: {// 最新の操作ログデータを取得する
        DBGLOG0("OPE_LOG_READ_REQ");

        // TODO テストデータ
        MDLSTRG_DATA_OPE_LOG_T test_ope_log;
        MDLSTRG_DATA_OPE_LOG_T *ptest_ope_log = &test_ope_log;

        MDLSTRG_REQUEST_T OPE_LOG_READ_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_OPE_LOG,        // 操作ログ
            .request_type = MDLSTRG_REQ_TYPE_READ,         // 読込
            .data = ptest_ope_log,
        };
        mdlstrg_request(&OPE_LOG_READ_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        result = s_strgmdl_req_state;

        if (result != true) {
            if (ptest_ope_log->log_date->year == LOG_NO_DATA) {
                DBGLOG0("OPE_LOG_READ_REQ no data.");
            } else {
                DBGLOG0("OPE_LOG_READ_REQ read error.");
            }
        }

        cmndbg_hexdump(ptest_ope_log, sizeof(MDLSTRG_DATA_OPE_LOG_T), "OPE LOG");
        break;
    }
    case 25: {// 最新のエラーログデータを取得する
        DBGLOG0("ERR_LOG_READ_REQ");

        // TODO テストデータ
        MDLSTRG_DATA_ERR_LOG_T test_err_log;
        MDLSTRG_DATA_ERR_LOG_T *ptest_err_log = &test_err_log;

        MDLSTRG_REQUEST_T ERR_LOG_READ_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_ERR_LOG,        // エラーログ
            .request_type = MDLSTRG_REQ_TYPE_READ,         // 読込
            .data = ptest_err_log,
        };
        mdlstrg_request(&ERR_LOG_READ_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        result = s_strgmdl_req_state;

        if (result != true) {
            if (ptest_err_log->log_date->year == LOG_NO_DATA) {
                DBGLOG0("ERR_LOG_READ_REQ no data.");
            } else {
                DBGLOG0("ERR_LOG_READ_REQ read error.");
            }
        }

        cmndbg_hexdump(ptest_err_log, sizeof(MDLSTRG_DATA_ERR_LOG_T), "ERR LOG");
        break;
    }
    case 26: {// 操作ログ読込カウンタ更新
        DBGLOG0("OPE_LOG_INCREMENT_REQ");
        int loopCntMax;

        if (!opt2) {
            loopCntMax = 1;
        }else{
            loopCntMax = atoi(opt2);		// cmd
        }
        DBGLOG1("loopCntMax = %d", loopCntMax);

        MDLSTRG_REQUEST_T OPE_LOG_INCREMENT_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_OPE_LOG,        // 操作ログ
            .request_type = MDLSTRG_REQ_TYPE_INCREMENT
        };
        int loopCnt = 0;
        for(loopCnt=0; loopCnt < loopCntMax; loopCnt++){
            DBGLOG1("=== loopCnt=%d ===", loopCnt + 1);

            mdlstrg_request(&OPE_LOG_INCREMENT_REQ, strg_callback);
            er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
            assert(er == E_OK);
            result = s_strgmdl_req_state;

            if (result != E_OK) {
                DBGLOG0("OPE_LOG_INCREMENT_REQ write error.");
                break;
            }
        }
        break;
    }
    case 27: {// エラーログ読込カウンタ更新
        DBGLOG0("ERR_LOG_INCREMENT_REQ");
        int loopCntMax;

        if (!opt2) {
            loopCntMax = 1;
        }else{
            loopCntMax = atoi(opt2);		// cmd
        }
        DBGLOG1("loopCntMax = %d", loopCntMax);

        MDLSTRG_REQUEST_T ERR_LOG_INCREMENT_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_ERR_LOG,        // エラーログ
            .request_type = MDLSTRG_REQ_TYPE_INCREMENT
        };
        int loopCnt = 0;
        for(loopCnt=0; loopCnt < loopCntMax; loopCnt++){
            DBGLOG1("=== loopCnt=%d ===", loopCnt + 1);

            mdlstrg_request(&ERR_LOG_INCREMENT_REQ, strg_callback);
            er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
            assert(er == E_OK);
            result = s_strgmdl_req_state;

            if (result != E_OK) {
                DBGLOG0("ERR_LOG_INCREMENT_REQ write error.");
                break;
            }
        }
        break;
    }
    case 28: {// 操作ログ書き込みカウンタ取得
        DBGLOG0("GET_OPE_LOG_CNT_REQ");

        // TODO テストデータ
        MDLSTRG_DATA_RW_COUNTER_T   log_cont_ope = {0};
        MDLSTRG_DATA_RW_COUNTER_T* plog_cont_ope = &log_cont_ope;

        MDLSTRG_REQUEST_T GET_OPE_LOG_CNT_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_OPE_LOG,        // 操作ログ
            .request_type = MDLSTRG_REQ_TYPE_CNT_READ,
            .data = plog_cont_ope,
        };
        mdlstrg_request(&GET_OPE_LOG_CNT_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        result = s_strgmdl_req_state;

        if (result != true) {
            if(plog_cont_ope->bitsw_rd[0] == LOG_CNT_NO_DATA && plog_cont_ope->bitsw_rd[1] == LOG_CNT_NO_DATA
                && plog_cont_ope->bitsw_wt[0] == LOG_CNT_NO_DATA && plog_cont_ope->bitsw_wt[1] == LOG_CNT_NO_DATA){
                DBGLOG0("GET_OPE_LOG_CNT_REQ Read,Write Counter no data.");
            } else {
                DBGLOG0("GET_OPE_LOG_CNT_REQ write error.");
            }
        }

        cmndbg_hexdump(&log_cont_ope, sizeof(MDLSTRG_DATA_RW_COUNTER_T), "OPE LOG CNT");
        break;
    }
    case 29: {// エラーログ書き込みカウンタ取得
        DBGLOG0("GET_ERR_LOG_CNT_REQ");

        // TODO テストデータ
        MDLSTRG_DATA_RW_COUNTER_T   log_cont_err = {0};
        MDLSTRG_DATA_RW_COUNTER_T* plog_cont_err = &log_cont_err;

        MDLSTRG_REQUEST_T GET_ERR_LOG_CNT_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_ERR_LOG,        // エラーログ
            .request_type = MDLSTRG_REQ_TYPE_CNT_READ,
            .data = plog_cont_err,
        };
        mdlstrg_request(&GET_ERR_LOG_CNT_REQ, strg_callback);
        er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
        assert(er == E_OK);
        result = s_strgmdl_req_state;

        if (result != true) {
            if(plog_cont_err->bitsw_rd[0] == LOG_CNT_NO_DATA && plog_cont_err->bitsw_rd[1] == LOG_CNT_NO_DATA
                && plog_cont_err->bitsw_wt[0] == LOG_CNT_NO_DATA && plog_cont_err->bitsw_wt[1] == LOG_CNT_NO_DATA){
                DBGLOG0("GET_ERR_LOG_CNT_REQ Read,Write Counter no data.");
            } else {
                DBGLOG0("GET_ERR_LOG_CNT_REQ write error.");
            }
        }

        cmndbg_hexdump(&log_cont_err, sizeof(MDLSTRG_DATA_RW_COUNTER_T), "ERR LOG CNT");
        break;
    }
    case 30: {// 操作ログ保存拡張（ログを連続して追加）
        DBGLOG0("MDLSTRG_REQ_TYPE_WRITE_EXTEND");

        static MDLSTRG_DATA_OPE_LOG_T test_ope_log_ex = {
                                                         {
                                                          {1,9,28,11,7,8},
                                                          {43690,1,2,3,4,5}
                                                          },
                                                         {
                                                          {1, 1, 0, 0, 0},
                                                          {2, 1, 0, 0, 0},
                                                          {3, 0, 0, 0, 0},
                                                          {4, 0, 0, 0, 0},
                                                          {1, 1, 0, 0, 0}
                                                          }
                                                         };
        MDLSTRG_REQUEST_T OPE_LOG_WRITE_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_OPE_LOG,        // 操作ログ
            .request_type = MDLSTRG_REQ_TYPE_WRITE,        // 書込み
            .data = &test_ope_log_ex,
            .opt1 = 1
        };

        // 引数(opt2)で指定された書き込み数を保存
        int loopCntMax;
        if (!opt2) {
            loopCntMax = 1;
        }else{
            loopCntMax = atoi(opt2);		// cmd
        }
        DBGLOG1("loopCntMax = %d", loopCntMax);

        // 連続して書き込む
        int loopCnt = 0;
        char dumpName[10];
        for(loopCnt=0; loopCnt < loopCntMax; loopCnt++){
            mdlstrg_request(&OPE_LOG_WRITE_REQ, strg_callback);
            er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
            assert(er == E_OK);

            memset(dumpName, 0, sizeof(dumpName));
            sprintf(dumpName, "log_%d", loopCnt + 1);
            cmndbg_hexdump(&test_ope_log_ex, sizeof(MDLSTRG_DATA_OPE_LOG_T), dumpName);

            result = s_strgmdl_req_state;
            if (result != E_OK) {
                DBGLOG0("MDLSTRG_REQ_TYPE_WRITE write error.");
                break;
            }

            test_ope_log_ex.log_date[0].year++;
        }

        break;
    }
    case 31: {// エラーログ保存拡張（ログを連続して追加）
        DBGLOG0("MDLSTRG_REQ_TYPE_WRITE_EXTEND");

        static MDLSTRG_DATA_ERR_LOG_T test_err_log = {{{1,9,28,11,7,8},
                                                       {48059,9,28,11,8,9}},
                                                       {0x3431,0x3030,0x3230,
                                                        0x3138,0x3132,0x3331,
                                                        0x3030,0x3031,0x3130,
                                                        0x3031,0x3030,0x3030}};
        MDLSTRG_REQUEST_T ERR_LOG_WRITE_REQ = {
            .data_type = MDLSTRG_DATA_TYPE_ERR_LOG,        // エラーログ
            .request_type = MDLSTRG_REQ_TYPE_WRITE,        // 書込み
            .data = &test_err_log,
        };

        // 引数(opt2)で指定された書き込み数を保存
        int loopCntMax;
        if (!opt2) {
            loopCntMax = 1;
        }else{
            loopCntMax = atoi(opt2);		// cmd
        }
        DBGLOG1("loopCntMax = %d", loopCntMax);

        // 連続して書き込む
        int loopCnt = 0;
        char dumpName[10];
        for(loopCnt=0; loopCnt < loopCntMax; loopCnt++){
            mdlstrg_request(&ERR_LOG_WRITE_REQ, strg_callback);
            er = twai_flg(FLG_APLCON, FLGPTN_MDLSTRG_REQ_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 3000);
            assert(er == E_OK);

            memset(dumpName, 0, sizeof(dumpName));
            sprintf(dumpName, "log_%d", loopCnt + 1);
            cmndbg_hexdump(&test_err_log, sizeof(MDLSTRG_DATA_ERR_LOG_T), dumpName);

            result = s_strgmdl_req_state;
            if (result != E_OK) {
                DBGLOG0("MDLSTRG_REQ_TYPE_WRITE write error.");
                break;
            }

            test_err_log.log_date[0].year++;
        }
        break;
    }


    default:
        break;
    }

end:
    return 0;
}

