/*
 * VA-X BLEアプリケーション
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/01/19 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

// ソースの対になるヘッダ
#include "aplble.h"

// 標準C
#include <string.h>

// OS関係
#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"

// その他ヘッダ
#include "mdlble.h"
#include "cmncrypt.h"
#include "mdlstrg.h"
#include "mdlstrg_data.h"
#include "drvrng.h"
#include "drvwifi.h"
#include "aplicc.h"
#include "aplui.h"
#include "aplauth.h"
#include "aplicc.h"

// デバッグログ
#if 1
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[APLBLE]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[APLBLE]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[APLBLE]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[APLBLE]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

//#define TEST_USE_DUMMY_CARD_DATA
#define TEST_NO_VALIDATE_USER_SIGNATURE	// 【暫定対応】ユーザー鍵の検証をしない
#define SERVER_IP "192.168.43.1"
#define SERVER_PORT 5000
#define MAX_TRY 5 // Try to connect to TCP server 5 times

#define RECEIVE_BUFFER_SIZE 4096

/*
 * 型
 */

typedef struct {
    uint8_t slotno;
    uint8_t user_type;
    uint8_t finger_type;
    uint8_t namelen;
    uint32_t name[32];
} REGINFO_T;

typedef struct {
    uint8_t size;
    uint16_t command;
    uint8_t* body;
} APLBLE_COMMAND_DATA_T;

/*
 * 定数
 */
static const FLGPTN FLGPTN_MDLBLE_INITIALIZE_COMPLETE =	(0x1 << 0);
static const FLGPTN FLGPTN_MDLBLE_START_COMPLETE =		(0x1 << 1);
static const FLGPTN FLGPTN_MDLBLE_DATA_RECEIVED =		(0x1 << 2);
static const FLGPTN FLGPTN_MDLBLE_SEND_COMPLETE =		(0x1 << 3);
static const FLGPTN FLGPTN_MDLSTRG_REQUEST_COMPLETE =	(0x1 << 4);
static const FLGPTN FLGPTN_MDLBLE_RESTART_COMPLETE =	(0x1 << 5);

static const FLGPTN FLGPTN_DRVWIFI_INITIALIZE_COMPLETE  = (0x1 << 6);
static const FLGPTN FLGPTN_DRVWIFI_AP_CONNECT_COMPLETE  = (0x1 << 7);
static const FLGPTN FLGPTN_DRVWIFI_TCP_CONNECT_COMPLETE = (0x1 << 8);
static const FLGPTN FLGPTN_DRVWIFI_TCP_SEND_COMPLETE    = (0x1 << 9);
static const FLGPTN FLGPTN_DRVWIFI_TCP_RECEIVE_COMPLETE = (0x1 << 10);
static const FLGPTN FLGPTN_DRVWIFI_TCP_SERVER_COMPLETE  = (0x1 << 11);

static const uint8_t const BLE_PIN[] = {0x31, 0x32, 0x33, 0x34};

static const uint8_t const TEST_KEY_PUB[] = {
    0x30, 0x82, 0x01, 0x22, 0x30, 0x0d, 0x06, 0x09, 0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x01,
    0x01, 0x05, 0x00, 0x03, 0x82, 0x01, 0x0f, 0x00, 0x30, 0x82, 0x01, 0x0a, 0x02, 0x82, 0x01, 0x01,
    0x00, 0xb0, 0x55, 0x8a, 0xa9, 0xec, 0x19, 0x74, 0x15, 0x60, 0x42, 0x25, 0x25, 0x2d, 0xfe, 0xda,
    0xbd, 0x5e, 0xb5, 0xae, 0x8f, 0xee, 0xa8, 0xf9, 0x39, 0xd5, 0xb7, 0xfa, 0xd8, 0x80, 0xbb, 0xc5,
    0x2f, 0x8b, 0xd5, 0xbc, 0xc0, 0x3c, 0x90, 0x1f, 0x45, 0xfd, 0x24, 0x98, 0x57, 0xe6, 0x24, 0xe3,
    0x98, 0xf0, 0x4f, 0xe3, 0xd3, 0xf7, 0xf2, 0xf1, 0x1b, 0x48, 0x4c, 0xf1, 0x6d, 0x0b, 0xc6, 0x81,
    0xe1, 0xa3, 0x73, 0xf1, 0x86, 0x1b, 0x5a, 0xd9, 0xb1, 0x70, 0x22, 0x67, 0xd3, 0x0f, 0xd6, 0xe3,
    0x35, 0x0f, 0x0e, 0x85, 0xb7, 0x28, 0x72, 0xf4, 0xbd, 0x5a, 0x92, 0xa7, 0xd6, 0xeb, 0x8b, 0xb9,
    0x0a, 0xa1, 0x6f, 0x09, 0x56, 0x82, 0xc2, 0x98, 0xd8, 0x50, 0x1d, 0xf8, 0x6f, 0x66, 0x11, 0x2d,
    0x54, 0x2c, 0xf4, 0xef, 0x0d, 0xbc, 0x2c, 0xbf, 0xde, 0x5b, 0xcd, 0x32, 0xad, 0xf3, 0xf3, 0x5c,
    0x3c, 0x00, 0x24, 0x88, 0x88, 0xe9, 0x89, 0xdb, 0x28, 0xfd, 0x4a, 0x33, 0x6d, 0xae, 0xba, 0x01,
    0x33, 0x7a, 0xf2, 0x83, 0xa0, 0x56, 0xd6, 0x9a, 0x85, 0xb3, 0x3c, 0x25, 0x4d, 0x14, 0x8f, 0x87,
    0x61, 0x47, 0xf9, 0xbf, 0x43, 0xb4, 0x81, 0x90, 0x4d, 0x28, 0x79, 0x08, 0x2d, 0x30, 0xbc, 0xef,
    0xf4, 0xb9, 0x50, 0x97, 0x69, 0x66, 0x20, 0x13, 0x29, 0xcb, 0x38, 0x95, 0x07, 0x43, 0xd7, 0xff,
    0x0e, 0x18, 0x74, 0xd8, 0xb0, 0x52, 0x27, 0xdc, 0xde, 0xa3, 0xc4, 0x42, 0x05, 0xfa, 0xbe, 0xe0,
    0xe0, 0x40, 0xfe, 0xef, 0xb2, 0x6c, 0x0d, 0xfe, 0xf7, 0xff, 0x7d, 0x28, 0x19, 0x88, 0x76, 0x13,
    0x31, 0xb0, 0x7b, 0xc3, 0xea, 0xc1, 0x9d, 0x26, 0xcc, 0x4a, 0x89, 0x38, 0x8d, 0x7a, 0x8c, 0x57,
    0xeb, 0x5b, 0x5c, 0xd1, 0xa6, 0x2f, 0xe1, 0x35, 0xae, 0xe5, 0x77, 0x8c, 0xbf, 0x1d, 0xa0, 0xee,
    0x75, 0x02, 0x03, 0x01, 0x00, 0x01,
};

#ifdef TEST_USE_DUMMY_CARD_DATA
typedef struct {
    uint8_t		slotno;			// 登録スロット
	uint8_t		namelen;		// 登録名バイト長
	uint8_t		padding0;
	uint8_t		padding1;
	uint32_t	name[32];		// 登録名
	uint8_t		card_id[16];	// カードに印字されている番号(サービスコード300Bの情報)
} TEST_CARD_DATA_T;
#define TEST_CARD_DATA_NUM 2
static const TEST_CARD_DATA_T const TEST_CARD_DATA[TEST_CARD_DATA_NUM] = {
    {
        1,
        32 * 4,
        0, 0,
        {
            0x00005c71, 0x00007530, 0x0000592a, 0x000090ce, 0x00000034, 0x00000035, 0x00000036, 0x00000037,
            0x00000038, 0x00000039, 0x00000030, 0x00000031, 0x00000032, 0x00000033, 0x00000034, 0x00000035,
            0x00000036, 0x00000037, 0x00000038, 0x00000039, 0x00000030, 0x00000031, 0x00000032, 0x00000033,
            0x00000034, 0x00000035, 0x00000036, 0x00000037, 0x00000038, 0x00000045, 0x0000004e, 0x00000044,
        },
        {
            0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30, 0x31, 0x32, 0x20, 0x20, 0x20,
        },
    },
    {
        2,
        4 * 4,
        0, 0,
        {
            0x00005c71, 0x00004e0b, 0x00009054, 0x000090ce, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
        },
        {
            0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
        },
    },
};
#endif	// TEST_USE_DUMMY_CARD_DATA

#define NRANDOM	16	// 乱数バイト数

//
#define CARD_INITIAL	0xFFFF	// カード登録状態初期値
#define CARD_ENTRY_None	0x7FFF	// カード登録状態
#define CARD_ENTRY		0x3FFF	// カード登録状態

// Command list
#define APLBLE_CMD_AUTHENTICATE        (0x3030U) // "00"
#define APLBLE_CMD_SET_DB_WIFI         (0x3131U) // "11"
#define APLBLE_CMD_SET_PHONE_WIFI      (0x3132U) // "12"
#define APLBLE_CMD_SET_DEVICE_INFO     (0x3133U) // "13"
#define APLBLE_CMD_SET_COUNTER_INFO    (0x3134U) // "14"
#define APLBLE_CMD_SET_ICCARD_INFO1    (0x3135U) // "15"
#define APLBLE_CMD_UPDATE_FIRMWARE     (0x3145U) // "1E"
#define APLBLE_CMD_INITILIZE_DATA      (0x3146U) // "1F"

#define APLBLE_CMD_GET_VERSION         (0x3231U) // "21"
#define APLBLE_CMD_GET_SERIAL          (0x3232U) // "22"
#define APLBLE_CMD_GET_BLE_MAC         (0x3233U) // "23"
#define APLBLE_CMD_GET_ROOM_INFO       (0x3234U) // "24"
#define APLBLE_CMD_GET_DB_WIFI_INFO    (0x3235U) // "25"
#define APLBLE_CMD_GET_ERROR_LOG       (0x3236U) // "26"
#define APLBLE_CMD_GET_ICCARD_INFO1    (0x3237U) // "27"
#define APLBLE_CMD_GET_ICCARD_INFO2    (0x3238U) // "28"
#define APLBLE_CMD_GET_PARAMETERS      (0x3239U) // "29"
#define APLBLE_CMD_GET_DIAGNOSTIC      (0x3241U) // "2A"

#define APLBLE_CMD_CHECK_ICCARD1_SIZE  (0x3337U) // "37"
#define APLBLE_CMD_CHECK_ICCARD2_SIZE  (0x3338U) // "38"

// 接続状態
enum {
    SESSION_IDLE = 0,
    SESSION_CONNECTED,
    SESSION_AUTHENTICATED,
};

// カード操作
enum {
    CARDOP_NONE = 0,
    CARDOP_AUTH,
    CARDOP_REGISTER,
};

// サーバー許可フラグ
static const uint32_t PERMISSION_PAIRING =		(0x1 << 0);
static const uint32_t PERMISSION_DEPAIRING =	(0x1 << 1);

/*
 * 内部関数プロトタイプ
 */
static void mdlble_callback(int event, intptr_t opt1, intptr_t opt2);
static void aplble_authenticate(const APLBLE_COMMAND_DATA_T* data);
static void aplble_set_phone_wifi(const APLBLE_COMMAND_DATA_T* data);
static void aplble_update_firmware(const APLBLE_COMMAND_DATA_T* data);
static void mdlstrg_store_program_write(uint8_t* data, uint32_t address, size_t size, int index);
static void mdlstrg_store_program_delete(size_t size, int index);

static void aplble_process_data_1(const MDLBLE_DATA_T* data);
static int aplble_process_data_1_05(uint8_t* resp_buf, const uint8_t* data, size_t data_len);
static int aplble_process_data_1_06(uint8_t* resp_buf, const uint8_t* data, size_t data_len);
static void aplble_process_data_2(const MDLBLE_DATA_T* data);
static void mdlstrg_callback(int event, intptr_t opt1, intptr_t opt2);
static void aplble_process_command(const MDLBLE_DATA_T* data);
static void drvwifi_callback(int32_t evt, int32_t error, intptr_t opt);

/*
 * 内部変数
 */
static int ble_process = true;
// イベント送信先
static APLEVT_EVENT_RECEIVER_FUNC_T s_event_dest = NULL;
static uint8_t s_ssid[33];
static uint8_t s_password[33];
static uint8_t s_received_data[RECEIVE_BUFFER_SIZE];

// コンテキスト
static struct {
    struct {
        uint8_t response[MDLBLE_MAX_DATA_LEN];
    } buf;	//
    struct {
        int state;
        int auth_index;
        uint32_t permissions;
        uint8_t random[NRANDOM];
        MDLSTRG_DATA_APPKEY_T appkey;
    } session;
    struct {
        int state;
        int auth_index;
    } regstate;
    struct {
        bool_t pending;
        MDLSTRG_DATA_AUTH_META_T auth_meta;
        MDLSTRG_DATA_CARD_META_T card_meta[MDLSTRG_MAX_NDATA_CARD];
        APLEVT_EXTRA_DATA_T extra_data;
    } reg;
    struct {
        int op;
        int op_slot;
        int op_result;
        bool_t regcard_authenticated;
    } cardstate;
    union {
        MDLSTRG_DATA_AUTH_META_T auth_meta;
    } tmp;
    struct {
        intptr_t opt1;
        intptr_t opt2;
    } strg_result;
    struct {
        uint8_t status1;
        uint8_t status2;
    } device_status;
    BNVA_DEVICE_ID_T device_id;
} s_context;

extern int32_t event_ble(APLEVT_EVENT_T* event);

//static MDLSTRG_DATA_CHASH_META_T cashe_meta[MDLSTRG_MAX_NDATA_CARD] = {0};

static int card_flg = 0;
/*
 * 共通初期化API(APLEVT_INITIALIZE_FUNC_T)
 */
int32_t aplble_initialize(APLEVT_EVENT_RECEIVER_FUNC_T receiver_func)
{
    assert(!s_event_dest);
    assert(receiver_func);

    // イベント送信先を登録
    s_event_dest = receiver_func;

    // タスクを起動
    ER er = E_OK;
    er = act_tsk(TSK_APLBLE);
    assert(er == E_OK);
    er = act_tsk(TSK_APLBLE_RX);
    assert(er == E_OK);

    memset(s_ssid, 0, sizeof(s_ssid));
    memset(s_password, 0, sizeof(s_password));

    return 0;
}

/*
 * 共通イベント受信API(APLEVT_EVENT_RECEIVER_FUNC_T)
 */
int32_t aplble_event(APLEVT_EVENT_T* event)
{
    ER er = aplevt_queue_event(DTQ_APLBLE, event, 0);
    assert(er == E_OK);

    return 0;
}

/*
 * メインタスク
 */
void aplble_task(intptr_t exinf)
{
	DBGLOG1("aplble_task() starts (exinf = %d).", (int_t) exinf);

    FLGPTN flgptn = 0;
    ER er = E_OK;
    APLEVT_EXTRA_DATA_T extra_data;
   
#if 0	// 機器識別情報に固定値を書込み
    {
        memcpy(s_context.device_id.serialno, "AQUBIO2018100100", sizeof(s_context.device_id.serialno));
//        memcpy(s_context.device_id.serialno, "AQUBIO2018072000", sizeof(s_context.device_id.serialno));
        MDLSTRG_REQUEST_T strg_req = {
            .data_type = MDLSTRG_DATA_TYPE_DEVICE_ID,
            .request_type = MDLSTRG_REQ_TYPE_WRITE,
            .data = (intptr_t)&(s_context.device_id),
            .size = sizeof(s_context.device_id),
        };
        mdlstrg_request(&strg_req, mdlstrg_callback);
        er = twai_flg(FLG_APLBLE, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 2000);
        assert(er == E_OK);
    }
#endif

    // 機器識別情報を読み出し
    MDLSTRG_REQUEST_T strg_req = {
        .data_type = MDLSTRG_DATA_TYPE_DEVICE_ID,
        .request_type = MDLSTRG_REQ_TYPE_READ,
        .data = (intptr_t)&(s_context.device_id),
        .size = sizeof(s_context.device_id),
    };
    mdlstrg_request(&strg_req, mdlstrg_callback);
    er = twai_flg(FLG_APLBLE, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 2000);
    assert(er == E_OK);
    DBGLOG1("device_id: %s", s_context.device_id.serialno);
/*
    // BLEミドル起動
    clr_flg(FLG_APLBLE, ~FLGPTN_MDLBLE_INITIALIZE_COMPLETE);
    mdlble_initialize(mdlble_callback, &s_context.device_id);
    er = twai_flg(FLG_APLBLE, FLGPTN_MDLBLE_INITIALIZE_COMPLETE, TWF_ANDW, &flgptn, 2000);
    assert(er == E_OK);

    clr_flg(FLG_APLBLE, ~FLGPTN_MDLBLE_START_COMPLETE);
    mdlble_start();
    er = twai_flg(FLG_APLBLE, FLGPTN_MDLBLE_START_COMPLETE, TWF_ANDW, &flgptn, 2000);
    assert(er == E_OK);
    
    aplui_dbg_led_pattern_off();        //ALL OFF
    aplui_dbg_set_led_pattern(0, 2);	//R
*/
    
    while (true) {
        APLEVT_EVENT_T* event = NULL;
        ER er_rcv = aplevt_receive_event(DTQ_APLBLE, &event, TMO_FEVR);
        assert(er_rcv == E_OK);
        DBGLOG1("aplble_task() received: (msg = %d).", event->code);

        switch (event->code) {
        case APLEVT_BLE_ENABLE: {
            DBGLOG0("APLEVT_BLE_RECIEVE_ENABLE");
            clr_flg(FLG_APLBLE, ~FLGPTN_MDLBLE_INITIALIZE_COMPLETE);
            mdlble_initialize(mdlble_callback, &s_context.device_id);
            er = twai_flg(FLG_APLBLE, FLGPTN_MDLBLE_INITIALIZE_COMPLETE, TWF_ANDW, &flgptn, 2000);
            assert(er == E_OK);

            clr_flg(FLG_APLBLE, ~FLGPTN_MDLBLE_START_COMPLETE);
            mdlble_start();
            er = twai_flg(FLG_APLBLE, FLGPTN_MDLBLE_START_COMPLETE, TWF_ANDW, &flgptn, 2000);
            assert(er == E_OK);

            // Wifi	//20181205Miyauchi VUP MERGE
            clr_flg(FLG_APLBLE, ~FLGPTN_DRVWIFI_INITIALIZE_COMPLETE);
            drvwifi_initialize(drvwifi_callback);
            er = twai_flg(FLG_APLBLE, FLGPTN_DRVWIFI_INITIALIZE_COMPLETE, TWF_ANDW, &flgptn, 2000);
            assert(er == E_OK);

//            extra_data.reg_prep_req.index = ble_process;
            aplui_dbg_led_pattern_off();        //ALL OFF
            aplui_dbg_set_led_pattern(0, 2);	//R
            break;
        }    
            
        case APLEVT_ICC_REGIST_COMPLETE: {
            DBGLOG0("APLEVT_ICC_REGIST_COMPLETE");
            s_context.device_status.status1 = 0x0;
            s_context.device_status.status2 = 0x0;
            //LED点灯用イベント
            aplevt_send_event(APLEVT_AUTH_COMPLETE, 0, NULL, aplui_event);
            dly_tsk(1000);
            aplevt_send_event(APLEVT_AUTH_READY, 0, NULL, aplui_event);
            break;
        }
        case APLEVT_ICC_REGAUTH_COMPLETE: {
            DBGLOG0("APLEVT_ICC_REGAUTH_COMPLETE");
                s_context.device_status.status1 = 0x0;
                s_context.device_status.status2 = 0x0;
//                aplui_dbg_led_pattern_off();        //ALL OFF
//                aplui_dbg_set_led_pattern(1, 2);
                //LED点灯用イベント
                aplevt_send_event(APLEVT_ICC_REGAUTH_COMPLETE, 0, NULL, aplui_event);
                dly_tsk(1000);
                aplevt_send_event(APLEVT_AUTH_READY, 0, NULL, aplui_event);
            break;
        }
        case APLEVT_ICC_AUTH_COMPLETE: {
            DBGLOG0("APLEVT_ICC_AUTH_COMPLETE");
            switch (event->error) {
                case 0:
                DBGLOG0("APLEVT_ICC_AUTH_COMPLETE_TO_IDLE");
                s_context.device_status.status1 = 0x0;
                s_context.device_status.status2 = 0x0;
                aplui_dbg_led_pattern_off();        //ALL OFF
                aplui_dbg_set_led_pattern(1, 2);
                break;
                case APLICC_ERROR_NO_REGIST_CARD:
                DBGLOG0("APLICC_ERROR_NO_REGIST_CARD");
                s_context.device_status.status1 = 0x3;
                s_context.device_status.status2 = 0x0;
                aplui_dbg_led_pattern_off();        //ALL OFF
                aplui_dbg_set_led_pattern(0, 2);
                extra_data.reg_prep_req.index = APLICC_CARD_REGIST;
                extra_data.reg_prep_req.timeout = 30000;
                DBGLOG2("extra_data %d %d", extra_data.reg_prep_req.index, extra_data.reg_prep_req.timeout);
                aplevt_send_event(APLEVT_ICC_REGIST_REQ, 0, &extra_data, s_event_dest);
                break;
            }
            break;
        }
        case APLEVT_REGISTRATION_READY: {
            s_context.device_status.status1 = 0x2;
            s_context.device_status.status2 = 0x1;
            break;
        }
        case APLEVT_REGISTRATION_PROCESSING: {
            s_context.device_status.status1 = 0x2;
            s_context.device_status.status2 = 0x2;
            break;
        }
        case APLEVT_REGISTRATION_COMPLETE: {
            if (s_context.reg.pending) {
                if (event->error == BNVA_ERROR_NONE) {
                    // データ保存
                    MDLSTRG_REQUEST_T strg_req = {
                        .data_type = MDLSTRG_DATA_TYPE_AUTH_META,
                        .request_type = MDLSTRG_REQ_TYPE_WRITE,
                        .data = (intptr_t)&(s_context.reg.auth_meta),
                        .size = sizeof(s_context.reg.auth_meta),
                        .opt1 = s_context.reg.auth_meta.slotno - 1,
                    };
                    mdlstrg_request(&strg_req, mdlstrg_callback);
                    ER er = twai_flg(FLG_APLBLE, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 2000);
                    assert(er == E_OK);
                } else {
                    
                }
            }
            s_context.reg.pending = false;
            s_context.device_status.status1 = 0x0;
            s_context.device_status.status2 = 0x0;
            break;
        }
        default:
            assert(false);
            break;
        }
    }
}

/*
 * 受信データ処理タスク
 */
void aplble_rx_task(intptr_t exinf)
{
	DBGLOG1("aplble_rx_task() starts (exinf = %d).", (int_t) exinf);

    while (true) {
        MDLBLE_DATA_T* data = NULL;
        ER er_rcv = trcv_dtq(DTQ_APLBLE_RX, (intptr_t*)&data, TMO_FEVR);
        assert(er_rcv == E_OK);
//        DBGLOG2("aplble_rx_task() received: (data=0x%08x, len=%d).", data->body, data->length);
        DBGLOG3("aplble_rx_task() received: (data_sv=%d, data_bd=0x%08x, len=%d).", data->service, data->body, data->length);
        
#if 0	// オウム返しテスト
        clr_flg(FLG_APLBLE, ~FLGPTN_MDLBLE_SEND_COMPLETE);
        mdlble_send(data->service, data->body, data->length);
        FLGPTN flgptn = 0;
        ER er = twai_flg(FLG_APLBLE, FLGPTN_MDLBLE_SEND_COMPLETE, TWF_ANDW, &flgptn, 5000);
        assert(er == E_OK);
        mdlble_return_buffer(data);
#endif

#if 1
        switch (data->service) {
        case 1:	// 通信制御
            aplble_process_data_1(data);
            break;
        case 2:	// 本体制御
            aplble_process_data_2(data);
            break;
        case MDLBLE_SERVICE_COMMAND:
            aplble_process_command(data);
            break;
        default:
            assert(false);
            break;
        }
#endif

        if (data) {
            mdlble_return_buffer((const uint8_t*)data);
            data = NULL;
        }
    }
}

void mdlble_callback(int event, intptr_t opt1, intptr_t opt2)
{
    ER er = 0;

    switch (event) {
    case MDLBLE_EVT_INITIALIZE_COMPLETE:
        set_flg(FLG_APLBLE, FLGPTN_MDLBLE_INITIALIZE_COMPLETE);
        DBGLOG0("[CALLBACK]MDLBLE_EVT_INITIALIZE_COMPLETE");
        break;
    case MDLBLE_EVT_START_COMPLETE:
        set_flg(FLG_APLBLE, FLGPTN_MDLBLE_START_COMPLETE);
        DBGLOG0("[CALLBACK]FLGPTN_MDLBLE_START_COMPLETE");
        break;
    case MDLBLE_EVT_RESTART_COMPLETE:
        set_flg(FLG_APLBLE, FLGPTN_MDLBLE_RESTART_COMPLETE);
        DBGLOG0("[CALLBACK]FLGPTN_MDLBLE_RESTART_COMPLETE");
        break;
    case MDLBLE_EVT_CONNECTED:
        break;
    case MDLBLE_EVT_DISCONNECTED:
        break;
    case MDLBLE_EVT_VALIDATED:
        break;
    case MDLBLE_EVT_VALIDATION_FAILED:
        break;
    case MDLBLE_EVT_DATA_RECEIVED:
        set_flg(FLG_APLBLE, FLGPTN_MDLBLE_DATA_RECEIVED);
        DBGLOG0("[CALLBACK]MDLBLE_EVT_DATA_RECEIVED");

        er = tsnd_dtq(DTQ_APLBLE_RX, (intptr_t)opt1, TMO_POL);
        assert(er == E_OK);
        break;
    case MDLBLE_EVT_SEND_COMPLETE:
        set_flg(FLG_APLBLE, FLGPTN_MDLBLE_SEND_COMPLETE);
        DBGLOG1("[CALLBACK]FLGPTN_MDLBLE_SEND_COMPLETE, %d", opt1);
        break;
    default:
        break;
    }
}

void aplble_authenticate(const APLBLE_COMMAND_DATA_T* data)
{
    if ((data->size == sizeof(BLE_PIN)) && (0 == memcmp(data->body, BLE_PIN, sizeof(BLE_PIN)))) {
        clr_flg(FLG_APLBLE, ~FLGPTN_MDLBLE_SEND_COMPLETE);
        mdlble_send2(data->command, (uint8_t*)"OK", 2);
        FLGPTN flgptn = 0;
        ER er = twai_flg(FLG_APLBLE, FLGPTN_MDLBLE_SEND_COMPLETE, TWF_ANDW, &flgptn, 2000);
        assert(er == E_OK);
        DBGLOG0("aplble_authenticate OK");
    } else {
        clr_flg(FLG_APLBLE, ~FLGPTN_MDLBLE_SEND_COMPLETE);
        mdlble_send2(data->command, (uint8_t*)"NG", 2);
        FLGPTN flgptn = 0;
        ER er = twai_flg(FLG_APLBLE, FLGPTN_MDLBLE_SEND_COMPLETE, TWF_ANDW, &flgptn, 2000);
        assert(er == E_OK);
        DBGLOG0("aplble_authenticate FAIL");
    }
}

void aplble_set_phone_wifi(const APLBLE_COMMAND_DATA_T* data)
{
    DBGLOG0("aplble_set_phone_wifi");
    memcpy(s_ssid, &data->body[0], 32);
    memcpy(s_password, &data->body[32], 32);
    DBGLOG2("SSID: %s, password: %s", s_ssid, s_password);
    clr_flg(FLG_APLBLE, ~FLGPTN_MDLBLE_SEND_COMPLETE);
    mdlble_send2(data->command, (uint8_t*)"OK", 2);
    FLGPTN flgptn = 0;
    ER er = twai_flg(FLG_APLBLE, FLGPTN_MDLBLE_SEND_COMPLETE, TWF_ANDW, &flgptn, 2000);
    assert(er == E_OK);
}

void aplble_update_firmware(const APLBLE_COMMAND_DATA_T* data)
{
    DBGLOG0("aplble_update_firmware");
    FLGPTN flgptn = 0;
    ER er = E_OK;

    uint32_t firmware_size = 0;
    int ret = 0;
    if (s_ssid[0] == 0) {
        ret = 0;
        DBGLOG0("Wifi SSID has not been configured");
    } else {
        ret = 1;

        if (ret) {
            DBGLOG0("AP Connect");
            clr_flg(FLG_APLBLE, ~FLGPTN_DRVWIFI_AP_CONNECT_COMPLETE);
            drvwifi_ap_connect2(s_ssid, s_password);
            er = twai_flg(FLG_APLBLE, FLGPTN_DRVWIFI_AP_CONNECT_COMPLETE, TWF_ANDW, &flgptn, 30000);
            assert(er == E_OK);
            if (er != E_OK) {
                ret = 0;
            }
        }

        if (ret) {
            clr_flg(FLG_APLBLE, ~FLGPTN_MDLBLE_SEND_COMPLETE);
            mdlble_send2(data->command, (uint8_t*) "OK", 2);
            FLGPTN flgptn = 0;
            ER er = twai_flg(FLG_APLBLE, FLGPTN_MDLBLE_SEND_COMPLETE, TWF_ANDW, &flgptn, 5000);
            assert(er == E_OK);
        } else {
            clr_flg(FLG_APLBLE, ~FLGPTN_MDLBLE_SEND_COMPLETE);
            mdlble_send2(data->command, (uint8_t*) "NG", 2);
            FLGPTN flgptn = 0;
            ER er = twai_flg(FLG_APLBLE, FLGPTN_MDLBLE_SEND_COMPLETE, TWF_ANDW, &flgptn, 5000);
            assert(er == E_OK);
        }

        if (ret) {
            DBGLOG0("TCP connect");
            clr_flg(FLG_APLBLE, ~FLGPTN_DRVWIFI_TCP_CONNECT_COMPLETE);
            drvwifi_tcp_connect((uint8_t*)SERVER_IP, SERVER_PORT);
            er = twai_flg(FLG_APLBLE, FLGPTN_DRVWIFI_TCP_CONNECT_COMPLETE, TWF_ANDW, &flgptn, 5000);
            assert(er == E_OK);
            if (er != E_OK) {
                ret = 0;
            }
        }

        if (ret) {
            DBGLOG0("TCP send");
            clr_flg(FLG_APLBLE, ~FLGPTN_DRVWIFI_TCP_SEND_COMPLETE);
            drvwifi_send((uint8_t*)"OK", 2);
            er = twai_flg(FLG_APLBLE, FLGPTN_DRVWIFI_TCP_SEND_COMPLETE, TWF_ANDW, &flgptn, 3000);
            assert(er == E_OK);
            if (er != E_OK) {
                ret = 0;
            }
        }

        if (ret) {
            DBGLOG0("TCP Receive Updated message");
            clr_flg(FLG_APLBLE, ~FLGPTN_DRVWIFI_TCP_RECEIVE_COMPLETE);
            drvwifi_receive(s_received_data, 6);
            er = twai_flg(FLG_APLBLE, FLGPTN_DRVWIFI_TCP_RECEIVE_COMPLETE, TWF_ANDW, &flgptn, TMO_FEVR);
            assert(er == E_OK);
            if (er != E_OK) {
                ret = 0;
            } else {
                s_received_data[6] = 0;
                DBGLOG1("Wifi Recv: %s", s_received_data);
                drvwifi_send((uint8_t*) "OK", 2);
                er = twai_flg(FLG_APLBLE, FLGPTN_DRVWIFI_TCP_SEND_COMPLETE, TWF_ANDW, &flgptn, 3000);
                assert(er == E_OK);
            }
        }

        if (ret) {
            DBGLOG0("TCP Receive size");
            clr_flg(FLG_APLBLE, ~FLGPTN_DRVWIFI_TCP_RECEIVE_COMPLETE);
            drvwifi_receive(s_received_data, 4);
            er = twai_flg(FLG_APLBLE, FLGPTN_DRVWIFI_TCP_RECEIVE_COMPLETE, TWF_ANDW, &flgptn, TMO_FEVR);
            assert(er == E_OK);
            if (er != E_OK) {
                ret = 0;
            } else {
                firmware_size = (s_received_data[0] << 24) | (s_received_data[1] << 16) | (s_received_data[2] << 8) | (s_received_data[3]);
                assert(firmware_size < 0x000C0000);
                if (firmware_size >= 0x000C0000) {
                    ret = 0;
                    DBGLOG2("Firmware size ERROR: %d, %08x", firmware_size, firmware_size);
                } else {
                    DBGLOG2("Firmware size: %d, %08x", firmware_size, firmware_size);
                    drvwifi_send((uint8_t*) "OK", 2);
                    er = twai_flg(FLG_APLBLE, FLGPTN_DRVWIFI_TCP_SEND_COMPLETE, TWF_ANDW, &flgptn, 3000);
                    assert(er == E_OK);
                }
            }
        }

        if (ret) {
            DBGLOG0("Start data transfer");
            uint32_t start_address = 0;
            uint32_t receive_length;
            while (start_address < firmware_size) {
                receive_length = firmware_size - start_address;
                if (receive_length > RECEIVE_BUFFER_SIZE) {
                    receive_length = RECEIVE_BUFFER_SIZE;
                }
                DBGLOG1("Wait for: (%d)", receive_length);
                clr_flg(FLG_APLBLE, ~FLGPTN_DRVWIFI_TCP_RECEIVE_COMPLETE);
                drvwifi_receive(s_received_data, receive_length);
                er = twai_flg(FLG_APLBLE, FLGPTN_DRVWIFI_TCP_RECEIVE_COMPLETE, TWF_ANDW, &flgptn, 10000);
                assert(er == E_OK);
                if (er != E_OK) {
                    ret = 0;
                    break;
                } else {
                    DBGLOG1("TCP Receive: (%d)", receive_length);
                    // Write receive buffer to QSPI flash
                    mdlstrg_store_program_write(s_received_data, start_address, receive_length, 0);
                    start_address += receive_length;

                    drvwifi_send((uint8_t*) "OK", 2);
                    er = twai_flg(FLG_APLBLE, FLGPTN_DRVWIFI_TCP_SEND_COMPLETE, TWF_ANDW, &flgptn, 3000);
                    assert(er == E_OK);
                }
            }

            if (!ret) {
                DBGLOG2("Firmware Received Error: (%d/%d)", start_address, firmware_size);

                // TODO: Mark the new firmware as invalid to be erased next reboot
            } else {
                DBGLOG1("Firmware Received: (%d)", firmware_size);

                // Store firmware size and set the new_firmware_flag
                mdlstrg_store_program_delete(firmware_size, 0);
            }
        }
    }
}

void aplble_process_data_1(const MDLBLE_DATA_T* data)
{
    assert(data);

    uint8_t type = data->body[0];
    const uint8_t* other = &(data->body[1]);
    size_t other_len = data->length - 1;
    uint8_t* resp_buf = s_context.buf.response;
    size_t resp_size = 0;
    ER er = E_OK;
    FLGPTN flgptn = {0};

    DBGLOG3("[ch1]type: %02x, other: [%02x, %02x,...]", type, other[0], other[1]);

    switch (type) {
    case 0x01: {	// 接続通知応答
        if (s_context.session.state == SESSION_IDLE) {
            memset(&(s_context.session), 0x0, sizeof(s_context.session));
            s_context.session.state = SESSION_CONNECTED;
        }
        aplui_dbg_led_pattern_off();        //ALL OFF
        aplui_dbg_set_led_pattern(2, 4);	//R
        resp_buf[0] = 0x81;
        resp_size = 1;
        break;
    }
    case 0x02: {	// アイドル応答
        resp_buf[0] = 0x82;
        resp_size = 1;
        break;
    }
    case 0x03: {	// 接続終了
        // セッション情報クリア
        memset(&(s_context.session), 0x0, sizeof(s_context.session));
        s_context.session.state = SESSION_IDLE;

        // BLEモジュール再起動
        DBGLOG0("Disconnect Request(1:0x03) => Restart BLE");
        mdlble_restart();
        er = twai_flg(FLG_APLBLE, FLGPTN_MDLBLE_RESTART_COMPLETE, TWF_ANDW, &flgptn, 2000);
        assert(er == E_OK);

        resp_size = 0;
        break;
    }
    case 0x04: {	// 機器認証開始応答
        // 機器インデックス保存
        s_context.session.auth_index = other[0];
        drvrng_get_random(s_context.session.random, NRANDOM);	// 16バイト乱数

        resp_buf[0] = 0x84;
        memcpy(&(resp_buf[1]), s_context.session.random, NRANDOM);
        resp_size = 1 + NRANDOM;
        break;
    }
    case 0x05: {	// 機器認証要求
        resp_size = aplble_process_data_1_05(resp_buf, other, other_len);
        break;
    }
    case 0x06: {	// アプリ公開鍵登録要求
        resp_size = aplble_process_data_1_06(resp_buf, other, other_len);
        break;
    }
    default:
        assert(false);
        break;
    }

    if (resp_size > 0) {
        mdlble_send(1, resp_buf, resp_size);
    }

    return;
}

int aplble_process_data_1_05(uint8_t* resp_buf, const uint8_t* data, size_t data_len)
{
//    uint8_t resp_code = 1;
    uint8_t resp_code = 0;

    s_context.session.state = SESSION_AUTHENTICATED;
    s_context.cardstate.regcard_authenticated = true;
    
    if (data_len != 292) {
        goto end;
    }

    // 機器インデックス
    int index = s_context.session.auth_index;
    if (index < 0 || index >= 5) {
        goto end;
    }

    const uint8_t* pubkey = NULL;
    if (index == 0) {
        // サーバーキー
        DBGLOG0("Server Key");
        pubkey = TEST_KEY_PUB;
    } else {
        // アプリキー
        DBGLOG0("App Key");
        MDLSTRG_REQUEST_T strg_req = {
            .data_type = MDLSTRG_DATA_TYPE_BLE_APPKEY,
            .request_type = MDLSTRG_REQ_TYPE_READ,
            .data = (intptr_t)&(s_context.session.appkey),
            .size = sizeof(MDLSTRG_DATA_APPKEY_T),
            .opt1 = index - 1,
        };
        mdlstrg_request(&strg_req, mdlstrg_callback);
        ER er = twai_flg(FLG_APLBLE, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 2000);
        assert(er == E_OK);
        pubkey = s_context.session.appkey.pubkey;
    }

#ifdef TEST_NO_VALIDATE_USER_SIGNATURE
    if (index == 0) {
#endif	// TEST_NO_VALIDATE_USER_SIGNATURE
    // 署名チェック
    int valid = cmncrypt_verify_sig(&(data[36]), 256, &(data[0]), 36, pubkey, sizeof(TEST_KEY_PUB));
    if (valid != 1) {
        DBGLOG0("Signature check NG");
        resp_code = 1;
        goto end;
    }
#ifdef TEST_NO_VALIDATE_USER_SIGNATURE
    }
#endif	// TEST_NO_VALIDATE_USER_SIGNATURE
    DBGLOG0("Signature check OK");

    // 乱数チェック
    if (memcmp(&(data[0]), &(s_context.session.random), NRANDOM) != 0) {
        DBGLOG0("Random number check NG");
        resp_code = 1;
        goto end;
    }
    DBGLOG0("Random number check OK");

    // シリアル番号チェック
    if (memcmp(&(data[16]), &(s_context.device_id.serialno), sizeof(s_context.device_id.serialno)) != 0) {
        DBGLOG0("Device serial number check NG");
        resp_code = 1;
        goto end;
    }
    DBGLOG0("Device serial number check OK");

    // すべて正常
    resp_code = 0;
    
    // 状態更新
    s_context.session.state = SESSION_AUTHENTICATED;
    if (index == 0) {	// サーバー認証
        memcpy(&(s_context.session.permissions), &(data[32]), sizeof(uint32_t));
    }

end:
    resp_buf[0] = 0x85;
    resp_buf[1] = resp_code;
    return 2;	// respサイズ
}
    
int aplble_process_data_1_06(uint8_t* resp_buf, const uint8_t* data, size_t data_len)
{
    DBGLOG1("aplble_process_data_1_06(len=%d)", data_len);

    uint8_t resp_code = 1;
    uint8_t peer_index = 0;

    // 状態チェック
    if ((s_context.session.state != SESSION_AUTHENTICATED) ||		// 認証済みでない
        (s_context.session.auth_index != 0) ||						// サーバー認証でない
        !(s_context.session.permissions | PERMISSION_PAIRING)) {	// ペアリング許可がない
        DBGLOG0("Invalid session");
        resp_code = 1;
        goto end;
    }

    if (data_len != sizeof(MDLSTRG_DATA_APPKEY_T) - 2) {	// 末尾のパディング分を引く
        DBGLOG1("data_len != %d", sizeof(MDLSTRG_DATA_APPKEY_T) - 2);
        resp_code = 1;
        goto end;
    }

    const MDLSTRG_DATA_APPKEY_T* appkey = (const MDLSTRG_DATA_APPKEY_T*)data;

    // 鍵チェック
    int error = cmncrypt_verify_pubkey(appkey->pubkey, sizeof(appkey->pubkey));
    if (error != 0) {
        DBGLOG0("cmncrypt_verify_pubkey() error");
        resp_code = 1;
        goto end;
    }

    // ペアリング情報保存
    MDLSTRG_REQUEST_T strg_req = {
        .data_type = MDLSTRG_DATA_TYPE_BLE_APPKEY,
        .request_type = MDLSTRG_REQ_TYPE_WRITE,
        .data = (intptr_t)appkey,
        .size = sizeof(MDLSTRG_DATA_APPKEY_T),
    };
    mdlstrg_request(&strg_req, mdlstrg_callback);
    ER er = twai_flg(FLG_APLBLE, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 2000);
    assert(er == E_OK);

    // すべて正常
    DBGLOG0("aplble_process_data_1_06() end successfully");
    resp_code = 0;
    peer_index = 1;

end:
    resp_buf[0] = 0x86;
    resp_buf[1] = resp_code;
    resp_buf[2] = peer_index;
    return 3;	// respサイズ
}

// 本体制御(チャネル2)
void aplble_process_data_2(const MDLBLE_DATA_T* data)
{
    assert(data);

//    int ble_process = true;
    uint8_t type = data->body[0];
    const uint8_t* other = &(data->body[1]);
    size_t other_len = data->length - 1;
    uint8_t* resp_buf = s_context.buf.response;
    uint8_t* tmp_resp_buf = s_context.buf.response;
    size_t resp_size = 0;
    MDLSTRG_REQUEST_T strg_req = {0};
    int indx;
    int cs_indx;
    DB_HEADER_T cs_header = {0};
    
//    cs_header.data_id  = 0x0;			// データID
//    cs_header.data_len = 0x0;			// データ長
//    cs_header.db_state = CARD_INITIAL;	// データブロック状態
    if(card_flg == false){
        cashe_meta[cs_indx].db_state = CARD_INITIAL;	// データブロック状態;
    }
    cashe_meta[cs_indx].slotno   = 1;		// 登録スロット名
    cashe_meta[cs_indx].name_len = 8;		// 名前長さ

    
    
    ER er = E_OK;
    
    APLEVT_EXTRA_DATA_T extra_data;
    
    DBGLOG3("[ch2]type: %02x, other: [%02x, %02x,...]", type, other[0], other[1]);

    DBGLOG3("resp_buf[1]:%x resp_buf[2]:%x resp_buf[2]:%x", resp_buf[0], resp_buf[1], resp_buf[2]);

    DBGLOG2("init_cashe_meta[cs_indx].name_len: %d, other[1]:%d", cashe_meta[cs_indx].name_len, other[1]);
    
    DBGLOG1("cashe_meta[0].card[0]: %d", cashe_meta[0].card[0]);
    
    switch (type) {
    case 0x01: {	// 本体ステータス問い合わせ
        resp_buf[0] = 0x81;
        if (s_context.session.state == SESSION_AUTHENTICATED) {		// 認証済み
            // 保持している状態を返す
            resp_buf[1] = s_context.device_status.status1;
            resp_buf[2] = s_context.device_status.status2;
            DBGLOG2("0x81_resp_buf[1]:%x_resp_buf[2]:%x", resp_buf[1], resp_buf[2]);
            
        } else {	// 認証済みでなければ0固定
            resp_buf[1] = 0x00;
            resp_buf[2] = 0x00;
        }
        resp_size = 3;
        break;
    }
    case 0x02: {	// 血流登録状況問い合わせ
        resp_buf[0] = 0x82;
        if (s_context.session.state == SESSION_AUTHENTICATED) {		// 認証済み
            int count = 0;
            int respbuf_pos = 2;
            for (int i = 0; i < MDLSTRG_MAX_NDATA_AUTH; i++) {
                // 認証メタ情報取得
                strg_req.data_type = MDLSTRG_DATA_TYPE_AUTH_META;
                strg_req.request_type = MDLSTRG_REQ_TYPE_READ;
                strg_req.data = (intptr_t)&(resp_buf[respbuf_pos]);
                strg_req.size = sizeof(MDLSTRG_DATA_AUTH_META_T);
                strg_req.opt1 = i;

                mdlstrg_request(&strg_req, mdlstrg_callback);
                ER er = twai_flg(FLG_APLBLE, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 2000);
                assert(er == E_OK);

                if (s_context.strg_result.opt1) {
                    respbuf_pos += sizeof(MDLSTRG_DATA_AUTH_META_T);
                    count++;
                }
            }
            resp_buf[1] = count;
            resp_size = respbuf_pos;
            for(int x = 0; x < 9; x++){
                DBGLOG3("0x02 resp_buf[i]:%x resp_buf[i+1]:%x resp_buf[i+2]:%x", resp_buf[x], resp_buf[x+1], resp_buf[x+2]);
                x = x + 2;
            }
            
        } else {
            // 認証済みでなければ0件固定
            resp_buf[1] = 0x0;
            resp_size = 2;
        }
        break;
    }
    case 0x03: {	// 血流登録開始要求
        resp_buf[0] = 0x83;
        if (s_context.session.state == SESSION_AUTHENTICATED) {		// 認証済み
            // 登録開始処理
            DBGLOG0("SESSION_AUTHENTICATED successfully");
            memcpy(&(s_context.reg.auth_meta), other, sizeof(s_context.reg.auth_meta));
            s_context.reg.pending = true;
            s_context.reg.extra_data.reg_prep_req.index = s_context.reg.auth_meta.slotno - 1;
//            aplevt_send_event(APLEVT_REGISTRATION_PREPARE_REQ, 0, &(s_context.reg.extra_data), s_event_dest);
            extra_data.reg_prep_req.index = ble_process;
            aplevt_send_event(APLEVT_VAREG_REQ, 0, &extra_data, aplauth_event);
            
            // 状態を認証中に変更
            s_context.device_status.status1 = 0x2;
            DBGLOG1("0x83_resp_buf[1]", resp_buf[1]);
            resp_buf[1] = 0x00;
            
#if 0
            // 認証メタ情報取得
            strg_req.data_type = MDLSTRG_DATA_TYPE_AUTH_META;
            strg_req.request_type = MDLSTRG_REQ_TYPE_WRITE;
            strg_req.data = (intptr_t)&(s_context.reg.auth_meta);
            strg_req.size = sizeof(MDLSTRG_DATA_AUTH_META_T);
            strg_req.opt1 = s_context.reg.auth_meta.slotno - 1;
            mdlstrg_request(&strg_req, mdlstrg_callback);
            ER er = twai_flg(FLG_APLBLE, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 2000);
            assert(er == E_OK);
#endif
        } else {
            // 認証済みでなければエラー
            resp_buf[1] = 0x01;
            DBGLOG0("SESSION_AUTHENTICATED error");
        }
        resp_size = 2;
        break;
    }
    case 0x04: {	// 血流登録終了要求
        resp_buf[0] = 0x84;
        if (s_context.session.state == SESSION_AUTHENTICATED) {		// 認証済み
            resp_buf[1] = 0x00;
        } else {
            // 認証済みでなければエラー
            resp_buf[1] = 0x01;
        }
        resp_size = 2;
        break;
    }
    case 0x05: {	// 血流登録削除要求
        resp_buf[0] = 0x85;
        DBGLOG1("VA Delete Req (slot=%d)", other[0]);
        if (s_context.session.state == SESSION_AUTHENTICATED) {		// 認証済み
            // メタ情報削除
            MDLSTRG_REQUEST_T strg_req = {
                .data_type = MDLSTRG_DATA_TYPE_AUTH,
                .request_type = MDLSTRG_REQ_TYPE_DELETE,
                .opt1 = other[0] - 1,	// index
            };
            mdlstrg_request(&strg_req, mdlstrg_callback);
            ER er = twai_flg(FLG_APLBLE, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 2000);
            assert(er == E_OK);
        } else {
            // 認証済みでなければエラー
            DBGLOG0("Error: Non-authorized session");
            resp_buf[1] = 0x01;
        }
        resp_size = 2;
        break;
    }
    case 0x06: {	// 解錠要求
        resp_buf[0] = 0x86;
        if (s_context.session.state == SESSION_AUTHENTICATED) {		// 認証済み
            resp_buf[1] = 0x00;
        } else {
            // 認証済みでなければエラー
            resp_buf[1] = 0x01;
        }
        resp_size = 2;
        break;
    }
    case 0x07: {	// 解錠要求
        resp_buf[0] = 0x87;
        if (s_context.session.state == SESSION_AUTHENTICATED) {		// 認証済み
            resp_buf[1] = 0x00;
        } else {
            // 認証済みでなければエラー
            resp_buf[1] = 0x01;
        }
        resp_size = 2;
        break;
    }
    case 0x08: {	// データバックアップ要求
        resp_buf[0] = 0x88;
        if (s_context.session.state == SESSION_AUTHENTICATED) {		// 認証済み
            resp_buf[1] = 0x00;
        } else {
            // 認証済みでなければエラー
            resp_buf[1] = 0x01;
        }
        resp_size = 2;
        break;
    }
    case 0x09: {	// データリストア要求
        resp_buf[0] = 0x89;
        if (s_context.session.state == SESSION_AUTHENTICATED) {		// 認証済み
            resp_buf[1] = 0x00;
        } else {
            // 認証済みでなければエラー
            resp_buf[1] = 0x01;
        }
        resp_size = 2;
        break;
    }
    case 0x0b: {	// カード認証開始要求
        DBGLOG1("Card Auth Start Req (type=%d)", other[0]);
        resp_size = 0;
        resp_buf[resp_size] = 0x8b;
        resp_size += 1;
        if (s_context.session.state == SESSION_AUTHENTICATED) {		// 認証済み
            // TODO: カード認証開始処理
            // TODO: タイプ=0は登録抹消カード
            DBGLOG0("APLEVT_ICC_AUTH_REQ");

            extra_data.reg_prep_req.index = APLICC_CARD_REGIST;
            extra_data.reg_prep_req.timeout = 30000;
            DBGLOG2("extra_data %d %d", extra_data.reg_prep_req.index, extra_data.reg_prep_req.timeout);
            aplevt_send_event(APLEVT_ICC_AUTH_REQ, 0, &extra_data, s_event_dest);
            
            // 動作中状態設定
            s_context.device_status.status1 = 0x03;  //カード認証中
            s_context.cardstate.op = CARDOP_AUTH;
            s_context.cardstate.op_slot = other[0];

            // 応答
            resp_buf[resp_size] = 0x00;
            resp_buf[resp_size] = 0x00;
            resp_size += 1;
        } else {
            // 認証済みでなければエラー
            DBGLOG0("Non-authorized session");
            resp_buf[resp_size] = 0xff;
            resp_size += 1;
        }
        break;
    }
    case 0x0c: {	// カード登録状況問い合わせ
        DBGLOG0("Card Query Req");
        resp_size = 0;
        resp_buf[resp_size] = 0x8c;
        resp_size += 1;
        cs_indx = 0;
        aplui_dbg_led_pattern_off();        //ALL OFF
        aplui_dbg_set_led_pattern(2, 4);	//G
        
        DBGLOG1("resp_buf[resp_size]: x= 0x%x", resp_buf[0]);
        if ((s_context.session.state == SESSION_AUTHENTICATED) &&
            s_context.cardstate.regcard_authenticated) {		// 認証済み
#ifdef TEST_USE_DUMMY_CARD_DATA
            resp_buf[resp_size] = TEST_CARD_DATA_NUM;
            resp_size += 1;
            memcpy(&(resp_buf[resp_size]), TEST_CARD_DATA, sizeof(TEST_CARD_DATA) * TEST_CARD_DATA_NUM);
            resp_size += sizeof(TEST_CARD_DATA);
            // TODO: カード登録情報を読み出し
#endif	// TEST_USE_DUMMY_CARD_DATA
                
            int count = 0;
            int respbuf_pos = 2;
            for (int i = 0; i < MDLSTRG_MAX_NDATA_CARD; i++) {
                DBGLOG1("MDLSTRG_MAX_NDATA_CARD i= %d", i);
                // 認証メタ情報取得
                strg_req.data_type = MDLSTRG_DATA_TYPE_CARD_META;
                strg_req.request_type = MDLSTRG_REQ_TYPE_READ;
                strg_req.data = (intptr_t)&(tmp_resp_buf[respbuf_pos]);
                strg_req.size = sizeof(MDLSTRG_DATA_CHASH_META_T);
                strg_req.opt1 = i;
                mdlstrg_request(&strg_req, mdlstrg_callback);
                ER er = twai_flg(FLG_APLBLE, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 2000);
                assert(er == E_OK);

                if (s_context.strg_result.opt1) {
                    respbuf_pos += sizeof(MDLSTRG_DATA_CHASH_META_T);
                    count++;
                }
            }
            resp_buf[1] = count;
            resp_size = respbuf_pos;
            for(int x = 146; x < 156; x++){
                DBGLOG3("type:0c tmp_resp_buf[i]:%x resp_buf[i+1]:%x resp_buf[i+2]:%x", tmp_resp_buf[x], tmp_resp_buf[x+1], tmp_resp_buf[x+2]);
                x = x + 2;
            }
            for(int x = 2; x < 150; x++){
                //DB_HEADER_Tのサイズ分(12バイト)シフト
                    resp_buf[x] = tmp_resp_buf[x + 12];
            }
            //カード番号固定値入力
            resp_buf[136] = 0x63;
            resp_buf[137] = 0x61;
            resp_buf[138] = 0x72;
            resp_buf[139] = 0x64;
            resp_buf[140] = 0x30;
            resp_buf[141] = 0x30;
            resp_buf[142] = 0x31;
            for(int x = 0; x < 9; x++){
                DBGLOG3("type:0c resp_buf[i]:%x resp_buf[i+1]:%x resp_buf[i+2]:%x", resp_buf[x], resp_buf[x+1], resp_buf[x+2]);
                x = x + 2;
            }
            int x = sizeof(cs_header);
            DBGLOG1("sizeof(cs_header): x= %d", x);
            //RAMに退避 + 2:カードメタ情報のみ
            cashe_meta[cs_indx].data_id  = 0x0000;			// データID;
            cashe_meta[cs_indx].data_len = 0x0000;			// データ長;
            cashe_meta[cs_indx].slotno   = resp_buf[2];		// 登録スロット名
            cashe_meta[cs_indx].name_len = resp_buf[3];		// 名前長さ
            DBGLOG1("resp_buf[resp_size]: x= 0x%x", resp_buf[0]);
            DBGLOG1("cashe_meta[cs_indx].slotno: x= 0x%x", cashe_meta[cs_indx].slotno);
            for(int y=0; y < 9; y++){
                DBGLOG1("cashe_meta[cs_indx].name[y]: x= 0x%x", cashe_meta[cs_indx].name[y]);
            }
                
        } else {
            // 認証済みでなければエラー
            DBGLOG0("Non-authorized session");
            resp_buf[resp_size] = 0xff;
            resp_size += 1;
        }
        break;
    }
    case 0x0d: {	// カード登録開始要求
        DBGLOG1("Card Register Req (slot=%d)", other[0]);
        indx = other[0] - 1;
        DBGLOG1("[ch2]type: %02x]", type);
        for(int y = 0; y < 9; y++){
            DBGLOG3("other: [%02x, %02x, %02x,...]", other[y], other[y+1], other[y+2]);
            y = y + 2;
        }
//        cmndbg_hexdump(&(s_context.reg.card_meta), sizeof(s_context.reg.card_meta), "0c_CARD_META flash");
        
        resp_size = 0;
        resp_buf[resp_size] = 0x8d;
        resp_size += 1;
        if (s_context.session.state == SESSION_AUTHENTICATED) {		// 認証済み
            if (other[0] == 0) {	// slot=0は登録・抹消カード
                // TODO: 登録・抹消カード登録開始処理
                // 動作中状態セット
                s_context.cardstate.op = CARDOP_REGISTER;
                s_context.cardstate.op_slot = other[0];
                // 応答
                resp_buf[resp_size] = 0x00;
                resp_size += 1;
            } else {				// それ以外はDroomカード
                // Droomカード登録の場合は事前に登録・抹消カードでの認証が必要
                if (s_context.cardstate.regcard_authenticated) {
                    cs_indx = 0;
                    card_flg = true;    //カード登録フラグ
                    // 登録開始処理
                    DBGLOG0("CARD_SESSION_AUTHENTICATED successfully");
                    DBGLOG1("cs_indx: %d", cs_indx);
                    //データブロックカード状態更新
                    cashe_meta[cs_indx].db_state = CARD_ENTRY;	// データブロック状態;
                    cashe_meta[cs_indx].slotno   = other[0] - 1;			// 登録スロット名
                    cashe_meta[cs_indx].name_len = other[1];		// 名前長さ
                    for(int m = 0 ; m < 4; m++){
                        cashe_meta[cs_indx].card[m]  = 1;		// 名前長さ
                    }
                    DBGLOG2("0d_cashe_meta[cs_indx].name_len: %d, other[1]:%d", cashe_meta[cs_indx].name_len, other[1]);
                    
                    memcpy(&cashe_meta[cs_indx].name[0], (&other[4]), 32);
                    s_context.reg.pending = true;
                    s_context.reg.extra_data.reg_prep_req.index = s_context.reg.card_meta[indx].slotno - 1;
                    
                    extra_data.reg_prep_req.index = APLICC_CARD_DROOM1;
                    extra_data.reg_prep_req.timeout = 30000;
                    extra_data.reg_prep_req.card_index = 1;
                    memcpy(extra_data.reg_prep_req.user_name, "たかし", sizeof(extra_data.reg_prep_req.user_name));
                    aplevt_send_event(APLEVT_ICC_REGIST_REQ, 0, &extra_data, s_event_dest);
                    
                    s_context.device_status.status1 = 0x03;  //カード認証中
                    s_context.cardstate.op = CARDOP_REGISTER;
                    s_context.cardstate.op_slot = other[0];
                    DBGLOG1("0d_cashe_meta[cs_indx].db_state: x= 0x%x", cashe_meta[cs_indx].db_state);
                    for(int y=0; y < 9; y++){
                        DBGLOG1("0d_cashe_meta[cs_indx].name[y]: x= 0x%x", cashe_meta[cs_indx].name[y]);
                    }
                    // 応答
                    resp_buf[resp_size] = 0x00;
                    resp_size += 1;
                } else {
                    // 認証済みでなければエラー
                    DBGLOG0("Non-authorized session");
                    resp_buf[resp_size] = 0xff;
                    resp_size += 1;
                }
            }
        } else {
            // 認証済みでなければエラー
            DBGLOG0("Non-authorized session");
            resp_buf[resp_size] = 0xff;
            resp_size += 1;
        }
        break;
    }
    case 0x0e: {	// カード登録終了要求
        DBGLOG0("Card Register End Req");
        cashe_meta[cs_indx].name_len = other[1];		// 名前長さ
        resp_size = 0;
        resp_buf[resp_size] = 0x8e;
        resp_size += 1;
        if (s_context.session.state == SESSION_AUTHENTICATED) {		// 認証済み
            if (s_context.cardstate.op == CARDOP_REGISTER) {
                // TODO: カード登録終了
                // TODO: 読み取り中であればキャンセル
                // TODO: Droomカード登録開始処理
                // D-roomカード保存
                MDLSTRG_REQUEST_T strg_req = {
                    .data_type = MDLSTRG_DATA_TYPE_CARD_META,
                    .request_type = MDLSTRG_REQ_TYPE_WRITE,
                    .data = (intptr_t)&(cashe_meta[0]),
                    .size = sizeof(cashe_meta[0]),
                    .opt1 = 0,
                };
                mdlstrg_request(&strg_req, mdlstrg_callback);
                ER er = twai_flg(FLG_APLBLE, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 2000);
                assert(er == E_OK);
                // 応答
                resp_buf[resp_size] = s_context.cardstate.op_result;
                resp_size += 1;
                // 動作中状態クリア
                s_context.cardstate.op = CARDOP_NONE;
                s_context.cardstate.op_result = 0;
            } else {
                // 認証済みでなければエラー
                DBGLOG0("Non-authorized session");
                resp_buf[resp_size] = 0xff;
                resp_size += 1;
            }
        } else {
            // 認証済みでなければエラー
            DBGLOG0("Non-authorized session");
            resp_buf[resp_size] = 0xff;
            resp_size += 1;
        }
        break;
    }
    case 0x0f: {	// カード登録削除要求
        DBGLOG1("Card Delete Req (slot=%d)", other[0]);
        resp_size = 0;
        resp_buf[resp_size] = 0x8f;
        resp_size += 1;
        if ((s_context.session.state == SESSION_AUTHENTICATED) &&
            s_context.cardstate.regcard_authenticated) {		// 認証済み
            // TODO: カード削除処理
            extra_data.reg_prep_req.card_index = 1;
            aplevt_send_event(APLEVT_ICC_DELETE_REQ, 0, &extra_data, s_event_dest);
            // 応答
            resp_buf[resp_size] = 0x00;
            resp_size += 1;
        } else {
            // 認証済みでなければエラー
            DBGLOG0("Non-authorized session");
            resp_buf[resp_size] = 0xff;
            resp_size += 1;
        }
        break;
    }
    case 0x10: {	// カード認証終了要求
        DBGLOG0("Card Auth End Req");
        resp_size = 0;
        resp_buf[resp_size] = 0x90;
        resp_size += 1;
        if (s_context.session.state == SESSION_AUTHENTICATED) {		// 認証済み
            if (s_context.cardstate.op == CARDOP_AUTH) {
                // TODO: カード読み取り終了
                // TODO: 読み取り中であればキャンセル

                // 登録・抹消カードの認証
                if (s_context.cardstate.op_slot == 0) {
                    s_context.cardstate.regcard_authenticated = true;
                }

                // 応答
                resp_buf[resp_size] = s_context.cardstate.op_result;
                resp_size += 1;

                // 動作中状態クリア
                s_context.cardstate.op = CARDOP_NONE;
                s_context.cardstate.op_result = 0;
            } else {
                // 認証済みでなければエラー
                DBGLOG0("Non-authorized session");
                resp_buf[resp_size] = 0xff;
                resp_size += 1;
            }
        } else {
            // 認証済みでなければエラー
            DBGLOG0("Non-authorized session");
            resp_buf[resp_size] = 0xff;
            resp_size += 1;
        }
        break;
    }
    default:
        assert(false);
        break;
    }

    if (resp_size > 0) {
        mdlble_send(2, resp_buf, resp_size);
    }

    return;
}

/*
 * ストレージミドルコールバック
 */
void mdlstrg_callback(int event, intptr_t opt1, intptr_t opt2)
{
    //DBGLOG1("mdlstrg_callback: e=%d", event);
    switch(event) {
    case MDLSTRG_EVT_REQUEST_COMPLETE:
//        DBGLOG0("mdlstrg_callback: MDLSTRG_EVT_REQUEST_COMPLETE");
        set_flg(FLG_APLBLE, FLGPTN_MDLSTRG_REQUEST_COMPLETE);
        break;
    default:
        assert(false);
        break;
    }

    s_context.strg_result.opt1 = opt1;
    s_context.strg_result.opt2 = opt2;
}

void aplble_process_command(const MDLBLE_DATA_T* data)
{
    APLBLE_COMMAND_DATA_T aplble_command = {0};
    aplble_command.command = (data->body[0] << 8) | data->body[1];
    aplble_command.size = (uint8_t)(data->length - 2);
    aplble_command.body = (uint8_t*)&data->body[2];
    DBGLOG2("Command received %04x, size %d", aplble_command.command, aplble_command.size);

    switch (aplble_command.command) {
    case APLBLE_CMD_AUTHENTICATE:
        aplble_authenticate(&aplble_command);
        break;
    case APLBLE_CMD_SET_PHONE_WIFI:
        aplble_set_phone_wifi(&aplble_command);
        break;
    case APLBLE_CMD_UPDATE_FIRMWARE:
        aplble_update_firmware(&aplble_command);
        break;
    default:
        assert(false);
        break;
    }
}

void drvwifi_callback(int32_t evt, int32_t error, intptr_t opt)
{
    DBGLOG3("drvwifi_callback(evt=%d,error=%d,opt=0x%08x)", evt, error, opt);

    switch (evt) {
    case DRVWIFI_EVT_INITIALIZE_COMPLETE:
        set_flg(FLG_APLBLE, FLGPTN_DRVWIFI_INITIALIZE_COMPLETE);
        break;
    case DRVWIFI_EVT_AP_CONNECT_COMPLETE:
        set_flg(FLG_APLBLE, FLGPTN_DRVWIFI_AP_CONNECT_COMPLETE);
        break;
    case DRVWIFI_EVT_TCP_CONNECT_COMPLETE:
        set_flg(FLG_APLBLE, FLGPTN_DRVWIFI_TCP_CONNECT_COMPLETE);
        break;
    case DRVWIFI_EVT_TCP_SEND_COMPLETE:
        set_flg(FLG_APLBLE, FLGPTN_DRVWIFI_TCP_SEND_COMPLETE);
        break;
    case DRVWIFI_EVT_TCP_RECEIVE_COMPLETE:
        set_flg(FLG_APLBLE, FLGPTN_DRVWIFI_TCP_RECEIVE_COMPLETE);
        break;
    case DRVWIFI_EVT_TCP_SERVER_COMPLETE:
        set_flg(FLG_APLBLE, FLGPTN_DRVWIFI_TCP_SERVER_COMPLETE);
        break;
    default:
        assert(false);
        break;
    }
}

void mdlstrg_store_program_write(uint8_t* data, uint32_t address, size_t size, int index)
{
    DBGLOG0("mdlstrg_store_program_write");
    ER er;

    assert(data);

    MDLSTRG_REQUEST_T UPDATE_REQ = {
        .data_type = MDLSTRG_DATA_TYPE_STORE_PROGRAM,
        .request_type = MDLSTRG_REQ_TYPE_WRITE,
        .data = (intptr_t)data,
        .size = size,
        .opt1 = index,
        .opt2 = address,
    };
    clr_flg(FLG_APLBLE, ~FLGPTN_MDLSTRG_REQUEST_COMPLETE);
    mdlstrg_request(&UPDATE_REQ, mdlstrg_callback);
    er = twai_flg(FLG_APLBLE, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 10000);
    assert(er == E_OK);
}

void mdlstrg_store_program_delete(size_t size, int index)
{
    DBGLOG0("mdlstrg_store_program_delete");
    ER er;

    MDLSTRG_REQUEST_T UPDATE_REQ = {
        .data_type = MDLSTRG_DATA_TYPE_STORE_PROGRAM,
        .request_type = MDLSTRG_REQ_TYPE_DELETE,
        .size = size,
        .opt1 = index,
    };
    clr_flg(FLG_APLBLE, ~FLGPTN_MDLSTRG_REQUEST_COMPLETE);
    mdlstrg_request(&UPDATE_REQ, mdlstrg_callback);
    er = twai_flg(FLG_APLBLE, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &(FLGPTN){0}, TMO_FEVR);
    assert(er == E_OK);
}
