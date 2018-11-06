/*
 * VA-X UIアプリケーション
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/01/25 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

// ソースの対になるヘッダ
#include "aplui.h"

// 標準C

// OS関係
#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"

// その他ヘッダ
#include "drvled.h"
#include "drvbuz.h"
#include "drvpbt.h"

//
// マクロ
//

// デバッグログ
#if 1
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[APLUI]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[APLUI]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[APLUI]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[APLUI]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

/*
 * 定数
 */
// イベントフラグ
static const FLGPTN FLGPTN_LEDPATTERN_CHANGED =	(0x1 << 1);
static const FLGPTN FLGPTN_BUZPATTERN_CHANGED =	(0x1 << 2);

// セマフォタイムアウト
static const RELTIM LOCK_TIMEOUT = 1000;

/*
 * LEDパターン関係
 */
// LEDパターン番号
enum {
    LEDPATTERN_NONE = 0,
    LEDPATTERN_OFF,
    LEDPATTERN_ON,
    LEDPATTERN_BLINK_1HZ,
    LEDPATTERN_BLINK_2HZ,
    LEDPATTERN_BLINK_4HZ,
    LEDPATTERN_BLINK_8HZ,
    LEDPATTERN_BLINK2_2HZ,
    LEDPATTERN_BLINK3,
    LEDPATTERN_1000MS,
    LEDPATTERN_2000MS,
    LEDPATTERN_3000MS,
    LEDPATTERN_ON_AFTER_3000MS,
    LEDPATTERN_BLINK_8HZ_1S,
    NUM_LEDPATTERN,
};

// LEDアクション
enum {
    LEDACTION_NOCHANGE = 0,
    LEDACTION_TURNOFF,
    LEDACTION_TURNON,
    LEDACTION_STOP,
};

typedef struct {
    int action;
    int duration;
} LEDACTION_T;

// LEDパターン
typedef struct {
    int pattern;
    const LEDACTION_T* sequence;
    size_t sequence_len;
    int repeats;
} LED_PATTERN_T;

// シーケンス定義
static const LEDACTION_T LEDSEQ_OFF[] =			{ { LEDACTION_TURNOFF, TMO_FEVR } };
static const LEDACTION_T LEDSEQ_ON[] =			{ { LEDACTION_TURNON, TMO_FEVR } };
static const LEDACTION_T LEDSEQ_BLINK_1HZ[] =	{ { LEDACTION_TURNON, 500 }, { LEDACTION_TURNOFF, 500 } };
static const LEDACTION_T LEDSEQ_BLINK_2HZ[] =	{ { LEDACTION_TURNON, 250 }, { LEDACTION_TURNOFF, 250 } };
static const LEDACTION_T LEDSEQ_BLINK_4HZ[] =	{ { LEDACTION_TURNON, 125 }, { LEDACTION_TURNOFF, 125 } };
static const LEDACTION_T LEDSEQ_BLINK_8HZ[] =	{ { LEDACTION_TURNON, 62 }, { LEDACTION_TURNOFF, 62 } };
static const LEDACTION_T LEDSEQ_BLINK2_2HZ[] =	{ { LEDACTION_TURNON, 50 }, { LEDACTION_TURNOFF, 450} };
static const LEDACTION_T LEDSEQ_BLINK3[] =		{ { LEDACTION_TURNON, 125 }, { LEDACTION_TURNOFF, 125 }, { LEDACTION_TURNON, 125 }, { LEDACTION_TURNOFF, 125 }, { LEDACTION_TURNOFF, 1000 } };
static const LEDACTION_T LEDSEQ_1000MS[] =		{ { LEDACTION_TURNON, 1000 }, { LEDACTION_TURNOFF, -1} };
static const LEDACTION_T LEDSEQ_2000MS[] =		{ { LEDACTION_TURNON, 2000 }, { LEDACTION_TURNOFF, -1} };
static const LEDACTION_T LEDSEQ_3000MS[] =		{ { LEDACTION_TURNON, 3000 }, { LEDACTION_TURNOFF, -1} };
static const LEDACTION_T LEDSEQ_ON_AFTER_3000MS[] =		{ { LEDACTION_TURNOFF, 3000 }, { LEDACTION_TURNON, -1} };

// パターンテーブル
static const LED_PATTERN_T LED_PATTERNS[] = {
    { LEDPATTERN_NONE,	 			NULL,					0,													0	},
    { LEDPATTERN_OFF,	 			LEDSEQ_OFF,				sizeof(LEDSEQ_OFF)/sizeof(LEDACTION_T),				0	},
    { LEDPATTERN_ON,	 			LEDSEQ_ON,				sizeof(LEDSEQ_ON)/sizeof(LEDACTION_T),				0	},
    { LEDPATTERN_BLINK_1HZ,			LEDSEQ_BLINK_1HZ,		sizeof(LEDSEQ_BLINK_1HZ)/sizeof(LEDACTION_T),		0	},
    { LEDPATTERN_BLINK_2HZ,			LEDSEQ_BLINK_2HZ,		sizeof(LEDSEQ_BLINK_2HZ)/sizeof(LEDACTION_T),		0	},
    { LEDPATTERN_BLINK_4HZ,			LEDSEQ_BLINK_4HZ,		sizeof(LEDSEQ_BLINK_4HZ)/sizeof(LEDACTION_T),		0	},
    { LEDPATTERN_BLINK_8HZ,			LEDSEQ_BLINK_8HZ,		sizeof(LEDSEQ_BLINK_8HZ)/sizeof(LEDACTION_T),		0	},
    { LEDPATTERN_BLINK2_2HZ,		LEDSEQ_BLINK2_2HZ,		sizeof(LEDSEQ_BLINK2_2HZ)/sizeof(LEDACTION_T),		0	},
    { LEDPATTERN_BLINK3,			LEDSEQ_BLINK3,			sizeof(LEDSEQ_BLINK3)/sizeof(LEDACTION_T),			0	},
    { LEDPATTERN_1000MS,			LEDSEQ_1000MS,			sizeof(LEDSEQ_1000MS)/sizeof(LEDACTION_T),			0	},
    { LEDPATTERN_2000MS,			LEDSEQ_1000MS,			sizeof(LEDSEQ_1000MS)/sizeof(LEDACTION_T),			0	},
    { LEDPATTERN_3000MS,			LEDSEQ_3000MS,			sizeof(LEDSEQ_3000MS)/sizeof(LEDACTION_T),			0	},
    { LEDPATTERN_ON_AFTER_3000MS,	LEDSEQ_ON_AFTER_3000MS,	sizeof(LEDSEQ_ON_AFTER_3000MS)/sizeof(LEDACTION_T),	0	},
    { LEDPATTERN_BLINK_8HZ_1S,		LEDSEQ_BLINK_8HZ,		sizeof(LEDSEQ_BLINK_8HZ)/sizeof(LEDACTION_T),		8	},
};

// LEDのコンテキスト
typedef struct {
    int pattern;
    int seq_pos;
    int repeats;
} LED_CONTEXT_T;

// LEDテーブル
static struct {
    const int led;
    LED_CONTEXT_T context;
    int next_action_in_ms;
} s_led_table[] = {
    { DRVLED_LED_FRONT_RED,		{.pattern = LEDPATTERN_OFF},	0	},
    { DRVLED_LED_FRONT_BLUE,	{.pattern = LEDPATTERN_OFF},	0	},
    { DRVLED_LED_FRONT_GREEN,	{.pattern = LEDPATTERN_ON},		0	},
    { DRVLED_LED_FRONT_YGREEN,	{.pattern = LEDPATTERN_OFF},	0	},
    { DRVLED_LED_REAR_1,		{.pattern = LEDPATTERN_OFF},	0	},
};

/*
 * ブザーパターン関係
 */
// 周波数テーブル(x10)
#define NUM_NOTES	255
static const int FREQ_TBL[NUM_NOTES] = {
//	C		C#		D		D#		E		F		F#		G		G#		A		A#		B
    82,		87,		92,		97,		103,	109,	116,	122,	130,	138,	146,	154,	// -1
    164,	173,	184,	194,	206,	218,	231,	245,	260,	275,	291,	309,	// 0
    327,	346,	367,	389,	412,	437,	462,	490,	519,	550,	583,	617,	// 1
    654,	693,	734,	778,	824,	873,	925,	980,	1038,	1100,	1165,	1235,	// 2
    1308,	1386,	1468,	1556,	1648,	1746,	1850,	1960,	2077,	2200,	2331,	2469,	// 3
    2616,	2772,	2937,	3111,	3296,	3492,	3700,	3920,	4153,	4400,	4662,	4939,	// 4
    5233,	5544,	5873,	6223,	6593,	6985,	7400,	7840,	8306,	8800,	9323,	9878,	// 5
    10465,	11087,	11747,	12445,	13185,	13969,	14800,	15680,	16612,	17600,	18647,	19755,	// 6
    20930,	22175,	23493,	24890,	26370,	27938,	29600,	31360,	33224,	35200,	37293,	39511,	// 7
    41860,	44349,	46986,	49780,	52740,	55877,	59199,	62719,	66449,	70400,	74586,	79021,	// 8
    83720,	88698,	93973,	99561,	105481,	111753,	118398,	125439,									// 9
};

// オクターブ
enum {
    OCTAVE_M1 = 0,	// -1
    OCTAVE_0,		// 0
    OCTAVE_1,		// 1
    OCTAVE_2,
    OCTAVE_3,
    OCTAVE_4,
    OCTAVE_5,
    OCTAVE_6,
    OCTAVE_7,
    OCTAVE_8,
    OCTAVE_9,
};

// ノート
enum {
    NOTE_C = 0,	// C
    NOTE_CS,	// C#
    NOTE_D,
    NOTE_DS,
    NOTE_E,
    NOTE_F,
    NOTE_FS,
    NOTE_G,
    NOTE_GS,
    NOTE_A,
    NOTE_AS,
    NOTE_B,
};

typedef struct {
    int octave;
    int note;
} NOTE_T;

// 周波数マクロ
#define BUZFREQ(anote)	(FREQ_TBL[((size_t)((anote).octave * 12 + (anote).note)) % NUM_NOTES])

// ブザーパターン番号
enum {
    BUZPATTERN_NONE = 0,
    BUZPATTERN_OFF,
    BUZPATTERN_BOOT,
    BUZPATTERN_ACCEPT,
    BUZPATTERN_REJECT,
    BUZPATTERN_DOROPN,
    BUZPATTERN_DORERR,
    BUZPATTERN_LCKERR,
    BUZPATTERN_OK,
    BUZPATTERN_NG,
    NUM_BUZPATTERN,
};

// ブザーアクション
enum {
    BUZACTION_NOCHANGE = 0,
    BUZACTION_TURNOFF,
    BUZACTION_TURNON,
    BUZACTION_STOP,
};

typedef struct {
    int action;
    NOTE_T note;
    int duration;
} BUZACTION_T;

// ブザーパターン
typedef struct {
    int pattern;
    const BUZACTION_T* sequence;
    size_t sequence_len;
    int repeats;
} BUZ_PATTERN_T;

// シーケンス定義
static const BUZACTION_T BUZSEQ_OFF[] =		{ { BUZACTION_TURNOFF, {0}, TMO_FEVR } };
static const BUZACTION_T BUZSEQ_BOOT[] =	{ { BUZACTION_TURNON, {OCTAVE_4, NOTE_D}, 125 }, { BUZACTION_TURNON, {OCTAVE_4, NOTE_A}, 125 } };
static const BUZACTION_T BUZSEQ_ACCEPT[] =	{ { BUZACTION_TURNON, {OCTAVE_4, NOTE_A}, 125 } };
static const BUZACTION_T BUZSEQ_REJECT[] =	{ { BUZACTION_TURNON, {OCTAVE_4, NOTE_A}, 100 }, { BUZACTION_TURNOFF, {0}, 25 } };
static const BUZACTION_T BUZSEQ_DOROPN[] =	{ { BUZACTION_TURNON, {OCTAVE_4, NOTE_A}, 125 }, { BUZACTION_TURNON, {OCTAVE_4, NOTE_D}, 125 } };
static const BUZACTION_T BUZSEQ_DORERR[] =	{ { BUZACTION_TURNON, {OCTAVE_2, NOTE_B}, 250 }, { BUZACTION_TURNON, {OCTAVE_2, NOTE_E}, 250 } };
static const BUZACTION_T BUZSEQ_LCKERR[] =	{ { BUZACTION_TURNON, {OCTAVE_5, NOTE_C},  50 }, { BUZACTION_TURNON, {OCTAVE_5, NOTE_G},  50 } };

// パターンテーブル
static const BUZ_PATTERN_T BUZ_PATTERNS[] = {
    { BUZPATTERN_NONE,	 		NULL,				0												, 1	},
    { BUZPATTERN_OFF,	 		BUZSEQ_OFF,			sizeof(BUZSEQ_OFF)/sizeof(BUZACTION_T)			, 1	},
    { BUZPATTERN_BOOT,	 		BUZSEQ_BOOT,		sizeof(BUZSEQ_BOOT)/sizeof(BUZACTION_T)			, 1	},
    { BUZPATTERN_ACCEPT,	 	BUZSEQ_ACCEPT,		sizeof(BUZSEQ_ACCEPT)/sizeof(BUZACTION_T)		, 1	},
    { BUZPATTERN_REJECT,	 	BUZSEQ_REJECT,		sizeof(BUZSEQ_REJECT)/sizeof(BUZACTION_T)		, 2	},
    { BUZPATTERN_DOROPN,	 	BUZSEQ_DOROPN,		sizeof(BUZSEQ_DOROPN)/sizeof(BUZACTION_T)		, 4	},
    { BUZPATTERN_DORERR,	 	BUZSEQ_DORERR,		sizeof(BUZSEQ_DORERR)/sizeof(BUZACTION_T)		, 4	},
    { BUZPATTERN_LCKERR,	 	BUZSEQ_LCKERR,		sizeof(BUZSEQ_LCKERR)/sizeof(BUZACTION_T)		,10	},
    { BUZPATTERN_OK,	 		BUZSEQ_ACCEPT,		sizeof(BUZSEQ_ACCEPT)/sizeof(BUZACTION_T)		, 1	},
    { BUZPATTERN_NG,	 		BUZSEQ_REJECT,		sizeof(BUZSEQ_REJECT)/sizeof(BUZACTION_T)		, 2	},
};

// ブザーのコンテキスト
typedef struct {
    int pattern;
    int seq_pos;
    int repeats;
    int next_action_in_ms;
} BUZ_CONTEXT_T;

BUZ_CONTEXT_T s_buz_context = {
    .pattern = BUZPATTERN_OFF,
};

/*
 * 内部関数プロトタイプ
 */
//static void mdlble_callback(int event, intptr_t opt1, intptr_t opt2);
static void drvpbt_callback(uint32_t button, int32_t evt);

static int process_event(APLEVT_EVENT_T* event);
static int led_action(int led, LED_CONTEXT_T* context, int time_elapsed);
static int buz_action(BUZ_CONTEXT_T* context, int time_elapsed);
static void led_set_pattern(int led, int pattern);
static void led_turn_off_all();
static void buz_set_pattern(int pattern);

/*
 * 内部変数
 */
// イベント送信先
static APLEVT_EVENT_RECEIVER_FUNC_T s_event_dest = NULL;


/*********************************************************************
 * 公開関数
 ********************************************************************/

/*
 * 共通初期化API(APLEVT_INITIALIZE_FUNC_T)
 */
int32_t aplui_initialize(APLEVT_EVENT_RECEIVER_FUNC_T receiver_func)
{
    assert(!s_event_dest);
    assert(receiver_func);

    // イベント送信先を登録
    s_event_dest = receiver_func;

    // タスクを起動
    ER er = E_OK;
    er = act_tsk(TSK_APLUI);
    assert(er == E_OK);

    return 0;
}

/*
 * 共通イベント受信API(APLEVT_EVENT_RECEIVER_FUNC_T)
 */
int32_t aplui_event(APLEVT_EVENT_T* event)
{
    ER er = aplevt_queue_event(DTQ_APLUI, event, 0);
    assert(er == E_OK);

    return 0;
}

/*
 * メインタスク
 */
void aplui_task(intptr_t exinf)
{
	DBGLOG0("aplui_task() starts.");

    ER er = E_OK;

    // LEDドライバを初期化
    drvled_initialize();

    // ブザードライバを初期化
    drvbuz_initialize();

    // プッシュボタンドライバを初期化
    drvpbt_initialize(drvpbt_callback);

    // LEDタスク起動
    er = act_tsk(TSK_APLUI_LED);
    assert(er == E_OK);

    // ブザータスク起動
    er = act_tsk(TSK_APLUI_BUZ);
    assert(er == E_OK);

    while (true) {
        APLEVT_EVENT_T* event = NULL;
        ER er_rcv = aplevt_receive_event(DTQ_APLUI, &event, TMO_FEVR);
        assert(er_rcv == E_OK);
        syslog(LOG_NOTICE, "aplui_task() received: (msg = %d).", event->code);

        process_event(event);

        switch (event->code) {
            
        default:
            //assert(false);
            break;
        }

        aplevt_return_event(event);
        event = NULL;
    }
}

/*
 * LEDタスク
 */
void aplui_led_task(intptr_t exinf)
{
	DBGLOG0("aplui_led_task() starts.");

    // ループ
    int time_elapsed = 0;
    int next_timeout = 0;
    SYSTIM systim1 = 0, systim2 = 0;
    static const uint32_t TIM_MAX = 0xffff;	// 65535ms
    while (true) {
        next_timeout = TIM_MAX;
        time_elapsed = ((systim2 >= systim1) ? (systim2) : (systim2 + TIM_MAX)) - systim1;

        ER er = twai_sem(SEM_APLUI_LEDTBL, LOCK_TIMEOUT);	// ロック獲得
        assert(er == E_OK);

        for (int i = 0; i < sizeof(s_led_table)/sizeof(s_led_table[0]); i++) {
            if (s_led_table[i].next_action_in_ms == TMO_FEVR) {
                // 何もしない
            } else if (s_led_table[i].next_action_in_ms <= time_elapsed) {
                // 処理実行
                s_led_table[i].next_action_in_ms = led_action(s_led_table[i].led, &(s_led_table[i].context), time_elapsed);
            } else {
                // 時間だけ減らす
                s_led_table[i].next_action_in_ms -= time_elapsed;
            }

            // 直近の時間を求める
            if ((s_led_table[i].next_action_in_ms != TMO_FEVR) && (s_led_table[i].next_action_in_ms < next_timeout)) {
                next_timeout = s_led_table[i].next_action_in_ms;
            }
        }

        er = sig_sem(SEM_APLUI_LEDTBL);	// ロック返却
        assert(er == E_OK);

        get_tim(&systim1);
        systim1 &= TIM_MAX;
        // 待ち(タイムアウト or パターン変更)
        er = twai_flg(FLG_APLUI, FLGPTN_LEDPATTERN_CHANGED, TWF_ANDW, &(FLGPTN){0}, next_timeout);
        assert(er == E_OK || er == E_TMOUT);

        get_tim(&systim2);
        systim2 &= TIM_MAX;
    }
}

/*
 * ブザータスク
 */
void aplui_buz_task(intptr_t exinf)
{
	DBGLOG0("aplui_buz_task() starts.");

    ER er = E_OK;

    // ループ
    int time_elapsed = 0;
    SYSTIM systim1 = 0, systim2 = 0;
    static const uint32_t TIM_MAX = 0xffff;	// 65535ms
    while (true) {

        time_elapsed = ((systim2 >= systim1) ? (systim2) : (systim2 + TIM_MAX)) - systim1;

        er = twai_sem(SEM_APLUI_BUZCTX, LOCK_TIMEOUT);	// ロック獲得
        assert(er == E_OK);

        if (s_buz_context.next_action_in_ms == TMO_FEVR) {
            // 何もしない
        } else if (s_buz_context.next_action_in_ms <= time_elapsed) {
            // 処理実行
            s_buz_context.next_action_in_ms = buz_action(&s_buz_context, time_elapsed);
        } else {
            // 時間だけ減らす
            s_buz_context.next_action_in_ms -= time_elapsed;
        }

        er = sig_sem(SEM_APLUI_BUZCTX);	// ロック返却
        assert(er == E_OK);

        get_tim(&systim1);
        systim1 &= TIM_MAX;
        // 待ち(タイムアウト or パターン変更)
        er = twai_flg(FLG_APLUI, FLGPTN_BUZPATTERN_CHANGED, TWF_ANDW, &(FLGPTN){0}, s_buz_context.next_action_in_ms);
        assert(er == E_OK || er == E_TMOUT);

        get_tim(&systim2);
        systim2 &= TIM_MAX;
    }
}

// 【デバッグ用】LEDパターン変更
void aplui_dbg_set_led_pattern(int led, int pattern)
{
    led_set_pattern(led, pattern);
}

// 【デバッグ用】ブザーパターン変更
void aplui_dbg_set_buz_pattern(int pattern)
{
    buz_set_pattern(pattern);
}

/*********************************************************************
 * 内部関数
 ********************************************************************/

/*
 * イベント処理
 */
int process_event(APLEVT_EVENT_T* event)
{
    int result = 0;

    switch(event->code) {
    case APLEVT_AUTH_READY:
        led_turn_off_all();
        led_set_pattern(DRVLED_LED_FRONT_GREEN, LEDPATTERN_ON);
        break;
    case APLEVT_REGISTRATION_READY:
        led_turn_off_all();
        led_set_pattern(DRVLED_LED_FRONT_GREEN, LEDPATTERN_ON);
        buz_set_pattern(BUZPATTERN_ACCEPT);
        break;
    case APLEVT_AUTH_PROCESSING:
        led_turn_off_all();
        led_set_pattern(DRVLED_LED_FRONT_YGREEN, LEDPATTERN_ON);
        break;
    case APLEVT_REGISTRATION_PROCESSING:
        led_turn_off_all();
        led_set_pattern(DRVLED_LED_FRONT_YGREEN, LEDPATTERN_ON);
        buz_set_pattern(BUZPATTERN_ACCEPT);
        break;
    case APLEVT_AUTH_COMPLETE:
        if (!event->error) {
            led_turn_off_all();
            led_set_pattern(DRVLED_LED_FRONT_BLUE, LEDPATTERN_3000MS);
            led_set_pattern(DRVLED_LED_FRONT_GREEN, LEDPATTERN_ON_AFTER_3000MS);
        } else {
            led_turn_off_all();
            led_set_pattern(DRVLED_LED_FRONT_RED, LEDPATTERN_3000MS);
            led_set_pattern(DRVLED_LED_FRONT_GREEN, LEDPATTERN_ON_AFTER_3000MS);
        }
        break;
    case APLEVT_REGISTRATION_COMPLETE:
        if (!event->error) {
            led_turn_off_all();
            led_set_pattern(DRVLED_LED_FRONT_BLUE, LEDPATTERN_3000MS);
            led_set_pattern(DRVLED_LED_FRONT_GREEN, LEDPATTERN_ON_AFTER_3000MS);
            buz_set_pattern(BUZPATTERN_ACCEPT);
        } else {
            led_turn_off_all();
            led_set_pattern(DRVLED_LED_FRONT_RED, LEDPATTERN_3000MS);
            led_set_pattern(DRVLED_LED_FRONT_GREEN, LEDPATTERN_ON_AFTER_3000MS);
            buz_set_pattern(BUZPATTERN_REJECT);
        }
        break;
    case APLEVT_AUTH_RETRYING:
        led_turn_off_all();
        led_set_pattern(DRVLED_LED_FRONT_RED, LEDPATTERN_BLINK_8HZ_1S);
        led_set_pattern(DRVLED_LED_FRONT_GREEN, LEDPATTERN_ON_AFTER_3000MS);
        break;
    case APLEVT_STORAGE_PROCESSING:
        led_set_pattern(DRVLED_LED_FRONT_BLUE, LEDPATTERN_BLINK_8HZ);
        break;
    case APLEVT_STORAGE_COMPLETE:
        led_set_pattern(DRVLED_LED_FRONT_BLUE, LEDPATTERN_OFF);
        break;
    case APLEVT_USER_BLE_ENABLE:
        led_set_pattern(DRVLED_LED_REAR_1, LEDPATTERN_OFF);
        break;
    case APLEVT_BATTERY_LOW:
        led_set_pattern(DRVLED_LED_FRONT_RED, LEDPATTERN_BLINK_2HZ);
        break;
    case APLEVT_LOCK_ERROR:
        switch(event->error) {
        case 0x00:                              //通知解除
            buz_set_pattern(BUZPATTERN_OFF);
            break;
        case 0x01:                              //扉過開放時通知
            buz_set_pattern(BUZPATTERN_DOROPN);
            break;
        case 0x02:                              //扉こじ開け時通知
            buz_set_pattern(BUZPATTERN_DORERR);
            break;
        case 0x03:                              //施解錠不良時通知
            buz_set_pattern(BUZPATTERN_LCKERR);
            break;
            default:
            result = -1;
            break;
        }
        break;
    default:
        result = -1;
        break;
    }

    return result;
}

/*
 * LED処理
 */
int led_action(int led, LED_CONTEXT_T* context, int time_elapsed)
{
    assert(context);
    assert(led < DRVLED_NUM_LEDS);

    int next_action_in_ms = 0;
    const LED_PATTERN_T* pat = &(LED_PATTERNS[context->pattern]);
    if (pat->pattern == LEDPATTERN_NONE) {
        next_action_in_ms = TMO_FEVR;
        goto end;
    }

    // 繰り返しを完了したら停止する
    if ((pat->repeats > 0) && (context->repeats >= pat->repeats)) {
        drvled_set_state(led, false);
        next_action_in_ms = TMO_FEVR;
        goto end;
    }

    assert(pat->pattern == context->pattern);
    const LEDACTION_T* act = &(pat->sequence[context->seq_pos]);

    switch (act->action) {
    case LEDACTION_NOCHANGE:
        // なし
        break;
    case LEDACTION_TURNOFF:
        drvled_set_state(led, false);
        break;
    case LEDACTION_TURNON:
        drvled_set_state(led, true);
        break;
    case LEDACTION_STOP:
        // なし
        break;
    default:
        assert(false);
        break;
    }
    next_action_in_ms = act->duration;

    // シーケンスを進める
    context->seq_pos++;
    if (context->seq_pos >= pat->sequence_len) {
        context->seq_pos = 0;
        context->repeats++;
    }

end:
    return next_action_in_ms;
}

/*
 * ブザー処理
 */
int buz_action(BUZ_CONTEXT_T* context, int time_elapsed)
{
    assert(context);

    int next_action_in_ms = 0;
    const BUZ_PATTERN_T* pat = &(BUZ_PATTERNS[context->pattern]);
    if (pat->pattern == BUZPATTERN_NONE) {
        next_action_in_ms = TMO_FEVR;
        goto end;
    }

    // 繰り返しを完了したら停止する
    if ((pat->repeats > 0) && (context->repeats >= pat->repeats)) {
        drvbuz_stop();
        next_action_in_ms = TMO_FEVR;
        goto end;
    }

    assert(pat->pattern == context->pattern);
    const BUZACTION_T* act = &(pat->sequence[context->seq_pos]);

    switch (act->action) {
    case BUZACTION_NOCHANGE:
        // なし
        break;
    case BUZACTION_TURNOFF:
        drvbuz_stop();
        break;
    case BUZACTION_TURNON:
        drvbuz_start(BUZFREQ(act->note));
        break;
    case BUZACTION_STOP:
        // なし
        break;
    default:
        assert(false);
        break;
    }
    next_action_in_ms = act->duration;

    // シーケンスを進める
    context->seq_pos++;
    if (context->seq_pos >= pat->sequence_len) {
        context->seq_pos = 0;
        context->repeats++;
    }

end:
    return next_action_in_ms;
}

/*
 * LEDパターン変更
 */
void led_set_pattern(int led, int pattern)
{
    assert(led >= 0 && led < DRVLED_NUM_LEDS);
    assert(pattern >= 0 && pattern < NUM_LEDPATTERN);

    LED_CONTEXT_T* context = NULL;
    int* next_action_in_ms = NULL;

    ER er = twai_sem(SEM_APLUI_LEDTBL, LOCK_TIMEOUT);	// ロック獲得
    assert(er == E_OK);

    for (int i = 0; i < sizeof(s_led_table)/sizeof(s_led_table[0]); i++) {
        if (s_led_table[i].led == led) {
            context = &(s_led_table[i].context);
            next_action_in_ms = &(s_led_table[i].next_action_in_ms);
            break;
        }
    }
    assert(context);

    // パターンを設定
    context->pattern = pattern;
    context->seq_pos = 0;
    context->repeats = 0;
    // 待ち時間をクリア
    *next_action_in_ms = 0;

    er = sig_sem(SEM_APLUI_LEDTBL);	// ロック返却
    assert(er == E_OK);

    // 変更フラグを設定
    er = set_flg(FLG_APLUI, FLGPTN_LEDPATTERN_CHANGED);
    assert(er == E_OK);
}

/*
 * LED全消灯
 */
void led_turn_off_all()
{
    for (int i = 0; i < DRVLED_NUM_LEDS; i++) {
        led_set_pattern(i, LEDPATTERN_OFF);
    }
}

/*
 * ブザーパターン変更
 */
void buz_set_pattern(int pattern)
{
    assert(pattern >= 0 && pattern < NUM_BUZPATTERN);

    ER er = twai_sem(SEM_APLUI_BUZCTX, LOCK_TIMEOUT);	// ロック獲得
    assert(er == E_OK);

    // パターンを設定
    s_buz_context.pattern = pattern;
    s_buz_context.seq_pos = 0;
    s_buz_context.repeats = 0;
    // 待ち時間をクリア
    s_buz_context.next_action_in_ms = 0;

    er = sig_sem(SEM_APLUI_BUZCTX);	// ロック返却
    assert(er == E_OK);

    // 変更フラグを設定
    er = set_flg(FLG_APLUI, FLGPTN_BUZPATTERN_CHANGED);
    assert(er == E_OK);
}

/*
 *
 */
void drvpbt_callback(uint32_t button, int32_t evt)
{
    //
    DBGLOG2("drvpbt_callback: b:%d, e:%d", button, evt);

}

