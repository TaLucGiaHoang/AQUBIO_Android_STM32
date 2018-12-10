/*
 * VA-X 共用タイマータスク
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/04/24 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "cmntimer.h"

#include <string.h>

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"

//
// マクロ
//

// デバッグログ
#if 1
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[CMNTIMER]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[CMNTIMER]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[CMNTIMER]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[CMNTIMER]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

//
// 型
//

// タイマー状態
typedef struct {
    int type;						// タイマー種別
    int state;						// 状態
    int interval;					// インターバル
    int expired_in;					// 満了までの残り時間
    CMNTIMER_CALLBACK_T callback;	// コールバック関数
    intptr_t opt;					// オプション
} TIMER_CONTEXT_T;

//
// 定数
//

// タイマー状態
enum {
    STATE_STOPPED = 0,
    STATE_STARTING,
    STATE_RUNNING,
};

// イベントフラグ
static const FLGPTN FLGPTN_TIMER_CHANGED =	(0x1 << 0);

// ロックタイムアウト
static const int LOCK_TIMEOUT = 1000;

//
// 内部変数
//
static TIMER_CONTEXT_T s_timers[CMNTIMER_NUM_TIMERS];
static TIMER_CONTEXT_T s_expired_timers[CMNTIMER_NUM_TIMERS];

// 初期化
void cmntimer_initialize()
{
    // タスクを起動
    ER er = E_OK;
    er = act_tsk(TSK_CMNTIMER);
    assert(er == E_OK);
}

/*
 * タイマー設定
 */
void cmntimer_set_timer(int timer_id, int type, int interval, CMNTIMER_CALLBACK_T callback, intptr_t opt)
{

    DBGLOG1("set_timer_id: %d", timer_id);
    
    assert((timer_id >= 0) && (timer_id < CMNTIMER_NUM_TIMERS));
    assert((type == CMNTIMER_TYPE_ONESHOT) || (type == CMNTIMER_TYPE_CYCLIC));
    assert(interval >= 0);
    assert(callback);
    assert(s_timers[timer_id].state == STATE_STOPPED);

    ER er = twai_sem(SEM_CMNTIMER, LOCK_TIMEOUT);	// ロック獲得
    assert(er == E_OK);

    s_timers[timer_id].type = type;
    s_timers[timer_id].interval = interval;
    s_timers[timer_id].expired_in = interval;
    s_timers[timer_id].callback = callback;
    s_timers[timer_id].opt = opt;

    s_timers[timer_id].state = STATE_STARTING;	// 状態を「STARTING」に変更

    er = sig_sem(SEM_CMNTIMER);	// ロック返却
    assert(er == E_OK);

    // 設定変更フラグ
    er = set_flg(FLG_CMNTIMER, FLGPTN_TIMER_CHANGED);
    assert(er == E_OK);

    return;
}

/*
 * タイマーキャンセル
 */
void cmntimer_cancell_timer(int timer_id)
{
//	DBGLOG0("cancell_timer starts.");
    DBGLOG1("cancell_timer starts: e=%d", timer_id);
    assert((timer_id >= 0) && (timer_id < CMNTIMER_NUM_TIMERS));

    ER er = twai_sem(SEM_CMNTIMER, LOCK_TIMEOUT);	// ロック獲得
    assert(er == E_OK);

    s_timers[timer_id].state = STATE_STOPPED;	// 状態を「STOPPED」に変更

    er = sig_sem(SEM_CMNTIMER);	// ロック返却
    assert(er == E_OK);

    // 設定変更フラグ
    er = set_flg(FLG_CMNTIMER, FLGPTN_TIMER_CHANGED);
    assert(er == E_OK);

    return;
}

/*
* タイマー設定(非タスクコンテキスト用)
 */
void isr_cmntimer_set_timer(int timer_id, int type, int interval, CMNTIMER_CALLBACK_T callback, intptr_t opt)
{

    DBGLOG1("set_timer_id: %d", timer_id);
    
    assert((timer_id >= 0) && (timer_id < CMNTIMER_NUM_TIMERS));
    assert((type == CMNTIMER_TYPE_ONESHOT) || (type == CMNTIMER_TYPE_CYCLIC));
    assert(interval >= 0);
    assert(callback);
    assert(s_timers[timer_id].state == STATE_STOPPED);

//    ER er = twai_sem(SEM_CMNTIMER, LOCK_TIMEOUT);	// ロック獲得
//    assert(er == E_OK);

    s_timers[timer_id].type = type;
    s_timers[timer_id].interval = interval;
    s_timers[timer_id].expired_in = interval;
    s_timers[timer_id].callback = callback;
    s_timers[timer_id].opt = opt;

    s_timers[timer_id].state = STATE_STARTING;	// 状態を「STARTING」に変更

//    er = sig_sem(SEM_CMNTIMER);	// ロック返却
//    assert(er == E_OK);

    // 設定変更フラグ
    ER er = iset_flg(FLG_CMNTIMER, FLGPTN_TIMER_CHANGED);
    assert(er == E_OK);
    DBGLOG1("set_timer_er: %x", er);

    return;
}

/*
 * タイマーキャンセル(非タスクコンテキスト用)
 */
void isr_cmntimer_cancell_timer(int timer_id)
{
//	DBGLOG0("cancell_timer starts.");
    DBGLOG1("cancell_timer starts: e=%d", timer_id);
    assert((timer_id >= 0) && (timer_id < CMNTIMER_NUM_TIMERS));

//    ER er = twai_sem(SEM_CMNTIMER, LOCK_TIMEOUT);	// ロック獲得
//    assert(er == E_OK);

    s_timers[timer_id].state = STATE_STOPPED;	// 状態を「STOPPED」に変更

//    er = sig_sem(SEM_CMNTIMER);	// ロック返却
//    assert(er == E_OK);

    // 設定変更フラグ
    ER er = iset_flg(FLG_CMNTIMER, FLGPTN_TIMER_CHANGED);
    assert(er == E_OK);

    return;
}


/*
 *  メインタスク
 */
void cmntimer_task(intptr_t exinf)
{
	DBGLOG0("cmntimer_task() starts.");

    // 初期化
    memset(&s_timers, 0, sizeof(s_timers));
    memset(s_expired_timers, 0, sizeof(s_expired_timers));

    // ループ
    int time_elapsed = 0;
    int next_timeout = 0;
    bool_t has_active_timer = false;
    SYSTIM systim1 = 0, systim2 = 0;
    static const uint32_t WAIT_MAX = 0xffff;	// 65535ms
    static const uint32_t TIM_MASK = 0x1ffff;
    while (true) {
        DBGLOG1("time_elapsed: %d", time_elapsed);
        has_active_timer = false;
        next_timeout = WAIT_MAX;

        ER er = twai_sem(SEM_CMNTIMER, LOCK_TIMEOUT);	// ロック獲得
        assert(er == E_OK);

        for (int i = 0; i < CMNTIMER_NUM_TIMERS; i++) {
            if (s_timers[i].state == STATE_RUNNING) {
                if (s_timers[i].expired_in <= time_elapsed) {
                    // タイマー満了
                    DBGLOG2("time_elapsed: %d, expired_in: %d", time_elapsed, s_timers[i].expired_in);
                    s_expired_timers[i].callback = s_timers[i].callback;
                    s_expired_timers[i].opt = s_timers[i].opt;
                    if (s_timers[i].type == CMNTIMER_TYPE_ONESHOT) {
                        s_timers[i].state = STATE_STOPPED;
                        s_timers[i].callback = NULL;
                    } else if (s_timers[i].type == CMNTIMER_TYPE_CYCLIC) {
                        s_timers[i].expired_in = s_timers[i].interval;	// リロード
                    }
                } else {
                    // 時間だけ減らす
                    s_timers[i].expired_in -= time_elapsed;
                }

            } else if (s_timers[i].state == STATE_STARTING) {
                // これから開始
                s_timers[i].state = STATE_RUNNING;
            } else if (s_timers[i].state == STATE_STOPPED) {
                // 何もしない
            }

            if (s_timers[i].state == STATE_RUNNING) {
                has_active_timer = true;
                if (s_timers[i].expired_in < next_timeout) {
                    next_timeout = s_timers[i].expired_in;
                }
            }
        }

        er = sig_sem(SEM_CMNTIMER);	// ロック返却
        assert(er == E_OK);

        // コールバック
        // ロックを保持したままコールバックすると, コールバック先からタイマー関数を呼ばれた場合にデッドロックする!
        for (int i = 0; i < CMNTIMER_NUM_TIMERS; i++) {
            if (s_expired_timers[i].callback) {
                s_expired_timers[i].callback(i, s_expired_timers[i].opt);
                s_expired_timers[i].callback = NULL;
            }
        }

        get_tim(&systim1);
        systim1 &= TIM_MASK;

        // 待ち(タイムアウト or タイマ設定変更)
        if (!has_active_timer) {
            next_timeout = TMO_FEVR;
        }
        DBGLOG1("next_timeout: %d", next_timeout);
        er = twai_flg(FLG_CMNTIMER, FLGPTN_TIMER_CHANGED, TWF_ANDW, &(FLGPTN){0}, next_timeout);
        assert(er == E_OK || er == E_TMOUT);

        get_tim(&systim2);
        systim2 &= TIM_MASK;

        // 経過時間を算出
        time_elapsed = ((systim2 >= systim1) ? (systim2) : (systim2 + TIM_MASK)) - systim1;
    }
}

/*********************************************************************
 * 内部関数
 ********************************************************************/


