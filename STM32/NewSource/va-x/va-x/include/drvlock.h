/*
 * VA-X 錠ドライバ
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/05/07 Takashi Otsuka <otsuka-takashi@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>
#include <kernel.h>

/* コールバック関数型 */
typedef void (*DRVLOCK_CBFUNC_T)(int evt, int opt);

/* 定数 */
enum {
    DRVLOCK_EVT_LOCK = 0,
    DRVLOCK_EVT_UNLOCK,
    DRVLOCK_EVT_DOOROPN,
    DRVLOCK_EVT_DOORERR,
//    DRVLOCK_EVT_MOTORRDY,
    DRVLOCK_EVT_NEUTRAL,
    DRVLOCK_EVT_INTMDET,
//    DRVLOCK_EVT_ERROR,
};

/* イベント ステータス*/
enum {
    OKEY_EVTS = 0,  //ステータス異常無し、状態情報無し
    OPN_EVTS,       //ドアOPEN履歴有り
    DOR_ERR,        //ドアエラー
    MTR_ERR         //モータエラー
};

/*
 * 公開関数
 */

/* ペリフェラル初期化 */
void drvlock_initialize_peripherals();

/* ドライバ初期化 */
void drvlock_initialize(DRVLOCK_CBFUNC_T callback);

/* 鍵錠 状態検出 */
void drvlock_get_state();

/* 鍵錠 解除要求 */
void drvlock_open();

/* 鍵錠 解除要求 */
void drvlock_close();
/*
 * 内部関数
 */

/* 割込みサービスルーチン */
int32_t drvlock_sensor_isr(uint32_t pinno);

/* タスク */
void drvlock_task(intptr_t exinf);

/* 割込みサービスルーチン */
int32_t drvlock_exti_isr(uint32_t pinno);


