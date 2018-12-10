/*
 * VA-X 電源関係共通処理
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/06/27 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>

/*
 * マクロ
 */

/*
 * 型
 */
// RTC時刻
typedef struct {
    int year;
    int month;
    int day;
    int weekday;
    int hour;
    int minute;
    int second;
    int msecond;
} DRVPWR_RTC_DATETIME_T;

/* コールバック関数型(バッテリー電圧) */
typedef void (*DRVPWR_CBFUNC_T)(int evt, uint32_t opt);

/*
 * 定数
 */
// 曜日 (RTC_DR の WDU ビット)
#define DRVPWR_RTC_MONDAY		(0x1)
#define DRVPWR_RTC_TUESDAY		(0x2)
#define DRVPWR_RTC_WEDNESDAY	(0x3)
#define DRVPWR_RTC_THURSDAY		(0x4)
#define DRVPWR_RTC_FRIDAY		(0x5)
#define DRVPWR_RTC_SATURDAY		(0x6)
#define DRVPWR_RTC_SUNDAY		(0x7)

// Wakeup ピン
enum {
    DRVPWR_WUPIN_NONE = 0,
    DRVPWR_WUPIN_1_PA0,
    DRVPWR_WUPIN_2_PA2,
    DRVPWR_WUPIN_3_PC1,
    DRVPWR_WUPIN_4_PC13,
    DRVPWR_WUPIN_5_PI8,
    DRVPWR_WUPIN_6_PI11,
    DRVPWR_WUPIN_NUM,
};

// RTCによる起動
enum {
    DRVPWR_WURTC = DRVPWR_WUPIN_NUM,
};

// Wakeup ポラリティ
#define DRVPWR_WUPOL_RISING		0
#define DRVPWR_WUPOL_FALLING	1

// デバイス
enum {
    DRVPWR_CAM = 0,
    DRVPWR_IRLED,
    DRVPWR_ICC,
    DRVPWR_WIFI,
    DRVPWR_NUM_DEVS,
};

// コールバックイベント
enum {
    DRVPWR_EVT_NONE = 0,
    DRVPWR_EVT_BAT_VOLTAGE,
};

/*
 * 公開関数
 */
// ペリフェラル初期化
void drvpwr_initialize_peripherals();

// ドライバ初期化
void drvpwr_initialize();

// Wakeup 要因取得
int drvpwr_get_wakeup_cause();

// Wakeup ピン有効
void drvpwr_enable_wakeup(int pin, int polarity);

// スタンバイモード遷移
void drvpwr_enter_standby_mode();

// RTC 時刻取得
void drvpwr_rtc_get(DRVPWR_RTC_DATETIME_T* datetime);

// RTC 時刻設定
void drvpwr_rtc_set(const DRVPWR_RTC_DATETIME_T* datetime);

// RTC Wakeup設定
void drvpwr_rtc_set_wakeup(uint32_t wakeup_after_ms);

// デバイス使用設定
void drvpwr_use_device(int device, bool_t use);

// バッテリー電圧取得
void drvpwr_get_bat_voltage(DRVPWR_CBFUNC_T callback);

/*
 * 内部用関数
 */

// ADC割込みサービスルーチン
void drvpwr_adc_isr(intptr_t exinf);

// タスク
void drvpwr_task(intptr_t exinf);


