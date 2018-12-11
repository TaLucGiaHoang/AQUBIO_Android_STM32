/*
 * VA-X
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/13 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>

/*
 * マクロ
 */

// LED種別
enum {
    DRVLED_LED_FRONT_RED = 0,	// 前面 赤
    DRVLED_LED_FRONT_BLUE,		// 前面 青
    DRVLED_LED_FRONT_GREEN,		// 前面 緑
    DRVLED_LED_FRONT_YGREEN,	// 前面 黄緑
    DRVLED_LED_REAR_1,			// 背面 1
    DRVLED_NUM_LEDS,			// LED個数
};

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
/*
 * 公開関数
 */

/* 初期化 */
void drvled_initialize_peripherals();

/* 初期化 */
void drvled_initialize();

/* LED 点灯/消灯 */
void drvled_set_state(int led_no, bool_t on);

