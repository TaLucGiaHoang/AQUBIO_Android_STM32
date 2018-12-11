/*
 * VA-X
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/13 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

/*
 *  各タスクの優先度の定義
 */

#define MAIN_PRIORITY	5		/* メインタスクの優先度 */
								/* HIGH_PRIORITYより高くすること */

#define HIGH_PRIORITY	9		/* 並行実行されるタスクの優先度 */
#define MID_PRIORITY	10
#define LOW_PRIORITY	11


#ifndef STACK_SIZE
//#define	STACK_SIZE		4096		/* タスクのスタックサイズ */
#define	STACK_SIZE		1024		/* タスクのスタックサイズ */
#endif /* STACK_SIZE */

/*
 * 共通エラーコード
 */
enum {
    BNVA_ERROR_NONE = 0,
    BNVA_ERROR_TIMEOUT,
    BNVA_ERROR_INVALID_STATE,
    BNVA_ERROR_NOTFOUND,
    BNVA_ERROR_HWFAILURE,
};

/*
 * 機器動作モード
 */
enum {
    BNVA_DEVICE_MODE_NONE = 0,
};

/*
 * デバイス識別情報
 */
typedef struct {
    uint8_t serialno[16];
} BNVA_DEVICE_ID_T;
