/*
 * VA-X 認証ミドル
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/01/23 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>

/*
 * マクロ定数
 */

// 動作モード
#define MDLAUTH_MODE_AUTH			1	// 通常の認証(1対N認証)
#define MDLAUTH_MODE_REGISTRATION	2	// 登録
#define MDLAUTH_MODE_VERIFYREG		3	// 登録の確認
#define MDLAUTH_MODE_ADMINAUTH		4	// 管理者認証
#define MDLAUTH_MODE_SPECIFIEDAUTH	5	// 特定の人の認証(1対1認証)

// イベント種別
#define MDLAUTH_EVT_INITIALIZE_COMPLETE		1	// 初期化完了
#define MDLAUTH_EVT_PREPARE_COMPLETE		2	// 認証準備完了
#define MDLAUTH_EVT_TS_DETECT				3	// 生体センサー検出
#define MDLAUTH_EVT_CAPTURE_COMPLETE		4	// カメラ撮影完了
#define MDLAUTH_EVT_AUTH_COMPLETE			5	// 認証完了
#define MDLAUTH_EVT_REGISTRATION_COMPLETE	6	// 登録完了
#define MDLAUTH_EVT_CANCEL_COMPLETE			7	// キャンセル完了
#define MDLAUTH_EVT_AUTH_RETRYING			8	// リトライ中

/* メモリプールブロック長(内部用) */
#define MDLAUTH_MPFBLK_SIZE		12
#define MDLAUTH_DTQ_SIZE		10

/*
 * 型定義
 */

/* コールバック関数型 */
typedef void (*MDLAUTH_CALLBACK_T)(int event, intptr_t opt1, intptr_t opt2);

/*
 * 公開関数
 */

// 事前開始処理
void mdlauth_trigger_start();

// 初期化
void mdlauth_initialize(MDLAUTH_CALLBACK_T callback);

// 認証準備
int mdlauth_prepare(int mode ,int index);

// 認証・登録をキャンセルし、初期状態に戻す
int mdlauth_cancel();

/*
 * 内部関数
 */

// タスク
void mdlauth_task(intptr_t exinf);


