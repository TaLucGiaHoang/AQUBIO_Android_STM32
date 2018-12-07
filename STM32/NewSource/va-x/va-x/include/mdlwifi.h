/*
 * VA-X Wifiミドル
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/03/29 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>
#include "va-x_def.h"

/*
 * マクロ定数
 */

/* イベント種別 */
enum {
    MDLWIFI_EVT_NONE = 0,
    MDLWIFI_EVT_INITIALIZE_COMPLETE,
    MDLWIFI_EVT_READY_COMPLETE,
    MDLWIFI_EVT_STOP_COMPLETE,
    MDLWIFI_EVT_SERVERTASK_COMPLETE,
    MDLWIFI_EVT_UPLOAD_ERR_LOG_COMPLETE,
    MDLWIFI_EVT_UPLOAD_OPE_LOG_COMPLETE,
    MDLWIFI_EVT_DOWNLOAD_CARD_COMPLETE,
//    MDLWIFI_EVT_START,
};

/* エラー種別 */
enum {
    MDLWIFI_ERR_NONE = 0,
    MDLWIFI_ERR_INITIALIZE,
    MDLWIFI_ERR_AP_CONNECT,
    MDLWIFI_ERR_AP_DISCONNECT,
    MDLWIFI_ERR_HTTP_CONNECT,
    MDLWIFI_ERR_HTTP_POST,
    MDLWIFI_ERR_HTTP_DISCONNECT,
    MDLWIFI_ERR_POWER_OFF,
};

/* メモリプールブロック長(内部用) */
#define MDLWIFI_MPFBLK_SIZE	20
#define MDLWIFI_DTQ_SIZE		10
// #define MDLWIFI_MAX_DATA_LEN	4096
// #define MDLWIFI_NUM_DATA_MPF	2

// サーバ切替（コメントアウトでHTTPS対応の外部サーバ接続に変わる）
//#define HTTP_LOCALSERVER

/*
 * 型定義
 */

/* コールバック関数型 */
typedef void (*MDLWIFI_CALLBACK_T)(int event, intptr_t error, intptr_t opt1, intptr_t opt2);

/* サーバタスクレスポンス */
typedef struct {
    uint8_t task_code1[2];
    uint8_t task_code2[2];
    uint8_t task_code3[2];
    uint8_t task_code4[2];
} MDLWIFI_RES_SERVERTASK_T;

/* 空室カードレスポンス */
typedef struct {
    uint8_t idm[20][8];
} MDLWIFI_RES_VACANCY_CARD_T;

/* レスポンス共通データ */
typedef struct {
    int res_result;                     // 電文上のtrue / false
    uint8_t now_date[12];
//    int drvwifi_result;                 // ドライバWiFiからのエラー情報
} MDLWIFI_RES_COMMON_T;

/*
 * 公開関数
 */

// 初期化
void mdlwifi_initialize(MDLWIFI_CALLBACK_T callback);

// WIFI開始
void mdlwifi_start(intptr_t data, size_t length, const BNVA_DEVICE_ID_T* serialno);

// WIFI制御中止
void mdlwifi_stop();

// サーバタスク確認
void mdlwifi_check_servertask(intptr_t data, size_t length);

// エラーログアップロード処理
void mdlwifi_upload_err_log();

// 操作ログアップロード処理
void mdlwifi_upload_ope_log();

// 空室カードダウンロード処理
void mdlwifi_download_vacancy_card();

/*
 * 内部関数
 */
// タスク
void mdlwifi_task(intptr_t exinf);


