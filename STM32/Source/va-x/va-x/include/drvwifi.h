/*
 * VA-X Wifiドライバ (SX-ULPGN)
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/03/22 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>

/*
 * マクロ
 */


/* エラー種別 */
#define DRVWIFI_ERROR_NONE			0
#define DRVWIFI_ERROR_TIMEOUT		1

#define DRVWIFI_MAX_ESSID_LEN		32	// ESSID最大長
#define DRVWIFI_MAX_PASSPHRASE_LEN	64	// パスフレーズ最大長
#define DRVWIFI_MAX_HOSTNAME_LEN	32	// ホスト名最大長
#define DRVWIFI_MAX_PATH_LEN		32	// パス最大長

/* メモリプールブロック長(内部用) */
#define DRVWIFI_MPFBLK_SIZE		12
#define DRVWIFI_DTQ_SIZE		10

/*
 * 型定義
 */
/* コールバック関数型 */
typedef void (*DRVWIFI_CALLBACK_T)(int32_t evt, int32_t error, intptr_t opt);

/* AP情報 */
typedef struct {
    uint8_t essid[DRVWIFI_MAX_ESSID_LEN];
    size_t essid_len;
    int signal_strength;
} DRVWIFI_AP_INFO;

/* WIFI接続設定 */
typedef struct {
    uint8_t essid[DRVWIFI_MAX_ESSID_LEN];
    size_t essid_len;
    uint8_t passphrase[DRVWIFI_MAX_PASSPHRASE_LEN];
    size_t passphrase_len;
    int sec_type;
} DRVWIFI_CONFIG;

/* HTTPS接続設定 */
typedef struct {
    uint8_t hostname[DRVWIFI_MAX_HOSTNAME_LEN];
    size_t hostname_len;
} DRVWIFI_HTTPS_CONFIG;

/* HTTPS GET要求 */
typedef struct {
    uint8_t path[DRVWIFI_MAX_PATH_LEN];
    size_t path_len;
    uint8_t resp_buf[DRVWIFI_MAX_PATH_LEN];
    size_t buf_len;
} DRVWIFI_HTTPS_GET_REQ;

/* HTTPS POST要求 */
typedef struct {
    uint8_t path[DRVWIFI_MAX_PATH_LEN];
    size_t path_len;
} DRVWIFI_HTTPS_POST_REQ;

/*
 * 定数
 */
/* コールバック種別 */
enum {
    DRVWIFI_EVT_NONE = 0,
    DRVWIFI_EVT_INITIALIZE_COMPLETE,
    DRVWIFI_EVT_DEVICE_AVAILABLE_COMPLETE,
    DRVWIFI_EVT_FACTORY_RESET_COMPLETE,
    DRVWIFI_EVT_RESET_COMPLETE,
    DRVWIFI_EVT_AP_SCAN_COMPLETE,
    DRVWIFI_EVT_AP_CONNECT_COMPLETE,
    DRVWIFI_EVT_AP_CONNECT_WPS_COMPLETE,
    DRVWIFI_EVT_HTTP_CONNECT_COMPLETE,
    DRVWIFI_EVT_HTTP_GET_COMPLETE,
    DRVWIFI_EVT_HTTP_POST_COMPLETE,
    DRVWIFI_EVT_HTTP_DISCONNECT_COMPLETE,
    DRVWIFI_EVT_DISCONNECTED,
};

// セキュリティ
enum {
    DRVWIFI_SECURITY_NONE = 0,
    DRVWIFI_SECURITY_WPA,
    DRVWIFI_SECURITY_WPA2,
};

/*
 * 公開関数
 */

/* ペリフェラル初期化 */
void drvwifi_peripheral_initialize();

/* ドライバ初期化 */
void drvwifi_initialize(DRVWIFI_CALLBACK_T callback);

/* wifiモジュールの通信確認 */
void drvwifi_device_available();

/* wifiモジュールの出荷初期化 */
void drvwifi_factory_reset();

/* wifiモジュールのリセット */
void drvwifi_software_reset();

/* APを検索 */
void drvwifi_ap_scan(DRVWIFI_AP_INFO* ap, size_t num_ap);

/* APに接続 */
void drvwifi_ap_connect(const uint8_t* essid, size_t essid_len, const uint8_t* pass, size_t pass_len);

/* WPSでAPに接続 */
void drvwifi_ap_connect_wps();

/* HTTPS 接続 */
void drvwifi_https_connect();

/* HTTPS GET */
void drvwifi_https_get();

/* HTTPS POST */
void drvwifi_https_post();

/* HTTPS 切断 */
void drvwifi_https_disconnect();

/*
 * 内部関数
 */
// タスク
void drvwifi_task(intptr_t exinf);

// 受信タスク
void drvwifi_rx_task(intptr_t exinf);

