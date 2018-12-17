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
#define DRVWIFI_ERROR_CANCELLED     2

#define DRVWIFI_MAX_ESSID_LEN		32	// ESSID最大長
#define DRVWIFI_MAX_PASSPHRASE_LEN	64	// パスフレーズ最大長
#define DRVWIFI_MAX_HOSTNAME_LEN	32	// ホスト名最大長
#define DRVWIFI_MAX_EXHDRNAME_LEN	32	// 拡張ヘッダname
#define DRVWIFI_MAX_EXHDRVALUE_LEN	32	// 拡張ヘッダvalue
#define DRVWIFI_MAX_PATH_LEN		240	// パス最大長
#define DRVWIFI_MAX_BODY_LEN		256	// Body最大長
#define DRVWIFI_MAX_IPADR_LEN		48		//IP adress最大長（v4/v6）

/* メモリプールブロック長(内部用) */
#define DRVWIFI_MPFBLK_SIZE		16
#define DRVWIFI_DTQ_SIZE		16 //10

/*
 * 型定義
 */
/* コールバック関数型 */
typedef void (*DRVWIFI_CALLBACK_T)(int32_t evt, int32_t error, intptr_t opt);

/* WIFI接続設定 */
typedef struct {
    uint8_t* essid;
    size_t essid_len;
    uint8_t* passphrase;
    size_t passphrase_len;
    int wpaver;					//WPA ver(1 or 2)
    int cipher;						//暗号アルゴリズム(TKIP or CCMP)
} DRVWIFI_CONFIG;

/* TCP configuration */
typedef struct {
    uint8_t* ip_address;
	size_t ipadr_len; // DRVWIFI_MAX_IPADR_LEN
    uint16_t port;
} DRVWIFI_TCP_CONFIG;

/* HTTPS接続設定 */
typedef struct {
    uint8_t* hostname;
    size_t hostname_len;
    uint8_t* exhdrname;
    size_t exhdrname_len;
    uint8_t* exhdrvalue;
    size_t exhdrval_len;
    int schema;					//0:http, 1:https
} DRVWIFI_HTTPS_CONFIG;

/* HTTPS POST要求 */
typedef struct {
    uint8_t* path;
    size_t path_len;
    uint8_t* body;
    size_t body_len;
} DRVWIFI_HTTPS_REQ;

/* HTTPS POST要求 */
typedef struct {
    uint8_t* ip_address;
    size_t ipadr_len;
} DRVWIFI_PING;

/*
 * 定数
 */
/* コールバック種別 */
enum {
    DRVWIFI_EVT_NONE = 0,
    DRVWIFI_EVT_INITIALIZE_COMPLETE,
    DRVWIFI_EVT_POWER_OFF_COMPLETE,
    // DRVWIFI_EVT_DEVICE_AVAILABLE_COMPLETE,
    // DRVWIFI_EVT_FACTORY_RESET_COMPLETE,
    // DRVWIFI_EVT_RESET_COMPLETE,
    // DRVWIFI_EVT_AP_SCAN_COMPLETE,
    DRVWIFI_EVT_AP_CONNECT_COMPLETE,
    DRVWIFI_EVT_AP_CONNECT_WPS_COMPLETE,
    DRVWIFI_EVT_AP_DISCONNECTED,
    DRVWIFI_EVT_HTTP_CONNECT_COMPLETE,
    DRVWIFI_EVT_HTTP_GET_COMPLETE,
    DRVWIFI_EVT_HTTP_POST_COMPLETE,
    DRVWIFI_EVT_HTTP_DISCONNECT_COMPLETE,
    DRVWIFI_EVT_PING_COMPLETE,
    DRVWIFI_EVT_TCP_CONNECT_COMPLETE,
    DRVWIFI_EVT_TCP_SEND_COMPLETE,
    DRVWIFI_EVT_TCP_RECEIVE_COMPLETE,
    DRVWIFI_EVT_TCP_SERVER_COMPLETE,
    DRVWIFI_EVT_DISCONNECTED,
};

// wpa ver
enum {
    DRVWIFI_SCHEMA_HTTP = 0,
    DRVWIFI_SCHEMA_HTTPS,
};

// セキュリティ
// wpa ver
enum {
    DRVWIFI_SECURITY_NONE = 0,
    DRVWIFI_SECURITY_WPA,
    DRVWIFI_SECURITY_WPA2RNS,
};

// 暗号アルゴリズム
enum {
    DRVWIFI_SECURITY_TKIP = 0,
    DRVWIFI_SECURITY_CCMP,
};

enum {
    DRVWIFI_RESULT_OK = 0,
    DRVWIFI_RESULT_ERROR,
    DRVWIFI_RESULT_BUSY,
    DRVWIFI_RESULT_NO_CARRIER,
    DRVWIFI_RESULT_CONNECT,
};


/*
 * 公開関数
 */

/* ペリフェラル初期化 */
void drvwifi_peripheral_initialize(void);

/* ドライバ初期化 */
void drvwifi_initialize(DRVWIFI_CALLBACK_T callback);

/* wifiモジュールの通信確認 */
void drvwifi_device_available();

/* wifiモジュールの出荷初期化 */
void drvwifi_factory_reset();

/* wifiモジュールのリセット */
void drvwifi_software_reset();

/* wifiモジュールのリセット */
void drvwifi_start(DRVWIFI_CALLBACK_T callback);

/* wifiモジュール電源Off */
void drvwifi_power_off(DRVWIFI_CALLBACK_T callback);

/* APに接続 */
void drvwifi_ap_connect(DRVWIFI_CALLBACK_T callback, DRVWIFI_CONFIG* ap);

/* WPSでAPに接続 */
void drvwifi_ap_connect_wps(DRVWIFI_CALLBACK_T callback);

/* AP切断 */
void drvwifi_ap_disconnect(DRVWIFI_CALLBACK_T callback);

/* TCP client */
void drvwifi_tcp_client(DRVWIFI_CALLBACK_T callback, DRVWIFI_TCP_CONFIG* tcpcfg);

/* TCP server */
void drvwifi_tcp_server(DRVWIFI_CALLBACK_T callback, DRVWIFI_TCP_CONFIG* tcpcfg);

/* Wifi send */
void drvwifi_send(DRVWIFI_CALLBACK_T callback, const uint8_t* data, size_t size);

/* Wifi receive */
void drvwifi_receive(DRVWIFI_CALLBACK_T callback, uint8_t* data, size_t size);

/* HTTPS 接続 */
void drvwifi_https_connect(DRVWIFI_CALLBACK_T callback, DRVWIFI_HTTPS_CONFIG* httpcfg);

/* HTTPS GET */
void drvwifi_https_get();

/* HTTPS POST */
void drvwifi_https_post(DRVWIFI_CALLBACK_T callback, DRVWIFI_HTTPS_REQ* req);

/* HTTPS 切断 */
void drvwifi_https_disconnect(DRVWIFI_CALLBACK_T callback);


/* PING */
void drvwifi_ping(DRVWIFI_CALLBACK_T callback, DRVWIFI_PING* ping);

/*
 * 内部関数
 */
// タスク
void drvwifi_task(intptr_t exinf);

// 受信タスク
void drvwifi_rx_task(intptr_t exinf);

