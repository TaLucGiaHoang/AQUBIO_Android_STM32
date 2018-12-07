#pragma once

#include <t_stddef.h>

/*
 * プリプロセッサマクロ
 */

// イベント追加データサイズ
//#define APLEVT_EXTRA_DATA_SIZE 32

// メモリプールブロック数
#define APLEVT_NUM_MPF_BLOCKS 32

/*
 * 型定義
 */

// アプリ間イベント
enum {
    APLEVT_INVALID = 0,
    APLEVT_AUTH_PREPARE_REQ,
    APLEVT_AUTH_READY,
    APLEVT_AUTH_PROCESSING,
    APLEVT_AUTH_COMPLETE,
    APLEVT_AUTH_RETRYING,
    APLEVT_REGISTRATION_PREPARE_REQ,
    APLEVT_REGISTRATION_READY,
    APLEVT_REGISTRATION_PROCESSING,
    APLEVT_REGISTRATION_COMPLETE,
    APLEVT_STORAGE_PROCESSING,
    APLEVT_STORAGE_COMPLETE,
    APLEVT_BLE_ENABLE,
    APLEVT_BLE_CONNECTION_ESTABLISHED,
    APLEVT_BLE_START_REMOTEOP,
    APLEVT_BLE_END_REMOTEOP,
    APLEVT_BLE_DISCONNECTED,
    APLEVT_BLE_OPEN_LOCK,
    APLEVT_USER_BLE_ENABLE,
    APLEVT_BATTERY_LOW,
    APLEVT_LOCK_ERROR,
    APLEVT_VAREG_REQ,
    APLEVT_LOCALEVT_START,	// 末尾に置く事
};

//#define VAREG_REQ_NAMELEN	32
typedef union {
    struct {
        int index;
    } reg_prep_req;
    uint32_t padding[32];
} APLEVT_EXTRA_DATA_T;

// イベント
typedef struct {
    uint32_t code;								// イベントコード
    int32_t error;								// エラー
    //uint8_t extra_data[APLEVT_EXTRA_DATA_SIZE];	// 追加データ
    //size_t extra_data_size;						// 追加データのサイズ
    APLEVT_EXTRA_DATA_T extra_data;
    int32_t ref_count;							// 参照カウント
} APLEVT_EVENT_T;

// イベント受信関数
typedef int32_t (*APLEVT_EVENT_RECEIVER_FUNC_T)(APLEVT_EVENT_T* event);

// 初期化関数
typedef int32_t (*APLEVT_INITIALIZE_FUNC_T)(APLEVT_EVENT_RECEIVER_FUNC_T receiver_func);

/*
 * 公開関数
 */

/*
 * イベントオブジェクトを生成する
 */
int32_t aplevt_create_event(APLEVT_EVENT_T** event, uint32_t code, int32_t error, const APLEVT_EXTRA_DATA_T* extra_data);

/*
 * イベントデータ領域をメモリプールに返却する
 */
int32_t aplevt_return_event(APLEVT_EVENT_T* event);

/*
 * イベントをキューイングする
 */
int32_t aplevt_queue_event(ID id_dtq, APLEVT_EVENT_T* event, TMO timeout);

/*
 * データキューからイベントを受け取る
 */
int32_t aplevt_receive_event(ID id_dtq, APLEVT_EVENT_T** event, TMO timeout);

/*
 * イベント生成→送信→返却
 */
int32_t aplevt_send_event(uint32_t code, int32_t error, const APLEVT_EXTRA_DATA_T* extra_data, APLEVT_EVENT_RECEIVER_FUNC_T receiver);

