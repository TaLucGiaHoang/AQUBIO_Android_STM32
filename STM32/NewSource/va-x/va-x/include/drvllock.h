/*
 * VA-X 鍵錠ドライバ (Linkey対応)
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/07/16 Takakashi Otsuka <otsuka-takashi@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>

/*
 * マクロ
 */


/* エラー種別 */
#define DRVLLOCK_ERROR_NONE			0
#define DRVLLOCK_ERROR_TIMEOUT		1

/* メモリプールブロック長(内部用) */
#define DRVLLOCK_MPFBLK_SIZE		16
#define DRVLLOCK_DTQ_SIZE		10

#define LNKY_HEADER_SIZE			12	/* Fakebit(1) + STX bit(1) + Length(1) type(1) + data(n) checksum(1) + stp bit(1) */
/*
 * 型定義
 */
/* コールバック関数型 */
typedef void (*DRVLLOCK_CBFUNC_T)(int32_t evt, int32_t error);
typedef void (*DRVLLOCK_CALLBACK_T)(int32_t evt, int32_t error);

/*
 * 定数
 */
/* コールバック種別 */
enum {
    DRVLLOCK_EVT_CONNECT_COMPLETE = 0,
    DRVLLOCK_EVT_OPEN_STATUS,
    DRVLLOCK_EVT_CLOSE_STATUS,
    DRVLLOCK_EVT_SUMTURN_OCCUR,
    DRVLLOCK_EVT_OPEN_CMDCOMP,
    DRVLLOCK_EVT_OPEN_ERROR,
    DRVLLOCK_EVT_CONECT_COMP,
    DRVLLOCK_EVT_CONECT_FALE,
    DRVLLOCK_EVT_DEDVOLT_CLOSE,
    DRVLLOCK_EVT_DEDVOLT_OPEN,
    DRVLLOCK_EVT_DOOR_OPENISR,
    DRVLLOCK_EVT_DOOR_CLOSEISR,
    DRVLLOCK_EVT_BATERRY_ERROR,
};

/*
 * 公開関数
 */

/* ペリフェラル初期化 */
void drvllock_initialize_peripherals();

/* ドライバ初期化 */
void drvllock_initialize(DRVLLOCK_CALLBACK_T callback);
void open_command_msg();
//void close_command(DRVLLOCK_CBFUNC_T callback);
void reset_cmd_msg();
void status_mgnt_command();
/*
void status_dedvlt_command(const uint8_t* data, size_t size);
*/
void status_chek_command();

int uart_receive(uint8_t* data, size_t data_len, TMO timeout);
int uart_err_receive(uint8_t* data, size_t size, int32_t timeout);
/*
void status_command(DRVLLOCK_CBFUNC_T callback);
void absence_command(DRVLLOCK_CBFUNC_T callback);
void absence_rq__command(DRVLLOCK_CBFUNC_T callback);
void rcv_command(DRVLLOCK_CBFUNC_T callback);
void dedvlt_rq_command(DRVLLOCK_CBFUNC_T callback);
void mgnet_rq_command(DRVLLOCK_CBFUNC_T callback);
void status_mgn_command(DRVLLOCK_CALLBACK_T callback);
*/
/*
 * 内部関数
 */

int32_t drvlilock_sensor_isr(uint32_t pinno);

//void drvllock_initialize_peripherals();

//void drvllock_initialize(DRVLLOCK_CALLBACK_T callback);

// タスク
void drvllock_task(intptr_t exinf);

// 受信タスク
void drvllock_rx_task(intptr_t exinf);

//static uint8_t l_header_buf[LNKY_HEADER_SIZE];
