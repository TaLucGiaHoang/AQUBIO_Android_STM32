#ifndef DRVADC_H__
#define DRVADC_H__

#include <t_stddef.h>

/* コールバック関数型 */
typedef void (*DRVADC_CBFUNC_T)(uint32_t button, int32_t evt);

/* 定数 */
#define DRVADC_EVT_START		1
#define DRVADC_EVT_STOP			2
#define DRVADC_EVT_EOC			3

#define DRVADC_SAMPLING_INTERVAL	50

/*
 * 公開関数
 */

/* 初期化 */
void drvadc_initialize(intptr_t exinf);

/* コールバック設定 */
void drvadc_set_callback(DRVADC_CBFUNC_T callback);

/* ADC開始 */
void drvadc_start();

/* ADC停止 */
void drvadc_stop();

/* 割込みサービスルーチン */
void drvadc_isr(intptr_t exinf);

/* 割込み処理タスク */
void drvadc_task(intptr_t exinf);

/* 周期ハンドラ */
void drvadc_cychdr(intptr_t exinf);

#endif	/* DRVADC_H__ */
