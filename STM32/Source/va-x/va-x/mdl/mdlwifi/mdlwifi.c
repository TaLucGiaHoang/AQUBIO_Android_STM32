/*
 * VA-X Wifiミドル
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/03/29 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "mdlwifi.h"

#include <string.h>

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "kernel_cfg.h"

#include "drvwifi.h"

/*
 * マクロ定義
 */

// デバッグログ
#if 1
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[MDLWIFI]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[MDLWIFI]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[MDLWIFI]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[MDLWIFI]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

// サービスコールのエラーのログ出力
static inline void svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}
#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))

/*
 * 型定義
 */


/*
 * 内部関数プロトタイプ
 */
static void drvwifi_callback(int32_t evt, int32_t error, intptr_t opt);

/*
 * 定数定義
 */
// イベントフラグ
static const FLGPTN FLGPTN_DRVWIFI_INITIALIZE_COMPLETE =		(0x1 << 0);

static const TMO DRV_CALLBACK_TIMEOUT	=	3000;


/*********************************************************************
 * 公開関数
 ********************************************************************/

/*
 * 初期化
 */
void mdlwifi_initialize(MDLWIFI_CALLBACK_T callback)
{
    // タスクを起動
    ER er = E_OK;
    er = act_tsk(TSK_MDLWIFI);
    assert(er == E_OK);
}

/*
 * タスク
 */
void mdlwifi_task(intptr_t exinf)
{
    DBGLOG0("mdlwifi_task");

}

/*********************************************************************
 * 内部関数
 ********************************************************************/

/*
 * Wifiドライバコールバック
 */
void drvwifi_callback(int32_t evt, int32_t error, intptr_t opt)
{
    DBGLOG3("drvwifi_callback(evt=%d,error=%d,opt=0x%08x)", evt, error, opt);

    ER er = 0;

    switch(evt) {
    case DRVWIFI_EVT_INITIALIZE_COMPLETE:
        er = set_flg(FLG_MDLWIFI, FLGPTN_DRVWIFI_INITIALIZE_COMPLETE);
        assert(er == E_OK);
        break;
    default:
        assert(false);
        break;
    }

}
