/*
 * VA-X アプリ間イベント
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/01/19 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "aplevt.h"

#include <string.h>
#include <kernel.h>
#include <syssvc/syslog.h>
#include "kernel_cfg.h"

/*
 * 公開関数
 */

/*
 * イベントオブジェクトを生成する
 */
int32_t aplevt_create_event(APLEVT_EVENT_T** pevent, uint32_t code, int32_t error, const APLEVT_EXTRA_DATA_T* extra_data)
{
    assert(pevent);

    APLEVT_EVENT_T* event = NULL;
    ER ercd = tget_mpf(MPF_APLEVT, (void**)&event, 0);
    assert(ercd == E_OK);
    syslog(LOG_NOTICE, "aplevt_create_event: get_mpf => 0x%08p", event);

    event->code = code;
    event->error = error;

    if (extra_data) {
        /* 追加データがあればコピー */
        //assert(size <= APLEVT_EXTRA_DATA_SIZE);
        memcpy(&(event->extra_data), extra_data, sizeof(APLEVT_EXTRA_DATA_T));
        //event->extra_data_size = size;
    } else {
        //event->extra_data_size = 0;
    }

    /* 参照カウントを1にする */
    event->ref_count = 1;

    *pevent = event;
    return ercd;
}

/*
 * イベントデータ領域をメモリプールに返却する
 */
int32_t aplevt_return_event(APLEVT_EVENT_T* event)
{
    assert(event);

    ER ercd = E_OK;

    bool_t not_in_use = false;
    {
        /* 参照カウントのロック獲得 */
        ercd = wai_sem(SEM_APLEVT_REFCOUNT);
        assert(ercd == E_OK);

        --(event->ref_count);
        assert(event->ref_count >= 0);
        if (event->ref_count == 0) {
            not_in_use = true;
        }

        /* 参照カウントのロック開放 */
        ercd = sig_sem(SEM_APLEVT_REFCOUNT);
        assert(ercd == E_OK);
    }

    /* 使用中でなければ開放する */
    if (not_in_use) {
        ercd = rel_mpf(MPF_APLEVT, (void*)event);
        assert(ercd == E_OK);
        syslog(LOG_NOTICE, "aplevt_return_event: rel_mpf(0x%08p)", event);
    }

    return 0;
}

/*
 * イベントをキューイングする
 */
int32_t aplevt_queue_event(ID id_dtq, APLEVT_EVENT_T* event, TMO timeout)
{
    assert(event);

    ER ercd = E_OK;

    {
        /* 参照カウントのロック獲得 */
        ercd = wai_sem(SEM_APLEVT_REFCOUNT);
        assert(ercd == E_OK);

        ++(event->ref_count);

        /* 参照カウントのロック開放 */
        ercd = sig_sem(SEM_APLEVT_REFCOUNT);
        assert(ercd == E_OK);
    }
    
    ercd = tsnd_dtq(id_dtq, (intptr_t)event, timeout);
    assert(ercd == E_OK || ercd == E_TMOUT);

    return ercd;
}

/*
 * データキューからイベントを受け取る
 */
int32_t aplevt_receive_event(ID id_dtq, APLEVT_EVENT_T** event, TMO timeout)
{
    assert(event);

    ER ercd = E_OK;
    APLEVT_EVENT_T* p_data = NULL;

    ercd = trcv_dtq(id_dtq, (intptr_t*)&p_data, timeout);
    assert((ercd == E_OK) || (ercd == E_TMOUT));

    *event = p_data;
    return ercd;
}

/*
 * イベント生成→送信→返却
 */
int32_t aplevt_send_event(uint32_t code, int32_t error, const APLEVT_EXTRA_DATA_T* extra_data, APLEVT_EVENT_RECEIVER_FUNC_T receiver)
{
    ER ercd = E_OK;
    APLEVT_EVENT_T* event = NULL;

    /* 生成 */
    ercd = aplevt_create_event(&event, code, error, extra_data);
    assert(ercd == E_OK);

    /* 送信 */
    ercd = receiver(event);

    /* 返却 */
    aplevt_return_event(event);

    return 0;
}

