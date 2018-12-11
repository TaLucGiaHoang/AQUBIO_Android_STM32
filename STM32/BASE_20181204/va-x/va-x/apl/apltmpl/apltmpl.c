/*
 * タスクテンプレート
 */

// ソースの対になるヘッダ
#include "apltmpl.h"

// 標準C

// OS関係
#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"

// その他ヘッダ

/*
 *  メインタスク
 */
void apltmpl_task(intptr_t exinf)
{
	syslog(LOG_NOTICE, "apltmpl_task() starts (exinf = %d).", (int_t) exinf);

    while (true) {
        int32_t msg = 0;
        ER er_rcv = rcv_dtq(DTQID_APLTMPL, (intptr_t*)&msg);
        assert(er_rcv == E_OK);
        syslog(LOG_NOTICE, "apltmpl_task() received: (msg = %d).", (int_t) msg);

        switch (msg) {
        case APLTMPL_EVT_XXX:
            break;
        default:
            assert(false);
            break;
        }
    }
}

