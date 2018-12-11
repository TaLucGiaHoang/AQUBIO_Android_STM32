#include "aplimgproc.h"

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
//#include "va-x.h"
#include "stm32f7xx.h"
#include "sil.h"

#include "calc_test.h"

#define APLIMGPROC_EVT_START 1

/*
 *
 */
void aplimgproc_start()
{
    snd_dtq(APLIMGPROC_DTQID, APLIMGPROC_EVT_START);
}

/*
 *  メインタスク
 */
void aplimgproc_task(intptr_t exinf)
{
	syslog(LOG_NOTICE, "aplimgproc_task() starts (exinf = %d).", (int_t) exinf);

    while (true) {
        int32_t msg = 0;
        ER er_rcv = rcv_dtq(APLIMGPROC_DTQID, (intptr_t*)&msg);
        assert((er_rcv == E_OK) || (er_rcv == E_TMOUT));
        syslog(LOG_NOTICE, "aplimgproc_task() received: (msg = %d).", (int_t) msg);

        switch (msg) {
        case APLIMGPROC_EVT_START:
            for (int32_t i = 0; i < 5; i++) {
                syslog(LOG_NOTICE, "calc_test_main() start.");
                SYSTIM systim_s = 0;
                ER ercd = get_tim(&systim_s);
                assert(ercd == E_OK);
                calc_test_main();
                SYSTIM systim_e = 0;
                ercd = get_tim(&systim_e);
                syslog(LOG_NOTICE, "calc_test_main() end. (%d msec)", systim_e - systim_s);
            }
            break;
        default:
            assert(false);
            break;
        }
    }
}

