#include "bootloader.h"

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"

#include "drvcmn_initialize.h"
#include "mdlstrg.h"
#include "aplmain.h"
#include "drviflx.h"

#if 1
#define DBGLOG0(msg)                    syslog(LOG_NOTICE, "[BOOTLOADER]" msg)
#define DBGLOG1(msg, arg1)              syslog(LOG_NOTICE, "[BOOTLOADER]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)        syslog(LOG_NOTICE, "[BOOTLOADER]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)  syslog(LOG_NOTICE, "[BOOTLOADER]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

#define PROGRAM_ADDRESS ADDR_FLASH_SECTOR_5
#define MAX_PROGRAM_SIZE 0x00040000

#define MAX_BUFFER_SIZE 1024

static const FLGPTN FLGPTN_BOOTLOAD_INITIALIZE_COMPLETE = (0x1 << 0);
static const FLGPTN FLGPTN_DRVIFLX_PROTECT_COMPLETE =     (0x1 << 1);
static const FLGPTN FLGPTN_DRVIFLX_ERASE_COMPLETE =       (0x1 << 2);


static void drviflx_callback(int event, intptr_t opt1, intptr_t opt2);

void bootloader(intptr_t exinf)
{
    ER er;
    FLGPTN flgptn = 0;
    DBGLOG0("bootloader() starts.");

    bool_t is_new_firmware = false;
    uint32_t program_size;
    uint32_t start_address = 0;
    uint32_t write_length;
    uint8_t buffer[MAX_BUFFER_SIZE];

    //TODO get is_new_firmware flag
    is_new_firmware = false; // <- change it

    if (is_new_firmware) {
        // Initialize internal flash driver
        drviflx_initialize(0);

        // TODO get program size
        program_size = 1024; // <- change it
        assert(program_size < MAX_PROGRAM_SIZE);

        // Disable Write-protect for the program area
        clr_flg(FLG_BOOTLOAD, ~FLGPTN_DRVIFLX_PROTECT_COMPLETE);
        drviflx_protect(PROGRAM_ADDRESS, program_size, false, drviflx_callback);
        er = twai_flg(FLG_BOOTLOAD, FLGPTN_DRVIFLX_PROTECT_COMPLETE, TWF_ANDW, &flgptn, 3000);
        assert(er == E_OK);

        // Erase program area
        DBGLOG0("Erase program area");
        clr_flg(FLG_BOOTLOAD, ~FLGPTN_DRVIFLX_ERASE_COMPLETE);
        drviflx_erase(PROGRAM_ADDRESS, program_size, drviflx_callback);
        er = twai_flg(FLG_BOOTLOAD, FLGPTN_DRVIFLX_ERASE_COMPLETE, TWF_ANDW, &flgptn, 3000);
        assert(er == E_OK);

        while (start_address < program_size) {
            write_length = program_size - start_address;
            if (write_length > MAX_BUFFER_SIZE) {
                write_length = MAX_BUFFER_SIZE;
            }

            // TODO load write_length bytes from external flash at start_address to buffer
            for (int i = 0; i < write_length; i++) {
                buffer[i] = (uint8_t)(i & 0xFF);
            }

            // Write data from buffer to program area
            drviflx_writei(PROGRAM_ADDRESS + start_address, buffer, write_length);

            start_address += write_length;
        }
    }

    // DEBUG
//    while (1) {
//        dly_tsk(10);
//    }

    // Go to program area
    aplmain_task(exinf);
}

void drviflx_callback(int event, intptr_t opt1, intptr_t opt2)
{
    switch (event) {
    case DRVIFLX_ERASE_COMPLETE:
        DBGLOG0("DRVIFLX_ERASE_COMPLETE");
        set_flg(FLG_BOOTLOAD, FLGPTN_DRVIFLX_ERASE_COMPLETE);
        break;
    case DRVIFLX_PROTECT_COMPLETE:
        DBGLOG0("DRVIFLX_PROTECT_COMPLETE");
        set_flg(FLG_BOOTLOAD, FLGPTN_DRVIFLX_PROTECT_COMPLETE);
        break;
    }
}
