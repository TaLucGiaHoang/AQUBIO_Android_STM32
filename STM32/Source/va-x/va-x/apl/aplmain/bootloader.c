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

#define PROGRAM_ADDRESS ADDR_FLASH_SECTOR_6 // 0x0008 0000
#define MAX_PROGRAM_SIZE (ADDR_FLASH_SECTOR_END - PROGRAM_ADDRESS + 1)

#define MAX_BUFFER_SIZE 4096

static struct {
    intptr_t opt1;
    intptr_t opt2;
} s_context;

static const FLGPTN FLGPTN_BOOTLOAD_INITIALIZE_COMPLETE = (0x1 << 0);
static const FLGPTN FLGPTN_DRVIFLX_PROTECT_COMPLETE =     (0x1 << 1);
static const FLGPTN FLGPTN_DRVIFLX_ERASE_COMPLETE =       (0x1 << 2);
static const FLGPTN FLGPTN_DRVIFLX_WRITE_COMPLETE =       (0x1 << 3);
static const FLGPTN FLGPTN_MDLSTRG_INITIALIZE_COMPLETE =  (0x1 << 4);
static const FLGPTN FLGPTN_MDLSTRG_REQUEST_COMPLETE =     (0x1 << 5);

static void mdlstrg_store_program_read(uint8_t* data, uint32_t address, size_t size, int index);
static bool_t mdlstrg_store_program_exist(size_t* size, int index);
static void mdlstrg_store_program_delete(size_t size, int index);
static void drviflx_callback(int event, intptr_t opt1, intptr_t opt2);
static void mdlstrg_callback(int event, intptr_t opt1, intptr_t opt2);

void bootloader(intptr_t exinf)
{
    ER er;
    FLGPTN flgptn = 0;
    DBGLOG0("bootloader() starts.");

    bool_t is_new_firmware = false;
    size_t program_size;
    uint32_t start_address = 0;
    uint32_t write_length;
    uint8_t buffer[MAX_BUFFER_SIZE];

    drvcmn_initialize_peripherals();

    clr_flg(FLG_BOOTLOAD, ~FLGPTN_MDLSTRG_INITIALIZE_COMPLETE);
    mdlstrg_initialize(mdlstrg_callback);
    er = twai_flg(FLG_BOOTLOAD, FLGPTN_MDLSTRG_INITIALIZE_COMPLETE, TWF_ANDW, &(FLGPTN ) { 0 }, 10000);
    assert(er == E_OK);

    // Get is_new_firmware flag and program size
    is_new_firmware = mdlstrg_store_program_exist(&program_size, 0);

    if (is_new_firmware) {
        DBGLOG1("New firmware found (%d)", program_size);
        assert(program_size < MAX_PROGRAM_SIZE);

        // Initialize internal flash driver
        drviflx_initialize(0);

//        // Disable Write-protect for the program area
//        clr_flg(FLG_BOOTLOAD, ~FLGPTN_DRVIFLX_PROTECT_COMPLETE);
//        drviflx_protect(PROGRAM_ADDRESS, program_size, false, drviflx_callback);
//        er = twai_flg(FLG_BOOTLOAD, FLGPTN_DRVIFLX_PROTECT_COMPLETE, TWF_ANDW, &flgptn, 3000);
//        assert(er == E_OK);

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

            // Load [write_length] bytes from external flash at [start_address] to buffer
            mdlstrg_store_program_read(buffer, start_address, write_length, 0);

            // Write data from buffer to program area
            clr_flg(FLG_BOOTLOAD, ~FLGPTN_DRVIFLX_WRITE_COMPLETE);
            drviflx_write(PROGRAM_ADDRESS + start_address, buffer, write_length, drviflx_callback);
            er = twai_flg(FLG_BOOTLOAD, FLGPTN_DRVIFLX_WRITE_COMPLETE, TWF_ANDW, &flgptn, 3000);
            assert(er == E_OK);

            start_address += write_length;
        }

        // Disable block
        mdlstrg_store_program_delete(0, 0);
    }

    // Go to program area
    aplmain_task(exinf);
}

void mdlstrg_store_program_read(uint8_t* data, uint32_t address, size_t size, int index)
{
    DBGLOG0("mdlstrg_store_program_read");
    ER er;

    assert(data);

    MDLSTRG_REQUEST_T UPDATE_REQ = {
        .data_type = MDLSTRG_DATA_TYPE_STORE_PROGRAM,
        .request_type = MDLSTRG_REQ_TYPE_READ,
        .data = (intptr_t)data,
        .size = size,
        .opt1 = index,
        .opt2 = address,
    };
    clr_flg(FLG_BOOTLOAD, ~FLGPTN_MDLSTRG_REQUEST_COMPLETE);
    mdlstrg_request(&UPDATE_REQ, mdlstrg_callback);
    er = twai_flg(FLG_BOOTLOAD, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 10000);
    assert(er == E_OK);
}

bool_t mdlstrg_store_program_exist(size_t* size, int index)
{
    DBGLOG0("mdlstrg_store_program_read");
    ER er;

    MDLSTRG_REQUEST_T UPDATE_REQ = {
        .data_type = MDLSTRG_DATA_TYPE_STORE_PROGRAM,
        .request_type = MDLSTRG_REQ_TYPE_EXISTS,
        .opt1 = index,
        .opt2 = (intptr_t)size,
    };
    clr_flg(FLG_BOOTLOAD, ~FLGPTN_MDLSTRG_REQUEST_COMPLETE);
    mdlstrg_request(&UPDATE_REQ, mdlstrg_callback);
    er = twai_flg(FLG_BOOTLOAD, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 10000);
    assert(er == E_OK);

    return (bool_t) s_context.opt1;
}

void mdlstrg_store_program_delete(size_t size, int index)
{
    DBGLOG0("mdlstrg_store_program_read");
    ER er;

    MDLSTRG_REQUEST_T UPDATE_REQ = {
        .data_type = MDLSTRG_DATA_TYPE_STORE_PROGRAM,
        .request_type = MDLSTRG_REQ_TYPE_DELETE,
        .size = size,
        .opt1 = index,
    };
    clr_flg(FLG_BOOTLOAD, ~FLGPTN_MDLSTRG_REQUEST_COMPLETE);
    mdlstrg_request(&UPDATE_REQ, mdlstrg_callback);
    er = twai_flg(FLG_BOOTLOAD, FLGPTN_MDLSTRG_REQUEST_COMPLETE, TWF_ANDW, &(FLGPTN){0}, 10000);
    assert(er == E_OK);
}

void mdlstrg_callback(int event, intptr_t opt1, intptr_t opt2)
{
    DBGLOG1("mdlstrg_callback: e=%d", event);
    switch(event) {
    case MDLSTRG_EVT_INITIALIZE_COMPLETE:
        DBGLOG0("mdlstrg_callback: MDLSTRG_EVT_INITIALIZE_COMPLETE");
        set_flg(FLG_BOOTLOAD, FLGPTN_MDLSTRG_INITIALIZE_COMPLETE);
        break;
    case MDLSTRG_EVT_REQUEST_COMPLETE:
        DBGLOG0("mdlstrg_callback: MDLSTRG_EVT_REQUEST_COMPLETE");
        set_flg(FLG_BOOTLOAD, FLGPTN_MDLSTRG_REQUEST_COMPLETE);
        break;
    default:
        assert(false);
        break;
    }

    s_context.opt1 = opt1;
    s_context.opt2 = opt2;
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
    case DRVIFLX_WRITE_COMPLETE:
        DBGLOG0("DRVIFLX_WRITE_COMPLETE");
        set_flg(FLG_BOOTLOAD, FLGPTN_DRVIFLX_WRITE_COMPLETE);
        break;
    }
}
