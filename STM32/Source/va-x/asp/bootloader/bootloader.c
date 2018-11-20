#include "bootloader.h"
#include "kernel_impl.h"
#include "drviflx.h"
#include "drvqflx.h"

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

#define EFLASH_PROGRAM_AREA_ADDR 0x130000
#define IFLASH_PROGRAM_AREA_ADDR ADDR_FLASH_SECTOR_6 // 0x0008 0000
#define MAX_PROGRAM_SIZE (ADDR_FLASH_SECTOR_END - IFLASH_PROGRAM_AREA_ADDR + 1)

#define MAX_BUFFER_SIZE 4096

typedef struct {
    uint32_t    leb_state;  // 論理イレースブロック状態
} LEB_HEADER_T;

// データブロックヘッダ
typedef struct {
    uint32_t    data_id;    // データID
    size_t      data_len;   // データ長
    uint32_t    db_state;   // データブロック状態
} DB_HEADER_T;

__attribute__ ((section(".ramfunc"))) void boot_load(void)
{
    //DEBUG
    target_initialize();

    drvqflx_initialize_peripherals();

    DBGLOG0("boot_load() starts.");

    bool_t is_new_firmware;
    size_t program_size;
    uint32_t start_address;
    uint32_t write_length;
    uint8_t buffer[MAX_BUFFER_SIZE];
    int i;

    // TODO Initialize external flash driver
    drvqflx_initialize();

    // TODO Change flash into memory mapped mode

    // TODO Get is_new_firmware flag and program size
    is_new_firmware = false;
    program_size = 1024;
    // CODING
//    intptr_t leb_addr = EFLASH_PROGRAM_AREA_ADDR;
//    intptr_t db_addr = 0;
//    DB_HEADER_T* db_header = 0x90000000 | EFLASH_PROGRAM_AREA_ADDR | 0x04;
//    db_addr = leb_addr + sizeof(LEB_HEADER_T);
//    DBGLOG3("QFlash Header %d %d %d", db_header->data_id, db_header->data_len, db_header->db_state);

    volatile uint32_t* tmp = 0x90130000;
    DBGLOG2("QFlash [0x%08x] %08x", tmp, *tmp);
    tmp = 0x90130004;
    DBGLOG2("QFlash [0x%08x] %08x", tmp, *tmp);
    tmp = 0x90130008;
    DBGLOG2("QFlash [0x%08x] %08x", tmp, *tmp);
    tmp = 0x9013000C;
    DBGLOG2("QFlash [0x%08x] %08x", tmp, *tmp);

//    intptr_t db_addr = EFLASH_PROGRAM_AREA_ADDR;
//    DB_HEADER_T db_header = {0};
//    drvqflx_read(&db_header, db_addr, sizeof(DB_HEADER_T));
//
//    DBGLOG3("QFlash Header %d %d %d", db_header.data_id, db_header.data_len, db_header.db_state);


    if (is_new_firmware) {
        DBGLOG1("New firmware found (%d)", program_size);
//        assert(program_size < MAX_PROGRAM_SIZE);

        // Initialize internal flash driver
        drviflx_initialize(0);

        //drviflx_protecti(IFLASH_PROGRAM_AREA_ADDR, program_size, false);

        //        // Disable Write-protect for the program area
        //        clr_flg(FLG_BOOTLOAD, ~FLGPTN_DRVIFLX_PROTECT_COMPLETE);
        //        drviflx_protect(PROGRAM_ADDRESS, program_size, false, drviflx_callback);
        //        er = twai_flg(FLG_BOOTLOAD, FLGPTN_DRVIFLX_PROTECT_COMPLETE, TWF_ANDW, &flgptn, 3000);
        //        assert(er == E_OK);

        // Erase program area
        DBGLOG0("Erase program area");
        drviflx_erase(IFLASH_PROGRAM_AREA_ADDR, program_size);

//        drviflx_erase(ADDR_FLASH_SECTOR_0, program_size);

        uint32_t* tmp = FLASHAXI_BASE + IFLASH_PROGRAM_AREA_ADDR;
        DBGLOG2("Mem at 0x%08x: %08x (Before writing)", tmp, *tmp);

        start_address = 0;
        while (start_address < program_size) {
            write_length = program_size - start_address;
            if (write_length > MAX_BUFFER_SIZE) {
                write_length = MAX_BUFFER_SIZE;
            }

            // TODO Load [write_length] bytes from external flash at [start_address] to buffer
            for (i = 0; i < write_length; i++) {
                buffer[i] = i;
            }

            // Write data from buffer to program area
            drviflx_writei(IFLASH_PROGRAM_AREA_ADDR + start_address, buffer, write_length);

            start_address += write_length;
        }

        DBGLOG2("Mem at 0x%08x: %08x (After writing)", tmp, *tmp);

        // TODO Clear is_new_firmware flag here (or we could disable it when program started)
    }

    //sta_ker();
}
