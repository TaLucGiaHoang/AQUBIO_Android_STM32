#include "bootloader.h"
#include "kernel_impl.h"
#include "drviflx.h"
#include "drvqflx.h"
#include "drvcmn.h"

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

#define SERIAL_PORT_ID  1

#define EFLASH_PROGRAM_AREA_ADDR 0x130000
#define IFLASH_PROGRAM_AREA_ADDR ADDR_FLASH_SECTOR_5 // 0x0004 0000
#define MAX_PROGRAM_SIZE (ADDR_FLASH_SECTOR_END - IFLASH_PROGRAM_AREA_ADDR + 1)

#define MAX_BUFFER_SIZE 1024

typedef struct {
    uint32_t    leb_state;  // 論理イレースブロック状態
} LEB_HEADER_T;

// データブロックヘッダ
typedef struct {
    uint32_t    data_id;    // データID
    size_t      data_len;   // データ長
    uint32_t    db_state;   // データブロック状態
} DB_HEADER_T;

static const uint32_t BLOCK_STATE_ERASED        =   ~(uint32_t)0x0; // 初期状態
static const uint32_t BLOCK_STATE_EMPTY         =   ~(uint32_t)0x1; // 空
static const uint32_t BLOCK_STATE_VALID         =   ~(uint32_t)0x3; // 有効データあり
static const uint32_t BLOCK_STATE_FULL          =   ~(uint32_t)0x7; // データフル
static const uint32_t BLOCK_STATE_INVALID       =   ~(uint32_t)0xF; // 無効(削除待ち)

static __inline__ void NVIC_SystemReset(void)
{
    uint32_t reg_value;
    reg_value = ((uint32_t) 0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
    drvcmn_setreg32(TADR_SCB_BASE + TOFF_SCB_AIRCR, 0, SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_SYSRESETREQ_Msk, reg_value);

    while (1) {}
}

void boot_load(void)
{
    // Initialize uart for debugging log
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

    // TODO Get is_new_firmware flag and program size
//    DB_HEADER_T* db_header = QSPI_BASE + EFLASH_PROGRAM_AREA_ADDR + 4;
//    DBGLOG3("QFlash Header %d %d %04x", db_header->data_id, db_header->data_len, db_header->db_state);
//    is_new_firmware = (db_header->db_state == BLOCK_STATE_VALID);
//    program_size = db_header->data_len;

    DB_HEADER_T db_header = {0};
    drvqflx_read(&db_header, EFLASH_PROGRAM_AREA_ADDR + 4, sizeof(DB_HEADER_T));
    DBGLOG3("QFlash Header %d %d %04x", db_header.data_id, db_header.data_len, db_header.db_state);
    is_new_firmware = (db_header.db_state == BLOCK_STATE_VALID);
    program_size = db_header.data_len;

    //DEBUG
//    is_new_firmware = false;
    uint32_t db_addr = EFLASH_PROGRAM_AREA_ADDR + 16;

    if (is_new_firmware) {
        DBGLOG1("New firmware found (%d)", program_size);

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

        start_address = 0;
        while (start_address < program_size) {
            write_length = program_size - start_address;
            if (write_length > MAX_BUFFER_SIZE) {
                write_length = MAX_BUFFER_SIZE;
            }

            // TODO Load [write_length] bytes from external flash at [start_address] to buffer
            drvqflx_read(buffer, db_addr + start_address, write_length);

            // Write data from buffer to program area
            drviflx_writei(IFLASH_PROGRAM_AREA_ADDR + start_address, buffer, write_length);

            start_address += write_length;
        }

        // Clear is_new_firmware flag
        db_header.db_state = BLOCK_STATE_INVALID;
        drvqflx_write(EFLASH_PROGRAM_AREA_ADDR + 4, &db_header, sizeof(db_header));

        // Reset board
        DBGLOG0("Reset...");
        NVIC_SystemReset();
        return;
    }

    sta_ker();
}
