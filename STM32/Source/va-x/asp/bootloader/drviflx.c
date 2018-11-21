/************************************************************************/
/* Internal Flash Driver                                                */
/* File name  : drviflx.h                                               */
/* DATE       : Jan 03, 2018                                            */
/* CPU TYPE   : STM32F765VI                                             */
/*                                                                      */
/*                                                                      */
/* Copyright (c) 2017 SH Consulting K.K.                                */
/* and SH Consulting VietNam. All Rights Reserved.                      */
/************************************************************************/

#include <kernel.h>
#include <t_syslog.h>
#include <string.h>
#include "drviflx.h"
#include "drviflx_hal.h"
#include "drvcmn.h"

#define RAMFUNC __attribute__ ((section(".ramfunc")))

#if 1
#define DUMP_REG32(name, addr) syslog(LOG_NOTICE, name " (0x%08p): => 0x%08x", addr, drvcmn_getreg32(addr, 0, UINT32_MAX));
#else
#define DUMP_REG32(name, addr)
#endif

#if 0
#define DBGLOG0(msg)                    syslog(LOG_NOTICE, "[DRVIFLX]" msg)
#define DBGLOG1(msg, arg1)              syslog(LOG_NOTICE, "[DRVIFLX]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)        syslog(LOG_NOTICE, "[DRVIFLX]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)  syslog(LOG_NOTICE, "[DRVIFLX]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

/* Const */
#if defined(TOPPERS_STM32F7_BNVA)
static const intptr_t DRVIFLX_ADDR_FLASH_SECTOR[] = {
    ADDR_FLASH_SECTOR_0,
    ADDR_FLASH_SECTOR_1,
    ADDR_FLASH_SECTOR_2,
    ADDR_FLASH_SECTOR_3,
    ADDR_FLASH_SECTOR_4,
    ADDR_FLASH_SECTOR_5,
    ADDR_FLASH_SECTOR_6,
    ADDR_FLASH_SECTOR_7,
    ADDR_FLASH_SECTOR_8,
    ADDR_FLASH_SECTOR_9,
    ADDR_FLASH_SECTOR_10,
    ADDR_FLASH_SECTOR_11,
#if defined(DUAL_BANK)
    ADDR_FLASH_SECTOR_12,
    ADDR_FLASH_SECTOR_13,
    ADDR_FLASH_SECTOR_14,
    ADDR_FLASH_SECTOR_15,
    ADDR_FLASH_SECTOR_16,
    ADDR_FLASH_SECTOR_17,
    ADDR_FLASH_SECTOR_18,
    ADDR_FLASH_SECTOR_19,
    ADDR_FLASH_SECTOR_20,
    ADDR_FLASH_SECTOR_21,
    ADDR_FLASH_SECTOR_22,
    ADDR_FLASH_SECTOR_23,
#endif
};

#elif defined(TOPPERS_STM32F7_DISCOVERY)
static const intptr_t DRVIFLX_ADDR_FLASH_SECTOR[] = {
    ADDR_FLASH_SECTOR_0,
    ADDR_FLASH_SECTOR_1,
    ADDR_FLASH_SECTOR_2,
    ADDR_FLASH_SECTOR_3,
    ADDR_FLASH_SECTOR_4,
    ADDR_FLASH_SECTOR_5,
    ADDR_FLASH_SECTOR_6,
    ADDR_FLASH_SECTOR_7,
};
#endif

/* Private variables */
static DRVIFLX_CALLBACK_T s_erase_callback = NULL;
static DRVIFLX_CALLBACK_T s_protect_callback = NULL;
static DRVIFLX_CALLBACK_T s_read_callback = NULL;
static DRVIFLX_CALLBACK_T s_write_callback = NULL;

static volatile bool_t s_flash_initialized = false;
static volatile bool_t s_write_start = false;
static volatile bool_t s_read_start = false;
static volatile bool_t s_protect_start = false;
static volatile bool_t s_erase_complete = false;
static volatile int s_erase_status;

static volatile uint8_t s_status = 0;

static uint8_t* s_read_dest_addr;
static intptr_t s_read_src_addr;
static intptr_t s_read_length;

static uint8_t* s_write_dest_addr;
static intptr_t s_write_src_addr;
static intptr_t s_write_length;

static uint32_t s_erase_first_sector;
static uint32_t s_erase_last_sector;

static uint32_t s_protect_first_sector;
static uint32_t s_protect_last_sector;
static bool_t s_protect_enable;
/* End Private variables */

/* Private functions */
static int get_sector(intptr_t address, size_t* sector, size_t* sector_size);
static int drviflx_flash_addr_ictm(intptr_t* flash_addr, intptr_t* mapped_addr, size_t* sector, size_t* sector_size);
static int drviflx_flash_addr_axim(intptr_t* flash_addr, intptr_t* mapped_addr, size_t* sector, size_t* sector_size);
/* End Private functions */

/*""FUNC COMMENT""********************************************************
 * ID          : Initialize the flash memory
 * Outline     :
 *-------------------------------------------------------------------------
 * Include     : n/a
 *-------------------------------------------------------------------------
 * Declaration : int drvflx_initialize(intptr_t opt)
 *-------------------------------------------------------------------------
 * Description : Initialize the flash memory
 *-------------------------------------------------------------------------
 * Argument    : intptr_t opt   : reversed
 *-------------------------------------------------------------------------
 * Return Value: int: 0x00 - OK
 *             :      0xFF - NG
 *-------------------------------------------------------------------------
 * Notice      :
 *""FUNC COMMENT END""*****************************************************/
int drviflx_initialize(intptr_t opt)
{
    DBGLOG0("drviflx_initialize");
    // Enable Flash global interrupt
//    ER er = ena_int(IRQ_VECTOR_FLASH);
//    assert(er == E_OK);

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    /* Allow Access to option bytes sector */
    HAL_FLASH_OB_Unlock();

    s_flash_initialized = true;

    return DRVIFLX_RES_OK;
}

/*""FUNC COMMENT""********************************************************
 * ID          : Read from flash memory
 * Outline     :
 *-------------------------------------------------------------------------
 * Include     : n/a
 *-------------------------------------------------------------------------
 * Declaration : int drviflx_read(uint8_t* dest, intptr_t src_addr, size_t length, DRVIFLX_CALLBACK_T callback)
 *-------------------------------------------------------------------------
 * Description : Read from flash memory
 *-------------------------------------------------------------------------
 * Argument    : uint8_t* dest               : Address to store the read-data.
 *             : intptr_t src_addr           : Flash memory address to read data
 *             : size_t length               : Size to read
 *             : DRVIFLX_CALLBACK_T callback : Callback function pointer
 *-------------------------------------------------------------------------
 * Return Value: int: 0x00 - OK
 *             :      0xFF - NG
 *-------------------------------------------------------------------------
 * Notice      :
 *""FUNC COMMENT END""*****************************************************/
int drviflx_read(uint8_t* dest, intptr_t src_addr, size_t length, DRVIFLX_CALLBACK_T callback)
{
    intptr_t mapped_addr;
    size_t sector, sector_size;

    if (s_status > 0) {
        return DRVIFLX_RES_NG;
    }
    s_status |= DRVIFLX_READING;

    if (get_sector(src_addr + (uint32_t) length - 1, &sector, &sector_size) != DRVIFLX_RES_OK) {
        s_status &= ~DRVIFLX_READING;
        return DRVIFLX_RES_NG;
    }

    mapped_addr = 0;
    if (drviflx_flash_addr_ictm(&src_addr, &mapped_addr, &sector, &sector_size) != DRVIFLX_RES_OK) {
        s_status &= ~DRVIFLX_READING;
        return DRVIFLX_RES_NG;
    }

    s_read_dest_addr = dest;
    s_read_src_addr = mapped_addr;
    s_read_length = length;

    // Set callback
    s_read_callback = callback;

    s_read_start = true;

    return DRVIFLX_RES_OK;
}

/*""FUNC COMMENT""********************************************************
 * ID          : Read from flash memory (for testing only)
 * Outline     :
 *-------------------------------------------------------------------------
 * Include     : n/a
 *-------------------------------------------------------------------------
 * Declaration : int drviflx_readi(uint8_t* dest, intptr_t src_addr, size_t length)
 *-------------------------------------------------------------------------
 * Description : Read from flash memory
 *-------------------------------------------------------------------------
 * Argument    : uint8_t* dest               : Address to store the read-data.
 *             : intptr_t src_addr           : Flash memory address to read data
 *             : size_t length               : Size to read
 *-------------------------------------------------------------------------
 * Return Value: int: 0x00 - OK
 *             :      0xFF - NG
 *-------------------------------------------------------------------------
 * Notice      :
 *""FUNC COMMENT END""*****************************************************/
int drviflx_readi(uint8_t* dest, intptr_t src_addr, size_t length)
{
    intptr_t mapped_addr;
    size_t sector, sector_size;

    if (s_status > 0) {
        return DRVIFLX_RES_NG;
    }
    s_status |= DRVIFLX_READING;

    if (get_sector(src_addr + (uint32_t) length - 1, &sector, &sector_size) != DRVIFLX_RES_OK) {
        s_status &= ~DRVIFLX_READING;
        return DRVIFLX_RES_NG;
    }

    mapped_addr = 0;
    if (drviflx_flash_addr_ictm(&src_addr, &mapped_addr, &sector, &sector_size) != DRVIFLX_RES_OK) {
        s_status &= ~DRVIFLX_READING;
        return DRVIFLX_RES_NG;
    }

    // Read from flash memory
    memcpy(dest, (uint8_t*) mapped_addr, length);
    s_status &= ~DRVIFLX_READING;

    return DRVIFLX_RES_OK;
}

/*""FUNC COMMENT""********************************************************
 * ID          : Write to flash memory
 * Outline     :
 *-------------------------------------------------------------------------
 * Include     : n/a
 *-------------------------------------------------------------------------
 * Declaration : int drviflx_write(intptr_t src_addr, uint8_t* dest, size_t length, DRVIFLX_CALLBACK_T callback)
 *-------------------------------------------------------------------------
 * Description : Write to flash memory
 *-------------------------------------------------------------------------
 * Argument    : intptr_t src_addr          : Flash memory address to write data
 *             : uint8_t* dest              : Address of write-data.
 *             : size_t length              : Size to write
 *             : DRVIFLX_CALLBACK_T callback : Callback function pointer
 *-------------------------------------------------------------------------
 * Return Value: int: 0x00 - OK
 *             :      0xFF - NG
 *-------------------------------------------------------------------------
 * Notice      :
 *""FUNC COMMENT END""*****************************************************/
int drviflx_write(intptr_t src_addr, uint8_t* dest, size_t length, DRVIFLX_CALLBACK_T callback)
{
    intptr_t mapped_addr;
    size_t sector, sector_size;

    if (s_status > 0) {
        return DRVIFLX_RES_NG;
    }
    s_status |= DRVIFLX_WRITING;

    if (get_sector(src_addr + (uint32_t) length - 1, &sector, &sector_size) != DRVIFLX_RES_OK) {
        s_status &= ~DRVIFLX_WRITING;
        return DRVIFLX_RES_NG;
    }

    mapped_addr = 0;
    if (drviflx_flash_addr_axim(&src_addr, &mapped_addr, &sector, &sector_size) != DRVIFLX_RES_OK) {
        s_status &= ~DRVIFLX_WRITING;
        return DRVIFLX_RES_NG;
    }

    s_write_dest_addr = dest;
    s_write_src_addr = mapped_addr;
    s_write_length = length;

    // Set callback
    s_write_callback = callback;

    s_write_start = true;

    return DRVIFLX_RES_OK;
}

/*""FUNC COMMENT""********************************************************
 * ID          : Write to flash memory without callback function
 * Outline     :
 *-------------------------------------------------------------------------
 * Include     : n/a
 *-------------------------------------------------------------------------
 * Declaration : int drviflx_writei(intptr_t src_addr, uint8_t* dest, size_t length)
 *-------------------------------------------------------------------------
 * Description : Write to flash memory without callback function. (Synchronization process.)
 *-------------------------------------------------------------------------
 * Argument    : intptr_t src_addr          : Flash memory address to write data
 *             : uint8_t* dest              : Address of write-data.
 *             : size_t length              : Size to write
 *-------------------------------------------------------------------------
 * Return Value: int: 0x00 - OK
 *             :      0xFF - NG
 *-------------------------------------------------------------------------
 * Notice      :
 *""FUNC COMMENT END""*****************************************************/
RAMFUNC int drviflx_writei(intptr_t src_addr, uint8_t* dest, size_t length)
{
    intptr_t mapped_addr;
    size_t sector, sector_size;
    int i;

    if (s_status > 0) {
        return DRVIFLX_RES_NG;
    }
    s_status |= DRVIFLX_WRITING;

    if (get_sector(src_addr + (uint32_t) length - 1, &sector, &sector_size) != DRVIFLX_RES_OK) {
        s_status &= ~DRVIFLX_WRITING;
        return DRVIFLX_RES_NG;
    }

    mapped_addr = 0;
    if (drviflx_flash_addr_axim(&src_addr, &mapped_addr, &sector, &sector_size) != DRVIFLX_RES_OK) {
        s_status &= ~DRVIFLX_WRITING;
        return DRVIFLX_RES_NG;
    }

    for (i = 0; i < length; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, mapped_addr + i, dest[i]) != HAL_OK) {
            s_status &= ~DRVIFLX_WRITING;
            return DRVIFLX_RES_NG;
        }
    }

    s_status &= ~DRVIFLX_WRITING;
    return DRVIFLX_RES_OK;
}

/*""FUNC COMMENT""********************************************************
 * ID          : Erase the flash memory
 * Outline     :
 *-------------------------------------------------------------------------
 * Include     : n/a
 *-------------------------------------------------------------------------
 * Declaration : int drviflx_erase(intptr_t addr, size_t length, DRVIFLX_CALLBACK_T callback)
 *-------------------------------------------------------------------------
 * Description : Erase the flash memory
 *-------------------------------------------------------------------------
 * Argument    : intptr_t addr              : Flash memory address to erase data
 *             : size_t length              : Size to erase
 *             : DRVIFLX_CALLBACK_T callback : Callback function pointer
 *-------------------------------------------------------------------------
 * Return Value: int: 0x00 - OK
 *             :      0xFF - NG
 *-------------------------------------------------------------------------
 * Notice      :
 *""FUNC COMMENT END""*****************************************************/
RAMFUNC int drviflx_erase(intptr_t addr, size_t length)
{
    size_t sector_size;
    intptr_t start_address, end_address;

    if (s_status > 0) {
        return DRVIFLX_RES_NG;
    }
    s_status |= DRVIFLX_ERASING;


    start_address = addr;
    end_address = addr + (uint32_t) length - 1;

    /* Get the 1st sector to erase */
    if (get_sector(start_address, &s_erase_first_sector, &sector_size) != DRVIFLX_RES_OK) {
        s_status &= ~DRVIFLX_ERASING;
        return DRVIFLX_RES_NG;
    }

    /* Get the last sector to erase */
    if (get_sector(end_address, &s_erase_last_sector, &sector_size) != DRVIFLX_RES_OK) {
        s_status &= ~DRVIFLX_ERASING;
        return DRVIFLX_RES_NG;
    }

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = s_erase_first_sector;
    EraseInitStruct.NbSectors = s_erase_last_sector - s_erase_first_sector + 1;

    uint32_t sectorError;
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &sectorError) != HAL_OK) {
        s_status &= ~DRVIFLX_ERASING;
        return DRVIFLX_RES_NG;
    }

    s_status &= ~DRVIFLX_ERASING;

    return DRVIFLX_RES_OK;
}

/*""FUNC COMMENT""********************************************************
 * ID          : Write-protect the flash memory
 * Outline     :
 *-------------------------------------------------------------------------
 * Include     : n/a
 *-------------------------------------------------------------------------
 * Declaration : int drviflx_protect(intptr_t addr, size_t length, bool_t protect, DRVIFLX_CALLBACK_T callback)
 *-------------------------------------------------------------------------
 * Description : Write-protect the flash memory
 *-------------------------------------------------------------------------
 * Argument    : intptr_t addr              : Flash memory address to protect data
 *             : size_t length              : Size to protect
 *             : bool_t protect             : Protect on/off
 *             : DRVIFLX_CALLBACK_T callback : Callback function pointer
 *-------------------------------------------------------------------------
 * Return Value: int: 0x00 - OK
 *             :      0xFF - NG
 *-------------------------------------------------------------------------
 * Notice      :
 *""FUNC COMMENT END""*****************************************************/
int drviflx_protect(intptr_t addr, size_t length, bool_t protect, DRVIFLX_CALLBACK_T callback)
{
    size_t sector_size;
    intptr_t start_address, end_address;

    if (s_status > 0) {
        return DRVIFLX_RES_NG;
    }
    s_status |= DRVIFLX_PROTECTING;

    start_address = addr;
    end_address = addr + length - 1;

    /* Get the 1st sector to erase */
    if (get_sector(start_address, &s_protect_first_sector, &sector_size) != DRVIFLX_RES_OK) {
        s_status &= ~DRVIFLX_ERASING;
        return DRVIFLX_RES_NG;
    }

    /* Get the last sector to erase */
    if (get_sector(end_address, &s_protect_last_sector, &sector_size) != DRVIFLX_RES_OK) {
        s_status &= ~DRVIFLX_ERASING;
        return DRVIFLX_RES_NG;
    }

    s_protect_enable = protect;

    // Set callback
    s_protect_callback = callback;

    s_protect_start = true;
    return DRVIFLX_RES_OK;
}

/*""FUNC COMMENT""********************************************************
 * ID          : Address mapping transition
 * Outline     :
 *-------------------------------------------------------------------------
 * Include     : n/a
 *-------------------------------------------------------------------------
 * Declaration : int drviflx_flash_addr(intptr_t* flash_addr, intptr_t* mapped_addr, size_t* sector, size_t* sector_size)
 *-------------------------------------------------------------------------
 * Description : Address mapping transition between physical flash memory address and mapped address (AXIM)
 *               When call this function, please set NULL for one of flash_addr or mapped_addr.
 *               If the mapped_addr is NULL, transition flash_addr to mapped_addr, and put value to the mapped_addr.
 *               And vice versa.
 *-------------------------------------------------------------------------
 * Argument    : intptr_t* flash_addr  : Address in Flash memory
 *             : intptr_t* mapped_addr : Mapped address (AXIM)
 *             : size_t* sector        : Sector number (STM32F765VI: 0..23, STM32F746: 0..7)
 *             : size_t* sector_size   : Sector size
 *-------------------------------------------------------------------------
 * Return Value: int: 0x00 - OK
 *             :      0xFF - NG
 *-------------------------------------------------------------------------
 * Notice      :
 *""FUNC COMMENT END""*****************************************************/
RAMFUNC int drviflx_flash_addr(intptr_t* flash_addr, intptr_t* mapped_addr, size_t* sector, size_t* sector_size)
{
    return drviflx_flash_addr_ictm(flash_addr, mapped_addr, sector, sector_size);
}

/*""FUNC COMMENT""********************************************************
 * ID          : Address mapping transition (AXIM)
 * Outline     :
 *-------------------------------------------------------------------------
 * Include     : n/a
 *-------------------------------------------------------------------------
 * Declaration : int drviflx_flash_addr_axim(intptr_t* flash_addr, intptr_t* mapped_addr, size_t* sector, size_t* sector_size)
 *-------------------------------------------------------------------------
 * Description : Address mapping transition between physical flash memory address and mapped address (AXIM)
 *               When call this function, please set NULL for one of flash_addr or mapped_addr.
 *               If the mapped_addr is NULL, transition flash_addr to mapped_addr, and put value to the mapped_addr.
 *               And vice versa.
 *-------------------------------------------------------------------------
 * Argument    : intptr_t* flash_addr  : Address in Flash memory
 *             : intptr_t* mapped_addr : Mapped address (AXIM)
 *             : size_t* sector        : Sector number
 *             : size_t* sector_size   : Sector size
 *-------------------------------------------------------------------------
 * Return Value: int: 0x00 - OK
 *             :      0xFF - NG
 *-------------------------------------------------------------------------
 * Notice      :
 *""FUNC COMMENT END""*****************************************************/
RAMFUNC static int drviflx_flash_addr_axim(intptr_t* flash_addr, intptr_t* mapped_addr, size_t* sector, size_t* sector_size)
{
    intptr_t base_address = FLASHAXI_BASE;
    intptr_t address;

    if (*mapped_addr == 0) {
        address = *flash_addr;
    } else if (*flash_addr == 0) {
        address = *mapped_addr - base_address;
    } else {
        return DRVIFLX_RES_NG;
    }

    // Get block and block size
    if (DRVIFLX_RES_OK == get_sector(address, sector, sector_size)) {
        if (*mapped_addr == 0) {
            *mapped_addr = address + base_address;
        } else {
            *flash_addr = address;
        }
        return DRVIFLX_RES_OK;
    } else {
        return DRVIFLX_RES_NG;
    }
}

/*""FUNC COMMENT""********************************************************
 * ID          : Address mapping transition (ICTM)
 * Outline     :
 *-------------------------------------------------------------------------
 * Include     : n/a
 *-------------------------------------------------------------------------
 * Declaration : int drviflx_flash_addr_ictm(intptr_t* flash_addr, intptr_t* mapped_addr, size_t* sector, size_t* sector_size)
 *-------------------------------------------------------------------------
 * Description : Address mapping transition between physical flash memory address and mapped address (ICTM)
 *               When call this function, please set NULL for one of flash_addr or mapped_addr.
 *               If the mapped_addr is NULL, transition flash_addr to mapped_addr, and put value to the mapped_addr.
 *               And vice versa.
 *-------------------------------------------------------------------------
 * Argument    : intptr_t* flash_addr  : Address in Flash memory
 *             : intptr_t* mapped_addr : Mapped address (ICTM)
 *             : size_t* sector        : Sector number
 *             : size_t* sector_size   : Sector size
 *-------------------------------------------------------------------------
 * Return Value: int: 0x00 - OK
 *             :      0xFF - NG
 *-------------------------------------------------------------------------
 * Notice      :
 *""FUNC COMMENT END""*****************************************************/
RAMFUNC static int drviflx_flash_addr_ictm(intptr_t* flash_addr, intptr_t* mapped_addr, size_t* sector, size_t* sector_size)
{
    intptr_t base_address = FLASHITCM_BASE;
    intptr_t address;

    if (*mapped_addr == 0) {
        address = *flash_addr;
    } else if (*flash_addr == 0) {
        address = *mapped_addr - base_address;
    } else {
        return DRVIFLX_RES_NG;
    }

    // Get block and block size
    if (DRVIFLX_RES_OK == get_sector(address, sector, sector_size)) {
        if (*mapped_addr == 0) {
            *mapped_addr = address + base_address;
        } else {
            *flash_addr = address;
        }
        return DRVIFLX_RES_OK;
    } else {
        return DRVIFLX_RES_NG;
    }
}

/*""FUNC COMMENT""********************************************************
 * ID          : Get sector and sector size of the address in the flash memory
 * Outline     :
 *-------------------------------------------------------------------------
 * Include     : n/a
 *-------------------------------------------------------------------------
 * Declaration : int get_sector(intptr_t address, size_t* sector, size_t* sector_size)
 *-------------------------------------------------------------------------
 * Description : Get sector and sector size of the address in the flash memory
 *-------------------------------------------------------------------------
 * Argument    : intptr_t address      : Address in Flash memory
 *             : size_t* sector        : Sector number
 *             : size_t* sector_size   : Sector size
 *-------------------------------------------------------------------------
 * Return Value: int: 0x00 - OK
 *             :      0xFF - NG
 *-------------------------------------------------------------------------
 * Notice      :
 *""FUNC COMMENT END""*****************************************************/
RAMFUNC static int get_sector(intptr_t address, size_t* sector, size_t* sector_size)
{
    int i;
    int num_of_sector;

    if (address > ADDR_FLASH_SECTOR_END) {
        return DRVIFLX_RES_NG;
    }

    num_of_sector = sizeof(DRVIFLX_ADDR_FLASH_SECTOR) / sizeof(DRVIFLX_ADDR_FLASH_SECTOR[0]);
    for (i = 1; i < num_of_sector; i++) {
        if (address < DRVIFLX_ADDR_FLASH_SECTOR[i]) {
            *sector = i - 1;
            *sector_size = DRVIFLX_ADDR_FLASH_SECTOR[i] - DRVIFLX_ADDR_FLASH_SECTOR[i - 1];
            return DRVIFLX_RES_OK;
        }
    }

    // Last sector
    *sector = num_of_sector - 1;
    *sector_size = ADDR_FLASH_SECTOR_END - DRVIFLX_ADDR_FLASH_SECTOR[num_of_sector - 1] + 1;

    return DRVIFLX_RES_OK;
}

/*""FUNC COMMENT""********************************************************
 * ID          : Internal Flash Task
 * Outline     :
 *-------------------------------------------------------------------------
 * Include     : n/a
 *-------------------------------------------------------------------------
 * Declaration : void drviflx_task(intptr_t exinf)
 *-------------------------------------------------------------------------
 * Description : Do Internal Flash tasks
 *-------------------------------------------------------------------------
 * Argument    : intptr_t exinf      : reserved
 *-------------------------------------------------------------------------
 * Return Value: none
 *-------------------------------------------------------------------------
 * Notice      :
 *""FUNC COMMENT END""*****************************************************/
void drviflx_task(intptr_t exinf)
{
    int ret;
    FLASH_OBProgramInitTypeDef OBInit;
    int i;

    while (!s_flash_initialized) {
        dly_tsk(1);
    }
    syslog(LOG_NOTICE, "drviflx_task start");
    while (1) {
        if (s_erase_complete) {
            // Callback for DRVIFLX_ERASE_COMPLETE
            s_status &= ~DRVIFLX_ERASING;
            if (s_erase_callback) {
                s_erase_callback(DRVIFLX_ERASE_COMPLETE, s_erase_status, 0);
                s_erase_callback = NULL;
            }
            s_erase_complete = false;
        }

        if (s_write_start) {
            // Write data to flash
            ret = DRVIFLX_RES_OK;
            for (int i = 0; i < s_write_length; i++) {
                if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, s_write_src_addr + i, s_write_dest_addr[i]) != HAL_OK) {
                    syslog(LOG_NOTICE, "drviflx_write error");
                    ret = DRVIFLX_RES_NG;
                    break;
                }
            }

            // Callback for DRVIFLX_WRITE_COMPLETE
            s_status &= ~DRVIFLX_WRITING;
            if (s_write_callback) {
                s_write_callback(DRVIFLX_WRITE_COMPLETE, ret, 0);
                s_write_callback = NULL;
            }
            s_write_start = false;
        }

        if (s_protect_start) {
            OBInit.OptionType = OPTIONBYTE_WRP;
            if (s_protect_enable) {
                OBInit.WRPState = OB_WRPSTATE_ENABLE;
            } else {
                OBInit.WRPState = OB_WRPSTATE_DISABLE;
            }

            OBInit.WRPSector = 0;

#if defined(DUAL_BANK)
            for (i = s_protect_first_sector/2; i <= s_protect_last_sector/2; i++) {
                OBInit.WRPSector |= 1 << (i+16);
            }
#else
            for (i = s_protect_first_sector; i <= s_protect_last_sector; i++) {
                OBInit.WRPSector |= 1 << (i + 16);
            }
#endif

            HAL_FLASHEx_OBProgram(&OBInit);

            if (FLASH_WaitForLastOperation((uint32_t) DRVIFLX_DEFAULT_TIMEOUT) == HAL_OK) {
                ret = DRVIFLX_RES_OK;
            } else {
                ret = DRVIFLX_RES_NG;
            }

            // Callback for DRVIFLX_PROTECT_COMPLETE
            s_status &= ~DRVIFLX_PROTECTING;
            if (s_protect_callback) {
                s_protect_callback(DRVIFLX_PROTECT_COMPLETE, ret, 0);
                s_protect_callback = NULL;
            }
            s_protect_start = false;
        }

        if (s_read_start) {
            // Read data
            memcpy(s_read_dest_addr, (uint8_t*) s_read_src_addr, s_read_length);

            // Callback for DRVIFLX_READ_COMPLETE
            s_status &= ~DRVIFLX_READING;
            if (s_read_callback) {
                s_read_callback(DRVIFLX_READ_COMPLETE, DRVIFLX_RES_OK, 0);
                s_read_callback = NULL;
            }
            s_read_start = false;
        }

        dly_tsk(1);
    }
}

/*""FUNC COMMENT""********************************************************
 * ID          : Callback for End of Operation
 * Outline     :
 *-------------------------------------------------------------------------
 * Include     : n/a
 *-------------------------------------------------------------------------
 * Declaration : void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue)
 *-------------------------------------------------------------------------
 * Description : Callback for End of Operation (erase or program)
 *-------------------------------------------------------------------------
 * Argument    : uint32_t ReturnValue
 *                 - Sectors Erase: Sector which has been erased
 *                                  (0xFFFFFFFF - all the selected sectors have been erased)
 *                 - Program      : Address which was selected for data program
 *                 - Mass Erase   : No return value expected
 *-------------------------------------------------------------------------
 * Return Value: none
 *-------------------------------------------------------------------------
 * Notice      :
 *""FUNC COMMENT END""*****************************************************/
RAMFUNC void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue)
{
    if ((s_status & DRVIFLX_ERASING) == DRVIFLX_ERASING) {
        if (ReturnValue == 0xffffffff) {
            s_erase_status = DRVIFLX_RES_OK;
            s_erase_complete = true;
        }
    }
}

/*""FUNC COMMENT""********************************************************
 * ID          : Callback for Operation Error
 * Outline     :
 *-------------------------------------------------------------------------
 * Include     : n/a
 *-------------------------------------------------------------------------
 * Declaration : void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue)
 *-------------------------------------------------------------------------
 * Description : Callback for Operation Error (erase or program)
 *-------------------------------------------------------------------------
 * Argument    : uint32_t ReturnValue
 *                 - Sectors Erase: Sector which has been erased
 *                                  (0xFFFFFFFF - all the selected sectors have been erased)
 *                 - Program      : Address which was selected for data program
 *                 - Mass Erase   : No return value expected
 *-------------------------------------------------------------------------
 * Return Value: none
 *-------------------------------------------------------------------------
 * Notice      :
 *""FUNC COMMENT END""*****************************************************/
RAMFUNC void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue)
{
    if ((s_status & DRVIFLX_ERASING) == DRVIFLX_ERASING) {
        s_erase_status = DRVIFLX_RES_NG;
        s_erase_complete = true;
    }
}
