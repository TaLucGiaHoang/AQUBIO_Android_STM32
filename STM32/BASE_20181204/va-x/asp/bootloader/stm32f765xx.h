/*
 * stm32f765xx.h
 *
 *  Created on: Jan 4, 2018
 *      Author: anhtran
 */

#ifndef VA_X_INCLUDE_STM32F765XX_H_
#define VA_X_INCLUDE_STM32F765XX_H_

#define FLASH_SECTOR_TOTAL  24 // FLASH Total Sectors Number
//#define DUAL_BANK

// Additional definitions for STM32F765
#define FLASH_OPTCR_nWRP_8            0x01000000U
#define FLASH_OPTCR_nWRP_9            0x02000000U
#define FLASH_OPTCR_nWRP_10           0x04000000U
#define FLASH_OPTCR_nWRP_11           0x08000000U
#define FLASH_OPTCR_nDBOOT            0x10000000U
#define FLASH_OPTCR_nDBANK            0x20000000U

#define FLASH_CR_MER1                 FLASH_CR_MER
#define FLASH_CR_MER2                 0x00008000U

#define FLASH_CR_SNB                  0x000000F8U

#if defined(DUAL_BANK)
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x00000000) /* Base address of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x00004000) /* Base address of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x00008000) /* Base address of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0000C000) /* Base address of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x00010000) /* Base address of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x00020000) /* Base address of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x00040000) /* Base address of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x00060000) /* Base address of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x00080000) /* Base address of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x000A0000) /* Base address of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x000C0000) /* Base address of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x000E0000) /* Base address of Sector 11, 128 Kbytes */
#define ADDR_FLASH_SECTOR_12    ((uint32_t)0x00100000) /* Base address of Sector 12, 16 Kbytes */
#define ADDR_FLASH_SECTOR_13    ((uint32_t)0x00104000) /* Base address of Sector 13, 16 Kbytes */
#define ADDR_FLASH_SECTOR_14    ((uint32_t)0x00108000) /* Base address of Sector 14, 16 Kbytes */
#define ADDR_FLASH_SECTOR_15    ((uint32_t)0x0010C000) /* Base address of Sector 15, 16 Kbytes */
#define ADDR_FLASH_SECTOR_16    ((uint32_t)0x00110000) /* Base address of Sector 16, 64 Kbytes */
#define ADDR_FLASH_SECTOR_17    ((uint32_t)0x00120000) /* Base address of Sector 17, 128 Kbytes */
#define ADDR_FLASH_SECTOR_18    ((uint32_t)0x00140000) /* Base address of Sector 18, 128 Kbytes */
#define ADDR_FLASH_SECTOR_19    ((uint32_t)0x00160000) /* Base address of Sector 19, 128 Kbytes */
#define ADDR_FLASH_SECTOR_20    ((uint32_t)0x00180000) /* Base address of Sector 20, 128 Kbytes */
#define ADDR_FLASH_SECTOR_21    ((uint32_t)0x001A0000) /* Base address of Sector 21, 128 Kbytes */
#define ADDR_FLASH_SECTOR_22    ((uint32_t)0x001C0000) /* Base address of Sector 22, 128 Kbytes */
#define ADDR_FLASH_SECTOR_23    ((uint32_t)0x001E0000) /* Base address of Sector 23, 128 Kbytes */
#else
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x00000000) /* Base address of Sector 0, 32 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x00008000) /* Base address of Sector 1, 32 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x00010000) /* Base address of Sector 2, 32 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x00018000) /* Base address of Sector 3, 32 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x00020000) /* Base address of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x00040000) /* Base address of Sector 5, 256 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x00080000) /* Base address of Sector 6, 256 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x000C0000) /* Base address of Sector 7, 256 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x00100000) /* Base address of Sector 8, 256 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x00140000) /* Base address of Sector 9, 256 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x00180000) /* Base address of Sector 10, 256 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x001C0000) /* Base address of Sector 11, 256 Kbytes */
#endif /* DUAL_BANK */
#define ADDR_FLASH_SECTOR_END   ((uint32_t)0x001FFFFFU) /*!< FLASH end address */

#endif /* VA_X_INCLUDE_STM32F765XX_H_ */
