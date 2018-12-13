/*
 * stm32f746xx.h
 *
 *  Created on: Jan 4, 2018
 *      Author: anhtran
 */

#ifndef VA_X_INCLUDE_STM32F746XX_H_
#define VA_X_INCLUDE_STM32F746XX_H_

#define FLASH_SECTOR_TOTAL  8 // FLASH Total Sectors Number

#define FLASH_CR_SNB        0x00000078U



#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x00000000) /* Base address of Sector 0, 32 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x00008000) /* Base address of Sector 1, 32 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x00010000) /* Base address of Sector 2, 32 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x00018000) /* Base address of Sector 3, 32 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x00020000) /* Base address of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x00040000) /* Base address of Sector 5, 256 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x00080000) /* Base address of Sector 6, 256 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x000C0000) /* Base address of Sector 7, 256 Kbytes */
#define ADDR_FLASH_SECTOR_END   ((uint32_t)0x000FFFFF) /*!< FLASH end address */

#endif /* VA_X_INCLUDE_STM32F746XX_H_ */
