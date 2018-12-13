/************************************************************************/
/* Internal Flash Driver Header                                         */
/* File name  : drvflx.h                                                */
/* DATE       : Jan 03, 2018                                            */
/* CPU TYPE   : STM32F765VI                                             */
/*                                                                      */
/*                                                                      */
/* Copyright (c) 2017 SH Consulting K.K.                                */
/* and SH Consulting VietNam. All Rights Reserved.                      */
/************************************************************************/

#ifndef VA_X_INCLUDE_DRVIFLX_H_
#define VA_X_INCLUDE_DRVIFLX_H_

#include <t_stddef.h>

#if defined(TOPPERS_STM32F7_BNVA)
#include "stm32f765xx.h"
#elif defined(TOPPERS_STM32F7_DISCOVERY)
#include "stm32f746xx.h"
#endif

/* RESPONSE */
#define DRVIFLX_RES_OK 0x00
#define DRVIFLX_RES_NG 0xFF

#define DRVIFLX_DEFAULT_TIMEOUT 5000 // 5s

#define DRVIFLX_ERASING 0x01
#define DRVIFLX_PROTECTING 0x02
#define DRVIFLX_WRITING 0x04
#define DRVIFLX_READING 0x08

typedef enum {
	DRVIFLX_READ_COMPLETE,
	DRVIFLX_WRITE_COMPLETE,
	DRVIFLX_ERASE_COMPLETE,
	DRVIFLX_PROTECT_COMPLETE,
} DRVIFLX_EVENTS;

typedef void (*DRVIFLX_CALLBACK_T)(int event, intptr_t opt1, intptr_t opt2);

int drviflx_initialize(intptr_t opt);
int drviflx_readi(uint8_t* dest, intptr_t src_addr, size_t length);
int drviflx_read(uint8_t* dest, intptr_t src_addr, size_t length, DRVIFLX_CALLBACK_T callback);
int drviflx_write(intptr_t src_addr, uint8_t* dest, size_t length, DRVIFLX_CALLBACK_T callback);
int drviflx_writei(intptr_t src_addr, uint8_t* dest, size_t length);
int drviflx_erase(intptr_t addr, size_t length);
int drviflx_protect(intptr_t addr, size_t length, bool_t protect, DRVIFLX_CALLBACK_T callback);
int drviflx_flash_addr(intptr_t* flash_addr, intptr_t* mapped_addr, size_t* block, size_t* block_size);

void drviflx_task(intptr_t exinf);

#endif /* VA_X_INCLUDE_DRVFLX_H_ */
