/*
 * VA-X I2C共通処理
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/20 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>
#include <sil.h>

/*
 * 定数定義
 */


/*
 * 型定義
 */

/* I2C設定 */
typedef struct {
    uint32_t dummy;
} DRVCMN_I2C_SETTING_T;


/*
 * 公開関数
 */

void drvcmn_i2c_initialize(int i2c, const DRVCMN_I2C_SETTING_T* setting);

int32_t drvcmn_i2c_wait_until_busy(int i2c);

void drvcmn_i2c_set_start(int i2c, uint16_t slave_addr, size_t nbytes, bool_t read);

int32_t drvcmn_i2c_wait_stop(int i2c, uint16_t slave_addr);

int32_t drvcmn_i2c_write(int i2c, uint8_t* data, size_t nbytes);

int32_t drvcmn_i2c_read(uint8_t* odata, int i2c, size_t nbytes);


/*
 * 内部関数
 */

