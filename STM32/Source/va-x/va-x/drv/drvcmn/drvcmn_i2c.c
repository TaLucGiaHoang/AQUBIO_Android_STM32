/*
 * VA-X I2C共通処理
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/20 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "drvcmn_i2c.h"

#include <t_syslog.h>
#include <sil.h>
#include <stm32f7xx.h>

#include "drvcmn.h"

/*
 * マクロ定義
 */

// レジスタダンプ
#if 0
  #define DUMP_REG32(name, addr) syslog(LOG_NOTICE, name " (0x%08p): => 0x%08x", addr, drvcmn_getreg32(addr, 0, UINT32_MAX));
#else
  #define DUMP_REG32(name, addr)
#endif


/*
 * 型定義
 */
typedef struct {
    uint32_t rccen_bit;
    uint32_t rccrst_bit;
    intptr_t reg_base;
} I2C_REG_T;

/*
 * 定数定義
 */

// I2C
static const I2C_REG_T I2C_REGS[] = {
    { 0,					0,						0,				},	// [0]なし
    { RCC_APB1ENR_I2C1EN,	RCC_APB1RSTR_I2C1RST,	TADR_I2C1_BASE,	},	// [1]I2C1
    { RCC_APB1ENR_I2C2EN,	RCC_APB1RSTR_I2C2RST,	TADR_I2C2_BASE,	},	// [2]I2C2
    { RCC_APB1ENR_I2C3EN,	RCC_APB1RSTR_I2C3RST,	TADR_I2C3_BASE,	},	// [3]I2C3
    { RCC_APB1ENR_I2C4EN,	RCC_APB1RSTR_I2C4RST,	TADR_I2C4_BASE,	},	// [4]I2C4
};

static const uint32_t I2C_CR2_BITS = (I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP);

/*
 * 内部変数
 */


/*
 * 内部関数のプロトタイプ
 */

// ステータスレジスタポーリング
static const int STATUS_POLLING_LIMIT = 2000000;

enum {
    LOGIC_EQ = 0,
    LOGIC_NEQ,
};

static inline int32_t wait_status_reg(intptr_t reg_addr, int shift, uint32_t bits, int comp_val, int logic)
{
    int error = -1;

    volatile uint32_t regval;
    for (volatile int i = 0; i < STATUS_POLLING_LIMIT; i++) {
        regval = drvcmn_getreg32(reg_addr, shift, bits);
        if (logic == LOGIC_EQ) {
            if (regval != comp_val) {
                error = 0;
                break;
            }
        } else if (logic == LOGIC_NEQ) {
            if (regval == comp_val) {
                error = 0;
                break;
            }
        } else {
            assert(false);
            break;
        }
    }

    if (error) {
        syslog(LOG_NOTICE, "regval: 0x%08x, comp_val: 0x%08x, logic: %d", regval, comp_val, logic);
    }

    return error;
}


/*********************************************************************
 * 公開関数
 ********************************************************************/

/*
 * I2C初期化
 */
void drvcmn_i2c_initialize(int i2c, const DRVCMN_I2C_SETTING_T* setting)
{
    assert((i2c >= 1) && (i2c <= 4));
    assert(setting);

    // RCC
    drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_APB1ENR, 0, I2C_REGS[i2c].rccen_bit, I2C_REGS[i2c].rccen_bit);
    drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_APB1RSTR, 0, I2C_REGS[i2c].rccrst_bit, I2C_REGS[i2c].rccrst_bit);	/* reset */
    drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_APB1RSTR, 0, I2C_REGS[i2c].rccrst_bit, 0);	/* release reset */

    // ベースアドレス
    intptr_t i2c_base = I2C_REGS[i2c].reg_base;

    // I2C_CR1_PE => 0
    drvcmn_setreg32(i2c_base + TOFF_I2C_CR1, 0, I2C_CR1_PE, 0);
    DUMP_REG32("I2C_CR1", i2c_base + TOFF_I2C_CR1);

    // I2C_TIMINGR
#if 1
    // 400kHz
    drvcmn_setreg32(i2c_base + TOFF_I2C_TIMINGR, 0,
                    (I2C_TIMINGR_SCLL | I2C_TIMINGR_SCLH | I2C_TIMINGR_SDADEL | I2C_TIMINGR_SCLDEL | I2C_TIMINGR_PRESC),
                    (0x0 << 28) |	// PRESC
                    (0xa0 << 20) |	// SCLDEL
                    (0x0 << 16) |	// SDADEL
                    (0x1b << 8) |	// SCLH
                    (0x5b << 0));	// SCLL
#else
    // 100kHz
    drvcmn_setreg32(i2c_base + TOFF_I2C_TIMINGR, 0,
                    (I2C_TIMINGR_SCLL | I2C_TIMINGR_SCLH | I2C_TIMINGR_SDADEL | I2C_TIMINGR_SCLDEL | I2C_TIMINGR_PRESC),
                    (0x6 << 28) |	// PRESC
                    (0x2 << 20) |	// SCLDEL
                    (0x0 << 16) |	// SDADEL
                    (0x1e << 8) |	// SCLH
                    (0x2b << 0));	// SCLL
#endif
    DUMP_REG32("I2C_TIMINGR", i2c_base + TOFF_I2C_TIMINGR);

    // I2C_CR1_PE => 0
    drvcmn_setreg32(i2c_base + TOFF_I2C_CR1, 0, I2C_CR1_PE, I2C_CR1_PE);
    DUMP_REG32("I2C_CR1", i2c_base + TOFF_I2C_CR1);
}

int32_t drvcmn_i2c_wait_until_busy(int i2c)
{
    assert((i2c >= 1) && (i2c <= 4));

    // ベースアドレス
    intptr_t i2c_base = I2C_REGS[i2c].reg_base;

    // BUSY がクリアされるのを待つ
    int error = wait_status_reg(i2c_base + TOFF_I2C_ISR, 0, I2C_ISR_BUSY, 0, LOGIC_NEQ);
    assert(!error);

    return 0;
}

void drvcmn_i2c_set_start(int i2c, uint16_t slave_addr, size_t nbytes, bool_t read)
{
    assert((i2c >= 1) && (i2c <= 4));
    assert(!(slave_addr & ~0x7F));	// 7bit
    assert(!(nbytes & ~0xFF));

    /* ARLO, NACK, STOP をクリア */
    drvcmn_setreg32(I2C_REGS[i2c].reg_base + TOFF_I2C_ICR, 0, (I2C_ICR_ARLOCF | I2C_ICR_NACKCF | I2C_ICR_STOPCF), ~0);

    // I2C_CR2
    drvcmn_setreg32(I2C_REGS[i2c].reg_base + TOFF_I2C_CR2, 0,
                    I2C_CR2_BITS,
                    (((nbytes << 16) & I2C_CR2_NBYTES)		|	// NBYTES
                     I2C_CR2_START							|	// START
                     (read ? I2C_CR2_RD_WRN : 0)			|	// RD_WRN
                     ((slave_addr << 1) & 0xFE)));				// SADD[7:1] (7bit)
    DUMP_REG32("I2C_CR2", I2C_REGS[i2c].reg_base + TOFF_I2C_CR2);
}

int32_t drvcmn_i2c_wait_stop(int i2c, uint16_t slave_addr)
{
    assert((i2c >= 1) && (i2c <= 4));
    assert(!(slave_addr & ~0x3FF));

    // ベースアドレス
    intptr_t i2c_base = I2C_REGS[i2c].reg_base;

    // I2C_CR2
    drvcmn_setreg32(i2c_base + TOFF_I2C_CR2, 0,
                    I2C_CR2_BITS,
                    (I2C_CR2_AUTOEND						|	// AUTOEND
                     ((slave_addr << 0) & I2C_CR2_SADD)));		// SADD
    DUMP_REG32("I2C_CR2", i2c_base + TOFF_I2C_CR2);

    /* STOPF ビットがセットされるまで待つ */
    int error = wait_status_reg(i2c_base + TOFF_I2C_ISR, 0, I2C_ISR_STOPF, 0, LOGIC_EQ);
    assert(!error);

    /* STOPF ビットをクリア */
    drvcmn_setreg32(i2c_base + TOFF_I2C_ISR, 0, I2C_ISR_STOPF, 0);

    return 0;
}

int32_t drvcmn_i2c_write(int i2c, uint8_t* data, size_t nbytes)
{
    assert((i2c >= 1) && (i2c <= 4));
    assert(data);
    assert(!(nbytes & ~0xFF));

    // ベースアドレス
    intptr_t i2c_base = I2C_REGS[i2c].reg_base;

    for (int i = 0; i < nbytes; i++) {
        // TXIS がセットされるのを待つ
        int error = wait_status_reg(i2c_base + TOFF_I2C_ISR, 0, (I2C_ISR_TXIS | I2C_ISR_NACKF | I2C_ISR_STOPF), 0, LOGIC_EQ);
        assert(!error);

        if (drvcmn_getreg32(i2c_base + TOFF_I2C_ISR, 0, (I2C_ISR_NACKF | I2C_ISR_STOPF)) != 0) {
            //DUMP_REG32("I2C_ISR", I2C_REGS[i2c].reg_base + TOFF_I2C_ISR);
            goto error;
        }

        // 送信データ書き込み
        drvcmn_setreg32(i2c_base + TOFF_I2C_TXDR, 0, 0xFF, data[i]);
    }
    
    // TC がセットされるのを待つ
    int error = wait_status_reg(i2c_base + TOFF_I2C_ISR, 0, I2C_ISR_TC, 0, LOGIC_EQ);
    assert(!error);

    return 0;

error:
    return -1;
}

int32_t drvcmn_i2c_read(uint8_t* odata, int i2c, size_t nbytes)
{
    assert(odata);
    assert((i2c >= 1) && (i2c <= 4));
    assert(!(nbytes & ~0xFF));

    // ベースアドレス
    intptr_t i2c_base = I2C_REGS[i2c].reg_base;

    for (int i = 0; i < nbytes; i++) {
        // RXNE がセットされるのを待つ
        int error = wait_status_reg(i2c_base + TOFF_I2C_ISR, 0, I2C_ISR_RXNE, 0, LOGIC_EQ);
        assert(!error);

        // 受信データ読み出し
        odata[i] = drvcmn_getreg32(i2c_base + TOFF_I2C_RXDR, 0, 0xFF);
    }

    return 0;
}


