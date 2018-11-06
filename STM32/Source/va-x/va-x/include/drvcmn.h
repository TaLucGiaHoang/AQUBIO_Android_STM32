/*
 * VA-X
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/13 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
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


/*
 * 関数プロトタイプ
 */


/*
 * インライン関数
 */

/**
 * レジスタ read-write-modify (32bit)
 * @param addr 対象アドレス
 * @param shift ビットシフト
 * @param mask 操作対象ビット
 * @param value 設定する値
 */
static inline void drvcmn_setreg32(intptr_t addr, int shift, uint32_t mask, uint32_t value)
{
    sil_wrw_mem((uint32_t*)addr, (sil_rew_mem((uint32_t*)addr) & ~(mask << shift)) | ((value << shift) & (mask << shift)));
}

/**
 * レジスタ read (32bit)
 * @param addr 対象アドレス
 * @param shift ビットシフト
 * @param mask 対象ビット
 */
static inline uint32_t drvcmn_getreg32(intptr_t addr, int shift, uint32_t mask)
{
    return (sil_rew_mem((uint32_t*)addr) & (mask << shift)) >> shift;
}

/**
 * レジスタ read-write-modify (16bit)
 * @param addr 対象アドレス
 * @param shift ビットシフト
 * @param mask 操作対象ビット
 * @param value 設定する値
 */
static inline void drvcmn_setreg16(intptr_t addr, int shift, uint16_t mask, uint16_t value)
{
    sil_wrh_mem((uint16_t*)addr, (sil_reh_mem((uint16_t*)addr) & ~(mask << shift)) | ((value << shift) & (mask << shift)));
}

/**
 * レジスタ read (16bit)
 * @param addr 対象アドレス
 * @param shift ビットシフト
 * @param mask 対象ビット
 */
static inline uint16_t drvcmn_getreg16(intptr_t addr, int shift, uint16_t mask)
{
    return (sil_reh_mem((uint16_t*)addr) & (mask << shift)) >> shift;
}

/**
 * レジスタ read-write-modify (8bit)
 * @param addr 対象アドレス
 * @param shift ビットシフト
 * @param mask 操作対象ビット
 * @param value 設定する値
 */
static inline void drvcmn_setreg8(intptr_t addr, int shift, uint8_t mask, uint8_t value)
{
    sil_wrb_mem((uint8_t*)addr, (sil_reb_mem((uint8_t*)addr) & ~(mask << shift)) | ((value << shift) & (mask << shift)));
}

/**
 * レジスタ read (8bit)
 * @param addr 対象アドレス
 * @param shift ビットシフト
 * @param mask 対象ビット
 */
static inline uint8_t drvcmn_getreg8(intptr_t addr, int shift, uint8_t mask)
{
    return (sil_reb_mem((uint8_t*)addr) & (mask << shift)) >> shift;
}
