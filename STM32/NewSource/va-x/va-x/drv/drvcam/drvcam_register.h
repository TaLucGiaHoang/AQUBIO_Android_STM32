/*
 * VA-X カメラドライバ レジスタ設定値
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/26 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once
#include <t_stddef.h>

#include "drvcam.h"

extern const DRVCAM_REGVAL_T DRVCAM_REGVAL_NCM13J_DEFAULT[];
extern const size_t DRVCAM_NUM_REGVAL_NCM13J_DEFAULT;

extern const uint8_t DRVCAM_DEBUG_DUMP_REGS[][2];
extern const size_t DRVCAM_NUM_DEBUG_DUMP_REGS;
