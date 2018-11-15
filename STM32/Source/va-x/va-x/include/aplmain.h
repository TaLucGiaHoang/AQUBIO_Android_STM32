/*
 * VA-X メインタスク
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/01/19 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */
#pragma once

#include <t_stddef.h>

/*
 * 
 */
__attribute__ ((section(".startup"))) void aplmain_task(intptr_t exinf);

