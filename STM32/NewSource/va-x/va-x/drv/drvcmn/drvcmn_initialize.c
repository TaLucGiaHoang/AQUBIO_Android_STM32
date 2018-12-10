/*
 * VA-X ペリフェラル初期化処理
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/19 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "drvcmn_initialize.h"

#include <t_syslog.h>
#include <sil.h>
#include <stm32f7xx.h>

#include "drvcmn.h"
#include "drvcmn_gpio.h"

#include "drvpwr.h"
#include "drvble.h"
#include "drvcam.h"
#include "drvirled.h"
#include "drvled.h"
#include "drvbuz.h"
#include "drvpbt.h"
#include "drvflx.h"
#include "drvicc.h"
#include "drvwifi.h"

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



/*
 * 定数定義
 */

/*
 * 内部変数
 */



/*
 * 内部関数のプロトタイプ
 */


/*
 * 公開関数
 */

/*
 *
 */
void drvcmn_initialize_peripherals()
{
    // 電源系初期化
    drvpwr_initialize_peripherals();

    // カメラ
    drvcam_initialize_peripherals();

    // IR-LED
    drvirled_initialize_peripherals();

    // BLE
    drvble_initialize_peripherals();

    // LED
    drvled_initialize_peripherals();

    // ブザー
    drvbuz_initialize_peripherals();

    // プッシュボタン
    drvpbt_initialize_peripherals();

    // フラッシュドライバ
    drvflx_initialize_peripherals();

    // タッチセンサドライバ
    drvts_initialize_peripherals();

    // 電気錠ドライバ
//    drvlock_initialize_peripherals();

    // 電気錠Linkeyドライバ
    drvllock_initialize_peripherals();
    
    // ICカードドライバ
    drvicc_initialize_peripherals();

    // Wifiドライバ
    drvwifi_peripheral_initialize();
}


/*
 * 内部関数関数
 */


