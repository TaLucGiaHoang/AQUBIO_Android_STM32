/*
 * VA-X 外部フラッシュドライバ
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/02/07 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "drvqflx.h"

#include <string.h>

#include <kernel.h>
#include <sil.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "stm32f7xx.h"
#include "kernel_cfg.h"

#include "drvcmn.h"
#include "drvcmn_gpio.h"
#include "drvcmn_dma.h"

/*
 * マクロ定義
 */

// デバッグログ
#if 1
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[DRVQFLX]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[DRVQFLX]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[DRVQFLX]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[DRVQFLX]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

// サービスコールのエラーのログ出力
static inline void svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}
#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))

// STM32 QUADSPI レジスタ設定値
#define FMODE_INDIRECT_WRITE 		0x0
#define FMODE_INDIRECT_READ 		0x1
#define FMODE_AUTOMATIC_POLLING 	0x2
#define FMODE_MEMORY_MAPPED        0x3

#define DMODE_NO_DATA 				0x0
#define DMODE_SINGLE_LINE 			0x1
#define DMODE_TWO_LINES 			0x2
#define DMODE_FOUR_LINES 			0x3

#define ABSIZE_8BIT 				0x0
#define ABSIZE_16BIT 				0x1
#define ABSIZE_24BIT 				0x2
#define ABSIZE_32BIT 				0x3

#define ABMODE_NO_ALTERNATE_BYTES 	0x0
#define ABMODE_SINGLE_LINE 			0x1
#define ABMODE_TWO_LINES 			0x2
#define ABMODE_FOUR_LINES 			0x3

#define ADSIZE_8BIT 				0x0
#define ADSIZE_16BIT 				0x1
#define ADSIZE_24BIT 				0x2
#define ADSIZE_32BIT 				0x3

#define ADMODE_NO_ADDRESS 			0x0
#define ADMODE_SINGLE_LINE 			0x1
#define ADMODE_TWO_LINES 			0x2
#define ADMODE_FOUR_LINES 			0x3

#define IMODE_NO_INSTRUCTION 		0x0
#define IMODE_SINGLE_LINE 			0x1
#define IMODE_TWO_LINES 			0x2
#define IMODE_FOUR_LINES 			0x3

// フラッシュサイズ
#define FLASH_SIZE	(2 * 1024 * 1024)

// フラッシュページサイズ(書込みの単位)
#define FLASH_PAGE_SIZE	256

// フラッシュセクタサイズ(消去の単位)
#define FLASH_SECTOR_SIZE	(1024 * 4)	// 4kB

/*
 * 型定義
 */

// メッセージ送信用メモリブロック
typedef struct {
    uint32_t msg;
    intptr_t src;
    intptr_t dest;
    size_t size;
    DRVFLX_CALLBACK_T callback;
} DRVFLX_MPFBLK_T;

// DRVBLE_MPFBLK_SIZE は sizeof(DRVBLE_MPFBKL) に一致させること
static_assert(sizeof(DRVFLX_MPFBLK_T) == DRVFLX_MPFBLK_SIZE, "MPF size");

// STM32 QUADSPI関連
typedef struct {
    uint8_t fmode;
    uint8_t dmode;
    uint8_t dcyc;
    uint8_t absize;
    uint8_t abmode;
    uint8_t adsize;
    uint8_t admode;
    uint8_t imode;
    uint8_t instruction;
} QSPI_CCR_T;

typedef struct {
    uint8_t fmode;
    uint8_t dmode;
    uint8_t dcyc;
    uint8_t absize;
    uint8_t abmode;
    uint8_t adsize;
    uint8_t admode;
    uint8_t imode;
    uint8_t instruction;
} QSPI_TRANSFER_CONFIG_T;

typedef struct {
    uint32_t ar;
    uint32_t abr;
    void* mcuaddr;
    size_t length;
} QSPI_TRANSFER_DATA_T;


/*
 * 内部関数プロトタイプ
 */
// QUADSPI初期化
static void qspi_initialize();

// メッセージ送信
static void mpf_send(int msg, intptr_t src, intptr_t dest, size_t size, DRVFLX_CALLBACK_T callback);

// フラッシュ設定
static void qspi_flash_configure();

// QSPI転送
static int qspi_transfer(const QSPI_TRANSFER_CONFIG_T* config, const QSPI_TRANSFER_DATA_T* data);

// DMA ISRコールバック
static void dma_isr_callback();

// フラッシュのステータス待ち
static void qspi_wait_flash_status(uint32_t mask, uint32_t match);

// SPIフラッシュ読み込み
static int qspi_flash_read(void* dest, intptr_t flash_addr, size_t length);

// SPIフラッシュ書込み
static int qspi_flash_write(const void* src, intptr_t flash_addr, size_t length);

// SPIフラッシュ消去
static int qspi_flash_erase(intptr_t flash_addr, size_t length);

/*
 * 定数定義
 */

// GPIO設定(QSPI, CE以外)
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_QSPI = {
    .mode = DRVCMN_GPIO_MODE_AF,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_VERYHIGH,
    .pupd = DRVCMN_GPIO_PUPD_NONE,
    .afno = 9,
};

// GPIO設定(QSPI, CE)
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_QSPI_CE = {
    .mode = DRVCMN_GPIO_MODE_AF,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_VERYHIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLUP,
    .afno = 10,
};

// GPIOピン設定(QSPI)
static const struct {
    DRVCMN_GPIO_PIN_T pin;
    const DRVCMN_GPIO_SETTING_T* setting;
} GPIO_PINS_QSPI[] = {
    { { DRVCMN_GPIO_PORT_A,	1	},	&GPIO_SETTING_QSPI		},	// QUADSPI_BK1_IO3(AF9)
    { { DRVCMN_GPIO_PORT_B,	2	},	&GPIO_SETTING_QSPI		},	// QUADSPI_CLK(AF9)
    { { DRVCMN_GPIO_PORT_B,	6	},	&GPIO_SETTING_QSPI_CE	},	// QUADSPI_BK1_NCS(AF10)
    { { DRVCMN_GPIO_PORT_C,	10	},	&GPIO_SETTING_QSPI		},	// QUADSPI_BK1_IO1(AF9)
    { { DRVCMN_GPIO_PORT_D,	11	},	&GPIO_SETTING_QSPI		},	// QUADSPI_BK1_IO0(AF9)
    { { DRVCMN_GPIO_PORT_E,	2	},	&GPIO_SETTING_QSPI		},	// QUADSPI_BK1_IO2(AF9)
};

// DMAストリーム
static const DRVCMN_DMA_STR_T DMA_STREAM = {2,	2};	// DMA2 Stream2

// DMAストリーム設定
static const DRVCMN_DMA_SETTING_T DMA_SETTING = {
    .chsel = 11,							// Channel selection
    .dbm = false,							// Double buffer mode
    .pl = DRVCMN_DMA_PL_VERY_HIGH,			// Priority level
    .data_size = DRVCMN_DMA_DATASIZE_8BIT,	// Memory/Peripheral data size
    .minc = true,							// Memory increment mode
    .pinc = false,							// Peripheral increment mode
    .dir = DRVCMN_DMA_DIR_P2M,				// Data transfer direction
};

// ステータスレジスタマスク: Write operation status(BUSY)
static const uint32_t FLASH_STATUS_MASK_BUSY = (0x1 << 7) | (0x1 << 0);	// [7,0]BUSY

// ステータスレジスタマスク: Write-Enable Latch status(WEL)
static const uint32_t FLASH_STATUS_MASK_WEL = (0x1 << 1);	// [1]WEL


// フラッシュコマンド
// Reset Enable
static const QSPI_TRANSFER_CONFIG_T FLASH_CMD_RSTEN = {
    .fmode = FMODE_INDIRECT_WRITE,
    .dmode = DMODE_NO_DATA,
    .dcyc = 0,
    .absize = ABSIZE_8BIT,
    .abmode = ABMODE_NO_ALTERNATE_BYTES,
    .adsize = ADSIZE_8BIT,
    .admode = ADMODE_NO_ADDRESS,
    .imode = IMODE_SINGLE_LINE,
    .instruction = 0x66,
};

// Reset
static const QSPI_TRANSFER_CONFIG_T FLASH_CMD_RST = {
    .fmode = FMODE_INDIRECT_WRITE,
    .dmode = DMODE_NO_DATA,
    .dcyc = 0,
    .absize = ABSIZE_8BIT,
    .abmode = ABMODE_NO_ALTERNATE_BYTES,
    .adsize = ADSIZE_8BIT,
    .admode = ADMODE_NO_ADDRESS,
    .imode = IMODE_SINGLE_LINE,
    .instruction = 0x99,
};

// Write Enable
static const QSPI_TRANSFER_CONFIG_T FLASH_CMD_WREN = {
    .fmode = FMODE_INDIRECT_WRITE,
    .dmode = DMODE_NO_DATA,
    .dcyc = 0,
    .absize = ABSIZE_8BIT,
    .abmode = ABMODE_NO_ALTERNATE_BYTES,
    .adsize = ADSIZE_8BIT,
    .admode = ADMODE_NO_ADDRESS,
    .imode = IMODE_SINGLE_LINE,
    .instruction = 0x06,
};

// Write Status Register
static const QSPI_TRANSFER_CONFIG_T FLASH_CMD_WRSR = {
    .fmode = FMODE_INDIRECT_WRITE,
    .dmode = DMODE_SINGLE_LINE,
    .dcyc = 0,
    .absize = ABSIZE_8BIT,
    .abmode = ABMODE_NO_ALTERNATE_BYTES,
    .adsize = ADSIZE_8BIT,
    .admode = ADMODE_NO_ADDRESS,
    .imode = IMODE_SINGLE_LINE,
    .instruction = 0x01,
};

// Global Block-Protection Unlock
static const QSPI_TRANSFER_CONFIG_T FLASH_CMD_ULBPR = {
    .fmode = FMODE_INDIRECT_WRITE,
    .dmode = DMODE_NO_DATA,
    .dcyc = 0,
    .absize = ABSIZE_8BIT,
    .abmode = ABMODE_NO_ALTERNATE_BYTES,
    .adsize = ADSIZE_24BIT,
    .admode = ADMODE_NO_ADDRESS,
    .imode = IMODE_SINGLE_LINE,
    .instruction = 0x98,
};

// Sector Erase
static const QSPI_TRANSFER_CONFIG_T FLASH_CMD_SE = {
    .fmode = FMODE_INDIRECT_WRITE,
    .dmode = DMODE_NO_DATA,
    .dcyc = 0,
    .absize = ABSIZE_8BIT,
    .abmode = ABMODE_NO_ALTERNATE_BYTES,
    .adsize = ADSIZE_24BIT,
    .admode = ADMODE_SINGLE_LINE,
    .imode = IMODE_SINGLE_LINE,
    .instruction = 0x20,
};

// Chip Erase
static const QSPI_TRANSFER_CONFIG_T FLASH_CMD_CE = {
    .fmode = FMODE_INDIRECT_WRITE,
    .dmode = DMODE_NO_DATA,
    .dcyc = 0,
    .absize = ABSIZE_8BIT,
    .abmode = ABMODE_NO_ALTERNATE_BYTES,
    .adsize = ADSIZE_8BIT,
    .admode = ADMODE_NO_ADDRESS,
    .imode = IMODE_SINGLE_LINE,
    .instruction = 0xc7,
};

// SPI Quad IO Read
static const QSPI_TRANSFER_CONFIG_T FLASH_CMD_SQOR = {
    .fmode = FMODE_INDIRECT_READ,
    .dmode = DMODE_FOUR_LINES,
    .dcyc = 8,
    .absize = ABSIZE_8BIT,
    .abmode = ABMODE_NO_ALTERNATE_BYTES,
    .adsize = ADSIZE_24BIT,
    .admode = ADMODE_SINGLE_LINE,
    .imode = IMODE_SINGLE_LINE,
    .instruction = 0x6b,
};

// SPI Quad Page Program
static const QSPI_TRANSFER_CONFIG_T FLASH_CMD_QPP = {
    .fmode = FMODE_INDIRECT_WRITE,
    .dmode = DMODE_FOUR_LINES,
    .dcyc = 0,
    .absize = ABSIZE_8BIT,
    .abmode = ABMODE_NO_ALTERNATE_BYTES,
    .adsize = ADSIZE_24BIT,
    .admode = ADMODE_FOUR_LINES,
    .imode = IMODE_SINGLE_LINE,
    .instruction = 0x32,
};

// SPI Memmap
static const QSPI_TRANSFER_CONFIG_T FLASH_CMD_MAP = {
    .fmode = FMODE_MEMORY_MAPPED,
    .dmode = DMODE_FOUR_LINES,
    .dcyc = 8,
    .absize = ABSIZE_8BIT,
    .abmode = ABMODE_NO_ALTERNATE_BYTES,
    .adsize = ADSIZE_24BIT,
    .admode = ADMODE_SINGLE_LINE,
    .imode = IMODE_SINGLE_LINE,
    .instruction = 0x6b,
};

// メッセージ番号
enum {
  MSG_NONE = 0,
  MSG_INITIALIZE,
  MSG_READ,
  MSG_WRITE,
  MSG_ERASE,
  MSG_START_MEMMAP,
  MSG_END_MEMMAP,
};

// データキュータイムアウト
static const int32_t DTQTIMEOUT = 1000;

// イベントフラグ
static const FLGPTN FLGPTN_DMA_COMPLETE = 			(0x1 << 0);
static const FLGPTN FLGPTN_QSPI_TRANSFER_COMPLETE = (0x1 << 1);
static const FLGPTN FLGPTN_QSPI_STATUS_MATCH = 		(0x1 << 2);

/*
 * 内部変数
 */
//static uint8_t s_testbuf[1024*4];// 4k

/*********************************************************************
 * 公開関数
 ********************************************************************/

/*
 * ペリフェラル初期化
 */
void drvqflx_initialize_peripherals()
{
    DBGLOG0("drvqflx_initialize_peripherals");

    // GPIOピン設定
    for (int i = 0; i < sizeof(GPIO_PINS_QSPI) / sizeof(GPIO_PINS_QSPI[0]); i++) {
        drvcmn_gpio_pin_initialize(&(GPIO_PINS_QSPI[i].pin), GPIO_PINS_QSPI[i].setting);
    }

    // DMA初期化
    //drvcmn_dma_initialize(&DMA_STREAM, &DMA_SETTING);

    // QUADSPI初期化
    qspi_initialize();

}

/*
 * ドライバ初期化
 */
void drvqflx_initialize()
{
    DBGLOG0("drvqflx_initialize");

    qspi_flash_configure();
}

/*
 * 読み込み
 */
void drvqflx_read(void* dest, intptr_t src, size_t length)
{
    DBGLOG0("drvqflx_read");

    qspi_flash_read((void*)dest, src, length);
}

/*
 * メモリマップ開始
 */
void drvqflx_start_memmap(DRVFLX_CALLBACK_T callback)
{
    DBGLOG0("drvqflx_memmap");

}

/*
 * メモリマップ終了
 */
void drvqflx_end_memmap(DRVFLX_CALLBACK_T callback)
{
    DBGLOG0("drvqflx_end_memmap");

}

/*
 * 書き込み
 */
void drvqflx_write(intptr_t dest, const void* src, size_t length)
{
    DBGLOG0("drvqflx_write");

    qspi_flash_write((void*)src, dest, length);
}

/*
 * 消去
 */
void drvqflx_erase(intptr_t addr, size_t length)
{
    DBGLOG0("drvqflx_erase");

    qspi_flash_erase(addr, length);
}

void drvqflx_qspi_isr(intptr_t exinf)
{
    //DBGLOG0("drvqflx_qspi_isr");

    ER er = E_OK;

    if (drvcmn_getreg32(QSPI_R_BASE + 0x0 /* QUADSPI_CR */, 19, 0x1) &&	// [19]SMIE
        drvcmn_getreg32(QSPI_R_BASE + 0x8 /* QUADSPI_SR */, 3, 0x1)) {	// [3]SMF
        // ステータスマッチ
        // フラグクリア
        drvcmn_setreg32(QSPI_R_BASE + 0xC /* QUADSPI_FCR */, 3, 0x1, 0x1);
        // 割込み無効
        drvcmn_setreg32(QSPI_R_BASE + 0x0 /* QUADSPI_CR */, 19, 0x1, 0);
        // 通知
        er = iset_flg(FLG_DRVFLX, FLGPTN_QSPI_STATUS_MATCH);
        assert(er == E_OK);
    } else if (drvcmn_getreg32(QSPI_R_BASE + 0x0 /* QUADSPI_CR */, 17, 0x1) &&	// [17]TCIE
               drvcmn_getreg32(QSPI_R_BASE + 0x8 /* QUADSPI_SR */, 1, 0x1)) {	// [1]TCF
        // 転送完了
        // フラグクリア
        drvcmn_setreg32(QSPI_R_BASE + 0xC /* QUADSPI_FCR */, 1, 0x1, 0x1);
        // 割込み無効
        drvcmn_setreg32(QSPI_R_BASE + 0x0 /* QUADSPI_CR */, 17, 0x1, 0);
        // 通知
        er = iset_flg(FLG_DRVFLX, FLGPTN_QSPI_TRANSFER_COMPLETE);
        assert(er == E_OK);
    } else if (drvcmn_getreg32(QSPI_R_BASE + 0x0 /* QUADSPI_CR */, 16, 0x1) &&	// [16]TEIE
               drvcmn_getreg32(QSPI_R_BASE + 0x8 /* QUADSPI_SR */, 0, 0x1)) {	// [0]TEF
        // 転送エラー(アドレス不正)
        // フラグクリア
        drvcmn_setreg32(QSPI_R_BASE + 0xC /* QUADSPI_FCR */, 0, 0x1, 0x0);	// [0]CTEF
        // 割込み無効
        drvcmn_setreg32(QSPI_R_BASE + 0x0 /* QUADSPI_CR */, 16, 0x1, 0);	// [16]TEIE
        // 通知
        er = iset_flg(FLG_DRVFLX, FLGPTN_QSPI_TRANSFER_COMPLETE);
        assert(er == E_OK);
    } else {
        DBGLOG1("sr: 0x%08x", drvcmn_getreg32(QSPI_R_BASE + 0x8 /* QUADSPI_SR */, 0, ~(uint32_t)0));
        assert(false);
    }

}

/*********************************************************************
 * 内部関数
 ********************************************************************/

/*
 * QSPI初期化
 */
void qspi_initialize()
{
    // QSPIクロック有効
    drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_AHB3ENR, 0, RCC_AHB3ENR_QSPIEN, ~(uint32_t)0);

    // QUADSPI_CR
    drvcmn_setreg32(QSPI_R_BASE + 0x0 /* QUADSPI_CR */, 24, 0xff, 2);	// PRESCALER => 2
    drvcmn_setreg32(QSPI_R_BASE + 0x0 /* QUADSPI_CR */, 22, 0x1, 1);	// APMS => 1
    drvcmn_setreg32(QSPI_R_BASE + 0x0 /* QUADSPI_CR */, 8, 0x1f, 0);	// FTHRES => 0
    drvcmn_setreg32(QSPI_R_BASE + 0x0 /* QUADSPI_CR */, 4, 0x1, 0);		// SSHIFT => 0

    // QUADSPI_DCR
    drvcmn_setreg32(QSPI_R_BASE + 0x4 /* QUADSPI_DCR */, 16, 0x1f, 20);	// FSIZE => 20 (2 ^ (20 + 1) = 2MB)
    drvcmn_setreg32(QSPI_R_BASE + 0x4 /* QUADSPI_DCR */, 8, 0x7, 1);	// CSHT => 1

    // QUADSPI有効
    drvcmn_setreg32(QSPI_R_BASE + 0x0 /* QUADSPI_CR */, 0, 0x1, 1);		// EN => 1
}

/*
 * メモリプールを使ったメッセージ送信
 */
void mpf_send(int msg, intptr_t src, intptr_t dest, size_t size, DRVFLX_CALLBACK_T callback)
{
    DRVFLX_MPFBLK_T* blk = NULL;
    ER er = 0;
    er = tget_mpf(MPF_DRVFLX, (void**)&blk, DTQTIMEOUT);
    assert(er == E_OK);

    blk->msg = msg;
    blk->src = src;
    blk->dest = dest;
    blk->size = size;
    blk->callback = callback;
    er = tsnd_dtq(DTQ_DRVFLX, (intptr_t)blk, DTQTIMEOUT);
    assert(er == E_OK);
}

/*
 * SPIフラッシュ設定
 */
void qspi_flash_configure()
{
    DBGLOG0("qspi_flash_configure");

     // BUSYクリア待ち
//    qspi_wait_flash_status(FLASH_STATUS_MASK_BUSY, 0);

    // リセットコマンド
//    qspi_transfer(&FLASH_CMD_RSTEN, NULL);
//    qspi_transfer(&FLASH_CMD_RST, NULL);
//    qspi_wait_flash_status(FLASH_STATUS_MASK_BUSY, 0);
//    dly_tsk(1);

    // Configuration Register の [1]IOC ビットをセット
    qspi_transfer(&FLASH_CMD_WREN, NULL);	// Write Enable
//    qspi_wait_flash_status(FLASH_STATUS_MASK_BUSY | FLASH_STATUS_MASK_WEL, FLASH_STATUS_MASK_WEL);
    qspi_transfer(&FLASH_CMD_WRSR,
                  &(QSPI_TRANSFER_DATA_T){.mcuaddr = (uint8_t[]){0x00, 0x02}, .length = 2});
//    qspi_wait_flash_status(FLASH_STATUS_MASK_BUSY, 0);

//    // 書込み保護を全て解除(Global Block-Protection Unlock)
//    qspi_transfer(&FLASH_CMD_WREN, NULL);	// Write Enable
//    qspi_wait_flash_status(FLASH_STATUS_MASK_BUSY | FLASH_STATUS_MASK_WEL, FLASH_STATUS_MASK_WEL);
//    qspi_transfer(&FLASH_CMD_ULBPR, NULL);
//    qspi_wait_flash_status(FLASH_STATUS_MASK_BUSY, 0);

    qspi_config(&FLASH_CMD_MAP);
}

/*
 * SPIフラッシュ読込み
 */
int qspi_flash_read(void* dest, intptr_t flash_addr, size_t length)
{
    assert(dest);
    assert(length > 0);

    // 読込み
    qspi_transfer(&FLASH_CMD_SQOR,
                  &(QSPI_TRANSFER_DATA_T){.ar = flash_addr, .mcuaddr = dest, .length = length});
    qspi_wait_flash_status(FLASH_STATUS_MASK_BUSY, 0);

    return 0;
}

/*
 * SPIフラッシュ書込み
 */
int qspi_flash_write(const void* src, intptr_t flash_addr, size_t length)
{
    assert(src);
    assert(length > 0);
    assert(flash_addr + length <= FLASH_SIZE);

    static uint8_t s_pagebuf[FLASH_PAGE_SIZE];

    const void* cur_mem = src;
    intptr_t cur_flash = flash_addr;
    size_t length_written = 0;

    const void* transfer_src = NULL;
    uint32_t ar = 0;
    size_t transfer_length = 0;

    // 開始アドレスがページの先頭から離れている場合, 0xffを付加してページ先頭から書き込む
    size_t skip = flash_addr % FLASH_PAGE_SIZE;
    if (skip != 0) {
        transfer_src = s_pagebuf;
        ar = cur_flash - skip;
        transfer_length = (length + skip > FLASH_PAGE_SIZE ? FLASH_PAGE_SIZE : length + skip);

        // 一時バッファにデータコピー
        memset(s_pagebuf, 0xff, skip);
        memcpy(s_pagebuf + skip, src, transfer_length - skip);

        // 書込み実行
        qspi_transfer(&FLASH_CMD_WREN, NULL);	// Write Enable
        qspi_wait_flash_status(FLASH_STATUS_MASK_BUSY | FLASH_STATUS_MASK_WEL, FLASH_STATUS_MASK_WEL);
        qspi_transfer(&FLASH_CMD_QPP,
                      &(QSPI_TRANSFER_DATA_T){.ar = ar, .mcuaddr = (void*)transfer_src, .length = transfer_length});
        qspi_wait_flash_status(FLASH_STATUS_MASK_BUSY, 0);

        length_written += transfer_length - skip;
        cur_mem += transfer_length - skip;
        cur_flash += transfer_length - skip;
    }

    while (length_written < length) {
        transfer_src = cur_mem;
        ar = cur_flash;
        transfer_length = (length - length_written > FLASH_PAGE_SIZE ? FLASH_PAGE_SIZE : length - length_written);

        // 書込み実行
        qspi_transfer(&FLASH_CMD_WREN, NULL);	// Write Enable
        qspi_wait_flash_status(FLASH_STATUS_MASK_BUSY | FLASH_STATUS_MASK_WEL, FLASH_STATUS_MASK_WEL);
        qspi_transfer(&FLASH_CMD_QPP,
                      &(QSPI_TRANSFER_DATA_T){.ar = ar, .mcuaddr = (void*)transfer_src, .length = transfer_length});
        qspi_wait_flash_status(FLASH_STATUS_MASK_BUSY, 0);

        length_written += transfer_length;
        cur_mem += transfer_length;
        cur_flash += transfer_length;
    }

    assert(length_written == length);

    return 0;
}

/*
 * SPIフラッシュ消去
 */
int qspi_flash_erase(intptr_t flash_addr, size_t length)
{
    assert(flash_addr % FLASH_SECTOR_SIZE == 0);
    assert(length > 0 && length % FLASH_SECTOR_SIZE == 0);

    intptr_t cur_flash = flash_addr;

    while (cur_flash < flash_addr + length) {
        // 消去実行
        qspi_transfer(&FLASH_CMD_WREN, NULL);	// Write Enable
        qspi_wait_flash_status(FLASH_STATUS_MASK_BUSY | FLASH_STATUS_MASK_WEL, FLASH_STATUS_MASK_WEL);
        qspi_transfer(&FLASH_CMD_SE, &(QSPI_TRANSFER_DATA_T){.ar = cur_flash});
        qspi_wait_flash_status(FLASH_STATUS_MASK_BUSY, 0);

        cur_flash += FLASH_SECTOR_SIZE;
    }

    return 0;
}


int qspi_config(const QSPI_TRANSFER_CONFIG_T* config)
{
    DBGLOG0("qspi_config");
    assert(config);

    ER er = E_OK;
    FLGPTN flgptn = 0;
    int spi_error = 0;

    assert(drvcmn_getreg32(QSPI_R_BASE + 0x8 /* QUADSPI_SR */, 5, 0x1) == 0);

    // ステータスレジスタクリア
    drvcmn_setreg32(QSPI_R_BASE + 0x0C /* QUADSPI_FCR */, 0, (0x1 << 1) | (0x1 << 0), ~(uint32_t) 0);    // CTCF,CTEF

    // イベントフラグクリア
//    er = clr_flg(FLG_DRVFLX, ~FLGPTN_QSPI_TRANSFER_COMPLETE);
//    assert(er == E_OK);

    // 割込み有効
//    drvcmn_setreg32(QSPI_R_BASE + 0x0 /* QUADSPI_CR */, 0, (0x1 << 16) | (0x1 << 17), ~(uint32_t) 0);    // TCIE, TEIE

    // QUADSPI_CCR
    uint32_t ccr_val = 0;
    ccr_val = drvcmn_getreg32(QSPI_R_BASE + 0x14 /* QUADSPI_CCR */, 0, ~(uint32_t) 0);   // QUADSPI_CCR
    drvcmn_setreg32((intptr_t) &ccr_val, 26, 0x3, config->fmode);        // FMODE
    drvcmn_setreg32((intptr_t) &ccr_val, 24, 0x3, config->dmode);        // DMODE
    drvcmn_setreg32((intptr_t) &ccr_val, 18, 0x1f, config->dcyc);        // DCYC
    drvcmn_setreg32((intptr_t) &ccr_val, 16, 0x3, config->absize);       // ABSIZE
    drvcmn_setreg32((intptr_t) &ccr_val, 14, 0x3, config->abmode);       // ABMODE
    drvcmn_setreg32((intptr_t) &ccr_val, 12, 0x3, config->adsize);       // ADSIZE
    drvcmn_setreg32((intptr_t) &ccr_val, 10, 0x3, config->admode);       // ADMODE
    drvcmn_setreg32((intptr_t) &ccr_val, 8, 0x3, config->imode);         // IMODE
    drvcmn_setreg32((intptr_t) &ccr_val, 0, 0xff, config->instruction);  // INSTRUCTION

    DBGLOG2("CCR %08x, FMODE %02x", ccr_val, config->fmode);
    drvcmn_setreg32(QSPI_R_BASE + 0x14 /* QUADSPI_CCR */, 0, ~(uint32_t) 0, ccr_val);    // set


//    er = twai_flg(FLG_DRVFLX, FLGPTN_QSPI_TRANSFER_COMPLETE, TWF_ANDW, &flgptn, 3000);
//    assert(er == E_OK);

    // 割込み無効
    drvcmn_setreg32(QSPI_R_BASE + 0x0 /* QUADSPI_CR */, 0, (0x1 << 16) | (0x1 << 17), 0);   // TCIE, TEIE

    return 0;
}

/*
 * QSPIの転送開始
 */
int qspi_transfer(const QSPI_TRANSFER_CONFIG_T* config, const QSPI_TRANSFER_DATA_T* data)
{
    assert(config);

    ER er = E_OK;
    FLGPTN flgptn = 0;
    int spi_error = 0;

    assert(drvcmn_getreg32(QSPI_R_BASE + 0x8 /* QUADSPI_SR */, 5, 0x1) == 0);

    // データ転送がある場合
    if (config->dmode != DMODE_NO_DATA) {
        // QUADSPI_DLR
        assert(data);
        assert(data->length > 0);
        drvcmn_setreg32(QSPI_R_BASE + 0x10 /* QUADSPI_DLR */, 0, ~(uint32_t)0, data->length - 1);	// QUADSPI_DLR
    }

    // ステータスレジスタクリア
    drvcmn_setreg32(QSPI_R_BASE + 0x0C /* QUADSPI_FCR */, 0, (0x1 << 1) | (0x1 << 0), ~(uint32_t)0);	// CTCF,CTEF

    // イベントフラグクリア
//    er = clr_flg(FLG_DRVFLX, ~FLGPTN_QSPI_TRANSFER_COMPLETE);
//    assert(er == E_OK);

    // 割込み有効
    drvcmn_setreg32(QSPI_R_BASE + 0x0 /* QUADSPI_CR */, 0, (0x1 << 16) | (0x1 << 17), ~(uint32_t)0);	// TCIE, TEIE

    // QUADSPI_CCR
    uint32_t ccr_val = 0;
    ccr_val = drvcmn_getreg32(QSPI_R_BASE + 0x14 /* QUADSPI_CCR */, 0, ~(uint32_t)0);	// QUADSPI_CCR
    drvcmn_setreg32((intptr_t)&ccr_val, 26, 0x3, config->fmode);		// FMODE
    drvcmn_setreg32((intptr_t)&ccr_val, 24, 0x3, config->dmode);		// DMODE
    drvcmn_setreg32((intptr_t)&ccr_val, 18, 0x1f, config->dcyc);		// DCYC
    drvcmn_setreg32((intptr_t)&ccr_val, 16, 0x3, config->absize);		// ABSIZE
    drvcmn_setreg32((intptr_t)&ccr_val, 14, 0x3, config->abmode);		// ABMODE
    drvcmn_setreg32((intptr_t)&ccr_val, 12, 0x3, config->adsize);		// ADSIZE
    drvcmn_setreg32((intptr_t)&ccr_val, 10, 0x3, config->admode);		// ADMODE
    drvcmn_setreg32((intptr_t)&ccr_val, 8, 0x3, config->imode);			// IMODE
    drvcmn_setreg32((intptr_t)&ccr_val, 0, 0xff, config->instruction);	// INSTRUCTION
    drvcmn_setreg32(QSPI_R_BASE + 0x14 /* QUADSPI_CCR */, 0, ~(uint32_t)0, ccr_val);	// set

    // アドレスがある場合
    if (config->admode != ADMODE_NO_ADDRESS) {
        assert(data);
        assert(data->ar < FLASH_SIZE);
        drvcmn_setreg32(QSPI_R_BASE + 0x18 /* QUADSPI_AR */, 0, ~(uint32_t)0, data->ar);
    }

    // Alternate bytes がある場合
    if (config->abmode != ABMODE_NO_ALTERNATE_BYTES) {
        assert(data);
        drvcmn_setreg32(QSPI_R_BASE + 0x1C /* QUADSPI_ABR */, 0, ~(uint32_t)0, data->abr);
    }

    // データ転送がある場合
    if (config->dmode != DMODE_NO_DATA) {
        assert(data);
        assert(data->length > 0);

        // フラグクリア
//        er = clr_flg(FLG_DRVFLX, ~FLGPTN_DMA_COMPLETE);
//        assert(er == E_OK);

        drvcmn_setreg32(QSPI_R_BASE + 0x20 /* QUADSPI_DR */, 0, ~(uint32_t)0, *(uint32_t*)data->mcuaddr);

//        // DMA設定
//        DRVCMN_DMA_XFER_SETTING_T dmas_setting = {0};
//        if (config->fmode == FMODE_INDIRECT_READ) {
//            dmas_setting.dir = DRVCMN_DMA_DIR_P2M;
//            dmas_setting.src = QSPI_R_BASE + 0x20;
//            dmas_setting.dest = (uintptr_t)data->mcuaddr;
//            dmas_setting.use_fifo = true;
//        } else if (config->fmode == FMODE_INDIRECT_WRITE) {
//            dmas_setting.dir = DRVCMN_DMA_DIR_M2P;
//            dmas_setting.src = (uintptr_t)data->mcuaddr;
//            dmas_setting.dest = QSPI_R_BASE + 0x20;
//            dmas_setting.use_fifo = true;
//        } else {
//            assert(false);
//        }
//        dmas_setting.nbytes = data->length;
//        dmas_setting.nxfer = 1;
//        dmas_setting.isr = dma_isr_callback;
//
//        // DMAストリーム有効
//        drvcmn_dma_transfer_enable(&DMA_STREAM, &dmas_setting);
//
//        // QSPI DMA有効
//        drvcmn_setreg32(QSPI_R_BASE + 0x0 /* QUADSPI_CR */, 2, 0x1, 1);	// DMAEN => 1
//
//        // QSPI & DMA 転送完了待ち
//        er = twai_flg(FLG_DRVFLX, FLGPTN_QSPI_TRANSFER_COMPLETE | FLGPTN_DMA_COMPLETE, TWF_ANDW, &flgptn, 3000);
//        assert(er == E_OK);
    } else {	// データ転送が無い場合
        // TCフラグ待ち
//        er = twai_flg(FLG_DRVFLX, FLGPTN_QSPI_TRANSFER_COMPLETE, TWF_ANDW, &flgptn, 3000);
//        assert(er == E_OK);
    }

    // 割込み無効
    drvcmn_setreg32(QSPI_R_BASE + 0x0 /* QUADSPI_CR */, 0, (0x1 << 16) | (0x1 << 17), 0);	// TCIE, TEIE
    // DMAストリーム無効
//    drvcmn_dma_transfer_disable(&DMA_STREAM);
    // QSPI DMA無効
    drvcmn_setreg32(QSPI_R_BASE + 0x0 /* QUADSPI_CR */, 2, 0x1, 0);	// DMAEN => 0

    return 0;
}

// DMAコールバック
void dma_isr_callback()
{
    //DBGLOG0("drvcam_dma_isr_callback");

    ER er = iset_flg(FLG_DRVFLX, FLGPTN_DMA_COMPLETE);
    assert(er == E_OK);
}


// フラッシュのステータスポーリング
void qspi_wait_flash_status(uint32_t mask, uint32_t match)
{
    ER er = E_OK;

    // ステータスマスク
    drvcmn_setreg32(QSPI_R_BASE + 0x24 /* QUADSPI_PSMKR */, 0, ~(uint32_t)0, mask);

    // ステータスマッチ
    drvcmn_setreg32(QSPI_R_BASE + 0x28 /* QUADSPI_PSMAR */, 0, ~(uint32_t)0, match);

    // ポーリングインターバル
    drvcmn_setreg32(QSPI_R_BASE + 0x2C /* QUADSPI_PIR */, 0, 0xffff, 0x10);
    
    // ステータスマッチフラグクリア
    drvcmn_setreg32(QSPI_R_BASE + 0x0C /* QUADSPI_FCR */, 0, (0x1 << 3), ~(uint32_t)0);	// CSMF

    // イベントフラグクリア
    er = clr_flg(FLG_DRVFLX, ~FLGPTN_QSPI_STATUS_MATCH);
    assert(er == E_OK);

    // ステータスマッチ割込み有効
    drvcmn_setreg32(QSPI_R_BASE + 0x0 /* QUADSPI_CR */, 0, (0x1 << 19), ~(uint32_t)0);	// SMIE

    drvcmn_setreg32(QSPI_R_BASE + 0x10 /* QUADSPI_DLR */, 0, ~(uint32_t)0, 0);	// QUADSPI_DLR
    uint32_t ccr_val = drvcmn_getreg32(QSPI_R_BASE + 0x14 /* QUADSPI_CCR */, 0, ~(uint32_t)0);	// QUADSPI_CCR
    drvcmn_setreg32((intptr_t)&ccr_val, 26, 0x3, FMODE_AUTOMATIC_POLLING);		// FMODE
    drvcmn_setreg32((intptr_t)&ccr_val, 24, 0x3, DMODE_SINGLE_LINE);			// DMODE
    drvcmn_setreg32((intptr_t)&ccr_val, 18, 0x1f, 0);							// DCYC
    drvcmn_setreg32((intptr_t)&ccr_val, 16, 0x3, ABSIZE_8BIT);					// ABSIZE
    drvcmn_setreg32((intptr_t)&ccr_val, 14, 0x3, ABMODE_NO_ALTERNATE_BYTES);	// ABMODE
    drvcmn_setreg32((intptr_t)&ccr_val, 12, 0x3, ADSIZE_8BIT);					// ADSIZE
    drvcmn_setreg32((intptr_t)&ccr_val, 10, 0x3, ADMODE_NO_ADDRESS);			// ADMODE
    drvcmn_setreg32((intptr_t)&ccr_val, 8, 0x3, IMODE_SINGLE_LINE);				// IMODE
    drvcmn_setreg32((intptr_t)&ccr_val, 0, 0xff, 0x05);							// INSTRUCTION => 0x05(Read Status Register)
    drvcmn_setreg32(QSPI_R_BASE + 0x14 /* QUADSPI_CCR */, 0, ~(uint32_t)0, ccr_val);	// set

    // ステータスマッチ割込み待ち
    FLGPTN flgptn = 0;
    er = twai_flg(FLG_DRVFLX, FLGPTN_QSPI_STATUS_MATCH, TWF_ANDW, &flgptn, 5000);
    assert(er == E_OK);

    // ステータスマッチ割込み無効
    drvcmn_setreg32(QSPI_R_BASE + 0x0 /* QUADSPI_CR */, 0, (0x1 << 19), 0);	// SMIE

    return;
}

void qspi_memmap()
{

}
