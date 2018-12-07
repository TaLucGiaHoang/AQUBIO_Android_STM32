/*
 * VA-X DMA共通処理
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/20 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "drvcmn_dma.h"

#include <kernel.h>
#include <t_syslog.h>
#include <sil.h>
#include <stm32f7xx.h>

#include "drvcmn.h"

/*
 * デバッグ変数
 */
uint32_t g_debug_array[32] = {0};

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
    intptr_t reg_base;
    intptr_t isr;
    uint32_t isr_bitoff;
    intptr_t ifcr;
    uint32_t ifcr_bitoff;
} DMA_STR_REG_T;


typedef struct {
    size_t bytes_per_data;
    bool_t dbm;
    uint32_t dir;
    DRVCMN_DMA_ISR_FUNC_T isr_tc;
    size_t nbytes;
    size_t nxfer;
} STR_SETTING_T;

/*
 * 定数定義
 */

// DMAストリーム毎のレジスタアドレス
static const DMA_STR_REG_T DMA_STR_REGS[] = {
    { TADR_DMA1_STM0_BASE,	TADR_DMA1_BASE + TOFF_DMAI_LISR,	0,	TADR_DMA1_BASE + TOFF_DMAI_LIFCR,	0,	},	// [0]DMA1_STREAM0
    { TADR_DMA1_STM1_BASE,	TADR_DMA1_BASE + TOFF_DMAI_LISR,	6,	TADR_DMA1_BASE + TOFF_DMAI_LIFCR,	6,	},	// [1]DMA1_STREAM1
    { TADR_DMA1_STM2_BASE,	TADR_DMA1_BASE + TOFF_DMAI_LISR,	16,	TADR_DMA1_BASE + TOFF_DMAI_LIFCR,	16,	},	// [2]DMA1_STREAM2
    { TADR_DMA1_STM3_BASE,	TADR_DMA1_BASE + TOFF_DMAI_LISR,	22,	TADR_DMA1_BASE + TOFF_DMAI_LIFCR,	22,	},	// [3]DMA1_STREAM3
    { TADR_DMA1_STM4_BASE,	TADR_DMA1_BASE + TOFF_DMAI_HISR,	0,	TADR_DMA1_BASE + TOFF_DMAI_HIFCR,	0,	},	// [4]DMA1_STREAM4
    { TADR_DMA1_STM5_BASE,	TADR_DMA1_BASE + TOFF_DMAI_HISR,	6,	TADR_DMA1_BASE + TOFF_DMAI_HIFCR,	6,	},	// [5]DMA1_STREAM5
    { TADR_DMA1_STM6_BASE,	TADR_DMA1_BASE + TOFF_DMAI_HISR,	16,	TADR_DMA1_BASE + TOFF_DMAI_HIFCR,	16,	},	// [6]DMA1_STREAM6
    { TADR_DMA1_STM7_BASE,	TADR_DMA1_BASE + TOFF_DMAI_HISR,	22,	TADR_DMA1_BASE + TOFF_DMAI_HIFCR,	22,	},	// [7]DMA1_STREAM7
    { TADR_DMA2_STM0_BASE,	TADR_DMA2_BASE + TOFF_DMAI_LISR,	0,	TADR_DMA2_BASE + TOFF_DMAI_LIFCR,	0,	},	// [8]DMA2_STREAM0
    { TADR_DMA2_STM1_BASE,	TADR_DMA2_BASE + TOFF_DMAI_LISR,	6,	TADR_DMA2_BASE + TOFF_DMAI_LIFCR,	6,	},	// [9]DMA2_STREAM1
    { TADR_DMA2_STM2_BASE,	TADR_DMA2_BASE + TOFF_DMAI_LISR,	16,	TADR_DMA2_BASE + TOFF_DMAI_LIFCR,	16,	},	// [10]DMA2_STREAM2
    { TADR_DMA2_STM3_BASE,	TADR_DMA2_BASE + TOFF_DMAI_LISR,	22,	TADR_DMA2_BASE + TOFF_DMAI_LIFCR,	22,	},	// [11]DMA2_STREAM3
    { TADR_DMA2_STM4_BASE,	TADR_DMA2_BASE + TOFF_DMAI_HISR,	0,	TADR_DMA2_BASE + TOFF_DMAI_HIFCR,	0,	},	// [12]DMA2_STREAM4
    { TADR_DMA2_STM5_BASE,	TADR_DMA2_BASE + TOFF_DMAI_HISR,	6,	TADR_DMA2_BASE + TOFF_DMAI_HIFCR,	6,	},	// [13]DMA2_STREAM5
    { TADR_DMA2_STM6_BASE,	TADR_DMA2_BASE + TOFF_DMAI_HISR,	16,	TADR_DMA2_BASE + TOFF_DMAI_HIFCR,	16,	},	// [14]DMA2_STREAM6
    { TADR_DMA2_STM7_BASE,	TADR_DMA2_BASE + TOFF_DMAI_HISR,	22,	TADR_DMA2_BASE + TOFF_DMAI_HIFCR,	22,	},	// [15]DMA2_STREAM7
};

// DMAストリーム毎の割込みベクタ
static const int DMA_STR_IRQS[] = {
    IRQ_VECTOR_DMA1_STREAM0,	// [0]DMA1_STREAM0
    IRQ_VECTOR_DMA1_STREAM1,	// [1]DMA1_STREAM1
    IRQ_VECTOR_DMA1_STREAM2,	// [2]DMA1_STREAM2
    IRQ_VECTOR_DMA1_STREAM3,	// [3]DMA1_STREAM3
    IRQ_VECTOR_DMA1_STREAM4,	// [4]DMA1_STREAM4
    IRQ_VECTOR_DMA1_STREAM5,	// [5]DMA1_STREAM5
    IRQ_VECTOR_DMA1_STREAM6,	// [6]DMA1_STREAM6
    IRQ_VECTOR_DMA1_STREAM7,	// [7]DMA1_STREAM7
    IRQ_VECTOR_DMA2_STREAM0,	// [8]DMA2_STREAM0
    IRQ_VECTOR_DMA2_STREAM1,	// [9]DMA2_STREAM1
    IRQ_VECTOR_DMA2_STREAM2,	// [10]DMA2_STREAM2
    IRQ_VECTOR_DMA2_STREAM3,	// [11]DMA2_STREAM3
    IRQ_VECTOR_DMA2_STREAM4,	// [12]DMA2_STREAM4
    IRQ_VECTOR_DMA2_STREAM5,	// [13]DMA2_STREAM5
    IRQ_VECTOR_DMA2_STREAM6,	// [14]DMA2_STREAM6
    IRQ_VECTOR_DMA2_STREAM7,	// [15]DMA2_STREAM7
};

/*
 * 内部変数
 */

// ストリーム設定
static STR_SETTING_T s_str_setting[16];

/*
 * 内部関数のプロトタイプ
 */


/*
 * 公開関数
 */

// DMAストリーム初期化
void drvcmn_dma_initialize(const DRVCMN_DMA_STR_T* stream, const DRVCMN_DMA_SETTING_T* setting)
{
    assert(stream && (stream->dmano == 1 || stream->dmano == 2));
    assert(stream && (stream->strno >= 0 && stream->strno <= 7));
    assert(setting);

    // RCC
    if (stream->dmano == 1) {
        drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_AHB1ENR, 0, RCC_AHB1ENR_DMA1EN, RCC_AHB1ENR_DMA1EN);
        DUMP_REG32("RCC_AHB1ENR", TADR_RCC_BASE + TOFF_RCC_AHB1ENR);
    } else if (stream->dmano == 2) {
        drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_AHB1ENR, 0, RCC_AHB1ENR_DMA2EN, RCC_AHB1ENR_DMA2EN);
        DUMP_REG32("RCC_AHB1ENR", TADR_RCC_BASE + TOFF_RCC_AHB1ENR);
    } else {
        assert(false);
    }

    int str_index = (stream->dmano - 1) * 8 + stream->strno;
    const DMA_STR_REG_T* str_reg = &(DMA_STR_REGS[str_index]);
    intptr_t dmas_base = str_reg->reg_base;

    // DMA_SxCR
    drvcmn_setreg32(dmas_base + TOFF_DMAS_CR, 0, 0x1FEFFFFF,
                    ((setting->chsel << 25) & DMA_SxCR_CHSEL)		|	// [27:25]CHSEL
                    (setting->dbm ? DMA_SxCR_DBM : 0)				|	// [18]DBM
                    ((setting->pl << 16) & DMA_SxCR_PL)				|	// [17:16]PL
                    ((setting->data_size << 13) & DMA_SxCR_MSIZE)	|	// [14:13]MSIZE
                    ((setting->data_size << 11) & DMA_SxCR_PSIZE)	|	// [12:11]PSIZE
                    (setting->minc ? DMA_SxCR_MINC : 0)				|	// [10]MINC
                    (setting->pinc ? DMA_SxCR_PINC : 0)				|	// [9]PINC
                    ((setting->dir << 6) & DMA_SxCR_DIR)			|	// [7:6]DIR
                    DMA_SxCR_TCIE									|	// [4]TCIE
                    DMA_SxCR_TEIE									|	// [2]TEIE
                    DMA_SxCR_DMEIE);									// [1]DMEIE
                    
    DUMP_REG32("DMA_SxCR", dmas_base + TOFF_DMAS_CR);

    // DMA_SxFCR
    drvcmn_setreg32(dmas_base + TOFF_DMAS_FCR, 0, DMA_SxFCR_DMDIS, 0x0);
    DUMP_REG32("DMA_SxFCR", dmas_base + TOFF_DMAS_FCR);

    // DMA_IFCR
    static const uint32_t IFCR_BITS = (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0);
    drvcmn_setreg32(str_reg->ifcr, str_reg->ifcr_bitoff, IFCR_BITS, IFCR_BITS);
    DUMP_REG32("DMA_IFCR", str_reg->ifcr);

    // 割込み禁止を解除
    ER er = ena_int(DMA_STR_IRQS[str_index]);
    assert(er == E_OK);

    // 設定を保持
    STR_SETTING_T* str_setting = &(s_str_setting[str_index]);
    switch (setting->data_size) {
    case DRVCMN_DMA_DATASIZE_8BIT:
        str_setting->bytes_per_data = 1;
        break;
    case DRVCMN_DMA_DATASIZE_16BIT:
        str_setting->bytes_per_data = 2;
        break;
    case DRVCMN_DMA_DATASIZE_32BIT:
        str_setting->bytes_per_data = 4;
        break;
    default:
        assert(false);
        break;
    }
    str_setting->dbm = setting->dbm;
    str_setting->dir = setting->dir;

    return;
}

// DMAストリームを有効にする.
void drvcmn_dma_transfer_enable(const DRVCMN_DMA_STR_T* stream, const DRVCMN_DMA_XFER_SETTING_T* xfer_setting)
{
    assert(stream);
    assert(xfer_setting);
    //assert(xfer_setting->nbytes / 4 <= 0xFFFF);

    int str_index = (stream->dmano - 1) * 8 + stream->strno;
    STR_SETTING_T* str_setting = &(s_str_setting[str_index]);

    // 設定を保持
    str_setting->dir = xfer_setting->dir;
    str_setting->nbytes = xfer_setting->nbytes;
    str_setting->nxfer = xfer_setting->nxfer;
    str_setting->isr_tc = xfer_setting->isr;

    // モードに応じて PAR, M0AR, M1AR のアドレスを設定
    uint32_t par = 0;
    uint32_t m0ar = 0;
    uint32_t m1ar = 0;
    if (str_setting->dir == DRVCMN_DMA_DIR_P2M) {
        par = xfer_setting->src;
        m0ar = xfer_setting->dest;
        if (str_setting->dbm) {
            m1ar = m0ar + xfer_setting->nbytes;
        }
    } else if (str_setting->dir == DRVCMN_DMA_DIR_M2P) {
        m0ar = xfer_setting->src;
        par = xfer_setting->dest;
    } else if (str_setting->dir == DRVCMN_DMA_DIR_M2M) {
        m0ar = xfer_setting->src;
        m1ar = xfer_setting->dest;
    }

    // ストリームベース
    intptr_t dmas_base = DMA_STR_REGS[str_index].reg_base;

    // DIR
    drvcmn_setreg32(dmas_base + TOFF_DMAS_CR, 6, 0x3, xfer_setting->dir);	// [7:6]DIR

    // PAR
    drvcmn_setreg32(dmas_base + TOFF_DMAS_PAR, 0, ~(uint32_t)0, par);
    DUMP_REG32("DMA_SxPAR", dmas_base + TOFF_DMAS_PAR);

    // M0AR
    drvcmn_setreg32(dmas_base + TOFF_DMAS_M0AR, 0, ~(uint32_t)0, m0ar);
    DUMP_REG32("DMA_SxM0AR", dmas_base + TOFF_DMAS_M0AR);

    // M1AR
    drvcmn_setreg32(dmas_base + TOFF_DMAS_M1AR, 0, ~(uint32_t)0, m1ar);
    DUMP_REG32("DMA_SxM1AR", dmas_base + TOFF_DMAS_M1AR);

    // NDTR
    drvcmn_setreg32(dmas_base + TOFF_DMAS_NDTR, 0, 0xFFFF, xfer_setting->nbytes / str_setting->bytes_per_data);
    DUMP_REG32("DMA_SxNDTR", dmas_base + TOFF_DMAS_NDTR);

    // FCR
    drvcmn_setreg32(dmas_base + TOFF_DMAS_FCR, 0, DMA_SxFCR_DMDIS, (xfer_setting->use_fifo ? DMA_SxFCR_DMDIS : 0));
    DUMP_REG32("DMA_SxFCR", dmas_base + TOFF_DMAS_FCR);

    // Double Buffer Mode の時は CT ビットをクリア
    if (str_setting->dbm) {
        // CT
        drvcmn_setreg32(dmas_base + TOFF_DMAS_CR, 19, 0x1, 0);	// [19]CT => 0
    }

    // DMA_SxCR
    drvcmn_setreg32(dmas_base + TOFF_DMAS_CR, 0, DMA_SxCR_EN, DMA_SxCR_EN);	// [0]EN
    DUMP_REG32("DMA_SxCR", dmas_base + TOFF_DMAS_CR);

    return;
}

void drvcmn_dma_transfer_disable(const DRVCMN_DMA_STR_T* stream)
{
    int str_index = (stream->dmano - 1) * 8 + stream->strno;
    intptr_t dmas_base = DMA_STR_REGS[str_index].reg_base;

    // DMA_SxCR
    drvcmn_setreg32(dmas_base + TOFF_DMAS_CR, 0, DMA_SxCR_EN, 0);	// [0]EN
    DUMP_REG32("DMA_SxCR", dmas_base + TOFF_DMAS_CR);

#if 0
    for (int i = 0;;i++) {
        if (drvcmn_getreg32(dmas_base + TOFF_DMAS_CR, 0, DMA_SxCR_EN) == 0) {
            break;
        }
    }
#endif
    
}

/*
 *
 */

// DMA ISR
void drvcmn_dma_isr(intptr_t exinf)
{
    assert(exinf >= 0 && exinf <= 15);
    //syslog(LOG_NOTICE, "drvcmn_dma_isr");

    STR_SETTING_T* str_setting = &(s_str_setting[exinf]);
    const DMA_STR_REG_T* str_reg = &(DMA_STR_REGS[exinf]);
    intptr_t dmas_base = str_reg->reg_base;

    static const uint32_t ISR_BITS = (DMA_LISR_TCIF0 | DMA_LISR_HTIF0 | DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_FEIF0);
    uint32_t isr = drvcmn_getreg32(str_reg->isr, str_reg->isr_bitoff, ISR_BITS);

    //syslog(LOG_NOTICE, "isr: 0x%08x", isr);

    uint32_t ifcr_bit = 0;
    if (isr & DMA_LISR_TCIF0) {	// Transfer complete interrupt
        if (str_setting->dbm) {
            if (str_setting->nxfer == 0) {
                // DMA停止
                // DMA_SxCR
                drvcmn_setreg32(dmas_base + TOFF_DMAS_CR, 0, DMA_SxCR_EN, 0);	// [0]EN
                // 上位側ISRへ通知
                if (str_setting->isr_tc) {
                    str_setting->isr_tc();
                }
            } else {
                // Double buffer mode 時は転送先アドレスを前に進める. 上位側に通知はしない.
                uint32_t ct = drvcmn_getreg32(dmas_base + TOFF_DMAS_CR, 0, DMA_SxCR_CT);	// [19]CT
                uint32_t current_addr = 0;
                if (ct == 0) {
                    // CT == 0
                    current_addr = drvcmn_getreg32(dmas_base + TOFF_DMAS_M0AR, 0, ~(uint32_t)0);
                    drvcmn_setreg32(dmas_base + TOFF_DMAS_M1AR, 0, ~(uint32_t)0, current_addr + str_setting->nbytes);
                } else {
                    // CT == 1
                    current_addr = drvcmn_getreg32(dmas_base + TOFF_DMAS_M1AR, 0, ~(uint32_t)0);
                    drvcmn_setreg32(dmas_base + TOFF_DMAS_M0AR, 0, ~(uint32_t)0, current_addr + str_setting->nbytes);
                }
            }
            --str_setting->nxfer;
        } else {
            // 上位側のISRへ通知
            if (str_setting->isr_tc) {
                str_setting->isr_tc();
            }
        }
        ifcr_bit = DMA_LIFCR_CTCIF0;
    } else if (isr & DMA_LISR_TEIF0) {	// Transfer error interrupt
        ifcr_bit = DMA_LIFCR_CTEIF0;
        assert(false);
    } else if (isr & DMA_LISR_DMEIF0) {	// Direct mode error interrupt
        ifcr_bit = DMA_LIFCR_CDMEIF0;
        assert(false);
    } else {
        if (isr & DMA_LISR_HTIF0) {	// Half transfer interrupt
#if 0	// 無視する
        ifcr_bit = DMA_LIFCR_CHTIF0;
        assert(false);
#endif
        } else {
            assert(false);
        }
    }

    // 割込みクリア
    sil_wrw_mem((uint32_t*)(str_reg->ifcr), ifcr_bit << str_reg->ifcr_bitoff);
}

