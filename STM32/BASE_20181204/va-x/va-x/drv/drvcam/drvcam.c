/*
 * VA-X カメラドライバ
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/27 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成(GC2155)
 * - 2017/03/14 Takuya Goto <goto-takuya@bionics-k.co.jp>: NCM13-J(日本ケミコン)用に変更
 */

#include "drvcam.h"

#include <string.h>
#include <kernel.h>
#include <sil.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "stm32f7xx.h"

#include "kernel_cfg.h"
#include "drvcmn.h"
#include "drvcmn_gpio.h"
#include "drvcmn_i2c.h"
#include "drvcmn_dma.h"
#include "drvcam_register.h"

/*
 * マクロ定義
 */

// ピン変更
#define USE_MCO_FOR_CAMERA

// レジスタダンプ
//#define DEBUG_USE_DUMP_CAMERA_REGS

// サービスコールのエラーのログ出力
#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))
inline void svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}

/*
 * 型
 */


/*
 * 定数
 */

// GPIOピン(DCMI)
static const DRVCMN_GPIO_PIN_T GPIO_PINS_DCMI[] = {
    { DRVCMN_GPIO_PORT_B, 7,	},	// DCMI_VSYNC
    { DRVCMN_GPIO_PORT_A, 4,	},	// DCMI_HSYNC
    { DRVCMN_GPIO_PORT_A, 6,	},	// DCMI_PIXCLK
    { DRVCMN_GPIO_PORT_C, 6,	},	// DCMI_D0
    { DRVCMN_GPIO_PORT_C, 7,	},	// DCMI_D1
    { DRVCMN_GPIO_PORT_C, 8,	},	// DCMI_D2
    { DRVCMN_GPIO_PORT_C, 9,	},	// DCMI_D3
    { DRVCMN_GPIO_PORT_E, 4,	},	// DCMI_D4
    { DRVCMN_GPIO_PORT_D, 3,	},	// DCMI_D5
    { DRVCMN_GPIO_PORT_E, 5,	},	// DCMI_D6
    { DRVCMN_GPIO_PORT_E, 6,	},	// DCMI_D7
};

// GPIOピン設定(DCMI)
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_DCMI = {
    .mode = DRVCMN_GPIO_MODE_AF,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLDOWN,
    .afno = 13,
};

// GPIOピン(カメラ電源)
//static const DRVCMN_GPIO_PIN_T GPIO_PIN_CAMERA_ON =		{ DRVCMN_GPIO_PORT_D, 8		};	// CAM_ON (Active L)
static const DRVCMN_GPIO_PIN_T GPIO_PIN_CAMERA_STDBY =	{ DRVCMN_GPIO_PORT_D, 7		};	// CAM_STDBY (Active H)
static const DRVCMN_GPIO_PIN_T GPIO_PIN_CAMERA_RESET =	{ DRVCMN_GPIO_PORT_E, 3		};	// CAMERA用RESET (Active L)
#if 1	// TODO: 電源管理で制御する.
static const DRVCMN_GPIO_PIN_T GPIO_PIN_1V8_ON =		{ DRVCMN_GPIO_PORT_E, 13	};	// 1.8V_ON (Active H)
static const DRVCMN_GPIO_PIN_T GPIO_PIN_AVDD_ON =		{ DRVCMN_GPIO_PORT_D, 9		};	// AVDD_ON (Active L)
#endif

// GPIOピン設定(カメラ電源)
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_CAMERA_POWER_PP = {
    .mode = DRVCMN_GPIO_MODE_GPOUT,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLDOWN,
};

static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_CAMERA_POWER_OD = {
    .mode = DRVCMN_GPIO_MODE_GPOUT,
    .otype = DRVCMN_GPIO_OTYPE_OD,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLUP,
};

static const struct {
    const DRVCMN_GPIO_PIN_T*		pin;
    const DRVCMN_GPIO_SETTING_T*	setting;
    bool_t							initial_value;
} GPIO_PINS_CAMERA_POWER[] = {
//    { &GPIO_PIN_CAMERA_ON,		&GPIO_SETTING_CAMERA_POWER_OD,	true	},	// CAM_ON (Active L)
    { &GPIO_PIN_CAMERA_STDBY,	&GPIO_SETTING_CAMERA_POWER_PP,	false	},	// CAM_STDBY (Active H)
    { &GPIO_PIN_CAMERA_RESET,	&GPIO_SETTING_CAMERA_POWER_OD,	true	},	// CAMERA用RESET (Active L)
#if 1	// TODO: 電源制御モジュールで制御する
    { &GPIO_PIN_1V8_ON,			&GPIO_SETTING_CAMERA_POWER_PP,	false	},	// 1.8V_ON (Active H)
    { &GPIO_PIN_AVDD_ON,		&GPIO_SETTING_CAMERA_POWER_OD,	true	},	// AVDD_ON (Active L)
#endif
};


// GPIOピン(I2C)
static const DRVCMN_GPIO_PIN_T GPIO_PINS_I2C[] = {
#ifdef USE_MCO_FOR_CAMERA
    { DRVCMN_GPIO_PORT_D,	13,	},	// I2C4_SDA
    { DRVCMN_GPIO_PORT_D,	12,	},	// I2C4_SCL
#else	// #ifdef USE_MCO_FOR_CAMERA
    { DRVCMN_GPIO_PORT_C,	9,	},	// I2C3_SDA
    { DRVCMN_GPIO_PORT_A,	8,	},	// I2C3_SCL
#endif	// #ifdef USE_MCO_FOR_CAMERA
};

// GPIOピン設定(I2C)
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_I2C = {
    .mode = DRVCMN_GPIO_MODE_AF,
    .otype = DRVCMN_GPIO_OTYPE_OD,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_NONE,
    .afno = 4,
};

#ifdef USE_MCO_FOR_CAMERA
// GPIOピン(MCO)
static const DRVCMN_GPIO_PIN_T GPIO_PIN_MCO = {
    DRVCMN_GPIO_PORT_A,	8	// MCO1
};

// GPIOピン設定(MCO)
static const DRVCMN_GPIO_SETTING_T GPIO_SETTING_MCO = {
    .mode = DRVCMN_GPIO_MODE_AF,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_NONE,
    .afno = 0,
};
#endif	// #ifdef USE_MCO_FOR_CAMERA

// カメラI2C番号
#ifdef USE_MCO_FOR_CAMERA
static const int32_t CAMERA_I2C = 4;	// I2C4
#else	// #ifdef USE_MCO_FOR_CAMERA
static const int32_t CAMERA_I2C = 3;	// I2C3
#endif	// #ifdef USE_MCO_FOR_CAMERA

// カメラI2Cスレーブアドレス
static const uint8_t CAMERA_I2C_SADD = (0xba >> 1);	// RWフラグを含めた8bit表記では write: 0xba, read: 0xbb

// DMAストリーム
static const DRVCMN_DMA_STR_T DMA_STREAM = {2,	1};	// DMA2 Stream1

// DMAストリーム設定
static const DRVCMN_DMA_SETTING_T DMA_SETTING = {
    .chsel = 1,								// Channel selection(0~7)
    .dbm = true,							// Double buffer mode
    .pl = DRVCMN_DMA_PL_VERY_HIGH,			// Priority level
    .data_size = DRVCMN_DMA_DATASIZE_32BIT,	// Memory/Peripheral data size
    .minc = true,							// Memory increment mode
    .pinc = false,							// Peripheral increment mode
    .dir = DRVCMN_DMA_DIR_P2M,				// Data transfer direction
};

// DMA転送設定
static const size_t DMA_NBYTES = (1024 * 4);
static const size_t DMA_NXFER = 74;

// カメラ設定後の待ち時間
static const int CAMERA_FRAME_SKIP = 2;

// レジスタ設定値
#define DRVCAM_REGVAL_DEFAULT		DRVCAM_REGVAL_NCM13J_DEFAULT		// (static const DRVCAM_REGVAL_T*)
#define DRVCAM_NUM_REGVAL_DEFAULT	DRVCAM_NUM_REGVAL_NCM13J_DEFAULT	// (static const size_t)

// メッセージ番号
enum {
    MSG_INITIALIZE = 1,
    MSG_POWER_ON,
    MSG_POWER_ON_COMPLETE,
    MSG_POWER_OFF,
    MSG_POWER_OFF_COMPLETE,
    MSG_PREPARE_CAPTURE,
    MSG_PREPARE_CAPTURE_COMPLETE,
    MSG_CAPTURE,
    MSG_CAPTURE_COMPLETE,
};

// イベントフラグ
static const FLGPTN FLGPTN_DMA_COMPLETE =	(0x1 << 0);
static const FLGPTN FLGPTN_FRAME_SKIP =	(0x1 << 1);

/*
 * 内部変数
*/

static DRVCAM_CALLBACK_T s_callback_initialize = NULL;
static DRVCAM_CALLBACK_T s_callback_power_on = NULL;
static DRVCAM_CALLBACK_T s_callback_power_off = NULL;
static DRVCAM_CALLBACK_T s_callback_prepare_capture = NULL;
static DRVCAM_CALLBACK_T s_callback_capture = NULL;
static DRVCAM_CAPTURE_PARAM_T s_capture_param = {0};
static uint8_t* s_capture_output;
static size_t s_capture_output_size;
static int s_frame_skip = -1;

/*
 * 内部関数プロトタイプ
 */

static void cam_power_on();
static void cam_power_off();
static void cam_prepare_capture();
static void cam_capture();

static int32_t set_camera_params();
static void dma_isr_callback();

static void dcmi_initialize();
static void dcmi_enable();
static void dcmi_disable();
static void dcmi_capture_enable();
static void dcmi_capture_disable();

static int32_t i2c_initialize();
static int32_t i2c_write(uint8_t reg_addr, uint16_t data);
static int32_t i2c_read(uint16_t *odata, uint8_t reg_addr);

#ifdef DEBUG_USE_DUMP_CAMERA_REGS
static void dump_camera_regs();
#endif	// #ifdef DEBUG_USE_DUMP_CAMERA_REGS

/*
 * カメラ関係ペリフェラル初期化
 */
void drvcam_initialize_peripherals()
{
    // カメラ電源用GPIO設定
    for (int i = 0; i < sizeof(GPIO_PINS_CAMERA_POWER)/sizeof(GPIO_PINS_CAMERA_POWER[0]); i++) {
        drvcmn_gpio_pin_initialize(GPIO_PINS_CAMERA_POWER[i].pin, GPIO_PINS_CAMERA_POWER[i].setting);
        drvcmn_gpio_pin_set(GPIO_PINS_CAMERA_POWER[i].pin, GPIO_PINS_CAMERA_POWER[i].initial_value);
    };

#ifdef USE_MCO_FOR_CAMERA
    //  MCO設定
    drvcmn_gpio_pin_initialize(&GPIO_PIN_MCO, &GPIO_SETTING_MCO);

    // RCC_CFGR
    drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_CFGR, 21, 0x3, 0x2);	// [22:21]MCO1
#endif	// #ifdef USE_MCO_FOR_CAMERA

    // I2C初期化
    i2c_initialize();

    // DCMI初期化
    dcmi_initialize();

    // DMA初期化
    drvcmn_dma_initialize(&DMA_STREAM, &DMA_SETTING);

    return;
}

/*
 * カメラドライバ初期化(タスク起動)
 */
void drvcam_initialize(DRVCAM_CALLBACK_T callback)
{
    assert(!s_callback_initialize);

    ER er = 0;

    er = act_tsk(TSK_DRVCAM);
    assert(er == E_OK);

    s_callback_initialize = callback;
    er = snd_dtq(DTQ_DRVCAM, (intptr_t)MSG_INITIALIZE);
    assert(er == E_OK);

    return;
}

/*
 * カメラ電源ON
*/
void drvcam_power_on(DRVCAM_CALLBACK_T callback)
{
    assert(!s_callback_power_on);

    s_callback_power_on = callback;

    ER er = snd_dtq(DTQ_DRVCAM, (intptr_t)MSG_POWER_ON);
    assert(er == E_OK);

    return;
}

/*
 * カメラ電源OFF
 */
void drvcam_power_off(DRVCAM_CALLBACK_T callback)
{
    assert(!s_callback_power_off);

    s_callback_power_off = callback;

    ER er = snd_dtq(DTQ_DRVCAM, (intptr_t)MSG_POWER_OFF);
    assert(er == E_OK);

    return;
}

/*
 * 撮影準備
 */
void drvcam_prepare_capture(DRVCAM_CALLBACK_T callback, const DRVCAM_CAPTURE_PARAM_T* param)
{
    assert(!s_callback_prepare_capture);
    assert(callback);
    assert(param);

    s_callback_prepare_capture = callback;
    memcpy(&(s_capture_param), param, sizeof(s_capture_param));

    ER er = snd_dtq(DTQ_DRVCAM, (intptr_t)MSG_PREPARE_CAPTURE);
    assert(er == E_OK);

    return;
}

/*
 * 撮影
 */
void drvcam_capture(DRVCAM_CALLBACK_T callback, uint8_t* output, size_t size)
{
    assert(!s_callback_capture);
    assert(!s_capture_output);
    assert(s_capture_output_size == 0);
    assert(callback);
    assert(output);
    assert(size >= DRVCAM_CAPTURE_BUFFER_SIZE);

    s_callback_capture = callback;
    s_capture_output = output;
    s_capture_output_size = size;

    ER er = snd_dtq(DTQ_DRVCAM, (intptr_t)MSG_CAPTURE);
    assert(er == E_OK);

    return;
}

/*
 * カメラドライバタスク
 */
void drvcam_task(intptr_t exinf)
{
    while (true) {
        intptr_t msg = 0;
        ER er_rcv = trcv_dtq(DTQ_DRVCAM, &msg, TMO_FEVR);
        assert((er_rcv == E_OK) || (er_rcv == E_TMOUT));

        SYSTIM systim1 = 0;
        get_tim(&systim1);

        switch (msg) {
        case MSG_INITIALIZE:
            s_callback_initialize(DRVCAM_CALLBACK_INITIALIZE, 0);
            s_callback_initialize = NULL;
            break;
        case MSG_POWER_ON:
            cam_power_on();
            break;
        case MSG_POWER_ON_COMPLETE:
            assert(false);
            break;
        case MSG_POWER_OFF:
            cam_power_off();
            break;
        case MSG_POWER_OFF_COMPLETE:
            assert(false);
            break;
        case MSG_PREPARE_CAPTURE:
            cam_prepare_capture();
            break;
        case MSG_PREPARE_CAPTURE_COMPLETE:
            assert(false);
            break;
        case MSG_CAPTURE:
            cam_capture();
            break;
        case MSG_CAPTURE_COMPLETE:
            assert(false);
            break;
        default:
            assert(false);
            break;
        }

        SYSTIM systim2 = 0;
        get_tim(&systim2);
        syslog(LOG_NOTICE, "drvcam_task: msg = %d, dtime=%d", msg, systim2-systim1);
    }
}

/*
 * 内部関数
 */

/*
 * カメラ電源ON
 */
void cam_power_on()
{
    assert(s_callback_power_on);

    // 電源ON
#if 1	// TODO: 電源制御で制御する
    drvcmn_gpio_pin_set(&GPIO_PIN_1V8_ON, true);	// 1.8V_ON (Active H) => H
    drvcmn_gpio_pin_set(&GPIO_PIN_AVDD_ON, false);	// AVDD_ON (Active L) => L
#endif
//    drvcmn_gpio_pin_set(&GPIO_PIN_CAMERA_ON, false);	// CAM_ON (Active L) =>L

    // リセット
    drvcmn_gpio_pin_set(&GPIO_PIN_CAMERA_RESET, false);	// CAMERA用RESET (Active L) => L
    dly_tsk(10);
    drvcmn_gpio_pin_set(&GPIO_PIN_CAMERA_RESET, true);	// CAMERA用RESET (Active L) => Z

#if 0
    /* カメラレジスタ設定 */
    set_camera_params();
#endif

#ifdef DEBUG_USE_DUMP_CAMERA_REGS
    // 全レジスタダンプ
    dump_camera_regs();
#endif	// #ifdef DEBUG_USE_DUMP_CAMERA_REGS

    /* コールバック */
    s_callback_power_on(DRVCAM_CALLBACK_POWERON, 0);
    s_callback_power_on = NULL;

    return;
}

/*
 * カメラ電源OFF
 */
void cam_power_off()
{
    assert(s_callback_power_off);

//    drvcmn_gpio_pin_set(&GPIO_PIN_CAMERA_ON, true);	// // CAM_ON (Active L) =>Z

#if 1	// TODO: 電源制御で制御する
    drvcmn_gpio_pin_set(&GPIO_PIN_1V8_ON, false);	// 1.8V_ON (Active H) => L
    drvcmn_gpio_pin_set(&GPIO_PIN_AVDD_ON, true);	// AVDD_ON (Active L) => Z
#endif

    /* コールバック */
    s_callback_power_off(DRVCAM_CALLBACK_POWEROFF, 0);
    s_callback_power_off = NULL;

    return;
}

/*
 * 撮影準備
 */
void cam_prepare_capture()
{
    assert(s_callback_prepare_capture);
    
#if 1
    /* カメラレジスタ設定 */
    set_camera_params();
#endif

//#ifdef DEBUG_USE_DUMP_CAMERA_REGS
#if 0
    // 全レジスタダンプ
    dump_camera_regs();
#endif	// #ifdef DEBUG_USE_DUMP_CAMERA_REGS

    /* DCMI有効化(VSYNCを数えるため) */
    dcmi_enable();

    /* 最初のフレームを読み飛ばす */
    if (CAMERA_FRAME_SKIP > 0) {
        s_frame_skip = CAMERA_FRAME_SKIP;
        FLGPTN flgptn = 0;
        ER ercd = twai_flg(FLG_DRVCAM, FLGPTN_FRAME_SKIP, TWF_ANDW, &flgptn, 2000);	// 2000msタイムアウト
        assert(ercd == E_OK);
    }

    /* DCMI無効化 */
    dcmi_disable();

    /* コールバック */
    s_callback_prepare_capture(DRVCAM_CALLBACK_PREPARE, 0);
    s_callback_prepare_capture = NULL;

    return;
}

/*
 * 撮影
 */
void cam_capture()
{
    assert(s_callback_capture);
    assert(s_capture_output);

    /* DCMI 有効化 */
    dcmi_enable();

    /* DMA転送有効 */
    DRVCMN_DMA_XFER_SETTING_T setting = {
        .dir = DRVCMN_DMA_DIR_P2M,
        .src = TADR_DCMI_BASE + TOFF_DCMI_DR,
        .dest = (intptr_t)(s_capture_output),
        .nbytes = DMA_NBYTES,
        .nxfer = DMA_NXFER,
        .use_fifo = true,
        .isr = dma_isr_callback,
    };
    drvcmn_dma_transfer_enable(&DMA_STREAM, &setting);

    /* 撮影開始 */
    dcmi_capture_enable();

    /* DMA完了待ち */
    FLGPTN flgptn = 0;
    ER ercd = twai_flg(FLG_DRVCAM, FLGPTN_DMA_COMPLETE, TWF_ANDW, &flgptn, 1000);	// 1000msタイムアウト
    assert(ercd == E_OK);

    /* DCMI,DMA 停止 */
    dcmi_disable();
    drvcmn_dma_transfer_disable(&DMA_STREAM);

    /* コールバック */
    s_callback_capture(DRVCAM_CALLBACK_CAPTURE, 0);
    s_callback_capture = NULL;
    s_capture_output = NULL;
    s_capture_output_size = 0;

    return;
}

// DCMI 初期化
void dcmi_initialize()
{
    /* RCC */
    drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_AHB2ENR, 0, RCC_AHB2ENR_DCMIEN, RCC_AHB2ENR_DCMIEN);

    /* GPIO ピンの設定 */
    for (int i = 0; i < sizeof(GPIO_PINS_DCMI) / sizeof(GPIO_PINS_DCMI[0]); i++) {
        drvcmn_gpio_pin_initialize(&(GPIO_PINS_DCMI[i]), &GPIO_SETTING_DCMI);
    }

    /* DCMI_CR */
    drvcmn_setreg32(TADR_DCMI_BASE + TOFF_DCMI_CR, 18, 0x1, 0x1);	// [18]OEBS
    drvcmn_setreg32(TADR_DCMI_BASE + TOFF_DCMI_CR, 16, 0x3, 0x1);	// [17:16]BSM
    drvcmn_setreg32(TADR_DCMI_BASE + TOFF_DCMI_CR, 10, 0x3, 0x0);	// [11:10]EDM
    drvcmn_setreg32(TADR_DCMI_BASE + TOFF_DCMI_CR, 8, 0x3, 0x0);	// [9:8]FCRC
    drvcmn_setreg32(TADR_DCMI_BASE + TOFF_DCMI_CR, 7, 0x1, 0x0);	// [7]VSPOL
    drvcmn_setreg32(TADR_DCMI_BASE + TOFF_DCMI_CR, 6, 0x1, 0x0);	// [6]HSPOL
    drvcmn_setreg32(TADR_DCMI_BASE + TOFF_DCMI_CR, 5, 0x1, 0x1);	// [5]PCKPOL

    drvcmn_setreg32(TADR_DCMI_BASE + TOFF_DCMI_CR, 4, 0x1, 0x0);	// [4]ESS
    drvcmn_setreg32(TADR_DCMI_BASE + TOFF_DCMI_CR, 1, 0x1, 0x1);	// [1]CM

    return;
}

void dcmi_enable()
{
    /* DCMI_IER */
    drvcmn_setreg32(TADR_DCMI_BASE + TOFF_DCMI_IER, 3, 0x1, 0x1);	// [3]VSYNC_IE

    /* DCMI_CR */
    drvcmn_setreg32(TADR_DCMI_BASE + TOFF_DCMI_CR, 14, 0x1, 0x1);	// [14]ENABLE
}

void dcmi_disable()
{
    /* DCMI_IER */
    drvcmn_setreg32(TADR_DCMI_BASE + TOFF_DCMI_IER, 3, 0x1, 0x0);	// [3]VSYNC_IE

    /* DCMI_CR */
    drvcmn_setreg32(TADR_DCMI_BASE + TOFF_DCMI_CR, 14, 0x1, 0x0);	// [14]ENABLE
    while (drvcmn_getreg32(TADR_DCMI_BASE + TOFF_DCMI_CR, 14, 0x1) != 0);
}

void dcmi_capture_enable()
{
    /* DCMI_CR */
    drvcmn_setreg32(TADR_DCMI_BASE + TOFF_DCMI_CR, 0, 0x1, 0x1); // [0]CAPTURE
}

void dcmi_capture_disable()
{
    /* DCMI_CR */
    drvcmn_setreg32(TADR_DCMI_BASE + TOFF_DCMI_CR, 0, 0x1, 0x0); // [0]CAPTURE
}

/*
 * 割込みサービスルーチン
 */
void drvcam_dcmi_isr(intptr_t exinf)
{
    uint32_t icr = 0;
    uint32_t mis = drvcmn_getreg32(TADR_DCMI_BASE + TOFF_DCMI_RISR, 0, 0x1F);

    //syslog(LOG_NOTICE, "drvcam_dcmi_isr mis:%0x", mis);

    if (mis & (0x1 << 3)) {	// VSYNC_RIS
        icr = (0x1 << 3);
        if (s_frame_skip > 0) {
            --s_frame_skip;
            if (s_frame_skip == 0) {
                s_frame_skip = -1;
                ER er = iset_flg(FLG_DRVCAM, FLGPTN_FRAME_SKIP);
                assert(er == E_OK);
            }
        }
    } else {
        assert(false);
        icr = mis;
    }

    // 割込みクリア
    drvcmn_setreg32(TADR_DCMI_BASE + TOFF_DCMI_ICR, 0, 0x1F, icr);
}

/*
 * DMAコールバック(ISRコンテキスト)
 */
void dma_isr_callback()
{
    syslog(LOG_NOTICE, "drvcam_dma_isr_callback");

    ER er = iset_flg(FLG_DRVCAM, FLGPTN_DMA_COMPLETE);
    assert(er == E_OK);
    //SVC_PERROR(er);
}

/*
 * カメラレジスタの設定(I2C)
 */
int32_t set_camera_params()
{
    syslog(LOG_NOTICE, "set_camera_params [S]");

    syslog(LOG_NOTICE, "set_camera_params [1]");

    for (int i = 0; i < DRVCAM_NUM_REGVAL_DEFAULT; i++) {
        i2c_write(DRVCAM_REGVAL_DEFAULT[i].addr, DRVCAM_REGVAL_DEFAULT[i].value);
        dly_tsk(20);
    }

    syslog(LOG_NOTICE, "set_camera_params [E]");
    return 0;
}

/*
 * カメラI2C初期化
 */
int32_t i2c_initialize()
{
    /* I2C Tx */
    drvcmn_gpio_pin_initialize(&(GPIO_PINS_I2C[0]), &GPIO_SETTING_I2C);

    /* IC2 Rx */
    drvcmn_gpio_pin_initialize(&(GPIO_PINS_I2C[1]), &GPIO_SETTING_I2C);

    DRVCMN_I2C_SETTING_T setting;
    drvcmn_i2c_initialize(CAMERA_I2C, &setting);

    return 0;
}

/*
 * カメラI2Cの書き込み
 */
int32_t i2c_write(uint8_t reg_addr, uint16_t data)
{
    //syslog(LOG_NOTICE, "i2c_write(dev:0x%08x, reg:0x%08x, data:0x%08x)", dev_addr, reg_addr, data);

    /* BUSY ビットがクリアされるまで待つ */
    drvcmn_i2c_wait_until_busy(1);

    uint8_t send_data[3];
    drvcmn_i2c_set_start(CAMERA_I2C, CAMERA_I2C_SADD, 3, false);
    send_data[0] = reg_addr;	/* レジスタアドレス */
    send_data[1] = ((uint8_t*)&data)[1];		/* レジスタ値 */
    send_data[2] = ((uint8_t*)&data)[0];		/* レジスタ値 */
    drvcmn_i2c_write(CAMERA_I2C, send_data, 3);

    /* 終了 */
    drvcmn_i2c_wait_stop(CAMERA_I2C, CAMERA_I2C_SADD);

    return 0;
}

/*
 * カメラI2Cの読み取り
 */
int32_t i2c_read(uint16_t *odata, uint8_t reg_addr)
{
    assert(odata);
    assert(!(reg_addr & ~0xFF));

    // BUSY がクリアされるのを待つ
    drvcmn_i2c_wait_until_busy(CAMERA_I2C);

    // レジスタアドレス(1byte)送信
    drvcmn_i2c_set_start(CAMERA_I2C, CAMERA_I2C_SADD, 1, false);

    uint8_t data8 = reg_addr;
    drvcmn_i2c_write(CAMERA_I2C, &data8, 1);

    // レジスタの値(2byte)受信
    drvcmn_i2c_set_start(CAMERA_I2C, CAMERA_I2C_SADD, 2, true);
    uint8_t buf[2] = {0};
    drvcmn_i2c_read(buf, CAMERA_I2C, 2);
    ((uint8_t*)odata)[0] = buf[1];
    ((uint8_t*)odata)[1] = buf[0];
    // 停止
    drvcmn_i2c_wait_stop(CAMERA_I2C, CAMERA_I2C_SADD);

    return 0;
}

// レジスタダンプ
void dump_camera_regs()
{
    const uint8_t (* regs)[2] = DRVCAM_DEBUG_DUMP_REGS;
    const size_t nregs = DRVCAM_NUM_DEBUG_DUMP_REGS;

    for (int i = 0; i < nregs; i++) {
        i2c_write(0xf0, regs[i][0]);	// ページ切り替え

        uint16_t regval = 0;
        i2c_read(&regval, regs[i][1]);
        syslog(LOG_NOTICE, "p%d:0x%02x=0x%04x", regs[i][0], regs[i][1], regval);
        dly_tsk(10);
    }
}
