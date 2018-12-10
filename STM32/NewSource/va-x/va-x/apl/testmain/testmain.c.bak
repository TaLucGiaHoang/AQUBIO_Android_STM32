/*
 * VA-X
 *
 * Copyright (C) 2017 Bionics co.,ltd.
 *
 * 変更:
 * - 2017/10/13 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "testmain.h"

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
#include "stm32f7xx.h"
#include "chip_serial.h"

#include "drvled.h"
#include "drvpbt.h"
#include "drvts.h"
#include "drvirled.h"
#include "drvcam.h"
#include "drvcmn_gpio.h"
#include "drvcmn.h"
#include "drvbuz.h"
#include "drvcmn_uart.h"
#include "drvbvol.h"
#include "drvble.h"
#include "mdlble.h"
#include "drvicc.h"

// サービスコールのエラーのログ出力
#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))
inline void svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}

// レジスタダンプ
#if 1
  #define DUMP_REG32(name, addr) syslog(LOG_NOTICE, name " (0x%08p): => 0x%08x", addr, drvcmn_getreg32(addr, 0, UINT32_MAX));
#else
  #define DUMP_REG32(name, addr)
#endif

//
static const FLGPTN FLGPTN_DRVCAM_CALLBACK =	(0x1 << 0);
static const FLGPTN FLGPTN_UART_TXCOMPLETE =	(0x1 << 2);
static const FLGPTN FLGPTN_UART_RXCOMPLETE =	(0x1 << 3);

static const FLGPTN FLGPTN_DRVBLE_FACTORY_RESET_COMPLETE =	(0x1 << 4);
static const FLGPTN FLGPTN_DRVBLE_GET_READY_COMPLETE =		(0x1 << 5);
static const FLGPTN FLGPTN_DRVBLE_POWER_OFF_COMPLETE =		(0x1 << 6);
static const FLGPTN FLGPTN_DRVBLE_SEND_COMPLETE =			(0x1 << 7);
static const FLGPTN FLGPTN_DRVBLE_RECEIVE_COMPLETE =		(0x1 << 8);

static const FLGPTN FLGPTN_MDLBLE_INITIALIZE_COMPLETE =		(0x1 << 9);
static const FLGPTN FLGPTN_MDLBLE_START_COMPLETE =		(0x1 << 10);
static const FLGPTN FLGPTN_MDLBLE_DATA_RECEIVED =		(0x1 << 11);

//static const FLGPTN FLGPTN_ =		(0x1 << );

// GPIOピン(P_ON)
static const DRVCMN_GPIO_PIN_T POWER_GPIO_PIN = {
    DRVCMN_GPIO_PORT_E, 15,
};

// GPIOピン(LED)
static const DRVCMN_GPIO_PIN_T LED0_GPIO_PIN[] = {
    { DRVCMN_GPIO_PORT_C, 0,},
    { DRVCMN_GPIO_PORT_C, 1,},
    { DRVCMN_GPIO_PORT_C, 2,},
    { DRVCMN_GPIO_PORT_C, 3,},
    { DRVCMN_GPIO_PORT_B, 6,},
};

// GPIOピン(REC_ON)
static const DRVCMN_GPIO_PIN_T REC_ON_GPIO_PIN = {
    DRVCMN_GPIO_PORT_D, 7,
};

// GPIOピン設定(PON)
static const DRVCMN_GPIO_SETTING_T POWER_GPIO_SETTING = {
    .mode = DRVCMN_GPIO_MODE_GPOUT,
    .otype = DRVCMN_GPIO_OTYPE_PP,
    .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
    .pupd = DRVCMN_GPIO_PUPD_PULLDOWN,
};

/*
 * 内部変数
 */
//__attribute__ ((section (".largebuf"))) static uint8_t s_camera_buf[1280*(240+1)];
//static uint8_t s_camera_buf[1024 * 128];

// 
static void test_drvcam();
static void drvcam_callback(int32_t type, int32_t error);
static void hexdump(const uint8_t* buf, size_t nbytes, const char* filename);
static void setup_backup_domain();
static void test_uart();
static void test_mco();

static void test_bvol();
static void bvol_callback(uint32_t voltage);

static bool_t uart_isr_callback(int type, uint32_t opt);
static void uart_send(const uint8_t* data, size_t size);
static void uart_receive(uint8_t* data, size_t size);

static void test_ts();

static void test_drvble();
static void drvble_callback(int32_t evt, int32_t error, intptr_t opt);

static void test_mdlble();
static void mdlble_callback(int event, intptr_t opt1, intptr_t opt2);

static void test_drvicc();
static void drvicc_callback(int event, intptr_t opt1, intptr_t opt2);

extern uint32_t crc32(uint32_t crc, const void *buf, size_t size);

/* コールバック関数(ボタン) */
void pbt_callback(uint32_t button, int32_t evt)
{
    snd_dtq(DTQ_TESTMAIN, (0x1 << 16) | evt);
}

/* コールバック関数(生体センサー) */
void ts_callback(int32_t evt)
{
    snd_dtq(DTQ_TESTMAIN, (0x2 << 16) | evt);
}

/*
 *  メインタスク
 */
void testmain_task(intptr_t exinf)
{
#if 1
    drvcmn_gpio_pin_initialize(&POWER_GPIO_PIN, &POWER_GPIO_SETTING);
    drvcmn_gpio_pin_set(&POWER_GPIO_PIN, true);
    //drvcmn_gpio_pin_set(&LED0_GPIO_PIN, true);

    // LEDを点灯
    for (int i = 0; i < sizeof(LED0_GPIO_PIN) / sizeof(LED0_GPIO_PIN[0]); i++) {
        drvcmn_gpio_pin_initialize(&(LED0_GPIO_PIN[i]), &POWER_GPIO_SETTING);
        drvcmn_gpio_pin_set(&(LED0_GPIO_PIN[i]), true);
    }
#endif

    setup_backup_domain();

    drvbuz_initialize(0);
#if 0
    drvbuz_start();
    dly_tsk(50);
    drvbuz_stop();
#endif

    //drvcam_initialize();
    //drvts_initialize();

	syslog(LOG_NOTICE, "testmain_task() starts (exinf = %d).", (int_t) exinf);

    //drvled_initialize(0);
    //drvpbt_initialize(pbt_callback);
    //dly_tsk(500);

    //for (int i = 0;; i++) ;
    //drvts_start(ts_callback);

    //drvled_set_state(0, true);

    //drvirled_initialize();
    //
    //dly_tsk(1000);
    //drvirled_set_state(true);
    //
    //dly_tsk(1000);
    //drvirled_set_duty(1, 100);
    //
    //dly_tsk(1000);
    //drvirled_set_duty(1, 50);
    //
    //dly_tsk(1000);
    //drvirled_set_duty(1, 20);
    //
    //dly_tsk(1000);
    //drvirled_set_state(false);

    //drvcam_initialize(0);

    //dly_tsk(1000);
    //test_drvcam();

    //test_uart();
	//test_drvble();
    test_mdlble();

    //test_mco();

    //test_bvol();

    //test_ts();

    //test_drvicc();

    while (true) {
        int32_t msg = 0;
        ER er_rcv = rcv_dtq(DTQ_TESTMAIN, (intptr_t*)&msg);
        assert((er_rcv == E_OK) || (er_rcv == E_TMOUT));
        int32_t msgh = (msg & 0xffff0000) >> 16;
        int32_t msgl = msg & 0x0000ffff;
        syslog(LOG_NOTICE, "testmain_task() received: (msg = (%d, %d)).", msgh, msgl);

        switch (msgh) {
        case 1:
            switch (msgl) {
            case DRVPBT_EVT_PRESS:
                drvled_set_state(0, true);
                break;
            case DRVPBT_EVT_RELEASE:
                drvled_set_state(0, false);
                break;
            default:
                assert(false);
                break;
            }
            break;
        case 2:
            break;
        }
    }
}

void test_drvcam()
{
#if 0
    ER ercd = 0;
    FLGPTN flgptn = 0;

    SYSTIM systim1;
    SYSTIM systim2;

    get_tim(&systim1);
    syslog(LOG_NOTICE, "initialize");
    drvcam_initialize();

    get_tim(&systim2);
    syslog(LOG_NOTICE, "initialize ok (%dms)", systim2 - systim1);

    get_tim(&systim1);
    syslog(LOG_NOTICE, "power_on");
    drvcam_power_on(drvcam_callback);

    ercd = twai_flg(FLG_TESTMAIN, FLGPTN_DRVCAM_CALLBACK, TWF_ANDW, &flgptn, 10000);
    assert(ercd == E_OK);
    get_tim(&systim2);
    syslog(LOG_NOTICE, "power_on ok (%dms)", systim2 - systim1);

    get_tim(&systim1);
    syslog(LOG_NOTICE, "prepare_capture");
    static DRVCAM_CAPTURE_PARAM_T capture_param = {
        .output = s_camera_buf,		// 出力先アドレス
        .size = 1280,				// 出力先領域サイズ
        .count = 240,
        .exposure = 0,				// 露出設定
        .regval = NULL,				// カメラレジスタ設定値(デバッグ用)
        .num_regval = 0,			// カメラレジスタ設定値の数
    };
    drvcam_prepare_capture(drvcam_callback, &capture_param);
    ercd = twai_flg(FLG_TESTMAIN, FLGPTN_DRVCAM_CALLBACK, TWF_ANDW, &flgptn, 10000);
    assert(ercd == E_OK);
    
    SVC_PERROR(ercd);
    get_tim(&systim2);
    syslog(LOG_NOTICE, "prepare_capture ok (%dms)", systim2 - systim1);

#if 1
    get_tim(&systim1);
    syslog(LOG_NOTICE, "capture");
    drvcam_capture(drvcam_callback);

    ercd = twai_flg(FLG_TESTMAIN, FLGPTN_DRVCAM_CALLBACK, TWF_ANDW, &flgptn, 1000);
    assert(ercd == E_OK);
    get_tim(&systim2);
    syslog(LOG_NOTICE, "capture ok (%dms)", systim2 - systim1);

    get_tim(&systim1);
    syslog(LOG_NOTICE, "power_off");
    drvcam_power_off(drvcam_callback);

    ercd = twai_flg(FLG_TESTMAIN, FLGPTN_DRVCAM_CALLBACK, TWF_ANDW, &flgptn, 1000);
    assert(ercd == E_OK);
    get_tim(&systim2);
    syslog(LOG_NOTICE, "power_off ok (%dms)", systim2 - systim1);

    drvbuz_start();
    dly_tsk(50);
    drvbuz_stop();

    syslog(LOG_NOTICE, "dump");
    dly_tsk(1000);
    hexdump(s_camera_buf, 307200, "camera_y8_640x480.bin");
#endif    
#endif
}

void drvcam_callback(int32_t type, int32_t error)
{
    syslog(LOG_NOTICE, "drvcam_callback");

    switch (type) {
    case DRVCAM_CALLBACK_POWERON:
        set_flg(FLG_TESTMAIN, FLGPTN_DRVCAM_CALLBACK);
        break;
    case DRVCAM_CALLBACK_POWEROFF:
        set_flg(FLG_TESTMAIN, FLGPTN_DRVCAM_CALLBACK);
        break;
    case DRVCAM_CALLBACK_PREPARE:
        set_flg(FLG_TESTMAIN, FLGPTN_DRVCAM_CALLBACK);
        break;
    case DRVCAM_CALLBACK_CAPTURE:
        set_flg(FLG_TESTMAIN, FLGPTN_DRVCAM_CALLBACK);
        break;
    default:
        assert(false);
        break;
    }
} 

// 16進ダンプ
void hexdump(const uint8_t* buf, size_t nbytes, const char* filename)
{
    const static char HEX_TABLE[] = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f',
    };
        
    syslog(LOG_NOTICE, "==== DUMP START \"%s\"====", filename);
    dly_tsk(50);

    static char linebuf[128];

    int bytes = 0;
    for (;;) {
        char hex[2];
        int linepos = 0;
        int i = 0;
        for (;;) {
            hex[0] = HEX_TABLE[(buf[bytes + i] & 0xF0) >> 4];
            hex[1] = HEX_TABLE[(buf[bytes + i] & 0x0F)];

            linebuf[linepos + 0] = hex[0];
            linebuf[linepos + 1] = hex[1];
            //linebuf[linepos + 2] = ' ';
            linepos += 2;

            ++i;
            if ((bytes + i >= nbytes) || (i >= 32)) {
                bytes += i;
                break;
            }
        }
        linebuf[linepos] = '\0';
        
#if 0	// syslog で出力
        syslog(LOG_NOTICE, linebuf);
#else	// シリアルに直接出力
        for (int i = 0;; i++) {
            if (linebuf[i] == '\0') {
                sio_pol_snd_chr('\r', SIO_PORTID);
                sio_pol_snd_chr('\n', SIO_PORTID);
                break;
            }
            sio_pol_snd_chr(linebuf[i], SIO_PORTID);
        }
#endif

        if (bytes >= nbytes) {
            break;
        }
    }

    dly_tsk(50);
    uint32_t crc = 0;
    crc = crc32(0, buf, nbytes);
    syslog(LOG_NOTICE, "==== DUMP END [%08x] ====", crc);
}

void setup_backup_domain()
{

    drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_APB1ENR, 0, RCC_APB1ENR_PWREN, RCC_APB1ENR_PWREN);	// []
    DUMP_REG32("RCC_APB1ENR", TADR_RCC_BASE + TOFF_RCC_APB1ENR);

    DUMP_REG32("PWR_CR1", TADR_PWR_BASE + TOFF_PWR_CR1);
    drvcmn_setreg32(TADR_PWR_BASE + TOFF_PWR_CR1, 0, PWR_CR1_DBP, PWR_CR1_DBP);	// []
    DUMP_REG32("PWR_CR1", TADR_PWR_BASE + TOFF_PWR_CR1);

    DUMP_REG32("RCC_BDCR (before)", TADR_RCC_BASE + TOFF_RCC_BDCR);

    uint32_t bdcr = drvcmn_getreg32(TADR_RCC_BASE + TOFF_RCC_BDCR, 0, RCC_BDCR_RTCSEL | RCC_BDCR_LSEON | RCC_BDCR_RTCEN);
    if (bdcr != (RCC_BDCR_RTCSEL_0 | RCC_BDCR_LSEON | RCC_BDCR_RTCEN)) {

        drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_BDCR, 0, RCC_BDCR_BDRST, RCC_BDCR_BDRST);	// [16]BDRST
        DUMP_REG32("RCC_BDCR", TADR_RCC_BASE + TOFF_RCC_BDCR);
        drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_BDCR, 0, RCC_BDCR_BDRST, 0);	// [16]BDRST
        DUMP_REG32("RCC_BDCR", TADR_RCC_BASE + TOFF_RCC_BDCR);

        drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_BDCR, 0, RCC_BDCR_RTCSEL, RCC_BDCR_RTCSEL_0);
        DUMP_REG32("RCC_BDCR", TADR_RCC_BASE + TOFF_RCC_BDCR);

        drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_BDCR, 0, RCC_BDCR_LSEON, RCC_BDCR_LSEON);
        DUMP_REG32("RCC_BDCR", TADR_RCC_BASE + TOFF_RCC_BDCR);

        drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_BDCR, 0, RCC_BDCR_RTCEN, RCC_BDCR_RTCEN);
        DUMP_REG32("RCC_BDCR", TADR_RCC_BASE + TOFF_RCC_BDCR);
    }

#if 0
    while (true) {
        DUMP_REG32("RTC_TR", TADR_RTC_BASE + TOFF_RTC_TR);
        dly_tsk(500);
    }
#endif
}

void test_uart()
{
    // GPIOピン()
    static const DRVCMN_GPIO_PIN_T UART_GPIO_PINS[] = {
        { DRVCMN_GPIO_PORT_B, 14 },
        { DRVCMN_GPIO_PORT_B, 15 },
        { DRVCMN_GPIO_PORT_D, 0  },
        { DRVCMN_GPIO_PORT_D, 1  },
    };

    static const DRVCMN_GPIO_SETTING_T UART_GPIO_SETTING = {
        .mode = DRVCMN_GPIO_MODE_AF,
        .otype = DRVCMN_GPIO_OTYPE_PP,
        .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
        .pupd = DRVCMN_GPIO_PUPD_PULLUP,
        .afno = 8,
    };

    for (int i = 0; i < sizeof(UART_GPIO_PINS) / sizeof(UART_GPIO_PINS[0]); i++) {
        drvcmn_gpio_pin_initialize(&(UART_GPIO_PINS[i]), &UART_GPIO_SETTING);
    }

    static const struct {
        DRVCMN_GPIO_PIN_T pin;
        int od;
    } BLE_GPIO_PINS[] = {
        {{ DRVCMN_GPIO_PORT_C, 11 }, 0},	// [0]RST
        {{ DRVCMN_GPIO_PORT_C, 12 }, 1},	// [1]WAKEUP
        {{ DRVCMN_GPIO_PORT_D, 14 }, 0},	// [2]BLE_ON
        {{ DRVCMN_GPIO_PORT_D, 15 }, 1},	// [3]MODE_BLE
        {{ DRVCMN_GPIO_PORT_D, 2  }, 1},	// [4]INIT
    };

    // GPIOピン設定(PON)
    static const DRVCMN_GPIO_SETTING_T BLE_GPIO_SETTING_OD = {
        .mode = DRVCMN_GPIO_MODE_GPOUT,
        .otype = DRVCMN_GPIO_OTYPE_OD,
        .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
        .pupd = DRVCMN_GPIO_PUPD_NONE,
    };

    static const DRVCMN_GPIO_SETTING_T BLE_GPIO_SETTING_PP = {
        .mode = DRVCMN_GPIO_MODE_GPOUT,
        .otype = DRVCMN_GPIO_OTYPE_PP,
        .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
        .pupd = DRVCMN_GPIO_PUPD_PULLDOWN,
    };

    for (int i = 0; i < sizeof(BLE_GPIO_PINS) / sizeof(BLE_GPIO_PINS[0]); i++) {
        drvcmn_gpio_pin_initialize(&(BLE_GPIO_PINS[i].pin), (BLE_GPIO_PINS[i].od ? &BLE_GPIO_SETTING_OD : &BLE_GPIO_SETTING_PP));
        drvcmn_gpio_pin_set(&(BLE_GPIO_PINS[i].pin), false);
    }
#if 1
    drvcmn_gpio_pin_set(&(BLE_GPIO_PINS[1].pin), true);		// WAKEUP => Z
    drvcmn_gpio_pin_set(&(BLE_GPIO_PINS[3].pin), false);	// MODE => L
    drvcmn_gpio_pin_set(&(BLE_GPIO_PINS[4].pin), false);		// INIT => L
    //drvcmn_gpio_pin_set(&(BLE_GPIO_PINS[0].pin), true);		// RESET => Z (リセット解除)

    drvcmn_gpio_pin_set(&(BLE_GPIO_PINS[2].pin), true);		// BLE_ON => Z
    dly_tsk(1);
    drvcmn_gpio_pin_set(&(BLE_GPIO_PINS[0].pin), true);		// RESET => H (リセット解除)
#else
    for (int i = 0; i < sizeof(BLE_GPIO_PINS) / sizeof(BLE_GPIO_PINS[0]); i++) {
        drvcmn_gpio_pin_set(&(BLE_GPIO_PINS[i]), false);
    }
//    drvcmn_gpio_pin_set(&(BLE_GPIO_PINS[2]), true);	// MODE => L
    drvcmn_gpio_pin_set(&(BLE_GPIO_PINS[0]), true);	// MODE => L
    drvcmn_gpio_pin_set(&(BLE_GPIO_PINS[1]), true);	// MODE => L
#endif
    // UART4
    static const DRVCMN_UART_SETTING_T UART_SETTING = {
        .clocksel = DRVCMN_UART_CLOCKSEL_SYS,
        .baudrate = 38400,
        .wordlen = DRVCMN_UART_WORD_8BIT,
        .parity = DRVCMN_UART_PARITY_NONE,
        .stop = DRVCMN_UART_STOP_1,
        .hwflow = true,
        .callback = uart_isr_callback,
    };
    drvcmn_uart_initialize(4, &UART_SETTING);

    //dly_tsk(500);
    //drvcmn_gpio_pin_set(&(BLE_GPIO_PINS[0]), true);		// RESET => Z (リセット解除)

    static uint8_t rx_buf[32];

    // Receive: OK
    uart_receive(rx_buf, 6);
    for (int i = 0; i < 6; i++) {
        syslog(LOG_NOTICE, "receive 0x%02x '%c'", rx_buf[i], rx_buf[i]);
    }

    dly_tsk(100);

    // Send: AT+DBI=ALL
    {
        const uint8_t command[] = "AT+DBI=ALL\r";
        uart_send(command, sizeof(command) - 1);
        for (int i = 0; i < sizeof(command) - 1; i++) {
            syslog(LOG_NOTICE, "send 0x%02x '%c'", command[i], command[i]);
        }
    }

    // Receive: OK
    uart_receive(rx_buf, 6);
    for (int i = 0; i < 6; i++) {
        syslog(LOG_NOTICE, "receive 0x%02x '%c'", rx_buf[i], rx_buf[i]);
    }

    // Send: AT+SBO
    {
        const uint8_t command[] = "AT+SBO\r";
        uart_send(command, sizeof(command) - 1);
        for (int i = 0; i < sizeof(command) - 1; i++) {
            syslog(LOG_NOTICE, "send 0x%02x '%c'", command[i], command[i]);
        }
    }

    // Receive: ACK
    uart_receive(rx_buf, 7);
    for (int i = 0; i < 7; i++) {
        syslog(LOG_NOTICE, "receive 0x%02x '%c'", rx_buf[i], rx_buf[i]);
    }

    // Receive: HELLO
    uart_receive(rx_buf, 5);
    for (int i = 0; i < 5; i++) {
        syslog(LOG_NOTICE, "receive 0x%02x '%c'", rx_buf[i], rx_buf[i]);
    }

    // Send: 10000bytes
    {
        SYSTIM systim1;
        get_tim(&systim1);

        uint32_t crc = 0;
        uint8_t data = 0;
        const size_t SEND_SIZE = 10 * 1000;
        for (int i = 0; i < SEND_SIZE; i++) {
            data = i;
            uart_send(&data, 1);
            syslog(LOG_NOTICE, "send 0x%02x", data);
            crc = crc32(crc, &data, 1);
        }
        SYSTIM systim2;
        get_tim(&systim2);
        syslog(LOG_NOTICE, "send %dbytes in %dms, sum: 0x%08x", SEND_SIZE, systim2 - systim1, crc);
    }

    // Receive: 
    for (;;) {
        uart_receive(rx_buf, 1);
        syslog(LOG_NOTICE, "receive 0x%02x '%c'", rx_buf[0], rx_buf[0]);
    }

}

void test_mco()
{
    DUMP_REG32("RCC_CR", TADR_RCC_BASE + TOFF_RCC_CR);
    DUMP_REG32("RCC_PLLCFGR", TADR_RCC_BASE + TOFF_RCC_PLLCFGR);
    while (drvcmn_getreg32(TADR_RCC_BASE + TOFF_RCC_CR, 17, 0x1) == 0) {
        DUMP_REG32("RCC_CR", TADR_RCC_BASE + TOFF_RCC_CR);
    }

    //drvcmn_getreg32(TADR_RCC_BASE + TOFF_RCC_CR, 17, 0x1)

    // RCC_CFGR
//    drvcmn_setreg32(TADR_RCC_BASE + TOFF_RCC_CFGR, 30, 0x3, 0x2);
    DUMP_REG32("RCC_CFGR", TADR_RCC_BASE + TOFF_RCC_CFGR);
    DUMP_REG32("RCC_SSCGR", TADR_RCC_BASE + TOFF_RCC_SSCGR);

    // GPIOピン()
    static const DRVCMN_GPIO_PIN_T MCO_GPIO_PINS[] = {
        { DRVCMN_GPIO_PORT_C, 9 },
        { DRVCMN_GPIO_PORT_A, 8 },
    };

    static const DRVCMN_GPIO_SETTING_T MCO_GPIO_SETTING = {
        .mode = DRVCMN_GPIO_MODE_AF,
        .otype = DRVCMN_GPIO_OTYPE_PP,
        .ospeed = DRVCMN_GPIO_OSPEED_HIGH,
        .pupd = DRVCMN_GPIO_PUPD_NONE,
        .afno = 0,
    };

    for (int i = 0; i < sizeof(MCO_GPIO_PINS) / sizeof(MCO_GPIO_PINS[0]); i++) {
        drvcmn_gpio_pin_initialize(&(MCO_GPIO_PINS[i]), &MCO_GPIO_SETTING);
    }

}

void test_bvol()
{
    drvbvol_initialize();

    while (true) {
        drvbvol_get_voltage(bvol_callback);
        dly_tsk(1000);
    }

}

void bvol_callback(uint32_t voltage_mv)
{
    syslog(LOG_NOTICE, "Battery Voltage: %d mv", voltage_mv);
}

bool_t uart_isr_callback(int type, uint32_t opt)
{
    switch (type) {
    case DRVCMN_UART_TXCOMPLETE:
        iset_flg(FLG_TESTMAIN, FLGPTN_UART_TXCOMPLETE);
        break;
    case DRVCMN_UART_RXCOMPLETE:
        iset_flg(FLG_TESTMAIN, FLGPTN_UART_RXCOMPLETE);
        break;
    default:
        assert(false);
        break;
    }

    return false;
}

void uart_send(const uint8_t* data, size_t size)
{
    drvcmn_uart_send(4, data, size);

    FLGPTN flgptn = 0;
    ER ercd = twai_flg(FLG_TESTMAIN, FLGPTN_UART_TXCOMPLETE, TWF_ANDW, &flgptn, -1);
    assert(ercd == E_OK);
}

void uart_receive(uint8_t* data, size_t size)
{
    drvcmn_uart_receive(4, data, size);

    FLGPTN flgptn = 0;
    ER ercd = twai_flg(FLG_TESTMAIN, FLGPTN_UART_RXCOMPLETE, TWF_ANDW, &flgptn, -1);
    assert(ercd == E_OK);
}

void test_ts()
{
    //drvts_initialize();

    drvts_start(ts_callback);
}

void test_drvble()
{
#if 0
    FLGPTN flgptn = 0;
    ER er = 0;
  
    drvble_initialize();

    syslog(LOG_NOTICE, "call: drvble_factory_reset()");
    drvble_factory_reset(drvble_callback);
    twai_flg(FLG_TESTMAIN, FLGPTN_DRVBLE_FACTORY_RESET_COMPLETE, TWF_ANDW, &flgptn, -1);
    assert(er == E_OK);

    syslog(LOG_NOTICE, "call: drvble_get_ready()");
    drvble_get_ready(drvble_callback);
    twai_flg(FLG_TESTMAIN, FLGPTN_DRVBLE_GET_READY_COMPLETE, TWF_ANDW, &flgptn, -1);
    assert(er == E_OK);

    uint8_t receive_buf[32];
    syslog(LOG_NOTICE, "call: drvble_receive()");
    drvble_receive(receive_buf, 4, -1, drvble_callback);
    twai_flg(FLG_TESTMAIN, FLGPTN_DRVBLE_RECEIVE_COMPLETE, TWF_ANDW, &flgptn, -1);
    assert(er == E_OK);

    syslog(LOG_NOTICE, "call: drvble_send()");
    drvble_send((const uint8_t*)"hello", 5, -1, drvble_callback);
    twai_flg(FLG_TESTMAIN, FLGPTN_DRVBLE_SEND_COMPLETE, TWF_ANDW, &flgptn, -1);
    assert(er == E_OK);

    syslog(LOG_NOTICE, "call: drvble_power_off()");
    drvble_power_off(drvble_callback);
    twai_flg(FLG_TESTMAIN, FLGPTN_DRVBLE_POWER_OFF_COMPLETE, TWF_ANDW, &flgptn, -1);
    assert(er == E_OK);
#endif
}

void drvble_callback(int32_t evt, int32_t error, intptr_t opt)
{
    syslog(LOG_NOTICE, "drvble_callback(%d, %d, %d)", evt, error, opt);

    switch (evt) {
    case DRVBLE_EVT_FACTORY_RESET_COMPLETE:
        set_flg(FLG_TESTMAIN, FLGPTN_DRVBLE_FACTORY_RESET_COMPLETE);
        break;
    case DRVBLE_EVT_GET_READY_COMPLETE:
        set_flg(FLG_TESTMAIN, FLGPTN_DRVBLE_GET_READY_COMPLETE);
        break;
    case DRVBLE_EVT_POWER_OFF_COMPLETE:
        set_flg(FLG_TESTMAIN, FLGPTN_DRVBLE_POWER_OFF_COMPLETE);
        break;
    case DRVBLE_EVT_SEND_COMPLETE:
        set_flg(FLG_TESTMAIN, FLGPTN_DRVBLE_SEND_COMPLETE);
        break;
    case DRVBLE_EVT_RECEIVE_COMPLETE:
        set_flg(FLG_TESTMAIN, FLGPTN_DRVBLE_RECEIVE_COMPLETE);
        break;
    default:
        break;
    }
}

void test_mdlble()
{
#if 0
    FLGPTN flgptn = 0;
    ER er = 0;

    syslog(LOG_NOTICE, "[CALL]drvble_initialize()");
    drvble_initialize();
    syslog(LOG_NOTICE, "[COMPLETE]drvble_initialize()");

#if 0
#if 1
    syslog(LOG_NOTICE, "[CALL]drvble_factory_reset()");
    drvble_factory_reset(drvble_callback);
    twai_flg(FLG_TESTMAIN, FLGPTN_DRVBLE_FACTORY_RESET_COMPLETE, TWF_ANDW, &flgptn, -1);
    assert(er == E_OK);
    syslog(LOG_NOTICE, "[COMPLETE]drvble_factory_reset()");
#endif

    // 初期化
    syslog(LOG_NOTICE, "[CALL]mdlble_initialize()");
    mdlble_initialize(mdlble_callback);
    er = twai_flg(FLG_TESTMAIN, FLGPTN_MDLBLE_INITIALIZE_COMPLETE, TWF_ANDW, &flgptn, -1);
    assert(er == E_OK);
    syslog(LOG_NOTICE, "[COMPLETE]mdlble_initialize()");

    // 開始
    syslog(LOG_NOTICE, "[CALL]mdlble_start()");
    mdlble_start();
    er = twai_flg(FLG_TESTMAIN, FLGPTN_MDLBLE_START_COMPLETE, TWF_ANDW, &flgptn, -1);
    assert(er == E_OK);
    syslog(LOG_NOTICE, "[COMPLETE]mdlble_start()");
    
    for (;;) {
        dly_tsk(1000);
    }
#endif
#endif
}

void mdlble_callback(int event, intptr_t opt1, intptr_t opt2)
{
    switch (event) {
    case MDLBLE_EVT_INITIALIZE_COMPLETE:
        set_flg(FLG_TESTMAIN, FLGPTN_MDLBLE_INITIALIZE_COMPLETE);
        syslog(LOG_NOTICE, "[CALLBACK]MDLBLE_EVT_INITIALIZE_COMPLETE");
        break;
    case MDLBLE_EVT_START_COMPLETE:
        set_flg(FLG_TESTMAIN, FLGPTN_MDLBLE_START_COMPLETE);
        syslog(LOG_NOTICE, "[CALLBACK]FLGPTN_MDLBLE_START_COMPLETE");
        break;
    case MDLBLE_EVT_CONNECTED:
        break;
    case MDLBLE_EVT_DISCONNECTED:
        break;
    case MDLBLE_EVT_VALIDATED:
        break;
    case MDLBLE_EVT_VALIDATION_FAILED:
        break;
    case MDLBLE_EVT_DATA_RECEIVED:
        set_flg(FLG_TESTMAIN, FLGPTN_MDLBLE_DATA_RECEIVED);
        syslog(LOG_NOTICE, "[CALLBACK]MDLBLE_EVT_DATA_RECEIVED");
        mdlble_return_buffer(opt1);
        break;
    case MDLBLE_EVT_SEND_COMPLETE:
        break;
    default:
        break;
    }
}

void test_drvicc()
{
    syslog(LOG_NOTICE, "[CALL]drvicc_initialize()");
    drvicc_initialize();
    syslog(LOG_NOTICE, "[COMPLETE]drvicc_initialize()");

    syslog(LOG_NOTICE, "[CALL]drvicc_check_uart()");
    drvicc_check_uart(drvicc_callback);
    syslog(LOG_NOTICE, "[COMPLETE]drvicc_check_uart()");
}

void drvicc_callback(int event, intptr_t opt1, intptr_t opt2)
{
    switch (event) {
    case DRVICC_EVT_CHECK_UART_COMPLETE:
        syslog(LOG_NOTICE, "[CALLBACK]DRVICC_EVT_CHECK_UART_COMPLETE");
        break;
#if 0
    case MDLBLE_EVT_INITIALIZE_COMPLETE:
        set_flg(FLG_TESTMAIN, FLGPTN_MDLBLE_INITIALIZE_COMPLETE);
        syslog(LOG_NOTICE, "[CALLBACK]MDLBLE_EVT_INITIALIZE_COMPLETE");
        break;
    case MDLBLE_EVT_START_COMPLETE:
        set_flg(FLG_TESTMAIN, FLGPTN_MDLBLE_START_COMPLETE);
        syslog(LOG_NOTICE, "[CALLBACK]FLGPTN_MDLBLE_START_COMPLETE");
        break;
    case MDLBLE_EVT_CONNECTED:
        break;
    case MDLBLE_EVT_DISCONNECTED:
        break;
    case MDLBLE_EVT_VALIDATED:
        break;
    case MDLBLE_EVT_VALIDATION_FAILED:
        break;
    case MDLBLE_EVT_DATA_RECEIVED:
        break;
    case MDLBLE_EVT_SEND_COMPLETE:
        break;
#endif
    default:
        break;
    }
}
