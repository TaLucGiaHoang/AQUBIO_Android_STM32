/*
 * VA-X デバッグ機能
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/02/23 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "cmndbg.h"

#include <string.h>

#include <kernel.h>
#include <kernel_cfg.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include <chip_serial.h>

/*
 * マクロ定義
 */

// サービスコールのエラーのログ出力
static inline void svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}
#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))

// バイトオーダー反転(16bit)
static inline uint16_t swap16(uint16_t input)
{
    return ((input >> 8) & 0x00ff) | ((input << 8) & 0xff00);
}

/*
 * 型定義
 */


/*
 * 内部関数プロトタイプ
 */
static void send_serial_raw(const void* data, size_t size);
static uint8_t sum_data_xor(uint8_t init, const void* adata, size_t size);

/*
 * 定数定義
 */


/*
 * 内部変数
 */


/*********************************************************************
 * 公開関数
 ********************************************************************/

extern uint32_t crc32(uint32_t crc, const void *buf, size_t size);

// 16進ダンプ
void cmndbg_hexdump(const void* abuf, size_t nbytes, const char* filename)
{
    const uint8_t* buf = (const uint8_t*)abuf;

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

    uint32_t crc = 0;
    crc = crc32(0, buf, nbytes);
    syslog(LOG_NOTICE, "==== DUMP END [%08x] ====", crc);
    dly_tsk(50);
}

// バイナリ形式のフレームを出力する
void cmndbg_binframe(const void* abuf, uint16_t type, uint16_t sizex, uint16_t sizey)
{
    const uint8_t* buf = (const uint8_t*)abuf;

    uint8_t txbuf[32];
    size_t bufpos = 0;
    uint16_t buf16 = 0;
    uint8_t sum = 0;

    // ヘッダ組立
    memcpy(&(txbuf[bufpos]), &"BN", 2);
    bufpos += 2;

    buf16 = swap16(type);
    memcpy(&(txbuf[bufpos]), &buf16, sizeof(uint16_t));
    bufpos += 2;

    buf16 = swap16(sizex);
    memcpy(&(txbuf[bufpos]), &buf16, sizeof(uint16_t));
    bufpos += 2;

    buf16 = swap16(sizey);
    memcpy(&(txbuf[bufpos]), &buf16, sizeof(uint16_t));
    bufpos += 2;

    // チェックサム
    sum = sum_data_xor(sum, txbuf, bufpos);

    // ヘッダ送信
    send_serial_raw(txbuf, bufpos);

    // チェックサム
    sum = sum_data_xor(sum, buf, sizex * sizey);

    // データ送信
    send_serial_raw(buf, sizex * sizey);

    // フッタ組立
    bufpos = 0;
    memset(&(txbuf[bufpos]), 0, 8);
    bufpos += 8;

    memcpy(&(txbuf[bufpos]), &"\r\n", 2);
    bufpos += 2;

    // チェックサム
    sum = sum_data_xor(sum, txbuf, bufpos);

    txbuf[bufpos] = sum;
    bufpos += 1;

    // フッタ送信
    send_serial_raw(txbuf, bufpos);
    
}

// シリアルで複数バイト送信
void send_serial_raw(const void* data, size_t size)
{
    const char* cdata = (const char*)data;
    for (int i = 0; i < size; i++) {
        sio_pol_snd_chr(cdata[i], SIO_PORTID);
    }
}

// チェックサム(XORのみ)
uint8_t sum_data_xor(uint8_t init, const void* adata, size_t size)
{
    const uint8_t* data = (const uint8_t*)adata;
    uint8_t sum = init;
    for (int i = 0; i < size; i++) {
        sum = sum ^ data[i];
    }
    return sum;
}
