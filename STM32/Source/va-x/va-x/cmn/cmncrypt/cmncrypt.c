/*
 * VA-X 動的メモリ
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/03/08 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#include "cmncrypt.h"

#include <string.h>
#include <t_stddef.h>
#include <t_syslog.h>

//#define TFM_DESC
//#define USE_TFM
#define LTM_DESC
#define USE_LTM
#include "tomcrypt.h"

#include "cmncrypt_malloc.h"

// デバッグログ
#if 1
#define DBGLOG0(msg)					syslog(LOG_NOTICE, "[CMNCRYPT]" msg)
#define DBGLOG1(msg, arg1)				syslog(LOG_NOTICE, "[CMNCRYPT]" msg, arg1)
#define DBGLOG2(msg, arg1, arg2)		syslog(LOG_NOTICE, "[CMNCRYPT]" msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)	syslog(LOG_NOTICE, "[CMNCRYPT]" msg, arg1, arg2, arg3)
#else
#define DBGLOG0(msg)
#define DBGLOG1(msg, arg1)
#define DBGLOG2(msg, arg1, arg2)
#define DBGLOG3(msg, arg1, arg2, arg3)
#endif

#define KEY_LENGTH_BYTE	(2048 / 8)

// プロトタイプ

// 署名検証
int cmncrypt_verify_sig(const uint8_t* sig, size_t siglen, const uint8_t* msg, size_t msglen, const uint8_t* pubkey, size_t pubkeylen)
{
    int ret = 0;
    int result = 0;

    // malloc 初期化
    cmncrypt_init_malloc();

    // libtommath を使用する
    ltc_mp = ltm_desc;

    // sha256 を使用可能にする
    register_hash(&sha256_desc);
    int hash_idx = find_hash("sha256");

    // ハッシュ値(sha256)算出
    uint8_t hashbuf[32];
    long unsigned int hashlen = sizeof(hashbuf);
    ret = hash_memory(hash_idx, msg, msglen, hashbuf, &hashlen);
    if (ret != CRYPT_OK) {
        DBGLOG1("hash_memory() error(%d)", ret);
        result = -1;
        goto end;
    }

    // キー読み込み
    rsa_key key = {0};
    ret = rsa_import(pubkey, pubkeylen, &key);
    if(ret != CRYPT_OK) {
        DBGLOG1("rsa_import() error(%d)", ret);
        result = -2;
        goto end;
    }

    // 検証
    int saltlen = rsa_sign_saltlen_get_max(hash_idx, &key);
    int stat = 0;
    ret = rsa_verify_hash_ex(sig, siglen,
                             hashbuf, hashlen,
                             LTC_PKCS_1_V1_5, hash_idx, saltlen, &stat, &key);

    if (ret == CRYPT_OK && stat == 1) {
        result = 1;
    } else if (ret == CRYPT_OK && stat == 0) {
        DBGLOG1("rsa_verify_hash_ex() verify NG(stat:%d)", stat);
        result = 0;
    } else {
        DBGLOG1("rsa_verify_hash_ex() error(%d)", ret);
        result = -3;
    }

end:
    return result;
}

// 鍵の確認
int cmncrypt_verify_pubkey(const uint8_t* data, size_t datalen)
{
    int error = 0;

    // malloc 初期化
    cmncrypt_init_malloc();

    // libtommath を使用する
    ltc_mp = ltm_desc;

    // キー読み込み
    rsa_key key = {0};
    int ret = rsa_import(data, datalen, &key);

    // 読込み結果
    if (ret != CRYPT_OK) {
        DBGLOG1("rsa_import() error(%d)", ret);
        error = 1;
        goto end;
    }

    // キー長(バイト)取得
    int keysize = rsa_get_size(&key);
    if (keysize != KEY_LENGTH_BYTE) {
        DBGLOG1("keysize(%d) error", keysize);
        error = 2;
        goto end;
    }

    // 公開鍵である事の確認
    if (key.type != PK_PUBLIC) {
        DBGLOG1("keytype(%d) error", key.type);
        error = 3;
    }

end:
    return error;
}
