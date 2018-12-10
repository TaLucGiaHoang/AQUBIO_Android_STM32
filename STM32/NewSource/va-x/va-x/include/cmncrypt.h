/*
 * VA-X 暗号アルゴリズム (libtomcrypt)
 *
 * Copyright (C) 2018 Bionics co.,ltd.
 *
 * 変更:
 * - 2018/03/08 Takuya Goto <goto-takuya@bionics-k.co.jp>: 新規作成
 */

#pragma once

#include <t_stddef.h>

/* 署名の検証 */
int cmncrypt_verify_sig(const uint8_t* sig, size_t siglen, const uint8_t* msg, size_t msglen, const uint8_t* pubkey, size_t pubkeylen);

/* 有効な鍵(RSA, 公開鍵, 2048bit, DER)かどうかを確認 */
int cmncrypt_verify_pubkey(const uint8_t* key, size_t keylen);

