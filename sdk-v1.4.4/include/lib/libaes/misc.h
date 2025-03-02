/* misc.h
 *
 * Copyright (C) 2006-2015 wolfSSL Inc.
 *
 * This file is part of wolfSSL. (formerly known as CyaSSL)
 *
 * wolfSSL is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * wolfSSL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA
 */


#ifndef WOLF_CRYPT_MISC_H
#define WOLF_CRYPT_MISC_H


// #include <wolfssl/wolfcrypt/types.h>


#ifdef __cplusplus
    extern "C" {
#endif


#ifdef NO_INLINE
WOLFSSL_LOCAL
uint32_t rotlFixed(uint32_t, uint32_t);
WOLFSSL_LOCAL
uint32_t rotrFixed(uint32_t, uint32_t);

WOLFSSL_LOCAL
uint32_t ByteReverseWord32(uint32_t);
WOLFSSL_LOCAL
void   ByteReverseWords(uint32_t*, const uint32_t*, uint32_t);

WOLFSSL_LOCAL
void XorWords(wolfssl_word*, const wolfssl_word*, uint32_t);
WOLFSSL_LOCAL
void xorbuf(void*, const void*, uint32_t);

WOLFSSL_LOCAL
void ForceZero(const void*, uint32_t);

WOLFSSL_LOCAL
int ConstantCompare(const byte*, const byte*, int);

#ifdef WORD64_AVAILABLE
WOLFSSL_LOCAL
word64 rotlFixed64(word64, word64);
WOLFSSL_LOCAL
word64 rotrFixed64(word64, word64);

WOLFSSL_LOCAL
word64 ByteReverseWord64(word64);
WOLFSSL_LOCAL
void   ByteReverseWords64(word64*, const word64*, uint32_t);
#endif /* WORD64_AVAILABLE */

#endif /* NO_INLINE */


#ifdef __cplusplus
    }   /* extern "C" */
#endif


#endif /* WOLF_CRYPT_MISC_H */

