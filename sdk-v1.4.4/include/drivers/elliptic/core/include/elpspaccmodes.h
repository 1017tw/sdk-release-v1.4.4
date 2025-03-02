/*
 * Copyright (c) 2015 Elliptic Technologies Inc.
 */

#ifndef _ELPSPACCMODES_H_
#define _ELPSPACCMODES_H_

enum eicvpos
{
  IP_ICV_OFFSET = 0,
  IP_ICV_APPEND = 1,
  IP_ICV_IGNORE = 2,
  IP_MAX
};

enum {
  ICV_HASH = 0,             /* HASH of plaintext */
  ICV_HASH_ENCRYPT = 1,     /* HASH the plaintext and encrypt the plaintext and ICV */
  ICV_ENCRYPT_HASH = 2,     /* HASH the ciphertext */
  ICV_IGNORE=3,
  IM_MAX,
};

enum crypto_modes {
  CRYPTO_MODE_NULL,
  CRYPTO_MODE_RC4_40,
  CRYPTO_MODE_RC4_128,
  CRYPTO_MODE_RC4_KS,
  CRYPTO_MODE_AES_ECB,
  CRYPTO_MODE_AES_CBC,
  CRYPTO_MODE_AES_CTR,
  CRYPTO_MODE_AES_CCM,
  CRYPTO_MODE_AES_GCM,
  CRYPTO_MODE_AES_F8,
  CRYPTO_MODE_AES_XTS,
  CRYPTO_MODE_AES_CFB,
  CRYPTO_MODE_AES_OFB,
  CRYPTO_MODE_AES_CS1,
  CRYPTO_MODE_AES_CS2,
  CRYPTO_MODE_AES_CS3,
  CRYPTO_MODE_MULTI2_ECB,
  CRYPTO_MODE_MULTI2_CBC,
  CRYPTO_MODE_MULTI2_OFB,
  CRYPTO_MODE_MULTI2_CFB,
  CRYPTO_MODE_3DES_CBC,
  CRYPTO_MODE_3DES_ECB,
  CRYPTO_MODE_DES_CBC,
  CRYPTO_MODE_DES_ECB,
  CRYPTO_MODE_KASUMI_ECB,
  CRYPTO_MODE_KASUMI_F8,
  CRYPTO_MODE_SNOW3G_UEA2,
  CRYPTO_MODE_ZUC_UEA3,

  CRYPTO_MODE_HASH_MD5,
  CRYPTO_MODE_HMAC_MD5,
  CRYPTO_MODE_HASH_SHA1,
  CRYPTO_MODE_HMAC_SHA1,
  CRYPTO_MODE_HASH_SHA224,
  CRYPTO_MODE_HMAC_SHA224,
  CRYPTO_MODE_HASH_SHA256,
  CRYPTO_MODE_HMAC_SHA256,
  CRYPTO_MODE_HASH_SHA384,
  CRYPTO_MODE_HMAC_SHA384,
  CRYPTO_MODE_HASH_SHA512,
  CRYPTO_MODE_HMAC_SHA512,
  CRYPTO_MODE_HASH_SHA512_224,
  CRYPTO_MODE_HMAC_SHA512_224,
  CRYPTO_MODE_HASH_SHA512_256,
  CRYPTO_MODE_HMAC_SHA512_256,

  CRYPTO_MODE_MAC_XCBC,
  CRYPTO_MODE_MAC_CMAC,
  CRYPTO_MODE_MAC_KASUMI_F9,
  CRYPTO_MODE_MAC_SNOW3G_UIA2,
  CRYPTO_MODE_MAC_ZUC_UIA3,

  CRYPTO_MODE_SSLMAC_MD5,
  CRYPTO_MODE_SSLMAC_SHA1,
  CRYPTO_MODE_HASH_CRC32,
  CRYPTO_MODE_MAC_MICHAEL,

  CRYPTO_MODE_HASH_SHA3_224,
  CRYPTO_MODE_HASH_SHA3_256,
  CRYPTO_MODE_HASH_SHA3_384,
  CRYPTO_MODE_HASH_SHA3_512,

  CRYPTO_MODE_LAST
};

#endif
