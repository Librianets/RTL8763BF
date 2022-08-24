/**
*********************************************************************************************************
*               Copyright(c) 2018, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     gpio_output_demo.c
* @brief    This file provides demo code of gpio output mode.
            Control LED flashing.
* @details
* @author   yuan
* @date     2018-12-07
* @version  v1.0
*********************************************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "trace.h"
#include "Aes_api.h"
#include <string.h>

/* Defines ------------------------------------------------------------------*/

#define BIG_ENDIAN_16                        0       // 0: least significant octet of 16 byte data is the first octet; 1: the most significant is the first
#define AES_ECB_MODE                         0       // 0: CBC mode for AES; 1: ECB mode for AES
#define AES_KEY_256                          0       // 0: the length of AES key is 128 bit; 1: the length of AES key is 256 bit
#define TEST_WORD_LEN                        256     // the word length of the plain text data for test, must be multiples of 4 (bytes of data length is multiple of 16) 

uint8_t in[TEST_WORD_LEN * 4] = {0};
uint8_t out[TEST_WORD_LEN * 4] = {0};
uint8_t dec_result[TEST_WORD_LEN * 4] = {0};

//  data_pattern: 9696a5a5a5a5c3c3c3c396967878f0f0
const uint8_t test_pattern[16] = {0x96, 0x96, 0xa5, 0xa5, 0xa5, 0xa5, 0xc3, 0xc3, 0xc3, 0xc3, 0x96, 0x96, 0x78, 0x78, 0xf0, 0xf0};

void test_hw_aes(void)
{

#if AES_KEY_256 == 0  // key_pattern: ec4174c29dfcf0916d172e6a652e168e
    uint8_t aes_key_16[16] = {0xec, 0x41, 0x74, 0xc2, 0x9d, 0xfc, 0xf0, 0x91, 0x6d, 0x17, 0x2e, 0x6a, 0x65, 0x2e, 0x16, 0x8e};
#else
    uint8_t aes_key_32[32] = { 0xec, 0x41, 0x74, 0xc2, 0x9d, 0xfc, 0xf0, 0x91, 0x6d, 0x17, 0x2e, 0x6a, 0x65, 0x2e, 0x16, 0x8e,
                               0xec, 0x41, 0x74, 0xc2, 0x9d, 0xfc, 0xf0, 0x91, 0x6d, 0x17, 0x2e, 0x6a, 0x65, 0x2e, 0x16, 0x8e
                             };
#endif

    /* prepare encrypted data */
    for (int i = 0; i < TEST_WORD_LEN * 4; i++)
    {
        in[i] = test_pattern[i % 16];
    }

#if AES_ECB_MODE == 0                                    // CBC MODE Encryption
    // iv: 000102030405060708090a0b0c0d0e0f
    const uint32_t IV[4] = {0x0c0d0e0f, 0x08090a0b, 0x04050607, 0x00010203};
    uint32_t iv[4] = {0};
    memcpy(iv, IV, 16);

#if BIG_ENDIAN_16 == 1

    aes128_cbc_encrypt_msb2lsb(in, (const uint8_t *)aes_key_16, out, iv, TEST_WORD_LEN);

#else  /* -------------------------------------------*/

#if AES_KEY_256 == 0
    aes128_cbc_encrypt(in, (const uint8_t *)aes_key_16, out, iv, TEST_WORD_LEN);
#else
    aes256_cbc_encrypt(in, (const uint8_t *)aes_key_32, out, iv, TEST_WORD_LEN);
#endif

#endif

#else                                                    // ECB MODE Encryption

#if BIG_ENDIAN_16 == 1

#if AES_KEY_256 == 0
    aes128_ecb_encrypt_msb2lsb_buffer(in, (const uint8_t *)aes_key_16, out, TEST_WORD_LEN);
#else
    aes256_ecb_encrypt_msb2lsb_buffer(in, (const uint8_t *)aes_key_32, out, TEST_WORD_LEN);
#endif

#else   /* -------------------------------------------*/

#if AES_KEY_256 == 0
    aes128_ecb_encrypt_buffer(in, (const uint8_t *)aes_key_16, out, TEST_WORD_LEN);
#else
    aes256_ecb_encrypt_buffer(in, (const uint8_t *)aes_key_32, out, TEST_WORD_LEN);
#endif

#endif

#endif

    DBG_DIRECT("IN:");
    for (int i = 0; i < TEST_WORD_LEN * 4; i += 8)
    {
        DBG_DIRECT("%x,%x,%x,%x,%x,%x,%x,%x", in[i], in[i + 1], in[i + 2], in[i + 3], in[i + 4], in[i + 5],
                   in[i + 6], in[i + 7]);
    }
    DBG_DIRECT("OUT:");
    for (int i = 0; i < TEST_WORD_LEN * 4; i += 8)
    {
        DBG_DIRECT("%x,%x,%x,%x,%x,%x,%x,%x", out[i], out[i + 1], out[i + 2], out[i + 3], out[i + 4],
                   out[i + 5], out[i + 6], out[i + 7]);
    }

#if AES_ECB_MODE == 0                                  // CBC MODE Descryption

    memcpy(iv, IV, 16);

#if BIG_ENDIAN_16 == 1

    aes128_cbc_decrypt_msb2lsb(out, aes_key_16, dec_result, iv, TEST_WORD_LEN);

#else  /* -------------------------------------------*/

#if AES_KEY_256 == 0
    aes128_cbc_decrypt(out, aes_key_16, dec_result, iv, TEST_WORD_LEN);
#else
    aes256_cbc_decrypt(out, aes_key_32, dec_result, iv, TEST_WORD_LEN);
#endif

#endif

#else                                                 // ECB MODE Descryption

#if BIG_ENDIAN_16 == 1

#if AES_KEY_256 == 0
    aes128_ecb_decrypt_msb2lsb_buffer(out, aes_key_16, dec_result, TEST_WORD_LEN);
#else
    aes256_ecb_decrypt_msb2lsb_buffer(out, aes_key_32, dec_result, TEST_WORD_LEN);
#endif

#else  /* -------------------------------------------*/

#if AES_KEY_256 == 0
    aes128_ecb_decrypt_buffer(out, aes_key_16, dec_result, TEST_WORD_LEN);
#else
    aes256_ecb_decrypt_buffer(out, aes_key_32, dec_result, TEST_WORD_LEN);
#endif

#endif

#endif

// Compare plain text with decrytion result
    for (int i = 0; i < TEST_WORD_LEN * 4; i ++)
    {
        if (dec_result[i] != in[i])
        {
            DBG_DIRECT("decryption failed, dec_result[%d] = 0x%x, in[%d] = 0x%x", i, dec_result[i], i, in[i]);
        }
    }

}

/**
  * @brief    Entry of app code
  * @return   int(To avoid compile warning)
  */
int main(void)
{
    test_hw_aes();

    while (1)
    {
    }
}

/******************* (C) COPYRIGHT 2018 Realtek Semiconductor Corporation *****END OF FILE****/
