#include "Aes_api.h"
#include <string.h>

typedef enum
{
    AES_MODE_NONE,
    AES_MODE_CBC,
    AES_MODE_ECB,
    AES_MODE_CFB,
    AES_MODE_OFB,
    AES_MODE_CTR
} T_HW_AES_MODE;

extern bool hw_aes_encrypt128(uint32_t *p_in, uint32_t *p_out, uint16_t data_word_len,
                              uint32_t *p_key,
                              uint32_t *p_iv, T_HW_AES_MODE mode);

extern bool hw_aes_decrypt128(uint32_t *p_in, uint32_t *p_out, uint16_t data_word_len,
                              uint32_t *p_key,
                              uint32_t *p_iv, T_HW_AES_MODE mode);

extern bool hw_aes_encrypt256(uint32_t *p_in, uint32_t *p_out, uint16_t data_word_len,
                              uint32_t *p_key,
                              uint32_t *p_iv, T_HW_AES_MODE mode);

extern bool hw_aes_decrypt256(uint32_t *p_in, uint32_t *p_out, uint16_t data_word_len,
                              uint32_t *p_key,
                              uint32_t *p_iv, T_HW_AES_MODE mode);

static uint32_t Iv[4] = {0};  // Initial Vector used only for CBC mode

static void swap_buf(const uint8_t *src, uint8_t *dst, uint16_t len)
{
    int i;

    for (i = 0; i < len; i++)
    {
        dst[len - 1 - i] = src[i];
    }
}

// ECB mode for 128-bit aes key
bool aes128_ecb_encrypt_buffer(uint8_t *plaintext, const uint8_t key[16], uint8_t *encrypted,
                               uint32_t data_word_len)
{
    bool ret;
    uint8_t *in = plaintext;
    uint8_t *out = encrypted;

    if (data_word_len % 4) { return false; }

    for (int i = 0; i < data_word_len; i += 4)
    {
        ret = aes128_ecb_encrypt(in, key, out);

        in += 16;
        out += 16;

        if (!ret) { break; }
    }

    return ret;
}

bool aes128_ecb_decrypt_buffer(uint8_t *input, const uint8_t key[16], uint8_t *output,
                               uint32_t data_word_len)
{
    bool ret;
    uint8_t *in = input;
    uint8_t *out = output;

    if (data_word_len % 4) { return false; }

    for (int i = 0; i < data_word_len; i += 4)
    {
        ret = aes128_ecb_decrypt(in, key, out);

        in += 16;
        out += 16;

        if (!ret) { break; }
    }

    return ret;
}

bool aes128_ecb_encrypt_msb2lsb_buffer(uint8_t *plaintext, const uint8_t key[16],
                                       uint8_t *encrypted, uint32_t data_word_len)
{
    bool ret;
    uint8_t *in = plaintext;
    uint8_t *out = encrypted;

    if (data_word_len % 4) { return false; }

    for (int i = 0; i < data_word_len; i += 4)
    {
        ret = aes128_ecb_encrypt_msb2lsb(in, key, out);

        in += 16;
        out += 16;

        if (!ret) { break; }
    }

    return ret;
}

bool aes128_ecb_decrypt_msb2lsb_buffer(uint8_t *input, const uint8_t key[16], uint8_t *output,
                                       uint32_t data_word_len)
{
    bool ret;
    uint8_t *in = input;
    uint8_t *out = output;

    if (data_word_len % 4) { return false; }

    for (int i = 0; i < data_word_len; i += 4)
    {
        ret = aes128_ecb_decrypt_msb2lsb(in, key, out);

        in += 16;
        out += 16;

        if (!ret) { break; }
    }

    return ret;
}

// ECB mode for 256-bit aes key
bool aes256_ecb_encrypt_buffer(uint8_t *plaintext, const uint8_t key[32], uint8_t *encrypted,
                               uint32_t data_word_len)
{
    bool ret;
    uint8_t *in = plaintext;
    uint8_t *out = encrypted;

    if (data_word_len % 4) { return false; }

    for (int i = 0; i < data_word_len; i += 4)
    {
        ret = aes256_ecb_encrypt(in, key, out);

        in += 16;
        out += 16;

        if (!ret) { break; }
    }

    return ret;
}

bool aes256_ecb_decrypt_buffer(uint8_t *input, const uint8_t key[32], uint8_t *output,
                               uint32_t data_word_len)
{
    bool ret;
    uint8_t *in = input;
    uint8_t *out = output;

    if (data_word_len % 4) { return false; }

    for (int i = 0; i < data_word_len; i += 4)
    {
        ret = aes256_ecb_decrypt(in, key, out);

        in += 16;
        out += 16;

        if (!ret) { break; }
    }

    return ret;
}

bool aes256_ecb_encrypt_msb2lsb_buffer(uint8_t *plaintext, const uint8_t key[32],
                                       uint8_t *encrypted, uint32_t data_word_len)
{
    bool ret;
    uint8_t *in = plaintext;
    uint8_t *out = encrypted;

    if (data_word_len % 4) { return false; }

    for (int i = 0; i < data_word_len; i += 4)
    {
        ret = aes256_ecb_encrypt_msb2lsb(in, key, out);

        in += 16;
        out += 16;

        if (!ret) { break; }
    }

    return ret;
}

bool aes256_ecb_decrypt_msb2lsb_buffer(uint8_t *input, const uint8_t key[32], uint8_t *output,
                                       uint32_t data_word_len)
{
    bool ret;
    uint8_t *in = input;
    uint8_t *out = output;

    if (data_word_len % 4) { return false; }

    for (int i = 0; i < data_word_len; i += 4)
    {
        ret = aes256_ecb_decrypt_msb2lsb(in, key, out);

        in += 16;
        out += 16;

        if (!ret) { break; }
    }

    return ret;
}

// CBC mode for 128-bit aes key
bool aes128_cbc_encrypt(uint8_t *plaintext, const uint8_t key[16], uint8_t *encrypted,
                        uint32_t *p_iv, uint32_t data_word_len)
{
    bool ret;

    uint8_t key_tmp[16] = {0};
    /* The most significant octet of key corresponds to key[0] */
    swap_buf(key, key_tmp, 16);

    uint8_t *in = plaintext;
    uint8_t *out = encrypted;

    if (p_iv != 0) { memcpy(Iv, p_iv, 16); }

    if (data_word_len % 4) { return false; }

    for (int i = 0; i < data_word_len; i += 4)
    {

        uint8_t in_tmp[16] = {0};
        uint8_t out_tmp[16] = {0};

        swap_buf(in, in_tmp, 16);

        ret = hw_aes_encrypt128((uint32_t *) in_tmp, (uint32_t *) out_tmp, 4, (uint32_t *)key_tmp, Iv,
                                AES_MODE_CBC);

        memcpy(Iv, out_tmp, 16);

        swap_buf(out_tmp, out, 16);

        in += 16;
        out += 16;

        if (!ret) { break; }
    }

    return ret;
}

bool aes128_cbc_decrypt(uint8_t *input, const uint8_t key[16], uint8_t *output, uint32_t *p_iv,
                        uint32_t data_word_len)
{
    bool ret;

    uint8_t key_tmp[16] = {0};
    swap_buf(key, key_tmp, 16);

    uint8_t *in = input;
    uint8_t *out = output;

    if (p_iv != 0) { memcpy(Iv, p_iv, 16); }

    if (data_word_len % 4) { return false; }

    for (int i = 0; i < data_word_len; i += 4)
    {

        uint8_t in_tmp[16] = {0};
        uint8_t out_tmp[16] = {0};

        swap_buf(in, in_tmp, 16);

        ret = hw_aes_decrypt128((uint32_t *) in_tmp, (uint32_t *) out_tmp, 4, (uint32_t *)key_tmp, Iv,
                                AES_MODE_CBC);

        memcpy(Iv, in_tmp, 16);

        swap_buf(out_tmp, out, 16);

        in += 16;
        out += 16;

        if (!ret) { break; }
    }

    return ret;
}


bool aes128_cbc_encrypt_msb2lsb(uint8_t plaintext[16], const uint8_t key[16], uint8_t *encrypted,
                                uint32_t *p_iv, uint32_t data_word_len)
{
    if (data_word_len % 4) { return false; }

    return hw_aes_encrypt128((uint32_t *) plaintext, (uint32_t *) encrypted, data_word_len,
                             (uint32_t *)key, p_iv,
                             AES_MODE_CBC);
}

bool aes128_cbc_decrypt_msb2lsb(uint8_t *input, const uint8_t key[16], uint8_t *output,
                                uint32_t *p_iv, uint32_t data_word_len)
{
    if (data_word_len % 4) { return false; }

    return hw_aes_decrypt128((uint32_t *)input, (uint32_t *)output, data_word_len, (uint32_t *)key,
                             p_iv,
                             AES_MODE_CBC);
}

// CBC mode for 256-bit aes key
bool aes256_cbc_encrypt(uint8_t *plaintext, const uint8_t key[32], uint8_t *encrypted,
                        uint32_t *p_iv, uint32_t data_word_len)
{
    bool ret;

    uint8_t key_tmp[32] = {0};
    /* The most significant octet of key corresponds to key[0] */
    swap_buf(key, key_tmp, 32);

    uint8_t *in = plaintext;
    uint8_t *out = encrypted;

    if (p_iv != 0) { memcpy(Iv, p_iv, 16); }

    if (data_word_len % 4) { return false; }

    for (int i = 0; i < data_word_len; i += 4)
    {

        uint8_t in_tmp[16] = {0};
        uint8_t out_tmp[16] = {0};

        swap_buf(in, in_tmp, 16);

        ret = hw_aes_encrypt256((uint32_t *) in_tmp, (uint32_t *) out_tmp, 4, (uint32_t *)key_tmp, Iv,
                                AES_MODE_CBC);

        memcpy(Iv, out_tmp, 16);

        swap_buf(out_tmp, out, 16);

        in += 16;
        out += 16;

        if (!ret) { break; }
    }

    return ret;
}

bool aes256_cbc_decrypt(uint8_t *input, const uint8_t key[32], uint8_t *output, uint32_t *p_iv,
                        uint32_t data_word_len)
{
    bool ret;

    uint8_t key_tmp[32] = {0};
    swap_buf(key, key_tmp, 32);

    uint8_t *in = input;
    uint8_t *out = output;

    if (p_iv != 0) { memcpy(Iv, p_iv, 16); }

    if (data_word_len % 4) { return false; }

    for (int i = 0; i < data_word_len; i += 4)
    {

        uint8_t in_tmp[16] = {0};
        uint8_t out_tmp[16] = {0};

        swap_buf(in, in_tmp, 16);

        ret = hw_aes_decrypt256((uint32_t *) in_tmp, (uint32_t *) out_tmp, 4, (uint32_t *)key_tmp, Iv,
                                AES_MODE_CBC);

        memcpy(Iv, in_tmp, 16);

        swap_buf(out_tmp, out, 16);

        in += 16;
        out += 16;

        if (!ret) { break; }
    }

    return ret;
}
