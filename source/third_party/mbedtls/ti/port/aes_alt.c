/******************************************************************************
 Copyright (c) 2019, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "mbedtls/aes.h"
#include "aes_alt.h"

#if defined(MBEDTLS_AES_ALT)

#include <string.h>
#include <assert.h>

#include <ti/drivers/cryptoutils/cryptokey/CryptoKeyPlaintext.h>
#include <ti/drivers/dpl/SemaphoreP.h>

#include <ti/drivers/AESECB.h>

/* These non-public functions are required to ensure thread-safe behavior
 * across multiple calls.
 */
extern bool AESECB_acquireLock(AESECB_Handle handle, uint32_t timeout);
extern void AESECB_releaseLock(AESECB_Handle handle);
extern void AESECB_enableThreadSafety(AESECB_Handle handle);
extern void AESECB_disableThreadSafety(AESECB_Handle handle);

const AESECB_HWAttrs defaultAesEcbHwAttrs = {~0};
AESECB_Object       ecbObject;                 /*!< Pointer to a driver specific data object */
AESECB_Config       ecbConfig;                 /*!< structure containing AESECB driver specific implementation  */
AESECB_Handle       ecbHandle;                 /*!< A handle that is returned by the AESECB driver  */

#if defined(MBEDTLS_CIPHER_MODE_CBC)

#include <ti/drivers/AESCBC.h>

/* These non-public functions are required to ensure thread-safe behavior
 * across multiple calls.
 */
extern bool AESCBC_acquireLock(AESCBC_Handle handle, uint32_t timeout);
extern void AESCBC_releaseLock(AESCBC_Handle handle);
extern void AESCBC_enableThreadSafety(AESCBC_Handle handle);
extern void AESCBC_disableThreadSafety(AESCBC_Handle handle);

const AESCBC_HWAttrs defaultAesCbcHwAttrs = {~0};
AESCBC_Object       cbcObject;                 /*!< Pointer to a driver specific data object */
AESCBC_Config       cbcConfig;                 /*!< structure containing AESCBC driver specific implementation  */
AESCBC_Handle       cbcHandle;                 /*!< A handle that is returned by the AESCBC driver  */

#endif /* MBEDTLS_CIPHER_MODE_CBC */

bool isInitialized = false;


/**
 * @brief Initialize AES context
 *
 * @param [in,out] ctx AES context to be initialized
 */
void mbedtls_aes_init(mbedtls_aes_context *ctx)
{
    memset(ctx, 0, sizeof(mbedtls_aes_context));

    if (isInitialized == false) {

        AESECB_Params ecbParams;

        AESECB_init();

        AESECB_Params_init(&ecbParams);
        ecbParams.returnBehavior = AESECB_RETURN_BEHAVIOR_POLLING;

        ecbConfig.object = &ecbObject;
        ecbConfig.hwAttrs = &defaultAesEcbHwAttrs;

        ecbHandle = AESECB_construct(&ecbConfig, &ecbParams);

        /* Since we will acquire the mutex manually, we do not need to check it
         * in the driver itself.
         */
        AESECB_disableThreadSafety(ecbHandle);

        #if defined(MBEDTLS_CIPHER_MODE_CBC)

        AESCBC_Params cbcParams;

        AESCBC_init();

        AESCBC_Params_init(&cbcParams);
        cbcParams.returnBehavior = AESCBC_RETURN_BEHAVIOR_POLLING;

        cbcConfig.object = &cbcObject;
        cbcConfig.hwAttrs = &defaultAesCbcHwAttrs;

        cbcHandle = AESCBC_construct(&cbcConfig, &cbcParams);

        /* Since we will acquire the mutex manually, we do not need to check it
         * in the driver itself.
         */
        AESCBC_disableThreadSafety(cbcHandle);

        #endif /* MBEDTLS_CIPHER_MODE_CBC */

        isInitialized = true;
    }
}

/**
 * @brief          Clear AES context
 *
 * \param ctx      AES context to be cleared
 */
void mbedtls_aes_free(mbedtls_aes_context *ctx)
{
    memset(ctx, 0, sizeof(mbedtls_aes_context));
}

/**
 * \brief          AES key schedule (encryption)
 *
 * \param ctx      AES context to be initialized
 * \param key      encryption key
 * \param keybits  must be 128 or 256
 *
 * \return         0 if successful, or MBEDTLS_ERR_AES_INVALID_KEY_LENGTH
 */
int mbedtls_aes_setkey_enc(mbedtls_aes_context *ctx, const unsigned char *key, unsigned int keybits)
{
    int_fast16_t statusCrypto = 0;

    if (keybits == 192) {
        return MBEDTLS_ERR_AES_FEATURE_UNAVAILABLE;
    }

    /* Initialize AES key */
    memcpy(ctx->keyMaterial, key, (keybits >> 3));
    statusCrypto = CryptoKeyPlaintext_initKey(&ctx->cryptoKey, (uint8_t*) ctx->keyMaterial, (keybits >> 3));
    assert(statusCrypto == 0);

    return (int)statusCrypto;
}

/**
 * \brief          AES key schedule (decryption)
 *
 * \param ctx      AES context to be initialized
 * \param key      decryption key
 * \param keybits  must be 128 or 256
 *
 * \return         0 if successful, or MBEDTLS_ERR_AES_INVALID_KEY_LENGTH
 */
int mbedtls_aes_setkey_dec(mbedtls_aes_context *ctx, const unsigned char *key, unsigned int keybits)
{
    int_fast16_t statusCrypto;

    if (keybits == 192) {
        return MBEDTLS_ERR_AES_FEATURE_UNAVAILABLE;
    }

    /* Initialize AES key */
    memcpy(ctx->keyMaterial, key, (keybits >> 3));
    statusCrypto = CryptoKeyPlaintext_initKey(&ctx->cryptoKey, (uint8_t*) ctx->keyMaterial, (keybits >> 3));
    assert(statusCrypto == 0);

    return (int)statusCrypto;
}

/**
 * \brief          AES-ECB block encryption/decryption
 *
 * \param ctx      AES context
 * \param mode     MBEDTLS_AES_ENCRYPT or MBEDTLS_AES_DECRYPT
 * \param input    16-byte input block
 * \param output   16-byte output block
 *
 * \return         0 if successful
 */
int mbedtls_aes_crypt_ecb(mbedtls_aes_context *ctx, int mode, const unsigned char input[16], unsigned char output[16])
{
    int statusCrypto;
    AESECB_Operation operationOneStep;

    /* We are waiting forever here on purpose to allow synchronous operation
     * without the potential overhead from using AESECB_RETURN_BEHAVIOR_BLOCKING
     * doing context switches to an ISR etc.
     */
    if (AESECB_acquireLock(ecbHandle, SemaphoreP_WAIT_FOREVER) == false) {
        return MBEDTLS_ERR_AES_HW_ACCEL_FAILED;
    }

    AESECB_Operation_init(&operationOneStep);

    operationOneStep.key = &ctx->cryptoKey;
    operationOneStep.inputLength = 16;
    operationOneStep.input = (uint8_t *)input;
    operationOneStep.output = (uint8_t *)output;

    if (mode == MBEDTLS_AES_DECRYPT) {
        statusCrypto = AESECB_oneStepDecrypt(ecbHandle, &operationOneStep);
    }
    else {
        statusCrypto = AESECB_oneStepEncrypt(ecbHandle, &operationOneStep);
    }

    AESECB_releaseLock(ecbHandle);

    return statusCrypto;
}

#if defined(MBEDTLS_CIPHER_MODE_CBC)
/*
 * AES-CBC buffer encryption/decryption
 */
int mbedtls_aes_crypt_cbc(mbedtls_aes_context *ctx,
                          int mode,
                          size_t length,
                          unsigned char iv[16],
                          const unsigned char *input,
                          unsigned char *output)
{
    int statusCrypto;
    AESCBC_Operation operationOneStep;

    /* We are waiting forever here on purpose to allow synchronous operation
     * without the potential overhead from using AESCBC_RETURN_BEHAVIOR_BLOCKING
     * doing context switches to an ISR etc.
     */
    if (AESCBC_acquireLock(cbcHandle, SemaphoreP_WAIT_FOREVER) == false) {
        return MBEDTLS_ERR_AES_HW_ACCEL_FAILED;
    }

    AESCBC_Operation_init(&operationOneStep);

    operationOneStep.key = &ctx->cryptoKey;
    operationOneStep.inputLength = length;
    operationOneStep.input = (uint8_t *)input;
    operationOneStep.output = (uint8_t *)output;
    operationOneStep.iv = (uint8_t *)iv;

    if (mode == MBEDTLS_AES_DECRYPT) {
        statusCrypto = AESCBC_oneStepDecrypt(cbcHandle, &operationOneStep);
    }
    else {
        statusCrypto = AESCBC_oneStepEncrypt(cbcHandle, &operationOneStep);
    }

    AESCBC_getNextIv(cbcHandle, iv);

    AESCBC_releaseLock(cbcHandle);

    return statusCrypto;
}
#endif /* MBEDTLS_CIPHER_MODE_CBC */

#if defined(MBEDTLS_CIPHER_MODE_CFB)
/*
 * AES-CFB128 buffer encryption/decryption
 */
int mbedtls_aes_crypt_cfb128( mbedtls_aes_context *ctx,
                       int mode,
                       size_t length,
                       size_t *iv_off,
                       unsigned char iv[16],
                       const unsigned char *input,
                       unsigned char *output )
{
    int c;
    size_t n = *iv_off;

    if( mode == MBEDTLS_AES_DECRYPT )
    {
        while( length-- )
        {
            if( n == 0 )
                mbedtls_aes_crypt_ecb( ctx, MBEDTLS_AES_ENCRYPT, iv, iv );

            c = *input++;
            *output++ = (unsigned char)( c ^ iv[n] );
            iv[n] = (unsigned char) c;

            n = ( n + 1 ) & 0x0F;
        }
    }
    else
    {
        while( length-- )
        {
            if( n == 0 )
                mbedtls_aes_crypt_ecb( ctx, MBEDTLS_AES_ENCRYPT, iv, iv );

            iv[n] = *output++ = (unsigned char)( iv[n] ^ *input++ );

            n = ( n + 1 ) & 0x0F;
        }
    }

    *iv_off = n;

    return 0;
}

/*
 * AES-CFB8 buffer encryption/decryption
 */
int mbedtls_aes_crypt_cfb8( mbedtls_aes_context *ctx,
                       int mode,
                       size_t length,
                       unsigned char iv[16],
                       const unsigned char *input,
                       unsigned char *output )
{
    unsigned char c;
    unsigned char ov[17];

    while( length-- )
    {
        memcpy( ov, iv, 16 );
        mbedtls_aes_crypt_ecb( ctx, MBEDTLS_AES_ENCRYPT, iv, iv );

        if( mode == MBEDTLS_AES_DECRYPT )
            ov[16] = *input;

        c = *output++ = (unsigned char)( iv[0] ^ *input++ );

        if( mode == MBEDTLS_AES_ENCRYPT )
            ov[16] = c;

        memcpy( iv, ov + 1, 16 );
    }

    return( 0 );
}
#endif /*MBEDTLS_CIPHER_MODE_CFB */

#if defined(MBEDTLS_CIPHER_MODE_CTR)
/*
 * AES-CTR buffer encryption/decryption
 */
int mbedtls_aes_crypt_ctr( mbedtls_aes_context *ctx,
                       size_t length,
                       size_t *nc_off,
                       unsigned char nonce_counter[16],
                       unsigned char stream_block[16],
                       const unsigned char *input,
                       unsigned char *output )
{
    int c, i;
    size_t n = *nc_off;

    while( length-- )
    {
        if( n == 0 ) {
            mbedtls_aes_crypt_ecb( ctx, MBEDTLS_AES_ENCRYPT, nonce_counter, stream_block );

            for( i = 16; i > 0; i-- )
                if( ++nonce_counter[i - 1] != 0 )
                    break;
        }
        c = *input++;
        *output++ = (unsigned char)( c ^ stream_block[n] );

        n = ( n + 1 ) & 0x0F;
    }

    *nc_off = n;

    return( 0 );
}
#endif /* MBEDTLS_CIPHER_MODE_CTR */

#endif /* MBEDTLS_AES_ALT */

