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

 ******************************************************************************/

/*!
 *  @file       sha256_alt.h
 *
 *  @brief      Hardware accelerated mbedtls sha256 implementation
 *
 *  # General
 *  This alternate sha256 implementation for mbedtls is intended to provide
 *  hardware acceleration on any SimpleLink platform that has implemented the
 *  SHA2 driver APIs.
 *
 *  The implementation is based on the SHA2 driver and thus
 *  also requires any link-time dependencies it has.
 *
 *  # Limitations
 *  The replaced functions may only be called from Task context when using an
 *  operating system. The driver instances are set up to block until they
 *  acquire the mutex protecting the hardware from concurrent accesses. Blocking
 *  is only permitted in Task context. The upside of this is that none of the
 *  calls will return an error because they were unable to access the hardware
 *  immediately. The downside is of course that the functions may not be called
 *  from Hwi or Swi context.
 */

#ifndef MBEDTLS_SHA256_ALT_H
#define MBEDTLS_SHA256_ALT_H

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#if defined(MBEDTLS_SHA256_ALT)
#ifdef __cplusplus
extern "C" {
#endif

#include <ti/devices/DeviceFamily.h>

#include <ti/drivers/SHA2.h>

#if (DeviceFamily_PARENT == DeviceFamily_PARENT_MSP432E4X1Y)

    #include <ti/drivers/sha2/SHA2MSP432E4.h>
    typedef SHA2MSP432E4_Object SHA2_Object;
    typedef SHA2MSP432E4_HWAttrs SHA2_HWAttrs;

#elif (DeviceFamily_PARENT == DeviceFamily_PARENT_CC13X2_CC26X2)

    #include <ti/drivers/sha2/SHA2CC26X2.h>
    typedef SHA2CC26X2_Object SHA2_Object;
    typedef SHA2CC26X2_HWAttrs SHA2_HWAttrs;

#else
#error "No valid DeviceFamily found for the SHA2 alternate implementation!"
#endif

/**
 * \brief          SHA-256 context structure
 */
typedef struct
{
    SHA2_Handle     handle;     /*!< A handle that is returned by the SHA driver  */
    SHA2_Config     config;     /*!< structure containing SHA2 driver specific implementation  */
    SHA2_Object     object;     /*!< Pointer to a driver specific data object */
}
mbedtls_sha256_context;


/**
 * \brief          This function initializes a SHA-256 context.
 *
 * \param ctx      The SHA-256 context to initialize.
 */
void mbedtls_sha256_init( mbedtls_sha256_context *ctx );

/**
 * \brief          This function clears a SHA-256 context.
 *
 * \param ctx      The SHA-256 context to clear.
 */
void mbedtls_sha256_free( mbedtls_sha256_context *ctx );

/**
 * \brief          This function clones the state of a SHA-256 context.
 *
 * \param dst      The destination context.
 * \param src      The context to clone.
 */
void mbedtls_sha256_clone( mbedtls_sha256_context *dst,
                           const mbedtls_sha256_context *src );

/**
 * \brief          This function starts a SHA-224 or SHA-256 checksum
 *                 calculation.
 *
 * \param ctx      The context to initialize.
 * \param is224    Determines which function to use.
 *                 <ul><li>0: Use SHA-256.</li>
 *                 <li>1: Use SHA-224.</li></ul>
 *
 * \return         \c 0 on success.
 */
int mbedtls_sha256_starts_ret( mbedtls_sha256_context *ctx, int is224 );

/**
 * \brief          This function feeds an input buffer into an ongoing
 *                 SHA-256 checksum calculation.
 *
 * \param ctx      SHA-256 context
 * \param input    buffer holding the data
 * \param ilen     length of the input data
 *
 * \return         \c 0 on success.
 */
int mbedtls_sha256_update_ret( mbedtls_sha256_context *ctx,
                               const unsigned char *input,
                               size_t ilen );

/**
 * \brief          This function finishes the SHA-256 operation, and writes
 *                 the result to the output buffer.
 *
 * \param ctx      The SHA-256 context.
 * \param output   The SHA-224 or SHA-256 checksum result.
 *
 * \return         \c 0 on success.
 */
int mbedtls_sha256_finish_ret( mbedtls_sha256_context *ctx,
                               unsigned char output[32] );

/**
 * \brief          This function processes a single data block within
 *                 the ongoing SHA-256 computation. This function is for
 *                 internal use only.
 *
 * \param ctx      The SHA-256 context.
 * \param data     The buffer holding one block of data.
 *
 * \return         \c 0 on success.
 */
int mbedtls_internal_sha256_process( mbedtls_sha256_context *ctx,
                                     const unsigned char data[64] );

#ifdef __cplusplus
}
#endif
#endif /* MBEDTLS_SHA256_ALT */

#endif /* MBEDTLS_SHA256_ALT_H */

