/*
 * Copyright (c) 2018 Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       CryptoResourceMSP432E4.h
 *
 *  @brief      Shared resources to arbitrate access to the keyStore, AES, and SHA2 engine
 *
 */

#ifndef ti_drivers_cryptoutils_sharedresources_CryptoResourceMSP432E4__include
#define ti_drivers_cryptoutils_sharedresources_CryptoResourceMSP432E4__include

#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>

#ifdef __cplusplus
extern "C" {
#endif

extern SemaphoreP_Handle CryptoResourceMSP432E4_AES_SHA2_operationSemaphore;
extern void CryptoResourceMSP432E4_AES_SHA2_incrementRefCount(void);
extern void CryptoResourceMSP432E4_AES_SHA2_decrementRefCount(void);
extern SemaphoreP_Status CryptoResourceMSP432E4_AES_SHA2_tryLock(uint32_t timeout);
extern void CryptoResourceMSP432E4_AES_SHA2_releaseLock(void);

extern SemaphoreP_Handle CryptoResourceMSP432E4_CRC_operationSemaphore;
extern void CryptoResourceMSP432E4_CRC_incrementRefCount(void);
extern void CryptoResourceMSP432E4_CRC_decrementRefCount(void);
extern SemaphoreP_Status CryptoResourceMSP432E4_CRC_tryLock(uint32_t timeout);
extern void CryptoResourceMSP432E4_CRC_releaseLock(void);

/* This function is shared between the AES and SHA2 domains */
extern void CryptoResourceMSP432E4_setHwiCallback(HwiP_Fxn function, void *arg, uint32_t priority);

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_cryptoutils_sharedresources_CryptoResourceMSP432E4__include */
