/*
 * Copyright (c) 2018-2019 Texas Instruments Incorporated
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

#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/cryptoutils/sharedresources/CryptoResourceMSP432E4.h>
#include <ti/drivers/dpl/DebugP.h>
#include <ti/drivers/power/PowerMSP432E4.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/msp432.h)
#include DeviceFamily_constructPath(driverlib/interrupt.h)

SemaphoreP_Handle CryptoResourceMSP432E4_AES_SHA2_operationSemaphore;
SemaphoreP_Handle CryptoResourceMSP432E4_CRC_operationSemaphore;

static SemaphoreP_Handle CryptoResourceMSP432E4_AES_SHA2_accessSemaphore;
static SemaphoreP_Handle CryptoResourceMSP432E4_CRC_accessSemaphore;

static uint32_t CryptoResourceMSP432E4_AES_SHA2_refCount;
static uint32_t CryptoResourceMSP432E4_CRC_refCount;

static HwiP_Handle CryptoResourceMSP432E4_hwi;

static bool CryptoResourceMSP432E4_initialized = false;

static void CryptoResourceMSP432E4_errorSpin(uintptr_t arg) {
    (void)arg;
    while(1);
}

extern void CryptoResourceMSP432E4_AES_SHA2_incrementRefCount(void)
{
    uintptr_t key = HwiP_disable();
    if (CryptoResourceMSP432E4_AES_SHA2_refCount == 0)
    {
        HwiP_Params hwiParams;
        HwiP_Params_init(&hwiParams);
        hwiParams.priority = (uint32_t)~0;
        hwiParams.enableInt = true;
        CryptoResourceMSP432E4_hwi = HwiP_create(INT_AES0, CryptoResourceMSP432E4_errorSpin, &hwiParams);

        CryptoResourceMSP432E4_AES_SHA2_accessSemaphore = SemaphoreP_createBinary(1);
        CryptoResourceMSP432E4_AES_SHA2_operationSemaphore = SemaphoreP_createBinary(0);
    }

    /* Power the crypto accelerator only up once at application start
     * and never power it down. */
    if (!CryptoResourceMSP432E4_initialized) {
        Power_setDependency(PowerMSP432E4_PERIPH_CCM0);
        CryptoResourceMSP432E4_initialized = true;
    }

    CryptoResourceMSP432E4_AES_SHA2_refCount++;
    HwiP_restore(key);
}

extern void CryptoResourceMSP432E4_AES_SHA2_decrementRefCount(void)
{
    uintptr_t key = HwiP_disable();
    CryptoResourceMSP432E4_AES_SHA2_refCount--;

    if (CryptoResourceMSP432E4_AES_SHA2_refCount == 0)
    {
        HwiP_delete(CryptoResourceMSP432E4_hwi);
        SemaphoreP_delete(CryptoResourceMSP432E4_AES_SHA2_accessSemaphore);
        SemaphoreP_delete(CryptoResourceMSP432E4_AES_SHA2_operationSemaphore);
    }

    HwiP_restore(key);
}

extern SemaphoreP_Status CryptoResourceMSP432E4_AES_SHA2_tryLock(uint32_t timeout)
{
    return SemaphoreP_pend(CryptoResourceMSP432E4_AES_SHA2_accessSemaphore, timeout);
}

extern void CryptoResourceMSP432E4_AES_SHA2_releaseLock(void)
{
    SemaphoreP_post(CryptoResourceMSP432E4_AES_SHA2_accessSemaphore);
}

extern void CryptoResourceMSP432E4_CRC_incrementRefCount(void)
{
    uintptr_t key = HwiP_disable();
    if (CryptoResourceMSP432E4_CRC_refCount == 0)
    {
        CryptoResourceMSP432E4_CRC_accessSemaphore = SemaphoreP_createBinary(1);
        CryptoResourceMSP432E4_CRC_operationSemaphore = SemaphoreP_createBinary(0);
    }

    /* Power the crypto accelerator only up once at application start
     * and never power it down. */
    if (!CryptoResourceMSP432E4_initialized) {
        Power_setDependency(PowerMSP432E4_PERIPH_CCM0);
        CryptoResourceMSP432E4_initialized = true;
    }

    CryptoResourceMSP432E4_CRC_refCount++;
    HwiP_restore(key);
}

extern void CryptoResourceMSP432E4_CRC_decrementRefCount(void)
{
    uintptr_t key = HwiP_disable();
    CryptoResourceMSP432E4_CRC_refCount--;

    if (CryptoResourceMSP432E4_CRC_refCount == 0)
    {
        SemaphoreP_delete(CryptoResourceMSP432E4_CRC_operationSemaphore);
        SemaphoreP_delete(CryptoResourceMSP432E4_CRC_accessSemaphore);
    }

    HwiP_restore(key);
}

extern SemaphoreP_Status CryptoResourceMSP432E4_CRC_tryLock(uint32_t timeout)
{
    return SemaphoreP_pend(CryptoResourceMSP432E4_CRC_accessSemaphore, timeout);
}

extern void CryptoResourceMSP432E4_CRC_releaseLock(void)
{
    SemaphoreP_post(CryptoResourceMSP432E4_CRC_accessSemaphore);
}

void CryptoResourceMSP432E4_setHwiCallback(HwiP_Fxn function, void *arg, uint32_t priority)
{
    HwiP_setFunc(CryptoResourceMSP432E4_hwi, function, (uintptr_t)arg);
    HwiP_setPriority(INT_AES0, priority);
}
