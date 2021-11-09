/*
 * Copyright (c) 2019, Texas Instruments Incorporated
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

/* UML state chart of this driver. Paste it into http://plantuml.com/

state "Setup" as Setup
state "Processing Data" as Data
state "Finished" as Finished
state "Cancelled" as Cancelled

[*] --> Setup
Setup --> Data
Setup --> Cancelled: cancelled
Data --> Finished: AES_INT_DMA_DATA_OUT
Data --> Cancelled: cancelled
Finished --> [*]
Cancelled --> [*]

Setup: entry: configure DMA

Data: entry: enable AES_INT_DMA_DATA_IN
Data: entry: start DMA (max 4096 bytes)
Data: AES_INT_DMA_DATA_IN [bytesRemaining > 0]: start DMA (max 4096 bytes)
Data: AES_INT_DMA_DATA_IN [0 < bytesRemaining <= 4096]: enable AES_INT_DMA_DATA_OUT
Data: exit: disable AES_INT_DMA_DATA_*

Finished: unlock crypto HW
Finished: callback

Cancelled: unlock crypto HW
Cancelled: callback

note "all states run in HWI context" as N1
note left of Setup
    crypto hardware already locked
    and configured
end note

 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <ti/drivers/dpl/DebugP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>

#include <ti/drivers/aesecb/AESECBMSP432E4.h>
#include <ti/drivers/cryptoutils/sharedresources/CryptoResourceMSP432E4.h>
#include <ti/drivers/cryptoutils/cryptokey/CryptoKey.h>
#include <ti/drivers/power/PowerMSP432E4.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/aes.h)
#include DeviceFamily_constructPath(driverlib/inc/hw_aes.h)
#include DeviceFamily_constructPath(driverlib/interrupt.h)
#include DeviceFamily_constructPath(driverlib/types.h)
#include DeviceFamily_constructPath(inc/msp432.h)

#define AESECB_BLOCK_SIZE_BYTES 16
#define AESECB_UNUSED(value) ((void)(value))

typedef enum {
    AESECB_Event_INT_DATA_IN = AES_INT_DATA_IN,
    AESECB_Event_DMA_CONTEXT_IN = AES_INT_DMA_CONTEXT_IN,
    AESECB_Event_DMA_DATA_IN = AES_INT_DMA_DATA_IN,
    AESECB_Event_DMA_DATA_OUT = AES_INT_DMA_DATA_OUT,
    AESECB_Event_StateEntered = (1 << 30),
} AESECB_Event;

typedef void (*AESECB_StateFunction)(AESECB_Handle, uint32_t);

/* Forward declarations */
static uint32_t floorUint32(uint32_t value, uint32_t divider);

static int_fast16_t AESECB_doOperation(AESECB_Handle handle,
                                          AESECB_Operation *operation,
                                          AESECB_OperationType operationType);
static int_fast16_t AESECB_doPollingMode(AESECB_Operation *operation);

/* Extern globals (board file) */
extern const AESECB_Config AESECB_config[];
extern const uint_least8_t AESECB_count;

/* Static globals */
static volatile bool AESECB_isinitialized = false;
static volatile bool AESECB_operationCancelled = false;

uint32_t floorUint32(uint32_t value, uint32_t divider)
{
    return (value / divider) * divider;
}

void AESECB_init(void)
{
    AESECB_isinitialized = true;
}

AESECB_Handle AESECB_construct(AESECB_Config *config, const AESECB_Params *params)
{
    AESECB_Handle               handle;
    AESECBMSP432E4_Object       *object;
    uint_fast8_t                key;

    handle = config;
    object = handle->object;

    key = HwiP_disable();
    if (!AESECB_isinitialized ||  object->isOpen)
    {
        HwiP_restore(key);
        return NULL;
    }
    object->isOpen = true;
    HwiP_restore(key);

    CryptoResourceMSP432E4_AES_SHA2_incrementRefCount();

    if (params == NULL)
    {
        params = (AESECB_Params *)&AESECB_defaultParams;
    }

    object->returnBehavior = params->returnBehavior;
    object->callbackFxn = params->callbackFxn;
    object->accessTimeout = params->returnBehavior == AESECB_RETURN_BEHAVIOR_BLOCKING ? params->timeout : SemaphoreP_NO_WAIT;
    object->threadSafe = true;

    return handle;
}

void AESECB_close(AESECB_Handle handle)
{
    AESECBMSP432E4_Object         *object;

    DebugP_assert(handle != NULL);

    object = handle->object;
    object->isOpen = false;
    CryptoResourceMSP432E4_AES_SHA2_decrementRefCount();
}

int_fast16_t AESECB_doOperation(AESECB_Handle handle,
                                       AESECB_Operation *operation,
                                       AESECB_OperationType operationType)
{
    DebugP_assert(handle != NULL);
    DebugP_assert(operation != NULL);

    AESECBMSP432E4_Object *object = handle->object;
    int_fast16_t returnStatus = AESECB_STATUS_ERROR;

    if (operation->key == NULL)
    {
        return AESECB_STATUS_ERROR;
    }

    if (operation->key->encoding != CryptoKey_PLAINTEXT)
    {
        return AESECB_STATUS_ERROR;
    }

    if (operation->inputLength > 0 || operation->inputLength % AESECB_BLOCK_SIZE_BYTES)
    {
        if ((operation->input == NULL) || (operation->output == NULL))
        {
            return AESECB_STATUS_ERROR;
        }
    }

    CryptoKey_Plaintext* key = &operation->key->u.plaintext;
    uint32_t keySizeConfig;
    if (key->keyLength == 16)
    {
        keySizeConfig = AES_CFG_KEY_SIZE_128BIT;
    }
    else if (key->keyLength == 24)
    {
        keySizeConfig = AES_CFG_KEY_SIZE_192BIT;
    }
    else if (key->keyLength == 32)
    {
        keySizeConfig = AES_CFG_KEY_SIZE_256BIT;
    }
    else
    {
        return AESECB_STATUS_ERROR;
    }

    /* Acquire lock after all validation checks have been made but before (!)
     * modifying the object. This protects us from altering ongoing transactions.
     */
    if (object->threadSafe) {
        /* All precalculations are in place, request hardware access now. */
        if (CryptoResourceMSP432E4_AES_SHA2_tryLock(object->accessTimeout) != SemaphoreP_OK)
        {
            return AESECB_STATUS_RESOURCE_UNAVAILABLE;
        }
    }

    object->operation     = operation;
    object->operationType = operationType;
    object->returnStatus  = AESECB_STATUS_SUCCESS;
    AESECB_operationCancelled = false;

    AESReset(AES_BASE);
    AESConfigSet(AES_BASE, AES_CFG_MODE_ECB |
               keySizeConfig |
               (operationType == AESECB_OPERATION_TYPE_ENCRYPT ? AES_CFG_DIR_ENCRYPT : AES_CFG_DIR_DECRYPT));

    AESKey1Set(AES_BASE, (uint32_t*)key->keyMaterial, keySizeConfig);
    AESLengthSet(AES_BASE, operation->inputLength);

    returnStatus = AESECB_doPollingMode(operation);

    if (object->threadSafe) {
        CryptoResourceMSP432E4_AES_SHA2_releaseLock();
    }

    if (object->returnBehavior == AESECB_RETURN_BEHAVIOR_CALLBACK)
    {
        object->callbackFxn(handle,
                            returnStatus,
                            operation,
                            operationType);

        /* The actual call to the top-level function will return a success
         * regardless of what the actual outcome of the AES operation is.
         */
        return AESECB_STATUS_SUCCESS;
    }

    return returnStatus;
}

/* Replacement for AESDataProcess in DriverLib without alignment
 * and padding restrictions.
 */
int_fast16_t AESECB_doPollingMode(AESECB_Operation *operation)
{
    uint32_t bytesWritten;

    /* Process data, but only up to the least 16 byte boundary */
    for(bytesWritten = 0; bytesWritten < floorUint32(operation->inputLength, 16); bytesWritten += 16)
    {
        AESDataWrite(AES_BASE, (uint32_t*)&operation->input[bytesWritten]);
        AESDataRead(AES_BASE, (uint32_t*)&operation->output[bytesWritten]);
        if (AESECB_operationCancelled == true)
        {
            return AESECB_STATUS_CANCELED;
        }
    }

    return AESECB_STATUS_SUCCESS;
}

int_fast16_t AESECB_cancelOperation(AESECB_Handle handle)
{
    uint32_t key = HwiP_disable();

    if (AESECB_operationCancelled == true)
    {
        HwiP_restore(key);
        return AESECB_STATUS_ERROR;
    }

    AESECB_operationCancelled = true;

    HwiP_restore(key);
    return AESECB_STATUS_SUCCESS;
}


int_fast16_t AESECB_oneStepEncrypt(AESECB_Handle handle, AESECB_Operation *operationStruct)
{

    return AESECB_doOperation(handle, operationStruct, AESECB_OPERATION_TYPE_ENCRYPT);
}

int_fast16_t AESECB_oneStepDecrypt(AESECB_Handle handle, AESECB_Operation *operationStruct)
{

    return AESECB_doOperation(handle, operationStruct, AESECB_OPERATION_TYPE_DECRYPT);
}

bool AESECB_acquireLock(AESECB_Handle handle, uint32_t timeout) {
    return CryptoResourceMSP432E4_AES_SHA2_tryLock(timeout) == SemaphoreP_OK;
}

void AESECB_releaseLock(AESECB_Handle handle) {
    CryptoResourceMSP432E4_AES_SHA2_releaseLock();
}

void AESECB_enableThreadSafety(AESECB_Handle handle) {
    AESECBMSP432E4_Object *object = (AESECBMSP432E4_Object*)handle->object;

    object->threadSafe = true;
}

void AESECB_disableThreadSafety(AESECB_Handle handle) {
    AESECBMSP432E4_Object *object = (AESECBMSP432E4_Object*)handle->object;

    object->threadSafe = false;
}
