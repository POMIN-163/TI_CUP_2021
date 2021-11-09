/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
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
#include <string.h>

#include <ti/drivers/dpl/DebugP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>

#include <ti/drivers/cryptoutils/sharedresources/CryptoResourceMSP432E4.h>
#include <ti/drivers/power/PowerMSP432E4.h>
#include <ti/drivers/sha2/SHA2MSP432E4.h>

#include <ti/devices/msp432e4/driverlib/shamd5.h>
#include <ti/devices/msp432e4/driverlib/inc/hw_shamd5.h>
#include <ti/devices/msp432e4/driverlib/interrupt.h>
#include <ti/devices/msp432e4/driverlib/types.h>
#include <ti/devices/msp432e4/inc/msp432.h>

#define SHA2_UNUSED(value)    ((void)(value))

typedef enum {
    SHA2_OperationType_SingleStep,
    SHA2_OperationType_MultiStep,
    SHA2_OperationType_Finalize,
} SHA2_OperationType;

static uint32_t floorUint32(uint32_t value, uint32_t divider);
static void SHA2_hwi(uintptr_t arg0);
static void SHA2_backupHwContext(SHA2MSP432E4_Object* object);
static void SHA2_restoreHwContext(SHA2MSP432E4_Object* object);
static int_fast16_t SHA2_writeWordToAccelerator(SHA2MSP432E4_Object *object, uint32_t word);
static int_fast16_t SHA2_processData(SHA2MSP432E4_Object *object);
static int_fast16_t SHA2_execAndWaitForFinished(SHA2_Handle handle);

static const uint32_t *SHA2_data;
static uint32_t SHA2_dataLength;
static void *SHA2_digestOutput;
static SHA2_OperationType SHA2_operationType;

uint32_t floorUint32(uint32_t value, uint32_t divider)
{
    return (value / divider) * divider;
}

/*
 * We don't use this interrupt handler to handle an actual
 * interrupt by the crypto accelerator. It serves us as a
 * trampoline to elevate the execution context up to HWI level
 * when being in callback mode.
 */
void SHA2_hwi(uintptr_t arg0)
{
    SHA2_Handle handle;
    SHA2MSP432E4_Object *object;
    uintptr_t key;
    int_fast16_t result;

    handle = (SHA2_Handle)arg0;
    object = (SHA2MSP432E4_Object *)handle->object;

    SHAMD5IntClear(SHAMD5_BASE, 0xFFFFFFFF);

    result = SHA2_processData(object);

    key = HwiP_disable();
    if ((result == SHA2_STATUS_CANCELED) || (object->operationCanceled))
    {
        object->returnStatus = SHA2_STATUS_CANCELED;
        object->bytesInBuffer  = 0;
        object->bytesProcessed = 0;
    }
    object->operationInProgress = false;
    HwiP_restore(key);

    CryptoResourceMSP432E4_AES_SHA2_releaseLock();

    if (object->callbackFxn != NULL)
    {
        object->callbackFxn(handle, object->returnStatus);
    }
}


/*
 * Nothing to do here since all objects live either in
 * .bss or .data and are initialized by the loader.
 */
void SHA2_init(void)
{
}

SHA2_Handle SHA2_construct(SHA2_Config *config, const SHA2_Params *params)
{
    SHA2_Handle handle;
    SHA2MSP432E4_Object *object;
    uint_fast8_t key;

    handle = (SHA2_Handle)config;
    object = handle->object;

    key = HwiP_disable();

    if (object->isOpen)
    {
        HwiP_restore(key);
        return NULL;
    }

    object->isOpen = true;
    object->operationCanceled = false;
    object->operationInProgress = false;

    HwiP_restore(key);

    if (params == NULL)
    {
        params = &SHA2_defaultParams;
    }

    object->bytesInBuffer  = 0;
    object->bytesProcessed = 0;
    object->returnBehavior = params->returnBehavior;
    object->callbackFxn    = params->callbackFxn;

    if (SHA2_setHashType(handle, params->hashType) != SHA2_STATUS_SUCCESS)
    {
        object->isOpen = false;
        HwiP_restore(key);
        return NULL;
    }

    if (params->returnBehavior == SHA2_RETURN_BEHAVIOR_BLOCKING)
    {
        object->accessTimeout = params->timeout;
    }
    else
    {
        object->accessTimeout = SemaphoreP_NO_WAIT;
    }

    CryptoResourceMSP432E4_AES_SHA2_incrementRefCount();

    return handle;
}

void SHA2_close(SHA2_Handle handle)
{
    SHA2MSP432E4_Object         *object;
    uintptr_t key;

    DebugP_assert(handle != NULL);

    object = handle->object;

    key = HwiP_disable();
    if (object->operationInProgress)
    {
        SHA2_cancelOperation(handle);
    }

    object->isOpen = false;

    HwiP_restore(key);

    CryptoResourceMSP432E4_AES_SHA2_decrementRefCount();
}

/*
 * Writes a data word to the HW accelerator's 64 byte input buffer. When the HW is busy
 * computing another 64 byte block, the function blocks until the HW is ready to read more
 * data. This will only happen when writing more than 64 bytes in one go.
 *
 * Returns SHA2_STATUS_SUCCESS when the word has been successfully written or
 * SHA2_STATUS_CANCELED on user abortion.
 */
int_fast16_t SHA2_writeWordToAccelerator(SHA2MSP432E4_Object *object, uint32_t word)
{
    while((HWREG(SHAMD5_BASE + SHAMD5_O_IRQSTATUS) & SHAMD5_INT_INPUT_READY) == 0)
    {
        if (object->operationCanceled)
        {
            return SHA2_STATUS_CANCELED;
        }
    }

    /*
     * It is sufficient to write into the first register.
     * The content is shifted internally on every write.
     */
    HWREG(SHAMD5_BASE + SHAMD5_O_DATA_0_IN) = word;
    return SHA2_STATUS_SUCCESS;
}

int_fast16_t SHA2_processData(SHA2MSP432E4_Object *object)
{
    uint32_t bytesToWriteFromBuffer;
    uint32_t bytesToWriteFromData;
    uint32_t bytesToMergeFromBuffer;
    uint32_t bytesToMergeFromData;
    uint32_t bytesToCopyFromData;
    uint32_t bytesWritten;
    bool bufferToBeFlushed;
    bool useAccelerator;

    if ((SHA2_operationType == SHA2_OperationType_SingleStep)
            || (SHA2_operationType == SHA2_OperationType_Finalize))
    {
        /*
         * Flush the buffer and write everything from data
         */
        bytesToWriteFromBuffer = (object->bytesInBuffer / 4) * 4;
        bytesToMergeFromBuffer = (object->bytesInBuffer % 4);
        bytesToMergeFromData   = (4 - bytesToMergeFromBuffer) % 4;

        if (SHA2_dataLength >= bytesToMergeFromData)
        {
            bytesToWriteFromData = SHA2_dataLength - bytesToMergeFromData;
        }
        else
        {
            bytesToMergeFromData = SHA2_dataLength;
            bytesToWriteFromData = 0;
        }

        bytesToCopyFromData    = 0;
        bufferToBeFlushed      = true;
        useAccelerator         = true;
    }
    else if ((object->bytesInBuffer + SHA2_dataLength) >= object->blockSize)
    {
        /*
         * Flush buffer + data stream up to block size
         */
        bytesToWriteFromBuffer = (object->bytesInBuffer / 4) * 4;
        bytesToMergeFromBuffer = (object->bytesInBuffer % 4);
        bytesToMergeFromData   = (4 - bytesToMergeFromBuffer) % 4;
        /*
         * The following equation cannot overflow because of above check.
         */
        bytesToWriteFromData   = floorUint32(object->bytesInBuffer + SHA2_dataLength, object->blockSize)
                                 - object->bytesInBuffer - bytesToMergeFromData;
        bytesToCopyFromData    = SHA2_dataLength - bytesToWriteFromData - bytesToMergeFromData;
        bufferToBeFlushed      = true;
        useAccelerator         = true;
    }
    else
    {
        /*
         * Buffer + data does not make up a full block. Just append data to buffer.
         */
        bytesToWriteFromBuffer = 0;
        bytesToMergeFromBuffer = 0;
        bytesToMergeFromData   = 0;
        bytesToWriteFromData   = 0;
        bytesToCopyFromData    = SHA2_dataLength;
        bufferToBeFlushed      = false;
        useAccelerator         = false;
    }

    uint32_t totalBytesToWrite = bytesToWriteFromBuffer
                                 + bytesToMergeFromBuffer
                                 + bytesToMergeFromData
                                 + bytesToWriteFromData;

    /*
     * Lazy initialization of the hardware.
     */
    if (useAccelerator)
    {
        SHAMD5Reset(SHAMD5_BASE);

        uint32_t config = object->hwConfig;

        if ((SHA2_operationType == SHA2_OperationType_SingleStep)
                || (SHA2_operationType == SHA2_OperationType_Finalize))
        {
            config |= SHAMD5_MODE_CLOSE_HASH;
        }

        if (object->bytesProcessed < object->blockSize)
        {
            config |= SHAMD5_MODE_ALGO_CONSTANT;
        }
        else
        {
            SHA2_restoreHwContext(object);
        }

        SHAMD5ConfigSet(SHAMD5_BASE, config);

        /*
         * Wait for the context to be ready before writing the mode.
         */
        while((HWREG(SHAMD5_BASE + SHAMD5_O_IRQSTATUS) & SHAMD5_INT_CONTEXT_READY) ==
              0)
        {
        }

        SHAMD5HashLengthSet(SHAMD5_BASE, totalBytesToWrite);
    }

    /*
     * Write buffer content.
     */
    for (bytesWritten = 0; bytesWritten < bytesToWriteFromBuffer; bytesWritten += 4)
    {
        int_fast16_t result = SHA2_writeWordToAccelerator(object, object->buffer[bytesWritten / 4]);
        if (result == SHA2_STATUS_CANCELED)
        {
            return SHA2_STATUS_CANCELED;
        }
    }

    /*
     * Assemble a full word from the remaining bytes in buffer and the first
     * bytes in data. Write the resulting word to the accelerator
     */
    if (bytesToMergeFromBuffer > 0)
    {
        uint32_t mask;
        uint32_t word;

        mask  = ~(0xFFFFFFFFu << (bytesToMergeFromBuffer * 8));
        word  =  (*SHA2_data) << (bytesToMergeFromBuffer * 8);
        word |= (object->buffer[bytesWritten / 4] & mask);

        int_fast16_t result = SHA2_writeWordToAccelerator(object, word);
        if (result == SHA2_STATUS_CANCELED)
        {
            return SHA2_STATUS_CANCELED;
        }

        SHA2_data = (const uint32_t*)((const uint8_t*)SHA2_data + bytesToMergeFromData);
    }

    if (bufferToBeFlushed)
    {
        object->bytesInBuffer = 0;
    }

    /*
     * Write remaining words from data stream. The accelerator accepts
     * only complete words.
     */
    for (bytesWritten = 0; bytesWritten < bytesToWriteFromData; bytesWritten += 4)
    {
        int_fast16_t result = SHA2_writeWordToAccelerator(object, *SHA2_data);
        if (result == SHA2_STATUS_CANCELED)
        {
            return SHA2_STATUS_CANCELED;
        }
        SHA2_data++;
    }

    /*
     * Any bytes left? Copy them to buffer
     */
    uint8_t *bufferTail = (uint8_t*)&object->buffer[0] + object->bytesInBuffer;
    memcpy(bufferTail, SHA2_data, bytesToCopyFromData);
    object->bytesInBuffer += bytesToCopyFromData;

    if (useAccelerator)
    {
        /*
         * Wait for the digest to be ready.
         */
        if (totalBytesToWrite > 0)
        {
            while((HWREG(SHAMD5_BASE + SHAMD5_O_IRQSTATUS) & SHAMD5_INT_OUTPUT_READY) ==
                  0)
            {
                if (object->operationCanceled)
                {
                    return SHA2_STATUS_CANCELED;
                }
            }
        }

        /*
         * Read the intermediate or final digest
         */
        if (SHA2_operationType == SHA2_OperationType_MultiStep)
        {
            SHA2_backupHwContext(object);
        }
        else
        {
            SHAMD5ResultRead(SHAMD5_BASE, (uint32_t*)SHA2_digestOutput);
            object->bytesProcessed  = 0;
        }
    }

    return SHA2_STATUS_SUCCESS;
}

void SHA2_backupHwContext(SHA2MSP432E4_Object* object)
{
    uint32_t i;
    for(i = 0; i < SHA2_DIGEST_LENGTH_BYTES_256; i += 4)
    {
        object->digest[i / 4] = HWREG(SHAMD5_BASE + SHAMD5_O_IDIGEST_A + i);
    }
    object->bytesProcessed = HWREG(SHAMD5_BASE + SHAMD5_O_DIGEST_COUNT);
}

/*
 * This function must be called before configuring the mode
 */
void SHA2_restoreHwContext(SHA2MSP432E4_Object* object)
{
    uint32_t i;

    /*
     * It may take some time after powering up for the hardware to be ready
     */
    while((HWREG(SHAMD5_BASE + SHAMD5_O_IRQSTATUS) & SHAMD5_INT_CONTEXT_READY) == 0)
    {
    }

    HWREG(SHAMD5_BASE + SHAMD5_O_DIGEST_COUNT) = object->bytesProcessed;

    for(i = 0; i < SHA2_DIGEST_LENGTH_BYTES_256; i += 4)
    {
        HWREG(SHAMD5_BASE + SHAMD5_O_IDIGEST_A + i) = object->digest[i / 4];
    }
}

void SHA2_reset(SHA2_Handle handle)
{
    SHA2MSP432E4_Object *object = (SHA2MSP432E4_Object*)handle->object;

    uint32_t key = HwiP_disable();

    if (object->operationInProgress == true)
    {
        SHA2_cancelOperation(handle);
    }

    object->bytesInBuffer  = 0;
    object->bytesProcessed = 0;

    HwiP_restore(key);

}

int_fast16_t SHA2_cancelOperation(SHA2_Handle handle)
{
    SHA2MSP432E4_Object *object = (SHA2MSP432E4_Object*)handle->object;
    uint32_t key;

    key =  HwiP_disable();

    if ((!object->operationInProgress))
    {
        HwiP_restore(key);
        return SHA2_STATUS_ERROR;
    }

    object->operationCanceled = true;

    HwiP_restore(key);

    return SHA2_STATUS_SUCCESS;
}

int_fast16_t SHA2_addData(SHA2_Handle handle, const void* data, size_t length)
{
    SHA2MSP432E4_Object *object = (SHA2MSP432E4_Object*)handle->object;

    /*
     * All precalculations are in place, request hardware access now.
     */
    if (CryptoResourceMSP432E4_AES_SHA2_tryLock(object->accessTimeout) != SemaphoreP_OK)
    {
        return SHA2_STATUS_RESOURCE_UNAVAILABLE;
    }

    SHA2_data          = data;
    SHA2_dataLength    = length;
    SHA2_operationType = SHA2_OperationType_MultiStep;

    return SHA2_execAndWaitForFinished(handle);
}

int_fast16_t SHA2_finalize(SHA2_Handle handle, void *digest)
{
    SHA2MSP432E4_Object *object = (SHA2MSP432E4_Object*)handle->object;

    if (digest == NULL)
    {
        return SHA2_STATUS_ERROR;
    }

    /*
     * All precalculations are in place, request hardware access now.
     */
    if (CryptoResourceMSP432E4_AES_SHA2_tryLock(object->accessTimeout) != SemaphoreP_OK)
    {
        return SHA2_STATUS_RESOURCE_UNAVAILABLE;
    }

    SHA2_data          = NULL;
    SHA2_dataLength    = 0;
    SHA2_digestOutput  = digest;
    SHA2_operationType = SHA2_OperationType_Finalize;

    return SHA2_execAndWaitForFinished(handle);
}

int_fast16_t SHA2_execAndWaitForFinished(SHA2_Handle handle)
{
    SHA2MSP432E4_Object *object         = (SHA2MSP432E4_Object*)handle->object;
    SHA2MSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;

    /*
     * Internal state needs to be modified atomically
     */
    uintptr_t key = HwiP_disable();
    object->operationCanceled = false;
    object->operationInProgress = true;
    HwiP_restore(key);

    if ((object->returnBehavior == SHA2_RETURN_BEHAVIOR_POLLING)
            || (object->returnBehavior == SHA2_RETURN_BEHAVIOR_BLOCKING))
    {
        int_fast16_t result = SHA2_processData(object);

        uintptr_t key = HwiP_disable();
        if ((result == SHA2_STATUS_CANCELED) || (object->operationCanceled))
        {
            object->returnStatus = SHA2_STATUS_CANCELED;
            object->bytesInBuffer  = 0;
            object->bytesProcessed = 0;
        }
        object->operationInProgress = false;
        object->operationCanceled = false;
        HwiP_restore(key);
        CryptoResourceMSP432E4_AES_SHA2_releaseLock();
    }
    else if (object->returnBehavior == SHA2_RETURN_BEHAVIOR_CALLBACK)
    {
        /*
         * Kick the HWI now. It might be executed immediately or later, depending on
         * the caller context and HWI priority level. Nothing else can disturb
         * the operation because we have locked the hardware.
         */
        CryptoResourceMSP432E4_setHwiCallback(&SHA2_hwi, handle, hwAttrs->intPriority);
        HwiP_post(INT_AES0);
    }

    return object->returnStatus;
}

int_fast16_t SHA2_hashData(SHA2_Handle handle, const void* data, size_t length, void *digest)
{
    SHA2MSP432E4_Object *object         = (SHA2MSP432E4_Object*)handle->object;

    object->bytesProcessed  = 0;
    object->bytesInBuffer   = 0;
    object->returnStatus    = SHA2_STATUS_SUCCESS;

    /*
     * All precalculations are in place, request hardware access now.
     */
    if (CryptoResourceMSP432E4_AES_SHA2_tryLock(object->accessTimeout) != SemaphoreP_OK)
    {
        return SHA2_STATUS_RESOURCE_UNAVAILABLE;
    }

    /*
     * Now it is safe to use static variables in this module.
     * Nobody can interrupt the operation until completion or cancellation.
     */
    SHA2_data          = data;
    SHA2_dataLength    = length;
    SHA2_digestOutput  = digest;
    SHA2_operationType = SHA2_OperationType_SingleStep;

    return SHA2_execAndWaitForFinished(handle);
}

int_fast16_t SHA2_setHashType(SHA2_Handle handle, SHA2_HashType type)
{
    SHA2MSP432E4_Object *object = (SHA2MSP432E4_Object*)handle->object;

    if (object->operationInProgress)
    {
        return SHA2_STATUS_ERROR;
    }

    switch (type)
    {
    case SHA2_HASH_TYPE_224:
        object->hwConfig     = SHAMD5_MODE_ALGO_SHA224;
        object->blockSize  = SHA2_BLOCK_SIZE_BYTES_224;
        break;
    case SHA2_HASH_TYPE_256:
        object->hwConfig     = SHAMD5_MODE_ALGO_SHA256;
        object->blockSize  = SHA2_BLOCK_SIZE_BYTES_256;
        break;
    default:
        return SHA2_STATUS_ERROR;
    }

    return SHA2_STATUS_SUCCESS;
}

