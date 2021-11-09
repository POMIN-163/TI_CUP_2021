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

#include <ti/drivers/aesctr/AESCTRMSP432E4.h>
#include <ti/drivers/cryptoutils/sharedresources/CryptoResourceMSP432E4.h>
#include <ti/drivers/cryptoutils/cryptokey/CryptoKey.h>
#include <ti/drivers/dma/UDMAMSP432E4.h>
#include <ti/drivers/power/PowerMSP432E4.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/aes.h)
#include DeviceFamily_constructPath(driverlib/inc/hw_aes.h)
#include DeviceFamily_constructPath(driverlib/interrupt.h)
#include DeviceFamily_constructPath(driverlib/types.h)
#include DeviceFamily_constructPath(inc/msp432.h)
#include DeviceFamily_constructPath(driverlib/udma.h)

#define AESCTR_BLOCK_SIZE_BYTES 16
#define AESCTR_MAX_DMA_BYTES    4096
#define AESCTR_UNUSED(value) ((void)(value))

typedef enum {
    AESCTR_Event_INT_DATA_IN = AES_INT_DATA_IN,
    AESCTR_Event_DMA_CONTEXT_IN = AES_INT_DMA_CONTEXT_IN,
    AESCTR_Event_DMA_DATA_IN = AES_INT_DMA_DATA_IN,
    AESCTR_Event_DMA_DATA_OUT = AES_INT_DMA_DATA_OUT,
    AESCTR_Event_StateEntered = (1 << 30),
} AESCTR_Event;

typedef void (*AESCTR_StateFunction)(AESCTR_Handle, uint32_t);

/* Forward declarations */
static uint32_t minUint32(uint32_t value1, uint32_t value2);
static uint32_t floorUint32(uint32_t value, uint32_t divider);

static int_fast16_t AESCTR_doOperation(AESCTR_Handle handle,
                                          AESCTR_Operation *operation,
                                          AESCTR_OperationType operationType);
static int_fast16_t AESCTR_doPollingMode(AESCTR_Operation *operation);

static void AESCTR_hwiFxn (uintptr_t arg0);
static void AESCTR_doInitialAndFinalState(AESCTR_Handle handle, uint32_t events);
static void AESCTR_doSetupState(AESCTR_Handle handle, uint32_t events);
static void AESCTR_doDataState(AESCTR_Handle handle, uint32_t events);
static void AESCTR_doFinishedState(AESCTR_Handle handle, uint32_t events);
static void AESCTR_doCancelledState(AESCTR_Handle handle, uint32_t events);

/* Non-public functions required by other drivers */
bool AESCTR_acquireLock(AESCTR_Handle handle, uint32_t timeout);
void AESCTR_releaseLock(AESCTR_Handle handle);
void AESCTR_enableThreadSafety(AESCTR_Handle handle);
void AESCTR_disableThreadSafety(AESCTR_Handle handle);

/* Static globals */
static UDMAMSP432E4_Handle dma;
static bool AESCTR_initialized = false;
static volatile bool AESCTR_operationCancelled = false;
static uint32_t AESCTR_processedDataBytes = 0;
static AESCTR_StateFunction AESCTR_state = AESCTR_doInitialAndFinalState;
static volatile AESCTR_StateFunction AESCTR_nextState = NULL;

uint32_t minUint32(uint32_t value1, uint32_t value2)
{
    if (value1 < value2)
    {
        return value1;
    }
    else
    {
        return value2;
    }
}

uint32_t floorUint32(uint32_t value, uint32_t divider)
{
    return (value / divider) * divider;
}

void AESCTR_hwiFxn(uintptr_t arg0)
{
    AESCTR_Handle handle = (AESCTR_Handle)arg0;

    uint32_t events = AESIntStatus(AES_BASE, true);
    AESIntClear(AES_BASE, events);

    /* This state machine implementation is limited to events
     * injected via the ISR status register. Other events
     * have to be checked manually in every state just like the
     * cancelled flag. */
    AESCTR_state(handle, events);
    while (AESCTR_state != AESCTR_nextState)
    {
        AESCTR_state = AESCTR_nextState;
        AESCTR_state(handle, AESCTR_Event_StateEntered);
    }
}

void AESCTR_init(void)
{
    UDMAMSP432E4_init();

    AESCTR_initialized = true;
}

AESCTR_Handle AESCTR_construct(AESCTR_Config *config, const AESCTR_Params *params)
{
    AESCTR_Handle               handle;
    AESCTRMSP432E4_Object        *object;
    uint_fast8_t                key;

    handle = (AESCTR_Handle)config;
    object = handle->object;

    key = HwiP_disable();
    if (!AESCTR_initialized ||  object->isOpen)
    {
        HwiP_restore(key);
        return NULL;
    }
    object->isOpen = true;
    HwiP_restore(key);

    CryptoResourceMSP432E4_AES_SHA2_incrementRefCount();

    if (params == NULL)
    {
        params = &AESCTR_defaultParams;
    }

    object->returnBehavior = params->returnBehavior;
    object->callbackFxn = params->callbackFxn;
    object->accessTimeout = params->returnBehavior == AESCTR_RETURN_BEHAVIOR_BLOCKING ? params->timeout : SemaphoreP_NO_WAIT;
    object->threadSafe = true;

    return handle;
}

void AESCTR_close(AESCTR_Handle handle)
{
    AESCTRMSP432E4_Object         *object;

    DebugP_assert(handle != NULL);

    object = handle->object;
    object->isOpen = false;
    CryptoResourceMSP432E4_AES_SHA2_decrementRefCount();
}

int_fast16_t AESCTR_doOperation(AESCTR_Handle handle,
                                       AESCTR_Operation *operation,
                                       AESCTR_OperationType operationType)
{
    DebugP_assert(handle != NULL);
    DebugP_assert(operation != NULL);

    AESCTRMSP432E4_Object *object = handle->object;
    AESCTRMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;

    if (operation->key == NULL)
    {
        return AESCTR_STATUS_ERROR;
    }

    if (operation->key->encoding != CryptoKey_PLAINTEXT)
    {
        return AESCTR_STATUS_ERROR;
    }

    if (operation->inputLength > 0)
    {
        if ((operation->input == NULL) || (operation->output == NULL))
        {
            return AESCTR_STATUS_ERROR;
        }
    }

    /* The DMA allows only word-aligned input/output data */
    if (object->returnBehavior != AESCTR_RETURN_BEHAVIOR_POLLING)
    {
        if (operation->inputLength > 0)
        {
            if ((((uint32_t)operation->input % 4) != 0) || (((uint32_t)operation->output % 4) != 0))
            {
                return AESCTR_STATUS_ERROR;
            }
        }
    }

    object->operation     = operation;
    object->operationType = operationType;
    object->returnStatus  = AESCTR_STATUS_SUCCESS;
    AESCTR_operationCancelled = false;

    const CryptoKey_Plaintext *key = &operation->key->u.plaintext;
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
        return AESCTR_STATUS_ERROR;
    }

    if (object->threadSafe) {
        /* All precalculations are in place, request hardware access now. */
        if (CryptoResourceMSP432E4_AES_SHA2_tryLock(object->accessTimeout) != SemaphoreP_OK)
        {
            return AESCTR_STATUS_RESOURCE_UNAVAILABLE;
        }
    }

    CryptoResourceMSP432E4_setHwiCallback(AESCTR_hwiFxn, handle, hwAttrs->intPriority);

    dma = UDMAMSP432E4_open();
    DebugP_assert(dma != NULL);

    AESReset(AES_BASE);
    AESConfigSet(AES_BASE, AES_CFG_MODE_CTR |
                           AES_CFG_CTR_WIDTH_128 |
                           keySizeConfig |
                           (operationType == AESCTR_OPERATION_TYPE_ENCRYPT ? AES_CFG_DIR_ENCRYPT : AES_CFG_DIR_DECRYPT));

    AESIVSet(AES_BASE, (uint32_t*)&operation->initialCounter[0]);
    AESKey1Set(AES_BASE, (uint32_t*)key->keyMaterial, keySizeConfig);
    AESLengthSet(AES_BASE, operation->inputLength);

    if (object->returnBehavior == AESCTR_RETURN_BEHAVIOR_POLLING)
    {
        object->returnStatus = AESCTR_doPollingMode(operation);

        if (object->threadSafe) {
            CryptoResourceMSP432E4_AES_SHA2_releaseLock();
        }

        UDMAMSP432E4_close(dma);
    }
    else if (object->returnBehavior == AESCTR_RETURN_BEHAVIOR_CALLBACK)
    {
        object->returnStatus = AESCTR_STATUS_SUCCESS;

        /* Kick the DMA state machine. */
        AESCTR_nextState = AESCTR_doSetupState;
        HwiP_post(INT_AES0);
    }
    else if (object->returnBehavior == AESCTR_RETURN_BEHAVIOR_BLOCKING)
    {
        object->returnStatus = AESCTR_STATUS_SUCCESS;

        /* Kick the DMA state machine. */
        AESCTR_nextState = AESCTR_doSetupState;
        HwiP_post(INT_AES0);

        /* Wait for the DMA state machine to complete. */
        SemaphoreP_pend(CryptoResourceMSP432E4_AES_SHA2_operationSemaphore, (uint32_t)SemaphoreP_WAIT_FOREVER);
    }
    return object->returnStatus;
}

/* Replacement for AESDataProcess in DriverLib without alignment
 * and padding restrictions.
 */
int_fast16_t AESCTR_doPollingMode(AESCTR_Operation *operation)
{
    uint32_t bytesWritten;

    /* Process data, but only up to the least 16 byte boundary */
    for(bytesWritten = 0; bytesWritten < floorUint32(operation->inputLength, 16); bytesWritten += 16)
    {
        AESDataWrite(AES_BASE, (uint32_t*)&operation->input[bytesWritten]);
        AESDataRead(AES_BASE, (uint32_t*)&operation->output[bytesWritten]);
        if (AESCTR_operationCancelled == true)
        {
            return AESCTR_STATUS_CANCELED;
        }
    }

    /* Process remaining data, but do not write beyond the output buffer boundaries */
    if (bytesWritten < operation->inputLength)
    {
        uint32_t buffer[4];
        AESDataWrite(AES_BASE, (uint32_t*)&operation->input[bytesWritten]);
        AESDataRead(AES_BASE, buffer);
        memcpy(operation->output + bytesWritten, buffer, operation->inputLength - bytesWritten);
        if (AESCTR_operationCancelled == true)
        {
            return AESCTR_STATUS_CANCELED;
        }
    }

    return AESCTR_STATUS_SUCCESS;
}

void AESCTR_doInitialAndFinalState(AESCTR_Handle handle, uint32_t events)
{
    AESCTR_UNUSED(handle);
    AESCTR_UNUSED(events);
}

void AESCTR_doSetupState(AESCTR_Handle handle, uint32_t events)
{
    AESCTR_UNUSED(handle);

    if (events & AESCTR_Event_StateEntered)
    {
        AESCTR_processedDataBytes = 0;

        /* Configure DMA channel for data output */
        uDMAChannelAssign(UDMA_CH15_AES0DOUT);
        uDMAChannelAttributeDisable(UDMA_CH15_AES0DOUT,
                                   UDMA_ATTR_ALTSELECT
                                   | UDMA_ATTR_USEBURST
                                   | UDMA_ATTR_HIGH_PRIORITY
                                   | UDMA_ATTR_REQMASK);

        uDMAChannelControlSet(UDMA_CH15_AES0DOUT | UDMA_PRI_SELECT,
                             UDMA_SIZE_32
                             | UDMA_SRC_INC_NONE
                             | UDMA_DST_INC_32
                             | UDMA_ARB_4
                             | UDMA_SRC_PROT_PRIV);

        /* Configure DMA channel for aad or data input */
        uDMAChannelAssign(UDMA_CH14_AES0DIN);
        uDMAChannelAttributeDisable(UDMA_CH14_AES0DIN, UDMA_ATTR_ALTSELECT
                                                       | UDMA_ATTR_USEBURST
                                                       | UDMA_ATTR_HIGH_PRIORITY
                                                       | UDMA_ATTR_REQMASK);

        uDMAChannelControlSet(UDMA_CH14_AES0DIN | UDMA_PRI_SELECT,
                             UDMA_SIZE_32
                             | UDMA_SRC_INC_32
                             | UDMA_DST_INC_NONE
                             | UDMA_ARB_4
                             | UDMA_DST_PROT_PRIV);

        /*
         * There might be pending interrupts from previous operations. The
         * DMA seems to trigger AES_INT_DMA_DATA_OUT twice. We can't fix that
         * at the end of the previous operation.
         */
        AESIntClear(AES_BASE, AES_INT_DMA_DATA_IN | AES_INT_DMA_DATA_OUT);

        AESCTR_nextState = AESCTR_doDataState;
    }

    if (AESCTR_operationCancelled == true)
    {
        AESCTR_nextState = AESCTR_doCancelledState;
    }
}

void AESCTR_doDataState(AESCTR_Handle handle, uint32_t events)
{
    AESCTRMSP432E4_Object *object = (AESCTRMSP432E4_Object*)handle->object;
    AESCTR_Operation* operation = object->operation;

    if (AESCTR_operationCancelled == true)
    {
        AESDMADisable(AES_BASE, AES_DMA_DATA_IN | AES_DMA_DATA_OUT);
        AESIntDisable(AES_BASE, AES_INT_DMA_DATA_IN | AES_INT_DMA_DATA_OUT);
        AESCTR_nextState = AESCTR_doCancelledState;
        return;
    }

    if (events & AESCTR_Event_StateEntered)
    {
        AESIntEnable(AES_BASE, AES_INT_DMA_DATA_IN);
    }

    if (events & AESCTR_Event_DMA_DATA_IN)
    {
        /*
         * We are using the DATA_IN and not the DATA_OUT interrupt because it usually takes
         * longer to reach this code location than the time between DATA_IN and DATA_OUT.
         * So we expect the operation to be complete. When using the NoRTOS DPL and highly
         * optimized code, this assumption is not always true. Thus, we need to add a
         * barrier that is only rarely executed.
         */
        while (!(AESIntStatus(AES_BASE, false) & AES_INT_DMA_DATA_OUT)) {}
    }

    if (events & (AESCTR_Event_StateEntered | AESCTR_Event_DMA_DATA_IN))
    {
        uint32_t bytesRemaining = operation->inputLength - AESCTR_processedDataBytes;
        uint32_t dmaBytes = minUint32(floorUint32(bytesRemaining, 16), AESCTR_MAX_DMA_BYTES);

        if (dmaBytes > 0)
        {
            uDMAChannelTransferSet(UDMA_CH15_AES0DOUT | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void *)(AES_BASE + AES_O_DATA_IN_0),
                                   (void *)&operation->output[AESCTR_processedDataBytes],
                                   dmaBytes / 4);

            uDMAChannelTransferSet(UDMA_CH14_AES0DIN | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void *)&operation->input[AESCTR_processedDataBytes],
                                   (void *)(AES_BASE + AES_O_DATA_IN_0),
                                   dmaBytes / 4);

            uDMAChannelEnable(UDMA_CH15_AES0DOUT);
            uDMAChannelEnable(UDMA_CH14_AES0DIN);

            AESDMAEnable(AES_BASE, AES_DMA_DATA_IN | AES_DMA_DATA_OUT);
            AESCTR_processedDataBytes += dmaBytes;
        }
        else
        {
            AESIntDisable(AES_BASE, AES_INT_DMA_DATA_IN);
            AESDMADisable(AES_BASE, AES_DMA_DATA_IN | AES_DMA_DATA_OUT);

            if (bytesRemaining > 0)
            {
                /*
                 * The MSP432E4 DMA can only transfer whole words and the AES
                 * accelerator does only accept multiples of 4 words (16 bytes).
                 * If the input data is not a multiple of 16, the DMA would thus
                 * overwrite the memory behind the input. It is quicker to
                 * process remaining data manually in polling mode than setting
                 * up an interrupt. We can start this while the previous crypto
                 * DMA operation is still running because AES_INT_DMA_DATA_IN
                 * has fired.
                 */
                DebugP_assert(bytesRemaining < 16);

                uint32_t buffer[4];
                AESDataWrite(AES_BASE, (uint32_t*)&operation->input[AESCTR_processedDataBytes]);
                AESDataRead(AES_BASE, buffer);
                memcpy(&operation->output[AESCTR_processedDataBytes], buffer, bytesRemaining);
                AESCTR_processedDataBytes += bytesRemaining;
            }

            AESCTR_nextState = AESCTR_doFinishedState;
        }
    }
}

void AESCTR_doFinishedState(AESCTR_Handle handle, uint32_t events)
{
    AESCTRMSP432E4_Object *object = (AESCTRMSP432E4_Object*)handle->object;
    AESCTR_Operation* operation = object->operation;

    if (events & AESCTR_Event_StateEntered)
    {
        object->returnStatus = AESCTR_STATUS_SUCCESS;

        if (object->threadSafe) {
            CryptoResourceMSP432E4_AES_SHA2_releaseLock();
        }

        UDMAMSP432E4_close(dma);

        AESCTR_nextState = AESCTR_doInitialAndFinalState;

        if (object->returnBehavior == AESCTR_RETURN_BEHAVIOR_BLOCKING)
        {
            SemaphoreP_post(CryptoResourceMSP432E4_AES_SHA2_operationSemaphore);
            return;
        }
        else if ((object->returnBehavior == AESCTR_RETURN_BEHAVIOR_CALLBACK) && (object->callbackFxn != NULL))
        {
            object->callbackFxn(handle, object->returnStatus, operation, object->operationType);
        }
    }
}

void AESCTR_doCancelledState(AESCTR_Handle handle, uint32_t events)
{
    AESCTRMSP432E4_Object *object = (AESCTRMSP432E4_Object*)handle->object;
    AESCTR_Operation* operation = object->operation;

    if (events & AESCTR_Event_StateEntered)
    {
        object->returnStatus = AESCTR_STATUS_CANCELED;

        if (object->threadSafe) {
            CryptoResourceMSP432E4_AES_SHA2_releaseLock();
        }

        UDMAMSP432E4_close(dma);

        AESCTR_nextState = AESCTR_doInitialAndFinalState;

        if (object->returnBehavior == AESCTR_RETURN_BEHAVIOR_BLOCKING)
        {
            SemaphoreP_post(CryptoResourceMSP432E4_AES_SHA2_operationSemaphore);
            return;
        }
        else if ((object->returnBehavior == AESCTR_RETURN_BEHAVIOR_CALLBACK) && (object->callbackFxn != NULL))
        {
            // Buffer status variables so that it is safe to start a new crypto operation from within the callback.
            AESCTR_OperationType operationtype = object->operationType;
            object->callbackFxn(handle, AESCTR_STATUS_CANCELED, operation, operationtype);
        }
    }
}

int_fast16_t AESCTR_cancelOperation(AESCTR_Handle handle)
{
    AESCTRMSP432E4_Object *object = (AESCTRMSP432E4_Object*)handle->object;
    uint32_t key = HwiP_disable();

    if ((AESCTR_operationCancelled == true) ||
        (AESCTR_state == AESCTR_doInitialAndFinalState) ||
        (AESCTR_state == AESCTR_doFinishedState))
    {
        HwiP_restore(key);
        return AESCTR_STATUS_ERROR;
    }

    AESCTR_operationCancelled = true;

    if (object->returnBehavior != AESCTR_RETURN_BEHAVIOR_POLLING)
    {
        HwiP_post(INT_AES0);
    }

    HwiP_restore(key);
    return AESCTR_STATUS_SUCCESS;
}


int_fast16_t AESCTR_oneStepEncrypt(AESCTR_Handle handle, AESCTR_Operation *operationStruct)
{

    return AESCTR_doOperation(handle, operationStruct, AESCTR_OPERATION_TYPE_ENCRYPT);
}

int_fast16_t AESCTR_oneStepDecrypt(AESCTR_Handle handle, AESCTR_Operation *operationStruct)
{

    return AESCTR_doOperation(handle, operationStruct, AESCTR_OPERATION_TYPE_DECRYPT);
}

bool AESCTR_acquireLock(AESCTR_Handle handle, uint32_t timeout) {
    return CryptoResourceMSP432E4_AES_SHA2_tryLock(timeout) == SemaphoreP_OK;
}

void AESCTR_releaseLock(AESCTR_Handle handle) {
    CryptoResourceMSP432E4_AES_SHA2_releaseLock();
}

void AESCTR_enableThreadSafety(AESCTR_Handle handle) {
    AESCTRMSP432E4_Object *object = (AESCTRMSP432E4_Object*)handle->object;

    object->threadSafe = true;
}
void AESCTR_disableThreadSafety(AESCTR_Handle handle) {
    AESCTRMSP432E4_Object *object = (AESCTRMSP432E4_Object*)handle->object;

    object->threadSafe = false;
}
