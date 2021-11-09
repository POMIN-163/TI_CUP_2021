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

#include <ti/drivers/aescbc/AESCBCMSP432E4.h>
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

#define AESCBC_BLOCK_SIZE_BYTES 16
#define AESCBC_MAX_DMA_BYTES    4096
#define AESCBC_UNUSED(value) ((void)(value))

typedef enum {
    AESCBC_Event_INT_DATA_IN = AES_INT_DATA_IN,
    AESCBC_Event_DMA_CONTEXT_IN = AES_INT_DMA_CONTEXT_IN,
    AESCBC_Event_DMA_DATA_IN = AES_INT_DMA_DATA_IN,
    AESCBC_Event_DMA_DATA_OUT = AES_INT_DMA_DATA_OUT,
    AESCBC_Event_StateEntered = (1 << 30),
} AESCBC_Event;

typedef void (*AESCBC_StateFunction)(AESCBC_Handle, uint32_t);

/* Forward declarations */
static uint32_t minUint32(uint32_t value1, uint32_t value2);
static uint32_t floorUint32(uint32_t value, uint32_t divider);

static int_fast16_t AESCBC_doOperation(AESCBC_Handle handle,
                                          AESCBC_Operation *operation,
                                          AESCBC_OperationType operationType);
static int_fast16_t AESCBC_doPollingMode(AESCBC_Operation *operation);

static void AESCBC_hwiFxn (uintptr_t arg0);
static void AESCBC_doInitialAndFinalState(AESCBC_Handle handle, uint32_t events);
static void AESCBC_doSetupState(AESCBC_Handle handle, uint32_t events);
static void AESCBC_doDataState(AESCBC_Handle handle, uint32_t events);
static void AESCBC_doFinishedState(AESCBC_Handle handle, uint32_t events);
static void AESCBC_doCancelledState(AESCBC_Handle handle, uint32_t events);

/* Static globals */
static UDMAMSP432E4_Handle dma;
static bool AESCBC_initialized = false;
static volatile bool AESCBC_operationCancelled = false;
static uint32_t AESCBC_processedDataBytes = 0;
static AESCBC_StateFunction AESCBC_state = AESCBC_doInitialAndFinalState;
static volatile AESCBC_StateFunction AESCBC_nextState = NULL;

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

void AESCBC_hwiFxn(uintptr_t arg0)
{
    AESCBC_Handle handle = (AESCBC_Handle)arg0;

    uint32_t events = AESIntStatus(AES_BASE, true);
    AESIntClear(AES_BASE, events);

    /* This state machine implementation is limited to events
     * injected via the ISR status register. Other events
     * have to be checked manually in every state just like the
     * cancelled flag. */
    AESCBC_state(handle, events);
    while (AESCBC_state != AESCBC_nextState)
    {
        AESCBC_state = AESCBC_nextState;
        AESCBC_state(handle, AESCBC_Event_StateEntered);
    }
}

void AESCBC_init(void)
{
    UDMAMSP432E4_init();

    AESCBC_initialized = true;
}

AESCBC_Handle AESCBC_construct(AESCBC_Config *config, const AESCBC_Params *params)
{
    AESCBC_Handle               handle;
    AESCBCMSP432E4_Object       *object;
    uint_fast8_t                key;

    handle = config;
    object = handle->object;

    key = HwiP_disable();
    if (!AESCBC_initialized ||  object->isOpen)
    {
        HwiP_restore(key);
        return NULL;
    }
    object->isOpen = true;
    HwiP_restore(key);

    CryptoResourceMSP432E4_AES_SHA2_incrementRefCount();

    if (params == NULL)
    {
        params = (AESCBC_Params *)&AESCBC_defaultParams;
    }

    object->returnBehavior = params->returnBehavior;
    object->callbackFxn = params->callbackFxn;
    object->accessTimeout = params->returnBehavior == AESCBC_RETURN_BEHAVIOR_BLOCKING ? params->timeout : SemaphoreP_NO_WAIT;
    object->threadSafe = true;

    return handle;
}

void AESCBC_close(AESCBC_Handle handle)
{
    AESCBCMSP432E4_Object         *object;

    DebugP_assert(handle != NULL);

    object = handle->object;
    object->isOpen = false;
    CryptoResourceMSP432E4_AES_SHA2_decrementRefCount();
}

int_fast16_t AESCBC_doOperation(AESCBC_Handle handle,
                                       AESCBC_Operation *operation,
                                       AESCBC_OperationType operationType)
{
    DebugP_assert(handle != NULL);
    DebugP_assert(operation != NULL);

    AESCBCMSP432E4_Object *object = handle->object;
    AESCBCMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;

    if (operation->key == NULL)
    {
        return AESCBC_STATUS_ERROR;
    }

    if (operation->key->encoding != CryptoKey_PLAINTEXT)
    {
        return AESCBC_STATUS_ERROR;
    }

    if (operation->inputLength > 0)
    {
        if ((operation->input == NULL) || (operation->output == NULL))
        {
            return AESCBC_STATUS_ERROR;
        }
    }

    /* The DMA allows only word-aligned input/output data */
    if (object->returnBehavior != AESCBC_RETURN_BEHAVIOR_POLLING)
    {
        if (operation->inputLength > 0)
        {
            if ((((uint32_t)operation->input % 4) != 0) || (((uint32_t)operation->output % 4) != 0))
            {
                return AESCBC_STATUS_ERROR;
            }
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
        return AESCBC_STATUS_ERROR;
    }

    /* Acquire lock after all validation checks have been made but before (!)
     * modifying the object. This protects us from altering ongoing transactions.
     */
    if (object->threadSafe == true) {
        /* Request hardware access now. */
        if (CryptoResourceMSP432E4_AES_SHA2_tryLock(object->accessTimeout) != SemaphoreP_OK)
        {
            return AESCBC_STATUS_RESOURCE_UNAVAILABLE;
        }
    }

    object->operation     = operation;
    object->operationType = operationType;
    object->returnStatus  = AESCBC_STATUS_SUCCESS;
    AESCBC_operationCancelled = false;

    AESReset(AES_BASE);
    AESConfigSet(AES_BASE, AES_CFG_MODE_CBC |
                           AES_CTRL_SAVE_CONTEXT |
                           keySizeConfig |
                           (operationType == AESCBC_OPERATION_TYPE_ENCRYPT ? AES_CFG_DIR_ENCRYPT : AES_CFG_DIR_DECRYPT));

    AESIVSet(AES_BASE, (uint32_t*)&operation->iv[0]);
    AESKey1Set(AES_BASE, (uint32_t*)key->keyMaterial, keySizeConfig);
    AESLengthSet(AES_BASE, operation->inputLength);

    if (object->returnBehavior == AESCBC_RETURN_BEHAVIOR_POLLING)
    {
        object->returnStatus = AESCBC_doPollingMode(operation);

        AESIVRead(AES_BASE, (uint32_t *)object->iv);

        if (object->threadSafe == true) {
            CryptoResourceMSP432E4_AES_SHA2_releaseLock();
        }
    }
    else if (object->returnBehavior == AESCBC_RETURN_BEHAVIOR_CALLBACK)
    {
        object->returnStatus = AESCBC_STATUS_SUCCESS;

        CryptoResourceMSP432E4_setHwiCallback(AESCBC_hwiFxn, handle, hwAttrs->intPriority);

        dma = UDMAMSP432E4_open();
        DebugP_assert(dma != NULL);

        /* Kick the DMA state machine. */
        AESCBC_nextState = AESCBC_doSetupState;
        HwiP_post(INT_AES0);
    }
    else if (object->returnBehavior == AESCBC_RETURN_BEHAVIOR_BLOCKING)
    {
        object->returnStatus = AESCBC_STATUS_SUCCESS;

        CryptoResourceMSP432E4_setHwiCallback(AESCBC_hwiFxn, handle, hwAttrs->intPriority);

        dma = UDMAMSP432E4_open();
        DebugP_assert(dma != NULL);

        /* Kick the DMA state machine. */
        AESCBC_nextState = AESCBC_doSetupState;
        HwiP_post(INT_AES0);

        /* Wait for the DMA state machine to complete. */
        SemaphoreP_pend(CryptoResourceMSP432E4_AES_SHA2_operationSemaphore, (uint32_t)SemaphoreP_WAIT_FOREVER);
    }
    return object->returnStatus;
}

/* Replacement for AESDataProcess in DriverLib without alignment
 * and padding restrictions.
 */
int_fast16_t AESCBC_doPollingMode(AESCBC_Operation *operation)
{
    uint32_t bytesWritten;

    /* Process data, but only up to the least 16 byte boundary */
    for(bytesWritten = 0; bytesWritten < floorUint32(operation->inputLength, 16); bytesWritten += 16)
    {
        AESDataWrite(AES_BASE, (uint32_t*)&operation->input[bytesWritten]);
        AESDataRead(AES_BASE, (uint32_t*)&operation->output[bytesWritten]);
        if (AESCBC_operationCancelled == true)
        {
            return AESCBC_STATUS_CANCELED;
        }
    }

    return AESCBC_STATUS_SUCCESS;
}

void AESCBC_doInitialAndFinalState(AESCBC_Handle handle, uint32_t events)
{
    AESCBC_UNUSED(handle);
    AESCBC_UNUSED(events);
}

void AESCBC_doSetupState(AESCBC_Handle handle, uint32_t events)
{
    AESCBC_UNUSED(handle);

    if (events & AESCBC_Event_StateEntered)
    {
        AESCBC_processedDataBytes = 0;

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

        AESCBC_nextState = AESCBC_doDataState;
    }

    if (AESCBC_operationCancelled == true)
    {
        AESCBC_nextState = AESCBC_doCancelledState;
    }
}

void AESCBC_doDataState(AESCBC_Handle handle, uint32_t events)
{
    AESCBCMSP432E4_Object *object = (AESCBCMSP432E4_Object*)handle->object;
    AESCBC_Operation* operation = object->operation;

    if (AESCBC_operationCancelled == true)
    {
        AESDMADisable(AES_BASE, AES_DMA_DATA_IN | AES_DMA_DATA_OUT);
        AESIntDisable(AES_BASE, AES_INT_DMA_DATA_IN | AES_INT_DMA_DATA_OUT);
        AESCBC_nextState = AESCBC_doCancelledState;
        return;
    }

    if (events & AESCBC_Event_StateEntered)
    {
        AESIntEnable(AES_BASE, AES_INT_DMA_DATA_IN);
    }

    if (events & AESCBC_Event_DMA_DATA_IN)
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

    if (events & (AESCBC_Event_StateEntered | AESCBC_Event_DMA_DATA_IN))
    {
        uint32_t bytesRemaining = operation->inputLength - AESCBC_processedDataBytes;
        DebugP_assert(bytesRemaining > 0);

        if (bytesRemaining <= 4096)
        {
            AESIntDisable(AES_BASE, AES_INT_DMA_DATA_IN);
            AESIntEnable(AES_BASE, AES_INT_DMA_DATA_OUT);
        }

        uint32_t dmaBytes = minUint32(bytesRemaining, AESCBC_MAX_DMA_BYTES);
        DebugP_assert((dmaBytes % 16) == 0);

        uDMAChannelTransferSet(UDMA_CH15_AES0DOUT | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC,
                               (void *)(AES_BASE + AES_O_DATA_IN_0),
                               (void *)&operation->output[AESCBC_processedDataBytes],
                               dmaBytes / 4);

        uDMAChannelTransferSet(UDMA_CH14_AES0DIN | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC,
                               (void *)&operation->input[AESCBC_processedDataBytes],
                               (void *)(AES_BASE + AES_O_DATA_IN_0),
                               dmaBytes / 4);

        uDMAChannelEnable(UDMA_CH15_AES0DOUT);
        uDMAChannelEnable(UDMA_CH14_AES0DIN);

        AESDMAEnable(AES_BASE, AES_DMA_DATA_IN | AES_DMA_DATA_OUT);
        AESCBC_processedDataBytes += dmaBytes;
    }

    if (events & AESCBC_Event_DMA_DATA_OUT)
    {
        AESDMADisable(AES_BASE, AES_DMA_DATA_IN | AES_DMA_DATA_OUT);
        AESIntDisable(AES_BASE, AES_INT_DMA_DATA_OUT);
        AESCBC_nextState = AESCBC_doFinishedState;
    }
}

void AESCBC_doFinishedState(AESCBC_Handle handle, uint32_t events)
{
    AESCBCMSP432E4_Object *object = (AESCBCMSP432E4_Object*)handle->object;
    AESCBC_Operation* operation = object->operation;

    if (events & AESCBC_Event_StateEntered)
    {
        object->returnStatus = AESCBC_STATUS_SUCCESS;

        if (object->threadSafe == true) {
            CryptoResourceMSP432E4_AES_SHA2_releaseLock();
        }

        UDMAMSP432E4_close(dma);

        AESCBC_nextState = AESCBC_doInitialAndFinalState;

        AESIVRead(AES_BASE, (uint32_t *)object->iv);

        if (object->returnBehavior == AESCBC_RETURN_BEHAVIOR_BLOCKING)
        {
            SemaphoreP_post(CryptoResourceMSP432E4_AES_SHA2_operationSemaphore);
            return;
        }
        else if ((object->returnBehavior == AESCBC_RETURN_BEHAVIOR_CALLBACK) && (object->callbackFxn != NULL))
        {
            object->callbackFxn(handle, object->returnStatus, operation, object->operationType);
        }
    }
}

void AESCBC_doCancelledState(AESCBC_Handle handle, uint32_t events)
{
    AESCBCMSP432E4_Object *object = (AESCBCMSP432E4_Object*)handle->object;
    AESCBC_Operation* operation = object->operation;

    if (events & AESCBC_Event_StateEntered)
    {
        object->returnStatus = AESCBC_STATUS_CANCELED;

        if (object->threadSafe == true) {
            CryptoResourceMSP432E4_AES_SHA2_releaseLock();
        }

        UDMAMSP432E4_close(dma);

        AESCBC_nextState = AESCBC_doInitialAndFinalState;

        if (object->returnBehavior == AESCBC_RETURN_BEHAVIOR_BLOCKING)
        {
            SemaphoreP_post(CryptoResourceMSP432E4_AES_SHA2_operationSemaphore);
            return;
        }
        else if ((object->returnBehavior == AESCBC_RETURN_BEHAVIOR_CALLBACK) && (object->callbackFxn != NULL))
        {
            // Buffer status variables so that it is safe to start a new crypto operation from within the callback.
            AESCBC_OperationType operationtype = object->operationType;
            object->callbackFxn(handle, AESCBC_STATUS_CANCELED, operation, operationtype);
        }
    }
}

int_fast16_t AESCBC_cancelOperation(AESCBC_Handle handle)
{
    AESCBCMSP432E4_Object *object = (AESCBCMSP432E4_Object*)handle->object;
    uint32_t key = HwiP_disable();

    if ((AESCBC_operationCancelled == true)
            || (AESCBC_state == AESCBC_doInitialAndFinalState)
            || (AESCBC_state == AESCBC_doFinishedState))
    {
        HwiP_restore(key);
        return AESCBC_STATUS_ERROR;
    }

    AESCBC_operationCancelled = true;

    if (object->returnBehavior != AESCBC_RETURN_BEHAVIOR_POLLING)
    {
        HwiP_post(INT_AES0);
    }

    HwiP_restore(key);
    return AESCBC_STATUS_SUCCESS;
}

int_fast16_t AESCBC_getNextIv(AESCBC_Handle handle, uint8_t *iv) {
    AESCBCMSP432E4_Object *object = handle->object;

    memcpy(iv, object->iv, sizeof(object->iv));

    return AESCBC_STATUS_SUCCESS;
}

int_fast16_t AESCBC_oneStepEncrypt(AESCBC_Handle handle, AESCBC_Operation *operationStruct)
{

    return AESCBC_doOperation(handle, operationStruct, AESCBC_OPERATION_TYPE_ENCRYPT);
}

int_fast16_t AESCBC_oneStepDecrypt(AESCBC_Handle handle, AESCBC_Operation *operationStruct)
{

    return AESCBC_doOperation(handle, operationStruct, AESCBC_OPERATION_TYPE_DECRYPT);
}

bool AESCBC_acquireLock(AESCBC_Handle handle, uint32_t timeout) {
    return CryptoResourceMSP432E4_AES_SHA2_tryLock(timeout) == SemaphoreP_OK;
}

void AESCBC_releaseLock(AESCBC_Handle handle) {
    CryptoResourceMSP432E4_AES_SHA2_releaseLock();
}

void AESCBC_enableThreadSafety(AESCBC_Handle handle) {
    AESCBCMSP432E4_Object *object = (AESCBCMSP432E4_Object*)handle->object;

    object->threadSafe = true;
}

void AESCBC_disableThreadSafety(AESCBC_Handle handle) {
    AESCBCMSP432E4_Object *object = (AESCBCMSP432E4_Object*)handle->object;

    object->threadSafe = false;
}
