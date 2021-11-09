/*
 * Copyright (c) 2018-2020, Texas Instruments Incorporated
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
state "Processing Aad" as Aad
state "Processing Data" as Data
state "Finished" as Finished
state "Cancelled" as Cancelled

state SetupChoice <<choice>>
state AadChoice <<choice>>

[*] --> Setup
Setup --> SetupChoice
Setup --> Cancelled: cancelled
SetupChoice --> Aad: [aadLength > 0]
SetupChoice --> Data: [else]
Aad --> AadChoice: AES_INT_DMA_DATA_IN
Aad --> Cancelled: cancelled
AadChoice --> Data: [plainTextLength > 0]
AadChoice --> Finished: [else]
Data --> Finished: done
Data --> Cancelled: cancelled
Finished --> [*]
Cancelled --> [*]

Setup: entry: configure DMA

Aad: entry: start DMA
Aad: AES_INT_DMA_DATA_IN [bytesRemaining > 4096]: start DMA (max 4096 bytes)
Aad: exit: disable AES_INT_DMA_DATA_IN

Data: entry: enable AES_INT_DMA_DATA_IN
Data: entry: start DMA (max 4096 bytes)
Data: AES_INT_DMA_DATA_IN [bytesRemaining >= 16]: start DMA (max 4096 bytes)
Data: AES_INT_DMA_DATA_IN [else]: read/write data (polling)
Data: exit: disable AES_INT_DMA_DATA_IN

Finished: [ mode == encryption] read tag
Finished: [ mode == decryption] verify tag
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

#include <ti/drivers/aesccm/AESCCMMSP432E4.h>
#include <ti/drivers/cryptoutils/sharedresources/CryptoResourceMSP432E4.h>
#include <ti/drivers/cryptoutils/cryptokey/CryptoKey.h>
#include <ti/drivers/cryptoutils/utils/CryptoUtils.h>
#include <ti/drivers/dma/UDMAMSP432E4.h>
#include <ti/drivers/power/PowerMSP432E4.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/aes.h)
#include DeviceFamily_constructPath(driverlib/inc/hw_aes.h)
#include DeviceFamily_constructPath(driverlib/interrupt.h)
#include DeviceFamily_constructPath(driverlib/types.h)
#include DeviceFamily_constructPath(inc/msp432.h)
#include DeviceFamily_constructPath(driverlib/udma.h)

#define AESCCM_BLOCK_SIZE_BYTES 16
#define AESCCM_MAX_DMA_BYTES    4096
#define AESCCM_UNUSED(value) ((void)(value))

typedef enum {
    AESCCM_Event_INT_DATA_IN = AES_INT_DATA_IN,
    AESCCM_Event_DMA_CONTEXT_IN = AES_INT_DMA_CONTEXT_IN,
    AESCCM_Event_DMA_DATA_IN = AES_INT_DMA_DATA_IN,
    AESCCM_Event_DMA_DATA_OUT = AES_INT_DMA_DATA_OUT,
    AESCCM_Event_StateEntered = (1 << 30),
} AESCCM_Event;

typedef void (*AESCCM_StateFunction)(AESCCM_Handle, uint32_t);

/* Forward declarations */
static uint32_t minUint32(uint32_t value1, uint32_t value2);
static uint32_t ceilUint32(uint32_t value, uint32_t divider);
static uint32_t floorUint32(uint32_t value, uint32_t divider);

static int_fast16_t AESCCM_doOperation(AESCCM_Handle handle,
                                          AESCCM_Operation *operation,
                                          AESCCM_OperationType operationType);
static int_fast16_t AESCCM_doPollingMode(AESCCM_Handle handle, AESCCM_Operation *operation);

static void AESCCM_hwiFxn (uintptr_t arg0);
static void AESCCM_doInitialAndFinalState(AESCCM_Handle handle, uint32_t events);
static void AESCCM_doSetupState(AESCCM_Handle handle, uint32_t events);
static void AESCCM_doAadState(AESCCM_Handle handle, uint32_t events);
static void AESCCM_doDataState(AESCCM_Handle handle, uint32_t events);
static void AESCCM_doFinishedState(AESCCM_Handle handle, uint32_t events);
static void AESCCM_doCancelledState(AESCCM_Handle handle, uint32_t events);

/* Static globals */
static UDMAMSP432E4_Handle dma;
static bool AESCCM_initialized = false;
static volatile bool AESCCM_operationCancelled = false;
static uint32_t AESCCM_processedDataBytes = 0;
static uint32_t AESCCM_processedAuthBytes = 0;
static AESCCM_StateFunction AESCCM_state = AESCCM_doInitialAndFinalState;
static volatile AESCCM_StateFunction AESCCM_nextState = NULL;

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

uint32_t ceilUint32(uint32_t value, uint32_t divider)
{
    uint32_t remainder;

    remainder = value % divider;
    if(remainder == 0)
    {
        return value;
    }
    else
    {
        return (value + (16 - remainder));
    }
}

uint32_t floorUint32(uint32_t value, uint32_t divider)
{
    return (value / divider) * divider;
}

void AESCCM_hwiFxn(uintptr_t arg0)
{
    AESCCM_Handle handle = (AESCCM_Handle)arg0;

    uint32_t events = AESIntStatus(AES_BASE, true);
    AESIntClear(AES_BASE, events);

    /* This state machine implementation is limited to events
     * injected via the ISR status register. Other events
     * have to be checked manually in every state just like the
     * cancelled flag. */
    AESCCM_state(handle, events);
    while (AESCCM_state != AESCCM_nextState)
    {
        AESCCM_state = AESCCM_nextState;
        AESCCM_state(handle, AESCCM_Event_StateEntered);
    }
}

void AESCCM_init(void)
{
    UDMAMSP432E4_init();

    AESCCM_initialized = true;
}

AESCCM_Handle AESCCM_construct(AESCCM_Config *config, const AESCCM_Params *params)
{
    AESCCM_Handle               handle;
    AESCCMMSP432E4_Object        *object;
    uint_fast8_t                key;

    handle = (AESCCM_Handle)config;
    object = handle->object;

    key = HwiP_disable();
    if (!AESCCM_initialized ||  object->isOpen)
    {
        HwiP_restore(key);
        return NULL;
    }
    object->isOpen = true;
    HwiP_restore(key);

    CryptoResourceMSP432E4_AES_SHA2_incrementRefCount();

    if (params == NULL)
    {
        params = (AESCCM_Params *)&AESCCM_defaultParams;
    }

    object->returnBehavior = params->returnBehavior;
    object->callbackFxn = params->callbackFxn;
    object->accessTimeout = params->returnBehavior == AESCCM_RETURN_BEHAVIOR_BLOCKING ? params->timeout : SemaphoreP_NO_WAIT;

    return handle;
}

void AESCCM_close(AESCCM_Handle handle)
{
    AESCCMMSP432E4_Object         *object;

    DebugP_assert(handle != NULL);

    object = handle->object;
    object->isOpen = false;
    CryptoResourceMSP432E4_AES_SHA2_decrementRefCount();
}

static int_fast16_t AESCCM_doOperation(AESCCM_Handle handle,
                                       AESCCM_Operation *operation,
                                       AESCCM_OperationType operationType)
{
    DebugP_assert(handle != NULL);
    DebugP_assert(operation != NULL);

    AESCCMMSP432E4_Object *object = handle->object;
    AESCCMMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;

    if (operation->key == NULL)
    {
        return AESCCM_STATUS_ERROR;
    }

    if (operation->key->encoding != CryptoKey_PLAINTEXT)
    {
        return AESCCM_STATUS_ERROR;
    }

    if ((operation->aadLength > 0) && (operation->aad == NULL))
    {
        return AESCCM_STATUS_ERROR;
    }

    if (operation->inputLength > 0)
    {
        if ((operation->input == NULL) || (operation->output == NULL))
        {
            return AESCCM_STATUS_ERROR;
        }
    }

    if (operation->mac == NULL)
    {
        return AESCCM_STATUS_ERROR;
    }

    /* The DMA allows only word-aligned input/output data */
    if (object->returnBehavior != AESCCM_RETURN_BEHAVIOR_POLLING)
    {
        if ((operation->aadLength > 0) && ((uint32_t)operation->aad % 4) != 0)
        {
            return AESCCM_STATUS_ERROR;
        }

        if (operation->inputLength > 0)
        {
            if ((((uint32_t)operation->input % 4) != 0) || (((uint32_t)operation->output % 4) != 0))
            {
                return AESCCM_STATUS_ERROR;
            }
        }
    }

    object->operation     = operation;
    object->operationType = operationType;
    object->returnStatus  = AESCCM_STATUS_SUCCESS;

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
        return AESCCM_STATUS_ERROR;
    }

    uint32_t mValueConfig;
    if (operation->macLength == 4)
    {
        mValueConfig = AES_CFG_CCM_M_4;
    }
    else if (operation->macLength == 6)
    {
        mValueConfig = AES_CFG_CCM_M_6;
    }
    else if (operation->macLength == 8)
    {
        mValueConfig = AES_CFG_CCM_M_8;
    }
    else if (operation->macLength == 10)
    {
        mValueConfig = AES_CFG_CCM_M_10;
    }
    else if (operation->macLength == 12)
    {
        mValueConfig = AES_CFG_CCM_M_12;
    }
    else if (operation->macLength == 14)
    {
        mValueConfig = AES_CFG_CCM_M_14;
    }
    else if (operation->macLength == 16)
    {
        mValueConfig = AES_CFG_CCM_M_16;
    }
    else
    {
        return AESCCM_STATUS_ERROR;
    }

    uint32_t lValueConfig;
    if (operation->nonceLength == 7)
    {
        lValueConfig = AES_CFG_CCM_L_8;
    }
    else if (operation->nonceLength == 8)
    {
        lValueConfig = AES_CFG_CCM_L_7;
    }
    else if (operation->nonceLength == 9)
    {
        lValueConfig = AES_CFG_CCM_L_6;
    }
    else if (operation->nonceLength == 10)
    {
        lValueConfig = AES_CFG_CCM_L_5;
    }
    else if (operation->nonceLength == 11)
    {
        lValueConfig = AES_CFG_CCM_L_4;
    }
    else if (operation->nonceLength == 12)
    {
        lValueConfig = AES_CFG_CCM_L_3;
    }
    else if (operation->nonceLength == 13)
    {
        lValueConfig = AES_CFG_CCM_L_2;
    }
    else
    {
        return AESCCM_STATUS_ERROR;
    }

    uint8_t iv[16] = {0};
    iv[0] = (15 - operation->nonceLength - 1);
    memcpy(&iv[1], operation->nonce, operation->nonceLength);

    /* All precalculations are in place, request hardware access now. */
    if (CryptoResourceMSP432E4_AES_SHA2_tryLock(object->accessTimeout) != SemaphoreP_OK)
    {
        return AESCCM_STATUS_RESOURCE_UNAVAILABLE;
    }

    CryptoResourceMSP432E4_setHwiCallback(AESCCM_hwiFxn, handle, hwAttrs->intPriority);

    AESCCM_operationCancelled = false;
    dma = UDMAMSP432E4_open();
    DebugP_assert(dma != NULL);

    AESReset(AES_BASE);

    AESConfigSet(AES_BASE, AES_CFG_MODE_CCM
                 | keySizeConfig
                 | lValueConfig
                 | mValueConfig
                 | AES_CFG_CTR_WIDTH_128
                 | ((operationType == AESCCM_OPERATION_TYPE_ENCRYPT) ? AES_CFG_DIR_ENCRYPT : AES_CFG_DIR_DECRYPT)
    );

    AESIVSet(AES_BASE, (uint32_t*)&iv[0]);
    AESKey1Set(AES_BASE, (uint32_t*)key->keyMaterial, keySizeConfig);
    AESAuthLengthSet(AES_BASE, operation->aadLength);
    AESLengthSet(AES_BASE, operation->inputLength);

    if (object->returnBehavior == AESCCM_RETURN_BEHAVIOR_POLLING)
    {
        object->returnStatus = AESCCM_doPollingMode(handle, operation);

        CryptoResourceMSP432E4_AES_SHA2_releaseLock();
        UDMAMSP432E4_close(dma);
    }
    else if (object->returnBehavior == AESCCM_RETURN_BEHAVIOR_CALLBACK)
    {
        object->returnStatus = AESCCM_STATUS_SUCCESS;

        /* Kick the DMA state machine. */
        AESCCM_nextState = AESCCM_doSetupState;
        HwiP_post(INT_AES0);

        /* We must return SUCCESS directly because object->returnStatus may
        have changed if the callback preempts us and executes very quickly */
        return AESCCM_STATUS_SUCCESS;
    }
    else if (object->returnBehavior == AESCCM_RETURN_BEHAVIOR_BLOCKING)
    {
        object->returnStatus = AESCCM_STATUS_SUCCESS;

        /* Kick the DMA state machine. */
        AESCCM_nextState = AESCCM_doSetupState;
        HwiP_post(INT_AES0);

        /* Wait for the DMA state machine to complete. */
        SemaphoreP_pend(CryptoResourceMSP432E4_AES_SHA2_operationSemaphore, (uint32_t)SemaphoreP_WAIT_FOREVER);
    }
    return object->returnStatus;
}

/* Replacement for AESDataProcessAuth in DriverLib without alignment
 * and padding restrictions.
 */
int_fast16_t AESCCM_doPollingMode(AESCCM_Handle handle, AESCCM_Operation *operation)
{
    AESCCMMSP432E4_Object *object = (AESCCMMSP432E4_Object*)handle->object;

    uint32_t bytesWritten;
    for(bytesWritten = 0; bytesWritten < operation->aadLength; bytesWritten += 16)
    {
        AESDataWrite(AES_BASE, (uint32_t*)(operation->aad + bytesWritten));

        if (AESCCM_operationCancelled == true)
        {
            return AESCCM_STATUS_CANCELED;
        }
    }

    /* Process data, but only up to the least 16 byte boundary */
    for(bytesWritten = 0; bytesWritten < floorUint32(operation->inputLength, 16); bytesWritten += 16)
    {
        AESDataWrite(AES_BASE, (uint32_t*)&operation->input[bytesWritten]);
        AESDataRead(AES_BASE, (uint32_t*)&operation->output[bytesWritten]);

        if (AESCCM_operationCancelled == true)
        {
            return AESCCM_STATUS_CANCELED;
        }
    }

    /* Process remaining data, but do not write beyond the output buffer boundaries */
    if (bytesWritten < operation->inputLength)
    {
        uint32_t buffer[4];
        AESDataWrite(AES_BASE, (uint32_t*)&operation->input[bytesWritten]);
        AESDataRead(AES_BASE, buffer);
        memcpy(operation->output + bytesWritten, buffer, operation->inputLength - bytesWritten);

        if (AESCCM_operationCancelled == true)
        {
            return AESCCM_STATUS_CANCELED;
        }
    }

    uint32_t tag[4];
    AESTagRead(AES_BASE, tag);

    /* When encrypting, copy the authentication tag as MAC to the target buffer */
    if (object->operationType == AESCCM_OPERATION_TYPE_ENCRYPT)
    {
        memcpy(operation->mac, tag, operation->macLength);
    }
    /* When decypting, verify that the provided MAC matches the calculated tag */
    else if (CryptoUtils_buffersMatch(operation->mac,
                                      tag,
                                      operation->macLength) == false)
    {
        return AESCCM_STATUS_MAC_INVALID;
    }

    return AESCCM_STATUS_SUCCESS;
}

void AESCCM_doInitialAndFinalState(AESCCM_Handle handle, uint32_t events)
{
    AESCCM_UNUSED(handle);
    AESCCM_UNUSED(events);
}

void AESCCM_doSetupState(AESCCM_Handle handle, uint32_t events)
{
    AESCCMMSP432E4_Object *object = (AESCCMMSP432E4_Object*)handle->object;
    AESCCM_Operation* operation = object->operation;

    if (events & AESCCM_Event_StateEntered)
    {
        AESCCM_processedDataBytes = 0;
        AESCCM_processedAuthBytes = 0;

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

        if (operation->aadLength > 0)
        {
            AESCCM_nextState = AESCCM_doAadState;
        }
        else
        {
            AESCCM_nextState = AESCCM_doDataState;
        }
    }

    if (AESCCM_operationCancelled == true)
    {
        AESCCM_nextState = AESCCM_doCancelledState;
    }
}

void AESCCM_doAadState(AESCCM_Handle handle, uint32_t events)
{
    AESCCMMSP432E4_Object *object = (AESCCMMSP432E4_Object*)handle->object;
    AESCCM_Operation* operation = object->operation;

    if (AESCCM_operationCancelled == true)
    {
        AESDMADisable(AES_BASE, AES_DMA_DATA_IN);
        AESIntDisable(AES_BASE, AES_INT_DMA_DATA_IN);
        AESCCM_nextState = AESCCM_doCancelledState;
        return;
    }

    if (events & AESCCM_Event_StateEntered)
    {
        AESIntEnable(AES_BASE, AES_INT_DMA_DATA_IN);
    }

    if (events & (AESCCM_Event_StateEntered | AESCCM_Event_DMA_DATA_IN))
    {
        uint32_t bytesRemaining = operation->aadLength - AESCCM_processedAuthBytes;

        if (bytesRemaining > 0)
        {
            uint32_t dmaBytes = minUint32(bytesRemaining, AESCCM_MAX_DMA_BYTES);

            uDMAChannelTransferSet(UDMA_CH14_AES0DIN | UDMA_PRI_SELECT,
                                      UDMA_MODE_BASIC, (void *)&operation->aad[AESCCM_processedDataBytes],
                                      (void *)(AES_BASE + AES_O_DATA_IN_0),
                                      ceilUint32(dmaBytes, 16) / 4);
            uDMAChannelEnable(UDMA_CH14_AES0DIN);
            AESDMAEnable(AES_BASE, AES_DMA_DATA_IN);
            AESCCM_processedAuthBytes += dmaBytes;
        }
        else
        {
            AESIntDisable(AES_BASE, AES_INT_DMA_DATA_IN);

            if (operation->inputLength > 0)
            {
                AESCCM_nextState = AESCCM_doDataState;
            }
            else
            {
                AESCCM_nextState = AESCCM_doFinishedState;
            }
        }
    }
}

void AESCCM_doDataState(AESCCM_Handle handle, uint32_t events)
{
    AESCCMMSP432E4_Object *object = (AESCCMMSP432E4_Object*)handle->object;
    AESCCM_Operation* operation = object->operation;

    if (AESCCM_operationCancelled == true)
    {
        AESDMADisable(AES_BASE, AES_DMA_DATA_IN | AES_DMA_DATA_OUT);
        AESIntDisable(AES_BASE, AES_INT_DMA_DATA_IN | AES_INT_DMA_DATA_OUT);
        AESCCM_nextState = AESCCM_doCancelledState;
        return;
    }

    if (events & AESCCM_Event_StateEntered)
    {
        AESIntEnable(AES_BASE, AES_INT_DMA_DATA_IN);
    }

    if (events & AESCCM_Event_DMA_DATA_IN)
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

    if (events & (AESCCM_Event_StateEntered | AESCCM_Event_DMA_DATA_IN))
    {
        uint32_t bytesRemaining = operation->inputLength - AESCCM_processedDataBytes;
        uint32_t dmaBytes = minUint32(floorUint32(bytesRemaining, 16), AESCCM_MAX_DMA_BYTES);

        if (dmaBytes > 0)
        {
            uDMAChannelTransferSet(UDMA_CH15_AES0DOUT | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void *)(AES_BASE + AES_O_DATA_IN_0),
                                   (void *)&operation->output[AESCCM_processedDataBytes],
                                   dmaBytes / 4);

            uDMAChannelTransferSet(UDMA_CH14_AES0DIN | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void *)&operation->input[AESCCM_processedDataBytes],
                                   (void *)(AES_BASE + AES_O_DATA_IN_0),
                                   dmaBytes / 4);

            uDMAChannelEnable(UDMA_CH15_AES0DOUT);
            uDMAChannelEnable(UDMA_CH14_AES0DIN);

            AESDMAEnable(AES_BASE, AES_DMA_DATA_IN | AES_DMA_DATA_OUT);
            AESCCM_processedDataBytes += dmaBytes;
        }
        else
        {
            AESIntDisable(AES_BASE, AES_INT_DMA_DATA_IN);
            AESDMADisable(AES_BASE, AES_DMA_DATA_IN | AES_DMA_DATA_OUT);

            if (bytesRemaining > 0)
            {
                /*
                 * The MSP432E4 DMA can only transfer whole words and the AES accelerator does only accept multiples of 4 words (16 bytes).
                 * If the input data is not a multiple of 16, the DMA would thus overwrite the memory behind the input.
                 * It is quicker to process remaining data manually in polling mode than setting up an interrupt. We can start this
                 * while the previous crypto DMA operation is still running because AES_INT_DMA_DATA_IN has fired.
                 */
                DebugP_assert(bytesRemaining < 16);

                uint32_t buffer[4];
                AESDataWrite(AES_BASE, (uint32_t*)&operation->input[AESCCM_processedDataBytes]);
                AESDataRead(AES_BASE, buffer);
                memcpy(&operation->output[AESCCM_processedDataBytes], buffer, bytesRemaining);
                AESCCM_processedDataBytes += bytesRemaining;
            }

            AESCCM_nextState = AESCCM_doFinishedState;
        }
    }
}

void AESCCM_doFinishedState(AESCCM_Handle handle, uint32_t events)
{
    AESCCMMSP432E4_Object *object = (AESCCMMSP432E4_Object*)handle->object;
    AESCCM_Operation* operation = object->operation;

    if (events & AESCCM_Event_StateEntered)
    {
        object->returnStatus = AESCCM_STATUS_SUCCESS;

        uint32_t tag[4];
        AESTagRead(AES_BASE, tag);

        /* When encrypting, copy the authentication tag as MAC to the target buffer */
        if (object->operationType == AESCCM_OPERATION_TYPE_ENCRYPT)
        {
            memcpy(operation->mac, tag, operation->macLength);
        }
        /* When decypting, verify that the provided MAC matches the calculated tag */
        else if (memcmp(operation->mac, tag, operation->macLength) != 0)
        {
            object->returnStatus = AESCCM_STATUS_MAC_INVALID;
        }

        CryptoResourceMSP432E4_AES_SHA2_releaseLock();

        UDMAMSP432E4_close(dma);

        AESCCM_nextState = AESCCM_doInitialAndFinalState;

        if (object->returnBehavior == AESCCM_RETURN_BEHAVIOR_BLOCKING)
        {
            SemaphoreP_post(CryptoResourceMSP432E4_AES_SHA2_operationSemaphore);
            return;
        }
        else if ((object->returnBehavior == AESCCM_RETURN_BEHAVIOR_CALLBACK) && (object->callbackFxn != NULL))
        {
            object->callbackFxn(handle, object->returnStatus, operation, object->operationType);
        }
    }
}

void AESCCM_doCancelledState(AESCCM_Handle handle, uint32_t events)
{
    AESCCMMSP432E4_Object *object = (AESCCMMSP432E4_Object*)handle->object;
    AESCCM_Operation* operation = object->operation;

    if (events & AESCCM_Event_StateEntered)
    {
        object->returnStatus = AESCCM_STATUS_CANCELED;
        CryptoResourceMSP432E4_AES_SHA2_releaseLock();

        UDMAMSP432E4_close(dma);

        AESCCM_nextState = AESCCM_doInitialAndFinalState;

        if (object->returnBehavior == AESCCM_RETURN_BEHAVIOR_BLOCKING)
        {
            SemaphoreP_post(CryptoResourceMSP432E4_AES_SHA2_operationSemaphore);
            return;
        }
        else if ((object->returnBehavior == AESCCM_RETURN_BEHAVIOR_CALLBACK) && (object->callbackFxn != NULL))
        {
            // Buffer status variables so that it is safe to start a new crypto operation from within the callback.
            AESCCM_OperationType operationtype = object->operationType;
            object->callbackFxn(handle, AESCCM_STATUS_CANCELED, operation, operationtype);
        }
    }
}

int_fast16_t AESCCM_cancelOperation(AESCCM_Handle handle)
{
    AESCCMMSP432E4_Object *object = (AESCCMMSP432E4_Object*)handle->object;
    uint32_t key = HwiP_disable();

    if ((AESCCM_operationCancelled == true)
            || (AESCCM_state == AESCCM_doInitialAndFinalState)
            || (AESCCM_state == AESCCM_doFinishedState))
    {
        HwiP_restore(key);
        return AESCCM_STATUS_ERROR;
    }

    AESCCM_operationCancelled = true;

    if (object->returnBehavior != AESCCM_RETURN_BEHAVIOR_POLLING)
    {
        HwiP_post(INT_AES0);
    }

    HwiP_restore(key);
    return AESCCM_STATUS_SUCCESS;
}

int_fast16_t AESCCM_oneStepEncrypt(AESCCM_Handle handle, AESCCM_Operation *operationStruct)
{

    return AESCCM_doOperation(handle, operationStruct, AESCCM_OPERATION_TYPE_ENCRYPT);
}

int_fast16_t AESCCM_oneStepDecrypt(AESCCM_Handle handle, AESCCM_Operation *operationStruct)
{

    return AESCCM_doOperation(handle, operationStruct, AESCCM_OPERATION_TYPE_DECRYPT);
}
