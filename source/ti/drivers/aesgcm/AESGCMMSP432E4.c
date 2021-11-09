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

#include <ti/drivers/aesgcm/AESGCMMSP432E4.h>
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

#define AESGCM_BLOCK_SIZE_BYTES 16
#define AESGCM_MAX_DMA_BYTES    4096
#define AESGCM_UNUSED(value) ((void)(value))

typedef enum {
    AESGCM_Event_INT_DATA_IN = AES_INT_DATA_IN,
    AESGCM_Event_DMA_CONTEXT_IN = AES_INT_DMA_CONTEXT_IN,
    AESGCM_Event_DMA_DATA_IN = AES_INT_DMA_DATA_IN,
    AESGCM_Event_DMA_DATA_OUT = AES_INT_DMA_DATA_OUT,
    AESGCM_Event_StateEntered = (1 << 30),
} AESGCM_Event;

typedef void (*AESGCM_StateFunction)(AESGCM_Handle, uint32_t);

/* Forward declarations */
static uint32_t minUint32(uint32_t value1, uint32_t value2);
static uint32_t ceilUint32(uint32_t value, uint32_t divider);
static uint32_t floorUint32(uint32_t value, uint32_t divider);

static int_fast16_t AESGCM_doOperation(AESGCM_Handle handle,
                                          AESGCM_Operation *operation,
                                          AESGCM_OperationType operationType);
static int_fast16_t AESGCM_doPollingMode(AESGCM_Handle handle, AESGCM_Operation *operation);

static void AESGCM_hwiFxn (uintptr_t arg0);
static void AESGCM_doInitialAndFinalState(AESGCM_Handle handle, uint32_t events);
static void AESGCM_doSetupState(AESGCM_Handle handle, uint32_t events);
static void AESGCM_doAadState(AESGCM_Handle handle, uint32_t events);
static void AESGCM_doDataState(AESGCM_Handle handle, uint32_t events);
static void AESGCM_doFinishedState(AESGCM_Handle handle, uint32_t events);
static void AESGCM_doCancelledState(AESGCM_Handle handle, uint32_t events);

/* Static globals */
static UDMAMSP432E4_Handle dma;
static bool AESGCM_initialized = false;
static volatile bool AESGCM_operationCancelled = false;
static uint32_t AESGCM_processedDataBytes = 0;
static uint32_t AESGCM_processedAuthBytes = 0;
static AESGCM_StateFunction AESGCM_state = AESGCM_doInitialAndFinalState;
static volatile AESGCM_StateFunction AESGCM_nextState = NULL;

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

/*
 *  ======== AESGCM_hwiFxn ========
 */
void AESGCM_hwiFxn(uintptr_t arg0)
{
    AESGCM_Handle handle = (AESGCM_Handle)arg0;

    uint32_t events = AESIntStatus(AES_BASE, true);
    AESIntClear(AES_BASE, events);

    /* This state machine implementation is limited to events
     * injected via the ISR status register. Other events
     * have to be checked manually in every state just like the
     * cancelled flag. */
    AESGCM_state(handle, events);
    while (AESGCM_state != AESGCM_nextState)
    {
        AESGCM_state = AESGCM_nextState;
        AESGCM_state(handle, AESGCM_Event_StateEntered);
    }
}

/*
 *  ======== AESGCM_init ========
 */
void AESGCM_init(void)
{
    UDMAMSP432E4_init();

    AESGCM_initialized = true;
}

/*
 *  ======== AESGCM_construct ========
 */
AESGCM_Handle AESGCM_construct(AESGCM_Config *config, const AESGCM_Params *params)
{
    AESGCM_Handle               handle;
    AESGCMMSP432E4_Object        *object;
    uint_fast8_t                key;

    handle = (AESGCM_Handle)config;
    object = handle->object;

    key = HwiP_disable();
    if (!AESGCM_initialized ||  object->isOpen)
    {
        HwiP_restore(key);
        return NULL;
    }
    object->isOpen = true;
    HwiP_restore(key);

    CryptoResourceMSP432E4_AES_SHA2_incrementRefCount();

    if (params == NULL)
    {
        params = (AESGCM_Params *)&AESGCM_defaultParams;
    }

    object->returnBehavior = params->returnBehavior;
    object->callbackFxn = params->callbackFxn;
    object->accessTimeout = params->returnBehavior == AESGCM_RETURN_BEHAVIOR_BLOCKING ? params->timeout : SemaphoreP_NO_WAIT;

    return handle;
}

/*
 *  ======== AESGCM_close ========
 */
void AESGCM_close(AESGCM_Handle handle)
{
    AESGCMMSP432E4_Object         *object;

    DebugP_assert(handle != NULL);

    object = handle->object;
    object->isOpen = false;
    CryptoResourceMSP432E4_AES_SHA2_decrementRefCount();
}

/*
 *  ======== AESGCM_doOperation ========
 */
static int_fast16_t AESGCM_doOperation(AESGCM_Handle handle,
                                       AESGCM_Operation *operation,
                                       AESGCM_OperationType operationType)
{
    DebugP_assert(handle != NULL);
    DebugP_assert(operation != NULL);

    AESGCMMSP432E4_Object *object = handle->object;
    AESGCMMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;

    if (operation->key == NULL)
    {
        return AESGCM_STATUS_ERROR;
    }

    if (operation->key->encoding != CryptoKey_PLAINTEXT)
    {
        return AESGCM_STATUS_ERROR;
    }

    if ((operation->aadLength > 0) && (operation->aad == NULL))
    {
        return AESGCM_STATUS_ERROR;
    }

    if (operation->inputLength > 0)
    {
        if ((operation->input == NULL) || (operation->output == NULL))
        {
            return AESGCM_STATUS_ERROR;
        }
    }

    if (operation->mac == NULL)
    {
        return AESGCM_STATUS_ERROR;
    }

    /* The DMA allows only word-aligned input/output data */
    if (object->returnBehavior != AESGCM_RETURN_BEHAVIOR_POLLING)
    {
        if ((operation->aadLength > 0) && ((uint32_t)operation->aad % 4) != 0)
        {
            return AESGCM_STATUS_ERROR;
        }

        if (operation->inputLength > 0)
        {
            if ((((uint32_t)operation->input % 4) != 0) || (((uint32_t)operation->output % 4) != 0))
            {
                return AESGCM_STATUS_ERROR;
            }
        }
    }

    object->operation     = operation;
    object->operationType = operationType;
    object->returnStatus  = AESGCM_STATUS_SUCCESS;
    AESGCM_operationCancelled = false;

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
        return AESGCM_STATUS_ERROR;
    }

    /*
     * 16 bytes is NIST recommendation, but less is also ok.
     */
    if (operation->macLength > 16)
    {
        return AESGCM_STATUS_ERROR;
    }

    /*
     * 12 bytes is NIST recommendation and implementing
     * anything else would be a considerable larger effort.
     */
    uint32_t counterConfig;
    if (operation->ivLength == 12)
    {
        counterConfig = AES_CFG_CTR_WIDTH_32;
    }
    else
    {
        return AESGCM_STATUS_ERROR;
    }

    /* Internal generation not supported */
    if (operation->ivInternallyGenerated)
    {
        return AESGCM_STATUS_ERROR;

    }

    /* All precalculations are in place, request hardware access now. */
    if (CryptoResourceMSP432E4_AES_SHA2_tryLock(object->accessTimeout) != SemaphoreP_OK)
    {
        return AESGCM_STATUS_RESOURCE_UNAVAILABLE;
    }

    CryptoResourceMSP432E4_setHwiCallback(AESGCM_hwiFxn, handle, hwAttrs->intPriority);

    dma = UDMAMSP432E4_open();
    DebugP_assert(dma != NULL);

    AESReset(AES_BASE);

    AESConfigSet(AES_BASE, AES_CFG_MODE_GCM_HY0CALC
                 | keySizeConfig
                 | counterConfig
                 | ((operationType == AESGCM_OPERATION_TYPE_ENCRYPT) ? AES_CFG_DIR_ENCRYPT : AES_CFG_DIR_DECRYPT)
    );

    /*
     * Write the IV in the most efficient way instead of shuffling it around
     */
    HWREG(AES_BASE + AES_O_IV_IN_0) = ((uint32_t *)operation->iv)[0];
    HWREG(AES_BASE + AES_O_IV_IN_1) = ((uint32_t *)operation->iv)[1];
    HWREG(AES_BASE + AES_O_IV_IN_2) = ((uint32_t *)operation->iv)[2];
    HWREG(AES_BASE + AES_O_IV_IN_3) = 0x01000000;

    AESKey1Set(AES_BASE, (uint32_t*)key->keyMaterial, keySizeConfig);
    AESAuthLengthSet(AES_BASE, operation->aadLength);
    AESLengthSet(AES_BASE, operation->inputLength);

    if (object->returnBehavior == AESGCM_RETURN_BEHAVIOR_POLLING)
    {
        object->returnStatus = AESGCM_doPollingMode(handle, operation);

        CryptoResourceMSP432E4_AES_SHA2_releaseLock();
        UDMAMSP432E4_close(dma);
    }
    else if (object->returnBehavior == AESGCM_RETURN_BEHAVIOR_CALLBACK)
    {
        object->returnStatus = AESGCM_STATUS_SUCCESS;

        /* Kick the DMA state machine. */
        AESGCM_nextState = AESGCM_doSetupState;
        HwiP_post(INT_AES0);

        /* We must return SUCCESS directly because object->returnStatus may
        have changed if the callback preempts us and executes very quickly */
        return AESGCM_STATUS_SUCCESS;
    }
    else if (object->returnBehavior == AESGCM_RETURN_BEHAVIOR_BLOCKING)
    {
        object->returnStatus = AESGCM_STATUS_SUCCESS;

        /* Kick the DMA state machine. */
        AESGCM_nextState = AESGCM_doSetupState;
        HwiP_post(INT_AES0);

        /* Wait for the DMA state machine to complete. */
        SemaphoreP_pend(CryptoResourceMSP432E4_AES_SHA2_operationSemaphore, (uint32_t)SemaphoreP_WAIT_FOREVER);
    }
    return object->returnStatus;
}

/*
 *  ======== AESGCM_doPollingMode ========
 * Replacement for AESDataProcessAuth in DriverLib without alignment
 * and padding restrictions.
 */
int_fast16_t AESGCM_doPollingMode(AESGCM_Handle handle, AESGCM_Operation *operation)
{
    AESGCMMSP432E4_Object *object = (AESGCMMSP432E4_Object*)handle->object;

    uint32_t bytesWritten;
    for(bytesWritten = 0; bytesWritten < operation->aadLength; bytesWritten += 16)
    {
        AESDataWrite(AES_BASE, (uint32_t*)(operation->aad + bytesWritten));

        if (AESGCM_operationCancelled == true)
        {
            return AESGCM_STATUS_CANCELED;
        }
    }

    /* Process data, but only up to the least 16 byte boundary */
    for(bytesWritten = 0; bytesWritten < floorUint32(operation->inputLength, 16); bytesWritten += 16)
    {
        AESDataWrite(AES_BASE, (uint32_t*)&operation->input[bytesWritten]);
        AESDataRead(AES_BASE, (uint32_t*)&operation->output[bytesWritten]);

        if (AESGCM_operationCancelled == true)
        {
            return AESGCM_STATUS_CANCELED;
        }
    }

    /* Process remaining data, but do not write beyond the output buffer boundaries */
    if (bytesWritten < operation->inputLength)
    {
        uint32_t buffer[4];
        AESDataWrite(AES_BASE, (uint32_t*)&operation->input[bytesWritten]);
        AESDataRead(AES_BASE, buffer);
        memcpy(operation->output + bytesWritten, buffer, operation->inputLength - bytesWritten);

        if (AESGCM_operationCancelled == true)
        {
            return AESGCM_STATUS_CANCELED;
        }
    }

    uint32_t tag[4];
    AESTagRead(AES_BASE, tag);

    /* When encrypting, copy the authentication tag as MAC to the target buffer */
    if (object->operationType == AESGCM_OPERATION_TYPE_ENCRYPT)
    {
        memcpy(operation->mac, tag, operation->macLength);
    }
    /* When decypting, verify that the provided MAC matches the calculated tag */
    else if (CryptoUtils_buffersMatch(operation->mac,
                                      tag,
                                      operation->macLength) == false)
    {
        return AESGCM_STATUS_MAC_INVALID;
    }

    return AESGCM_STATUS_SUCCESS;
}

/*
 *  ======== AESGCM_doInitialAndFinalState ========
 */
void AESGCM_doInitialAndFinalState(AESGCM_Handle handle, uint32_t events)
{
    AESGCM_UNUSED(handle);
    AESGCM_UNUSED(events);
}

/*
 *  ======== AESGCM_doSetupState ========
 */
void AESGCM_doSetupState(AESGCM_Handle handle, uint32_t events)
{
    AESGCMMSP432E4_Object *object = (AESGCMMSP432E4_Object*)handle->object;
    AESGCM_Operation* operation = object->operation;

    if (events & AESGCM_Event_StateEntered)
    {
        AESGCM_processedDataBytes = 0;
        AESGCM_processedAuthBytes = 0;

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
            AESGCM_nextState = AESGCM_doAadState;
        }
        else
        {
            AESGCM_nextState = AESGCM_doDataState;
        }
    }

    if (AESGCM_operationCancelled == true)
    {
        AESGCM_nextState = AESGCM_doCancelledState;
    }
}

/*
 *  ======== AESGCM_doAadState ========
 */
void AESGCM_doAadState(AESGCM_Handle handle, uint32_t events)
{
    AESGCMMSP432E4_Object *object = (AESGCMMSP432E4_Object*)handle->object;
    AESGCM_Operation* operation = object->operation;

    if (AESGCM_operationCancelled == true)
    {
        AESDMADisable(AES_BASE, AES_DMA_DATA_IN);
        AESIntDisable(AES_BASE, AES_INT_DMA_DATA_IN);
        AESGCM_nextState = AESGCM_doCancelledState;
        return;
    }

    if (events & AESGCM_Event_StateEntered)
    {
        AESIntEnable(AES_BASE, AES_INT_DMA_DATA_IN);
    }

    if (events & (AESGCM_Event_StateEntered | AESGCM_Event_DMA_DATA_IN))
    {
        uint32_t bytesRemaining = operation->aadLength - AESGCM_processedAuthBytes;

        if (bytesRemaining > 0)
        {
            uint32_t dmaBytes = minUint32(bytesRemaining, AESGCM_MAX_DMA_BYTES);

            uDMAChannelTransferSet(UDMA_CH14_AES0DIN | UDMA_PRI_SELECT,
                                      UDMA_MODE_BASIC, (void *)&operation->aad[AESGCM_processedDataBytes],
                                      (void *)(AES_BASE + AES_O_DATA_IN_0),
                                      ceilUint32(dmaBytes, 16) / 4);
            uDMAChannelEnable(UDMA_CH14_AES0DIN);
            AESDMAEnable(AES_BASE, AES_DMA_DATA_IN);
            AESGCM_processedAuthBytes += dmaBytes;
        }
        else
        {
            AESIntDisable(AES_BASE, AES_INT_DMA_DATA_IN);

            if (operation->inputLength > 0)
            {
                AESGCM_nextState = AESGCM_doDataState;
            }
            else
            {
                AESGCM_nextState = AESGCM_doFinishedState;
            }
        }
    }
}

/*
 *  ======== AESGCM_doDataState ========
 */
void AESGCM_doDataState(AESGCM_Handle handle, uint32_t events)
{
    AESGCMMSP432E4_Object *object = (AESGCMMSP432E4_Object*)handle->object;
    AESGCM_Operation* operation = object->operation;

    if (AESGCM_operationCancelled == true)
    {
        AESDMADisable(AES_BASE, AES_DMA_DATA_IN | AES_DMA_DATA_OUT);
        AESIntDisable(AES_BASE, AES_INT_DMA_DATA_IN | AES_INT_DMA_DATA_OUT);
        AESGCM_nextState = AESGCM_doCancelledState;
        return;
    }

    if (events & AESGCM_Event_StateEntered)
    {
        AESIntEnable(AES_BASE, AES_INT_DMA_DATA_IN);
    }

    if (events & AESGCM_Event_DMA_DATA_IN)
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

    if (events & (AESGCM_Event_StateEntered | AESGCM_Event_DMA_DATA_IN))
    {
        uint32_t bytesRemaining = operation->inputLength - AESGCM_processedDataBytes;
        uint32_t dmaBytes = minUint32(floorUint32(bytesRemaining, 16), AESGCM_MAX_DMA_BYTES);

        if (dmaBytes > 0)
        {
            uDMAChannelTransferSet(UDMA_CH15_AES0DOUT | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void *)(AES_BASE + AES_O_DATA_IN_0),
                                   (void *)&operation->output[AESGCM_processedDataBytes],
                                   dmaBytes / 4);

            uDMAChannelTransferSet(UDMA_CH14_AES0DIN | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void *)&operation->input[AESGCM_processedDataBytes],
                                   (void *)(AES_BASE + AES_O_DATA_IN_0),
                                   dmaBytes / 4);

            uDMAChannelEnable(UDMA_CH15_AES0DOUT);
            uDMAChannelEnable(UDMA_CH14_AES0DIN);

            AESDMAEnable(AES_BASE, AES_DMA_DATA_IN | AES_DMA_DATA_OUT);
            AESGCM_processedDataBytes += dmaBytes;
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
                 * up an interrupt.
                 */
                DebugP_assert(bytesRemaining < 16);

                uint32_t buffer[4];
                AESDataWrite(AES_BASE, (uint32_t*)&operation->input[AESGCM_processedDataBytes]);
                AESDataRead(AES_BASE, buffer);
                memcpy(&operation->output[AESGCM_processedDataBytes], buffer, bytesRemaining);
                AESGCM_processedDataBytes += bytesRemaining;
            }

            AESGCM_nextState = AESGCM_doFinishedState;
        }
    }
}

/*
 *  ======== AESGCM_doFinishedState ========
 */
void AESGCM_doFinishedState(AESGCM_Handle handle, uint32_t events)
{
    AESGCMMSP432E4_Object *object = (AESGCMMSP432E4_Object*)handle->object;
    AESGCM_Operation* operation = object->operation;

    if (events & AESGCM_Event_StateEntered)
    {
        object->returnStatus = AESGCM_STATUS_SUCCESS;

        uint32_t tag[4];
        AESTagRead(AES_BASE, tag);

        /* When encrypting, copy the authentication tag as MAC to the target buffer */
        if (object->operationType == AESGCM_OPERATION_TYPE_ENCRYPT)
        {
            memcpy(operation->mac, tag, operation->macLength);
        }
        /* When decypting, verify that the provided MAC matches the calculated tag */
        else if (memcmp(operation->mac, tag, operation->macLength) != 0)
        {
            object->returnStatus = AESGCM_STATUS_MAC_INVALID;
        }

        CryptoResourceMSP432E4_AES_SHA2_releaseLock();

        UDMAMSP432E4_close(dma);

        AESGCM_nextState = AESGCM_doInitialAndFinalState;

        if (object->returnBehavior == AESGCM_RETURN_BEHAVIOR_BLOCKING)
        {
            SemaphoreP_post(CryptoResourceMSP432E4_AES_SHA2_operationSemaphore);
            return;
        }
        else if ((object->returnBehavior == AESGCM_RETURN_BEHAVIOR_CALLBACK) && (object->callbackFxn != NULL))
        {
            object->callbackFxn(handle, object->returnStatus, operation, object->operationType);
        }
    }
}

/*
 *  ======== AESGCM_doCancelledState ========
 */
void AESGCM_doCancelledState(AESGCM_Handle handle, uint32_t events)
{
    AESGCMMSP432E4_Object *object = (AESGCMMSP432E4_Object*)handle->object;
    AESGCM_Operation* operation = object->operation;

    if (events & AESGCM_Event_StateEntered)
    {
        object->returnStatus = AESGCM_STATUS_CANCELED;
        CryptoResourceMSP432E4_AES_SHA2_releaseLock();

        UDMAMSP432E4_close(dma);

        AESGCM_nextState = AESGCM_doInitialAndFinalState;

        if (object->returnBehavior == AESGCM_RETURN_BEHAVIOR_BLOCKING)
        {
            SemaphoreP_post(CryptoResourceMSP432E4_AES_SHA2_operationSemaphore);
            return;
        }
        else if ((object->returnBehavior == AESGCM_RETURN_BEHAVIOR_CALLBACK) && (object->callbackFxn != NULL))
        {
            // Buffer status variables so that it is safe to start a new crypto operation from within the callback.
            AESGCM_OperationType operationtype = object->operationType;
            object->callbackFxn(handle, AESGCM_STATUS_CANCELED, operation, operationtype);
        }
    }
}

/*
 *  ======== AESGCM_cancelOperation ========
 */
int_fast16_t AESGCM_cancelOperation(AESGCM_Handle handle)
{
    AESGCMMSP432E4_Object *object = (AESGCMMSP432E4_Object*)handle->object;
    uint32_t key = HwiP_disable();

    if ((AESGCM_operationCancelled == true)
            || (AESGCM_state == AESGCM_doInitialAndFinalState)
            || (AESGCM_state == AESGCM_doFinishedState))
    {
        HwiP_restore(key);
        return AESGCM_STATUS_ERROR;
    }

    AESGCM_operationCancelled = true;

    if (object->returnBehavior != AESGCM_RETURN_BEHAVIOR_POLLING)
    {
        HwiP_post(INT_AES0);
    }

    HwiP_restore(key);
    return AESGCM_STATUS_SUCCESS;
}

/*
 *  ======== AESGCM_oneStepEncrypt ========
 */
int_fast16_t AESGCM_oneStepEncrypt(AESGCM_Handle handle, AESGCM_Operation *operationStruct)
{

    return AESGCM_doOperation(handle, operationStruct, AESGCM_OPERATION_TYPE_ENCRYPT);
}

/*
 *  ======== AESGCM_oneStepDecrypt ========
 */
int_fast16_t AESGCM_oneStepDecrypt(AESGCM_Handle handle, AESGCM_Operation *operationStruct)
{

    return AESGCM_doOperation(handle, operationStruct, AESGCM_OPERATION_TYPE_DECRYPT);
}
