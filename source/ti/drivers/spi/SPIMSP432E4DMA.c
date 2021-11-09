/*
 * Copyright (c) 2017-2018, Texas Instruments Incorporated
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
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <ti/devices/msp432e4/inc/msp432.h>

#include <ti/devices/msp432e4/driverlib/gpio.h>
#include <ti/devices/msp432e4/driverlib/ssi.h>
#include <ti/devices/msp432e4/driverlib/inc/hw_ssi.h>
#include <ti/devices/msp432e4/driverlib/sysctl.h>
#include <ti/devices/msp432e4/driverlib/udma.h>
#include <ti/devices/msp432e4/driverlib/types.h>

#include <ti/drivers/dma/UDMAMSP432E4.h>
#include <ti/drivers/dpl/ClockP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/gpio/GPIOMSP432E4.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432E4.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPIMSP432E4DMA.h>

#define MAX_DMA_TRANSFER_AMOUNT     (1024)

#define PARAMS_DATASIZE_MIN         (4)
#define PARAMS_DATASIZE_MAX         (16)

void SPIMSP432E4DMA_close(SPI_Handle handle);
int_fast16_t SPIMSP432E4DMA_control(SPI_Handle handle, uint_fast16_t cmd,
    void *arg);
void SPIMSP432E4DMA_init(SPI_Handle handle);
static void SPIMSP432E4DMA_hwiFxn(uintptr_t arg);
SPI_Handle SPIMSP432E4DMA_open(SPI_Handle handle, SPI_Params *params);
bool SPIMSP432E4DMA_transfer(SPI_Handle handle, SPI_Transaction *transaction);
void SPIMSP432E4DMA_transferCancel(SPI_Handle handle);

static void blockingTransferCallback(SPI_Handle handle,
    SPI_Transaction *transaction);
static void configNextTransfer(SPIMSP432E4DMA_Object *object,
    SPIMSP432E4DMA_HWAttrs const *hwAttrs);
static inline uint32_t getDmaRemainingXfers(uint32_t channel);
static uint8_t getPowerMgrId(uint32_t baseAddr);
static void initHw(SPIMSP432E4DMA_Object *object,
    SPIMSP432E4DMA_HWAttrs const *hwAttrs);
static inline void primeTransfer(SPIMSP432E4DMA_Object *object,
    SPIMSP432E4DMA_HWAttrs const *hwAttrs);
static void spiPollingTransfer(SPIMSP432E4DMA_Object *object,
    SPIMSP432E4DMA_HWAttrs const *hwAttrs, SPI_Transaction *transaction);
static inline bool spiBusy(SPIMSP432E4DMA_Object *object,
                           SPIMSP432E4DMA_HWAttrs const *hwAttrs);

/* SPI function table for SPIMSP432E4DMA implementation */
const SPI_FxnTable SPIMSP432E4DMA_fxnTable = {
    SPIMSP432E4DMA_close,
    SPIMSP432E4DMA_control,
    SPIMSP432E4DMA_init,
    SPIMSP432E4DMA_open,
    SPIMSP432E4DMA_transfer,
    SPIMSP432E4DMA_transferCancel
};

static const uint8_t frameFormat[] = {
    SSI_FRF_MOTO_MODE_0,    /* SPI_POL0_PHA0 */
    SSI_FRF_MOTO_MODE_1,    /* SPI_POL0_PHA1 */
    SSI_FRF_MOTO_MODE_2,    /* SPI_POL1_PHA0 */
    SSI_FRF_MOTO_MODE_3,    /* SPI_POL1_PHA1 */
    SSI_FRF_TI              /* SPI_TI */
};

/*
 * These lookup tables are used to configure the DMA channels for the
 * appropriate (8bit or 16bit) transfer sizes.
 */
static const uint32_t dmaTxConfig[] = {
    UDMA_SIZE_8  | UDMA_SRC_INC_8    | UDMA_DST_INC_NONE | UDMA_ARB_4,
    UDMA_SIZE_16 | UDMA_SRC_INC_16   | UDMA_DST_INC_NONE | UDMA_ARB_4
};

static const uint32_t dmaRxConfig[] = {
    UDMA_SIZE_8  | UDMA_SRC_INC_NONE | UDMA_DST_INC_8    | UDMA_ARB_4,
    UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16   | UDMA_ARB_4
};

static const uint32_t dmaNullConfig[] = {
    UDMA_SIZE_8  | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_4,
    UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_4
};

/*
 *  ======== SPIMSP432E4DMA_close ========
 */
void SPIMSP432E4DMA_close(SPI_Handle handle)
{
    uint8_t                       port;
    SPIMSP432E4DMA_Object        *object = handle->object;
    SPIMSP432E4DMA_HWAttrs const *hwAttrs = handle->hwAttrs;

    SSIDisable(hwAttrs->baseAddr);

    if (object->hwiHandle) {
        HwiP_delete(object->hwiHandle);
        object->hwiHandle = NULL;
    }

    if (object->dmaHandle) {
        UDMAMSP432E4_close(object->dmaHandle);
        object->dmaHandle = NULL;
    }

    if (object->transferComplete) {
        SemaphoreP_delete(object->transferComplete);
        object->transferComplete = NULL;
    }

    GPIOMSP432E4_undoPinConfig(hwAttrs->clkPinMask);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->clkPinMask);
    Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));

    if (hwAttrs->xdat0PinMask != SPIMSP432E4_PIN_NO_CONFIG) {
        GPIOMSP432E4_undoPinConfig(hwAttrs->xdat0PinMask);
        port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->xdat0PinMask);
        Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));
    }

    if (hwAttrs->xdat1PinMask != SPIMSP432E4_PIN_NO_CONFIG) {
        GPIOMSP432E4_undoPinConfig(hwAttrs->xdat1PinMask);
        port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->xdat1PinMask);
        Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));
    }

    if (hwAttrs->fssPinMask != SPIMSP432E4_PIN_NO_CONFIG) {
        GPIOMSP432E4_undoPinConfig(hwAttrs->fssPinMask);
        port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->fssPinMask);
        Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));
    }

    Power_releaseDependency(getPowerMgrId(hwAttrs->baseAddr));

    object->isOpen = false;
}

/*
 *  ======== SPIMSP432E4DMA_control ========
 */
int_fast16_t SPIMSP432E4DMA_control(SPI_Handle handle, uint_fast16_t cmd,
    void *arg)
{
    return (SPI_STATUS_UNDEFINEDCMD);
}

/*
 *  ======== SPIMSP432E4DMA_hwiFxn ========
 */
static void SPIMSP432E4DMA_hwiFxn(uintptr_t arg)
{
    uint32_t                      channelMode;
    uint32_t                      freeChannel;
    uint32_t                      intStatus;
    uintptr_t                     key;
    SPI_Transaction              *msg;
    size_t                       *transferSize;
    SPIMSP432E4DMA_Object        *object = ((SPI_Handle) arg)->object;
    SPIMSP432E4DMA_HWAttrs const *hwAttrs = ((SPI_Handle) arg)->hwAttrs;
    uint8_t                       i;

    intStatus = SSIIntStatus(hwAttrs->baseAddr, true);
    SSIIntClear(hwAttrs->baseAddr, intStatus);
    if ((intStatus & SSI_DMARX) == 0) {
        /*
         * SSI_DMARX is the only interrupt of interest & enabled by this
         * driver.  Return immediately if the interrupt was triggered by
         * anything else.
         */
        return;
    }

    /*
     * We check both channels for completion; this is done in case the second
     * channel finishes while we are still configuring the first.
     */
    for (i = 0; i < 2; i++) {
        if (object->headPtr == NULL){
            /* When i was 0, we finished the last transaction */
            break;
        }

        if (object->activeChannel == UDMA_PRI_SELECT) {
            transferSize = &object->priTransferSize;
        }
        else {
            transferSize = &object->altTransferSize;
        }

        channelMode = uDMAChannelModeGet(hwAttrs->rxDmaChannel | object->activeChannel);
        if (channelMode == UDMA_MODE_STOP && *transferSize != 0) {
            key = HwiP_disable();

            object->framesTransferred += *transferSize;
            freeChannel = object->activeChannel;
            object->activeChannel = (freeChannel == UDMA_PRI_SELECT) ?
                UDMA_ALT_SELECT : UDMA_PRI_SELECT;

            /*
             * Set the channel's transfer size to 0; 0 lets
             * configNextTransfer() know that there is a free channel.
             */
            *transferSize = 0;

            if (object->framesQueued < object->headPtr->count ||
                object->framesTransferred < object->headPtr->count) {
                /*
                 * In this case we need to reconfigure the channel to continue
                 * transferring frames. configNextTransfer() will continue
                 * queuing frames for the current transfer or start
                 * the following transaction if necessary.
                 */
                configNextTransfer(object, hwAttrs);

                HwiP_restore(key);
            }
            else {
                /*
                 * All data has been transferred for the current transaction.
                 * Set transaction status & store a reference to it in a
                 * temporary pointer.  This is required because the
                 * head pointer is moved to the following transaction.
                 */
                object->headPtr->status = SPI_TRANSFER_COMPLETED;
                msg = object->headPtr;
                object->headPtr = object->headPtr->nextPtr;
                msg->nextPtr = NULL;

                /* Update object variables for the following transfer. */
                object->framesQueued = (object->activeChannel == UDMA_PRI_SELECT) ?
                    object->priTransferSize : object->altTransferSize;
                object->framesTransferred = 0;

                if (object->headPtr != NULL) {
                    /* Reconfigure the channel for the following transaction */
                    configNextTransfer(object, hwAttrs);
                }
                else {
                    /* There are no more queued transfers; disable peripheral */
                    SSIDMADisable(hwAttrs->baseAddr, SSI_DMA_TX | SSI_DMA_RX);
                    SSIIntDisable(hwAttrs->baseAddr, SSI_DMARX);

                    SSIDisable(hwAttrs->baseAddr);
                }

                HwiP_restore(key);

                object->transferCallbackFxn((SPI_Handle) arg, msg);
            }
        }
    }
}

/*
 *  ======== SPIMSP432E4DMA_init ========
 */
void SPIMSP432E4DMA_init(SPI_Handle handle)
{
    UDMAMSP432E4_init();
}

/*
 *  ======== SPIMSP432E4DMA_open ========
 */
SPI_Handle SPIMSP432E4DMA_open(SPI_Handle handle, SPI_Params *params)
{
    uint8_t                       pin;
    uint8_t                       port;
    uint8_t                       powerMgrId;
    uint32_t                      pinMap;
    uintptr_t                     key;
    HwiP_Params                   hwiParams;
    SPIMSP432E4DMA_Object        *object = handle->object;
    SPIMSP432E4DMA_HWAttrs const *hwAttrs = handle->hwAttrs;

    key = HwiP_disable();

    /* Failure Conditions */
    if (object->isOpen ||
        params->dataSize > PARAMS_DATASIZE_MAX ||
        params->dataSize < PARAMS_DATASIZE_MIN) {
        HwiP_restore(key);

        return (NULL);
    }
    object->isOpen = true;

    HwiP_restore(key);

    /* SPI_MW is unsupported */
    if (params->frameFormat == SPI_MW) {
        object->isOpen = false;

        return (NULL);
    }

    powerMgrId = getPowerMgrId(hwAttrs->baseAddr);
    if (powerMgrId > PowerMSP432E4_NUMRESOURCES) {
        object->isOpen = false;

        return (NULL);
    }
    Power_setDependency(powerMgrId);

    /* Set GPIO power dependencies & configure SPI pins */
    if (hwAttrs->clkPinMask == SPIMSP432E4_PIN_NO_CONFIG) {
        /* SPI clock pin is required */
        Power_releaseDependency(powerMgrId);
        object->isOpen = false;

        return (NULL);
    }

    pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->clkPinMask);
    pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->clkPinMask);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->clkPinMask);
    Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
    GPIOPinConfigure(pinMap);
    GPIOPinTypeSSI(GPIOMSP432E4_getGpioBaseAddr(port), pin);

    if (hwAttrs->xdat0PinMask != SPIMSP432E4_PIN_NO_CONFIG) {
        pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->xdat0PinMask);
        pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->xdat0PinMask);
        port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->xdat0PinMask);
        Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
        GPIOPinConfigure(pinMap);
        GPIOPinTypeSSI(GPIOMSP432E4_getGpioBaseAddr(port), pin);
    }

    if (hwAttrs->xdat1PinMask != SPIMSP432E4_PIN_NO_CONFIG) {
        pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->xdat1PinMask);
        pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->xdat1PinMask);
        port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->xdat1PinMask);
        Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
        GPIOPinConfigure(pinMap);
        GPIOPinTypeSSI(GPIOMSP432E4_getGpioBaseAddr(port), pin);
    }

    if (hwAttrs->fssPinMask != SPIMSP432E4_PIN_NO_CONFIG) {
        pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->fssPinMask);
        pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->fssPinMask);
        port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->fssPinMask);
        Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
        GPIOPinConfigure(pinMap);
        GPIOPinTypeSSI(GPIOMSP432E4_getGpioBaseAddr(port), pin);
    }

    object->dmaHandle = UDMAMSP432E4_open();
    if (object->dmaHandle == NULL) {
        SPIMSP432E4DMA_close(handle);

        return (NULL);
    }

    HwiP_Params_init(&hwiParams);
    hwiParams.arg = (uintptr_t) handle;
    hwiParams.priority = hwAttrs->intPriority;
    object->hwiHandle = HwiP_create(hwAttrs->intNum, SPIMSP432E4DMA_hwiFxn,
        &(hwiParams));
    if (object->hwiHandle == NULL) {
        SPIMSP432E4DMA_close(handle);

        return (NULL);
    }

    if (params->transferMode == SPI_MODE_BLOCKING) {
        /*
         * Create a semaphore to block task execution for the duration of the
         * SPI transfer
         */
        object->transferComplete = SemaphoreP_createBinary(0);
        if (object->transferComplete == NULL) {
            SPIMSP432E4DMA_close(handle);

            return (NULL);
        }

        object->transferCallbackFxn = blockingTransferCallback;
    }
    else {
        if (params->transferCallbackFxn == NULL) {
            SPIMSP432E4DMA_close(handle);

            return (NULL);
        }

        object->transferCallbackFxn = params->transferCallbackFxn;
    }

    object->headPtr = NULL;
    object->tailPtr = NULL;
    object->bitRate = params->bitRate;
    object->dataSize = params->dataSize;
    object->format = frameFormat[params->frameFormat];
    object->spiMode = params->mode;
    object->transferMode = params->transferMode;
    object->transferTimeout = params->transferTimeout;
    object->busyBit = (params->mode == SPI_MASTER ? SSI_SR_BSY : SSI_SR_TFE);

    initHw(object, hwAttrs);

    return (handle);
}

/*
 *  ======== SPIMSP432E4DMA_transfer ========
 */
bool SPIMSP432E4DMA_transfer(SPI_Handle handle, SPI_Transaction *transaction)
{
    uint8_t                       alignMask;
    bool                          buffersAligned;
    uintptr_t                     key;
    SPIMSP432E4DMA_Object        *object = handle->object;
    SPIMSP432E4DMA_HWAttrs const *hwAttrs = handle->hwAttrs;

    if (transaction->count == 0) {
        return (false);
    }

    key = HwiP_disable();

    /*
     * alignMask is used to determine if the RX/TX buffers addresses are
     * aligned to the transfer size.
     */
    alignMask = (object->dataSize < 9) ? 0x0 : 0x01;
    buffersAligned = ((((uint32_t) transaction->rxBuf & alignMask) == 0) &&
        (((uint32_t) transaction->txBuf & alignMask) == 0));

    if (!buffersAligned ||
        (object->headPtr && object->transferMode == SPI_MODE_BLOCKING)) {
        transaction->status = SPI_TRANSFER_FAILED;

        HwiP_restore(key);

        return (false);
    }
    else {
        if (object->headPtr) {
            object->tailPtr->nextPtr = transaction;
            object->tailPtr = transaction;
            object->tailPtr->status = SPI_TRANSFER_QUEUED;
        }
        else {
            object->headPtr = transaction;
            object->tailPtr = transaction;

            object->framesQueued = 0;
            object->framesTransferred = 0;
            object->priTransferSize = 0;
            object->altTransferSize = 0;
            object->tailPtr->status = SPI_TRANSFER_STARTED;
        }

        object->tailPtr->nextPtr = NULL;
    }

    /*
     * Polling transfer if BLOCKING mode & transaction->count < threshold
     * Slaves not allowed to use polling unless timeout is disabled
     */
    if (object->transferMode == SPI_MODE_BLOCKING &&
        transaction->count < hwAttrs->minDmaTransferSize &&
        (object->spiMode == SPI_MASTER ||
        object->transferTimeout == SPI_WAIT_FOREVER)) {

        HwiP_restore(key);

        spiPollingTransfer(object, hwAttrs, transaction);

        /* Transaction completed; set status & mark SPI ready */
        object->headPtr->status = SPI_TRANSFER_COMPLETED;
        object->headPtr = NULL;
        object->tailPtr = NULL;
    }
    else {
        /*
         * Perform a DMA backed SPI transfer; we need exclusive access while
         * priming the transfer to prevent race conditions with
         * SPIMSP432E4_transferCancel().
         */
        primeTransfer(object, hwAttrs);

        HwiP_restore(key);

        if (object->transferMode == SPI_MODE_BLOCKING) {
            if (SemaphoreP_pend(object->transferComplete,
                object->transferTimeout) != SemaphoreP_OK) {
                /* Timeout occurred; cancel the transfer */
                object->headPtr->status = SPI_TRANSFER_FAILED;
                SPIMSP432E4DMA_transferCancel(handle);

                /*
                 * SPIMSP432EDMA_transferCancel() performs callback which posts
                 * transferComplete semaphore. This call consumes this extra
                 * post.
                 */
                SemaphoreP_pend(object->transferComplete, SemaphoreP_NO_WAIT);

                return (false);
            }
        }
    }

    return (true);
}

/*
 *  ======== SPIMSP432E4DMA_transferCancel ========
 */
void SPIMSP432E4DMA_transferCancel(SPI_Handle handle)
{
    uintptr_t                     key;
    uint32_t                      temp;
    SPI_Transaction              *head;
    SPI_Transaction              *next;
    SPIMSP432E4DMA_Object        *object = handle->object;
    SPIMSP432E4DMA_HWAttrs const *hwAttrs = handle->hwAttrs;

    /*
     * Acquire exclusive access to the driver.  Required to prevent race
     * conditions if preempted by code trying to configure another transfer.
     */
    key = HwiP_disable();

    if (object->headPtr == NULL) {
        HwiP_restore(key);

        return;
    }

    /*
     * There are 2 use cases in which to call transferCancel():
     *   1.  The driver is in CALLBACK mode.
     *   2.  The driver is in BLOCKING mode & there has been a transfer timeout.
     */
    if (object->transferMode != SPI_MODE_BLOCKING ||
        object->headPtr->status == SPI_TRANSFER_FAILED) {

        /* Prevent interrupt while canceling the transfer */
        HwiP_disableInterrupt(hwAttrs->intNum);

        /*
         * Disable the TX DMA channel first to stop feeding more frames to
         * the FIFO.  Next, wait until the TX FIFO is empty (all frames in
         * FIFO have been sent).  RX DMA channel is disabled later to allow
         * the DMA to move all frames already in FIFO to memory.
         */
        uDMAChannelDisable(hwAttrs->txDmaChannel);

        if (object->spiMode == SPI_MASTER) {
            /*
             * Wait until the TX FIFO is empty; this is to make sure the
             * chip select is deasserted before disabling the SPI.
             */
            while (SSIBusy(hwAttrs->baseAddr)) {}
        }

        SSIDisable(hwAttrs->baseAddr);

        /* Now disable the RX, DMA & interrupts */
        uDMAChannelDisable(hwAttrs->rxDmaChannel);
        SSIDMADisable(hwAttrs->baseAddr, SSI_DMA_TX | SSI_DMA_RX);
        SSIIntDisable(hwAttrs->baseAddr, SSI_DMARX);
        SSIIntClear(hwAttrs->baseAddr, SSI_DMARX);

        /*
         * Update transaction->count with the amount of frames which have
         * been transferred.
         */
        object->headPtr->count = object->framesTransferred;
        if (object->priTransferSize) {
            temp =
                getDmaRemainingXfers(hwAttrs->rxDmaChannel | UDMA_PRI_SELECT);

            if (temp <= object->priTransferSize) {
                object->headPtr->count +=
                    (object->priTransferSize - temp);
             }
        }

        if (object->altTransferSize) {
            temp =
                getDmaRemainingXfers(hwAttrs->rxDmaChannel | UDMA_ALT_SELECT);

            if (temp <= object->altTransferSize) {
                object->headPtr->count +=
                    (object->altTransferSize - temp);
            }
        }

        /*
         * Disables peripheral, clears all registers & reinitializes it to
         * parameters used in SPI_open()
         */
        initHw(object, hwAttrs);

        HwiP_clearInterrupt(hwAttrs->intNum);
        HwiP_enableInterrupt(hwAttrs->intNum);

        /* Set status CANCELED if we did not cancel due to timeout  */
        if (object->headPtr->status == SPI_TRANSFER_STARTED) {
            object->headPtr->status = SPI_TRANSFER_CANCELED;
        }

        /*
         * Use a temporary transaction pointer to store the transaction list
         * in case any of the callbacks attempt to perform another
         * SPI_transfer().  We also clear all driver object variables.
         */
        head = object->headPtr;
        next = object->headPtr->nextPtr;
        object->headPtr = NULL;
        object->tailPtr = NULL;
        object->framesQueued = 0;
        object->framesTransferred = 0;
        object->priTransferSize = 0;
        object->altTransferSize = 0;

        HwiP_restore(key);

        /* Run the first transaction's callback */
        head->nextPtr = NULL;
        object->transferCallbackFxn(handle, head);

        /* Iterate through list invoking callbacks for all queued transfers */
        head = next;
        while (head != NULL) {
            next = head->nextPtr;
            head->nextPtr = NULL;
            head->status = SPI_TRANSFER_CANCELED;
            head->count = 0;

            object->transferCallbackFxn(handle, head);
            head = next;
        }

        /* Must return here; do not call HwiP_restore() twice */
        return;
    }

    HwiP_restore(key);
}

/*
 *  ======== blockingTransferCallback ========
 */
static void blockingTransferCallback(SPI_Handle handle,
    SPI_Transaction *transaction)
{
    SPIMSP432E4DMA_Object *object = handle->object;

    SemaphoreP_post(object->transferComplete);
}

/*
 *  ======== configNextTransfer ========
 *  This function must be executed with interrupts disabled.
 */
static void configNextTransfer(SPIMSP432E4DMA_Object *object,
    SPIMSP432E4DMA_HWAttrs const *hwAttrs)
{
    void            *buf;
    uint32_t         channelOptions;
    uint32_t         channelSel;
    size_t           framesQueued;
    uint32_t         transferAmt;
    SPI_Transaction *transaction;
    uint8_t          optionsIndex;

    /*
     * The DMA options vary according to data frame size; options for 8-bit
     * data (or smaller) are in index 0.  Options for larger frame sizes are
     * in index 1.
     */
    optionsIndex = (object->dataSize < 9) ? 0 : 1;

    /*
     * object->framesQueued keeps track of how many frames (of the current
     * transaction) have been configured for DMA transfer.  If
     * object->framesQueued == transaction->count; all frames have been queued
     * & we should configure the free DMA channel to send the next transaction.
     * When the current transaction has completed; object->framesQueued
     * will be updated (in the ISR) to reflect the amount of frames queued
     * of the following transaction.
     */
    transaction = object->headPtr;
    if (object->framesQueued < transaction->count) {
        framesQueued = object->framesQueued;
    }
    else {
        transaction = object->headPtr->nextPtr;
        if (transaction == NULL) {
            /* There are no queued transactions */
            return;
        }

        framesQueued = 0;
        transaction->status = SPI_TRANSFER_STARTED;
    }

    /*
     * The DMA has a max transfer amount of 1024.  If the transaction is
     * greater; we must transfer it in chunks.  framesQueued keeps track of
     * how much data has been queued for transfer.
     */
    if ((transaction->count - framesQueued) > MAX_DMA_TRANSFER_AMOUNT) {
        transferAmt = MAX_DMA_TRANSFER_AMOUNT;
    }
    else {
        transferAmt = transaction->count - framesQueued;
    }

    /* Determine free channel & mark it as used by setting transfer size */
    if (object->priTransferSize == 0) {
        channelSel = UDMA_PRI_SELECT;
        object->priTransferSize = transferAmt;
    }
    else {
        channelSel = UDMA_ALT_SELECT;
        object->altTransferSize = transferAmt;
    }

    if (transaction->txBuf) {
        channelOptions = dmaTxConfig[optionsIndex];
        /*
         * Add an offset for the amount of data transfered.  The offset is
         * calculated by: object->framesQueued * (optionsIndex + 1).  This
         * accounts for 8 or 16-bit sized transfers.
         */
        buf = (void *) ((uint32_t) transaction->txBuf +
            ((uint32_t) framesQueued * (optionsIndex + 1)));
    }
    else {
        channelOptions = dmaNullConfig[optionsIndex];
        buf = (void *) &hwAttrs->defaultTxBufValue;
    }

    /* Setup the TX transfer characteristics & buffers */
    uDMAChannelControlSet(hwAttrs->txDmaChannel | channelSel, channelOptions);
    uDMAChannelTransferSet(hwAttrs->txDmaChannel | channelSel,
        UDMA_MODE_PINGPONG, buf,
        (void *) &(((SSI0_Type *) hwAttrs->baseAddr)->DR), transferAmt);

    if (transaction->rxBuf) {
        channelOptions = dmaRxConfig[optionsIndex];
        /*
         * Add an offset for the amount of data transfered.  The offset is
         * calculated by: object->framesQueued * (optionsIndex + 1).  This
         * accounts for 8 or 16-bit sized transfers.
         */
        buf = (void *) ((uint32_t) transaction->rxBuf +
            ((uint32_t) framesQueued * (optionsIndex + 1)));
    }
    else {
        channelOptions = dmaNullConfig[optionsIndex];
        buf = &(object->rxScratchBuf);
    }

    /* Setup the RX transfer characteristics & buffers */
    uDMAChannelControlSet(hwAttrs->rxDmaChannel | channelSel, channelOptions);
    uDMAChannelTransferSet(hwAttrs->rxDmaChannel | channelSel,
        UDMA_MODE_PINGPONG, (void *) &(((SSI0_Type *) hwAttrs->baseAddr)->DR),
        buf, transferAmt);

    if (transaction == object->headPtr) {
        /*
         * Only update object->framesQueued if we are configuring a DMA
         * channel for the current transaction.
         */
        object->framesQueued += transferAmt;
    }

    uDMAChannelEnable(hwAttrs->txDmaChannel | channelSel);
    uDMAChannelEnable(hwAttrs->rxDmaChannel | channelSel);
}

/*
 *  ======== getDmaRemainingXfers ========
 */
static inline uint32_t getDmaRemainingXfers(uint32_t channel)
{
    uint32_t          controlWord;
    tDMAControlTable *controlTable;

    controlTable = uDMAControlBaseGet();
    controlWord = controlTable[(channel & 0x3f)].ui32Control;

    return (((controlWord & UDMA_CHCTL_XFERSIZE_M) >> 4) + 1);
}

/*
 *  ======== getPowerMgrId ========
 */
static uint8_t getPowerMgrId(uint32_t baseAddr)
{
    switch (baseAddr) {
        case SSI0_BASE:
            return (PowerMSP432E4_PERIPH_SSI0);
        case SSI1_BASE:
            return (PowerMSP432E4_PERIPH_SSI1);
        case SSI2_BASE:
            return (PowerMSP432E4_PERIPH_SSI2);
        case SSI3_BASE:
            return (PowerMSP432E4_PERIPH_SSI3);
        default:
            return (~0);
    }
}

/*
 *  ======== getPeripheralResetId ========
 */
static uint32_t getPeripheralResetId(uint32_t baseAddr)
{
    switch (baseAddr) {
        case SSI0_BASE:
            return (SYSCTL_PERIPH_SSI0);
        case SSI1_BASE:
            return (SYSCTL_PERIPH_SSI1);
        case SSI2_BASE:
            return (SYSCTL_PERIPH_SSI2);
        case SSI3_BASE:
            return (SYSCTL_PERIPH_SSI3);
        default:
            /* Should never return default */
            return (~0);
    }
}

/*
 *  ======== initHw ========
 */
static void initHw(SPIMSP432E4DMA_Object *object,
    SPIMSP432E4DMA_HWAttrs const *hwAttrs)
{
    uintptr_t     key;
    ClockP_FreqHz freq;

    /*
     * SPI peripheral should remain disabled until a transfer is requested.
     * This is done to prevent the RX FIFO from gathering data from other
     * transfers.
     */
    SSIDisable(hwAttrs->baseAddr);

    /* Reset all peripheral registers */
    SysCtlPeripheralReset(getPeripheralResetId(hwAttrs->baseAddr));

    /* Set the SPI configuration */
    ClockP_getCpuFreq(&freq);
    SSIConfigSetExpClk(hwAttrs->baseAddr, freq.lo, object->format,
        object->spiMode, object->bitRate, object->dataSize);

    key = HwiP_disable();

    /* Claim the DMA channels used for transfers */
    uDMAChannelAssign(hwAttrs->rxDmaChannel);
    uDMAChannelAssign(hwAttrs->txDmaChannel);

    HwiP_restore(key);
}

/*
 *  ======== primeTransfer ========
 *  This function must be executed with interrupts disabled.
 */
static inline void primeTransfer(SPIMSP432E4DMA_Object *object,
    SPIMSP432E4DMA_HWAttrs const *hwAttrs)
{
    if (object->priTransferSize != 0 && object->altTransferSize != 0) {
        /*
         * Both primary & alternate channels are configured for a transfer.
         * In this case no work is required; the Hwi will configure channels
         * as transfers continue & complete.
         */
    }
    else if (object->priTransferSize == 0 && object->altTransferSize == 0) {
        /*
         * Primary & alternate channels are disabled; no active transfer,
         * configure a new transfer.
         *
         * DMA based transfers use the DMA in ping-pong mode.  If the transfer
         * is larger than what the primary channel can handle; the alternate
         * channel is configured to continue where the primary channel left off.
         * Channels are continuously reconfigured until the transfer is
         * completed.
         *
         * We disable the alternate channel initially.  This however causes an
         * undesired interrupt to be triggered; so we need to
         * disable/clear/re-enable the interrupt.
         */
        HwiP_disableInterrupt(hwAttrs->intNum);

        /* Set the primary DMA structure as active */
        uDMAChannelAttributeDisable(hwAttrs->rxDmaChannel, UDMA_ATTR_ALTSELECT);
        uDMAChannelAttributeDisable(hwAttrs->txDmaChannel, UDMA_ATTR_ALTSELECT);

        HwiP_clearInterrupt(hwAttrs->intNum);
        HwiP_enableInterrupt(hwAttrs->intNum);

        /* Configure RX & TX DMA transfers */
        configNextTransfer(object, hwAttrs);
        object->activeChannel = UDMA_PRI_SELECT;
        if (object->headPtr->count > MAX_DMA_TRANSFER_AMOUNT) {
            configNextTransfer(object, hwAttrs);
        }

        /* Enable DMA to generate interrupt on SPI peripheral */
        SSIDMAEnable(hwAttrs->baseAddr, SSI_DMA_TX | SSI_DMA_RX);
        SSIIntClear(hwAttrs->baseAddr, SSI_DMARX);
        SSIIntEnable(hwAttrs->baseAddr, SSI_DMARX);

        SSIEnable(hwAttrs->baseAddr);
    }
    else {
        /* One of the channels is active; configure the other channel */
        configNextTransfer(object, hwAttrs);
    }
}

/*
 *  ======== spiPollingTransfer ========
 */
static inline void spiPollingTransfer(SPIMSP432E4DMA_Object *object,
    SPIMSP432E4DMA_HWAttrs const *hwAttrs, SPI_Transaction *transaction)
{
    uint8_t   increment;
    uint32_t  dummyBuffer;
    size_t    transferCount;
    void     *rxBuf;
    void     *txBuf;

    if (transaction->rxBuf) {
        rxBuf = transaction->rxBuf;
    }
    else {
        rxBuf = &(object->rxScratchBuf);
    }

    if (transaction->txBuf) {
        txBuf = transaction->txBuf;
    }
    else {
        txBuf = (void *) &hwAttrs->defaultTxBufValue;
    }

    increment = (object->dataSize < 9) ? sizeof(uint8_t) : sizeof(uint16_t);
    transferCount = transaction->count;

    /* Start the polling transfer */
    SSIEnable(hwAttrs->baseAddr);

    while (transferCount--) {
        if (object->dataSize < 9) {
            SSIDataPut(hwAttrs->baseAddr, *((uint8_t *) txBuf));
            SSIDataGet(hwAttrs->baseAddr, &dummyBuffer);
            *((uint8_t *) rxBuf) = (uint8_t) dummyBuffer;
        }
        else {
            SSIDataPut(hwAttrs->baseAddr, *((uint16_t *) txBuf));
            SSIDataGet(hwAttrs->baseAddr, &dummyBuffer);
            *((uint16_t *) rxBuf) = (uint16_t) dummyBuffer;
        }

        /* Only increment source & destination if buffers were provided */
        if (transaction->rxBuf) {
            rxBuf = (void *) (((uint32_t) rxBuf) + increment);
        }
        if (transaction->txBuf) {
            txBuf = (void *) (((uint32_t) txBuf) + increment);
        }
    }

    while (spiBusy(object, hwAttrs)) {}

    SSIDisable(hwAttrs->baseAddr);
}

/*
 *  ======== spiBusy ========
 *  HW is busy when in master mode and BSY bit is set, or when in slave mode
 *  and TFE bit is not set.
 */
static inline bool spiBusy(SPIMSP432E4DMA_Object *object,
                           SPIMSP432E4DMA_HWAttrs const *hwAttrs)
{
    bool registerBit = (bool)(HWREG(hwAttrs->baseAddr + SSI_O_SR) & (object->busyBit));
    if (object->busyBit == SSI_SR_BSY){
        return(registerBit);
    }
    else
    {
        return(!registerBit);
    }
}
