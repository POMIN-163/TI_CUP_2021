/*
 * Copyright (c) 2020, Texas Instruments Incorporated
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

/*
 *  ======== UART2MSP432E4.c ========
 */

#include <stdint.h>
#include <stdbool.h>

#include <ti/devices/msp432e4/inc/msp432.h>
#include <ti/devices/msp432e4/driverlib/uart.h>
#include <ti/devices/msp432e4/driverlib/types.h>
#include <ti/devices/msp432e4/driverlib/inc/hw_uart.h>

#include <ti/drivers/dpl/ClockP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/gpio/GPIOMSP432E4.h>
#include <ti/drivers/power/PowerMSP432E4.h>
#include <ti/drivers/dma/UDMAMSP432E4.h>

#include <ti/drivers/uart2/UART2MSP432E4.h>

#if defined(__IAR_SYSTEMS_ICC__)
#include <intrinsics.h>
#endif

#define MIN(a,b) (((a)<(b))?(a):(b))

/* Mazimum number of bytes that DMA can transfer */
#define MAX_SIZE 1024

/* Number of bytes in the RX FIFO corresponding to the FIFO level UART_FIFO_RX2_8 */
#define RXFIFOBYTES 4

/* Options for DMA write and read */
#define TX_CONTROL_OPTS  (UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | \
                          UDMA_ARB_4)
#define RX_CONTROL_OPTS  (UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | \
                          UDMA_ARB_4)

/* Static functions */
static void configDmaRx(UART2_Handle handle);
static void configDmaTx(UART2_Handle handle);
static void UART2MSP432E4_hwiIntFxn(uintptr_t arg);
static void cancelDmaRx(UART2_Handle handle);
static void enableRX(UART2_Handle handle);
static int32_t readData(UART2_Handle handle, int32_t size);
static void readSemCallback(UART2_Handle handle, void *buffer, size_t count,
        void *userArg, int_fast16_t status);
static void readDone(UART2_Handle handle);
static void writeSemCallback(UART2_Handle handle, void *buffer, size_t count,
        void *userArg, int_fast16_t status);
static uint_fast16_t getPowerResourceId(uint32_t baseAddr);

/* UART function table for UART2MSP432E4 implementation */
const UART2_FxnTable UART2MSP432E4_fxnTable = {
    UART2MSP432E4_close,
    UART2MSP432E4_open,
    UART2MSP432E4_read,
    UART2MSP432E4_readCancel,
    UART2MSP432E4_write,
    UART2MSP432E4_writeCancel,
    UART2MSP432E4_flushRx,
};

/* Map UART2 data length to driverlib data length */
static const uint32_t dataLength[] = {
    UART_CONFIG_WLEN_5,     /* UART2_DataLen_5 */
    UART_CONFIG_WLEN_6,     /* UART2_DataLen_6 */
    UART_CONFIG_WLEN_7,     /* UART2_DataLen_7 */
    UART_CONFIG_WLEN_8      /* UART2_DataLen_8 */
};

/* Map UART2 stop bits to driverlib stop bits */
static const uint32_t stopBits[] = {
    UART_CONFIG_STOP_ONE,   /* UART2_StopBits_1 */
    UART_CONFIG_STOP_TWO    /* UART2_StopBits_2 */
};

/* Map UART2 parity type to driverlib parity type */
static const uint32_t parityType[] = {
    UART_CONFIG_PAR_NONE,   /* UART2_Parity_NONE */
    UART_CONFIG_PAR_EVEN,   /* UART2_Parity_EVEN */
    UART_CONFIG_PAR_ODD,    /* UART2_Parity_ODD */
    UART_CONFIG_PAR_ZERO,   /* UART2_Parity_ZERO */
    UART_CONFIG_PAR_ONE     /* UART2_Parity_ONE */
};

/*
 *  ======== uartDmaEnable ========
 *  Atomic version of DriverLib UARTDMAEnable()
 */
static inline void uartDmaEnable(uint32_t ui32Base, uint32_t ui32DMAFlags)
{
    uintptr_t key;

    key = HwiP_disable();
    /* Set the requested bits in the UART DMA control register. */
    HWREG(ui32Base + UART_O_DMACTL) |= ui32DMAFlags;

    HwiP_restore(key);
}

/*
 *  ======== uartDmaDisable ========
 *  Atomic version of DriverLib UARTDMADisable()
 */
static inline void uartDmaDisable(uint32_t ui32Base, uint32_t ui32DMAFlags)
{
    uintptr_t key;

    key = HwiP_disable();
    /* Clear the requested bits in the UART DMA control register. */
    HWREG(ui32Base + UART_O_DMACTL) &= ~ui32DMAFlags;

    HwiP_restore(key);
}

/*
 *  ======== getRxStatus ========
 *  Get the left-most bit set in the RX error status (OE, BE, PE, FE)
 *  read from the RSR register:
 *      bit#   3   2   1   0
 *             OE  BE  PE  FE
 *  e.g., if OE and FE are both set, OE wins.  This will make it easier
 *  to convert an RX error status to a UART2 error code.
 */
static inline uint32_t getRxStatus(uint32_t x)
{
#if defined(__TI_COMPILER_VERSION__)
    return ((uint32_t) (x & (0x80000000 >> __clz(x))));
#elif defined(__GNUC__)
    return ((uint32_t) (x & (0x80000000 >> __builtin_clz(x))));
#elif defined(__IAR_SYSTEMS_ICC__)
    return ((uint32_t) (x & (0x80000000 >>  __CLZ(x))));
#else
    #error "Unsupported compiler"
#endif
}

/*
 *  ======== rxStatus2ErrorCode ========
 *  Convert RX status (OE, BE, PE, FE) to a UART2 error code.
 */
static inline int_fast16_t rxStatus2ErrorCode(uint32_t x)
{
    uint32_t status;

    status = getRxStatus(x);
    return (-((int_fast16_t)status));
}

/*
 * Function for checking whether flow control is enabled.
 */
static inline bool isFlowControlEnabled(UART2MSP432E4_HWAttrs const *hwAttrs) {
    return (hwAttrs->flowControl == UART2MSP432E4_FLOWCTRL_HARDWARE);
}

static inline bool isRxEnabled(UART2MSP432E4_HWAttrs const  *hwAttrs) {
    return (hwAttrs->rxPin != UART2MSP432E4_PIN_UNASSIGNED);
}

static inline bool isTxEnabled(UART2MSP432E4_HWAttrs const  *hwAttrs) {
    return (hwAttrs->txPin != UART2MSP432E4_PIN_UNASSIGNED);
}

/*
 *  ======== UART2MSP432E4_close ========
 */
void UART2MSP432E4_close(UART2_Handle handle)
{
    UART2MSP432E4_Object        *object = handle->object;
    UART2MSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    uint8_t                      port; 

    /* Disable UART and interrupts. */
    UARTIntDisable(hwAttrs->baseAddr, UART_INT_RX | UART_INT_RT | UART_INT_OE |
                   UART_INT_BE | UART_INT_PE | UART_INT_FE);

    /*
     *  Disable the UART.  Do not call driverlib function
     *  UARTDisable() since it polls for BUSY bit to clear
     *  before disabling the UART FIFO and module.
     */
    /* Disable UART FIFO */
    HWREG(hwAttrs->baseAddr + UART_O_LCRH) &= ~(UART_LCRH_FEN);
    /* Disable UART module */
    HWREG(hwAttrs->baseAddr + UART_O_CTL) &= ~(UART_CTL_UARTEN | UART_CTL_TXE |
                                               UART_CTL_RXE);

    /* Deallocate pins */
    /* Remove dependencies on GPIO ports */
    if (isTxEnabled(hwAttrs)) {
        GPIOMSP432E4_undoPinConfig(hwAttrs->txPin);
        port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->txPin);
        Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));
    }

    if (isRxEnabled(hwAttrs)) {
        GPIOMSP432E4_undoPinConfig(hwAttrs->rxPin);
        port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->rxPin);
        Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));
    }

    if (isFlowControlEnabled(hwAttrs)) {
        if (hwAttrs->ctsPin != UART2MSP432E4_PIN_UNASSIGNED) {
            GPIOMSP432E4_undoPinConfig(hwAttrs->ctsPin);
            port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->ctsPin);
            Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));
        }

        if (hwAttrs->rtsPin != UART2MSP432E4_PIN_UNASSIGNED) {
            GPIOMSP432E4_undoPinConfig(hwAttrs->rtsPin);
            port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->rtsPin);
            Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));
        }
    }

    if (object->hwi) {
        HwiP_delete(object->hwi);
    }
    if (object->state.writeMode == UART2_Mode_BLOCKING) {
        SemaphoreP_delete(object->writeSem);
    }
    if (object->state.readMode == UART2_Mode_BLOCKING) {
        SemaphoreP_delete(object->readSem);
    }

    /* Release power dependency - i.e. potentially power down serial domain. */
    Power_releaseDependency(object->powerMgrId);

    object->state.opened = false;
}

/*
 *  ======== configDmaRx ========
 */
static void configDmaRx(UART2_Handle handle)
{
    UART2MSP432E4_Object        *object = handle->object;
    UART2MSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    uint32_t                     rxSize;
    uintptr_t                    key;
    
    rxSize = MIN(object->readCount, MAX_SIZE);
    object->rxSize = rxSize;

    key = HwiP_disable();

    uDMAChannelControlSet(hwAttrs->rxDmaChannel, RX_CONTROL_OPTS);
    uDMAChannelTransferSet(hwAttrs->rxDmaChannel, UDMA_MODE_BASIC, 
                           (void *)(hwAttrs->baseAddr + UART_O_DR),
                           (void *)(object->readBuf + object->bytesRead),
                           rxSize);

    uartDmaEnable(hwAttrs->baseAddr, UART_DMA_RX);

    UARTIntEnable(hwAttrs->baseAddr, UART_INT_DMARX);

    /* Enable DMA Channel - Set channel bit in ENASET register */
    uDMAChannelEnable(hwAttrs->rxDmaChannel);

    HwiP_restore(key);
}

/*
 *  ======== configDmaTx ========
 */
static void configDmaTx(UART2_Handle handle)
{
    UART2MSP432E4_Object        *object = handle->object;
    UART2MSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    uint32_t                     txSize;
    uintptr_t                    key;

    txSize = MIN(object->writeCount, MAX_SIZE);
    object->txSize = txSize;

    key = HwiP_disable();

    /* Enable TX */
    HWREG(hwAttrs->baseAddr + UART_O_CTL) |= UART_CTL_TXE;

    uDMAChannelControlSet(hwAttrs->txDmaChannel, TX_CONTROL_OPTS);
    uDMAChannelTransferSet(hwAttrs->txDmaChannel, UDMA_MODE_BASIC,
                           (void *)(object->writeBuf + object->bytesWritten),
                           (void *)(hwAttrs->baseAddr + UART_O_DR),
                           txSize);

    uartDmaEnable(hwAttrs->baseAddr, UART_DMA_TX);

    UARTIntEnable(hwAttrs->baseAddr, UART_INT_DMATX);

    /* Enable DMA Channel - Set channel bit in ENASET register */
    uDMAChannelEnable(hwAttrs->txDmaChannel);

    HwiP_restore(key);
}

/*
 *  ======== UART2MSP432E4_flushRx ========
 */
void UART2MSP432E4_flushRx(UART2_Handle handle)
{
    UART2MSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Clear any read errors */
    UARTRxErrorClear(hwAttrs->baseAddr);

    /* Read RX FIFO until empty */
    while (((int32_t)UARTCharGetNonBlocking(hwAttrs->baseAddr)) != -1);
}

/*
 *  ======== UART2MSP432E4_hwiIntFxn ========
 *  Hwi function that processes UART interrupts.
 */
static void UART2MSP432E4_hwiIntFxn(uintptr_t arg)
{
    uint32_t                     status;
    uint32_t                     errStatus = 0;
    uint32_t                     bytesToRead;
    uint32_t                     fifoThresholdBytes;
    UART2_Handle                 handle = (UART2_Handle)arg;
    UART2MSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    UART2MSP432E4_Object        *object = handle->object;

    /* Clear interrupts */
    status = UARTIntStatus(hwAttrs->baseAddr, true);
    UARTIntClear(hwAttrs->baseAddr, status);

    if (status & (UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE)) {
        /* Handle the error */
        if (object->state.readReturnMode != UART2_ReadReturnMode_PARTIAL) {
            UARTDMADisable(hwAttrs->baseAddr, UART_DMA_RX);

            /* Number of bytes not yet transferred by the DMA */
            bytesToRead = uDMAChannelSizeGet(hwAttrs->rxDmaChannel);
            object->bytesRead += (object->readSize - bytesToRead);
        }

        errStatus = UARTRxErrorGet(hwAttrs->baseAddr);
        object->rxStatus = rxStatus2ErrorCode(errStatus);

        readDone(handle);
    }
    else if (status & UART_INT_RT) {
        if ((object->state.readReturnMode == UART2_ReadReturnMode_PARTIAL) &&
            object->readCount > 0) {
            readData(handle, object->readCount);

            readDone(handle);
        }
    }
    else if ((status & UART_INT_RX) &&
             (object->state.readReturnMode == UART2_ReadReturnMode_PARTIAL)) {
        /* Must leave at least one byte in the fifo to get read timeout */
        fifoThresholdBytes = RXFIFOBYTES;
        bytesToRead = MIN(fifoThresholdBytes - 1, object->readCount);
        readData(handle, bytesToRead);

        if (object->readCount == 0) {
            readDone(handle);
        }
    }

    /* Read data if characters are available. */
    if ((object->state.readReturnMode != UART2_ReadReturnMode_PARTIAL) &&
        object->readSize && !uDMAChannelIsEnabled(hwAttrs->rxDmaChannel)) {
        UARTDMADisable(hwAttrs->baseAddr, UART_DMA_RX);

        errStatus = UARTRxErrorGet(hwAttrs->baseAddr);
        object->rxStatus = rxStatus2ErrorCode(errStatus);
        UARTRxErrorClear(hwAttrs->baseAddr);

        UARTIntDisable(hwAttrs->baseAddr, UART_INT_DMARX);
        UARTIntClear(hwAttrs->baseAddr, UART_INT_DMARX);

        if (object->readCount != 0) {
            object->bytesRead += object->rxSize;
            object->readCount -= object->rxSize;

            if (--object->nReadTransfers == 0) {
                readDone(handle);
            }
            else {
                /* Start a new DMA transfer */
                configDmaRx(handle);
            }
        }
    }

    /* Write completed. */
    if (object->writeCount && !uDMAChannelIsEnabled(hwAttrs->txDmaChannel)) {
        /*
         * Cannot clear DMATXRIS in the UART_RIS register without disabling
         * DMA
         */
        UARTDMADisable(hwAttrs->baseAddr, UART_DMA_TX);

        UARTIntDisable(hwAttrs->baseAddr, UART_INT_DMATX);
        UARTIntClear(hwAttrs->baseAddr, UART_INT_DMATX);

        object->bytesWritten += object->txSize;
        object->writeCount -= object->txSize;

        if ((object->writeCount) && (--object->nWriteTransfers > 0)) {
            configDmaTx(handle);
        }
        else if ((object->writeCount == 0) && (object->writeSize > 0)) {
            /* Expecting EOT soon so change TX interrupt mode from FIFO to EOT */
            UARTTxIntModeSet(hwAttrs->baseAddr, UART_TXINT_MODE_EOT);
            UARTIntEnable(hwAttrs->baseAddr, UART_INT_TX);
        }
    }

    /* EOT interrupt received */
    if (status & UART_INT_TX) {
        UARTIntDisable(hwAttrs->baseAddr, UART_INT_TX);
        UARTIntClear(hwAttrs->baseAddr, UART_INT_TX);

        /* Received EOT so change mode back to FIFO */
        UARTTxIntModeSet(hwAttrs->baseAddr, UART_TXINT_MODE_FIFO);

        /* Check txEnabled in case writeCancel was called */
        if ((object->writeCount == 0) && object->state.txEnabled) {
            object->state.txEnabled = false;
            object->writeSize = 0;
            object->writeCallback(handle, (void *)object->writeBuf,
                                  object->bytesWritten, object->userArg,
                                  object->txStatus);
        }
    }
}

/*
 *  ======== UART2MSP432E4_open ========
 */
UART2_Handle UART2MSP432E4_open(uint_least8_t index, UART2_Params *params)
{
    UART2_Handle                 handle = NULL;
    uintptr_t                    key;
    ClockP_FreqHz                freq;
    UART2MSP432E4_Object        *object;
    UART2MSP432E4_HWAttrs const *hwAttrs;
    HwiP_Params                  hwiParams;
    uint8_t                      pin;
    uint32_t                     port;
    uint32_t                     pinMap;
    uint_fast16_t                resourceId;

    if (index < UART2_count) {
        handle = (UART2_Handle)&(UART2_config[index]);
        hwAttrs = handle->hwAttrs;
        object = handle->object;
    }
    else {
        return (NULL);
    }

    /* Check for callback when in UART2_Mode_CALLBACK */
    if (((params->readMode == UART2_Mode_CALLBACK) &&
        (params->readCallback == NULL)) ||
         ((params->writeMode == UART2_Mode_CALLBACK) &&
          (params->writeCallback == NULL))) {
        return (NULL);
    }

    key = HwiP_disable();

    if (object->state.opened) {
        HwiP_restore(key);
        return (NULL);
    }
    object->state.opened = true;

    HwiP_restore(key);

    object->state.txEnabled      = false;
    object->state.readMode       = params->readMode;
    object->state.writeMode      = params->writeMode;
    object->state.readReturnMode = params->readReturnMode;
    object->readCallback         = params->readCallback;
    object->writeCallback        = params->writeCallback;
    object->userArg              = params->userArg;

    /* Set UART transaction variables to defaults. */
    object->writeBuf             = NULL;
    object->readBuf              = NULL;
    object->writeCount           = 0;
    object->readCount            = 0;
    object->writeSize            = 0;
    object->readSize             = 0;
    object->nWriteTransfers      = 0;
    object->nReadTransfers       = 0;
    object->rxStatus             = 0;
    object->txStatus             = 0;

    object->readSem = NULL;
    object->writeSem = NULL;
    object->hwi = NULL;

    resourceId = getPowerResourceId(hwAttrs->baseAddr);
    if (resourceId > PowerMSP432E4_NUMRESOURCES) {
        return (NULL);
    }

    Power_setDependency(resourceId);

    UDMAMSP432E4_init();
    object->udmaHandle = UDMAMSP432E4_open();
    if (object->udmaHandle == NULL) {
        return (NULL);
    }

    if (hwAttrs->rxPin != UART2MSP432E4_PIN_UNASSIGNED) {
        uDMAChannelAssign(hwAttrs->rxDmaChannel);

        uDMAChannelAttributeDisable(hwAttrs->rxDmaChannel, (UDMA_ATTR_ALTSELECT |
                                                            UDMA_ATTR_USEBURST |
                                                            UDMA_ATTR_REQMASK));
    }

    if (hwAttrs->txPin != UART2MSP432E4_PIN_UNASSIGNED) {
        uDMAChannelAssign(hwAttrs->txDmaChannel);

        uDMAChannelAttributeDisable(hwAttrs->txDmaChannel, (UDMA_ATTR_ALTSELECT |
                                                            UDMA_ATTR_USEBURST |
                                                            UDMA_ATTR_REQMASK));
    }

    if (isTxEnabled(hwAttrs)) {
        /* Configure the TX pin */
        pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->txPin);
        port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->txPin);
        pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->txPin);

        Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
        GPIOPinConfigure(pinMap);
        GPIOPinTypeUART(GPIOMSP432E4_getGpioBaseAddr(port), pin);
    }

    if (isRxEnabled(hwAttrs)) {
        /* Configure the RX pin */
        pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->rxPin);
        port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->rxPin);
        pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->rxPin);

        Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
        GPIOPinConfigure(pinMap);
        GPIOPinTypeUART(GPIOMSP432E4_getGpioBaseAddr(port), pin);
    }

    /* Configure flow control pins if enabled */
    if (isFlowControlEnabled(hwAttrs)) {
        if (hwAttrs->ctsPin != UART2MSP432E4_PIN_UNASSIGNED) {
            pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->ctsPin);
            port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->ctsPin);
            pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->ctsPin);

            Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
            GPIOPinConfigure(pinMap);
            GPIOPinTypeUART(GPIOMSP432E4_getGpioBaseAddr(port), pin);
        }

        if (hwAttrs->rtsPin != UART2MSP432E4_PIN_UNASSIGNED) {
            pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->rtsPin);
            port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->rtsPin);
            pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->rtsPin);

            Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
            GPIOPinConfigure(pinMap);
            GPIOPinTypeUART(GPIOMSP432E4_getGpioBaseAddr(port), pin);
        }

        /* Set flow control */
        UARTFlowControlSet(hwAttrs->baseAddr,
                           UART_FLOWCONTROL_TX | UART_FLOWCONTROL_RX);
    }
    else {
        /* Disable hardware flow control */
        UARTFlowControlSet(hwAttrs->baseAddr, UART_FLOWCONTROL_NONE);
    }

    HwiP_Params_init(&hwiParams);
    hwiParams.arg = (uintptr_t)handle;
    hwiParams.priority = hwAttrs->intPriority;

    object->hwi = HwiP_create(hwAttrs->intNum, UART2MSP432E4_hwiIntFxn,
                              &hwiParams);

    if (object->hwi == NULL) {
        UART2MSP432E4_close(handle);
        return (NULL);
    }

    /* If write mode is blocking create a semaphore and set callback. */
    if (object->state.writeMode == UART2_Mode_BLOCKING) {
        object->writeSem = SemaphoreP_createBinary(0);
        if (object->writeSem == NULL) {
            UART2MSP432E4_close(handle);
            return (NULL);
        }
        object->writeCallback = &writeSemCallback;
    }

    /* If read mode is blocking create a semaphore and set callback. */
    if (object->state.readMode == UART2_Mode_BLOCKING) {
        object->readSem = SemaphoreP_createBinary(0);
        if (object->readSem == NULL) {
            UART2MSP432E4_close(handle);
            return (NULL);
        }
        object->readCallback = &readSemCallback;
    }

   /*
     * Configure frame format and baudrate.  UARTConfigSetExpClk() can cause
     * interrupt errors so call this function before clearing interrupts.
     */
    ClockP_getCpuFreq(&freq);
    UARTConfigSetExpClk(hwAttrs->baseAddr, freq.lo, params->baudRate,
                        dataLength[params->dataLength] | stopBits[params->stopBits] |
                        parityType[params->parityType]);

    /* Enable UART and its interrupts. */
    UARTIntClear(hwAttrs->baseAddr, UART_INT_OE | UART_INT_BE | UART_INT_PE |
                                    UART_INT_FE | UART_INT_RT | UART_INT_TX |
                                    UART_INT_RX | UART_INT_CTS);
    UARTEnable(hwAttrs->baseAddr);

    UARTIntEnable(hwAttrs->baseAddr, UART_INT_RX | UART_INT_RT | UART_INT_OE |
                  UART_INT_BE | UART_INT_PE | UART_INT_FE);

    UARTFIFOLevelSet(hwAttrs->baseAddr, UART_FIFO_TX2_8, UART_FIFO_RX2_8);

    return (handle);
}

/*
 *  ======== UART2MSP432E4_read ========
 */
int_fast16_t UART2MSP432E4_read(UART2_Handle handle, void *buffer,
        size_t size, size_t *bytesRead, uint32_t timeout)
{
    uintptr_t                    key;
    UART2MSP432E4_Object        *object = handle->object;
    UART2MSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    int                          status = UART2_STATUS_SUCCESS;
    unsigned char               *buf = (unsigned char *)buffer;
    int32_t                      data;
    size_t                       numBytesRead;
    size_t                      *pNumBytesRead = bytesRead;
    uint32_t                     errStatus;

    pNumBytesRead = (bytesRead == NULL) ? &numBytesRead : bytesRead;
    *pNumBytesRead = 0;

    /* Check for Rx error since the last read */
    errStatus = UARTRxErrorGet(hwAttrs->baseAddr);
    status = rxStatus2ErrorCode(errStatus);
    UARTRxErrorClear(hwAttrs->baseAddr); /* Clear receive errors */

    if (status != UART2_STATUS_SUCCESS) {
        object->readSize = 0;
        return (status);
    }

    key = HwiP_disable();

    if (object->readSize) {
        /* Another read is ongoing */
        HwiP_restore(key);
        return (UART2_STATUS_EINUSE);
    }

    /* Save the data to be read and restore interrupts. */
    object->readBuf = (unsigned char *)buffer;
    object->readSize = size;
    object->readCount = size; /* Number remaining to be read */
    object->bytesRead = 0;    /* Number of bytes read */
    object->rxStatus = 0;     /* Clear receive errors */

    HwiP_restore(key);

    /* Enable RX and set the Power constraint */
    enableRX(handle);

    if (object->state.readMode == UART2_Mode_POLLING) {
        while (size) {
            data = UARTCharGetNonBlocking(hwAttrs->baseAddr);
            if (data == -1) {
                break;
            }
            /* Convert error code in upper bytes of data to a UART2 status */
            status = rxStatus2ErrorCode((data >> 8) & 0xF);
            if (status < 0) {
                break;
            }
            *buf++ = data & 0xFF;
            size--;
        }
        *pNumBytesRead = object->readSize - size;

        /* Set readSize to 0 to allow another read */
        object->readCount = 0;
        object->readSize = 0;

        return (status);
    }

    /*
     *  Don't use DMA for return partial mode, since we have to leave
     *  one byte in the FIFO to get the read timeout.
     */
    if (object->state.readReturnMode != UART2_ReadReturnMode_PARTIAL) {
        /* Can transfer maximum of 1024 bytes in one DMA transfer */
        object->nReadTransfers = (object->readSize + MAX_SIZE - 1) / MAX_SIZE;
        configDmaRx(handle);
    }

    if (object->state.readMode == UART2_Mode_BLOCKING) {
        if (SemaphoreP_OK != SemaphoreP_pend(object->readSem, timeout)) {
            key = HwiP_disable();
            if (object->readSize == 0) {
                /* Interrupt occurred just after SemaphoreP_pend() timeout */
                status = object->rxStatus;
                SemaphoreP_pend(object->readSem, SemaphoreP_NO_WAIT);
                HwiP_restore(key);
            }
            else {
                status = UART2_STATUS_ETIMEOUT;

                /* Set readCount to 0 to prevent ISR from doing more work */
                object->readCount = 0;
                HwiP_restore(key);

                cancelDmaRx(handle);
            }
        }
        else {
            status = object->rxStatus;
        }

        *pNumBytesRead = object->bytesRead;
        object->readSize = 0;  /* Allow the next read */
    }
    else {
        /* ISR will do the work */
    }

    return (status);
}

/*
 *  ======== UART2MSP432E4_readCancel ========
 */
void UART2MSP432E4_readCancel(UART2_Handle handle)
{
    UART2MSP432E4_Object          *object = handle->object;
    uintptr_t                      key;

    if (object->state.readMode == UART2_Mode_POLLING) {
        return;
    }

    key = HwiP_disable();

    if (object->readSize == 0) {
        HwiP_restore(key);
        return;
    }

    cancelDmaRx(handle);

    object->readSize = 0;
    object->rxStatus = UART2_STATUS_ECANCELLED;

    HwiP_restore(key);

    object->readCallback(handle, object->readBuf, object->bytesRead,
                         object->userArg, UART2_STATUS_ECANCELLED);
}

/*
 *  ======== UART2MSP432E4_write ========
 */
int_fast16_t UART2MSP432E4_write(UART2_Handle handle, const void *buffer,
        size_t size, size_t *bytesWritten, uint32_t timeout)
{
    UART2MSP432E4_Object        *object = handle->object;
    UART2MSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    uintptr_t                    key;
    int                          status = UART2_STATUS_SUCCESS;
    uint32_t                     writeCount;
    unsigned char               *buf = (unsigned char *)buffer;
    size_t                       nBytesWritten;
    size_t                      *pNBytesWritten;

    if (size == 0) {
        return (UART2_STATUS_EINVALID);
    }

    pNBytesWritten = (bytesWritten == NULL) ? &nBytesWritten : bytesWritten;

    key = HwiP_disable();

    if (!object->state.opened) {
        HwiP_restore(key);
        return (UART2_STATUS_EINVALID);
    }

    /*
     *  Make sure any previous write has finished.
     *  In blocking mode, UART2_cancel() may have posted the semaphore,
     *  so we use writeSize to ensure that there are no on-going writes.
     *  This ensures that we can return the txStatus to the caller without
     *  it having been reset to 0 by a pre-empting thread.
     */
    if (object->state.txEnabled ||
        (object->writeSize &&
         (object->state.writeMode == UART2_Mode_BLOCKING))) {
        HwiP_restore(key);
        return (UART2_STATUS_EINUSE);
    }

    /* Save the data to be written and restore interrupts. */
    object->writeBuf = buffer;
    object->writeSize = size;
    object->writeCount = size;
    object->bytesWritten = 0;
    object->state.txEnabled = true;

    object->txStatus = UART2_STATUS_SUCCESS;

    HwiP_restore(key);

    if (object->state.writeMode == UART2_Mode_POLLING) {
        writeCount = 0;
        while (size) {
            if (!UARTCharPutNonBlocking(hwAttrs->baseAddr, *buf)) {
                break;
            }
            buf++;
            writeCount++;
            size--;
        }
        *pNBytesWritten = writeCount;
        object->state.txEnabled = false;
        return (status);
    }

    object->nWriteTransfers = (object->writeSize + MAX_SIZE - 1) / MAX_SIZE;
    configDmaTx(handle);

    /* If writeMode is blocking, block and get the state. */
    if (object->state.writeMode == UART2_Mode_BLOCKING) {
        /* Pend on semaphore and wait for Hwi to finish. */
        if (SemaphoreP_OK != SemaphoreP_pend(object->writeSem, timeout)) {
            /* Disable the TX interrupt and clear in case it's pending */
            key = HwiP_disable();
            UARTIntDisable(hwAttrs->baseAddr, UART_INT_TX);
            UARTIntClear(hwAttrs->baseAddr, UART_INT_TX);

            object->state.txEnabled = false;
            object->txStatus = UART2_STATUS_ETIMEOUT;

            HwiP_restore(key);
        }
        status = object->txStatus; /* UART2_cancel() may have posted semaphore */
        *pNBytesWritten = object->bytesWritten;
        object->writeSize = 0;  /* Allow the next UART2_write */
        object->state.txEnabled = false;
    }

    return (status);
}

/*
 *  ======== UART2MSP432E4_writeCancel ========
 */
void UART2MSP432E4_writeCancel(UART2_Handle handle)
{
    UART2MSP432E4_Object        *object = handle->object;
    UART2MSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    uintptr_t                    key;
    uint32_t                     bytesRemaining;

    key = HwiP_disable();

    uDMAChannelDisable(hwAttrs->txDmaChannel);
    uartDmaDisable(hwAttrs->baseAddr, UART_DMA_TX);

    UARTIntDisable(hwAttrs->baseAddr, UART_INT_DMATX);
    UARTIntClear(hwAttrs->baseAddr, UART_INT_DMATX);

    /* Return if there is no write */
    if ((object->writeSize == 0) && !object->state.txEnabled) {
        HwiP_restore(key);
        return;
    }

    object->state.txEnabled = false;

    bytesRemaining = uDMAChannelSizeGet(hwAttrs->txDmaChannel);
    object->bytesWritten = object->writeSize - bytesRemaining;

    object->writeSize = 0;
    object->txStatus = UART2_STATUS_ECANCELLED;

    HwiP_restore(key);

    object->writeCallback(handle, (void *)object->writeBuf,
                          object->bytesWritten, object->userArg,
                          UART2_STATUS_ECANCELLED);
}

/*
 *  ======== cancelDmaRx ========
 */
static void cancelDmaRx(UART2_Handle handle)
{
    UART2MSP432E4_Object        *object = handle->object;
    UART2MSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    uintptr_t                    key;
    uint32_t                     bytesRemaining;

    if ((object->state.readReturnMode == UART2_ReadReturnMode_PARTIAL) ||
        (object->state.readMode == UART2_Mode_POLLING)) {
        return;
    }

    key = HwiP_disable();

    uDMAChannelDisable(hwAttrs->rxDmaChannel);
    uartDmaDisable(hwAttrs->baseAddr, UART_DMA_RX);

    UARTIntDisable(hwAttrs->baseAddr, UART_INT_DMARX);
    UARTIntClear(hwAttrs->baseAddr, UART_INT_DMARX);

    /* Return if there is no read */
    if (object->readSize == 0) {
        HwiP_restore(key);
        return;
    }

    bytesRemaining = uDMAChannelSizeGet(hwAttrs->rxDmaChannel);

    object->bytesRead = object->readSize - bytesRemaining;

    HwiP_restore(key);
}

/*
 *  ======== enableRX ========
 */
static void enableRX(UART2_Handle handle)
{
    UART2MSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    uintptr_t                  key;

    key = HwiP_disable();

    /* Enable RX but not interrupts, since we may be using DMA */
    HWREG(hwAttrs->baseAddr + UART_O_CTL) |= UART_CTL_RXE;

    HwiP_restore(key);
}

/*
 *  ======== getPowerResourceId ========
 */
static uint_fast16_t getPowerResourceId(uint32_t baseAddr)
{
    switch (baseAddr) {
        case UART0_BASE:
            return (PowerMSP432E4_PERIPH_UART0);
        case UART1_BASE:
            return (PowerMSP432E4_PERIPH_UART1);
        case UART2_BASE:
            return (PowerMSP432E4_PERIPH_UART2);
        case UART3_BASE:
            return (PowerMSP432E4_PERIPH_UART3);
        case UART4_BASE:
            return (PowerMSP432E4_PERIPH_UART4);
        case UART5_BASE:
            return (PowerMSP432E4_PERIPH_UART5);
        case UART6_BASE:
            return (PowerMSP432E4_PERIPH_UART6);
        case UART7_BASE:
            return (PowerMSP432E4_PERIPH_UART7);
        default:
            return ((uint_fast16_t)(~0));
    }
}

/*
 *  ======== readData ========
 */
static int32_t readData(UART2_Handle handle, int32_t size)
{
    UART2MSP432E4_Object         *object = handle->object;
    UART2MSP432E4_HWAttrs const  *hwAttrs = handle->hwAttrs;
    int32_t                       data;

    if (size > object->readCount) {
        size = object->readCount;
    }

    while (size) {
        data = UARTCharGetNonBlocking(hwAttrs->baseAddr);
        if (data == -1) {
            break;
        }
        *(object->readBuf + object->bytesRead) = (uint8_t)(data & 0xFF);
        object->bytesRead++;
        object->readCount--;
        size--;
    }

    return (size);
}

/*
 *  ======== readDone ========
 *  Called from the ISR only.
 */
static void readDone(UART2_Handle handle)
{
    UART2MSP432E4_Object *object = (UART2MSP432E4_Object *)(handle->object);
    size_t                count;

    count = object->bytesRead;

    /* Return if no ongoing read */
    if (object->readSize == 0) {
        return;
    }

    /* Set readSize to 0 to allow UART2_read() to be called */
    object->readSize = 0;

    object->readCallback(handle, object->readBuf, count, object->userArg,
                         object->rxStatus);
}

/*
 *  ======== readSemCallback ========
 *  Simple callback to post a semaphore for the blocking mode.
 */
static void readSemCallback(UART2_Handle handle, void *buffer, size_t count,
        void *userArg, int_fast16_t status)
{
    UART2MSP432E4_Object *object = handle->object;

    SemaphoreP_post(object->readSem);
}

/*
 *  ======== writeSemCallback ========
 *  Simple callback to post a semaphore for the blocking mode.
 */
static void writeSemCallback(UART2_Handle handle, void *buffer, size_t count,
        void *userArg, int_fast16_t status)
{
    UART2MSP432E4_Object *object = handle->object;

    SemaphoreP_post(object->writeSem);
}
