/*
 * Copyright (c) 2017-2020, Texas Instruments Incorporated
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
#include <stdint.h>

#include <ti/devices/msp432e4/inc/msp432.h>

#include <ti/devices/msp432e4/driverlib/uart.h>

#include <ti/drivers/dpl/ClockP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/gpio/GPIOMSP432E4.h>
#include <ti/drivers/power/PowerMSP432E4.h>
#include <ti/drivers/uart/UARTMSP432E4.h>

/* UARTMSP432E4 functions */
void         UARTMSP432E4_close(UART_Handle handle);
int_fast16_t UARTMSP432E4_control(UART_Handle handle, uint_fast16_t cmd, void *arg);
void         UARTMSP432E4_init(UART_Handle handle);
UART_Handle  UARTMSP432E4_open(UART_Handle handle, UART_Params *params);
int_fast32_t UARTMSP432E4_read(UART_Handle handle, void *buffer, size_t size);
void         UARTMSP432E4_readCancel(UART_Handle handle);
int_fast32_t UARTMSP432E4_readPolling(UART_Handle handle, void *buffer,
                size_t size);
int_fast32_t UARTMSP432E4_write(UART_Handle handle, const void *buffer,
                size_t size);
void         UARTMSP432E4_writeCancel(UART_Handle handle);
int_fast32_t UARTMSP432E4_writePolling(UART_Handle handle, const void *buffer,
                                   size_t size);

/* Static functions */
static void readBlockingTimeout(uintptr_t arg);
static bool readIsrBinaryBlocking(UART_Handle handle);
static bool readIsrBinaryCallback(UART_Handle handle);
static bool readIsrTextBlocking(UART_Handle handle);
static bool readIsrTextCallback(UART_Handle handle);
static void readSemCallback(UART_Handle handle, void *buffer, size_t count);
static int  readTaskBlocking(UART_Handle handle);
static int  readTaskCallback(UART_Handle handle);
static int  ringBufGet(UART_Handle object, unsigned char *data);
static void writeData(UART_Handle handle);
static void writeSemCallback(UART_Handle handle, void *buffer, size_t count);

static uint_fast16_t getPowerResourceId(uint32_t baseAddr);

/*
 * Function for checking whether flow control is enabled.
 */
static inline bool isFlowControlEnabled(UARTMSP432E4_HWAttrs const  *hwAttrs) {
    return ((hwAttrs->flowControl == UARTMSP432E4_FLOWCTRL_HARDWARE) &&
        (hwAttrs->ctsPin != UARTMSP432E4_PIN_UNASSIGNED) &&
            (hwAttrs->rtsPin != UARTMSP432E4_PIN_UNASSIGNED));
}

static inline bool isRxEnabled(UARTMSP432E4_HWAttrs const  *hwAttrs) {
    return (hwAttrs->rxPin != UARTMSP432E4_PIN_UNASSIGNED);
}

static inline bool isTxEnabled(UARTMSP432E4_HWAttrs const  *hwAttrs) {
    return (hwAttrs->txPin != UARTMSP432E4_PIN_UNASSIGNED);
}

/* UART function table for UARTMSP432E4 implementation */
const UART_FxnTable UARTMSP432E4_fxnTable = {
    UARTMSP432E4_close,
    UARTMSP432E4_control,
    UARTMSP432E4_init,
    UARTMSP432E4_open,
    UARTMSP432E4_read,
    UARTMSP432E4_readPolling,
    UARTMSP432E4_readCancel,
    UARTMSP432E4_write,
    UARTMSP432E4_writePolling,
    UARTMSP432E4_writeCancel
};

static const uint32_t dataLength[] = {
    UART_CONFIG_WLEN_5,     /* UART_LEN_5 */
    UART_CONFIG_WLEN_6,     /* UART_LEN_6 */
    UART_CONFIG_WLEN_7,     /* UART_LEN_7 */
    UART_CONFIG_WLEN_8      /* UART_LEN_8 */
};

static const uint32_t stopBits[] = {
    UART_CONFIG_STOP_ONE,   /* UART_STOP_ONE */
    UART_CONFIG_STOP_TWO    /* UART_STOP_TWO */
};

static const uint32_t parityType[] = {
    UART_CONFIG_PAR_NONE,   /* UART_PAR_NONE */
    UART_CONFIG_PAR_EVEN,   /* UART_PAR_EVEN */
    UART_CONFIG_PAR_ODD,    /* UART_PAR_ODD */
    UART_CONFIG_PAR_ZERO,   /* UART_PAR_ZERO */
    UART_CONFIG_PAR_ONE     /* UART_PAR_ONE */
};

/*
 *  ======== staticFxnTable ========
 *  This is a function lookup table to simplify the UART driver modes.
 */
static const UARTMSP432E4_FxnSet staticFxnTable[2][2] = {
    {/* UART_MODE_BLOCKING */
        {/* UART_DATA_BINARY */
            .readIsrFxn  = readIsrBinaryBlocking,
            .readTaskFxn = readTaskBlocking
        },
        {/* UART_DATA_TEXT */
            .readIsrFxn  = readIsrTextBlocking,
            .readTaskFxn = readTaskBlocking
        }
    },
    {/* UART_MODE_CALLBACK */
        {/* UART_DATA_BINARY */
            .readIsrFxn  = readIsrBinaryCallback,
            .readTaskFxn = readTaskCallback

        },
        {/* UART_DATA_TEXT */
            .readIsrFxn  = readIsrTextCallback,
            .readTaskFxn = readTaskCallback,
        }
    }
};

/*
 *  ======== UARTMSP432E4_close ========
 */
void UARTMSP432E4_close(UART_Handle handle)
{
    UARTMSP432E4_Object           *object = handle->object;
    UARTMSP432E4_HWAttrs const    *hwAttrs = handle->hwAttrs;
    uint_fast16_t                  resourceId;
    uint8_t                        port;

    /* Disable UART and interrupts. */
    UARTIntDisable(hwAttrs->baseAddr, UART_INT_TX | UART_INT_RX | UART_INT_RT |
         UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE);
    UARTDisable(hwAttrs->baseAddr);

    resourceId = getPowerResourceId(hwAttrs->baseAddr);
    if (resourceId < PowerMSP432E4_NUMRESOURCES) {
        Power_releaseDependency(resourceId);
    }

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
        GPIOMSP432E4_undoPinConfig(hwAttrs->ctsPin);
        port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->ctsPin);
        Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));

        GPIOMSP432E4_undoPinConfig(hwAttrs->rtsPin);
        port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->rtsPin);
        Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));
    }

    if (object->hwi) {
        HwiP_delete(object->hwi);
    }

    if (object->writeSem) {
        SemaphoreP_delete(object->writeSem);
    }

    if (object->readSem) {
        SemaphoreP_delete(object->readSem);
    }

    if (object->timeoutClk) {
        ClockP_delete(object->timeoutClk);
    }

    object->state.opened = false;
}

/*
 *  ======== UARTMSP432E4_control ========
 *  @pre    Function assumes that the handle is not NULL
 */
int_fast16_t UARTMSP432E4_control(UART_Handle handle, uint_fast16_t cmd, void *arg)
{
    UARTMSP432E4_Object         *object = handle->object;
    UARTMSP432E4_HWAttrs const  *hwAttrs = handle->hwAttrs;
    unsigned char                data;
    int                          bufferCount;

    bufferCount = RingBuf_peek(&object->ringBuffer, &data);

    switch (cmd) {
        /* Common UART CMDs */
        case (UART_CMD_PEEK):
            *(int *)arg = (bufferCount) ? data : UART_ERROR;
            return (UART_STATUS_SUCCESS);

        case (UART_CMD_ISAVAILABLE):
            *(bool *)arg = (bufferCount != 0);
            return (UART_STATUS_SUCCESS);

        case (UART_CMD_GETRXCOUNT):
            *(int *)arg = bufferCount;
            return (UART_STATUS_SUCCESS);

        case (UART_CMD_RXENABLE):
            if (!object->state.rxEnabled) {
                UARTIntEnable(hwAttrs->baseAddr, UART_INT_RX | UART_INT_RT |
                        UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE);
                object->state.rxEnabled = true;
                return (UART_STATUS_SUCCESS);
            }
            return (UART_STATUS_ERROR);

        case (UART_CMD_RXDISABLE):
            if (object->state.rxEnabled) {
                UARTIntDisable(hwAttrs->baseAddr, UART_INT_RX | UART_INT_RT |
                        UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE);
                object->state.rxEnabled = false;
                return (UART_STATUS_SUCCESS);
            }
            return (UART_STATUS_ERROR);

        default:
            return (UART_STATUS_UNDEFINEDCMD);
    }
}

/*
 *  ======== UARTMSP432E4_hwiIntFxn ========
 *  Hwi function that processes UART interrupts.
 *
 *  @param(arg)         The UART_Handle for this Hwi.
 */
static void UARTMSP432E4_hwiIntFxn(uintptr_t arg)
{
    uint32_t                   status;
    UARTMSP432E4_Object           *object = ((UART_Handle)arg)->object;
    UARTMSP432E4_HWAttrs const    *hwAttrs = ((UART_Handle)arg)->hwAttrs;
    uint32_t                   rxErrors;

    /* Clear interrupts */
    status = UARTIntStatus(hwAttrs->baseAddr, true);
    UARTIntClear(hwAttrs->baseAddr, status);

    if (status & (UART_INT_RX | UART_INT_RT | UART_INT_OE | UART_INT_BE |
            UART_INT_PE | UART_INT_FE)) {
        object->readFxns.readIsrFxn((UART_Handle)arg);
    }

    /* Reading the data from the FIFO doesn't mean we caught an overrrun! */
    rxErrors = UARTRxErrorGet(hwAttrs->baseAddr);
    if (rxErrors) {
        UARTRxErrorClear(hwAttrs->baseAddr);
        if (hwAttrs->errorFxn) {
            hwAttrs->errorFxn((UART_Handle)arg, rxErrors);
        }
    }

    if (status & UART_INT_TX) {
        writeData((UART_Handle)arg);
    }
}

/*
 *  ======== UARTMSP432E4_init ========
 */
void UARTMSP432E4_init(UART_Handle handle)
{
}

/*
 *  ======== UARTMSP432E4_open ========
 */
UART_Handle UARTMSP432E4_open(UART_Handle handle, UART_Params *params)
{
    uintptr_t                   key;
    ClockP_FreqHz               freq;
    UARTMSP432E4_Object        *object = handle->object;
    UARTMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    union {
        HwiP_Params             hwiParams;
        ClockP_Params           clockParams;
    } paramsUnion;
    uint8_t                     pin;
    uint32_t                    port;
    uint32_t                    pinMap;
    uint_fast16_t               resourceId;

    key = HwiP_disable();

    if (object->state.opened) {
        HwiP_restore(key);
        return (NULL);
    }
    object->state.opened = true;

    HwiP_restore(key);

    object->state.readMode       = params->readMode;
    object->state.writeMode      = params->writeMode;
    object->state.readReturnMode = params->readReturnMode;
    object->state.readDataMode   = params->readDataMode;
    object->state.writeDataMode  = params->writeDataMode;
    object->state.readEcho       = params->readEcho;
    object->readTimeout          = params->readTimeout;
    object->writeTimeout         = params->writeTimeout;
    object->readCallback         = params->readCallback;
    object->writeCallback        = params->writeCallback;
    object->baudRate             = params->baudRate;
    object->stopBits             = params->stopBits;
    object->dataLength           = params->dataLength;
    object->parityType           = params->parityType;
    object->readFxns =
        staticFxnTable[object->state.readMode][object->state.readDataMode];

    /* Set UART variables to defaults. */
    object->writeBuf             = NULL;
    object->readBuf              = NULL;
    object->writeCount           = 0;
    object->readCount            = 0;
    object->writeSize            = 0;
    object->readSize             = 0;

    object->readSem = NULL;
    object->writeSem = NULL;
    object->hwi = NULL;
    object->timeoutClk = NULL;

    RingBuf_construct(&object->ringBuffer, hwAttrs->ringBufPtr,
            hwAttrs->ringBufSize);

    HwiP_Params_init(&paramsUnion.hwiParams);
    paramsUnion.hwiParams.arg = (uintptr_t)handle;
    paramsUnion.hwiParams.priority = hwAttrs->intPriority;

    resourceId = getPowerResourceId(hwAttrs->baseAddr);
    if (resourceId > PowerMSP432E4_NUMRESOURCES) {
        return (NULL);
    }

    Power_setDependency(resourceId);

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
        pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->ctsPin);
        port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->ctsPin);
        pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->ctsPin);

        Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
        GPIOPinConfigure(pinMap);
        GPIOPinTypeUART(GPIOMSP432E4_getGpioBaseAddr(port), pin);

        pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->rtsPin);
        port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->rtsPin);
        pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->rtsPin);

        Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
        GPIOPinConfigure(pinMap);
        GPIOPinTypeUART(GPIOMSP432E4_getGpioBaseAddr(port), pin);

        /* Set flow control */
        UARTFlowControlSet(hwAttrs->baseAddr,
                UART_FLOWCONTROL_TX | UART_FLOWCONTROL_RX);
    }
    else {
        /* Disable hardware flow control */
        UARTFlowControlSet(hwAttrs->baseAddr, UART_FLOWCONTROL_NONE);
    }

    object->hwi = HwiP_create(hwAttrs->intNum, UARTMSP432E4_hwiIntFxn,
            &paramsUnion.hwiParams);

    if (object->hwi == NULL) {
        UARTMSP432E4_close(handle);
        return (NULL);
    }

    /* If write mode is blocking create a semaphore and set callback. */
    if (object->state.writeMode == UART_MODE_BLOCKING) {
        object->writeSem = SemaphoreP_createBinary(0);
        if (object->writeSem == NULL) {
            UARTMSP432E4_close(handle);
            return (NULL);
        }
        object->writeCallback = &writeSemCallback;
    }

    /* If read mode is blocking create a semaphore and set callback. */
    if (object->state.readMode == UART_MODE_BLOCKING) {
        object->readSem = SemaphoreP_createBinary(0);
        if (object->readSem == NULL) {
            UARTMSP432E4_close(handle);
            return (NULL);
        }
        object->readCallback = &readSemCallback;

        ClockP_Params_init(&paramsUnion.clockParams);
        paramsUnion.clockParams.period = 0;
        paramsUnion.clockParams.startFlag = false;
        paramsUnion.clockParams.arg = (uintptr_t)handle;
        object->timeoutClk = ClockP_create((ClockP_Fxn)&readBlockingTimeout,
                0 /* timeout */, &(paramsUnion.clockParams));
        if (object->timeoutClk == NULL) {
            UARTMSP432E4_close(handle);
            return (NULL);
        }
    }
    else {
        object->state.drainByISR = false;
    }

    /* Enable UART and its interrupt. */
    UARTIntClear(hwAttrs->baseAddr, UART_INT_TX | UART_INT_RX | UART_INT_RT);
    UARTEnable(hwAttrs->baseAddr);

    UARTFIFOLevelSet(hwAttrs->baseAddr, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

    ClockP_getCpuFreq(&freq);
    UARTConfigSetExpClk(hwAttrs->baseAddr, freq.lo, params->baudRate,
        dataLength[params->dataLength] | stopBits[params->stopBits] |
        parityType[params->parityType]);

    object->state.rxEnabled = true;
    UARTIntEnable(hwAttrs->baseAddr, UART_INT_RX | UART_INT_RT | UART_INT_OE |
        UART_INT_BE | UART_INT_PE | UART_INT_FE);

    /* Return the handle */
    return (handle);
}

/*
 *  ======== UARTMSP432E4_read ========
 */
int_fast32_t UARTMSP432E4_read(UART_Handle handle, void *buffer, size_t size)
{
    uintptr_t               key;
    UARTMSP432E4_Object    *object = handle->object;

    key = HwiP_disable();

    if ((object->state.readMode == UART_MODE_CALLBACK) && object->readSize) {
        HwiP_restore(key);

        return (UART_ERROR);
    }

    /* Save the data to be read and restore interrupts. */
    object->readBuf = buffer;
    object->readSize = size;
    object->readCount = size;

    HwiP_restore(key);

    return ((int_fast32_t)(object->readFxns.readTaskFxn(handle)));
}

/*
 *  ======== UARTMSP432E4_readCancel ========
 */
void UARTMSP432E4_readCancel(UART_Handle handle)
{
    uintptr_t               key;
    UARTMSP432E4_Object    *object = handle->object;

    if ((object->state.readMode != UART_MODE_CALLBACK) ||
        (object->readSize == 0)) {
        return;
    }

    key = HwiP_disable();

    object->state.drainByISR = false;
    /*
     * Indicate that what we've currently received is what we asked for so that
     * the existing logic handles the completion.
     */
    object->readSize -= object->readCount;
    object->readCount = 0;

    HwiP_restore(key);

    object->readFxns.readTaskFxn(handle);
}

/*
 *  ======== UARTMSP432E4_readPolling ========
 */
int_fast32_t UARTMSP432E4_readPolling(UART_Handle handle, void *buf, size_t size)
{
    int_fast32_t                count = 0;
    UARTMSP432E4_Object        *object = handle->object;
    UARTMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    unsigned char              *buffer = (unsigned char *)buf;

    /* Read characters. */
    while (size) {
        /* Grab data from the RingBuf before getting it from the RX data reg */
        UARTIntDisable(hwAttrs->baseAddr, UART_INT_RX | UART_INT_RT);
        if (ringBufGet(handle, buffer) == -1) {
            *buffer = UARTCharGet(hwAttrs->baseAddr);
        }
        if (object->state.rxEnabled) {
            UARTIntEnable(hwAttrs->baseAddr, UART_INT_RX | UART_INT_RT);
        }

        count++;
        size--;

        if (object->state.readDataMode == UART_DATA_TEXT && *buffer == '\r') {
            /* Echo character if enabled. */
            if (object->state.readEcho) {
                UARTCharPut(hwAttrs->baseAddr, '\r');
            }
            *buffer = '\n';
        }

        /* Echo character if enabled. */
        if (object->state.readDataMode == UART_DATA_TEXT &&
                object->state.readEcho) {
            UARTCharPut(hwAttrs->baseAddr, *buffer);
        }

        /* If read return mode is newline, finish if a newline was received. */
        if (object->state.readDataMode == UART_DATA_TEXT &&
                object->state.readReturnMode == UART_RETURN_NEWLINE &&
                *buffer == '\n') {
            return (count);
        }

        buffer++;
    }

    return (count);
}

/*
 *  ======== UARTMSP432E4_write ========
 */
int_fast32_t UARTMSP432E4_write(UART_Handle handle, const void *buffer, size_t size)
{
    uintptr_t                   key;
    UARTMSP432E4_Object        *object = handle->object;
    UARTMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;

    if (!size) {
        return 0;
    }

    key = HwiP_disable();

    if (object->writeCount || UARTBusy(hwAttrs->baseAddr)) {
        HwiP_restore(key);
        return (UART_ERROR);
    }

    /* Save the data to be written and restore interrupts. */
    object->writeBuf = buffer;
    object->writeSize = size;
    object->writeCount = size;

    HwiP_restore(key);

    if (!(UARTIntStatus(hwAttrs->baseAddr, false) & UART_INT_TX)) {
        /*
         *  Start the transfer going if the raw interrupt status TX bit
         *  is 0.  This will cause the ISR to fire when we enable
         *  UART_INT_TX.  If the RIS TX bit is not cleared, we don't
         *  need to call writeData(), since the ISR will fire once we
         *  enable the interrupt, causing the transfer to start.
         */
        writeData(handle);
    }
    if (object->writeCount) {
        UARTTxIntModeSet(hwAttrs->baseAddr, UART_TXINT_MODE_FIFO);
        UARTIntEnable(hwAttrs->baseAddr, UART_INT_TX);
    }

    /* If writeMode is blocking, block and get the state. */
    if (object->state.writeMode == UART_MODE_BLOCKING) {
        /* Pend on semaphore and wait for Hwi to finish. */
        if (SemaphoreP_pend(object->writeSem, object->writeTimeout) !=
                SemaphoreP_OK) {
            /* Semaphore timed out, make the write empty and log the write. */
            UARTIntDisable(hwAttrs->baseAddr, UART_INT_TX);
            UARTIntClear(hwAttrs->baseAddr, UART_INT_TX);
            object->writeCount = 0;
        }
        return ((int_fast32_t)(object->writeSize - object->writeCount));
    }

    return (0);
}

/*
 *  ======== UARTMSP432E4_writeCancel ========
 */
void UARTMSP432E4_writeCancel(UART_Handle handle)
{
    uintptr_t                    key;
    UARTMSP432E4_Object         *object = handle->object;
    UARTMSP432E4_HWAttrs const  *hwAttrs = handle->hwAttrs;
    unsigned int                 written;

    key = HwiP_disable();

    /* Return if there is no write. */
    if (!object->writeCount) {
        HwiP_restore(key);
        return;
    }

    /* Set size = 0 to prevent writing and restore interrupts. */
    written = object->writeCount;
    object->writeCount = 0;
    UARTIntDisable(hwAttrs->baseAddr, UART_INT_TX);
    UARTIntClear(hwAttrs->baseAddr, UART_INT_TX);

    HwiP_restore(key);

    /* Reset the write buffer so we can pass it back */
    object->writeCallback(handle, (void *)object->writeBuf,
        object->writeSize - written);
}

/*
 *  ======== UARTMSP432E4_writePolling ========
 */
int_fast32_t UARTMSP432E4_writePolling(UART_Handle handle, const void *buf, size_t size)
{
    int_fast32_t                count = 0;
    UARTMSP432E4_Object        *object = handle->object;
    UARTMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    unsigned char              *buffer = (unsigned char *)buf;

    /* Write characters. */
    while (size) {
        if (object->state.writeDataMode == UART_DATA_TEXT && *buffer == '\n') {
            UARTCharPut(hwAttrs->baseAddr, '\r');
            count++;
        }
        UARTCharPut(hwAttrs->baseAddr, *buffer);

        buffer++;
        count++;
        size--;
    }

    while (UARTBusy(hwAttrs->baseAddr)) {
        ;
    }

    return (count);
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
 *  ======== readBlockingTimeout ========
 */
static void readBlockingTimeout(uintptr_t arg)
{
    UARTMSP432E4_Object *object = ((UART_Handle)arg)->object;
    object->state.bufTimeout = true;
    SemaphoreP_post(object->readSem);
}

/*
 *  ======== readIsrBinaryBlocking ========
 *  Function that is called by the ISR
 */
static bool readIsrBinaryBlocking(UART_Handle handle)
{
    UARTMSP432E4_Object           *object = handle->object;
    UARTMSP432E4_HWAttrs const    *hwAttrs = handle->hwAttrs;
    int32_t                        readIn;

    while (UARTCharsAvail(hwAttrs->baseAddr)) {
        /*
         *  If the Ring buffer is full, leave the data in the FIFO.
         *  This will allow flow control to work, if it is enabled.
         */
        if (RingBuf_isFull(&object->ringBuffer)) {
            return (false);
        }

        readIn = UARTCharGetNonBlocking(hwAttrs->baseAddr);
        /*
         *  Bits 0-7 contain the data, bits 8-11 are used for error codes.
         *  (Bits 12-31 are reserved and read as 0)  If readIn > 0xFF, an
         *  error has occurred.
         */
        if (readIn > 0xFF) {
            if (hwAttrs->errorFxn) {
                hwAttrs->errorFxn(handle, (uint32_t)((readIn >> 8) & 0xF));
            }
            UARTRxErrorClear(hwAttrs->baseAddr);
            return (false);
        }

        RingBuf_put(&object->ringBuffer, (unsigned char)readIn);

        if (object->state.callCallback) {
            object->state.callCallback = false;
            object->readCallback(handle, NULL, 0);
        }
    }
    return (true);
}

/*
 *  ======== readIsrBinaryCallback ========
 */
static bool readIsrBinaryCallback(UART_Handle handle)
{
    UARTMSP432E4_Object        *object = handle->object;
    UARTMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    int32_t                     readIn;
    bool                        ret = true;

    while (UARTCharsAvail(hwAttrs->baseAddr)) {
        if (RingBuf_isFull(&object->ringBuffer)) {
            ret = false;
            break;
        }

        readIn = UARTCharGetNonBlocking(hwAttrs->baseAddr);
        if (readIn > 0xFF) {
            if (hwAttrs->errorFxn) {
                hwAttrs->errorFxn(handle, (uint32_t)((readIn >> 8) & 0xF));
            }
            UARTRxErrorClear(hwAttrs->baseAddr);
            ret = false;
            break;
        }

        RingBuf_put(&object->ringBuffer, (unsigned char)readIn);
    }

    /*
     * Check and see if a UART_read in callback mode told use to continue
     * servicing the user buffer...
     */
    if (object->state.drainByISR) {
        readTaskCallback(handle);
    }

    return (ret);
}

/*
 *  ======== readIsrTextBlocking ========
 */
static bool readIsrTextBlocking(UART_Handle handle)
{
    UARTMSP432E4_Object        *object = handle->object;
    UARTMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    int32_t                     readIn;

    while (UARTCharsAvail(hwAttrs->baseAddr)) {
        /*
         *  If the Ring buffer is full, leave the data in the FIFO.
         *  This will allow flow control to work, if it is enabled.
         */
        if (RingBuf_isFull(&object->ringBuffer)) {
            return (false);
        }

        readIn = UARTCharGetNonBlocking(hwAttrs->baseAddr);
        if (readIn > 0xFF) {
            if (hwAttrs->errorFxn) {
                hwAttrs->errorFxn(handle, (uint32_t)((readIn >> 8) & 0xF));
            }
            UARTRxErrorClear(hwAttrs->baseAddr);
            return (false);
        }

        if (readIn == '\r') {
            /* Echo character if enabled. */
            if (object->state.readEcho) {
                UARTCharPut(hwAttrs->baseAddr, '\r');
            }
            readIn = '\n';
        }
        RingBuf_put(&object->ringBuffer, (unsigned char)readIn);

        if (object->state.readEcho) {
            UARTCharPut(hwAttrs->baseAddr, (unsigned char)readIn);
        }
        if (object->state.callCallback) {
            object->state.callCallback = false;
            object->readCallback(handle, NULL, 0);
        }
    }
    return (true);
}

/*
 *  ======== readIsrTextCallback ========
 */
static bool readIsrTextCallback(UART_Handle handle)
{
    UARTMSP432E4_Object        *object = handle->object;
    UARTMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    int32_t                     readIn;
    bool                        ret = true;

    while (UARTCharsAvail(hwAttrs->baseAddr)) {
        /*
         *  If the Ring buffer is full, leave the data in the FIFO.
         *  This will allow flow control to work, if it is enabled.
         */
        if (RingBuf_isFull(&object->ringBuffer)) {
            ret = false;
            break;
        }

        readIn = UARTCharGetNonBlocking(hwAttrs->baseAddr);
        if (readIn > 0xFF) {
            if (hwAttrs->errorFxn) {
                hwAttrs->errorFxn(handle, (uint32_t)((readIn >> 8) & 0xF));
            }
            UARTRxErrorClear(hwAttrs->baseAddr);
            ret = false;
            break;
        }

        if (readIn == '\r') {
            /* Echo character if enabled. */
            if (object->state.readEcho) {
                UARTCharPut(hwAttrs->baseAddr, '\r');
            }
            readIn = '\n';
        }
        RingBuf_put(&object->ringBuffer, (unsigned char)readIn);

        if (object->state.readEcho) {
            UARTCharPut(hwAttrs->baseAddr, (unsigned char)readIn);
        }
    }

    /*
     * Check and see if a UART_read in callback mode told use to continue
     * servicing the user buffer...
     */
    if (object->state.drainByISR) {
        readTaskCallback(handle);
    }

    return (ret);
}

/*
 *  ======== readSemCallback ========
 *  Simple callback to post a semaphore for the blocking mode.
 */
static void readSemCallback(UART_Handle handle, void *buffer, size_t count)
{
    UARTMSP432E4_Object *object = handle->object;

    SemaphoreP_post(object->readSem);
}

/*
 *  ======== readTaskBlocking ========
 */
static int readTaskBlocking(UART_Handle handle)
{
    unsigned char            readIn;
    uintptr_t                key;
    UARTMSP432E4_Object     *object = handle->object;
    unsigned char           *buffer = object->readBuf;

    object->state.bufTimeout = false;
    object->state.callCallback = false;

    /*
     * It is possible for the object->timeoutClk and the callback function to
     * have posted the object->readSem Semaphore from the previous UART_read
     * call (if the code below didn't get to stop the clock object in time).
     * To clear this, we simply do a NO_WAIT pend on (binary) object->readSem
     * so that it resets the Semaphore count.
     */
    SemaphoreP_pend(object->readSem, SemaphoreP_NO_WAIT);

    if ((object->readTimeout != 0) &&
            (object->readTimeout != UART_WAIT_FOREVER)) {
        ClockP_setTimeout(object->timeoutClk, object->readTimeout);
        ClockP_start(object->timeoutClk);
    }

    while (object->readCount) {
        key = HwiP_disable();

        if (ringBufGet(handle, &readIn) < 0) {
            object->state.callCallback = true;
            HwiP_restore(key);

            if (object->readTimeout == 0) {
                break;
            }

            SemaphoreP_pend(object->readSem, SemaphoreP_WAIT_FOREVER);
            if (object->state.bufTimeout == true) {
                break;
            }
            ringBufGet(handle, &readIn);
        }
        else {
            HwiP_restore(key);
        }

        *buffer = readIn;
        buffer++;
        /* In blocking mode, readCount doesn't not need a lock */
        object->readCount--;

        if (object->state.readDataMode == UART_DATA_TEXT &&
                object->state.readReturnMode == UART_RETURN_NEWLINE &&
                readIn == '\n') {
            break;
        }
    }

    ClockP_stop(object->timeoutClk);
    return (object->readSize - object->readCount);
}

/*
 *  ======== readTaskCallback ========
 *  This function is called the first time by the UART_read task and tries to
 *  get all the data it can get from the ringBuffer. If it finished, it will
 *  perform the user supplied callback. If it didn't finish, the ISR must handle
 *  the remaining data. By setting the drainByISR flag, the UART_read function
 *  handed over the responsibility to get the remaining data to the ISR.
 */
static int readTaskCallback(UART_Handle handle)
{
    uintptr_t               key;
    UARTMSP432E4_Object    *object = handle->object;
    unsigned char           readIn;
    unsigned char          *bufferEnd;
    bool                    makeCallback = false;
    size_t                  tempCount;

    object->state.drainByISR = false;
    bufferEnd = (unsigned char*) object->readBuf + object->readSize;

    while (object->readCount) {
        key = HwiP_disable();
        if (ringBufGet(handle, &readIn) < 0) {
            /* Not all data has been read */
            object->state.drainByISR = true;
            HwiP_restore(key);
            break;
        }
        HwiP_restore(key);

        *(unsigned char *) (bufferEnd - object->readCount *
            sizeof(unsigned char)) = readIn;

        object->readCount--;

        if ((object->state.readDataMode == UART_DATA_TEXT) &&
                (object->state.readReturnMode == UART_RETURN_NEWLINE) &&
                (readIn == '\n')) {
            makeCallback = true;
            break;
        }
    }

    if (!object->readCount || makeCallback) {
        object->state.readCallbackPending = true;
        if (object->state.inReadCallback == false) {
            while (object->state.readCallbackPending) {
                object->state.readCallbackPending = false;
                tempCount = object->readSize;
                object->readSize = 0;

                object->state.inReadCallback = true;
                object->readCallback(handle, object->readBuf,
                        tempCount - object->readCount);
                object->state.inReadCallback = false;
            }
        }
    }

    return (0);
}

/*
 *  ======== ringBufGet ========
 */
static int ringBufGet(UART_Handle handle, unsigned char *data)
{
    UARTMSP432E4_Object    *object = handle->object;
    UARTMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    uintptr_t               key;
    int32_t                 readIn;
    int                     count;

    key = HwiP_disable();

    if (RingBuf_isFull(&object->ringBuffer)) {
        count = RingBuf_get(&object->ringBuffer, data);

        readIn = UARTCharGetNonBlocking(hwAttrs->baseAddr);
        if (readIn != -1) {
            RingBuf_put(&object->ringBuffer, (unsigned char)readIn);
            count++;
        }
        HwiP_restore(key);
    }
    else {
        count = RingBuf_get(&object->ringBuffer, data);
        HwiP_restore(key);
    }

    return (count);
}

/*
 *  ======== writeData ========
 */
static void writeData(UART_Handle handle)
{
    UARTMSP432E4_Object        *object = handle->object;
    UARTMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    unsigned char              *writeOffset;

    writeOffset = (unsigned char *)object->writeBuf +
        object->writeSize * sizeof(unsigned char);
    while (object->writeCount) {
        if (!UARTCharPutNonBlocking(hwAttrs->baseAddr,
                *(writeOffset - object->writeCount))) {
            /* TX FIFO is FULL */
            break;
        }
        if ((object->state.writeDataMode == UART_DATA_TEXT) &&
            (*(writeOffset - object->writeCount) == '\n')) {
            UARTCharPut(hwAttrs->baseAddr, '\r');
        }
        object->writeCount--;
    }

    if (!object->writeCount) {
        UARTIntDisable(hwAttrs->baseAddr, UART_INT_TX);
        UARTIntClear(hwAttrs->baseAddr, UART_INT_TX);

        /*
         *  Set TX interrupt for end of transmission mode.
         *  The TXRIS bit will be set only when all the data
         *  (including stop bits) have left the serializer.
         */
        UARTTxIntModeSet(hwAttrs->baseAddr, UART_TXINT_MODE_EOT);

        if (!UARTBusy(hwAttrs->baseAddr)) {
            object->writeCallback(handle, (void *)object->writeBuf,
                    object->writeSize);
        }
        else {
            UARTIntEnable(hwAttrs->baseAddr, UART_INT_TX);
        }
    }
}

/*
 *  ======== writeSemCallback ========
 *  Simple callback to post a semaphore for the blocking mode.
 */
static void writeSemCallback(UART_Handle handle, void *buffer, size_t count)
{
    UARTMSP432E4_Object *object = handle->object;

    SemaphoreP_post(object->writeSem);
}
