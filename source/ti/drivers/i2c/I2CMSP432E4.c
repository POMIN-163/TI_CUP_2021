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

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <ti/devices/msp432e4/inc/msp432.h>

#include <ti/devices/msp432e4/driverlib/gpio.h>
#include <ti/devices/msp432e4/driverlib/i2c.h>
#include <ti/devices/msp432e4/driverlib/inc/hw_i2c.h>
#include <ti/devices/msp432e4/driverlib/types.h>
#include <ti/devices/msp432e4/driverlib/sysctl.h>
#include <ti/devices/msp432e4/driverlib/rom_map.h>

#include <ti/drivers/dpl/ClockP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/gpio/GPIOMSP432E4.h>
#include <ti/drivers/i2c/I2CMSP432E4.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432E4.h>

#define MSP432E4_ERROR_INTS (I2C_MASTER_INT_NACK | \
    I2C_MASTER_INT_ARB_LOST | I2C_MASTER_INT_TIMEOUT)

#define I2CMSP432E4_TRANSFER_INTS (I2C_MASTER_INT_STOP | \
    MSP432E4_ERROR_INTS)

#define I2CMSP432E4_FIFO_SIZE 8
#define I2CMSP432E4_MAX_BURST 255

/* Prototypes */
void I2CMSP432E4_cancel(I2C_Handle handle);
void I2CMSP432E4_close(I2C_Handle handle);
int_fast16_t I2CMSP432E4_control(I2C_Handle handle, uint_fast16_t cmd,
    void *arg);
void I2CMSP432E4_init(I2C_Handle handle);
I2C_Handle I2CMSP432E4_open(I2C_Handle handle,
    I2C_Params *params);
void I2CMSP432E4_reset(uint32_t baseAddress);
int_fast16_t I2CMSP432E4_transfer(I2C_Handle handle,
    I2C_Transaction *transaction, uint32_t timeout);

static void I2CMSP432E4_blockingCallback(I2C_Handle handle,
    I2C_Transaction *msg, bool transferStatus);
static void I2CMSP432E4_completeTransfer(I2C_Handle handle);
static void I2CMSP432E4_initHw(I2C_Handle handle);
static uint32_t I2CMSP432E4_getPowerMgrId(uint32_t baseAddress);
static uint32_t I2CMSP432E4_getSysCtrlId(uint32_t baseAddress);
static void I2CMSP432E4_primeWriteBurst(I2CMSP432E4_Object *object,
    I2CMSP432E4_HWAttrs const *hwAttrs);
static void I2CMSP432E4_primeReadBurst(I2CMSP432E4_Object  *object,
    I2CMSP432E4_HWAttrs const *hwAttrs);
static int_fast16_t I2CMSP432E4_primeTransfer(I2CMSP432E4_Object  *object,
    I2CMSP432E4_HWAttrs const *hwAttrs, I2C_Transaction *transaction);
static void I2CMSP432E4_readRecieveFifo(I2CMSP432E4_Object  *object,
    I2CMSP432E4_HWAttrs const *hwAttrs);
static void I2CMSP432E4_updateReg(uint32_t reg, uint32_t mask, uint32_t val);

/* I2C function table for I2CMSP432E4 implementation */
const I2C_FxnTable I2CMSP432E4_fxnTable = {
    I2CMSP432E4_cancel,
    I2CMSP432E4_close,
    I2CMSP432E4_control,
    I2CMSP432E4_init,
    I2CMSP432E4_open,
    I2CMSP432E4_transfer
};

/*
 *  ======== I2CMSP432E4_blockingCallback ========
 */
static void I2CMSP432E4_blockingCallback(I2C_Handle handle,
    I2C_Transaction *msg, bool transferStatus)
{
    I2CMSP432E4_Object  *object = handle->object;

    /* Indicate transfer complete */
    SemaphoreP_post(object->transferComplete);
}

/*
 *  ======== I2CMSP432E4_fillTransmitFifo ========
 */
static inline void I2CMSP432E4_fillTransmitFifo(I2CMSP432E4_Object *object,
    I2CMSP432E4_HWAttrs const *hwAttrs)
{
    while(object->writeCount && object->burstCount &&
        I2CFIFODataPutNonBlocking(hwAttrs->baseAddr, *(object->writeBuf))) {

        object->writeBuf++;
        object->writeCount--;
        object->burstCount--;
    }

    I2CMasterIntClearEx(hwAttrs->baseAddr, I2C_MASTER_INT_TX_FIFO_EMPTY);
}

/*
 *  ======== I2CMSP432E4_initHw ========
 */
static void I2CMSP432E4_initHw(I2C_Handle handle)
{
    I2CMSP432E4_Object *object = handle->object;
    I2CMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    ClockP_FreqHz freq;

    /* Disable all interrupts */
    I2CMasterIntDisableEx(hwAttrs->baseAddr, 0xFFFFFFFF);

    /* Get CPU Frequency */
    ClockP_getCpuFreq(&freq);

    /*
     *  Driverlib does not support configuring 1Mbps
     */
    if (object->bitRate == I2C_1000kHz) {
        I2CMasterEnable(hwAttrs->baseAddr);

        /*
         * The timer period (TP) is calculated using the following:
         * TP = (System Clock / (2 * (SCL_LP + SCL_HP) * SCL)) - 1
         * The system clock is fixed to 120MHz.
         * The SCL low period is fixed to 6.
         * The SCL high period is fixed to 4.
         * The SCL represents the desired serial clock speed, 1,000,000.
         */
        HWREG(hwAttrs->baseAddr + I2C_O_MTPR) =
            ((freq.lo + (2 * 10 * 1000000) - 1) / (2 * 10 * 1000000)) - 1;
    }
    else {
        /* I2CMasterEnable() invoked in this call */
        I2CMasterInitExpClk(hwAttrs->baseAddr, freq.lo,
            object->bitRate > I2C_100kHz);
    }

    /* Flush the FIFOs. They must be empty before re-assignment */
    I2CTxFIFOFlush(hwAttrs->baseAddr);
    I2CRxFIFOFlush(hwAttrs->baseAddr);

    /* Set TX and RX FIFOs to master mode */
    I2CTxFIFOConfigSet(hwAttrs->baseAddr, I2C_FIFO_CFG_TX_MASTER);
    I2CRxFIFOConfigSet(hwAttrs->baseAddr, I2C_FIFO_CFG_RX_MASTER);

    /* Clear any pending interrupts */
    I2CMasterIntClearEx(hwAttrs->baseAddr, 0xFFFFFFFF);
}

/*
 *  ======== I2CMSP432E4_completeTransfer ========
 */
static void I2CMSP432E4_completeTransfer(I2C_Handle handle)
{
    I2CMSP432E4_Object *object = handle->object;
    I2CMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Disable and clear all interrupts */
    I2CMasterIntDisableEx(hwAttrs->baseAddr, 0xFFFFFFFF);
    I2CMasterIntClearEx(hwAttrs->baseAddr, 0xFFFFFFFF);

    /*
     *  If this current transaction was canceled, we need to cancel
     *  any queued transactions
     */
    if (object->currentTransaction->status == I2C_STATUS_CANCEL) {

        /*
         * Allow callback to run. If in CALLBACK mode, the application
         * may initiate a transfer in the callback which will call
         * primeTransfer().
         */
        object->transferCallbackFxn(handle, object->currentTransaction, false);

        /*
         * After each callback, set the transaction status as the application
         * may have modified the status.
         */
        object->currentTransaction->status = I2C_STATUS_CANCEL;

        /* also dequeue and call the transfer callbacks for any queued transfers */
        while (object->headPtr != object->tailPtr) {
            object->headPtr = object->headPtr->nextPtr;
            object->headPtr->status = I2C_STATUS_CANCEL;
            object->transferCallbackFxn(handle, object->headPtr, false);
            object->headPtr->status = I2C_STATUS_CANCEL;
        }

        /* clean up object */
        object->currentTransaction = NULL;
        object->headPtr = NULL;
        object->tailPtr = NULL;

        /*
         * The I2C peripheral may be in an undefined state.
         * Perform a peripheral reset and re initialize the hardware.
         */
        I2CMSP432E4_reset(hwAttrs->baseAddr);
        I2CMSP432E4_initHw(handle);
    }
    else if (object->currentTransaction->status == I2C_STATUS_TIMEOUT) {
        /*
         * This can only occur in BLOCKING mode. No need to call the blocking
         * callback as the Semaphore has already timed out.
         */
        object->headPtr = NULL;
        object->tailPtr = NULL;
    }
    /* See if we need to process any other transactions */
    else if (object->headPtr == object->tailPtr) {

        /* No other transactions need to occur */
        object->headPtr = NULL;
        object->tailPtr = NULL;

        /*
         * Allow callback to run. If in CALLBACK mode, the application
         * may initiate a transfer in the callback which will call
         * primeTransfer().
         */
        object->transferCallbackFxn(handle, object->currentTransaction,
            (object->currentTransaction->status == I2C_STATUS_SUCCESS));
    }
    else {
        /* Another transfer needs to take place */
        object->headPtr = object->headPtr->nextPtr;

        /*
         * Allow application callback to run. The application may
         * initiate a transfer in the callback which will add an
         * additional transfer to the queue.
         */
        object->transferCallbackFxn(handle, object->currentTransaction,
            (object->currentTransaction->status == I2C_STATUS_SUCCESS));

        I2CMSP432E4_primeTransfer(object,
            (I2CMSP432E4_HWAttrs const *) handle->hwAttrs, object->headPtr);
    }
}

/*
 *  ======== I2CMSP432E4_getPowerMgrId ========
 */
static uint32_t I2CMSP432E4_getPowerMgrId(uint32_t baseAddress)
{
    switch (baseAddress) {
        case I2C0_BASE:
            return (PowerMSP432E4_PERIPH_I2C0);
        case I2C1_BASE:
            return (PowerMSP432E4_PERIPH_I2C1);
        case I2C2_BASE:
            return (PowerMSP432E4_PERIPH_I2C2);
        case I2C3_BASE:
            return (PowerMSP432E4_PERIPH_I2C3);
        case I2C4_BASE:
            return (PowerMSP432E4_PERIPH_I2C4);
        case I2C5_BASE:
            return (PowerMSP432E4_PERIPH_I2C5);
        case I2C6_BASE:
            return (PowerMSP432E4_PERIPH_I2C6);
        case I2C7_BASE:
            return (PowerMSP432E4_PERIPH_I2C7);
        case I2C8_BASE:
            return (PowerMSP432E4_PERIPH_I2C8);
        case I2C9_BASE:
            return (PowerMSP432E4_PERIPH_I2C9);
        default:
            return (~0);
    }
}

/*
 *  ======== I2CMSP432E4_getSysCtrlId ========
 */
static uint32_t I2CMSP432E4_getSysCtrlId(uint32_t baseAddress)
{
    switch (baseAddress) {
        case I2C0_BASE:
            return (SYSCTL_PERIPH_I2C0);
        case I2C1_BASE:
            return (SYSCTL_PERIPH_I2C1);
        case I2C2_BASE:
            return (SYSCTL_PERIPH_I2C2);
        case I2C3_BASE:
            return (SYSCTL_PERIPH_I2C3);
        case I2C4_BASE:
            return (SYSCTL_PERIPH_I2C4);
        case I2C5_BASE:
            return (SYSCTL_PERIPH_I2C5);
        case I2C6_BASE:
            return (SYSCTL_PERIPH_I2C6);
        case I2C7_BASE:
            return (SYSCTL_PERIPH_I2C7);
        case I2C8_BASE:
            return (SYSCTL_PERIPH_I2C8);
        case I2C9_BASE:
            return (SYSCTL_PERIPH_I2C9);
        default:
            return (~0);
    }
}

/*
 *  ======== I2CMSP432E4_hwiFxn ========
 *  Hwi interrupt handler to service the I2C peripheral
 *
 *  The handler is a generic handler for a I2C object.
 */
static void I2CMSP432E4_hwiFxn(uintptr_t arg)
{
    /* Get the pointer to the object and hwAttrs */
    I2CMSP432E4_Object *object = ((I2C_Handle)arg)->object;
    I2CMSP432E4_HWAttrs const *hwAttrs = ((I2C_Handle)arg)->hwAttrs;
    uint32_t intStatus;

    /* Read and clear masked interrupts */
    intStatus = I2CMasterIntStatusEx(hwAttrs->baseAddr, true);
    I2CMasterIntClearEx(hwAttrs->baseAddr, intStatus);

    /*
     * If we receive a NACK && the MSA still contains the mastercode, we just
     * sent out the mastercode for a 3Mbps transmission.
     */
    if ((intStatus & I2C_MASTER_INT_NACK) &&
        (HWREG(hwAttrs->baseAddr + I2C_O_MSA) == hwAttrs->masterCode)) {

        /* If arbitration was lost during transmitting the master code */
        if (intStatus & I2C_MASTER_INT_ARB_LOST) {
            /*
             * Another I2C master owns the bus.
             */
            I2CMasterIntDisableEx(hwAttrs->baseAddr, 0xFFFFFFFF);
            object->currentTransaction->status = I2C_STATUS_ARB_LOST;
            HWREG(hwAttrs->baseAddr + I2C_O_MCS) = I2C_MCS_STOP;
            I2CMSP432E4_completeTransfer((I2C_Handle) arg);
        }
        else if (object->writeCount) {
            I2CMSP432E4_primeWriteBurst(object, hwAttrs);
        }
        else {
            I2CMSP432E4_primeReadBurst(object, hwAttrs);
        }
    }
    else if (intStatus & MSP432E4_ERROR_INTS) {

        /* The MSR contains all error bits */
        uint32_t status = HWREG(hwAttrs->baseAddr + I2C_O_MCS);

        /* Decode interrupt status */
        if (status & I2C_MCS_ARBLST) {
            object->currentTransaction->status = I2C_STATUS_ARB_LOST;
        }
        else if (status & I2C_MCS_DATACK) {
            object->currentTransaction->status = I2C_STATUS_DATA_NACK;
        }
        else if (status & I2C_MCS_ADRACK) {
            object->currentTransaction->status = I2C_STATUS_ADDR_NACK;
        }
        else if (status & I2C_MCS_CLKTO) {
            object->currentTransaction->status = I2C_STATUS_CLOCK_TIMEOUT;
        }
        else {
            object->currentTransaction->status = I2C_STATUS_ERROR;
        }

        /* If arbitration is lost, another I2C master owns the bus */
        if (status & I2C_MCS_ARBLST) {
            I2CMSP432E4_completeTransfer((I2C_Handle) arg);
            return;
        }

        /* If the stop interrupt has not occurred, force a STOP condition */
        if (!(intStatus & I2C_MASTER_INT_STOP)) {
            HWREG(hwAttrs->baseAddr + I2C_O_MCS) = I2C_MCS_STOP;
            return;
        }
    }
    else if (intStatus & I2C_MASTER_INT_TX_FIFO_EMPTY) {

        /* If there is more data to transmit */
        if (object->writeCount) {

            /* If we need to configure a new burst */
            if (0 == object->burstCount) {
                I2CMSP432E4_primeWriteBurst(object, hwAttrs);
            }
            else {
                I2CMSP432E4_fillTransmitFifo(object, hwAttrs);
            }
        }
        /* Finished writing, is there data to receive? */
        else if (object->readCount) {

            /* Disable TX interrupt */
            I2CMasterIntDisableEx(hwAttrs->baseAddr,
                I2C_MASTER_INT_TX_FIFO_EMPTY);

            object->burstStarted = false;
            I2CMSP432E4_primeReadBurst(object, hwAttrs);
        }
    }
    else if (intStatus & I2C_MASTER_INT_RX_FIFO_REQ ||
       !(HWREG(hwAttrs->baseAddr + I2C_O_FIFOSTATUS) & I2C_FIFOSTATUS_RXFE)) {

        I2CMSP432E4_readRecieveFifo(object, hwAttrs);

        /* If we need to configure a new burst */
        if (object->readCount && (0 == object->burstCount)) {
            I2CMSP432E4_primeReadBurst(object, hwAttrs);
        }
    }

    /* If the STOP condition was set, complete the transfer */
    if (intStatus & I2C_MASTER_INT_STOP) {

        /* If the transaction completed, update the transaction status */
        if (object->currentTransaction->status == I2C_STATUS_INCOMPLETE &&
            (!(object->readCount || object->writeCount))) {

            uint32_t fifoStatus = I2CFIFOStatus(hwAttrs->baseAddr) &
                (I2C_FIFO_TX_EMPTY | I2C_FIFO_RX_EMPTY);

            /* If both FIFOs are empty */
            if (fifoStatus == (I2C_FIFO_RX_EMPTY | I2C_FIFO_TX_EMPTY)) {
                object->currentTransaction->status = I2C_STATUS_SUCCESS;
            }
        }

        I2CMSP432E4_completeTransfer((I2C_Handle) arg);
    }
}

/*
 *  ======== I2CMSP432E4_primeReadBurst =======
 */
static void I2CMSP432E4_primeReadBurst(I2CMSP432E4_Object *object,
    I2CMSP432E4_HWAttrs const *hwAttrs)
{
    uint32_t command;

    /* Specify the I2C slave address in receive mode */
    I2CMasterSlaveAddrSet(hwAttrs->baseAddr,
        object->currentTransaction->slaveAddress, true);

    /* Determine the size of this burst */
    if(object->readCount > I2CMSP432E4_MAX_BURST) {
        object->burstCount = I2CMSP432E4_MAX_BURST;
    }
    else {
        object->burstCount = object->readCount;
    }

    /*
     * If we are reading less than FIFO_SIZE bytes, set the RX FIFO
     * REQ interrupt to trigger when we finish burstCount bytes.
     * Else we are reading more than 8 bytes. set the RX FIFO REQ
     * interrupt to trigger when full. If we are reading less than RXTRIG
     * bytes during the final byte reads, the RXDONE interrupt will allow
     * us to complete the read transaction.
     */
    if(object->burstCount < I2CMSP432E4_FIFO_SIZE) {
        I2CMSP432E4_updateReg(hwAttrs->baseAddr + I2C_O_FIFOCTL,
            I2C_FIFOCTL_RXTRIG_M, object->burstCount << I2C_FIFOCTL_RXTRIG_S);
    }
    else {
        I2CMSP432E4_updateReg(hwAttrs->baseAddr + I2C_O_FIFOCTL,
            I2C_FIFOCTL_RXTRIG_M, I2C_FIFOCTL_RXTRIG_M);
    }

    I2CMasterBurstLengthSet(hwAttrs->baseAddr, object->burstCount);

    /* If we will be sending multiple bursts */
    if(object->readCount > I2CMSP432E4_MAX_BURST) {
         /* Setting the ACK enable ensures we ACK the last byte of a burst */
         command = (I2C_MCS_BURST | I2C_MCS_ACK);
    }
    else {
         command = (I2C_MCS_BURST | I2C_MCS_STOP);
    }

    /* Only generate a start condition if the burst hasn't started */
    if (!object->burstStarted) {
        command |= I2C_MCS_START;
        object->burstStarted = true;
    }

    I2CMasterIntEnableEx(hwAttrs->baseAddr, I2C_MASTER_INT_RX_FIFO_REQ |
        I2CMSP432E4_TRANSFER_INTS);

    HWREG(hwAttrs->baseAddr + I2C_O_MCS) = command;
}

/*
 *  ======== I2CMSP432E4_primeWriteBurst =======
 */
static void I2CMSP432E4_primeWriteBurst(I2CMSP432E4_Object  *object,
    I2CMSP432E4_HWAttrs const *hwAttrs)
{
    uint32_t command;

    /* Write I2C Slave Address and transmit direction */
    I2CMasterSlaveAddrSet(hwAttrs->baseAddr,
        object->currentTransaction->slaveAddress, false);

    /* Determine the size of this burst */
    if(object->writeCount > I2CMSP432E4_MAX_BURST) {
        object->burstCount = I2CMSP432E4_MAX_BURST;
    }
    else {
        object->burstCount = object->writeCount;
    }

    /* Write burst length */
    I2CMasterBurstLengthSet(hwAttrs->baseAddr,
        object->burstCount);

    /* If we will be sending multiple bursts */
    if (object->readCount || object->writeCount > I2CMSP432E4_MAX_BURST) {
        command = I2C_MCS_BURST;
    }
    else {
        command = I2C_MCS_BURST | I2C_MCS_STOP;
    }

    /* Only generate a start condition if the burst hasn't started */
    if (!object->burstStarted) {
        command |= I2C_MCS_START;
        object->burstStarted = true;
    }

    /* Fill transmit FIFO. This will modify the object counts */
    I2CMSP432E4_fillTransmitFifo(object, hwAttrs);

    /* Enable TXFIFOEMPTY interrupt and other standard transfer interrupts */
    I2CMasterIntEnableEx(hwAttrs->baseAddr, I2C_MASTER_INT_TX_FIFO_EMPTY
        | I2CMSP432E4_TRANSFER_INTS);

    /* Write the Master Control & Status register */
    HWREG(hwAttrs->baseAddr + I2C_O_MCS) = command;
}

/*
 *  ======== I2CMSP432E4_primeTransfer =======
 */
static int_fast16_t I2CMSP432E4_primeTransfer(I2CMSP432E4_Object  *object,
    I2CMSP432E4_HWAttrs const *hwAttrs, I2C_Transaction *transaction)
{
    int_fast16_t status = I2C_STATUS_SUCCESS;

    /* Store the new internal counters and pointers */
    object->currentTransaction = transaction;

    object->writeBuf = transaction->writeBuf;
    object->writeCount = transaction->writeCount;

    object->readBuf = transaction->readBuf;
    object->readCount = transaction->readCount;

    object->burstCount = 0;
    object->burstStarted = false;

    /*
     * Transaction is incomplete unless the stop condition occurs AND
     * all bytes have been sent and received. This condition is checked
     * in the hardware interrupt when the STOP condition occurs.
     */
    transaction->status = I2C_STATUS_INCOMPLETE;

    /* Flush the FIFOs */
    I2CTxFIFOFlush(hwAttrs->baseAddr);
    I2CRxFIFOFlush(hwAttrs->baseAddr);

    /* Determine if the bus is in use by another I2C Master */
    if (I2CMasterBusBusy(hwAttrs->baseAddr))
    {
        transaction->status = I2C_STATUS_BUS_BUSY;
        status = I2C_STATUS_BUS_BUSY;
    }
    /* Are we operating in High Speed Mode (3.33Mbps) */
    else if (object->bitRate == I2C_3330kHz) {

        /*
         * Place the master code into the master slave address (MAS) register.
         */
        HWREG(hwAttrs->baseAddr + I2C_O_MSA) = hwAttrs->masterCode;

        /*
         * In High Speed mode, arbitration takes place during transmission of
         * the master code. If successful, a NACK interrupt is generated.
         */
        I2CMasterIntEnableEx(hwAttrs->baseAddr, I2CMSP432E4_TRANSFER_INTS);

        /*
         * Send master code using high speed burst mode.
         */
        HWREG(hwAttrs->baseAddr + I2C_O_MCS) = I2C_MCS_BURST | I2C_MCS_HS;
    }
    else if (object->writeCount) {

        I2CMSP432E4_primeWriteBurst(object, hwAttrs);
    }
    else {

        I2CMSP432E4_primeReadBurst(object, hwAttrs);
    }

    return (status);
}

/*
 *  ======== I2CMSP432E4_readRecieveFifo ========
 */
static void I2CMSP432E4_readRecieveFifo(I2CMSP432E4_Object *object,
    I2CMSP432E4_HWAttrs const *hwAttrs)
{

    while(object->readCount && object->burstCount &&
        I2CFIFODataGetNonBlocking(hwAttrs->baseAddr, object->readBuf)) {
        object->readBuf++;
        object->readCount--;
        object->burstCount--;
    }

    I2CMasterIntClearEx(hwAttrs->baseAddr, I2C_MASTER_INT_RX_FIFO_REQ);
}

/*
 *  ======== I2CMSP432E4_updateReg ========
 */
static void I2CMSP432E4_updateReg(uint32_t reg, uint32_t mask, uint32_t val)
{
    uint32_t regL = HWREG(reg) & ~mask;
    HWREG(reg) = (regL | (val & mask));
}

/*
 *  ======== I2CMSP432E4_cancel ========
 */
void I2CMSP432E4_cancel(I2C_Handle handle)
{
    I2CMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    I2CMSP432E4_Object *object = handle->object;
    uintptr_t key;

    key = HwiP_disable();

    /* Return if no transfer in progress */
    if (!object->headPtr) {
        HwiP_restore(key);
        return;
    }

    /*
     * Writing the STOP command may put the I2C peripheral's FSM in a
     * bad state, so we write RUN | STOP such that the current burst
     * is transferred or received.
     */
    I2CMasterControl(hwAttrs->baseAddr, I2C_MASTER_CMD_BURST_SEND_FINISH);

    /* Set current transaction as canceled */
    object->currentTransaction->status = I2C_STATUS_CANCEL;

    HwiP_restore(key);
}

/*
 *  ======== I2CMSP432E4_close ========
 */
void I2CMSP432E4_close(I2C_Handle handle)
{
    I2CMSP432E4_Object         *object = handle->object;
    I2CMSP432E4_HWAttrs const  *hwAttrs = handle->hwAttrs;
    uint8_t                     port;

    /* Mask I2C interrupts */
    I2CMasterIntDisableEx(hwAttrs->baseAddr, I2CMSP432E4_TRANSFER_INTS);

    /* Disable the I2C Master */
    I2CMasterDisable(hwAttrs->baseAddr);

    /* Undo SCL pin and release dependency */
    GPIOMSP432E4_undoPinConfig(hwAttrs->sclPin);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->sclPin);
    Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));

    /* Undo SDA pin and release dependency */
    GPIOMSP432E4_undoPinConfig(hwAttrs->sdaPin);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->sdaPin);
    Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));

    /* Release peripheral dependency */
    Power_releaseDependency(I2CMSP432E4_getPowerMgrId(hwAttrs->baseAddr));

    if (object->hwiHandle) {
        HwiP_delete(object->hwiHandle);
        object->hwiHandle = NULL;
    }

    if (object->mutex) {
        SemaphoreP_delete(object->mutex);
        object->mutex = NULL;
    }

    if (object->transferComplete) {
        SemaphoreP_delete(object->transferComplete);
        object->transferComplete = NULL;
    }

    object->isOpen = false;

    return;
}

/*
 *  ======== I2CMSP432E4_control ========
 *  @pre    Function assumes that the handle is not NULL
 */
int_fast16_t I2CMSP432E4_control(I2C_Handle handle, uint_fast16_t cmd, void *arg)
{
    /* No implementation yet */
    return (I2C_STATUS_UNDEFINEDCMD);
}

/*
 *  ======== I2CMSP432E4_init ========
 */
void I2CMSP432E4_init(I2C_Handle handle)
{
}

/*
 *  ======== I2CMSP432E4_open ========
 */
I2C_Handle I2CMSP432E4_open(I2C_Handle handle, I2C_Params *params)
{
    uintptr_t                   key;
    I2CMSP432E4_Object          *object = handle->object;
    I2CMSP432E4_HWAttrs const   *hwAttrs = handle->hwAttrs;
    HwiP_Params                 hwiParams;
    uint32_t                    pinMap;
    uint32_t                    powerMgrId;
    uint8_t                     port;
    uint8_t                     pin;

    /* Determine if the device index was already opened */
    key = HwiP_disable();
    if (object->isOpen) {
        HwiP_restore(key);

        return (NULL);
    }

    /* If using I2C_3330kHz, a master code must be specified */
    if (params->bitRate == I2C_3330kHz && (hwAttrs->masterCode < 0x08)) {
        HwiP_restore(key);

        return (NULL);
    }

    /* Mark the handle as being used */
    object->isOpen = true;
    HwiP_restore(key);

    powerMgrId = I2CMSP432E4_getPowerMgrId(hwAttrs->baseAddr);
    if (powerMgrId > PowerMSP432E4_NUMRESOURCES) {
        object->isOpen = false;

        return (NULL);
    }

    /* Enable clocks to the I2C peripheral */
    Power_setDependency(powerMgrId);


    /* Set Power dependencies, SCL & SDA Pin may use different GPIO port */
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->sclPin);
    Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
    pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->sclPin);
    GPIOPinConfigure(pinMap);
    pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->sclPin);
    GPIOPinTypeI2CSCL(GPIOMSP432E4_getGpioBaseAddr(port), pin);

    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->sdaPin);
    Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
    pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->sdaPin);
    GPIOPinConfigure(pinMap);
    pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->sdaPin);
    GPIOPinTypeI2C(GPIOMSP432E4_getGpioBaseAddr(port), pin);

    /* Save parameters */
    object->transferMode = params->transferMode;
    object->transferCallbackFxn = params->transferCallbackFxn;
    object->bitRate = params->bitRate;

    /* Create Hwi object for this I2C peripheral */
    HwiP_Params_init(&hwiParams);
    hwiParams.arg = (uintptr_t)handle;
    hwiParams.priority = hwAttrs->intPriority;
    object->hwiHandle = HwiP_create(hwAttrs->intNum, I2CMSP432E4_hwiFxn,
                            &hwiParams);

    if (object->hwiHandle == NULL) {
        I2CMSP432E4_close(handle);

        return (NULL);
    }

    /*
     * Create threadsafe handles for this I2C peripheral
     * Semaphore to provide exclusive access to the I2C peripheral
     */
    object->mutex = SemaphoreP_createBinary(1);

    if (object->mutex == NULL) {
        I2CMSP432E4_close(handle);

        return (NULL);
    }

    /*
     * Store a callback function that posts the transfer complete
     * semaphore for synchronous mode
     */
    if (object->transferMode == I2C_MODE_BLOCKING) {
        /*
         * Semaphore to cause the waiting task to block for the I2C
         * to finish
         */
        object->transferComplete = SemaphoreP_createBinary(0);

        if (object->transferComplete == NULL) {
            I2CMSP432E4_close(handle);

            return (NULL);
        }

        /* Store internal callback function */
        object->transferCallbackFxn = I2CMSP432E4_blockingCallback;
    }

    /* Clear the head pointer */
    object->headPtr = NULL;
    object->tailPtr = NULL;

    /* Set the I2C configuration */
    I2CMSP432E4_initHw(handle);

    /* Return the address of the handle */
    return (handle);
}

/*
 *  ======== I2CMSP432E4_reset ========
 */
void I2CMSP432E4_reset(uint32_t baseAddress)
{
    /* Obtain the sysCtrl I2C peripheral ID */
    uint32_t sysCtrlId = I2CMSP432E4_getSysCtrlId(baseAddress);

    /* Disable I2C peripheral */
    SysCtlPeripheralSleepDisable(sysCtrlId);
    SysCtlPeripheralDisable(sysCtrlId);

    /* Reset I2C Peripheral */
    SysCtlPeripheralReset(sysCtrlId);

    /* Re-enable I2C peripheral */
    SysCtlPeripheralEnable(sysCtrlId);
    SysCtlPeripheralSleepEnable(sysCtrlId);

    /* Wait for I2C peripheral to be ready */
    while(!SysCtlPeripheralReady(sysCtrlId)) {}
}

/*
 *  ======== I2CMSP432E4_transfer ========
 */
int_fast16_t I2CMSP432E4_transfer(I2C_Handle handle,
    I2C_Transaction *transaction, uint32_t timeout)
{
    uintptr_t                   key;
    I2CMSP432E4_Object         *object = handle->object;
    I2CMSP432E4_HWAttrs const  *hwAttrs = handle->hwAttrs;
    int_fast16_t               ret;

    /* Check if anything needs to be written or read */
    if ((transaction->writeCount == 0) &&
        (transaction->readCount == 0)) {
        transaction->status = I2C_STATUS_INVALID_TRANS;
        /* Nothing to write or read */
        return (transaction->status);
    }

    key = HwiP_disable();

    if (object->transferMode == I2C_MODE_CALLBACK) {
        /* Check if a transfer is in progress */
        if (object->headPtr) {

            /*
             * Queued transactions are being canceled. Can't allow additional
             * transactions to be queued.
             */
            if (object->headPtr->status == I2C_STATUS_CANCEL) {
                transaction->status = I2C_STATUS_INVALID_TRANS;
                ret = transaction->status;
            }
            /* Transfer in progress */
            else {

                /*
                 * Update the message pointed by the tailPtr to point to the
                 * next message in the queue
                 */
                object->tailPtr->nextPtr = transaction;

                /* Update the tailPtr to point to the last message */
                object->tailPtr = transaction;

                /* Set queued status */
                transaction->status = I2C_STATUS_QUEUED;

                ret = I2C_STATUS_SUCCESS;
            }

            HwiP_restore(key);
            return (ret);
        }
    }

    /* Store the headPtr indicating I2C is in use */
    object->headPtr = transaction;
    object->tailPtr = transaction;

    /* In blocking mode, transactions can queue on the I2C mutex */
    transaction->status = I2C_STATUS_QUEUED;

    HwiP_restore(key);

    /* Get the lock for this I2C handle */
    if (SemaphoreP_pend(object->mutex, SemaphoreP_NO_WAIT)
        == SemaphoreP_TIMEOUT) {

        /* We were unable to get the mutex in CALLBACK mode */
        if (object->transferMode == I2C_MODE_CALLBACK) {
            /*
             * Recursively call transfer() and attempt to place transaction
             * on the queue. This may only occur if a thread is preempted
             * after restoring interrupts and attempting to grab this mutex.
             */
            return (I2CMSP432E4_transfer(handle, transaction, timeout));
        }

        /* Wait for the I2C lock. If it times-out before we retrieve it, then
         * return false to cancel the transaction. */
        if(SemaphoreP_pend(object->mutex, timeout) == SemaphoreP_TIMEOUT) {
            transaction->status = I2C_STATUS_TIMEOUT;
            return (I2C_STATUS_TIMEOUT);
        }
    }

    if (object->transferMode == I2C_MODE_BLOCKING) {
       /*
        * In the case of a timeout, the timed-out transaction may post the
        * object->transferComplete semaphore. To clear this, we simply do
        * a NO_WAIT pend on (binary) object->transferComplete, so that
        * it resets the semaphore count.
        */
        SemaphoreP_pend(object->transferComplete, SemaphoreP_NO_WAIT);
    }

    /*
     * I2CMSP432E4_primeTransfer is a longer process and
     * protection is needed from the I2C interrupt
     */
    HwiP_disableInterrupt(hwAttrs->intNum);
    ret = I2CMSP432E4_primeTransfer(object, hwAttrs, transaction);
    HwiP_enableInterrupt(hwAttrs->intNum);

    if (ret != I2C_STATUS_SUCCESS) {
        /* Error occurred, fall through */
    }
    else if (object->transferMode == I2C_MODE_BLOCKING) {
        /* Wait for the primed transfer to complete */
        if (SemaphoreP_pend(object->transferComplete, timeout)
            == SemaphoreP_TIMEOUT) {

            key = HwiP_disable();

            /*
             * Protect against a race condition in which the transfer may
             * finish immediately after the timeout. If this occurs, we
             * will preemptively consume the semaphore on the next initiated
             * transfer.
             */
            if (object->headPtr) {

                /*
                 * It's okay to call cancel here since this is blocking mode.
                 * Cancel will generate a STOP condition and immediately
                 * return.
                 */
                I2CMSP432E4_cancel(handle);

                HwiP_restore(key);

                /*
                 *  We must wait until the STOP interrupt occurs. When this
                 *  occurs, the semaphore will be posted. Since the STOP
                 *  condition will finish the current burst, we can't
                 *  unblock the object->mutex until this occurs.
                 *
                 *  This may block forever if the slave holds the clock line--
                 *  rendering the I2C bus un-usable.
                 */
                SemaphoreP_pend(object->transferComplete,
                    SemaphoreP_WAIT_FOREVER);

                transaction->status = I2C_STATUS_TIMEOUT;
            }
            else {
                HwiP_restore(key);
            }
        }

        ret = transaction->status;
    }

    /* Release the lock for this particular I2C handle */
    SemaphoreP_post(object->mutex);

    return (ret);
}
