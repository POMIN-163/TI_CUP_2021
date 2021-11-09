/*
 * Copyright (c) 2018, Texas Instruments Incorporated
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

#include <ti/devices/msp432e4/driverlib/can.h>

#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/gpio/GPIOMSP432E4.h>
#include <ti/drivers/power/PowerMSP432E4.h>
#include <ti/drivers/can/CANMSP432E4.h>

/* Max number of supported CAN channels */
#define MAX_CAN_CHANNELS 2

/* Hardware channel specific metadata */
typedef struct CANMSP432E4_Hw
{
    /* linked list of objects bound to this hardware */
    List_List   list;
    /* Bit mask of message objects in use */
    uint32_t    inUseMask;
    /* bus off count statistic */
    uint32_t    busOffCount;
    /* Hwi Object */
    HwiP_Handle hwi;
} CANMSP432E4_Hw;

/* Hardware channel specific metadata instances */
static CANMSP432E4_Hw instance[MAX_CAN_CHANNELS];

/* private "platform" helper methods */
int_fast32_t CAN_readHelper(CAN_Handle handle, void *buffer, size_t size,
                            SemaphoreP_Handle sem, CAN_Mode mode,
                            CAN_Direction direction,
                            uint32_t timeout, StructRingBuf_Handle ringBuffer);
int_fast32_t CAN_writeHelper(CAN_Handle handle, const void *buffer, size_t size,
                             SemaphoreP_Handle sem, CAN_Mode mode,
                             CAN_Direction direction,
                             uint32_t timeout, StructRingBuf_Handle ringBuffer);

/* CANMSP432E4 functions */
void         CANMSP432E4_close(CAN_Handle handle);
int_fast16_t CANMSP432E4_control(CAN_Handle handle, uint_fast16_t cmd, void *arg);
void         CANMSP432E4_init(CAN_Handle handle);
CAN_Handle   CANMSP432E4_open(CAN_Handle handle, CAN_Params *params);
int_fast32_t CANMSP432E4_read(CAN_Handle handle, void *buffer, size_t size);
int_fast32_t CANMSP432E4_write(CAN_Handle handle, const void *buffer,
                size_t size);
void CANMSP432E4_txMsg(CAN_Handle handle);

/* CAN function table for CANMSP432E4 implementation */
const CAN_FxnTable CANMSP432E4_fxnTable = {
    CANMSP432E4_close,
    CANMSP432E4_control,
    CANMSP432E4_init,
    CANMSP432E4_open,
    CANMSP432E4_read,
    CANMSP432E4_write,
    CANMSP432E4_txMsg
};

static uint_fast16_t getPowerResourceId(uint32_t baseAddr);
static int32_t allocateMessageObject(CAN_Handle handle, bool receive);
static void releaseMessageObjectMask(CAN_Handle handle, uint32_t mask);
static CANMSP432E4_Hw *getHwInstance(CANMSP432E4_HWAttrs const *hwAttrs);

/*
 *  ======== objectIndexToBitMask ========
 */
static inline uint32_t objectIndexToBitMask(int index)
{
    return (0x1 << (index - 1));
}

/*
 *  ======== objectBitMaskToIndex ========
 */
static inline int objectBitMaskToIndex(uint32_t mask)
{
#if defined(__TI_COMPILER_VERSION__)
    return __clz(__rbit(mask)) + 1;
#elif defined(__GNUC__)
    return __builtin_ctz(mask) + 1;
#elif defined(__IAR_SYSTEMS_ICC__)
    return __CLZ(__RBIT(mask)) + 1;
#else
#error "Unsupported compiler used"
#endif
}

/*
 *  ======== txBuffersFlush ========
 */
static inline void txBuffersFlush(CANMSP432E4_HWAttrs const *hwAttrs)
{
    CANMSP432E4_Hw *hw = getHwInstance(hwAttrs);
    List_Elem      *le;

    /* traverse object list looking for transmit Message Objects */
    for (le = List_tail(&hw->list); le != NULL; le = List_prev(le)) {
        CANMSP432E4_Object *object = (CANMSP432E4_Object*)le;

        if (object->txInUseMask) {
            struct can_frame canFrame;

            CANMessageClear(hwAttrs->baseAddr, objectBitMaskToIndex(object->txInUseMask));
            while (StructRingBuf_get(&object->txBuffer, &canFrame) >= 0) {
                SemaphoreP_post(object->writeSem);
            }
        }
        object->txPending = false;
    }
}

/*
 *  ======== CANMSP432E4_close ========
 */
void CANMSP432E4_close(CAN_Handle handle)
{
    uintptr_t                  key;
    CANMSP432E4_Object        *object = handle->object;
    CANMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    CANMSP432E4_Hw            *hw = getHwInstance(hwAttrs);
    uint_fast16_t              resourceId;
    uint8_t                    port;

    releaseMessageObjectMask(handle, object->rxInUseMask);
    releaseMessageObjectMask(handle, object->txInUseMask);

    key = HwiP_disable();
    if (!List_empty(&hw->list)) {
        List_remove(&hw->list, &object->elem);
    }
    HwiP_restore(key);

    if (List_empty(&hw->list)) {

        /* Disable CAN and interrupts. */
        CANIntDisable(hwAttrs->baseAddr,
                      CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
        CANDisable(hwAttrs->baseAddr);

        resourceId = getPowerResourceId(hwAttrs->baseAddr);
        if (resourceId < PowerMSP432E4_NUMRESOURCES) {
            Power_releaseDependency(resourceId);
        }

        /* Remove dependencies on GPIO ports */
        GPIOMSP432E4_undoPinConfig(hwAttrs->txPin);
        port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->txPin);
        Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));

        GPIOMSP432E4_undoPinConfig(hwAttrs->rxPin);
        port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->rxPin);
        Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));

        if (hw->hwi) {
            HwiP_delete(hw->hwi);
        }
    }

    if (object->writeSem) {
        SemaphoreP_delete(object->writeSem);
    }

    if (object->readSem) {
        SemaphoreP_delete(object->readSem);
    }

    object->opened = false;
}

/*
 *  ======== CANMSP432E4_control ========
 *  @pre    Function assumes that the handle is not NULL
 */
int_fast16_t CANMSP432E4_control(CAN_Handle handle, uint_fast16_t cmd, void *arg)
{
    switch (cmd) {
        default:
            return (CAN_STATUS_UNDEFINEDCMD);
    }
}

/*
 *  ======== CANMSP432E4_hwiIntFxn ========
 *  Hwi function that processes CAN interrupts.
 *
 *  @param(arg)         The CANMSP432E4_HWAttrs const for this Hwi.
 */
static void CANMSP432E4_hwiIntFxn(uintptr_t arg)
{
    uint32_t                   status;
    CANMSP432E4_HWAttrs const *hwAttrs = (CANMSP432E4_HWAttrs const*)arg;
    CANMSP432E4_Hw            *hw = getHwInstance(hwAttrs);

    /* Clear interrupts */
    status = CANIntStatus(hwAttrs->baseAddr, CAN_INT_STS_CAUSE);

    if (status == CAN_INT_INTID_STATUS) {
        status = CANStatusGet(hwAttrs->baseAddr, CAN_STS_CONTROL);
        if (status & CAN_STATUS_EWARN) {
            txBuffersFlush(hwAttrs);
        }
        if (status & CAN_STATUS_BUS_OFF) {
            txBuffersFlush(hwAttrs);
            ++hw->busOffCount;
        }
        /** @todo add error callback */
    }
    else if (status != 0 && status <= 32) {
        List_Elem *le;

        /* traverse object list looking for receive Message Objects */
        for (le = List_tail(&hw->list); le != NULL; le = List_prev(le)) {
            CANMSP432E4_Object *object = (CANMSP432E4_Object*)le;

            if (object->rxInUseMask & objectIndexToBitMask(status)) {
                struct can_frame canFrame;
                tCANMsgObject canMessage;

                canMessage.pui8MsgData = canFrame.data;
                CANMessageGet(hwAttrs->baseAddr, status, &canMessage, true);
                if (StructRingBuf_isFull(&object->rxBuffer)) {
                    ++object->overrunCount;
                }
                else {
                    canFrame.id = canMessage.ui32MsgID;
                    canFrame.rtr = canMessage.ui32Flags & MSG_OBJ_REMOTE_FRAME ? 1 : 0;
                    canFrame.eff = canMessage.ui32Flags & MSG_OBJ_EXTENDED_ID ? 1 : 0;
                    canFrame.err = 0;
                    canFrame.dlc = canMessage.ui32MsgLen;
                    StructRingBuf_put(&object->rxBuffer, &canFrame);
                    SemaphoreP_post(object->readSem);
                }
                return;
            }
        }

        /* traverse object list looking for transmit Message Objects */
        for (le = List_tail(&hw->list); le != NULL; le = List_prev(le)) {
            CANMSP432E4_Object *object = (CANMSP432E4_Object*)le;

            if (object->txInUseMask & objectIndexToBitMask(status)) {
                struct can_frame  dummy;
                struct can_frame *canFrame;

                CANIntClear(hwAttrs->baseAddr, status);
                StructRingBuf_get(&object->txBuffer, &dummy);
                SemaphoreP_post(object->writeSem);

                if (StructRingBuf_peek(&object->txBuffer, (void**)&canFrame)) {
                    tCANMsgObject canMessage;
                    canMessage.ui32MsgID = canFrame->id;
                    canMessage.ui32MsgIDMask = 0;
                    canMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
                    canMessage.ui32Flags |= canFrame->eff ? MSG_OBJ_EXTENDED_ID : 0;
                    canMessage.ui32Flags |= canFrame->rtr ? MSG_OBJ_REMOTE_FRAME : 0;
                    canMessage.ui32MsgLen = canFrame->dlc;
                    canMessage.pui8MsgData = canFrame->data;
                    CANMessageSet(hwAttrs->baseAddr, status, &canMessage, MSG_OBJ_TYPE_TX);
                }
                else {
                    object->txPending = false;
                }
                return;
            }
        }
    }
}

/*
 *  ======== CANMSP432E4_init ========
 */
void CANMSP432E4_init(CAN_Handle handle)
{
    CANMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    CANMSP432E4_Hw            *hw = getHwInstance(hwAttrs);

    List_clearList(&hw->list);
    hw->inUseMask = 0;
    hw->busOffCount = 0;
    hw->hwi = NULL;
}

/*
 *  ======== CANMSP432E4_open ========
 */
CAN_Handle CANMSP432E4_open(CAN_Handle handle, CAN_Params *params)
{
    uintptr_t                  key;
    ClockP_FreqHz              freq;
    CANMSP432E4_Object        *object = handle->object;
    CANMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    CANMSP432E4_Hw            *hw = getHwInstance(hwAttrs);
    HwiP_Params                hwiParams;
    uint8_t                    pin;
    uint32_t                   port;
    uint32_t                   pinMap;
    uint_fast16_t              resourceId;

    key = HwiP_disable();

    if (object->opened) {
        HwiP_restore(key);
        return (NULL);
    }
    object->opened = true;

    HwiP_restore(key);

    StructRingBuf_construct(&object->txBuffer, handle->txBufPtr,
                            handle->txBufSize / sizeof(struct can_frame),
                            sizeof(struct can_frame));

    StructRingBuf_construct(&object->rxBuffer, handle->rxBufPtr,
                            handle->rxBufSize / sizeof(struct can_frame),
                            sizeof(struct can_frame));

    object->mode = params->mode;
    object->direction = params->direction;
    object->readTimeout = params->readTimeout;
    object->writeTimeout = params->writeTimeout;
    object->overrunCount = 0;
    object->txPending = false;

    if (List_empty(&hw->list)) {
        List_put(&hw->list, &object->elem);

        resourceId = getPowerResourceId(hwAttrs->baseAddr);
        if (resourceId > PowerMSP432E4_NUMRESOURCES) {
            return (NULL);
        }

        Power_setDependency(resourceId);

        /* Configure the TX pin */
        pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->txPin);
        port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->txPin);
        pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->txPin);

        Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
        GPIOPinConfigure(pinMap);
        GPIOPinTypeCAN(GPIOMSP432E4_getGpioBaseAddr(port), pin);

        /* Configure the RX pin */
        pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->rxPin);
        port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->rxPin);
        pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->rxPin);

        Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
        GPIOPinConfigure(pinMap);
        GPIOPinTypeCAN(GPIOMSP432E4_getGpioBaseAddr(port), pin);

        HwiP_Params_init(&hwiParams);
        hwiParams.arg = (uintptr_t)hwAttrs;
        hwiParams.priority = hwAttrs->intPriority;

        hw->hwi = HwiP_create(hwAttrs->intNum, CANMSP432E4_hwiIntFxn,
                              &hwiParams);

        if (hw->hwi == NULL) {
            CANMSP432E4_close(handle);
            return (NULL);
        }

        /* Enable CAN and its interrupt. */
        CANIntClear(hwAttrs->baseAddr, CAN_INT_INTID_STATUS);
        CANEnable(hwAttrs->baseAddr);

        /** @todo we should really set the buadrate based on a table.  The
         * auto baudrate setup for the driverlib is not optimal
         */
        ClockP_getCpuFreq(&freq);
        CANBitRateSet(hwAttrs->baseAddr, freq.lo, hwAttrs->baudRate);

        CANIntEnable(hwAttrs->baseAddr,
                     CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    }   // if (List_empty(&hw->list))

    object->readSem = SemaphoreP_create(0, NULL);
    if (object->readSem == NULL) {
        CANMSP432E4_close(handle);
        return (NULL);
    }

    /** Create a write semaphore resource counter. Counts down until no more
     * free space in the transmit queue.
     */
    object->writeSem = SemaphoreP_create(
        handle->txBufSize / sizeof(struct can_frame), NULL);
    if (object->writeSem == NULL) {
        CANMSP432E4_close(handle);
        return (NULL);
    }

    if (object->direction & CAN_DIRECTION_WRITE) {
        int32_t index = allocateMessageObject(handle, true);
        if (index <= 0) {
            CANMSP432E4_close(handle);
            return (NULL);
        }
        else {
            object->txInUseMask = objectIndexToBitMask(index);
        }
    }

    if (object->direction & CAN_DIRECTION_READ) {
        int32_t index = allocateMessageObject(handle, true);
        if (index <= 0) {
            CANMSP432E4_close(handle);
            return (NULL);
        }
        else {
            tCANMsgObject canMessage;

            object->rxInUseMask = objectIndexToBitMask(index);
            canMessage.ui32MsgID = params->filterID;
            canMessage.ui32MsgIDMask = params->filterMask;
            canMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE +
                                   MSG_OBJ_USE_ID_FILTER;
            canMessage.ui32MsgLen = 8;
            CANMessageSet(hwAttrs->baseAddr, index, &canMessage, MSG_OBJ_TYPE_RX);
        }
    }

    /* Return the handle */
    return (handle);
}

/*
 *  ======== CANMSP432E4_read ========
 */
int_fast32_t CANMSP432E4_read(CAN_Handle handle, void *buffer, size_t size)
{
    CANMSP432E4_Object *object = handle->object;

    return CAN_readHelper(handle, buffer, size, object->readSem, object->mode,
                          object->direction, object->readTimeout,
                          &object->rxBuffer);
}

/*
 *  ======== CANMSP432E4_write ========
 */
int_fast32_t CANMSP432E4_write(CAN_Handle handle, const void *buffer, size_t size)
{
    CANMSP432E4_Object *object = handle->object;

    return CAN_writeHelper(handle, buffer, size, object->writeSem, object->mode,
                           object->direction, object->writeTimeout,
                           &object->txBuffer);
}

/*
 *  ======== CANMSP432E4_txMsg ========
 */
void CANMSP432E4_txMsg(CAN_Handle handle)
{
    uintptr_t           key;
    CANMSP432E4_Object *object = handle->object;
    CANMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;

    key = HwiP_disable();
    if (object->txPending == false) {
        struct can_frame *canFrame;

        if (StructRingBuf_peek(&object->txBuffer, (void**)&canFrame)) {
            tCANMsgObject canMessage;
            canMessage.ui32MsgID = canFrame->id;
            canMessage.ui32MsgIDMask = 0;
            canMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
            canMessage.ui32Flags |= canFrame->eff ? MSG_OBJ_EXTENDED_ID : 0;
            canMessage.ui32Flags |= canFrame->rtr ? MSG_OBJ_REMOTE_FRAME : 0;
            canMessage.ui32MsgLen = canFrame->dlc;
            canMessage.pui8MsgData = canFrame->data;
            CANMessageSet(hwAttrs->baseAddr,
                          objectBitMaskToIndex(object->txInUseMask),
                          &canMessage, MSG_OBJ_TYPE_TX);
            object->txPending = true;
        }
    }
    HwiP_restore(key);
}

/*
 *  ======== getPowerResourceId ========
 */
static uint_fast16_t getPowerResourceId(uint32_t baseAddr)
{
    switch (baseAddr) {
        case CAN0_BASE:
            return (PowerMSP432E4_PERIPH_CAN0);
        case CAN1_BASE:
            return (PowerMSP432E4_PERIPH_CAN1);
        default:
            return ((uint_fast16_t)(~0));
    }
}

/*
 *  ======== allocateMessageObject ========
 */
static int32_t allocateMessageObject(CAN_Handle handle, bool receive)
{
    uintptr_t                  key;
    CANMSP432E4_Object        *object = handle->object;
    CANMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    CANMSP432E4_Hw            *hw = getHwInstance(hwAttrs);
    int                        i;

    key = HwiP_disable();
    for (i = 0; i < 32; ++i) {
        if ((hw->inUseMask & (0x1 << i)) == 0) {
            if (receive) {
                object->rxInUseMask |= (0x1 << i);
            }
            else {
                object->txInUseMask |= (0x1 << i);
            }
            hw->inUseMask |= (0x1 << i);
            HwiP_restore(key);
            return i + 1;
        }
    }

    HwiP_restore(key);
    return -1;
}

/*
 *  ======== releaseMessageObjectMask ========
 */
static void releaseMessageObjectMask(CAN_Handle handle, uint32_t mask)
{
    uintptr_t                  key;
    CANMSP432E4_Object        *object = handle->object;
    CANMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    CANMSP432E4_Hw            *hw = getHwInstance(hwAttrs);

    key = HwiP_disable();
    object->rxInUseMask &= ~mask;
    object->txInUseMask &= ~mask;
    hw->inUseMask &= ~mask;
    HwiP_restore(key);
}

/*
 *  ======== getHwInstance ========
 */
static CANMSP432E4_Hw *getHwInstance(CANMSP432E4_HWAttrs const *hwAttrs)
{
    if (hwAttrs->baseAddr == CAN0_BASE) {
        return &instance[0];
    }
    else {
        return &instance[1];
    }
}
