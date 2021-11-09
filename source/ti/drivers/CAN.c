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
/*
 *  ======== CAN.c ========
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include <ti/drivers/CAN.h>

#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/utils/StructRingBuf.h>

extern const CAN_Config CAN_config[];
extern const uint_least8_t CAN_count;

/* DefaultCAN parameters structure */
const CAN_Params CAN_defaultParams = {
    CAN_MODE_BLOCKING,                      /* mode */
    CAN_DIRECTION_READWRITE,                /* communication mode */
    0,                                      /* filterID */
    0,                                      /* filterMask */
    CAN_WAIT_FOREVER,                       /* readTimeout */
    CAN_WAIT_FOREVER                        /* writeTimeout */
};

static bool isInitialized = false;

/*
 *  ======== CAN_close ========
 */
void CAN_close(CAN_Handle handle)
{
    handle->fxnTablePtr->closeFxn(handle);
}

/*
 *  ======== CAN_control ========
 */
int_fast16_t CAN_control(CAN_Handle handle, uint_fast16_t cmd, void *arg)
{
    return (handle->fxnTablePtr->controlFxn(handle, cmd, arg));
}

/*
 *  ======== CAN_init ========
 */
void CAN_init(void)
{
    uint_least8_t i;
    uint_fast32_t key;

    key = HwiP_disable();

    if (!isInitialized) {
        isInitialized = (bool) true;

        /* Call each driver's init function */
        for (i = 0; i < CAN_count; i++) {
            CAN_config[i].fxnTablePtr->initFxn((CAN_Handle) &(CAN_config[i]));
        }
    }

    HwiP_restore(key);
}

/*
 *  ======== CAN_open ========
 */
CAN_Handle CAN_open(uint_least8_t index, CAN_Params *params)
{
    CAN_Handle handle = NULL;

    if (isInitialized && (index < CAN_count)) {
        /* If params are NULL use defaults */
        if (params == NULL) {
            params = (CAN_Params *) &CAN_defaultParams;
        }

        /* Get handle for this driver instance */
        handle = (CAN_Handle)&(CAN_config[index]);
        handle = handle->fxnTablePtr->openFxn(handle, params);
    }

    return (handle);
}

/*
 *  ======== CAN_Params_init ========
 */
void CAN_Params_init(CAN_Params *params)
{
    *params = CAN_defaultParams;
}

/*
 *  ======== CAN_read ========
 */
int_fast32_t CAN_read(CAN_Handle handle, void *buffer, size_t size)
{
    return handle->fxnTablePtr->readFxn(handle, buffer, size);
}

/*
 *  ======== CAN_write ========
 */
int_fast32_t CAN_write(CAN_Handle handle, const void *buffer, size_t size)
{
    return handle->fxnTablePtr->writeFxn(handle, buffer, size);
}

/*
 *  ======== CAN_readHelper ========
 */
int_fast32_t CAN_readHelper(CAN_Handle handle, void *buffer, size_t size,
                            SemaphoreP_Handle sem, CAN_Mode mode,
                            CAN_Direction direction,
                            uint32_t timeout, StructRingBuf_Handle ringBuffer)
{
    struct can_frame *data = (struct can_frame*)buffer;
    int_fast32_t      result = 0;

    if (!(direction & CAN_DIRECTION_READ)) {
        //return -EINVAL;
        return -22;
    }

    if (mode == CAN_MODE_NONBLOCKING) {
        timeout = SemaphoreP_NO_WAIT;
    }

    while (size) {
        int               remaining;
        SemaphoreP_Status semStatus;

        semStatus = SemaphoreP_pend(sem, timeout);

        if (semStatus == SemaphoreP_TIMEOUT) {
            /* no more data to read */
            if (result > 0) {
                break;
            }
            else if (mode == CAN_MODE_NONBLOCKING) {
                //return -EAGAIN;
                return -11;
            }
            else {
                //return -ETIMEDOUT;
                return -110;
            }
        }

        remaining = StructRingBuf_get(ringBuffer, data);
        if (remaining == -1) {
            /* Should only happen in NONBLOCKING case, no data available */
            //return -EAGAIN;
            return -11;
        }

        size -= sizeof(struct can_frame);
        ++data;
        result += sizeof(struct can_frame);
    }

    return result;
}

/*
 *  ======== CAN_writeHelper ========
 */
int_fast32_t CAN_writeHelper(CAN_Handle handle, const void *buffer, size_t size,
                             SemaphoreP_Handle sem, CAN_Mode mode,
                             CAN_Direction direction,
                             uint32_t timeout, StructRingBuf_Handle ringBuffer)
{
    const struct can_frame *data = (const struct can_frame*)buffer;
    int_fast32_t            result = 0;

    if (!(direction & CAN_DIRECTION_WRITE)) {
        //return -EINVAL;
        return -22;
    }

    if (mode == CAN_MODE_NONBLOCKING) {
        timeout = SemaphoreP_NO_WAIT;
    }

    while (size) {
        int               total;
        SemaphoreP_Status semStatus;

        semStatus = SemaphoreP_pend(sem, timeout);

        if (semStatus == SemaphoreP_TIMEOUT) {
            /* no more data to read */
            if (result > 0) {
                break;
            }
            else if (mode == CAN_MODE_NONBLOCKING) {
                //return -EAGAIN;
                return -11;
            }
            else {
                //return -ETIMEDOUT;
                return -110;
            }
        }

        total = StructRingBuf_put(ringBuffer, data);
        if (total == -1) {
            /* Should only happen in NONBLOCKING case, no space available */
            //return -EAGAIN;
            return -11;
        }

        handle->fxnTablePtr->txMsgFxn(handle);
        size -= sizeof(struct can_frame);
        ++data;
        result += sizeof(struct can_frame);
    }

    return result;
}
