/*
 * Copyright (c) 2017-2019, Texas Instruments Incorporated
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

#include <ti/devices/msp432e4/driverlib/udma.h>

#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dma/UDMAMSP432E4.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432E4.h>

extern const UDMAMSP432E4_Config UDMAMSP432E4_config[];

/* Call uDMAInit() only once, after uDMAEnable */
static bool defChannelsSet = false;
static bool dmaInitialized = false;

/*
 *  ======== UDMAMSP432E4_close ========
 */
void UDMAMSP432E4_close(UDMAMSP432E4_Handle handle)
{
    Power_releaseDependency(PowerMSP432E4_PERIPH_UDMA);
}

/*
 *  ======== UDMAMSP432E4_init ========
 */
void UDMAMSP432E4_init()
{
    HwiP_Params           hwiParams;
    UDMAMSP432E4_Handle   handle = (UDMAMSP432E4_Handle)&(UDMAMSP432E4_config[0]);
    UDMAMSP432E4_HWAttrs  const *hwAttrs = handle->hwAttrs;
    UDMAMSP432E4_Object  *object = handle->object;

    if (!dmaInitialized) {
        object->isOpen = false;

        HwiP_Params_init(&hwiParams);
        hwiParams.priority = hwAttrs->intPriority;

        object->hwiHandle = HwiP_create(hwAttrs->intNum, hwAttrs->dmaErrorFxn,
                &hwiParams);
        if (object->hwiHandle != NULL) {
            dmaInitialized = true;
        }
    }
}

/*
 *  ======== UDMAMSP432E4_open ========
 */
UDMAMSP432E4_Handle UDMAMSP432E4_open()
{
    UDMAMSP432E4_Handle   handle = (UDMAMSP432E4_Handle)&(UDMAMSP432E4_config);
    UDMAMSP432E4_Object  *object = handle->object;
    UDMAMSP432E4_HWAttrs  const *hwAttrs = handle->hwAttrs;
    uintptr_t             key;

    if (!dmaInitialized) {
        return (NULL);
    }

    Power_setDependency(PowerMSP432E4_PERIPH_UDMA);

    key = HwiP_disable();

    /*
     *  If the UDMA has not been opened yet, initialize the control
     *  table base address.
     */
    if (!object->isOpen) {
        uDMAEnable();
        uDMAControlBaseSet(hwAttrs->controlBaseAddr);

        /* Some channels are hooked up by default and can cause issues in
         * certain configs, so we set those to unused channels once */
        if (!defChannelsSet) {
            uDMAInit();
            defChannelsSet = true;
        }

        object->isOpen = true;
    }

    HwiP_restore(key);

    return (handle);
}
