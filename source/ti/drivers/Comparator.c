/*
 * Copyright (c) 2019, Texas Instruments Incorporated
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
 * ============ Comparator.c ==========
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/Comparator.h>

extern const Comparator_Config Comparator_config[];
extern const uint_least8_t Comparator_count;

static bool isInitialized = false;

/* default Comparator parameters */
static Comparator_Params Comparator_defaultParams = {
    .callbackFxn = NULL,
    .outputPolarity = Comparator_OUTPUT_NORMAL,
    .interruptLevel = Comparator_INTERRUPT_NONE
};

/*
 * ====== Comparator_close ========
 */
void Comparator_close(Comparator_Handle handle)
{
    handle->fxnTablePtr->closeFxn(handle);
}

/*
 * ======= Comparator_getLevel ======
 */
Comparator_OutputLevel Comparator_getLevel(Comparator_Handle handle)
{
    /* Device-specific implementations return uint32_t, change to enum type */
    if (handle->fxnTablePtr->getLevelFxn(handle))
    {
        return (Comparator_OUTPUT_HIGH);
    }
    else
    {
        return (Comparator_OUTPUT_LOW);
    }
}

/*
 * ========= Comparator_init ========
 */
void Comparator_init()
{
    uintptr_t key;

    key = HwiP_disable();
    if (!isInitialized)
    {
        isInitialized = true;

        uint_least8_t i;
        /* Call each comparator's init function */
        for (i = 0; i < Comparator_count; i++)
        {
            Comparator_Handle handle = (Comparator_Handle)&Comparator_config[i];
            handle->fxnTablePtr->initFxn(handle);
        }
    }
    HwiP_restore(key);
}

/*
 * ======== Comparator_open =========
 */
Comparator_Handle Comparator_open(uint32_t index, Comparator_Params *params)
{
    /* Use the default parameters if NULL parameters are passed in */
    Comparator_Handle handle = (Comparator_Handle) &Comparator_config[index];
    if (params == NULL)
    {
        return (handle->fxnTablePtr->openFxn(handle,
                                             &Comparator_defaultParams));
    }
    return (handle->fxnTablePtr->openFxn(handle, params));
}

/*
 * ====== Comparator_Params_init =======
 */
void Comparator_Params_init(Comparator_Params *params)
{
    *params = Comparator_defaultParams;
}

/*
 * ========= Comparator_start ==========
 */
int_fast16_t Comparator_start(Comparator_Handle handle)
{
    return (handle->fxnTablePtr->startFxn(handle));
}

/*
 * ======== Comparator_stop ============
 */
void Comparator_stop(Comparator_Handle handle)
{
    handle->fxnTablePtr->stopFxn(handle);
}

/*
 * ======== Comparator_getParams ========
 */
int_fast16_t Comparator_getParams(Comparator_Handle handle, Comparator_Params *params)
{
    return (handle->fxnTablePtr->getParamsFxn(handle, params));
}

/*
 * ======= Comparator_setParams ==========
 */
int_fast16_t Comparator_setParams(Comparator_Handle handle, Comparator_Params *params)
{
    return (handle->fxnTablePtr->setParamsFxn(handle, params));
}
