/*
 * Copyright (c) 2015-2018, Texas Instruments Incorporated
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
 *  ======== NDK_SemaphoreP_tirtos.c ========
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <xdc/std.h>
#include <xdc/runtime/Error.h>

#include <ti/sysbios/knl/Semaphore.h>

#include "NDK_SemaphoreP.h"


/*
 *  ======== NDK_SemaphoreP_create ========
 */
NDK_SemaphoreP_Handle NDK_SemaphoreP_create(unsigned int count)
{
    Semaphore_Handle  handle;

    /* default params create a counting semaphore, that's what we want */
    handle = Semaphore_create(count, NULL, Error_IGNORE);

    return ((NDK_SemaphoreP_Handle)handle);
}

/*
 *  ======== NDK_SemaphoreP_createBinary ========
 */
NDK_SemaphoreP_Handle NDK_SemaphoreP_createBinary(unsigned int count)
{
    Semaphore_Handle  handle;
    Semaphore_Params  semaphoreParams;

    Semaphore_Params_init(&semaphoreParams);

    semaphoreParams.mode = Semaphore_Mode_BINARY;
    handle = Semaphore_create(count, &semaphoreParams, Error_IGNORE);

    return ((NDK_SemaphoreP_Handle)handle);
}

/*
 *  ======== NDK_SemaphoreP_delete ========
 */
void NDK_SemaphoreP_delete(NDK_SemaphoreP_Handle handle)
{
    Semaphore_Handle semaphore = (Semaphore_Handle)handle;

    Semaphore_delete(&semaphore);
}

/*
 *  ======== NDK_SemaphoreP_pend ========
 */
NDK_SemaphoreP_Status NDK_SemaphoreP_pend(NDK_SemaphoreP_Handle handle,
        uint32_t timeout)
{
    Bool flag;

    flag = Semaphore_pend((Semaphore_Handle)handle, timeout);
    if (FALSE == flag) {
        return (NDK_SemaphoreP_TIMEOUT);
    }

    return (NDK_SemaphoreP_OK);
}

/*
 *  ======== NDK_SemaphoreP_post ========
 */
void NDK_SemaphoreP_post(NDK_SemaphoreP_Handle handle)
{
    Semaphore_post((Semaphore_Handle)handle);
}
