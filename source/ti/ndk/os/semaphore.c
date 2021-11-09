/*
 * Copyright (c) 2012-2018, Texas Instruments Incorporated
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
 * */
/*
 * ======== semaphore.c ========
 *
 * Semaphore management using BIOS6
 *
 */
#include <netmain.h>
#include <_stack.h>
#include <_oskern.h>

#include "NDK_SemaphoreP.h"

// still need some BIOS APIs
#ifndef NDK_FREERTOS_BUILD
#include <ti/sysbios/knl/Semaphore.h>
#endif

#include <stdlib.h>

/* Sanity check timeout values.  The code below assumes these match */
#if (SEM_FOREVER != NDK_SemaphoreP_WAIT_FOREVER)
#error SEM_FOREVER != NDK_SemaphoreP_WAIT_FOREVER
#endif

/*
 *  ======== SemCreate ========
 *  Create a semaphore.
 */
void *SemCreate(int Count)
{
    void *sem;

    /* create the new semaphore */
    sem = (void *)NDK_SemaphoreP_create((unsigned int)Count);

    if (!sem) {
        /* Should not get here */
        DbgPrintf(DBG_WARN,"SemCreate(): could not create semaphore");
    }

    return (sem);
}

/*
 *  ======== SemCreateBinary ========
 *  Create a binary semaphore.
 */
void *SemCreateBinary(int Count)
{
    void *sem;

    /* create the new semaphore */
    sem = (void *)NDK_SemaphoreP_createBinary((unsigned int)Count);

    if (!sem) {
        /* Should not get here */
        DbgPrintf(DBG_WARN,"SemCreateBinary(): could not create semaphore");
    }

    return (sem);
}

/*
 *  ======== SemDeleteBinary ========
 *  Delete a binary semaphore.
 */
void SemDeleteBinary(void *hSem)
{
    /* needed for Linux support which requires binary sem APIs */
    SemDelete(hSem);
}


/*
 *  ======== SemDelete ========
 *  Delete a semaphore.
 */
void SemDelete(void *hSem)
{
    NDK_SemaphoreP_delete((NDK_SemaphoreP_Handle)hSem);
}

/*
 *  ======== SemCount ========
 *  Get the current semaphore count.
 */
int SemCount(void *hSem)
{
#ifndef NDK_FREERTOS_BUILD
    return (Semaphore_getCount((Semaphore_Handle)hSem));
#else
    return (0);
#endif
}

/*
 *  ======== SemPendBinary ========
 *  Wait for a binary semaphore.
 */
int SemPendBinary(void *hSem, uint32_t Timeout)
{
    /* needed for Linux support which requires binary sem APIs */
    return (SemPend(hSem, Timeout));
}

/*
 *  ======== SemPend ========
 *  Wait for a semaphore.
 */
int SemPend(void *hSem, uint32_t Timeout)
{
    NDK_SemaphoreP_Status status;

    status = NDK_SemaphoreP_pend((NDK_SemaphoreP_Handle)hSem, Timeout);

    if (status == NDK_SemaphoreP_OK) {
        return (1);
    }
    else {
        // status is NDK_SemaphoreP_FAILURE || NDK_SemaphoreP_TIMEOUT
        return (0);
    }
}

/*
 *  ======== SemPostBinary ========
 *  Signal a binary semaphore.
 */
void SemPostBinary(void *hSem)
{
    /* needed for Linux support which requires STKEVENT_signal to call this */
    SemPost(hSem);
}

/*
 *  ======== SemPost ========
 *  Signal a semaphore.
 */
void SemPost(void *hSem)
{
    NDK_SemaphoreP_post((NDK_SemaphoreP_Handle)hSem);
}

/*
 *  ======== SemReset ========
 *  Reset a semaphore's count.
 */
void SemReset(void *hSem, int Count)
{
#ifndef NDK_FREERTOS_BUILD
    Semaphore_reset((Semaphore_Handle)hSem, Count);
#else
    /* SemReset should bring the sem's count down to the user's desired value */
    while (NDK_SemaphoreP_pend((NDK_SemaphoreP_Handle)hSem, 0) ==
            NDK_SemaphoreP_OK);
#endif
}
