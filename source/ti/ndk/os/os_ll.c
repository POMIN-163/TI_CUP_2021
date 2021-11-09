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
 * ======== os_ll.c ========
 * Defines priority based and semaphore based exclusion APIs
 */

#include <stdbool.h>

#include <netmain.h>
#include <_stack.h>
#include <_oskern.h>

/*
 * Low Resource Flag
 * We can only process a low resource condition when
 * we are not in kernel mode, so we use a flag and
 * then process it on a call to llExit()
 */
static uint32_t _TaskFlagLowResource = 0;

/* signal that system resources are low */
void NotifyLowResource(void)
{
    _TaskFlagLowResource = 1;
}

/*-------------------------------------------------------------------- */
/* llEnter / llExit - Kernel Mode gate functions */
/*-------------------------------------------------------------------- */
/* NOTE: There are two versions of the llEnter()/llExit() pairing. */
/* The default version uses task priority to implement exclusion. The */
/* second version uses a semaphore. The priority based version is */
/* faster, but not as safe if care is not taken when setting task */
/* priorities. */

/* NOTE: Whether or not to use a semaphore for exclusion is independent */
/*       of the choice to use a semaphore for scheduling in NETCTRL. */
/*       The concepts are unrelated. */

/* To enable the semaphore version, set the below #define to 1 */
/* Note that when _NDK_EXTERN_CONFIG is defined, this USE_LL_SEMAPHORE */
/* is defined outside the module. */
/*-------------------------------------------------------------------- */
#ifndef _NDK_EXTERN_CONFIG
#define USE_LL_SEMAPHORE 0
#endif

static void *hSemGate = 0;
static volatile int InKernel = 0;

#if USE_LL_SEMAPHORE
/*-------------------------------------------------------------------- */
/* llEnter() */
/* Enter the IP stack */
/*-------------------------------------------------------------------- */
void llEnter()
{
    /* If we don't have our semaphore yet, allocate one with "1" entry */
    if( !hSemGate )
    {
        if( !(hSemGate = SemCreate(1)) )
        {
            DbgPrintf(DBG_ERROR,"llEnter (sem): Could not create llEnter() semaphore");
            return;
        }
    }

    /* Wait on entering the stack. */
    SemPend(hSemGate, SEM_FOREVER);

    /* If case something goes wrong with the semaphore, track */
    /* problems entering the stack */
    if( InKernel )
        DbgPrintf(DBG_ERROR,"llEnter (sem): Illegal call to llEnter()");

    InKernel=1;
}
/*-------------------------------------------------------------------- */
/* llExit() */
/* Release the IP stack */
/*-------------------------------------------------------------------- */
void llExit()
{
    /* Handle the low resource condition */
    if( _TaskFlagLowResource )
    {
        ExecLowResource();
        _TaskFlagLowResource = 0;
    }

    /* The "InKernel" flag traps calls to llExit() before calling */
    /* llEnter(). */
    if( !InKernel )
    {
        DbgPrintf(DBG_ERROR,"llExit (sem): Illegal call to llExit()");
        return;
    }
    InKernel=0;

    /* Signal that we've exited the stack. */
    SemPost(hSemGate);
}
#else
static int OldPriority = 0;
/*-------------------------------------------------------------------- */
/* llEnter() */
/* Enter the IP stack */
/*-------------------------------------------------------------------- */
void llEnter()
{
    int tmpOldPriority;

    /* If we don't have our semaphore yet, allocate one with "0" entry */
    if( !hSemGate )
    {
        if( !(hSemGate = SemCreate(0)) )
        {
            DbgPrintf(DBG_ERROR,
                    "llEnter: Could not create llEnter() semaphore");
            return;
        }
    }

    /* Since this task is running (assume for now it has a priority LESS */
    /* THAN the kernel level - we know that nothing is running at the */
    /* kernel level. */

    /* We try to use priority to block other calls from coming into */
    /* llEnter(). However, if we are re-entered, we'll fall back to */
    /* using the semaphore */

    /* Set this task's priority at kernel level */
    tmpOldPriority = TaskSetPri( TaskSelf(), OS_TASKPRIKERN );

    /* Verify this call was legal */
    if( tmpOldPriority >= OS_TASKPRIKERN )
    {
        if( InKernel )
        {
            DbgPrintf(DBG_ERROR,
                    "llEnter: Illegal reentrant call to llEnter()");
        }
        else
        {
            DbgPrintf(DBG_ERROR,"llEnter: Illegal priority call to llEnter()");
        }
        return;
    }

    /* Verify there has been no "reentrance". */
    /* If there was, use the semaphore. */
    if( ++InKernel > 1 )
    {
        /* Wait on entering the stack. */
        SemPend(hSemGate, SEM_FOREVER);
    }

    /* Store the priority of the task that owns kernel mode */
    OldPriority = tmpOldPriority;
}
/*-------------------------------------------------------------------- */
/* llExit() */
/* Release the IP stack */
/*-------------------------------------------------------------------- */
void llExit()
{
    /* Handle the low resource condition */
    if( _TaskFlagLowResource )
    {
        ExecLowResource();
        _TaskFlagLowResource = 0;
    }

    /* Verify we were at kernel level */
    if( !InKernel )
    {
        DbgPrintf(DBG_ERROR,"llExit: Illegal call to llExit()");
        return;
    }

    /* If a task is waiting at llEnter(), signal it */
    if( --InKernel > 0 )
    {
        /* Signal that we've exited the stack. */
        SemPost(hSemGate);
    }

    /* Restore this task's priority */
    TaskSetPri( TaskSelf(), OldPriority );
}
#endif
