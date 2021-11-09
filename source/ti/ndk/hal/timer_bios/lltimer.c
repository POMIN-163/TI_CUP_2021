/*
 * Copyright (c) 2013-2018, Texas Instruments Incorporated
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
 * ======== lltimer.c ========
 *
 *  A timer must be defined to execute the llTimerTick() function every 100ms;
 *  This is the "NDK heartbeat" which drives the NDK network scheduler.
 */

#include <ti/ndk/inc/stkmain.h>

#include <time.h>

/*--------------------------------------------------------------------- */
/* TIMER Functions */
/*--------------------------------------------------------------------- */

/*----------------------- */
/* Timer Device */
/*----------------------- */
static uint32_t    TimeStart;       /* Time of initialization */
static uint32_t    TimeS;           /* Time in seconds */
static uint32_t    TimeMS;          /* Time fraction in MS */
static unsigned long long LastCLKTime; /* Last time stamp in nanoseconds */
static uint32_t    LastCLKValid;     /* Last Time value returned from CLK function */
static int         TimerOpen = 0;   /* Flag to know if we should be active */

/* This is the semaphore to signal when we have an event */
static STKEVENT_Handle hEvent;

/*-------------------------------------------------------------------- */
/* llTimerTick() */
/* This task just counts HS Ticks as it is called from a periodic */
/* timer running at 100ms. */
/*-------------------------------------------------------------------- */
void llTimerTick()
{

    if( TimerOpen )
    {
        struct timespec timestamp;

        clock_gettime(CLOCK_MONOTONIC, &timestamp);

        /* save timestamp in nanoseconds */
        LastCLKTime = ((unsigned long long)timestamp.tv_sec) * 1000000000
                + timestamp.tv_nsec;
        LastCLKValid = 1;

        TimeMS += 100;
        if( TimeMS >= 1000 )
        {
            TimeS++;
            TimeMS -= 1000;
        }
        STKEVENT_signal( hEvent, STKEVENT_TIMER, 1 );
    }
}

/*-------------------------------------------------------------------- */
/* _llTimerInit() */
/* Initiate low-level timer support. Called with current time */
/* in seconds. Timer has a 130 year wrap, so starting time */
/* reference should be chosen with some care. The reference is */
/* irrelevant, but should be consistent. */
/*-------------------------------------------------------------------- */
void _llTimerInit( STKEVENT_Handle h, uint32_t ctime )
{
    hEvent = h;
    TimeStart = TimeS = ctime;
    TimeMS = 0;
    TimerOpen = 1;
    LastCLKValid = 0;
}

/*-------------------------------------------------------------------- */
/* _llTimerShutdown() */
/* Shutdown low-level timer support */
/*-------------------------------------------------------------------- */
void _llTimerShutdown()
{
    TimerOpen = 0;
}

/*-------------------------------------------------------------------- */
/* llTimerGetTime() */
/* Called to get the system time in S and optionally MS from bootup. */
/*-------------------------------------------------------------------- */
uint32_t llTimerGetTime( uint32_t *pMSFrac )
{
    register uint32_t mask,sec;
    struct timespec currTime;
    unsigned long long timeDiffNs;

    mask = OEMSysCritOn();

    sec = TimeS;
    if(pMSFrac)
    {
        if (LastCLKValid) {
            clock_gettime(CLOCK_MONOTONIC, &currTime);
            /* compute the time difference in nanoseconds */
            timeDiffNs = (((unsigned long long)currTime.tv_sec) * 1000000000
                    + currTime.tv_nsec) - LastCLKTime;

            /* convert back to milliseconds and store in caller supplied var */
            *pMSFrac = TimeMS + (timeDiffNs / 1000000);
        }
        else {
            *pMSFrac = TimeMS;
        }
    }

    OEMSysCritOff(mask);

    return(sec);
}

/*-------------------------------------------------------------------- */
/* llTimerGetStartTime() */
/* Called to get the time in S at which the timer was initialized. */
/*-------------------------------------------------------------------- */
uint32_t llTimerGetStartTime()
{
    return( TimeStart );
}
