/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
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
/*!*****************************************************************************
 *  @file       TimerMSP432E4.h
 *  @brief      Timer driver interface for MSP432E4 devices
 *
 *  # Operation #
 *  This driver implements half and full width general purpose timers for the
 *  MSP432E4 device. For MSP432E4 devices, the system clock is 120 MHz and a 16-bit
 *  timer has an 8-bit prescaler. The desired period may not always be
 *  achieved due to hardware limitations, such as the aforementioned. The timer
 *  resolution is limited to 8.33ns due to the 120 MHz clock. A timer period no
 *  greater than 546,133us can be achieved when operating in 16-bit mode.
 *  Similarly, a period no greater than 35,791,394us can be achieved when
 *  operating in 32-bit mode. The same time constraints apply to the 16-bit
 *  timer when attempting to use a frequency less than 5 Hertz. For additional
 *  details, refer to the device's technical reference manual.
 *
 *  The timer always operates in count down mode. When using a half width timer,
 *  an 8-bit prescaler will be implemented by the driver if necessary. If the
 *  timer is operating in Timer_FREE_RUNNING, the timer will count down from the
 *  specified period to 0 before restarting.
 *
 *  When using a half width timer, Timer_getCount() will return the
 *  value of the counter in bits 15:0 and bits 23:16 will contain the
 *  current free-running value of the prescaler. Bits 31:24 are always 0.
 *  When using a full width timer, Timer_getCount() will return the
 *  the value of the 32-bit timer.
 *
 *  #Timer_ONESHOT_CALLBACK is non-blocking. After Timer_start() is called,
 *  the calling thread will continue execution. When the timer interrupt
 *  is triggered, the specified callback function will be called. The timer
 *  will not generate another interrupt unless Timer_start() is called again.
 *  Calling Timer_stop() or Timer_close() after Timer_start() but, before the
 *  timer interrupt, will prevent the specified callback from ever being
 *  invoked.
 *
 *  #Timer_ONESHOT_BLOCKING is a blocking call. A semaphore is used to block
 *  the calling thead's execution until the timer generates an interrupt. If
 *  Timer_stop() is called, the calling thread will become unblocked
 *  immediately. The behavior of the timer in this mode is similar to a sleep
 *  function.
 *
 *  #Timer_CONTINUOUS_CALLBACK is non-blocking. After Timer_start() is called,
 *  the calling thread will continue execution. When the timer interrupt is
 *  treiggered, the specified callback function will be called. The timer is
 *  automatically restarted and will continue to periodically generate
 *  interrupts until Timer_stop() is called.
 *
 *  #Timer_FREE_RUNNING is non-blocking. After Timer_start() is called,
 *  the calling thread will continue execution. The timer will not
 *  generate an interrupt in this mode. The timer will count down from the
 *  specified period until it reaches 0. The timer will automatically reload
 *  the period and start over. The timer will continue running until
 *  Timer_stop() is called.
 *
 *  # Resource Allocation #
 *  Each general purpose timer block contains two timers, Timer A and Timer B,
 *  that can be configured to operate independently; or concatenated to operate
 *  as one 32-bit timer. This behavior is managed through a set of resource
 *  allocation APIs. For example, the TimerMSP432E4_allocateTimerResource API
 *  will allocate a timer for exclusive use. Any attempt to allocate this
 *  resource in the future will result in a false value being returned from the
 *  allocation API. To free a timer resource, the TimerMSP432E4_freeTimerResource
 *  is used. The application is not responsible for calling these allocation
 * APIs directly.
 *
 *  ============================================================================
 */

#ifndef ti_drivers_timer_TimerMSP432E4__include
#define ti_drivers_timer_TimerMSP432E4__include

#include <stdbool.h>
#include <stdint.h>

#include <ti/drivers/Timer.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*!
 *  @def    TimerMSP432E4_SubTimer
 *
 *  @brief  Sub-timers on the MSP432E4
 *
 *  The timer peripheral supports full width and half width timer operation.
 *  Use the definitions in this enumerated type to specify a full width timer
 *  (32-bit) or half width timer (16-bit) in the hardware attributes. There are
 *  two half width timers per single timer peripheral. A 16-bit timer on this
 *  device has an 8-bit prescaler.
 */
typedef enum {
    TimerMSP432E4_timer16A = 0x0001,    /*!< Half width timer A */
    TimerMSP432E4_timer16B = 0x0002,    /*!< Half width timer B */
    TimerMSP432E4_timer32  = 0x0003,    /*!< Full width timer   */
} TimerMSP432E4_SubTimer;

extern const Timer_FxnTable TimerMSP432E4_fxnTable;

/*!
 *  @brief TimerMSP432E4 Hardware Attributes
 *
 *  Timer hardware attributes that tell the TimerMSP432E4 driver specific hardware
 *  configurations and interrupt/priority settings.
 *
 *  A sample structure is shown below:
 *  @code
 *  const TimerMSP432E4_HWAttrs timerMSP432E4HWAttrs[] =
 *  {
 *      {
 *          .baseAddress = TIMER2_BASE,
 *          .subTimer = TimerMSP432E4_timer32,
 *          .intNum = INT_TIMER02A,
 *          .intPriority = ~0
 *      },
 *      {
 *          .baseAddress = TIMER1_BASE,
 *          .subTimer = TimerMSP432E4_timer16A,
 *          .intNum = INT_TIMER1A,
 *          .intPriority = ~0
 *      },
 *      {
 *          .baseAddress = TIMER1_BASE,
 *          .subTimer = TimerMSP432E4_timer16B,
 *          .intNum = INT_TIMER1B,
 *          .intPriority = ~0
 *      }
 *  };
 *  @endcode
 */
typedef struct {
    /*! The base address of the timer peripheral. */
    uint32_t             baseAddress;

    /*! Specifies a full width timer or half-width timer. */
    TimerMSP432E4_SubTimer subTimer;

    /*! The hardware interrupt number for the timer peripheral. */
    uint32_t             intNum;

    /*! The interrupt priority. */
    uint32_t             intPriority;
} TimerMSP432E4_HWAttrs;

/*!
 *  @brief TimerMSP432E4_Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct {
    HwiP_Handle         hwiHandle;
    Power_NotifyObj     notifyObj;
    SemaphoreP_Handle   timerSem;
    Timer_CallBackFxn   callBack;
    Timer_Mode          mode;
    uint32_t            timer;
    uint32_t            period;
    uint32_t            prescaler;
    bool                isRunning;
} TimerMSP432E4_Object;

/*!
 *  @brief  Function to allocate a timer peripheral.
 *
 *  This function is intended to be used by any driver which implements a
 *  timer hardware peripheral. Calling this function will enable power to the
 *  timer peripheral specified by the parameter, baseAddress.
 *
 *  @param  baseAddress The base address of a timer hardware peripheral.
 *
 *  @param  subTimer    The TimerMSP432E4_subTimer to be allocated.
 *
 *  @return A bool returning true if the timer resource was successfully
 *          allocated. If the base address is not valid or if the resource is
 *          not available, false is returned.
 *
 *  @sa     TimerMSP432E4_freeTimerResource()
 */
extern bool TimerMSP432E4_allocateTimerResource(uint32_t baseAddress,
    TimerMSP432E4_SubTimer subTimer);

/*!
 *  @brief  Function to de-allocate a timer peripheral.
 *
 *  This function is intended to be used by any driver which implements a
 *  timer hardware peripheral. Calling this function will disable power to the
 *  timer peripheral specified by the parameter, baseAddress, if and only if
 *  the timer peripheral is no longer in use.
 *
 *  @pre    A successful call to TimerMSP432E4_allocateTimerResource() using the
 *          baseAddress and subTimer must have been made prior to calling this
 *          API.
 *
 *  @param  baseAddress The base address of a timer hardware peripheral.
 *
 *  @param  subTimer    The TimerMSP432E4_subTimer to be freed.
 *
 *  @sa     TimerMSP432E4_allocateTimerResource()
 */
extern void TimerMSP432E4_freeTimerResource(uint32_t baseAddress,
    TimerMSP432E4_SubTimer subTimer);

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_timer_TimerMSP432E4__include */
