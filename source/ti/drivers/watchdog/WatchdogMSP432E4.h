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
/*!****************************************************************************
 *  @file       WatchdogMSP432E4.h
 *
 *  @brief      Watchdog driver implementation for MSP432E4
 *
 *  The Watchdog header file for MSP432E4 should be included in an application
 *  as follows:
 *  @code
 *  #include <ti/drivers/Watchdog.h>
 *  #include <ti/drivers/watchdog/WatchdogMSP432E4.h>
 *  @endcode
 *
 *  Refer to @ref Watchdog.h for a complete description of APIs.
 *
 *  ## Overview #
 *  This Watchdog driver implementation is designed to operate on a MSP432E4
 *  device. Once opened, MSP432E4 Watchdog will count down from the reload
 *  value specified in the WatchdogMSP432E4_HWAttrs. If it times out, the
 *  Watchdog interrupt flag will be set, and a user-provided callback function
 *  will be called. If resets have been enabled in the #Watchdog_Params and
 *  the Watchdog Timer is allowed to timeout again while the interrupt flag is
 *  still pending, a reset signal will be generated. To prevent a reset,
 *  Watchdog_clear() must be called to clear the interrupt flag.
 *
 *  @warning The watchdog peripheral does not support a Non-Maskable Interrupt (NMI).
 *
 *  The reload value from which the Watchdog Timer counts down may be changed
 *  during runtime using Watchdog_setReload().
 *
 *  Watchdog_close() is <b>not</b> supported by this driver implementation.
 *
 *  By default the Watchdog driver has resets turned on. However, they may be
 *  turned off in the #Watchdog_Params which allows the Watchdog Timer to be
 *  used like another timer interrupt. This functionality is <b>not</b>
 *  supported by all platforms, refer to device specific documentation for
 *  details.
 *
 *  To have a user-defined function run at the warning interrupt, first define
 *  a void-type function that takes a Watchdog_Handle cast to a uintptr_t as an
 *  argument such as the one shown below.
 *
 *  @code
 *  void callback(uintptr_t handle);
 *
 *  ...
 *
 *  Watchdog_Handle handle;
 *  Watchdog_Params params;
 *
 *  Watchdog_Params_init(&params);
 *  params.callbackFxn = callback;
 *  handle = Watchdog_open(CONFIG_WATCHDOG0, &params);
 *  @endcode
 ******************************************************************************
 */
#ifndef ti_drivers_watchdog_WatchdogMSP432E4__include
#define ti_drivers_watchdog_WatchdogMSP432E4__include

#include <stdbool.h>
#include <stdint.h>

#include <ti/drivers/Watchdog.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  @addtogroup Watchdog_STATUS
 *  WatchdogMSP432E4_STATUS_* macros are command codes only defined in the
 *  WatchdogMSP432E4.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/watchdog/WatchdogMSP432E4.h>
 *  @endcode
 *  @{
 */

/* Add WatchdogMSP432E4_STATUS_* macros here */

/** @}*/

/**
 *  @addtogroup Watchdog_CMD
 *  WatchdogMSP432E4_CMD_* macros are command codes only defined in the
 *  WatchdogMSP432E4.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/watchdog/WatchdogMSP432E4.h>
 *  @endcode
 *  @{
 */

/* Add WatchdogMSP432E4_CMD_* macros here */

/** @}*/

/*! @brief  Watchdog function table for MSP432E4 */
extern const Watchdog_FxnTable WatchdogMSP432E4_fxnTable;

/*!
 *  @brief  Watchdog hardware attributes for MSP432E4
 *
 *  Watchdog hardware attributes for the WatchdogMSP432E4 driver's
 *  specific hardware configurations and interrupt priority settings.
 *
 *  The intPriority is the Watchdog timer's interrupt priority, as defined by
 *  the underlying OS.  It is passed unmodified to the underlying OS's
 *  interrupt handler creation code, so you need to refer to the OS
 *  documentation for usage.  For example, for SYS/BIOS applications, refer to
 *  the ti.sysbios.family.arm.m3.Hwi documentation for SYS/BIOS usage of
 *  interrupt priorities.  If the driver uses the ti.dpl interface
 *  instead of making OS calls directly, then the HwiP port handles the
 *  interrupt priority in an OS specific way.  In the case of the SYS/BIOS
 *  port, intPriority is passed unmodified to Hwi_create().
 *
 *  The reloadValue is specified in Watchdog Timer ticks. The MSP432E4 contains
 *  two Watchdog Timer peripherals. The first one (Watchdog Timer 0) is clocked
 *  by the system clock and the second (Watchdog Timer 1) is clocked by the
 *  clock source programmed in the ALTCLK field of the Alternate Clock
 *  Configuration (ALTCLKCFG) register.
 *
 *  A sample structure is shown below:
 *  @code
 *  const WatchdogMSP432E4_HWAttrs watchdogMSP432E4HWAttrs[] =
 *  {
 *      {
 *          .baseAddr = WATCHDOG0_BASE,
 *          .intNum = INT_WATCHDOG,
 *          .intPriority = ~0,
 *          .reloadValue = 80000000
 *      },
 *  };
 *  @endcode
 */
typedef struct {
    uint32_t baseAddr;       /*!< Base address for Watchdog */
    uint32_t intNum;         /*!< WDT interrupt number */
    uint32_t intPriority;    /*!< WDT interrupt priority */
    uint32_t reloadValue;    /*!< Reload value in Watchdog timer ticks */
} WatchdogMSP432E4_HWAttrs;

/*!
 *  @brief  Watchdog Object for MSP432E4
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct {
    bool         isOpen;    /* Flag for open/close status */
} WatchdogMSP432E4_Object;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_watchdog_WatchdogMSP432E4__include */
