/*
 * Copyright (c) 2017, Texas Instruments Incorporated
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

#include <stdint.h>
#include <stdlib.h>

#include <ti/devices/msp432e4/inc/msp432.h>

#include <ti/devices/msp432e4/driverlib/sysctl.h>
#include <ti/devices/msp432e4/driverlib/watchdog.h>

#include <ti/drivers/dpl/ClockP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432E4.h>
#include <ti/drivers/watchdog/WatchdogMSP432E4.h>

/* Function prototypes */
void WatchdogMSP432E4_clear(Watchdog_Handle handle);
void WatchdogMSP432E4_close(Watchdog_Handle handle);
int_fast16_t WatchdogMSP432E4_control(Watchdog_Handle handle, uint_fast16_t cmd,
    void *arg);
void WatchdogMSP432E4_init(Watchdog_Handle handle);
Watchdog_Handle WatchdogMSP432E4_open(Watchdog_Handle handle,
    Watchdog_Params *params);
int_fast16_t WatchdogMSP432E4_setReload(Watchdog_Handle handle, uint32_t value);
uint32_t WatchdogMSP432E4_convertMsToTicks(Watchdog_Handle handle,
    uint32_t milliseconds);

static uint16_t getPowerMgrId(uint32_t baseAddr);

/* Watchdog function table for MSP432E4 implementation */
const Watchdog_FxnTable WatchdogMSP432E4_fxnTable = {
    WatchdogMSP432E4_clear,
    WatchdogMSP432E4_close,
    WatchdogMSP432E4_control,
    WatchdogMSP432E4_init,
    WatchdogMSP432E4_open,
    WatchdogMSP432E4_setReload,
    WatchdogMSP432E4_convertMsToTicks
};

/*
 *  ======== WatchdogMSP432E4_clear ========
 */
void WatchdogMSP432E4_clear(Watchdog_Handle handle)
{
    WatchdogMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;

    WatchdogIntClear(hwAttrs->baseAddr);
}

/*
 *  ======== WatchdogMSP432E4_close ========
 */
void WatchdogMSP432E4_close(Watchdog_Handle handle)
{
    /* Not supported for MSP432E4 */
}

/*
 *  ======== WatchdogMSP432E4_control ========
 *  @pre    Function assumes that the handle is not NULL
 */
int WatchdogMSP432E4_control(Watchdog_Handle handle, unsigned int cmd,
    void *arg)
{
    /* No implementation yet */
    return (Watchdog_STATUS_UNDEFINEDCMD);
}

/*
 *  ======== Watchdog_init ========
 */
void WatchdogMSP432E4_init(Watchdog_Handle handle)
{
}

/*
 *  ======== WatchdogMSP432E4_open ========
 */
Watchdog_Handle WatchdogMSP432E4_open(Watchdog_Handle handle,
    Watchdog_Params *params)
{
    uintptr_t                       key;
    uint16_t                        powerMgrId;
    HwiP_Params                     hwiParams;
    HwiP_Handle                     hwiHandle;
    WatchdogMSP432E4_Object        *object = handle->object;
    WatchdogMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Don't allow preemption */
    key = HwiP_disable();

    /* Check if the Watchdog is open already with the HWAttrs */
    if (object->isOpen) {
        HwiP_restore(key);

        return (NULL);
    }
    object->isOpen = true;

    HwiP_restore(key);

    /* Get the Power id */
    powerMgrId = getPowerMgrId(hwAttrs->baseAddr);
    if (powerMgrId > PowerMSP432E4_NUMRESOURCES) {
        object->isOpen = false;

        return (NULL);
    }
    Power_setDependency(powerMgrId);

    /* Construct Hwi object for watchdog */
    if (params->callbackFxn) {
        HwiP_Params_init(&hwiParams);
        hwiParams.arg = (uintptr_t) handle;
        hwiParams.priority = hwAttrs->intPriority;
        hwiHandle = HwiP_create(hwAttrs->intNum, params->callbackFxn,
            &hwiParams);

        if (hwiHandle == NULL) {
            Power_releaseDependency(powerMgrId);
            object->isOpen = false;

            return (NULL);
        }
    }

    WatchdogUnlock(hwAttrs->baseAddr);
    WatchdogReloadSet(hwAttrs->baseAddr, hwAttrs->reloadValue);
    WatchdogIntClear(hwAttrs->baseAddr);

    /* Set reset mode */
    if (params->resetMode == Watchdog_RESET_ON) {
        WatchdogResetEnable(hwAttrs->baseAddr);
    }
    else {
        WatchdogResetDisable(hwAttrs->baseAddr);
    }

    /* Set debug stall mode */
    if (params->debugStallMode == Watchdog_DEBUG_STALL_ON) {
        WatchdogStallEnable(hwAttrs->baseAddr);
    }
    else {
        WatchdogStallDisable(hwAttrs->baseAddr);
    }

    WatchdogEnable(hwAttrs->baseAddr);

    WatchdogLock(hwAttrs->baseAddr);

    /* Return handle of the Watchdog object */
    return (handle);
}

/*
 *  ======== WatchdogMSP432E4_setReload ========
 */
int_fast16_t WatchdogMSP432E4_setReload(Watchdog_Handle handle, uint32_t value)
{
    WatchdogMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Set value */
    WatchdogUnlock(hwAttrs->baseAddr);
    WatchdogReloadSet(hwAttrs->baseAddr, value);
    WatchdogLock(hwAttrs->baseAddr);

    return (Watchdog_STATUS_SUCCESS);
}

/*
 *  ======== WatchdogMSP432E4_convertMsToTicks ========
 */
uint32_t WatchdogMSP432E4_convertMsToTicks(Watchdog_Handle handle,
    uint32_t milliseconds)
{
    uint32_t                        wdtFreq;
    ClockP_FreqHz                   clockFreq;
    WatchdogMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;

    if (milliseconds == 0) {
        return (0);
    }

    /* Watchdog Timer 0 uses the system clock */
    if (hwAttrs->baseAddr == WATCHDOG0_BASE) {
        ClockP_getCpuFreq(&clockFreq);
        wdtFreq = clockFreq.lo;
    }
    else {
        /* Watchdog Timer 1 uses alternate clock */
        switch (SYSCTL->ALTCLKCFG) {
            case SYSCTL_ALTCLK_PIOSC:
                wdtFreq = 16000000;
                break;
            case SYSCTL_ALTCLK_RTCOSC:
                wdtFreq = 32768;
                break;
            case SYSCTL_ALTCLK_LFIOSC:
                wdtFreq = 33000;
                break;
            default:
                return (0);
        }
    }

    if (((((uint64_t) wdtFreq) * milliseconds) / 1000) <= 0xFFFFFFFF) {
        return ((uint32_t)((((uint64_t) wdtFreq) * milliseconds) / 1000));
    }

    return (0);
}

/*
 *  ======== getPowerMgrId ========
 */
static uint16_t getPowerMgrId(uint32_t baseAddr)
{
    switch (baseAddr) {
        case WATCHDOG0_BASE:
            return (PowerMSP432E4_PERIPH_WDOG0);
        case WATCHDOG1_BASE:
            return (PowerMSP432E4_PERIPH_WDOG1);
        default:
            return (~0);
    }
}
