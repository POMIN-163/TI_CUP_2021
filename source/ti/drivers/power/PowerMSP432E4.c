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
/*
 *  ======== PowerMSP432E4.c ========
 */

#include <stdint.h>

#include <ti/devices/msp432e4/inc/msp432.h>

#include <ti/devices/msp432e4/driverlib/sysctl.h>

#include <ti/drivers/dpl/HwiP.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432E4.h>

/* externs */
extern const PowerMSP432E4_Config PowerMSP432E4_config;

/* Module_State */
PowerMSP432E4_ModuleState PowerMSP432E4_module = {
    /* dbRecords */
    {
        SYSCTL_PERIPH_ADC0,
        SYSCTL_PERIPH_ADC1,
        SYSCTL_PERIPH_CAN0,
        SYSCTL_PERIPH_CAN1,
        SYSCTL_PERIPH_CCM0,
        SYSCTL_PERIPH_COMP0,
        SYSCTL_PERIPH_GPIOA,
        SYSCTL_PERIPH_GPIOB,
        SYSCTL_PERIPH_GPIOC,
        SYSCTL_PERIPH_GPIOD,
        SYSCTL_PERIPH_GPIOE,
        SYSCTL_PERIPH_GPIOF,
        SYSCTL_PERIPH_GPIOG,
        SYSCTL_PERIPH_GPIOH,
        SYSCTL_PERIPH_GPIOJ,
        SYSCTL_PERIPH_GPIOK,
        SYSCTL_PERIPH_GPIOL,
        SYSCTL_PERIPH_GPIOM,
        SYSCTL_PERIPH_GPION,
        SYSCTL_PERIPH_GPIOP,
        SYSCTL_PERIPH_GPIOQ,
        SYSCTL_PERIPH_GPIOR,
        SYSCTL_PERIPH_GPIOS,
        SYSCTL_PERIPH_GPIOT,
        SYSCTL_PERIPH_EMAC0,
        SYSCTL_PERIPH_EPHY0,
        SYSCTL_PERIPH_EPI0,
        SYSCTL_PERIPH_I2C0,
        SYSCTL_PERIPH_I2C1,
        SYSCTL_PERIPH_I2C2,
        SYSCTL_PERIPH_I2C3,
        SYSCTL_PERIPH_I2C4,
        SYSCTL_PERIPH_I2C5,
        SYSCTL_PERIPH_I2C6,
        SYSCTL_PERIPH_I2C7,
        SYSCTL_PERIPH_I2C8,
        SYSCTL_PERIPH_I2C9,
        SYSCTL_PERIPH_LCD0,
        SYSCTL_PERIPH_ONEWIRE0,
        SYSCTL_PERIPH_PWM0,
        SYSCTL_PERIPH_QEI0,
        SYSCTL_PERIPH_SSI0,
        SYSCTL_PERIPH_SSI1,
        SYSCTL_PERIPH_SSI2,
        SYSCTL_PERIPH_SSI3,
        SYSCTL_PERIPH_TIMER0,
        SYSCTL_PERIPH_TIMER1,
        SYSCTL_PERIPH_TIMER2,
        SYSCTL_PERIPH_TIMER3,
        SYSCTL_PERIPH_TIMER4,
        SYSCTL_PERIPH_TIMER5,
        SYSCTL_PERIPH_TIMER6,
        SYSCTL_PERIPH_TIMER7,
        SYSCTL_PERIPH_UART0,
        SYSCTL_PERIPH_UART1,
        SYSCTL_PERIPH_UART2,
        SYSCTL_PERIPH_UART3,
        SYSCTL_PERIPH_UART4,
        SYSCTL_PERIPH_UART5,
        SYSCTL_PERIPH_UART6,
        SYSCTL_PERIPH_UART7,
        SYSCTL_PERIPH_UDMA,
        SYSCTL_PERIPH_USB0,
        SYSCTL_PERIPH_WDOG0,
        SYSCTL_PERIPH_WDOG1,
        SYSCTL_PERIPH_CAN0,  /* CAN0 domain */
        SYSCTL_PERIPH_CAN1,  /* CAN1 domain */
        SYSCTL_PERIPH_CCM0,  /* CCM0 domain */
        SYSCTL_PERIPH_EMAC0, /* EMAC domain */
        SYSCTL_PERIPH_EPHY0, /* EPHY domain */
        SYSCTL_PERIPH_USB0   /* USB0 domain */
    },
    /* policyFxn */
    NULL,
    /* enablePolicy */
    false,
    /* initialized */
    false,
    /* refCount */
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

/*
 *  ======== Power_disablePolicy ========
 *  Disable the configured policy
 */
bool Power_disablePolicy(void)
{
    bool enablePolicy = PowerMSP432E4_module.enablePolicy;
    PowerMSP432E4_module.enablePolicy = false;
    return (enablePolicy);
}

/*
 *  ======== Power_enablePolicy ========
 *  Enable the configured policy
 */
void Power_enablePolicy(void)
{
    PowerMSP432E4_module.enablePolicy = true;
}

/*
 *  ======== Power_getDependencyCount ========
 *  Get the dependency count currently declared for a resource
 */
int_fast16_t Power_getDependencyCount(uint_fast16_t resourceId)
{
    int_fast16_t status;

    if (resourceId >= PowerMSP432E4_NUMRESOURCES) {
        status = Power_EINVALIDINPUT;
    }
    else {
        status = PowerMSP432E4_module.refCount[resourceId];
    }

    return (status);
}

/*
 *  ======== Power_idleFunc ========
 *  Idle loop function for invoking policy.
 *  It calls the configured policy function if the 'enablePolicy' flag is true.
 */
void Power_idleFunc()
{
    if (PowerMSP432E4_module.enablePolicy) {
        if (PowerMSP432E4_module.policyFxn != NULL) {
            (*(PowerMSP432E4_module.policyFxn))();
        }
    }
}

/*
 *  ======== Power_init ========
 *  Initialize Power Manager state
 */
int_fast16_t Power_init()
{
    if (PowerMSP432E4_module.initialized) {
        return (Power_SOK);
    }

    PowerMSP432E4_module.initialized = true;

    PowerMSP432E4_module.enablePolicy = PowerMSP432E4_config.enablePolicy;
    PowerMSP432E4_module.policyFxn = PowerMSP432E4_config.policyFxn;

    /* disable peripheral power domains that are enabled by default */
    SysCtlPeripheralPowerOff(SYSCTL_PERIPH_CAN0);
    SysCtlPeripheralPowerOff(SYSCTL_PERIPH_CAN1);
    SysCtlPeripheralPowerOff(SYSCTL_PERIPH_CCM0);
    SysCtlPeripheralPowerOff(SYSCTL_PERIPH_EMAC0);
    SysCtlPeripheralPowerOff(SYSCTL_PERIPH_USB0);

    /* enable auto clock gating in sleep modes */
    SysCtlPeripheralClockGating(true);

    return (Power_SOK);
}

/*
 *  ======== Power_releaseDependency ========
 *  Release a previously declared dependency
 */
int_fast16_t Power_releaseDependency(uint_fast16_t resourceId)
{
    int_fast16_t status = Power_SOK;
    uint8_t count;
    uint32_t id;
    uint32_t key;

    /* first check that resourceId is valid */
    if (resourceId >= PowerMSP432E4_NUMRESOURCES) {
        status = Power_EINVALIDINPUT;
    }

    /* next check that the resource is present on this device */
    else if (
        !SysCtlPeripheralPresent(PowerMSP432E4_module.dbRecords[resourceId])) {
        status = Power_EINVALIDINPUT;
    }

    /* if resourceId is OK ... */
    else {

        /* disable interrupts */
        key = HwiP_disable();

        /* read the reference count */
        count = PowerMSP432E4_module.refCount[resourceId];

        /* ensure dependency count is not already zero */
        if (count == 0) {
            status = Power_EFAIL;
        }

        /* if not already zero ... */
        else {

            /* decrement the reference count */
            count--;

            /* if this was the last dependency being released.., */
            if (count == 0) {
                /* deactivate this resource ... */
                id = PowerMSP432E4_module.dbRecords[resourceId];

                /* if a peripheral resource, disable run and sleep clocks */
                if (resourceId <= PowerMSP432E4_LASTPERIPH) {
                    SysCtlPeripheralDisable(id);
                    SysCtlPeripheralSleepDisable(id);
                }
                /* else is a domain resource, disable the domain */
                else {
                    SysCtlPeripheralPowerOff(id);
                }
            }

            /* save the updated count */
            PowerMSP432E4_module.refCount[resourceId] = count;
        }

        /* restore interrupts */
        HwiP_restore(key);
    }

    return (status);
}

/*
 *  ======== Power_setDependency ========
 *  Declare a dependency upon a resource
 */
int_fast16_t Power_setDependency(uint_fast16_t resourceId)
{
    int_fast16_t status = Power_SOK;
    uint8_t count;
    uint32_t id;
    uint32_t key;

    /* ensure resourceId is valid */
    if (resourceId >= PowerMSP432E4_NUMRESOURCES) {
        status = Power_EINVALIDINPUT;
    }

    /* next ensure that the resource is present on this device */
    else if (
        !SysCtlPeripheralPresent(PowerMSP432E4_module.dbRecords[resourceId])) {
        status = Power_EINVALIDINPUT;
    }

    /* resourceId is OK ... */
    else {

        /* disable interrupts */
        key = HwiP_disable();

        /* read and increment reference count */
        count = PowerMSP432E4_module.refCount[resourceId]++;

        /* if resource was NOT activated previously ... */
        if (count == 0) {
            /* now activate this resource ... */
            id = PowerMSP432E4_module.dbRecords[resourceId];

            /* if a peripheral resource, enable run and sleep clocks */
            if (resourceId <= PowerMSP432E4_LASTPERIPH) {
                SysCtlPeripheralEnable(id);
                SysCtlPeripheralSleepEnable(id);
                /* wait until peripheral is ready for access */
                while (!SysCtlPeripheralReady(id)) {};
            }
            /* else is a domain resource, enable the domain */
            else {
                SysCtlPeripheralPowerOn(id);
            }
        }

        /* restore interrupts */
        HwiP_restore(key);
    }

    return (status);
}

/*
 *  ======== Power_setPolicy ========
 *  Set the Power policy function
 */
void Power_setPolicy(Power_PolicyFxn policy)
{
    PowerMSP432E4_module.policyFxn = policy;
}
