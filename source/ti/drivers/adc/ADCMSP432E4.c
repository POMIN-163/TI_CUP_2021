/*
 * Copyright (c) 2018-2019 Texas Instruments Incorporated
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
#include <stdbool.h>

#include <ti/drivers/adc/ADCMSP432E4.h>
#include <ti/drivers/dpl/DebugP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/gpio/GPIOMSP432E4.h>
#include <ti/drivers/power/PowerMSP432E4.h>

/* driverlib header files */
#include <ti/devices/msp432e4/driverlib/adc.h>
#include <ti/devices/msp432e4/driverlib/interrupt.h>
#include <ti/devices/msp432e4/driverlib/rom_map.h>
#include <ti/devices/msp432e4/inc/msp432.h>

#define PinConfigChannel(config) (((config) >> 16) & 0x1F)
#define PinConfigPort(config) (((config << 4) & 0x000FF000) | 0x40000000)
#define PinConfigPin(config) ((config) & 0xFF)

void ADCMSP432E4_close(ADC_Handle handle);
int_fast16_t ADCMSP432E4_control(ADC_Handle handle, uint_fast16_t cmd, void *arg);
int_fast16_t ADCMSP432E4_convert(ADC_Handle handle, uint16_t *value);
uint32_t ADCMSP432E4_convertToMicroVolts(ADC_Handle handle,
    uint16_t adcValue);
void ADCMSP432E4_init(ADC_Handle handle);
ADC_Handle ADCMSP432E4_open(ADC_Handle handle, ADC_Params *params);
static uint_fast16_t ADCMSP432E4_getPowerResourceId(uint32_t port);

/* Keep track of the number of ADC instances per module */
static uint_fast16_t adc0Instance = 0;
static uint_fast16_t adc1Instance = 0;

/* Global mutex ensuring exclusive access to ADC during conversions */
static SemaphoreP_Handle globalMutex[2] = {NULL, NULL};

/* Table of ADC sequencer interrupt vectors */
static const uint8_t adcInterrupts[2][4] = {
    {INT_ADC0SS0, INT_ADC0SS1, INT_ADC0SS2, INT_ADC0SS3},
    {INT_ADC1SS0, INT_ADC1SS1, INT_ADC1SS2, INT_ADC1SS3}
};

/* ADC function table for ADCMSP432E4 implementation */
const ADC_FxnTable ADCMSP432E4_fxnTable = {
    ADCMSP432E4_close,
    ADCMSP432E4_control,
    ADCMSP432E4_convert,
    ADCMSP432E4_convertToMicroVolts,
    ADCMSP432E4_init,
    ADCMSP432E4_open
};

/*
 *  ======== ADCMSP432E4_close ========
 */
void ADCMSP432E4_close(ADC_Handle handle)
{
    uintptr_t         key;
    ADCMSP432E4_Object *object = handle->object;
    ADCMSP432E4_HWAttrsV1 const *hwAttrs = handle->hwAttrs;

    key = HwiP_disable();

    /* Disable sample sequencer*/
    MAP_ADCSequenceDisable(hwAttrs->adcModule, hwAttrs->adcSeq);

    if (hwAttrs->adcModule == ADCMSP432E4_MOD0) {
        adc0Instance--;
        if (adc0Instance == 0) {
            /* Disable interrupts and the ADC module 0 */
            MAP_ADCIntDisableEx(hwAttrs->adcModule, 0xFFF);
            Power_releaseDependency(
                    ADCMSP432E4_getPowerResourceId(hwAttrs->adcModule));
        }
    }
    else {
        adc1Instance--;
        if (adc1Instance == 0) {
            /* Disable interrupts and the ADC module 1 */
            MAP_ADCIntDisableEx(hwAttrs->adcModule, 0xFFF);
            Power_releaseDependency(
                    ADCMSP432E4_getPowerResourceId(hwAttrs->adcModule));
        }
    }

    object->isOpen = false;

    HwiP_restore(key);

    DebugP_log0("ADC: Object closed");
}


/*
 *  ======== ADCMSP432E4_control ========
 */
int_fast16_t ADCMSP432E4_control(ADC_Handle handle, uint_fast16_t cmd,
                                 void *arg)
{
    /* No implementation yet */
    return (ADC_STATUS_UNDEFINEDCMD);
}

/*
 *  ======== ADCMSP432E4_convert ========
 */
int_fast16_t ADCMSP432E4_convert(ADC_Handle handle, uint16_t *value)
{
    ADCMSP432E4_Object          *object = handle->object;
    ADCMSP432E4_HWAttrsV1 const *hwAttrs = handle->hwAttrs;
    uint32_t tempValue = 0;
    uint8_t base = (hwAttrs->adcModule == ADC0_BASE) ? 0 : 1;

    if (object->isProtected) {
        /* Acquire the lock for this particular ADC handle */
        SemaphoreP_pend(globalMutex[base], SemaphoreP_WAIT_FOREVER);
    }

    ADCIntClear(hwAttrs->adcModule, hwAttrs->adcSeq);

    /* Configure ADC sequencer to take one sample at given channel*/
    MAP_ADCSequenceStepConfigure(
            hwAttrs->adcModule, hwAttrs->adcSeq, 0,
            PinConfigChannel(hwAttrs->adcPin) | ADC_CTL_IE | ADC_CTL_END);

    /* Since sample sequencer is now configured, it must be enabled. */
    MAP_ADCSequenceEnable(hwAttrs->adcModule, hwAttrs->adcSeq);

    /* Enable the Interrupt generation from the sequencer */
    MAP_IntEnable(adcInterrupts[base][hwAttrs->adcSeq]);


    /* Trigger the ADC conversion. */
    MAP_ADCProcessorTrigger(hwAttrs->adcModule, hwAttrs->adcSeq);

    /* Wait for conversion to be completed. */
    while(!MAP_ADCIntStatus(hwAttrs->adcModule, hwAttrs->adcSeq, false)) {
    }

    /* Clear the ADC interrupt flag. */
    MAP_ADCIntClear(hwAttrs->adcModule, hwAttrs->adcSeq);

    /* Disable conversion */
    MAP_ADCSequenceDisable(hwAttrs->adcModule, hwAttrs->adcSeq);

    /* Read ADC Value. */
    MAP_ADCSequenceDataGet(hwAttrs->adcModule, hwAttrs->adcSeq, &tempValue);

    /* ADCSequenceDataGet returns a 32-bit number but ADCBuf_convert takes in a
     * 16-bit pointer; typecast tempValue to value as a workaround */
    *value = (uint16_t) tempValue;

    if (object->isProtected) {
        /* Release the lock for this particular ADC handle */
        SemaphoreP_post(globalMutex[base]);
    }

    DebugP_log0("ADC: Convert completed");

    /* Return the number of bytes transfered by the ADC */
    return (ADC_STATUS_SUCCESS);
}

/*
 *  ======== ADCMSP432E4_convertToMicroVolts ========
 */
uint32_t ADCMSP432E4_convertToMicroVolts(ADC_Handle handle,
    uint16_t adcValue)
{
    uint32_t refVoltage = 3300000;
    uint32_t retVal = 0;

    if (adcValue == 0xFFF) {
        retVal = refVoltage;
    }
    else {
        retVal = (((uint64_t)adcValue * refVoltage) / 0x1000);
    }
    return retVal;
}

/*
 *  ======== ADCMSP432E4_init ========
 */
void ADCMSP432E4_init(ADC_Handle handle)
{
    uintptr_t         key;
    SemaphoreP_Handle sem0;
    SemaphoreP_Handle sem1;

    /* Speculatively create a binary semaphore for thread safety per ADC module */
    sem0 = SemaphoreP_createBinary(1);
    sem1 = SemaphoreP_createBinary(1);
    /* sem == NULL will be detected in 'open' */

    key = HwiP_disable();

    /* Create Semaphore for ADC0 */
    if (globalMutex[0] == NULL) {
        /* Use the binary sem created above */
        globalMutex[0] = sem0;

        HwiP_restore(key);
    }
    else {
        /* Init already called */
        HwiP_restore(key);

        if (sem0) {
            /* Delete unused Semaphore */
            SemaphoreP_delete(sem0);
        }
    }

    key = HwiP_disable();

    /* Create Semaphore for ADC1 */
    if (globalMutex[1] == NULL) {
        /* Use the binary sem created above */
        globalMutex[1] = sem1;

        HwiP_restore(key);
    }
    else {
        /* Init already called */
        HwiP_restore(key);

        if (sem1) {
            /* Delete unused Semaphore */
            SemaphoreP_delete(sem1);
        }
    }
}

/*
 *  ======== ADCMSP432E4_open ========
 */
ADC_Handle ADCMSP432E4_open(ADC_Handle handle, ADC_Params *params)
{
    uintptr_t                  key;
    ADCMSP432E4_Object          *object = handle->object;
    ADCMSP432E4_HWAttrsV1 const *hwAttrs = handle->hwAttrs;
    uint32_t powerID;
    uint8_t base = (hwAttrs->adcModule == ADC0_BASE) ? 0 : 1;

    if (globalMutex[base] == NULL) {
        ADCMSP432E4_init(handle);
        if (globalMutex[base] == NULL) {
            DebugP_log0("ADC: mutex Semaphore_create() failed:.");
            ADCMSP432E4_close(handle);
            return (NULL);
        }
    }

    /* Determine if the driver was already opened */
    key = HwiP_disable();

    if (object->isOpen) {
        HwiP_restore(key);

        DebugP_log0("ADC: Error! Already in use.");
        return (NULL);
    }

    if (hwAttrs->adcModule == ADCMSP432E4_MOD0) {
        if (adc0Instance == 0) {
            Power_setDependency(
                    ADCMSP432E4_getPowerResourceId(hwAttrs->adcModule));
        }
        adc0Instance++;
    }
    else {
        if (adc1Instance == 0) {
            Power_setDependency(
                    ADCMSP432E4_getPowerResourceId(hwAttrs->adcModule));
        }
        adc1Instance++;
    }
    /* Set trigger source */
    MAP_ADCSequenceConfigure(hwAttrs->adcModule, hwAttrs->adcSeq,
                             ADC_TRIGGER_PROCESSOR, 2);

    object->isOpen = true;

    /* Remember thread safety protection setting */
    object->isProtected = params->isProtected;

    HwiP_restore(key);

    /* Enable clock for GPIO port */
    powerID = ADCMSP432E4_getPowerResourceId(PinConfigPort(hwAttrs->adcPin));
    if (powerID < PowerMSP432E4_NUMRESOURCES) {
        Power_setDependency(powerID);
    }
    else {
        return NULL;
    }

    /* Config GPIO for ADC channel analog input */
    MAP_GPIOPinTypeADC(PinConfigPort(hwAttrs->adcPin),
    PinConfigPin(hwAttrs->adcPin));

    DebugP_log0("ADC: Object opened");

    return (handle);
}

/*
 *  ======== ADCMSP432E4_getPowerResourceId ========
 */
static uint_fast16_t ADCMSP432E4_getPowerResourceId(uint32_t port) {
    uint_fast16_t resourceID;
    switch (port) {
    case GPIO_PORTB_BASE:
        resourceID = PowerMSP432E4_PERIPH_GPIOB;
        break;
    case GPIO_PORTD_BASE:
        resourceID = PowerMSP432E4_PERIPH_GPIOD;
        break;
    case GPIO_PORTE_BASE:
        resourceID = PowerMSP432E4_PERIPH_GPIOE;
        break;
    case GPIO_PORTK_BASE:
        resourceID = PowerMSP432E4_PERIPH_GPIOK;
        break;
    case GPIO_PORTP_BASE:
        resourceID = PowerMSP432E4_PERIPH_GPIOP;
        break;
    case ADC0_BASE:
        resourceID = PowerMSP432E4_PERIPH_ADC0;
        break;
    case ADC1_BASE:
        resourceID = PowerMSP432E4_PERIPH_ADC1;
        break;
    default:
        resourceID = (uint_fast16_t)-1;
        break;
    }
    return resourceID;
}
