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

#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/Comparator.h>
#include <ti/drivers/comparator/ComparatorMSP432E4.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432E4.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOMSP432E4.h>

#include <ti/devices/msp432e4/driverlib/comp.h>
#include <ti/devices/msp432e4/driverlib/gpio.h>
#include <ti/devices/msp432e4/driverlib/pin_map.h>


/* Comparator number defines */
#define ComparatorMSP432E4_COMP0            0
#define ComparatorMSP432E4_COMP1            1
#define ComparatorMSP432E4_COMP2            2

/* Register macros */
/*
 *      Comparator Control Register layout (top 16 are reserved)
 *      Masks correspond to one or more bitfields in this register
 *
 *   15    11     10      8     7        6      4        3      1      0
 *  -----------------------------------------------------------------------
 * | RES | TOEN | ASRCP | RES | TSLVAL | TSEN | ISLVAL | ISEN | CINV | RES |
 *  -----------------------------------------------------------------------
 */
#define COMP_CTLREG_OUTINV_MASK     0x00000002 /* 1 - CINV */
#define COMP_CTLREG_INT_MASK        0x0000001C /* 4:2 - ISLVAL + ISEN */
#define COMP_CTLREG_TRIG_MASK       0x000008E0 /* 11, 7:5 - TOEN TSLVAL TSEN */
#define COMP_CTLREG_POS_MASK        0x00000600 /* 10:9 - ASRCP */

#define COMP_CTLREG_GET_OUTINV(x)   (x & COMP_CTLREG_OUTINV_MASK)
#define COMP_CTLREG_GET_INT(x)      (x & COMP_CTLREG_INT_MASK)
#define COMP_CTLREG_GET_TRIG(x)     (x & COMP_CTLREG_TRIG_MASK)
#define COMP_CTLREG_GET_POS(x)      (x & COMP_CTLREG_POS_MASK)

#define COMP_CTLREG_SET_OUTINV(x, y) ((x & ~COMP_CTLREG_OUTINV_MASK) | y)
#define COMP_CTLREG_SET_INT(x, y)    ((x & ~COMP_CTLREG_INT_MASK)    | y)
#define COMP_CTLREG_SET_TRIG(x, y)   ((x & ~COMP_CTLREG_TRIG_MASK)   | y)
#define COMP_CTLREG_SET_POS(x, y)    ((x & ~COMP_CTLREG_POS_MASK)    | y)
/* For use with Comparator_stop, clearing equivalent to "disable" */
#define COMP_CLEARALL               0x0

/* Function declarations */
void ComparatorMSP432E4_init(const Comparator_Handle handle);
Comparator_Handle ComparatorMSP432E4_open(Comparator_Handle handle,
                                          Comparator_Params *params);
int_fast16_t ComparatorMSP432E4_start(Comparator_Handle handle);
uint32_t ComparatorMSP432E4_getLevel(Comparator_Handle handle);
void ComparatorMSP432E4_stop(Comparator_Handle handle);
void ComparatorMSP432E4_close(Comparator_Handle handle);
int_fast16_t ComparatorMSP432E4_getParams(Comparator_Handle handle,
                                     Comparator_Params *params);
int_fast16_t ComparatorMSP432E4_setParams(Comparator_Handle handle,
                                     Comparator_Params *params);

/* Comparator function table for ComparatorMSP432E4 implementation */
const Comparator_FxnTable ComparatorMSP432E4_fxnTable = {
    ComparatorMSP432E4_close,
    ComparatorMSP432E4_getLevel,
    ComparatorMSP432E4_init,
    ComparatorMSP432E4_open,
    ComparatorMSP432E4_start,
    ComparatorMSP432E4_stop,
    ComparatorMSP432E4_getParams,
    ComparatorMSP432E4_setParams
};

/* Static Function Declarations */

/*
 * ========= ComparatorMSP432E4_hwiIntFxn ===========
 */
static void ComparatorMSP432E4_hwiIntFxn(uintptr_t arg)
{
    ComparatorMSP432E4_HWAttrs const *hwAttrs =
            ((Comparator_Handle) arg)->hwAttrs;
    ComparatorMSP432E4_Object *object = ((Comparator_Handle) arg)->object;

    /* Clear pending interrupts */
    ComparatorIntClear(COMP_BASE, hwAttrs->compNum);

    if (object->callbackFxn)
    {
        object->callbackFxn((Comparator_Handle) arg,
                            object->interruptLevel + Comparator_EVENT_RESERVED);
    }
}

/*
 * ========== ComparatorMSP432E4_init ==============
 */

void ComparatorMSP432E4_init(Comparator_Handle handle)
{
    ComparatorMSP432E4_Object *object = handle->object;
    object->isOpen = false;
    object->isRunning = false;
}

/*
 * =========== ComparatorMSP432E4_open ==============
 */
Comparator_Handle ComparatorMSP432E4_open(Comparator_Handle handle,
                                          Comparator_Params *params)
{
    uintptr_t key;
    ComparatorMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    ComparatorMSP432E4_Object *object = handle->object;
    HwiP_Params hwiParams;
    uint32_t pinConfig;
    uint32_t interruptConfig;

    /* Check if this instance is already in use */
    key = HwiP_disable();

    if(object->isOpen == true)
    {
        HwiP_restore(key);
        ComparatorMSP432E4_close(handle);
        return (NULL);
    }

    object->isOpen = true;

    HwiP_restore(key);

    /* Set up Comparator HWI */
    HwiP_Params_init(&hwiParams);
    hwiParams.arg = (uintptr_t) handle;
    hwiParams.priority = hwAttrs->intPriority;
    if(params->interruptLevel == Comparator_INTERRUPT_NONE)
    {
        hwiParams.enableInt = false;
    }
    else
    {
        hwiParams.enableInt = true;
    }

    object->hwi = HwiP_create(hwAttrs->intNum, &ComparatorMSP432E4_hwiIntFxn,
                              &hwiParams);
    if (NULL == object->hwi)
    {
        object->isOpen = false;
        return (NULL);
    }

    /* Map API interrupt level to MSP432E4 interrupt configuration */
    switch(params->interruptLevel) {
        case Comparator_INTERRUPT_NONE:
            interruptConfig = 0;
            break;
        case Comparator_INTERRUPT_RISING:
            interruptConfig = COMP_INT_RISE;
            break;
        case Comparator_INTERRUPT_FALLING:
            interruptConfig = COMP_INT_FALL;
            break;
        case Comparator_INTERRUPT_BOTH:
            interruptConfig = COMP_INT_BOTH;
            break;
        case Comparator_INTERRUPT_HIGH:
            interruptConfig = COMP_INT_HIGH;
            break;
        case Comparator_INTERRUPT_LOW:
            interruptConfig = COMP_INT_LOW;
            break;
        default:
            /* Incorrect argument */
            HwiP_delete(object->hwi);
            object->isOpen = false;
            return (NULL);
    }

    /* Set up object by copying in appropriate params and hwattrs */
    object->interruptLevel = params->interruptLevel;
    object->callbackFxn = params->callbackFxn;
    object->controlReg  = (uint32_t) hwAttrs->positiveTerminal
                        | (uint32_t) hwAttrs->triggerLevel
                        | interruptConfig;

    if(params->outputPolarity == Comparator_OUTPUT_INVERTED)
    {
        object->controlReg |= COMP_OUTPUT_INVERT;
    }
    else if(params->outputPolarity == Comparator_OUTPUT_NORMAL)
    {
        object->controlReg |= COMP_OUTPUT_NORMAL;
    }
    else
    {
        HwiP_delete(object->hwi);
        object->isOpen = false;
        return (NULL);
    }

    /* turn on clock & power shared by all comparator modules */
    Power_setDependency(PowerMSP432E4_PERIPH_COMP0);

    /* disable comparator interrupts for this comparator during configuration */
    ComparatorIntDisable(COMP_BASE, hwAttrs->compNum);

    /* Only set reference level if explicitly requested */
    if(hwAttrs->positiveTerminal == ComparatorMSP432E4_INPUT_REF)
    {
        ComparatorRefSet(COMP_BASE, hwAttrs->refLevel);
    }

    /* Give power to proper GPIO ports */

    /* Inputs for C0 & C1 are on Port C */
    if((hwAttrs->positiveTerminal == ComparatorMSP432E4_INPUT_PIN0)
            || (hwAttrs->compNum < ComparatorMSP432E4_COMP2))
    {
        Power_setDependency(
                GPIOMSP432E4_getPowerResourceId(GPIOMSP432E4_PORTC));
    }

    /* Inputs for C2 are on Port P, check separately for case of PIN0 mode */
    if(hwAttrs->compNum == ComparatorMSP432E4_COMP2)
    {
        Power_setDependency(
                GPIOMSP432E4_getPowerResourceId(GPIOMSP432E4_PORTP));
    }

    /* Set up output port */
    if(hwAttrs->outputPin != COMP_NO_OUTPUT)
    {
        Power_setDependency(
                GPIOMSP432E4_getPowerResourceId(
                        GPIOMSP432E4_getPortFromPinConfig(hwAttrs->outputPin)));
    }
    /* Pin setup*/

    /* Positive terminal setup */
    if((hwAttrs->positiveTerminal == ComparatorMSP432E4_INPUT_PIN0)
            || ((hwAttrs->positiveTerminal != ComparatorMSP432E4_INPUT_REF)
            && (hwAttrs->compNum == ComparatorMSP432E4_COMP0)))
    {
        GPIOPinTypeComparator(GPIO_PORTC_BASE, GPIO_PIN_6);
    }
    else if((hwAttrs->positiveTerminal != ComparatorMSP432E4_INPUT_REF)
            && (hwAttrs->compNum == ComparatorMSP432E4_COMP1))
    {
        GPIOPinTypeComparator(GPIO_PORTC_BASE, GPIO_PIN_5);
    }
    else if((hwAttrs->positiveTerminal != ComparatorMSP432E4_INPUT_REF)
            && (hwAttrs->compNum == ComparatorMSP432E4_COMP2))
    {
        GPIOPinTypeComparator(GPIO_PORTP_BASE, GPIO_PIN_0);
    }
    /* If positive input isn't a REF, it has to be a pin. */
    else if(hwAttrs->positiveTerminal != ComparatorMSP432E4_INPUT_REF)
    {
        HwiP_delete(object->hwi);
        object->isOpen = false;
        return (NULL);
    }

    /* Negative Terminal setup */
    if(hwAttrs->compNum == ComparatorMSP432E4_COMP0)
    {
        GPIOPinTypeComparator(GPIO_PORTC_BASE, GPIO_PIN_7);
    }
    else if(hwAttrs->compNum == ComparatorMSP432E4_COMP1)
    {
        GPIOPinTypeComparator(GPIO_PORTC_BASE, GPIO_PIN_4);
    }
    else if(hwAttrs->compNum == ComparatorMSP432E4_COMP2)
    {
        GPIOPinTypeComparator(GPIO_PORTP_BASE, GPIO_PIN_1);
    }
    else
    {
        HwiP_delete(object->hwi);
        object->isOpen = false;
        return (NULL);
    }

    /* Output Pin */
    if(hwAttrs->outputPin != COMP_NO_OUTPUT)
    {
        GPIOPinTypeComparatorOutput(
                GPIOMSP432E4_getGpioBaseAddr(
                    GPIOMSP432E4_getPortFromPinConfig(hwAttrs->outputPin)
                ),
                GPIOMSP432E4_getPinFromPinConfig(hwAttrs->outputPin)
        );
    }

    /* Only configure a pin if we need to */
    if(hwAttrs->outputPin != COMP_NO_OUTPUT)
    {
        pinConfig = 0;

        switch(hwAttrs->outputPin)
        {
        case COMP0_OUTPUT_PIN_PD0:
            pinConfig = GPIO_PD0_C0O;
            break;
        case COMP0_OUTPUT_PIN_PL2:
            pinConfig = GPIO_PL2_C0O;
            break;
        case COMP1_OUTPUT_PIN_PD1:
            pinConfig = GPIO_PD1_C1O;
            break;
        case COMP1_OUTPUT_PIN_PL3:
            pinConfig = GPIO_PL3_C1O;
            break;
        case COMP2_OUTPUT_PIN_PD2:
            pinConfig = GPIO_PD2_C2O;
            break;
        default:
            HwiP_delete(object->hwi);
            object->isOpen = false;
            return (NULL);
        }

        GPIOPinConfigure(pinConfig);
    }

    return (handle);
}

/*
 * ============ ComparatorMSP432E4_start =============
 */
int_fast16_t ComparatorMSP432E4_start(Comparator_Handle handle)
{
    ComparatorMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    ComparatorMSP432E4_Object *object = handle->object;
    uintptr_t key;

    /* Check if current comparator is already in use */
    key = HwiP_disable();
    if ((object->isRunning == true) || (object->isOpen == false))
    {
        HwiP_restore(key);
        return (Comparator_STATUS_ERROR);
    }
    object->isRunning = true;
    HwiP_restore(key);

    /* Configuring comparator equivalent to enabling comparator */
    ComparatorConfigure(COMP_BASE, hwAttrs->compNum, object->controlReg);
    ComparatorIntEnable(COMP_BASE, hwAttrs->compNum);

    return (Comparator_STATUS_SUCCESS);
}

/*
 * ============= ComparatorMSP432E4_getLevel ==============
 */
uint32_t ComparatorMSP432E4_getLevel(Comparator_Handle handle)
{
    ComparatorMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    /* Type-change to enum value handled in Comparator.h API */
    return (ComparatorValueGet(COMP_BASE, hwAttrs->compNum));
}

/*
 * ============== ComparatorMSP432E4_stop ==================
 */
void ComparatorMSP432E4_stop(Comparator_Handle handle)
{
    ComparatorMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    ComparatorMSP432E4_Object *object = handle->object;
    uintptr_t key;

    /* Disable interrupts while re-configuring comparator */
    key = HwiP_disable();
    if(object->isRunning == false)
    {
        HwiP_restore(key);
        return;
    }
    /* 0-configuration equivalent to "disable" */
    ComparatorConfigure(COMP_BASE, hwAttrs->compNum, COMP_CLEARALL);
    ComparatorIntDisable(COMP_BASE, hwAttrs->compNum);
    object->isRunning = false;
    HwiP_restore(key);
}

/*
 * ============= ComparatorMSP432E4_close ==================
 */
void ComparatorMSP432E4_close(Comparator_Handle handle)
{
    uintptr_t key;
    ComparatorMSP432E4_Object *object = handle->object;
    ComparatorMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;

    key = HwiP_disable();
    if(object->isOpen == false)
    {
        HwiP_restore(key);
        return;
    }

    if(object->isRunning == true)
    {
        ComparatorMSP432E4_stop(handle);
    }

    object->isOpen = false;
    HwiP_restore(key);

    /* Remove comparator power dependency */
    Power_releaseDependency(PowerMSP432E4_PERIPH_COMP0);

    /* Remove GPIO dependencies */
    /* Inputs for C0 & C1 are on Port C */
    if((COMP_CTLREG_GET_POS(object->controlReg)
            == ComparatorMSP432E4_INPUT_PIN0)
            || (hwAttrs->compNum < ComparatorMSP432E4_COMP2))
    {
        Power_releaseDependency(
                GPIOMSP432E4_getPowerResourceId(GPIOMSP432E4_PORTC));
    }

    /* Inputs for C2 are on Port P, check separately for case of PIN0 mode */
    if(hwAttrs->compNum == ComparatorMSP432E4_COMP2)
    {
        Power_releaseDependency(
                GPIOMSP432E4_getPowerResourceId(GPIOMSP432E4_PORTP));
    }

    if(hwAttrs->outputPin != COMP_NO_OUTPUT)
    {
        switch(hwAttrs->outputPin)
        {
        case COMP0_OUTPUT_PIN_PD0:
        case COMP1_OUTPUT_PIN_PD1:
        case COMP2_OUTPUT_PIN_PD2:
            Power_releaseDependency(
                    GPIOMSP432E4_getPowerResourceId(GPIOMSP432E4_PORTD));
            break;
        case COMP0_OUTPUT_PIN_PL2:
        case COMP1_OUTPUT_PIN_PL3:
            Power_releaseDependency(
                    GPIOMSP432E4_getPowerResourceId(GPIOMSP432E4_PORTL));
            break;
        default:
            return;
        }
    }

    if(object->hwi)
    {
        HwiP_delete(object->hwi);
        object->hwi = NULL;
    }
}

/*
 * ============= ComparatorMSP432E4_getParams ================
 */
int_fast16_t ComparatorMSP432E4_getParams(Comparator_Handle handle,
                                     Comparator_Params *params)
{
    ComparatorMSP432E4_Object *object = handle->object;
    uint32_t inversion;
    uintptr_t key;

    key = HwiP_disable();

    params->callbackFxn = object->callbackFxn;

    inversion = COMP_CTLREG_GET_OUTINV(object->controlReg);
    if(inversion == COMP_OUTPUT_INVERT)
    {
        params->outputPolarity = Comparator_OUTPUT_INVERTED;
    }
    else
    {
        params->outputPolarity = Comparator_OUTPUT_NORMAL;
    }

    params->interruptLevel = object->interruptLevel;

    HwiP_restore(key);

    return (Comparator_STATUS_SUCCESS);
}

/*
 * =============== ComparatorMSP432E4_setParams =================
 */
int_fast16_t ComparatorMSP432E4_setParams(Comparator_Handle handle,
                                     Comparator_Params *params)
{
    ComparatorMSP432E4_Object *object = handle->object;
    ComparatorMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    uintptr_t key;
    uint32_t outPol;
    uint32_t interruptConfig;

    /* Disable interrupts while reconfiguring comparator */
    key = HwiP_disable();

    /* Validate input before making changes */
    if(params->outputPolarity == Comparator_OUTPUT_INVERTED)
    {
        outPol = COMP_OUTPUT_INVERT;
    }
    else if(params->outputPolarity == Comparator_OUTPUT_NORMAL)
    {
        outPol = COMP_OUTPUT_NORMAL;
    }
    else
    {
        return (Comparator_STATUS_ERROR);
    }

    /* Map API interrupt level to MSP432E4 interrupt configuration */
    switch(params->interruptLevel) {
        case Comparator_INTERRUPT_NONE:
            interruptConfig = 0;
            break;
        case Comparator_INTERRUPT_RISING:
            interruptConfig = COMP_INT_RISE;
            break;
        case Comparator_INTERRUPT_FALLING:
            interruptConfig = COMP_INT_FALL;
            break;
        case Comparator_INTERRUPT_BOTH:
            interruptConfig = COMP_INT_BOTH;
            break;
        case Comparator_INTERRUPT_HIGH:
            interruptConfig = COMP_INT_HIGH;
            break;
        case Comparator_INTERRUPT_LOW:
            interruptConfig = COMP_INT_LOW;
            break;
        default:
            return (Comparator_STATUS_ERROR);
    }

    /* En/Disable the interrupt according to new setting */
    if(params->interruptLevel == Comparator_INTERRUPT_NONE)
    {
        HwiP_disableInterrupt(hwAttrs->intNum);
    }
    else if((params->interruptLevel == Comparator_INTERRUPT_RISING)
            || params->interruptLevel == Comparator_INTERRUPT_FALLING
            || params->interruptLevel == Comparator_INTERRUPT_BOTH
            || params->interruptLevel == Comparator_INTERRUPT_HIGH
            || params->interruptLevel == Comparator_INTERRUPT_LOW)
    {
        HwiP_enableInterrupt(hwAttrs->intNum);
    }
    else
    {
        /* Not a valid interrupt type */
        return (Comparator_STATUS_ERROR);
    }

    /* Record new state in object */
    object->interruptLevel = params->interruptLevel;
    object->controlReg = COMP_CTLREG_SET_OUTINV(object->controlReg, outPol);
    object->controlReg = COMP_CTLREG_SET_INT(object->controlReg,
                                             interruptConfig);
    object->callbackFxn = params->callbackFxn;
    /* Change comparator configuration */
    ComparatorConfigure(COMP_BASE, hwAttrs->compNum, object->controlReg);
    HwiP_restore(key);

    return (Comparator_STATUS_SUCCESS);
}
