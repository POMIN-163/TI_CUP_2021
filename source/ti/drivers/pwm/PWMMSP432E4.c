/*
 * Copyright (c) 2017-2018, Texas Instruments Incorporated
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

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <ti/devices/msp432e4/inc/msp432.h>

#include <ti/devices/msp432e4/driverlib/gpio.h>
#include <ti/devices/msp432e4/driverlib/pwm.h>

#include <ti/drivers/dpl/ClockP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/gpio/GPIOMSP432E4.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432E4.h>
#include <ti/drivers/pwm/PWMMSP432E4.h>

void PWMMSP432E4_close(PWM_Handle handle);
int_fast16_t PWMMSP432E4_control(PWM_Handle handle, unsigned int cmd, void *arg);
void PWMMSP432E4_init(PWM_Handle handle);
PWM_Handle PWMMSP432E4_open(PWM_Handle handle, PWM_Params *params);
int_fast16_t PWMMSP432E4_setDuty(PWM_Handle handle, uint32_t dutyValue);
int_fast16_t PWMMSP432E4_setPeriod(PWM_Handle handle, uint32_t periodValue);
int_fast16_t PWMMSP432E4_setDutyAndPeriod(PWM_Handle handle, uint32_t dutyValue, uint32_t periodValue);
void PWMMSP432E4_start(PWM_Handle handle);
void PWMMSP432E4_stop(PWM_Handle handle);

/* PWM function table for PWMMSP432E4 implementation */
const PWM_FxnTable PWMMSP432E4_fxnTable = {
    PWMMSP432E4_close,
    PWMMSP432E4_control,
    PWMMSP432E4_init,
    PWMMSP432E4_open,
    PWMMSP432E4_setDuty,
    PWMMSP432E4_setPeriod,
    PWMMSP432E4_setDutyAndPeriod,
    PWMMSP432E4_start,
    PWMMSP432E4_stop
};

/*
 * Internal value to notify an error has occurred while calculating a duty
 * or period.
 */
#define PWM_INVALID_VALUE (~0)

/*
 * PWM peripheral registers have 16 bit resolution.  The max register value
 * which be set is 65535.
 */
#define PWM_MAX_MATCH_REG_VALUE (~0)

/*
 * PWM peripheral has a max prescalar of 64.
 */
#define PWM_MAX_PRESCALAR (64)

/*!
 *  @brief If the PWM period is lower than this value, setDutyAndPeriod
 *  will briefly disable the PWM channel to set the new values.
 *
 *  This is to prevent the case where the period, but not the duty, is
 *  applied before the timeout and the next cycle is in an undetermined state.
 */
#define PWM_PERIOD_FOR_GLITCH_PROTECTION 0xF

/* Internal PWM status structures */
static PWMMSP432E4_Status PWMMSP432E4_PWM_STATUS[PWMMSP432E4_NUM_PWM_PERIPHERALS] = {0};

/* Returns the PWM output's corresponding PWM generator */
#define getPwmGenFromOutput(pwmOutput) (pwmOutput & 0xFF0)

/* Returns a pointer to a given PWM outputs period in the status struct */
#define getPwmGenPeriodPtr(statusStruct, pwmOutput) \
    ((statusStruct)->genPeriods + ((pwmOutput / PWM_OUT_0) - 1))

/*
 *  ======== calculatePrescalar ========
 *  Calculates PWM prescalar for a given period.
 *
 *  @param   period       PWM period in timer ticks
 *  @param  *presRegVal   pointer to store the prescalar register value
 *  @param  *prescalar    pointer to store the prescalar decimal value
 *
 */
static inline void calculatePrescalar(uint32_t period, uint32_t *presRegVal,
    uint8_t  *prescalar)
{
    uint8_t  prescalarCount = 0;
    uint32_t prescalarIncrement = 0;

    /* Initialize to a prescalar of 1 */
    *presRegVal = PWM_SYSCLK_DIV_1;
    *prescalar = 1;

    if (period > (uint16_t)PWM_MAX_MATCH_REG_VALUE) {
        while (period > (uint16_t)PWM_MAX_MATCH_REG_VALUE) {
            ++prescalarCount;
            period /=2;
        }

        /* Calculate PWM prescalar - device dependent */
        *presRegVal = PWM_SYSCLK_DIV_2;
        prescalarIncrement = (PWM_SYSCLK_DIV_4 - PWM_SYSCLK_DIV_2);
        *presRegVal += (prescalarIncrement * (prescalarCount - 1));
        *prescalar = 1 << prescalarCount;
    }
}

/*
 *  ======== configurePrescalar ========
 *  Set PWM prescalar for a given peripheral.
 *
 *  @param  pwmBase    PWM peripheral base address
 *  @param  presRegVal prescalar register value
 */
static inline void configurePrescalar(uint32_t pwmBase, uint32_t presRegVal)
{
    /* Set PWM prescalar - device dependent */
    PWMClockSet(pwmBase, presRegVal);
}

/*
 *  ======== checkValidDuty ========
 *
 *  @param   period       PWM period in timer ticks
 *  @param   duty         PWM duty in timer ticks
 */
static int checkValidDuty(PWM_Handle handle, uint32_t period, uint32_t duty)
{
    if (duty == (uint32_t)PWM_INVALID_VALUE) {
        return (PWM_STATUS_ERROR);
    }

    if (duty > period) {
        return (PWM_STATUS_INVALID_DUTY);
    }

    return (PWM_STATUS_SUCCESS);
}

/*
 *  ======== checkValidPeriod ========
 *
 *  @param   period       PWM period in timer ticks
 *  @param   prescalar    PWM prescalar
 */
static int checkValidPeriod(PWM_Handle handle, uint32_t period,
    uint8_t prescalar)
{
    if (period == (uint32_t)PWM_INVALID_VALUE) {
        return (PWM_STATUS_ERROR);
    }

    if ((period == 0) || (prescalar > (uint32_t)PWM_MAX_PRESCALAR)) {
        return (PWM_STATUS_INVALID_PERIOD);
    }

    return (PWM_STATUS_SUCCESS);
}

/*
 *  ======== getDutyTicks ========
 */
static uint32_t getDutyTicks(PWM_Duty_Units dutyUnits, uint32_t dutyValue,
    uint32_t periodCounts)
{
    uint32_t     duty = 0;
    ClockP_FreqHz freq;

    ClockP_getCpuFreq(&freq);

    switch (dutyUnits) {
        case PWM_DUTY_COUNTS:
            duty = dutyValue;
            break;

        case PWM_DUTY_FRACTION:
            duty = (((uint64_t) dutyValue) * ((uint64_t) periodCounts)) /
                PWM_DUTY_FRACTION_MAX;
            break;

        case PWM_DUTY_US:
            duty = (dutyValue != 0) ? (dutyValue * (freq.lo/1000000)) - 1 : 0;
            break;

        default:
            /* Unsupported duty units return an invalid duty */
            duty = PWM_INVALID_VALUE;
    }

    return (duty);
}

/*
 *  ======== getPeriodTicks ========
 */
static uint32_t getPeriodTicks(PWM_Period_Units periodUnits,
    uint32_t periodValue)
{
    uint32_t     period = 0;
    ClockP_FreqHz freq;

    ClockP_getCpuFreq(&freq);

    switch (periodUnits) {
        case PWM_PERIOD_COUNTS:
            period = periodValue;
            break;

        case PWM_PERIOD_HZ:
            if (periodValue && periodValue <= freq.lo) {
                period = (freq.lo / periodValue) - 1;
            }
            break;

        case PWM_PERIOD_US:
            period = (periodValue * (freq.lo/1000000)) - 1;
            break;

        default:
            /* Unsupported period units return an invalid period */
            period = PWM_INVALID_VALUE;
    }

    return (period);
}

/*
 *  ======== initHw ========
 */
static int initHw(PWM_Handle handle, uint32_t period, uint32_t duty)
{
    PWMMSP432E4_Object        *object = handle->object;
    PWMMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    uintptr_t                  key;
    int32_t                    result;
    uint32_t                   prescalarRegVal;
    uint32_t                  *pwmGenPeriodPtr;
    uint32_t                   pwmGenOptions;
    uint32_t                   periodTicks;
    uint32_t                   dutyTicks;
    uint16_t                   pwmGenerator;
    bool                       invertOutput;
    uint8_t                    prescalar;

    periodTicks = getPeriodTicks(object->periodUnits, period);
    dutyTicks = getDutyTicks(object->dutyUnits, duty, periodTicks);
    calculatePrescalar(periodTicks, &prescalarRegVal, &prescalar);

    result = checkValidPeriod(handle, periodTicks, prescalar);
    if (result != PWM_STATUS_SUCCESS) {
        return (result);
    }

    result = checkValidDuty(handle, periodTicks, dutyTicks);
    if (result != PWM_STATUS_SUCCESS) {
        return (result);
    }

    key = HwiP_disable();

    /* Get the instance's PWM generator & period */
    pwmGenerator = getPwmGenFromOutput(hwAttrs->pwmOutput);
    pwmGenPeriodPtr = getPwmGenPeriodPtr(object->pwmStatusStruct,
            hwAttrs->pwmOutput);

    if (prescalar > (uint32_t)PWM_MAX_PRESCALAR) {
        HwiP_restore(key);

        return (PWM_STATUS_INVALID_PERIOD);
    }

    /* Verify if PWM peripheral has been initialized by another PWM instance */
    if ((object->pwmStatusStruct)->prescalar) {
        /*
         * The PWM peripheral has already been initialized. Ensure the
         * prescalar is the same.
         */
        if ((object->pwmStatusStruct)->prescalar != prescalar) {
            HwiP_restore(key);

            return (PWM_STATUS_INVALID_PERIOD);
        }

        /* Check if the other output on the generator is active */
        if (*pwmGenPeriodPtr) {
            /* Ensure period are the same */
            if (periodTicks != *pwmGenPeriodPtr) {
                HwiP_restore(key);

                return (PWM_STATUS_INVALID_PERIOD);
            }

            /* Get PWM generator options & ensure options are the same */
            pwmGenOptions = (*(uint32_t *) (((uint32_t) PWM0) + pwmGenerator)) &
                ~PWM_0_CTL_ENABLE;
            if (hwAttrs->pwmGenOpts != pwmGenOptions) {
                HwiP_restore(key);

                return (PWM_STATUS_ERROR);
            }
        }
    }
    else {
        /* PWM has not been initialized by another instance */
        (object->pwmStatusStruct)->prescalar = prescalar;
        configurePrescalar(hwAttrs->pwmBaseAddr, prescalarRegVal);
    }

    /* Store configuration & mark PWM instance as active */
    (object->pwmStatusStruct)->pwmDuties[object->pwmOutputNum] = dutyTicks;
    (object->pwmStatusStruct)->activeOutputs |= object->pwmOutputBit;

    /* Initialize PWM signal generator */
    if (*pwmGenPeriodPtr == 0) {
        *pwmGenPeriodPtr = periodTicks;
        PWMGenConfigure(hwAttrs->pwmBaseAddr, pwmGenerator,
            hwAttrs->pwmGenOpts);
        PWMGenPeriodSet(hwAttrs->pwmBaseAddr, pwmGenerator,
            periodTicks / prescalar);
    }

    /*
     * The PWM peripheral cannot generate a duty equal to the period when in
     * count down mode or a duty of 0 when in count up-down mode.  In these
     * the PWM output is inverted.  See comments in PWMMSP432E4_setDuty for more
     * information.
     */
    if (hwAttrs->pwmGenOpts & PWM_GEN_MODE_UP_DOWN) {
        invertOutput = (dutyTicks != 0);
    }
    else {
        invertOutput = (dutyTicks != periodTicks);
    }

    /*
     * Special corner case, if duty is 0 we set it to the period without
     * inverting output
     */
    if (dutyTicks == 0) {
        dutyTicks = periodTicks;
    }

    /* Set the PWM output to active-high */
    PWMOutputInvert(hwAttrs->pwmBaseAddr, object->pwmOutputBit, invertOutput);
    PWMPulseWidthSet(hwAttrs->pwmBaseAddr, hwAttrs->pwmOutput,
        (periodTicks - dutyTicks) / prescalar);
    PWMGenEnable(hwAttrs->pwmBaseAddr, pwmGenerator);

    HwiP_restore(key);

    return (PWM_STATUS_SUCCESS);
}

/*
 *  ======== PWMMSP432E4_close ========
 *  @pre    Function assumes that the handle is not NULL
 */
void PWMMSP432E4_close(PWM_Handle handle)
{
    PWMMSP432E4_Object        *object = handle->object;
    PWMMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    uintptr_t                 key;
    uint8_t                   pwmPairOutputBit;
    uint8_t                   port;

    PWMMSP432E4_stop(handle);

    key = HwiP_disable();

    /* Mark PWM as inactive */
    (object->pwmStatusStruct)->activeOutputs &= ~(object->pwmOutputBit);
    object->pwmStatusStruct->pwmDuties[object->pwmOutputNum] = 0;
    object->isOpen = false;

    /* Disable PWM generator if the pair output is not being used. */
    pwmPairOutputBit = (hwAttrs->pwmOutput & 0x01) ?
        (object->pwmOutputBit >> 1) : (object->pwmOutputBit << 1);

    if (((object->pwmStatusStruct)->activeOutputs & pwmPairOutputBit) == 0) {
        PWMGenDisable(hwAttrs->pwmBaseAddr,
            getPwmGenFromOutput(hwAttrs->pwmOutput));
        *(getPwmGenPeriodPtr(object->pwmStatusStruct,
            hwAttrs->pwmOutput)) = 0;
    }

    if ((object->pwmStatusStruct)->activeOutputs == 0) {
        /* PWM completely off, clear all settings */
        (object->pwmStatusStruct)->prescalar = 0;
    }

    HwiP_restore(key);

    /* Release power dependency */
    GPIOMSP432E4_undoPinConfig(hwAttrs->pinConfig);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->pinConfig);
    Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));
    Power_releaseDependency(PowerMSP432E4_PERIPH_PWM0);
}

/*
 *  ======== PWMMSP432E4_control ========
 *  @pre    Function assumes that the handle is not NULL
 */
int_fast16_t PWMMSP432E4_control(PWM_Handle handle, uint_fast16_t cmd, void *arg)
{
    /* No implementation yet */
    return (PWM_STATUS_UNDEFINEDCMD);
}

/*
 *  ======== PWMMSP432E4_init ========
 *  @pre    Function assumes that the handle is not NULL
 */
void PWMMSP432E4_init(PWM_Handle handle)
{
}

/*
 *  ======== PWMMSP432E4_open ========
 *  @pre    Function assumes that the handle is not NULL
 */
PWM_Handle PWMMSP432E4_open(PWM_Handle handle, PWM_Params *params)
{

    PWMMSP432E4_Object        *object = handle->object;
    PWMMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    uintptr_t                  key;
    uint8_t                    structIndex = 0;
    uint8_t                    port;

    key = HwiP_disable();
    if (object->isOpen) {
        HwiP_restore(key);

        return (NULL);
    }
    object->isOpen = true;

    HwiP_restore(key);

    /* Assign corresponding status structure to the PWM instance */
    structIndex = (hwAttrs->pwmBaseAddr - PWM0_BASE) / 0x1000;
    object->pwmStatusStruct = &(PWMMSP432E4_PWM_STATUS[structIndex]);
    object->pwmOutputNum = hwAttrs->pwmOutput & 0x00F;
    object->pwmOutputBit = 1 << object->pwmOutputNum;

    /* Store PWM configuration */
    object->dutyUnits = params->dutyUnits;
    object->idleLevel = params->idleLevel;
    object->periodUnits = params->periodUnits;

    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->pinConfig);
    Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
    Power_setDependency(PowerMSP432E4_PERIPH_PWM0);

    /* Initialize the peripheral & set the period & duty */
    if (initHw(handle, params->periodValue, params->dutyValue) !=
        PWM_STATUS_SUCCESS) {
        PWMMSP432E4_close(handle);

        return (NULL);
    }

    /* Called to set the initial idleLevel */
    PWMMSP432E4_stop(handle);

    return (handle);
}

/*
 *  ======== PWMMSP432E4_setDuty ========
 *  @pre    Function assumes that handle is not NULL
 */
int_fast16_t PWMMSP432E4_setDuty(PWM_Handle handle, uint32_t dutyValue)
{
    uintptr_t                 key;
    uint32_t                  duty;
    uint32_t                  period;
    uint32_t                  *currentDuty;
    PWMMSP432E4_Object        *object = handle->object;
    PWMMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    int_fast16_t              result;

    key = HwiP_disable();

    period = *(getPwmGenPeriodPtr(object->pwmStatusStruct, hwAttrs->pwmOutput));
    duty = getDutyTicks(object->dutyUnits, dutyValue, period);
    currentDuty = &((object->pwmStatusStruct)->pwmDuties[object->pwmOutputNum]);

    result = checkValidDuty(handle, period, duty);
    if (result != PWM_STATUS_SUCCESS) {
        HwiP_restore(key);

        return (result);
    }

    /*
     * The PWM peripheral cannot generate a duty equal to the period when in
     * count down mode or a duty of 0 when in count up-down mode.  To generate a
     * duty equal to the period when in count down mode, the PWM duty is set
     * to the period value and output polarity is inverted.  Additionally, if
     * the output is changed, the PWM output polarity must be inverted again.
     *
     * Likewise, to generate a duty equal to the 0 when in count up-down mode,
     * the PWM duty is set to the period value and the output polarity is
     * inverted.
     *
     * The code below determines if the PWM is in count down or count up-down
     * mode and inverts the PWM output polarity if necessary.
     * For more details refer to the device specific datasheet
     */
    if (hwAttrs->pwmGenOpts & PWM_GEN_MODE_UP_DOWN) {
        /*
         * PWM in count up/down mode - invert output if setting duty to or
         * changing from 0
         */
        if (((duty == 0) && (*currentDuty != 0)) ||
            ((duty != 0) && (*currentDuty == 0))) {
            PWM0->INVERT ^= (uint32_t) object->pwmOutputBit;
        }

    }
    else {
        /*
         * PWM in count down mode - invert output if setting duty to or
         * changing from the period value
         */
        if (((duty == period) && (*currentDuty != period)) ||
            ((duty != period) && (*currentDuty == period))) {
            PWM0->INVERT ^= (uint32_t) object->pwmOutputBit;
        }
    }

    /*
     * Set & store the new duty.  IMPORTANT: this must be saved after output
     * inversion is determined and before the duty = 0 corner case.
     */
    *currentDuty = duty;

    /*
     * Special corner case, if duty is 0 we set it to the period without
     * inverting output
     */
    if (duty == 0) {
        duty = period;
    }

    duty = (period - duty) / (object->pwmStatusStruct)->prescalar;
    PWMPulseWidthSet(hwAttrs->pwmBaseAddr, hwAttrs->pwmOutput, duty);

    HwiP_restore(key);

    return (PWM_STATUS_SUCCESS);
}

/*
 *  ======== PWMMSP432E4_setPeriod ========
 *  @pre    Function assumes that handle is not NULL
 */
int_fast16_t PWMMSP432E4_setPeriod(PWM_Handle handle, uint32_t periodValue)
{
    uintptr_t                  key;
    int32_t                    result;
    uint32_t                   duty;
    uint32_t                   pairDuty;
    uint32_t                   period;
    uint32_t                   prescalarRegVal;
    PWMMSP432E4_Object        *object = handle->object;
    PWMMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    uint8_t                    prescalar;

    key = HwiP_disable();

    period = getPeriodTicks(object->periodUnits, periodValue);
    calculatePrescalar(period, &prescalarRegVal, &prescalar);
    result = checkValidPeriod(handle, period, prescalar);
    if (result != PWM_STATUS_SUCCESS) {
        HwiP_restore(key);
        return (result);
    }

    /* Ensure new period can be generated with the current prescalar. */
    if (prescalar != (object->pwmStatusStruct)->prescalar) {
        HwiP_restore(key);

        return (PWM_STATUS_INVALID_PERIOD);
    }

    /*
     * Due to generators being the source for two PWM outputs, we need to
     * ensure the new period is greater than both output duties.
     */
    duty = (object->pwmStatusStruct)->pwmDuties[object->pwmOutputNum];
    pairDuty = (object->pwmOutputNum & 0x01) ?
        (object->pwmStatusStruct)->pwmDuties[(object->pwmOutputNum - 1)] :
        (object->pwmStatusStruct)->pwmDuties[(object->pwmOutputNum + 1)];
    if (period <= duty || period <= pairDuty) {
        HwiP_restore(key);

        return (PWM_STATUS_INVALID_PERIOD);
    }

    *(getPwmGenPeriodPtr(object->pwmStatusStruct, hwAttrs->pwmOutput)) = period;
    PWMGenPeriodSet(hwAttrs->pwmBaseAddr,
        getPwmGenFromOutput(hwAttrs->pwmOutput), period / prescalar);

    HwiP_restore(key);

    return (PWM_STATUS_SUCCESS);
}

/*
 *  ======== PWMMSP432E4_setDutyAndPeriod ========
 *  @pre    Function assumes that handle is not NULL
 */
int_fast16_t PWMMSP432E4_setDutyAndPeriod(PWM_Handle handle, uint32_t dutyValue, uint32_t periodValue)
{
    uintptr_t                  key;
    uint32_t                   duty;
    uint32_t                   period;
    bool                       stopped = false;
    uint32_t                   pairDuty;
    uint32_t                  *currentDuty;
    uint32_t                   prescalarRegVal;
    PWMMSP432E4_Object        *object = handle->object;
    PWMMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    uint32_t                   oldPeriod;
    uint8_t                    prescalar;
    int32_t                    result;

    key = HwiP_disable();

    oldPeriod = *(getPwmGenPeriodPtr(object->pwmStatusStruct, hwAttrs->pwmOutput));

    period = getPeriodTicks(object->periodUnits, periodValue);
    calculatePrescalar(period, &prescalarRegVal, &prescalar);
    result = checkValidPeriod(handle, period, prescalar);

    if (result != PWM_STATUS_SUCCESS) {
        HwiP_restore(key);
        return (result);
    }

    duty = getDutyTicks(object->dutyUnits, dutyValue, period);
    currentDuty = &((object->pwmStatusStruct)->pwmDuties[object->pwmOutputNum]);
    result = checkValidDuty(handle, period, duty);

    if (result != PWM_STATUS_SUCCESS) {
        HwiP_restore(key);
        return (result);
    }

    /* Ensure new period can be generated with the current prescalar. */
    if (prescalar != (object->pwmStatusStruct)->prescalar) {
        HwiP_restore(key);
        return (PWM_STATUS_INVALID_PERIOD);
    }

    /*
     * Due to generators being the source for two PWM outputs, we need to
     * ensure the new period is greater than both output duties.
     */
    pairDuty = (object->pwmOutputNum & 0x01) ?
        (object->pwmStatusStruct)->pwmDuties[(object->pwmOutputNum - 1)] :
        (object->pwmStatusStruct)->pwmDuties[(object->pwmOutputNum + 1)];

    /* Case where period = this duty handled below.
    Case where period = pairDuty not handled so considered an error. */
    if (period < duty || period <= pairDuty) {
        HwiP_restore(key);
        return (PWM_STATUS_INVALID_PERIOD);
    }

    /*
     * The PWM peripheral cannot generate a duty equal to the period when in
     * count down mode or a duty of 0 when in count up-down mode.  To generate a
     * duty equal to the period when in count down mode, the PWM duty is set
     * to the period value and output polarity is inverted.  Additionally, if
     * the output is changed, the PWM output polarity must be inverted again.
     *
     * Likewise, to generate a duty equal to the 0 when in count up-down mode,
     * the PWM duty is set to the period value and the output polarity is
     * inverted.
     *
     * The code below determines if the PWM is in count down or count up-down
     * mode and inverts the PWM output polarity if necessary.
     * For more details refer to the device specific datasheet
     */
    if (hwAttrs->pwmGenOpts & PWM_GEN_MODE_UP_DOWN) {
        /*
         * PWM in count up/down mode - invert output if setting duty to or
         * changing from 0
         */
        if (((duty == 0) && (*currentDuty != 0)) ||
            ((duty != 0) && (*currentDuty == 0))) {
            PWM0->INVERT ^= (uint32_t) object->pwmOutputBit;
        }

    }
    else {
        /*
         * PWM in count down mode - invert output if setting duty to or
         * changing from the period value
         */
        if (((duty == period) && (*currentDuty != period)) ||
            ((duty != period) && (*currentDuty == period))) {
            PWM0->INVERT ^= (uint32_t) object->pwmOutputBit;
        }
    }

    /*
     * Set & store the new duty.  IMPORTANT: this must be saved after output
     * inversion is determined and before the duty = 0 corner case.
     */
    *currentDuty = duty;

    /*
     * Special corner case, if duty is 0 we set it to the period without
     * inverting output
     */
    if (duty == 0) {
        duty = period;
    }

    if (object->isRunning && oldPeriod <= PWM_PERIOD_FOR_GLITCH_PROTECTION) {
        stopped = true;
        PWMOutputState(hwAttrs->pwmBaseAddr, object->pwmOutputBit, false);
    }

    *(getPwmGenPeriodPtr(object->pwmStatusStruct, hwAttrs->pwmOutput)) = period;
    PWMGenPeriodSet(hwAttrs->pwmBaseAddr, getPwmGenFromOutput(hwAttrs->pwmOutput), period / prescalar);

    duty = (period - duty) / (object->pwmStatusStruct)->prescalar;
    PWMPulseWidthSet(hwAttrs->pwmBaseAddr, hwAttrs->pwmOutput, duty);

    if (stopped) {
        PWMOutputState(hwAttrs->pwmBaseAddr, object->pwmOutputBit, true);
    }

    HwiP_restore(key);
    return (PWM_STATUS_SUCCESS);
}

/*
 *  ======== PWMMSP432E4_start ========
 *  @pre    Function assumes that handle is not NULL
 */
void PWMMSP432E4_start(PWM_Handle handle)
{
    PWMMSP432E4_Object        *object = handle->object;
    PWMMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    uintptr_t                 key;
    uint32_t                  gpioBaseAddress;

    key = HwiP_disable();
    object->isRunning = 1;

    /* Enable PWM output & set pinmux to PWM mode */
    PWMOutputState(hwAttrs->pwmBaseAddr, object->pwmOutputBit, true);
    GPIOPinConfigure(GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->pinConfig));
    gpioBaseAddress = GPIOMSP432E4_getGpioBaseAddr(
        GPIOMSP432E4_getPortFromPinConfig(hwAttrs->pinConfig));
    GPIOPinTypePWM(gpioBaseAddress,
        GPIOMSP432E4_getPinFromPinConfig(hwAttrs->pinConfig));

    HwiP_restore(key);
}

/*
 *  ======== PWMMSP432E4_stop ========
 *  @pre    Function assumes that handle is not NULL
 */
void PWMMSP432E4_stop(PWM_Handle handle)
{
    PWMMSP432E4_Object        *object = handle->object;
    PWMMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;
    uintptr_t                 key;
    uint32_t                  gpioBaseAddress;
    uint8_t                   pin;
    uint8_t                   output;

    key = HwiP_disable();

    /* Set pin as GPIO with IdleLevel value & disable PWM output */
    pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->pinConfig);
    gpioBaseAddress = GPIOMSP432E4_getGpioBaseAddr(
                          GPIOMSP432E4_getPortFromPinConfig(hwAttrs->pinConfig));

    output = (object->idleLevel) ? pin : 0;
    GPIOPinTypeGPIOOutput(gpioBaseAddress, pin);
    GPIOPinWrite(gpioBaseAddress, pin, output);
    PWMOutputState(hwAttrs->pwmBaseAddr, object->pwmOutputBit, false);

    object->isRunning = 0;
    HwiP_restore(key);
}
