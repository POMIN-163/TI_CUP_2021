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

/* !****************************************************************************
 *  @file       ComparatorMSP432E4.h
 *  @brief      Comparator driver implementation for MSP432E4 family devices
 *
 *  This Comparator driver is meant to operate on a Comparator peripheral
 *  for MSP432E4.
 *
 *  Refer to @ref Comparator.h for a complete description of APIs & example use.
 *
 *******************************************************************************
 */

#ifndef ti_drivers_comparator_ComparatorMSP432E4__include
#define ti_drivers_comparator_ComparatorMSP432E4__include

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/gpio/GPIOMSP432E4.h>

#include <ti/devices/msp432e4/driverlib/comp.h>

/*!
 *  @defgroup ComparatorMSP432E4_EVENT values that will be passed to the
 *  Comparator callback function. In the case of both edges, the edge will
 *  not be determined for accuracy.
 *  @{
 */
#define ComparatorMSP432E4_EVENT_RISING     (Comparator_EVENT_RESERVED + 1)
#define ComparatorMSP432E4_EVENT_FALLING    (Comparator_EVENT_RESERVED + 2)
#define ComparatorMSP432E4_EVENT_EDGE       (Comparator_EVENT_RESERVED + 3)
#define ComparatorMSP432E4_EVENT_HIGH       (Comparator_EVENT_RESERVED + 4)
#define ComparatorMSP432E4_EVENT_LOW        (Comparator_EVENT_RESERVED + 5)
/** @}*/

/*!
 *  @defgroup ComparatorMSP432E4OutputPin values that can be given to the
 *  outputPin field of ComparatorMSP432E4 Hardware Attributes to select the
 *  output pin of a given comparator. Comparator 2 only has a single output
 *  option.
 *  @{
 */
#define COMP0_OUTPUT_PIN_PD0    GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTD, 0, 0)
#define COMP0_OUTPUT_PIN_PL2    GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTL, 2, 0)
#define COMP1_OUTPUT_PIN_PD1    GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTD, 1, 0)
#define COMP1_OUTPUT_PIN_PL3    GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTL, 3, 0)
#define COMP2_OUTPUT_PIN_PD2    GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTD, 2, 0)
#define COMP_NO_OUTPUT          (0)
/** @}*/



#ifdef __cplusplus
extern "C" {
#endif


/*!
 *  @brief ComparatorMSP432E4 Reference Level
 *  These fields are used by MSP432E4 HWAttrs to specify
 *  the voltage output of the shared internal reference.
 *
 */
typedef enum {
    ComparatorMSP432E4_REF_OFF       = COMP_REF_OFF,
    ComparatorMSP432E4_REF_0V        = COMP_REF_0V,
    ComparatorMSP432E4_REF_0_1375V   = COMP_REF_0_1375V,
    ComparatorMSP432E4_REF_0_275V    = COMP_REF_0_275V,
    ComparatorMSP432E4_REF_0_4125V   = COMP_REF_0_4125V,
    ComparatorMSP432E4_REF_0_55V     = COMP_REF_0_55V,
    ComparatorMSP432E4_REF_0_6875V   = COMP_REF_0_6875V,
    ComparatorMSP432E4_REF_0_825V    = COMP_REF_0_825V,
    ComparatorMSP432E4_REF_0_928125V = COMP_REF_0_928125V,
    ComparatorMSP432E4_REF_0_9625V   = COMP_REF_0_9625V,
    ComparatorMSP432E4_REF_1_03125V  = COMP_REF_1_03125V,
    ComparatorMSP432E4_REF_1_134375V = COMP_REF_1_134375V,
    ComparatorMSP432E4_REF_1_1V      = COMP_REF_1_1V,
    ComparatorMSP432E4_REF_1_2375V   = COMP_REF_1_2375V,
    ComparatorMSP432E4_REF_1_340625V = COMP_REF_1_340625V,
    ComparatorMSP432E4_REF_1_375V    = COMP_REF_1_375V,
    ComparatorMSP432E4_REF_1_44375V  = COMP_REF_1_44375V,
    ComparatorMSP432E4_REF_1_5125V   = COMP_REF_1_5125V,
    ComparatorMSP432E4_REF_1_546875V = COMP_REF_1_546875V,
    ComparatorMSP432E4_REF_1_65V     = COMP_REF_1_65V,
    ComparatorMSP432E4_REF_1_753125V = COMP_REF_1_753125V,
    ComparatorMSP432E4_REF_1_7875V   = COMP_REF_1_7875V,
    ComparatorMSP432E4_REF_1_85625V  = COMP_REF_1_85625V,
    ComparatorMSP432E4_REF_1_925V    = COMP_REF_1_925V,
    ComparatorMSP432E4_REF_1_959375V = COMP_REF_1_959375V,
    ComparatorMSP432E4_REF_2_0625V   = COMP_REF_2_0625V,
    ComparatorMSP432E4_REF_2_165625V = COMP_REF_2_165625V,
    ComparatorMSP432E4_REF_2_26875V  = COMP_REF_2_26875V,
    ComparatorMSP432E4_REF_2_371875V = COMP_REF_2_371875V
} ComparatorMSP432E4_ReferenceLevel;

/*!
 *  @brief  ComparatorMSP432E4 Trigger Level
 *  These fields are used by ComparatorMSP432E4_HWAttrs to specify the
 *  comparator output conditions to trigger an ADC conversion on each device.
 *  Check the ADC section of the TRM for a full description of ADC trigger
 *  options.
 *
 */
typedef enum {
    ComparatorMSP432E4_TRIG_NONE = COMP_TRIG_NONE,
    ComparatorMSP432E4_TRIG_HIGH = COMP_TRIG_HIGH,
    ComparatorMSP432E4_TRIG_LOW = COMP_TRIG_LOW,
    ComparatorMSP432E4_TRIG_RISE = COMP_TRIG_RISE,
    ComparatorMSP432E4_TRIG_FALL = COMP_TRIG_FALL,
    ComparatorMSP432E4_TRIG_BOTH = COMP_TRIG_BOTH
} ComparatorMSP432E4_ADCTriggerLevel;

/*!
 *  @brief  ComparatorMSP432E4 Positive Input Channel
 *  These fields are intended to serve as the options for the @ref Comparator.h
 *  API Positive Terminal inputs.
 *
 *  The comparators on the MSP432E4 support three positive input sources:
 *  the module-specific input pin, all sharing the input pin of Comparator 0,
 *  and using a (shared) internal reference.
 */
typedef enum {
    ComparatorMSP432E4_INPUT_PIN = COMP_ASRCP_PIN,
    ComparatorMSP432E4_INPUT_PIN0 = COMP_ASRCP_PIN0,
    ComparatorMSP432E4_INPUT_REF = COMP_ASRCP_REF
} ComparatorMSP432E4_PositiveInputChannel;

/*!
 *  @brief  ComparatorMSP432E4 Hardware attributes
 *
 *  A sample structure is shown below:
 *  @code
 *  const ComparatorMSP432E4_HWAttrs comparatorHWAttrs[1] = {
 *      {
 *        .compNum = MSP_EXP432E401Y_COMP0,
 *        .intNum = INT_COMP0,
 *        .intPriority = (~0),
 *        .refLevel = ComparatorMSP432E4_REF_1_1V,
 *        .triggerLevel = ComparatorMSP432E4_TRIG_NONE,
 *        .outputPin = COMP0_OUTPUT_PIN_PD0
 *      }
 *  };
 *  @endcode
 */
typedef struct {
    /*!< Device-specific interrupt number */
    uint32_t intNum;
    uint32_t intPriority;
    /*!< Comparator module ID */
    uint32_t compNum;
    ComparatorMSP432E4_PositiveInputChannel positiveTerminal;
    ComparatorMSP432E4_ADCTriggerLevel triggerLevel;
    ComparatorMSP432E4_ReferenceLevel refLevel;
    /*!< Output pin. Some comparators only have 1 option.*/
    uint32_t outputPin;
} ComparatorMSP432E4_HWAttrs;

/*!
 *  @brief  ComparatorMSP432E4 Object
 *
 *  The application must not access any member variables of this structure!
 */

typedef struct {
    bool isOpen; /* Determine if Comparator is open */
    bool isRunning; /* Determine if Comparator instance is already running */
    uint32_t controlReg; /* No getter function for comparator settings,
                            record here to simplify get/setParams API */
    HwiP_Handle hwi;
    Comparator_CallBackFxn callbackFxn;
    Comparator_InterruptLevel interruptLevel;
} ComparatorMSP432E4_Object;

extern const Comparator_FxnTable ComparatorMSP432E4_fxnTable;


#ifdef __cplusplus
}
#endif


#endif /* COMPARATORMSP432E4_H_ */
