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
/** ============================================================================
 * @file       PWMMSP432E4.h
 *
 * @brief      PWM driver implementation for MSP432E4 PWM peripherals.
 *
 * The PWM header file should be included in an application as follows:
 * @code
 * #include <ti/drivers/PWM.h>
 * #include <ti/drivers/pwm/PWMMSP432E4.h>
 * @endcode
 *
 *  Refer to @ref PWM.h for a complete description of APIs & example of use.
 *
 * ## Operation #
 * This driver implementation uses the Pulse Width Modulator (PWM) peripherals
 * present on MSP432E4 devices to generate PWM signals.  Each PWM peripheral
 * instance contains 4 PWM signal generators, each controlling 2 PWM outputs
 * (8 PWM outputs total).  This driver manages each PWM output individually
 * (each output has it's own PWM handle/instance).  However since a single clock
 * prescalar is available for a peripheral and a generator is responsible for
 * producing 2 outputs, there are some limitations in place to ensure proper
 * operation:
 *     - The peripheral prescalar will be set according to the period of the
 *       first PWM instance opened.  Any subsequent outputs will fail to open if
 *       a greater prescalar is required to generate the PWM period.
 *     - A PWM generator's period is set to the period of first instance
 *       opened.  Opening the second output will fail if the period used is
 *       not the same as what was set by the first output.
 *     - A PWM generator's options are set by the first instance opened.  Opening
 *       the second output will fail if the options are not the same as what was
 *       set by the first output.
 *
 * Since the period and duty registers are 16 bits wide the prescalar is used to
 * divide the input clock and allow for larger periods.  The maximum period
 * supported is calculated as:
 *     - MAX_PERIOD = (MAX_PRESCALAR * MAX_MATCH_VALUE) / CYCLES_PER_US
 *     - Ex:
 *         - 80 MHz clock: (64 * 65535) / 80 = 52428 microseconds
 *         - 120 MHz clock: (64 * 65535) / 120 = 34952 microseconds
 *
 * After opening, the PWM_setPeriodl() API can be used to change a PWM
 * generator period. However, the clock prescalar is shared by all generators
 * so the new period must be a value that can be generated with the same
 * prescaler.  Also keep in mind that changing a period affects both generator
 * outputs, so the period must be larger than both duties.  The equation
 * below can be used to determine the prescalar for a given period (the
 * prescalar will be the following power of 2 (2^x)):
 *     - prescalar = (period * CYCLES_PER_US) / MAX_MATCH_VALUE
 *     - Ex:
 *         - 100 microseconds -> (100 * 80) / 65535 = (0.1220) = 1
 *         - 10000 microseconds -> (10000 * 80) / 65535 = (12.20) = 16
 *
 * =============================================================================
 */

#ifndef ti_drivers_pwm_PWMMSP432E4__include
#define ti_drivers_pwm_PWMMSP432E4__include

#include <stdbool.h>
#include <stdint.h>

#include <ti/devices/msp432e4/inc/msp432.h>

#include <ti/devices/msp432e4/driverlib/gpio.h>
#include <ti/devices/msp432e4/driverlib/pin_map.h>

#include <ti/drivers/gpio/GPIOMSP432E4.h>
#include <ti/drivers/PWM.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  @addtogroup PWM_STATUS
 *  PWMMSP432E4_STATUS_* macros are command codes only defined in the
 *  PWMMSP432E4.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/pwm/PWMMSP432E4.h>
 *  @endcode
 *  @{
 */

/* Add PWMMSP432E4_STATUS_* macros here */

/** @}*/

/**
 *  @addtogroup PWM_CMD
 *  PWMMSP432E4_CMD_* macros are command codes only defined in the
 *  PWMMSP432E4.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/pwm/PWMMSP432E4.h>
 *  @endcode
 *  @{
 */

/** @}*/

/* Number of PWM peripherals available on a device. */
#define PWMMSP432E4_NUM_PWM_PERIPHERALS      (1)

/* Number of signal generator blocks per PWM peripheral. */
#define PWMMSP432E4_NUM_PWM_GENERATORS       (4)

 /* Number of PWM signals a PWM peripheral can generate. */
#define PWMMSP432E4_NUM_PWM_OUTPUTS          (8)

/*!
 * @brief PF0 is used for M0PWM0
 */
#define PWMMSP432E4_PF0_M0PWM0 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTF, 0, GPIO_PF0_M0PWM0)

/*!
 * @brief PR0 is used for M0PWM0
 */
#define PWMMSP432E4_PR0_M0PWM0 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTR, 0, GPIO_PR0_M0PWM0)

/*!
 * @brief PF1 is used for M0PWM1
 */
#define PWMMSP432E4_PF1_M0PWM1 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTF, 1, GPIO_PF1_M0PWM1)

/*!
 * @brief PR1 is used for M0PWM1
 */
#define PWMMSP432E4_PR1_M0PWM1 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTR, 1, GPIO_PR1_M0PWM1)

/*!
 * @brief PF2 is used for M0PWM2
 */
#define PWMMSP432E4_PF2_M0PWM2 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTF, 2, GPIO_PF2_M0PWM2)

/*!
 * @brief PR2 is used for M0PWM2
 */
#define PWMMSP432E4_PR2_M0PWM2 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTR, 2, GPIO_PR2_M0PWM2)

/*!
 * @brief PF3 is used for M0PWM3
 */
#define PWMMSP432E4_PF3_M0PWM3 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTF, 3, GPIO_PF3_M0PWM3)

/*!
 * @brief PR3 is used for M0PWM3
 */
#define PWMMSP432E4_PR3_M0PWM3 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTR, 3, GPIO_PR3_M0PWM3)

/*!
 * @brief PG0 is used for M0PWM4
 */
#define PWMMSP432E4_PG0_M0PWM4 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTG, 0, GPIO_PG0_M0PWM4)

/*!
 * @brief PR4 is used for M0PWM4
 */
#define PWMMSP432E4_PR4_M0PWM4 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTR, 4, GPIO_PR4_M0PWM4)

/*!
 * @brief PG1 is used for M0PWM5
 */
#define PWMMSP432E4_PG1_M0PWM5 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTG, 1, GPIO_PG1_M0PWM5)

/*!
 * @brief PR5 is used for M0PWM5
 */
#define PWMMSP432E4_PR5_M0PWM5 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTR, 5, GPIO_PR5_M0PWM5)

/*!
 * @brief PK4 is used for M0PWM6
 */
#define PWMMSP432E4_PK4_M0PWM6 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTK, 4, GPIO_PK4_M0PWM6)

/*!
 * @brief PR6 is used for M0PWM6
 */
#define PWMMSP432E4_PR6_M0PWM6 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTR, 6, GPIO_PR6_M0PWM6)

/*!
 * @brief PK5 is used for M0PWM7
 */
#define PWMMSP432E4_PK5_M0PWM7 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTK, 5, GPIO_PK5_M0PWM7)

/*!
 * @brief PR7 is used for M0PWM7
 */
#define PWMMSP432E4_PR7_M0PWM7 GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTR, 7, GPIO_PR7_M0PWM7)

/* PWM function table pointer */
extern const PWM_FxnTable PWMMSP432E4_fxnTable;

/*!
 *  @brief  PWMMSP432E4 Hardware attributes
 *
 *  These fields are used by driverlib APIs and therefore must be populated by
 *  driverlib macro definitions. For MSP432E4 driverlib these definitions are found in:
 *      - ti/devices/msp432e4/inc/msp432e4xy.h
 *      - driverlib/gpio.h
 *      - driverlib/pwm.h
 *
 *  A sample structure is shown below:
 *  @code
 *  const PWMMSP432E4_HWAttrs PWMMSP432E4HWAttrs[] = {
 *      {
 *          .pwmBaseAddr = PWM0_BASE,
 *          .pwmOutput = PWM_OUT_0,
 *          .pwmGenOpts = PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN,
 *          .pinPwmMode = PWMMSP432E4__PF0_M0PWM0
 *      }
 *  };
 *  @endcode
 */
typedef struct {
    /*! PWM peripheral base address (ex.: PWM0_BASE, PWM1_BASE, etc.). */
    uint32_t pwmBaseAddr;
    /*! Encoded PWM offset address (ex.: PWM_OUT_0, PWM_OUT_5, etc.). */
    uint32_t pwmOutput;
    /*! Generator options for PWM (ex.: PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN). */
    uint32_t pwmGenOpts;
    /*! PWM pin config (ex.: PWMMSP432E4_PF0_M0PWM0, PWMMSP432E4_PF1_M0PWM1, etc.). */
    uint32_t pinConfig;
} PWMMSP432E4_HWAttrs;

/*!
 *  @brief PWMMSP432E4_Status
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct {
    uint32_t pwmDuties[PWMMSP432E4_NUM_PWM_OUTPUTS];
    uint32_t genPeriods[PWMMSP432E4_NUM_PWM_GENERATORS];
    uint8_t  prescalar;
    uint8_t  activeOutputs;
} PWMMSP432E4_Status;

/*!
 *  @brief  PWMMSP432E4 Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct {
    PWMMSP432E4_Status *pwmStatusStruct;
    PWM_Duty_Units      dutyUnits;
    PWM_Period_Units    periodUnits;
    PWM_IdleLevel       idleLevel;
    uint8_t             pwmOutputNum;
    uint8_t             pwmOutputBit;
    bool                isOpen;
    bool                isRunning;
} PWMMSP432E4_Object;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_pwm_PWMMSP432E4__include */
