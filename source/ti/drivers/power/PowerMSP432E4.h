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
 *  @file       PowerMSP432E4.h
 *
 *  @brief      Power manager interface for MSP432E
 *
 *  The Power header files should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/Power.h>
 *  #include <ti/drivers/power/PowerMSP432E4.h>
 *  @endcode
 *
 *  Refer to @ref Power.h for a complete description of APIs.
 *
 *  ## Implementation #
 *  This module defines the Power APIs and resources for MSP432E.
 *
 *  A reference power policy is provided to transition the CPU from the active
 *  state to a clock-gated sleep state (WFI) when the CPU is idle.
 *
 *  ============================================================================
 */

#ifndef ti_drivers_power_PowerMSP432E4__include
#define ti_drivers_power_PowerMSP432E4__include

#include <stdint.h>
#include <ti/drivers/Power.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Power resources */
#define PowerMSP432E4_PERIPH_ADC0         0
/*!< Resource ID: ADC Module 0 */

#define PowerMSP432E4_PERIPH_ADC1         1
/*!< Resource ID: ADC Module 1 */

#define PowerMSP432E4_PERIPH_CAN0         2
/*!< Resource ID: CAN Controller 0 */

#define PowerMSP432E4_PERIPH_CAN1         3
/*!< Resource ID: CAN Controller 1 */

#define PowerMSP432E4_PERIPH_CCM0         4
/*!< Resource ID: CRC Module */

#define PowerMSP432E4_PERIPH_COMP0        5
/*!< Resource ID: Analog Comparator Module */

#define PowerMSP432E4_PERIPH_GPIOA        6
/*!< Resource ID: GPIO Port A */

#define PowerMSP432E4_PERIPH_GPIOB        7
/*!< Resource ID: GPIO Port B */

#define PowerMSP432E4_PERIPH_GPIOC        8
/*!< Resource ID: GPIO Port C */

#define PowerMSP432E4_PERIPH_GPIOD        9
/*!< Resource ID: GPIO Port D */

#define PowerMSP432E4_PERIPH_GPIOE        10
/*!< Resource ID: GPIO Port E */

#define PowerMSP432E4_PERIPH_GPIOF        11
/*!< Resource ID: GPIO Port F */

#define PowerMSP432E4_PERIPH_GPIOG        12
/*!< Resource ID: GPIO Port G */

#define PowerMSP432E4_PERIPH_GPIOH        13
/*!< Resource ID: GPIO Port H */

#define PowerMSP432E4_PERIPH_GPIOJ        14
/*!< Resource ID: GPIO Port J */

#define PowerMSP432E4_PERIPH_GPIOK        15
/*!< Resource ID: GPIO Port K */

#define PowerMSP432E4_PERIPH_GPIOL        16
/*!< Resource ID: GPIO Port L */

#define PowerMSP432E4_PERIPH_GPIOM        17
/*!< Resource ID: GPIO Port M */

#define PowerMSP432E4_PERIPH_GPION        18
/*!< Resource ID: GPIO Port N */

#define PowerMSP432E4_PERIPH_GPIOP        19
/*!< Resource ID: GPIO Port P */

#define PowerMSP432E4_PERIPH_GPIOQ        20
/*!< Resource ID: GPIO Port Q */

#define PowerMSP432E4_PERIPH_GPIOR        21
/*!< Resource ID: GPIO Port R */

#define PowerMSP432E4_PERIPH_GPIOS        22
/*!< Resource ID: GPIO Port S */

#define PowerMSP432E4_PERIPH_GPIOT        23
/*!< Resource ID: GPIO Port T */

#define PowerMSP432E4_PERIPH_EMAC0        24
/*!< Resource ID: Ethernet Controller Module */

#define PowerMSP432E4_PERIPH_EPHY0        25
/*!< Resource ID: Ethernet PHY */

#define PowerMSP432E4_PERIPH_EPI0         26
/*!< Resource ID: External Peripheral Interface */

#define PowerMSP432E4_PERIPH_I2C0         27
/*!< Resource ID: I2C Module 0 */

#define PowerMSP432E4_PERIPH_I2C1         28
/*!< Resource ID: I2C Module 1 */

#define PowerMSP432E4_PERIPH_I2C2         29
/*!< Resource ID: I2C Module 2 */

#define PowerMSP432E4_PERIPH_I2C3         30
/*!< Resource ID: I2C Module 3 */

#define PowerMSP432E4_PERIPH_I2C4         31
/*!< Resource ID: I2C Module 4 */

#define PowerMSP432E4_PERIPH_I2C5         32
/*!< Resource ID: I2C Module 5 */

#define PowerMSP432E4_PERIPH_I2C6         33
/*!< Resource ID: I2C Module 6 */

#define PowerMSP432E4_PERIPH_I2C7         34
/*!< Resource ID: I2C Module 7 */

#define PowerMSP432E4_PERIPH_I2C8         35
/*!< Resource ID: I2C Module 8 */

#define PowerMSP432E4_PERIPH_I2C9         36
/*!< Resource ID: I2C Module 9 */

#define PowerMSP432E4_PERIPH_LCD0         37
/*!< Resource ID: LCD Controller */

#define PowerMSP432E4_PERIPH_ONEWIRE0     38
/*!< Resource ID: 1-Wire Master Module */

#define PowerMSP432E4_PERIPH_PWM0         39
/*!< Resource ID: Pulse Width Modulator0 */

#define PowerMSP432E4_PERIPH_QEI0         40
/*!< Resource ID: Quadrature Encoder Interface */

#define PowerMSP432E4_PERIPH_SSI0         41
/*!< Resource ID: QSSI Module 0 */

#define PowerMSP432E4_PERIPH_SSI1         42
/*!< Resource ID: QSSI Module 1 */

#define PowerMSP432E4_PERIPH_SSI2         43
/*!< Resource ID: QSSI Module 2 */

#define PowerMSP432E4_PERIPH_SSI3         44
/*!< Resource ID: QSSI Module 3 */

#define PowerMSP432E4_PERIPH_TIMER0       45
/*!< Resource ID: General Purpose Timer Module 0 */

#define PowerMSP432E4_PERIPH_TIMER1       46
/*!< Resource ID: General Purpose Timer Module 1 */

#define PowerMSP432E4_PERIPH_TIMER2       47
/*!< Resource ID: General Purpose Timer Module 2 */

#define PowerMSP432E4_PERIPH_TIMER3       48
/*!< Resource ID: General Purpose Timer Module 3 */

#define PowerMSP432E4_PERIPH_TIMER4       49
/*!< Resource ID: General Purpose Timer Module 4 */

#define PowerMSP432E4_PERIPH_TIMER5       50
/*!< Resource ID: General Purpose Timer Module 5 */

#define PowerMSP432E4_PERIPH_TIMER6       51
/*!< Resource ID: General Purpose Timer Module 6 */

#define PowerMSP432E4_PERIPH_TIMER7       52
/*!< Resource ID: General Purpose Timer Module 7 */

#define PowerMSP432E4_PERIPH_UART0        53
/*!< Resource ID: UART Module 0 */

#define PowerMSP432E4_PERIPH_UART1        54
/*!< Resource ID: UART Module 1 */

#define PowerMSP432E4_PERIPH_UART2        55
/*!< Resource ID: UART Module 2 */

#define PowerMSP432E4_PERIPH_UART3        56
/*!< Resource ID: UART Module 3 */

#define PowerMSP432E4_PERIPH_UART4        57
/*!< Resource ID: UART Module 4 */

#define PowerMSP432E4_PERIPH_UART5        58
/*!< Resource ID: UART Module 5 */

#define PowerMSP432E4_PERIPH_UART6        59
/*!< Resource ID: UART Module 6 */

#define PowerMSP432E4_PERIPH_UART7        60
/*!< Resource ID: UART Module 7 */

#define PowerMSP432E4_PERIPH_UDMA         61
/*!< Resource ID: uDMA Controller */

#define PowerMSP432E4_PERIPH_USB0         62
/*!< Resource ID: USB Controller */

#define PowerMSP432E4_PERIPH_WDOG0        63
/*!< Resource ID: Watchdog Timer 0 */

#define PowerMSP432E4_PERIPH_WDOG1        64
/*!< Resource ID: Watchdog Timer 1 */

#define PowerMSP432E4_DOMAIN_CAN0         65
/*!< Resource ID: CAN0 Power Domain */

#define PowerMSP432E4_DOMAIN_CAN1         66
/*!< Resource ID: CAN1 Power Domain */

#define PowerMSP432E4_DOMAIN_CCM0         67
/*!< Resource ID: CCM0 Power Domain */

#define PowerMSP432E4_DOMAIN_EMAC0        68
/*!< Resource ID: EMAC0 Power Domain */

#define PowerMSP432E4_DOMAIN_EPHY0        69
/*!< Resource ID: EPHY0 Power Domain */

#define PowerMSP432E4_DOMAIN_USB0         70
/*!< Resource ID: USB0 Power Domain */

/* \cond */
/*
 * Last peripheral resource; any resources beyond this will be considered
 * domain resources.  This is an optimization to reduce the footprint of the
 * database, eliminating an extra byte per resource to designate the type of
 * the resource.
 */
#define PowerMSP432E4_LASTPERIPH          PowerMSP432E4_PERIPH_WDOG1

/* Total number of resources in database */
#define PowerMSP432E4_NUMRESOURCES       71
/* \endcond */

/*! @brief Power global configuration structure */
typedef struct {
    /*!
     *  @brief The Power Policy function
     *
     *  When enabled, this function is invoked in the idle loop, to
     *  opportunistically select and activate sleep states.
     */
    Power_PolicyFxn policyFxn;
    /*!
     *  @brief Boolean specifying if the Power Policy function is enabled
     *
     *  If 'true', the policy function will be invoked once for each pass
     *  of the idle loop.
     *
     *  If 'false', the policy will not be invoked.
     *
     *  In addition to this static setting, the Power Policy can be dynamically
     *  enabled and disabled at runtime, via the Power_enablePolicy() and
     *  Power_disablePolicy() functions, respectively.
     */
    bool enablePolicy;
} PowerMSP432E4_Config;

/*!
 *  @cond NODOC
 *  Internal structure defining Power manager state.
 */
typedef struct {
    uint32_t dbRecords[PowerMSP432E4_NUMRESOURCES];
    Power_PolicyFxn policyFxn;
    bool enablePolicy;
    bool initialized;
    uint8_t refCount[PowerMSP432E4_NUMRESOURCES];
} PowerMSP432E4_ModuleState;
/*! @endcond */

/*! OS-specific power policy function */
void PowerMSP432E4_sleepPolicy(void);

/* \cond */
/*
 * The following APIs are not implemented for PowerMSP432E4, and are replaced
 * with the following values ...
 */
#define Power_getConstraintMask(void)                 0
#define Power_getTransitionLatency(state, type)       0
#define Power_getTransitionState(void)                Power_ACTIVE
#define Power_getPerformanceLevel(void)               0
#define Power_registerNotify(object, types, fxn, arg) Power_SOK
#define Power_releaseConstraint(id)                   Power_SOK
#define Power_setConstraint(id)                       Power_SOK
#define Power_setPerformanceLevel(level)              Power_EFAIL
#define Power_shutdown(state, time)                   Power_EINVALIDINPUT
#define Power_sleep(state, time)                      Power_SOK
#define Power_unregisterNotify(object)
/* \endcond */

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_power_PowerMSP432E4__include */
