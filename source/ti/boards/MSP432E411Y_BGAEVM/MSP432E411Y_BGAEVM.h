/*
 * Copyright (c) 2018, Texas Instruments Incorporated
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
 *  @file       MSP432E411Y_BGAEVM.h
 *
 *  @brief      MSP432E411Y_BGAEVM Board Specific APIs
 *
 *  The MSP432E411Y_BGAEVM header file should be included in an application as
 *  follows:
 *  @code
 *  #include <MSP432E411Y_BGAEVM.h>
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef __MSP432E411Y_BGAEVM_H
#define __MSP432E411Y_BGAEVM_H

#ifdef __cplusplus
extern "C" {
#endif

/* LEDs on MSP432E411Y_BGAEVM are active high. */
#define MSP432E411Y_BGAEVM_GPIO_LED_OFF (0)
#define MSP432E411Y_BGAEVM_GPIO_LED_ON  (1)

/*!
 *  @def    MSP432E411Y_BGAEVM_ADCName
 *  @brief  Enum of ADC channels on the MSP432E411Y_BGAEVM dev board
 */
typedef enum MSP432E411Y_BGAEVM_ADCName {
    MSP432E411Y_BGAEVM_ADC0 = 0,
    MSP432E411Y_BGAEVM_ADC1,

    MSP432E411Y_BGAEVM_ADCCOUNT
} MSP432E411Y_BGAEVM_ADCName;

/*!
 *  @def    MSP432E411Y_BGAEVM_ADCBufName
 *  @brief  Enum of ADC hardware peripherals on the MSP432E411Y_BGAEVM dev board
 */
typedef enum MSP432E411Y_BGAEVM_ADCBufName {
    MSP432E411Y_BGAEVM_ADCBUF0 = 0,

    MSP432E411Y_BGAEVM_ADCBUFCOUNT
} MSP432E411Y_BGAEVM_ADCBufName;

/*!
 *  @def    MSP432E411Y_BGAEVM_ADCBuf0ChannelName
 *  @brief  Enum of ADCBuf channels on the MSP432E411Y_BGAEVM dev board
 */
typedef enum MSP432E411Y_BGAEVM_ADCBuf0ChannelName {
    MSP432E411Y_BGAEVM_ADCBUF0CHANNEL0 = 0,
    MSP432E411Y_BGAEVM_ADCBUF0CHANNEL1,
    MSP432E411Y_BGAEVM_ADCBUF0CHANNEL2,
    MSP432E411Y_BGAEVM_ADCBUF0CHANNEL3,
    MSP432E411Y_BGAEVM_ADCBUF0CHANNEL4,

    MSP432E411Y_BGAEVM_ADCBUF0CHANNELCOUNT
} MSP432E411Y_BGAEVM_ADCBuf0ChannelName;

/*!
 *  @def    MSP432E411Y_BGAEVM_GPIOName
 *  @brief  Enum of LED names on the MSP432E411Y_BGAEVM dev board
 */
typedef enum MSP432E411Y_BGAEVM_GPIOName {
    MSP432E411Y_BGAEVM_GPIO_USR_SW1 = 0,
    MSP432E411Y_BGAEVM_GPIO_USR_SW2,
    MSP432E411Y_BGAEVM_SPI_MASTER_READY,
    MSP432E411Y_BGAEVM_SPI_SLAVE_READY,
    MSP432E411Y_BGAEVM_GPIO_D1,
    MSP432E411Y_BGAEVM_GPIO_D2,

    MSP432E411Y_BGAEVM_SDSPI_CS,

    /* Sharp LCD Pins */
    MSP432E411Y_BGAEVM_LCD_CS,
    MSP432E411Y_BGAEVM_LCD_POWER,
    MSP432E411Y_BGAEVM_LCD_ENABLE,

    MSP432E411Y_BGAEVM_GPIOCOUNT
} MSP432E411Y_BGAEVM_GPIOName;

/*!
 *  @def    MSP432E411Y_BGAEVM_I2CName
 *  @brief  Enum of I2C names on the MSP432E411Y_BGAEVM dev board
 */
typedef enum MSP432E411Y_BGAEVM_I2CName {
    MSP432E411Y_BGAEVM_I2C0 = 0,

    MSP432E411Y_BGAEVM_I2CCOUNT
} MSP432E411Y_BGAEVM_I2CName;

/*!
 *  @def    MSP432E411Y_BGAEVM_NVSName
 *  @brief  Enum of NVS names on the MSP432E411Y_BGAEVM dev board
 */
typedef enum MSP432E411Y_BGAEVM_NVSName {
    MSP432E411Y_BGAEVM_NVSMSP432E40 = 0,

    MSP432E411Y_BGAEVM_NVSCOUNT
} MSP432E411Y_BGAEVM_NVSName;

/*!
 *  @def    MSP432E411Y_BGAEVM_PWMName
 *  @brief  Enum of PWM names on the MSP432E411Y_BGAEVM dev board
 */
typedef enum MSP432E411Y_BGAEVM_PWMName {
    MSP432E411Y_BGAEVM_PWM0 = 0,

    MSP432E411Y_BGAEVM_PWMCOUNT
} MSP432E411Y_BGAEVM_PWMName;

/*!
 *  @def    MSP432E411Y_BGAEVM_SDFatFSName
 *  @brief  Enum of SDFatFS names on the MSP432E411Y_BGAEVM dev board
 */
typedef enum MSP432E411Y_BGAEVM_SDFatFSName {
    MSP432E411Y_BGAEVM_SDFatFS0 = 0,

    MSP432E411Y_BGAEVM_SDFatFSCOUNT
} MSP432E411Y_BGAEVM_SDFatFSName;

/*!
 *  @def    MSP432E411Y_BGAEVM_SDName
 *  @brief  Enum of SD names on the MSP432E411Y_BGAEVM dev board
 */
typedef enum MSP432E411Y_BGAEVM_SDName {
    MSP432E411Y_BGAEVM_SDSPI0 = 0,

    MSP432E411Y_BGAEVM_SDCOUNT
} MSP432E411Y_BGAEVM_SDName;

/*!
 *  @def    MSP432E411Y_BGAEVM_SPIName
 *  @brief  Enum of SPI names on the MSP432E411Y_BGAEVM dev board
 */
typedef enum MSP432E411Y_BGAEVM_SPIName {
    MSP432E411Y_BGAEVM_SPI2 = 0,
    MSP432E411Y_BGAEVM_SPI3,

    MSP432E411Y_BGAEVM_SPICOUNT
} MSP432E411Y_BGAEVM_SPIName;

/*!
 *  @def    MSP432E411Y_BGAEVM_TimerName
 *  @brief  Enum of Timer names on the MSP432E411Y_BGAEVM dev board
 */
typedef enum MSP432E411Y_BGAEVM_TimerName {
    MSP432E411Y_BGAEVM_TIMER0 = 0,
    MSP432E411Y_BGAEVM_TIMER1,
    MSP432E411Y_BGAEVM_TIMER2,

    MSP432E411Y_BGAEVM_TIMERCOUNT
} MSP432E411Y_BGAEVM_TimerName;

/*!
 *  @def    MSP432E411Y_BGAEVM_UARTName
 *  @brief  Enum of UARTs on the MSP432E411Y_BGAEVM dev board
 */
typedef enum MSP432E411Y_BGAEVM_UARTName {
    MSP432E411Y_BGAEVM_UART0 = 0,

    MSP432E411Y_BGAEVM_UARTCOUNT
} MSP432E411Y_BGAEVM_UARTName;

/*
 *  @def    MSP432E411Y_BGAEVM_WatchdogName
 *  @brief  Enum of Watchdogs on the MSP432E411Y_BGAEVM dev board
 */
typedef enum MSP432E411Y_BGAEVM_WatchdogName {
    MSP432E411Y_BGAEVM_WATCHDOG0 = 0,

    MSP432E411Y_BGAEVM_WATCHDOGCOUNT
} MSP432E411Y_BGAEVM_WatchdogName;

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings.
 *  This includes:
 *     - Enable clock sources for peripherals
 */
extern void MSP432E411Y_BGAEVM_initGeneral(void);

#ifdef __cplusplus
}
#endif

#endif /* __MSP432E411Y_BGAEVM_H */
