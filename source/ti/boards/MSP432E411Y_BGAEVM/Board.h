/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
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

#ifndef __BOARD_H
#define __BOARD_H

#define Board_MSP432E411Y_BGAEVM

#include <ti/drivers/Board.h>
#include "MSP432E411Y_BGAEVM.h"

#ifdef __cplusplus
extern "C" {
#endif

#define Board_ADC0                  MSP432E411Y_BGAEVM_ADC0
#define Board_ADC1                  MSP432E411Y_BGAEVM_ADC1

#define Board_ADCBUF0               MSP432E411Y_BGAEVM_ADCBUF0
#define Board_ADCBUF0CHANNEL0       MSP432E411Y_BGAEVM_ADCBUF0CHANNEL0
#define Board_ADCBUF0CHANNEL1       MSP432E411Y_BGAEVM_ADCBUF0CHANNEL1
#define Board_ADCBUF0CHANNEL2       MSP432E411Y_BGAEVM_ADCBUF0CHANNEL2
#define Board_ADCBUF0CHANNEL3       MSP432E411Y_BGAEVM_ADCBUF0CHANNEL3
#define Board_ADCBUF0CHANNEL4       MSP432E411Y_BGAEVM_ADCBUF0CHANNEL4

#define Board_GPIO_LED_ON           MSP432E411Y_BGAEVM_GPIO_LED_ON
#define Board_GPIO_LED_OFF          MSP432E411Y_BGAEVM_GPIO_LED_OFF
#define Board_GPIO_LED0             MSP432E411Y_BGAEVM_GPIO_D1
#define Board_GPIO_LED1             MSP432E411Y_BGAEVM_GPIO_D2
#define Board_GPIO_LED2             MSP432E411Y_BGAEVM_GPIO_D2
#define Board_GPIO_BUTTON0          MSP432E411Y_BGAEVM_GPIO_USR_SW1
#define Board_GPIO_BUTTON1          MSP432E411Y_BGAEVM_GPIO_USR_SW2

#define Board_I2C0                  MSP432E411Y_BGAEVM_I2C0
#define Board_I2C_TMP               MSP432E411Y_BGAEVM_I2C0

#define Board_NVSINTERNAL           MSP432E411Y_BGAEVM_NVSMSP432E40

#define Board_PWM0                  MSP432E411Y_BGAEVM_PWM0

#define Board_SD0                   MSP432E411Y_BGAEVM_SDSPI0

#define Board_SDFatFS0              MSP432E411Y_BGAEVM_SDSPI0

#define Board_SPI0                  MSP432E411Y_BGAEVM_SPI2
#define Board_SPI1                  MSP432E411Y_BGAEVM_SPI3

#define Board_SPI_MASTER            MSP432E411Y_BGAEVM_SPI2
#define Board_SPI_SLAVE             MSP432E411Y_BGAEVM_SPI2
#define Board_SPI_MASTER_READY      MSP432E411Y_BGAEVM_SPI_MASTER_READY
#define Board_SPI_SLAVE_READY       MSP432E411Y_BGAEVM_SPI_SLAVE_READY

#define Board_TIMER0                MSP432E411Y_BGAEVM_TIMER0
#define Board_TIMER1                MSP432E411Y_BGAEVM_TIMER1
#define Board_TIMER2                MSP432E411Y_BGAEVM_TIMER2

#define Board_UART0                 MSP432E411Y_BGAEVM_UART0

#define Board_WATCHDOG0             MSP432E411Y_BGAEVM_WATCHDOG0

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
