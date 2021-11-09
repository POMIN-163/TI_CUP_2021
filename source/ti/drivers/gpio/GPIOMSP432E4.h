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
 *  @file       GPIOMSP432E4.h
 *
 *  @brief      MSP432E4 GPIO driver
 *
 *  The GPIO header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/gpio/GPIOMSP432E4.h>
 *  @endcode
 *
 *  Refer to @ref GPIO.h for a complete description of APIs.
 *
 *  # Operation #
 *
 *  The GPIO module allows you to manage General Purpose I/O pins via simple
 *  and portable APIs. The application is required to supply a GPIOMSP432E4_Config
 *  structure to the module (see example below). This structure communicates to
 *  the GPIO module how to configure the pins used by the application (See the
 *  description of GPIO_PinConfig in the GPIO.h file).
 *
 *  The application is required to call GPIO_init(). This function will
 *  initialize all the GPIO pins defined in the GPIO_PinConfig table to the
 *  configurations specified. Once completed the other APIs can be used to
 *  access the pins.

 *  GPIO configuration code:
 *  @code
 *  #include <ti/drivers/GPIO.h>
 *  #include <ti/drivers/gpio/GPIOMSP432E4.h>
 *
 *  // Array of pin configurations
 *  const GPIO_PinConfig gpioPinConfigs[] = {
 *      // Input pins
 *      // USR_SW1
 *      GPIOMSP432E4_PJ0 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING,
 *      // USR_SW2
 *      GPIOMSP432E4_PJ1 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING,
 *
 *      // Output pins
 *      // USR_D1
 *      GPIOMSP432E4_PN1 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_LOW,
 *      // USR_D2
 *      GPIOMSP432E4_PN0 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_LOW,
 *  };
 *
 *  // Array of callback function pointers
 *  const GPIO_callbackFxn gpioCallbackFunctions[] = {
 *      NULL,       // USR_SW1
 *      NULL        // USR_SW2
 *  };
 *
 *  // The device-specific GPIO_config structure
 *  const GPIOMSP432E4_Config GPIOMSP432E4_config = {
 *      .pinConfigs = (GPIO_PinConfig *) gpioPinConfigs,
 *      .callbacks = (GPIO_CallbackFxn *) gpioCallbackFunctions,
 *      .numberOfPinConfigs = sizeof(gpioPinConfigs) / sizeof(GPIO_PinConfig),
 *      .numberOfCallbacks = sizeof(gpioCallbackFunctions) / sizeof(GPIO_CallbackFxn),
 *      .intPriority = (~0)
 *  };
 *  @endcode
 *
 *  Keep in mind that the callback functions will be called in the context of
 *  an interrupt service routine and should be designed accordingly.  When an
 *  interrupt is triggered, the interrupt status of all (interrupt enabled) pins
 *  on a port will be read, cleared, and the respective callbacks will be
 *  executed.  Callbacks will be called in order from least significant bit to
 *  most significant bit.
 *
 *  On select MSP432E4 devices, ports P & Q are capable of per-pin interrupts (each
 *  pin has its own interrupt vector).  If multiple interrupts on port P (or Q)
 *  are triggered, the first interrupt will clear all flags and execute the
 *  respective callbacks.  Interrupts for other pins on P (or Q) will run, but
 *  will not execute callbacks.
 *
 *  ============================================================================
 */

#ifndef ti_drivers_GPIOMSP432E4__include
#define ti_drivers_GPIOMSP432E4__include

#include <stdint.h>

#include <ti/devices/msp432e4/inc/msp432.h>

#include <ti/devices/msp432e4/driverlib/gpio.h>

#include <ti/drivers/GPIO.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 *  @brief  GPIO device specific driver configuration structure
 */
typedef struct {
    /*! Pointer to the board's PinConfig array */
    GPIO_PinConfig *pinConfigs;

    /*! Pointer to the board's callback array */
    GPIO_CallbackFxn *callbacks;

    /*! Number of pin configs defined */
    uint32_t numberOfPinConfigs;

    /*! Number of callbacks defined */
    uint32_t numberOfCallbacks;

    /*! Interrupt priority used for call back interrupts.
     *
     *  intPriority is the interrupt priority, as defined by the
     *  underlying OS.  It is passed unmodified to the underlying OS's
     *  interrupt handler creation code, so you need to refer to the OS
     *  documentation for usage.  For example, for SYS/BIOS applications,
     *  refer to the ti.sysbios.family.arm.m3.Hwi documentation for SYS/BIOS
     *  usage of interrupt priorities.  If the driver uses the ti.dpl
     *  interface instead of making OS calls directly, then the HwiP port
     *  handles the interrupt priority in an OS specific way.  In the case
     *  of the SYS/BIOS port, intPriority is passed unmodified to Hwi_create().
     *
     *  Setting ~0 will configure the lowest possible priority
     */
    uint32_t intPriority;
} GPIOMSP432E4_Config;

/*!
 *   @brief   Port Id that can be passed to GPIOMSP432E4_getGpioBaseAddr()
 */
/*
 *  Note that the port Ids (0x10 and 0x11) for Ports P and Q are out of
 *  sequence.  This is to facilitate access to the interrupt vector
 *  table in the implementation.  Each pin on ports P and Q has a dedicated
 *  interrupt vector.
 */
#define GPIOMSP432E4_PORTA 0x00
#define GPIOMSP432E4_PORTB 0x01
#define GPIOMSP432E4_PORTC 0x02
#define GPIOMSP432E4_PORTD 0x03
#define GPIOMSP432E4_PORTE 0x04
#define GPIOMSP432E4_PORTF 0x05
#define GPIOMSP432E4_PORTG 0x06
#define GPIOMSP432E4_PORTH 0x07
#define GPIOMSP432E4_PORTJ 0x08
#define GPIOMSP432E4_PORTK 0x09
#define GPIOMSP432E4_PORTL 0x0A
#define GPIOMSP432E4_PORTM 0x0B
#define GPIOMSP432E4_PORTN 0x0C
#define GPIOMSP432E4_PORTP 0x10
#define GPIOMSP432E4_PORTQ 0x11
#define GPIOMSP432E4_PORTR 0x0D
#define GPIOMSP432E4_PORTS 0x0E
#define GPIOMSP432E4_PORTT 0x0F

/*!
 *  @brief  Device specific port/pin definition macros
 *
 *  Below are the port/pin definitions to be used within the board's pin
 *  configuration table.  These macros should be OR'd in with the respective pin
 *  configuration settings.
 */
#define GPIOMSP432E4_EMPTY_PIN 0x0000

#define GPIOMSP432E4_PA0      ((GPIOMSP432E4_PORTA << 8) | GPIO_PIN_0)
#define GPIOMSP432E4_PA1      ((GPIOMSP432E4_PORTA << 8) | GPIO_PIN_1)
#define GPIOMSP432E4_PA2      ((GPIOMSP432E4_PORTA << 8) | GPIO_PIN_2)
#define GPIOMSP432E4_PA3      ((GPIOMSP432E4_PORTA << 8) | GPIO_PIN_3)
#define GPIOMSP432E4_PA4      ((GPIOMSP432E4_PORTA << 8) | GPIO_PIN_4)
#define GPIOMSP432E4_PA5      ((GPIOMSP432E4_PORTA << 8) | GPIO_PIN_5)
#define GPIOMSP432E4_PA6      ((GPIOMSP432E4_PORTA << 8) | GPIO_PIN_6)
#define GPIOMSP432E4_PA7      ((GPIOMSP432E4_PORTA << 8) | GPIO_PIN_7)

#define GPIOMSP432E4_PB0      ((GPIOMSP432E4_PORTB << 8) | GPIO_PIN_0)
#define GPIOMSP432E4_PB1      ((GPIOMSP432E4_PORTB << 8) | GPIO_PIN_1)
#define GPIOMSP432E4_PB2      ((GPIOMSP432E4_PORTB << 8) | GPIO_PIN_2)
#define GPIOMSP432E4_PB3      ((GPIOMSP432E4_PORTB << 8) | GPIO_PIN_3)
#define GPIOMSP432E4_PB4      ((GPIOMSP432E4_PORTB << 8) | GPIO_PIN_4)
#define GPIOMSP432E4_PB5      ((GPIOMSP432E4_PORTB << 8) | GPIO_PIN_5)
#define GPIOMSP432E4_PB6      ((GPIOMSP432E4_PORTB << 8) | GPIO_PIN_6)
#define GPIOMSP432E4_PB7      ((GPIOMSP432E4_PORTB << 8) | GPIO_PIN_7)

#define GPIOMSP432E4_PC0      ((GPIOMSP432E4_PORTC << 8) | GPIO_PIN_0)
#define GPIOMSP432E4_PC1      ((GPIOMSP432E4_PORTC << 8) | GPIO_PIN_1)
#define GPIOMSP432E4_PC2      ((GPIOMSP432E4_PORTC << 8) | GPIO_PIN_2)
#define GPIOMSP432E4_PC3      ((GPIOMSP432E4_PORTC << 8) | GPIO_PIN_3)
#define GPIOMSP432E4_PC4      ((GPIOMSP432E4_PORTC << 8) | GPIO_PIN_4)
#define GPIOMSP432E4_PC5      ((GPIOMSP432E4_PORTC << 8) | GPIO_PIN_5)
#define GPIOMSP432E4_PC6      ((GPIOMSP432E4_PORTC << 8) | GPIO_PIN_6)
#define GPIOMSP432E4_PC7      ((GPIOMSP432E4_PORTC << 8) | GPIO_PIN_7)

#define GPIOMSP432E4_PD0      ((GPIOMSP432E4_PORTD << 8) | GPIO_PIN_0)
#define GPIOMSP432E4_PD1      ((GPIOMSP432E4_PORTD << 8) | GPIO_PIN_1)
#define GPIOMSP432E4_PD2      ((GPIOMSP432E4_PORTD << 8) | GPIO_PIN_2)
#define GPIOMSP432E4_PD3      ((GPIOMSP432E4_PORTD << 8) | GPIO_PIN_3)
#define GPIOMSP432E4_PD4      ((GPIOMSP432E4_PORTD << 8) | GPIO_PIN_4)
#define GPIOMSP432E4_PD5      ((GPIOMSP432E4_PORTD << 8) | GPIO_PIN_5)
#define GPIOMSP432E4_PD6      ((GPIOMSP432E4_PORTD << 8) | GPIO_PIN_6)
#define GPIOMSP432E4_PD7      ((GPIOMSP432E4_PORTD << 8) | GPIO_PIN_7)

#define GPIOMSP432E4_PE0      ((GPIOMSP432E4_PORTE << 8) | GPIO_PIN_0)
#define GPIOMSP432E4_PE1      ((GPIOMSP432E4_PORTE << 8) | GPIO_PIN_1)
#define GPIOMSP432E4_PE2      ((GPIOMSP432E4_PORTE << 8) | GPIO_PIN_2)
#define GPIOMSP432E4_PE3      ((GPIOMSP432E4_PORTE << 8) | GPIO_PIN_3)
#define GPIOMSP432E4_PE4      ((GPIOMSP432E4_PORTE << 8) | GPIO_PIN_4)
#define GPIOMSP432E4_PE5      ((GPIOMSP432E4_PORTE << 8) | GPIO_PIN_5)
#define GPIOMSP432E4_PE6      ((GPIOMSP432E4_PORTE << 8) | GPIO_PIN_6)
#define GPIOMSP432E4_PE7      ((GPIOMSP432E4_PORTE << 8) | GPIO_PIN_7)

#define GPIOMSP432E4_PF0      ((GPIOMSP432E4_PORTF << 8) | GPIO_PIN_0)
#define GPIOMSP432E4_PF1      ((GPIOMSP432E4_PORTF << 8) | GPIO_PIN_1)
#define GPIOMSP432E4_PF2      ((GPIOMSP432E4_PORTF << 8) | GPIO_PIN_2)
#define GPIOMSP432E4_PF3      ((GPIOMSP432E4_PORTF << 8) | GPIO_PIN_3)
#define GPIOMSP432E4_PF4      ((GPIOMSP432E4_PORTF << 8) | GPIO_PIN_4)
#define GPIOMSP432E4_PF5      ((GPIOMSP432E4_PORTF << 8) | GPIO_PIN_5)
#define GPIOMSP432E4_PF6      ((GPIOMSP432E4_PORTF << 8) | GPIO_PIN_6)
#define GPIOMSP432E4_PF7      ((GPIOMSP432E4_PORTF << 8) | GPIO_PIN_7)

#define GPIOMSP432E4_PG0      ((GPIOMSP432E4_PORTG << 8) | GPIO_PIN_0)
#define GPIOMSP432E4_PG1      ((GPIOMSP432E4_PORTG << 8) | GPIO_PIN_1)
#define GPIOMSP432E4_PG2      ((GPIOMSP432E4_PORTG << 8) | GPIO_PIN_2)
#define GPIOMSP432E4_PG3      ((GPIOMSP432E4_PORTG << 8) | GPIO_PIN_3)
#define GPIOMSP432E4_PG4      ((GPIOMSP432E4_PORTG << 8) | GPIO_PIN_4)
#define GPIOMSP432E4_PG5      ((GPIOMSP432E4_PORTG << 8) | GPIO_PIN_5)
#define GPIOMSP432E4_PG6      ((GPIOMSP432E4_PORTG << 8) | GPIO_PIN_6)
#define GPIOMSP432E4_PG7      ((GPIOMSP432E4_PORTG << 8) | GPIO_PIN_7)

#define GPIOMSP432E4_PH0      ((GPIOMSP432E4_PORTH << 8) | GPIO_PIN_0)
#define GPIOMSP432E4_PH1      ((GPIOMSP432E4_PORTH << 8) | GPIO_PIN_1)
#define GPIOMSP432E4_PH2      ((GPIOMSP432E4_PORTH << 8) | GPIO_PIN_2)
#define GPIOMSP432E4_PH3      ((GPIOMSP432E4_PORTH << 8) | GPIO_PIN_3)
#define GPIOMSP432E4_PH4      ((GPIOMSP432E4_PORTH << 8) | GPIO_PIN_4)
#define GPIOMSP432E4_PH5      ((GPIOMSP432E4_PORTH << 8) | GPIO_PIN_5)
#define GPIOMSP432E4_PH6      ((GPIOMSP432E4_PORTH << 8) | GPIO_PIN_6)
#define GPIOMSP432E4_PH7      ((GPIOMSP432E4_PORTH << 8) | GPIO_PIN_7)

#define GPIOMSP432E4_PJ0      ((GPIOMSP432E4_PORTJ << 8) | GPIO_PIN_0)
#define GPIOMSP432E4_PJ1      ((GPIOMSP432E4_PORTJ << 8) | GPIO_PIN_1)
#define GPIOMSP432E4_PJ2      ((GPIOMSP432E4_PORTJ << 8) | GPIO_PIN_2)
#define GPIOMSP432E4_PJ3      ((GPIOMSP432E4_PORTJ << 8) | GPIO_PIN_3)
#define GPIOMSP432E4_PJ4      ((GPIOMSP432E4_PORTJ << 8) | GPIO_PIN_4)
#define GPIOMSP432E4_PJ5      ((GPIOMSP432E4_PORTJ << 8) | GPIO_PIN_5)
#define GPIOMSP432E4_PJ6      ((GPIOMSP432E4_PORTJ << 8) | GPIO_PIN_6)
#define GPIOMSP432E4_PJ7      ((GPIOMSP432E4_PORTJ << 8) | GPIO_PIN_7)

#define GPIOMSP432E4_PK0      ((GPIOMSP432E4_PORTK << 8) | GPIO_PIN_0)
#define GPIOMSP432E4_PK1      ((GPIOMSP432E4_PORTK << 8) | GPIO_PIN_1)
#define GPIOMSP432E4_PK2      ((GPIOMSP432E4_PORTK << 8) | GPIO_PIN_2)
#define GPIOMSP432E4_PK3      ((GPIOMSP432E4_PORTK << 8) | GPIO_PIN_3)
#define GPIOMSP432E4_PK4      ((GPIOMSP432E4_PORTK << 8) | GPIO_PIN_4)
#define GPIOMSP432E4_PK5      ((GPIOMSP432E4_PORTK << 8) | GPIO_PIN_5)
#define GPIOMSP432E4_PK6      ((GPIOMSP432E4_PORTK << 8) | GPIO_PIN_6)
#define GPIOMSP432E4_PK7      ((GPIOMSP432E4_PORTK << 8) | GPIO_PIN_7)

#define GPIOMSP432E4_PL0      ((GPIOMSP432E4_PORTL << 8) | GPIO_PIN_0)
#define GPIOMSP432E4_PL1      ((GPIOMSP432E4_PORTL << 8) | GPIO_PIN_1)
#define GPIOMSP432E4_PL2      ((GPIOMSP432E4_PORTL << 8) | GPIO_PIN_2)
#define GPIOMSP432E4_PL3      ((GPIOMSP432E4_PORTL << 8) | GPIO_PIN_3)
#define GPIOMSP432E4_PL4      ((GPIOMSP432E4_PORTL << 8) | GPIO_PIN_4)
#define GPIOMSP432E4_PL5      ((GPIOMSP432E4_PORTL << 8) | GPIO_PIN_5)
#define GPIOMSP432E4_PL6      ((GPIOMSP432E4_PORTL << 8) | GPIO_PIN_6)
#define GPIOMSP432E4_PL7      ((GPIOMSP432E4_PORTL << 8) | GPIO_PIN_7)

#define GPIOMSP432E4_PM0      ((GPIOMSP432E4_PORTM << 8) | GPIO_PIN_0)
#define GPIOMSP432E4_PM1      ((GPIOMSP432E4_PORTM << 8) | GPIO_PIN_1)
#define GPIOMSP432E4_PM2      ((GPIOMSP432E4_PORTM << 8) | GPIO_PIN_2)
#define GPIOMSP432E4_PM3      ((GPIOMSP432E4_PORTM << 8) | GPIO_PIN_3)
#define GPIOMSP432E4_PM4      ((GPIOMSP432E4_PORTM << 8) | GPIO_PIN_4)
#define GPIOMSP432E4_PM5      ((GPIOMSP432E4_PORTM << 8) | GPIO_PIN_5)
#define GPIOMSP432E4_PM6      ((GPIOMSP432E4_PORTM << 8) | GPIO_PIN_6)
#define GPIOMSP432E4_PM7      ((GPIOMSP432E4_PORTM << 8) | GPIO_PIN_7)

#define GPIOMSP432E4_PN0      ((GPIOMSP432E4_PORTN << 8) | GPIO_PIN_0)
#define GPIOMSP432E4_PN1      ((GPIOMSP432E4_PORTN << 8) | GPIO_PIN_1)
#define GPIOMSP432E4_PN2      ((GPIOMSP432E4_PORTN << 8) | GPIO_PIN_2)
#define GPIOMSP432E4_PN3      ((GPIOMSP432E4_PORTN << 8) | GPIO_PIN_3)
#define GPIOMSP432E4_PN4      ((GPIOMSP432E4_PORTN << 8) | GPIO_PIN_4)
#define GPIOMSP432E4_PN5      ((GPIOMSP432E4_PORTN << 8) | GPIO_PIN_5)
#define GPIOMSP432E4_PN6      ((GPIOMSP432E4_PORTN << 8) | GPIO_PIN_6)
#define GPIOMSP432E4_PN7      ((GPIOMSP432E4_PORTN << 8) | GPIO_PIN_7)

#define GPIOMSP432E4_PP0      ((GPIOMSP432E4_PORTP << 8) | GPIO_PIN_0)
#define GPIOMSP432E4_PP1      ((GPIOMSP432E4_PORTP << 8) | GPIO_PIN_1)
#define GPIOMSP432E4_PP2      ((GPIOMSP432E4_PORTP << 8) | GPIO_PIN_2)
#define GPIOMSP432E4_PP3      ((GPIOMSP432E4_PORTP << 8) | GPIO_PIN_3)
#define GPIOMSP432E4_PP4      ((GPIOMSP432E4_PORTP << 8) | GPIO_PIN_4)
#define GPIOMSP432E4_PP5      ((GPIOMSP432E4_PORTP << 8) | GPIO_PIN_5)
#define GPIOMSP432E4_PP6      ((GPIOMSP432E4_PORTP << 8) | GPIO_PIN_6)
#define GPIOMSP432E4_PP7      ((GPIOMSP432E4_PORTP << 8) | GPIO_PIN_7)

#define GPIOMSP432E4_PQ0      ((GPIOMSP432E4_PORTQ << 8) | GPIO_PIN_0)
#define GPIOMSP432E4_PQ1      ((GPIOMSP432E4_PORTQ << 8) | GPIO_PIN_1)
#define GPIOMSP432E4_PQ2      ((GPIOMSP432E4_PORTQ << 8) | GPIO_PIN_2)
#define GPIOMSP432E4_PQ3      ((GPIOMSP432E4_PORTQ << 8) | GPIO_PIN_3)
#define GPIOMSP432E4_PQ4      ((GPIOMSP432E4_PORTQ << 8) | GPIO_PIN_4)
#define GPIOMSP432E4_PQ5      ((GPIOMSP432E4_PORTQ << 8) | GPIO_PIN_5)
#define GPIOMSP432E4_PQ6      ((GPIOMSP432E4_PORTQ << 8) | GPIO_PIN_6)
#define GPIOMSP432E4_PQ7      ((GPIOMSP432E4_PORTQ << 8) | GPIO_PIN_7)

#define GPIOMSP432E4_PR0      ((GPIOMSP432E4_PORTR << 8) | GPIO_PIN_0)
#define GPIOMSP432E4_PR1      ((GPIOMSP432E4_PORTR << 8) | GPIO_PIN_1)
#define GPIOMSP432E4_PR2      ((GPIOMSP432E4_PORTR << 8) | GPIO_PIN_2)
#define GPIOMSP432E4_PR3      ((GPIOMSP432E4_PORTR << 8) | GPIO_PIN_3)
#define GPIOMSP432E4_PR4      ((GPIOMSP432E4_PORTR << 8) | GPIO_PIN_4)
#define GPIOMSP432E4_PR5      ((GPIOMSP432E4_PORTR << 8) | GPIO_PIN_5)
#define GPIOMSP432E4_PR6      ((GPIOMSP432E4_PORTR << 8) | GPIO_PIN_6)
#define GPIOMSP432E4_PR7      ((GPIOMSP432E4_PORTR << 8) | GPIO_PIN_7)

#define GPIOMSP432E4_PS0      ((GPIOMSP432E4_PORTS << 8) | GPIO_PIN_0)
#define GPIOMSP432E4_PS1      ((GPIOMSP432E4_PORTS << 8) | GPIO_PIN_1)
#define GPIOMSP432E4_PS2      ((GPIOMSP432E4_PORTS << 8) | GPIO_PIN_2)
#define GPIOMSP432E4_PS3      ((GPIOMSP432E4_PORTS << 8) | GPIO_PIN_3)
#define GPIOMSP432E4_PS4      ((GPIOMSP432E4_PORTS << 8) | GPIO_PIN_4)
#define GPIOMSP432E4_PS5      ((GPIOMSP432E4_PORTS << 8) | GPIO_PIN_5)
#define GPIOMSP432E4_PS6      ((GPIOMSP432E4_PORTS << 8) | GPIO_PIN_6)
#define GPIOMSP432E4_PS7      ((GPIOMSP432E4_PORTS << 8) | GPIO_PIN_7)

#define GPIOMSP432E4_PT0      ((GPIOMSP432E4_PORTT << 8) | GPIO_PIN_0)
#define GPIOMSP432E4_PT1      ((GPIOMSP432E4_PORTT << 8) | GPIO_PIN_1)
#define GPIOMSP432E4_PT2      ((GPIOMSP432E4_PORTT << 8) | GPIO_PIN_2)
#define GPIOMSP432E4_PT3      ((GPIOMSP432E4_PORTT << 8) | GPIO_PIN_3)

/*
 *  GPIO_Pxx_xxxx defines (driverlib/pin_map.h) don't use the upper 12
 *  bits of the mask.  We'll encode the port and pin ids in the top 8 bits
 *  and the GPIO_Pxx_xxxxx in the lower 24 bits of the following masks.
 */
#define GPIOMSP432E4_pinConfigMask(port, pin, mapping) \
    (((port) << 27) | ((pin) << 24) | ((mapping) & 0x00FFFFFF))

/*
 *  Macros to decode the pinConfigMask from above.
 */
#define GPIOMSP432E4_getPortFromPinConfig(config) (((config) >> 27) & 0x1F)
#define GPIOMSP432E4_getPinFromPinConfig(config) (1 << (((config) >> 24) & 0x7))
#define GPIOMSP432E4_getPinMapFromPinConfig(config) ((config) & 0xFFFFFF)

/*!
 *  @brief      Get the base address for a given port Id.
 *
 *  @param      port        GPIO port id.  Must be one of GPIOMSP432E4_PORTA
 *                          through GPIOMSP432E4_PORTT.
 *
 *  @return     The base address of the GPIO port.
 */
uint32_t GPIOMSP432E4_getGpioBaseAddr(uint8_t port);

/*!
 *  @brief      Get the Power resource Id for a given port Id.
 *
 *  @param      port        GPIO port id.  Must be one of GPIOMSP432E4_PORTA
 *                          through GPIOMSP432E4_PORTT.
 *
 *  @return     The resource Id that can be passed to Power_setDependency().
 */
uint_fast16_t GPIOMSP432E4_getPowerResourceId(uint8_t port);

/*!
 *  @brief      Undo pin configuration
 *
 *  @param      pinConfig   pin configuration from GPIOMSP432E4_pinConfigMask
 */
void GPIOMSP432E4_undoPinConfig(uint32_t pinConfig);

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_GPIOMSP432E4__include */
