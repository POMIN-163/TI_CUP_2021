/*
 * Copyright (c) 2017-2020, Texas Instruments Incorporated
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
/*!****************************************************************************
 *  @file       I2CMSP432E4.h
 *
 *  @brief      I2C driver implementation for a MSP432E4 I2C controller.
 *
 *  The I2C header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/I2C.h>
 *  #include <ti/drivers/i2c/I2CMSP432E4.h>
 *  @endcode
 *
 *  Refer to @ref I2C.h for a complete description of APIs & example of use.
 *
 *  ## Supported Bit Rates ##
 *    - #I2C_100kHz
 *    - #I2C_400kHz
 *    - #I2C_1000kHz
 *    - #I2C_3330kHz
 *
 *  ## Using High Speed Mode (#I2C_3330kHz) ##
 *  When operating at #I2C_3330kHz, the #I2CMSP432E4_HWAttrs.masterCode must
 *  be provided. If the master code is not provided, I2C_open() will fail.
 *  This driver assumes that the system clock is 120MHz. If the system clock
 *  is not 120MHz, I2C_open() will fail.
 *
 ******************************************************************************
 */

#ifndef ti_drivers_i2c_I2CMSP432E4__include
#define ti_drivers_i2c_I2CMSP432E4__include

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <ti/devices/msp432e4/inc/msp432.h>

#include <ti/devices/msp432e4/driverlib/gpio.h>
#include <ti/devices/msp432e4/driverlib/pin_map.h>

#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/gpio/GPIOMSP432E4.h>
#include <ti/drivers/I2C.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  @addtogroup I2C_STATUS
 *  I2CMSP432E4_STATUS_* macros are command codes only defined in the
 *  I2CMSP432E4.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/i2c/I2CMSP432E4.h>
 *  @endcode
 *  @{
 */

/* Add I2CMSP432E4_STATUS_* macros here */

/** @}*/

/**
 *  @addtogroup I2C_CMD
 *  I2CMSP432E4_CMD_* macros are command codes only defined in the
 *  I2CMSP432E4.h driver implementation and need to:
 *  @code
 *  #include <ti/drivers/i2c/I2CMSP432E4.h>
 *  @endcode
 *  @{
 */

/* Add I2CMSP432E4_CMD_* macros here */

/** @}*/

/*!
 * @brief PB2 is used for I2C0SCL
 */
#define I2CMSP432E4_PB2_I2C0SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTB, 2, GPIO_PB2_I2C0SCL)

/*!
 * @brief PB3 is used for I2C0SDA
 */
#define I2CMSP432E4_PB3_I2C0SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTB, 3, GPIO_PB3_I2C0SDA)

/*!
 * @brief PG0 is used for I2C1SCL
 */
#define I2CMSP432E4_PG0_I2C1SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTG, 0, GPIO_PG0_I2C1SCL)

/*!
 * @brief PR0 is used for I2C1SCL
 */
#define I2CMSP432E4_PR0_I2C1SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTR, 0, GPIO_PR0_I2C1SCL)

/*!
 * @brief PG1 is used for I2C1SDA
 */
#define I2CMSP432E4_PG1_I2C1SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTG, 1, GPIO_PG1_I2C1SDA)

/*!
 * @brief PR1 is used for I2C1SDA
 */
#define I2CMSP432E4_PR1_I2C1SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTR, 1, GPIO_PR1_I2C1SDA)

/*!
 * @brief PL1 is used for I2C2SCL
 */
#define I2CMSP432E4_PL1_I2C2SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTL, 1, GPIO_PL1_I2C2SCL)

/*!
 * @brief PP5 is used for I2C2SCL
 */
#define I2CMSP432E4_PP5_I2C2SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTP, 5, GPIO_PP5_I2C2SCL)

/*!
 * @brief PN5 is used for I2C2SCL
 */
#define I2CMSP432E4_PN5_I2C2SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTN, 5, GPIO_PN5_I2C2SCL)

/*!
 * @brief PG2 is used for I2C2SCL
 */
#define I2CMSP432E4_PG2_I2C2SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTG, 2, GPIO_PG2_I2C2SCL)

/*!
 * @brief PR2 is used for I2C2SCL
 */
#define I2CMSP432E4_PR2_I2C2SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTR, 2, GPIO_PR2_I2C2SCL)

/*!
 * @brief PL0 is used for I2C2SDA
 */
#define I2CMSP432E4_PL0_I2C2SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTL, 0, GPIO_PL0_I2C2SDA)

/*!
 * @brief PN4 is used for I2C2SDA
 */
#define I2CMSP432E4_PN4_I2C2SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTN, 4, GPIO_PN4_I2C2SDA)

/*!
 * @brief PP6 is used for I2C2SDA
 */
#define I2CMSP432E4_PP6_I2C2SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTP, 6, GPIO_PP6_I2C2SDA)

/*!
 * @brief PG3 is used for I2C2SDA
 */
#define I2CMSP432E4_PG3_I2C2SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTG, 3, GPIO_PG3_I2C2SDA)

/*!
 * @brief PR3 is used for I2C2SDA
 */
#define I2CMSP432E4_PR3_I2C2SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTR, 3, GPIO_PR3_I2C2SDA)

/*!
 * @brief PK4 is used for I2C3SCL
 */
#define I2CMSP432E4_PK4_I2C3SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTK, 4, GPIO_PK4_I2C3SCL)

/*!
 * @brief PG4 is used for I2C3SCL
 */
#define I2CMSP432E4_PG4_I2C3SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTG, 4, GPIO_PG4_I2C3SCL)

/*!
 * @brief PR4 is used for I2C3SCL
 */
#define I2CMSP432E4_PR4_I2C3SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTR, 4, GPIO_PR4_I2C3SCL)

/*!
 * @brief PK5 is used for I2C3SDA
 */
#define I2CMSP432E4_PK5_I2C3SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTK, 5, GPIO_PK5_I2C3SDA)

/*!
 * @brief PG5 is used for I2C3SDA
 */
#define I2CMSP432E4_PG5_I2C3SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTG, 5, GPIO_PG5_I2C3SDA)

/*!
 * @brief PR5 is used for I2C3SDA
 */
#define I2CMSP432E4_PR5_I2C3SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTR, 5, GPIO_PR5_I2C3SDA)

/*!
 * @brief PK6 is used for I2C4SCL
 */
#define I2CMSP432E4_PK6_I2C4SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTK, 6, GPIO_PK6_I2C4SCL)

/*!
 * @brief PG6 is used for I2C4SCL
 */
#define I2CMSP432E4_PG6_I2C4SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTG, 6, GPIO_PG6_I2C4SCL)

/*!
 * @brief PR6 is used for I2C4SCL
 */
#define I2CMSP432E4_PR6_I2C4SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTR, 6, GPIO_PR6_I2C4SCL)

/*!
 * @brief PK7 is used for I2C4SDA
 */
#define I2CMSP432E4_PK7_I2C4SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTK, 7, GPIO_PK7_I2C4SDA)

/*!
 * @brief PG7 is used for I2C4SDA
 */
#define I2CMSP432E4_PG7_I2C4SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTG, 7, GPIO_PG7_I2C4SDA)

/*!
 * @brief PR7 is used for I2C4SDA
 */
#define I2CMSP432E4_PR7_I2C4SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTR, 7, GPIO_PR7_I2C4SDA)

/*!
 * @brief PB0 is used for I2C5SCL
 */
#define I2CMSP432E4_PB0_I2C5SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTB, 0, GPIO_PB0_I2C5SCL)

/*!
 * @brief PB4 is used for I2C5SCL
 */
#define I2CMSP432E4_PB4_I2C5SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTB, 4, GPIO_PB4_I2C5SCL)

/*!
 * @brief PB1 is used for I2C5SDA
 */
#define I2CMSP432E4_PB1_I2C5SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTB, 1, GPIO_PB1_I2C5SDA)

/*!
 * @brief PB5 is used for I2C5SDA
 */
#define I2CMSP432E4_PB5_I2C5SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTB, 5, GPIO_PB5_I2C5SDA)

/*!
 * @brief PA6 is used for I2C6SCL
 */
#define I2CMSP432E4_PA6_I2C6SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 6, GPIO_PA6_I2C6SCL)

/*!
 * @brief PB6 is used for I2C6SCL
 */
#define I2CMSP432E4_PB6_I2C6SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTB, 6, GPIO_PB6_I2C6SCL)

/*!
 * @brief PA7 is used for I2C6SDA
 */
#define I2CMSP432E4_PA7_I2C6SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 7, GPIO_PA7_I2C6SDA)

/*!
 * @brief PB7 is used for I2C6SDA
 */
#define I2CMSP432E4_PB7_I2C6SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTB, 7, GPIO_PB7_I2C6SDA)

/*!
 * @brief PD0 is used for I2C7SCL
 */
#define I2CMSP432E4_PD0_I2C7SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTD, 0, GPIO_PD0_I2C7SCL)

/*!
 * @brief PA4 is used for I2C7SCL
 */
#define I2CMSP432E4_PA4_I2C7SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 4, GPIO_PA4_I2C7SCL)

/*!
 * @brief PD1 is used for I2C7SDA
 */
#define I2CMSP432E4_PD1_I2C7SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTD, 1, GPIO_PD1_I2C7SDA)

/*!
 * @brief PA5 is used for I2C7SDA
 */
#define I2CMSP432E4_PA5_I2C7SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 5, GPIO_PA5_I2C7SDA)

/*!
 * @brief PD2 is used for I2C8SCL
 */
#define I2CMSP432E4_PD2_I2C8SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTD, 2, GPIO_PD2_I2C8SCL)

/*!
 * @brief PA2 is used for I2C8SCL
 */
#define I2CMSP432E4_PA2_I2C8SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 2, GPIO_PA2_I2C8SCL)

/*!
 * @brief PD3 is used for I2C8SDA
 */
#define I2CMSP432E4_PD3_I2C8SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTD, 3, GPIO_PD3_I2C8SDA)

/*!
 * @brief PA3 is used for I2C8SDA
 */
#define I2CMSP432E4_PA3_I2C8SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 3, GPIO_PA3_I2C8SDA)

/*!
 * @brief PA0 is used for I2C9SCL
 */
#define I2CMSP432E4_PA0_I2C9SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 0, GPIO_PA0_I2C9SCL)

/*!
 * @brief PE6 is used for I2C9SCL
 */
#define I2CMSP432E4_PE6_I2C9SCL GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTE, 6, GPIO_PE6_I2C9SCL)

/*!
 * @brief PA1 is used for I2C9SDA
 */
#define I2CMSP432E4_PA1_I2C9SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 1, GPIO_PA1_I2C9SDA)

/*!
 * @brief PE7 is used for I2C9SDA
 */
#define I2CMSP432E4_PE7_I2C9SDA GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTE, 7, GPIO_PE7_I2C9SDA)


/* I2C function table pointer */
extern const I2C_FxnTable I2CMSP432E4_fxnTable;

/*!
 *  @brief  I2CMSP432E4 Hardware attributes
 *
 *  The baseAddr and intNum fields are used by driverlib APIs and therefore must
 *  be populated by driverlib macro definitions. For MSP432E4 driverlib these
 *  definitions are found in:
 *      - ti/devices/msp432e4/inc/msp432e4x1y.h
 *
 *  intPriority is the I2C peripheral's interrupt priority, as defined by the
 *  underlying OS.  It is passed unmodified to the underlying OS's interrupt
 *  handler creation code, so you need to refer to the OS documentation
 *  for usage.  For example, for SYS/BIOS applications, refer to the
 *  ti.sysbios.family.arm.m3.Hwi documentation for SYS/BIOS usage of
 *  interrupt priorities.  If the driver uses the ti.dpl interface
 *  instead of making OS calls directly, then the HwiP port handles the
 *  interrupt priority in an OS specific way.  In the case of the SYS/BIOS
 *  port, intPriority is passed unmodified to Hwi_create().
 *
 *  A sample structure is shown below:
 *  @code
 *  const I2CMSP432E4_HWAttrs i2cMSP432E4HWAttrs[] = {
 *      {
 *          .baseAddr = I2C7_BASE,
 *          .intNum = INT_I2C7,
 *          .intPriority = (~0),
 *          .sclPin = I2CMSP432E4_PD0_I2C7SCL,
 *          .sdaPin = I2CMSP432E4_PD1_I2C7SDA,
 *          .masterCode = 0x08
 *      },
 *      {
 *          .baseAddr = I2C8_BASE,
 *          .intNum = INT_I2C8,
 *          .intPriority = (~0),
 *          .sclPin = I2CMSP432E4_PA2_I2C8SCL,
 *          .sdaPin = I2CMSP432E4_PA3_I2C8SDA,
 *          .masterCode = 0x09
 *      },
 *  };
 *  @endcode
 */
typedef struct {
    /*! I2C Peripheral's base address */
    uint32_t baseAddr;
    /*! I2C Peripheral's interrupt vector */
    uint32_t intNum;
    /*! I2C Peripheral's interrupt priority */
    uint32_t intPriority;
    /*! I2C Serial Clock Line pin configuration */
    uint32_t sclPin;
    /*! I2C Serial Data Line pin configuration */
    uint32_t sdaPin;
    /*!
     *  I2C Master Code. The master code can be between 08h to 0Fh. Each master
     *  on a shared bus should use a unique master code. The master code is
     *  only used when operating at a bit rate of #I2C_3330kHz. Therefore, this
     *  parameter is ignored when operating at other bit rates.
     */
    uint8_t  masterCode;
} I2CMSP432E4_HWAttrs;

/*!
 *  @brief  I2CMSP432E4 Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct {
    /* Grants exclusive access to I2C */
    SemaphoreP_Handle mutex;

    /* Notify finished I2C transfer */
    SemaphoreP_Handle transferComplete;

    /* Hardware interrupt handle */
    HwiP_Handle hwiHandle;

    /* Blocking or Callback mode */
    I2C_TransferMode transferMode;

    /* Application callback function pointer */
    I2C_CallbackFxn transferCallbackFxn;

    /* Pointer to current I2C transaction */
    I2C_Transaction *currentTransaction;

    /* Head and tail pointers for queued transactions in I2C_MODE_CALLBACK */
    I2C_Transaction * volatile headPtr;
    I2C_Transaction *tailPtr;

    /* Pointers to transaction buffers */
    const uint8_t * writeBuf;
    uint8_t *readBuf;

    /* Read, write, and burst counters */
    size_t writeCount;
    size_t readCount;
    uint16_t burstCount;
    int_fast16_t status;
    bool burstStarted;

    /* Enumerated bit rate */
    I2C_BitRate bitRate;

    bool      isOpen;
} I2CMSP432E4_Object;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_i2c_I2CMSP432E4__include */
