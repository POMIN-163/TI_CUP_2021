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
/** ============================================================================
 *  @file       CANMSP432E4.h
 *
 *  @brief      CAN driver implementation for a MSP432E4 CAN controller
 *
 *  The CAN header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/CAN.h>
 *  #include <ti/drivers/can/CANMSP432E4.h>
 *  @endcode
 *
 *  # Stack requirements #
 *  The CANMSP432E4 driver is a (ring) buffered driver that stores data it may
 *  have already received in a user-supplied background buffer.
 *  ============================================================================
 */

#ifndef ti_drivers_can_CANMSP432E4__include
#define ti_drivers_can_CANMSP432E4__include

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <ti/devices/msp432e4/inc/msp432.h>

#include <ti/devices/msp432e4/driverlib/gpio.h>
#include <ti/devices/msp432e4/driverlib/pin_map.h>

#include <ti/drivers/dpl/ClockP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>

#include <ti/drivers/gpio/GPIOMSP432E4.h>
#include <ti/drivers/CAN.h>
#include <ti/drivers/utils/List.h>
#include <ti/drivers/utils/StructRingBuf.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief PA0 is used for CAN0 RX
 */
#define CANMSP432E4_PA0_CAN0RX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 0, GPIO_PA0_CAN0RX)

/*!
 * @brief PA1 is used for CAN0 RX
 */
#define CANMSP432E4_PA1_CAN0TX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 1, GPIO_PA1_CAN0TX)

/*!
 * @brief PB0 is used for CAN1 RX
 */
#define CANMSP432E4_PB0_CAN1RX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTB, 0, GPIO_PB0_CAN1RX)

/*!
 * @brief PB1 is used for CAN1 RX
 */
#define CANMSP432E4_PB1_CAN1TX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTB, 1, GPIO_PB1_CAN1TX)

/* CAN function table pointer */
extern const CAN_FxnTable CANMSP432E4_fxnTable;

/*!
 *  @brief      The definition of an optional callback function used by the CAN
 *              driver to notify the application when a receive error (FIFO overrun,
 *              parity error, etc) occurs.
 *
 *  @param      CAN_Handle             CAN_Handle
 *
 *  @param      error                   The current value of the receive
 *                                      status register.  Please refer to the
 *                                      device data sheet to interpret this
 *                                      value.
 */
typedef void (*CANMSP432E4_ErrorCallback) (CAN_Handle handle,
        uint32_t error);

/*!
 *  @brief      CANMSP432E4 Hardware attributes
 *
 *  The baseAddr, intNum, and flowControl fields
 *  are used by driverlib APIs and therefore must be populated by
 *  driverlib macro definitions. For MSP432E4 driverlib these definitions are
 *  found in:
 *      - ti/devices/msp432e4/driverlib/inc/msp432e4xxx.h
 *      - ti/devices/msp432e4/driverlib/interrupt.h
 *      - ti/devices/msp432e4/driverlib/can.h
 *
 *  intPriority is the CAN peripheral's interrupt priority, as defined by the
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
 *  struct can_frame canMSP432E4TxBuffer[2][4];
 *  struct can_frame canMSP432E4RxBuffer[2][4];
 *
 *  const CANMSP432E4_HWAttrs canMSP432E4HWAttrs[] = {
 *      {
 *          .baseAddr = CAN0_BASE,
 *          .intNum = INT_CAN0,
 *          .intPriority = (~0),
 *          .rxPin = CANMSP432E4_PA0_U0RX,
 *          .txPin = CANMSP432E4_PA1_U0TX,
 *          .baudRate = 125000;
 *          .errorFxn = NULL
 *      },
 *      {
 *          .baseAddr = CAN1_BASE,
 *          .intNum = INT_CAN1,
 *          .intPriority = (~0),
 *          .rxPin = CANMSP432E4_PB0_U1RX,
 *          .txPin = CANMSP432E4_PB1_U1TX,
 *          .baudRate = 125000;
 *          .errorFxn = NULL,
 *      }
 *  };
 *  @endcode
 */
typedef struct {
    /*! CAN Peripheral's base address */
    uint32_t        baseAddr;
    /*! CAN Peripheral's interrupt vector */
    uint32_t        intNum;
    /*! CAN Peripheral's interrupt priority */
    uint32_t        intPriority;
    /*! CAN RX pin assignment */
    uint32_t        rxPin;
    /*! CAN TX pin assignment */
    uint32_t        txPin;
    /*! Baud rate for CAN */
    uint32_t        baudRate;
    /*! Application error function to be called on receive errors */
    CANMSP432E4_ErrorCallback errorFxn;
} CANMSP432E4_HWAttrs;

/*!
 *  @brief      CANMSP432E4 Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct {
    List_Elem            elem;          /*!< list element of hardware binding */

    StructRingBuf_Object txBuffer;      /*!< transmit object ring buffer */
    StructRingBuf_Object rxBuffer;      /*!< receive object ring buffer */

    SemaphoreP_Handle    readSem;       /*!< read Semaphore */
    SemaphoreP_Handle    writeSem;      /*!< write Semaphore */

    CAN_Mode             mode;          /*!< blocking or nonblocking mode */
    CAN_Direction        direction;     /*!< read/write mode */
    uint32_t             writeTimeout;  /*!< blocking mode write timeout */
    uint32_t             readTimeout;   /*!< blocking mode read timeout */

    uint32_t             rxInUseMask;   /*!< receive message objects in use */
    uint32_t             txInUseMask;   /*!< transmit message objects in use */

    uint32_t             overrunCount;  /*!< receive overrun count statistic */

    bool                 opened    : 1; /*!< Has the obj been opened */
    bool                 txPending : 1; /*!< transmission is in progress */
} CANMSP432E4_Object, *CANMSP432E4_Handle;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_can_CANMSP432E4__include */
