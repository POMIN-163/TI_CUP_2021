/*
 * Copyright (c) 2020, Texas Instruments Incorporated
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
 *  @file       UART2MSP432E4.h
 *
 *  @brief      UART driver implementation for a MSP432E4 UART controller
 *
 *  The UART header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/UART2.h>
 *  #include <ti/drivers/uart2/UART2MSP432E4.h>
 *  @endcode
 *
 *  Refer to @ref UART2.h for a complete description of APIs and examples
 *  of use.
 *
 *  ============================================================================
 */

#ifndef ti_drivers_uart2_UART2MSP432E4__include
#define ti_drivers_uart2_UART2MSP432E4__include

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <ti/devices/msp432e4/inc/msp432.h>

#include <ti/devices/msp432e4/driverlib/gpio.h>
#include <ti/devices/msp432e4/driverlib/pin_map.h>
#include <ti/devices/msp432e4/driverlib/udma.h>

#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/gpio/GPIOMSP432E4.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/dma/UDMAMSP432E4.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Indicates a pin is not being used
 *
 *  If hardware flow control is not being used, the UART CTS and RTS
 *  pins should be set to UART2MSP432E4_PIN_UNASSIGNED.
 */
#define UART2MSP432E4_PIN_UNASSIGNED   0xFFFFFFFF

/*!
 * @brief Indicates a DMA channel is not being used
 *
 *  If DMA is not being used, the UART rxDmaChannel and txDmaChannel
 *  should be set to UART2MSP432E4_DMACH_UNASSIGNED.
 */
#define UART2MSP432E4_DMACH_UNASSIGNED 0xFF

/*!
 * @brief No hardware flow control
 */
#define UART2MSP432E4_FLOWCTRL_NONE 0

/*!
 * @brief Hardware flow control
 */
#define UART2MSP432E4_FLOWCTRL_HARDWARE 1

/*!
 * @brief PA0 is used for UART0 RX
 */
#define UART2MSP432E4_PA0_U0RX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 0, GPIO_PA0_U0RX)

/*!
 * @brief PA1 is used for UART0 RX
 */
#define UART2MSP432E4_PA1_U0TX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 1, GPIO_PA1_U0TX)

/*!
 * @brief PH1 is used for UART0 CTS
 */
#define UART2MSP432E4_PH1_U0CTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTH, 1, GPIO_PH1_U0CTS)

/*!
 * @brief PM4 is used for UART0 CTS
 */
#define UART2MSP432E4_PM4_U0CTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTM, 4, GPIO_PM4_U0CTS)

/*!
 * @brief PB4 is used for UART0 CTS
 */
#define UART2MSP432E4_PB4_U0CTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTB, 4, GPIO_PB4_U0CTS)

/*!
 * @brief PE6 is used for UART0 CTS
 */
#define UART2MSP432E4_PE6_U0CTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTE, 6, GPIO_PE6_U0CTS)

/*!
 * @brief PG4 is used for UART0 CTS
 */
#define UART2MSP432E4_PG4_U0CTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTG, 4, GPIO_PG4_U0CTS)

/*!
 * @brief PH0 is used for UART0 RTS
 */
#define UART2MSP432E4_PH0_U0RTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTH, 0, GPIO_PH0_U0RTS)

/*!
 * @brief PB5 is used for UART0 RTS
 */
#define UART2MSP432E4_PB5_U0RTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTB, 5, GPIO_PB5_U0RTS)

/*!
 * @brief PE7 is used for UART0 RTS
 */
#define UART2MSP432E4_PE7_U0RTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTE, 7, GPIO_PE7_U0RTS)

/*!
 * @brief PG5 is used for UART0 RTS
 */
#define UART2MSP432E4_PG5_U0RTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTG, 5, GPIO_PG5_U0RTS)


/*!
 * @brief PB0 is used for UART1 RX
 */
#define UART2MSP432E4_PB0_U1RX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTB, 0, GPIO_PB0_U1RX)

/*!
 * @brief PQ4 is used for UART1 RX
 */
#define UART2MSP432E4_PQ4_U1RX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTQ, 4, GPIO_PQ4_U1RX)

/*!
 * @brief PR5 is used for UART1 RX
 */
#define UART2MSP432E4_PR5_U1RX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTR, 5, GPIO_PR5_U1RX)

/*!
 * @brief PB1 is used for UART1 TX
 */
#define UART2MSP432E4_PB1_U1TX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTB, 1, GPIO_PB1_U1TX)

/*!
 * @brief PQ5 is used for UART1 TX
 */
#define UART2MSP432E4_PQ5_U1TX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTQ, 5, GPIO_PQ5_U1TX)

/*!
 * @brief PR6 is used for UART1 TX
 */
#define UART2MSP432E4_PR6_U1TX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTR, 6, GPIO_PR6_U1TX)

/*!
 * @brief PP3 is used for UART1 CTS
 */
#define UART2MSP432E4_PP3_U1CTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTP, 3, GPIO_PP3_U1CTS)

/*!
 * @brief PN1 is used for UART1 CTS
 */
#define UART2MSP432E4_PN1_U1CTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTN, 1, GPIO_PN1_U1CTS)

/*!
 * @brief PE0 is used for UART1 RTS
 */
#define UART2MSP432E4_PE0_U1RTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTE, 0, GPIO_PE0_U1RTS)

/*!
 * @brief PN0 is used for UART1 RTS
 */
#define UART2MSP432E4_PN0_U1RTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTN, 0, GPIO_PN0_U1RTS)

/*!
 * @brief PN7 is used for UART1 RTS
 */
#define UART2MSP432E4_PN7_U1RTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTN, 7, GPIO_PN7_U1RTS)


/*!
 * @brief PA6 is used for UART2 RX
 */
#define UART2MSP432E4_PA6_U2RX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 6, GPIO_PA6_U2RX)

/*!
 * @brief PD4 is used for UART2 RX
 */
#define UART2MSP432E4_PD4_U2RX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTD, 4, GPIO_PD4_U2RX)

/*!
 * @brief PA7 is used for UART2 TX
 */
#define UART2MSP432E4_PA7_U2TX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 7, GPIO_PA7_U2TX)

/*!
 * @brief PD5 is used for UART2 TX
 */
#define UART2MSP432E4_PD5_U2TX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTD, 5, GPIO_PD5_U2TX)

/*!
 * @brief PN3 is used for UART2 CTS
 */
#define UART2MSP432E4_PN3_U2CTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTN, 3, GPIO_PN3_U2CTS)

/*!
 * @brief PD7 is used for UART2 CTS
 */
#define UART2MSP432E4_PD7_U2CTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTD, 7, GPIO_PD7_U2CTS)

/*!
 * @brief PJ3 is used for UART2 CTS
 */
#define UART2MSP432E4_PJ3_U2CTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTJ, 3, GPIO_PJ3_U2CTS)

/*!
 * @brief PN2 is used for UART2 RTS
 */
#define UART2MSP432E4_PN2_U2RTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTN, 2, GPIO_PN2_U2RTS)

/*!
 * @brief PD6 is used for UART2 RTS
 */
#define UART2MSP432E4_PD6_U2RTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTD, 6, GPIO_PD6_U2RTS)

/*!
 * @brief PJ2 is used for UART2 RTS
 */
#define UART2MSP432E4_PJ2_U2RTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTJ, 2, GPIO_PJ2_U2RTS)


/*!
 * @brief PA4 is used for UART3 RX
 */
#define UART2MSP432E4_PA4_U3RX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 4, GPIO_PA4_U3RX)

/*!
 * @brief PJ0 is used for UART3 RX
 */
#define UART2MSP432E4_PJ0_U3RX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTJ, 0, GPIO_PJ0_U3RX)

/*!
 * @brief PA5 is used for UART3 TX
 */
#define UART2MSP432E4_PA5_U3TX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 5, GPIO_PA5_U3TX)

/*!
 * @brief PJ1 is used for UART3 TX
 */
#define UART2MSP432E4_PJ1_U3TX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTJ, 1, GPIO_PJ1_U3TX)

/*!
 * @brief PP5 is used for UART3 CTS
 */
#define UART2MSP432E4_PP5_U3CTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTP, 5, GPIO_PP5_U3CTS)

/*!
 * @brief PN5 is used for UART3 CTS
 */
#define UART2MSP432E4_PN5_U3CTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTN, 5, GPIO_PN5_U3CTS)

/*!
 * @brief PJ5 is used for UART3 CTS
 */
#define UART2MSP432E4_PJ5_U3CTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTJ, 5, GPIO_PJ5_U3CTS)

/*!
 * @brief PP4 is used for UART3 RTS
 */
#define UART2MSP432E4_PP4_U3RTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTP, 4, GPIO_PP4_U3RTS)

/*!
 * @brief PN4 is used for UART3 RTS
 */
#define UART2MSP432E4_PN4_U3RTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTN, 4, GPIO_PN4_U3RTS)

/*!
 * @brief PJ4 is used for UART3 RTS
 */
#define UART2MSP432E4_PJ4_U3RTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTJ, 4, GPIO_PJ4_U3RTS)


/*!
 * @brief PA2 is used for UART4 RX
 */
#define UART2MSP432E4_PA2_U4RX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 2, GPIO_PA2_U4RX)

/*!
 * @brief PK0 is used for UART4 RX
 */
#define UART2MSP432E4_PK0_U4RX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTK, 0, GPIO_PK0_U4RX)

/*!
 * @brief PR1 is used for UART4 RX
 */
#define UART2MSP432E4_PR1_U4RX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTR, 1, GPIO_PR1_U4RX)

/*!
 * @brief PA3 is used for UART4 TX
 */
#define UART2MSP432E4_PA3_U4TX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTA, 3, GPIO_PA3_U4TX)

/*!
 * @brief PK1 is used for UART4 TX
 */
#define UART2MSP432E4_PK1_U4TX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTK, 1, GPIO_PK1_U4TX)

/*!
 * @brief PR0 is used for UART4 TX
 */
#define UART2MSP432E4_PR0_U4TX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTR, 0, GPIO_PR0_U4TX)

/*!
 * @brief PK3 is used for UART4 CTS
 */
#define UART2MSP432E4_PK3_U4CTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTK, 3, GPIO_PK3_U4CTS)

/*!
 * @brief PJ7 is used for UART4 CTS
 */
#define UART2MSP432E4_PJ7_U4CTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTJ, 7, GPIO_PJ7_U4CTS)

/*!
 * @brief PN7 is used for UART4 CTS
 */
#define UART2MSP432E4_PN7_U4CTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTN, 7, GPIO_PN7_U4CTS)

/*!
 * @brief PK2 is used for UART4 RTS
 */
#define UART2MSP432E4_PK2_U4RTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTK, 2, GPIO_PK2_U4RTS)

/*!
 * @brief PJ6 is used for UART4 RTS
 */
#define UART2MSP432E4_PJ6_U4RTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTJ, 6, GPIO_PJ6_U4RTS)

/*!
 * @brief PN6 is used for UART4 RTS
 */
#define UART2MSP432E4_PN6_U4RTS GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTN, 6, GPIO_PN6_U4RTS)


/*!
 * @brief PC6 is used for UART5 RX
 */
#define UART2MSP432E4_PC6_U5RX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTC, 6, GPIO_PC6_U5RX)

/*!
 * @brief PH6 is used for UART5 RX
 */
#define UART2MSP432E4_PH6_U5RX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTH, 6, GPIO_PH6_U5RX)

/*!
 * @brief PC7 is used for UART5 TX
 */
#define UART2MSP432E4_PC7_U5TX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTC, 7, GPIO_PC7_U5TX)

/*!
 * @brief PH7 is used for UART5 TX
 */
#define UART2MSP432E4_PH7_U5TX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTH, 7, GPIO_PH7_U5TX)


/*!
 * @brief PP0 is used for UART6 RX
 */
#define UART2MSP432E4_PP0_U6RX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTP, 0, GPIO_PP0_U6RX)

/*!
 * @brief PP1 is used for UART6 TX
 */
#define UART2MSP432E4_PP1_U6TX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTP, 1, GPIO_PP1_U6TX)


/*!
 * @brief PC4 is used for UART7 RX
 */
#define UART2MSP432E4_PC4_U7RX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTC, 4, GPIO_PC4_U7RX)

/*!
 * @brief PH6 is used for UART7 RX
 */
#define UART2MSP432E4_PH6_U7RX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTH, 6, GPIO_PH6_U7RX)

/*!
 * @brief PC5 is used for UART7 TX
 */
#define UART2MSP432E4_PC5_U7TX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTC, 5, GPIO_PC5_U7TX)

/*!
 * @brief PH7 is used for UART7 TX
 */
#define UART2MSP432E4_PH7_U7TX GPIOMSP432E4_pinConfigMask(GPIOMSP432E4_PORTH, 7, GPIO_PH7_U7TX)

/* UART2MSP432E4 functions */
extern void UART2MSP432E4_close(UART2_Handle handle);
extern UART2_Handle UART2MSP432E4_open(uint_least8_t, UART2_Params *params);
extern int_fast16_t UART2MSP432E4_read(UART2_Handle handle, void *buffer,
        size_t size, size_t *bytesRead, uint32_t timeout);
extern void UART2MSP432E4_readCancel(UART2_Handle handle);
extern int_fast16_t UART2MSP432E4_write(UART2_Handle handle,
        const void *buffer, size_t size, size_t *bytesWritten,
        uint32_t timeout);
extern void UART2MSP432E4_writeCancel(UART2_Handle handle);
extern void UART2MSP432E4_flushRx(UART2_Handle handle);

/* UART2 function table pointer */
extern const UART2_FxnTable UART2MSP432E4_fxnTable;

/*!
 *  @brief      UART2MSP432E4 Hardware attributes
 *
 *  The fields, baseAddr and intNum are used by driverlib
 *  APIs and therefore must be populated by
 *  driverlib macro definitions. These definitions are found under the
 *  device family in:
 *      - inc/hw_memmap.h
 *      - inc/hw_ints.h
 *      - driverlib/uart.h
 *
 *  intPriority is the UART peripheral's interrupt priority, as defined by the
 *  underlying OS.  It is passed unmodified to the underlying OS's interrupt
 *  handler creation code, so you need to refer to the OS documentation
 *  for usage.  For example, for SYS/BIOS applications, refer to the
 *  ti.sysbios.family.arm.m3.Hwi documentation for SYS/BIOS usage of
 *  interrupt priorities.  If the driver uses the ti.dpl interface
 *  instead of making OS calls directly, then the HwiP port handles the
 *  interrupt priority in an OS specific way.  In the case of the SYS/BIOS
 *  port, intPriority is passed unmodified to Hwi_create().
 *  The MSP432E401Y uses three of the priority bits, meaning ~0 has the same
 *  effect as (7 << 5).
 *
 *        (7 << 5) will apply the lowest priority.
 *        (1 << 5) will apply the highest priority.
 *
 *  Setting the priority to 0 is not supported by this driver.  HWI's with
 *  priority 0 ignore the HWI dispatcher to support zero-latency interrupts,
 *  thus invalidating the critical sections in this driver.
 *
 *  A sample structure is shown below:
 *  @code
 *  const UART2MSP432E4_HWAttrs uart2MSP432E4HWAttrs[] = {
 *      {
 *          .baseAddr     = UART0_BASE,
 *          .intNum       = INT_UART0,
 *          .intPriority  = (~0),
 *          .flowControl  = UART2MSP432E4_FLOWCTRL_NONE,
 *          .rxPin        = UART2MSP432E4_PA0_U0RX,
 *          .txPin        = UART2MSP432E4_PA1_U0TX,
 *          .ctsPin       = UART2MSP432E4_PIN_UNASSIGNED,
 *          .rtsPin       = UART2MSP432E4_PIN_UNASSIGNED,
 *          .rxDmaChannel = UDMA_CH8_UART0RX,
 *          .txDmaChannel = UDMA_CH9_UART0TX,
 *      },
 *      {
 *          .baseAddr     = UART3_BASE,
 *          .intNum       = INT_UART3,
 *          .intPriority  = (~0),
 *          .flowControl  = UART2MSP432E4_FLOWCTRL_HARDWARE,
 *          .rxPin = UART2MSP432E4_PA4_U3RX,
 *          .txPin = UART2MSP432E4_PA5_U3TX,
 *          .ctsPin = UART2MSP432E4_PN5_U3CTS,
 *          .rtsPin = UART2MSP432E4_PN4_U3RTS,
 *          .rxDmaChannel = UDMA_CH16_UART3RX,
 *          .txDmaChannel = UDMA_CH17_UART3TX,

 *      },
 *  };
 *  @endcode
 *
 *  To enable flow control, the .ctsPin and/or .rtsPin must be assigned.
 *  In addition, .flowControl must be set to UART2MSP432E4_FLOWCTL_HARDWARE.
 */
typedef struct {
    /*! UART Peripheral's base address */
    uint32_t        baseAddr;
    /*! UART Peripheral's interrupt vector */
    int             intNum;
    /*! UART Peripheral's interrupt priority */
    uint8_t         intPriority;
    /*! Hardware flow control setting */
    uint32_t        flowControl;
    /*! UART RX pin assignment */
    uint32_t        rxPin;
    /*! UART TX pin assignment */
    uint32_t        txPin;
    /*! UART clear to send (CTS) pin assignment */
    uint32_t        ctsPin;
    /*! UART request to send (RTS) pin assignment */
    uint32_t        rtsPin;
    /*! uDMA channel for RX data */
    uint32_t        rxDmaChannel;
    /*! uDMA channel for TX data */
    uint32_t        txDmaChannel;
} UART2MSP432E4_HWAttrs;

/*!
 *  @brief      UART2MSP432E4 Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct {
    /* UART2 state variable */
    struct {
        bool             opened:1;         /* Has the obj been opened */
        UART2_Mode       readMode;         /* Mode for read calls */
        UART2_Mode       writeMode;        /* Mode for write calls */
        UART2_ReadReturnMode readReturnMode:1; /* RX return mode (partial/full) */
        bool             txEnabled:1;      /* Flag to show ongoing transmit */
    } state;

    HwiP_Handle          hwi;              /* Hwi object for interrupts */
    int32_t              rxStatus;         /* RX status */
    int32_t              txStatus;         /* TX status */
    void                *userArg;          /* User supplied arg for callbacks */
    UDMAMSP432E4_Handle  udmaHandle;       /* For setting power dependency */

    /* UART read variables */
    unsigned char       *readBuf;          /* Buffer data pointer */
    size_t               readSize;         /* Number of bytes to read */
    uint32_t             nReadTransfers;   /* Number of DMA transfers needed */
    size_t               readCount;        /* Number of bytes left to read */
    size_t               rxSize;           /* # of bytes to read in DMA xfer */
    size_t               bytesRead;        /* Number of bytes read */
    SemaphoreP_Handle    readSem;          /* UART read semaphore */
    UART2_Callback       readCallback;     /* Pointer to read callback */

    /* UART write variables */
    const unsigned char *writeBuf;         /* Buffer data pointer */
    size_t               writeSize;        /* Number of bytes to write*/
    uint32_t             nWriteTransfers;  /* Number of DMA transfers needed */
    size_t               writeCount;       /* Number of bytes left to write */
    size_t               txSize;           /* # of bytes to write with DMA */
    size_t               bytesWritten;     /* Number of bytes written */
    SemaphoreP_Handle    writeSem;         /* UART write semaphore*/
    UART2_Callback       writeCallback;    /* Pointer to write callback */

    /* For Power management */
    unsigned int         powerMgrId;       /* Determined from base address */
} UART2MSP432E4_Object, *UART2MSP432E4_Handle;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_uart2_UART2MSP432E4__include */
