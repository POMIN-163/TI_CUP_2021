/*
 * Copyright (c) 2018-2019 Texas Instruments Incorporated
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
 *  @file       ADCMSP432E4.h
 *  @brief      ADC driver implementation for the ADC peripheral on MSP432E4
 *
 *  This ADC driver implementation is designed to operate on a ADC peripheral
 *  for MSP432E4.
 *
 *  Refer to @ref ADC.h for a complete description of APIs & example of use.
 *
 ******************************************************************************
 */
#ifndef ti_drivers_adc_ADCMSP432E4__include
#define ti_drivers_adc_ADCMSP432E4__include

#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/ADC.h>
#include <ti/devices/msp432e4/inc/msp432.h>
#include <ti/devices/msp432e4/driverlib/adc.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ADC port/pin defines for pin configuration.  Ports B, D, E, and K are
 *  configurable through the port mapping controller.  None of the port
 *  mappings support ADC.
 *  Channel specifies the ADC channel and ranges from 0 to 20.
 *  pin range: 0 - 7, port range: 0 - 15
 *
 *
 *    32 - 21  20 - 16  15 - 8  7 - 0
 *  ---------------------------------
 *  | X X X X | CHANNEL | PORT | PIN |
 *  ---------------------------------
 *
 *  channel = (((config) >> 16) & 0x1F)
 *  port = (((config << 4) & 0x000FF000) | 0x40000000)
 *  pin = ((config) & 0xFF)
 *
 */
/* Port B */
#define ADCMSP432E4_PB_4_A10 ((10 << 16) | 0x5910) /* ch 10, port B, pin 4 */
#define ADCMSP432E4_PB_5_A11 ((11 << 16) | 0x5920) /* ch 11, port B, pin 5 */

/* Port D */
#define ADCMSP432E4_PD_0_A15 ((15 << 16) | 0x5B01) /* ch 15, port D, pin 0 */
#define ADCMSP432E4_PD_1_A14 ((14 << 16) | 0x5B02) /* ch 14, port D, pin 1 */
#define ADCMSP432E4_PD_2_A13 ((13 << 16) | 0x5B04) /* ch 13, port D, pin 2 */
#define ADCMSP432E4_PD_3_A12 ((12 << 16) | 0x5B08) /* ch 12, port D, pin 3 */
#define ADCMSP432E4_PD_4_A7  ((7 << 16) | 0x5B10)  /* ch 7, port D, pin 4 */
#define ADCMSP432E4_PD_5_A6  ((6 << 16) | 0x5B20)  /* ch 6, port D, pin 5 */
#define ADCMSP432E4_PD_6_A5  ((5 << 16) | 0x5B40)  /* ch 5, port D, pin 6 */
#define ADCMSP432E4_PD_7_A4  ((4 << 16) | 0x5B80)  /* ch 4, port D, pin 7 */

/* Port E */
#define ADCMSP432E4_PE_0_A3  ((3 << 16) | 0x5C01) /* ch 3, port E, pin 0 */
#define ADCMSP432E4_PE_1_A2  ((2 << 16) | 0x5C02) /* ch 2, port E, pin 1 */
#define ADCMSP432E4_PE_2_A1  ((1 << 16) | 0x5C04) /* ch 1, port E, pin 2 */
#define ADCMSP432E4_PE_3_A0  ((0 << 16) | 0x5C08) /* ch 0, port E, pin 3 */
#define ADCMSP432E4_PE_4_A9  ((9 << 16) | 0x5C10) /* ch 9, port E, pin 4 */
#define ADCMSP432E4_PE_5_A8  ((8 << 16) | 0x5C20) /* ch 8, port E, pin 5 */
#define ADCMSP432E4_PE_6_A20  ((20 << 16) | 0x5C40) /* ch 20, port E, pin 6 */
#define ADCMSP432E4_PE_7_A21  ((21 << 16) | 0x5C80) /* ch 21, port E, pin 7 */

/* Port K */
#define ADCMSP432E4_PK_0_A16 ((16 << 16) | 0x6101) /* ch 16, port K, pin 0 */
#define ADCMSP432E4_PK_1_A17 ((17 << 16) | 0x6102) /* ch 17, port K, pin 1 */
#define ADCMSP432E4_PK_2_A18 ((18 << 16) | 0x6104) /* ch 18, port K, pin 2 */
#define ADCMSP432E4_PK_3_A19 ((19 << 16) | 0x6108) /* ch 19, port K, pin 3 */

/* Port P */
#define ADCMSP432E4_PP_6_A23  ((23 << 16) | 0x6540) /* ch 23, port P, pin 6 */
#define ADCMSP432E4_PP_7_A22  ((22 << 16) | 0x6580) /* ch 22, port P, pin 7 */

/*!
 *  @brief  ADCMSP432E4 reference source
 *  These fields are used by ADCMSP432E4_HWAttrs to specify the reference
 *  voltage for each channel.
 *
 */
typedef enum {
    ADCMSP432E4_VREF_INTERNAL = ADC_REF_INT,
    ADCMSP432E4_VREF_EXTERNAL_3V = ADC_REF_EXT_3V
} ADCMSP432E4_ReferenceSource;


/*!
 *  @brief  ADCMSP432E4 Sequencer
 *  These fields are used by ADCMSP432E4_HWAttrs to specify the sample
 *  sequencer for each channel.
 *
 *  Each ADC module contains four programmable sequencers allowing the
 *  sampling of multiple analog input sources without controller intervention.
 */
typedef enum {
    ADCMSP432E4_SEQ0 = 0,
    ADCMSP432E4_SEQ1 = 1,
    ADCMSP432E4_SEQ2 = 2,
    ADCMSP432E4_SEQ3 = 3
} ADCMSP432E4_Sequencer;

/*!
 *  @brief  ADCMSP432E4 Module
 *  These fields are used by ADCMSP432E4_HWAttrs to specify the ADC module
 *  for each channel.
 *
 */
typedef enum {
    ADCMSP432E4_MOD0 = ADC0_BASE,
    ADCMSP432E4_MOD1 = ADC1_BASE
} ADCMSP432E4_Module;


/* ADC function table pointer */
extern const ADC_FxnTable ADCMSP432E4_fxnTable;

/*!
 *  @brief  ADCMSP432E4 Hardware attributes
 *  These fields are used by driverlib APIs and therefore must be populated by
 *  driverlib macro definitions. For SimpleLink these definitions are found in:
 *      - adc.h
 *      - gpio.h
 *
 *  A sample structure is shown below:
 *  @code
 *  const ADCMSP432E4_HWAttrsV1 adcMSP432E4HWAttrs[1] = {
 *      {
 *          .adcPin = ADCMSP432E4_PE_0_A3,
 *          .refVoltage = ADCMSP432E4_VREF_INTERNAL,
 *          .adcModule = ADCMSP432E4_MOD0,
 *          .adcSeq = ADCMSP432E4_SEQ0
 *      }
 *  };
 *  @endcode
 */
typedef struct {
    uint_fast16_t  adcPin;     /*!< ADC pin, port channel */
    uint_fast16_t  refVoltage; /*!< Reference voltage for ADC channel */
    ADCMSP432E4_Module adcModule;
    ADCMSP432E4_Sequencer adcSeq;
} ADCMSP432E4_HWAttrsV1;

/*!
 *  @brief  ADCMSP432 Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct {
    bool isOpen;               /* To determine if the ADC is open */
    bool isProtected;          /* Flag to indicate if thread safety is ensured
                                  by the driver */
} ADCMSP432E4_Object;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_adc_ADCMSP432E4__include */
