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
/*!****************************************************************************
 *  @file       ADCBufMSP432E4.h
 *
 *  @brief      ADCBuf driver implementation for a MSP432E4 analog-to-digital
 *              converter
 *
 *  # Overview #
 *  This driver takes \p n samples using the ADC and stores the results in a
 *  buffer. This implementation supports both #ADCBuf_RECURRENCE_MODE_ONE_SHOT
 *  and #ADCBuf_RECURRENCE_MODE_CONTINUOUS. The ADCBuf_MSP432E4 driver
 *  supports use of both ADC peripherals and their sample sequencers.
 *
 *  # General Behavior #
 *  The MSP342E4 microcontroller has two ADC modules and each ADC module has
 *  four sample sequencers, SS0-SS3, that control and capture sample data.
 *  Sequencer priority can be initialized in #ADCBufMSP432E4_HWAttrsV1 by
 *  creating an array that holds the corresponding #ADCBufMSP432E4_SequencePriorities.
 *
 *  To initialize the sequencers, in the board file "MSP_EXP432E401Y.c"/board.h
 *  - Specify the sequencer to use by setting ".adcSequence" in the array
 *    #ADCBufMSP432E4_Channels for each channel/sequencer combination needed.
 *  - Change the sequencer priority for each sequencer to be used in the array
 *    #ADCBufMSP432E4_SequencePriorities.
 *
 *  @anchor ti_drivers_adcbuf_ADCBufMSP432E4_SequencePriorities
 *  ## Sequencer Priorities - #ADCBufMSP432E4_SequencePriorities
 *
 *  @code
 *  static ADCBufMSP432E4_SequencePriorities seqPriorities[ADCBufMSP432E4_SEQUENCER_COUNT] = {
 *      ADCBufMSP432E4_Priority_0,
 *      ADCBufMSP432E4_Seq_Disable,
 *      ADCBufMSP432E4_Seq_Disable,
 *      ADCBufMSP432E4_Seq_Disable
 *  };
 *  @endcode
 *
 *  The MSP432E4 ADC sample sequencers can be triggered by several
 *  different types of software and hardware events. The driver currently
 *  only supports  #ADCBufMSP432E4_SOFTWARE_AUTOMATIC_TRIGGER and
 *  #ADCBufMSP432E4_TIMER_TRIGGER trigger sources. Multiple sequencers
 *  can share the same trigger source. An unique #ADCBufMSP432E4_TriggerSource
 *  array can be defined for each ADCBuf instance. That is, each
 *  #ADCBufMSP432E4_HWAttrsV1.adcTriggerSource can point to a unique
 *  #ADCBufMSP432E4_TriggerSource array.
 *
 *  The application is responsible for defining a #ADCBufMSP432E4_Channels
 *  array. Each index corresponds to a ADC channel configuration. A unique
 *  #ADCBufMSP432E4_Channels array can be defined for each ADCBuf instance.
 *  That is, each #ADCBufMSP432E4_HWAttrsV1.channelSetting can point to a unique
 *  #ADCBufMSP432E4_Channels array.
 *
 *  The #ADCBufMSP432E4_Channels array lets the user select the #ADCBufMSP432E4_Sequencer,
 *  #ADCBufMSP432E4_DifferentialMode, the differential pins if needed, MSP432E4
 *  specific ADC modes like #ADCBufMSP432E4_TEMPERATURE_MODE that let the user
 *  configure the internal temperature mode
 *
 *  @anchor ti_drivers_adcbuf_ADCBufMSP432E4_ADCBufMSP432E4_Channels
 *  ## Channel section - #ADCBufMSP432E4_Channels
 *  @code
 *  ADCBufMSP432E4_Channels adcBuf0MSP432E4Channels[ADCBUF0CHANNELCOUNT] = {
 *  {
 *      .adcPin = ADCBufMSP432E4_PE_3_A0,
 *      .adcSequence = ADCBufMSP432E4_Seq_0,
 *      .adcInputMode = ADCBufMSP432E4_SINGLE_ENDED,
 *      .adcDifferentialPin = ADCBufMSP432E4_PIN_NONE,
 *      .adcInternalSource = ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF,
 *      .refVoltage = 3300000
 *  },
 *  {
 *      .adcPin = ADCBufMSP432E4_PE_2_A1,
 *      .adcSequence = ADCBufMSP432E4_Seq_1,
 *      .adcInputMode = ADCBufMSP432E4_SINGLE_ENDED,
 *      .adcDifferentialPin = ADCBufMSP432E4_PIN_NONE,
 *      .adcInternalSource = ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF,
 *      .refVoltage = 3300000
 *  },
 *  @endcode
 *
 *  Please refer to #ADCBufMSP432E4_HWAttrsV1 structure for other hardware
 *  dependent configuration settings Some examples include voltage reference
 *  source selection and optional DMA usage.
 *
 *  This driver supports various modes of operation and sampling.
 *  #ADCBufMSP432E4_HWAttrsV1.useDMA can be set to \p 1 to improve performance.
 *  Additionally, samples can be performed using #ADCBufMSP432E4_DIFFERENTIAL
 *  or #ADCBufMSP432E4_TEMPERATURE_MODE modes.
 *

 *
 *  In the application,
 *  - Call ADCBuf_init() once per ADC module.
 *  - Each ADC module requires a separate ADCBuf_open() call.
 *  - Create and initialize "ADCBuf_Conversion" object for each sequencer.
 *  - To make each sequencer sample multiple channels, create the conversion
 *    object as an array, example ADCBuf_Conversion continuousConversion[2]
 *    and initialize each channel within the conversion object.
 *
 *  Refer to @ref ADCBuf.h for a complete description of APIs & example use.
 *
 ******************************************************************************
 */

#ifndef ti_drivers_adcbuf_ADCBufMSP432E4__include
#define ti_drivers_adcbuf_ADCBufMSP432E4__include

#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/dma/UDMAMSP432E4.h>
#include <ti/devices/msp432e4/driverlib/adc.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ADCBuf port/pin defines for pin configuration.  Ports B, D, E, and K are
 *  configurable through the port mapping controller.  None of the port
 *  mappings support ADC.
 *  Channel specifies the ADC channel and ranges from 0 to 23.
 *  pin range: 0 - 7, port range: 0 - 15
 *
 *      31-24        23-16      15-8     7-0
 *  -------------------------------------------
 *  |  RESERVED  |  CHANNEL  |  PORT  |  PIN  |
 *  -------------------------------------------
 *
 *  channel = (((config) >> 16) & 0x10F)
 *  port    = (((config << 4) & 0x000FF000) | 0x40000000)
 *  pin     = ((config) & 0xFF)
 *
 */

/*!
 *  \defgroup ADCBufMSP432E4PinIdentifier used for
 *  #ADCBufMSP432E4_Channels.adcPin and
 *  #ADCBufMSP432E4_Channels.adcDifferentialPin.
 *
 *  Syntax: PB4 => Port B Pin 4
 *  @{
 */
/*!
 *  @name PB4, Analog Channel 10
 *  @{
 */
#define ADCBufMSP432E4_PB_4_A10 ((10 << 16) | 0x5910)
/*! @} */
/*!
 *  @name PB5, Analog Channel 11
 *  @{
 */
#define ADCBufMSP432E4_PB_5_A11 ((11 << 16) | 0x5920)
/*! @} */

/*!
 *  @name PD0, Analog Channel 15
 *  @{
 */
#define ADCBufMSP432E4_PD_0_A15 ((15 << 16) | 0x5B01)
/*! @} */
/*!
 *  @name PD1, Analog Channel 14
 *  @{
 */
#define ADCBufMSP432E4_PD_1_A14 ((14 << 16) | 0x5B02)
/*! @} */
/*!
 *  @name PD2, Analog Channel 13
 *  @{
 */
#define ADCBufMSP432E4_PD_2_A13 ((13 << 16) | 0x5B04)
/*! @} */
/*!
 *  @name PD3, Analog Channel 12
 *  @{
 */
#define ADCBufMSP432E4_PD_3_A12 ((12 << 16) | 0x5B08)
/*! @} */
/*!
 *  @name PD4, Analog Channel 7
 *  @{
 */
#define ADCBufMSP432E4_PD_4_A7  ((7 << 16) | 0x5B10)
/*! @} */
/*!
 *  @name PD5, Analog Channel 6
 *  @{
 */
#define ADCBufMSP432E4_PD_5_A6  ((6 << 16) | 0x5B20)
/*! @} */
/*!
 *  @name PD6, Analog Channel 5
 *  @{
 */
#define ADCBufMSP432E4_PD_6_A5  ((5 << 16) | 0x5B40)
/*! @} */
/*!
 *  @name PD7, Analog Channel 4
 *  @{
 */
#define ADCBufMSP432E4_PD_7_A4  ((4 << 16) | 0x5B80)
/*! @} */
/*!
 *  @name PE0, Analog Channel 3
 *  @{
 */
#define ADCBufMSP432E4_PE_0_A3  ((3 << 16) | 0x5C01)
/*! @} */
/*!
 *  @name PE1, Analog Channel 2
 *  @{
 */
#define ADCBufMSP432E4_PE_1_A2  ((2 << 16) | 0x5C02)
/*! @} */
/*!
 *  @name PE2, Analog Channel 1
 *  @{
 */
#define ADCBufMSP432E4_PE_2_A1  ((1 << 16) | 0x5C04)
/*! @} */
/*!
 *  @name PE3, Analog Channel 0
 *  @{
 */
#define ADCBufMSP432E4_PE_3_A0  ((0 << 16) | 0x5C08)
/*! @} */
/*!
 *  @name PE4, Analog Channel 9
 *  @{
 */
#define ADCBufMSP432E4_PE_4_A9  ((9 << 16) | 0x5C10)
/*! @} */
/*!
 *  @name PE5, Analog Channel 8
 *  @{
 */
#define ADCBufMSP432E4_PE_5_A8  ((8 << 16) | 0x5C20)
/*! @} */
/*!
 *  @name PE6, Analog Channel 20
 *  @{
 */
#define ADCBufMSP432E4_PE_6_A20  (((20-16) << 16) | 0x5C40 | 0x01000000)
/*! @} */
/*!
 *  @name PE7, Analog Channel 21
 *  @{
 */
#define ADCBufMSP432E4_PE_7_A21  (((21-16) << 16) | 0x5C80 | 0x01000000)
/*! @} */
/*!
 *  @name PK0, Analog Channel 16
 *  @{
 */
#define ADCBufMSP432E4_PK_0_A16 (((16-16) << 16) | 0x6101 | 0x01000000)
/*! @} */
/*!
 *  @name PK1, Analog Channel 17
 *  @{
 */
#define ADCBufMSP432E4_PK_1_A17 (((17-16) << 16) | 0x6102 | 0x01000000)
/*! @} */
/*!
 *  @name PK2, Analog Channel 18
 *  @{
 */
#define ADCBufMSP432E4_PK_2_A18 (((18-16) << 16) | 0x6104 | 0x01000000)
/*! @} */
/*!
 *  @name PK3, Analog Channel 19
 *  @{
 */
#define ADCBufMSP432E4_PK_3_A19 (((19-16) << 16) | 0x6108 | 0x01000000)
/*! @} */
/*!
 *  @name PP6, Analog Channel 23
 *  @{
 */
#define ADCBufMSP432E4_PP_6_A23  (((23-16) << 16) | 0x6540 | 0x01000000)
/*! @} */
/*!
 *  @name PP7, Analog Channel 22
 *  @{
 */
#define ADCBufMSP432E4_PP_7_A22  (((22-16) << 16) | 0x6580 | 0x01000000)
/*! @} */
/*!
 *  @name PIN NONE.
 *  @{
 */
#define ADCBufMSP432E4_PIN_NONE  0
/*! @} */
/*! @} */

/*! Number of available ADC channels */
#define MSP432E4_NUM_ADC_CHANNELS (24)

/*! Number of #ADCBufMSP432E4_Sequencer per peripheral */
#define ADCBufMSP432E4_SEQUENCER_COUNT 4

/* ADC function table pointer */
extern const ADCBuf_FxnTable ADCBufMSP432E4_fxnTable;

/*!
 *  @brief  ADCBufMSP432E4 Sequencer Priorities
 *
 *  The application is responsible for defining a
 *  #ADCBufMSP432E4_SequencePriorities array of #ADCBufMSP432E4_SEQUENCER_COUNT
 *  length. Each index corresponds to one of the four sequencers respectively.
 *  A unique #ADCBufMSP432E4_SequencePriorities array can be defined for each
 *  ADCBuf instance. That is, each #ADCBufMSP432E4_HWAttrsV1.sequencePriority
 *  can point to a unique #ADCBufMSP432E4_TriggerSource array.
 *
 *  A sequencer's priority is set by assigning its respective index to one of
 *  the #ADCBufMSP432E4_SequencePriorities definitions. Multiple sequencers
 *  can share the same priority.
 *
 *  In the code example below, sequencer 0 and sequencer 2 have a priority of
 *  #ADCBufMSP432E4_Priority_3. Sequencer 1 has a priority of
 *  #ADCBufMSP432E4_Priority_0. Sequencer 3 is disabled.
 *
 * @code
 *  static ADCBufMSP432E4_SequencePriorities sequencePriority[ADCBufMSP432E4_SEQUENCER_COUNT] = {
 *       ADCBufMSP432E4_Priority_3,
 *       ADCBufMSP432E4_Priority_0,
 *       ADCBufMSP432E4_Priority_3,
 *       ADCBufMSP432E4_Seq_Disable
 *  };
 * @endcode
 */
typedef enum {
    /*! ADC Sequencer priority 0, the highest priority */
    ADCBufMSP432E4_Priority_0 = 0,

    /*! ADC Sequencer priority 1 */
    ADCBufMSP432E4_Priority_1 = 1,

    /*! ADC Sequencer priority 2 */
    ADCBufMSP432E4_Priority_2 = 2,

    /*! ADC Sequencer priority 3, the lowest priority */
    ADCBufMSP432E4_Priority_3 = 3,

    /*! Disables a sequencer */
    ADCBufMSP432E4_Seq_Disable = 4
} ADCBufMSP432E4_SequencePriorities;

/*!
 *  @brief  ADCBufMSP432E4 Sequencer
 *
 *  These fields are used to assign a sequencer to a channel by using the
 *  #ADCBufMSP432E4_Channels.adcSequence field. Each sequencer can only
 *  support a limited number of ADC channels. It is the applications
 *  responsibility to ensure an appropriate number of channels are assigned
 *  to each sequencer. Assigning more ADC channels than a sequencer can
 *  support may lead to undefined behavior. Each ADC peripheral has
 *  dedicated sequencers.
 */
typedef enum {
    /*! ADC Sample Sequencer 0, can support up to 8 channels */
    ADCBufMSP432E4_Seq_0 = 0,

    /*! ADC Sample Sequencer 1, can support up to 4 channels */
    ADCBufMSP432E4_Seq_1 = 1,

    /*! ADC Sample Sequencer 2, can support up to 4 channels */
    ADCBufMSP432E4_Seq_2 = 2,

    /*! ADC Sample Sequencer 3, can support 1 channel */
    ADCBufMSP432E4_Seq_3 = 3
} ADCBufMSP432E4_Sequencer;

/*!
 *  @brief  ADCBufMSP432E4 Internal Source Mode
 *
 *  These fields are used by #ADCBufMSP432E4_Channels.adcInternalSource
 *  to specify if an internal source mode is selected. If using an internal
 *  source, #ADCBufMSP432E4_Channels.adcInputMode must be set to
 *  #ADCBufMSP432E4_SINGLE_ENDED.
 */
typedef enum {
    /*! No internal source mode is used */
    ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF = 0,

    /*!
     *  The ADC will sample the internal temperature. The
     *  #ADCBufMSP432E4_Channels.adcPin should be set to
     *  #ADCBufMSP432E4_PIN_NONE
     */
    ADCBufMSP432E4_TEMPERATURE_MODE = ADC_CTL_TS,
} ADCBufMSP432E4_InternalSourceMode;

/*!
 *  @brief  ADCBufMSP432E4 Differential Mode
 *
 *  These fields are used by #ADCBufMSP432E4_Channels.adcInputMode to specify
 *  if a differential sampling mode is selected. If using the differential
 *  sampling mode, #ADCBufMSP432E4_Channels.adcInternalSource must be set to
 *  #ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF.
 */
typedef enum {
    /*! Use single ended sampling mode */
    ADCBufMSP432E4_SINGLE_ENDED = 0,

    /*!
     * Use differential sampling mode. The ADC will measure the voltage
     * difference between two channels. The user is responsible for setting
     * the #ADCBufMSP432E4_Channels.adcDifferentialPin.
     */
    ADCBufMSP432E4_DIFFERENTIAL = ADC_CTL_D
} ADCBufMSP432E4_DifferentialMode;

/*!
 *  @brief  ADCBufMSP432E4 Trigger Source
 *
 *  The application is responsible for defining a #ADCBufMSP432E4_TriggerSource
 *  array of #ADCBufMSP432E4_SEQUENCER_COUNT length. Each index corresponds to
 *  one of the four sequencers respectively. A unique
 *  #ADCBufMSP432E4_TriggerSource array can be defined for each ADCBuf
 *  instance. That is, each #ADCBufMSP432E4_HWAttrsV1.adcTriggerSource can
 *  point to a unique #ADCBufMSP432E4_TriggerSource array.
 *
 *  A sequencer's trigger source is set by assigning its respective index to
 *  one of the #ADCBufMSP432E4_TriggerSource definitions. Multiple sequencers
 *  can share the same trigger source.
 *
 *  In the code example below, sequencer 0, sequencer 2 and sequencer 3 have a
 *  trigger source of #ADCBufMSP432E4_TIMER_TRIGGER. Sequencer 1 has a
 *  trigger source of #ADCBufMSP432E4_SOFTWARE_AUTOMATIC_TRIGGER.
 *
 * @code
 *  static ADCBufMSP432E4_TriggerSource adcTriggerSource[ADCBufMSP432E4_SEQUENCER_COUNT] = {
 *       ADCBufMSP432E4_TIMER_TRIGGER,
 *       ADCBufMSP432E4_SOFTWARE_AUTOMATIC_TRIGGER,
 *       ADCBufMSP432E4_TIMER_TRIGGER,
 *       ADCBufMSP432E4_TIMER_TRIGGER
 *  };
 * @endcode
 */
typedef enum {
    /*!
     *  Automatically and continuously trigger ADC sampling. Precaution should
     *  be taken when using this mode with multiple sequencers. If the
     *  sequencer's priority using the software trigger is too high, it is
     *  possible to starve other lower priority sequencers. Generally, a
     *  sequencer using #ADCBufMSP432E4_SOFTWARE_AUTOMATIC_TRIGGER should be
     *  set to the lowest priority.
     */
    ADCBufMSP432E4_SOFTWARE_AUTOMATIC_TRIGGER = ADC_TRIGGER_ALWAYS,

    /*!
     *  Trigger ADC samples using a general purpose timer. When using both
     *  ADC peripherals, both must be initialized to use the same general
     *  purpose timer. The application is responsible for providing
     *  #ADCBufMSP432E4_HWAttrsV1.adcTimerSource.
     */
    ADCBufMSP432E4_TIMER_TRIGGER = ADC_TRIGGER_TIMER,
}ADCBufMSP432E4_TriggerSource;

/*!
 *  @brief  ADCBufMSP432E4 phase delay
 *
 *  These fields are used by #ADCBufMSP432E4_HWAttrsV1 to specify the phase
 *  delay between a trigger and the start of a sequence for the ADC module.
 *  By selecting a different phase delay for a pair of ADC modules (such as
 *  \b ADCBufMSP432E4_Phase_Delay_0 and \b ADCBufMSP432E4_Phase_Delay_180)
 *  and having each ADC module sample the same analog input, it is possible
 *  to increase the sampling rate of the analog input (with samples N, N+2,
 *  N+4, and so on, coming from the first ADC and samples N+1, N+3, N+5, and
 *  so on, coming from the second ADC). The ADC module has a single phase
 *  delay that is applied to all sample sequences within that module.
 */
typedef enum {
    /*! Use phase delay of 0 degrees */
    ADCBufMSP432E4_Phase_Delay_0 = ADC_PHASE_0,

    /*! Use phase delay of 22.5 degrees */
    ADCBufMSP432E4_Phase_Delay_22_5 = ADC_PHASE_22_5,

    /*! Use phase delay of 45 degrees */
    ADCBufMSP432E4_Phase_Delay_45 = ADC_PHASE_45,

    /*! Use phase delay of 67.5 degrees */
    ADCBufMSP432E4_Phase_Delay_67_5 = ADC_PHASE_67_5,

    /*! Use phase delay of 90 degrees */
    ADCBufMSP432E4_Phase_Delay_90 = ADC_PHASE_90,

    /*! Use phase delay of 112.5 degrees */
    ADCBufMSP432E4_Phase_Delay_112_5 = ADC_PHASE_112_5,

    /*! Use phase delay of 135 degrees */
    ADCBufMSP432E4_Phase_Delay_135 = ADC_PHASE_135,

    /*! Use phase delay of 157.5 degrees */
    ADCBufMSP432E4_Phase_Delay_157_5 = ADC_PHASE_157_5,

    /*! Use phase delay of 180 degrees */
    ADCBufMSP432E4_Phase_Delay_180 = ADC_PHASE_180,

    /*! Use phase delay of 202.5 degrees */
    ADCBufMSP432E4_Phase_Delay_202_5 = ADC_PHASE_202_5,

    /*! Use phase delay of 225 degrees */
    ADCBufMSP432E4_Phase_Delay_225 = ADC_PHASE_225,

    /*! Use phase delay of 247.5 degrees */
    ADCBufMSP432E4_Phase_Delay_247_5 = ADC_PHASE_247_5,

    /*! Use phase delay of 270 degrees */
    ADCBufMSP432E4_Phase_Delay_270 = ADC_PHASE_270,

    /*! Use phase delay of 292.5 degrees */
    ADCBufMSP432E4_Phase_Delay_292_5 = ADC_PHASE_292_5,

    /*! Use phase delay of 315 degrees */
    ADCBufMSP432E4_Phase_Delay_315 = ADC_PHASE_315,

    /*! Use phase delay of 337.5 degrees */
    ADCBufMSP432E4_Phase_Delay_337_5 = ADC_PHASE_337_5,
} ADCBufMSP432E4_Phase;

/*!
 *  @brief  ADCMSP432E4 Sampling Duration
 *
 *  These fields define the sample and hold time in the pulse width units.
 *  The application can specify the sampling duration at runtime using
 *  the #ADCBufMSP432E4_ParamsExtension.samplingDuration field. The ADC clock
 *  is the same as the system clock.
 */
typedef enum {
    /*! Use pulse width of 4 ADC clocks */
    ADCBufMSP432E4_SamplingDuration_PULSE_WIDTH_4 = ADC_CTL_SHOLD_4,

    /*! Use pulse width of 8 ADC clocks */
    ADCBufMSP432E4_SamplingDuration_PULSE_WIDTH_8 = ADC_CTL_SHOLD_8,

    /*! Use pulse width of 16 ADC clocks */
    ADCBufMSP432E4_SamplingDuration_PULSE_WIDTH_16 = ADC_CTL_SHOLD_16,

    /*! Use pulse width of 32 ADC clocks */
    ADCBufMSP432E4_SamplingDuration_PULSE_WIDTH_32 = ADC_CTL_SHOLD_32,

    /*! Use pulse width of 64 ADC clocks */
    ADCBufMSP432E4_SamplingDuration_PULSE_WIDTH_64 = ADC_CTL_SHOLD_64,

    /*! Use pulse width of 128 ADC clocks */
    ADCBufMSP432E4_SamplingDuration_PULSE_WIDTH_128 = ADC_CTL_SHOLD_128,

    /*! Use pulse width of 256 ADC clocks */
    ADCBufMSP432E4_SamplingDuration_PULSE_WIDTH_256 = ADC_CTL_SHOLD_256
} ADCBufMSP432E4_SamplingDuration;

/*!
 *  @brief  ADCBufMSP432E4 Reference Source
 *
 *  These fields are used by #ADCBufMSP432E4_HWAttrsV1.refSource to specify
 *  the reference source for the ADC peripheral. This is therefore shared
 *  amongst all channels for an ADCBuf instance.
 */
typedef enum {
    /*! Use internal reference voltage source of 3V. */
    ADCBufMSP432E4_VREF_INTERNAL = ADC_REF_INT,

    /*! Use an external 3V reference source supplied to the AVREF pin. */
    ADCBufMSP432E4_VREF_EXTERNAL_3V = ADC_REF_EXT_3V
} ADCBufMSP432E4_ReferenceSource;

/*!
 *  @brief  MSP432E4 #ADCBuf_Params Parameter Extensions
 *
 *  To use non-default MSP432E4 specific parameters when calling ADCBuf_open(),
 *  a pointer to an instance of this structure must be specified in
 *  #ADCBuf_Params.custom. Alternatively, these values can be set using the
 *  ADCBuf_control() function after calling ADCBuf_open().
 */
typedef struct {
    /*! ADC sampling duration, #ADCBufMSP432E4_SamplingDuration */
    ADCBufMSP432E4_SamplingDuration samplingDuration;
} ADCBufMSP432E4_ParamsExtension;

/*!
 *  @brief  ADCBufMSP432E4 Channel Settings
 *
 *  The application is responsible for defining a
 *  #ADCBufMSP432E4_Channels array. Each index corresponds to a ADC channel
 *  configuration. A unique #ADCBufMSP432E4_Channels array can be defined for
 *  each ADCBuf instance. That is, each
 *  #ADCBufMSP432E4_HWAttrsV1.channelSetting can point to a unique
 *  #ADCBufMSP432E4_Channels array.
 *
 *  In the code example below, three channels are configured. The first channel
 *  will sample the differential between two external analog signals,\p .adcPin
 *  and \p .adcDifferentialPin. The second channel will sample the internal
 *  temperature analog signal. The third channel will sample an external
 *  analog signal on \p .adcPin.
 *
 * @code
 *  ADCBufMSP432E4_Channels adcBufMSP432E4Channels[3] = {
 *      {
 *          .adcPin = ADCBufMSP432E4_PE_0_A3,
 *          .adcSequence = ADCBufMSP432E4_Seq_0,
 *          .adcInputMode = ADCBufMSP432E4_DIFFERENTIAL,
 *          .adcDifferentialPin = ADCBufMSP432E4_PE_1_A2,
 *          .adcInternalSource = ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF,
 *          .refVoltage = 3300000
 *      },
 *      {
 *          .adcPin = ADCBufMSP432E4_PIN_NONE,
 *          .adcSequence = ADCBufMSP432E4_Seq_1,
 *          .adcInputMode = ADCBufMSP432E4_SINGLE_ENDED,
 *          .adcDifferentialPin = ADCBufMSP432E4_PIN_NONE,
 *          .adcInternalSource = ADCBufMSP432E4_TEMPERATURE_MODE,
 *          .refVoltage = 3300000
 *      },
 *      {
 *          .adcPin = ADCBufMSP432E4_PD_3_A12,
 *          .adcSequence = ADCBufMSP432E4_Seq_3,
 *          .adcInputMode = ADCBufMSP432E4_SINGLE_ENDED,
 *          .adcDifferentialPin = ADCBufMSP432E4_PIN_NONE,
 *          .adcInternalSource = ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF,
 *          .refVoltage = 3300000
 *      }
 *  };
 * @endcode
 */
typedef struct {
    /*! ADC reference voltage in microVolts */
    uint32_t refVoltage;

    /*! An \ref ADCBufMSP432E4PinIdentifier */
    uint32_t adcPin;

    /*! An #ADCBufMSP432E4_Sequencer */
    ADCBufMSP432E4_Sequencer adcSequence;

    /*!
     *  Specify if this channel uses differential sampling mode. If using the
     *  differential sampling mode, #ADCBufMSP432E4_Channels.adcInternalSource
     *  must be set to #ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF.
     */
    ADCBufMSP432E4_DifferentialMode adcInputMode;

    /*!
     *  Specify if  this channel uses internal source mode. If using the
     *  internal source mode, #ADCBufMSP432E4_Channels.adcInputMode must be
     *  set to #ADCBufMSP432E4_SINGLE_ENDED. */
    ADCBufMSP432E4_InternalSourceMode adcInternalSource;

    /*!
     *  Specify this channel's \ref ADCBufMSP432E4PinIdentifier when #adcInputMode
     *  is #ADCBufMSP432E4_DIFFERENTIAL. Otherwise, use
     *  #ADCBufMSP432E4_PIN_NONE.
     */
    uint32_t adcDifferentialPin;
} ADCBufMSP432E4_Channels;

/*!
 *  @brief  ADCBufMSP432E4 Hardware attributes
 *
 *  The ADCBufMSP432E4 hardware attributes define hardware specific
 *  settings for a ADCBuf driver instance.
 *
 *  A sample structure is shown below:
 *  @code
 *
 *  static ADCBufMSP432E4_SequencePriorities sequencePriority[ADCBufMSP432E4_SEQUENCER_COUNT] = {
 *      ADCBufMSP432E4_Priority_0,
 *      ADCBufMSP432E4_Seq_Disable,
 *      ADCBufMSP432E4_Seq_Disable,
 *      ADCBufMSP432E4_Seq_Disable
 *  };
 *
 *  static ADCBufMSP432E4_TriggerSource adcTriggerSource[ADCBufMSP432E4_SEQUENCER_COUNT] = {
 *      ADCBufMSP432E4_TIMER_TRIGGER,
 *      ADCBufMSP432E4_TIMER_TRIGGER,
 *      ADCBufMSP432E4_TIMER_TRIGGER,
 *      ADCBufMSP432E4_TIMER_TRIGGER
 *  };
 *
 *  ADCBufMSP432E4_Channels adcBufMSP432E4Channels[1] = {
 *      {
 *          .adcPin = ADCBufMSP432E4_PE_3_A0,
 *          .adcSequence = ADCBufMSP432E4_Seq_0,
 *          .adcInputMode = ADCBufMSP432E4_SINGLE_ENDED,
 *          .adcDifferentialPin = ADCBufMSP432E4_PIN_NONE,
 *          .adcInternalSource = ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF,
 *          .refVoltage = 3300000
 *      }
 *  };
 *
 *  const ADCBufMSP432E4_HWAttrsV1 adcbufMSP432E4HWAttrs[1] = {
 *      {
 *          .intPriority = ~0,
 *          .adcBase = ADC0_BASE,
 *          .channelSetting = adcBufMSP432E4Channels,
 *          .sequencePriority = sequencePriority,
 *          .adcTriggerSource = adcTriggerSource,
 *          .modulePhase =  ADCBufMSP432E4_Phase_Delay_0,
 *          .refSource = ADCBufMSP432E4_VREF_INTERNAL,
 *          .useDMA = 1,
 *         .adcTimerSource = TIMER1_BASE,
 *      }
 * };
 *  @endcode
 */
typedef struct {
    /*! ADC interrupt priority */
    uint32_t intPriority;

    /*! ADC peripheral base address.*/
    uint32_t adcBase;

    /*!
     *  Base address of timer peripheral to use as a timer trigger source when
     *  a sequencer is using #ADCBufMSP432E4_TIMER_TRIGGER. When using both
     *  ADC peripherals, both must be initialized to use the same
     *  #adcTimerSource.
     */
    uint32_t adcTimerSource;

    /*! Pointer to an array of #ADCBufMSP432E4_Channels. */
    ADCBufMSP432E4_Channels *channelSetting;

    /*! Pointer to an array of #ADCBufMSP432E4_TriggerSource. */
    ADCBufMSP432E4_TriggerSource *adcTriggerSource;

    /*!
     *  ADC phase delay between the detection of an ADC trigger event and the
     *  start of the sample sequencer
     */
    ADCBufMSP432E4_Phase modulePhase;

    /*! ADC reference voltage source */
    ADCBufMSP432E4_ReferenceSource refSource;

    /*! Pointer to an array of #ADCBufMSP432E4_SequencePriorities */
    ADCBufMSP432E4_SequencePriorities *sequencePriority;

    /*!
     *  Enables or disables driver use of DMA. Use \b 1 to enable and \b 0 to
     *  disable.
     */
    uint8_t useDMA;
} ADCBufMSP432E4_HWAttrsV1;

/*
 *  @brief  ADCBufMSP432E4 Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct {
    /* Grants exclusive access to ADC */
    SemaphoreP_Handle   mutex;
    /* Notify finished ADC convert */
    SemaphoreP_Handle   convertComplete;

    /* Hardware interrupt handles, one per sequencer */
    HwiP_Handle         sequencerHwiHandles[4];

    /* PingPong flag indicates which sample buffer is active in the conversion */
    uint_fast8_t        pingpongFlag[4];
    /* Count of sampling channels */
    uint_fast8_t        channelCount[4];
    ADCBuf_Conversion   *conversions[4];

    /* Callback function pointer */
    ADCBuf_Callback     callBackFxn;

    uint16_t            *sampleBuffer[4];
    /* Internal dec. conversion buffer counter */
    uint_fast16_t        sampleIndex[4];
    /* Total sampling count per channel */
    uint_fast16_t        sampleCount[4];

    /* ADC sampling duration */
    ADCBufMSP432E4_SamplingDuration samplingDuration;
    /* Timeout for read semaphore in ADCBuf_RETURN_MODE_BLOCKING */
    uint32_t                        semaphoreTimeout;
    /* Frequency in Hz at which the ADC is triggered */
    uint32_t                        samplingFrequency;
    /* Should we convert continuously or one-shot */
    ADCBuf_Recurrence_Mode          recurrenceMode;
    /* Mode for all conversions */
    ADCBuf_Return_Mode              returnMode;
    UDMAMSP432E4_Handle dmaHandle;
    /* To determine if the ADC is open */
    bool                isOpen;
} ADCBufMSP432E4_Object;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_adcbuf_ADCBufMSP432E4__include */
