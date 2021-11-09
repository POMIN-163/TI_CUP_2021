/*
 * Copyright (c) 2019-2020, Texas Instruments Incorporated
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
/*!*****************************************************************************
 *  @file       Comparator.h
 *  @brief      <b>PRELIMINARY</b> Comparator Driver Interface
 *
 *  <b>WARNING</b> These APIs are <b>PRELIMINARY</b>, and subject to
 *  change in the next few months.
 *
 *  @anchor ti_drivers_Comparator_Overview
 *  # Overview
 *
 *  The Comparator driver serves as the main interface for a typical RTOS
 *  application. Its purpose is to redirect the Comparator APIs to device
 *  specific implementations which are specified using a pointer to a
 *  #Comparator_FxnTable. The device specific implementations are responsible
 *  for creating all the RTOS specific primitives to allow for thread-safe
 *  operation.
 *
 *  The Comparator driver is an analog driver that accepts a configuration of
 *  two analog input signals, compares those values, and then outputs high on
 *  the output signal if the positive input is greater than that of the
 *  negative input terminal. This output signal can be configured in either
 *  inverted or non-inverted based off the user configuration.
 *
 *  Device specific capabilities such as output analog filters and programmable
 *  hysteresis are configured via the implementation specific hwAttrs
 *  configuration structure. This top level driver provides all APIs needed to
 *  provide a uniform and generic Comparator driver experience regardless of the
 *  underlying peripheral implementation.
  *
 *  <hr>
 *  @anchor ti_drivers_Comparator_Usage
 *  # Usage
 *
 *  This documentation provides a basic @ref ti_drivers_Comparator_Synopsis
 *  "usage summary" and a set of @ref ti_drivers_Comparator_Examples "examples"
 *  in the form of commented code fragments. Detailed descriptions of the
 *  APIs are provided in subsequent sections.
 *
 *  @anchor ti_drivers_Comparator_Synopsis
 *  ## Synopsis
 *
 *  The Comparator driver is used to monitor two input analog signals and
 *  generate an output if the potential of the positive input terminal is
 *  greater than that of the negative input channel. This is commonly used for
 *  power supply supervision as well as precision slope analog-to-digital
 *  conversions. The code below sets up two arbitrary input signals to
 *  the Comparator driver and handles triggers of the voltage crossing in
 *  the appropriate callback function.
 *
 *  The Comparator driver supports two distinct methods of accessing/utilizing
 *  the underlying comparator peripheral's output signal: accessing the output
 *  level dynamically and providing a user callback function.
 *
 *  Accessing the output level directly can be done by calling the
 *  Comparator_getLevel() API after the driver has been successfully opened
 *  and started. This function will return a #Comparator_OUTPUT_HIGH if the
 *  positive input terminal is more positive than the negative terminal and
 *  #Comparator_OUTPUT_LOW if the reverse is true. If the output level of the
 *  comparator peripheral cannot be determined or the device is in an error
 *  state a value of #Comparator_OUTPUT_NOT_AVAILABLE is returned.
 *
 *  The user callback functionality provides a way for the Comparator driver
 *  and its underlying implementation to communicate functional events as well
 *  as errors to the calling user application. If a non-null function pointer is
 *  provided to the callbackFxn configuration parameter a call to the callback
 *  will be invoked whenever a relevant event occurs in the driver. Primarily
 *  this callback will be invoked when the output of the comparator is
 *  triggered (or inversely triggered), however error events can also be
 *  passed through this callback.
 *
 *  Note that these programming models are compatible with each other and can
 *  be mixed accordingly. For example a user callback can be provided and the
 *  Comparator_getLevel() API can be called in any scenario.
 *
 *  @anchor ti_drivers_Comparator_Synopsis_Code
 *
 *  @code
 *
 *  #include <ti/drivers/Comparator.h>
 *
 *  void comparatorCallback(Comparator_Handle handle, int_fast16_t status)
 *  {
 *      switch (status)
 *      {
 *      case Comparator_EVENT_OUTPUT_TRIGGERED:
 *          // Output triggered
 *          sem_post(&someSemaphore);
 *          break;
 *      case Comparator_EVENT_OUTPUT_INVERTED_TRIGGERED:
 *          // Output triggered in the other direction
 *          sem_post(&someSemaphore);
 *          break;
 *      case Comparator_EVENT_ERROR:
 *          __breakpoint(0);
 *          break;
 *      default:
 *          break;
 *      }
 *  }
 *
 *  void someComparatorFunction()
 *  {
 *      Comparator_Handle    handle;
 *      Comparator_Params    params;
 *
 *      Comparator_Params_init(&params);
 *      params.callbackFxn = &comparatorCallback;
 *      params.interruptLevel = Comparator_INTERRUPT_RISING;
 *
 *      handle = Comparator_open(CONFIG_COMPARATOR0, &params);
 *
 *      if (handle == NULL)
 *      {
 *          //Comparator_open() failed
 *          while(1);
 *      }
 *
 *      status = Comparator_start(handle);
 *
 *      if (status == Comparator_STATUS_ERROR)
 *      {
 *          //Comparator_start() failed
 *          while(1);
 *      }
 *
 *      // Waiting for some output event to signal from the callback
 *      sem_wait(&someSemaphore);
 *
 *      Comparator_stop(handle);
 *  }
 *
 *  @endcode
 *
 *  Note that while the code above operates with a callback function provided,
 *  if a NULL value is given as the callbackFxn parameter the
 *  Comparator_getLevel() function can be invoked to dynamically get the output
 *  level of an initialized/started Comparator driver instance.
 *
 *  <hr>
 *  @anchor ti_drivers_Comparator_Examples
 *  # Examples
 *
 *  @li @ref ti_drivers_Comparator_Examples_open "Opening a Comparator instance"
 *
 *  @anchor ti_drivers_Comparator_Examples_open
 *  ## Opening a Comparator instance
 *
 *  @code
 *  Comparator_Handle comparator;
 *  Comparator_Params params;
 *
 *  Comparator_Params_init(&params);
 *  comparator = Comparator_open(CONFIG_COMP0, &params);
 *  if (comparator == NULL)
 *  {
 *      // Comparator_open() failed
 *      while (1);
 *  }
 *  @endcode
 *
 *  <hr>
 *  @anchor ti_drivers_Comparator_Configuration
 *  # Configuration
 *
 *  Refer to the @ref driver_configuration "Driver's Configuration" section
 *  for driver configuration information.
 *  <hr>
 *
 *******************************************************************************
 */

#ifndef ti_drivers_Comparator__include
#define ti_drivers_Comparator__include

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*!
 *  @defgroup Comparator_STATUS Comparator status codes to be used with
 *  Comparator.h APIs.
 *  @{
 */

/*!
 * Common Comparator_control status code reservation offset.
 * Comparator driver implementations should offset status codes with
 * Comparator_STATUS_RESERVED growing negatively.
 *
 * Example implementation specific status codes:
 * @code
 * #define ComparatorXYZ_STATUS_ERROR0     Comparator_STATUS_RESERVED - 0
 * #define ComparatorXYZ_STATUS_ERROR1     Comparator_STATUS_RESERVED - 1
 * @endcode
 */
#define Comparator_STATUS_RESERVED         (-32)

/*!
 * @brief   Successful status code.
 *
 */
#define Comparator_STATUS_SUCCESS          (0)

/*!
 * @brief   Generic error status code.
 *
 */
#define Comparator_STATUS_ERROR            (-1)
/** @}*/

/*!
 *  @defgroup Comparator_EVENT event codes that can be passed into the event
 *  parameter of the driver callback.
 *  @{
 */

/*!
 * Common Comparator event code reservation offset.
 * Comparator driver implementations should offset event codes with
 * Comparator_EVENT_RESERVED growing positively.
 *
 * Example implementation specific event codes:
 * @code
 * #define ComparatorXYZ_EVENT_EVENT0   Comparator_EVENT_RESERVED + 0
 * #define ComparatorXYZ_EVENT_EVENT1   Comparator_EVENT_RESERVED + 1
 * @endcode
 */
#define Comparator_EVENT_RESERVED                    (32)

/*!
 * @brief   The comparator output triggered in the positive direction
 *
 * Callback function called with Comparator_EVENT_OUTPUT_TRIGGERED if the
 * comparator triggered in the positive direction.
 * @{
 * @ingroup Comparator_EVENT
 */
#define Comparator_EVENT_OUTPUT_TRIGGERED             (1)

/*!
 * @brief   The comparator output triggered in the inverted direction
 *
 * Callback function called with Comparator_EVENT_OUTPUT_INVERTED_TRIGGERED if
 * the comparator triggered in the negative direction.
 */
#define Comparator_EVENT_OUTPUT_INVERTED_TRIGGERED    (2)

/*!
 * @brief   An error occurred with the underlying comparator driver
 *
 * Callback function called with Comparator_EVENT_ERROR if an error happened
 * in the operation of the driver.
 */
#define Comparator_EVENT_ERROR                        (-1)

/** @}*/

/** @}*/

/*!
 *  @brief  A handle that is returned from a Comparator_open() call.
 */
typedef struct Comparator_Config_ *Comparator_Handle;

/*!
 *  @brief Comparator Output Polarity
 *
 *  This enum defines the polarity of the comparator output (inverted/normal)
 *
 */
typedef enum
{
    Comparator_OUTPUT_NORMAL,   /*!< Comparator output is normal (output
                                     logical one when the positive input
                                     terminal is more positive than
                                     the negative input terminal) */
    Comparator_OUTPUT_INVERTED, /*!< Comparator output is normal (output
                                     logical zero when the positive input
                                     terminal is more positive than the
                                     negative input terminal) */
} Comparator_OutputPolarity;

/*!
 *  @brief Comparator Output Level
 *
 *  This enum defines the level of the output. This level has a direct
 *  correlation to the by the outputPolarity parameter passed into the
 *  #Comparator_Params while opening the device with Comparator_open().
 *
 */
typedef enum
{
    Comparator_OUTPUT_HIGH,           /*!< Positive input terminal is more
                                           positive than the negative
                                           input terminal */
    Comparator_OUTPUT_LOW,            /*!< Negative input terminal is more
                                           positive than the positive
                                           input terminal */
    Comparator_OUTPUT_NOT_AVAILABLE, /*!< Output level cannot be determined
                                          (possibly due to error or it not
                                          being ready) */
} Comparator_OutputLevel;

/*!
 *  @brief  Comparator Interrupt Level
 *
 *  This enum defines the comparator output condition that will trigger an
 *  interrupt. Not all output conditions are supported on all devices, refer
 *  to the device documentation to see which interrupt levels are supported.
 */
typedef enum
{
    Comparator_INTERRUPT_NONE,          /*!< The comparator module should not
                                             trigger an interrupt */
    Comparator_INTERRUPT_RISING,        /*!< A rising edge on the comparator
                                             output will trigger an interrupt */
    Comparator_INTERRUPT_FALLING,       /*!< A falling edge on the comparator
                                             output will trigger an interrupt */
    Comparator_INTERRUPT_BOTH,          /*!< Either a rising or falling edge on
                                             the comparator output will trigger
                                             an interrupt */
    Comparator_INTERRUPT_HIGH,          /*!< A high comparator output will
                                             trigger an interrupt */
    Comparator_INTERRUPT_LOW            /*!< A low comparator output will
                                             trigger an interrupt */
} Comparator_InterruptLevel;

/*!
 *  @brief  Comparator callback function
 *
 *  Callback from the Comparator driver. This will be called when the driver is
 *  setup with a callback function pointer provided to the
 *  #Comparator_Params.callbackFxn parameter and is used to communicate events
 *  and errors to the calling application.
 *
 *  @param[out]  handle       #Comparator_Handle
 *
 *  @param[out]  status       Status of the comparator driver during an
 *                            interrupt
 */
typedef void (*Comparator_CallBackFxn)(Comparator_Handle handle,
                                       int_fast16_t status);

/*!
 *  @brief Comparator Parameters
 *
 *  Comparator parameters are used by the Comparator_open() call. Default values
 *  for these parameters are set using Comparator_Params_init().
 *
 */
typedef struct
{
    /*! Callback function called when events happen in the Comparator driver */
    Comparator_CallBackFxn callbackFxn;

    /*! Output polarity of the Comparator module (inverted/normal) */
    Comparator_OutputPolarity outputPolarity;

    /*! Interrupt level of the Comparator module (edge or level triggers) */
    Comparator_InterruptLevel interruptLevel;

} Comparator_Params;

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              Comparator_close().
 */
typedef void (*Comparator_CloseFxn)(Comparator_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              Comparator_getLevel().
 */
typedef uint32_t (*Comparator_GetLevelFxn)(Comparator_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              Comparator_init().
 */
typedef void (*Comparator_InitFxn)(Comparator_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              Comparator_open().
 */
typedef Comparator_Handle (*Comparator_OpenFxn)(Comparator_Handle handle,
                                                Comparator_Params *params);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              Comparator_start().
 */
typedef int_fast16_t (*Comparator_StartFxn)(Comparator_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              Comparator_stop().
 */
typedef void (*Comparator_StopFxn)(Comparator_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              Comparator_getParams().
 */
typedef int_fast16_t (*Comparator_GetParamsFxn)(Comparator_Handle handle,
                                                Comparator_Params *params);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              Comparator_setParams().
 */
typedef int_fast16_t (*Comparator_SetParamsFxn)(Comparator_Handle handle,
                                                Comparator_Params *params);

/*!
 *  @brief      The definition of a comparator function table that contains the
 *              required set of functions to control a specific comparator
 *              driver implementation.
 */
typedef struct
{
    /*! Function to close the specified peripheral. */
    Comparator_CloseFxn closeFxn;

    /*! Function to get comparator output level. */
    Comparator_GetLevelFxn getLevelFxn;

    /*! Function to initialize the given data object. */
    Comparator_InitFxn initFxn;

    /*! Function to open the specified peripheral. */
    Comparator_OpenFxn openFxn;

    /*! Function to start the specified peripheral. */
    Comparator_StartFxn startFxn;

    /*! Function to stop the specified peripheral. */
    Comparator_StopFxn stopFxn;

    /*! Function to get the parameters of the specified peripheral */
    Comparator_GetParamsFxn getParamsFxn;

    /*! Function to set the parameters of the specified peripheral */
    Comparator_SetParamsFxn setParamsFxn;
} Comparator_FxnTable;

/*!
 *  @brief Comparator driver's custom @ref driver_configuration "configuration"
 *  structure.
 *
 *  @sa     Comparator_init()
 */
typedef struct Comparator_Config_
{
    /*! Pointer to a @ref driver_function_table "function pointer table"
     *  with driver-specific implementations of Comparator APIs */
    Comparator_FxnTable const *fxnTablePtr;

    /*! Pointer to a driver specific @ref driver_objects "data object". */
    void *object;

    /*! Pointer to a driver specific @ref driver_hardware_attributes
     *  "hardware attributes structure". */
    void const *hwAttrs;
} Comparator_Config;

/*!
 *  @brief  Function to close a Comparator driver instance.
 *
 *  @pre    Comparator_open() has been called.
 *
 *  @param[in]  handle  A #Comparator_Handle returned from Comparator_open().
 *
 *  @sa     Comparator_open()
 */
extern void Comparator_close(Comparator_Handle handle);

/*!
 *  @brief  Function performs implementation specific features on a given
 *          #Comparator_Handle.
 *
 *  @pre    Comparator_open() has been called.
 *
 *  @param[in]  handle   A #Comparator_Handle returned from Comparator_open().
 *  @param[in]  cmd      A command value defined by the driver specific
 *                       implementation.
 *  @param[in]  arg      A pointer to an optional R/W (read/write) argument that
 *                       is accompanied with cmd.
 *
 *  @return Implementation specific return codes. Negative values indicate
 *          unsuccessful operations.
 */
extern int_fast16_t Comparator_control(Comparator_Handle handle,
                                       uint_fast16_t cmd,
                                       void *arg);

/*!
 *  @brief  Function returns the level of the comparator output. The output
 *          level correlates to the polarity specified by the parameter passed
 *          into the #Comparator_Params.outputPolarity parameter while
 *          opening the device with Comparator_open().
 *
 *  @pre    Comparator_open() and Comparator_start() have been called.
 *
 *  @param[in] handle      A Comparator handle returned from Comparator_open().
 *
 *  @return A value of type #Comparator_OutputLevel describing the output state
 *          of the comparator peripheral.
 *
 *  @sa     Comparator_start()
 */
extern Comparator_OutputLevel Comparator_getLevel(Comparator_Handle handle);

/*!
 *  @brief  Function to initialize the Comparator driver
 *
 *  This function must also be called before any other Comparator driver APIs.
 */
extern void Comparator_init(void);

/*!
 *  @brief  Function to initialize the Comparator peripheral
 *
 *  Function to initialize the Comparator peripheral specified by the
 *  particular index value.
 *
 *  @pre    Comparator_init() has been called.
 *
 *  @param[in]  index     Index in the @p Comparator_Config[] array.
 *  @param[in]  params    Pointer to an initialized #Comparator_Params
 *                        structure.
 *                        If NULL, the default #Comparator_Params values are
 *                        used.
 *
 *  @return A #Comparator_Handle on success or NULL on an error.
 *
 *  @sa     Comparator_init()
 *  @sa     Comparator_close()
 */
extern Comparator_Handle Comparator_open(uint32_t index,
                                         Comparator_Params *params);

/*!
 *  @brief  Function to initialize the #Comparator_Params structure to
 *          its default values.
 *
 *  @param[in]  params  A pointer to #Comparator_Params structure for
 *                      initialization.
 *
 *  Defaults values are:
 *      @arg #Comparator_Params.callbackFxn = NULL
 *      @arg #Comparator_Params.outputPolarity = Comparator_OUTPUT_NORMAL
 */
extern void Comparator_Params_init(Comparator_Params *params);

/*!
 *  @brief  Function to start the comparator instance.
 *
 *  @pre    Comparator_open() has been called.
 *
 *  @param[in]  handle  A #Comparator_Handle returned from Comparator_open().
 *
 *  @retval #Comparator_STATUS_SUCCESS The comparator successfully started.
 *  @retval #Comparator_STATUS_ERROR  The comparator failed to start.
 *
 *  @sa     Comparator_stop()
 */
extern int_fast16_t Comparator_start(Comparator_Handle handle);

/*!
 *  @brief  Function to stop a comparator instance. If the comparator instance
 *          is already stopped this function has no effect.
 *
 *  @pre    Comparator_open() has been called.
 *
 *  @param[in]  handle  A #Comparator_Handle returned from Comparator_open().
 *
 *  @sa     Comparator_start()
 *
 */
extern void Comparator_stop(Comparator_Handle handle);

/*!
 *  @brief  Function to get the parameters of a comparator instance.
 *
 *  @pre    Comparator_open() has been called
 *
 *  @param[in]  handle  A #Comparator_Handle returned from Comparator_open().
 *  @param[in]  params  A pointer to a #Comparator_Params structure to be
 *                      filled with the current comparator settings.
 *
 *  @retval #Comparator_STATUS_SUCCESS The params were obtained successfully.
 *  @retval #Comparator_STATUS_ERROR The params were unable to be obtained.
 *
 *  @sa     Comparator_getParams()
 */
extern int_fast16_t Comparator_getParams(Comparator_Handle handle,
                                         Comparator_Params *params);

/*!
 *  @brief  Function to get the parameters of a comparator instance.
 *
 *  @pre    Comparator_open() has been called
 *
 *  @param[in]  handle  A #Comparator_Handle returned from Comparator_open().
 *  @param[in]  params  A pointer to a #Comparator_Params structure holding
 *                      the new comparator settings.
 *
 *  @retval #Comparator_STATUS_SUCCESS The params were set successfully.
 *  @retval #Comparator_STATUS_ERROR The params were unable to be set.
 *
 *  @sa     Comparator_setParams()
 */
extern int_fast16_t Comparator_setParams(Comparator_Handle handle,
                                         Comparator_Params *params);

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_Comparator__include */
