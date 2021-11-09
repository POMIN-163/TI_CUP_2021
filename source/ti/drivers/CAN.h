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
/*!*****************************************************************************
 *  @file       CAN.h
 *  @brief      <b>PRELIMINARY</b> CAN driver interface
 *
 *  <b>WARNING</b> These APIs are <b>PRELIMINARY</b>, and subject to
 *  change in the next few months.
 *
 *  To use the CAN driver, ensure that the correct driver library for your
 *  device is linked in and include this header file as follows:
 *  @code
 *  #include <ti/drivers/CAN.h>
 *  @endcode
 *
 *  This module serves as the main interface for applications.  Its purpose
 *  is to redirect the CAN APIs to specific driver implementations
 *  which are specified using a pointer to a #CAN_FxnTable.
 *
 *  @anchor ti_drivers_CAN_Overview
 *  # Overview #
 *  The Controller Area Network (CAN) driver is a generic driver that allows
 *  for communication on a CAN bus. It is a two-wire, half-duplex, LAN system
 *  that is collision free. The main method of transfer is by broadcasting.
 *  The CAN protocol defines the format of data transfer, and this CAN driver
 *  allows full functionality as a transmitting and receiving node on a bus.
 *  However, there can be higher-level software layers and stacks that use this
 *  driver to enable more advanced features.
 *  Functional modes available in this driver include blocking and non-blocking.
 *
 *  The APIs in this driver serve as an interface to a typical RTOS
 *  application. The specific peripheral implementations are responsible for
 *  creating all the RTOS specific primitives to allow for thread-safe
 *  operation.
 *
 *  @anchor ti_drivers_CAN_Usage
 *  # Usage #
 *
 *  The CAN driver interface provides device independent APIs, data types,
 *  and macros.
 *
 *  @anchor ti_drivers_CAN_Synopsis
 *  ## Synopsis #
 *  The following code example opens a CAN instance, creates
 *  an incrementing CAN frame, and continually writes them to the CAN bus.
 *  NOTE: a CAN receiver on this bus is needed, or else this transmitter will
 *  continually throw an error if it does not detect an ACK.
 *
 *  @code
 *    uint8_t i;
 *    // Initialize the CAN driver
 *    CAN_init();
 *
 *    CAN_Handle canHandle;
 *    CAN_Params canParams;
 *    CAN_Params_init(&canParams);
 *    canHandle = CAN_open(CONFIG_CAN0, &canParams);
 *
 *    if (canHandle == NULL) {
 *        // CAN_open() failed
 *        while (1);
 *    }
 *
 *    for (i = 0; ; ++i) {
 *        CAN_Frame canFrame[1];
 *        canFrame[0].can_id = i;
 *        canFrame[0].err = 0;
 *        canFrame[0].rtr = 0;
 *        canFrame[0].eff = 1;
 *        canFrame[0].dlc = i % 9;
 *        canFrame[0].data[0] = i;
 *        canFrame[0].data[1] = i + 1;
 *        canFrame[0].data[2] = i + 2;
 *        canFrame[0].data[3] = i + 3;
 *        canFrame[0].data[4] = i + 4;
 *        canFrame[0].data[5] = i + 5;
 *        canFrame[0].data[6] = i + 6;
 *        canFrame[0].data[7] = i + 7;
 *
 *        CAN_write(canHandle, canFrame, sizeof(canFrame));
 *    }
 *  @endcode
 *
 *  Details for the example code above are described in the following
 *  subsections.
 *
 *
 *  @anchor ti_drivers_CAN_Configuration
 *  ### CAN Driver Configuration #
 *
 *  In order to use the CAN APIs, the application is required to
 *  provide device-specific CAN configuration in the ti_drivers_config.c file.
 *  The CAN driver interface defines a configuration data structure:
 *
 *  @code
 *  typedef struct {
 *      CAN_FxnTable  const    *fxnTablePtr;
 *      void                   *object;
 *      void          const    *hwAttrs;
 *      CAN_Frame              *rxBufPtr;
 *      CAN_frame              *txBufPtr;
 *      size_t                  rxBufSize;
 *      size_t                  txBufSize;
 *  } CAN_Config;
 *  @endcode
 *
 *  You will need to check the device-specific CAN driver implementation's
 *  header file for example configuration.  Please also refer to the
 *  ti_drivers_config.c file to see the CAN configuration.
 *
 *  ### Initializing the CAN Driver #
 *
 *  CAN_init() must be called before any other CAN APIs.  This function
 *  calls the device implementation's CAN initialization function, for each
 *  element of CAN_config[].
 *
 *  ### Opening the CAN Driver #
 *
 *  Opening a CAN requires four steps:
 *  1.  Create and initialize a CAN_Params structure.
 *  2.  Fill in the desired parameters.
 *  3.  Call CAN_open(), passing the index of the CAN in the CAN_config
 *      structure, and the address of the CAN_Params structure.  The
 *      CAN instance is specified by the index in the CAN_config structure.
 *  4.  Check that the CAN handle returned by CAN_open() is non-NULL,
 *      and save it.  The handle will be used to read and write to the
 *      CAN you just opened.
 *
 *  Only one CAN index can be used at a time; calling CAN_open() a second
 *  time with the same index previously passed to CAN_open() will result in
 *  an error.  You can, though, re-use the index if the instance is closed
 *  via CAN_close().
 *  In the example code, CONFIG_CAN0 is passed to CAN_open(). This macro
 *  is defined in the applications "ti_drivers_config.h" file.
 *
 *
 *  ### Modes of Operation #
 *
 *  The CAN driver can operate in blocking mode or nonblocking mode, by
 *  setting the mode parameters passed to CAN_open().
 *  If these parameters are not set, as in the example code, the CAN
 *  driver defaults to blocking mode.  Options for the mode parameter are
 *  #CAN_MODE_BLOCKING and #CAN_MODE_NONBLOCKING:
 *
 *  - #CAN_MODE_BLOCKING uses a semaphore to block while data is being sent
 *    or read. The context of calling CAN_read() or CAN_write() must be a Task
 *    when using #CAN_MODE_BLOCKING. The CAN_write() or CAN_read() call
 *    will block until all data is sent or received, or the write timeout or
 *    read timeout expires, whichever happens first.
 *
 *  - #CAN_MODE_NONBLOCKING is non-blocking and CAN_read() and CAN_write()
 *    will return either with the number of bytes successfully read/written,
 *    or a negative error number.
 *
 *  ### Reading and Writing data #
 *
 *  The example code reads one CAN frame from the CAN instance, and then writes
 *  one CAN frame back to the same instance:
 *
 *  @code
 *  CAN_read(can, &canFrame, sizeof(canFrame));
 *  CAN_write(can, &canFrame, sizeof(canFrame));
 *  @endcode
 *
 *  The CAN driver allows CAN_read() and CAN_write() calls to happen for any
 *  node at any time from the CAN bus. Please see the CAN protocol for how it
 *  handles collisions. The ability to filter incoming messages are also
 *  available through CAN_Params.
 *
 *  # Implementation #
 *
 *  The CAN driver interface module is joined (at link time) to an
 *  array of CAN_Config data structures named *CAN_config*.
 *  CAN_config is implemented in the application with each entry being an
 *  instance of a CAN peripheral. Each entry in *CAN_config* contains a:
 *  - (CAN_FxnTable *) to a set of functions that implement a CAN peripheral
 *  - (void *) data object that is associated with the CAN_FxnTable
 *  - (void *) hardware attributes that are associated with the CAN_FxnTable
 *
 *  The CAN APIs are redirected to the device specific implementations
 *  using the CAN_FxnTable pointer of the CAN_config entry.
 *  In order to use device specific functions of the CAN driver directly,
 *  link in the correct driver library for your device and include the
 *  device specific CAN driver header file (which in turn includes CAN.h).
 *  For example, for the MSP432 family of devices, you would include the
 *  following header file:
 *    @code
 *    #include <ti/drivers/can/CANMSP432.h>
 *    @endcode
 *
 *  ============================================================================
 */

#ifndef ti_drivers_CAN__include
#define ti_drivers_CAN__include

#include <stddef.h>
#include <stdint.h>

#include <ti/drivers/can/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  @defgroup CAN_CONTROL CAN_control command and status codes
 *  These CAN macros are reservations for CAN.h
 *  @{
 */

/*!
 * Common CAN_control command code reservation offset.
 * CAN driver implementations should offset command codes with
 * CAN_CMD_RESERVED growing positively
 *
 * Example implementation specific command codes:
 * @code
 * #define CANXYZ_CMD_COMMAND0     CAN_CMD_RESERVED + 0
 * #define CANXYZ_CMD_COMMAND1     CAN_CMD_RESERVED + 1
 * @endcode
 */
#define CAN_CMD_RESERVED           (32)

/*!
 * Common CAN_control status code reservation offset.
 * CAN driver implementations should offset status codes with
 * CAN_STATUS_RESERVED growing negatively.
 *
 * Example implementation specific status codes:
 * @code
 * #define CANXYZ_STATUS_ERROR0    CAN_STATUS_RESERVED - 0
 * #define CANXYZ_STATUS_ERROR1    CAN_STATUS_RESERVED - 1
 * #define CANXYZ_STATUS_ERROR2    CAN_STATUS_RESERVED - 2
 * @endcode
 */
#define CAN_STATUS_RESERVED        (-32)

/**
 *  @defgroup CAN_STATUS Status Codes
 *  CAN_STATUS_* macros are general status codes returned by CAN_control()
 *  @{
 *  @ingroup CAN_CONTROL
 */

/*!
 * @brief   Successful status code returned by CAN_control().
 *
 * CAN_control() returns CAN_STATUS_SUCCESS if the control code was executed
 * successfully.
 */
#define CAN_STATUS_SUCCESS         (0)

/*!
 * @brief   Generic error status code returned by CAN_control().
 *
 * CAN_control() returns CAN_STATUS_ERROR if the control code was not executed
 * successfully.
 */
#define CAN_STATUS_ERROR           (-1)

/*!
 * @brief   An error status code returned by CAN_control() for undefined
 * command codes.
 *
 * CAN_control() returns CAN_STATUS_UNDEFINEDCMD if the control code is not
 * recognized by the driver implementation.
 */
#define CAN_STATUS_UNDEFINEDCMD    (-2)
/** @}
 *  @}*/

/*!
 *  @brief    Wait forever define
 */
#define  CAN_WAIT_FOREVER           (~(0U))

/*!
 *  @brief      A handle that is returned from a CAN_open() call.
 */
typedef struct CAN_Config_    *CAN_Handle;

/*!
 *  @brief      CAN mode settings
 *
 *  This enum defines the read, write, and blocking modes for the configured
 *  CAN.
 */
typedef enum {
    /*!
      *  Blocking and will return only when at least one CAN frame has been
      *  processed by CAN_write() or CAN_read() data, or if the optional
      *  timeout occurs.
      */
    CAN_MODE_BLOCKING,
    /*!
      *  Non-blocking and will return immediately with or without write
      *  or read data. Error flags could be thrown if invalid CAN_write()
      *  or CAN_read().
      */
    CAN_MODE_NONBLOCKING
} CAN_Mode;

/*!
 *  @brief      CAN communication mode
 *
 *  This enum defines read or write communication direction for the configured
 *  CAN.
 */
typedef enum {
    /*! Read only mode, a transmit object is not used */
    CAN_DIRECTION_READ        = 0x1,
    /*! Write only mode, a receive object is not used */
    CAN_DIRECTION_WRITE       = 0x2,
    /*! Read and write mode.  A single transmit object is used and at least one
     *  receive object is used.
     *  NOTE: enum value must be logical OR of CAN_DIRECTION_READ and CAN_DIRECTION_WRITE
     */
    CAN_DIRECTION_READWRITE   = 0x3,
} CAN_Direction;

/*!
 *  @brief    CAN Parameters
 *
 *  CAN parameters are used with the CAN_open() call. Default values for
 *  these parameters are set using CAN_Params_init().
 *
 *  @sa       CAN_Params_init()
 */
typedef struct {
    CAN_Mode  mode;         /*!< Mode to open device in */
    CAN_Direction  direction; /*!< Read, write, or readwrite mode */
    uint32_t  filterID;     /*!< Initial receive filter ID */
    uint32_t  filterMask;   /*!< Initial receive filter mask */
    uint32_t  readTimeout;  /*!< Timeout for read calls in blocking mode. */
    uint32_t  writeTimeout; /*!< Timeout for write calls in blocking mode. */
} CAN_Params;

/*!
 * @brief    CAN frame structure
 *
 * The structure that makes up a CAN message.  The unions are provided in order
 * for there to be structural naming compatibility with SocketCAN while
 * at the same time providing an alternative easier to use naming convention.
 * We diverge a bit with TI structural naming convention of the struct in order
 * to provide an option to be compatible with SocketCAN conventions.
 *
 * @sa       CAN_write()
 * @sa       CAN_read()
 */
typedef struct can_frame CAN_Frame;

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              CAN_CloseFxn().
 */
typedef void (*CAN_CloseFxn) (CAN_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              CAN_ControlFxn().
 */
typedef int_fast16_t (*CAN_ControlFxn) (CAN_Handle handle, uint_fast16_t cmd, void *arg);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              CAN_InitFxn().
 */
typedef void (*CAN_InitFxn) (CAN_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              CAN_OpenFxn().
 */
typedef CAN_Handle (*CAN_OpenFxn) (CAN_Handle handle, CAN_Params *params);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              CAN_ReadFxn().
 */
typedef int_fast32_t (*CAN_ReadFxn) (CAN_Handle handle, void *buffer,
    size_t size);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              CAN_WriteFxn().
 */
typedef int_fast32_t (*CAN_WriteFxn) (CAN_Handle handle, const void *buffer,
    size_t size);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              CAN_TxMsgFxn().
 */
typedef void (*CAN_TxMsgFxn) (CAN_Handle handle);

/*!
 *  @brief      The definition of a CAN function table that contains the
 *              required set of functions to control a specific CAN driver
 *              implementation.
 */
typedef struct {
    /*! Function to close the specified peripheral */
    CAN_CloseFxn        closeFxn;

    /*! Function to implementation specific control function */
    CAN_ControlFxn      controlFxn;

    /*! Function to initialize the given data object */
    CAN_InitFxn         initFxn;

    /*! Function to open the specified peripheral */
    CAN_OpenFxn         openFxn;

    /*! Function to read from the specified peripheral */
    CAN_ReadFxn         readFxn;

    /*! Function to write from the specified peripheral */
    CAN_WriteFxn        writeFxn;

    /*! Function to kick the transmitter */
    CAN_TxMsgFxn        txMsgFxn;
} CAN_FxnTable;

/*!
 *  @brief  CAN Global configuration
 *
 *  The CAN_Config structure contains a set of pointers used to characterize
 *  the CAN driver implementation.
 *
 *  This structure needs to be defined before calling CAN_init() and it must
 *  not be changed thereafter.
 *
 *  @sa     CAN_init()
 */
typedef struct CAN_Config_ {
    /*! Pointer to a table of driver-specific implementations of CAN APIs */
    CAN_FxnTable   const *fxnTablePtr;

    /*! Pointer to a driver specific data object */
    void                 *object;

    /*! Pointer to a driver specific hardware attributes structure */
    void           const *hwAttrs;

    /*! Pointer to an application RX ring buffer */
    CAN_Frame            *rxBufPtr;

    /*! Pointer to an application TX ring buffer */
    CAN_Frame            *txBufPtr;

    /*! Size of rxBufPtr in bytes*/
    size_t                rxBufSize;

    /*! Size of txBufPtr in bytes */
    size_t                txBufSize;
} CAN_Config;

/*!
 *  @brief  Function to close a CAN peripheral specified by the CAN handle
 *
 *  @pre    CAN_open() has been called.
 *  @pre    No active CAN_read() or CAN_write() call.
 *
 *  @param  handle      A #CAN_Handle returned from CAN_open()
 *
 *  @sa     CAN_open()
 */
extern void CAN_close(CAN_Handle handle);

/*!
 *  @brief  Function to initialize the CAN module
 *
 *  @pre    The CAN_config structure must exist and be persistent before this
 *          function can be called. This function must also be called before
 *          any other CAN driver APIs.
 */
extern void CAN_init(void);

/*!
 *  @brief  Function performs implementation specific features on a given
 *          #CAN_Handle.
 *
 *  Commands for %CAN_control() can originate from CAN.h or from implementation
 *  specific CAN*.h (_CANTCAN2550.h_, _CANMSP432.h_, etc.. ) files.
 *  While commands from CAN.h are API portable across driver implementations,
 *  not all implementations may support all these commands.
 *  Conversely, commands from driver implementation specific CAN*.h files add
 *  unique driver capabilities but are not API portable across all CAN driver
 *  implementations.
 *
 *  Commands supported by CAN.h follow a CAN_CMD_\<cmd\> naming
 *  convention.<br>
 *  Commands supported by CAN*.h follow a CAN*_CMD_\<cmd\> naming
 *  convention.<br>
 *  Each control command defines @b arg differently. The types of @b arg are
 *  documented with each command.
 *
 *  @pre    CAN_open() has to be called.
 *
 *  @param  handle      A CAN handle returned from CAN_open()
 *
 *  @param  cmd         CAN.h or CAN*.h commands.
 *
 *  @param  arg         An optional R/W (read/write) command argument
 *                      accompanied with cmd
 *
 *  @return Implementation specific return codes. Negative values indicate
 *          unsuccessful operations.
 *
 *  @sa     CAN_open()
 */
extern int_fast16_t CAN_control(CAN_Handle handle, uint_fast16_t cmd, void *arg);

/*!
 *  @brief  Function to initialize a given CAN peripheral
 *
 *  Function to initialize a given CAN peripheral specified by the
 *  particular index value.
 *
 *  @pre    CAN_init() has been called
 *
 *  @param  index         Logical peripheral number for the CAN indexed into
 *                        the CAN_config table
 *
 *  @param  params        Pointer to a parameter block. If NULL, default
 *                        parameter values will be used. All the fields in
 *                        this structure are RO (read-only).
 *
 *  @return A #CAN_Handle upon success. NULL if an error occurs, or if the
 *          indexed CAN peripheral is already opened.
 *
 *  @sa     CAN_init()
 *  @sa     CAN_close()
 */
extern CAN_Handle CAN_open(uint_least8_t index, CAN_Params *params);

/*!
 *  @brief  Function to initialize the CAN_Params struct to its defaults
 *
 *  @param  params      An pointer to CAN_Params structure for
 *                      initialization
 *
 *  Defaults values are:
 *      mode = CAN_MODE_BLOCKING;
 *      direction = CAN_DIRECTION_READWRITE;
 *      readTimeout = CAN_WAIT_FOREVER;
 *      writeTimeout = CAN_WAIT_FOREVER;
 *      baudRate = 125000;
 */
extern void CAN_Params_init(CAN_Params *params);

/*!
 *  @brief  Function that writes data to a CAN with interrupts enabled.
 *
 *  %CAN_write() writes data from a memory buffer to the CAN interface.
 *  The source is specified by \a buffer and the number of bytes to write
 *  is given by \a size.
 *
 *  In #CAN_MODE_BLOCKING, CAN_write() blocks task execution until at least
 *  one CAN frame data in buffer has been written.
 *
 *  In #CAN_MODE_NONBLOCKING, %CAN_write() returns immediately with the number
 *  of bytes (in frame size chunks) that were able to be immediately written.
 *
 *  @param  handle      A #CAN_Handle returned by CAN_open()
 *
 *  @param  buffer      A read-only pointer to buffer containing CAN frames to
 *                      be written to the CAN interface (#CAN_Frame,
 *                      struct can_frame)
 *
 *  @param  size        The number of bytes in the buffer that should be written
 *                      to the CAN interface, sizeof(CAN_frame)
 *
 *  @return Returns the number of bytes that have been written to the CAN
 *          interface. If an error occurs, a negative error number is returned.
 *          If in non-blocking mode, and if there is no more room for write
 *          data, then -EAGAIN is return.  If in blocking mode and a timeout
 *          occurs, then -ETIMEDOUT is returned.
 */
extern int_fast32_t CAN_write(CAN_Handle handle, const void *buffer, size_t size);

/*!
 *  @brief  Function that reads data from a CAN with interrupt enabled.
 *
 *  %CAN_read() reads data into a memory buffer from the CAN interface.
 *  The destination is specified by \a buffer and the number of bytes to read
 *  is given by \a size.
 *
 *  In #CAN_MODE_BLOCKING, CAN_read() blocks task execution until at least
 *  one CAN frame data can be received.
 *
 *  In #CAN_MODE_NONBLOCKING, %CAN_write() returns immediately with the number
 *  of bytes (in frame size chunks) that were able to be immediately received.
 *
 *  @param  handle      A #CAN_Handle returned by CAN_open()
 *
 *  @param  buffer      A pointer to buffer containing space for CAN frames to
 *                      be read into from the CAN interface (#CAN_Frame,
 *                      struct can_frame)
 *
 *  @param  size        The number of bytes in the buffer that can be filled
 *                      with receive data from the CAN interface,
 *                      sizeof(CAN_frame)
 *
 *  @return Returns the number of bytes that have been read from the CAN
 *          interface. If an error occurs, a negative error number is returned.
 *          If in non-blocking mode, and if there is no data to be read
 *          then -EAGAIN is returned.  If in blocking mode and a timeout
 *          occurs, then -ETIMEDOUT is returned.
 */
extern int_fast32_t CAN_read(CAN_Handle handle, void *buffer, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_CAN__include */
