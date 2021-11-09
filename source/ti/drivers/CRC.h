/*
 * Copyright (c) 2019, Texas Instruments Incorporated
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
 *  @file       CRC.h
 *  @brief      CRC driver interface
 *
 *  @anchor ti_drivers_CRC_Overview
 *  # Overview #
 *
 *  The CRC driver interface provides device independent APIs, data types,
 *  and macros. The CRC header file should be included in an application as
 *  follows:
 *  @code
 *  #include <ti/drivers/CRC.h>
 *  @endcode
 *
 *  The Cyclic Redundancy Check (CRC) driver is a generic driver that supports
 *  calculating a variety of standard CRC codes on blocks of input data.
 *
 *  <hr>
 *  @anchor ti_drivers_CRC_Usage
 *  # Usage #
 *  To calculate the CRC of a block of available data, an application should call:
 *    - CRC_init(): Initialize the CRC driver.
 *    - CRC_Params_init():  Initialize a default #CRC_Params structure.
 *    - CRC_open(): Open an instance of the CRC driver, passing the
 *      initialized parameters, or NULL, and an index (described later).
 *    - CRC_calculateFull(): Calculate the CRC of a data block.
 *    - CRC_close(): De-initialize the CRC instance.
 *
 *  If the data is only available in noncontiguous memory or is being made
 *  available in blocks (e.g. over UART) then the addData and finalise methods
 *  may be used to calculate the CRC of individual blocks.
 *
 *  The CRC driver only accepts data lengths that are a multiple of the CRC size.
 *  If asked to CRC 7 bytes with a 4-byte CRC, it will throw an error. Similarly,
 *  if padding bytes are required for a particular application this must be handled
 *  by the caller.
 *
 *  @anchor ti_drivers_CRC_Synopsis
 *  ## Synopsis
 *  @anchor ti_drivers_CRC_Synopsis_Code
 *  @code
 *  CRC_Handle      handle;
 *  CRC_Params      params;
 *  int_fast16_t    status;
 *  uint32_t        result;
 *  uint8_t         source[NUM_BYTES];
 *
 *  CRC_init();  // Initialize the CRC driver
 *
 *  CRC_Params_init(&params);     // Initialize CRC parameters
 *  params.returnBehavior = CRC_RETURN_BEHAVIOR_BLOCKING;
 *
 *  params.polynomial = CRC_POLYNOMIAL_CRC_16_CCITT;
 *  params.dataSize = CRC_DATA_SIZE_32BIT;
 *  params.seed = 0xFFFF;
 *  params.byteSwapInput = CRC_BYTESWAP_UNCHANGED;
 *
 *  handle = CRC_open(CONFIG_CRC0, &params);
 *  if (handle == NULL) {
 *      while (1);  // CRC_open() failed
 *  }
 *
 *  // Fill in source
 *
 *  status = CRC_calculateFull(handle, source, NUM_BYTES, &result);
 *
 *  if (status != CRC_STATUS_SUCCESS) {
 *      // Error with parameters, or CRC resource was unavailable
 *      while (1);
 *  }
 *  @endcode
 *
 *  More details on usage are provided in the following sections.
 *
 *  <hr>
 *  @anchor ti_drivers_CRC_Examples
 *  # Examples
 *
 *  @li @ref ti_drivers_CRC_Examples_open "Opening a CRC instance"
 *  @li @ref ti_drivers_CRC_Examples_calculateFull "Calculating a full check value"
 *  @li @ref ti_drivers_CRC_Examples_calculatePartial "Partial check calculations"
 *
 *  @anchor ti_drivers_CRC_Examples_open
 *  ## Opening a CRC instance
 *  After initializing the CRC driver by calling CRC_init(), the application
 *  can open a CRC instance by calling CRC_open().  This function
 *  takes an index into the CRC_config[] array, and a CRC parameters data
 *  structure. The CRC instance is specified by the index of the CRC in
 *  CRC_config[]. Only one CRC index can be used at a time;
 *  calling CRC_open() a second time with the same index previously
 *  passed to CRC_open() will result in an error. You can,
 *  though, re-use the index if the instance is closed via CRC_close().
 *
 *  If no CRC_Params structure is passed to CRC_open(), default values are
 *  used. If the open call is successful, it returns a non-NULL value.
 *
 *  @code
 *  CRC_Handle handle;
 *  CRC_Params params;
 *
 *  // Initialize the CRC driver
 *  CRC_init();
 *
 *  // Initialize optional CRC parameters for CALLBACK mode
 *  CRC_Params_init(&params);
 *  params.returnBehavior = CRC_RETURN_BEHAVIOR_CALLBACK;
 *  params.callbackFxn = myCallbackFunction;
 *
 *  handle = CRC_open(CONFIG_CRC0, &params);
 *  if (handle == NULL) {
 *      // CRC_open() failed
 *      while (1);
 *  }
 *  @endcode
 *
 *  @anchor ti_drivers_CRC_Examples_calculateFull
 *  ## Using the calculateFull API
 *
 *  ### CRC-8-CCITT without data processing
 *  An 8-bit CRC with no data processing options. Note that the
 *  default POLLING mode uses the CPU to move data so will not
 *  allow the device to enter standby in low-power applications.
 *
 *  @code
 *  CRC_Handle      handle;
 *  int_fast16_t    status;
 *  uint8_t         source[NUM_BYTES];
 *  uint8_t         result;
 *
 *  // Initialize the CRC driver
 *  CRC_init();
 *
 *  // The defaults are set to a POLLING mode 8-bit CRC with seed 0xFF
 *  // We can pass NULL instead of a Params struct to make use of this
 *  handle = CRC_open(CONFIG_CRC0, NULL);
 *
 *  if (handle == NULL) {
 *      while (1);  // CRC_open() failed
 *  }
 *
 *  status = CRC_calculateFull(handle, source, NUM_BYTES, &result);
 *
 *  if (status != CRC_STATUS_SUCCESS) {
 *      // Error with parameters, or CRC resource was unavailable
 *      while (1);
 *  }
 *  @endcode
 *
 *  ### CRC-32-IEEE with endianness reversal
 *  A 32-bit CRC with data processing options, in BLOCKING mode.
 *
 *  @code
 *  CRC_Handle      handle;
 *  CRC_Params      params;
 *  int_fast16_t    status;
 *  uint32_t        result;
 *  uint8_t         source[NUM_BYTES];
 *
 *  CRC_init();  // Initialize the CRC driver
 *
 *  CRC_Params_init(&params);     // Initialize CRC parameters
 *  params.returnBehavior = CRC_RETURN_BEHAVIOR_BLOCKING;
 *
 *  params.byteSwapInput = CRC_BYTESWAP_BYTES_AND_HALF_WORDS;
 *  params.polynomial = CRC_POLYNOMIAL_CRC_32_IEEE;
 *  params.dataSize = CRC_DATA_SIZE_32BIT;
 *  params.seed = 0xFFFFFFFF;
 *
 *  handle = CRC_open(CONFIG_CRC0, &params);
 *
 *  if (handle == NULL) {
 *      while (1);  // CRC_open() failed
 *  }
 *
 *  // Obtain data for the source buffer
 *
 *  status = CRC_calculateFull(handle, source, NUM_BYTES, &result);
 *
 *  if (status != CRC_STATUS_SUCCESS) {
 *      // Error with parameters, or CRC resource was unavailable
 *      while (1);
 *  }
 *  @endcode
 *
 *  @anchor ti_drivers_CRC_Examples_calculatePartial
 *  ## Using the addData API
 *
 *  It may be desirable to use the CRC to calculate over blocks of data that
 *  are available at different times or non-contiguous in memory. A
 *  block-by-block API is available to do this. The following code calculates
 *  the CRC of two separate arrays as though they were concatenated.
 *
 *  @code
 *  CRC_Handle      handle;
 *  CRC_Params      params;
 *  int_fast16_t    status;
 *  uint32_t        result;
 *
 *  uint32_t         sourceA [NUM_BYTES] = { ... };
 *  uint32_t         sourceB [NUM_BYTES] = { ... };
 *
 *  CRC_init();
 *  CRC_Params_init(&params);
 *
 *  params.byteSwapInput = CRC_BYTESWAP_UNCHANGED;
 *  params.polynomial = CRC_POLYNOMIAL_CRC_32C;
 *  params.dataSize = CRC_DATA_SIZE_32BIT;
 *  params.seed = 0xFFFFFFFF;
 *
 *  handle = CRC_open(CONFIG_CRC0, &params);
 *
 *  if (handle == NULL) {
 *      while (1);  // CRC_open() failed
 *  }
 *
 *  status = CRC_addData(handle, sourceA, NUM_BYTES);
 *
 *  if (status != CRC_STATUS_SUCCESS) {
 *      // CRC resource was unavailable
 *      while (1);
 *  }
 *
 *  status = CRC_addData(handle, sourceB, NUM_BYTES);
 *
 *  if (status != CRC_STATUS_SUCCESS) {
 *      // CRC resource was unavailable
 *      while (1);
 *  }
 *
 *  CRC_finalize(handle, &result);
 *  @endcode
 *
 *******************************************************************************
 */

#ifndef ti_drivers_CRC__include
#define ti_drivers_CRC__include

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CRC_STATUS_RESERVED                 (-32)

/*! Operation completed successfully */
#define CRC_STATUS_SUCCESS                  (0)

/*! Returned for incorrect parameters */
#define CRC_STATUS_ERROR                    (-1)

/*! The handle or HW accelerator is currently busy with another operation */
#define CRC_STATUS_RESOURCE_UNAVAILABLE     (-2)

/*! Returned for polynomial (or programmable polynomial) not supported */
#define CRC_STATUS_OPERATION_NOT_SUPPORTED  (-3)

/*! Returned when the number of bytes passed is not a multiple of the data size */
#define CRC_STATUS_LEFTOVER_BYTES_PRESENT   (-4)

/*!
 *  @brief CRC Global configuration
 *
 *  The CRC_Config structure contains a set of pointers used to characterize
 *  the CRC driver implementation.
 *
 *  This structure needs to be defined before calling CRC_init() and it must
 *  not be changed thereafter.
 *
 *  @sa     CRC_init()
 */
typedef struct {
    void               *object;
    void         const *hwAttrs;
} CRC_Config;

/*!
 *  @brief  A handle that is returned from an CRC_open() call.
 */
typedef CRC_Config *CRC_Handle;

/*! @brief  The way in which CRC function calls return after completing.
 *
 * This setting controls the return behavior of CRC_calculateFull and CRC_addData.
 * These functions have restrictions on the context from which they may be called
 * (see below). All other functions return immediately.
 *
 * |                             | Task  | Hwi   | Swi   |
 * |-----------------------------|-------|-------|-------|
 * |CRC_RETURN_BEHAVIOR_CALLBACK | X     | X     | X     |
 * |CRC_RETURN_BEHAVIOR_BLOCKING | X     |       |       |
 * |CRC_RETURN_BEHAVIOR_POLLING  | X     | X     | X     |
 */
typedef enum {
    CRC_RETURN_BEHAVIOR_CALLBACK = 1,   /*!< The function call will return immediately while the
                                         *   CRC operation goes on in the background. The registered
                                         *   callback function is called after the operation completes.
                                         *   The context the callback function is called (task, HWI, SWI)
                                         *   is implementation-dependent. */
    CRC_RETURN_BEHAVIOR_BLOCKING = 2,   /*!< The function call will block while CRC operation goes
                                         *   on in the background. CRC operation results are available
                                         *   after the function returns. */
    CRC_RETURN_BEHAVIOR_POLLING  = 4,   /*!< The CPU is used to feed data to the CRC.
                                         *   CRC operation results are available after the function returns. */
} CRC_ReturnBehavior;

/* Not all polynomials are supported on all implementations; see the device specific
 * header files for details. OPERATION_NOT_SUPPORTED will be returned if an
 * unsupported polynomial is requested.
 *
 * Texas Instruments does not provide advice on polynomial suitability. The
 * availability of polynomials in this driver or on a particular device does
 * not imply fitness for any particular task. If highly reliable error
 * detection capabilities are required, please consult a domain expert
 * before choosing a polynomial.
 */
typedef enum {
    /*! CRC-8 CCITT:            polynomial 0x07 */
    CRC_POLYNOMIAL_CRC_8_CCITT,
    /*! CRC-16 CCITT:           polynomial 0x1021 */
    CRC_POLYNOMIAL_CRC_16_CCITT,
    /*! CRC-16 IBM:             polynomial 0x8005 */
    CRC_POLYNOMIAL_CRC_16_IBM,
    /*! CRC-32 IEEE/Ethernet:   polynomial 0x04C11DB7  */
    CRC_POLYNOMIAL_CRC_32_IEEE,
    /*! CRC-32C Castagnoli:     polynomial 0x1EDC6F41 */
    CRC_POLYNOMIAL_CRC_32C,
    /*! CRC-32 IO-LINK:         polynomial 0xF4ACFB13 */
    CRC_POLYNOMIAL_CRC_32_IO_LINK,

    /*! The TCP Checksum does not have a traditional polynomial, as it consists
     *  of repeated addition rather than typical long division operations. */
    CRC_POLYNOMIAL_CRC_TCP,

    /*! Some implementations support programmable polynomials. In this case,
     *  use CUSTOM_PROGRAMMABLE and set the polynomial in the programmablePoly
     *  field of the Operation struct.  */
    CRC_POLYNOMIAL_CUSTOM_PROGRAMMABLE,
} CRC_Polynomial;

/*!
 * @brief These byte swapping configurations are primarily for dealing with endianness mismatch.
 *        Not all implementations support all configurations.
 *
 * Specific configurations are only permitted for sufficiently wide data sizes:
 *  - 32-bit data size: all byte swap configurations are permitted
 *  - 16-bit data size: only UNCHANGED and BYTES_IN_HALF_WORDS are permitted
 *  - 8-bit data size: only UNCHANGED is permitted */
typedef enum {
    CRC_BYTESWAP_UNCHANGED,             /* A B C D -> A B C D */
    CRC_BYTESWAP_HALF_WORDS,            /* A B C D -> C D A B */
    CRC_BYTESWAP_BYTES_IN_HALF_WORDS,   /* A B C D -> B A D C */
    CRC_BYTESWAP_BYTES_AND_HALF_WORDS,  /* A B C D -> D C B A */
} CRC_ByteSwap;

/*!
 * @brief The CRC driver will consume data in blocks of this size.
 * Not all implementations support all sizes.
 */
typedef enum {
    CRC_DATA_SIZE_8BIT,
    CRC_DATA_SIZE_16BIT,
    CRC_DATA_SIZE_32BIT,
} CRC_DataSize;

/*!
 *  @brief  The definition of a callback function used by the CRC driver
 *          when used in CALLBACK mode
 *
 *  @param  handle Handle of the client that started the CRC operation.
 *
 *  @param  status Contains the CRC status code from the operation.
 *
 *  @param  result Contains the 8/16/32-bit result of the CRC operation.
 *                 The rest of the word (if unused) will contain zeroes.
 */
typedef void (*CRC_CallbackFxn) (CRC_Handle handle, int_fast16_t status, void *result);

/*!
 *  @brief Struct containing the parameters required for calculating
 *  the CRC of a data block. Default values can be set with CRC_Params_init.
 */
typedef struct {
    /*! Blocking, callback, or polling return behavior */
    CRC_ReturnBehavior      returnBehavior;
    /*! Callback function pointer */
    CRC_CallbackFxn         callbackFxn;
    /*! Maximum time in ticks the driver will wait for hardware
     *  to become available. Also limits maximum operation time,
     *  but only in BLOCKING mode. */
    uint32_t                timeout;
    /*! Custom argument used by driver implementation */
    void                    *custom;

    /*! Typically 0x0000... or 0xFFFF...
     *  This value should always be the width of the polynomial. */
    uint32_t          seed;
    /*! Which polynomial to use for calculation
     * @sa CRC_Polynomial */
    CRC_Polynomial    polynomial;
    /*! If programmable polynomials are supported, set the polynomial here */
    uint32_t          programmablePoly;
    /*! If programmable polynomials are supported, set the order of the polynomial here */
    uint32_t          programmablePolyOrder;

    /*! Determines the width of the operation (i.e. is data consumed in 8, 16 or 32-bit blocks).
     * Does not impact the width of the result, which is always the same as the polynomial order. */
    CRC_DataSize      dataSize;
    /*! If endianness processing is needed on the input.
     * @sa CRC_ByteSwap */
    CRC_ByteSwap      byteSwapInput;

    /*! This reverses the bits in the input register */
    uint8_t           reverseInputBits;
    /*! This inverts the output bits of the final result */
    uint8_t           invertOutputBits;
    /*! This reverses the bits of the final result */
    uint8_t           reverseOutputBits;
    /*! After calculation is complete, the result will be XORed with this field */
    uint32_t          finalXorValue;
} CRC_Params;

/*!
 *  @brief Default CRC_Params structure
 *
 *  @sa     CRC_Params_init()
 */
extern const CRC_Params CRC_defaultParams;

/*!
 *  @brief  This function initializes the CRC module.
 *
 *  @pre    The CRC_config structure must exist and be persistent before this
 *          function can be called. This function must also be called before
 *          any other CRC driver APIs. This function call does not modify any
 *          peripheral registers.
 */
extern void CRC_init(void);

/*!
 *  @brief  Function to initialize the CRC_Params struct to its defaults
 *
 *  @param  params      An pointer to CRC_Params structure for initialization
 *
 *  Defaults values are:
 *      returnBehavior          = CRC_RETURN_BEHAVIOR_POLLING
 *      callbackFxn             = NULL
 *      timeout                 = SemaphoreP_WAIT_FOREVER
 *      custom                  = NULL
 *
 *      seed                    = 0xFFFFFFFF,
 *      polynomial              = CRC_POLYNOMIAL_CRC_8_CCITT,
 *      programmablePoly        = 0,
 *      programmablePolyOrder   = 0,
 *
 *      dataSize                = CRC_DATA_SIZE_8BIT,
 *      finalXorValue           = 0,
 *      byteSwapInput           = CRC_BYTESWAP_UNCHANGED,
 *      reverseInputBits        = false,
 *      invertOutputBits        = false,
 *      reverseOutputBits       = false,
 */
extern void CRC_Params_init(CRC_Params *params);

/*!
 *  @brief  This function opens a given CRC peripheral.
 *
 *  @pre    CRC controller has been initialized using CRC_init()
 *
 *  @param  index         Logical peripheral number for the CRC indexed into
 *                        the CRC_config table
 *
 *  @param  params        Pointer to an parameter block, if NULL it will use
 *                        default values. All the fields in this structure are
 *                        RO (read-only).
 *
 *  @return A CRC_Handle on success or a NULL on an error or if it has been
 *          opened already.
 *
 *  @sa     CRC_init()
 *  @sa     CRC_close()
 */
extern CRC_Handle CRC_open(uint_least8_t index, const CRC_Params *params);

/*!
 *  @brief  Performs the CRC of the provided bytes, placing the final CRC
 *          into result. Waits for HW access.
 *
 *  @note   The data source array will be accessed in multiples of the data
 *          size defined in the operation struct. If padding bytes are required
 *          (e.g. to pad a 9-byte message to 12 bytes for 32-bit operation) this
 *          must be handled by the application.
 *
 *  @pre    CRC_open() has to be called first.
 *
 *  @param  [in] handle       A CRC handle returned from CRC_open()
 *
 *  @param  [in] source       A pointer to a data source array.
 *
 *  @param  [in] sourceBytes  The length of the source array in bytes.
 *
 *  @param  [out] result      A pointer to a memory location for the result.
 *                            Should be at least the width of the data size.
 *
 *  @return A status code
 */
extern int_fast16_t CRC_calculateFull(CRC_Handle handle, const void *source, size_t sourceBytes, void *result);

/*!
 *  @brief  Performs the CRC of the provided bytes. Waits for HW access.
 *
 *  @note   The data source array will be accessed in multiples of the data
 *          size defined in the operation struct. If padding bytes are required
 *          (e.g. to pad a 9-byte message to 12 bytes for 32-bit operation) this
 *          must be handled by the application.
 *
 *  @pre    CRC_open() has to be called first.
 *
 *  @post   CRC_addData to check another data block or CRC_finalize to extract the result.
 *          CRC_reset will clear an existing partial result (finalise also does this.)
 *
 *  @param  [in] handle       A CRC handle returned from CRC_open()
 *
 *  @param  [in] source       A pointer to a data source array.
 *
 *  @param  [in] sourceBytes  The length of the source array in bytes.
 *
 *  @return A status code
 *
 *  @sa CRC_setup
 */
extern int_fast16_t CRC_addData(CRC_Handle handle, const void *source, size_t sourceBytes);

/*!
 *  @brief  Completes the CRC calculation and places the final CRC into result.
 *
 *  @note   The data source array will be accessed in multiples of the data
 *          size defined in the operation struct. If padding bytes are required
 *          (e.g. to pad a 9-byte message to 12 bytes for 32-bit operation) this
 *          must be handled by the application.
 *
 *  @pre    CRC_open() must be called.
 *          CRC_addData should be called at least once.
 *
 *  @param  [in] handle       A CRC handle returned from CRC_open()
 *
 *  @param  [out] result      A pointer to a memory location containing the result.
 *
 *  @sa CRC_setup
 *  @sa CRC_addData
 */
extern void CRC_finalize(CRC_Handle handle, void *result);

/*!
 *  @brief  Clears any intermediate results such that the next addData call will begin a new CRC.
 *
 *  @pre    CRC_open() must be called.
 *          CRC_addData should be called at least once.
 *
 *  @param  [in] handle       A CRC handle returned from CRC_open()
 *
 *  @sa CRC_setup
 *  @sa CRC_addData
 */
extern void CRC_reset(CRC_Handle handle);

/*!
 *  @brief  Function to close a CRC peripheral specified by the CRC handle
 *
 *  @pre    CRC_open() has to be called first.
 *
 *  @param  handle A CRC handle returned from CRC_open()
 *
 *  @sa     CRC_open()
 */
extern void CRC_close(CRC_Handle handle);

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_CRC__include */
