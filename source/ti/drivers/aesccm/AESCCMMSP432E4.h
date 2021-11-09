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
/**
 *  @file       AESCCMMSP432E4.h
 *
 *  @brief      AESCCM driver implementation for the MSP432E4 family
 *
 * The generic driver interface can be found in AESCCM.h.
 * This file should only be included in the board file to fill the
 * #AESCCM_Config struct.
 *
 * <h3>Features</h3>
 *
 * - Uses the dedicated AES hardware accelerator.
 *
 * - Utilizes the uDMA to transfer data from/to the AES accelerator when
 *   using #AESCCM_RETURN_BEHAVIOR_BLOCKING and #AESCCM_RETURN_BEHAVIOR_CALLBACK.
 *
 * - Uses a hardware interrupt to allow asynchronous operation.
 *   The interrupt priority can be configured in
 *   #AESCCMMSP432E4_HWAttrs::intPriority
 *
 *
 * <h3>Limitations</h3>
 *
 * - When using #AESCCM_RETURN_BEHAVIOR_BLOCKING and #AESCCM_RETURN_BEHAVIOR_CALLBACK,
 *   the pointers #AESCCM_Operation::input and #AESCCM_Operation::output must be
 *   word-aligned. This is required by the uDMA.
 *
 * - Only plaintext #CryptoKey types keys are supported by this implementation.
 *
 * - Only one operation can be carried out on the accelerator at a time.
 *   Mutual exclusion is implemented at the driver level and coordinated
 *   between all drivers relying on the accelerator. It is transparent to the
 *   application and only noted to ensure sensible access timeouts are set
 *   for #AESCCM_Params::timeout.
 *
 * - This implementation does not support internal generation of initialization
 *   vectors.
 *
 *
 * <h3>Performance</h3>
 *
 * Polling mode offers better performance than blocking and callback mode
 * because no DMA and no interrupt handling is involved. It is recommended
 * to use polling mode for messages shorter than 512 bytes.
 *
 * Measurements have been taken with 16 bytes key length and 16 bytes
 * aad length.
 *
 * Encryption:
 *
 * | Length (bytes) | Polling mode (microseconds) | Blocking/callback mode |
 * | -------------- | --------------------------- | ---------------------- |
 * | 16             | 47                          | 89                     |
 * | 30             | 50                          | 92                     |
 * | 60             | 53                          | 92                     |
 * | 120            | 61                          | 100                    |
 * | 240            | 73                          | 106                    |
 * | 512            | 105                         | 126                    |
 * | 1024           | 151                         | 165                    |
 * | 4096           | 512                         | 394                    |
 *
 * Decryption and authentication:
 *
 * | Length (bytes) | Polling mode (microseconds) | Blocking/callback mode |
 * | -------------- | --------------------------- | ---------------------- |
 * | 16             | 33                          | 75                     |
 * | 30             | 36                          | 78                     |
 * | 60             | 39                          | 78                     |
 * | 120            | 46                          | 86                     |
 * | 240            | 59                          | 91                     |
 * | 512            | 91                          | 112                    |
 * | 1024           | 150                         | 150                    |
 * | 4096           | 512                         | 380                    |
 *
 */

#ifndef ti_drivers_aesccm_AESCCMMSP432E4__include
#define ti_drivers_aesccm_AESCCMMSP432E4__include

#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/AESCCM.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 *  @brief Hardware-specific configuration attributes
 *
 *  AESCCMMSP432E4 hardware attributes are used in the board file by the
 *  #AESCCM_Config struct.
 */
typedef struct {
    uint8_t    intPriority; /*!< Hardware interrupt priority of the AES accelerator.
                             *
                             * The MSP432E4 provides 8 interrupt priority levels encoded in three bits:
                             *
                             * Value        | Description
                             * ------------ | -----------------------
                             * (~0)         | Special value: always lowest priority across all OS kernels.
                             * (7 << 5)     | Priority level 7: lowest, but rather use ~0 instead.
                             * ..           | ..
                             * (0 << 5)     | Priority level 0: highest, not supported by this driver
                             *
                             * Hardware interrupts with priority level 0 ignore the hardware interrupt dispatcher
                             * for minimum latency. This is not supported by this driver.
                             */
} AESCCMMSP432E4_HWAttrs;

/*! \cond Internal APIs */

/*
 * AESCCMMSP432E4 context object
 * The application must not access any member variables of this structure.
 */
typedef struct {
    bool                            isOpen;
    int_fast16_t                    returnStatus;
    AESCCM_OperationType            operationType;
    AESCCM_ReturnBehavior           returnBehavior;
    uint32_t                        accessTimeout;
    AESCCM_CallbackFxn              callbackFxn;
    AESCCM_Operation                *operation;
} AESCCMMSP432E4_Object;

/*! \endcond */

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_aesccm_AESCCMMSP432E4__include */
