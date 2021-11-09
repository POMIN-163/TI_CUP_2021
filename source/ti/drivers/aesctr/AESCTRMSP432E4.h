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


/*!
 * @file       AESCTRMSP432E4.h
 *
 * @brief      AESCTR driver implementation for the MSP432E4 family
 *
 * The generic driver interface can be found in AESCTR.h.
 * This file should only be included in the board file to fill the
 * #AESCTR_Config struct.
 *
 * <h3>Features</h3>
 *
 * - Uses the dedicated AES hardware accelerator.
 *
 * - Utilizes the uDMA to transfer data from/to the AES accelerator when
 *   using #AESCTR_RETURN_BEHAVIOR_BLOCKING and #AESCTR_RETURN_BEHAVIOR_CALLBACK.
 *
 * - Uses a hardware interrupt to allow asynchronous operation.
 *   The interrupt priority can be configured in
 *   #AESCTRMSP432E4_HWAttrs::intPriority
 *
 *
 * <h3>Limitations</h3>
 *
 * - When using #AESCTR_RETURN_BEHAVIOR_BLOCKING and #AESCTR_RETURN_BEHAVIOR_CALLBACK,
 *   the pointers #AESCTR_Operation::input and #AESCTR_Operation::output must be
 *   word-aligned. This is required by the uDMA.
 *
 * - Only plaintext #CryptoKey types keys are supported by this implementation.
 *
 * - Only one operation can be carried out on the accelerator at a time.
 *   Mutual exclusion is implemented at the driver level and coordinated
 *   between all drivers relying on the accelerator. It is transparent to the
 *   application and only noted to ensure sensible access timeouts are set
 *   for #AESCTR_Params::timeout.
 *
 */

#ifndef ti_drivers_aesctr_AESCTRMSP432E4__include
#define ti_drivers_aesctr_AESCTRMSP432E4__include

#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/AESCTR.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 *  @brief Hardware-specific configuration attributes
 *
 *  AESCTRMSP432E4 hardware attributes are used in the board file by the
 *  #AESCTR_Config struct.
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
} AESCTRMSP432E4_HWAttrs;

/*! \cond Internal APIs */

/*
 * AESCTRMSP432E4 context object
 * The application must not access any member variables of this structure.
 */
typedef struct {
    bool                            isOpen;
    bool                            threadSafe;
    AESCTR_ReturnBehavior           returnBehavior;
    AESCTR_OperationType            operationType;
    int_fast16_t                    returnStatus;
    uint32_t                        accessTimeout;
    AESCTR_CallbackFxn              callbackFxn;
    AESCTR_Operation                *operation;
} AESCTRMSP432E4_Object;

/*! \endcond */

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_aesctr_AESCTRMSP432E4__include */
