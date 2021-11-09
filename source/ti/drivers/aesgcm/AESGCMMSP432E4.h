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
 *  @file       AESGCMMSP432E4.h
 *
 *  @brief      AESGCM driver implementation for the MSP432E4 family
 *
 * The generic driver interface can be found in AESGCM.h.
 * This file should only be included in the board file to fill the
 * #AESGCM_Config struct.
 *
 * <h3>Features</h3>
 *
 * - Uses the dedicated AES hardware accelerator.
 *
 * - Utilizes the DMA to transfer data from/to the AES accelerator when
 *   using #AESGCM_RETURN_BEHAVIOR_BLOCKING and #AESGCM_RETURN_BEHAVIOR_CALLBACK.
 *
 * - Uses a hardware interrupt to allow asynchronous operation.
 *   The interrupt priority can be configured in
 *   #AESGCMMSP432E4_HWAttrs::intPriority
 *
 *
 * <h3>Limitations</h3>
 *
 * - When using #AESGCM_RETURN_BEHAVIOR_BLOCKING and #AESGCM_RETURN_BEHAVIOR_CALLBACK,
 *   the pointers #AESGCM_Operation::input and #AESGCM_Operation::output must be
 *   word-aligned. This is required by the DMA.
 *
 * - Only plaintext #CryptoKey types keys are supported by this implementation.
 *
 * - Only one operation can be carried out on the accelerator at a time.
 *   Mutual exclusion is implemented at the driver level and coordinated
 *   between all drivers relying on the accelerator. It is transparent to the
 *   application and only noted to ensure sensible access timeouts are set
 *   for #AESGCM_Params::timeout.
 *
 * - The length of initialization vectors is limited to 12 bytes.
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
 */

#ifndef ti_drivers_aesgcm_AESGCMMSP432E4__include
#define ti_drivers_aesgcm_AESGCMMSP432E4__include

#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/AESGCM.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 *  @brief Hardware-specific configuration attributes
 *
 *  AESGCMMSP432E4 hardware attributes are used in the board file by the
 *  #AESGCM_Config struct.
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
} AESGCMMSP432E4_HWAttrs;

/*! \cond Internal APIs */

/*
 * AESGCMMSP432E4 context object
 * The application must not access any member variables of this structure.
 */
typedef struct {
    bool                            isOpen;
    int_fast16_t                    returnStatus;
    AESGCM_OperationType            operationType;
    AESGCM_ReturnBehavior           returnBehavior;
    uint32_t                        accessTimeout;
    AESGCM_CallbackFxn              callbackFxn;
    AESGCM_Operation                *operation;
} AESGCMMSP432E4_Object;

/*! \endcond */

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_aesGCM_AESGCMMSP432E4__include */
