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


/*!
 * @file       AESECBMSP432E4.h
 *
 * @brief      AESECB driver implementation for the MSP432E4 family
 *
 * The generic driver interface can be found in AESECB.h.
 * This file should only be included in the board file to fill the
 * #AESECB_Config struct.
 *
 * <h3>Features</h3>
 *
 * - Uses the dedicated AES hardware accelerator.
 *
 * <h3>Limitations</h3>
 * - Only plaintext #CryptoKey types keys are supported by this implementation.
 *
 * - Only one operation can be carried out on the accelerator at a time.
 *   Mutual exclusion is implemented at the driver level and coordinated
 *   between all drivers relying on the accelerator. It is transparent to the
 *   application and only noted to ensure sensible access timeouts are set
 *   for #AESECB_Params::timeout.
 *
 * - This implementation does not support internal generation of initialization
 *   vectors.
 *
 */

#ifndef ti_drivers_aesECB_AESECBMSP432E4__include
#define ti_drivers_aesECB_AESECBMSP432E4__include

#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/AESECB.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 *  @brief Hardware-specific configuration attributes
 *
 *  AESECBMSP432E4 hardware attributes are used in the board file by the
 *  #AESECB_Config struct.
 */
typedef struct {
    uint8_t    dummy;
} AESECBMSP432E4_HWAttrs;

/*! \cond Internal APIs */

/*
 * AESECBMSP432E4 context object
 * The application must not access any member variables of this structure.
 */
typedef struct {
    bool                            isOpen;
    bool                            threadSafe;
    AESECB_ReturnBehavior           returnBehavior;
    AESECB_OperationType            operationType;
    int_fast16_t                    returnStatus;
    uint32_t                        accessTimeout;
    AESECB_CallbackFxn              callbackFxn;
    AESECB_Operation                *operation;
} AESECBMSP432E4_Object;

/*! \endcond */

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_aesECB_AESECBMSP432E4__include */
