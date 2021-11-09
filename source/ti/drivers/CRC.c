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

#include <ti/drivers/CRC.h>
#include <ti/drivers/dpl/SemaphoreP.h>

const CRC_Params CRC_defaultParams = {
    .returnBehavior         = CRC_RETURN_BEHAVIOR_POLLING,
    .callbackFxn            = NULL,
    .timeout                = SemaphoreP_WAIT_FOREVER,
    .custom                 = NULL,

    .seed                   = 0xFFFFFFFF,
    .polynomial             = CRC_POLYNOMIAL_CRC_8_CCITT,
    .programmablePoly       = 0,
    .programmablePolyOrder  = 0,

    .dataSize               = CRC_DATA_SIZE_8BIT,
    .finalXorValue          = 0,
    .byteSwapInput          = CRC_BYTESWAP_UNCHANGED,
    .reverseInputBits       = false,
    .invertOutputBits       = false,
    .reverseOutputBits      = false,
};

/* ======== CRC_Params_init ======== */
void CRC_Params_init(CRC_Params *params) {
    *params = CRC_defaultParams;
}
