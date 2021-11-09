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

/** ============================================================================
 *  @file       CRCMSP432E4.h
 *
 *  @brief      CRC driver implementation for MSP432E4
 *
 *  The CRC header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/CRC.h>
 *  #include <ti/drivers/crc/CRCMSP432E4.h>
 *  @endcode
 *
 *  # Supported Functionality #
 *
 *  This driver supports the following CRCs:
 *      - CRC_POLYNOMIAL_CRC_16_IBM
 *      - CRC_POLYNOMIAL_CRC_16_CCITT
 *      - CRC_POLYNOMIAL_CRC_32_IEEE
 *      - CRC_POLYNOMIAL_CRC_32C
 *      - CRC_POLYNOMIAL_CRC_TCP
 *
 *  This driver supports only the following size options. Data processing
 *  is only supported for CRC_DATA_SIZE_32BIT.
 *      - CRC_DATA_SIZE_8BIT
 *      - CRC_DATA_SIZE_32BIT
 */

#ifndef ti_drivers_crc_CRCMSP432E4__include
#define ti_drivers_crc_CRCMSP432E4__include

#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/CRC.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 *  @brief      CRCMSP432E4 Hardware Attributes
 *
 *  The CRCMSP432E4 driver does not have any configurable hardware attributes.
 *  NULL should be used for the hwAttrs parameter of the CRC_Config struct.
 */

/*!
 *  @brief      CRCMSP432E4 Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct {
    CRC_CallbackFxn     callbackFxn;
    uint32_t            timeout;
    uint32_t            seed;
    uint32_t            finalXorValue;
    uint32_t            configRegister;
    void               *resultLocation;
    uint32_t            resultRawPartial;
    uint32_t            resultProcessedPartial;
    CRC_DataSize        dataSize;
    CRC_ByteSwap        byteSwapInput;
    CRC_ReturnBehavior  returnBehavior;
    uint8_t             isOpen;
    uint8_t             ongoingPartial;
} CRCMSP432E4_Object;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_crc_CRCMSP432E4__include */
