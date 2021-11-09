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

/* Drivers */
#include <ti/drivers/CRC.h>
#include <ti/drivers/crc/CRCMSP432E4.h>
#include <ti/drivers/power/PowerMSP432E4.h>
#include <ti/drivers/cryptoutils/sharedresources/CryptoResourceMSP432E4.h>

/* Driverlib */
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/msp432.h)
#include DeviceFamily_constructPath(driverlib/crc.h)
#include DeviceFamily_constructPath(driverlib/types.h)
#include DeviceFamily_constructPath(driverlib/sysctl.h)
#include DeviceFamily_constructPath(driverlib/inc/hw_ccm.h)

/* Forward declarations */
static void resetHardware(void);
static void cleanup(CRC_Handle handle);
static void calculateCpu(CRC_DataSize size, CRC_ByteSwap swap, const void *source, size_t sourceBytes);
static int_fast16_t calculate(CRC_Handle handle, const void *source, size_t sourceBytes);
static void setResultLocation(CRC_DataSize size, void* resultLocation, uint32_t crcResult);
static int_fast16_t getHwConfigRegister(const CRC_Params *params, uint32_t *crcConfig);

/* External symbols defined in the board file */
extern const CRC_Config CRC_config[];
extern const uint_least8_t CRC_count;

/* Static globals */
static uint8_t isInitialized = false;

/* There is currently a bug in the MSPE4 Driverlib where the SHW and SBHW
 * definitions are swapped. These definitions are used instead. */
#define CRC_CONFIG_ENDIAN_SBHW     0x00000010  // Swap byte in half-word
#define CRC_CONFIG_ENDIAN_SHW      0x00000020  // Swap half-word

/* ======== CRC_init ======== */
void CRC_init(void) {
    uintptr_t key;

    key = HwiP_disable();

    if (!isInitialized) {

        /* CCM hardware has a rare issue where any given power state change can cause the state to hang.
         * As a workaround, we only change the power state once, at init(), and do not power it down. */
        resetHardware();
        isInitialized = true;
    }

    HwiP_restore(key);
}

/* ======== CRC_open ======== */
CRC_Handle CRC_open(uint_least8_t index, const CRC_Params *params) {
    CRC_Handle handle;
    CRCMSP432E4_Object *object;
    uintptr_t key;
    int_fast16_t status;

    handle = (CRC_Handle) &CRC_config[index];
    object = handle->object;

    /* If params are NULL, use defaults */
    if (params == NULL) {
        params = (const CRC_Params *) &CRC_defaultParams;
    }

    key = HwiP_disable();

    if (!isInitialized || object->isOpen) {
        HwiP_restore(key);
        return NULL;
    }

    status = getHwConfigRegister(params, &object->configRegister);
    if (status != CRC_STATUS_SUCCESS) {
        HwiP_restore(key);
        return NULL;
    }

    object->isOpen = true;
    HwiP_restore(key);

    object->returnBehavior = params->returnBehavior;
    object->callbackFxn = params->callbackFxn;
    object->timeout = params->timeout;

    object->ongoingPartial = 0;
    object->resultRawPartial = 0;
    object->resultProcessedPartial = 0;

    object->seed = params->seed;
    object->dataSize = params->dataSize;
    object->finalXorValue = params->finalXorValue;
    object->byteSwapInput = params->byteSwapInput;

    CryptoResourceMSP432E4_CRC_incrementRefCount();
    return handle;
}

/* ======== CRC_calculateFull ======== */
int_fast16_t CRC_calculateFull(CRC_Handle handle, const void *source, size_t sourceBytes, void *result) {
    CRCMSP432E4_Object *object = handle->object;
    int_fast16_t status;

    status = calculate(handle, source, sourceBytes);
    setResultLocation(object->dataSize, result, object->resultProcessedPartial);

    if (object->returnBehavior == CRC_RETURN_BEHAVIOR_CALLBACK) {
        object->callbackFxn(handle, CRC_STATUS_SUCCESS, result);
    }
    return status;
}

/* ======== CRC_addData ======== */
int_fast16_t CRC_addData(CRC_Handle handle, const void *source, size_t numBytes) {
    CRCMSP432E4_Object *object = handle->object;
    int_fast16_t status;

    if (object->ongoingPartial) {
        object->seed = object->resultRawPartial;
    }
    object->ongoingPartial = 1;
    status = calculate(handle, source, numBytes);

    if (object->returnBehavior == CRC_RETURN_BEHAVIOR_CALLBACK) {
        object->callbackFxn(handle, status, NULL);
        return CRC_STATUS_SUCCESS;
    } else {
        return status;
    }
}

/* ======== CRC_finalize ======== */
void CRC_finalize(CRC_Handle handle, void *result) {
    CRCMSP432E4_Object *object = handle->object;

    object->ongoingPartial = 0;
    setResultLocation(object->dataSize, result, object->resultProcessedPartial);
}

void CRC_reset(CRC_Handle handle) {
    CRCMSP432E4_Object *object = handle->object;

    object->ongoingPartial = 0;
}

/* ======== CRC_close ======== */
void CRC_close(CRC_Handle handle) {
    CRCMSP432E4_Object *object = handle->object;

    cleanup(handle);
    CryptoResourceMSP432E4_CRC_decrementRefCount();
    object->isOpen = false;
}

/* ======== calculate ========
 * Configures the hardware using the configuration provided by setupOperation
 * and kicks off calculation of the checksum for the provided bytes */
static int_fast16_t calculate(CRC_Handle handle, const void *source, size_t sourceBytes) {
    CRCMSP432E4_Object *object = handle->object;

    if (object->dataSize == CRC_DATA_SIZE_32BIT && (sourceBytes % 4) != 0) {
        return CRC_STATUS_LEFTOVER_BYTES_PRESENT;
    }

    if (SemaphoreP_OK != CryptoResourceMSP432E4_CRC_tryLock(object->timeout)) {
        return CRC_STATUS_RESOURCE_UNAVAILABLE;
    }

    HWREG(CCM0_BASE + CCM_O_CRCCTRL) = object->configRegister;
    HWREG(CCM0_BASE + CCM_O_CRCSEED) = object->seed;

    /* Use the CPU to calculate the CRC (result available in the peripheral register) */
    calculateCpu(object->dataSize, object->byteSwapInput, source, sourceBytes);
    /* Extract the result into the object, reset the CRC state and then return */
    cleanup(handle);

    return CRC_STATUS_SUCCESS;
}

/* ======== calculateCpu ========
 * Performs a CRC calculation using the CPU. Returns after completion. */
static void calculateCpu(CRC_DataSize size, CRC_ByteSwap swap, const void *source, size_t sourceBytes) {
    if (size == CRC_DATA_SIZE_8BIT) {
        uint8_t *source8 = (uint8_t *) source;

        while (sourceBytes--) {
            HWREG(CCM0_BASE + CCM_O_CRCDIN) = *(source8++);
        }
    }
    else if (size == CRC_DATA_SIZE_32BIT) {
        uint32_t *source32 = (uint32_t *) source;
        sourceBytes /= 4;

        while (sourceBytes--) {
            HWREG(CCM0_BASE + CCM_O_CRCDIN) = *(source32++);
        }
    }
}

/* ======== cleanup ========
 * Call this after the calculation finishes to move the results out of the CCM,
 * and close/reset all the relevant handles/statics. */
static void cleanup (CRC_Handle handle) {
    CRCMSP432E4_Object *object = handle->object;

    object->resultRawPartial = HWREG(CCM0_BASE + CCM_O_CRCSEED);
    object->resultProcessedPartial = HWREG(CCM0_BASE + CCM_O_CRCRSLTPP) ^ object->finalXorValue;

    CryptoResourceMSP432E4_CRC_releaseLock();
}

/* ======== resetHardware ========
 * Enables and resets the CCM. */
static void resetHardware() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CCM0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_CCM0))
    { }

    /* Soft reset the accelerator */
    SysCtlPeripheralReset(SYSCTL_PERIPH_CCM0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_CCM0))
    { }
}

/* ======== setResultLocation ========
 * Correctly applies crcResult to resultLocation for different bit widths */
static void setResultLocation(CRC_DataSize size, void* resultLocation, uint32_t crcResult) {
    switch (size) {
        case CRC_DATA_SIZE_8BIT:
            *((uint8_t*) resultLocation) = (uint8_t) crcResult;
            break;
        case CRC_DATA_SIZE_32BIT:
            *((uint32_t*) resultLocation) = crcResult;
            break;
        default:
            break;
    }
}

/* ======== getHwConfigRegister ========
 * Uses the settings in the operation to put together configuration register settings.
 * Returns CRC_OPERATION_NOT_SUPPORTED for config errors or CRC_STATUS_SUCCESS. */
static int_fast16_t getHwConfigRegister(const CRC_Params *params, uint32_t *crcConfig) {
    /* Always manually control the seed instead of using the builtin
    options for 0x0 and 0xf. This avoids additional processing work
    and makes the partial API easier to use. */
    *crcConfig = CRC_CFG_INIT_SEED;

    /* No support for 16-bit data on MSP432E4 */
    switch (params->dataSize) {
        case CRC_DATA_SIZE_8BIT:
            *crcConfig |= CRC_CFG_SIZE_8BIT;
            break;
        case CRC_DATA_SIZE_32BIT:
            *crcConfig |= CRC_CFG_SIZE_32BIT;
            break;
        default:
            return CRC_STATUS_OPERATION_NOT_SUPPORTED;
    }

    /* There is no HW support for processing options in 8-bit mode,
    and supporting them in SW would take too much processing time for
    BLOCKING and CALLBACK modes. Instead, throw an error.
    The CRC_ByteSwap enum documents this limitation. */
    if (params->dataSize == CRC_DATA_SIZE_8BIT &&
            params->byteSwapInput != CRC_BYTESWAP_UNCHANGED) {
        return CRC_STATUS_OPERATION_NOT_SUPPORTED;
    }

    switch (params->byteSwapInput) {
        case CRC_BYTESWAP_UNCHANGED:
            break; /* No action required */
        case CRC_BYTESWAP_HALF_WORDS:
            *crcConfig |= CRC_CONFIG_ENDIAN_SHW;
            break;
        case CRC_BYTESWAP_BYTES_IN_HALF_WORDS:
            *crcConfig |= CRC_CONFIG_ENDIAN_SBHW;
            break;
        case CRC_BYTESWAP_BYTES_AND_HALF_WORDS:
            *crcConfig |= CRC_CONFIG_ENDIAN_SHW | CRC_CONFIG_ENDIAN_SBHW;
            break;
    }

    if (params->reverseInputBits) {
        *crcConfig |= CCM_CRCCTRL_BR;
    }

    if (params->reverseOutputBits) {
        *crcConfig |= CRC_CFG_OBR;
    }

    if (params->invertOutputBits) {
        *crcConfig |= CRC_CFG_RESINV;
    }

    switch (params->polynomial) {
        case CRC_POLYNOMIAL_CRC_16_IBM:
            *crcConfig |= CRC_CFG_TYPE_P8005;
            break;
        case CRC_POLYNOMIAL_CRC_16_CCITT:
            *crcConfig |= CRC_CFG_TYPE_P1021;
            break;
        case CRC_POLYNOMIAL_CRC_32_IEEE:
            *crcConfig |= CRC_CFG_TYPE_P4C11DB7;
            break;
        case CRC_POLYNOMIAL_CRC_32C:
            *crcConfig |= CRC_CFG_TYPE_P1EDC6F41;
            break;
        case CRC_POLYNOMIAL_CRC_TCP:
            *crcConfig |= CRC_CFG_TYPE_TCPCHKSUM;
            break;
        default:
            return CRC_STATUS_OPERATION_NOT_SUPPORTED;
    }

    return CRC_STATUS_SUCCESS;
}
