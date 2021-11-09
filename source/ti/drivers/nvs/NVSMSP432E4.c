/*
 * Copyright (c) 2017-2018, Texas Instruments Incorporated
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

/*
 *  ======== NVSMSP432E4.c ========
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <ti/devices/msp432e4/inc/msp432.h>

#include <ti/devices/msp432e4/driverlib/flash.h>

#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>

#include <ti/drivers/NVS.h>
#include <ti/drivers/nvs/NVSMSP432E4.h>

/* 16-KB FLASH sector size */
#define FLASH_SECTOR_SIZE  (0x4000)
#define SECTOR_BASE_MASK   (~(FLASH_SECTOR_SIZE - 1))

/* max number of bytes to write at a time to minimize interrupt latency */
#define MAX_WRITE_INCREMENT (8)

static int_fast16_t checkEraseRange(NVS_Handle handle, size_t offset, size_t size);
static int_fast16_t doErase(NVS_Handle handle, size_t offset, size_t size);
static int_fast16_t writeWordsToFlash(uint32_t *dstBuf, uint32_t *srcBuf,
    size_t size);

extern NVS_Config NVS_config[];
extern const uint8_t NVS_count;

/* NVS function table for NVSMSP432E4 implementation */
const NVS_FxnTable NVSMSP432E4_fxnTable = {
    NVSMSP432E4_close,
    NVSMSP432E4_control,
    NVSMSP432E4_erase,
    NVSMSP432E4_getAttrs,
    NVSMSP432E4_init,
    NVSMSP432E4_lock,
    NVSMSP432E4_open,
    NVSMSP432E4_read,
    NVSMSP432E4_unlock,
    NVSMSP432E4_write
};

/*
 *  Semaphore to synchronize access to flash region.
 */
static SemaphoreP_Handle writeSem;

/*
 *  ======== NVSMSP432E4_close ========
 */
void NVSMSP432E4_close(NVS_Handle handle)
{
    ((NVSMSP432E4_Object *) handle->object)->opened = false;
}

/*
 *  ======== NVSMSP432E4_control ========
 */
int_fast16_t NVSMSP432E4_control(NVS_Handle handle, uint_fast16_t cmd,
    uintptr_t arg)
{
    return (NVS_STATUS_UNDEFINEDCMD);
}

/*
 *  ======== NVSMSP432E4_erase ========
 */
int_fast16_t NVSMSP432E4_erase(NVS_Handle handle, size_t offset, size_t size)
{
    int_fast16_t status;

    SemaphoreP_pend(writeSem, SemaphoreP_WAIT_FOREVER);

    status = doErase(handle, offset, size);

    SemaphoreP_post(writeSem);

    return (status);
}

/*
 *  ======== NVSMSP432E4_getAttrs ========
 */
void NVSMSP432E4_getAttrs(NVS_Handle handle, NVS_Attrs *attrs)
{
    NVSMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;

    attrs->regionBase = hwAttrs->regionBase;
    attrs->regionSize = hwAttrs->regionSize;
    attrs->sectorSize = FLASH_SECTOR_SIZE;
}

/*
 *  ======== NVSMSP432E4_init ========
 */
void NVSMSP432E4_init()
{
    uintptr_t         key;
    SemaphoreP_Handle sem;

    /* speculatively create a binary semaphore for thread safety */
    sem = SemaphoreP_createBinary(1);
    /* sem == NULL will be detected in 'open' */

    key = HwiP_disable();

    if (writeSem == NULL) {
        /* use the binary sem created above */
        writeSem = sem;

        HwiP_restore(key);
    }
    else {
        /* init already called */
        HwiP_restore(key);

        /* delete unused Semaphore */
        if (sem) {
            SemaphoreP_delete(sem);
        }
    }
}

/*
 *  ======== NVSMSP432E4_lock =======
 */
int_fast16_t NVSMSP432E4_lock(NVS_Handle handle, uint32_t timeout)
{
    if (SemaphoreP_pend(writeSem, timeout) != SemaphoreP_OK) {
        return (NVS_STATUS_TIMEOUT);
    }
    return (NVS_STATUS_SUCCESS);
}

/*
 *  ======== NVSMSP432E4_open =======
 */
NVS_Handle NVSMSP432E4_open(uint_least8_t index, NVS_Params *params)
{
    NVS_Handle                 handle;
    NVSMSP432E4_Object        *object;
    NVSMSP432E4_HWAttrs const *hwAttrs;

    /* Confirm that 'init' has successfully completed */
    if (writeSem == NULL) {
        NVSMSP432E4_init();
        if (writeSem == NULL) {
            return (NULL);
        }
    }

    if (index >= NVS_count) {
        return (NULL);
    }

    handle = &NVS_config[index];
    object = NVS_config[index].object;
    hwAttrs = NVS_config[index].hwAttrs;

    SemaphoreP_pend(writeSem, SemaphoreP_WAIT_FOREVER);

    if (object->opened) {
        SemaphoreP_post(writeSem);

        return (NULL);
    }

    /* The regionBase must be aligned on a flash page boundary */
    if ((size_t)(hwAttrs->regionBase) & (FLASH_SECTOR_SIZE - 1)) {
        SemaphoreP_post(writeSem);

        return (NULL);
    }

    /* The region cannot be smaller than a sector size */
    if (hwAttrs->regionSize < FLASH_SECTOR_SIZE) {
        SemaphoreP_post(writeSem);

        return (NULL);
    }

    /* The region size must be a multiple of sector size */
    if (hwAttrs->regionSize != (hwAttrs->regionSize & SECTOR_BASE_MASK)) {
        SemaphoreP_post(writeSem);

        return (NULL);
    }

    object->opened = true;

    SemaphoreP_post(writeSem);

    return (handle);
}

/*
 *  ======== NVSMSP432E4_read =======
 */
int_fast16_t NVSMSP432E4_read(NVS_Handle handle, size_t offset, void *buffer,
    size_t bufferSize)
{
    NVSMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Validate offset and bufferSize */
    if (offset + bufferSize > hwAttrs->regionSize) {
        return (NVS_STATUS_INV_OFFSET);
    }

    /*
     *  Get exclusive access to the region.  We don't want someone
     *  else to erase the region while we are reading it.
     */
    SemaphoreP_pend(writeSem, SemaphoreP_WAIT_FOREVER);

    memcpy(buffer, ((uint8_t *) hwAttrs->regionBase) + offset, bufferSize);

    SemaphoreP_post(writeSem);

    return (NVS_STATUS_SUCCESS);
}

/*
 *  ======== NVSMSP432E4_unlock =======
 */
void NVSMSP432E4_unlock(NVS_Handle handle)
{
    SemaphoreP_post(writeSem);
}

/*
 *  ======== NVSMSP432E4_write =======
 */
int_fast16_t NVSMSP432E4_write(NVS_Handle handle, size_t offset, void *buffer,
    size_t bufferSize, uint_fast16_t flags)
{
    size_t                     i;
    size_t                     size;
    int_fast16_t               result = NVS_STATUS_ERROR;
    uint8_t                   *srcBuf;
    uint8_t                   *dstBuf;
    uint8_t                   *packingPtr;
    uint32_t                  *alignedAddr;
    uint32_t                   packingWord;
    size_t                     count;
    NVSMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Validate offset and bufferSize */
    if ((offset + bufferSize) > hwAttrs->regionSize) {
        return (NVS_STATUS_INV_OFFSET);
    }

    /* Get exclusive access to the Flash region */
    SemaphoreP_pend(writeSem, SemaphoreP_WAIT_FOREVER);

    /* If erase is set, erase destination sector(s) first */
    if (flags & NVS_WRITE_ERASE) {
        size = bufferSize & SECTOR_BASE_MASK;
        if (bufferSize & (~SECTOR_BASE_MASK)) {
            size += FLASH_SECTOR_SIZE;
        }

        result = doErase(handle, offset & SECTOR_BASE_MASK, size);
        if (result != NVS_STATUS_SUCCESS) {
            SemaphoreP_post(writeSem);

            return (result);
        }
    }
    else if (flags & NVS_WRITE_PRE_VERIFY) {
        /*
         *  If pre-verify, each destination byte must be able to be changed to the
         *  source byte (1s to 0s, not 0s to 1s).
         *  this is satisfied by the following test:
         *     src == (src & dst)
         */
        dstBuf = ((uint8_t *) hwAttrs->regionBase) + offset;
        srcBuf = buffer;
        for (i = 0; i < bufferSize; i++) {
            if (srcBuf[i] != (srcBuf[i] & dstBuf[i])) {
                SemaphoreP_post(writeSem);

                return (NVS_STATUS_INV_WRITE);
            }
        }
    }

    size = bufferSize;
    dstBuf = ((uint8_t *) hwAttrs->regionBase) + offset;
    srcBuf = buffer;

    if (((uint32_t) dstBuf) & 0x03) {
        /*
         * Attempting to write to an unaligned address; 'packingWord' is used
         * to read the word-aligned 'base' address.  Unaligned bytes in the
         * buffer are packed into 'packingWord' & then written into memory
         * (read-modify-write).
         */
        alignedAddr = (uint32_t *) ((uint32_t) dstBuf & (~0x03));
        packingWord = *(alignedAddr);
        packingPtr = ((uint8_t *) &packingWord) + ((uint32_t) dstBuf & (0x03));

        /*
         * count contains the number of bytes left till the next word; however,
         * it may be possible you do not want to write the entire word.  In
         * this case size < count; so set count = size.
         */
        count = sizeof(uint32_t) - ((uint32_t) dstBuf & (0x03));
        if (size < count) {
            count = size;
        }

        for (i = 0; i < count; i++) {
            *packingPtr++ = *srcBuf++;
        }

        result = writeWordsToFlash(alignedAddr, &packingWord, 1);
        if (result != NVS_STATUS_SUCCESS) {
            SemaphoreP_post(writeSem);

            return (result);
        }

        size -= count;
        dstBuf += count;
    }

    if (size) {
        /*
         * At this point we know we are writing to an aligned address;
         * bulk write as much word-aligned word-length data as possible
         */
        count = size / sizeof(uint32_t);
        if (count) {
            result = writeWordsToFlash((uint32_t *) dstBuf, (uint32_t *)
                srcBuf, count);
            if (result != NVS_STATUS_SUCCESS) {
                SemaphoreP_post(writeSem);

                return (result);
            }

            size -= (count * sizeof(uint32_t));
            dstBuf += (count * sizeof(uint32_t));
            srcBuf += (count * sizeof(uint32_t));
        }

        /*
         * Finally, pack any remaining bytes into a word & write it to memory.
         */
        if (size) {
            packingWord = *((uint32_t *) dstBuf);
            packingPtr = (uint8_t *) &packingWord;

            for (i = 0; i < size; i++) {
                *packingPtr++ = *srcBuf++;
            }

            result = writeWordsToFlash((uint32_t *) dstBuf, &packingWord, 1);
            if (result != NVS_STATUS_SUCCESS) {
                SemaphoreP_post(writeSem);

                return (result);
            }
        }
    }

    if (result == NVS_STATUS_SUCCESS && (flags & NVS_WRITE_POST_VERIFY)) {
        /*
         *  Note: This validates the entire region even on erase mode.
         */
        dstBuf = ((uint8_t *) hwAttrs->regionBase) + offset;
        srcBuf = buffer;

        for (i = 0; i < bufferSize; i++) {
            if (srcBuf[i] != dstBuf[i]) {
                result = NVS_STATUS_ERROR;
                break;
            }
        }
    }

    SemaphoreP_post(writeSem);

    return (result);
}

/*
 *  ======== checkEraseRange ========
 */
static int_fast16_t checkEraseRange(NVS_Handle handle, size_t offset, size_t size)
{
    NVSMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;

    if (offset != (offset & SECTOR_BASE_MASK)) {
        /* poorly aligned start address */
        return (NVS_STATUS_INV_ALIGNMENT);
    }

    if (offset >= hwAttrs->regionSize) {
        /* offset is past end of region */
        return (NVS_STATUS_INV_OFFSET);
    }

    if (offset + size > hwAttrs->regionSize) {
        /* size is too big */
        return (NVS_STATUS_INV_SIZE);
    }

    if (size != (size & SECTOR_BASE_MASK)) {
        /* size is not a multiple of sector size */
        return (NVS_STATUS_INV_SIZE);
    }

    return (NVS_STATUS_SUCCESS);
}

/*
 *  ======== doErase ========
 */
static int_fast16_t doErase(NVS_Handle handle, size_t offset, size_t size)
{
    uintptr_t                  key;
    uint32_t                   sectorBase;
    int_fast16_t               status;
    NVSMSP432E4_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* sanity test the erase args */
    status = checkEraseRange(handle, offset, size);
    if (status != NVS_STATUS_SUCCESS) {
        return (status);
    }

    sectorBase = ((uint32_t) hwAttrs->regionBase) + offset;

    while (size) {
        key = HwiP_disable();

        status = FlashErase(sectorBase);

        HwiP_restore(key);

        if (status < 0) {
            break;
        }

        sectorBase += FLASH_SECTOR_SIZE;
        size -= FLASH_SECTOR_SIZE;
    }

    return (status);
}

/*
 *  ======== writeToFlash ========
 */
static int_fast16_t writeWordsToFlash(uint32_t *dstBuf, uint32_t *srcBuf,
    size_t size)
{
    uintptr_t    key;
    size_t       writeAmount;
    int_fast16_t result = NVS_STATUS_ERROR;

    size *= sizeof(uint32_t);

    while (size) {
        if (size > MAX_WRITE_INCREMENT) {
            writeAmount = MAX_WRITE_INCREMENT;
        }
        else {
            writeAmount = size;
        }

        key = HwiP_disable();

        result = FlashProgram(srcBuf, (uint32_t) dstBuf, writeAmount);

        HwiP_restore(key);

        if (result != NVS_STATUS_SUCCESS) {
            break;
        }
        else {
            size -= writeAmount;
            srcBuf = (uint32_t *) ((uint32_t) srcBuf + writeAmount);
            dstBuf = (uint32_t *) ((uint32_t) dstBuf + writeAmount);
        }
    }

    return (result);
}
