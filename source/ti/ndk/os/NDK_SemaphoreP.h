/*
 * Copyright (c) 2015-2018, Texas Instruments Incorporated
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

#ifndef ti_ndk_os_NDK_SemaphoreP__include
#define ti_ndk_os_NDK_SemaphoreP__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/*!
 *  @brief    Number of bytes greater than or equal to the size of any RTOS
 *            SemaphoreP object.
 *
 *  SysBIOS:  28
 */
#define NDK_SemaphoreP_STRUCT_SIZE   (28)

/*
 *  Opaque structure that should be large enough to hold any of the
 *  RTOS specific SemaphoreP objects.
 */
typedef union NDK_SemaphoreP_Struct {
    uint32_t dummy;  /*!< Align object */
    char     data[NDK_SemaphoreP_STRUCT_SIZE];
} NDK_SemaphoreP_Struct;

#define NDK_SemaphoreP_WAIT_FOREVER ~(0)

typedef enum NDK_SemaphoreP_Status {
    NDK_SemaphoreP_OK = 0,
    NDK_SemaphoreP_TIMEOUT = -1
} NDK_SemaphoreP_Status;

typedef void *NDK_SemaphoreP_Handle;

extern NDK_SemaphoreP_Handle NDK_SemaphoreP_create(unsigned int count);
extern NDK_SemaphoreP_Handle NDK_SemaphoreP_createBinary(unsigned int count);
extern void NDK_SemaphoreP_delete(NDK_SemaphoreP_Handle handle);
extern NDK_SemaphoreP_Status NDK_SemaphoreP_pend(NDK_SemaphoreP_Handle handle,
        uint32_t timeout);
extern void NDK_SemaphoreP_post(NDK_SemaphoreP_Handle handle);

#ifdef __cplusplus
}
#endif

#endif /* ti_ndk_os_NDK_SemaphoreP__include */
