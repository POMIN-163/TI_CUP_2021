/*
 * Copyright (c) 2012-2018, Texas Instruments Incorporated
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
 * */
/*
 * ======== stacksys.h ========
 *
 * These are the base include files required to build an application that
 * uses the stack, but does not have visibility into it.
 *
 */

#ifndef _C_SYS_H_
#define _C_SYS_H_

/* Standard C includes */
#include <stdarg.h>
#include <string.h>
#include <stddef.h>

/* C++ / C Function Declarations */
#ifdef __cplusplus
#define _extern extern "C"
#define _externfar extern "C" far
#else
#define _extern extern
#define _externfar extern far
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __TI_COMPILER_VERSION__
/* Check for BE targets on TI code gen and define NDK's BE macro accordingly */
#if defined(__ARM_BIG_ENDIAN) || defined(__big_endian__)
#define NDK_BIGENDIAN
#endif
#elif defined (__IAR_SYSTEMS_ICC__)
/* Check for BE targets on IAR code gen and define NDK's BE macro accordingly */
#if defined(__BIG_ENDIAN__)
#define NDK_BIGENDIAN
#endif
#elif defined (__GNUC__)
/* Check for BE targets on GCC code gen and define NDK's BE macro accordingly */
#if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#define NDK_BIGENDIAN
#endif
#else
#warning Unsupported compiler, defaulting to little endian
#endif

#ifdef __cplusplus
}
#endif /* extern "C" */

/* Additional usertypes for use by the Stack, OS, and Applications */
#include "usertype.h"
#include "socket.h" /* use original NDK style sockets prototypes */
#include "serrno.h"

/* Operating System */
#include "os/osif.h"

/* Hardware Driver Support */
#include "hal/hal.h"

#endif
