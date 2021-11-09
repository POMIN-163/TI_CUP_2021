/*
 * Copyright (c) 2013-2019, Texas Instruments Incorporated
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
 * ======== usertype.h ========
 *
 * - Basic C types
 * - Some IP related equates
 * - Data access macros
 *
 */
/* Note64: check bitwise operations for 64 bit */
#ifndef _C_USERTYPE_INC
#define _C_USERTYPE_INC

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* IPv6 Address in Network Format. */
typedef struct IP6N
{
    union
    {
        unsigned char addr8[16];
        uint16_t  addr16[8];
        uint32_t  addr32[4];
    }u;
}IP6N;

/* Macro to convert struct in6_addr to IP6N */
#define _IPv6_a2i(a) (*(IP6N *)&a)

#ifdef NDK_BIGENDIAN

/* BIG ENDIAN */

/*----------------------------------------------------------------------- */
/* Data Host/Network Byte Order Conversion */
#define HNC16(a) (a)
#define HNC32(a) (a)

/*----------------------------------------------------------------------- */
/* 32 Bit Data Macros (from 16 bit aligned data) */
/*#define RdNet32(x)   (*(uint32_t *)(x)) */
/*#define WrNet32(x,y) (*(uint32_t *)(x) = y) */
#define RdNet32(x)   (((uint32_t)(*(uint16_t *)(x))<<16)|(uint32_t)(*(uint16_t *)(((unsigned char *)(x))+2)))
#define WrNet32(x,y) *(uint16_t *)(x)=(uint16_t)((y)>>16); *(uint16_t *)(((unsigned char *)(x))+2)=(uint16_t)(y)

/*----------------------------------------------------------------------- */
/* READ/WRITE Macros (aligned) */
#define READ32(x)    (*(volatile unsigned int *)x)
#define WRITE32(x,y) (*(volatile unsigned int *)(x) = (y))

/*----------------------------------------------------------------------- */
/* IP Address Related Equates */
// Note64: constants 0x8000 0000, 0xc000 0000, etc. possibly problematic
// Note64: consider adding "U" suffix to all of these hex constants
#define IN_CLASSA(x)            (((uint32_t)(x) & 0x80000000) == 0)
#define IN_CLASSB(x)            (((uint32_t)(x) & 0xc0000000) == 0x80000000)
#define IN_CLASSC(x)            (((uint32_t)(x) & 0xe0000000) == 0xc0000000)
#define IN_CLASSD(x)            (((uint32_t)(x) & 0xf0000000) == 0xe0000000)
#define IN_MULTICAST(x)         IN_CLASSD(x)
#define IN_EXPERIMENTAL(x)      (((uint32_t)(x) & 0xf0000000) == 0xf0000000)
#define IN_LOOPBACK(x)          (((uint32_t)(x) & 0xff000000) == 0x7f000000)
#define NDK_LOOPBACK            0x7F000001     /* 127.0.0.1 */
#define INADDR_ANY              0x00000000     /* 0.0.0.0 */
// Note64: 0xffffffff constant converts to 0x00000000ffffffff in 64 bit land. use
//   -1 here? --> NO. this value fits into 32 bits and has most signifcant bit
//   set, => type is unsigned int
#define INADDR_BROADCAST        0xffffffff     /* 255.255.255.255 */

#else

/* LITTLE ENDIAN */

/*----------------------------------------------------------------------- */
/* Data Host/Network Byte Order Conversion */
#define HNC16(a) ( (((a)>>8)&0xff) + (((a)<<8)&0xff00) )

/*
 * Fix warning when compiling for IAR (SDOCM00103001):
 *
 *     Warning[Pe061]: integer operation result is out of range
 *
 * This macro has been updated to perform masking operations before shifting.
 * In its previous form (which shifts THEN masks), the IAR compiler generated
 * a warning because it did not like shaving off bits, as it is a (potential)
 * accidental loss of data.  Changing the code to mask first (purposefully
 * losing the data) then shifting afterward fixes the warning.
 *
 * Note that the TI and GCC compilers never cared about this ...
 *
 */
// Note64: 0xff00.. is unsigned int. Others (0x00ff00) are signed int. Ok?
#define HNC32(a) ((((a) & 0xff000000) >> 24) | (((a) & 0x00ff0000) >> 8) | \
                  (((a) & 0x0000ff00) << 8)  | (((a) & 0x000000ff) << 24))

/*----------------------------------------------------------------------- */
/* 32 Bit Data Macros (from 16 bit aligned data) */
/*#define RdNet32(x)   (*(uint32_t *)(x)) */
/*#define WrNet32(x,y) (*(uint32_t *)(x) = y) */
#define RdNet32(x)   ((uint32_t)(*(uint16_t *)(x))|((uint32_t)(*(uint16_t *)(((unsigned char *)(x))+2))<<16))
#define WrNet32(x,y) *(uint16_t *)(x)=(uint16_t)(y); *(uint16_t *)(((unsigned char *)(x))+2)=(uint16_t)((y)>>16)

/*----------------------------------------------------------------------- */
/* READ/WRITE Macros (aligned) */
#define READ32(x)    (*(volatile unsigned int *)x)
#define WRITE32(x,y) (*(volatile unsigned int *)(x) = (y))

/*----------------------------------------------------------------------- */
/* IP Address Related Equates */
#define IN_CLASSA(x)            (((uint32_t)(x) & 0x00000080) == 0)
#define IN_CLASSB(x)            (((uint32_t)(x) & 0x000000c0) == 0x00000080)
#define IN_CLASSC(x)            (((uint32_t)(x) & 0x000000e0) == 0x000000c0)
#define IN_CLASSD(x)            (((uint32_t)(x) & 0x000000f0) == 0x000000e0)
#define IN_MULTICAST(x)         IN_CLASSD(x)
#define IN_EXPERIMENTAL(x)      (((uint32_t)(x) & 0x000000f0) == 0x000000f0)
#define IN_LOOPBACK(x)          (((uint32_t)(x) & 0x000000ff) == 0x0000007f)
#define NDK_LOOPBACK            0x0100007F     /* 127.0.0.1 */
#define INADDR_ANY              0x00000000     /* 0.0.0.0 */
#define INADDR_BROADCAST        0xffffffff     /* 255.255.255.255 */

#endif

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
