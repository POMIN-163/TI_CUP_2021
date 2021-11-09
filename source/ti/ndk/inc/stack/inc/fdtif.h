/*
 * Copyright (c) 2012-2017, Texas Instruments Incorporated
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
 * ======== fdtif.h ========
 *
 * File descriptor table and file descriptor handle interface 
 * functions.
 *
 */

#ifndef _FDTIF_INC_
#define _FDTIF_INC_

#ifdef __cplusplus
extern "C" {
#endif

#define FDMAXSHARE  6

/* File descriptor header */
typedef struct _filedesc {
             uint32_t Type;             /* Set to HTYPE_SOCK or HTYPE_PIPE */
             uint16_t OpenCount;          /* Number of opens on this descriptor */
             uint16_t LockCount;          /* Number of locks on this descriptor */
             void *hFDTWait[FDMAXSHARE];  /* File desc table of waiting task */
             unsigned char EventFlags[FDMAXSHARE]; /* Event conditions */
         } FILEDESC;


/* FD Events */
#define FD_EVENT_READ       0x01
#define FD_EVENT_WRITE      0x02
#define FD_EVENT_EXCEPT     0x04
#define FD_EVENT_INVALID    0x08

/* FD Functions */
extern void   FdSignalEvent( void *hFd, unsigned char EventFlags );
extern int    FdWaitEvent( void *hFd, unsigned char EventFlags, uint32_t timeout );

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif

