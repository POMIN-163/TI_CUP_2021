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
 * ======== fdt.h ========
 *
 * File Management Includes
 *
 */

#ifndef _FDT_H
#define _FDT_H

#ifdef __cplusplus
extern "C" {
#endif

/* File Descriptor Table */
typedef struct _fdtable {
        uint32_t         Type;          /* Set to HTYPE_FDTABLE */
        uint32_t         RefCount;      /* Object reference count */
        void            *hSem;          /* File Event semaphore */
        int              error;         /* FileOp Error Number */
        unsigned char          fClosing;      /* Flag when tasks can not use sockets */
        unsigned char          fEvented;      /* Flag if we were "evented" */
        unsigned char          fAbortPoll;    /* Flag if fdSelectAbort() called */
      } FDTABLE;


/*-------------------------------------------------------------------- */
/* Private Functions */
extern FDTABLE  *fdint_getfdt( void *hTask );
extern void fdint_freefdt( FDTABLE *pfdt );
extern void fdint_signalevent( FDTABLE *pfdt );
extern void fdint_signaltimeout( FDTABLE *pfdt );
extern int  fdint_waitevent( FDTABLE *pfdt, uint32_t timeout );

extern int  fdint_setevent( FDTABLE *pfdt, FILEDESC *pfd, unsigned char EventFlags );
extern void fdint_clearevent( FDTABLE *pfdt, FILEDESC *pfd );

extern int  fdint_lockfd( FILEDESC *pfd, uint32_t Type );
extern void fdint_unlockfd( FILEDESC *pfd, uint32_t error );
extern void fdint_setinvalid( FILEDESC *pfdt );

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* _FDT_H */
