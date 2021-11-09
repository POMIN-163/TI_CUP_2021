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
 * ======== pipeif.h ========
 *
 * Pipe object include
 *
 */

#include "fdtif.h"

#ifndef _PIPEIF_INC_
#define _PIPEIF_INC_

#ifdef __cplusplus
extern "C" {
#endif

/* Pipe Object Structure */
typedef struct _pipe {
             FILEDESC     fd;           /* File descriptor header */
             struct _pipe *pConnect;    /* Pointer to other end of pipe */
             void         *hSBRx;        /* Rx Buffer */
             int32_t      TxSpace;      /* Tx Space Required for "writeable" */
        } PIPE;


/* Pipe Access Functions */

/*------------------------------------------------------------------------ */
/* General Access Functions (called from upper layers) */
extern int    PipeNew( void **phPipe1, void **phPipe2 );
extern int    PipeClose( void *hPipe );

extern int    PipeCheck( void *hPipe, int IoType );
#define  PIPE_READ       0
#define  PIPE_WRITE      1

extern int    PipeStatus( void *hPipe, int request, int *results );

extern int    PipeRecv( void *hPipe, char *pBuf, int32_t size,
                        int flags, int32_t *pRetSize );
extern int    PipeSend( void *hPipe, char *pBuf, int32_t size,
                        int flags, int32_t *pRetSize );

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
