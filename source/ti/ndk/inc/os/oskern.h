/*
 * Copyright (c) 2012-2019, Texas Instruments Incorporated
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
 * ======== oskern.h ========
 *
 * Private include files for kernel objects
 *
 */

#ifndef _C_OSKERN_H
#define _C_OSKERN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Kernel HTYPE's */
#define HTYPE_CFG               0x0102
#define HTYPE_CFGENTRY          0x0103

/*--------------------------------------------- */
/*--------------------------------------------- */
/* TASK */
/*--------------------------------------------- */
/*--------------------------------------------- */

/* Task Functions used by the Scheduler */
extern void   _TaskInit();
extern void   _TaskShutdown();

/*--------------------------------------------- */
/*--------------------------------------------- */
/* MEMORY */
/*--------------------------------------------- */
/*--------------------------------------------- */

/* P.I.T. Entry */
typedef struct {
                unsigned char         *pPageStart;
                uint32_t        PageSize;
                uint32_t        BlockSize;
                uint32_t        BlockSizeIdx;
                uint32_t        BlocksPerPage;
                uint32_t        AllocCount;
                uint32_t        IdxFreeCheck;
               } PITENTRY;

/* Memory functions not used by the Scheduler */
extern int    _mmInit();
extern void   _mmCheck( uint32_t CallMode, int (*pPrn)(const char *,...) );
#define MMCHECK_MAP             0       /* Map out allocated memory */
#define MMCHECK_DUMP            1       /* Dump allocated block ID's */
#define MMCHECK_SHUTDOWN        2       /* Dump allocated block's & free */

extern void   _mmBulkAllocSeg( uint32_t segId );

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
