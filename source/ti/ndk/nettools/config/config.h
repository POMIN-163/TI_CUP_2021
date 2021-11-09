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
 * ======== config.h ========
 *
 * Configuration Manager Private Defines
 *
 */

#ifndef _CONFIG_H
#define _CONFIG_H

#include <netmain.h>
#include <_oskern.h>

#ifdef __cplusplus
extern "C" {
#endif

/*--------------------------------------------------------------------------- */
/* Private Declarations */

/* Private Config Entry Structure */
typedef struct _cfgentry {
        uint32_t           Type;        /* Handle Type (HTYPE_CFGENTRY) */
        uint32_t           RefCount;    /* Reference Count */
        struct _cfgentry   *pNext;      /* Pointer to next in the chain */
        uint32_t           Active;      /* Entry is active */
        uint32_t           NoSave;      /* Entry is not saved */
        void               *hCfg;        /* Handle to configuration */
        uint32_t           Tag;         /* Configuration Tag */
        uint32_t           Item;        /* Configuration Item */
        int                DataSize;    /* User data size */
        unsigned char            UserData[1]; /* User data */
        } CFGENTRY;

/* Private Config Structure */
typedef struct _cfgmain {
        uint32_t Type;                       /* Handle Type (HTYPE_CFG) */
        uint32_t Active;                     /* Active config flag */
        void  *hOwner;                       /* Owning task when locked */
        void  *hSem;                         /* Onwer's Semaphore */
        uint32_t OwnCnt;                     /* Owner's count */
        uint32_t OpenOrder[CFGTAG_MAX];      /* List of Open Order for Tags */
        uint32_t CloseOrder[CFGTAG_MAX];     /* List of Close Order for Tags */
        /* List of entry chains by tag */
        struct _cfgentry *pCfgEntry[CFGTAG_MAX];
        /* List of service providers */
        int (*pCbService[CFGTAG_MAX])(void *,uint32_t,uint32_t,uint32_t,void *);
        } CFGMAIN;

/* Saved Config Entry Structure (simplified for space) */
typedef struct _cfgentrysave {
        uint32_t           Record;      /* Record number */
        uint32_t           RecordSize;  /* Total Record Size */
        uint32_t           Tag;         /* Configuration Tag */
        uint32_t           Item;        /* Configuration Item */
        int                DataSize;    /* User data size */
        unsigned char            UserData[1]; /* User data */
        } CFGENTRYSAVE;

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif

