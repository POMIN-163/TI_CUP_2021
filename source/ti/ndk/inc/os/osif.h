/*
 * Copyright (c) 2012-2020, Texas Instruments Incorporated
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
 * ======== osif.h ========
 *
 * OS Interface Functions
 *
 */

#ifndef _C_OSIF_H
#define _C_OSIF_H

#include <stdint.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

/*--------------------------------------------- */
/*--------------------------------------------- */
/* OS Environment Globals */
/*--------------------------------------------- */
/*--------------------------------------------- */

/* Configuration Structure */
typedef struct _osenvcfg {
    uint32_t     DbgPrintLevel;     /* Debug msg print threshhold */
    uint32_t     DbgAbortLevel;     /* Debug msg sys abort theshhold */
    int      TaskPriLow;        /* Lowest priority for stack task */
    int      TaskPriNorm;       /* Normal priority for stack task */
    int      TaskPriHigh;       /* High priority for stack task */
    int      TaskPriKern;       /* Kernel-level priority (highest) */
    int      TaskStkLow;        /* Minimum stack size */
    int      TaskStkNorm;       /* Normal stack size */
    int      TaskStkHigh;       /* Stack size for high volume tasks */
    int      TaskStkBoot;       /* Stack size for NS_BootTask */
    } OSENVCFG;

/* Configuration */
extern OSENVCFG _oscfg;

/* Equates used in code */
#define DBG_PRINT_LEVEL         (_oscfg.DbgPrintLevel)
#define DBG_ABORT_LEVEL         (_oscfg.DbgAbortLevel)
#define OS_TASKPRILOW           (_oscfg.TaskPriLow)
#define OS_TASKPRINORM          (_oscfg.TaskPriNorm)
#define OS_TASKPRIHIGH          (_oscfg.TaskPriHigh)
#define OS_TASKPRIKERN          (_oscfg.TaskPriKern)
#define OS_TASKSTKLOW           (_oscfg.TaskStkLow)
#define OS_TASKSTKNORM          (_oscfg.TaskStkNorm)
#define OS_TASKSTKHIGH          (_oscfg.TaskStkHigh)
#define OS_TASKSTKBOOT          (_oscfg.TaskStkBoot)

/* Default values */
#define DEF_DBG_PRINT_LEVEL     DBG_INFO
#define DEF_DBG_ABORT_LEVEL     DBG_ERROR
#define OS_TASKPRILOW_DEF       3
#define OS_TASKPRINORM_DEF      5
#define OS_TASKPRIHIGH_DEF      7
#define OS_TASKPRIKERN_DEF      9   /* Leave room to run scheduler at 8 */
#define OS_TASKSTKLOW_DEF       3072
#define OS_TASKSTKNORM_DEF      4096
#define OS_TASKSTKHIGH_DEF      5120
#define OS_TASKSTKBOOT_DEF      2048

#define OS_SCHEDULER_HIGHPRI    (_oscfg.TaskPriKern-1)
#define OS_SCHEDULER_LOWPRI     (_oscfg.TaskPriLow-1)

extern int stricmp(const char *s1, const char *s2);

/*----------------------------------------------------------------------- */
/*----[ PACKET BUFFER MANAGER ]------------------------------------------ */
/*----------------------------------------------------------------------- */

/* Packet Buffer Object */
typedef struct _PBM_Pkt {
    uint32_t        Type;         /* Identifier (Read Only) */
    struct _PBM_Pkt *pPrev;       /* Previous record */
    struct _PBM_Pkt *pNext;       /* Next record */
    unsigned char         *pDataBuffer; /* Pointer to Data Buffer (Read Only) */
    uint32_t        BufferLen;    /* Physical Length of buffer (Read Only) */
    uint32_t        Flags;        /* Packet Flags */
    uint32_t        ValidLen;     /* Length of valid data in buffer */
    uint32_t        DataOffset;   /* Byte offset to valid data */
    uint32_t        EtherType;    /* Ether Type Code */
    uint32_t        L2HdrLen;     /* Length of L2 Hdr (on 'L3' Rx pkts) */
    uint32_t        IpHdrLen;     /* Length of Ip Hdr */
    void            *hIFRx;        /* Rx Interface */
    void            *hIFTx;        /* Tx Interface */
    void            *hRoute;       /* Handle to Route */
    uint16_t        PktPriority;  /* Priority of the packet. */
    uint32_t        Aux1;         /* Aux1 Data */
    uint32_t        Aux2;         /* Aux2 Data */
    TimestampFxn    pTimestampFxn;/* Callout function pointer to */
                                  /* timestamp TX */
    unsigned char         *pIpHdr;      /* Pointer to IP Header  */

    /* Hardware Checksum Offload fields (used for devices with partial CSO) */
    uint32_t        csStartPos; /* Start byte for HW to begin CS computation */
    uint32_t        csNumBytes; /* Number of bytes to CS or RX pkt was a frag */
    uint32_t        csInsertPos; /* Byte offset where HW should insert CS */

#ifdef _INCLUDE_IPv6_CODE
    void            *hRoute6;      /* Handle to Route6 object. */
    IP6N            SrcAddress;   /* IPv6 Source Address of the packet  */
    uint32_t        SrcPort;
#endif
    } PBM_Pkt;

/* PBM Handle */
typedef void *PBM_Handle;

/* Packet Buffer Manager Initialization Functions */
extern uint32_t        PBM_open();
extern void        PBM_close();

/* Packet Buffer Functions (re-entrant and "kernel mode" agnostic) */
extern PBM_Handle  PBM_alloc( uint32_t MaxSize );
extern PBM_Handle  PBM_copy( PBM_Handle hPkt );
extern void        PBM_free( PBM_Handle hPkt );

/* The following field Functions can be used by device drivers */
/* All other PBM_Pkt fields are reserved */
#define PBM_getBufferLen(hPkt)      (((PBM_Pkt*)hPkt)->BufferLen)
#define PBM_getDataBuffer(hPkt)     (((PBM_Pkt*)hPkt)->pDataBuffer)
#define PBM_getValidLen(hPkt)       (((PBM_Pkt*)hPkt)->ValidLen)
#define PBM_getDataOffset(hPkt)     (((PBM_Pkt*)hPkt)->DataOffset)
#define PBM_getIFRx(hPkt)           (((PBM_Pkt*)hPkt)->hIFRx)

#define PBM_setValidLen(hPkt,x)     (((PBM_Pkt*)hPkt)->ValidLen=(x))
#define PBM_setDataOffset(hPkt,x)   (((PBM_Pkt*)hPkt)->DataOffset=(x))
#define PBM_setIFRx(hPkt,x)         (((PBM_Pkt*)hPkt)->hIFRx=(x))

/*----------------------------------------------------------------------- */
/*----[ PACKET BUFFER QUEUE ]-------------------------------------------- */
/*----------------------------------------------------------------------- */

/* Packet Buffer Queue Object */
typedef struct _PBMQ {
  uint32_t              Count;      /* Number of packets in queue */
  PBM_Pkt           *pHead;     /* Pointer to first packet */
  PBM_Pkt           *pTail;     /* Pointer to last packet */
} PBMQ;

/* Packet Queue Functions (re-entrant and "kernel mode" agnostic) */
#define            PBMQ_init(pQ)   mmZeroInit( (pQ), sizeof(PBMQ) )
#define            PBMQ_count(pQ) ((pQ)->Count)
extern void       PBMQ_enqHead( PBMQ *pQ, PBM_Handle hPkt );
extern void       PBMQ_enq( PBMQ *pQ, PBM_Handle hPkt );
extern PBM_Handle PBMQ_deq( PBMQ *pQ );

/*----------------------------------------------------------------------- */
/*----[ STACK EVENT OBJECT ]--------------------------------------------- */
/*----------------------------------------------------------------------- */

#define STKEVENT_NUMEVENTS    5

#define STKEVENT_TIMER        0
#define STKEVENT_ETHERNET     1
#define STKEVENT_SERIAL       2
#define STKEVENT_LINKUP       3
#define STKEVENT_LINKDOWN     4

/* Stack Event Object */
typedef struct _stkevent {
    void *hSemEvent;
    uint32_t EventCodes[STKEVENT_NUMEVENTS];
} STKEVENT;


/* STKEVENT Handle */
typedef void *STKEVENT_Handle;


/* Packet Queue Functions (kernel mode agnostic */

/* void STKEVENT_signal( STKEVENT_Handle hEventRec, uint32_t Event, */
/*                       uint32_t Ext ); */
/*          hEventRec       Handle to Event Record */
/*          Event           Event Code to Signal */
/*          Ext             Set to 1 if triggered by an external event (isr) */
/*                          Set to 0 if detected by polling function */
#define STKEVENT_signal(h,event,ext) { \
                                ((STKEVENT *)(h))->EventCodes[(event)]=1; \
                                if( (ext) && ((STKEVENT *)(h))->hSemEvent ) \
                                SemPostBinary( ((STKEVENT *)(h))->hSemEvent ); }

#define STKEVENT_init(h,hSem) { \
                                mmZeroInit( (h), sizeof(STKEVENT) ); \
                                ((STKEVENT *)(h))->hSemEvent = (hSem); }


/*--------------------------------------------- */
/*--------------------------------------------- */
/* TASK */
/*--------------------------------------------- */
/*--------------------------------------------- */

#define OS_TASKGETPRIFAIL -2
#define OS_TASKSETPRIFAIL -3

/*
 * Define struct to pass args to a thread function. Any task fxn must cast the
 * argument passed to it to a pointer to this struct in order to extract the
 * encapsulated arguments.
 */
typedef struct ti_ndk_os_TaskArgs {
    void (*arg0)();
    uintptr_t arg1;
    uintptr_t arg2;
} ti_ndk_os_TaskArgs;

/* These functions may need to be hooked or ported */
extern void   TaskBlock(void *h);
extern void *TaskCreate( void(*pFun)(), char *Name,
                          int Priority, uint32_t StackSize,
                          uintptr_t Arg1, uintptr_t Arg2, uintptr_t Arg3 );
extern void   TaskDestroy( void *h );
extern void   TaskExit();
extern void *TaskGetEnv( void *h, int Slot );
extern int    TaskGetPri(void *h);
extern void *TaskSelf();
extern void   TaskSetEnv( void *h, int Slot, void *hEnv );
extern int    TaskSetPri(void *h, int priority);
extern void   TaskSleep(uint32_t delay);
extern void   TaskYield();

/* Kernel Level Gateway Functions */
extern void  llEnter();
extern void  llExit();

/* signal that system resources are low */
extern void NotifyLowResource(void);

/*--------------------------------------------- */
/*--------------------------------------------- */
/* SEM */
/*--------------------------------------------- */
/*--------------------------------------------- */
/*#define SEM_FOREVER SYS_FOREVER */
#define SEM_FOREVER     ~(0)

/* SEM fxn mappings */
extern void *SemCreate(int Count);
extern void *SemCreateBinary(int Count);
extern void SemDelete(void *hSem);
extern void SemDeleteBinary(void *hSem);
extern int SemCount(void *hSem);
extern int SemPend(void *hSem, uint32_t Timeout);
extern int SemPendBinary(void *hSem, uint32_t Timeout);
extern void SemPost(void *hSem);
extern void SemPostBinary(void *hSem);
extern void SemReset(void *hSem, int Count);

/*--------------------------------------------- */
/*--------------------------------------------- */
/* MEMORY */
/*--------------------------------------------- */
/*--------------------------------------------- */

// Note64: better way to deterime 4 byte/8 byte alignment?
#define TI_NDK_OS_MEM_WORD_ALIGN ((sizeof(void *) - (size_t)(1)))

extern void   *mmAlloc( uint32_t Size );
extern void   mmFree( void* pv );
extern void   mmCopy( void* pDst, void* pSrc, uint32_t Size );
extern void   mmZeroInit( void* pDst, uint32_t Size );
extern void   *mmBulkAlloc( int32_t Size );
extern void   mmBulkFree( void *pMemory );

#ifdef _INCLUDE_JUMBOFRAME_SUPPORT
/*--------------------------------------------- */
/*--------------------------------------------- */
/* JUMBO MEMORY ( >3K ) */
/*--------------------------------------------- */
/*--------------------------------------------- */

extern void   *jumbo_mmAlloc( uint32_t Size );
extern void   jumbo_mmFree( void* pv );
#endif

/*--------------------------------------------- */
/*--------------------------------------------- */
/* DEBUG */
/*--------------------------------------------- */
/*--------------------------------------------- */
/* Debug Log */
extern int   DebugCritError;                /* Set on critical error */

extern void    DbgPrintf(uint32_t Level, char *fmt, ... );
#define DBG_INFO        1
#define DBG_WARN        2
#define DBG_ERROR       3
#define DBG_NONE        4

/*--------------------------------------------- */
/*--------------------------------------------- */
/* APIs to enter/exit critical sections */
/*--------------------------------------------- */
/*--------------------------------------------- */
extern uint32_t OEMSysCritOn();
extern void OEMSysCritOff( uint32_t enable );

/*--------------------------------------------- */
/*--------------------------------------------- */
/* PRINTF */
/*--------------------------------------------- */
/*--------------------------------------------- */

extern int  NDK_sprintf(char *s, const char *format, ...);
extern int  NDK_vsprintf(char *s, const char *format, va_list arg);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
