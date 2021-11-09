/*
 * Copyright (c) 2013-2017, Texas Instruments Incorporated
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
 * ======== mem.c ========
 *
 * Block oriented memory manager implemented using BIOS6
 *
 */

#include <stdint.h>

#include <netmain.h>
#include <_oskern.h>

/*
 * warning: order of this header matters, conflict with sys/select.h
 * happens if placed above netmain.h include.
 */
#include <stdlib.h>

/* Raw Memory Configuration */

extern const int ti_ndk_config_Global_rawPageSize;
extern const int ti_ndk_config_Global_rawPageCount;

/* Memory Block Structures */

/* Memory Block */
typedef struct {
                uint32_t        Flags;
#define MBF_BUSY        0x8000
#define MBF_INDEXMASK   0x03FF
                unsigned char         bData[1];
               } MEMORYBLOCK;

/* P.I.T. */
#ifdef _TMS320C6X
extern far PITENTRY ti_ndk_config_Global_pit[];
extern far unsigned char  ti_ndk_config_Global_pitBuffer[];
#else
extern PITENTRY ti_ndk_config_Global_pit[];
extern unsigned char  ti_ndk_config_Global_pitBuffer[];
#endif

/* P.I.T. Info */
static uint32_t   PITCount = 0;
static uint32_t   PITUsed  = 0;
static uint32_t   PITHigh  = 0;

/* Memory Bucket Information */

#define MEMORY_ID_COUNT         7

extern const int ti_ndk_config_Global_smallest;
extern const int ti_ndk_config_Global_largest;

/* Memory Slot Tracking */
#ifdef _TMS320C6X
extern far uint32_t ti_ndk_config_Global_Id2Size[];
#else
extern uint32_t ti_ndk_config_Global_Id2Size[];
#endif

/* High Water Mark Tracking */
static int BlockMax[ MEMORY_ID_COUNT ];
static int BlockCount[ MEMORY_ID_COUNT ];
static int PageMax[ MEMORY_ID_COUNT ];
static int PageCount[ MEMORY_ID_COUNT ];

static uint32_t  mmCalls     = 0;
static uint32_t  mmFrees     = 0;
static uint32_t  mmFails     = 0;
static uint32_t  mmBulkCalls = 0;
static uint32_t  mmBulkFails = 0;
static uint32_t  mmBulkFrees = 0;

/* Initialize the Memory System */
int _mmInit()
{
    int32_t w;

    /* Init PIT */
    PITCount = ti_ndk_config_Global_rawPageCount;
    PITCount = PITHigh = PITUsed = 0;
    for(w=0; w<ti_ndk_config_Global_rawPageCount; w++)
    {
        ti_ndk_config_Global_pit[w].PageSize = ti_ndk_config_Global_rawPageSize;
        ti_ndk_config_Global_pit[w].pPageStart =
                ti_ndk_config_Global_pitBuffer +
                (ti_ndk_config_Global_rawPageSize*w);
        ti_ndk_config_Global_pit[w].BlockSize  = 0;
        PITCount++;
    }

    /* Init Block Statistics */
    for(w=0; w<MEMORY_ID_COUNT; w++)
    {
        BlockMax[w]   = 0;
        BlockCount[w] = 0;
        PageMax[w]    = 0;
        PageCount[w]  = 0;
    }

    return(1);
}

/* Initialize a new Page, subdividing into blocks */
static void mmInitPage( uint32_t PitIdx, uint32_t SizeIdx )
{
    uint32_t         w;
    uint32_t         Inc;
    unsigned char*         pb;

    /* Init the PIT Entry */
    ti_ndk_config_Global_pit[PitIdx].BlockSize =
            ti_ndk_config_Global_Id2Size[SizeIdx];
    ti_ndk_config_Global_pit[PitIdx].BlockSizeIdx  = SizeIdx;
    ti_ndk_config_Global_pit[PitIdx].BlocksPerPage =
            ti_ndk_config_Global_pit[PitIdx].PageSize /
            ti_ndk_config_Global_Id2Size[SizeIdx];
    ti_ndk_config_Global_pit[PitIdx].AllocCount    = 0;
    ti_ndk_config_Global_pit[PitIdx].IdxFreeCheck  = 0;

    /* Init the Memory Blocks in the New Page */
    pb   = ti_ndk_config_Global_pit[PitIdx].pPageStart;
    Inc  = ti_ndk_config_Global_pit[PitIdx].BlockSize;
    for( w=0; w<ti_ndk_config_Global_pit[PitIdx].BlocksPerPage; w++ )
    {
        ((MEMORYBLOCK *)pb)->Flags = PitIdx;
        pb += Inc;
    }

    /* Bump used count */
    PITUsed++;
    if( PITUsed > PITHigh )
        PITHigh = PITUsed;

    /* Maintian Page Block Statistics */
    w = ti_ndk_config_Global_pit[PitIdx].BlockSizeIdx;
    PageCount[ w ]++;
    if( PageCount[ w ] > PageMax[ w ] )
        PageMax[ w ] = PageCount[ w ];
}

/* Uninit an sub-divided Page */
static void mmUnInitPage( uint32_t PitIdx )
{
    /* Init the PIT Entry */
    ti_ndk_config_Global_pit[PitIdx].BlockSize = 0;

    /* Dec used count */
    PITUsed--;

    /* Maintian Page Block */
    PageCount[ ti_ndk_config_Global_pit[PitIdx].BlockSizeIdx ]--;
}

/* Allocate Memory */
/* Note64: consider changing parameter to type size_t 64 bit */
void *mmAlloc( uint32_t Size )
{
    uint32_t UseSizeIdx,UseSize;
    uint32_t PitIdx, PitFree, tmp1, tmp2;
    MEMORYBLOCK* pmb;
    uint32_t CritState;

    /* Get index to bucket size, including memory block rsvd uint32_t */
    UseSize = Size+sizeof(uint32_t);

    /* Verify size request at boundary conditions first */
    if( UseSize <= (unsigned int)ti_ndk_config_Global_smallest )
        UseSizeIdx = 0;
    else if( UseSize > (unsigned int)ti_ndk_config_Global_largest )
        return(0);
    else
    {
        UseSizeIdx = MEMORY_ID_COUNT/2;
        if( ti_ndk_config_Global_Id2Size[UseSizeIdx] >= UseSize )
        {
            while( ti_ndk_config_Global_Id2Size[UseSizeIdx-1] >= UseSize )
                UseSizeIdx--;
        }
        else
        {
            while( ti_ndk_config_Global_Id2Size[UseSizeIdx] < UseSize )
                UseSizeIdx++;
        }
    }

    CritState = OEMSysCritOn();

    mmCalls++;

    /* Look for a PIT already using this size */
    PitIdx  = 0;                        /* Index */
    tmp1    = 0;                        /* Number of PITs examined */
    PitFree = PITCount;                 /* Set "free" to invalid */
    while( PitIdx < PITCount )
    {
        /* Only examined blocks currently in use */
        if( !ti_ndk_config_Global_pit[PitIdx].BlockSize )
            PitFree = PitIdx;
        else
        {
            /* Bump the "examined" count of USED entries */
            tmp1++;

            /* See if we can use this page. It must be our size, */
            /* plus have an entry available */
            if( ti_ndk_config_Global_pit[PitIdx].BlockSizeIdx == UseSizeIdx &&
                    ti_ndk_config_Global_pit[PitIdx].AllocCount <
                    ti_ndk_config_Global_pit[PitIdx].BlocksPerPage )
                goto MMA_PITVALID;
        }

        /* If we've checked all the used entries and have a free entry, */
        /* then use the free entry now */
        if( tmp1 == PITUsed && PitFree != PITCount )
        {
            /* Set Free Page */
            PitIdx = PitFree;

            /* Initialize free page */
            mmInitPage( PitIdx, UseSizeIdx );

            goto MMA_PITVALID;
        }

        /* Nothing found yet - try next entry */
        PitIdx++;
    }

    /* Here we didn't find a free or usable PIT, so we have an OOM */
    /* error or a fatal error */
    if( PITUsed != PITCount )
        DbgPrintf(DBG_ERROR,"mmAlloc: PIT Used Sync");
    goto MMA_ERROR;

MMA_PITVALID:
    /* Allocate the Memory */
    UseSize = ti_ndk_config_Global_Id2Size[UseSizeIdx];

    /* Init our search point (tmp1) to the most likely free block */
    tmp1 = tmp2 = ti_ndk_config_Global_pit[PitIdx].IdxFreeCheck;

    pmb = (MEMORYBLOCK *)(ti_ndk_config_Global_pit[PitIdx].pPageStart +
            (tmp1*UseSize));

    /* Find a free memory page */
    while( pmb->Flags & MBF_BUSY )
    {
        /* Bump the pmb */
        if( ++tmp1 == ti_ndk_config_Global_pit[PitIdx].BlocksPerPage )
        {
            tmp1 = 0;
            pmb = (MEMORYBLOCK *)(ti_ndk_config_Global_pit[PitIdx].pPageStart);
        }
        else
            pmb = (MEMORYBLOCK *)((unsigned char *)pmb + UseSize );

        /* Check for error (tmp1 wrapped) */
        if( tmp1 == tmp2 )
        {
            /* FATAL */
            DbgPrintf(DBG_ERROR,"mmAlloc: PIT FreeBlk Sync");
            goto MMA_ERROR;
        }
    }

    /* Allocate the memory page */
    pmb->Flags |= MBF_BUSY;
    ti_ndk_config_Global_pit[PitIdx].AllocCount++;

    /* Setup next possible free idx */
    if( ++tmp1 == ti_ndk_config_Global_pit[PitIdx].BlocksPerPage )
        tmp1 = 0;
    ti_ndk_config_Global_pit[PitIdx].IdxFreeCheck = tmp1;

    /* Maintain Block Statistics */
    BlockCount[ UseSizeIdx ]++;
    if( BlockCount[ UseSizeIdx ] > BlockMax[ UseSizeIdx ] )
        BlockMax[ UseSizeIdx ] = BlockCount[ UseSizeIdx ];

    OEMSysCritOff( CritState );
    return( pmb->bData );

MMA_ERROR:
    mmFails++;
    OEMSysCritOff( CritState );
    return( 0 );
}

/* Free Memory */
void mmFree( void *p )
{
    MEMORYBLOCK* pmb;
    uint32_t        PitIdx;
    uint32_t        CritState;

    pmb = (MEMORYBLOCK *)((unsigned char *)p - sizeof(uint32_t));

    /* Check for double free */
    if( !(pmb->Flags & MBF_BUSY) )
    {
        /* FATAL */
        DbgPrintf(DBG_WARN,"mmFree: Double Free");
        return;
    }

    /* Get the index */
    PitIdx = (uint32_t)(pmb->Flags & MBF_INDEXMASK);

    CritState = OEMSysCritOn();

    mmFrees++;

    /* Unallocate the block */
    pmb->Flags &= ~MBF_BUSY;

    /* Maintain Block */
    BlockCount[ ti_ndk_config_Global_pit[PitIdx].BlockSizeIdx ]--;

    /* Free the page if no blocks in use */
    if( !(--ti_ndk_config_Global_pit[PitIdx].AllocCount) )
        mmUnInitPage( PitIdx );

    OEMSysCritOff( CritState );

    return;
}

/* Memory Copy */
void mmCopy( void* pDst, void* pSrc, uint32_t len )
{
    uint32_t *pSrc32, *pDst32;
    unsigned char  *pSrc8, *pDst8;

    /* Fast Case */
    /* Check if addresses are word aligned */
    if( !(((uintptr_t)pDst) & TI_NDK_OS_MEM_WORD_ALIGN) &&
                !(((uintptr_t)pSrc) & TI_NDK_OS_MEM_WORD_ALIGN) )
    {
        pSrc32 = pSrc;
        pDst32 = pDst;

        while( len > 3 )
        {
            *pDst32++ = *pSrc32++;
            len -= 4;
        }

        if( len )
        {
            pSrc8 = (unsigned char *)pSrc32;
            pDst8 = (unsigned char *)pDst32;

            while( len-- )
                *pDst8++ = *pSrc8++;
        }
    }
    else
    {
        pSrc8 = pSrc;
        pDst8 = pDst;

        while( len-- )
            *pDst8++ = *pSrc8++;
    }
}

/* Memory Clear */
void mmZeroInit( void *pDst, uint32_t len )
{
    uint32_t *pDst32;
    unsigned char  *pDst8;

    pDst8 = pDst;

    /* Copy 'till aligned */
    while( (((uintptr_t)pDst8) & TI_NDK_OS_MEM_WORD_ALIGN) && len > 0 )
    {
        *pDst8++ = 0;
        len--;
    }

    if( len )
    {
        pDst32 = (uint32_t *)pDst8;

        /* Copy 'till less than 4 bytes */
        while( len > 3 )
        {
            *pDst32++ = 0;
            len -= 4;
        }

        if( len )
        {
            pDst8 = (unsigned char *)pDst32;

            /* Copy 'till done */
            while( len-- )
                *pDst8++ = 0;
        }
    }
}


/*------------------------------------------------------------- */
/* _mmBulkAllocSeg */
/* This function is used to change the default segment for */
/* mmBulkAlloc() and mmBulkFree(). This function will only */
/* work if no calls to mmBulkAlloc() have been made. */
/*------------------------------------------------------------- */
/* ARGSUSED */
void _mmBulkAllocSeg( uint32_t segId )
{
    /*
     *  For BIOS 6, the default Heap will be used for allocations.  If the user
     *  wishes to change the Heap for mmBulkAlloc(), this should be done using
     *  BIOS or XDC APIs as appropriate.
    uint32_t CritState;

    CritState = OEMSysCritOn();

    if( !mmBulkCalls )
        BulkSegId = segId;

    OEMSysCritOff( CritState );
     */
}


/*------------------------------------------------------------- */
/* mmBulkAlloc */
/* This function is used to allocate memory larger than */
/* 3000 bytes */
/*------------------------------------------------------------- */
// Note64: change param type to size_t?
void *mmBulkAlloc( int32_t Size )
{
    uint32_t *ptr;

    mmBulkCalls++;

    Size += 8;

    ptr = (uint32_t *)malloc((size_t)Size);

    /* ensure allocation is valid */
    if (ptr != NULL) {
        *ptr = 0x87654321;
        *(ptr+1) = (uint32_t)Size;
        return ((void *)(ptr+2));
    }
    else {
        /*
         *  fix SDOCM00100363 and SDOCM00100867.  No need to reset stack if
         *  allocation fails.
         */
        DbgPrintf(DBG_WARN, "mmBulkAlloc(): could not allocate memory.");
        mmBulkFails++;
        return (0);
   }
}

/*------------------------------------------------------------- */
/* mmBulkFree */
/* This function is used to free memory allocated with */
/* mmBulkAlloc() */
/*------------------------------------------------------------- */
void mmBulkFree( void *pMemory )
{
    uint32_t *ptr = (uint32_t *)pMemory;

    if( !ptr )
    {
        DbgPrintf(DBG_ERROR,"mmBulkFree: NULL pointer");
        return;
    }
    ptr -= 2;
    if( *ptr != 0x87654321 )
    {
        DbgPrintf(DBG_ERROR,"mmBulkFree: Corrupted mem or bad ptr (%08x)",ptr);
        return;
    }

    mmBulkFrees++;
    free((void *)ptr);
}

/*------------------------------------------------------------- */
/* Memory Check */
/* CallMode MMCHECK_MAP      : Map out allocated memory, but */
/*                             don't dump ID's */
/*          MMCHECK_DUMP     : Dump allocated block ID's */
/*          MMCHECK_SHUTDOWN : Dump allocated block's & free */
/*------------------------------------------------------------- */
void _mmCheck( uint32_t CallMode, int (*pPrn)(const char *,...) )
{
    uint32_t   w;
    uint32_t tmp;
    uint32_t   SizeIdx;
    uint32_t Total;
    uint32_t   PitIdx;

    if( !pPrn )
        goto SHUTDOWN;

    /* Memory Usage */
    SizeIdx = 0;
    Total    = 0;
    while( SizeIdx < MEMORY_ID_COUNT )
    {
        if( !(SizeIdx&3) )
            (*pPrn)("\n");
        (*pPrn)("%4u:%-4u ",BlockMax[SizeIdx],
                ti_ndk_config_Global_Id2Size[SizeIdx]);
        tmp = (uint32_t)(BlockMax[SizeIdx]) *
                (uint32_t)(ti_ndk_config_Global_Id2Size[SizeIdx]);
        Total  += tmp;
        if(tmp)
            (*pPrn)("(%3d%%)  ",
            (tmp*100)/((uint32_t)PageMax[SizeIdx]*
            (uint32_t)ti_ndk_config_Global_rawPageSize));
        else
            (*pPrn)("        ");

        SizeIdx++;
    }

    (*pPrn)("\n(%u/", PITHigh*(uint32_t)ti_ndk_config_Global_rawPageSize);
    (*pPrn)("%u",     PITCount*(int32_t)ti_ndk_config_Global_rawPageSize);
    (*pPrn)(" mmAlloc: %u/%u/%u,",mmCalls,mmFails,mmFrees);
    (*pPrn)(" mmBulk: %u/%u/%u)\n",mmBulkCalls,mmBulkFails,mmBulkFrees);

    /* Walk Memory */
    if( PITUsed )
    {
        (*pPrn)("\n");
        for( PitIdx=0; PitIdx<PITCount; PitIdx++ )
        {
            if( ti_ndk_config_Global_pit[PitIdx].BlockSize )
            {
                MEMORYBLOCK *pmb;

                if( !ti_ndk_config_Global_pit[PitIdx].AllocCount )
                    (*pPrn)("IE: No blocks in alloced page\n");
                else
                    (*pPrn)("%d blocks alloced in %d byte page\n",
                            ti_ndk_config_Global_pit[PitIdx].AllocCount,
                            ti_ndk_config_Global_pit[PitIdx].BlockSize );

                if( CallMode != MMCHECK_MAP )
                {
                    w = 0;
                    while( w < ti_ndk_config_Global_pit[PitIdx].BlocksPerPage )
                    {
                        pmb = (MEMORYBLOCK *)
                                (ti_ndk_config_Global_pit[PitIdx].pPageStart +
                                (w*ti_ndk_config_Global_pit[PitIdx].BlockSize));
                        if( pmb->Flags & MBF_BUSY )
                            (*pPrn)("(%04X)  ", *((uint32_t *)pmb->bData));
                        w++;
                    }
                    (*pPrn)("\n");
                }
            }
        }
    }

    (*pPrn)("\n");

SHUTDOWN:
    if( CallMode == MMCHECK_SHUTDOWN )
        PITCount = 0;
}
