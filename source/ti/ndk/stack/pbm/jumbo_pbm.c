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
 * ======== jumbo_pbm.c ========
 *
 * Jumbo Packet Buffer Manager.
 *
 * This file has functions which handle dynamic memory
 * management (heap allocations) for packet buffer
 * sizes greater than 3K. These functions are typically
 * called for buffer management of jumbo frames greater
 * than 3K size.
 *
 */
#ifdef _INCLUDE_JUMBOFRAME_SUPPORT

#include <netmain.h>
#include <_oskern.h>

/**********************************************************************
 ******************* Global Variables / Definitions *******************
 **********************************************************************/

/*
 * Raw Memory Configuration
 */

/* Maximum size of memory block that can be allocated = 10K */
#define JUMBO_RAW_PAGE_SIZE           	10240

/* Number of such Maximum size blocks that can be allocated.
 * This number is limited by the memory available to the
 * system.
 */
#define JUMBO_RAW_PAGE_COUNT          	16

/*
 * Memory Block Definitions
 */
#define MBF_BUSY        				0x8000
#define MBF_INDEXMASK   				0x03FF

/** 
 * @brief 
 *  This structure defines the Jumbo memory block structure.
 *
 * @details
 *  This structure signifies each memory block allocation
 *  done using the jumbo packet buffer manager.
 *  
 */
typedef struct JUMBO_MEMORYBLOCK
{
    /**
     * @brief   Flags to indicate the memory block status.
     */        
    uint32_t    Flags;

    /**
     * @brief   Pointer to the memory block.
     */        
    unsigned char     bData[1];
} JUMBO_MEMORYBLOCK;

/** 
 * @brief 
 *  This structure defines the Page Information entry.
 *
 * @details
 *  This structure holds details of the Pages within
 *  a memory block.
 *  
 */
typedef struct {
    /**
     * @brief   Pointer to the first page / partition.
     */        
    unsigned char     *pPageStart;
    /**
     * @brief   Size of the page.
     */        
    uint32_t    PageSize;
    /**
     * @brief   Block size.
     */        
    uint32_t    BlockSize;
    /**
     * @brief   Index of the block size in the block sizes
     *          array.
     */        
    uint32_t    BlockSizeIdx;
    /**
     * @brief   Number of blocks per page.
     */        
    uint32_t    BlocksPerPage;
    /**
     * @brief   Number of pages allocated.
     */        
    uint32_t    AllocCount;
    /**
     * @brief   Index to the first possibly free page to start
     *          searching for next free slot. 
     */            
    uint32_t    IdxFreeCheck;
} JUMBO_PITENTRY;

/* P.I.T */
#ifdef __TI_COMPILER_VERSION__
#if defined(_TMS320C6X)
#pragma DATA_SECTION(jumbo_pit, ".far:NDK_JMMBUFFER");
#pragma DATA_SECTION(jumbo_pitBuffer, ".far:NDK_JMMBUFFER");
#else
#pragma DATA_SECTION(jumbo_pit, ".bss:NDK_JMMBUFFER");
#pragma DATA_SECTION(jumbo_pitBuffer, ".bss:NDK_JMMBUFFER");
#endif
#endif
JUMBO_PITENTRY jumbo_pit[JUMBO_RAW_PAGE_COUNT];
unsigned char    jumbo_pitBuffer[JUMBO_RAW_PAGE_SIZE*JUMBO_RAW_PAGE_COUNT];

/* P.I.T. Info */
uint32_t   JumboPITCount = 0;
uint32_t   JumboPITUsed  = 0;
uint32_t   JumboPITHigh  = 0;

/*
 * Memory Bucket Information
 */

#define JUMBO_MEMORY_ID_COUNT   7
#define JUMBO_SMALLEST          3072
#define JUMBO_LARGEST           (JUMBO_RAW_PAGE_SIZE)

/* Memory Slot Tracking */
#ifdef __TI_COMPILER_VERSION__
#if defined(_TMS320C6X)
#pragma DATA_SECTION(JumboId2Size, ".far:NDK_JMMBUFFER");
#else
#pragma DATA_SECTION(JumboId2Size, ".bss:NDK_JMMBUFFER");
#endif
#endif
uint32_t JumboId2Size[]  = { JUMBO_SMALLEST, 4096, 5120, 6144, 7168, 8192, JUMBO_LARGEST };

/* High Water Mark Tracking */
int JumboBlockMax[ JUMBO_MEMORY_ID_COUNT ];
int JumboBlockCount[ JUMBO_MEMORY_ID_COUNT ];
int JumboPageMax[ JUMBO_MEMORY_ID_COUNT ];
int JumboPageCount[ JUMBO_MEMORY_ID_COUNT ];

static uint32_t  jmmCalls     = 0;
static uint32_t  jmmFrees     = 0;
static uint32_t  jmmFails     = 0;

/**********************************************************************
 ************* Jumbo Frame Memory Management functions ****************
 **********************************************************************/

/** 
 *  @b Description
 *  @n  
 *      This function initializes the jumbo frame memory system.
 *
 *  @param[in]  
 *      None
 *
 *  @retval
 *      Always returns 1 to indicate success.
 */
int _jumbo_mmInit()
{
    uint32_t w;

    /* Init PIT */
    JumboPITCount = JUMBO_RAW_PAGE_COUNT;
    JumboPITCount = JumboPITHigh = JumboPITUsed = 0;
    for(w=0; w<JUMBO_RAW_PAGE_COUNT; w++)
    {
        jumbo_pit[w].PageSize   = JUMBO_RAW_PAGE_SIZE;
        jumbo_pit[w].pPageStart = jumbo_pitBuffer + (JUMBO_RAW_PAGE_SIZE*w);
        jumbo_pit[w].BlockSize  = 0;
        JumboPITCount++;
    }

    /* Init Block Statistics */
    for(w=0; w<JUMBO_MEMORY_ID_COUNT; w++)
    {
        JumboBlockMax[w]   = 0;
        JumboBlockCount[w] = 0;
        JumboPageMax[w]    = 0;
        JumboPageCount[w]  = 0;
    }

    return(1);
}

/** 
 *  @b Description
 *  @n  
 *      This function initializes a new Page, subdividing into blocks.
 *
 *  @param[in]  PitIdx
 *      Page Index in the Page Info Table.
 *  @param[in]  SizeIdx
 *      Block size index to indicate what size blocks the page
 *      needs to be divided into.
 *
 *  @retval
 *      None.
 */
static void jumbo_mmInitPage( uint32_t PitIdx, uint32_t SizeIdx )
{
    uint32_t     w;
    uint32_t     Inc;
    unsigned char*     pb;

    /* Init the PIT Entry */
    jumbo_pit[PitIdx].BlockSize     = JumboId2Size[SizeIdx];
    jumbo_pit[PitIdx].BlockSizeIdx  = SizeIdx;
    jumbo_pit[PitIdx].BlocksPerPage = jumbo_pit[PitIdx].PageSize / JumboId2Size[SizeIdx];
    jumbo_pit[PitIdx].AllocCount    = 0;
    jumbo_pit[PitIdx].IdxFreeCheck  = 0;

    /* Init the Memory Blocks in the New Page */
    pb   = jumbo_pit[PitIdx].pPageStart;
    Inc  = jumbo_pit[PitIdx].BlockSize;
    for( w=0; w<jumbo_pit[PitIdx].BlocksPerPage; w++ )
    {
        ((JUMBO_MEMORYBLOCK *)pb)->Flags = PitIdx;
        pb += Inc;
    }

    /* Bump used count */
    JumboPITUsed++;
    if( JumboPITUsed > JumboPITHigh )
        JumboPITHigh = JumboPITUsed;

    /* Maintian Page Block Statistics */
    w = jumbo_pit[PitIdx].BlockSizeIdx;
    JumboPageCount[ w ]++;
    if( JumboPageCount[ w ] > JumboPageMax[ w ] )
        JumboPageMax[ w ] = JumboPageCount[ w ];
}

/** 
 *  @b Description
 *  @n  
 *      This function de-initializes a sub-divided Page.
 *
 *  @param[in]  PitIdx
 *      Page Index in the Page Info Table.
 *
 *  @retval
 *      None.
 */
static void jumbo_mmUnInitPage( uint32_t PitIdx )
{
    /* Init the PIT Entry */
    jumbo_pit[PitIdx].BlockSize = 0;

    /* Dec used count */
    JumboPITUsed--;

    /* Maintian Page Block */
    JumboPageCount[ jumbo_pit[PitIdx].BlockSizeIdx ]--;
}

/** 
 *  @b Description
 *  @n  
 *      This function allocates the required memory size
 *      from jumbo memory area.
 *
 *  @param[in]  Size
 *      Page Index in the Page Info Table.
 *
 *  @retval
 *      Returns the pointer to allocated memory block.
 */
void *jumbo_mmAlloc( uint32_t Size )
{
    uint32_t UseSizeIdx,UseSize;
    uint32_t PitIdx, PitFree, tmp1, tmp2;
    JUMBO_MEMORYBLOCK* pmb;
    uint32_t CritState;

    /* Get index to bucket size, including memory block rsvd uint32_t */
    UseSize = Size+sizeof(uint32_t);

    /* Verify size request at boundary conditions first */
    if( UseSize <= JUMBO_SMALLEST )
        UseSizeIdx = 0;
    else if( UseSize > JUMBO_LARGEST )
        return(0);
    else
    {
        UseSizeIdx = JUMBO_MEMORY_ID_COUNT/2;
        if( JumboId2Size[UseSizeIdx] >= UseSize )
        {
            while( JumboId2Size[UseSizeIdx-1] >= UseSize )
                UseSizeIdx--;
        }
        else
        {
            while( JumboId2Size[UseSizeIdx] < UseSize )
                UseSizeIdx++;
        }
    }

    CritState = OEMSysCritOn();

    jmmCalls++;

    /* Look for a PIT already using this size */
    PitIdx  = 0;                        /* Index */
    tmp1    = 0;                        /* Number of PITs examined */
    PitFree = JumboPITCount;                 /* Set "free" to invalid */
    while( PitIdx < JumboPITCount )
    {
        /* Only examined blocks currently in use */
        if( !jumbo_pit[PitIdx].BlockSize )
            PitFree = PitIdx;
        else
        {
            /* Bump the "examined" count of USED entries */
            tmp1++;

            /* See if we can use this page. It must be our size,
             * plus have an entry available
             */
            if( jumbo_pit[PitIdx].BlockSizeIdx == UseSizeIdx &&
                    jumbo_pit[PitIdx].AllocCount < jumbo_pit[PitIdx].BlocksPerPage )
                goto MMA_PITVALID;
        }

        /* If we've checked all the used entries and have a free entry,
         * then use the free entry now
         */
        if( tmp1 == JumboPITUsed && PitFree != JumboPITCount )
        {
            /* Set Free Page */
            PitIdx = PitFree;

            /* Initialize free page*/
            jumbo_mmInitPage( PitIdx, UseSizeIdx );

            goto MMA_PITVALID;
        }

        /* Nothing found yet - try next entry */
        PitIdx++;
    }

    /* Here we didn't find a free or usable PIT, so we have an OOM
     * error or a fatal error
     */
    if( JumboPITUsed != JumboPITCount )
        DbgPrintf(DBG_ERROR,"jumbo_mmAlloc: PIT Used Sync");
    goto MMA_ERROR;

MMA_PITVALID:
    /* Allocate the Memory */
    UseSize = JumboId2Size[UseSizeIdx];

    /* Init our search point (tmp1) to the most likely free block */
    tmp1 = tmp2 = jumbo_pit[PitIdx].IdxFreeCheck;

    pmb = (JUMBO_MEMORYBLOCK *)(jumbo_pit[PitIdx].pPageStart+(tmp1*UseSize));

    /* Find a free memory page */
    while( pmb->Flags & MBF_BUSY )
    {
        /* Bump the pmb */
        if( ++tmp1 == jumbo_pit[PitIdx].BlocksPerPage )
        {
            tmp1 = 0;
            pmb = (JUMBO_MEMORYBLOCK *)(jumbo_pit[PitIdx].pPageStart);
        }
        else
            pmb = (JUMBO_MEMORYBLOCK *)((unsigned char *)pmb + UseSize );

        /* Check for error (tmp1 wrapped) */
        if( tmp1 == tmp2 )
        {
            /* FATAL */
            DbgPrintf(DBG_ERROR,"jumbo_mmAlloc: PIT FreeBlk Sync");
            goto MMA_ERROR;
        }
    }

    /* Allocate the memory page */
    pmb->Flags |= MBF_BUSY;
    jumbo_pit[PitIdx].AllocCount++;

    /* Setup next possible free idx */
    if( ++tmp1 == jumbo_pit[PitIdx].BlocksPerPage )
        tmp1 = 0;
    jumbo_pit[PitIdx].IdxFreeCheck = tmp1;

    /* Maintain Block Statistics */
    JumboBlockCount[ UseSizeIdx ]++;
    if( JumboBlockCount[ UseSizeIdx ] > JumboBlockMax[ UseSizeIdx ] )
        JumboBlockMax[ UseSizeIdx ] = JumboBlockCount[ UseSizeIdx ];

    OEMSysCritOff( CritState );
    return( pmb->bData );

MMA_ERROR:
    jmmFails++;
    OEMSysCritOff( CritState );
    return( 0 );
}

/** 
 *  @b Description
 *  @n  
 *      This function frees the memory pointed to 
 *      by the arguments to it.
 *
 *  @param[in]  p
 *     Pointer to the memory block that is to be 
 *     freed.
 *
 *  @retval
 *      None.
 */
void jumbo_mmFree( void *p )
{
    JUMBO_MEMORYBLOCK* pmb;
    uint32_t        PitIdx;
    uint32_t        CritState;

    pmb = (JUMBO_MEMORYBLOCK *)((unsigned char *)p - sizeof(uint32_t));

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

    jmmFrees++;

    /* Unallocate the block */
    pmb->Flags &= ~MBF_BUSY;

    /* Maintain Block */
    JumboBlockCount[ jumbo_pit[PitIdx].BlockSizeIdx ]--;

    /* Free the page if no blocks in use */
    if( !(--jumbo_pit[PitIdx].AllocCount) )
        jumbo_mmUnInitPage( PitIdx );

    OEMSysCritOff( CritState );

    return;
}

/** 
 *  @b Description
 *  @n  
 *      This function iterates through the PIT table and 
 *      dumps the current memory usage stats according
 *      to the arguments specified.
 *
 *  @param[in]  CallMode
 *      This argument specifies the type of stats / 
 *      operation that needs to be performed by this function.
 *      The 3 supported call modes are:
 *      MMCHECK_MAP      : Map out allocated memory, but
 *                         don't dump ID's
 *      MMCHECK_DUMP     : Dump allocated block ID's
 *      MMCHECK_SHUTDOWN : Dump allocated block's & free
 *     
 *  @param[in]  pPrn
 *     Callback function pointer to be notified.
 *
 *  @retval
 *      None.
 */
void _jumbo_mmCheck( uint32_t CallMode, int (*pPrn)(const char *,...) )
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
    while( SizeIdx < JUMBO_MEMORY_ID_COUNT )
    {
        if( !(SizeIdx&3) )
            (*pPrn)("\n");
        (*pPrn)("%4u:%-4u ",JumboBlockMax[SizeIdx],JumboId2Size[SizeIdx]);
        tmp = (uint32_t)(JumboBlockMax[SizeIdx]) * (uint32_t)(JumboId2Size[SizeIdx]);
        Total  += tmp;
        if(tmp)
            (*pPrn)("(%3d%%)  ",
            (tmp*100)/((uint32_t)JumboPageMax[SizeIdx]*(uint32_t)JUMBO_RAW_PAGE_SIZE));
        else
            (*pPrn)("        ");

        SizeIdx++;
    }

    (*pPrn)("\n(%u/", JumboPITHigh*(uint32_t)JUMBO_RAW_PAGE_SIZE);
    (*pPrn)("%u",     JumboPITCount*(int32_t)JUMBO_RAW_PAGE_SIZE);
    (*pPrn)(" jumbo_mmAlloc: %u/%u/%u,",jmmCalls,jmmFails,jmmFrees);

    /* Walk Memory */
    if( JumboPITUsed )
    {
        (*pPrn)("\n");
        for( PitIdx=0; PitIdx<JumboPITCount; PitIdx++ )
        {
            if( jumbo_pit[PitIdx].BlockSize )
            {
                JUMBO_MEMORYBLOCK *pmb;

                if( !jumbo_pit[PitIdx].AllocCount )
                    (*pPrn)("IE: No blocks in alloced page\n");
                else
                    (*pPrn)("%d blocks alloced in %d byte page\n",
                           jumbo_pit[PitIdx].AllocCount, jumbo_pit[PitIdx].BlockSize );

                if( CallMode != MMCHECK_MAP )
                {
                    w = 0;
                    while( w < jumbo_pit[PitIdx].BlocksPerPage )
                    {
                        pmb = (JUMBO_MEMORYBLOCK *) (jumbo_pit[PitIdx].pPageStart +
                                               (w*jumbo_pit[PitIdx].BlockSize));
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
        JumboPITCount = 0;
}

#endif /* _INCLUDE_JUMBOFRAME_SUPPORT */
