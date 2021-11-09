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
 * ======== ip.c ========
 *
 * Routines related to IP layer
 *
 */

#include <stkmain.h>
#include "ip.h"

uint32_t _IPExecuting = 0;

IPSTATS NDK_ips;

static void *hRtCache = 0;
static uint32_t IPCache;

/*-------------------------------------------------------------------- */
/* IPMsg() */
/* Sevices intialization and resource messages */
/*-------------------------------------------------------------------- */
void IPMsg( uint32_t Msg )
{
    static void *hTimer;

    switch( Msg )
    {
    /* System Initialization */
    case MSG_EXEC_SYSTEM_INIT:
        /* Timer is a prime to keep it staggered from others */
        hTimer = TimerNew( &IPMsg, TIMER_TICKS_IP, MSG_IP_TIMER );
        mmZeroInit( &NDK_ips, sizeof( IPSTATS ) );
        _IPExecuting  = 1;
        break;

    /* System Shutdown */
    case MSG_EXEC_SYSTEM_SHUTDOWN:
        _IPExecuting = 0;
        /* Flush the IP Reassembly queue */
        while( _IPReasmPtr )
            IPReasmFree( _IPReasmPtr, 1 );

        /* DeRef the cached route */
        if( hRtCache )
        {
            RtDeRef( hRtCache );
            hRtCache = 0;
        }
        if( hTimer )
            TimerFree( hTimer );
        break;

    /* Half Second Timer */
    case MSG_IP_TIMER:
        if( _IPReasmPtr )
            IPReasmTimeout();
        break;
    }
}

/*-------------------------------------------------------------------- */
/* void IPChecksum( unsigned char *pbHdr ) */
/* Checksums an IP header */
/*-------------------------------------------------------------------- */
void IPChecksum( IPHDR *pIpHdr )
{
    int     tmp1;
    uint16_t  *pw;
    uint32_t  TSum = 0;

    /* Get header size in 4 byte chunks */
    tmp1 = pIpHdr->VerLen & 0xF;

    /* Checksum field is NULL in checksum calculations */
    pIpHdr->Checksum = 0;

    /* Checksum the header */
    pw = (uint16_t *)pIpHdr;
    do {
        TSum += (uint32_t)*pw++;
        TSum += (uint32_t)*pw++;
        /*
         * Note: more work here. Hit inf loop b/c VerLen was 0. This change
         * prevents the inf loop, but why was VerLen 0? Seems like bad pkt ...
         */
        tmp1 = tmp1 - 1;
    } while( tmp1 > 0 );
    TSum = (TSum&0xFFFF) + (TSum>>16);
    TSum = (TSum&0xFFFF) + (TSum>>16);
    TSum = ~TSum;

    /* Note checksum is Net/Host byte order independent */
    pIpHdr->Checksum = (uint16_t)TSum;
}

/*-------------------------------------------------------------------- */
/* IPGetRoute( uint32_t RtCallFlags, uint32_t IPDst ) */
/* Returns a route to the final IP destination. */
/*-------------------------------------------------------------------- */
void *IPGetRoute( uint32_t RtCallFlags, uint32_t IPDst )
{
    void *hRt, *hRtGate;

    /* First, try the cached route */
    if( hRtCache && IPCache == IPDst )
    {
        NDK_ips.CacheHit++;
        RtRef( hRtCache );
        return( hRtCache );
    }

    /* Find the route */
    if( !(hRt = RtFind( RtCallFlags, IPDst )) )
        return(0);

    /* If this is a GATEWAY route, we need to get the route to */
    /* the GateIP */
    if( RtGetFlags( hRt ) & FLG_RTE_GATEWAY )
    {
        /* Get a route to the Gateway IP */
        IPDst = RtGetGateIP( hRt );
        RtDeRef( hRt );

        /* First, try the cached route */
        if( hRtCache && IPCache == IPDst )
        {
            NDK_ips.CacheHit++;
            RtRef( hRtCache );
            return( hRtCache );
        }

        /* Find the route to the gateway */
        /* Note that this time, only HOST routes are allowed */
        if( !(hRtGate = RtFind( RtCallFlags|FLG_RTF_HOST, IPDst )) )
            return(0);

        /* Switch to the Gateway route */
        hRt = hRtGate;
    }

    /* We've officially missed the cache */
    NDK_ips.CacheMiss++;

    /* As this is not the cached route, if it is a host route, we'll */
    /* make it the new cache route. */
    if( RtGetFlags( hRt ) & FLG_RTE_HOST )
    {
        if( hRtCache )
            RtDeRef( hRtCache );
        RtRef( hRt );
        hRtCache = hRt;
        IPCache  = IPDst;
    }

    /* Return whatever route we found */
    return( hRt );
}

/*-------------------------------------------------------------------- */
/* IPRtChange( void *hRt ) */
/* Called to notify IP that a route has changed. */
/*-------------------------------------------------------------------- */
void IPRtChange( void *hRt )
{
    /* If the changed route is our cached route, then toss it */
    if( hRtCache == hRt )
    {
        hRtCache = 0;
        RtDeRef( hRt );
    }
}

/*-------------------------------------------------------------------- */
/* IPIsLoopback( uint32_t IPDst ) */
/* Check if a PBM_Pkt is destined to a loopback address */
/*-------------------------------------------------------------------- */
bool IPIsLoopback( uint32_t IPDst )
{
    /*
     * pPkt will be considered destined to a loopback address if the packet's
     * ip dest is "127.0.0.1" OR if IPDst is bound to an interface
     */
    return ((IPDst == NDK_LOOPBACK) || BindIPHost2IF(IPDst));
}
