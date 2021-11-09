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
 * ======== sockpcb.c ========
 *
 * Object member functions for the Sock device object. These
 * functions include protocol control type access to the SOCKET
 * object.
 *
 */

#include <stkmain.h>
#include "sock.h"
#include "../fdt/fdt.h"

static SOCK *pSockList[] = { 0, 0, 0, 0, 0 };

/* Next User Port for Ephemeral Ports */
static uint32_t wUserPort = SOCK_USERPORT_FIRST;

static SOCK *SockPcbFind( SOCK *psFirst, uint32_t LIP, uint32_t LPort,
                          uint32_t FIP, uint32_t FPort, int *pwc, int FindAll );

static SOCK *SockPcbFindRaw( SOCK *psFirst, uint32_t Prot, uint32_t LIP,
                             uint32_t FIP, int *pwc, int FindAll );

/*-------------------------------------------------------------------- */
/* SockPcbAttach() */
/* Adds a socket to the supplied socket list */
/*-------------------------------------------------------------------- */
int SockPcbAttach( void *hSock )
{
    SOCK *ps = (SOCK *)hSock;

#ifdef _STRONG_CHECKING
    if( ps->fd.Type != HTYPE_SOCK )
    {
        DbgPrintf(DBG_ERROR,"SockPcbAttach: HTYPE %04x",ps->fd.Type);
        return( 0 );
    }

    if( ps->SockProt > 4 )
    {
        DbgPrintf(DBG_ERROR,"SockPcbAttach: Invalid SockProt");
        return( 0 );
    }
#endif

    /* We'll insert at head of list since its quicker */
    ps->pProtPrev = 0;
    ps->pProtNext = pSockList[ps->SockProt];
    pSockList[ps->SockProt] = (void *)ps;

    /* Patch entry which follows us (if any) */
    if( ps->pProtNext )
        ps->pProtNext->pProtPrev = ps;

    return(0);
}

/*-------------------------------------------------------------------- */
/* SockPcbDetach() */
/* Removes a socket to the supplied socket list */
/*-------------------------------------------------------------------- */
int SockPcbDetach( void *hSock )
{
    SOCK *ps   = (SOCK *)hSock;

#ifdef _STRONG_CHECKING
    if( ps->fd.Type != HTYPE_SOCK )
    {
        DbgPrintf(DBG_ERROR,"SockPcbDetach: HTYPE %04x",ps->fd.Type);
        return( 0 );
    }

    if( ps->SockProt > 4 )
    {
        DbgPrintf(DBG_ERROR,"SockPcbDetach: Invalid SockProt");
        return( 0 );
    }
#endif

    /* Patch preceeding entry */
    if( !ps->pProtPrev )
        pSockList[ps->SockProt] = (void *)ps->pProtNext;
    else
        ps->pProtPrev->pProtNext = ps->pProtNext;

    /* Patch following entry */
    if( ps->pProtNext )
        ps->pProtNext->pProtPrev = ps->pProtPrev;

    /* Just to be tidy... */
    ps->pProtPrev = 0;
    ps->pProtNext = 0;

    return(0);
}

/*-------------------------------------------------------------------- */
/* SockPcbFind() */
/* Find the "best" match for the supplied parameters. */
/*-------------------------------------------------------------------- */
static SOCK *SockPcbFind( SOCK *ps, uint32_t LIP, uint32_t LPort, uint32_t FIP,
                          uint32_t FPort, int *pwc, int FindAll )
{
    SOCK *psBest = 0;
    int  wcBest = 9;
    int  wc;

    /* Find the "best" match for the supplied parameters. */

    for( ; ps; ps = ps->pProtNext )
    {
        /* Clear wild cards */
        wc = 0;

        /* Local port NULL means socket is not bound */
        if( !ps->LPort )
            continue;

        /* If local ports don't match, this entry can't match */
        if( ps->LPort != LPort )
            continue;

        /* Check if the local IP is a multicast packet. In that case we need to search
         * the multicast socket chain for a match */
        if (IN_MULTICAST(LIP))
        {
            MCAST_SOCK_REC* ptr_mcast_rec;

            /* Cycle through all the multicast records on the socket. */
            ptr_mcast_rec = (MCAST_SOCK_REC *)list_get_head ((NDK_LIST_NODE**)&ps->pMcastList);
            while (ptr_mcast_rec != NULL)
            {
                /* Check if we get a hit? */
                if (ptr_mcast_rec->mreq.imr_multiaddr.s_addr == LIP)
                    return ps;

                /* Get the next multicast record. */
                ptr_mcast_rec = (MCAST_SOCK_REC *)list_get_next((NDK_LIST_NODE*)ptr_mcast_rec);
            }

            /* Control comes here on no match. */
            continue;
       }

        /* Local IP must match, or be wildcard, or be a Broadcast */
        if( ps->LIP != LIP )
        {
            if (!LIP || !ps->LIP || LIP == INADDR_BROADCAST)
                wc++;
            else
                continue;
        }

        /* Foreign Port+IP must both match or both be wildcard */
        if( ps->FPort != FPort || ps->FIP != FIP )
        {
            if( (!FIP && !FPort) || (!ps->FIP && !ps->FPort) )
                wc++;
            else
                continue;
        }

        if( wc < wcBest )
        {
            wcBest = wc;
            psBest = ps;
            if( !wcBest )
                break;
        }

        if( FindAll )
            break;
    }

    if( pwc )
        *pwc = wcBest;
    return(psBest);
}

/*-------------------------------------------------------------------- */
/* SockPcbFindRaw() */
/* Find the "best" match for the supplied parameters. */
/*-------------------------------------------------------------------- */
static SOCK *SockPcbFindRaw( SOCK *ps, uint32_t Prot, uint32_t LIP, uint32_t FIP,
                             int *pwc, int FindAll )
{
    SOCK *psBest = 0;
    int  wcBest = 9;
    int  wc;

    /* Find the "best" match for the supplied parameters. */

    for( ; ps; ps = ps->pProtNext )
    {
        /* Clear wild cards */
        wc = 0;

        /* Protocol must match, or be wildcard */
        if( ps->Protocol && ps->Protocol != Prot )
            continue;

        /* Local IP must match, or be wildcard */
        if( ps->LIP != LIP )
        {
            if( !LIP || !ps->LIP )
                wc++;
            else
                continue;
        }

        /* Foreign IP must match, or be wildcard */
        if( ps->FIP != FIP )
        {
            if( !FIP || !ps->FIP )
                wc++;
            else
                continue;
        }

        if( wc < wcBest )
        {
            wcBest = wc;
            psBest = ps;
            if( !wcBest )
                break;
        }

        if( FindAll )
            break;
    }

    if( pwc )
        *pwc = wcBest;
    return(psBest);
}

/*-------------------------------------------------------------------- */
/* SockPcbBind() */
/* Binds a socket to specified local LIP and LPort */
/* Note: LIP can be NULL (wildcard), but in this implementaion, LPort */
/*       is never a wildcard. When not specified, LPort is assigned to */
/*       an ephemeral port. */
/*-------------------------------------------------------------------- */
int SockPcbBind( void *hSock, uint32_t LIP, uint32_t LPort )
{
    SOCK   *ps    = (SOCK *)hSock;
    SOCK   *psTmp;
    int    fEphem = 0;
    uint32_t SockOpts;

#ifdef _STRONG_CHECKING
    if( ps->fd.Type != HTYPE_SOCK )
    {
        DbgPrintf(DBG_ERROR,"SockPcbBind: HTYPE %04x",ps->fd.Type);
        return( 0 );
    }

    if( ps->SockProt > 4 )
    {
        DbgPrintf(DBG_ERROR,"SockPcbBind: Invalid SockProt");
        return( 0 );
    }
#endif

    /* Anything goes with Raw sockets */
    if( ps->SockProt == SOCKPROT_RAW )
    {
        SockOpts = 0;
        goto SockPcbBindOk;
    }

    /* On multicast addrs, REUSEADDR is the same as REUSEPORT */
    SockOpts = ps->OptionFlags & (SO_REUSEPORT|SO_REUSEADDR);
    if( SockOpts == SO_REUSEADDR )
    {
        if( IN_MULTICAST( LIP ) )
            SockOpts = (SO_REUSEADDR|SO_REUSEPORT);
    }

    /* If LPort is NULL, and we are already bound to an LPort, */
    /* then we'll use the previously bound value. */
    if( !LPort && ps->LPort )
        LPort = ps->LPort;

    /* OPTION: We *may* want to restrict IP addresses to those actually */
    /* available on our system. */
    /* OPTION: We *may* want to restrict reserved ports to some "system" */
    /* class user. */

    /* Check for ephemeral port */
SockPcbBindEphem:
    if( !LPort )
    {
        if( ++wUserPort > SOCK_USERPORT_LAST )
              wUserPort = SOCK_USERPORT_FIRST;
        LPort = HNC16(wUserPort);
        fEphem = 1;
    }

    /* Find the best match */
    psTmp = SockPcbFind( pSockList[ps->SockProt], LIP, LPort, 0, 0, 0, 0 );

    /* Check for conflict (and that we didn't match ourselves) */
    if( psTmp && psTmp != ps && LPort == psTmp->LPort )
    {
        /* If the IP's are different, we're OK */
        if( LIP && psTmp->LIP && LIP != psTmp->LIP )
            goto SockPcbBindOk;

        /* We don't allow conflicts on ephemeral ports */
        if( fEphem )
        {
            LPort = 0;
            goto SockPcbBindEphem;
        }

        /* We have a port conflict. See if we keep going anyway */

        /* Address Conflict */

        /* Exception 1: Check for "REUSEPORT" on both sockets */
        if( psTmp->OptionFlags & SockOpts & SO_REUSEPORT )
            goto SockPcbBindOk;

        /* Exception 2: One or the other Local IP is "wildcard", but not both, */
        /*              and both have REUSEADDR set. */
        if( ((!LIP && psTmp->LIP) || (LIP && !psTmp->LIP)) &&
                      (psTmp->OptionFlags & SockOpts & SO_REUSEADDR) )
            goto SockPcbBindOk;

        return( NDK_EADDRINUSE );
    }

SockPcbBindOk:
    /* If we got here, the addr is OK */
    ps->LIP   = LIP;
    ps->LPort = LPort;
    ps->OptionFlags |= SockOpts;

    /*
     * If an ephemeral port was chosen, ensure that it is bound to the socket
     * when SockDisconnect() is called.
     */
    if( fEphem )
    {
        ps->BPort = LPort;
    }

    return(0);
}

/*-------------------------------------------------------------------- */
/* SockPcbConnect() */
/* Set the foreign address to that specified in FIP and FPort */
/*-------------------------------------------------------------------- */
int SockPcbConnect( void *hSock, uint32_t FIP, uint32_t FPort )
{
    SOCK   *ps    = (SOCK *)hSock;
    SOCK   *psTmp;
    int    wc;

#ifdef _STRONG_CHECKING
    if( ps->fd.Type != HTYPE_SOCK )
    {
        DbgPrintf(DBG_ERROR,"SockPcbConnect: HTYPE %04x",ps->fd.Type);
        return( 0 );
    }

    if( ps->SockProt > 4 )
    {
        DbgPrintf(DBG_ERROR,"SockPcbConnect: Invalid SockProt");
        return( 0 );
    }
#endif

    /* Anything goes with Raw sockets */
    if( ps->SockProt == SOCKPROT_RAW )
        goto SockPcbConnectOk;

    /* Verify that we have a local binding */
    /* Note that LIP can be NULL if one really odd case... */
    if( !ps->LIP || !ps->LPort )
    {
        /* We can bind to [0.0.0.0] if we've specified an IFTx device */
        if( !ps->LPort || !ps->hIFTx )
            return( NDK_EINVAL );
    }

    /* Verify we have a desired IP, and port */
    if( !FIP || !FPort )
        return( NDK_EINVAL );

    /* Find the best match */
    psTmp = SockPcbFind( pSockList[ps->SockProt],
                         ps->LIP, ps->LPort, FIP, FPort, &wc, 0 );

    /* Check for conflict */
    if( psTmp && psTmp != ps && wc == 0 )
        return( NDK_EADDRINUSE );

SockPcbConnectOk:
    /* If we got here, the addr is OK */
    ps->FIP   = FIP;
    ps->FPort = FPort;

    return(0);
}

/*-------------------------------------------------------------------- */
/* SockPcbResolve() */
/* Find the "best" match given the supplied parameters, and */
/* potentially spawn a new socket from an "accepting" (listening) */
/* socket. */
/*-------------------------------------------------------------------- */
void *SockPcbResolve( uint32_t SockProt, uint32_t LIP, uint32_t LPort, uint32_t FIP,
                       uint32_t FPort, uint32_t Match, uint32_t * MaxFlag )
{
    SOCK *ps;
    SOCK *psSpawn;
    int  wc;
    int error;

    /* Assume max connections are not exceeded */
    *MaxFlag = 0;

    /* The first thing to do is the find the best match */
    ps = SockPcbFind( pSockList[SockProt],
                      LIP, LPort, FIP, FPort, &wc, 0 );

    /* Return NULL for no match, or for wildcard match when EXACT required */
    if( !ps || (wc && Match == SOCK_RESOLVE_EXACT) )
        return(0);

    /* If match type is not SPAWN, then we return what we got */
    if( Match != SOCK_RESOLVE_SPAWN )
        return( ps );

    /* We have a match type of SPAWN with wildcards. Here, we only return */
    /* a socket if we can spawn it. i.e.: exact match is a failure */
    /* Also, it must be accepting connections */
    if( !wc || !(ps->OptionFlags & SO_ACCEPTCONN) )
        return(0);

    /* If we're out of connect space, abort */
    if( ps->ConnTotal >= ps->ConnMax ) {
        *MaxFlag = 1;
        return(0);
    }

    /* Create a new socket - return NULL if we can't create it */
    error = SockNew(ps->Family, ps->SockType, ps->Protocol,
                ps->RxBufSize, ps->TxBufSize, (void **)&psSpawn);

    /*
     * If the socket could not be created because of an error set an error on
     * the parent socket and notify it. MaxFlag is overidden with 1 to inform
     * the caller of SockPcbResolve that something went wrong with SockNew
     */
    if(error) {
        SockSet((void *)ps, SOL_SOCKET, SO_ERROR, &error, sizeof(error));
        SockNotify((void *)ps, SOCK_NOTIFY_ERROR );
        *MaxFlag = 1;
        return(0);
    }

    /* Inherit SOCKET Options */
    psSpawn->OptionFlags |= ps->OptionFlags & ~(SO_ACCEPTCONN);
    psSpawn->dwLingerTime = ps->dwLingerTime;
    psSpawn->RxTimeout    = ps->RxTimeout;
    psSpawn->TxTimeout    = ps->TxTimeout;

    /* Inherit IP Options */
    psSpawn->IpFlags   = ps->IpFlags;
    psSpawn->IpTtl     = ps->IpTtl;
    psSpawn->IpTos     = ps->IpTos;
    if( ps->IpOptSize )
    {
        psSpawn->IpOptSize = ps->IpOptSize;
        mmCopy( psSpawn->IpOptions, ps->IpOptions, ps->IpOptSize );
    }

    /* Inherit Some Protocol Options */
    SockPrInherit( ps, psSpawn );

    /* We're going to cheat here. We know the specified parameters do */
    /* not conflict (as there was no exact match). */
    psSpawn->LIP    = LIP;
    psSpawn->LPort  = LPort;
    psSpawn->FIP    = FIP;
    psSpawn->FPort  = FPort;

    /* Enqueue the spawned packet onto the parent's Pending queue */
    psSpawn->pParent     = ps;
    psSpawn->pPrevQ      = ps;
    psSpawn->pPending    = ps->pPending;
    if( psSpawn->pPending )
        psSpawn->pPending->pPrevQ = psSpawn;
    ps->pPending         = psSpawn;
    psSpawn->StateFlags |= SS_PENDINGQ;
    ps->ConnTotal++;

    return( psSpawn );
}

/*-------------------------------------------------------------------- */
/* SockPcbResolveChain() */
/* This is a chain version of PCB resolve, finding all sockets which */
/* match the selection criteria. */
/*-------------------------------------------------------------------- */
void *SockPcbResolveChain( void *hSock, uint32_t SockProt, uint32_t Prot,
                            uint32_t LIP, uint32_t LPort, uint32_t FIP, uint32_t FPort )
{
    SOCK *ps = (SOCK *)hSock;

    /* We chain off the supplied SOCK, or start at the head of the list */
    /* on a NULL */
    if( !ps )
        ps = pSockList[SockProt];
    else
        ps = ps->pProtNext;

    /* Find a match */
    if( SockProt != SOCKPROT_RAW )
        ps = SockPcbFind( ps, LIP, LPort, FIP, FPort, 0, 1 );
    else
        ps = SockPcbFindRaw( ps, Prot, LIP, FIP, 0, 1 );

    return(ps);
}

/*-------------------------------------------------------------------- */
/* SockPcbCleanup() */
/* Abort all closing sockets */
/*-------------------------------------------------------------------- */
void SockPcbCleanup()
{
    SOCK *ps;
    int  GotOne;

    /* The only type of socket that can be "closing" is TCP */
    /* We'll search and we'll search until no Closing socket lives */
    do
    {
        GotOne = 0;
        ps = pSockList[SOCKPROT_TCP];
        while( ps )
        {
            if( ps->StateFlags & SS_CLOSING )
            {
                GotOne = 1;
                SockIntAbort( ps );
                break;
            }
            ps = ps->pProtNext;
        }
    } while( GotOne );
}

/*-------------------------------------------------------------------- */
/* SockCtlError() */
/* Called to notify the socket layer that an error has occurred */
/*-------------------------------------------------------------------- */
void SockPcbCtlError( uint32_t Code, IPHDR *pIpHdr )
{
    uint32_t LIP,FIP;
    uint32_t offset,SockProt;
    int    wc;
    UDPHDR *pUdpHdr;
    SOCK   *ps;

    FIP = RdNet32( &pIpHdr->IPDst );
    LIP = RdNet32( &pIpHdr->IPSrc );

    /* Get the offset to the transport hdr (assume UDP) */
    offset = (pIpHdr->VerLen & 0xF) * 4;
    pUdpHdr = (UDPHDR *)(((unsigned char *)pIpHdr)+offset);

    /* Get the correct SockProt to notify */
    if( pIpHdr->Protocol == 6 )
        SockProt = SOCKPROT_TCP;
    else if( pIpHdr->Protocol == 17 )
        SockProt = SOCKPROT_UDP;
    else
        return;

    /* Notify all exact matches */
    ps = pSockList[SockProt];

    while( ps )
    {
        /* Find a match */
        ps = SockPcbFind( ps, LIP, pUdpHdr->SrcPort,
                                     FIP, pUdpHdr->DstPort, &wc, 1 );
        /* Notify the protocol */
        if( ps && !wc )
        {
            /* For redirect errors, we just update the route. */
            if( Code >= PRC_REDIRECT_NET && Code <= PRC_REDIRECT_TOSHOST )
                SockValidateRoute( ps );
            else
                SockPrCtlError( ps, Code );
        }

        /* Get the next matching socket */
        if( ps )
            ps = ps->pProtNext;
    }
}

/*-------------------------------------------------------------------- */
/* SockPcbRtChange() */
/* Called to notify the socket layer that a route has changed */
/*-------------------------------------------------------------------- */
void SockPcbRtChange( void *hRt )
{
    SOCK   *ps;
    int    i;

    for( i=SOCKPROT_TCP; i<=SOCKPROT_RAW; i++ )
    {
        ps = pSockList[i];

        while(ps)
        {
            if( ps->hRoute == hRt )
            {
                ps->hRoute = 0;
                RtDeRef( hRt );
            }
            SockValidateRoute( ps );
            ps = ps->pProtNext;
        }
    }
}

/*-------------------------------------------------------------------- */
/* SockGetPcb() */
/* Return socket PCB list */
/*-------------------------------------------------------------------- */
int SockGetPcb( uint32_t SockProt, uint32_t BufSize, unsigned char *pBuf )
{
    SOCK    *ps;
    SOCKPCB *ppcb;
    int     i;

    if( SockProt<SOCKPROT_TCP || SockProt>SOCKPROT_RAW )
        return(0);

    ps = pSockList[ SockProt ];

    i=0;
    while( ps && BufSize >= sizeof( SOCKPCB ))
    {
        ppcb = (SOCKPCB *)pBuf;

        ppcb->IPAddrLocal   = ps->LIP;
        ppcb->PortLocal     = ps->LPort;
        ppcb->IPAddrForeign = ps->FIP;
        ppcb->PortForeign   = ps->FPort;
        if( ps->hTP )
            ppcb->State = SockPrGetState( ps, ps->hTP );
        else
            ppcb->State = 0;
        if( !ppcb->State && (ps->OptionFlags & SO_ACCEPTCONN) )
            ppcb->State = 1;

        i++;
        BufSize -= sizeof(SOCKPCB);
        pBuf    += sizeof(SOCKPCB);

        ps = ps->pProtNext;
    }

    return(i);
}

/**
 *  @b Description
 *  @n
 *      The goal of this function is to allow a clean
 *      reboot/shutdown or IP address change, without leaving any dangling
 *      sockets. Additionally, control must be given back to an application
 *      thread that's using an affected socket, so that it can handle such a
 *      change appropriately. NOTE: This function should never block!
 *
 *      The function cycles through all the socket entries for the
 *      specified protocol family and attempts to close sockets which match the
 *      IP Address specified. The function was added to ensure that when
 *      an IP address is modified OR when the stack is shut down or rebooted,
 *      then any sockets matching the IP Address provided are closed.
 *
 *      There are 3 special cases - linger sockets, lingering sockets and
 *      listen queue sockets.
 *
 *      1. Linger sockets are sockets that will linger for a time period prior
 *      to closing down. Since a close on a linger socket that's initiated from
 *      this function would result in lingering, the linger option is disabled
 *      for such sockets prior to closing.
 *
 *      2. Lingering sockets are currently in the process of being shut down by
 *      their owning thread. No lingering sockets should be killed here, as
 *      this results in the owning thread having the 'rug pulled out' from under
 *      it and will crash as a result. This function forces lingering sockets
 *      to stop lingering and allows the owning thread to finish the job it
 *      already started.
 *
 *      3. Listen Q sockets are the queued up incoming connections that are
 *      chained to a socket that is currently accepting connections (i.e.
 *      blocked on a call to accept()). Listen Q sockets must be closed in a
 *      special way, otherwise a crash may result (similar to the reasons given
 *      for lingering sockets).
 *      In all cases, the socket is set to have an error and signaled so that
 *      the owning thread can handle this change (e.g. so a call blocked on
 *      recv() will return with error, or the next socket operation attempted
 *      will fail).
 *
 *      This is for *internal* NDK Stack Usage.
 *
 *  @note You must call llEnter() prior to calling this function, and
 *      llExit() upon returning from this function.
 *
 *  @param[in]  SockProt
 *      Socket Family
 *  @param[in]  IPAddress
 *      OLD IP Address to which a socket might be bound to and which needs
 *      to be cleaned.
 *
 *  @retval
 *      Not Applicable
 */
void SockCleanPcb(uint32_t SockProt, uint32_t IPAddress)
{
    SOCK *ps;
    SOCK *psNext;
    int   error;

    /* Validate the socket protocol family. */
    if ((SockProt < SOCKPROT_TCP) || (SockProt > SOCKPROT_RAW)) {
        return;
    }

    /* Cycle through all the entries in the socket table. */
    ps = pSockList[SockProt];
    while (ps != NULL)
    {
        /* Save next entry before SockClose() sets it to 0 (fixes mem leak) */
        psNext = ps->pProtNext;

        /* Clean the entry only if we have a match */
        if (ps->LIP == IPAddress) {

            /*
             * Invalidate the socket. This causes any further socket API
             * calls on it to fail
             */
            fdint_setinvalid((FILEDESC *)ps);

            /* Set an error on the socket */
            error = NDK_ENETDOWN;
            SockSet((void *)ps, SOL_SOCKET, SO_ERROR, &error, sizeof(error));

            /*
             *  Signal the socket, allowing owning thread to handle the error
             *  and return
             */
            FdSignalEvent(ps, FD_EVENT_READ | FD_EVENT_WRITE | FD_EVENT_EXCEPT);

            /* Don't close lingering sockets here */
            if (ps->StateFlags & SS_LINGERING) {
                /*
                 * Unset the SS_LINGERING flag
                 *
                 * Lingering sockets are currently in the process of being
                 * closed by their owning thread, and are blocked awaiting a
                 * linger timeout. However, if the IP address is being removed,
                 * or the stack is shutting down, lingering no longer makes
                 * sense. Therefore, we unset the SS_LINGERING flag, which will
                 * (in combination with signaling the socket) allow the owning
                 * thread to break out of the while loop in SockClose() and
                 * finish closing the socket.
                 */
                ps->StateFlags ^= SS_LINGERING;

                /* Move on to the next socket */
                ps = psNext;
                continue;
            }

            /*
             * Kill the socket
             *
             * Listen Q sockets are special case; for all others, pass
             * to SockClose() directly
             */
            if (ps->StateFlags & (SS_READYQ | SS_PENDINGQ)) {
                /*
                 * If this socket is in a listening socket's ready or pend Q,
                 * then it must be removed from that parent socket's Q.
                 * Otherwise, a double free of the socket is possible as well
                 * as crash due to NULL pointer dereference of the hTP field.
                 */
                SockSpawnAbort(ps);
            }
            else {
                /*
                 * Unset the SO_LINGER option
                 *
                 * Do not linger when closing a socket from this function.
                 * For any linger socket, disable linger before closing.
                 * Lingering from this function results in race condition
                 * crashes due to the exit from kernel mode before sleeping for
                 * linger time.
                 */
                if (ps->OptionFlags & SO_LINGER) {
                    ps->OptionFlags ^= SO_LINGER;
                }

                SockClose(ps);
            }
        }

        /* Get the next entry. */
        ps = psNext;
    }
    return;
}
