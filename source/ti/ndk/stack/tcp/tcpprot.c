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
 * ======== tcpprot.c ========
 *
 * TCP Protocol Functions + Misc Functions
 *
 */

#include <stkmain.h>
#include "tcp.h"

/*-------------------------------------------------------------------- */
/* TcpPrAttach() */
/* Creates a new TCP side "socket" */
/*-------------------------------------------------------------------- */
int TcpPrAttach( void *h, void **phTcp )
{
    TCPPROT *pt;

    /* Allocate the TCP Protocol Control Block */
    if( !(pt = mmAlloc(sizeof(TCPPROT))) )
    {
        DbgPrintf(DBG_WARN,"TcpPrAttach: OOM");
        NotifyLowResource();

        /* Return the error */
        return( NDK_ENOMEM );
    }

    /* Default init to zero */
    mmZeroInit( pt, sizeof(TCPPROT) );

    /* Initialize the TCP Protocol Control Block */
    pt->hSock    = h;
    pt->t_state  = TSTATE_CLOSED;
#ifdef NDK_DEBUG_TCP_STATES
    DbgPrintf(DBG_INFO, "TcpPrAttach: initialize TCP state: TSTATE_CLOSED "
        "(skt: 0x%x, tcp: 0x%x, task: 0x%x)", h, pt, TaskSelf());
#endif
    pt->t_mss    = TCP_MSS_DEFAULT;
    pt->t_flags  = TCP_TFLAGS_DEFAULT;

    pt->t_srtt   = TCPTV_SRTTBASE << TCP_FIXP_SHIFT;
    pt->t_rttvar = TCPTV_RTTDFLT << TCP_FIXP_SHIFT;
    pt->t_trtx   = pt->t_rttvar >> (TCP_FIXP_SHIFT-1);
    pt->t_maxrtt = TCPTV_MAXRTTDFLT;

    pt->snd_cwnd = TCP_MAXWIN;

    /* Get local copies of the SOCK buffers */
    pt->hSBRx = SockGetRx( h );
    pt->hSBTx = SockGetTx( h );

    /* Initialize our handle in the SOCK object */
    *phTcp = (void *)pt;

    /* Now we attach the Socket to our handler list */
    SockPcbAttach( h );
    TcpTimeoutAdd( pt );

    return(0);
}

/*-------------------------------------------------------------------- */
/* TcpPrDetach() */
/* Closes a TCP side "socket", detaches, and free resources */
/*-------------------------------------------------------------------- */
int TcpPrDetach( void *hSock, void **phTcp, int fatal )
{
    TCPPROT *pt = *(TCPPROT **)phTcp;

    /* Mark this so close doesn't notify the socket layer */
    pt->t_flags |= TF_DETACHED;

    /* We CLOSE an unconnected socket, but if something is in progress, */
    /* use DROP. We are detaching, so can't do anything gracefully. */
    if( !fatal && pt->t_state >= TSTATE_SYNRCVD && pt->t_state != TSTATE_TIMEWAIT )
        TcpDrop( pt, 0 );       /* TcpDrop() calls TcpClose() */
    else
        TcpClose( pt );         /* Must call - frees some resources */

    /* Detach the socket and free the TCP control */
    TcpTimeoutRemove( pt );
    SockPcbDetach( hSock );

    /* Zap the TCP Proto Structure */
    *phTcp = 0;

        /* RFC 2018 - SACK */
        if (pt->pSack != NULL) {
            mmFree(pt->pSack);
        }
    mmFree( pt );

    return(0);
}

/*-------------------------------------------------------------------- */
/* TcpPrListen() */
/* Called to mark TCP "socket" as a listening socket */
/*-------------------------------------------------------------------- */
int TcpPrListen( void *h, void *hTcp )
{
    TCPPROT  *pt = (TCPPROT *)hTcp;
    TCPREASM *pR;

    (void)h;

    /* This really kills the socket as far as TCP is concerned. */
    /* It simply becomes a "spawnable" mark in the PCB table */

    pt->t_state = TSTATE_LISTEN;

    /* Also free held resources */
    while( (pR = pt->pReasm) )
    {
        pt->pReasm = pR->pNext;
        PBM_free( pR->pPkt );
    }
    pt->reasm_pkt_cnt = 0;

    /* Also zap the local copy of the buffer handles */
    pt->hSBRx = pt->hSBTx = 0;

    return(0);
}

/*-------------------------------------------------------------------- */
/* TcpPrConnect() */
/* Request a connection for a TCP side socket */
/*-------------------------------------------------------------------- */
int TcpPrConnect( void *h, void *hTcp )
{
    TCPPROT *pt = (TCPPROT *)hTcp;
    int     error = 0;

    (void)h;

    /* Bump the stats */
    NDK_tcps.ConnAttempt++;

    /* Set inititial send sequence number */
    pt->snd_una = pt->snd_nxt = pt->snd_max = pt->snd_up = pt->iss = tcp_iss;
    tcp_iss += TCP_ISSINCR / 2;

    /* Send the SYN ("KeepAlive" timer is also our connection timeout timer) */
    pt->t_state   = TSTATE_SYNSENT;
#ifdef NDK_DEBUG_TCP_STATES
    DbgPrintf(DBG_INFO, "TcpPrConnect: set TCP state: TSTATE_SYNSENT "
        "(skt: 0x%x, tcp: 0x%x, task: 0x%x)", h, pt, TaskSelf());
#endif
    pt->TicksKeep = TCPTV_KEEP_INIT;

    error = TcpOutput( hTcp );

    return( error );
}

/*-------------------------------------------------------------------- */
/* TcpPrDisconnect() */
/* Disconnects a TCP side "socket" - performs "half close" */
/*-------------------------------------------------------------------- */
int TcpPrDisconnect( void *hSock, void *hTcp )
{
    TCPPROT *pt = (TCPPROT *)hTcp;

    /* If less than connected, just close */
    if( pt->t_state < TSTATE_ESTAB )
    {
#ifdef NDK_DEBUG_TCP
        DbgPrintf(DBG_INFO, "TcpPrDisconnect: closing socket simply "
            "(skt: 0x%x, tcp: 0x%x, task: 0x%x)", hSock, pt, TaskSelf());
#endif
        TcpClose( pt );
    }
    /* Else if connected, move to FINWAIT1 */
    else if( pt->t_state == TSTATE_ESTAB )
    {
        pt->t_state = TSTATE_FINWAIT1;
#ifdef NDK_DEBUG_TCP_STATES
        DbgPrintf(DBG_INFO, "TcpPrDisconnect: set TCP state: TSTATE_FINWAIT1 "
            "(skt: 0x%x, tcp: 0x%x, task: 0x%x)", hSock, pt, TaskSelf());
#endif
        TcpOutput( pt );
    }
    /* Else we can only be at CLOSEWAIT */
    /* (unless called more than once in which case we don't care) */
    else if( pt->t_state == TSTATE_CLOSEWAIT )
    {
        pt->t_state = TSTATE_LASTACK;
#ifdef NDK_DEBUG_TCP_STATES
        DbgPrintf(DBG_INFO, "TcpPrDisconnect: set TCP state: TSTATE_LASTACK "
            "(skt: 0x%x, tcp: 0x%x, task: 0x%x)", hSock, pt, TaskSelf());
#endif
        TcpOutput( pt );
    }

    /* Notify the Socket if we received a FIN already */
    /* (should only happen if we were in CLOSEWAIT) */
    if( pt->t_state > TSTATE_FINWAIT1 && pt->t_state != TSTATE_FINWAIT2 )
        SockNotify( hSock, SOCK_NOTIFY_DISCONNECT );

    return(0);
}

/*-------------------------------------------------------------------- */
/* TcpPrRecv() */
/* Deal with Rx data being removed from buffer */
/*-------------------------------------------------------------------- */
int TcpPrRecv( void *h, void *hTcp )
{
    (void)h;

    /* A read may cause us to readjust our window */
    return( TcpOutput( hTcp ) );
}

/*-------------------------------------------------------------------- */
/* TcpPrSend() */
/* Copy data to send out of the application buffer and into the socket */
/* TX buffer, then let TCP protocol handle the actual transmission. */
/*-------------------------------------------------------------------- */
int TcpPrSend( void *h, void *hTcp, unsigned char *pBuf, int32_t Size, int32_t *pRetSize )
{
    TCPPROT  *pt = (TCPPROT *)hTcp;
    int32_t  Space;

    (void)h;

    /* Append as much data as possible to the output buffer */
    Space = SBGetSpace( pt->hSBTx );
    if( Space < Size )
        Size = Space;

    /* Copy out the data */
    if( Size )
        Size = SBWrite( pt->hSBTx, Size, pBuf, 0 );
    *pRetSize = Size;

    /* Data write may cause us to send data */
    if( Size )
        return( TcpOutput( hTcp ) );
    else
        return( 0 );
}

/*-------------------------------------------------------------------- */
/* TcpPrSendOOB() */
/* Send data */
/*-------------------------------------------------------------------- */
int TcpPrSendOOB( void *h, void *hTcp, unsigned char *pBuf,
                  int32_t Size, int32_t *pRetSize )
{
    TCPPROT  *pt = (TCPPROT *)hTcp;
    int32_t  Space;
    int32_t  UseSize;

    (void)h;

    /* Append as much data as possible to the output buffer */
    Space = SBGetSpace( pt->hSBTx );
    if( Space < Size )
        UseSize = Space;
    else
        UseSize = Size;

    /* Copy out the data */
    if( UseSize )
        UseSize = SBWrite( pt->hSBTx, UseSize, pBuf, 0 );
    *pRetSize = UseSize;

    /* The last byte of the buffer is the urgent data. */
    /* Update urgent pointer if we've appended all the data */
    if( UseSize == Size )
        pt->snd_up = pt->snd_una + SBGetTotal(pt->hSBTx);

    /* Data write may cause us to send data */
    if( UseSize )
        return( TcpOutput( hTcp ) );
    else
        return( 0 );
}

/*-------------------------------------------------------------------- */
/* TcpPrInherit() */
/* Inherit properties from the parent */
/*-------------------------------------------------------------------- */
void TcpPrInherit( void *hParent, void *hTcpParent,
                   void *hChild, void *hTcpChild )
{
    TCPPROT  *ptP = (TCPPROT *)hTcpParent;
    TCPPROT  *ptC = (TCPPROT *)hTcpChild;

    (void)hParent;
    (void)hChild;

    /* Inherit the max RTT setting from the parent. */
    ptC->t_maxrtt = ptP->t_maxrtt;

    /* RFC 2018 - SACK */
    ptC->t_flags |= ptP->t_flags &
                    (TF_NOPUSH|TF_NOOPT|TF_NODELAY|TF_SACKPERMITTED);

    if (ptC->t_flags & TF_SACKPERMITTED) {
        ptC->pSack = mmAlloc(sizeof(struct SACK_Info));
        /* Can not allocate memory, then disable the SACK */
        if (ptC->pSack == NULL) {
            ptC->t_flags &= ~TF_SACKPERMITTED;
        }
        else {
            ptC->pSack->txTableBottom = 0;
            ptC->pSack->txTableTop = 0;
            ptC->pSack->rcvTableBottom = 0;
            ptC->pSack->rcvTableTop = 0;
            ptC->pSack->rexmtTimeout = 0;
            ptC->TicksSackRexmt = 0;
        }
    }
}


/*-------------------------------------------------------------------- */
/* TcpPrGetState() */
/* Return TCP State */
/*-------------------------------------------------------------------- */
uint32_t TcpPrGetState( void *hParent, void *hTcp )
{
    (void)hParent;

    return( ((TCPPROT *)hTcp)->t_state );
}

/*-------------------------------------------------------------------- */
/* TcpPrSetOption() */
/* Set TCP Options */
/*-------------------------------------------------------------------- */
int TcpPrSetOption(void *h, void *hTcp, int Prop, void *pBuf, int size)
{
    TCPPROT  *pt = (TCPPROT *)hTcp;
    int      value;

    (void)h;

    if( size != sizeof(int) || !pBuf )
        return( NDK_EINVAL );

    value = *(int *)pBuf;

    switch( Prop )
    {
    case NDK_TCP_NODELAY:
        if( value )
            pt->t_flags |= TF_NODELAY;
        else
            pt->t_flags &= ~TF_NODELAY;
        return( 0 );

    case NDK_TCP_NOPUSH:
        if( value )
            pt->t_flags |= TF_NOPUSH;
        else
            pt->t_flags &= ~TF_NOPUSH;
        return( 0 );

    case NDK_TCP_NOOPT:
        if( value )
            pt->t_flags |= TF_NOOPT;
        else
            pt->t_flags &= ~TF_NOOPT;
        return( 0 );

    case NDK_TCP_MAXSEG:
        if( value < 0 )
        {
            return (NDK_EDOM);
        }

        if( (uint32_t)value < pt->t_mss )
        {
            TcpValidateMetrics( pt, (uint32_t)value );
        }
        return( 0 );

    case NDK_TCP_SACKPERMITTED:
        /*
         * RCF 2018 - SACK
         * Check if SACK is already set
         * SACK can only be set if socket state is TSTATE_CLOSED
         * (i.e. prior to calling listen() for server sockets, and prior to
         * calling connect() for client sockets)
         */
        if (pt->t_state != TSTATE_CLOSED) {
            return (NDK_EINVAL);
        }

        if (value) {
            if(pt->pSack != NULL) {
                return (NDK_EALREADY);
            }
            pt->pSack = mmAlloc(sizeof(struct SACK_Info));
            if (pt->pSack == NULL) {
                return (NDK_ENOMEM);
            }
            pt->pSack->txTableBottom = 0;
            pt->pSack->txTableTop = 0;
            pt->pSack->rcvTableBottom = 0;
            pt->pSack->rcvTableTop = 0;
            pt->t_flags |= TF_SACKPERMITTED;
            pt->TicksSackRexmt = 0;
        }
        else {
            pt->t_flags &= ~TF_SACKPERMITTED;
            if (pt->pSack != NULL) {
                mmFree(pt->pSack);
                pt->pSack = NULL;
                pt->sackActive = 0;
                pt->TicksSackRexmt = 0;
            }
        }
        return( 0 );

    case NDK_TCP_MAXRTT:
        if( value < 0)
        {
            return (NDK_EDOM);
        }

        /* convert to clock ticks and make sure a realistic value is used. */
        pt->t_maxrtt = ((uint32_t)value+99)/100;
        if( pt->t_maxrtt < TCPTV_MINIMALMAXRTT )
        {
            pt->t_maxrtt = TCPTV_MINIMALMAXRTT;
        }
        return( 0 );
    }
    return( NDK_EINVAL );
}

/*-------------------------------------------------------------------- */
/* TcpPrGetOptions() */
/* Get TCP Options */
/*-------------------------------------------------------------------- */
int TcpPrGetOption(void *h, void *hTcp, int Prop, void *pBuf, int *psize)
{
    TCPPROT  *pt = (TCPPROT *)hTcp;

    (void)h;

    if( *psize < (int)sizeof(int) )
        return( NDK_EINVAL );

    *psize = (int)sizeof(int);

    switch( Prop )
    {
    case NDK_TCP_NODELAY:
        if( pt->t_flags & TF_NODELAY )
            *(int *)pBuf = 1;
        else
            *(int *)pBuf = 0;
        return( 0 );

    case NDK_TCP_NOPUSH:
        if( pt->t_flags & TF_NOPUSH )
            *(int *)pBuf = 1;
        else
            *(int *)pBuf = 0;
        return( 0 );

    case NDK_TCP_NOOPT:
        if( pt->t_flags & TF_NOOPT )
            *(int *)pBuf = 1;
        else
            *(int *)pBuf = 0;
        return( 0 );

    case NDK_TCP_MAXSEG:
        *(int *)pBuf = (int)pt->t_mss;
        return( 0 );

    case NDK_TCP_SACKPERMITTED:
        /* RFC 2018 - SACK */
        if( pt->t_flags & TF_SACKPERMITTED )
            *(int *)pBuf = 1;
        else
            *(int *)pBuf = 0;
        return( 0 );

    case NDK_TCP_MAXRTT:
        /* convert to ms instead of clock ticks */
        *(int *)pBuf = (int)pt->t_maxrtt * 100;
        return( 0 );
    }
    return( NDK_EINVAL );
}

/*-------------------------------------------------------------------- */
/* TcpPrCtlError() */
/* Inform TCP of an error */
/*-------------------------------------------------------------------- */
void TcpPrCtlError( void *h, void *hTcp, uint32_t Code, int Error )
{
    TCPPROT  *pt = (TCPPROT *)hTcp;

    /* Quench - close congestion window */
    if( Code == PRC_QUENCH )
        pt->snd_cwnd = pt->t_mss;
    else if( Error )
    {
        /* Depending on state, we may ignore some errors */
        if( pt->t_state == TSTATE_ESTAB &&
                ( Error == NDK_EHOSTUNREACH || Error == NDK_EHOSTDOWN ) )
            return;

        if( pt->t_state<TSTATE_ESTAB && pt->t_rtxindex>3 && pt->t_softerror )
        {
#ifdef NDK_DEBUG_TCP
            DbgPrintf(DBG_INFO, "TcpPrCtlError: Error: set error for TCP state "
                "< TSTATE_ESTAB (skt: 0x%x, tcp: 0x%x, task: 0x%x)", h, pt,
                TaskSelf());
#endif
            SockSetError( h, Error );
        }
        else
            pt->t_softerror = Error;

        SockNotify( h, SOCK_NOTIFY_ERROR );
    }
}

/*-------------------------------------------------------------------- */
/* The following are protocol support functions */
/*-------------------------------------------------------------------- */

/*-------------------------------------------------------------------- */
/* TcpDrop() */
/* Drops a TCP side "socket" with ABORT */
/*-------------------------------------------------------------------- */
void TcpDrop( TCPPROT *pt, int Error )
{
    /* Drop the connection with "ABORTED" error */
    if( pt->t_state >= TSTATE_SYNRCVD )
    {
        pt->t_state = TSTATE_CLOSED;
#ifdef NDK_DEBUG_TCP_STATES
        DbgPrintf(DBG_INFO, "TcpDrop: set TCP state: TSTATE_CLOSED "
            "(skt: 0x%x, tcp: 0x%x, task: 0x%x)", pt->hSock, pt, TaskSelf());
#endif
        TcpOutput( pt );
        NDK_tcps.Drops++;
    }
    else
    {
#ifdef NDK_DEBUG_TCP
        DbgPrintf(DBG_INFO, "TcpDrop: connection drop (skt: 0x%x, tcp: 0x%x, "
            "task: 0x%x)", pt->hSock, pt, TaskSelf());
#endif
        NDK_tcps.ConnDrops++;
    }

    /* Change a TIMEOUT to the soft error if any, and set error code */
    if( Error == NDK_ETIMEDOUT && pt->t_softerror )
        Error = pt->t_softerror;
    SockSetError( pt->hSock, Error );

    TcpClose( pt );
}

/*-------------------------------------------------------------------- */
/* TcpClose() */
/* Closes down the TCP protocol block */
/*-------------------------------------------------------------------- */
void TcpClose( TCPPROT *pt )
{
    TCPREASM *pR;

    pt->t_state = TSTATE_CLOSED;
#ifdef NDK_DEBUG_TCP_STATES
    DbgPrintf(DBG_INFO, "TcpClose: set TCP state: TSTATE_CLOSED "
        "(skt: 0x%x, tcp: 0x%x, task: 0x%x)", pt->hSock, pt, TaskSelf());
#endif

    /* Disable all timers */
    pt->TicksRexmt   = 0;
    pt->TicksPersist = 0;
    pt->TicksKeep    = 0;
    pt->TicksWait2   = 0;

    /* Also free held resources */
    while( (pR = pt->pReasm) )
    {
        pt->pReasm = pR->pNext;
        PBM_free( pR->pPkt );
    }
    pt->reasm_pkt_cnt = 0;

        /* RFC 2018 - SACK */
        pt->sackActive = 0;

    /* Notify the Socket (if not already detached) */
    if( !(pt->t_flags & TF_DETACHED) )
        SockNotify( pt->hSock, SOCK_NOTIFY_CLOSED );
}

/*-------------------------------------------------------------------- */
/* TcpValidateMetrics */
/* Called to update TCP information based on socket route */
/* Returns current MSS to advertise */
/*-------------------------------------------------------------------- */
uint32_t TcpValidateMetrics( TCPPROT *pt, uint32_t rcvmss )
{
    void *hRoute;
    uint32_t ourmss;

    /* Validate the route plus any route related metrics (like t_mss) */
    if( !(hRoute = SockGetRoute( pt->hSock )) )
        hRoute = SockValidateRoute( pt->hSock );

    /* Get our (best case) mss */
    if( !hRoute )
        ourmss = TCP_MSS_DEFAULT_NR;
    else
        ourmss = RtGetMTU( hRoute );

    /* Take off IpHdr & TcpHdr */
    ourmss -= SockGetIpHdrSize(pt->hSock) + TCPHDR_SIZE;

    /* A previous write overrides us */
    if( (pt->t_flags & TF_RCVD_MSS) && pt->t_mss < ourmss )
        ourmss = pt->t_mss;

    /* If we have received an mss, record it */
    if( rcvmss )
    {
        /* Use the smallest of the valid mss values */
        pt->t_flags |= TF_RCVD_MSS;
        if( rcvmss < ourmss )
            pt->t_mss = rcvmss;
        else
            pt->t_mss = ourmss;
    }

    /* If we have a valid (negotiated) mss, return it. Else, */
    /* return what we'd like to advertise. */
    if( pt->t_flags & TF_RCVD_MSS )
        return( pt->t_mss );
    return( ourmss );
}

