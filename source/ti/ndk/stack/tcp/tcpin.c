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
 * ======== tcpin.c ========
 *
 * TCP In functions
 *
 */

#include <stkmain.h>
#include "tcp.h"
#include "../sock/sock.h"

/*-------------------------------------------------------------------- */
/* processAck() */
/* process ACK to remove entries form SACK Tx Table */
/*-------------------------------------------------------------------- */
static inline void processAck(TCPPROT* pt, TCPHDR *pTcpHdr)
{
    uint32_t  i;
    uint32_t  ack;
    SACK_TxEntry* pEntry;

    /* If any, remove ACKed segments from table */
    ack = RdNet32(&pTcpHdr->Ack);
    ack = HNC32(ack);
    if (SEQ_GT(ack, pt->snd_una))  {
        i = pt->pSack->txTableTop;
        while (i != pt->pSack->txTableBottom) {
            pEntry = &pt->pSack->SACK_txTable[i];
            if (SEQ_LEQ(pEntry->rightEdge, ack)) {
                pt->pSack->txTableTop++;
                pt->pSack->txTableTop &= (SACK_TABLE_SIZE - 1);
            }
            i++;
            i &= (SACK_TABLE_SIZE - 1);
        }
    }

    if (pt->pSack->txTableBottom == pt->pSack->txTableTop) {
        pt->TicksSackRexmt = 0;
        pt->pSack->rexmtTimeout = 0;
        pt->pSack->txTableBottom = 0;
        pt->pSack->txTableTop = 0;
    }
}

/*-------------------------------------------------------------------- */
/* processSackOptions() */
/* process SACK options */
/*-------------------------------------------------------------------- */
static inline void processSackOptions(TCPPROT* pt, TCPHDR *pTcpHdr,
                                      uint32_t optionOffset)
{
    uint32_t  i;
    uint32_t  j;
    uint32_t  optLen;
    uint16_t* pSack;
    uint32_t  blockLE;
    uint32_t  blockRE;
    uint32_t  startTimer;
    SACK_TxEntry* pEntry;

    optLen = pTcpHdr->Options[optionOffset + 1] - 2;
    pSack = (uint16_t*) &pTcpHdr->Options[optionOffset + 2];
    startTimer = 0;

    /*
     * For every SACK'd block, check txTable
     * to mark segments in the range of block
     */
    for (i = 0; i < optLen; i += SACK_OPTION_SIZE) {
        /* SACK Blocks are at 16-bit aligned addresses */
        blockLE = RdNet32(pSack);
        blockLE = HNC32(blockLE);
        pSack += 2;
        blockRE = RdNet32(pSack);
        blockRE = HNC32(blockRE);
        pSack += 2;
        j = pt->pSack->txTableTop;
        while (j != pt->pSack->txTableBottom) {
            pEntry = &pt->pSack->SACK_txTable[j];
            if (SEQ_GEQ(pEntry->leftEdge, blockLE) &&
                SEQ_LEQ(pEntry->rightEdge, blockRE)) {
                pEntry->sacked = 1;
                startTimer = 1;
            }
            j++;
            j &= (SACK_TABLE_SIZE - 1);
        }
    }

    if (startTimer && pt->TicksSackRexmt == 0) {
        TCPT_RANGESET(pt->TicksSackRexmt,
                      (pt->t_trtx / TCP_SACK_REXMIT_TIMER_RATIO),
                      1,
                      pt->t_trtx
                     );
        pt->pSack->rexmtTimeout = 0;
        pt->TicksRexmt += TCP_SACK_TICKS_REXMIT;
    }
}

/*-------------------------------------------------------------------- */
/* TcpEnterTimeWait() */
/* TCP Connection enters TimeWait state */
/*-------------------------------------------------------------------- */
static void TcpEnterTimeWait( TCPPROT *pt )
{
    pt->TicksRexmt   = 0;
    pt->TicksPersist = 0;
    pt->TicksKeep    = 0;
    pt->TicksWait2   = 2 * TCPTV_MSL;
    pt->t_state = TSTATE_TIMEWAIT;
#ifdef NDK_DEBUG_TCP_STATES
    DbgPrintf(DBG_INFO, "TcpEnterTimeWait: set TCP state: TSTATE_TIMEWAIT "
        "(skt: 0x%x, tcp: 0x%x, task: 0x%x)", pt->hSock, pt, TaskSelf());
#endif
    SBFlush( pt->hSBTx, 1 );
    SockNotify( pt->hSock, SOCK_NOTIFY_DISCONNECT );
}

/*-------------------------------------------------------------------- */
/* TcpInput() */
/* Rx TCP Packet */
/*-------------------------------------------------------------------- */
void TcpInput( PBM_Pkt *pPkt )
{
    TCPPROT    *pt;
    void       *hSock;
    uint32_t   w,IPHdrLen,TCPLen;
    uint32_t   IPSrc,IPDst;
    unsigned char    TcpFlags;
    IPHDR      *pIpHdr;
    TCPHDR     *pTcpHdr;
    TCPREASM   *pR;
    int        OptLen;
    uint32_t   ack,seq,tmpseq,eatseq=0,eatflag=0;
    int32_t    len,win,todrop;
    int        AbortSpawnSocket = 0;    /* Set to abort spawned socket */
    int        CanOutput = 0;   /* Set if we may be able the send more data */
    int        FinAcked  = 0;   /* Set if our FIN was ACK'd */
    NETIF_DEVICE *ptr_net_device;

    if (!pPkt) {
#ifdef _STRONG_CHECKING
        DbgPrintf(DBG_WARN,"TcpInput: received NULL packet");
#endif
        return;
    }

    /* Get the IP header len */
    IPHdrLen = pPkt->IpHdrLen;

    /* Assign an IP header pointer */
    pIpHdr = (IPHDR *)(pPkt->pDataBuffer + pPkt->DataOffset);

    /* Assign a TCP header pointer */
    pTcpHdr = (TCPHDR *)(pPkt->pDataBuffer + pPkt->DataOffset + IPHdrLen);

    /* Count the total number of packets in */
    NDK_tcps.RcvTotal++;

    /* Get the total length of the TCP message */
    TCPLen = (uint32_t)(HNC16( pIpHdr->TotalLen )) - IPHdrLen;

    /* Check for packet too small */
    if( TCPLen < TCPHDR_SIZE )
    {
        NDK_tcps.RcvShort++;
        PBM_free( pPkt );
        return;
    }

    /* Check for bad HdrLen */
    w = (pTcpHdr->HdrLen >> 4) << 2;
    OptLen = (int)w - TCPHDR_SIZE;   /* Set OptLen = TCP Option Length */
    len = (int32_t)(TCPLen - w);       /* Set len = TCP Payload Length */
    if( w < TCPHDR_SIZE || w > TCPLen )
    {
        NDK_tcps.RcvHdrSize++;
        PBM_free( pPkt );
        return;
    }

    /* Reset packet to point to payload data */
    pPkt->DataOffset += IPHdrLen + w;
    pPkt->ValidLen = len;

    /* We need some stuff for the pseudo header */
    /* Get source and destination */
    IPSrc = RdNet32(&pIpHdr->IPSrc);
    IPDst = RdNet32(&pIpHdr->IPDst);

    /* Save TCP Flags */
    TcpFlags = pTcpHdr->Flags;

    /*
     * Allow driver to control the means of checksum computation.
     *
     * If 'flags' does not have the appropriate h/w checksum offload
     * bits set, then the checksum is computed in software.
     *
     * If 'flags' does have the appropriate h/w checksum offload bits
     * set, then don't call the checksum fxn here, as the computation will be
     * done in the hardware. In this case, driver is responsible for enabling
     * h/w checksum computations.
     *
     * Note: CS verification of reassembled (fragmented) packets must always
     * be done in SW
     *
     * If the packet is destined to a loopback address then don't complete a s/w
     * checksum, as this packet never traveled over ethernet.
     */
    ptr_net_device = (NETIF_DEVICE *)pPkt->hIFRx;
    if ((!ptr_net_device ||
        !((ptr_net_device->flags & NIMU_DEVICE_ENABLE_HW_CHKSM_RX_ALL) ||
          (ptr_net_device->flags & NIMU_DEVICE_ENABLE_HW_CHKSM_RX_TCP)) ||
        (pPkt->csNumBytes > 0)) &&
        !(IPIsLoopback(IPDst) && IPIsLoopback(IPSrc)))
    {
        /*
         * Verify checksum in SW (HW not configured for checksum offload
         * or this is a reassembled packet)
         */

        /* Init pseudo header for checksum */
        tpseudo.IPSrc  = IPSrc;
        tpseudo.IPDst  = IPDst;
        tpseudo.Null     = 0;
        tpseudo.Protocol = 6;
        tpseudo.Length   = HNC16(TCPLen);

        w = (uint32_t)pTcpHdr->TCPChecksum;
        if( w == 0xFFFF )
            w = 0;
        TcpChecksum( pTcpHdr );
        if( w != (uint32_t)pTcpHdr->TCPChecksum )
        {
            NDK_tcps.RcvBadSum++;
            PBM_free( pPkt );
            return;
        }
    }

    /* Get readable seq and ack values */
    seq = RdNet32(&pTcpHdr->Seq);
    seq = HNC32(seq);
    ack = RdNet32(&pTcpHdr->Ack);
    ack = HNC32(ack);

    /* Convert 16 bit header values right in the header */
    pTcpHdr->WindowSize = HNC16(pTcpHdr->WindowSize);
    pTcpHdr->UrgPtr = HNC16(pTcpHdr->UrgPtr);

    tmpseq = tcp_iss;                 /* This may get reset in another loop */
findpcb:
    /* If this is a new SYN request, then we'll need to spawn a */
    /* socket to accomodate it (assuming we're listening). Otherwise, */
    /* we must have an exact match for this connection. */
    if( (TcpFlags & (TCP_SYN|TCP_ACK)) == TCP_SYN )
    {
        uint32_t MaxFlag;

        /* New Connection Attempt */
        hSock = SockPcbResolve(SOCKPROT_TCP, IPDst,
                               (uint32_t)pTcpHdr->DstPort,
                               IPSrc, (uint32_t)pTcpHdr->SrcPort,
                               SOCK_RESOLVE_SPAWN, &MaxFlag);

	/* If we have exceeded the max connections (listen backlog) for */
	/* this port, just drop the packet; i.e., do not send a RST */
    /* MaxFlag can also be set if there was an error in the SockNew() call */
    /* for spawned sockets. In either case of MaxFlag being set we want to */
    /* silently drop the packet */
	if ( !hSock && MaxFlag ) {
	    goto drop;
	}

        /* There's a chance that our first ACK was lost, so the sender may */
        /* send another SYN. If this is the case, the SPAWN will fail, but */
        /* the standard resolve will succeed */
        if( !hSock )
            goto normal_resolve;

        pt = (TCPPROT *)SockGetTP( hSock );
        if( !pt )
            goto dropwithresetfatal;

        /* Set the new socket state to LISTEN */
        pt->t_state = TSTATE_LISTEN;
#ifdef NDK_DEBUG_TCP_STATES
        DbgPrintf(DBG_INFO, "TcpInput: set TCP state: TSTATE_LISTEN "
            "(skt: 0x%x, tcp: 0x%x, task: 0x%x)", hSock, pt, TaskSelf());
#endif

        AbortSpawnSocket = 1;
    }
    else
    {
        uint32_t MaxFlag;
normal_resolve:

        /* Send to existing socket */
        hSock = SockPcbResolve(SOCKPROT_TCP, IPDst,
                               (uint32_t)pTcpHdr->DstPort, IPSrc,
                               (uint32_t)pTcpHdr->SrcPort,
                               SOCK_RESOLVE_EXACT, &MaxFlag);
        if( !hSock )
            goto dropwithreset;
        pt = (TCPPROT *)SockGetTP( hSock );
        if( !pt )
            goto dropwithresetfatal;

        /* RFC 2018 - SACK */
        if (pt->sackActive) {
            processAck(pt, pTcpHdr);
        }
    }

    /* We now have a valid TCPROT* in pt, and a valid SOCK in hSock. */

    /* If the state is closed, perform silent drop */
    if( pt->t_state == TSTATE_CLOSED ) {
#ifdef NDK_DEBUG_TCP
        DbgPrintf(DBG_INFO, "TcpInput: Error: state TSTATE_CLOSED, "
            "drop silently (skt: 0x%x, tcp: 0x%x, task: 0x%x)", hSock, pt,
            TaskSelf());
#endif
        goto drop;
    }

    /* Since segment received, reset IDLE */
    pt->t_tidle   = 0;

    /* Process Options - RFC 2018 SACK */
    if (OptLen > 0) {
        int    i;
        char olen;
        uint32_t mss;

        for (i = 0; i < OptLen; i+=olen ){
             switch (pTcpHdr->Options[i]) {
                 case TCPOPT_EOL:
                 case TCPOPT_NOP:
                     olen = 1;
                     break;

                 case TCPOPT_MAXSEG:
                     olen = (char) pTcpHdr->Options[i+1];
                     if (olen == TCPOLEN_MAXSEG ) {
                     /* Get the MSS and notify our metrics routine */
                         mss = pTcpHdr->Options[i+3]+pTcpHdr->Options[i+2]*256;
                         TcpValidateMetrics( pt, mss );
                     }
                     break;

                 case TCPOPT_SACKPERMITTED:
                     olen = TCPOPLEN_SACKPERMITTED;
                     if (((TcpFlags & TCP_SYN) ||
                         ((TcpFlags & TCP_SYN) && (TcpFlags & TCP_ACK))) &&
                         (pt->t_flags & TF_SACKPERMITTED)) {
                         pt->sackActive = 1;
                     }
                     break;

                 case TCPOPT_SACK:
                     olen = (char) pTcpHdr->Options[i+1];
                     if (pt->sackActive) {
                         processSackOptions(pt, pTcpHdr, i);
                     }
                     break;

                 default:
                     olen = (char) pTcpHdr->Options[i+1];
                     break;
            }
        }
    }

    /* Get our receive window (function should not return <0, but */
    /* check anyway). We also know our window can not shrink. */
    /* (We can not reduce the size of an allocated receive buffer). */
    win = SBGetSpace( pt->hSBRx );
    if( win < 0 )
        win = 0;
    else if( win > TCP_MAXWIN )
        win = TCP_MAXWIN;

    /*  Handle LISTEN and SYNSENT special */
    if( pt->t_state == TSTATE_LISTEN )
    {
        /* Socket is listening for a new connection */
        if( TcpFlags & TCP_RST )
            goto drop;
        if( TcpFlags & TCP_ACK )
            goto dropwithreset;
        if( !(TcpFlags & TCP_SYN) )
            goto drop;
        /* RFC1122 - Discard SYN pkts sent via BCAST */
        if( pPkt->Flags & (FLG_PKT_MACBCAST | FLG_PKT_MACMCAST) )
            goto drop;

        /* Initialize ISS (tmpseq=tcp_iss, or set later for a "re-connect") */
        pt->iss = tmpseq;
        tcp_iss += TCP_ISSINCR / 2;
        pt->snd_una = pt->snd_nxt = pt->snd_max = pt->snd_up = pt->iss;

        /* Initialize receive sequence (one past SYN) */
        pt->rcv_adv = pt->rcv_nxt = seq+1;

        /* Set the new state */
        pt->t_flags |= TF_ACKNOW;
        pt->t_state = TSTATE_SYNRCVD;
        pt->TicksKeep = TCPTV_KEEP_INIT;
#ifdef NDK_DEBUG_TCP_STATES
        DbgPrintf(DBG_INFO, "TcpInput: set TCP state: TSTATE_SYNRCVD (2) "
            "(skt: 0x%x, tcp: 0x%x, task: 0x%x)", hSock, pt, TaskSelf());
#endif

        /* Bump Stats */
        NDK_tcps.Accepts++;

        /* Try and get a route for this socket */
        TcpValidateMetrics( pt, 0 );

        /* Don't drop the (possibly) newly spawned socket */
        AbortSpawnSocket = 0;

        goto continueSYN;
    }
    else if( pt->t_state == TSTATE_SYNSENT )
    {
        /* Socket is connecting */

        /* Check for an illegal ACK */
        if( (TcpFlags & TCP_ACK) &&
                 (SEQ_LEQ(ack, pt->iss) || SEQ_GT(ack, pt->snd_max)) )
            goto dropwithreset;

        /* Check for refused connection */
        if( TcpFlags & TCP_RST )
        {
            /* If ACK set (we know its OK at this point), send */
            /* Connection Refused. Else we just drop the segment */
            if( TcpFlags & TCP_ACK )
                TcpDrop( pt, NDK_ECONNREFUSED );
            goto drop;
        }

        /* Check for no SYN */
        if( !(TcpFlags & TCP_SYN) )
            goto drop;

        /* Initialize receive sequence (one past SYN) */
        pt->rcv_adv = pt->rcv_nxt = seq+1;

        /* We ack the SYN no matter what... */
        pt->t_flags |= TF_ACKNOW;

        /* See if sender is ACK'ing. If not, enter the SYNRCVD state */
        if( !(TcpFlags & TCP_ACK) )
        {
            pt->t_state = TSTATE_SYNRCVD;
#ifdef NDK_DEBUG_TCP_STATES
            DbgPrintf(DBG_INFO, "TcpInput: set TCP state: TSTATE_SYNRCVD (1) "
                "(skt: 0x%x, tcp: 0x%x, task: 0x%x)", hSock, pt, TaskSelf());
#endif
        }
        else
        {
            /* We're connected */
            NDK_tcps.Connects++;

            /* Update ACK'd data */
            pt->snd_una = ack;

            /* Turn off retransmit timer */
            pt->TicksRexmt = 0;

            /* Reset KEEPALIVE to its normal "connected" state timer */
            pt->TicksKeep = TCPTV_KEEP_IDLE;

            /* Set state */
            pt->t_state = TSTATE_ESTAB;
#ifdef NDK_DEBUG_TCP_STATES
            DbgPrintf(DBG_INFO, "TcpInput: set TCP state: TSTATE_ESTAB (1) "
                "(skt: 0x%x, tcp: 0x%x, task: 0x%x)", hSock, pt, TaskSelf());
#endif

            /* Notify Socket */
            SockNotify( pt->hSock, SOCK_NOTIFY_CONNECT );
        }

continueSYN:
        /* Advance Seq to account for the SYN */
        seq++;

        /* Trim data for fit our window */
        if( len > win )
        {
            len = win;
            /* If we trim, we can't have a FIN (we trimmed it!) */
            TcpFlags &= ~TCP_FIN;
        }

        /* Init urgent data sequence number */
        pt->rcv_up  = seq;

        /* Force a window update */
        goto forcedUpdate;
    }

    /*  The state is other than LISTEN or SYNSENT */

    /* Since not a new connection, safe to reset KEEPALIVE */
    pt->TicksKeep = TCPTV_KEEP_IDLE;

    /* Drop duplicate data */
    todrop = pt->rcv_nxt - seq;
    if( todrop > 0 )
    {
        /* Adjust URG pointer first (affected by both SYN and data) */
        if( TcpFlags & TCP_URG )
        {
            if( pTcpHdr->UrgPtr > (uint16_t)todrop )
                pTcpHdr->UrgPtr -= (uint16_t)todrop;
            else
                TcpFlags &= ~TCP_URG;
        }

        /* If there's a SYN, its one less byte to drop */
        if( TcpFlags & TCP_SYN )
        {
            /* Skip SYN (we've already seen it) */
            TcpFlags &= ~TCP_SYN;

            /* Adjust seq to reflect elimination of SYN */
            seq++;

            /* Now we don't drop as much */
            todrop--;
        }

        /* See if entire packet is redundant */
        if( todrop > len ||
            (todrop == len && !(TcpFlags & TCP_FIN)) )
        {
            /* Duplicate packet */
            NDK_tcps.RcvDupPack++;
            NDK_tcps.RcvDupByte += (uint32_t)len;

            /* Set todrop to entire packet */
            todrop = len;

            /* Drop any DUP FIN */
            TcpFlags &= ~TCP_FIN;

            /* ACK now to resynch */
            pt->t_flags |= TF_ACKNOW;
        }
        else
        {
            /* Partial dup packet */
            NDK_tcps.RcvPartDupPack++;
            NDK_tcps.RcvPartDupByte += (uint32_t)todrop;
        }

        /* Now adjust the packet data payload */
        seq += todrop;
        len -= todrop;
        pPkt->DataOffset += (uint32_t)todrop;
    }

    /* Can't receive data on a closed segment */
    if( pt->t_state > TSTATE_ESTAB && len &&
             pt->t_state != TSTATE_FINWAIT1 && pt->t_state != TSTATE_FINWAIT2 )
    {
        TcpClose( pt );
        NDK_tcps.RcvAfterClose++;
        goto dropwithreset;
    }

    /* Trim data to our window size */
    todrop = (seq + (uint32_t)len) - (pt->rcv_nxt + win);
    if( todrop > 0 )
    {
        NDK_tcps.RcvAfterWinPack++;

        if( todrop < len )
            NDK_tcps.RcvAfterWinByte += (uint32_t)todrop;
        else
        {
            /* If this is a SYN request and we are in TIMEWAIT, */
            /* we can close now and restart */
            if( (TcpFlags & TCP_SYN) && pt->t_state == TSTATE_TIMEWAIT &&
                SEQ_GT( seq, pt->rcv_nxt ) )
            {
                tmpseq = pt->rcv_nxt + TCP_ISSINCR;
                TcpClose( pt );
                goto findpcb;
            }

            /* If our window is closed and the new data is */
            /* right at the edge, it is considered a probe. */
            /* We can't use the data, but we ACK it now. */
            if( !win && seq == pt->rcv_nxt )
            {
                pt->t_flags |= TF_ACKNOW;
                NDK_tcps.RcvWinProbe++;
            }
            else
            {
                /* The packet is outside our window, and not a probe. */
                /* We drop with ACK to try and se-sync. */
                goto dropafterack;
            }
        }

        /* Drop data past window */
        len -= todrop;

        /* Ignore PSH, and we must have trimmed off any FIN */
        TcpFlags &= ~(TCP_PSH|TCP_FIN);
    }

    /* Process a RST */
    if( TcpFlags & TCP_RST )
    {
        /* What we do is state dependent */
        /* 1. Set error as required */
        /* 2. Bump the Drops stat as needed */
        /* 3. Proceed directly to CLOSED without passing GO */
        /* 4. Drop the packet */
        switch( pt->t_state )
        {
        case TSTATE_SYNRCVD:
        case TSTATE_ESTAB:
        case TSTATE_FINWAIT1:
        case TSTATE_FINWAIT2:
        case TSTATE_CLOSEWAIT:
            if( pt->t_state == TSTATE_SYNRCVD )
                SockSetError( pt->hSock, NDK_ECONNREFUSED );
            else
                SockSetError( pt->hSock, NDK_ECONNRESET );
            NDK_tcps.Drops++;

            /* OK to fall through */

        case TSTATE_CLOSING:
        case TSTATE_LASTACK:
        case TSTATE_TIMEWAIT:
            if (((SOCK *)pt->hSock)->StateFlags & SS_LINGERING) {
                /*
                 * Don't call TcpClose on a lingering socket.
                 * Lingering sockets are already in the process of being closed
                 * by their owning thread. Stop the socket from lingering, wake
                 * the owning thread and let it finish the close process that it
                 * already started:
                 */
                ((SOCK *)pt->hSock)->StateFlags ^= SS_LINGERING;

                FdSignalEvent(pt->hSock, FD_EVENT_READ | FD_EVENT_WRITE | FD_EVENT_EXCEPT);
            }
            else {
                TcpClose( pt );
            }
            goto drop;
        }
    }

    /* If we still have a SYN (i.e.: wasn't removed as a dup), */
    /* then we've got problems */
    if( TcpFlags & TCP_SYN )
    {
        TcpDrop( pt, NDK_ECONNRESET );
        goto dropwithreset;
    }

    /* If the ACK bit isn't set at this point, then its an illegal packet */
    if( !(TcpFlags & TCP_ACK) )
        goto drop;

    /* Now process the ACK */

    /* See if other side ACK'd our SYN */
    if( pt->t_state == TSTATE_SYNRCVD )
    {
        /* Check for illegal ACK */
        if( SEQ_GT(pt->snd_una, ack) || SEQ_GT(ack, pt->snd_max) )
            goto dropwithreset;

        /* We're connected */
        NDK_tcps.Connects++;

        /* Set state */
        pt->t_state = TSTATE_ESTAB;
#ifdef NDK_DEBUG_TCP_STATES
        DbgPrintf(DBG_INFO, "TcpInput: set TCP state: TSTATE_ESTAB (2) "
            "(skt: 0x%x, tcp: 0x%x, task: 0x%x)", hSock, pt, TaskSelf());
#endif

        /* Notify Socket */
        SockNotify( pt->hSock, SOCK_NOTIFY_CONNECT );
    }

    /* General ACK processing */
    if( (pt->t_state>TSTATE_SYNRCVD) && SEQ_GT(ack, pt->snd_una) )
    {
        /* Make sure ACK is in range */
        if( SEQ_GT( ack, pt->snd_max ) )
        {
            NDK_tcps.RcvAckTooMuch++;
            goto dropafterack;
        }

        /* If we were timing this, then use the results */
        if( pt->t_trtt && SEQ_GT( ack, pt->t_rttseq ) )
            TcpXmitTimer( pt, pt->t_trtt );

        /* If all data has been ACK'd, turn off the rexmt timer */
        if( ack == pt->snd_max ) {
            pt->TicksRexmt = 0;
            /*
             * RFC2018 - SACK
             * If SACK is active, reset SACK TX related information
             *
             */
            if (pt->sackActive) {
                pt->TicksSackRexmt = 0;
                pt->pSack->txTableTop = 0;
                pt->pSack->txTableBottom = 0;
            }
        }
        else
        {
            /* Here we have more data, but we have been ACK'd, so we'll */
            /* reset the Rexmt timer (if not persisting) */
            if( !pt->TicksPersist )
                pt->TicksRexmt = pt->t_trtx;
        }

        /* Since new data had been ACK'd, open congestion window */
        tmpseq = pt->snd_cwnd + (uint32_t)pt->t_mss;
        if( tmpseq <= TCP_MAXWIN )
            pt->snd_cwnd = tmpseq;

        /* Get amount ACK'd and update snd_una */
        tmpseq = ack - pt->snd_una;
        pt->snd_una = ack;
        NDK_tcps.RcvAckPack++;
        NDK_tcps.RcvAckByte += tmpseq;

        pt->snd_wnd -= tmpseq;

        /* Adjust the amount of data held in SB */
        if( tmpseq <= (uint32_t)SBGetTotal( pt->hSBTx ) )
            SBRead( pt->hSBTx, tmpseq, 0, 0, 0, 0, 0 );
        else
        {
            /* Since more than all the data was ACK'd, we have */
            /* had a SYN or a FIN ack'd */
            SBFlush( pt->hSBTx, 0 );
            FinAcked = 1;
        }

        /* Notify Sock of Data ACK */
        SockNotify( pt->hSock, SOCK_NOTIFY_RCVACK );

        if( SEQ_LT( pt->snd_nxt, pt->snd_una ) )
            pt->snd_nxt = pt->snd_una;

        /* If there is no data outstanding, set CanOutput */
        /* (previous output call may have been aborted via NDK_TCP_NOPUSH) */
        if( pt->snd_nxt == pt->snd_una )
            CanOutput = 1;

        /* Special FIN ACK processing */
        if( FinAcked && pt->t_state >= TSTATE_FINWAIT1 )
        {
            if( pt->t_state == TSTATE_FINWAIT1 )
            {
                pt->TicksWait2 = TCPTV_MAX_IDLE;
                pt->t_state = TSTATE_FINWAIT2;
#ifdef NDK_DEBUG_TCP_STATES
                DbgPrintf(DBG_INFO, "TcpInput: set TCP state: TSTATE_FINWAIT2 "
                    "(skt: 0x%x, tcp: 0x%x, task: 0x%x)", hSock, pt, TaskSelf());
#endif
            }
            else if( pt->t_state == TSTATE_CLOSING )
                TcpEnterTimeWait( pt );
            else if( pt->t_state == TSTATE_LASTACK )
            {
                TcpClose( pt );
                goto drop;
            }
        }

        /* If in TIMEWAIT, all we can do is process a FIN */
        if( pt->t_state == TSTATE_TIMEWAIT )
            goto processFIN;
    }

    /* Update TCP Send Window if ACK present and window size has */
    /* changed. There is no fool-proof way of checking the age of */
    /* the packet (if we assume windows can shrink), so we'll look */
    /* for any change in size on an "up to date" ACK packet. */
    /* Note: If we got here, we know the ACK flag is set. */
    if( (pt->snd_una == ack) && ((uint32_t)pTcpHdr->WindowSize != pt->snd_wnd) )
    {
        /* Track pure Window updates */
        if( !len && (pt->snd_wndack == ack) )
            NDK_tcps.RcvWinUpd++;

forcedUpdate:
        pt->snd_wnd = (int32_t)pTcpHdr->WindowSize;
        if( pt->snd_wnd > pt->snd_wnd_max )
            pt->snd_wnd_max = pt->snd_wnd;
        pt->snd_wndack = ack;

        /* The new window may allow TcpOutput to send data */
        CanOutput = 1;
    }

    /* Process Urgent Data */
    if( (TcpFlags & TCP_URG) &&
        pTcpHdr->UrgPtr && (int32_t)pTcpHdr->UrgPtr <= len &&
        (pt->t_state < TSTATE_CLOSEWAIT || pt->t_state == TSTATE_FINWAIT1 ||
         pt->t_state == TSTATE_FINWAIT2) )
    {
        /* First, we'll set the sequence */
        tmpseq = seq + (uint32_t)pTcpHdr->UrgPtr;
        if( SEQ_GT( tmpseq, pt->rcv_up ) )
        {
            pt->rcv_up = tmpseq;
            SockSetOOBMark( pt->hSock, (pt->rcv_up - pt->rcv_nxt) +
                            SBGetTotal(pt->hSBRx) - 1 );
        }

        /* If not in OOBINLINE mode, then remove the inline data */
        if( !(SockGetOptionFlags(pt->hSock) & SO_OOBINLINE) )
        {
            unsigned char *pbTmp = pPkt->pDataBuffer + pPkt->DataOffset +
                          (int32_t)pTcpHdr->UrgPtr;
            SockSetOOBData( pt->hSock, *(pbTmp-1) );
            if( (int32_t)pTcpHdr->UrgPtr < len )
                mmCopy( pbTmp-1, pbTmp, (uint32_t)(len - (int32_t)pTcpHdr->UrgPtr) );
            eatflag = 1;
            eatseq  = seq + (int32_t)pTcpHdr->UrgPtr;
        }
    }

    /* Zap the FIN flag if its in an out of order pkt */
    if( (TcpFlags & TCP_FIN) && SEQ_GT(seq, pt->rcv_nxt) )
        TcpFlags &= ~TCP_FIN;

    /* Process Payload Data */
    if( len )
    {
        /*
         * RFC2018 - SACK requires knowing the arrival order of segments.
         * Hence; NDK maintains a counter which is incremented
         * by every data segment.
         *
         */
        pt->arrivalOrder++;

        /* If the seq is beyond us, we KNOW we can't assemble */
        /* anything! */
        if( SEQ_GT(seq, pt->rcv_nxt) )
        {
            /* If we're already holding to many, punt this one */
            if( pt->reasm_pkt_cnt >= TCP_REASM_MAXPKT )
                PBM_free( pPkt );
            else
            {
                TCPREASM *pRNext, *pRPrev;

                /* Bump the held count */
                pt->reasm_pkt_cnt++;

                /* Use the start of Frag memory as our storage */
                pR = (TCPREASM *)pPkt->pDataBuffer;
                pR->pPkt    = pPkt;
                pR->seq     = seq;
                pR->end_seq = seq+len;
                pR->eatseq  = eatseq;
                pR->eatflag = eatflag;
                pR->arrival = pt->arrivalOrder; /* RFC 2018 - SACK */
                pR->pData   = pPkt->pDataBuffer + pPkt->DataOffset;

                /* Track out of order packets */
                NDK_tcps.RcvOOPack++;
                NDK_tcps.RcvOOByte += len;

                /* We'll put it in order so we can reassemble accurately */
                pRNext = pt->pReasm;
                pRPrev = 0;
                for(;;)
                {
                    if( !pRNext || SEQ_LT( pR->seq, pRNext->seq ) )
                        break;
                    pRPrev = pRNext;
                    pRNext = pRPrev->pNext;
                    if( !pRNext )
                        break;
                }

                /* We go after pRPrev and before pRNext */
                pR->pNext = pRNext;
                if( !pRPrev )
                    pt->pReasm = pR;
                else
                    pRPrev->pNext = pR;
            }

            /*
             *  RFC2018 - SACK
             *  Force an ACK on out of order data when SACK is active
             */

            if (pt->sackActive) {
                pt->t_flags |= TF_ACKNOW;
            }
        }
        else
        {
            /* Assemble the first frag direct from our local variables */
            NDK_tcps.RcvPack++;
            NDK_tcps.RcvByte += len;
            pt->rcv_nxt += len;
            pt->t_flags |= TF_DELACK;

            /* If this frag contains an eatseq, then adjust the length */
            if( eatflag && SEQ_LEQ( seq, eatseq )
                            && SEQ_LEQ( eatseq, (seq+(uint32_t)len) ) )
                len--;

            /* If any length left, copy the data to the SB */
            if( len )
                SBWrite( pt->hSBRx, len, pPkt->pDataBuffer+pPkt->DataOffset, pPkt );
            else
                PBM_free( pPkt );

            /* Assembling this frag may allow us to assemble more */
            while( (pR = pt->pReasm) )
            {
                /* If not yet ready for this frag, break */
                if( SEQ_GT(pR->seq, pt->rcv_nxt) )
                    break;

                /* Unchain this frag */
                pt->pReasm = pR->pNext;

                /* Adjust the held count */
                pt->reasm_pkt_cnt--;

                /* Trim the whole packet if possible */
                if( SEQ_GEQ(pt->rcv_nxt, pR->end_seq) )
                {
                    NDK_tcps.RcvDupPack++;
                    NDK_tcps.RcvDupByte += (pR->end_seq - pR->seq);
                    /* We don't want this Frag (and neither does anyone else) */
                    PBM_free( pR->pPkt );
                    /* We keep trying though... */
                    continue;
                }

                /* Trim off leading data if necessary */
                if( SEQ_LT(pR->seq, pt->rcv_nxt) )
                {
                    /* Add offset to data for the amount we trimmed */
                    pR->pData += (pt->rcv_nxt - pR->seq);

                    /* Set new sequence */
                    pR->seq = pt->rcv_nxt;

                    /* If we trimmed the eatseq, then the offset is really */
                    /* one byte "back". */
                    if( eatflag && SEQ_GT(pR->seq, pR->eatseq) )
                        pR->pData--;
                }

                /* Now ready to "receive" */
                len = pR->end_seq - pR->seq;
                NDK_tcps.RcvPack++;
                NDK_tcps.RcvByte += len;
                pt->rcv_nxt += len;

                /* If this frag contains an eatseq, then adjust the length */
                if( pR->eatflag && SEQ_LEQ( pR->seq, pR->eatseq )
                                && SEQ_LEQ( pR->eatseq, pR->end_seq ) )
                    len--;

                /* If any length left, copy the data to the SB */
                if( len )
                    SBWrite( pt->hSBRx, len, pR->pData, pR->pPkt );
                else
                    PBM_free( pR->pPkt );
            }

            /* When we get here, its time to nofity Socket if the socket */
            /* rejects the data, then we ACK it immediately. This will also */
            /* update our receive window. */
            if( !SockNotify( pt->hSock, SOCK_NOTIFY_RCVDATA ) )
                pt->t_flags |= TF_ACKNOW;
        }

        /* If DELACK is set because of this packet, we'll make it an */
        /* immediate ACK if the packet has the PSH flag set. */
        if( (TcpFlags & TCP_PSH) && (pt->t_flags & TF_DELACK) )
            pt->t_flags |= TF_ACKNOW;

        /* We consumed the packet in this routine */
        pPkt = 0;
    }

    /* Process FIN */
processFIN:
    if( TcpFlags & TCP_FIN )
    {
        pt->t_flags |= TF_ACKNOW;

        if( pt->t_state <= TSTATE_ESTAB ||
            pt->t_state == TSTATE_FINWAIT2 ||
            pt->t_state == TSTATE_FINWAIT1 )
        {
#ifdef NDK_DEBUG_TCP
            DbgPrintf(DBG_INFO, "TcpInput: process FIN (skt: 0x%x, tcp: 0x%x, "
                "task: 0x%x)", hSock, pt, TaskSelf());
#endif
            pt->rcv_nxt++;

            switch( pt->t_state )
            {
            case TSTATE_SYNRCVD:
            case TSTATE_ESTAB:
                pt->t_state = TSTATE_CLOSEWAIT;
#ifdef NDK_DEBUG_TCP_STATES
                DbgPrintf(DBG_INFO, "TcpInput: set TCP state: "
                    "TSTATE_CLOSEWAIT (skt: 0x%x, tcp: 0x%x, task: 0x%x)",
                    hSock, pt, TaskSelf());
#endif
                SockNotify( pt->hSock, SOCK_NOTIFY_RCVFIN );
                break;

            case TSTATE_FINWAIT1:
                pt->t_state = TSTATE_CLOSING;
#ifdef NDK_DEBUG_TCP_STATES
                DbgPrintf(DBG_INFO, "TcpInput: set TCP state: TSTATE_CLOSING "
                    "(skt: 0x%x, tcp: 0x%x, task: 0x%x)", hSock, pt, TaskSelf());
#endif
                SockNotify( pt->hSock, SOCK_NOTIFY_RCVFIN );
                break;

            case TSTATE_FINWAIT2:
                TcpEnterTimeWait( pt );         /* Notifies "DISCONNECT" */
                break;
            }
        }
        else if( pt->t_state == TSTATE_TIMEWAIT )
            pt->TicksWait2 = 2 * TCPTV_MSL;
    }

    if( pPkt )
        PBM_free( pPkt );

    /* Call TcpOutput if needed */
    if( (CanOutput && SBGetTotal( pt->hSBTx )) || (pt->t_flags & TF_ACKNOW) )
        TcpOutput( pt );
    goto checkdestroy;

dropafterack:
    /* If this packet came in with the RST flag set, we silently */
    /* discard it. */
    if( TcpFlags & TCP_RST )
        goto drop;

    if( pPkt )
        PBM_free( pPkt );

    pt->t_flags |= TF_ACKNOW;
    TcpOutput( pt );
    goto checkdestroy;

dropwithresetfatal:
#ifdef _STRONG_CHECKING
    DbgPrintf(DBG_ERROR,"TcpIn: Fatal Socket Error");
#endif

dropwithreset:
    /* If this packet came in with the RST flag set, we silently */
    /* discard it. */
    if( TcpFlags & TCP_RST )
        goto drop;

    /* Don't gen a reset from a multicast */
    if( pPkt->Flags & (FLG_PKT_MACBCAST | FLG_PKT_MACMCAST) )
        goto drop;

    if( TcpFlags & TCP_ACK )
        TcpGenPacket( 0, IPSrc, (uint32_t)pTcpHdr->SrcPort, IPDst,
                      (uint32_t)pTcpHdr->DstPort, 0, ack, TCP_RST );
    else
    {
        if( TcpFlags & TCP_SYN )
        {
            len++;
            NDK_tcps.ConnAttemptNoPort++;
        }
        TcpGenPacket( 0, IPSrc, (uint32_t)pTcpHdr->SrcPort, IPDst,
                     (uint32_t)pTcpHdr->DstPort, seq+len, 0, TCP_RST|TCP_ACK );
    }

drop:
    if( pPkt )
        PBM_free( pPkt );

checkdestroy:
    /* Destroy Socket if needed */
    if( AbortSpawnSocket )
        SockSpawnAbort( pt->hSock );

    return;
}
