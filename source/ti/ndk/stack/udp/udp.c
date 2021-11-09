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
 * IPv4 stack - UDP layer
 *
 * Handles UDP layer processing in IPv4 stack.
 *
 */

#include <stkmain.h>

/* The byte offset, from the start of the UDP header, to the CS field */
#define NDK_UDP_CHKSM_OFFSET 6

/* Private and Public Globals */
UDPSTATS NDK_udps;                      /* Stats */

PSEUDO   upseudo;                   /* Pseudo header for checksum */

/* Utility function to checksum only the pseudo header */
static uint32_t udpPseudoCs()
{
    int tmp1;
    uint16_t *pw;
    uint32_t TSum = 0;

    pw = (uint16_t *)&upseudo;
    for (tmp1 = 0; tmp1 < 6; tmp1++) {
        TSum += (uint32_t)*pw++;
    }

    return (TSum);
}

/*-------------------------------------------------------------------- */
/* void UdpChecksum( UDPHDR *pbHdr ) */
/* Checksums the UDP header, payload, and pseudo header */
/*-------------------------------------------------------------------- */
void UdpChecksum( UDPHDR *pUdpHdr )
{
    int     tmp1;
    uint16_t  *pw;
    uint32_t  TSum;

    /* Get size in bytes (includes both the header and payload lengths) */
    tmp1 = (int)HNC16(upseudo.Length);

    /* Checksum field is NULL in checksum calculations */
    pUdpHdr->UDPChecksum = 0;

    /* Checksum the header and payload */
    pw = (uint16_t *)pUdpHdr;
    TSum = 0;
    for( ; tmp1 > 1; tmp1 -= 2 )
        TSum += (uint32_t)*pw++;
#ifdef NDK_BIGENDIAN
    if( tmp1 )
        TSum += (uint32_t)(*pw & 0xFF00);
#else
    if( tmp1 )
        TSum += (uint32_t)(*pw & 0x00FF);
#endif

    /* Checksum the pseudo header */
    TSum += udpPseudoCs();

    /*
     * The 1's compliment checksum must be stored into 16 bits. Since
     * the sum may have exceeded 65535, shift over the higher order
     * bits (the carry) so as not to lose this part of the sum when
	 * storing it into the 16 bit checksum field in the header.
     */
    TSum = (TSum&0xFFFF) + (TSum>>16);
    TSum = (TSum&0xFFFF) + (TSum>>16);

    /* Special case the 0xFFFF checksum - don't use a checksum */
    /* value of 0x0000 */
    if( TSum != 0xFFFF )
        TSum = ~TSum;

    /* Note checksum is Net/Host byte order independent */
    pUdpHdr->UDPChecksum = (uint16_t)TSum;
}

/*-------------------------------------------------------------------- */
/* UdpOutput() */
/* Called when UDP packet should be sent */
/*-------------------------------------------------------------------- */
int UdpOutput( void *hSock, unsigned char *buf, int32_t size, int32_t *pRetSize )
{
    PBM_Pkt     *pPkt;
    void     *hRoute, *hIFTx;
    uint32_t    IPHdrLen;
    int32_t     mss;
    UDPHDR      *pUdpHdr;
    uint32_t    length;
    int         error;
    SOCK        *pSock = (SOCK *) hSock;
    NETIF_DEVICE *ptr_net_device;
    uint32_t IPSrc;
    uint32_t IPDst;
    uint32_t pseudoSum = 0;

    /* Bound the size to something we can handle */
    if (pSock->TxBufSize == 0) {
        /* If send buffer size (SO_SNDBUF) is not set: */
        /* We'll try and get the MTU from the route, then the egress IF, */
        /* or finally we just assume a default. */
        if ((hRoute = SockGetRoute( hSock ))) {
            mss = (int32_t)RtGetMTU( hRoute );
            hIFTx = 0;
        }
        else if ((hIFTx = SockGetIFTx( hSock ))) {
		    /*
             * No route was set, but the socket has a TX IF (this means that
             * hSock->hIFTx was set via setsockopt(SO_IFDEVICE)). Use it to get
             * the MTU.
             */
            mss = (int32_t)IFGetMTU( hIFTx );
        }
        else {
		    /* else, use the default (both hRoute and hIFTx will be 0) */
            mss = UDP_MSS_DEFAULT;
        }
    }
    else {
        mss = pSock->TxBufSize;
        hRoute = SockGetRoute (hSock);
        if( hRoute != 0 )
            hIFTx = 0;
        else
            hIFTx = SockGetIFTx (hSock);
    }

    mss -= SockGetIpHdrSize(hSock) + UDPHDR_SIZE;   /* Sub off IpHdr & UdpHdr */

    if( size > mss )
    {
        error = NDK_EMSGSIZE;
        goto UDP_EXIT;
    }

    /* Create the packet */
    /* Payload = size */
    /* Reserve = UDPHDR_SIZE */
    length = UDPHDR_SIZE + (uint32_t)size;
    if( !(pPkt = SockCreatePacket( hSock, (uint32_t)size + UDPHDR_SIZE)) )
    {
        NDK_udps.SndNoPacket++;
        error = NDK_ENOBUFS;
        goto UDP_EXIT;
    }

    /* Get the IP header len */
    IPHdrLen = pPkt->IpHdrLen;

    /* Assign a UDP header pointer */
    pUdpHdr = (UDPHDR *)(pPkt->pDataBuffer + pPkt->DataOffset + IPHdrLen);

    /* Fill in UDP Header */

    /* Set some UDP header stuff */
    pUdpHdr->SrcPort = SockGetLPort(hSock);
    pUdpHdr->DstPort = SockGetFPort(hSock);
    pUdpHdr->Length  = HNC16( length );

    /* Copy the data */
    mmCopy( ((unsigned char *)pUdpHdr)+UDPHDR_SIZE, buf, (uint32_t)size );

    IPSrc = SockGetLIP(hSock);
    IPDst = SockGetFIP(hSock);

    /* Check if we have an IF. If we don't, then try to find one: */
    if (hIFTx) {
        ptr_net_device = (NETIF_DEVICE *)hIFTx;
    }
    else if (hRoute) {
        /* Get the IF from the route */
        ptr_net_device = RtGetIF(hRoute);
    }
    else {
        /* Finally, if we don't have a route, get the IF from our local IP */
        ptr_net_device = BindIPHost2IF(IPSrc);
    }


    /* Prep the pseudo header for checksum calculations */
    upseudo.IPSrc    = IPSrc;
    upseudo.IPDst    = IPDst;
    upseudo.Null     = 0;
    upseudo.Protocol = IPPROTO_UDP;
    upseudo.Length   = pUdpHdr->Length;

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
     * If the packet is destined to a loopback address then don't complete a s/w
     * checksum, as this packet will never travel over ethernet.
     */
    if ((!ptr_net_device ||
        !((ptr_net_device->flags & NIMU_DEVICE_ENABLE_HW_CHKSM_TX_ALL) ||
          (ptr_net_device->flags & NIMU_DEVICE_ENABLE_HW_CHKSM_TX_UDP))) &&
        !IPIsLoopback(IPDst))
    {
        /* Compute checksum in SW (HW not configured for checksum offload) */
        UdpChecksum( pUdpHdr );
    }
    else
    {
        /* Checksum will be computed in HW (omit SW checksum computation) */

        if (((ptr_net_device != NULL) &&
             (ptr_net_device->flags & NIMU_DEVICE_HW_CHKSM_PARTIAL)) &&
            !IPIsLoopback(IPDst)) {
            /*
             * Overview of steps required for partial checksums
             *
             * For HW that supports partial L4 checksums, must do/provide:
             * 
             * 1. The SW must compute/insert the L4 pseudo header CS
             * 
             * 2. The byte range to compute the CS over (byte offset of where
             *    to start and the number of bytes in the range)
             * 
             * 3. The byte offset of where to store the CS
             */

            /*
             * 1. CS the pseudo header and insert it into the L4 header (This
             *    ensures it will be part of the final CS calc done by the HW)
             */
            pseudoSum = udpPseudoCs();

            /*
             * The 1's compliment checksum must be stored into 16 bits. Since
             * the sum may have exceeded 65535, shift over the higher order
             * bits (the carry) so as not to lose this part of the sum when
			 * storing it into the 16 bit checksum field in the header.
             */
            pseudoSum = (pseudoSum & 0xFFFF) + (pseudoSum >> 16);
            pseudoSum = (pseudoSum & 0xFFFF) + (pseudoSum >> 16);

            pUdpHdr->UDPChecksum = pseudoSum;

            /*
             * 2.a Compute the byte offset to the start of the CS range
             *
             * This must ultimately point to the start of the L4 header:
             *
             * startPos = <L2 header size> + <L3 header size> + 1;
             *
             * However, to adhere to layering principles, the header offsets
             * of the lower layers will be calculated at those respective
             * layers. Since HW offset is 1 based, initialize the offset to 1
             * here.
             */
            pPkt->csStartPos = 1;

            /* 2.b Provide the number of bytes to CS (L4 header + payload sz) */
            pPkt->csNumBytes = (uint32_t)NDK_ntohs(pUdpHdr->Length);
 
            /*
             * 3. Compute the byte offset to the L4 header's CS field
             *
             * This must ultimately point to the CS field in the L4 header:
             *
             * startPos = [L2 hdr sz] + [L3 hdr sz] + [CS field offset] + 1;
             *
             * Again, due to layering, only L4 offsets are done here.
             * Since HW offset is 1 based, add 1 to the offset here.
             */
            pPkt->csInsertPos = NDK_UDP_CHKSM_OFFSET + 1;
        }
        else {
            /* HW CS's are fully offloaded; set the CS field to zero */
            pUdpHdr->UDPChecksum = 0;
        }
    }

    /* Indicate the preferred route and IF */
    PBM_setRoute( pPkt, hRoute );
    pPkt->hIFTx = hIFTx;

    /* Set the timestamp call-out function */
    pPkt->pTimestampFxn = pSock->pTimestampFxn;

    /* Count it */
    NDK_udps.SndTotal++;

    /* Send the packet */
    error=(int)IPTxPacket(pPkt,SockGetOptionFlags(hSock)&FLG_IPTX_SOSUPPORTED);

UDP_EXIT:
    if( !error )
        *pRetSize = size;
    else
        *pRetSize = 0;

    return( error );
}

/*-------------------------------------------------------------------- */
/* UdpInput() */
/* Rx UDP Packet */
/*-------------------------------------------------------------------- */
void UdpInput( PBM_Pkt *pPkt )
{
    PBM_Pkt    *pPktCopy;
    void       *hSock, *hSockNext, *hSBRx, *hIFTx;
    uint32_t   w,IPHdrLen,UDPLen;
    uint32_t   IPSrc,IPDst;
    IPHDR      *pIpHdr;
    UDPHDR     *pUdpHdr;
    uint32_t   MaxFlag;
    NETIF_DEVICE *ptr_net_device;

    /* Get the IP header len */
    IPHdrLen = pPkt->IpHdrLen;

    /* Assign an IP header pointer */
    pIpHdr = (IPHDR *)(pPkt->pDataBuffer + pPkt->DataOffset);

    /* Assign a UDP header pointer */
    pUdpHdr = (UDPHDR *)(pPkt->pDataBuffer + pPkt->DataOffset + IPHdrLen);

    /* Count the total number of packets in */
    NDK_udps.RcvTotal++;

    /* Get the total length of the UDP message */
    UDPLen = (uint32_t)(HNC16( pIpHdr->TotalLen )) - IPHdrLen;

    /* Check for packet too small */
    if( UDPLen < UDPHDR_SIZE )
    {
        NDK_udps.RcvShort++;
        PBM_free( pPkt );
        return;
    }

    /* Check for bad header length */
    if( pUdpHdr->Length != HNC16(UDPLen) )
    {
        NDK_udps.RcvBadLen++;
        PBM_free( pPkt );
        return;
    }

    /* We need some stuff for the pseudo header */
    /* Get source and destination */
    IPSrc = RdNet32(&pIpHdr->IPSrc);
    IPDst = RdNet32(&pIpHdr->IPDst);

    if( pUdpHdr->UDPChecksum )
    {
        /* Init pseudo header for checksum */
        upseudo.IPSrc    = IPSrc;
        upseudo.IPDst    = IPDst;
        upseudo.Null     = 0;
        upseudo.Protocol = IPPROTO_UDP;
        upseudo.Length   = pUdpHdr->Length;

        /*
         * Allow driver to control the means of checksum computation.
         *
         * If 'flags' does not have the appropriate h/w checksum offload
         * bits set, then the checksum is computed in software.
         *
         * If 'flags' does have the appropriate h/w checksum offload bits
         * set, then don't call the checksum fxn here, as the computation will
         * be done in the hardware. In this case, driver is responsible for
         * enabling h/w checksum computations.
         *
         * Note: CS verification of reassembled (fragmented) packets must always
         * be done in SW
         */
        ptr_net_device = (NETIF_DEVICE *)pPkt->hIFRx;
        if (!ptr_net_device ||
            !((ptr_net_device->flags & NIMU_DEVICE_ENABLE_HW_CHKSM_RX_ALL) ||
              (ptr_net_device->flags & NIMU_DEVICE_ENABLE_HW_CHKSM_RX_UDP)) ||
            pPkt->csNumBytes > 0) {
            /*
             * Verify checksum in SW (HW not configured for checksum offload
             * or this is a reassembled packet)
             */
            w = (uint32_t)pUdpHdr->UDPChecksum;
            UdpChecksum( pUdpHdr );
            if( w != (uint32_t)pUdpHdr->UDPChecksum )
            {
                NDK_udps.RcvBadSum++;
                PBM_free( pPkt );
                return;
            }
        }
    }

    /* Get the actual number of bytes to "receive" */
    UDPLen -= UDPHDR_SIZE;

    /* Set the Frag Parms to hand off to a SB */
    pPkt->Aux1 = 0x10000 | (uint32_t)(pUdpHdr->SrcPort);
    pPkt->Aux2 = IPSrc;
    pPkt->ValidLen = UDPLen;
    pPkt->DataOffset += IPHdrLen+UDPHDR_SIZE;

    /* Handle BROADCAST/MULTICAST */
    if( IPDst == INADDR_BROADCAST || IN_MULTICAST( IPDst ) ||
            (pPkt->Flags & (FLG_PKT_MACMCAST|FLG_PKT_MACBCAST)) )
    {
        /* Multicast Packet */
        /* We have to copy the frag for each socket with a matchng PCB */
        hSock = SockPcbResolveChain( 0, SOCKPROT_UDP, 0, IPDst,
                                     (uint32_t)pUdpHdr->DstPort, IPSrc,
                                     (uint32_t)pUdpHdr->SrcPort );
        w = 0;          /* Recv Flag */
        while( hSock )
        {
            /* Get the handle to the next match (so we know if we have one) */
            hSockNext = SockPcbResolveChain( hSock, SOCKPROT_UDP, 0, IPDst,
                                             (uint32_t)pUdpHdr->DstPort, IPSrc,
                                             (uint32_t)pUdpHdr->SrcPort );

            /* Get the broadcast IF of this socket */
            hIFTx = SockGetIFTx( hSock );

            /* If there is a specified BCAST device, and the packet's */
            /* Rx device doesn't match it, then we ignore the packet */
            if( hIFTx && ( hIFTx != pPkt->hIFRx ) )
            {
                hSock = hSockNext;
                continue;
            }

            /* Flag that we have matched a socket */
            w = 1;

            /* Get the Linear Receive Buffer */
            hSBRx = SockGetRx( hSock );

            if( SBGetSpace( hSBRx ) < (int32_t)UDPLen )
                NDK_udps.RcvFull++;
            else
            {
                /* Copy the frag if there may be more matches */
                /* Else this is our last time through the loop */
                if( !hSockNext || !(pPktCopy = PBM_copy( pPkt )) )
                {
                    pPktCopy = pPkt;
                    pPkt = 0;
                    hSockNext = 0;
                }

                /* Give the frag to the SB */
                SBWrite( hSBRx, (int32_t)UDPLen, 0, pPktCopy );

                /* Notify the Socket */
                SockNotify( hSock, SOCK_NOTIFY_RCVDATA );
            }

            /* Check next matching socket */
            hSock = hSockNext;
        }

        /* If we didn't match anyone, count it */
        if( !w )
            NDK_udps.RcvNoPortB++;

        /* Free the packet if we didn't use it */
        if( pPkt )
            PBM_free( pPkt );

        return;
    }

    /* Find a PCB for this packet */
    hSock = SockPcbResolve( SOCKPROT_UDP, IPDst, (uint32_t)pUdpHdr->DstPort,
                            IPSrc, (uint32_t)pUdpHdr->SrcPort,
                            SOCK_RESOLVE_BEST, &MaxFlag );

    /* If there's no PCB, send an ICMP error */
    if( !hSock )
    {
        if( !(pPkt->Flags & FLG_PKT_MACBCAST) && UDP_SEND_ICMP_PORTUNREACH)
            ICMPGenPacket( pIpHdr, pPkt->hIFRx, ICMP_UNREACH,
                           ICMP_UNREACH_PORT, 0 );
        NDK_udps.RcvNoPort++;
        PBM_free( pPkt );
        return;
    }

    /* Get the Receive Socket Buffer */
    hSBRx = SockGetRx( hSock );

    /* If there's no space on the receiver queue, then discard the packet */
    /* with an ICMP error */
    if( SBGetSpace( hSBRx ) < (int32_t)UDPLen )
    {
        if( !(pPkt->Flags & FLG_PKT_MACBCAST) )
            ICMPGenPacket( pIpHdr, pPkt->hIFRx,
                           ICMP_SOURCEQUENCH, 0, 0 );
        NDK_udps.RcvFull++;
        PBM_free( pPkt );
        return;
    }

    /* Give the frag to the SB */
    SBWrite( hSBRx, (int32_t)UDPLen, 0, pPkt );

    /* Notify the Socket */
    SockNotify( hSock, SOCK_NOTIFY_RCVDATA );
}

