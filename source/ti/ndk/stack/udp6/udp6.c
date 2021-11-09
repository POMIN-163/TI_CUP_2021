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
 * ======== udp6.c ========
 *
 * The file implements the UDP protocol over IPv6.
 *
 */


#include <stkmain.h>

#ifdef _INCLUDE_IPv6_CODE

/* The byte offset, from the start of the UDP header, to the CS field */
#define NDK_UDP6_CHKSM_OFFSET 6

/**********************************************************************
 *************************** Global Variables *************************
 **********************************************************************/

/* UDP Statistics for the IPv6 UDP stack. */
UDPSTATS NDK_udp6_stats;

/**********************************************************************
 ***************************** UDP6 Functions *************************
 **********************************************************************/

static void processMulticast(PBM_Pkt * pPkt, IP6N srcAddr, int srcPort,
                             IP6N dstAddr, int dstPort, int UDPLen);

/**
 *  @b Description
 *  @n
 *      The function is called to transmit a UDP packet over an IPv6 network.
 *
 *  @param[in]  hSock
 *      Socket handle using which the packet is sent out.
 *  @param[in]  buf
 *      Data Buffer which is to be sent out.
 *  @param[in]  size
 *      Length of the data buffer to be sent out.
 *  @param[out] pRetSize
 *      Actual Length of data sent out.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   Non Zero
 */
int Udp6Output( void *hSock, unsigned char *buf, int32_t size, int32_t *pRetSize )
{
    PBM_Pkt*      pPkt;
    void         *hRoute6;
    uint16_t      mss;
    UDPHDR*       pUdpHdr;
    uint32_t      length;
    SOCK6*        pSock;
    PSEUDOV6      pseudo_hdr;
    BIND6_ENTRY  *hbind6Obj = NULL;
    NETIF_DEVICE *ptr_net_device = NULL;
    uint32_t      pseudoSum = 0;

    /* Initialize the return value. */
    *pRetSize = 0;

    /* Get the socket Information. */
    pSock = (SOCK6 *)hSock;

    /* Is the socket associated with a Transmission Buffer Size? */
    if (pSock->TxBufSize == 0)
    {
        /* No; none is specified. Check if there is a ROUTE associated with the socket? */
        hRoute6 = Sock6GetRoute (hSock);
        if (hRoute6)
        {
            /* YES. Get the MTU associated with the Route. */
            mss = (int32_t)Rt6GetMTU (hRoute6);
        }
        else
        {
            mss = UDP_MSS_DEFAULT;
        }
    }
    else
    {
        /* There is a transmit buffer size configured. */
        mss = pSock->TxBufSize;

        /* Get the ROUTE6 Information. */
        hRoute6 = Sock6GetRoute (hSock);
    }

    /* Subtract off the IP Header + Extension headers Size and UDP Header Size. */
    mss -= Sock6GetIpHdrSize(hSock) + UDPHDR_SIZE;
    if( size > mss )
        return NDK_EMSGSIZE;

    /* Allocate memory for the packet. */
    length = UDPHDR_SIZE + (uint32_t)size;
    if( !(pPkt = Sock6CreatePacket( hSock, length, IPPROTO_UDP )))
    {
        NDK_udp6_stats.SndNoPacket++;
        PBM_free (pPkt);
        return NDK_ENOBUFS;
    }

    /* Store the ROUTE6 Information in the packet itself and increment the reference
     * counter for the ROUTE6 Entry since now the packet is carrying the ROUTE6 Handle. */
    PBM_setRoute6 (pPkt, hRoute6);

    /* Get the pointer to the UDP Header. */
    pUdpHdr = (UDPHDR *)(pPkt->pDataBuffer + pPkt->DataOffset + pPkt->IpHdrLen);

    /* Populate the UDP Header. */
    pUdpHdr->SrcPort = Sock6GetLPort(hSock);
    pUdpHdr->DstPort = Sock6GetFPort(hSock);
    pUdpHdr->Length  = HNC16(length);

    /* Copy the data into the data payload. */
    mmCopy( ((unsigned char *)pUdpHdr)+UDPHDR_SIZE, buf, (uint32_t)size);

    /*
     * Compute the UDPv6 Checksum.
     *
     * For TX packets, the checksum should be initialized to 0. This is true
     * for both possible cases:
     *
     * a. CS computed in s/w: IPv6Layer4ComputeChecksum() requires the CS to be
     *    set to 0
     *
     * b. CS computed in h/w: The CS should be zero when it reaches the TX CSO
     *    engine.
     */
    pUdpHdr->UDPChecksum = 0;

    /* Find the egress interface */
    if (hRoute6) {
        /* Get the IF from the route */
        ptr_net_device = Rt6GetIF(hRoute6);
    }
    else {
        /* If we don't have a route, attempt to get the IF from our local IP */
        hbind6Obj = Bind6FindByHost(NULL, Sock6GetLIP(hSock));
        if (hbind6Obj) {
            ptr_net_device = Bind6GetInterfaceHandle((void *)hbind6Obj);
        }
    }

    /* Create the Pseudo Header for checksum calculations. */
    pseudo_hdr.SrcAddr = Sock6GetLIP(hSock);
    pseudo_hdr.DstAddr = Sock6GetFIP(hSock);
    pseudo_hdr.PktLen  = pUdpHdr->Length;
    pseudo_hdr.Rsvd[0] = 0;
    pseudo_hdr.Rsvd[1] = 0;
    pseudo_hdr.Rsvd[2] = 0;
    pseudo_hdr.NxtHdr  = IPPROTO_UDP;

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
     */
    if (!ptr_net_device ||
        !((ptr_net_device->flags & NIMU_DEVICE_ENABLE_HW_CHKSM_TX_ALL) ||
          (ptr_net_device->flags & NIMU_DEVICE_ENABLE_HW_CHKSM_TX_UDP))) {
        /* Compute checksum in SW (HW not configured for checksum offload) */
        pUdpHdr->UDPChecksum =
            IPv6Layer4ComputeChecksum ((unsigned char *)pUdpHdr, &pseudo_hdr);
    }
    else
    {
        /* Checksum will be computed in HW (omit SW checksum computation) */

        if ((ptr_net_device->flags & NIMU_DEVICE_HW_CHKSM_PARTIAL)) {
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
            pseudoSum = IPv6Layer4PseudoHdrChecksum(&pseudo_hdr);

            /*
             * The 1's compliment checksum must be stored into 16 bits. Since
             * the sum may have exceeded 65535, shift over the higher order
             * bits (the carry) so as not to lose this part of the sum when
			 * storing it into the 16 bit checksum field in the header.
             */
            while (pseudoSum >> 16) {
                pseudoSum = (pseudoSum & 0xFFFF) + (pseudoSum >> 16);
            }

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
            pPkt->csInsertPos = NDK_UDP6_CHKSM_OFFSET + 1;
        }
    }

    /* Increment the transmission statistics. */
    NDK_udp6_stats.SndTotal++;

    /* Pass the packet to the IPv6 Layer for transmission. */
    IPv6TxPacket (pPkt, 0);

    /* The packet has been successfully transmitted */
    *pRetSize = size;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is the receive handler which is invoked when a UDP packet
 *      is received over IPv6.
 *
 *  @param[in]  pPkt
 *      The actual UDP6 packet received.
 *  @param[in]  ptr_ipv6hdr
 *      The IPv6 Header of the received UDP packet.
 *
 *  @retval
 *      Not Applicable.
 */
void Udp6Input (PBM_Pkt* pPkt, IPV6HDR* ptr_ipv6hdr)
{
    UDPHDR*     ptr_udpHdr;
    uint16_t    PayloadLength;
    PSEUDOV6    pseudo_hdr;
    uint16_t    checksum;
    void     *hSock;
    void     *hSBRx;
    IP6N        srcAddr;
    IP6N        dstAddr;
    uint32_t    MaxFlag;
    NETIF_DEVICE *ptr_net_device;

    /*
     * NOTE: use mmCopy to copy the 16 byte IP6N addresses out of ptr_ipv6hdr
     * as this data is potentially un-aligned (SDOCM00097361)
     */
    mmCopy((char *)&srcAddr, (char *)&ptr_ipv6hdr->SrcAddr, sizeof(IP6N));
    mmCopy((char *)&dstAddr, (char *)&ptr_ipv6hdr->DstAddr, sizeof(IP6N));

    /* Get the pointer to the UDP header. */
    ptr_udpHdr = (UDPHDR *)(pPkt->pDataBuffer + pPkt->DataOffset);

    /* Increment the number of UDP Packets received. */
    NDK_udp6_stats.RcvTotal++;

    /* Get the length of the UDP packet. */
    PayloadLength = NDK_ntohs(ptr_ipv6hdr->PayloadLength);

    /* Check for packet too small */
    if( PayloadLength < UDPHDR_SIZE )
    {
        /* Packet is too small; cleanup and return. */
        NDK_udp6_stats.RcvShort++;
        PBM_free( pPkt );
        return;
    }

    // TODO UDP CS is optional, therefore check for CS==0 here just like udp v4
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
     */
    ptr_net_device = (NETIF_DEVICE *)pPkt->hIFRx;
    if (!ptr_net_device ||
        !((ptr_net_device->flags & NIMU_DEVICE_ENABLE_HW_CHKSM_RX_ALL) ||
          (ptr_net_device->flags & NIMU_DEVICE_ENABLE_HW_CHKSM_RX_UDP)) ||
        pPkt->csNumBytes > 0)
    {
        /* H/w is not configured to compute the checksum, must do it here */

        /* Get the original checksum. */
        checksum = ptr_udpHdr->UDPChecksum;
        if (checksum == 0xFFFF) {
            checksum = 0;
        }

        /* Create the Pseudo Header for checksum calculations. */
        pseudo_hdr.SrcAddr = srcAddr;
        pseudo_hdr.DstAddr = dstAddr;
        pseudo_hdr.PktLen  = NDK_htons(pPkt->ValidLen);
        pseudo_hdr.Rsvd[0] = 0;
        pseudo_hdr.Rsvd[1] = 0;
        pseudo_hdr.Rsvd[2] = 0;
        pseudo_hdr.NxtHdr  = IPPROTO_UDP;

        /* Before we compute the checksum; initialize it to 0. */
        ptr_udpHdr->UDPChecksum = 0;

        ptr_udpHdr->UDPChecksum = IPv6Layer4ComputeChecksum ((unsigned char *)ptr_udpHdr, &pseudo_hdr);

        /* Validate the checksum. */
        if( checksum != ptr_udpHdr->UDPChecksum)
        {
            /* Checksums are not correct; packet is dropped */
            NDK_udp6_stats.RcvBadSum++;
            PBM_free (pPkt);
            return;
        }
    }

    /* Skip the UDP Header. */
    pPkt->DataOffset += (sizeof(UDPHDR));
    pPkt->ValidLen   -= (sizeof(UDPHDR));

    /* Compute the actual "data" payload. */
    PayloadLength = PayloadLength - UDPHDR_SIZE;

    /* Is the Destination Address MULTICAST? */
    if (IPv6IsMulticast(dstAddr)) {
        processMulticast(pPkt, srcAddr, ptr_udpHdr->SrcPort,
                         dstAddr, ptr_udpHdr->DstPort, PayloadLength);

        return;
    }

    /* Control comes here; implies that the packet is a UNICAST packet; we need to get a
     * matching socket handle for the packet. */
    hSock = Sock6PcbResolve (SOCKPROT_UDP, dstAddr, ptr_udpHdr->DstPort,
                             srcAddr, ptr_udpHdr->SrcPort, SOCK_RESOLVE_BEST, &MaxFlag);
    if (hSock == 0)
    {
        /* There is no matching socket for this combination. In this case we send out the
         * Destination Unreachable Message (Code = ICMPV6_DST_UNREACH_PORT) */
        ICMPv6SendDstUnreachable (ICMPV6_DST_UNREACH_PORT, pPkt);

        /* Increment the statistics. */
        NDK_udp6_stats.RcvNoPort++;

        /* Cleanup the original packet. */
        PBM_free (pPkt);
        return;
    }

    /* Get the Receive Socket Buffer and ensure there is space in the receiver socket queue
     * to be abke to handle the packet. */
    hSBRx = Sock6GetRx( hSock );

    /* Ensure there is space in the socket buffer. */
    if (SB6GetSpace( hSBRx ) < PayloadLength)
    {
        NDK_udp6_stats.RcvFull++;
        PBM_free( pPkt );
        return;
    }

    /* Copy the Source address/port for "PEER" Information. */
    mmCopy((void *)&pPkt->SrcAddress,(void *)&srcAddr, sizeof(IP6N));
    pPkt->SrcPort = ptr_udpHdr->SrcPort;

    /* Give the frag to the Socket Buffer. */
    SB6Write( hSBRx, pPkt->ValidLen, 0, pPkt );

    /* Notify the Socket */
    Sock6Notify( hSock, SOCK_NOTIFY_RCVDATA );
    return;
}

static void processMulticast(PBM_Pkt * pPkt, IP6N srcAddr, int srcPort,
                             IP6N dstAddr, int dstPort, int UDPLen)
{
    void     *hSock;
    void     *hSockNext;
    void     *hSBRx;
    PBM_Pkt *   pPktCopy;
    int         w;

    hSock = Sock6PcbResolveChain(0, SOCKPROT_UDP, 0, dstAddr,
                                 dstPort, srcAddr, srcPort);

    w = 0;          /* Recv Flag */
    while (hSock) {
        /* Get the handle to the next match (so we know if we have one) */
        hSockNext = Sock6PcbResolveChain(hSock, SOCKPROT_UDP, 0, dstAddr,
                                         dstPort, srcAddr, srcPort);

        /* Flag that we have matched a socket */
        w = 1;

        /* Get the Linear Receive Buffer */
        hSBRx = Sock6GetRx(hSock);

        if (SBGetSpace(hSBRx) < (int32_t)UDPLen) {
            NDK_udp6_stats.RcvFull++;
        }
        else {
            /* Copy the frag if there may be more matches */
            /* Else this is our last time through the loop */
            if (!hSockNext || !(pPktCopy = PBM_copy( pPkt ))) {
                pPktCopy = pPkt;
                pPkt = 0;
                hSockNext = 0;
            }

            /* Give the frag to the SB */
            SBWrite(hSBRx, (int32_t)UDPLen, 0, pPktCopy);

            /* Copy the Source address/port for "PEER" Information. */
            mmCopy((void *)&pPktCopy->SrcAddress, (void *)&srcAddr,
                   sizeof(IP6N));
            pPktCopy->SrcPort = srcPort;

            /* Notify the Socket */
            Sock6Notify(hSock, SOCK_NOTIFY_RCVDATA);
        }

        /* Check next matching socket */
        hSock = hSockNext;
    }

    /* If we didn't match anyone, count it */
    if (!w) {
        NDK_udp6_stats.RcvNoPortB++;
    }

    /* Free the packet if we didn't use it */
    if (pPkt) {
        PBM_free(pPkt);
    }

    return;
}

#endif /* _INCLUDE_IPv6_CODE */
