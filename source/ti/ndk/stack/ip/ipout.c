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
 * ======== ipout.c ========
 *
 * Routines related to IP Transmit
 *
 */

#include <stkmain.h>
#include "ip.h"

/*-------------------------------------------------------------------- */
/* IPTxPacket() */
/* Handles requests to send IP packets */
/*-------------------------------------------------------------------- */
int IPTxPacket( PBM_Pkt *pPkt, uint32_t Flags )
{
    void  *hIFRx, *hIFTx=0, *hRt=0;
    uint32_t     w,Valid;
    IPHDR    *pIpHdr;
    uint32_t IPDst,IPSrc;
    NETIF_DEVICE *ptr_net_device;

/*///// Simulate Trouble //////// */
/* static int foo = 0; */
/* if( !(foo++ % 13) ){ PktFree(hPkt); return(IPTX_SUCCESS); } */
/*/////////////////////////////// */

    if( !_IPExecuting )
    {
        PBM_free( pPkt );
        return(IPTX_SUCCESS);
    }

    /* Get the IP header pointer */
    pIpHdr = (IPHDR *)(pPkt->pDataBuffer + pPkt->DataOffset);

    /* Save the pIpHdr in case of timestamping  */
    pPkt->pIpHdr = (unsigned char*) pIpHdr;

    /* Get the the receiving interface (we use it a lot) */
    hIFRx = pPkt->hIFRx;
    ptr_net_device = (NETIF_DEVICE *)pPkt->hIFTx;

    /* Get valid bytes (we use it a lot) */
    Valid = pPkt->ValidLen;

    /*
     * Patch up header if not forwarding or in raw mode.
     *
     * This code WILL patch:
     *     TotalLength, Frag/Offset, and Id
     *
     * This code WILL NOT patch:
     *     Ttl, Protocol, IpSrc, IpDst
     */
    if( !(Flags & (FLG_IPTX_FORWARDING|FLG_IPTX_RAW)) )
    {
        /* Set Total Length */
        pIpHdr->TotalLen = HNC16( Valid );

        /* Set IP Identification */
        pIpHdr->Id = HNC16(IP_INDEX);
        IP_INDEX++;

        /* Set Don't Frag (when specified) */
        if( Flags & FLG_IPTX_DONTFRAG )
            pIpHdr->FlagOff = HNC16( IP_DF );
        else
            pIpHdr->FlagOff = 0;
    }
    else if( Flags & FLG_IPTX_RAW )
    {
        /* We'll follow BSD's lead on this one */
        if( !pIpHdr->Id )
        {
            pIpHdr->Id = HNC16(IP_INDEX);
            IP_INDEX++;
        }
    }

    /* Set Ethertype */
    pPkt->EtherType = 0x800;

    /* Now find a route and send! */

    /* Get IP Source and Destination */
    IPSrc = RdNet32( &pIpHdr->IPSrc );
    IPDst = RdNet32( &pIpHdr->IPDst );

    /*
     * Our final goal is to have the following:
     *
     * Valid EtherType in Packet Object
     * Valid IFTx in Pkt
     * Handle to a route or mapable IP Dst
     *
     * If any of these are not valid now, we can't hand the packet
     * off the the device.
     */

    /*
     * Find an egress IF
     *
     * First, check for directed IP - override to standard BCAST
     * If not, then use the egress IF set in the packet. Finally, if that's
     * not set, look up the IF associated with the local IP address
     */
    if (IP_DIRECTED_BCAST && (hIFTx = BindGetIFByDBCast(IPDst))) {
        IPDst = INADDR_BROADCAST;
    }
    else if(!(hIFTx = pPkt->hIFTx)) {
        hIFTx = BindIPHost2IF( IPSrc );
    }

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
    ptr_net_device = (NETIF_DEVICE *)hIFTx;
    if ((!ptr_net_device ||
        !((ptr_net_device->flags & NIMU_DEVICE_ENABLE_HW_CHKSM_TX_ALL) ||
          (ptr_net_device->flags & NIMU_DEVICE_ENABLE_HW_CHKSM_TX_IP))) &&
        !IPIsLoopback(IPDst))
    {
        /* Compute IP checksum in SW (HW not configured for checksum offload) */
        IPChecksum( pIpHdr );

        /*
         * For devices that support partial CS offloading, IP must contribute
         * its header offset information (even if the IP CS is computed in SW).
         * This is needed if L4 CS's are offloaded, but not L3 CS's, for example
         */
        if ((ptr_net_device != NULL) &&
            (ptr_net_device->flags & NIMU_DEVICE_HW_CHKSM_PARTIAL)) {
            /* Skip over the IP header for HW CS's */
            pPkt->csStartPos += pPkt->IpHdrLen;
    
            /* Skip over the IP header for HW CS's */
            pPkt->csInsertPos += pPkt->IpHdrLen;
        }
    }
    else
    {
        /* Checksum will be computed in HW (omit SW checksum computation) */

        /*
         * Future handling of offloading the IP header CS for devices that
         * support NIMU_DEVICE_HW_CHKSM_PARTIAL CSO would go here.
         * In this case, the CS start offset would only need to
         * skip over the L2 header (which will be summed at L2), so it could be
         * initialized to 0 here. The CS insert position should be the offset
         * to the IP header's CS field, from the start of the IP header.
         */

        /* When Checksums are fully offloaded, set the CS to zero */
        pIpHdr->Checksum = 0;
    }

    /* Check for old style BCAST - override to standard BCAST */
    if( IPDst == INADDR_ANY )
        IPDst = INADDR_BROADCAST;

    /* If we have a multicast or broadcast IP addr, we mark it as valid */
    /* and distribute it directly to the egress device(s). */

    /* IP BROADCAST/MULTICAST */
    if( IPDst == INADDR_BROADCAST || IN_MULTICAST(IPDst) )
    {
        /* If this socket can not broadcast, then this is an error */
        if( IPDst == INADDR_BROADCAST && !(Flags & FLG_IPTX_BROADCAST) )
        {
            PBM_free( pPkt );
            return( IPTX_ERROR_EACCES );
        }

        /* We need an egress interface */
        if( !hIFTx )
        {
            PBM_free( pPkt );
            return( IPTX_ERROR_ADDRNOTAVAIL );
        }

        /* Set the egress IF */
        pPkt->hIFTx = hIFTx;

        /* We don't need a route */
        PBM_setRoute( pPkt, 0 );

        goto IpRouteDone;
    }

    /* We never transmit a loopback address */
    if( IN_LOOPBACK(IPDst) )
        goto local_packet;

    /* Get preliminary route info */
    hRt   = pPkt->hRoute;

    /* If the DONTROUTE flag is set, we route to HOST routes only. */
    /* The search will discard intermediate GATEWAYS. This allows a */
    /* packet to find a local host in the event that a redirect */
    /* has been mistakenly entered into the route table. */
    if( Flags & FLG_IPTX_DONTROUTE )
    {
        hRt = 0;
        w = FLG_RTF_CLONE|FLG_RTF_REPORT|FLG_RTF_HOST;
    }
    else
        w = FLG_RTF_CLONE|FLG_RTF_REPORT;

    /* We need a route to get hIFTx */
    /* If we don't have a route, we must find one */
    if( !hRt )
    {
        /* Now find the next hop for this packet. Search for a route */
        /* WITH cloning. */
        if( !(hRt = IPGetRoute(w, IPDst)) )
        {
            if( !(Flags & FLG_IPTX_FORWARDING) )
                NDK_ips.Localnoroute++;
            else
            {
                NDK_ips.Cantforward++;
                ICMPGenPacket(pIpHdr,hIFRx,ICMP_UNREACH,ICMP_UNREACH_NET,0);
            }
            PBM_free( pPkt );
            return( IPTX_ERROR_UNREACH );
        }

        /* Set the route in the packet */
        PBM_setRoute( pPkt, hRt );

        /* Now we can deref the local copy */
        RtDeRef( hRt );
    }

    /* Here we have a route we think we can use */
    w = RtGetFlags( hRt );

    /* If the route is down, we're still unreachable */
    if( !(w & FLG_RTE_UP) )
    {
        if( !(Flags & FLG_IPTX_FORWARDING) )
            NDK_ips.Localnoroute++;
        else
        {
            NDK_ips.Cantforward++;
            ICMPGenPacket(pIpHdr,hIFRx,ICMP_UNREACH,ICMP_UNREACH_HOST,0);
        }
        PBM_free( pPkt );
        return( IPTX_ERROR_HOSTDOWN );
    }

    /* If the route is rejecting, we're still unreachable */
    if( w & FLG_RTE_REJECT )
    {
        if( !(Flags & FLG_IPTX_FORWARDING) )
            NDK_ips.Localnoroute++;
        else
        {
            NDK_ips.Cantforward++;
            ICMPGenPacket(pIpHdr,hIFRx,ICMP_UNREACH,ICMP_UNREACH_HOST,0);
        }
        PBM_free( pPkt );
        return( IPTX_ERROR_REJECTED );
    }

    /* If the route is a black hole, stop here */
    if( w & FLG_RTE_BLACKHOLE )
    {
        PBM_free( pPkt );
        return( IPTX_SUCCESS );
    }

    /* If the route is local, give the packet to the input function */
    if( w & FLG_RTE_IFLOCAL )
    {
local_packet:
        /* Note: If hIFRx is not null, we have an error */
        if( hIFRx )
        {
            /* Setting the IF to NULL tell Rx that this is a LOCAL IF */
            pPkt->hIFRx = 0;
            /* Still, better warn the operator */
            DbgPrintf(DBG_WARN,"IPTxPacket: Route loop");
        }

        /* No sense keeping the route around */
        PBM_setRoute( pPkt, 0 );

        /* Give the packet to Rx */
        IPRxPacket( pPkt );
        return( IPTX_SUCCESS );
    }

    /* Make sure the egress interface is set */
    hIFTx = RtGetIF( hRt );
    if(!hIFTx)
    {
        PBM_free(pPkt);
        return(IPTX_ERROR);
    }
    pPkt->hIFTx = hIFTx;

IpRouteDone:
    if( !(Flags & FLG_IPTX_FORWARDING) )
        NDK_ips.Localout++;
    else
    {
        /* Bump Forwarding Stats and Check for redirects */
        NDK_ips.Forward++;

        if( hIFTx == hIFRx && hRt )
        {
            uint32_t IPTAddr, IPTMask;

            IPTAddr = RtGetIPAddr( hRt );
            IPTMask = RtGetIPMask( hRt );

            /* A redirect is NOT sent in the following cases: */
            /*   - The packet was source routed */
            /*   - The Route was created or modifed by ICMP (DYNAMIC|MODIFIED) */
            /*   - The Route is the default route (IPMask != 0) */
            /*   - The sender is not a part of the next-hop subnet */

            if( !(Flags&FLG_IPTX_SRCROUTE) &&
                !(w&(FLG_RTE_DYNAMIC|FLG_RTE_MODIFIED)) &&
                IPTMask!=0 && ((IPSrc&IPTMask)==IPTAddr) )
            {
                /* We passed all the tests - we should generate an */
                /* ICMP redirect. */
                NDK_ips.Redirectsent++;
                ICMPGenPacket( pIpHdr, hIFRx, ICMP_REDIRECT,
                               ICMP_REDIRECT_HOST, IPTAddr );
            }
        }
    }

    /* Make sure the packet can fit out the egress device */
    w     = IFGetMTU( hIFTx );
    Valid = HNC16(pIpHdr->TotalLen);
    if( w < Valid )
    {
        /* Fragmentation required */
        uint16_t off,hlen,size,offbase,offtmp;
        PBM_Pkt *pPkt2;
        IPHDR   *pIpHdr2;

        /*
         * Get the 4 bit header length value from the VerLen field in the IP
         * header (VerLen stores both the IP version and the header length).
         * Units of header length are in 32-bit words (so multiply by 4)
         */
        hlen = (pIpHdr->VerLen & 0xF) * 4;

        /* If the DF flag is set, we're done */
        if( pIpHdr->FlagOff & HNC16(IP_DF) )
            goto IpRouteCantFrag;

        /* Record the offset base (we may be fragmenting a packet fragment */
        /* from a previous hop) */
        offbase = HNC16(pIpHdr->FlagOff);

        /*
         * Get the payload bytes per frag amount
         *
         * The number of bytes in each frag is computed as the MTU of the IF,
         * minus the size of the packet's header, and then rounded down to
         * an 8 byte boundary by clearing the low order 3 bits (& ~7).
         * (See TCP/IP Illustrated Vol 2, Section 10.3 Fragmentation)
         */
        w = (w - hlen) & ~7;
        if( w )
        {
            /* Send out a bunch of fragmented packets */
            for( off=0; off<(uint16_t)Valid-hlen; off+=w )
            {
                /* Get the payload size for this fragment */
                size = w;
                if( (off+size) > ((uint16_t)Valid-hlen) )
                    size = (uint16_t)Valid - hlen - off;

                /* Create the packet */
                if( !(pPkt2 = NIMUCreatePacket( size+hlen )) )
                    goto IpRouteCantFrag;

                /* Get the IP header pointer */
                pIpHdr2 = (IPHDR *)(pPkt2->pDataBuffer + pPkt2->DataOffset);

                /* Fixup packet frag info */
                pPkt2->ValidLen = size+hlen;

                /* Copy the IP header and data */
                mmCopy( pIpHdr2, pIpHdr, hlen );
                mmCopy( ((unsigned char *)pIpHdr2)+hlen,
                        ((unsigned char *)pIpHdr)+hlen+off, size );

                /* Add this fragment offset to the base offset */
                offtmp = offbase + (off>>3);

                /* Set the MF bit as required (may already be set in offbase) */
                if( (off+size) < ((uint16_t)Valid-hlen) )
                    offtmp |= IP_MF;

                /* Set the offset */
                pIpHdr2->FlagOff = HNC16(offtmp);

                /* Set the total length */
                pIpHdr2->TotalLen = HNC16( (size+hlen) );

                /*
                 * Allow driver to control the means of checksum computation.
                 *
                 * If 'flags' does not have the appropriate h/w checksum offload
                 * bits set, then the checksum is computed in software.
                 *
                 * If 'flags' does have the appropriate h/w checksum offload
                 * bits set, then don't call the checksum fxn here, as the
                 * computation will be done in the hardware. In this case,
                 * driver is responsible for enabling h/w checksum computations.
                 */
                if (!ptr_net_device ||
                    !((ptr_net_device->flags & NIMU_DEVICE_ENABLE_HW_CHKSM_TX_ALL) ||
                      (ptr_net_device->flags & NIMU_DEVICE_ENABLE_HW_CHKSM_TX_IP))) {

                    /* h/w not configured to compute checksum, do it here */
                    IPChecksum( pIpHdr2 );
                }
                else {
                    /* When Checksums are offloaded, set the CS to zero */
                    pIpHdr2->Checksum = 0;
                }

                /* Set Ethertype */
                pPkt2->EtherType = 0x800;

                /* Set Destination Info */
                pPkt2->hIFTx = hIFTx;
                PBM_setRoute( pPkt2, pPkt->hRoute );

                NDK_ips.Ofragments++;
                /*
                 *  Send the packet
                 *
                 *  Get the network interface object on which the packet will
                 *  be transmitted.  Check if we need to support the ARP
                 *  protocol or not? If not then we can bypass the ARP
                 *  resolution.
                 */
                ptr_net_device = (NETIF_DEVICE *)hIFTx;
                if (ptr_net_device &&
                    ptr_net_device->flags & NIMU_DEVICE_NO_ARP)
                {
                    /* Send the packet on the interface and return */
                    NIMUSendPacket (hIFTx, pPkt2);
                }
                else
                {
                    /* Pass the packet for the resolution. */
                    LLITxIpPacket (pPkt2, IPDst);
                }
            }

            NDK_ips.Fragmented++;
            PBM_free( pPkt );
            return( IPTX_SUCCESS );
        }

IpRouteCantFrag:
        NDK_ips.Cantfrag++;
        ICMPGenPacket(pIpHdr,hIFRx,ICMP_UNREACH,ICMP_UNREACH_NEEDFRAG,0);
        PBM_free( pPkt );
        return( IPTX_ERROR_MSGSIZE );
    }
    /*
     *  Send the packet
     *
     *  Get the network interface object on which the packet will
     *  be transmitted.  Check if we need to support the ARP
     *  protocol or not? If not then we can bypass the ARP
     *  resolution.
     */
    {
        ptr_net_device = (NETIF_DEVICE *)hIFTx;
        if (ptr_net_device &&
            ptr_net_device->flags & NIMU_DEVICE_NO_ARP)
        {
            /* Send the packet on the interface and return */
            NIMUSendPacket (hIFTx, pPkt);
        }
        else
        {
            /* Pass the packet for the resolution. */
            LLITxIpPacket (pPkt, IPDst);
        }
    }

    return( IPTX_SUCCESS );
}

