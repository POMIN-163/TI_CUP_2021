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
 * ======== icmp.c ========
 *
 * Routines related to ICMP
 *
 */

#include <stkmain.h>

uint32_t NDK_ICMPInErrors  = 0;
uint32_t NDK_ICMPOutErrors = 0;
uint32_t NDK_ICMPIn[ ICMP_MAXTYPE+1 ];
uint32_t NDK_ICMPOut[ ICMP_MAXTYPE+1 ];

/*-------------------------------------------------------------------- */
/* void ICMPChecksum( ICMPHDR *picmp ) */
/* Checksums an ICMP header */
/*-------------------------------------------------------------------- */
void ICMPChecksum( ICMPHDR *pIcHdr, uint32_t Size )
{
    int     tmp1;
    uint16_t  *pw;
    uint32_t  TSum;

    /* Checksum field is NULL in checksum calculations */
    pIcHdr->Checksum = 0;

    /* Checksum the header */
    pw = (uint16_t *)pIcHdr;
    TSum = 0;
    for( tmp1=Size; tmp1 > 1; tmp1 -= 2 )
        TSum += (uint32_t)*pw++;
#ifdef NDK_BIGENDIAN
    if( tmp1 )
        TSum += (uint32_t)(*pw & 0xFF00);
#else
    if( tmp1 )
        TSum += (uint32_t)(*pw & 0x00FF);
#endif
    TSum = (TSum&0xFFFF) + (TSum>>16);
    TSum = (TSum&0xFFFF) + (TSum>>16);
    TSum = ~TSum;

    /* Note checksum is Net/Host byte order independent */
    pIcHdr->Checksum = (uint16_t)TSum;
}

/*-------------------------------------------------------------------- */
/* ICMPGenPacket( pbHdr, hIFRx, Type, Code, dwAux ) */
/* Generates 8 byte ICMP Message Packet */
/*-------------------------------------------------------------------- */
void ICMPGenPacket( IPHDR *pIpHdr, void *hIFRx,
                    uint32_t Type, uint32_t Code, uint32_t Aux )
{
    PBM_Pkt  *pPkt;
    uint32_t     IPHdrLen,ICMPLen;
    unsigned char  *pb;
    ICMPHDR  *pIcHdr;
    uint32_t IPSrc,IPTmp;
    NETIF_DEVICE *ptr_net_device;

    /* Don't let idiots crash us */
    if( Type > ICMP_MAXTYPE )
    {
        NDK_ICMPOutErrors++;
        return;
    }

    /* Get the source of this disaster */
    IPSrc = RdNet32( &pIpHdr->IPSrc );

    /* Get the IP header len */
    IPHdrLen = (pIpHdr->VerLen & 0xF) * 4;

    /* Don't send ICMP for anything but first fragment */
    if( pIpHdr->FlagOff & ~(HNC16(IP_DF|IP_MF)) )
        return;

    /* Don't ICMP to bad address */
    if( IPSrc == INADDR_ANY || IPSrc == INADDR_BROADCAST ||
            IN_EXPERIMENTAL(IPSrc) || IN_MULTICAST(IPSrc) ||
            IN_LOOPBACK(IPSrc) )
        return;

    /* Don't ICMP because of a packet sent to a bad address */
    IPTmp = RdNet32( &pIpHdr->IPDst );
    if( IPTmp == INADDR_ANY || IPTmp == INADDR_BROADCAST ||
            IN_EXPERIMENTAL(IPTmp) || IN_MULTICAST(IPTmp) ||
            IN_LOOPBACK(IPTmp) )
        return;

    /* Bump the stats */
    NDK_ICMPOut[ Type ]++;

    /* Create the packet */
    /* Payload = ICMPHDR + 4 bytes, PLUS the org IP header + 8 bytes */
    /* Also add in the size for a STANDARD IP header for the new pkt */
    ICMPLen = ICMPHDR_SIZE + 4 + IPHdrLen + 8;

    if( !(pPkt = NIMUCreatePacket( ICMPLen + IPHDR_SIZE )) )
    {
        NDK_ICMPOutErrors++;
        return;
    }

    /* Get a pointer to the new ICMP header */
    /* pb --> Layer3 */
    /* pIcHdr --> Layer4 */
    pb     = pPkt->pDataBuffer + pPkt->DataOffset;
    pIcHdr = (ICMPHDR *)(pb + IPHDR_SIZE);

    /* Set the type and code */
    pIcHdr->Type = (unsigned char)Type;
    pIcHdr->Code = (unsigned char)Code;

    /* The next four bytes are supplied to us in Aux */
    WrNet32( pIcHdr->Data, Aux );

    /* Copy the original IP header, plus 8 bytes of original payload */
    mmCopy( pIcHdr->Data+4, pIpHdr, IPHdrLen+8 );


    /* Find an egress IF */
    if (hIFRx) {
        /* If we have the receiving IF, use it */
        ptr_net_device = (NETIF_DEVICE *)hIFRx;
    }
    else {
        /* Else get the IF from source IP (here IPTmp is our IP address) */
        ptr_net_device = BindIPHost2IF(IPTmp);
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
     */
    if (!ptr_net_device ||
        !((ptr_net_device->flags & NIMU_DEVICE_ENABLE_HW_CHKSM_TX_ALL) ||
          (ptr_net_device->flags & NIMU_DEVICE_ENABLE_HW_CHKSM_TX_ICMP))) {
        /* The h/w is not configured to compute the checksum, must do it here */
        ICMPChecksum( pIcHdr, ICMPLen );
    }
    else {
        /* When Checksums are offloaded, set the CS to zero */
        pIcHdr->Checksum = 0;
    }

    /* Get a pointer to the IP header - the packet is already Layer3 */
    pIpHdr  = (IPHDR *)pb;

    /* Set some IP header stuff */
    pIpHdr->VerLen   = 0x45;      /* Required when creating own header */
    pIpHdr->Ttl      = ICMP_TTL;  /* Use default ICMP Ttl */
    pIpHdr->Tos      = 0;         /* Rcmd'd for ICMP errors */
    pIpHdr->Protocol = 1;         /* ICMP */

    WrNet32( &pIpHdr->IPDst, IPSrc );

    /*
     * Find a good IP source address
     *
     * If we have a valid interface handle, then get its associated IP
     * address (inner call to BindIF2IPHost).
     * Otherwise, if the interface handle is NULL, try to find the best (local)
     * IP address based on the other side's IP address (IPSrc is the
     * remote host's source IP at this point in the code). This is done via
     * the call to BindIFNet2IPHost(), which uses each local IF's netmask to
     * find the network that matches the remote host's network and returns the
     * local IP address of that matching IF.
     */
    if( hIFRx || !(IPSrc = BindIFNet2IPHost( 0, IPSrc )) )
        IPSrc = BindIF2IPHost( hIFRx );
    WrNet32( &pIpHdr->IPSrc, IPSrc );

    /* Set the fragment valid data size */
    pPkt->ValidLen = ICMPLen+IPHDR_SIZE;

    /* Send the packet */
    IPTxPacket( pPkt, 0 );
}

/*-------------------------------------------------------------------- */
/* ICMPSendRtAdv( void *hIFTx, uint32_t Life, uint32_t IPAddr, int32_t dwPref ) */
/* Generates 8 byte ICMP Message Packet */
/* 'Pref' should be of type int in order to match type of */
/* struct IPCONFIG.RtcAdvPref in resif.h (a signed int). */
/*-------------------------------------------------------------------- */
void ICMPSendRtAdv( void *hIFTx, uint32_t Life, uint32_t IPAddr, int32_t Pref )
{
    PBM_Pkt    *pPkt;
    uint32_t   ICMPLen;
    unsigned char    *pb;
    ICMPRTAHDR *pRtaHdr;
    ICMPHDR    *pIcHdr;
    IPHDR      *pIpHdr;
    NETIF_DEVICE *ptr_net_device;

    /* Create the packet */
    /* Payload = ICMPHDR + ICMPRTAHDR */
    /* Also add in the size for a STANDARD IP header for the new pkt */
    ICMPLen = ICMPHDR_SIZE + ICMPRTAHDR_SIZE;

    if( !(pPkt = NIMUCreatePacket( ICMPLen + IPHDR_SIZE )) )
    {
        NDK_ICMPOutErrors++;
        return;
    }

    /* Get a pointer to the new ICMP header */
    /* pb --> Layer3 */
    /* pIcHdr --> Layer4 */
    pb     = pPkt->pDataBuffer + pPkt->DataOffset;
    pIcHdr = (ICMPHDR *)(pb + IPHDR_SIZE);

    /* Bump the stats */
    NDK_ICMPOut[ ICMP_ROUTERADVERT ]++;

    /* Set the type and code */
    pIcHdr->Type = ICMP_ROUTERADVERT;
    pIcHdr->Code = 0;

    /* Get a pointer to the RTA header */
    pRtaHdr= (ICMPRTAHDR *)pIcHdr->Data;

    /* Validate Header */
    pRtaHdr->NumAddr   =  1;
    pRtaHdr->Size      =  2;
    pRtaHdr->Lifetime     =  HNC16(Life);
    WrNet32( &pRtaHdr->rta[0].IPAddr, IPAddr );
    WrNet32( &pRtaHdr->rta[0].Pref, HNC32(Pref) );

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
    ptr_net_device = (NETIF_DEVICE *)hIFTx;
    if (!ptr_net_device ||
        !((ptr_net_device->flags & NIMU_DEVICE_ENABLE_HW_CHKSM_TX_ALL) ||
          (ptr_net_device->flags & NIMU_DEVICE_ENABLE_HW_CHKSM_TX_ICMP))) {
        /* The h/w is not configured to compute the checksum, must do it here */
        ICMPChecksum( pIcHdr, ICMPLen );
    }
    else {
        /* When Checksums are offloaded, set the CS to zero */
        pIcHdr->Checksum = 0;
    }

    /* Get a pointer to the IP header - the packet is already Layer3 */
    pIpHdr  = (IPHDR *)pb;

    /* Set some IP header stuff */
    pIpHdr->VerLen   = 0x45;      /* Required when creating own header */
    pIpHdr->Ttl      = ICMP_TTL;  /* Use default ICMP Ttl */
    pIpHdr->Tos      = 0;         /* Rcmd'd for ICMP errors */
    pIpHdr->Protocol = 1;         /* ICMP */

    WrNet32( &pIpHdr->IPSrc, IPAddr );
    WrNet32( &pIpHdr->IPDst, INADDR_BROADCAST );

    /* Set the fragment valid data size */
    pPkt->ValidLen = ICMPLen+IPHDR_SIZE;

    /* Since we know the egress IF, may as well tell IP */
    pPkt->hIFTx = hIFTx;

    /* Send the packet */
    IPTxPacket( pPkt, FLG_IPTX_BROADCAST );
}

