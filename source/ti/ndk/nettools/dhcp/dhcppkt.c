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
 * ======== dhcppkt.c ========
 *
 * Simple DHCP client utility
 *
 */

#include <string.h>
#include "dhcp.h"

#if DEBUGON
static void dhcpPacketDump(DHCPLEASE *pLease, int sz);
static int  dhcpTagDump(unsigned char **pBuf);
#endif

/*
 * dhcpVerifyMessage()
 *
 * Validate DHCP reply Packet and return message type
 *
 * This function also verifies the IPSERVER, and in the case of an ACK,
 * verified the IPAddress offered.
 *
 * Returns 0 on failure
 */
uint16_t dhcpVerifyMessage(DHCPLEASE *pLease, uint32_t *pIPAddr, uint32_t *pIPServer )
{
    DHCP  *pb = (DHCP *)pLease->Buffer;
    unsigned char  *pv;
    uint16_t MessageType;
    int    BufferSize;
    int    TagLength;
    uint32_t IPAddr;
    uint32_t IPServer;

#if DEBUGON
    DbgPrintf(DBG_INFO, "dhcpVerifyMessage: GetMessageType\r\n");
#endif

    BufferSize = pLease->ReceivedSize;

    MessageType = 0;
    IPServer    = 0;

    /*  If packet not for me return Null MessageType */
    if( (pb->op != BOOTREPLY) || (pb->xid != pLease->Xid) )
        return( 0 );

    /* Get the Offered IP Address */
    memmove( &IPAddr, &pb->yiaddr, 4 );

    /* PV is our options pointer */
    pv = (unsigned char *)pb->options + 4;
    /*
     * Note64: sizeof returns size_t, which is 64 bit unsigned for
     * LP64. May need to cast to int32_t to avoid implicit type conversions
     */
    BufferSize -= sizeof( DHCP ) - (SZOPTIONS-4);
    if( BufferSize < 0 )
        return(0);

    while( BufferSize > 0 )
    {
        switch( *pv++ )
        {
        case DHCPOPT_END:
            BufferSize = 0;
            break;

        case DHCPOPT_PAD:
            BufferSize--;
            break;

        case DHCPOPT_DHCP_MESSAGE_TYPE:
            TagLength = *pv++;
            MessageType = *pv;
            pv += TagLength;
            BufferSize -= TagLength+2;
            break;

        case DHCPOPT_SERVER_IDENTIFIER:
            TagLength = *pv++;
            if( TagLength == 4 )
                memmove( &IPServer, pv, 4 );
            pv += TagLength;
            BufferSize -= TagLength+2;
            break;

        default:
            /* skip over by using the length field */
            TagLength = *pv++;
            pv += TagLength;
            BufferSize -= TagLength+2;
            break;
        }
    }

    /* Drop a packet with bad options */
    if( BufferSize != 0 )
        return(0);

#if DEBUGON
    DbgPrintf(DBG_INFO, "dhcpVerifyMessage: DHCP Message Type  ");
    DbgPrintf(DBG_INFO, "dhcpVerifyMessage:   %d\r\n", MessageType);
#endif

    /* Verify Server ID */
    /*  - Must be non-zero */
    /*  - If we've already got one, it must match */
    if( !IPServer ||
        (pLease->IPServer != INADDR_BROADCAST && pLease->IPServer != IPServer) )
        return(0);

    /* Verify IP Addr on DHCPACK */
    if( MessageType == DHCPACK &&
              pLease->IPAddress != INADDR_ANY && pLease->IPAddress != IPAddr )
        return(0);

    /* Return IPAddr and IPServer if desired */
    if( pIPAddr )
        *pIPAddr = IPAddr;
    if( pIPServer )
        *pIPServer = IPServer;

    return(MessageType);
}

/*
 * dhcpPacketProcess()
 *
 * Process a DHCP reply Packet.
 *
 * By the time this function is called, we've already verified:
 *   - Message Type
 *   - IP Address Offered
 *   - Server ID
 */
void dhcpPacketProcess(DHCPLEASE *pLease)
{
    DHCP   *pb = (DHCP *)pLease->Buffer;
    unsigned char  *pv,Tag;
    int    TagLength;
    int    BufferSize;

    /* Clear any previous set options */
    dhcpOptionsClear();

    BufferSize = pLease->ReceivedSize;

#if DEBUGON
    DbgPrintf(DBG_INFO, "dhcpPacketProcess: Start \r\n");
    dhcpPacketDump(pLease, BufferSize);
#endif

    /* PV is our options pointer */
    pv = (unsigned char *)pb->options + 4;

    /*
     * Note64: sizeof returns size_t, which is 64 bit unsigned for
     * LP64. May need to cast to int32_t to avoid implicit type conversions
     */
    BufferSize -= sizeof( DHCP ) - (SZOPTIONS-4);

    while( BufferSize > 0 )
    {
        Tag = *pv++;
        switch( Tag )
        {
        case DHCPOPT_END:
            BufferSize = 0;
            break;

        case DHCPOPT_PAD:
            BufferSize--;
            break;

        case DHCPOPT_SUBNET_MASK:
            TagLength = *pv++;
            if( TagLength == 4 )
                memmove( &pLease->IPSubnetMask, pv, 4 );
            dhcpDecodeType( Tag, TagLength, pv );
            pv += TagLength;
            BufferSize -= TagLength+2;
            break;

        case DHCPOPT_ROUTER:
            TagLength = *pv++;
            if( TagLength >= 4 )
                memmove( &pLease->IPGate, pv, 4 );
            dhcpDecodeType( Tag, TagLength, pv );
            pv += TagLength;
            BufferSize -= TagLength+2;
            break;

        case DHCPOPT_DOMAIN_NAME:
            TagLength = *pv++;
            if( TagLength > SZFNAME-1 )
            {
                memmove( &pLease->DomainName, pv, SZFNAME-1 );
                pLease->DomainName[SZFNAME-1] = 0;
            }
            else
            {
                memmove( &pLease->DomainName, pv, TagLength );
                pLease->DomainName[TagLength] = 0;
            }
            dhcpDecodeType( Tag, TagLength, pv );
            pv += TagLength;
            BufferSize -= TagLength+2;
            break;

        case DHCPOPT_IP_ADDRESS_LEASE_TIME:
            TagLength = *pv++;
            if (TagLength == 4)
                memmove( &pLease->LeaseTime, pv, 4 );
            dhcpDecodeType( Tag, TagLength, pv );
            pv += TagLength;
            BufferSize -= TagLength+2;
            break;

        case DHCPOPT_RENEWAL_T1_TIME_VALUE:
            TagLength = *pv++;
            if (TagLength == 4)
                memmove( &pLease->RenewalT1Time, pv, 4 );
            dhcpDecodeType( Tag, TagLength, pv );
            pv += TagLength;
            BufferSize -= TagLength+2;
            break;

        case DHCPOPT_RENEWAL_T2_TIME_VALUE:
            TagLength = *pv++;
            if (TagLength == 4)
                memmove( &pLease->RenewalT2Time, pv, 4 );
            dhcpDecodeType( Tag, TagLength, pv );
            pv += TagLength;
            BufferSize -= TagLength+2;
            break;

        default:
            /* Add this tag to the configuration */
            TagLength = *pv++;
            dhcpDecodeType( Tag, TagLength, pv );
            pv += TagLength;
            BufferSize -= TagLength+2;
            break;
        }
    }
}


/* DEBUG FUNCTIONS */

#if DEBUGON

int dhcpTagDump(unsigned char **pBuf)
{
    char szTag[80];
    int TagLength;
    unsigned char *pv;

    pv = *pBuf;

    DbgPrintf(DBG_INFO, "dhcpTagDump: Tag %d ", *pv);
    pv++;
    TagLength = *pv++;
    DbgPrintf(DBG_INFO, "dhcpTagDump: Length is %d ", TagLength);

    {
       unsigned char *pd;
       int Length = TagLength;
       pd = pv;
       while (Length--) {
           DbgPrintf(DBG_INFO, "dhcpTagDump: %0X", *pd++);
       }
    }

    DbgPrintf(DBG_INFO, "\r\n");

    *pBuf = pv;

    return(TagLength);
}

/*
 * dhcpPacketDump()
 *
 * Dump a DHCP reply Packet.
 */
void dhcpPacketDump(DHCPLEASE *pLease, int sz)
{
    DHCP  *pb = (DHCP *)pLease->Buffer;
    unsigned char  *pv;
    int    TagLength;

    DbgPrintf(DBG_INFO, "dhcpPacketDump: Start, packet size is %d \r\n", sz);

    DbgPrintf(DBG_INFO, "dhcpPacketDump: op %d, htype %d, hlen %d, hops %d\r\n",
        pb->op,         pb->htype,      pb->hlen,       pb->hops );

    DbgPrintf(DBG_INFO, "dhcpPacketDump: C 0x%X, Y 0x%X, S 0x%X, G 0x%X\r\n",
        pb->ciaddr,     pb->yiaddr,     pb->siaddr,     pb->giaddr);

    /* PV is our options pointer */
    pv = (unsigned char *)pb->options + 4;
    /*
     * Note64: sizeof returns size_t, which is 64 bit unsigned for
     * LP64. May need to cast to int32_t to avoid implicit type conversions
     */
    sz -= sizeof( DHCP ) - (SZOPTIONS-4);
    while( sz )
    {
        switch( *pv++ )
        {
           case DHCPOPT_END:
                sz = 0;
           break;
           default:
              /* skip over by using the length field */
              pv--;
              TagLength = dhcpTagDump(&pv);
              pv += TagLength;
              sz -= TagLength+2;
           break;
        }
    }

    return;
}

#endif
