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
 * ======== ip6if.h ========
 *
 * Common structures and definitions for IPv6
 *
 */


#ifndef _C_IPV6IF_INC
#define _C_IPV6IF_INC  /* #defined if this .h file has been included */

#ifdef __cplusplus
extern "C" {
#endif

/* MINIMUM IPV6 MTU required for an interface to be able to execute the IPv6 Protocol. */
#define MIN_IPV6_MTU            1280
#define MAX_IPV6_PKTSIZE        65535

/* Next Header definitons (RFC 2460). */
#define IPV6_HOPOPTS_HEADER     0x0     /* Hop-by-Hop options */
#define IPV6_ROUTING_HEADER     0x2b    /* Routing Header */
#define IPV6_FRAGMENT_HEADER    0x2c    /* Fragmentation Header */
#define ICMPv6_HEADER           0x3a    /* ICMPv6 Header */
#define IPV6_NONE_HEADER        0x3b    /* No Next Header */
#define IPV6_DSTOPTS_HEADER     0x3c    /* Destination options Header */

/* Options that can be carried in Hop-by-Hop/Destination options headers */
#define IPV6_PAD1_OPT   0x0
#define IPV6_PADN_OPT   0x1

/* More fragment bit definition for fragmented packets */
#define IPV6_MF_BIT     0x0001

/* Lifetime values for which the address bindings remain valid as specified by the RFC. */
extern const uint32_t INFINITE_LT;

/* Some well defined IPv6 Address and Mask Information. */
extern IP6N  IPV6_LINKLOCALMASK;
extern IP6N  IPV6_MULTICASTMASK;
extern IP6N  IPV6_UNSPECIFIED_ADDRESS;
extern IP6N  IPV6_LOOPBACK_ADDRESS;
extern IP6N  IPV6_HOST_MASK;
extern IP6N  IPV6_ALL_NODES_ADDRESS;
extern IP6N  IPV6_ALL_ROUTER_ADDRESS;

/* IPv6 Protocol Constants. */
extern const uint32_t IPV6_MAX_MULTICAST_SOLICIT;
extern const uint32_t IPV6_MAX_UNICAST_SOLICIT;
extern const uint32_t IPV6_RETRANS_TIME;
extern const uint32_t IPV6_DELAY_FIRST_PROBE_TIME;
extern const uint32_t IPV6_REACHABLE_TIME;
extern const uint32_t IPV6_MAX_RTR_SOLICITATION_DELAY;
extern const uint32_t IPV6_RTR_SOLICITATION_INTERVAL;
extern const uint32_t IPV6_MAX_RTR_SOLICITATIONS;

/**
 * @brief
 *  The structure describes the IPv6 Device Record
 *
 * @details
 *  The IPv6 Device record is maintained per each NIMU Network Interface
 *  Object in the system. If this record is NULL then IPv6 has not been
 *  instantiated on the interface and so the interface will not be able
 *  to receive or transmit IPv6 packets. The structure is used to store
 *  IPV6 host specific parameters on a per interface basis.
 */
typedef struct IPV6_DEV_RECORD
{
    /**
     * @brief   This is the  64 bit EUI Identifier.
     */
    uint32_t        EUI64[2];

    /**
     * @brief   This is the handle to the Link Local BIND Object
     */
    void            *hLinkLocalBind;

    /**
     * @brief   This is the handle to the NIMU Network Object.
     */
    void            *hIF;

    /**
     * @brief   This is the handle to Link Loopback route.
     */
    void            *hLinkLoopbackRoute;

    /**
     * @brief   This is the hop limit to be used when sending UNICAST packets.
     */
    unsigned char         CurHopLimit;

    /**
     * @brief   This is the Link MTU associated with the device.
     */
    uint16_t        LinkMTU;

    /**
     * @brief   This is the base Value used for computing the random value
     */
    uint32_t        BaseReachableTime;

    /**
     * @brief   This is the time for which a neighbour is considered reachable
     */
    uint32_t        ReachableTime;

    /**
     * @brief   This is the time between retransmissions of NS messages
     */
    uint32_t        RetransTimer;

    /**
     * @brief   This is the Managed Flag copied from the RA Message.
     */
    unsigned char         ManagedFlag;

    /**
     * @brief   This is the Other Config Flag copied from the RA Message.
     */
    unsigned char         OtherConfigFlag;

    /**
     * @brief   This is DAD Status call back function which is invoked after the
     * DAD State machine has been executed and can be used to notify application
     * developers on the result.
     */
    void (*DADStatus)(IP6N Address, uint16_t dev_index, unsigned char Status);
}IPV6_DEV_RECORD;

/**
 * @brief
 *  The structure describes the IPv6 Statistics Block.
 *
 * @details
 *  This structure is used to hold various packet stats for
 *  receive and transmit paths of the core IPv6 module.
 *  The stats maintained in this structure are defined
 *  as per IP MIBs defined in RFC 4293.
 */
typedef struct {

    /**
     * @brief    Total number of packets received.
     */
    uint32_t  InReceives;

    /**
     * @brief    Packets dropped because of header errors.
     */
    uint32_t  InHdrErrors;

    /**
     * @brief    Number of multicast packets received.
     */
    uint32_t  InMcastPkts;

    /**
     * @brief    Number of truncated packets received.
     */
    uint32_t  InTruncatedPkts;

    /**
     * @brief    Number of packets dropped because we had to forward.
     */
    uint32_t  InForwErrors;

    /**
     * @brief    Packets dropped because extension header parse failed.
     */
    uint32_t  InExtnHdrErrors;

    /**
     * @brief    Number of fragments awaiting reassembly.
     */
    uint32_t  InReasmReqds;

    /**
     * @brief    Number of fragment reassembly failures/timeouts.
     */
    uint32_t  InReasmFails;

    /**
     * @brief    Number of successful reassemblies.
     */
    uint32_t  InReasmOKs;

    /**
     * @brief    Packets dropped because the L4 Proto was not supported.
     */
    uint32_t  InUnknownProtos;

    /**
     * @brief    Packets successfully delivered to host stack.
     */
    uint32_t  InDelivers;

    /**
     * @brief    Total number of packets transmit requests from host stack.
     */
    uint32_t  OutRequests;

    /**
     * @brief    Packets dropped because of no route to destination.
     */
    uint32_t  OutNoRoutes;

    /**
     * @brief    Number of Packets on Tx path that need to be fragmented
     * in order to be transmitted successfully.
     */
    uint32_t  OutFragReqds;

    /**
     * @brief    Number of Packets on Tx path that need to be fragmented but
     * couldnt be fragmented because of some error.
     */
    uint32_t  OutFragFails;

    /**
     * @brief    Number of Packets on Tx path that were fragmented successfully.
     */
    uint32_t  OutFragOKs;

    /**
     * @brief    Number of Packets generated on Tx path due to fragmentation.
     */
    uint32_t  OutFragCreates;

    /**
     * @brief    Number of Multicast packets sent out by the host IPv6 stack.
     */
    uint32_t  OutMcastPkts;

    /**
     * @brief    Number of packets discarded because of errors on Tx path.
     */
    uint32_t  OutDiscards;

    /**
     * @brief    Number of packets successfully transmitted by the host IPv6 stack.
     */
    uint32_t  OutTransmits;

} IPV6STATS;


/* This is the IPv6 Master Control Block and stores information about all the
 * data structures used by the IPv6 Module in the NDK Core stack. */
typedef struct IPV6_MCB
{
    unsigned char     IsRouter;   /* Set to 1 to operate as an IPv6 Router. */
    IPV6STATS   ip6stats;   /* IPv6 Statistics */
    unsigned char     IsZeroCompressionEnabled;   /* Set to 1 to enable IPv6 stack to recognize and process
                                             * compressesed IPv6 addresses.
                                             */
}IPV6_MCB;

/* Global IPv6 MCB */
extern IPV6_MCB NDK_ipv6mcb;

/**********************************************************************
 * Exported API (KERNEL MODE):
 *  These functions are exported by the IPv6 Module and are available
 *  for internal NDK core stack usage only.
 ***********************************************************************/
extern void IPv6Init (void);
extern void IPv6ParseExtnHeaders (PBM_Pkt* pPkt, IPV6HDR* ptr_ipv6hdr);
extern int IPv6ParseFragHdr (PBM_Pkt* pPkt, IPV6HDR* ptr_ipv6hdr);
extern void IPv6RxPacket (PBM_Pkt *pPkt);
extern void *IPv6GetRoute (IP6N DstIP);
extern int IPv6TxPacket (PBM_Pkt *pPkt, uint32_t Flags);

/**********************************************************************
 * Exported API (KERNEL MODE SAFE):
 *  These functions are exported to System Developers and can be called
 *  outside of kernel context.
 ***********************************************************************/
extern int    IPv6InterfaceInit (uint16_t index, void (*DADStatus)(IP6N Address, uint16_t index, unsigned char Status));
extern int    IPv6InterfaceDeInit (uint16_t dev_index);
extern int    IPv6AddAddress (uint16_t dev_index, IP6N Address,uint16_t NetBits, uint32_t VLT, uint32_t PLT,unsigned char IsAny);
extern int    IPv6DelAddress (uint16_t dev_index, IP6N Address);

/* Utility Functions: For IPv6 Address Managements etc. */
extern void   IPv6GetSubnetMaskFromBits (IP6N* SubnetMask, uint16_t bits);
extern void   IPv6DisplayIPAddress (IP6N address);
extern int    IPv6StringToIPAddress (char* StringIP, IP6N* address);
extern void   IPv6IPAddressToString (IP6N address, char* strIPAddress);
extern uint16_t IPv6CompareAddress (IP6N addr1, IP6N addr2);
extern uint16_t IPv6IsMulticast (IP6N address);
extern uint16_t IPv6IsLinkLocal (IP6N address);

/* IPv6 Checksum Calculation APIs */
extern uint32_t IPv6Layer4PseudoHdrChecksum(PSEUDOV6* ptr_pseudoHdr);
extern uint16_t IPv6Layer4ComputeChecksum (unsigned char *ptr_l4Hdr, PSEUDOV6* ptr_pseudoHdr);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* _C_IPV6IF_INC */
