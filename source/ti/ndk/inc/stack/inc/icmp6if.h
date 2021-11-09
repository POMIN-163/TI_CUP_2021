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
 * ======== icmp6if.h ========
 *
 * Common structures and definitions for ICMPv6
 *
 */


#ifndef _C_ICMPV6IF_INC
#define _C_ICMPV6IF_INC  /* #defined if this .h file has been included */

#ifdef __cplusplus
extern "C" {
#endif

/* Message Identifiers used by the ICMPv6 Module. */
#define MSG_ICMPV6_NEEDTIMER        (ID_ICMPV6*MSG_BLOCK + 0)
#define MSG_ICMPV6_TIMER            (ID_ICMPV6*MSG_BLOCK + 1)

/*---------------------------------------------------- */
/* ICMPv6 Type Definitions. */
#define ICMPV6_DST_UNREACHABLE          1
#define ICMPV6_PACKET_TOO_BIG           2
#define ICMPV6_TIME_EXCEEDED            3
#define ICMPV6_PARAMETER_PROBLEM        4
#define ICMPV6_ECHO_REQUEST             128
#define ICMPV6_ECHO_REPLY               129
#define ICMPV6_ROUTER_SOLICITATION      133
#define ICMPV6_ROUTER_ADVERTISMENT      134
#define ICMPV6_NEIGH_SOLICIT            135
#define ICMPV6_NEIGH_ADVERTISMENT       136
#define ICMPV6_REDIRECT                 137

/*---------------------------------------------------- */
/* ICMPv6 RS HEADER */
typedef struct {
                unsigned char  Type;
                unsigned char  Code;
                uint16_t Checksum;
                uint32_t Reserved;
               } ICMPV6_RS_HDR;

/*---------------------------------------------------- */
/* ICMPv6 RA Flags Definiton */
#define ICMPV6_RA_M_FLAG        0x80
#define ICMPV6_RA_O_FLAG        0x40

/*---------------------------------------------------- */
/* ICMPv6 RA HEADER */
typedef struct {
                unsigned char  Type;
                unsigned char  Code;
                uint16_t Checksum;
                unsigned char  CurHopLimit;
                unsigned char  Flags;
                uint16_t RouterLifetime;
                unsigned char  ReachableTime[4];
                unsigned char  RetransTime[4];
               } ICMPV6_RA_HDR;

/*---------------------------------------------------- */
/* ICMPv6 NS HEADER */
typedef struct {
                unsigned char  Type;
                unsigned char  Code;
                uint16_t Checksum;
                uint32_t Reserved;
                IP6N     TargetAddress;
               } ICMPV6_NS_HDR;

/*---------------------------------------------------- */
/* ICMPv6 REDIRECT HEADER */
typedef struct {
                unsigned char  Type;
                unsigned char  Code;
                uint16_t Checksum;
                uint32_t Reserved;
                IP6N     TargetAddress;
                IP6N     DestinationAddress;
               } ICMPV6_REDIRECT_HDR;

/*---------------------------------------------------- */
/* ICMPv6 NA HEADER Specific Definitions */

#define ICMPV6_NA_R_FLAG    0x80
#define ICMPV6_NA_S_FLAG    0x40
#define ICMPV6_NA_O_FLAG    0x20

/*---------------------------------------------------- */
/* ICMPv6 NA HEADER */
typedef struct {
                unsigned char  Type;
                unsigned char  Code;
                uint16_t Checksum;
                unsigned char  Flags;
                unsigned char  Reserved[3];
                IP6N     TargetAddress;
               } ICMPV6_NA_HDR;

/*---------------------------------------------------- */
/* ICMPv6 Echo Request/Reply HEADER */
typedef struct {
                unsigned char  Type;
                unsigned char  Code;
                uint16_t Checksum;
                uint16_t Identifier;
                uint16_t SequenceNum;
               } ICMPV6_ECHO_HDR;

/*---------------------------------------------------- */
/* ICMPv6 Time Exceeded Codes */
#define ICMPV6_TIME_EXCEEDED_HOP_LIMIT  0
#define ICMPV6_TIME_EXCEEDED_FRAGMENT   1

/*---------------------------------------------------- */
/* ICMPv6 Time Exceeded HEADER */
typedef struct {
                unsigned char  Type;
                unsigned char  Code;
                uint16_t Checksum;
                uint32_t Unused;
               } ICMPV6_TIME_EXCEEDED_HDR;

/*---------------------------------------------------- */
/* ICMPv6 Parameter Problem Codes */
#define ICMPV6_PARAM_PROBLEM_ERR_HEADER     0
#define ICMPV6_PARAM_PROBLEM_INV_NEXT_HDR   1
#define ICMPV6_PARAM_PROBLEM_INV_OPTION     2

/*---------------------------------------------------- */
/* ICMPv6 Parameter Problem HEADER */
typedef struct {
                unsigned char  Type;
                unsigned char  Code;
                uint16_t Checksum;
                uint32_t Pointer;
               } ICMPV6_PARAM_PROB_HDR;

/*---------------------------------------------------- */
/* ICMPv6 Destination Unreachable Codes */
#define ICMPV6_DST_UNREACH_NO_ROUTE         0
#define ICMPV6_DST_UNREACH_ADMIN            1
#define ICMPV6_DST_UNREACH_UNUSED           2
#define ICMPV6_DST_UNREACH_ADDRESS          3
#define ICMPV6_DST_UNREACH_PORT             4

/*---------------------------------------------------- */
/* ICMPv6 Destination Unreachable Message */
typedef struct {
                unsigned char  Type;
                unsigned char  Code;
                uint16_t Checksum;
                uint32_t Unused;
               } ICMPV6_DST_UNREACHABLE_HDR;

/*---------------------------------------------------- */
/* ICMPv6 Packet Too Big HEADER */
typedef struct {
                unsigned char  Type;
                unsigned char  Code;
                uint16_t Checksum;
                uint32_t mtu;
               } ICMPV6_PKT_TOO_BIG_HDR;

/*---------------------------------------------------- */
/* ICMPv6 Option Types */
#define ICMPV6_SOURCE_LL_ADDRESS        0x1
#define ICMPV6_TARGET_LL_ADDRESS        0x2
#define ICMPV6_PREFIX_INFORMATION       0x3
#define ICMPV6_REDIRECTED_HEADER        0x4
#define ICMPV6_MTU                      0x5

/*---------------------------------------------------- */
/* ICMPv6 Options Format */
typedef struct {
                unsigned char  Type;
                unsigned char  Length;
               } ICMPV6_OPT;

/*---------------------------------------------------- */
/* ICMPv6 Prefix Option Flag Definitions */
#define ICMPV6_PREFIX_OPT_L_FLAG       0x80
#define ICMPV6_PREFIX_OPT_A_FLAG       0x40

/*---------------------------------------------------- */
/* ICMPv6 Prefix Information Option Format */
typedef struct {
                unsigned char  Type;
                unsigned char  Length;
                unsigned char  PrefixLength;
                unsigned char  Flags;
                unsigned char  ValidLifetime[4];
                unsigned char  PreferredLifetime[4];
                unsigned char  Reserved[4];
                IP6N     Prefix;
               } ICMPV6_PREFIX_OPT;

/*---------------------------------------------------- */
/* ICMPv6 Redirected Header Option Format */
typedef struct {
                unsigned char  Type;
                unsigned char  Length;
                uint16_t Reserved1;
                uint32_t Reserved2;
                unsigned char  data[1];
               }ICMPV6_REDIRECTED_OPT;

/**
 * @brief
 *  The structure describes the ICMPv6 Statistics Block.
 *
 * @details
 *  This structure is used to hold ICMPv6 stats. This
 *  structure is defined as per MIBs described for ICMP
 *  in RFC 4293.
 */
typedef struct {

    /**
     * @brief   Packets received by ICMPv6 module.
     */
    uint32_t  InMsgs;

    /**
     * @brief   Packets droppped because of checksum error or
     * message validation error.
     */
    uint32_t  InErrors;

    /**
     * @brief   Packets transmitted from ICMPv6.
     */
    uint32_t  OutMsgs;

    /**
     * @brief   Packets droppped because of any errors.
     */
    uint32_t  OutErrors;

    /**
     * @brief   Number of DAD Successes.
     */
    uint32_t  DADSuccess;

    /**
     * @brief   Number of DAD Failures.
     */
    uint32_t  DADFailures;

} ICMPV6STATS;

/* Global IPv6 Statistics. */
extern ICMPV6STATS     NDK_icmp6stats;

/**********************************************************************
 * Exported API (KERNEL MODE):
 *  These functions are exported by the ICMP6 Module and are available
 *  for internal NDK core stack usage only.
 ***********************************************************************/
extern void ICMPv6Init (void);
extern void ICMPV6Msg (uint32_t Msg);
extern int ICMPv6RxPacket (PBM_Pkt* pPkt, IPV6HDR* ptr_ipv6hdr);
extern int ICMPv6RecvNS (PBM_Pkt* pPkt, IPV6HDR* ptr_ipv6hdr);
extern int ICMPv6RecvNA (PBM_Pkt* pPkt, IPV6HDR* ptr_ipv6hdr);
extern int ICMPv6RecvRA (PBM_Pkt* pPkt, IPV6HDR* ptr_ipv6hdr);
extern int ICMPv6RecvRedirect (PBM_Pkt* pPkt, IPV6HDR* ptr_ipv6hdr);
extern int ICMPv6RecvRS (PBM_Pkt* pPkt, IPV6HDR* ptr_ipv6hdr);
extern int ICMPv6SendDstUnreachable (unsigned char Code, PBM_Pkt* pOrgPkt);
extern int ICMPv6SendTimeExceeded (unsigned char Code, PBM_Pkt* pOrgPkt);
extern int ICMPv6SendParameterProblem (unsigned char Code, PBM_Pkt* pOrgPkt, uint32_t Pointer);
extern int ICMPv6SendNS (NETIF_DEVICE* ptr_device, IP6N DstAddress, IP6N SrcAddress, IP6N TargetAddress);
extern int ICMPv6SendNA (NETIF_DEVICE* ptr_device, IP6N DstAddress, IP6N SrcAddress, IP6N TargetAddress, unsigned char Flags);
extern int ICMPv6SendRS (NETIF_DEVICE* ptr_device, IP6N SrcAddress);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* _C_ICMPV6IF_INC */
