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
 * ======== netdf.h ========
 *
 * Basic network data formats
 *
 */

#ifndef _C_NETDF_INC
#define _C_NETDF_INC  /* #defined if this .h file has been included */

#ifdef __cplusplus
extern "C" {
#endif

/*---------------- */
/* Ethernet Header */
#define ETHHDR_SIZE     14

typedef struct {
                unsigned char DstMac[6];
                unsigned char SrcMac[6];
                uint16_t  Type;
               } ETHHDR;

/*---------------- */
/* VLAN Header */
#define VLANHDR_SIZE     4

typedef struct {
                uint16_t  TCI;
                uint16_t  EncapProtocol;
               } VLANHDR;

/*----------------------------------------------------------------------- */
/* Ethernet ARP Protocol Header */

#define ARPHDR_SIZE     28

typedef struct {
                uint16_t HardType;
                uint16_t ProtocolType;
                unsigned char  HardSize;
                unsigned char  ProtocolSize;
                uint16_t Op;
                unsigned char  SrcAddr[6];
                unsigned char  IPSrc[4];
                unsigned char  DstAddr[6];
                unsigned char  IPDst[4];
               } ARPHDR;

/*---------------------------------------------------- */
/* IP HEADER */

#define IPHDR_SIZE      20

typedef struct {
                unsigned char  VerLen;
                unsigned char  Tos;
                uint16_t TotalLen;
                uint16_t Id;
                uint16_t FlagOff;
                unsigned char  Ttl;
                unsigned char  Protocol;
                uint16_t Checksum;
                uint32_t IPSrc;
                uint32_t IPDst;
                unsigned char  Options[1];
               } IPHDR;

/*---------------------------------------------------- */
/* IPv6 HEADER */

#define IPv6HDR_SIZE      40

typedef struct {
                unsigned char  VerTC;
                unsigned char  FlowLabel[3];
                uint16_t PayloadLength;
                unsigned char  NextHeader;
                unsigned char  HopLimit;
                IP6N     SrcAddr;
                IP6N     DstAddr;
               } IPV6HDR;

/*---------------------------------------------------- */
/* ICMP HEADER */

#define ICMPHDR_SIZE    4

typedef struct {
                unsigned char  Type;
                unsigned char  Code;
                uint16_t Checksum;
                unsigned char  Data[1];
               } ICMPHDR;

#define ICMPREQHDR_SIZE 4

typedef struct {
                uint16_t  Id;
                uint16_t  Seq;
                unsigned char   Data[1];
               } ICMPREQHDR;

#define ICMPRTAHDR_SIZE 12

typedef struct {
                unsigned char  NumAddr;
                unsigned char  Size;
                uint16_t Lifetime;
                struct _rta {
                             uint32_t IPAddr;
                             int32_t Pref;
                        } rta[1];
               } ICMPRTAHDR;

/*---------------------------------------------------- */
/* ICMPv6 HEADER */

typedef struct {
                unsigned char  Type;
                unsigned char  Code;
                uint16_t Checksum;
               } ICMPV6HDR;

/*---------------------------------------------------- */
/* IGMP HEADER */

#define IGMPHDR_SIZE    8

typedef struct {
                unsigned char  VerType;
                unsigned char  MaxTime;
                uint16_t Checksum;
                uint32_t IpAddr;
               } IGMPHDR;


/*---------------------------------------------------- */
/* TCP HEADER */

#define TCPHDR_SIZE     20

typedef struct {
                uint16_t SrcPort;
                uint16_t DstPort;
                uint32_t Seq;
                uint32_t Ack;
                unsigned char  HdrLen;
                unsigned char  Flags;
                uint16_t WindowSize;
                uint16_t TCPChecksum;
                uint16_t UrgPtr;
                unsigned char  Options[1];
               } TCPHDR;

/*---------------------------------------------------- */
/* UDP HEADER */
#define UDPHDR_SIZE     8

typedef struct {
                uint16_t SrcPort;
                uint16_t DstPort;
                uint16_t Length;
                uint16_t UDPChecksum;
               } UDPHDR;

/* Pseudo Header for Checksum */
typedef struct {
                uint32_t IPSrc;
                uint32_t IPDst;
                uint16_t  Length;
                unsigned char Null;
                unsigned char Protocol;
                } PSEUDO;

/*---------------------------------------------------- */
/* V6 PSEUDO HEADER */

typedef struct {
                IP6N     SrcAddr;
                IP6N     DstAddr;
                uint32_t PktLen;
                unsigned char  Rsvd[3];
                unsigned char  NxtHdr;
               } PSEUDOV6;

/*---------------------------------------------------- */
/* IPV6 Hop-by-Hop Options HEADER */

#define IPV6_HOPOPTSHDR_SIZE      2

typedef struct {
                unsigned char  NextHeader;
                unsigned char  HdrExtLen;
               } IPV6_HOPOPTSHDR;

/*---------------------------------------------------- */
/* IPV6 Routing HEADER */

#define IPV6_ROUTINGHDR_SIZE      4

typedef struct {
                unsigned char  NextHeader;
                unsigned char  HdrExtLen;
                unsigned char  RoutingType;
                unsigned char  SegmentsLeft;
               } IPV6_ROUTINGHDR;

/*---------------------------------------------------- */
/* IPV6 Routing HEADER Type 0 */

#define IPV6_RTGHDR_TYPE0_SIZE      8

typedef struct {
                unsigned char  NextHeader;
                unsigned char  HdrExtLen;
                unsigned char  RoutingType;
                unsigned char  SegmentsLeft;
                unsigned char  Rsvd[4];
               } IPV6_RTGHDR_TYPE0;

/*---------------------------------------------------- */
/* IPV6 Fragmentation HEADER */

#define IPV6_FRAGHDR_SIZE      8

typedef struct {
                unsigned char  NextHeader;
                unsigned char  Rsvd;
                uint16_t FragOffset;
                unsigned char  FragId[4];
               } IPV6_FRAGHDR;

/*---------------------------------------------------- */
/* IPV6 Destination Options HEADER */

#define IPV6_DSTOPTSHDR_SIZE      2

typedef struct {
                unsigned char  NextHeader;
                unsigned char  HdrExtLen;
               } IPV6_DSTOPTSHDR;

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* _C_NETDF_INC */

