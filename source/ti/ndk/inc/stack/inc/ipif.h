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
 * ======== ipif.h ========
 *
 */


#ifndef _C_IPIF_INC
#define _C_IPIF_INC  /* #defined if this .h file has been included */

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------------------------------------------- */
/* Global Task Information */

/* Defined Messages */
#define MSG_IP_TIMER                    (ID_IP*MSG_BLOCK + 0)

/*----------------------------------------------------------------------- */
/* Standard IP Equates */

/* IP FRAGMENTATION BITS */
#define IP_DF   0x4000                  /* Ip DON'T FRAGMENT Bit */
#define IP_MF   0x2000                  /* Ip MORE FRAGMENTS Bit */

/* IP Statistics */
typedef struct {
        uint32_t  Total;          /* total packets received */
        uint32_t  Odropped;       /* lost packets due to nobufs, etc. */
        uint32_t  Badsum;         /* checksum bad */
        uint32_t  Badhlen;        /* ip header length < data size */
        uint32_t  Badlen;         /* ip length < ip header length */
        uint32_t  Badoptions;     /* error in option processing */
        uint32_t  Badvers;        /* ip version != 4 */
        uint32_t  Forward;        /* packets forwarded */
        uint32_t  Noproto;        /* unknown or unsupported protocol */
        uint32_t  Delivered;      /* datagrams delivered to upper level */
        uint32_t  Cantforward;    /* packets rcvd for unreachable dest */
        uint32_t  CantforwardBA;  /* packets rcvd with illegal addressing */
        uint32_t  Expired;        /* Packets not forwards becaused expired */
        uint32_t  Redirectsent;   /* packets forwarded on same net */
        uint32_t  Localout;       /* total ip packets generated here */
        uint32_t  Localnoroute;   /* Local packets discarded due to no route */
        uint32_t  Reassembled;    /* total packets reassembled ok */
        uint32_t  Fragments;      /* fragments received */
        uint32_t  Fragdropped;    /* frags dropped (dups, out of space) */
        uint32_t  Fragtimeout;    /* fragments timed out */
        uint32_t  Fragmented;     /* datagrams successfully fragmented */
        uint32_t  Ofragments;     /* output fragments created */
        uint32_t  Cantfrag;       /* don't fragment flag was set, etc. */
        uint32_t  CacheHit;       /* Route cache hit */
        uint32_t  CacheMiss;      /* Route cache miss */
        uint32_t  Filtered;       /* packets filtered by firewall */
        } IPSTATS;

/* IP OPTIONS FLAGS */
#define IPOPT_EOL       0               /* End of Options */
#define IPOPT_NOP       1               /* NOP */
#define IPOPT_RR        7               /* Record Route */
#define IPOPT_TS        68              /* timestamp */
#define IPOPT_SECURITY  130             /*/provide s,c,h,tcc */
#define IPOPT_LSRR      131             /* Loose Source Record Route */
#define IPOPT_SATID     136             /* satnet id */
#define IPOPT_SSRR      137             /* Strict Source Record Route */
#define IPOPT_RA        148             /* router alert */

/* Offsets to fields in options other than EOL and NOP. */
#define IPOPT_OPTVAL        0       /* option ID */
#define IPOPT_OLEN      1       /* option length */
#define IPOPT_OFFSET        2       /* offset within option */
#define IPOPT_MINOFF        4       /* min value of above */

/* Global Statistics */
extern IPSTATS NDK_ips;

/* Access Functions */
extern int    IPTxPacket( PBM_Pkt *pPkt, uint32_t Flags );
extern void   IPRxPacket( PBM_Pkt *pPkt );
extern void   IPChecksum( IPHDR *pbHdr );
extern void *IPGetRoute( uint32_t RtCallFlags, uint32_t IPDst );
extern void   IPRtChange( void *hRt );
extern bool   IPIsLoopback( uint32_t IPDst );
extern void   IPReasm( PBM_Pkt *pPkt );
extern void   IPFilterSet( uint32_t IPAddr, uint32_t IPMask );

/* IP Tx Parameters */

/* --[ We asume that all SO_ supported flags are above 0xF ]---------- */
#if ((0xF & (SO_BROADCAST | SO_DONTROUTE)) != 0)
#error Flag Conflict with SOCKET.H
#endif
/* ------------------------------------------------------------------- */

/* Flags Parameter Values */
#define FLG_IPTX_FORWARDING     0x0001       /* Forwarding packet from IPRx */
#define FLG_IPTX_DONTFRAG       0x0002       /* Set the IP_DF bit */
#define FLG_IPTX_RAW            0x0004       /* All IP header is valid */
#define FLG_IPTX_SRCROUTE       0x0008       /* Packet was source routed */
#define FLG_IPTX_DONTROUTE      SO_DONTROUTE /* Send only to local subnets */
#define FLG_IPTX_BROADCAST      SO_BROADCAST /* Allow broadcast packets */
#define FLG_IPTX_SOSUPPORTED    (SO_DONTROUTE|SO_BROADCAST)

/* Return Values */
#define IPTX_SUCCESS            0               /* Packet OK */
#define IPTX_ERROR              NDK_EINVAL          /* Invalid packet */
#define IPTX_ERROR_UNREACH      NDK_EHOSTUNREACH    /* Route not found */
#define IPTX_ERROR_HOSTDOWN     NDK_EHOSTDOWN       /* Route not UP */
#define IPTX_ERROR_REJECTED     NDK_EHOSTDOWN       /* REJECT Route */
#define IPTX_ERROR_EACCES       NDK_EACCES          /* Illegal operation */
#define IPTX_ERROR_ADDRNOTAVAIL NDK_EADDRNOTAVAIL   /* Could not alloc MAC addr */
#define IPTX_ERROR_MSGSIZE      NDK_EMSGSIZE        /* Fragmentation failed */

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif

