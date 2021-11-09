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
 * ======== dns.h ========
 *
 * Basic DNS Resolution Routine Private Include
 *
 */

#ifndef _DNS_H_
#define _DNS_H_

#include <netmain.h>
#include <_stack.h>

#ifdef __cplusplus
extern "C" {
#endif

/* General Equates */
#define DNS_PORT                53        /* DNS UDP (and TCP) Port */
#define DNS_NAME_MAX            128       /* Max name we'll ever handle */
#define DNS_PACKET_MAX          512       /* Max DNS packet */
#define DNS_DEFAULT_TTL         (3600*4)  /* 4 Hours */
#define MAX_PACKET              1000      /* Packet buffer size we use */

/* Special return code from DnsCheckReply() */
#define DNSCODE_RETRY          -1

/* DNS Record */
typedef struct _dnsrec {
        struct      _dnsrec *pNext;     /* Next Record */
        unsigned char     Name[DNS_NAME_MAX]; /* Domain/Query Name */
        uint16_t    Type;               /* Record Type */
        uint16_t    Class;              /* Record Class */
        uint32_t    Ttl;                /* Time to live */
        uint16_t    DataLength;         /* Valid Data Length */
        unsigned char     Data[DNS_NAME_MAX]; /* Data */
        } DNSREC;

/* DNS Reply Record */
typedef struct _dnsreply {
        uint16_t    Flags;              /* DNS reply code / RA / AA */
        uint16_t    NumAns;             /* Number of answers */
        DNSREC      *pAns;              /* Chain of answers */
        uint16_t    NumAuth;            /* Number of authorities */
        DNSREC      *pAuth;             /* Chain of authorities */
        uint16_t    NumAux;             /* Number of additional records */
        DNSREC      *pAux;              /* Chain of additional records */
        } DNSREPLY;

/* DNS Packet */
typedef struct {
        uint16_t    Id;                 /* Identification (from client) */
        uint16_t    Flags;              /* Type Flags */
#define FLG_DNS_QR      0x8000          /* 0 - Query  1 - Response */
#define MASK_DNS_OP     0x7800          /* Opcode */
#define DNS_OP_STD      0x0000          /* Standard Query        (0) */
#define DNS_OP_INV      0x0800          /* Inverse Query         (1) */
#define DNS_OP_STATUS   0x1000          /* Server Status Request (2) */
#define FLG_DNS_AA      0x0400          /* Authorative answer 1 = Yes */
#define FLG_DNS_TC      0x0200          /* Truncated (reply larger than 512) */
#define FLG_DNS_RD      0x0100          /* Recursion Desired */
#define FLG_DNS_RA      0x0080          /* Recursion Available */
#define MASK_DNS_RES    0x0070          /* Reserved Bits */
#define MASK_DNS_RCODE  0x000F          /* Return Code */
        uint16_t    NumQ;               /* Number of Queries */
        uint16_t    NumA;               /* Number of Answers */
        uint16_t    NumAuth;            /* Number of Authorities */
        uint16_t    NumAux;             /* Number of Additional records */
        unsigned char     Data[500];          /* Queries and Replies */
        } DNSHDR;

/* Resource Record Equates (that we use) */

/* 16 bit pointer detect */
#define RR_PTR          0xc0            /* When 2 MSBs set, cnt is a 16 bit ptr */

/* Record Class */
#define C_IN            1               /* Internet */

/* Record Types */
#define T_A             1               /* host address */
#define T_NS            2               /* authoritative name server */
#define T_CNAME         5               /* canonical name for an alias */
#define T_PTR           12              /* domain name pointer */
#define T_HINFO         13              /* host information */
/* New Type for IPv6 Host Address */
#define T_AAAA          28              /* IPv6 Host Address (RFC 3596) */

/* Shared Functions for Server and Client */
extern int  DNSResolveQuery(uint16_t ReqType, DNSREC *pQuery, DNSREPLY **ppReply);
extern void DNSReplyFree( DNSREPLY *pReply, uint32_t fFreeReplyBuffer );

int DNSGetQuery( DNSHDR *pDNS, DNSREC *pQuery );
int DNSGetReply( DNSHDR *pDNS, uint16_t Id, int cc, DNSREPLY *pReply );
int DNSBuildRequest( DNSHDR *pDNS, uint16_t Id, DNSREC *pQuery );
int DNSBuildReply( DNSHDR *pDNS, uint16_t Id, DNSREC *pQ, DNSREPLY *pA );

/* Resource Records are unsigned char aligned */
/* These MACROS assume "p" is an unsigned char pointer */
#define DNSREAD16(s, p) { \
        (s) = ((uint16_t)*(p) << 8) | ((uint16_t)*((p)+1)); \
        (p) += 2; }

#define DNSREAD32(l, p) { \
        (l) = ((uint32_t)*(p) << 24) | ((uint32_t)*((p)+1) << 16) | \
              ((uint32_t)*((p)+2) << 8) | ((uint32_t)*((p)+3)); \
        (p) += 4; }

#define DNSWRITE16(s, p) { \
        *(p)++ = (unsigned char)((s)>>8); \
        *(p)++ = (unsigned char)(s); };

#define DNSWRITE32(l, p) { \
        *(p)++ = (unsigned char)((l)>>24); \
        *(p)++ = (unsigned char)((l)>>16); \
        *(p)++ = (unsigned char)((l)>>8); \
        *(p)++ = (unsigned char)(l); };

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* _DNS_H_ */
