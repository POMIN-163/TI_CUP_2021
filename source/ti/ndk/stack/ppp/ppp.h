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
 * ======== ppp.h ========
 *
 * Basic includes for PPP
 *
 */

#ifndef _PPP_H
#define _PPP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Special Read MACRO (is processor independent) */
#define RdNet16s(x)  ((uint16_t)((*(unsigned char *)(x)))<<8 | (uint16_t)(*(((unsigned char *)(x))+1)))

/*----------------------------------------------- */
/* Network Headers */
/*----------------------------------------------- */

/*-------------------- */
/* LCP Protocol Header */
typedef struct {
                unsigned char Code;
                unsigned char Id;
                uint16_t  Length;
                unsigned char TagData[];
               } LCPHDR;
#define SIZE_LCPHDR     4

/*----------------------------------------------- */
/* PPP Protocols */
#define PPPPROT_IP      0x0021
#define PPPPROT_IPCP    0x8021
#define PPPPROT_LCP     0xc021
#define PPPPROT_PAP     0xc023
#define PPPPROT_CHAP    0xc223

/*----------------------------------------------- */
/* State Codes for LCP and IPCP */
#define PROT_STATE_CLOSED       0       /* Layer Closed */
#define PROT_STATE_OPEN         1       /* Layer Open */
#define PROT_STATE_CONNECTED    2       /* Negotiation completed */
#define PROT_STATE_STOPPED      3       /* Layer Stopped */

/*----------------------------------------------- */
/* State Code for each half of a LCP/IPCP negotiation */
#define PROT_CFG_IDLE           0       /* No connection */
#define PROT_CFG_PENDING        1       /* CFG pending */
#define PROT_CFG_AUTH           2       /* CFG authentication */
#define PROT_CFG_OK             3       /* CFG established */

/*----------------------------------------------- */
/* LCP Instance Structure */
typedef struct {
    uint32_t    State;       /* State of LCP session */
    uint32_t    StateCFG;    /* LCP CFG State machine */
    uint32_t    StateACK;    /* LCP ACK State machine */
    unsigned char     LastId;      /* Id of last CFG sent */
    int         Timer;       /* Restart timer */
    int         Count;       /* Restart event count */
    uint32_t    PeerMagic;   /* Peer's Magic Number */
    uint32_t    OurMagic;    /* Our Magic Number */
    uint32_t    OptMask;     /* Available Options */
    uint32_t    UseMask;     /* Options in use */
    } LCP_SESSION;

/*----------------------------------------------- */
/* AUTH Instance Structure */
typedef struct {
    uint32_t    State;       /* State of LCP session */
    unsigned char     LastId;      /* Id of last CFG sent */
    int         Timer;       /* Restart timer */
    int         Count;       /* Restart event count */
    uint16_t    Protocol;    /* Authentication protocol */
    unsigned char     SeedLen;     /* MD5 Data seed length */
    unsigned char     SeedData[32];/* MD5 Data seed data */
    char        AuthName[32];/* Authenticator Name */
    } AUTH_SESSION;

/*----------------------------------------------- */
/* IPCP Instance Structure */
typedef struct {
    uint32_t    State;       /* State of IPCP session */
    uint32_t    StateCFG;    /* IPCP CFG State machine */
    uint32_t    StateACK;    /* IPCP ACK State machine */
    unsigned char     LastId;      /* Id of last CFG sent */
    int         Timer;       /* Restart timer */
    int         Count;       /* Restart event count */
    uint32_t    OptMask;     /* Available Options */
    uint32_t    UseMask;     /* Options in use */
    } IPCP_SESSION;

/*----------------------------------------------- */
/* PPP Instance Structure */
typedef struct {
/*---[ These fields match IF structure ]--- */
    uint32_t     Type;             /* Set to HTYPE_PPP */
    uint32_t     llIndex;          /* Low-level Device Index */
    uint32_t     ProtMTU;          /* MTU of payload */
/*----------------------------------------- */
    uint32_t     InUse;            /* Non-standard Reference count */
#define INUSE_IDLE      1
#define INUSE_LOCKED    65535
    void      *hSI;              /* Handle to serial interface driver */
    void      *hRoute;           /* Route when connected */
    void      *hNet;             /* Handle to network address in CFG */
    void      *hClient;          /* Handle to client record in CFG */
    uint32_t     Flags;            /* Mode flags */
    uint32_t     CMap;             /* Our desired CMap */
    LCP_SESSION  lcp;              /* LCP Session */
    AUTH_SESSION auth;             /* AUTH Session */
    IPCP_SESSION ipcp;             /* LCP Session */
    uint32_t     MTU_Phys;         /* Physical MTU */
    uint32_t     MTU_Rx;           /* MTU of peer */
    uint32_t     MTU_Tx;           /* Our negotiated MTU */
    uint32_t     IPServer;         /* Server IP (us in server mode) */
    uint32_t     IPMask;           /* Server IP Mask (when in server mode) */
    uint32_t     IPClient;         /* Client IP (us in client mode) */
    uint32_t     IPDNS1;           /* DNS Server 1 */
    uint32_t     IPDNS2;           /* DNS Server 2 */
    uint32_t     IPNBNS1;          /* NBNS Server 1 */
    uint32_t     IPNBNS2;          /* NBNS Server 2 */
    char         UserId[PPPNAMELEN];    /* Username */
    char         Password[PPPNAMELEN];  /* Password */
    void (*SICtrl)( void *, uint32_t, uint32_t, PBM_Pkt * );
    NETIF_DEVICE*   ptr_ppp_device; /* The PPP NIMU Network Interface Object  */
    } PPP_SESSION;

/*----------------------------------------------- */
/* PPP Events */
#define PPP_EVENT_LCP_CONNECT           1
#define PPP_EVENT_LCP_STOPPED           2
#define PPP_EVENT_AUTH_CONNECT          3
#define PPP_EVENT_AUTH_STOPPED          4
#define PPP_EVENT_IPCP_CONNECT          5
#define PPP_EVENT_IPCP_STOPPED          6

/*----------------------------------------------- */
/* Private PPP Functions */
void   pppEvent( void *hPPP, uint32_t Event );

/*----------------------------------------------- */
/* Private LCP Functions */
void lcpInit( PPP_SESSION *p );                 /* Init LCP instance */
void lcpOpen( PPP_SESSION *p, uint32_t fStart );    /* Open and/or Start LCP */
void lcpClose( PPP_SESSION *p );                /* Stop the CFG process */
void lcpInput( PPP_SESSION *p, PBM_Pkt *pPkt ); /* Rx a Packet */
void lcpReject( PPP_SESSION *p, PBM_Pkt *pPkt );/* Reject bad protocol Packet */
void lcpTimer( PPP_SESSION *p );                /* Timer Tick */

/*----------------------------------------------- */
/* Private AUTH Functions */
void authInit( PPP_SESSION *p );                /* Init AUTH instance */
void authStart( PPP_SESSION *p );               /* Start AUTH */
void authTimer( PPP_SESSION *p );               /* Timer Tick */
void papInput( PPP_SESSION *p, PBM_Pkt *pPkt ); /* Rx a Packet */
void chapInput( PPP_SESSION *p, PBM_Pkt *pPkt );/* Rx a Packet */

/*----------------------------------------------- */
/* Private IPCP Functions */
void ipcpInit( PPP_SESSION *p );                /* Init IPCP instance */
void ipcpStart( PPP_SESSION *p );               /* Start IPCP */
void ipcpInput( PPP_SESSION *p, PBM_Pkt *pPkt );/* Rx a Packet */
void ipcpTimer( PPP_SESSION *p );               /* Timer Tick */

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* _PPP_H */
