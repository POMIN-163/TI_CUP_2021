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
 * ======== natif.h ========
 *
 */

#ifndef _C_NATIF_INC
#define _C_NATIF_INC  /* #defined if this .h file has been included */

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------------------------------------------- */
/* Global Task Information */
#define NAT_MAXENTRIES                  2048

/* Defined Messages */
#define MSG_NAT_TIMER                   (ID_NAT*MSG_BLOCK + 0)

/* NAT Statistics */
typedef struct {
    uint32_t    TxExamined;     /* Number of packets examined */
    uint32_t    TxQualified;    /* Number of possible packets */
    uint32_t    TxAltered;      /* Number of packets altered */
    uint32_t    RxExamined;     /* Number of packets examined */
    uint32_t    RxQualified;    /* Number of possible packets */
    uint32_t    RxAltered;      /* Number of packets altered */
    uint32_t    Entries;        /* Number of translation entries */
    uint32_t    MaxEntries;     /* Max number of translation entries */
    uint32_t    LongTerm;       /* Entries with extended timeouts */
    uint32_t    MaxLongTerm;    /* Max entries with extended timeouts */
    } NATSTATS;

/* NAT Statistics */
extern NATSTATS NDK_nats;

/* NAT "Shared" IP Server */
extern uint32_t NatIpServer;

/* Time a NAT entry can be idle */
#define NAT_IDLE_SECONDS                60              /* Non-Tcp tranlation */
#define NAT_TCP_IDLE_SECONDS            60              /* Unconnected socket */
#define NAT_TCP_SYN_SECONDS             60              /* Connecting socket */
#define NAT_TCP_ACTIVE_SECONDS          (3600*24*5)     /* Connected socket */
#define NAT_TCP_CLOSED_SECONDS          5               /* Closed socket */

/* NAT Access Functions called by IP */
int  NatIpTxInput( PBM_Pkt *pPkt );
int  NatIpRxInput( PBM_Pkt *pPkt );

/* NAT Access Functions called by USER */
void NatSetConfig( uint32_t IPAddr, uint32_t IPMask, uint32_t IPServer, uint32_t MTU );

/*------------------------------------------------------------------------- */
/* NAT Info Entry Structure */
typedef struct _natinfo {
    uint32_t            TcpState;       /* Current TCP State (Simplified) */
#define NI_TCP_IDLE         0               /* Closed or closing */
#define NI_TCP_SYNSENT_TX   1               /* Connecing */
#define NI_TCP_SYNSENT_RX   2               /* Connecing */
#define NI_TCP_SYNSENT_ALL  3               /* Both SYN sent - waiting ACK */
#define NI_TCP_ESTAB        4               /* Established */
#define NI_TCP_CLOSING_TX   5               /* Closing */
#define NI_TCP_CLOSING_RX   6               /* Closing */
#define NI_TCP_CLOSED       7               /* Closed */
    uint32_t            IPLocal;        /* Translated IP Address */
    uint16_t            PortLocal;      /* Translated TCP/UDP Port */
    uint32_t            IPForeign;      /* IP Adress of Foreign Peer */
    uint16_t            PortForeign;    /* Port of Foreign Peer */
    unsigned char             Protocol;       /* IP Potocol */
    uint16_t            PortMapped;     /* Locally Mapped TCP/UDP Port (router) */
    void                *hProxyEntry;    /* Handle to Proxy Entry (if any) */
    uint32_t            Timeout;        /* Expiration time in SECONDS */
    void                *pUserData;     /* Pointer to proxy callback data */
    } NATINFO;

/*-------------------------------------------------------------------- */
/* Nat Entry Routines */
/*-------------------------------------------------------------------- */
extern void *NatNew( uint32_t IPLocal, uint16_t PortLocal,
                        uint32_t IPForeign, uint16_t PortForeign,
                        unsigned char Protocol, uint16_t PortMapped,
                        uint32_t Timeout );
extern void     NatFree( void *hNat );
extern NATINFO *NatGetPNI( void *hNat );

/*-------------------------------------------------------------------- */
/* Proxy Routines */
/*-------------------------------------------------------------------- */

/* Mode used by Nat and Proxy */
#define NAT_MODE_TX           0
#define NAT_MODE_RX           1

/* Proxy Access Functions called by USER */
extern void *ProxyNew( uint32_t NatMode, unsigned char Protocol, uint16_t Port,
                        uint32_t IPTarget,
                        int (*pfnEnableCb)( NATINFO *, uint32_t ),
                        int (*pfnTxCb)( NATINFO *, IPHDR * ),
                        int (*pfnRxCb)( NATINFO *, IPHDR * ) );
extern void   ProxyFree( void *hProxy );
extern IPHDR  *ProxyPacketMod( uint32_t Offset, uint32_t OldSize,
                               uint32_t NewSize, unsigned char *pData );

/* Proxy Access Functions called by NAT */
extern int    ProxyEntrySpawn( uint32_t NatMode, unsigned char Protocol, uint16_t Port,
                               void **phProxyEntry, uint32_t *pIPLocal );
#define       ProxyEntryFree(x) mmFree(x)
extern int    ProxyEnable( NATINFO *pni, uint32_t Enable );
extern void   ProxyTx( NATINFO *pni, PBM_Pkt *pPkt, IPHDR *pIpHdr );
extern void   ProxyRx( NATINFO *pni, PBM_Pkt *pPkt, IPHDR *pIpHdr );

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
