/*
 * Copyright (c) 2012-2018, Texas Instruments Incorporated
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
 * ======== sockif.h ========
 *
 * Sock object public definitions
 *
 */

#include "fdtif.h"
#include "listlib.h"

#ifndef _SOCKIF_INC_
#define _SOCKIF_INC_

#ifdef __cplusplus
extern "C" {
#endif

/* Socket Ephemeral Port Range (Public and Reserved) */
#define SOCK_USERPORT_FIRST 0xE000      /* Local User Ports */
#define SOCK_USERPORT_LAST  0xEFFF
#define SOCK_RESPORT_FIRST  0xF000      /* Reserved NAT Ports */
#define SOCK_RESPORT_LAST   0xFFFE

/* Protocol Control Commands */
#define PRC_IFDOWN              0       /* interface transition */
#define PRC_ROUTEDEAD           1       /* select new route if possible ??? */
#define PRC_QUENCH2             3       /* DEC congestion bit says slow down */
#define PRC_QUENCH              4       /* some one said to slow down */
#define PRC_MSGSIZE             5       /* message size forced drop */
#define PRC_HOSTDEAD            6       /* host appears to be down */
#define PRC_HOSTUNREACH         7       /* deprecated (use PRC_UNREACH_HOST) */
#define PRC_UNREACH_NET         8       /* no route to network */
#define PRC_UNREACH_HOST        9       /* no route to host */
#define PRC_UNREACH_PROTOCOL    10      /* dst says bad protocol */
#define PRC_UNREACH_PORT        11      /* bad port # */
#define PRC_UNREACH_NEEDFRAG    12      /* (use PRC_MSGSIZE) */
#define PRC_UNREACH_SRCFAIL     13      /* source route failed */
#define PRC_REDIRECT_NET        14      /* net routing redirect */
#define PRC_REDIRECT_HOST       15      /* host routing redirect */
#define PRC_REDIRECT_TOSNET     16      /* redirect for type of service & net */
#define PRC_REDIRECT_TOSHOST    17      /* redirect for tos & host */
#define PRC_TIMXCEED_INTRANS    18      /* packet lifetime expired in transit */
#define PRC_TIMXCEED_REASS      19      /* lifetime expired on reass q */
#define PRC_PARAMPROB           20      /* header incorrect */
#define PRC_NCMDS               21

/* IP Socket Options (IpFlags) */
#define SOCK_IP_HDRINCL         0x0001  /* Include IP Header (raw only) */
#define SOCK_IP_OPTIONS         0x0002  /* Use supplied IP Options */

/* Multicast Socket Record Information:
 *  Multicast requests are now stored on per socket basis; instead of a per
 *  interface basis. This structure is a linked list of multicast records which
 *  are stored on each socket. Sockets could have joined multiple multicast
 *  groups. */
typedef struct _mcast_sock
{
    NDK_LIST_NODE           links;          /* List of multicast socket records */
    struct ip_mreq      mreq;           /* Multicast Request                */
    void                *hIf;            /* Handle to the interface          */
}MCAST_SOCK_REC;

typedef struct mcast_sock_rec6
{
    NDK_LIST_NODE           links;
    struct ipv6_mreq    mreq;
    void                *hIf;
} MCAST_SOCK_REC6;

/* Socket Priority
 *  Each socket can be associated with a specific priority. This priority
 *  can be configured through the setsockopt API. All packets transmitted
 *  through this socket will have the same priority. This is used by the
 *  VLAN drivers in the system to remark the packets with an appropriate
 *  Layer2 User Priority value.
 *  By default; there is no priority associated with the socket. */
#define PRIORITY_UNDEFINED 0xFFFF

#define SOCKPROT_NONE   0
#define SOCKPROT_TCP    1
#define SOCKPROT_UDP    2
#define SOCKPROT_RAW    3

/* Sock Object Structure */
typedef struct _sock {
             FILEDESC   fd;             /* File descriptor header */
             int32_t    Ctx;            /* Socket context */

             uint32_t   Family;         /* Address Family as AF_ in socket.h */
             uint32_t   SockType;       /* Type as SOCK_ in socket.h */
             uint32_t   Protocol;       /* IP Protocol: TCP, UDP, ICMP, etc. */

             uint32_t   IpFlags;        /* IP Protocol Options (raw only) */
             uint32_t   IpTtl;          /* IP TTL */
             uint32_t   IpTos;          /* IP TOS */
             uint32_t   IpOptSize;      /* IP Header Options Size */
             unsigned char    IpOptions[40];  /* IP Header Options */

             uint16_t   SockPriority;   /* Socket Priority. */
             uint32_t   SockProt;       /* Socket Protocol Handler */

             uint32_t   OptionFlags;    /* SO_ options as defined in socket.h */
             uint32_t   StateFlags;     /* SS_ flags as defined below */
             uint32_t   dwLingerTime;   /* Time used when SO_LINGER set */

             /* Protocol Control */
             struct _sock *pProtNext;   /* Pointer to next in protocol list */
             struct _sock *pProtPrev;   /* Pointer to prev in protocol list */
             uint32_t   FIP;            /* Foreign IP address */
             uint32_t   FPort;          /* Foreign Port */
             uint32_t   LIP;            /* Local IP address (NULL for wildcard) */
             uint32_t   LPort;          /* Local Port (NULL if not bound) */
             uint32_t   BIP;            /* Bound IP address (SS_ADDR only) */
             uint32_t   BPort;          /* Bound Port (SS_ADDR only) */
             void       *hTP;            /* Handle to protocol specific data */
             void       *hRoute;         /* Handle to cached route */
             void       *hIFTx;          /* Handle to Default IF for transmit */

             /* Connection State Stuff */
             struct _sock *pParent;     /* Pointer back to accept socket */
             struct _sock *pPrevQ;      /* Prev socket in pend/ready queue */
             struct _sock *pPending;    /* Pending connection sockets */
             struct _sock *pReady;      /* Ready connected sockets */
             uint32_t   ConnMax;        /* Max pending/ready connections */
             uint32_t   ConnTotal;      /* Total connections */

             /* Read/Write Stuff */
             int        ErrorPending;   /* Error returned on next socket call */
             int32_t    OOBMark;        /* Out of band info mark */
             uint32_t   OOBData;        /* Out of band info data */
             uint32_t   RxTimeout;      /* Timeout for Rx IO on stream */
             uint32_t   TxTimeout;      /* Timeout for Tx IO on stream */
             uint32_t   RxBufSize;      /* Rx Buffer Size */
             uint32_t   TxBufSize;      /* Tx Buffer Size */
             void       *hSBRx;          /* Rx Buffer */
             void       *hSBTx;          /* Tx Buffer */

             MCAST_SOCK_REC* pMcastList; /* List of multicast addresses on the socket. */

             TimestampFxn pTimestampFxn; /* Callout function pointer to */
                                         /* timestamp TX  */
        } SOCK;

/* Socket Protocol Block */
typedef struct _sockpcb {
             uint32_t   IPAddrLocal;    /* Local IP Address */
             uint32_t   PortLocal;      /* Local IP Port */
             uint32_t   IPAddrForeign;  /* Foreign IP Address */
             uint32_t   PortForeign;    /* Foreign IP Port */
             uint32_t   State;          /* Socket State (protocol dependent) */
        } SOCKPCB;

/* Socket Access Functions */

/*------------------------------------------------------------------------ */
/* General Access Functions (called from upper layers) */
extern int    SockNew( int Family, int Type, int Protocol,
                        int RxBufSize, int TxBufSize, void **phSock );
extern int    SockClose( void *hSock );

extern int    SockCheck( void *hSock, int IoType );
#define  SOCK_READ       0
#define  SOCK_WRITE      1
#define  SOCK_EXCEPT     2

extern int    SockStatus( void *hSock, int request, int *results );

extern int    SockSet(void *hSock, int Type, int Prop, void *pbuf, int size);
extern int    SockGet(void *hSock, int Type, int Prop, void *pbuf, int *psize);

extern int    SockShutdown( void *hSock, int how );

extern int    SockConnect( void *hSock, struct sockaddr *pName );
extern int    SockDisconnect( void *hSock );
extern int    SockBind( void *hSock, struct sockaddr *pName );

extern int    SockGetName( void *hSock, struct sockaddr *pSockName, struct sockaddr *pPeerName );

extern int    SockListen( void *hSock, int maxcon );
extern int    SockAccept( void *hSock, void **phSock );

extern int    SockRecv( void *hSock, char *pBuf, int32_t size,
                        int flags, struct sockaddr *pPeer, int32_t *pRetSize );
extern int    SockSend( void *hSock, char *pBuf, int32_t size,
                        int flags, int32_t *pRetSize );
extern int    SockRecvNC( void *hSock, int flags, struct sockaddr *pPeer, PBM_Pkt **ppPkt );

extern int    SockGetPcb( uint32_t SockProt, uint32_t BufSize, unsigned char *pBuf );
extern void SockCleanPcb (uint32_t SockProt, uint32_t IPAddress);
extern int    SockGetCtx (void *hSock);

/*------------------------------------------------------------------------ */
/* PCB Related Socket Access Functions (called from stack protocols) */
extern int      SockPcbAttach( void *hSock );
extern int      SockPcbDetach( void *hSock );
extern void     SockPcbCleanup();
extern int      SockPcbBind( void *hSock, uint32_t IP, uint32_t Port );
extern int      SockPcbConnect( void *hSock, uint32_t IP, uint32_t Port );
extern void    *SockPcbResolve( uint32_t SockProt, uint32_t LIP, uint32_t LPort,
                                uint32_t FIP, uint32_t FPort, uint32_t Match,
                                uint32_t * MaxFlag );
#define SOCK_RESOLVE_BEST       1       /* Return Best Match */
#define SOCK_RESOLVE_EXACT      2       /* Return Exact Match */
#define SOCK_RESOLVE_SPAWN      3       /* Return exact, or spawn on best match */
extern void *SockPcbResolveChain( void *hSock, uint32_t wSockProt,
                                   uint32_t Prot, uint32_t LIP, uint32_t LPort,
                                   uint32_t FIP, uint32_t FPort );
extern void   SockPcbCtlError( uint32_t Code, IPHDR *pIpHdr );
extern void   SockPcbRtChange( void *hRt );

/*------------------------------------------------------------------------ */
/* Low-level Access Functions (called from stack protocols) */
extern int    SockNotify( void *hSock, int Notification );
#define SOCK_NOTIFY_CONNECT     1
#define SOCK_NOTIFY_RCVACK      2
#define SOCK_NOTIFY_RCVDATA     3
#define SOCK_NOTIFY_RCVFIN      4
#define SOCK_NOTIFY_DISCONNECT  5
#define SOCK_NOTIFY_CLOSED      6
#define SOCK_NOTIFY_ERROR       7

extern void   SockSetOOBMark( void *hSock, int32_t OOBMark );
extern void   SockSetOOBData( void *hSock, unsigned char OOBData );
extern void   SockSpawnAbort( void *hSock );
extern void *SockValidateRoute( void *hSock );
extern PBM_Pkt *SockCreatePacket( void *hSock, uint32_t Size );

/* Low Level Object Interface */

#ifdef _STRONG_CHECKING
extern uint32_t   SockGetProtocol( void *h );
extern uint32_t   SockGetIpHdrSize( void *h );
extern void *SockGetTx( void *h );
extern void *SockGetRx( void *h );
extern void *SockGetTP( void *h );
extern uint32_t SockGetLIP( void *h );
extern uint32_t SockGetFIP( void *h );
extern uint32_t   SockGetLPort( void *h );
extern uint32_t   SockGetFPort( void *h );
extern void *SockGetRoute( void *h );
extern void *SockGetIFTx( void *h );
extern uint32_t   SockGetOptionFlags( void *h );
extern void   SockSetError( void *h, int Error );
#else
#define SockGetProtocol( h )     (((SOCK *)h)->Protocol)
#define SockGetIpHdrSize( h )    ((((SOCK *)h)->IpFlags & IP_HDRINCL) ? 0 : (IPHDR_SIZE + (((SOCK *)h)->IpOptSize)))
#define SockGetTx( h )           (((SOCK *)h)->hSBTx)
#define SockGetRx( h )           (((SOCK *)h)->hSBRx)
#define SockGetTP( h )           (((SOCK *)h)->hTP)
#define SockGetLIP( h )          (((SOCK *)h)->LIP)
#define SockGetFIP( h )          (((SOCK *)h)->FIP)
#define SockGetLPort( h )        (((SOCK *)h)->LPort)
#define SockGetFPort( h )        (((SOCK *)h)->FPort)
#define SockGetRoute( h )        (((SOCK *)h)->hRoute)
#define SockGetIFTx( h )         (((SOCK *)h)->hIFTx)
#define SockGetOptionFlags( h )  (((SOCK *)h)->OptionFlags)
#define SockSetError( h, e )     (((SOCK *)h)->ErrorPending=(e))
#endif

#ifdef _INCLUDE_IPv6_CODE

/**
 * @brief
 *  The structure describes the IPv6 Socket data structure.
 *
 * @details
 *  This data structure identifies a socket used for communication
 *  over IPv6.
 */
typedef struct SOCK6 {
    /**
     * @brief   File descriptor header
     */
    FILEDESC    fd;

    /**
     * @brief   Socket context
     */
    int32_t    Ctx;

    /**
     * @brief   Address family of the socket (AF_INET6)
     */
    unsigned char     Family;

    /**
     * @brief   Socket Type (SOCK_STREAM/SOCK_DGRAM/SOCK_RAW)
     */
    unsigned char     SockType;

    /**
     * @brief   IP Protocol (TCP/UDP/ICMPV6 etc)
     */
    unsigned char     Protocol;

    /* IPv6 Specific Options */
    /**
     * @brief   Flow label associated with the socket.
     */
    uint32_t    FlowLabel;

    /**
     * @brief   Hop Limit for Unicast packets.
     */
    unsigned char      HopLimit;

    /**
     * @brief   Scope ID associated with the socket.
     */
    uint32_t    ScopeId;
    /* End of IPv6 Specific Options */

    /**
     * @brief   Socket Priority.
     */
    uint16_t    SockPriority;

    /**
     * @brief   Socket Protocol Handler (SOCKPROT_NONE/SOCKPROT_TCP/
     *                                   SOCKPROT_UDP/SOCKPROT_RAW)
     */
    uint32_t    SockProt;

    /**
     * @brief   Socket Option flags (SO_...) as defined in socket.h
     */
    uint32_t   OptionFlags;

    /**
     * @brief   Socket State flags (SS_...). Defined and used by
     *          NDK core stack internally.
     */
    uint32_t   StateFlags;

    /**
     * @brief   Time used when SO_LINGER set
     */
    uint32_t    dwLingerTime;

    /* Protocol Control */
    /**
     * @brief   Pointer to next socket in protocol list
     */
    struct SOCK6   *pProtNext;

    /**
     * @brief   Pointer to previous socket in protocol list
     */
     struct SOCK6  *pProtPrev;

    /**
     * @brief   Foreign Host's IPv6 Address
     */
    IP6N        FIP;

    /**
     * @brief   Foreign Host's Port
     */
    uint16_t    FPort;

    /**
     * @brief   Local IPv6 Address (NULL for wildcard)
     */
    IP6N        LIP;

    /**
     * @brief   Local Port to which socket is bound (NULL if not bound)
     */
    uint16_t    LPort;

    /**
     * @brief   IPv6 Address to which this socket is bound (valid for DGRAM sockets)
     */
     IP6N       BIP;

    /**
     * @brief   Port to which this socket is bound (valid for DGRAM sockets)
     */
     uint16_t   BPort;

    /**
     * @brief   Handle to Protocol specific data
     */
    void        *hTP;

    /**
     * @brief   Handle to the cached V6 Route.
     */
    void        *hRoute6;

    /**
     * @brief   Handle to default Interface for transmit
     */
    void        *hIFTx;

    /* End of Protocol Control */

    /* Connection State Stuff */
    /**
     * @brief   Handle to the parent socket.
     */
    struct SOCK6   *pParent;

    /**
     * @brief   Previous Socket in Pending/ready queue.
     */
    struct SOCK6   *pPrevQ;

    /**
     * @brief   Pending connection sockets list.
     */
    struct SOCK6   *pPending;

    /**
     * @brief   Ready/connected sockets list.
     */
    struct SOCK6   *pReady;

    /**
     * @brief   Maximum allowed pending/ready connections on this
     *          socket.
     */
     uint32_t   ConnMax;

    /**
     * @brief   Total number of current connections on socket.
     */
     uint32_t   ConnTotal;

    /* End of Connection State Stuff */

    /* Read/Write Stuff */

    /**
     * @brief   Error returned on socket call
     */
    int         ErrorPending;

    /**
     * @brief   Out of band info mark
     */
    int32_t    OOBMark;

    /**
     * @brief   Out of band info data
     */
    uint32_t   OOBData;

    /**
     * @brief   Receive Timeout for stream sockets.
     */
    uint32_t    RxTimeout;

    /**
     * @brief   Transmit Timeout for stream sockets.
     */
    uint32_t    TxTimeout;

    /**
     * @brief   Receive buffer size
     */
    uint32_t    RxBufSize;

    /**
     * @brief   Transmit buffer size
     */
    uint32_t    TxBufSize;

    /**
     * @brief   Handle to Receive buffer of the socket.
     */
    void        *hSBRx;

    /**
     * @brief   Handle to Transmit buffer of the socket.
     */
    void        *hSBTx;

    /**
     * @brief   List of multicast addresses on the socket.
     */
    MCAST_SOCK_REC6 * pMcastList;

    /* End of Read/Write Stuff */
} SOCK6;

/**
 * @brief
 *  The structure describes the IPv6 Socket Protocol specific
 *  properties.
 *
 */
typedef struct SOCK6PCB {
    /**
     * @brief   Local IPv6 Address
     */
    IP6N        IPAddrLocal;

    /**
     * @brief   Local IP Port
     */
    uint16_t    PortLocal;

    /**
     * @brief   Foreign host's IPv6 Address
     */
    IP6N        IPAddrForeign;

    /**
     * @brief   Foreign IP Port
     */
    uint16_t    PortForeign;

    /**
     * @brief   Socket state (protocol dependent)
     */
    uint32_t    State;
} SOCK6PCB;

/* Socket Access Functions */

/*------------------------------------------------------------------------ */
/* General Access Functions (called from upper layers) */
extern int    Sock6New( int Family, int Type, int Protocol,
                        int RxBufSize, int TxBufSize, void **phSock );
extern int    Sock6Close( void *hSock );

extern int    Sock6Check( void *hSock, int IoType );

extern int    Sock6Status( void *hSock, int request, int *results );

extern int    Sock6Set(void *hSock, int Type, int Prop, void *pbuf, int size);
extern int    Sock6Get(void *hSock, int Type, int Prop, void *pbuf, int *psize);

extern int    Sock6Shutdown( void *hSock, int how );

extern int    Sock6Connect( void *hSock, struct sockaddr *pName );
extern int    Sock6Disconnect( void *hSock );
extern int    Sock6Bind( void *hSock, struct sockaddr *pName );

extern int    Sock6GetName( void *hSock, struct sockaddr *pSockName, struct sockaddr *pPeerName );

extern int    Sock6Listen( void *hSock, int maxcon );
extern int    Sock6Accept( void *hSock, void **phSock );

extern int    Sock6Recv( void *hSock, char *pBuf, int32_t size,
                        int flags, struct sockaddr *pPeer, int32_t *pRetSize );
extern int    Sock6Send( void *hSock, char *pBuf, int32_t size,
                        int flags, int32_t *pRetSize );
extern int    Sock6RecvNC( void *hSock, int flags, struct sockaddr *pPeer, PBM_Pkt **ppPkt );

extern int    Sock6GetPcb( uint32_t SockProt, uint32_t BufSize, unsigned char *pBuf );
extern void   Sock6CleanPcb (uint32_t SockProt, IP6N IPAddress);
extern int    Sock6GetCtx (void *hSock);

/*------------------------------------------------------------------------ */
/* PCB Related Socket Access Functions (called from stack protocols) */
extern int    Sock6PcbAttach( void *hSock );
extern int    Sock6PcbDetach( void *hSock );
extern void   Sock6PcbCleanup();
extern int    Sock6PcbBind( void *hSock, IP6N IP, uint32_t Port );
extern int    Sock6PcbConnect( void *hSock, IP6N IP, uint32_t Port );
extern void *Sock6PcbResolve( uint32_t SockProt, IP6N LIP, uint32_t LPort,
                               IP6N FIP, uint32_t FPort, uint32_t Match,
                               uint32_t * MaxFlag );
extern void *Sock6PcbResolveChain( void *hSock, uint32_t wSockProt,
                                    uint32_t Prot, IP6N LIP, uint32_t LPort,
                                    IP6N FIP, uint32_t FPort );
extern void   Sock6PcbRtChange( void *hRt );

/*------------------------------------------------------------------------ */
/* Low-level Access Functions (called from stack protocols) */
extern int    Sock6Notify( void *hSock, int Notification );
extern void   Sock6SetOOBMark( void *hSock, int32_t OOBMark );
extern void   Sock6SetOOBData( void *hSock, unsigned char OOBData );
extern void   Sock6SpawnAbort( void *hSock );
extern PBM_Pkt *Sock6CreatePacket( void *hSock, uint32_t Size,
                                   uint32_t NextHeader );
extern void *Sock6ValidateRoute(void *h);

/*------------------------------------------------------------------------ */
/* Protocol Specific Socket Functions */
extern void Sock6PrCtlError(SOCK6 *ps, uint32_t Code);

/* Low Level Object Interface */
#define Sock6GetProtocol( h )     (((SOCK6 *)h)->Protocol)
#define Sock6GetIpHdrSize( h )    (IPv6HDR_SIZE)
#define Sock6GetTx( h )           (((SOCK6 *)h)->hSBTx)
#define Sock6GetRx( h )           (((SOCK6 *)h)->hSBRx)
#define Sock6GetTP( h )           (((SOCK6 *)h)->hTP)
#define Sock6GetLIP( h )          (((SOCK6 *)h)->LIP)
#define Sock6GetFIP( h )          (((SOCK6 *)h)->FIP)
#define Sock6GetLPort( h )        (((SOCK6 *)h)->LPort)
#define Sock6GetFPort( h )        (((SOCK6 *)h)->FPort)
#define Sock6GetRoute( h )        (((SOCK6 *)h)->hRoute6)
#define Sock6GetIFTx( h )         (((SOCK6 *)h)->hIFTx)
#define Sock6GetOptionFlags( h )  (((SOCK6 *)h)->OptionFlags)
#define Sock6SetError( h, e )     (((SOCK6 *)h)->ErrorPending=(e))

#endif /* _INCLUDE_IPv6_CODE */

#define SOCKPROT_RAWETH    4

/**
 * @brief
 *  The structure describes the Raw Ethernet Sock Object Structure.
 *
 * @details
 *  This data structure identifies a socket used for communication
 *  over Raw Ethernet Sockets.
 */
typedef struct _SOCKRAWETH
{
    /**
     * @brief       File descriptor header
     */
    FILEDESC        fd;

    /**
     * @brief   Socket context
     */
    int32_t         Ctx;

    /**
     * @brief       Address family of the socket (AF_RAWETH)
     */
    uint32_t        Family;

    /**
     * @brief       Socket Type (SOCK_RAWETH)
     */
    uint32_t        SockType;

    /**
     * @brief       Layer3 Protocol (can by any custom L3 types)
     */
    uint32_t        Protocol;

    /**
     * @brief       Socket Priority.
     */
    uint16_t        SockPriority;

    /**
     * @brief       Socket Protocol Handler (SOCKPROT_RAWETH)
     */
    uint32_t        SockProt;

    /**
     * @brief       Socket State flags (SS_...). Defined and used by
     *              NDK core stack internally.
     */
    uint32_t        StateFlags;

    /**
     * @brief       Pointer to next socket in protocol list
     */
    struct _SOCKRAWETH     *pPrev;

    /**
     * @brief       Pointer to previous socket in protocol list
     */
    struct _SOCKRAWETH     *pNext;

    /* Protocol Control */

    /**
     * @brief       Handle to Interface on which packets need to
     *              be Rxed or Txed
     */
    void            *hIF;

    /* Read-Write Stuff */

    /**
     * @brief       Error returned on socket call
     */
    int32_t         ErrorPending;

    /**
     * @brief       Receive Timeout for the socket.
     */
    uint32_t        RxTimeout;

    /**
     * @brief       Receive buffer size
     */
    uint32_t        RxBufSize;

    /**
     * @brief       Transmit buffer size
     */
    uint32_t        TxBufSize;

    /**
     * @brief       Handle to Receive buffer of the socket.
     */
    void            *hSBRx;

    /**
     * @brief       Handle to Transmit buffer of the socket.
     */
    void            *hSBTx;

    /**
     * @brief       Handle to Timestamp function.
     */
    TimestampFxn   	pTimestampFxn;

} SOCKRAWETH;

/* Socket Access Functions */

/*------------------------------------------------------------------------ */
/* General Access Functions (called from upper layers) */
extern int    RawEthSockNew( int Family, int Type, int Protocol,
                        int RxBufSize, int TxBufSize, void **phSock );
extern int    RawEthSockClose( void *hSock );

extern int    RawEthSockCheck( void *hSock, int IoType );

extern int    RawEthSockStatus( void *hSock, int request, int *results );

extern int    RawEthSockSet(void *hSock, int Type, int Prop, void *pbuf, int size);
extern int    RawEthSockGet(void *hSock, int Type, int Prop, void *pbuf, int *psize);

extern int    RawEthSockShutdown( void *hSock, int how );

extern int    RawEthSockSend( void *hSock, char *pBuf, int32_t size,
                        int32_t *pRetSize );
extern int    RawEthSockRecvNC( void *hSock, PBM_Pkt **ppPkt );
extern int    RawEthSockSendNC( void *hSock, char *pBuf, int32_t size, void *hPkt,
                        int32_t *pRetSize );
extern int    RawEthSockGetCtx (void *hSock);


/*------------------------------------------------------------------------ */
/* PCB Related Socket Access Functions (called from stack protocols) */
extern int    RawEthSockPcbAttach( void *hSock );
extern int    RawEthSockPcbDetach( void *hSock );
extern void   RawEthSockPcbCleanup(void);
extern int    RawEthSockPcbInit (void);
extern SOCKRAWETH* RawEthSockPcbFind( uint32_t  Protocol, void *hIF );


/*------------------------------------------------------------------------ */
/* Low-level Access Functions (called from stack protocols) */
extern int    RawEthSockNotify( void *hSock, int Notification );
extern PBM_Pkt* RawEthSockCreatePacket( void *hSock, uint32_t Payload, uint32_t* pError );

#define RawEthSockGetProtocol( h )        (((SOCKRAWETH *)h)->Protocol)
#define RawEthSockGetRx( h )              (((SOCKRAWETH *)h)->hSBRx)
#define RawEthSockGetIF( h )              (((SOCKRAWETH *)h)->hIF)
#define RawEthSockGetPriority( h )        (((SOCKRAWETH *)h)->SockPriority)
#define RawEthSockSetError( h, e )        (((SOCKRAWETH *)h)->ErrorPending=(e))

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* _SOCKIF_INC_ */

