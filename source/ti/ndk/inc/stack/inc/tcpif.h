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
 * ======== tcpif.h ========
 *
 * TCP interface functions
 *
 */


#ifndef _TCPIF_INC_
#define _TCPIF_INC_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *  The structure describes the TCP Statistics Block.
 *
 * @details
 *  This structure holds the TCP related statistics and
 *  counters.
 */
typedef struct _tcpstat {

    /**
     * @brief   Number of Connection attempts.
     */
    uint32_t  ConnAttempt;

    /**
     * @brief   Connnection attempt dropped due to no open port
     */
    uint32_t  ConnAttemptNoPort;

    /**
     * @brief   Number of dropped connection attempts.
     */
    uint32_t  ConnDrops;

    /**
     * @brief   Number of connections accepted.
     */
    uint32_t  Accepts;

    /**
     * @brief   Number of connections dropped.
     */
    uint32_t  Drops;

    /**
     * @brief   Number of times "Persist timer" expired.
     */
    uint32_t  PersistTimeout;

    /**
     * @brief   Number of times "Keep"/"Connect" timer expired.
     */
    uint32_t  KeepTimeout;

    /**
     * @brief   Number of Retransmission timeouts.
     */
    uint32_t  RexmtTimeout;

    /**
     * @brief   Number of Keep probes sent.
     */
    uint32_t  KeepProbe;

    /**
     * @brief   Number of connections dropped in keep.
     */
    uint32_t  KeepDrops;

    /**
     * @brief   Number of connections dropped due to
     * Retransmission timeout.
     */
    uint32_t  TimeoutDrops;

    /**
     * @brief   Number of connections established.
     */
    uint32_t  Connects;

    /**
     * @brief   Number of times RTT timer was successful.
     */
    uint32_t  RttUpdated;

    /**
     * @brief   Number of Delayed ACKs sent out.
     */
    uint32_t  DelAck;

    /**
     * @brief   Number of TCP packets sent out.
     */
    uint32_t  SndTotal;

    /**
     * @brief   Number of TCP window probes sent.
     */
    uint32_t  SndProbe;

    /**
     * @brief   Number of data packets sent out.
     */
    uint32_t  SndPack;

    /**
     * @brief   Number of data bytes sent out.
     */
    uint32_t  SndByte;

    /**
     * @brief   Number of data packets retransmitted.
     */
    uint32_t  SndRexmitPack;

    /**
     * @brief   Number of data bytes retransmitted.
     */
    uint32_t  SndRexmitByte;

    /**
     * @brief   Number of ACK-only packets sent out.
     */
    uint32_t  SndAcks;

    /**
     * @brief   Number of TCP Control (SYN/FIN/RST) packets sent.
     */
    uint32_t  SndCtrl;

    /**
     * @brief   Number of TCP packets with "URG" flag set.
     */
    uint32_t  SndUrg;

    /**
     * @brief   Number of window update-only packets sent.
     */
    uint32_t  SndWinUp;

    /**
     * @brief   Number of "out of buffer" errors encountered
     * during TCP transmission.
     */
    uint32_t  SndNoBufs;

    /**
     * @brief   segs where we tried to get rtt
     */
    uint32_t  SegsTimed;

    /**
     * @brief   Total packets received.
     */
    uint32_t  RcvTotal;

    /**
     * @brief   Number of packets received that were too short.
     */
    uint32_t  RcvShort;

    /**
     * @brief   Number of packets received with bad TCP header size.
     */
    uint32_t  RcvHdrSize;

    /**
     * @brief   Number of packets received with checksum errors.
     */
    uint32_t  RcvBadSum;

    /**
     * @brief   Number of duplicate-only packets received.
     */
    uint32_t  RcvDupPack;

    /**
     * @brief   Number of duplicate-only bytes received.
     */
    uint32_t  RcvDupByte;

    /**
     * @brief   Number of partial duplicate packets received.
     */
    uint32_t  RcvPartDupPack;

    /**
     * @brief   Number of duplicate bytes from partial duplicate packets
     * received.
     */
    uint32_t  RcvPartDupByte;

    /**
     * @brief   Number of packets received after "close"
     */
    uint32_t  RcvAfterClose;

    /**
     * @brief   Number of packets with data past our window.
     */
    uint32_t  RcvAfterWinPack;

    /**
     * @brief   Number of bytes received past our window
     */
    uint32_t  RcvAfterWinByte;

    /**
     * @brief   Number of Window probe packets recieved.
     */
    uint32_t  RcvWinProbe;

    /**
     * @brief   Number of Duplicate ACK packets recieved.
     */
    uint32_t  RcvDupAck;

    /**
     * @brief   Number of Duplicate ACK packets recieved
     * for unsent data.
     */
    uint32_t  RcvAckTooMuch;

    /**
     * @brief   Number of ACK packets recieved
     */
    uint32_t  RcvAckPack;

    /**
     * @brief   Number of bytes acked by ACK packets recieved
     */
    uint32_t  RcvAckByte;

    /**
     * @brief   Number of Window update packets recieved
     */
    uint32_t  RcvWinUpd;

    /**
     * @brief   Number of packets received in sequence.
     */
    uint32_t  RcvPack;

    /**
     * @brief   Number of bytes received in sequence.
     */
    uint32_t  RcvByte;

    /**
     * @brief   Number of packets received out of sequence.
     */
    uint32_t  RcvOOPack;

    /**
     * @brief   Number of bytes received out of sequence.
     */
    uint32_t  RcvOOByte;

} TCPSTATS;

extern TCPSTATS NDK_tcps;
extern PSEUDO   tpseudo;

#ifdef _INCLUDE_IPv6_CODE
/* Statistics for the TCP Module over V6. */
extern TCPSTATS NDK_tcp6_stats;
#endif

/* Sequence MACROS */
#if defined(__TMS470__)

/*
 *  These static inlines are needed to workaround:
 *
 *  SDSCM00038834 - unsigned compare macro problems with Arm 4.6.x codegen
 */
static inline int SEQ_LT(uint32_t a, uint32_t b)
{
    volatile int32_t zero = 0;
    return ((int32_t)(a - b) < zero);
}

static inline int SEQ_LEQ(uint32_t a, uint32_t b)
{
    volatile int32_t zero = 0;
    return ((int32_t)(a - b) <= zero);
}

static inline int SEQ_GT(uint32_t a, uint32_t b)
{
    volatile int32_t zero = 0;
    return ((int32_t)(a - b) > zero);
}

static inline int SEQ_GEQ(uint32_t a, uint32_t b)
{
    volatile int32_t zero = 0;
    return ((int32_t)(a - b) >= zero);
}

#else

#define SEQ_LT(a,b) ((int32_t)((a)-(b)) < 0)
#define SEQ_LEQ(a,b) ((int32_t)((a)-(b)) <= 0)
#define SEQ_GT(a,b) ((int32_t)((a)-(b)) > 0)
#define SEQ_GEQ(a,b) ((int32_t)((a)-(b)) >= 0)

#endif

/* TCP Functions */
extern void  TcpTimeoutCheck();    /* Called every 0.1 sec */
extern uint32_t TcpPseudoCs();
extern void  TcpChecksum( TCPHDR * );
extern void  TcpInput( PBM_Pkt *pPkt );
extern void  TcpQuench( void *hTcp );

/* TCP Protocol Entry Points */
extern int   TcpPrAttach( void *h, void **phTcp );
extern int   TcpPrDetach( void *h, void **phTcp, int fatal );
extern int   TcpPrListen( void *h, void *hTcp );
extern int   TcpPrConnect( void *h, void *hTcp );
extern int   TcpPrDisconnect( void *h, void *hTcp );
extern int   TcpPrRecv( void *h, void *hTcp );
extern int   TcpPrSend( void *h, void *hTcp, unsigned char *pBuf, int32_t sz, int32_t *prsz );
extern int   TcpPrSendOOB( void *h, void *hTcp, unsigned char *pBuf, int32_t sz, int32_t *prsz );
extern void  TcpPrInherit( void *hP, void *hTcpP, void *hC, void *hTcpC );
extern int   TcpPrSetOption( void *h, void *hTcp, int Prop, void *pbuf, int size );
extern int   TcpPrGetOption( void *h, void *hTcp, int Prop, void *pbuf, int *psize );
extern void  TcpPrCtlError( void *h, void *hTcp, uint32_t Code, int Error );
extern uint32_t  TcpPrGetState( void *h, void *hTcp );

#ifdef _INCLUDE_IPv6_CODE
extern void TCP6Input (PBM_Pkt* pPkt, IPV6HDR* ptr_ipv6hdr);
extern void TCP6TimeoutCheck();

extern int  TCP6PrAttach( void *h, void **phTcp );
extern int  TCP6PrDetach( void *h, void **phTcp, int fatal );
extern int  TCP6PrListen( void *h, void *hTcp );
extern int  TCP6PrConnect( void *h, void *hTcp );
extern int  TCP6PrDisconnect( void *h, void *hTcp );
extern int  TCP6PrRecv( void *h, void *hTcp );
extern int  TCP6PrSend( void *h, void *hTcp, unsigned char *pBuf, int32_t Size, int32_t *pRetSize );
extern int  TCP6PrSendOOB( void *h, void *hTcp, unsigned char *pBuf, int32_t Size, int32_t *pRetSize );
extern void TCP6PrInherit (void *hParent, void *hTcpParent, void *hChild, void *hTcpChild);
extern int  TCP6PrSetOption( void *h, void *hTcp, int Prop, void *pbuf, int size );
extern int  TCP6PrGetOption( void *h, void *hTcp, int Prop, void *pbuf, int *psize );
extern void TCP6PrCtlError( void *h, void *hTcp, uint32_t Code, int Error );
extern uint32_t TCP6PrGetState( void *h, void *hTcp );

#endif /* _INCLUDE_IPv6_CODE */

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
