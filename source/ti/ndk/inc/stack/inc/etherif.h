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
 * ======== etherif.h ========
 *
 */

#ifndef _C_ETHERIF_INC
#define _C_ETHERIF_INC  /* #defined if this .h file has been included */

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------- */
/* Device Structure */

/* Our Max Multicast (Independent of Low-level MAX) */
#define ETH_MIN_PAYLOAD  46

/* Maximum Ethernet Payload Size. */
#ifdef _INCLUDE_JUMBOFRAME_SUPPORT
#define ETH_MAX_PAYLOAD  10236
#else
#define ETH_MAX_PAYLOAD  1514
#endif

/* Message */
typedef struct _ethdev {
/*---[ These fields match IF structure ]--- */
        uint32_t Type;                     /* Set to HTYPE_ETH */
        uint32_t llIndex;                  /* Low-level Device Index */
        uint32_t ProtMTU;                  /* MTU less ETHER stuff */
/*-------------------------------------- */
        unsigned char  MacAddr[6];               /* Device's Unicast Addr */
        uint32_t PktFilter;                /* Current packet filter */
        uint32_t EthHdrSize;               /* Byte Size of ETH header */
        uint32_t OffDstMac;                /* Hdr offset to dst mac */
        uint32_t OffSrcMac;                /* Hdr offset to src mac */
        uint32_t OffEtherType;             /* Hdr offset to eth type */
        uint32_t PacketPad;                /* Bytes of pad at end of packet */
        uint32_t PhysMTU;                  /* Physical pkt size (with all) */
        TimestampFxn pTimestampFxn;           /* Callout function pointer to */
                                              /* timestamp for receieved */
                                              /* datagrams  */
       } ETHDRV;

/*----------------------------------------------------------------------- */
/* Ether Packet Filter (cumulative) */
#define ETH_PKTFLT_NOTHING      0
#define ETH_PKTFLT_DIRECT       1
#define ETH_PKTFLT_BROADCAST    2
#define ETH_PKTFLT_MULTICAST    3
#define ETH_PKTFLT_ALLMULTICAST 4
#define ETH_PKTFLT_ALL          5

/*----------------------------------------------------------------------- */
/* Packet Access Functions */
extern void   EtherRxPacket( PBM_Handle hPkt );

/*----------------------------------------------------------------------- */
/* Global Access Functions */
extern void *EtherNew( uint32_t  llIndex );
extern void EtherFree( void *hEther );
extern void EtherConfig( void *hEther, uint32_t PhysMTU, uint32_t EthHdrSize,
                         uint32_t OffDstMac, uint32_t OffSrcMac,
                         uint32_t OffEtherType, uint32_t PacketPad,
                         TimestampFxn timestampFxn);
extern void EtherSetPktFilter( void *hEther, uint32_t PktFilter );

extern uint32_t EtherAddMCast( void *hEther, unsigned char *pMCastAddr );
extern uint32_t EtherDelMCast( void *hEther, unsigned char *pMCastAddr );
extern void     EtherClearMCast( void *hEther );
extern uint32_t EtherGetMacAddr( void *hEther, unsigned char *pMacAddr,
                                 uint32_t MaxLen );


/*----------------------------------------------------------------------- */
/* Direct Functions */
#ifdef _STRONG_CHECKING
extern uint32_t   EtherGetPktFilter( void *hEther );
#else
#define EtherGetPktFilter( h )   (((ETHDRV *)h)->PktFilter)
#endif

/*----------------------------------------------------------------------- */
/* PBM Packet Flags Maintained by this module */

/* Packet Flags (maintained by the stack lib) */
#define FLG_PKT_MACBCAST        0x00000002  /* Pkt Received as a LL BCast */
#define FLG_PKT_MACMCAST        0x00000001  /* Pkt Received as a LL MCast */
#define FLG_PKT_NAT             0x00000004  /* NAT applied to Pkt */

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
