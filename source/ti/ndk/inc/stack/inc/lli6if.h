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
 * ======== lli6if.h ========
 *
 * Extern functions exported by LLI6 module.
 *
 */

#ifndef _C_LLI6_INC
#define _C_LLI6_INC  /* #defined if this .h file has been included */

#ifdef __cplusplus
extern "C" {
#endif

/* LLI6 Status Defintions: These definitions define the state in which the
 * LLI6 Entry exists. This is as per the Neighbor Unreachability Detection
 * Algorithm. */
#define ICMPV6_LLI_INCOMPLETE   0x1
#define ICMPV6_LLI_REACHABLE    0x2
#define ICMPV6_LLI_STALE        0x3
#define ICMPV6_LLI_DELAY        0x4
#define ICMPV6_LLI_PROBE        0x5
#define ICMPV6_LLI_DEAD			0x6

/** 
 * @brief 
 *  The structure describes the LLI6 Entry. 
 *
 * @details
 *  This is the Neighbor Cache which maps the unicast IPv6 address to the Link Layer 
 *  Address. The structure also contains the necessary information to execute the
 *  Neighbor Unreachability Detection Algorithm.
 *
 *  Since interfaces can have multiple IPv6 addresses. It is possible that there can 
 *  be multiple Unicast IPv6 addresses being mapped to a single LLI6 entry. 
 */
typedef struct LLI6_ENTRY
{
    /**
     * @brief   Links to other LLI6 Objects
     */
    NDK_LIST_NODE           links;

    /**
     * @brief   Reference Counter
     */
    uint32_t				RefCount;

    /**
     * @brief   The IsRouter Flag is maintained per NEIGH Entry.
     */
    unsigned char             IsRouter;

    /**
     * @brief   Device on which the LLI6 Entry exists.
     */
	NETIF_DEVICE* 		ptr_device;

    /**
     * @brief   Packet awaiting resolution.
     */
    PBM_Pkt*            pPkt;

    /**
     * @brief   MAC Address of the Entry.
     */
    unsigned char             MacAddr[6];

    /**
     * @brief   Neighbor Status as per the Neighbor Unreachability Detection.
     */
    uint32_t            status;

    /**
     * @brief   Source Address to be used while sending Neighbor Solicitations.
     */
    IP6N                SrcAddr;

    /**
     * @brief   Target Address to be used while sending Neighbor Solicitations.
     */
	IP6N                TargetAddr;

    /**
     * @brief   Number of Neighbor Solicitation Probes that have been sent out.
     */
    uint16_t            NumProbes;

    /**
     * @brief   Timestamp when the Last NS was sent out.
     */
    uint32_t            LastSendNS;

    /**
     * @brief   Timestamp of the Last Reachability Confirmation
     */    
    uint32_t            LastReachTS;
}LLI6_ENTRY;

/********************************************************************** 
 * Exported API (KERNEL MODE):
 *  These functions are exported by the LLI6 Module and are available 
 *  for internal NDK core stack usage only.
 ***********************************************************************/
extern void *LLI6New (RT6_ENTRY* ptr_rt6, unsigned char* pMacAddress, unsigned char IsRouter);
extern void   LLI6Update (void *hLLI6, unsigned char* mac_address, uint32_t RxPktType, uint32_t Flags);
extern void   LLI6Free (void *hLLI6);
extern unsigned char LLI6IsRouter (void *hLLI6);
extern unsigned char LLI6IsDead (void *hLLI6);
extern unsigned char LLI6IsValid (void *hLLI6);
extern void   LLI6IncRefCount (void *hLLI6);
extern void   LLI6SetIncomplete (void *hLLI6);
extern NETIF_DEVICE* LLI6GetNetDevice (void *hLLI6);
extern void   LLI6TxIPPacket (PBM_Pkt *pPkt, void *hlli6, NETIF_DEVICE* ptr_device,
							   IP6N Address);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* _C_LLI6_INC */

