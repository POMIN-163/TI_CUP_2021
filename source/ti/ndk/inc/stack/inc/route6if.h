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
 * ======== route6if.h ========
 *
 * Common structures and definitions used for the ROUTE6 Module.
 *
 */


#ifndef _C_ROUTE6_INC
#define _C_ROUTE6_INC  /* #defined if this .h file has been included */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *  The structure describes the ROUTE6 Entry.
 *
 * @details
 *  There exists an entry for each of the following:-
 *
 *   - Every Network (Prefix List) to which the box is connected
 *      These routes are known as CLONING Routes since they can spawn
 *      multiple Host Routes (b) below. These routes do -not- have an
 *      LLI6 Entry since they map to a network route which is ON LINK
 *
 *   - Every Host on the network to which packets have been sent
 *      These routes are typically CLONED routes. These routes always
 *      have a matching LLI6 Entry since they this maps a HOST IPv6
 *      Address to an LLI6 Entry.
 *
 *   - Default Routes (Gateway Routes)
 *      These routing entries exist for default gateways; the 'IPAddr'
 *      in this case is :: and so is the 'IPMask'. The 'NextHop' is
 *      the Address of the GATEWAY. These routes are always added to
 *      the end of the routing table since they will always match.
 *      These routes always have an LLI6 Entry.
 *
 * Thus currently the Flags supported here are:-
 *   -  FLG_RTE_CLONING
 *         This identifies (a) above.
 *   -  FLG_RTE_HOST
 *         This identifies (b) above.
 *   -  FLG_RTE_GATEWAY
 *         This identifies (c) above.
 */
typedef struct RT6_ENTRY
{
    /**
     * @brief   Links to the the next entry.
     */
    NDK_LIST_NODE       links;

    /**
     * @brief   Routing Flag Entries. This can be set to Host, Network
     *          or Gateway Routes.
     */
    uint32_t        Flags;

    /**
     * @brief   Reference Counter
     */
    uint32_t        RefCount;

    /**
     * @brief   Expiration time in SECONDS
     */
    uint32_t        dwTimeout;

    /**
     * @brief   Network Address
     */
    IP6N            NetworkAddr;

    /**
     * @brief   IP Address.
     */
    IP6N            IPAddr;

    /**
     * @brief   IP Mask
     */
    IP6N            IPMask;

    /**
     * @brief   Next Hop IP Address.
     */
    IP6N            NextHop;

    /**
     * @brief   Protocol MTU for this route.
     */
    uint32_t        ProtMTU;

    /**
     * @brief   Pointer to the network interface on which the route resides.
     */
    NETIF_DEVICE*   ptr_device;

    /**
     * @brief   void *to the LLI6 Entry
     */
    void            *hLLI6;
}RT6_ENTRY;

/**********************************************************************
 * Exported API (KERNEL MODE):
 *  These functions are exported by the ROUTE6 Module and are available
 *  for internal NDK core stack usage only.
 ***********************************************************************/
extern void *Rt6Create (uint32_t flags, IP6N Addr, IP6N Mask, IP6N NextHop,
                          unsigned char* mac_address, NETIF_DEVICE* ptr_device, uint32_t ValidLifetime);
extern void *Rt6Modify (void *hRoute6, uint32_t flags, IP6N IPAddr, IP6N IPMask, IP6N NextHop,
                          unsigned char* mac_address,NETIF_DEVICE* ptr_device, uint32_t ValidLifetime);
extern void   Rt6Update (void *hRoute, uint32_t RxPacketType, unsigned char* mac_address,
                         uint32_t Flags, uint32_t Lifetime);
extern void *Rt6Find (uint32_t Flags, IP6N IPAddress, NETIF_DEVICE* ptr_device);
extern void *Rt6FindDefaultRouter (IP6N RouterAddress, NETIF_DEVICE* ptr_device);
extern void   Rt6Free   (void *hRoute6);
extern void   Rt6FlushInterfaceRoutes (NETIF_DEVICE* ptr_device);
extern void *Rt6GetIF  (void *hRoute6);
extern void *Rt6GetLLI (void *hRoute6);
extern uint32_t Rt6GetMTU (void *hRoute6);
extern int    Rt6IsLocalRoute (void *hRoute);
extern void   Rt6IncRefCount (void *hRoute6);
extern int Rt6IsDefaultRoute (void *hRoute6);
extern void Rt6GetNetworkAddr (void *hRoute6, IP6N* IPNet);

/**********************************************************************
 * Exported API (KERNEL MODE SAFE):
 *  These functions are exported by the ROUTE6 Module and are available
 *  for application developers to be used.
 ***********************************************************************/
extern RT6_ENTRY* Rt6GetTable (void);
extern void       Rt6CleanTable (RT6_ENTRY* ptr_list);

#ifdef _INCLUDE_IPv6_CODE
static inline void PBM_setRoute6( PBM_Pkt *pPkt, void *hRoute6 )
{
    if (hRoute6)
        Rt6IncRefCount (hRoute6);
    if (pPkt->hRoute6)
        Rt6Free (pPkt->hRoute6);
    pPkt->hRoute6 = hRoute6;
}
#endif /* _INCLUDE_IPv6_CODE */

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* _C_ROUTE6_INC */

