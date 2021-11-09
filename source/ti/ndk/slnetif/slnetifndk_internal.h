/*
 * Copyright (c) 2017-2019 Texas Instruments Incorporated - http://www.ti.com
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
 */

/*
 * ======== slnetifndk_internal.h ========
 * Internal header for SlNetIfNDK
 */

#ifndef ti_ndk_slnetif_SlNetIfNDK_internal__include
#define ti_ndk_slnetif_SlNetIfNDK_internal__include

#include <ti/net/slnetsock.h>
#include <ti/net/slnetif.h>

#ifdef    __cplusplus
extern "C" {
#endif

/*
 * ======== SlNetIfNDK_socket ========
 * Only supports
 *    - domain: SLNETSOCK_AF_INET, SLNETSOCK_AF_INET6
 *    - type: SLNETSOCK_SOCK_STREAM, SLNETSOCK_SOCK_DGRAM, SLNETSOCK_SOCK_RAW
 *    - protocol: SLNETSOCK_PROTO_TCP, SLNETSOCK_PROTO_UDP
 */
int16_t SlNetIfNDK_socket(void *ifContext, int16_t domain, int16_t type,
        int16_t protocol, void **psdContext);

/*
 * ======== SlNetIfNDK_close ========
 */
int32_t SlNetIfNDK_close(int16_t sd, void *sdContext);

/*
 * ======== SlNetIfNDK_shutdown ========
 */
int32_t SlNetIfNDK_shutdown(int16_t sd, void *sdContext, int16_t how);

/*
 * ======== SlNetIfNDK_accept ========
 */
int16_t SlNetIfNDK_accept(int16_t sd, void *sdContext, SlNetSock_Addr_t *addr,
        SlNetSocklen_t *addrlen, uint8_t flags, void **acceptedSdContext);

/*
 * ======== SlNetIfNDK_bind ========
 */
int32_t SlNetIfNDK_bind(int16_t sd, void *sdContext,
        const SlNetSock_Addr_t *addr, int16_t addrlen);

/*
 * ======== SlNetIfNDK_listen ========
 */
int32_t SlNetIfNDK_listen(int16_t sd, void *sdContext, int16_t backlog);

/*
 * ======== SlNetIfNDK_connect ========
 */
int32_t SlNetIfNDK_connect(int16_t sd, void *sdContext,
        const SlNetSock_Addr_t *addr, SlNetSocklen_t addrlen, uint8_t flags);

/*
 * ======== SlNetIfNDK_getPeerName ========
 */
int32_t SlNetIfNDK_getPeerName(int16_t sd, void *sdContext,
        SlNetSock_Addr_t *addr, SlNetSocklen_t *addrlen);

/*
 * ======== SlNetIfNDK_getSockName ========
 */
int32_t SlNetIfNDK_getSockName(int16_t sd, void *sdContext,
        SlNetSock_Addr_t *addr, SlNetSocklen_t *addrlen);

/*
 * ======== SlNetIfNDK_select ========
 */
int32_t SlNetIfNDK_select(void *ifContext, int16_t nfds,
        SlNetSock_SdSet_t *readsds, SlNetSock_SdSet_t *writesds,
        SlNetSock_SdSet_t *exceptsds, SlNetSock_Timeval_t *timeout);

/*
 * ======== SlNetIfNDK_setSockOpt ========
 * List of unsupported options
 * - SLNETSOCK_LVL_SOCKET: SLNETSOCK_OPSOCK_KEEPALIVE_TIME,
 *     SLNETSOCK_OPSOCK_NON_IP_BOUNDARY
 * - SLNETSOCK_LVL_IP: SLNETSOCK_OPIP_RAW_RX_NO_HEADER,
 *     SLNETSOCK_OPIP_MULTICAST_TTL
 * - SLNETSOCK_LVL_PHY: all options
 */
int32_t SlNetIfNDK_setSockOpt(int16_t sd, void *sdContext, int16_t level,
        int16_t optname, void *optval, SlNetSocklen_t optlen);

/*
 * ======== SlNetIfNDK_getSockOpt ========
 * List of unsupported options is same as SlNetIfNDK_setSockOpt
 */
int32_t SlNetIfNDK_getSockOpt(int16_t sd, void *sdContext, int16_t level,
        int16_t optname, void *optval, SlNetSocklen_t *optlen);

/*
 * ======== SlNetIfNDK_recv ========
 * Receives a message from a connection-mode socket
 *
 * If no messages are available to be received and the peer has performed an
 * orderly shutdown, 0 is returned.
 */
int32_t SlNetIfNDK_recv(int16_t sd, void *sdContext, void *buf, uint32_t len,
        uint32_t flags);

/*
 * ======== SlNetIfNDK_recvFrom ========
 */
int32_t SlNetIfNDK_recvFrom(int16_t sd, void *sdContext, void *buf,
        uint32_t len, uint32_t flags, SlNetSock_Addr_t *from,
        SlNetSocklen_t *fromlen);

/*
 * ======== SlNetIfNDK_send ========
 */
int32_t SlNetIfNDK_send(int16_t sd, void *sdContext, const void *buf,
        uint32_t len, uint32_t flags);

/*
 * ======== SlNetIfNDK_sendTo ========
 */
int32_t SlNetIfNDK_sendTo(int16_t sd, void *sdContext, const void *buf,
        uint32_t len, uint32_t flags, const SlNetSock_Addr_t *to,
        SlNetSocklen_t tolen);

/*
 * ======== SlNetIfNDK_getHostByName ========
 */
int32_t SlNetIfNDK_getHostByName(void *ifContext, char *name,
        const uint16_t nameLen, uint32_t *ipAddr, uint16_t *ipAddrLen,
        const uint8_t family);

/*
 * ======== SlNetIfNDK_ifCreateContext ========
 */
int32_t SlNetIfNDK_ifCreateContext(uint16_t ifID, const char *ifName,
        void **ifContext);

/*
 * ======== SlNetIfNDK_getIPAddr ========
 * Currently there is no IPv6 support in this function.
 */
int32_t SlNetIfNDK_getIPAddr(void *context, SlNetIfAddressType_e addrType,
        uint16_t *addrConfig, uint32_t *ipAddr);

/*
 * ======== SlNetIfNDK_getConnectionStatus ========
 * Unimplemented - TBD.
 */
int32_t SlNetIfNDK_getConnectionStatus(void *context);

#ifdef SLNETIFNDK_ENABLEMBEDTLS
/*
 * ======== SlNetIfNDK_closeSec ========
 * Secure version of SlNetIfNDK_close
 */
int32_t SlNetIfNDK_closeSec(int16_t sd, void *sdContext);

/*
 * ======== SlNetIfNDK_recvSec ========
 * Secure version of SlNetIfNDK_recv
 * Flags are unsupported.
 * If no messages are available to be received and the peer has performed an
 * orderly shutdown, 0 is returned.
 */
int32_t SlNetIfNDK_recvSec(int16_t sd, void *sdContext, void *buf, uint32_t len,
        uint32_t flags);

/*
 * ======== SlNetIfNDK_recvFromSec ========
 * Secure version of SlNetIfNDK_recvFrom
 * Flags are only supported when socket is not secure.
 */
int32_t SlNetIfNDK_recvFromSec(int16_t sd, void *sdContext, void *buf,
        uint32_t len, uint32_t flags, SlNetSock_Addr_t *from,
        SlNetSocklen_t *fromlen);

/*
 * ======== SlNetIfNDK_sendSec ========
 * Secure version of SlNetIfNDK_send
 * Flags are unsupported.
 */
int32_t SlNetIfNDK_sendSec(int16_t sd, void *sdContext, const void *buf,
        uint32_t len, uint32_t flags);

/*
 * ======== SlNetIfNDK_sendToSec ========
 * Secure version of SlNetIfNDK_sendTo
 * Flags are only supported when socket is not secure.
 */
int32_t SlNetIfNDK_sendToSec(int16_t sd, void *sdContext, const void *buf,
        uint32_t len, uint32_t flags, const SlNetSock_Addr_t *to,
        SlNetSocklen_t tolen);

/*
 * ======== SlNetIfNDK_sockStartSec ========
 * Unsupported attributes:
 *   - SLNETSOCK_SEC_ATTRIB_METHOD
 *   - SLNETSOCK_SEC_ATTRIB_CIPHERS
 *   - SLNETSOCK_SEC_ATTRIB_ALPN
 *   - SLNETSOCK_SEC_ATTRIB_EXT_CLIENT_CHLNG_RESP
 */
int32_t SlNetIfNDK_sockStartSec(int16_t sd, void *sdContext,
        SlNetSockSecAttrib_t *secAttrib, uint8_t flags);

/*
 * ======== SlNetIfNDK_loadSecObj ========
 */
int32_t SlNetIfNDK_loadSecObj(void *ifContext, uint16_t objType, char *objName,
        int16_t objNameLen, uint8_t *objBuff, int16_t objBuffLen);

#endif /* SLNETIFNDK_ENABLEMBEDTLS */

#ifdef  __cplusplus
}
#endif /* __cplusplus */

#endif /* ti_ndk_slnetif_SlNetIfNDK_internal__include */
