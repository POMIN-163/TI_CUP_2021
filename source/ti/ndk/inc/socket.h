/*
 * Copyright (c) 2013-2018, Texas Instruments Incorporated
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
 * ======== socket.h ========
 *
 * NDK legacy socket support
 *
 */

#ifndef _SOCKET_H_
#define _SOCKET_H_

/* include NDK's socket API */
#include "socketndk.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef NDK_NOUSERAPIS

/* Define legacy socket compatibility layer */

/*
 * ======== accept ========
 */
static inline SOCKET accept(SOCKET s, struct sockaddr *pName, int *plen)
{
    return NDK_accept(s, pName, plen);
}

/*
 * ======== bind ========
 */
static inline int bind(SOCKET s, struct sockaddr *pName, int len)
{
    return NDK_bind(s, pName, len);
}

/*
 * ======== connect ========
 */
static inline int connect(SOCKET s, struct sockaddr *pName, int len)
{
    return NDK_connect(s, pName, len);
}

/*
 * ======== getpeername ========
 */
static inline int getpeername(SOCKET s, struct sockaddr *pName, int *plen)
{
    return NDK_getpeername(s, pName, plen);
}

/*
 * ======== getsendncbuff ========
 */
static inline int getsendncbuff(SOCKET s, uint32_t bufSize, void **phBuf,
        void **phPkt)
{
    return NDK_getsendncbuff(s, bufSize, phBuf, phPkt);
}

/*
 * ======== getsockname ========
 */
static inline int getsockname(SOCKET s, struct sockaddr *pName, int *plen)
{
    return NDK_getsockname(s, pName, plen);
}

/*
 * ======== getsockopt ========
 */
static inline int getsockopt(SOCKET s, int level, int op, void *pbuf,
        int *pbufsize)
{
    return NDK_getsockopt(s, level, op, pbuf, pbufsize);
}

/*
 * ======== listen ========
 */
static inline int listen(SOCKET s, int maxcon)
{
    return NDK_listen(s, maxcon);
}

/*
 * ======== recv ========
 */
static inline int recv(SOCKET s, void *pbuf, int size, int flags)
{
    return NDK_recv(s, pbuf, size, flags);
}

/*
 * ======== recvfrom ========
 */
static inline int recvfrom(SOCKET s, void *pbuf, int size, int flags, struct sockaddr *pName,
        int *plen)
{
    return NDK_recvfrom(s, pbuf, size, flags, pName, plen);
}

/*
 * ======== recvnc ========
 */
static inline int recvnc(SOCKET s, void **ppbuf, int flags, void **pHandle)
{
    return NDK_recvnc(s, ppbuf, flags, pHandle);
}

/*
 * ======== recvfromnc ========
 */
static inline int recvncfrom(SOCKET s, void **ppbuf, int flags, struct sockaddr *pName,
        int *plen, void **pHandle)
{
    return NDK_recvncfrom(s, ppbuf, flags, pName, plen, pHandle);
}

/*
 * ======== recvncfree ========
 */
static inline void recvncfree(SOCKET Handle)
{
    NDK_recvncfree(Handle);
}

/*
 * ======== send ========
 */
static inline int send(SOCKET s, void *pbuf, int size, int flags)
{
    return NDK_send(s, pbuf, size, flags);
}

/*
 * ======== sendto ========
 */
static inline int sendto(SOCKET s, void *pbuf, int size, int flags, struct sockaddr *pName,
        int len)
{
    return NDK_sendto(s, pbuf, size, flags, pName, len);
}

/*
 * ======== sendnc ========
 */
static inline int sendnc(SOCKET s, void *pbuf, int size, void *hPkt, int flags)
{
    return NDK_sendnc(s, pbuf, size, hPkt, flags);
}

/*
 * ======== sendncfree ========
 */
static inline void sendncfree(SOCKET Handle)
{
    NDK_sendncfree(Handle);
}

/*
 * ======== setsockopt ========
 */
static inline int setsockopt(SOCKET s, int level, int op, void *pbuf,
        int bufsize)
{
    return NDK_setsockopt(s, level, op, pbuf, bufsize);
}

/*
 * ======== shutdown ========
 */
static inline int shutdown(SOCKET s, int how)
{
    return NDK_shutdown(s, how);
}

/*
 * ======== socket ========
 */
static inline SOCKET socket(int domain, int type, int protocol)
{
    return NDK_socket(domain, type, protocol);
}

#endif

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
