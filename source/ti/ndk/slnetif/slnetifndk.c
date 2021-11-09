/*
 * Copyright (c) 2017-2019, Texas Instruments Incorporated - http://www.ti.com
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


/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include <stddef.h>
#include <stdint.h>

#include <netmain.h>
#include <stkmain.h>
#include <stdlib.h>

#include <assert.h>

#include "slnetifndk.h"
#include "slnetifndk_internal.h"

#include <ti/net/slnetif.h>
#include <ti/net/slnetsock.h>
#include <ti/net/slneterr.h>
#include <ti/ndk/inc/stack/inc/nimuif.h>

#include <semaphore.h>

#ifdef SLNETIFNDK_ENABLEMBEDTLS
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/certs.h"
#include "mbedtls/x509.h"
#include "mbedtls/ssl_cookie.h"
#include "mbedtls/timing.h"
#include "mbedtls/net_sockets.h"
#include <entropy_alt.h>
#endif

/*****************************************************************************/
/* Macro declarations                                                        */
/*****************************************************************************/
#define MAX_SEC_OBJS 20  /* Maximum number of security objects per interface */

/* large enough for IPv6 & v4 */
/* needed since _INCLUDE_IPv6_CODE may not be thrown */
#define MAXADDRLEN 50

/* minimum RSA bit length supported in certificates */
#define RSA_MIN_BITLEN  1024

/*
 * The implementation assumes the various SLNETSOCK_MSG_ flags passed
 * to SlNetSock send/recv (and friends) are equivalent to the NDK
 * definitions.  Yeah, it's a cheat... but we'll call it efficient
 * instead.
 *
 * Here we defensively ensure (at compile time) that this assumption
 * is valid.  If these #errors ever fire, it means there is likely
 * flag-translation work needed.
 */
#if (SLNETSOCK_MSG_OOB != MSG_OOB)
#error Conflicting definitions of MSG_OOB
#endif
#if (SLNETSOCK_MSG_PEEK != MSG_PEEK)
#error Conflicting definitions of MSG_PEEK
#endif
#if (SLNETSOCK_MSG_WAITALL != MSG_WAITALL)
#error Conflicting definitions of MSG_WAITALL
#endif
#if (SLNETSOCK_MSG_DONTWAIT != MSG_DONTWAIT)
#error Conflicting definitions of MSG_DONTWAIT
#endif
#if (SLNETSOCK_MSG_DONTROUTE != MSG_DONTROUTE)
#error Conflicting definitions of MSG_DONTROUTE
#endif
#if (SLNETSOCK_MSG_NOSIGNAL != MSG_NOSIGNAL)
#error Conflicting definitions of MSG_NOSIGNAL
#endif

/*****************************************************************************/
/* Structure/Enum declarations                                               */
/*****************************************************************************/
typedef struct {
    char *name;
    int16_t nameLen;
    uint8_t *buf;
    int16_t bufLen;
} _SlNetIfNDK_SecObj;

typedef struct {
    int physIf;
    char name[MAX_INTERFACE_NAME_LEN];
    _SlNetIfNDK_SecObj secObjList[MAX_SEC_OBJS];
} _SlNetIfNDK_IfContext;

typedef struct {
    bool isSecure;
    bool initKey;
    bool initLocalCert;
    bool initCaCert;
    bool initConf;
    bool initCookie;
#ifdef SLNETIFNDK_ENABLEMBEDTLS
    mbedtls_ssl_context *ssl;
    mbedtls_ssl_config conf;
    mbedtls_x509_crt localCert;
    mbedtls_x509_crt caCert;
    mbedtls_x509_crt_profile crtProfile;
    mbedtls_pk_context key;
    mbedtls_ssl_cookie_ctx cookieCtx;
    mbedtls_timing_delay_context timer;
#endif
    int clientSd;
    uint16_t type;
    _SlNetIfNDK_IfContext *ifCtx;
    char *domainName;
} _SlNetIfNDK_SocketContext;

/*****************************************************************************/
/* Global declarations                                                       */
/*****************************************************************************/
extern uint32_t ti_ndk_socket_max_fd;
extern SOCKET ti_ndk_socket_fdtable[];
extern NDK_HookFxn NDK_createSockCtx;

/*
 * SlNetIfConfigNDK structure contains all the function callbacks that are
 * expected to be filled by the relevant network stack interface
 * Each interface has different capabilities, so not all the API's must be
 * supported. Interface that is not supporting a non-mandatory API are set to
 * NULL
 */
SlNetIf_Config_t SlNetIfConfigNDK =
{
    SlNetIfNDK_socket,     /* sockCreate */
    SlNetIfNDK_close,      /* sockClose */
    SlNetIfNDK_shutdown,   /* sockShutdown */
    SlNetIfNDK_accept,     /* sockAccept */
    SlNetIfNDK_bind,       /* sockBind */
    SlNetIfNDK_listen,     /* sockListen */
    SlNetIfNDK_connect,    /* sockConnect */
    SlNetIfNDK_getPeerName,/* sockGetPeerName */
    SlNetIfNDK_getSockName,/* sockGetLocalName */
    SlNetIfNDK_select,     /* sockSelect */
    SlNetIfNDK_setSockOpt, /* sockSetOpt */
    SlNetIfNDK_getSockOpt, /* sockGetOpt */
    SlNetIfNDK_recv,       /* sockRecv */
    SlNetIfNDK_recvFrom,   /* sockRecvFrom */
    SlNetIfNDK_send,                /* sockSend */
    SlNetIfNDK_sendTo,              /* sockSendTo */
    NULL,                           /* sockstartSec */
    SlNetIfNDK_getHostByName,       /* utilGetHostByName */
    SlNetIfNDK_getIPAddr,           /* ifGetIPAddr */
    SlNetIfNDK_getConnectionStatus, /* ifGetConnectionStatus */
    NULL,                           /* ifLoadSecObj */
    SlNetIfNDK_ifCreateContext      /* ifCreateContext */
};

#ifdef SLNETIFNDK_ENABLEMBEDTLS
/*
 * SlNetIfConfigNDKSec structure contains all the function callbacks that are
 * expected to be filled by the relevant network stack interface
 * in order to deal with both secure and non-secure connections.
 */
SlNetIf_Config_t SlNetIfConfigNDKSec =
{
    SlNetIfNDK_socket,     /* sockCreate */
    SlNetIfNDK_closeSec,   /* sockClose */
    SlNetIfNDK_shutdown,   /* sockShutdown */
    SlNetIfNDK_accept,     /* sockAccept */
    SlNetIfNDK_bind,       /* sockBind */
    SlNetIfNDK_listen,     /* sockListen */
    SlNetIfNDK_connect,    /* sockConnect */
    SlNetIfNDK_getPeerName,/* sockGetPeerName */
    SlNetIfNDK_getSockName,/* sockGetLocalName */
    SlNetIfNDK_select,     /* sockSelect */
    SlNetIfNDK_setSockOpt, /* sockSetOpt */
    SlNetIfNDK_getSockOpt, /* sockGetOpt */
    SlNetIfNDK_recvSec,             /* sockRecv */
    SlNetIfNDK_recvFromSec,         /* sockRecvFrom */
    SlNetIfNDK_sendSec,             /* sockSend */
    SlNetIfNDK_sendToSec,           /* sockSendTo */
    SlNetIfNDK_sockStartSec,        /* sockstartSec */
    SlNetIfNDK_getHostByName,       /* utilGetHostByName */
    SlNetIfNDK_getIPAddr,           /* ifGetIPAddr */
    SlNetIfNDK_getConnectionStatus, /* ifGetConnectionStatus */
    SlNetIfNDK_loadSecObj,          /* ifLoadSecObj */
    SlNetIfNDK_ifCreateContext      /* ifCreateContext */
};
#endif /* SLNETIFNDK_ENABLEMBEDTLS */

static sem_t SlNetIfNDK_sem;

/*
 * IF ID translation table. Index is the SlNetSock IF number, element stored is
 * the NDK IF number.
 */
/*
 * TODO: update this to store pointers to the  _SlNetIfNDK_SocketContext
 *       objects. We can get the same NDK IF ID information from the context, as
 *       well as being able to access everything else in the object, which may
 *       come in handy one day.
 */
static uint32_t SlNetIfNDK_ifTable[SLNETIF_MAX_IF];

/*****************************************************************************/
/* Function definitions                                                      */
/*****************************************************************************/
#ifdef SLNETIFNDK_ENABLEMBEDTLS
/*
 *  ======== netSend ========
 *  Function used to satisfy mbedtls_ssl_set_bio()
 */
static int netSend(void *ctx, const unsigned char *buf, size_t len)
{
    int ret;
    int sd;
    int error;

    assert(buf != NULL);

    if (!ctx) {
        return (MBEDTLS_ERR_NET_INVALID_CONTEXT);
    }

    sd = *((int *) ctx);

    ret = NDK_send(ti_ndk_socket_fdtable[sd], (void *)buf, len, 0);
    if (ret < 0) {
        error = fdError();
        if (error == NDK_EAGAIN) {
            ret = MBEDTLS_ERR_SSL_WANT_WRITE;
        }
        else {
            ret = MBEDTLS_ERR_NET_SEND_FAILED;
        }
    }

    return (ret);
}

/*
 *  ======== netRecv ========
 *  Function used to satisfy mbedtls_ssl_set_bio()
 */
static int netRecv(void *ctx, unsigned char *buf, size_t len)
{
    int ret;
    int sd;
    int error;

    assert(buf != NULL);

    if (!ctx) {
        return (MBEDTLS_ERR_NET_INVALID_CONTEXT);
    }

    sd = *((int *) ctx);

    ret = NDK_recv(ti_ndk_socket_fdtable[sd], buf, len, 0);
    if (ret < 0) {
        error = fdError();
        if (error == NDK_EAGAIN) {
            ret = MBEDTLS_ERR_SSL_WANT_READ;
        }
        else {
            ret = MBEDTLS_ERR_NET_RECV_FAILED;
        }
    }

    return (ret);
}
#endif /* SLNETIFNDK_ENABLEMBEDTLS */

/*
 *  ======== getBSDSlNetErr ========
 *  Translate fdError() to the appropriate BSD SlNetErr code
 */
static int16_t getBSDSlNetErr()
{
    int16_t ret;

    switch (fdError()) {
        case NDK_EPFNOSUPPORT:
            ret = SLNETERR_BSD_EAFNOSUPPORT;
            break;
        case NDK_EPROTOTYPE:
            ret = SLNETERR_BSD_EPROTOTYPE;
            break;
        case NDK_EACCES:
            ret = SLNETERR_BSD_EACCES;
            break;
        case NDK_ENOMEM:
            ret = SLNETERR_BSD_ENOMEM;
            break;
        case NDK_EINVAL:
            ret = SLNETERR_BSD_EINVAL;
            break;
        case NDK_EOPNOTSUPP:
            ret = SLNETERR_BSD_EOPNOTSUPP;
            break;
        case NDK_EAGAIN:  /* NDK_EWOULDBLOCK */
            ret = SLNETERR_BSD_EAGAIN;
            break;
        case NDK_ECONNREFUSED:
            ret = SLNETERR_BSD_ECONNREFUSED;
            break;
        case NDK_ECONNRESET:
            ret = SLNETERR_BSD_ECONNRESET;
            break;
        case NDK_ETIMEDOUT:
            ret = SLNETERR_BSD_ETIMEDOUT;
            break;
        case NDK_EHOSTUNREACH:
            ret = SLNETERR_BSD_EHOSTUNREACH;
            break;
        case NDK_EHOSTDOWN:
            ret = SLNETERR_BSD_EHOSTDOWN;
            break;
        case NDK_EADDRNOTAVAIL:
            ret = SLNETERR_BSD_EADDRNOTAVAIL;
            break;
        case NDK_EBADF:
            ret = SLNETERR_BSD_EBADF;
            break;
        case NDK_ESOCKTNOSUPPORT:
            ret = SLNETERR_BSD_ESOCKTNOSUPPORT;
            break;
        case NDK_ENOTCONN:
            ret = SLNETERR_BSD_ENOTCONN;
            break;
        case NDK_ENOTSOCK:
            ret = SLNETERR_BSD_ENOTSOCK;
            break;
        case NDK_ENOPROTOOPT:
            ret = SLNETERR_BSD_ENOPROTOOPT;
            break;
        case NDK_ENXIO:
            ret = SLNETERR_BSD_ENXIO;
            break;
        case NDK_ENOBUFS:
            ret = SLNETERR_BSD_ENOBUFS;
            break;
        case NDK_EMSGSIZE:
            ret = SLNETERR_BSD_EMSGSIZE;
            break;
        case NDK_ESHUTDOWN:
            ret = SLNETERR_BSD_ESHUTDOWN;
            break;
        case NDK_EISCONN:
            ret = SLNETERR_BSD_EISCONN;
            break;
        case NDK_ECONNABORTED:
            ret = SLNETERR_BSD_ECONNABORTED;
            break;
        case NDK_EADDRINUSE:
            ret = SLNETERR_BSD_EADDRINUSE;
            break;
        case NDK_ENETDOWN:
            ret = SLNETERR_BSD_ENETDOWN;
            break;
        case NDK_ENFILE:
            ret = SLNETERR_BSD_ENSOCK;
            break;
        case NDK_EDOM:
            ret = SLNETERR_BSD_EDOM;
            break;
        case NDK_EALREADY:
            ret = SLNETERR_BSD_EALREADY;
            break;
        default:
            ret = SLNETERR_UNKNOWN_ERR;
            break;
    }

    return (ret);
}

/*
 *  ======== addSktToTable ========
 *  Find and empty spot in the fd table to put this socket.
 *  Returns the file descriptor (array index) or -1 if the table is full.
 */
static int16_t addSktToTable(uintptr_t s)
{
    uint16_t i;

    for (i = 0; i < ti_ndk_socket_max_fd; i++) {
        if (ti_ndk_socket_fdtable[i] == NULL) {
            ti_ndk_socket_fdtable[i] = (SOCKET)s;
            return ((int16_t)i);
        }
    }
    return (-1);
}

/*
 *  ======== removeSktFromTable ========
 *  Remove a socket from the fd table. If sockets's ctx is not valid that is
 *  okay because the socket was likely intentionally made without a ctx and
 *  thus never placed on the fd table.
 */
static int16_t removeSktFromTable(uintptr_t s)
{
    int ctx = NDK_getSockCtx((SOCKET)s);

    if(ctx >= 0 && ctx < ti_ndk_socket_max_fd) {
        ti_ndk_socket_fdtable[ctx] = NULL;
    }

    return (0);
}

/*
 *  ======== findSlNetSockIf ========
 *  Given the NDK interface ID, find the corresponding SlNetSock interface ID.
 *  Returns the SlNetSock IF number on success or 0 if not found.
 */
static uint32_t findSlNetSockIf(uint32_t ndkIf)
{
    uint32_t i;

    /* Valid indexes start at 1 */
    for (i = 1; i < SLNETIF_MAX_IF; i++) {
        if (SlNetIfNDK_ifTable[i] == ndkIf) {
            return (i);
        }
    }
    return (0);
}

/******************************************************************************
 *
 * SlNetIfNDK_socket - Create an endpoint for communication
 *
 *****************************************************************************/
int16_t SlNetIfNDK_socket(void *ifContext, int16_t domain, int16_t type,
        int16_t protocol, void **psdContext)
{
    int32_t  ret = SLNETERR_RET_CODE_OK;
    SOCKET s;
    int ndkDomain;
    int ndkType;
    int ndkProto;

    _SlNetIfNDK_SocketContext *sCtxPtr = calloc(1,
            sizeof(_SlNetIfNDK_SocketContext));
    if (sCtxPtr == NULL) {
        ret = SLNETERR_BSD_ENOMEM;
    }
    else {
        sCtxPtr->ifCtx = (_SlNetIfNDK_IfContext *)ifContext;
        sCtxPtr->type = type;
        sCtxPtr->domainName = NULL;
        assert(psdContext != NULL);
        *psdContext = sCtxPtr;
    }

    switch (domain) {
        case SLNETSOCK_AF_INET:
            ndkDomain = AF_INET;
            break;
        case SLNETSOCK_AF_INET6:
            ndkDomain = AF_INET6;
            break;
        default:
            ret = SLNETERR_BSD_EAFNOSUPPORT;
    }

    switch (type) {
        case SLNETSOCK_SOCK_STREAM:
            ndkType = SOCK_STREAM;
            break;
        case SLNETSOCK_SOCK_DGRAM:
            ndkType = SOCK_DGRAM;
            break;
        case SLNETSOCK_SOCK_RAW:
            ndkType = SOCK_RAW;
            break;
        default:
            ret = SLNETERR_BSD_ESOCKTNOSUPPORT;
    }

    switch (protocol) {
        case SLNETSOCK_PROTO_TCP:
            ndkProto = IPPROTO_TCP;
            break;
        case SLNETSOCK_PROTO_UDP:
            ndkProto = IPPROTO_UDP;
            break;
        case SLNETSOCK_PROTO_RAW:
        case SLNETSOCK_PROTO_SECURE:
        default:
            ret = SLNETERR_BSD_EPROTONOSUPPORT;
    }

    if (ret == SLNETERR_RET_CODE_OK) {
        s = NDK_socket(ndkDomain, ndkType, ndkProto);
        if (s == INVALID_SOCKET) {
            ret = getBSDSlNetErr();
        }
        else {
            ret = NDK_getSockCtx(s);

            /* This means the socket never got a context, so we can't use it */
            if (ret == NDK_NO_CTX) {
                fdClose(s);
            }
        }
    }

    return (ret);
}


/******************************************************************************
 *
 * SlNetIfNDK_close - Gracefully close socket
 *
 *****************************************************************************/
int32_t SlNetIfNDK_close(int16_t sd, void *sdContext)
{
    int rc;

    assert(sdContext != NULL);
    assert(sd >= 0);
    assert((uint16_t)sd < ti_ndk_socket_max_fd);

    rc = fdClose(ti_ndk_socket_fdtable[sd]);
    if (rc == -1) {
        rc = SLNETERR_RET_CODE_FUNCTION_FAILED;
    }

    return (rc);
}

/******************************************************************************
 *
 * SlNetIfNDK_shutdown - shut down all or part of a socket
 *
 *****************************************************************************/
int32_t SlNetIfNDK_shutdown(int16_t sd, void *sdContext, int16_t how)
{
    int status;
    int32_t ndkHow;

    assert(sdContext != NULL);
    assert(sd >= 0);

    /* Translate SlNetSock values to NDK values */
    switch (how) {
        case SLNETSOCK_SHUT_RD:
            ndkHow = SHUT_RD;
            break;
        case SLNETSOCK_SHUT_WR:
            ndkHow = SHUT_WR;
            break;
        case SLNETSOCK_SHUT_RDWR:
            ndkHow = SHUT_RDWR;
            break;
        default:
            /* Invalid argument passed */
            return (SLNETERR_BSD_EINVAL);
    }

    status = NDK_shutdown(ti_ndk_socket_fdtable[sd], ndkHow);

    if (status == SOCKET_ERROR) {
        /* Translate error code from an NDK value to an SlNetSock value */
        status = getBSDSlNetErr();
    }

    return (status);
}

#ifdef SLNETIFNDK_ENABLEMBEDTLS
/******************************************************************************
 *
 * SlNetIfNDK_closeSec - Gracefully close socket
 *
 *****************************************************************************/
int32_t SlNetIfNDK_closeSec(int16_t sd, void *sdContext)
{
    int rc;
    _SlNetIfNDK_SocketContext *sCtxPtr = sdContext;

    assert(sdContext != NULL);
    assert(sd >= 0);
    assert((uint16_t)sd < ti_ndk_socket_max_fd);

    if (sCtxPtr->isSecure) {
        mbedtls_ssl_close_notify(sCtxPtr->ssl);
        mbedtls_ssl_free(sCtxPtr->ssl);
    }

    rc = SlNetIfNDK_close(sd, sdContext);

    if (sCtxPtr->isSecure) {
        free(sCtxPtr->ssl);
        if (sCtxPtr->initCookie) {
            mbedtls_ssl_cookie_free(&sCtxPtr->cookieCtx);
        }
        if (sCtxPtr->initLocalCert) {
            mbedtls_x509_crt_free(&sCtxPtr->localCert);
        }
        if (sCtxPtr->initCaCert) {
            mbedtls_x509_crt_free(&sCtxPtr->caCert);
        }
        if (sCtxPtr->initKey) {
            mbedtls_pk_free(&sCtxPtr->key);
        }
        if (sCtxPtr->initConf) {
            mbedtls_ssl_config_free(&sCtxPtr->conf);
        }
        if (sCtxPtr->domainName != NULL) {
            free(sCtxPtr->domainName);
        }
    }
    return (rc);
}
#endif /* SLNETIFNDK_ENABLEMBEDTLS */

/******************************************************************************
 *
 * SlNetIfNDK_accept - Accept a connection on a socket
 *
 *****************************************************************************/
int16_t SlNetIfNDK_accept(int16_t sd, void *sdContext, SlNetSock_Addr_t *addr,
        SlNetSocklen_t *addrlen, uint8_t flags, void **acceptedSdContext)
{
    int32_t ret = SLNETERR_RET_CODE_OK;
    SOCKET s;
    _SlNetIfNDK_SocketContext *sCtxPtr = sdContext;
    _SlNetIfNDK_SocketContext *acceptedCtxPtr;
    int len;
    int newFd;

    assert(acceptedSdContext != NULL);
    assert(sdContext != NULL);
    assert(sd >= 0);
    assert((uint16_t)sd < ti_ndk_socket_max_fd);

    if (addrlen != NULL) {
        len = *addrlen;
        s = NDK_accept(ti_ndk_socket_fdtable[sd], (struct sockaddr *)addr,
                &len);
    }
    else {
        /* No need to check addr since NDK_accept does it */
        s = NDK_accept(ti_ndk_socket_fdtable[sd], (struct sockaddr *)addr,
                NULL);
    }

    if (s == INVALID_SOCKET) {
        ret = getBSDSlNetErr();
    }
    else {
        newFd = NDK_getSockCtx(s);
        /* This means the socket never got a context, so we can't use it */
        if(newFd == NDK_NO_CTX) {
            fdClose(s);
            return newFd;
        }
    }

    if (ret == SLNETERR_RET_CODE_OK) {
        acceptedCtxPtr = calloc(1, sizeof(_SlNetIfNDK_SocketContext));
        if (acceptedCtxPtr == NULL) {
            ret = SLNETERR_BSD_ENOMEM;
        }
        else {
            acceptedCtxPtr->ifCtx = sCtxPtr->ifCtx;
            acceptedCtxPtr->type = sCtxPtr->type;

#ifdef SLNETIFNDK_ENABLEMBEDTLS
            /*
             * Can create SlNetIfNDK_acceptSec and remove this memcpy from
             * SlNetIfNDK_accept, but let's keep one version for now for
             * simplicity.
             */
            memcpy(&acceptedCtxPtr->conf, &sCtxPtr->conf, sizeof(sCtxPtr->conf));
#endif /* SLNETIFNDK_ENABLEMBEDTLS */

            *acceptedSdContext = acceptedCtxPtr;
        }
    }

    if(ret == SLNETERR_RET_CODE_OK) {
        if (addrlen != NULL) {
            *addrlen = len; /* TODO: might result in truncation? */
        }
        ret = newFd;
    }

    return (ret);
}


/*******************************************************************************
 *
 * SlNetIfNDK_bind - Assign a name to a socket
 *
 ******************************************************************************/
int32_t SlNetIfNDK_bind(int16_t sd, void *sdContext,
        const SlNetSock_Addr_t *addr, int16_t addrlen)
{
    int32_t ret = SLNETERR_RET_CODE_OK;

    assert(sd >= 0);
    assert((uint16_t)sd < ti_ndk_socket_max_fd);

    ret = NDK_bind(ti_ndk_socket_fdtable[sd], (struct sockaddr *)addr, addrlen);

    if(ret == SOCKET_ERROR)
    {
        ret = getBSDSlNetErr();
    }

    return (ret);
}


/******************************************************************************
 *
 * SlNetIfNDK_listen - Listen for connections on a socket
 *
 *****************************************************************************/
int32_t SlNetIfNDK_listen(int16_t sd, void *sdContext, int16_t backlog)
{
    int32_t ret = SLNETERR_RET_CODE_OK;

    assert(sd >= 0);
    assert((uint16_t)sd < ti_ndk_socket_max_fd);

    ret = NDK_listen(ti_ndk_socket_fdtable[sd], backlog);

    if(ret == SOCKET_ERROR)
    {
        ret = getBSDSlNetErr();
    }

    return (ret);
}


/******************************************************************************
 *
 * SlNetIfNDK_connect - Initiate a connection on a socket
 *
 *****************************************************************************/
int32_t SlNetIfNDK_connect(int16_t sd, void *sdContext,
        const SlNetSock_Addr_t *addr, SlNetSocklen_t addrlen, uint8_t flags)
{
    SlNetSock_SockAddrStorage_t ndkAddrStorage;
    SlNetSock_AddrIn6_t *ndkAddr6;
    SlNetSock_AddrIn_t *ndkAddr;
    int      status;
    int32_t  ret = SLNETERR_RET_CODE_OK;
    uint32_t slNetSockIf;

    assert(addr != NULL);
    assert(sd >= 0);
    assert((uint16_t)sd < ti_ndk_socket_max_fd);

    /*
     * The 'addr' address passed in contains SlNetSock values. Translate these
     * into NDK values:
     */
    memset(&ndkAddrStorage, 0, sizeof(SlNetSock_SockAddrStorage_t));
    if (addr->sa_family == SLNETSOCK_AF_INET6) {
        /* Reference our generic storage space as an IPv6 address struct */
        ndkAddr6 = (SlNetSock_AddrIn6_t *)&ndkAddrStorage;

        /* Need a local copy of 'addr', as we have to modify its contents */
        *ndkAddr6 = *((const SlNetSock_AddrIn6_t *)addr);

        /* Translate family from an SlNetSock value to an NDK value */
        ndkAddr6->sin6_family = AF_INET6;

        /*
         * The scope id coming into this function call is an SlNetSock IF
         * number. Translate it to the underlying NDK IF number:
         */
        slNetSockIf = ((const SlNetSock_AddrIn6_t *)addr)->sin6_scope_id;
        ndkAddr6->sin6_scope_id = SlNetIfNDK_ifTable[slNetSockIf];
    }
    else if (addr->sa_family == SLNETSOCK_AF_INET) {
        /* Reference our generic storage space as an IPv4 address struct */
        ndkAddr = (SlNetSock_AddrIn_t *)&ndkAddrStorage;

        /* Need a local copy of 'to', as we have to modify its contents */
        *ndkAddr = *((const SlNetSock_AddrIn_t *)addr);

        /* Translate family from an SlNetSock value to an NDK value */
        ndkAddr->sin_family = AF_INET;
    }

    /*
     * Note: if family is invalid, then NDK_connect will be called with a
     * zeroed out sockaddr struct. This will be caught and handled internally
     * in the stack code, so omitting the check here.
     */

    status = NDK_connect(ti_ndk_socket_fdtable[sd],
            (struct sockaddr *)&ndkAddrStorage, addrlen);

    if (status < 0) {
        ret = getBSDSlNetErr();
    }

    return (ret);
}

/******************************************************************************
 *
 * SlNetIfNDK_getPeerName - Returns the address info of the peer connected to
 *                          the socket descriptor
 *
 *****************************************************************************/
int32_t SlNetIfNDK_getPeerName(int16_t sd, void *sdContext,
        SlNetSock_Addr_t *addr, SlNetSocklen_t *addrlen)
{
    int len;
    int status;

    assert(addr != NULL);
    assert(addrlen != NULL);
    assert(sd >= 0);
    assert((uint16_t)sd < ti_ndk_socket_max_fd);

    len = *addrlen;

    status = NDK_getpeername(ti_ndk_socket_fdtable[sd], (struct sockaddr *)addr,
            &len);

    if (status != SOCKET_ERROR) {
        /* Translate family from an NDK value to an SlNetSock value */
        switch (addr->sa_family) {
            case AF_INET:
                addr->sa_family = SLNETSOCK_AF_INET;
                break;
            case AF_INET6:
                addr->sa_family = SLNETSOCK_AF_INET6;
                break;
            default:
                /* Unsupported family. Not really an error, leave value as is */
                break;
        }

        *addrlen = len; /* TODO: might result in truncation? */
    }
    else {
        /* Translate error code from an NDK value to an SlNetSock value */
        status = getBSDSlNetErr();
    }

    return (status);
}


/******************************************************************************
 *
 * SlNetIfNDK_getSockName - Returns the local address info of the socket
 *                         descriptor
 *
 *****************************************************************************/
int32_t SlNetIfNDK_getSockName(int16_t sd, void *sdContext,
        SlNetSock_Addr_t *addr, SlNetSocklen_t *addrlen)
{
    int len;
    int status;

    assert(addr != NULL);
    assert(addrlen != NULL);
    assert(sd >= 0);
    assert((uint16_t)sd < ti_ndk_socket_max_fd);

    len = *addrlen;

    status = NDK_getsockname(ti_ndk_socket_fdtable[sd], (struct sockaddr *)addr,
            &len);

    if (status != SOCKET_ERROR) {
        /* Translate family from an NDK value to an SlNetSock value */
        switch (addr->sa_family) {
            case AF_INET:
                addr->sa_family = SLNETSOCK_AF_INET;
                break;
            case AF_INET6:
                addr->sa_family = SLNETSOCK_AF_INET6;
                break;
            default:
                /* Unsupported family. Not really an error, leave value as is */
                break;
        }

        *addrlen = len; /* TODO: might result in truncation? */
    }
    else {
        /* Translate error code from an NDK value to an SlNetSock value */
        status = getBSDSlNetErr();
    }

    return (status);
}


/******************************************************************************
 *
 * SlNetIfNDK_select - Monitor socket activity
 *
 *****************************************************************************/
int32_t SlNetIfNDK_select(void *ifContext, int16_t nfds,
        SlNetSock_SdSet_t *readsds, SlNetSock_SdSet_t *writesds,
        SlNetSock_SdSet_t *exceptsds, SlNetSock_Timeval_t *timeout)
{
    uint32_t currNdkFd;
    int16_t numFds;
    NDK_fd_set ndkReadSds;
    NDK_fd_set ndkWriteSds;
    NDK_fd_set ndkExceptSds;

    NDK_FD_ZERO(&ndkReadSds);
    NDK_FD_ZERO(&ndkWriteSds);
    NDK_FD_ZERO(&ndkExceptSds);

    /* find the NDK sockets that've been set in the SlNetSock_SdSet_t objs */
    for (currNdkFd = 0; currNdkFd < ti_ndk_socket_max_fd; currNdkFd++) {

        /* check if readsds is set and if so, translate to NDK */
        if ((readsds != NULL) && (SlNetSock_sdsIsSet(currNdkFd, readsds))) {
            NDK_FD_SET(ti_ndk_socket_fdtable[currNdkFd], &ndkReadSds);
        }

        /* check if writesds is set and if so, translate to NDK */
        if ((writesds != NULL) && (SlNetSock_sdsIsSet(currNdkFd, writesds))) {
            NDK_FD_SET(ti_ndk_socket_fdtable[currNdkFd], &ndkWriteSds);
        }

        /* check if exceptsds is set and if so, translate to NDK */
        if ((exceptsds != NULL) && (SlNetSock_sdsIsSet(currNdkFd, exceptsds))) {
            NDK_FD_SET(ti_ndk_socket_fdtable[currNdkFd], &ndkExceptSds);
        }

    }

    numFds = fdSelect(nfds, &ndkReadSds, &ndkWriteSds, &ndkExceptSds,
            (struct timeval *)timeout);
    if (numFds < 0) {
        return (getBSDSlNetErr());
    }

    /* Return the SdSets */
    for (currNdkFd = 0; currNdkFd < ti_ndk_socket_max_fd; currNdkFd++) {

        /* check if readsds is set and if so, clear it if it's not ready */
        if ((readsds != NULL) && (SlNetSock_sdsIsSet(currNdkFd, readsds))) {
            if (!NDK_FD_ISSET(ti_ndk_socket_fdtable[currNdkFd], &ndkReadSds)) {
                SlNetSock_sdsClr(currNdkFd, readsds);
            }
        }

        /* check if writesds is set and if so, clear it if it's not ready */
        if ((writesds != NULL) && (SlNetSock_sdsIsSet(currNdkFd, writesds))) {
            if (!NDK_FD_ISSET(ti_ndk_socket_fdtable[currNdkFd], &ndkWriteSds)) {
                SlNetSock_sdsClr(currNdkFd, writesds);
            }
        }

        /* check if exceptsds is set and if so, clear it if it's not ready */
        if ((exceptsds != NULL) && (SlNetSock_sdsIsSet(currNdkFd, exceptsds))) {
            if (!NDK_FD_ISSET(ti_ndk_socket_fdtable[currNdkFd], &ndkExceptSds)) {
                SlNetSock_sdsClr(currNdkFd, exceptsds);
            }
        }

    }

    return (numFds);
}


/******************************************************************************
 *
 * SlNetIfNDK_setSockOpt - Set socket options
 *
 *****************************************************************************/
int32_t SlNetIfNDK_setSockOpt(int16_t sd, void *sdContext, int16_t level,
        int16_t optname, void *optval, SlNetSocklen_t optlen)
{
    int status;
    int ndkLevel;
    SlNetSock_Winsize_t *slSize;
    int winSize;
    SlNetSock_Timeval_t *slTimeVal;
    struct timeval tv;
    SlNetSock_Keepalive_t *slKAEnableOption;
    int keepaliveEnabled;
    SlNetSock_linger_t *slLinger;
    struct linger ndkLinger;
    SlNetSock_Nonblocking_t *slNBEnableOption;
    int blocking;
    SlNetSock_Broadcast_t *slBroadcastEnableOption;
    int broadcastOn;
    SlNetSock_NoDelay_t *slNoDelay;
    int noDelay;
    SlNetSock_MaxSeg_t *slMaxSeg;
    int maxSeg;
    SlNetSock_NoPush_t *slNoPush;
    int noPush;
    SlNetSock_NoOpt_t *slNoOpt;
    int noOpt;
    SlNetSock_SackPermitted_t *slSackPermitted;
    int sackPermitted;
    SlNetSock_MaxRtt_t *slMaxRtt;
    int maxRtt;
    uint32_t *slHops;
    int hopsVal;
    SlNetSock_IpMreq_t *slMreq;
    struct ip_mreq ipMreq;
    SlNetSock_IpV6Mreq_t *slv6Mreq;
    struct ipv6_mreq ipv6Mreq;
    uint32_t *slHeader;
    int h;

    assert(optval != NULL);
    assert(sd >= 0);
    assert((uint16_t)sd < ti_ndk_socket_max_fd);

    switch (level) {
        case SLNETSOCK_LVL_SOCKET:
            ndkLevel = SOL_SOCKET;
            switch (optname) {
                case SLNETSOCK_OPSOCK_RCV_BUF:
                    slSize = optval;
                    winSize = slSize->winSize;
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            SO_RCVBUF, &winSize, sizeof(int));
                    break;
                case SLNETSOCK_OPSOCK_RCV_TIMEO:
                    slTimeVal = optval;
                    tv.tv_sec = slTimeVal->tv_sec;
                    tv.tv_usec = slTimeVal->tv_usec;
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            SO_RCVTIMEO, &tv, sizeof(struct timeval));
                    break;
                case SLNETSOCK_OPSOCK_SND_TIMEO:
                    slTimeVal = optval;
                    tv.tv_sec = slTimeVal->tv_sec;
                    tv.tv_usec = slTimeVal->tv_usec;
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            SO_SNDTIMEO, &tv, sizeof(struct timeval));
                    break;
                case SLNETSOCK_OPSOCK_KEEPALIVE:
                    slKAEnableOption = optval;
                    keepaliveEnabled = slKAEnableOption->keepaliveEnabled;
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            SO_KEEPALIVE, &keepaliveEnabled,
                            sizeof(int));
                    break;
                case SLNETSOCK_OPSOCK_LINGER:
                    slLinger = optval;
                    ndkLinger.l_onoff = slLinger->l_onoff;
                    ndkLinger.l_linger = slLinger->l_linger;
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            SO_LINGER, &ndkLinger, sizeof(struct linger));
                    break;
                case SLNETSOCK_OPSOCK_NON_BLOCKING:
                    slNBEnableOption = optval;
                    blocking = (slNBEnableOption->nonBlockingEnabled ? 0 : 1);
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            SO_BLOCKING, &blocking, sizeof(int));
                    break;
                case SLNETSOCK_OPSOCK_BROADCAST:
                    slBroadcastEnableOption = optval;
                    broadcastOn = slBroadcastEnableOption->broadcastEnabled;
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            SO_BROADCAST, &broadcastOn, sizeof(int));
                    break;
                case SLNETSOCK_OPSOCK_REUSEPORT:
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            SO_REUSEPORT, optval, optlen);
                    break;
                case SLNETSOCK_OPSOCK_REUSEADDR:
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            SO_REUSEADDR, optval, optlen);
                    break;
                case SLNETSOCK_OPSOCK_SND_BUF:
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            SO_SNDBUF, optval, optlen);
                    break;
                case SLNETSOCK_OPSOCK_KEEPALIVE_TIME:
                case SLNETSOCK_OPSOCK_NON_IP_BOUNDARY:
                default:
                    /* unsupported by NDK */
                    status = SLNETERR_BSD_EINVAL;
                    break;
            }
            break;
        case SLNETSOCK_LVL_IP:
            ndkLevel = IPPROTO_IP;
            switch (optname) {
                case SLNETSOCK_OPIP_ADD_MEMBERSHIP:
                    slMreq = optval;
                    memcpy(&ipMreq.imr_multiaddr, &slMreq->imr_multiaddr,
                            sizeof(struct in_addr));
                    ipMreq.imr_interface.s_addr = slMreq->imr_interface;
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            IP_ADD_MEMBERSHIP, &ipMreq, sizeof(struct ip_mreq));
                    break;
                case SLNETSOCK_OPIP_DROP_MEMBERSHIP:
                    slMreq = optval;
                    memcpy(&ipMreq.imr_multiaddr, &slMreq->imr_multiaddr,
                            sizeof(struct in_addr));
                    ipMreq.imr_interface.s_addr = slMreq->imr_interface;
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            IP_DROP_MEMBERSHIP, &ipMreq,
                            sizeof(struct ip_mreq));
                    break;
                case SLNETSOCK_OPIPV6_ADD_MEMBERSHIP:
                    ndkLevel = IPPROTO_IPV6;
                    slv6Mreq = optval;
                    memcpy(&ipv6Mreq.ipv6mr_multiaddr,
                            &slv6Mreq->ipv6mr_multiaddr,
                            sizeof(struct in6_addr));
                    ipv6Mreq.ipv6mr_interface = slv6Mreq->ipv6mr_interface;
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            IP_ADD_MEMBERSHIP, &ipv6Mreq,
                            sizeof(struct ipv6_mreq));
                    break;
                case SLNETSOCK_OPIPV6_DROP_MEMBERSHIP:
                    ndkLevel = IPPROTO_IPV6;
                    slv6Mreq = optval;
                    memcpy(&ipv6Mreq.ipv6mr_multiaddr,
                            &slv6Mreq->ipv6mr_multiaddr,
                            sizeof(struct in6_addr));
                    ipv6Mreq.ipv6mr_interface = slv6Mreq->ipv6mr_interface;
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            IP_DROP_MEMBERSHIP, &ipv6Mreq,
                            sizeof(struct ipv6_mreq));
                    break;
                case SLNETSOCK_OPIP_HDRINCL:
                case SLNETSOCK_OPIP_RAW_IPV6_HDRINCL:
                    if (optname == SLNETSOCK_OPIP_RAW_IPV6_HDRINCL) {
                        ndkLevel = IPPROTO_IPV6;
                    }
                    slHeader = optval;
                    h = *slHeader;
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            IP_HDRINCL, &h, sizeof(int));
                    break;
                case SLNETSOCK_OPIPV6_MULTICAST_HOPS:
                    ndkLevel = IPPROTO_IPV6;
                    slHops = optval;
                    hopsVal = *slHops;
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            IPV6_MULTICAST_HOPS, &hopsVal,
                            sizeof(int));
                    break;
                case SLNETSOCK_OPIP_RAW_RX_NO_HEADER:
                case SLNETSOCK_OPIP_MULTICAST_TTL:
                default:
                    /* unsupported by NDK */
                    status = SLNETERR_BSD_EINVAL;
                    break;
            }
            break;
        case SLNETSOCK_PROTO_TCP:
            ndkLevel = IPPROTO_TCP;
            switch (optname) {
                case SLNETSOCK_TCP_NODELAY:
                    slNoDelay = optval;
                    noDelay = slNoDelay->noDelayEnabled;
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            NDK_TCP_NODELAY, &noDelay, sizeof(int));
                    break;
                case SLNETSOCK_TCP_MAXSEG:
                    slMaxSeg = optval;
                    maxSeg = slMaxSeg->maxSeg;
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            NDK_TCP_MAXSEG, &maxSeg, sizeof(int));
                    break;
                case SLNETSOCK_TCP_NOPUSH:
                    slNoPush = optval;
                    noPush = slNoPush->noPushEnabled;
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            NDK_TCP_NOPUSH, &noPush, sizeof(int));
                    break;
                case SLNETSOCK_TCP_NOOPT:
                    slNoOpt = optval;
                    noOpt = slNoOpt->noOptEnabled;
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            NDK_TCP_NOOPT, &noOpt, sizeof(int));
                    break;
                case SLNETSOCK_TCP_SACKPERMITTED:
                    slSackPermitted = optval;
                    sackPermitted = slSackPermitted->sackPermittedEnabled;
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            NDK_TCP_SACKPERMITTED, &sackPermitted, sizeof(int));
                    break;
                case SLNETSOCK_TCP_MAXRTT:
                    slMaxRtt = optval;
                    maxRtt = slMaxRtt->maxRtt;
                    status = NDK_setsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            NDK_TCP_MAXRTT, &maxRtt, sizeof(int));
                    break;
                default:
                    /* unsupported by NDK */
                    status = SLNETERR_BSD_EINVAL;
                    break;
            }
            break;
        case SLNETSOCK_LVL_PHY:
        default:
            /* Unsupported level */
            status = SLNETERR_BSD_EINVAL;
            break;
    }

    if ((status < 0) && (status != SLNETERR_BSD_EINVAL)) {
        status = getBSDSlNetErr();
    }

    return (status);
}


/******************************************************************************
 *
 * SlNetIfNDK_getSockOpt - Get socket options
 *
 *****************************************************************************/
int32_t SlNetIfNDK_getSockOpt(int16_t sd, void *sdContext, int16_t level,
        int16_t optname, void *optval, SlNetSocklen_t *optlen)
{
    int status;
    int ndkLevel;
    int len;
    SlNetSock_Winsize_t *slSize;
    int winSize;
    SlNetSock_Timeval_t *slTimeVal;
    struct timeval tv;
    SlNetSock_Keepalive_t *slKAEnableOption;
    int keepaliveEnabled;
    SlNetSock_linger_t *slLinger;
    struct linger ndkLinger;
    SlNetSock_Nonblocking_t *slNBEnableOption;
    int blocking;
    SlNetSock_Broadcast_t *slBroadcastEnableOption;
    int broadcastOn;
    SlNetSock_NoDelay_t *slNoDelay;
    int noDelay;
    SlNetSock_MaxSeg_t *slMaxSeg;
    int maxSeg;
    SlNetSock_NoPush_t *slNoPush;
    int noPush;
    SlNetSock_NoOpt_t *slNoOpt;
    int noOpt;
    SlNetSock_SackPermitted_t *slSackPermitted;
    int sackPermitted;
    SlNetSock_MaxRtt_t *slMaxRtt;
    int maxRtt;
    uint32_t *slHops;
    int hopsVal;
    SlNetSock_IpMreq_t *slMreq;
    struct ip_mreq ipMreq;
    SlNetSock_IpV6Mreq_t *slv6Mreq;
    struct ipv6_mreq ipv6Mreq;
    uint32_t *slHeader;
    int h;

    assert(optlen != NULL);
    assert(optval != NULL);
    assert(sd >= 0);
    assert((uint16_t)sd < ti_ndk_socket_max_fd);

    switch (level) {
        case SLNETSOCK_LVL_SOCKET:
            ndkLevel = SOL_SOCKET;
            switch (optname) {
                case SLNETSOCK_OPSOCK_RCV_BUF:
                    if (*optlen < sizeof(SlNetSock_Winsize_t)) {
                        return (SLNETERR_RET_CODE_INVALID_INPUT);
                    }
                    len = sizeof(winSize);
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            SO_RCVBUF, &winSize, &len);
                    slSize = optval;
                    slSize->winSize = winSize;
                    *optlen = sizeof(SlNetSock_Winsize_t);
                    break;
                case SLNETSOCK_OPSOCK_RCV_TIMEO:
                    if (*optlen < sizeof(SlNetSock_Timeval_t)) {
                        return (SLNETERR_RET_CODE_INVALID_INPUT);
                    }
                    len = sizeof(tv);
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            SO_RCVTIMEO, &tv, &len);
                    slTimeVal = optval;
                    slTimeVal->tv_sec = tv.tv_sec;
                    slTimeVal->tv_usec = tv.tv_usec;
                    *optlen = sizeof(SlNetSock_Timeval_t);
                    break;
                case SLNETSOCK_OPSOCK_SND_TIMEO:
                    if (*optlen < sizeof(SlNetSock_Timeval_t)) {
                        return (SLNETERR_RET_CODE_INVALID_INPUT);
                    }
                    len = sizeof(tv);
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            SO_SNDTIMEO, &tv, &len);
                    slTimeVal = optval;
                    slTimeVal->tv_sec = tv.tv_sec;
                    slTimeVal->tv_usec = tv.tv_usec;
                    *optlen = sizeof(SlNetSock_Timeval_t);
                    break;
                case SLNETSOCK_OPSOCK_KEEPALIVE:
                    if (*optlen < sizeof(SlNetSock_Keepalive_t)) {
                        return (SLNETERR_RET_CODE_INVALID_INPUT);
                    }
                    len = sizeof(keepaliveEnabled);
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            SO_KEEPALIVE, &keepaliveEnabled, &len);
                    slKAEnableOption = optval;
                    slKAEnableOption->keepaliveEnabled = keepaliveEnabled;
                    *optlen = sizeof(SlNetSock_Keepalive_t);
                    break;
                case SLNETSOCK_OPSOCK_LINGER:
                    if (*optlen < sizeof(SlNetSock_linger_t)) {
                        return (SLNETERR_RET_CODE_INVALID_INPUT);
                    }
                    len = sizeof(ndkLinger);
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            SO_LINGER, &ndkLinger, &len);
                    slLinger = optval;
                    slLinger->l_onoff = ndkLinger.l_onoff;
                    slLinger->l_linger = ndkLinger.l_linger;
                    *optlen = sizeof(SlNetSock_linger_t);
                    break;
                case SLNETSOCK_OPSOCK_NON_BLOCKING:
                    if (*optlen < sizeof(SlNetSock_Nonblocking_t)) {
                        return (SLNETERR_RET_CODE_INVALID_INPUT);
                    }
                    len = sizeof(blocking);
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            SO_BLOCKING, &blocking, &len);
                    slNBEnableOption = optval;
                    slNBEnableOption->nonBlockingEnabled = (blocking ? 0 : 1);
                    *optlen = sizeof(SlNetSock_Nonblocking_t);
                    break;
                case SLNETSOCK_OPSOCK_BROADCAST:
                    if (*optlen < sizeof(SlNetSock_Broadcast_t)) {
                        return (SLNETERR_RET_CODE_INVALID_INPUT);
                    }
                    len = sizeof(broadcastOn);
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            SO_BROADCAST, &broadcastOn, &len);
                    slBroadcastEnableOption = optval;
                    slBroadcastEnableOption->broadcastEnabled = broadcastOn;
                    *optlen = sizeof(SlNetSock_Broadcast_t);
                    break;
                case SLNETSOCK_OPSOCK_REUSEPORT:
                    len = *optlen;
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            SO_REUSEPORT, optval, &len);
                    *optlen = len;
                    break;
                case SLNETSOCK_OPSOCK_REUSEADDR:
                    len = *optlen;
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            SO_REUSEADDR, optval, &len);
                    *optlen = len;
                    break;
                case SLNETSOCK_OPSOCK_SND_BUF:
                    len = *optlen;
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            SO_SNDBUF, optval, &len);
                    *optlen = len;
                    break;
                case SLNETSOCK_OPSOCK_KEEPALIVE_TIME:
                case SLNETSOCK_OPSOCK_NON_IP_BOUNDARY:
                default:
                    /* Unsupported by NDK */
                    status = SLNETERR_BSD_EINVAL;
                    break;
            }
            break;
        case SLNETSOCK_LVL_IP:
            ndkLevel = IPPROTO_IP;
            switch (optname) {
                case SLNETSOCK_OPIP_ADD_MEMBERSHIP:
                    if (*optlen < sizeof(SlNetSock_IpMreq_t)) {
                        return (SLNETERR_RET_CODE_INVALID_INPUT);
                    }
                    len = sizeof(ipMreq);
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            IP_ADD_MEMBERSHIP, &ipMreq, &len);
                    slMreq = optval;
                    memcpy(&slMreq->imr_multiaddr, &ipMreq.imr_multiaddr,
                            sizeof(struct in_addr));
                    slMreq->imr_interface = ipMreq.imr_interface.s_addr;
                    *optlen = sizeof(SlNetSock_IpMreq_t);
                    break;
                case SLNETSOCK_OPIP_DROP_MEMBERSHIP:
                    if (*optlen < sizeof(SlNetSock_IpMreq_t)) {
                        return (SLNETERR_RET_CODE_INVALID_INPUT);
                    }
                    len = sizeof(ipMreq);
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            IP_DROP_MEMBERSHIP, &ipMreq, &len);
                    slMreq = optval;
                    memcpy(&slMreq->imr_multiaddr, &ipMreq.imr_multiaddr,
                            sizeof(struct in_addr));
                    slMreq->imr_interface = ipMreq.imr_interface.s_addr;
                    *optlen = sizeof(SlNetSock_IpMreq_t);
                    break;
                case SLNETSOCK_OPIPV6_ADD_MEMBERSHIP:
                    ndkLevel = IPPROTO_IPV6;
                    if (*optlen < sizeof(SlNetSock_IpMreq_t)) {
                        return (SLNETERR_RET_CODE_INVALID_INPUT);
                    }
                    len = sizeof(ipv6Mreq);
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            IP_ADD_MEMBERSHIP, &ipv6Mreq, &len);
                    slv6Mreq = optval;
                    memcpy(&slv6Mreq->ipv6mr_multiaddr,
                            &ipv6Mreq.ipv6mr_multiaddr,
                            sizeof(SlNetSock_In6Addr_t));
                    slv6Mreq->ipv6mr_interface = ipv6Mreq.ipv6mr_interface;
                    *optlen = sizeof(SlNetSock_IpMreq_t);
                    break;
                case SLNETSOCK_OPIPV6_DROP_MEMBERSHIP:
                    ndkLevel = IPPROTO_IPV6;
                    if (*optlen < sizeof(SlNetSock_IpMreq_t)) {
                        return (SLNETERR_RET_CODE_INVALID_INPUT);
                    }
                    len = sizeof(ipv6Mreq);
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            IP_DROP_MEMBERSHIP, &ipv6Mreq, &len);
                    slv6Mreq = optval;
                    memcpy(&slv6Mreq->ipv6mr_multiaddr,
                            &ipv6Mreq.ipv6mr_multiaddr,
                            sizeof(SlNetSock_In6Addr_t));
                    slv6Mreq->ipv6mr_interface = ipv6Mreq.ipv6mr_interface;
                    *optlen = sizeof(SlNetSock_IpMreq_t);
                    break;
                case SLNETSOCK_OPIP_HDRINCL:
                case SLNETSOCK_OPIP_RAW_IPV6_HDRINCL:
                    if (optname == SLNETSOCK_OPIP_RAW_IPV6_HDRINCL) {
                        ndkLevel = IPPROTO_IPV6;
                    }
                    if (*optlen < sizeof(uint32_t)) {
                        return (SLNETERR_RET_CODE_INVALID_INPUT);
                    }
                    len = sizeof(h);
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            IP_HDRINCL, &h, &len);
                    slHeader = optval;
                    *slHeader = h;
                    *optlen = sizeof(uint32_t);
                    break;
                case SLNETSOCK_OPIPV6_MULTICAST_HOPS:
                    if (*optlen < sizeof(uint32_t)) {
                        return (SLNETERR_RET_CODE_INVALID_INPUT);
                    }
                    len = sizeof(hopsVal);
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            IPV6_MULTICAST_HOPS, &hopsVal, &len);
                    slHops = optval;
                    *slHops = hopsVal;
                    *optlen = sizeof(uint32_t);
                case SLNETSOCK_OPIP_MULTICAST_TTL:
                case SLNETSOCK_OPIP_RAW_RX_NO_HEADER:
                default:
                    /* unsupported by NDK */
                    status = SLNETERR_BSD_EINVAL;
                    break;
            }
            break;
        case SLNETSOCK_PROTO_TCP:
            ndkLevel = IPPROTO_TCP;
            switch (optname) {
                case SLNETSOCK_TCP_NODELAY:
                    if (*optlen < sizeof(SlNetSock_NoDelay_t)) {
                        return (SLNETERR_RET_CODE_INVALID_INPUT);
                    }
                    len = sizeof(noDelay);
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            NDK_TCP_NODELAY, &noDelay, &len);
                    slNoDelay = optval;
                    slNoDelay->noDelayEnabled = noDelay;
                    *optlen = sizeof(SlNetSock_NoDelay_t);
                    break;
                case SLNETSOCK_TCP_MAXSEG:
                    if (*optlen < sizeof(SlNetSock_MaxSeg_t)) {
                        return (SLNETERR_RET_CODE_INVALID_INPUT);
                    }
                    len = sizeof(maxSeg);
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            NDK_TCP_MAXSEG, &maxSeg, &len);
                    slMaxSeg = optval;
                    slMaxSeg->maxSeg = maxSeg;
                    *optlen = sizeof(SlNetSock_MaxSeg_t);
                    break;
                case SLNETSOCK_TCP_NOPUSH:
                    if (*optlen < sizeof(SlNetSock_NoPush_t)) {
                        return (SLNETERR_RET_CODE_INVALID_INPUT);
                    }
                    len = sizeof(noPush);
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            NDK_TCP_NOPUSH, &noPush, &len);
                    slNoPush = optval;
                    slNoPush->noPushEnabled = noPush;
                    *optlen = sizeof(SlNetSock_NoPush_t);
                    break;
                case SLNETSOCK_TCP_NOOPT:
                    if (*optlen < sizeof(SlNetSock_NoOpt_t)) {
                        return (SLNETERR_RET_CODE_INVALID_INPUT);
                    }
                    len = sizeof(noOpt);
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            NDK_TCP_NOOPT, &noOpt, &len);
                    slNoOpt = optval;
                    slNoOpt->noOptEnabled = noOpt;
                    *optlen = sizeof(SlNetSock_NoOpt_t);
                    break;
                case SLNETSOCK_TCP_SACKPERMITTED:
                    if (*optlen < sizeof(SlNetSock_SackPermitted_t)) {
                        return (SLNETERR_RET_CODE_INVALID_INPUT);
                    }
                    len = sizeof(sackPermitted);
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            NDK_TCP_SACKPERMITTED, &sackPermitted, &len);
                    slSackPermitted = optval;
                    slSackPermitted->sackPermittedEnabled = sackPermitted;
                    *optlen = sizeof(SlNetSock_SackPermitted_t);
                    break;
                case SLNETSOCK_TCP_MAXRTT:
                    if (*optlen < sizeof(SlNetSock_MaxRtt_t)) {
                        return (SLNETERR_RET_CODE_INVALID_INPUT);
                    }
                    len = sizeof(maxRtt);
                    status = NDK_getsockopt(ti_ndk_socket_fdtable[sd], ndkLevel,
                            NDK_TCP_MAXRTT, &maxRtt, &len);
                    slMaxRtt = optval;
                    slMaxRtt->maxRtt = maxRtt;
                    *optlen = sizeof(SlNetSock_MaxRtt_t);
                    break;
                default:
                    /* unsupported by NDK */
                    status = SLNETERR_BSD_EINVAL;
                    break;
            }
            break;
        case SLNETSOCK_LVL_PHY:
        default:
            /* Unsupported level */
            status = SLNETERR_BSD_EINVAL;
            break;
    }

    if ((status < 0) && (status != SLNETERR_BSD_EINVAL)) {
        status = getBSDSlNetErr();
    }

    return (status);
}


/******************************************************************************
 *
 * SlNetIfNDK_recv - Read data from TCP socket
 *
 *****************************************************************************/
int32_t SlNetIfNDK_recv(int16_t sd, void *sdContext, void *buf, uint32_t len,
        uint32_t flags)
{
    int nBytes;
    int32_t  ret = SLNETERR_RET_CODE_OK;

    assert(sdContext != NULL);
    assert(buf != NULL);
    assert(sd >= 0);
    assert((uint16_t)sd < ti_ndk_socket_max_fd);

    if (ret == SLNETERR_RET_CODE_OK) {
        /* Note, we assume here (and validated above!) that SLNETSOCK
         * flag values matched the NDK flag values.  So we can blindly
         * pass SLNETSOCK flags into this NDK call.
         */
        nBytes = NDK_recv(ti_ndk_socket_fdtable[sd], buf, len, flags);
        if (nBytes < 0) {
            ret = getBSDSlNetErr();
        }
    }

    if (ret == SLNETERR_RET_CODE_OK) {
        ret = nBytes;
    }

    return (ret);
}

#ifdef SLNETIFNDK_ENABLEMBEDTLS
/******************************************************************************
 *
 * SlNetIfNDK_recvSec - Read data from TCP socket
 *
 *****************************************************************************/
int32_t SlNetIfNDK_recvSec(int16_t sd, void *sdContext, void *buf, uint32_t len,
        uint32_t flags)
{
    int nBytes;
    int32_t  ret = SLNETERR_RET_CODE_OK;
    _SlNetIfNDK_SocketContext *sCtxPtr = sdContext;

    assert(sdContext != NULL);
    assert(buf != NULL);
    assert(sd >= 0);
    assert((uint16_t)sd < ti_ndk_socket_max_fd);

    if (ret == SLNETERR_RET_CODE_OK) {
        if (sCtxPtr->isSecure) {
            /* TODO: flags cannot be honored */
            nBytes = mbedtls_ssl_read(sCtxPtr->ssl, buf, len);
            if (nBytes < 0) {
                switch (nBytes) {
                    case MBEDTLS_ERR_SSL_WANT_READ:
                    case MBEDTLS_ERR_SSL_WANT_WRITE:
                        ret = SLNETERR_BSD_EAGAIN;
                        break;
                    case MBEDTLS_ERR_SSL_CLIENT_RECONNECT:
                    case MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY:
                        ret = SLNETERR_ESEC_CLOSE_NOTIFY;
                        break;
                    case MBEDTLS_ERR_SSL_TIMEOUT:
                        ret = SLNETERR_BSD_EAGAIN;
                        break;
                    case MBEDTLS_ERR_NET_CONN_RESET:
                        ret = SLNETERR_BSD_ECONNRESET;
                        break;
                    default:
                        ret = SLNETERR_UNKNOWN_ERR;
                        break;
                 }
            }
        }
        else {
            return (SlNetIfNDK_recv(sd, sdContext, buf, len, flags));
        }
    }

    if (ret == SLNETERR_RET_CODE_OK) {
        ret = nBytes;
    }

    return (ret);
}
#endif /* SLNETIFNDK_ENABLEMBEDTLS */


/******************************************************************************
 *
 * SlNetIfNDK_recvFrom - Read data from socket
 *
 *****************************************************************************/
int32_t SlNetIfNDK_recvFrom(int16_t sd, void *sdContext, void *buf,
        uint32_t len, uint32_t flags, SlNetSock_Addr_t *from,
        SlNetSocklen_t *fromlen)
{
    int nBytes;
    int32_t  ret = SLNETERR_RET_CODE_OK;
    int fLen;
    uint32_t slNetSockIf;
    uint32_t ndkIf;

    assert(sdContext != NULL);
    assert(buf != NULL);
    assert(sd >= 0);
    assert((uint16_t)sd < ti_ndk_socket_max_fd);

    if (ret == SLNETERR_RET_CODE_OK) {
        /* Note, we assume here (and validated above!) that SLNETSOCK
         * flag values matched the NDK flag values.  So we can blindly
         * pass SLNETSOCK flags into this NDK call.
         */
        if (fromlen != NULL) {
            fLen = *fromlen;
            nBytes = NDK_recvfrom(ti_ndk_socket_fdtable[sd], buf, len, flags,
                    (struct sockaddr *)from, &fLen);
            *fromlen = fLen;
        }
        else {
            nBytes = NDK_recvfrom(ti_ndk_socket_fdtable[sd], buf, len, flags,
                    (struct sockaddr *)from, NULL);
        }

        /*
         * Even if we receive 0 bytes of data the SlNetSock_Addr_t struct was
         * filled in by NDK_recvfrom(). This is because it is valid for both
         * UDP and TCP to send a packet containing 0 bytes of data.
         */
        if(nBytes >= 0) {
            /*
             * If 'from' was passed in, it was filled in with NDK values in the
             * above calls. Translate these NDK values to SlNetSock values.
             */
            if (from) {
                if (from->sa_family == AF_INET6) {
                    /*
                     * Translate family from an NDK value to an SlNetSock value
                     */
                    from->sa_family = SLNETSOCK_AF_INET6;

                    /*
                     * The scope id in 'from' was filled in by the NDK, and so
                     * has an NDK IF number. Translate it to the SlNetSock IF
                     * number:
                     */
                    ndkIf = ((SlNetSock_AddrIn6_t *)from)->sin6_scope_id;
                    slNetSockIf = findSlNetSockIf(ndkIf);
                    ((SlNetSock_AddrIn6_t *)from)->sin6_scope_id = slNetSockIf;
                }
                else {
                    assert(from->sa_family == AF_INET);

                    /*
                     * Translate family from an NDK value to an SlNetSock value
                     */
                    from->sa_family = SLNETSOCK_AF_INET;
                }
            }
        }
        else {
            ret = getBSDSlNetErr();
        }
    }

    if (ret == SLNETERR_RET_CODE_OK) {
        ret = nBytes;
    }

    return (ret);
}

#ifdef SLNETIFNDK_ENABLEMBEDTLS
/******************************************************************************
 *
 * SlNetIfNDK_recvFromSec - Read data from socket
 *
 *****************************************************************************/
int32_t SlNetIfNDK_recvFromSec(int16_t sd, void *sdContext, void *buf,
        uint32_t len, uint32_t flags, SlNetSock_Addr_t *from,
        SlNetSocklen_t *fromlen)
{
    int nBytes;
    int32_t  ret = SLNETERR_RET_CODE_OK;
    _SlNetIfNDK_SocketContext *sCtxPtr = sdContext;
    int fLen;

    assert(sdContext != NULL);
    assert(buf != NULL);
    assert(sd >= 0);
    assert((uint16_t)sd < ti_ndk_socket_max_fd);

    if (ret == SLNETERR_RET_CODE_OK) {
        if (sCtxPtr->isSecure) {
            if ((from != NULL) && (fromlen != NULL)) {
                fLen = *fromlen;
                if (NDK_getpeername(ti_ndk_socket_fdtable[sd],
                        (struct sockaddr *)from,
                        &fLen) == -1) {
                    ret = SLNETSOCK_ERR_SOCKGETPEERNAME_FAILED;
                }
                *fromlen = fLen;
            }
            else if ((from == NULL) && (fromlen == NULL)) {
                /* This is ok according to the spec */
            }
            else {
                ret = SLNETERR_RET_CODE_INVALID_INPUT;
            }
            if (ret == SLNETERR_RET_CODE_OK) {
                nBytes = SlNetIfNDK_recvSec(sd, sdContext, buf, len, flags);
            }
        }
        else {
            return (SlNetIfNDK_recvFrom(sd, sdContext, buf,
                    len, flags, from, fromlen));
        }
    }

    if (ret == SLNETERR_RET_CODE_OK) {
        ret = nBytes;
    }

    return (ret);
}
#endif /* SLNETIFNDK_ENABLEMBEDTLS */

/******************************************************************************
 *
 * SlNetIfNDK_send - Write data to TCP socket
 *
 *****************************************************************************/
int32_t SlNetIfNDK_send(int16_t sd, void *sdContext, const void *buf,
        uint32_t len, uint32_t flags)
{
    int nBytes;
    int32_t ret = 0;

    assert(sdContext != NULL);
    assert(buf != NULL);
    assert(sd >= 0);
    assert((uint16_t)sd < ti_ndk_socket_max_fd);

    if (ret == SLNETERR_RET_CODE_OK) {
        /* Note, we assume here (and validated above!) that SLNETSOCK
         * flag values matched the NDK flag values.  So we can blindly
         * pass SLNETSOCK flags into this NDK call.
         */
        nBytes = NDK_send(ti_ndk_socket_fdtable[sd], (void *)buf, len,
            flags);
    }

    if (ret == SLNETERR_RET_CODE_OK) {
        ret = nBytes;
    }

    return (ret);
}

#ifdef SLNETIFNDK_ENABLEMBEDTLS
/******************************************************************************
 *
 * SlNetIfNDK_sendSec - Write data to TCP socket
 *
 *****************************************************************************/
int32_t SlNetIfNDK_sendSec(int16_t sd, void *sdContext, const void *buf,
        uint32_t len, uint32_t flags)
{
    int nBytes;
    int32_t ret = 0;
    _SlNetIfNDK_SocketContext *sCtxPtr = sdContext;

    assert(sdContext != NULL);
    assert(buf != NULL);
    assert(sd >= 0);
    assert((uint16_t)sd < ti_ndk_socket_max_fd);

    if (ret == SLNETERR_RET_CODE_OK) {
        if (sCtxPtr->isSecure) {
            /* TODO: flags are not honored */
            nBytes = mbedtls_ssl_write(sCtxPtr->ssl, (unsigned char *)buf,
                    len);
            if (nBytes < 0) {
                switch (nBytes) {
                    case MBEDTLS_ERR_SSL_WANT_READ:
                    case MBEDTLS_ERR_SSL_WANT_WRITE:
                        ret = SLNETERR_BSD_EAGAIN;
                        break;
                    case MBEDTLS_ERR_NET_CONN_RESET:
                        ret = SLNETERR_BSD_ECONNRESET;
                        break;
                    default:
                        ret = SLNETERR_UNKNOWN_ERR;
                        break;
                 }
            }
        }
        else {
            return (SlNetIfNDK_send(sd, sdContext, buf, len, flags));
        }
    }

    if (ret == SLNETERR_RET_CODE_OK) {
        ret = nBytes;
    }

    return (ret);
}
#endif /* SLNETIFNDK_ENABLEMBEDTLS */

/******************************************************************************
 *
 * SlNetIfNDK_sendTo - Write data to socket
 *
 *****************************************************************************/
int32_t SlNetIfNDK_sendTo(int16_t sd, void *sdContext, const void *buf,
        uint32_t len, uint32_t flags, const SlNetSock_Addr_t *to,
        SlNetSocklen_t tolen)
{
    SlNetSock_SockAddrStorage_t ndkAddrStorage;
    SlNetSock_AddrIn6_t *ndkAddr6;
    SlNetSock_AddrIn_t *ndkAddr;
    uint32_t slNetSockIf;

    assert(sdContext != NULL);
    assert(buf != NULL);
    assert(to != NULL);
    assert(sd >= 0);
    assert((uint16_t)sd < ti_ndk_socket_max_fd);

    /*
     * The 'to' address passed in contains SlNetSock values. Translate these
     * into NDK values:
     */
    memset(&ndkAddrStorage, 0, sizeof(SlNetSock_SockAddrStorage_t));
    if (to->sa_family == SLNETSOCK_AF_INET6) {
        /* Reference our generic storage space as an IPv6 address struct */
        ndkAddr6 = (SlNetSock_AddrIn6_t *)&ndkAddrStorage;

        /* Need a local copy of 'to', as we have to modify its contents */
        *ndkAddr6 = *((const SlNetSock_AddrIn6_t *)to);

        /* Translate family from an SlNetSock value to an NDK value */
        ndkAddr6->sin6_family = AF_INET6;

        /*
         * The scope id coming into this function call is an SlNetSock IF
         * number. Translate it to the underlying NDK IF number:
         */
        slNetSockIf = ((const SlNetSock_AddrIn6_t *)to)->sin6_scope_id;
        ndkAddr6->sin6_scope_id = SlNetIfNDK_ifTable[slNetSockIf];
    }
    else if (to->sa_family == SLNETSOCK_AF_INET) {
        /* Reference our generic storage space as an IPv4 address struct */
        ndkAddr = (SlNetSock_AddrIn_t *)&ndkAddrStorage;

        /* Need a local copy of 'to', as we have to modify its contents */
        *ndkAddr = *((const SlNetSock_AddrIn_t *)to);

        /* Translate family from an SlNetSock value to an NDK value */
        ndkAddr->sin_family = AF_INET;
    }

    /*
     * Note: if family is invalid, then NDK_sendto will be called with a
     * zeroed out sockaddr struct. This will be caught and handled internally
     * in the stack code, so omitting the check here.
     */

    /*
     * Also note, we assume here (and validated above!) that SLNETSOCK
     * flag values matched the NDK flag values.  So we can blindly
     * pass SLNETSOCK flags into this NDK call.
     */
    return (NDK_sendto(ti_ndk_socket_fdtable[sd], (void *)buf, len,
            flags, (struct sockaddr *)&ndkAddrStorage, tolen));
}

#ifdef SLNETIFNDK_ENABLEMBEDTLS
/******************************************************************************
 *
 * SlNetIfNDK_sendToSec - Write data to socket
 *
 *****************************************************************************/
int32_t SlNetIfNDK_sendToSec(int16_t sd, void *sdContext, const void *buf,
        uint32_t len, uint32_t flags, const SlNetSock_Addr_t *to,
        SlNetSocklen_t tolen)
{
    _SlNetIfNDK_SocketContext *sCtxPtr = sdContext;

    assert(sdContext != NULL);
    assert(buf != NULL);
    assert(to != NULL);
    assert(sd >= 0);
    assert((uint16_t)sd < ti_ndk_socket_max_fd);

    /*
     * Ignoring 'to' and 'tolen' when secure, since we are already in a
     * session
     */
    if (sCtxPtr->isSecure) {
        return (SlNetIfNDK_sendSec(sd, sdContext, buf, len, flags));
    }
    else {
        return (SlNetIfNDK_sendTo(sd, sdContext, buf, len, flags, to, tolen));
    }
}

static int lookupObj(uint8_t *name, uint8_t **objBuf, unsigned int *len,
    _SlNetIfNDK_IfContext *ifCtx)
{
    int i;

    assert(name != NULL);
    assert(objBuf != NULL);
    assert(len != NULL);
    assert(ifCtx != NULL);

    for (i = 0; i < MAX_SEC_OBJS; i++) {
        if ((ifCtx->secObjList[i].name != NULL) &&
            (strcmp((char *)ifCtx->secObjList[i].name, (char *)name) == 0)) {
            *objBuf = ifCtx->secObjList[i].buf;
            *len = ifCtx->secObjList[i].bufLen;
            return (0);
        }
    }

    /* Object not found */
    return (-1);
}

/******************************************************************************
 *
 * SlNetIfNDK_sockStartSec - Start a security session on an opened socket
 *
 *****************************************************************************/
int32_t SlNetIfNDK_sockStartSec(int16_t sd, void *sdContext,
        SlNetSockSecAttrib_t *secAttrib, uint8_t flags)
{
    _SlNetIfNDK_SocketContext      *sCtxPtr = sdContext;
    SlNetSock_SecAttribNode_t      *tempSecAttrib;
    static mbedtls_entropy_context   entropy;
    static mbedtls_ctr_drbg_context  ctr_drbg;
    int16_t                   retVal         = SLNETERR_RET_CODE_OK;
    int                       status;
    unsigned char            *localCertPem;
    unsigned int              localCertPemLen;
    unsigned char            *caCertPem;
    unsigned int              caCertPemLen;
    unsigned char            *pk;
    unsigned int              pkLen;
    static bool               initialized = false;
    char                      pers[] = "TI SLNETIFNDK";
    bool                      initSsl = false;
    int                       transportType;
    bool                      isServer;
#ifdef UDPSUPPORT
    char                      cliName[MAXADDRLEN];  /* TODO: move to heap */
    int                       cliNameLen = sizeof(cliName);
    char                      ipAddr[MAXADDRLEN];
    int                       ipLen;
#endif

    assert(sdContext != NULL);
    assert(sd >= 0);
    assert((uint16_t)sd < ti_ndk_socket_max_fd);

    isServer = ((SLNETSOCK_SEC_IS_SERVER & flags)? true : false);

    if (!initialized) { /* avoid grabbing semaphore unnecessarily */
        sem_wait(&SlNetIfNDK_sem);

        /*
         * strange-looking 2nd check, but we're only here if multiple
         * threads called sockStartSec before 'initialized' was set and grabbed
         * the lock.  Only the first thread will see 'initialized' as
         * false and actually do the init.  Others will skip right by and
         * release the lock.
         */
        if (!initialized) {
            /* Setup the thread callbacks */
            mbedtls_threading_set_alt(threading_mutex_init_pthread,
                    threading_mutex_free_pthread, threading_mutex_lock_pthread,
                    threading_mutex_unlock_pthread);

            mbedtls_entropy_init(&entropy);

            /* Setup RNG */
            /*
             *  Note: entropy_source() is a sample implementation of entropy source
             *  which is not strong. This is recommended to be used for application
             *  development purposes only and an alternate user defined strong
             *  entropy source implementation should be used in production
             *  application.
             */
            status = mbedtls_entropy_add_source(&entropy, entropy_source, NULL, 128,
                    MBEDTLS_ENTROPY_SOURCE_STRONG);
            if (status != 0) {
                mbedtls_entropy_free(&entropy);
                sem_post(&SlNetIfNDK_sem);
                return (SLNETERR_RET_CODE_FUNCTION_FAILED);
            }

            mbedtls_ctr_drbg_init(&ctr_drbg);
            status = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func,
                    &entropy, (const unsigned char *)pers, strlen(pers));
            if (status != 0) {
                mbedtls_entropy_free(&entropy);
                sem_post(&SlNetIfNDK_sem);
                return (SLNETERR_RET_CODE_FUNCTION_FAILED);
            }
            initialized = true;
        }

        sem_post(&SlNetIfNDK_sem);
    }

    /* Determine transport type */
    if (sCtxPtr->type == SLNETSOCK_SOCK_STREAM) {
        transportType = MBEDTLS_SSL_TRANSPORT_STREAM;
    }
    else if (sCtxPtr->type == SLNETSOCK_SOCK_DGRAM) {
        transportType = MBEDTLS_SSL_TRANSPORT_DATAGRAM;
    }
    else {
        /* Unsupported type */
        retVal = SLNETERR_BSD_ESOCKTNOSUPPORT;
        goto error;
    }

    if (0 != (flags & SLNETSOCK_SEC_BIND_CONTEXT_ONLY)) {
        if (secAttrib == NULL) {
            /* Cannot be NULL during BIND CONTEXT phase */
            retVal = SLNETERR_RET_CODE_INVALID_INPUT;
            goto error;
        }

        tempSecAttrib = *secAttrib;

        /* run over all attributes and set them */
        while (NULL != tempSecAttrib) {
            switch (tempSecAttrib->attribName) {
                case SLNETSOCK_SEC_ATTRIB_PRIVATE_KEY:
                    status = lookupObj(tempSecAttrib->attribBuff, /*obj name */
                            &pk, &pkLen, sCtxPtr->ifCtx);
                    if (status == -1) {
                        retVal = SLNETERR_RET_CODE_FUNCTION_FAILED;
                        goto error;
                    }

                    if (!sCtxPtr->initKey) {
                        mbedtls_pk_init(&sCtxPtr->key);
                        sCtxPtr->initKey = true;
                    }
                    status = mbedtls_pk_parse_key(&sCtxPtr->key,
                            (const unsigned char *)pk, pkLen, NULL, 0);
                    if (status != 0) {
                        retVal = SLNETERR_RET_CODE_FUNCTION_FAILED;
                        goto error;
                    }
                    break;
                case SLNETSOCK_SEC_ATTRIB_LOCAL_CERT:
                    status = lookupObj(tempSecAttrib->attribBuff, /*obj name */
                            &localCertPem, &localCertPemLen, sCtxPtr->ifCtx);
                    if (status == -1) {
                        retVal = SLNETERR_RET_CODE_FUNCTION_FAILED;
                        goto error;
                    }
                    if (!sCtxPtr->initLocalCert) {
                        mbedtls_x509_crt_init(&sCtxPtr->localCert);
                        sCtxPtr->initLocalCert = true;
                    }
                    status = mbedtls_x509_crt_parse(&sCtxPtr->localCert,
                            (const unsigned char *)localCertPem,
                            localCertPemLen);
                    if (status != 0) {
                        retVal = SLNETERR_RET_CODE_FUNCTION_FAILED;
                        goto error;
                    }
                    break;
                case SLNETSOCK_SEC_ATTRIB_PEER_ROOT_CA:
                    status = lookupObj(tempSecAttrib->attribBuff, /*obj name */
                            &caCertPem, &caCertPemLen, sCtxPtr->ifCtx);
                    if (status == -1) {
                        retVal = SLNETERR_RET_CODE_FUNCTION_FAILED;
                        goto error;
                    }
                    if (!sCtxPtr->initCaCert) {
                        mbedtls_x509_crt_init(&sCtxPtr->caCert);
                        sCtxPtr->initCaCert = true;
                    }
                    status = mbedtls_x509_crt_parse(&sCtxPtr->caCert,
                            (const unsigned char *)caCertPem, caCertPemLen);
                    if (status != 0) {
                        retVal = SLNETERR_RET_CODE_FUNCTION_FAILED;
                        goto error;
                    }
                    break;
                case SLNETSOCK_SEC_ATTRIB_DOMAIN_NAME:
                    if (tempSecAttrib->attribBuff == NULL) {
                        retVal = SLNETERR_RET_CODE_INVALID_INPUT;
                        goto error;
                    }
                    if (sCtxPtr->domainName != NULL) {
                        free(sCtxPtr->domainName);
                    }
                    sCtxPtr->domainName = malloc(strlen(
                            (char *)tempSecAttrib->attribBuff) + 1);
                    if (sCtxPtr->domainName == NULL) {
                        retVal = SLNETERR_BSD_ENOMEM;
                        goto error;
                    }
                    strcpy(sCtxPtr->domainName,
                            (char *)tempSecAttrib->attribBuff);
                    break;
                case SLNETSOCK_SEC_ATTRIB_CIPHERS:
                    /* TBD */
                    break;
                case SLNETSOCK_SEC_ATTRIB_METHOD:
                    /* TBD */
                    break;
                case SLNETSOCK_SEC_ATTRIB_DISABLE_CERT_STORE:
                    /* Ignore this - there is no cert store on this device */
                    break;
                default:
                    /* Unsupported attribute encountered */
                    retVal = SLNETERR_RET_CODE_INVALID_INPUT;
                    goto error;
            }
            tempSecAttrib = tempSecAttrib->next;
        }

        /* Setup TLS configuration */
        mbedtls_ssl_config_init(&sCtxPtr->conf);
        sCtxPtr->initConf = true;

        if (isServer) {
            status = mbedtls_ssl_config_defaults(&sCtxPtr->conf,
                    MBEDTLS_SSL_IS_SERVER,
                    transportType, MBEDTLS_SSL_PRESET_DEFAULT);
        }
        else {
            status = mbedtls_ssl_config_defaults(&sCtxPtr->conf,
                    MBEDTLS_SSL_IS_CLIENT,
                    transportType, MBEDTLS_SSL_PRESET_DEFAULT);
        }
        if (status != 0) {
            retVal = SLNETERR_RET_CODE_FUNCTION_FAILED;
            goto error;
        }

        mbedtls_ssl_conf_renegotiation(&sCtxPtr->conf,
                MBEDTLS_SSL_RENEGOTIATION_ENABLED);

        /* Set rsa_min_bitlen to 1024 for better compatibility */
        sCtxPtr->crtProfile = mbedtls_x509_crt_profile_default;
        sCtxPtr->crtProfile.rsa_min_bitlen = RSA_MIN_BITLEN;
        mbedtls_ssl_conf_cert_profile(&sCtxPtr->conf, &sCtxPtr->crtProfile);

        /*
         * When the CA certificate attribute is set on a server,
         * we need to request client authentication to match the behavior
         * on CC32xx
         */
        if ((isServer) && (sCtxPtr->initCaCert)) {
            mbedtls_ssl_conf_authmode(&sCtxPtr->conf,
                MBEDTLS_SSL_VERIFY_REQUIRED);
        }

        mbedtls_ssl_conf_rng(&sCtxPtr->conf, mbedtls_ctr_drbg_random,
                &ctr_drbg);
        mbedtls_ssl_conf_ca_chain(&sCtxPtr->conf, &sCtxPtr->caCert, NULL);

        if ((sCtxPtr->initLocalCert) && (sCtxPtr->initKey)) {
            status = mbedtls_ssl_conf_own_cert(&sCtxPtr->conf, &sCtxPtr->localCert,
                    &sCtxPtr->key);
            if (status != 0) {
                retVal = SLNETERR_RET_CODE_FUNCTION_FAILED;
                goto error;
            }
        }

#ifdef UDPSUPPORT
        if ((isServer) && (transportType == MBEDTLS_SSL_TRANSPORT_DATAGRAM)) {
            mbedtls_ssl_cookie_init(&sCtxPtr->cookieCtx);
            sCtxPtr->initCookie = true;
            if ((mbedtls_ssl_cookie_setup(&sCtxPtr->cookieCtx,
                    mbedtls_ctr_drbg_random, &ctr_drbg)) != 0 ) {
                retVal = SLNETERR_RET_CODE_FUNCTION_FAILED;
                goto error;
            }

            mbedtls_ssl_conf_dtls_cookies( &sCtxPtr->conf,
                mbedtls_ssl_cookie_write,
                mbedtls_ssl_cookie_check, &sCtxPtr->cookieCtx );
        }
#endif
    }

    if (0 != (flags & SLNETSOCK_SEC_START_SECURITY_SESSION_ONLY)) {
        sCtxPtr->ssl = (mbedtls_ssl_context *)malloc(
                sizeof(mbedtls_ssl_context));
        if (!sCtxPtr->ssl) {
            retVal = SLNETERR_BSD_ENOMEM;
            goto error;
        }

        mbedtls_ssl_init(sCtxPtr->ssl);
        initSsl = true;
        status = mbedtls_ssl_setup(sCtxPtr->ssl, &sCtxPtr->conf);
        if (status != 0) {
            retVal = SLNETERR_RET_CODE_FUNCTION_FAILED;
            goto error;
        }

        if ((sCtxPtr->domainName) && (!isServer)) {
            if ((mbedtls_ssl_set_hostname(sCtxPtr->ssl,
                    sCtxPtr->domainName)) != 0) {
                retVal = SLNETERR_RET_CODE_FUNCTION_FAILED;
                goto error;
            }
        }

        sCtxPtr->clientSd = sd;
#ifdef UDPSUPPORT
        if (transportType == MBEDTLS_SSL_TRANSPORT_DATAGRAM) {
            mbedtls_ssl_set_timer_cb(sCtxPtr->ssl, &sCtxPtr->timer,
                    mbedtls_timing_set_delay, mbedtls_timing_get_delay);

            /* For HelloVerifyRequest cookies */
            if (isServer) {
                /* TODO: Need Steve to check this */
                NDK_getpeername(ti_ndk_socket_fdtable[sd],
                    (struct sockaddr *)cliName,
                    &cliNameLen);
                if (((struct sockaddr *)cliName)->sa_family == AF_INET) {
                    struct sockaddr_in *addr4 = (struct sockaddr_in *)cliName;
                    ipLen = sizeof(addr4->sin_addr.s_addr);
                    memcpy(ipAddr, &addr4->sin_addr.s_addr, ipLen);
                }
                else {
                    struct sockaddr_in6 *addr6 = (struct sockaddr_in6 *)cliName;
                    ipLen = sizeof(addr6->sin6_addr.s6_addr);
                    memcpy(ipAddr, &addr6->sin6_addr.s6_addr, ipLen);
                }

                if ((mbedtls_ssl_set_client_transport_id(sCtxPtr->ssl,
                        (unsigned char *)ipAddr, ipLen)) != 0 ) {
                    retVal = SLNETERR_RET_CODE_FUNCTION_FAILED;
                    goto error;
                }
            }
            mbedtls_ssl_set_bio(sCtxPtr->ssl, &sCtxPtr->clientSd, netSend,
                    netRecv, mbedtls_net_recv_timeout);
        }
        else {
#endif
            mbedtls_ssl_set_bio(sCtxPtr->ssl, &sCtxPtr->clientSd, netSend,
                    netRecv, NULL);
#ifdef UDPSUPPORT
        }
#endif

        /* Start TLS session */
        while ((status = mbedtls_ssl_handshake(sCtxPtr->ssl)) != 0) {
            if ((transportType == MBEDTLS_SSL_TRANSPORT_DATAGRAM) &&
                    (status == MBEDTLS_ERR_SSL_HELLO_VERIFY_REQUIRED)) {
                 /*
                  * If hello verify fails, return error. Client sd needs to be
                  * closed and recvfrom called again before app
                  * can call startSec on the new sd
                  */
                 status = SLNETERR_ESEC_HELLO_VERIFY_ERROR;
                 goto error;
            }
            if (status != MBEDTLS_ERR_SSL_WANT_READ
                    && status != MBEDTLS_ERR_SSL_WANT_WRITE) {
                retVal = SLNETERR_RET_CODE_FUNCTION_FAILED;
                goto error;
            }
        }
    }

    /*
     * Socket is considered a secure socket when the function is successful,
     * regardless of the flags, so that we release memory during
     * SlNetIfNDK_closeSec().
     */
    assert(retVal == SLNETERR_RET_CODE_OK);
    sCtxPtr->isSecure = true;

error:
    if (retVal != SLNETERR_RET_CODE_OK) {
        sCtxPtr->isSecure = false;
        if (initSsl) {
            mbedtls_ssl_free(sCtxPtr->ssl);
        }
        if (sCtxPtr->ssl) {
            free(sCtxPtr->ssl);
        }
        if (sCtxPtr->initCookie) {
            mbedtls_ssl_cookie_free(&sCtxPtr->cookieCtx);
        }
        if (sCtxPtr->initLocalCert) {
            mbedtls_x509_crt_free(&sCtxPtr->localCert);
        }
        if (sCtxPtr->initCaCert) {
            mbedtls_x509_crt_free(&sCtxPtr->caCert);
        }
        if (sCtxPtr->initKey) {
            mbedtls_pk_free(&sCtxPtr->key);
        }
        if (sCtxPtr->initConf) {
            mbedtls_ssl_config_free(&sCtxPtr->conf);
        }
        if (sCtxPtr->domainName) {
            free(sCtxPtr->domainName);
        }
    }
    return (retVal);
}
#endif /* SLNETIFNDK_ENABLEMBEDTLS */

/******************************************************************************
 *
 * SlNetIfNDK_getHostByName - Obtain the IP Address of machine on network, by
 *                             machine name
 *
 *****************************************************************************/
int32_t SlNetIfNDK_getHostByName(void *ifContext, char *name,
        const uint16_t nameLen, uint32_t *ipAddr, uint16_t *ipAddrLen,
        const uint8_t family)
{
    int slStatus = SLNETERR_RET_CODE_OK;
    int ndkStatus = NDK_DNS_NOERROR;

    if ((ipAddr == NULL) || (ipAddrLen == NULL)) {
        return (SLNETERR_RET_CODE_INVALID_INPUT);
    }

    if (family == SLNETSOCK_AF_INET) {
        ndkStatus = DNSGetHostArrayByName(name, AF_INET, ipAddr, ipAddrLen);
    }
    else if (family == SLNETSOCK_AF_INET6) {
        ndkStatus = DNSGetHostArrayByName(name, AF_INET6, ipAddr, ipAddrLen);
    }
    else {
        return (SLNETERR_NET_APP_DNS_INVALID_FAMILY_TYPE);
    }

    if (ndkStatus != NDK_DNS_NOERROR) {
        /* Translate NDK DNS error codes to SlNetSock error codes */
        switch (ndkStatus) {
            case NDK_DNS_EMEMERROR:
                slStatus = SLNETERR_NET_APP_DNS_ALLOC_ERROR;
                break;
            case NDK_DNS_EINVALIDARGS:
                slStatus = SLNETERR_NET_APP_DNS_PARAM_ERROR;
                break;
            case NDK_DNS_EINVFAMILY:
                slStatus = SLNETERR_NET_APP_DNS_INVALID_FAMILY_TYPE;
                break;
            case NDK_DNS_EIPv6DISABLED:
                slStatus = SLNETERR_NET_APP_DNS_IPV6_REQ_BUT_IPV6_DISABLED;
                break;
            case NDK_DNS_ENODNSREPLY:
            default:
                slStatus = SLNETERR_NET_APP_DNS_QUERY_FAILED;
                break;
        }
    }

    return (slStatus);
}

/******************************************************************************
 *
 * SlNetIfNDK_ifCreateContext - Create interface context object
 *
 * Note: This function is not thread-safe.
 *
 *****************************************************************************/
int32_t SlNetIfNDK_ifCreateContext(uint16_t ifID, const char *ifName,
        void **ifContext)
{
    uint32_t i;
    int32_t result = SLNETERR_RET_CODE_MALLOC_ERROR;
    _SlNetIfNDK_IfContext *context;
    NETIF_DEVICE *dev;
    static bool initialized = false;

    assert(ifContext != NULL);
    assert(ifName != NULL);

    /*
     * We are doing some one-time module-level initialization here, short of a
     * better place for it. Since the other functions in the module are not
     * called before at least one interface has been added, this should be ok
     */
    if (!initialized) {
        if (sem_init(&SlNetIfNDK_sem, 0, 1) != 0) {
            return (SLNETERR_RET_CODE_MUTEX_CREATION_FAILED);
        }
        /* Zero out the IF translation table */
        for (i = 0; i < SLNETIF_MAX_IF; i++) {
            SlNetIfNDK_ifTable[i] = 0;
        }

        NDK_registerHook(CREATE_SKT_CTX_HOOK, (NDK_HookFxn)(&addSktToTable));
        NDK_registerHook(CLOSE_SKT_CTX_HOOK, (NDK_HookFxn)(&removeSktFromTable));

        initialized = true;
    }

    context = calloc(1, sizeof(_SlNetIfNDK_IfContext));

    if (context) {
        dev = NIMUFindByName((char *)ifName);
        if (dev == NULL) {
            result = SLNETERR_RET_CODE_INVALID_INPUT;
        }
        else {
            /* Save interface ID mapping to table */
            SlNetIfNDK_ifTable[ifID] = dev->index;

            context->physIf = dev->index;
            strcpy(context->name, dev->name);
            *ifContext = context;
            result = SLNETERR_RET_CODE_OK;
        }
    }

    return (result);
}


/******************************************************************************
 *
 * SlNetIfNDK_getIPAddr - Get IP Address of specific interface
 *
 *****************************************************************************/
int32_t SlNetIfNDK_getIPAddr(void *ifContext, SlNetIfAddressType_e addrType,
        uint16_t *addrConfig, uint32_t *ipAddr)
{
    uint32_t ip;
    _SlNetIfNDK_IfContext *ifCtx = (_SlNetIfNDK_IfContext *)ifContext;

    assert(ipAddr != NULL);
    assert(addrConfig != NULL);
    assert(ifContext != NULL);

    if (!NtIfIdx2Ip(ifCtx->physIf, &ip)) {
        return (SLNETERR_RET_CODE_FUNCTION_FAILED);
    }

    switch (addrType) {
        case SLNETIF_IPV4_ADDR:
            *ipAddr = ip;
            break;
        case SLNETIF_IPV6_ADDR_LOCAL:
        case SLNETIF_IPV6_ADDR_GLOBAL:
            /* TODO: No IPv6 support, NtIfIdx2Ip only reports 32 bits */
        default:
            /* Invalid address type */
            return (SLNETERR_RET_CODE_INVALID_INPUT);
    }

    /* Not supported in NDK */
    *addrConfig = SLNETIF_ADDR_CFG_UNKNOWN;

    return (SLNETERR_RET_CODE_OK);
}

/******************************************************************************
 *
 * SlNetIfNDK_getConnectionStatus - Get interface connection status
 *
 *****************************************************************************/
int32_t SlNetIfNDK_getConnectionStatus(void *ifContext)
{
    int32_t retVal = SLNETIF_STATUS_DISCONNECTED;
    NIMU_IF_REQ ifReq;
    uint32_t isLinkUp;
    _SlNetIfNDK_IfContext *ifCtx = (_SlNetIfNDK_IfContext *)ifContext;

    assert(ifContext != NULL);

    ifReq.index = ifCtx->physIf;
    strcpy(ifReq.name, ifCtx->name);

    if (NIMUIoctl(NIMU_GET_DEVICE_ISLINKUP, &ifReq, &isLinkUp,
            sizeof(isLinkUp)) == 0) {
        if (isLinkUp == 1) {
            retVal = SLNETIF_STATUS_CONNECTED;
        }
    }

    return (retVal);
}

/******************************************************************************
 *
 * SlNetIfNDK_loadSecObj - Load secured buffer to the network stack
 *
 *****************************************************************************/
int32_t SlNetIfNDK_loadSecObj(void *ifContext, uint16_t objType, char *objName,
        int16_t objNameLen, uint8_t *objBuff, int16_t objBuffLen)
{
    int i;
    int32_t retVal;
    bool found = false;
    _SlNetIfNDK_IfContext *ifCtx = ifContext;
    _SlNetIfNDK_SecObj *firstEmpty = NULL;

    /*
     * No runtime param validation as they have already been validated
     * by front end API.  Just assert all is well.
     */
    assert(ifContext != NULL);
    assert(objName != NULL);
    assert(strlen(objName) == objNameLen);
    assert((objType == SLNETIF_SEC_OBJ_TYPE_RSA_PRIVATE_KEY) ||
            (objType == SLNETIF_SEC_OBJ_TYPE_CERTIFICATE) ||
            (objType == SLNETIF_SEC_OBJ_TYPE_DH_KEY));

    /* Search for existing name in array */
    for (i = 0; i < MAX_SEC_OBJS; i++)
    {
        _SlNetIfNDK_SecObj *secObj = &(ifCtx->secObjList[i]);

        if (secObj->name == NULL)
        {
            if (firstEmpty == NULL)
            {
                /* remember first empty slot, in case we need it later */
                firstEmpty = secObj;
            }
        }
        else if (strcmp(secObj->name, objName) == 0)
        {
            /* match */
            assert(secObj->nameLen == objNameLen);

            if (objBuff == NULL) {
                /* this is a "clear" request */
                free(secObj->name);
                secObj->name = NULL;
                secObj->nameLen = 0;
            }

            /* replace the sec obj buffer/len */
            secObj->buf = objBuff;
            secObj->bufLen = objBuffLen;

            found = true;
            break;
        }
    }

    if (found)
    {
        retVal = 0;
    }
    else
    {
        if (objBuff != NULL)
        {
            /* drop this in the first empty slot */
            if (firstEmpty != NULL)
            {
                firstEmpty->name = malloc(objNameLen + 1);
                if (firstEmpty->name != NULL)
                {
                    memcpy(firstEmpty->name, objName, objNameLen);
                    firstEmpty->name[objNameLen] = '\0';
                    firstEmpty->nameLen = objNameLen;
                    firstEmpty->buf = objBuff;
                    firstEmpty->bufLen = objBuffLen;

                    retVal = 0;
                }
                else
                {
                    retVal = SLNETERR_RET_CODE_MALLOC_ERROR;
                }
            }
            else
            {
                retVal = SLNETERR_RET_CODE_NO_FREE_SPACE;
            }
        }
        else
        {
            /* objBuff was NULL, but didn't find a matching name to clear */
            retVal = SLNETERR_RET_CODE_INVALID_INPUT;
        }
    }

    return (retVal);
}
