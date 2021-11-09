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
 * ======== dnsif.h ========
 *
 * Basic DNS resolution routine prototypes
 *
 */

#ifndef _DNSIF_H_
#define _DNSIF_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Response codes */
/* (Codes under 16 are from a DNS reply packet) */
#define NDK_DNS_NOERROR         0               /* (DNS) no error */
#define NDK_DNS_EFORMERR         1               /* (DNS) format error */
#define NDK_DNS_ESERVFAIL        2               /* (DNS) server failure */
#define NDK_DNS_ENXDOMAIN        3               /* (DNS) non existent domain */
#define NDK_DNS_ENOTIMP          4               /* (DNS) not implemented */
#define NDK_DNS_EREFUSED         5               /* (DNS) query refused */
#define NDK_DNS_EOVERFLOW        16              /* buffer overflow */
#define NDK_DNS_EMEMERROR        17              /* memory allocation error */
#define NDK_DNS_ESOCKETERROR     18              /* internal socket error */
#define NDK_DNS_ENODNSREPLY      19              /* DNS Server did not respond */
#define NDK_DNS_EIPv6DISABLED 20        /* IPv6 Resolution requested */
                                        /* but IPv6 is disabled */
#define NDK_DNS_EINVFAMILY    21        /* family must be AF_INET, AF_INET6 */
#define NDK_DNS_EINVALIDARGS  22        /* invalid arguments passed */

#define NDK_DNS_MAXIPADDR        8

/**
 * @brief
 *  The structure describes the Host Name - IP Address record
 *
 * @details
 *  The HOSTENT structure holds information such as IPv4/v6
 *  address, host name mappings for a given host. It is used
 *  by the DNS resolver in conveying such HostName - IP Address
 *  mappings to a user application.
 */
struct _hostent {
    /**
     * @brief   This is the official name / Fully Qualified Domain Name
     * (FQDN) of the host.
     */
    char   *h_name;

    /**
     * @brief   This indicates the address family of the IP address that
     * maps to the given hostname. The values it takes are AF_INET (v4) /
     * AF_INET6 (v6).
     */
    int     h_addrtype;

    /**
     * @brief   This indicates the length (in bytes) of the IP address that follows.
     * For IPv4 address it is set to 4, and for IPv6 address set to 16 bytes.
     */
     int     h_length;

    /**
     * @brief   This is the number of IP addresses returned for the given
     * hostname.
     */
     int     h_addrcnt;

#ifndef _INCLUDE_IPv6_CODE

    /**
     * @brief   List of upto NDK_DNS_MAXIPADDR IPv4 addresses (Network format) that map
     * to the given hostname.
     */
     uint32_t h_addr[NDK_DNS_MAXIPADDR];

#else

    /**
     * @brief   List of upto NDK_DNS_MAXIPADDR IPv4/IPv6 addresses that map to given hostname.
     */
     char*   h_addr_list[NDK_DNS_MAXIPADDR];

#endif
        };

typedef struct _hostent HOSTENT;

extern char *DNSErrorStrings[20];

#define DNSErrorStr(err) DNSErrorStrings[(err)]

/*-------------------------------------------------------------------- */
/* DNSGetHostname */
/* Requests the hostname which matches the IPHost supplied to SetConfig, */
/* or uses the first IP address found in the system if SetConfig was */
/* not supplied with a host address. The hostname is copied into the */
/* buffer pointed to by 'pNameBuf' with a max size of 'size'. */
/* NULL terminates the name when space allows. */
/* Returns 0 if OK, or error code */
/*-------------------------------------------------------------------- */
extern int DNSGetHostname( char *pNameBuf, int size );

/*-------------------------------------------------------------------- */
/* DNSGetHostByAddr */
/* Looks up the supplied IP address. On a successful return, pScrapBuf */
/* can be treated as a HOSTENT structure. The size of the scrap buffer */
/* (size) must be greater than the size of the structure as the */
/* structure will contain pointers into the scrap buffer, and the */
/* scrap buffer is also used for temporary name storage. 512 bytes */
/* should be sufficient for most requests. */
/* Returns 0 if OK, or error code */
/*-------------------------------------------------------------------- */
extern int DNSGetHostByAddr( uint32_t IPAddr, void *pScrapBuf, int size );

/*-------------------------------------------------------------------- */
/* DNSGetHostByName */
/* Looks up the supplied hostname. On a successful return, pScrapBuf */
/* can be treated as a HOSTENT structure. The size of the scrap buffer */
/* (size) must be greater than the size of the structure as the */
/* structure will contain pointers into the scrap buffer, and the */
/* scrap buffer is also used for temporary name storage. 512 bytes */
/* should be sufficient for most requests. */
/* If the host name 'Name' is terminated with a dot ('.'), the dot is */
/* removed. If the name contains a dot anywhere, it is used unmodified */
/* for an initial lookup. If the lookup fails - or if the name did not */
/* contain a dot, the default domain suffix is applied. */
/* Returns 0 if OK, or error code */
/*-------------------------------------------------------------------- */
extern int DNSGetHostByName( char *Name, void *pScrapBuf, int size );

#ifdef _INCLUDE_IPv6_CODE

/*-------------------------------------------------------------------- */
/* DNSGetHostByAddr2 */
/* Looks up the supplied IPv6 address. On a successful return, pScrapBuf */
/* can be treated as a HOSTENT structure. The size of the scrap buffer */
/* (size) must be greater than the size of the structure as the */
/* structure will contain pointers into the scrap buffer, and the */
/* scrap buffer is also used for temporary name storage. 512 bytes */
/* should be sufficient for most requests. */
/* Returns 0 if OK, or error code */
/*-------------------------------------------------------------------- */
extern int DNSGetHostByAddr2( IP6N IPAddr, void *pScrapBuf, int size );

/*-------------------------------------------------------------------- */
/* DNSGetHostByName2 */
/* Looks up the supplied hostname. On a successful return, pScrapBuf */
/* can be treated as a HOSTENT structure. The size of the scrap buffer */
/* (size) must be greater than the size of the structure as the */
/* structure will contain pointers into the scrap buffer, and the */
/* scrap buffer is also used for temporary name storage. 512 bytes */
/* should be sufficient for most requests. */
/* If the host name 'Name' is terminated with a dot ('.'), the dot is */
/* removed. If the name contains a dot anywhere, it is used unmodified */
/* for an initial lookup. If the lookup fails the appropriate DNS error */
/* code is returned. However, if the name did not contain a dot, a  */
/* format error is returned to the user. No default domain lookups are */
/* performed for IPv6 */
/* Returns 0 if OK, or error code */
/*-------------------------------------------------------------------- */
extern int DNSGetHostByName2( char *Name, unsigned char af_family, void *pScrapBuf, int size );

#endif

/* DNSGetHostArrayByName */
/* Perform a family specific DNS lookup and store results into a buffer */
extern int DNSGetHostArrayByName(const char *name, uint8_t family,
        uint32_t *ipBuffer, uint16_t *numIpAddrs);

/* DNSServerOpen */
/* Create an instance of the DNS Server */
extern void *DNSServerOpen( NTARGS *pNTA );

/* DNSServerClose */
/* Destroy an instance of the DNS Server */
extern void DNSServerClose( void *h );

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif


