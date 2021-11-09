/*
 * Copyright (c) 2013-2019, Texas Instruments Incorporated
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
 * ======== dnsclnt.c ========
 *
 * DNS Client Routines
 *
 */

#include <string.h>
#include "dns.h"
#include <stdlib.h>

#define NDK_DNSBUFSIZE 512

/* Error Code Strings */

static char chnull = 0;
#define NA &chnull
char *DNSErrorStrings[20] = { "No Error",        "Format Error",
                              "Server Failure",  "Does Not Exist",
                              "Not Implemented", "Refused",
                              NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
                              "Overflow",        "Out of Memory",
                              "Socket Error",    "No DNS Reply" };


/* Static Functions */

static int  DNSResolve( unsigned char *qBuf, unsigned char af_family, uint16_t type,
                        void *pBuf, int size );

/*-------------------------------------------------------------------- */
/* DNSGetHostname */

/* Requests the hostname which matches the IPHost supplied to SetConfig, */
/* or uses the first IP address found in the system if SetConfig was */
/* not supplied with a host address. The hostname is copied into the */
/* buffer pointed to by 'pNameBuf' with a max size of 'size'. */
/* NULL terminates the name when space allows. */

/* Returns 0 if OK, or error code */
/*-------------------------------------------------------------------- */
int DNSGetHostname( char *pNameBuf, int size )
{
    uint32_t IPAddr = 0;
    char   *tmpbuf;
    int    errcode;

    /* This call is just about the same as DNSGetHostByAddr, only */
    /* with our IP address. */

    /* Get the best IP address for this system */
    /* If we don't have an IP address, then don't copy a name */
    if( !NtGetPublicHost( &IPAddr, 0, 0 ) )
    {
        if( size > 0 )
            *pNameBuf = 0;
        return(0);
    }

    /* Allocate a scrap buffer for the resolver */
    tmpbuf = mmAlloc( 1024 );
    if( !tmpbuf )
        return( NDK_DNS_EMEMERROR );

    /* Now we can call DNSGetHostByAddr */
    errcode = DNSGetHostByAddr( IPAddr, tmpbuf, 1024 );

    /* If there's no error, copy the name (if we have one) */
    if( !errcode )
    {
        HOSTENT *ph = (HOSTENT *)tmpbuf;

        if( strlen( ph->h_name ) < (uint32_t)size )
            strcpy( pNameBuf, ph->h_name );
        else
            strncpy( pNameBuf, ph->h_name, size-1 );
    }

    /* Free the temp buffer */
    mmFree( tmpbuf );

    return( errcode );
}

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
int DNSGetHostByAddr( uint32_t IPAddr, void *pScrapBuf, int size )
{
    char qBuf[64];
    int  errcode;

    /* We only know our own host format (not IPN) */
    IPAddr = NDK_htonl( IPAddr );
    /* Note64: check format strings and bitwise operations for 64 bit */
    NDK_sprintf( qBuf, "%d.%d.%d.%d.in-addr.arpa",
            (unsigned char)(IPAddr & 0xFF), (unsigned char)(IPAddr>>8 & 0xFF),
            (unsigned char)(IPAddr>>16 & 0xFF), (unsigned char)(IPAddr>>24 & 0xFF) );

    /* Send the query */
    errcode = DNSResolve( (unsigned char *)qBuf, AF_INET, T_PTR, pScrapBuf, size );

    return( errcode );
}

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
int DNSGetHostByName( char *Name, void *pScrapBuf, int size )
{
    char qBuf[256];
    char Domain[64];
    int  i,sz,rc,dot;
    int  errcode;

    /* Make sure we can handle the name */
    sz = strlen( Name );
    if( sz > 255 )
        return( NDK_DNS_EMEMERROR );

    /* Copy the name to the question buffer */
    strcpy( qBuf, Name );

    /* Scan the name for any '.' */
    dot = 0;
    for(i=0; i<sz; i++)
        if( qBuf[i] == '.' )
        {
            dot = 1;
            break;
        }

    /* If the name ends with a dot, remove it */
    if( qBuf[sz-1] == '.' )
        qBuf[sz-1] = 0;

    /* If the name has (had) a dot, we try it as-is first. */
    if( dot )
    {
        errcode = DNSResolve( (unsigned char *)qBuf, AF_INET, T_A, pScrapBuf, size );
        if( !errcode )
            return( 0 );
    }

    /* Here, there was no dot, or the name query failed. If we have */
    /* a domain name suffix and there's room in the buffer, concatenate */
    /* the domain name */

    /* Try and get our domain name */
    rc = NtGetPublicHost( 0, 64, (unsigned char *)Domain );
    if( rc && ((i + strlen(Domain) + 1) < 256) )
    {
        strcat( qBuf, "." );
        strcat( qBuf, Domain );
    }
    else
    {
        /* Here we failed to concatenate a domain name. If we've already */
        /* attempted to lookup the name as is, we go ahead and return the */
        /* error from the first lookup */
        if( dot )
            return( errcode );
    }

    /* Send the query */
    errcode = DNSResolve( (unsigned char *)qBuf, AF_INET, T_A, pScrapBuf, size );

    return( errcode );
}

#ifdef _INCLUDE_IPv6_CODE

/**
 *  @b Description
 *  @n
 *      This function does DNS lookup on the supplied hostname.
 *      On a successful return, the pScrapBuf can be treated as a
 *      HOSTENT structure. The size of the scrap buffer (size)
 *      must be greater than the size of the structure as the
 *      structure will contain pointers into the scrap buffer, and
 *      the scrap buffer is also used for temporary name storage.
 *      512 bytes should be sufficient for most DNS requests.
 *
 *      If the host name "Name" is terminated with a dot ('.'), the
 *      dot is removed. If the name contains a dot anywhere, it is
 *      used unmodified for an initial lookup. If the lookup fails -
 *      the appropriate DNS error code is returned. No default
 *      domain lookups are performed for IPv6, so if the hostname
 *      provided by user doesnt contain a dot, implying no
 *      domain name is provided, this function returns a foramt error.
 *
 *  @param[in]   Name
 *      The hostname to be resolved supplied by the user.
 *
 *  @param[in]   af_family
 *      The family (AF_INET/AF_INET6) of the IP address to which the
 *      query needs to be resolved to.
 *
 *  @param[out]  pScrapBuf
 *      Scrap buffer area to hold the results on a successful
 *      DNS resolution.
 *
 *  @param[in]  size
 *      Size of the scrap buffer available.
 *
 *  @retval
 *      Success -   0
 *
 *  @retval
 *      Error   -   >0, error code to determine the error.
 */
int DNSGetHostByName2(char *Name, unsigned char af_family, void *pScrapBuf, int size)
{
    char qBuf[256];
    int  i,sz,dot;
    int  errcode;

    /* We only support AF_INET/AF_INET6 families */
    if(af_family == AF_INET)
        return DNSGetHostByName (Name, pScrapBuf, size);
    else if (af_family != AF_INET6)
        return (NDK_DNS_ENOTIMP);

    /* Make sure we can handle the name */
    sz = strlen( Name );
    if( sz > 255 )
        return( NDK_DNS_EMEMERROR );

    /* Copy the name to the question buffer */
    strcpy( qBuf, Name );

    /* Scan the name for any '.' */
    dot = 0;
    for(i=0; i<sz; i++)
        if( qBuf[i] == '.' )
        {
            dot = 1;
            break;
        }

    /* If the name ends with a dot, remove it */
    if( qBuf[sz-1] == '.' )
        qBuf[sz-1] = 0;

    /* If the name has (had) a dot, i.e., the name supplied has a domain name in it
     * we try it as-is. Send a AAAA(Quad-A) DNS query.
     */
    if( dot )
    {
        /* Send the query and obtain the results. */
        errcode = DNSResolve( (unsigned char *)qBuf, AF_INET6, T_AAAA,
                              pScrapBuf, size );
    }
    else
        return (NDK_DNS_EFORMERR);

    return( errcode );
}

/**
 *  @b Description
 *  @n
 *      This function does reverse DNS lookup on the supplied
 *      IPv6 Address. On a successful return, pScrapBuf can be
 *      treated as a HOSTENT structure. The size of the scrap
 *      buffer (size) must be greater than the size of the structure
 *      as the structure will contain pointers into the scrap
 *      buffer, and the scrp buffer is also used for temporary
 *      name storage. 512 bytes of scrap buffer memory should be
 *      sufficient for most requests.
 *
 *  @param[in]   IPAddr
 *      The IPv6 address that needs to be resolved in IP6N format.
 *
 *  @param[out]  pScrapBuf
 *      Scrap buffer area to hold the results on a successful
 *      DNS resolution.
 *
 *  @param[in]  size
 *      Size of the scrap buffer available.
 *
 *  @retval
 *      Success -   0
 *
 *  @retval
 *      Error   -   >0, error code to determine the error.
 */
int DNSGetHostByAddr2( IP6N IPAddr, void *pScrapBuf, int size )
{
    char qBuf[80];
    int  errcode, i;
    char    *strIPAddress;
    char    tmpStr[6];
    char*   ptr_tmpStr;

    /* Allocate and Initialize the memory to hold the
     * IPv6 Address in reverse dotted notation for
     * Reverse DNS name lookup request.
     */
    if(!(strIPAddress = mmAlloc(sizeof(char) * 64)))
            return (NDK_DNS_EMEMERROR);
    memset(strIPAddress, 0, 64);

    /* Convert the IPv6 address to reverse dotted string format
     * as specified in RFC 3596.
     * For eg: If IPv6 address to be resolved is
     *  4321:0:1:2:3:4:567:89ab
     * then the reverse name lookup string would be
     *  b.a.9.8.7.6.5.0.4.0.0.0.3.0.0.0.2.0.0.0.1.0.0.0.0.0.0.0.1.2.3.4.ip6.arpa
     */
    for (i = 15; i >= 0; i--)
    {
        /* Initialize the temp string at the beginning of each loop iteration. */
        ptr_tmpStr  = &tmpStr[0];
        *ptr_tmpStr = 0;

        /* Reverse each byte of the IPv6 address */
        /* Note64: check format strings and bitwise operations for 64 bit */
        if(i == 0)
            NDK_sprintf (tmpStr, "%x.%x",IPAddr.u.addr8[i] & 0xf, (IPAddr.u.addr8[i] >> 4) & 0xf);
        else
            NDK_sprintf (tmpStr, "%x.%x.",IPAddr.u.addr8[i] & 0xf, (IPAddr.u.addr8[i] >> 4) & 0xf);

        /* Concatenate to the final output. */
        strcat (strIPAddress, tmpStr);
    }

    /* According to RFC 3596, Section 2.5, a new
     * domain has been defined for IPv6 address
     * resolution called "IP6.ARPA". This needs to
     * be appended to the IPv6 address for Reverse
     * Address lookups.
     */
    /* Note64: check format strings and bitwise operations for 64 bit */
    NDK_sprintf( qBuf, "%s.ip6.arpa", strIPAddress);

    /* Clean up the temp string */
    mmFree(strIPAddress);

    /* Send the query */
    errcode = DNSResolve( (unsigned char *)qBuf, AF_INET6, T_PTR, pScrapBuf, size );

    return( errcode );
}

/**
 *  @b Description
 *  @n
 *      Utility Function which converts an IPv6 Address Received in DNS
 *      Reply - PTR Record to a valid IPv6 address in IP6N format.
 *
 *  @param[in]   StringIP
 *      The IPv6 Address in String Format
 *
 *  @param[out]  address
 *      The IPv6 Address in IP6N format.
 *
 *  @retval
 *   Success    -   0
 *
 *  @retval
 *   Error      -   -1
 */
int IPv6PTRStringToIPAddress (char* StringIP, IP6N* address)
{
    char*   tok1;
    unsigned char t1 = 0, t2 = 0;
    int     index = 15;
    uint32_t  temp_val = 0;

    /* Basic Validations: */
    if ((StringIP == NULL) || (address == NULL))
        return -1;

    /* Initialize the IPv6 Address. */
    address->u.addr32[0] = 0;
    address->u.addr32[1] = 0;
    address->u.addr32[2] = 0;
    address->u.addr32[3] = 0;

    /* Now we convert; get the initial token. */
    tok1 = strtok (StringIP, ".");
    while ((tok1 != NULL) && (index >= 0))
    {
        /* Extract each nibble of a byte and convert it
         * to a hexadecimal number.
         */
        t1 = strtol (tok1, NULL, 16);
        tok1  = strtok (NULL, ".");
        t2 = strtol (tok1, NULL, 16);

        temp_val = ((t2 << 4) & 0xFF) | (t1 & 0xF);

        /* Copy each byte of the address */
        address->u.addr8[index] = (temp_val);

        /* Jump to the next token. */
        index = index - 1;
        tok1  = strtok (NULL, ".");
    }
    return 0;
}

#endif

/**
 *  @b Description
 *  @n
 *      This function is reponsible for building a DNS query,
 *      Getting the Reply and Decoding it into a HOSTENT structure.
 *
 *  @param[in]   qBuf
 *      The query to be resolved in string format.
 *
 *  @param[in]   af_family
 *      The family appropriate for the query (AF_INET/AF_INET6).Based
 *      on this the results are formatted appropriately.
 *
 *  @param[in]   type
 *      Query Type (T_A/T_AAAA/T_PTR)
 *
 *  @param[out]  pScrapBuf
 *      User allocated buffer to hold the query results.
 *
 *  @param[in]   size
 *      Maximum Size of the buffer passed.
 *
 *  @retval
 *   Success    -   0
 *
 *  @retval
 *   Error      -   >0, DNS error code.
 */
static int DNSResolve( unsigned char *qBuf, unsigned char af_family, uint16_t type,
                       void *pScrapBuf, int size )
{
    DNSREC      *pQuery;
    volatile DNSREC *pRec;
    DNSREPLY    *pReply;
    HOSTENT     *phe;
    unsigned char     *pbWrite;
    uint16_t    RecordCount;
    int         rc,writeused;
    uint32_t    IPTmp;
#ifdef _INCLUDE_IPv6_CODE
    IP6N        IPv6Tmp;
#endif

    /* validate that query str passed does not exceed DNS record name length */
    if (strlen((char *)qBuf) >= DNS_NAME_MAX) {
        return (NDK_DNS_EOVERFLOW);
    }

    /* Allocate a query record */
    if( !(pQuery = mmAlloc( sizeof(DNSREC) )) )
        return( NDK_DNS_EMEMERROR );

    memset( pQuery, 0, sizeof( DNSREC ) );

    /* Init query record */
    strcpy( (char *)pQuery->Name, (char *)qBuf );
    pQuery->Type  = type;
    pQuery->Class = C_IN;

    /* Resolve the query */
    rc = DNSResolveQuery( DNS_OP_STD, pQuery, &pReply );
    mmFree( pQuery );

    if( !rc )
        return( NDK_DNS_ENODNSREPLY );

    /* If the reply contains an error, return it */
    rc = (pReply->Flags & MASK_DNS_RCODE);
    if( rc )
        goto drleave;

    /* Decode the reply */

    /* "Allocate" the HOSTENT structure */
    phe = (HOSTENT *)pScrapBuf;
    pbWrite = (unsigned char *)pScrapBuf + sizeof( HOSTENT );
    /*
     * Note64: sizeof returns size_t, which is 64 bit unsigned for
     * LP64. May need to cast to int32_t to avoid implicit type conversions
     */
    size -= sizeof( HOSTENT );

    /* Initialize the structure */
    phe->h_name     = 0;
    phe->h_addrtype = AF_INET;
    phe->h_length   = 4;
    phe->h_addrcnt  = 0;

#ifdef _INCLUDE_IPv6_CODE
    /*
     * Initialize all pointers in h_addr_list[] array to NULL (reuse
     * writeused variable for the loop counter). Previously, code to do this
     * had potential to write outside of the array boundary.
     */
    for (writeused = 0; writeused < NDK_DNS_MAXIPADDR; writeused++) {
        phe->h_addr_list[writeused] = 0;
    }
#endif

    /* Read the Answers */
    RecordCount = pReply->NumAns;
    pRec = pReply->pAns;
    while( RecordCount-- )
    {
        if( pRec->Class == C_IN )
        {
            /* Copy the name to the write buffer */
            writeused = strlen( (char *)(pRec->Name) ) + 1;
            if( writeused > size )
            {
                rc = NDK_DNS_EOVERFLOW;
                goto drleave;
            }
            strcpy( (char *)pbWrite, (char *)(pRec->Name) );

            /* What we do next is dependent on the type */
            switch( pRec->Type )
            {
            case T_A:
                /* IP Address Record */

                /* We only use this if the record size is == 4 */
                if( pRec->DataLength == 4 )
                {
                    /* If we don't have a CNAME yet, we'll use the */
                    /* "owner" of this record. */
                    if( !phe->h_name )
                    {
                        phe->h_name = (char *)pbWrite;
                        /* we need to advance the pbWrite pointer to
                         * skip over the Hostname and also the null
                         * byte termination after it, Thus the
                         * (writeused + 1).
                         */
                        pbWrite += (writeused + 1);
                        size -= (writeused + 1);
                    }

                    /* Get the IP address */
                    /* Note64: check bitwise operations for 64 bit */
                    IPTmp =  ((uint32_t)pRec->Data[0] << 24);
                    IPTmp |= ((uint32_t)pRec->Data[1] << 16);
                    IPTmp |= ((uint32_t)pRec->Data[2] << 8);
                    IPTmp |= ((uint32_t)pRec->Data[3]);
                    IPTmp = NDK_htonl( IPTmp );

#ifdef _INCLUDE_IPv6_CODE
                    writeused = sizeof(IPTmp) + 1;
                    if( writeused > size )
                    {
                        rc = NDK_DNS_EOVERFLOW;
                        goto drleave;
                    }


                    /* Copy the IPv4 address to the scrap buffer area */
                    WrNet32(pbWrite, IPTmp);

                    /* Finally set the HOSTENT address to point to the
                     * IPv4 address saved in the scrap buffer.
                     */
                    if( phe->h_addrcnt < NDK_DNS_MAXIPADDR )
                        phe->h_addr_list[ phe->h_addrcnt++ ] = (char *)pbWrite;

                    /*
                     * Increment the write position in our scrap buffer so that
                     * the IP address just saved to it (and now referenced in
                     * h_addr_list[]) won't be overwritten in the next iteration.
                     * Also update the total scrap buffer size available.
                     */
                    pbWrite += writeused;
                    size -= writeused;

#else
                    if( phe->h_addrcnt < NDK_DNS_MAXIPADDR )
                        phe->h_addr[ phe->h_addrcnt++ ] = IPTmp;
#endif
                }
                break;

#ifdef _INCLUDE_IPv6_CODE
            case T_AAAA:
                /* IPv6 Address Record */

                /* We only use this if the record size is 16 bytes */
                if( pRec->DataLength == 16 )
                {
                    /* If we don't have a CNAME yet, we'll use the */
                    /* "owner" of this record. */
                    if( !phe->h_name )
                    {
                        phe->h_name = (char *)pbWrite;
                        /* we need to advance the pbWrite pointer to
                         * skip over the Hostname and also the null
                         * byte termination after it, Thus the
                         * (writeused + 1).
                         */
                        pbWrite += (writeused + 1);
                        size -= (writeused + 1);
                    }

                    /* Configure the appropriate address family
                     * type and length.
                     */
                    phe->h_addrtype = AF_INET6;
                    phe->h_length   = 16;

                    /* Get the IPv6 address */
                    writeused = sizeof(IP6N) + 1;
                    if( writeused > size )
                    {
                        rc = NDK_DNS_EOVERFLOW;
                        goto drleave;
                    }

                    /* Save the IPv6 address in the scrap buffer area. */
                    mmCopy( (void *)pbWrite, (void *)(&pRec->Data[0]), writeused );

                    /* Finally, initialize the HOSTENT structure h_addr_list to
                     * point to the IPv6 address saved in the scrap buffer.
                     */
                    if( phe->h_addrcnt < NDK_DNS_MAXIPADDR )
                        phe->h_addr_list[ phe->h_addrcnt++ ] = (char *)pbWrite;

                    /*
                     * Increment the running pointer in Scrap buffer to skip
                     * over what we just wrote. Also decrement the size used
                     * in the scrap buffer.
                     */
                    pbWrite += writeused;
                    size -= writeused;
                }
                break;
#endif

            case T_PTR:
                /* Pointer Record (owner of an IP) */

                /* We'll take the IP address from this record if we */
                /* can */
                if(af_family == AF_INET)
                {
                    IPTmp = inet_addr( (char *)pbWrite );

                    /* The IPaddress obtained in DNS reverse name
                     * lookup reply is in reverse dotted notation.
                     * So convert the IP address to format we
                     * understand.
                     */
                    IPTmp = ((IPTmp>>24)&0xFF) | ((IPTmp>>8)&0xFF00) |
                            ((IPTmp<<8)&0xFF0000) | ((IPTmp<<24)&0xFF000000);

#ifdef _INCLUDE_IPv6_CODE
                    writeused = sizeof(IPTmp) + 1;
                    if( writeused > size )
                    {
                        rc = NDK_DNS_EOVERFLOW;
                        goto drleave;
                    }

                    /* Copy the IPv4 address to the scratch pad area */
                    mmCopy( (void *)pbWrite, (void *)(&IPTmp), writeused );

                    /* Finally, initialize the HOSTENT structure h_addr_list to
                     * point to the IPv4 address saved in the scrap buffer.
                     */
                    if( phe->h_addrcnt < NDK_DNS_MAXIPADDR )
                        phe->h_addr_list[ phe->h_addrcnt++ ] = (char *)pbWrite;

                    /* Increment the running pointer in Scrap buffer to skip
                     * over what we just wrote. Also decrement the size used
                     * in the scrap buffer.
                     */
                    pbWrite += writeused;
                    size -= writeused;
#else
                    if( phe->h_addrcnt < NDK_DNS_MAXIPADDR )
                        phe->h_addr[ phe->h_addrcnt++ ] = IPTmp;
#endif

                }
#ifdef _INCLUDE_IPv6_CODE
                else if (af_family == AF_INET6)
                {
                    /* Configure the appropriate address family
                     * type and length.
                     */
                    phe->h_addrtype = AF_INET6;
                    phe->h_length   = 16;

                    /* Convert the IPv6 address obtained from reverse
                     * DNS lookup to a format we understand.
                     */
                    IPv6PTRStringToIPAddress((char *)pbWrite, &IPv6Tmp);

                    writeused = sizeof(IP6N) + 1;
                    if( writeused > size )
                    {
                        rc = NDK_DNS_EOVERFLOW;
                        goto drleave;
                    }

                    /* Copy the IPv6 address to the scratch pad area */
                    mmCopy( (void *)pbWrite, (void *)(&IPv6Tmp), writeused );

                    /* Finally, initialize the HOSTENT structure h_addr_list to
                     * point to the IPv6 address saved in the scrap buffer.
                     */
                    if( phe->h_addrcnt < NDK_DNS_MAXIPADDR )
                        phe->h_addr_list[ phe->h_addrcnt++ ] = (char *)pbWrite;

                    /* Increment the running pointer in Scrap buffer to skip
                     * over what we just wrote. Also decrement the size used
                     * in the scrap buffer.
                     */
                    pbWrite += writeused;
                    size -= writeused;
                }
#endif
                /* Fall through */

            case T_CNAME:
                /* CNAME Record */

                /* Copy the data to the write buffer */
                writeused = pRec->DataLength + 1;
                if( writeused > size )
                {
                    rc = NDK_DNS_EOVERFLOW;
                    goto drleave;
                }

                /* Copy the "Host Name" present in the
                 * Domain Name field of the DNS reply.
                 */
                strncpy( (char *)pbWrite, (char *)(pRec->Data), writeused );

                /* Save the CNAME as the host name */
                phe->h_name = (char *)pbWrite;

                /* we need to advance the pbWrite pointer to
                 * skip over the Hostname and also the null
                 * byte termination after it, Thus the
                 * (writeused + 1).
                 */
                pbWrite += (writeused + 1);
                size -= (writeused - 1);
                break;
            }
        }

        /* Parse the next record */
        pRec = pRec->pNext;
    }

drleave:
    DNSReplyFree( pReply, 1 );

    return( rc );
}

/*
 * Utility fxn to do a DNS lookup, placing results into user supplied buffer in
 * host byte order. This function will handle the appropriate calls to
 * DNSGetHostByName and DNSGetHostByName2, depending on whether or not
 * _INCLUDE_IPv6_CODE was defined or not. It also handles the differences
 * between the HOSTENT struct with respect to the same macro.
 *
 * Parameters:
 *    name:       The host name to perform a DNS look up on
 *
 *    family:     AF_INET or AF_INET6
 *
 *    ipBuffer:   User supplied buffer, used to store IP addresses in host byte
 *                order.
 *
 *    numIpAddrs: A value/result parameter. Upon entry, it should be set to the
 *                number of IP addresses the supplied buffer can hold. Upon
 *                successful return, it will store the number of ip addresses
 *                that were written to the buffer, or zero upon failure.
 *
 * Returns 0 if OK, or error code
 */
int DNSGetHostArrayByName(const char *name, uint8_t family, uint32_t *ipBuffer,
        uint16_t *numIpAddrs)
{
    int i;
    uint32_t bytesLeft;
    int retval = NDK_DNS_NOERROR;
    HOSTENT *dnsInfo = NULL;
    char *buffer = NULL;
#ifdef _INCLUDE_IPv6_CODE
    IP6N *ipv6Tmp = NULL;
#endif

    if (!ipBuffer || !numIpAddrs || !name) {
        /* Error: invalid args passed */
        return (NDK_DNS_EINVALIDARGS);
    }

    /*
     * Upon entry, numIpAddrs contains the number of IP addresses that can fit
     * in the ipBuffer buffer. Convert this to bytes:
     */
    if (family == AF_INET) {
        bytesLeft =  (*numIpAddrs) * sizeof(uint32_t);
    }
    else if (family == AF_INET6) {
#ifndef _INCLUDE_IPv6_CODE
        /* Error: can't call this API for IPv6 in IPv4 only stack */
        return (NDK_DNS_EIPv6DISABLED);
#else
        bytesLeft =  (*numIpAddrs) * sizeof(IP6N);
#endif
    }
    else {
        /* Error: invalid family */
        return (NDK_DNS_EINVFAMILY);
    }

    /* Allocate buffer needed for DNS */
    buffer = calloc(1, NDK_DNSBUFSIZE);
    if (!buffer) {
        /* Error, could not allocate memory */
        return (NDK_DNS_EMEMERROR);
    }

    /*
     * Reset address count
     * It will be incremented for each successful IP resolution, if any
     */
    *numIpAddrs = 0;

    dnsInfo = (HOSTENT *)buffer;

    /* IPv4 DNS lookup */
    if (family == AF_INET) {
        retval = DNSGetHostByName((char *)name, buffer, NDK_DNSBUFSIZE);
        if (retval == NDK_DNS_NOERROR) {
            for (i = 0; i < dnsInfo->h_addrcnt; i++) {
                if (bytesLeft >= sizeof(uint32_t)) {
                    /* Store this result into the provided buffer. */
#ifdef _INCLUDE_IPv6_CODE
                    /*
                     * h_addr_list[i] was written to using WrNet32. Must read
                     * it out using RdNet32 (should come back in net byte
                     * order. Final step is to convert it to host byte order, as
                     * this is the expected format in SlNetIfNDK_getHostByName:
                     *
                     */
                    *ipBuffer = NDK_ntohl(RdNet32(dnsInfo->h_addr_list[i]));
#else
                    /*
                     * The IPv4 address in the DNS struct should be in net
                     * byte order. Convert to host byte order, as
                     * this is the expected format in SlNetIfNDK_getHostByName:
                     */
                    *ipBuffer = NDK_ntohl(dnsInfo->h_addr[i]);
#endif
                    /* update the available space after writing the IPv4 addr */
                    bytesLeft -= sizeof(uint32_t);

                    /* move to the next free spot in the buffer */
                    ipBuffer++;

                    /* increment the buffer's address count */
                    (*numIpAddrs)++;
                }
                else {
                    /* Insufficient space to store the next address */
                    break;
                }
            }
        }
    }
#ifdef _INCLUDE_IPv6_CODE
    /* IPv6 DNS lookup */
    else if (family == AF_INET6) {
        retval = DNSGetHostByName2((char *)name, AF_INET6, buffer,
                NDK_DNSBUFSIZE);

        if (retval == NDK_DNS_NOERROR) {
            /* Workaround for CQ114965 (IPv6 DNS false positive) */
            if (dnsInfo->h_addrcnt == 0) {
                /* Skip loop and return error */
                return (NDK_DNS_ENODNSREPLY);
            }

            for (i = 0; i < dnsInfo->h_addrcnt; i++) {
                if (bytesLeft >= sizeof(IP6N)) {

                    /* copy out the IPv6 address from the DNS reply */
                    memcpy(ipBuffer, dnsInfo->h_addr_list[i], sizeof(IP6N));

                    /* update the available space after writing the IPv6 addr */
                    bytesLeft -= sizeof(IP6N);

                    /*
                     * Change byte ordering to host order, as expected by
                     * SlNetIfNdk_getHostByName
                     */
                    ipv6Tmp = (IP6N *)ipBuffer;
                    ipv6Tmp->u.addr16[0] = NDK_ntohs(ipv6Tmp->u.addr16[0]);
                    ipv6Tmp->u.addr16[1] = NDK_ntohs(ipv6Tmp->u.addr16[1]);
                    ipv6Tmp->u.addr16[2] = NDK_ntohs(ipv6Tmp->u.addr16[2]);
                    ipv6Tmp->u.addr16[3] = NDK_ntohs(ipv6Tmp->u.addr16[3]);
                    ipv6Tmp->u.addr16[4] = NDK_ntohs(ipv6Tmp->u.addr16[4]);
                    ipv6Tmp->u.addr16[5] = NDK_ntohs(ipv6Tmp->u.addr16[5]);
                    ipv6Tmp->u.addr16[6] = NDK_ntohs(ipv6Tmp->u.addr16[6]);
                    ipv6Tmp->u.addr16[7] = NDK_ntohs(ipv6Tmp->u.addr16[7]);

                    /* move to the next free spot in the buffer */
                    ipBuffer += sizeof(IP6N) / sizeof(uint32_t);

                    /* increment the buffer's address count */
                    (*numIpAddrs)++;
                }
                else {
                    /* Insufficient space to store the next address */
                    break;
                }
            }
        }
    }
#endif

    free(buffer);

    return (retval);
}
