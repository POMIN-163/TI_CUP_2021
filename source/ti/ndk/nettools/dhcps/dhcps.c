/*
 * Copyright (c) 2014-2019, Texas Instruments Incorporated
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
 * ======== dhcps.c ========
 *
 * Simple DHCP Server Utility
 *
 */
#include <string.h>
#include <stdint.h>

#include "dhcps.h"

#define BUFFER_LENGTH           1024
#define TAGBUF_LENGTH           32
#define EXPIRATION_TIME         (86400 * 7)   /* 7 Days */
#define TEMPREC_TIME            (3600 * 4)    /* 4 Hours */
#define MAX_OPTION_SIZE         312
#define DHCP_RECORD             CI_CLIENT

/* DHCP Instance */
typedef struct _dhcpsrv {
        void              *hTask;         /* Handle to server task */
        void              *hIF;           /* Handle to target IF */
        void              *hCb;           /* Callback handle */
        void (*pCb)(void *,uint32_t);     /* Callback function */
        uint32_t           IfIdx;         /* Interface Index */
        uint32_t           IPServer;      /* Server's IP address */
        uint32_t           IPMask;        /* Subnet mask */
        uint32_t           IPNameServer;  /* DNS to report to client (or NULL) */
        uint32_t           PoolSize;      /* Number of addrs in pool */
        uint32_t           PoolBase;      /* Addr base */
        DHCP_RECORD        ipr;           /* Current IP Record */
        void              *hIPR;          /* Handle to the IPR we're holding */

        /* Socket state variables */

        SOCKET             s;
        struct sockaddr_in sin1;
        int                size;
        int                DomainLength;
        unsigned char            DomainName[CFG_DOMAIN_MAX];
        char               buf[BUFFER_LENGTH];
        int                tagsize;
        char               tags[TAGBUF_LENGTH];
        } DHCPSRV;

/* DHCPS Main Thread */
static void dhcpsMain( DHCPSRV *pDS );

/* DHCP IP Address Entry Record Functions */
#define IPR_MODE_FIND          1        /* Find Only */
#define IPR_MODE_CREATE        2        /* Create entry if not found */
#define IPR_MODE_CREATEFIXED   3        /* Create entry only if IP available */

static int  iprMacCmp( unsigned char *pMAC1, unsigned char *pMAC2 );
static void iprSync( DHCPSRV *pDS );
static int  iprGetByIndex( DHCPSRV *pDS, uint32_t Index );
static int  iprNew( DHCPSRV *pDS, uint32_t mode, uint32_t IPReqAddr, unsigned char *pMAC );
static void iprRemove( DHCPSRV *pDS );
static void iprUpdate( DHCPSRV *pDS, uint32_t Status );
static void iprTimeoutCheck( DHCPSRV *pDS );
static int  iprFindByIP( DHCPSRV *pDS, uint32_t IPReqAddr );
static int  iprFindByMAC( DHCPSRV *pDS, unsigned char *pMAC );


/* DHCP Server Packet Processing Functions */
static void ProcessMain( DHCPSRV *pDS );
static void SendDHCPReply( DHCPSRV *pDS, uint32_t IPAddr, unsigned char Message,
                           uint32_t flags);
#define SENDFLG_LEASETIME       1

/*---------------------------------------------------------------- */
/* DHCPSOpen() */

/* This function is called to initiate DHCPS control of the IP */
/* subnet on a given device. The base address does not have to be the */
/* base addr of the subnet (as defined by the mask). */

/* Returns a HANDLE to a DHCPS instance structure which is used in */
/* calls to other DHCPS functions like DHCPSClose(). */
/*---------------------------------------------------------------- */
void *DHCPSOpen( NTARGS *pNTA, NTPARAM_DHCPS *pNTP )
{
    DHCPSRV  *pDS;
    CI_IPNET ci_net;
    int      i,rc;

    /* We only support IFIDX calling mode */
    if( pNTA->CallMode != NT_MODE_IFIDX )
        return(0);

    /* Allocate Instance */
    pDS = mmBulkAlloc( sizeof(DHCPSRV) );
    if( !pDS )
        return(0);

    /* Initialize Instance */
    memset( pDS, 0, sizeof( DHCPSRV ) );

    /* Get the HANDLE to the interface */
    pDS->IfIdx = pNTA->IfIdx;

    {
        NIMU_IF_REQ ifreq;

        /* Initialize the NIMU Interface Object. */
        mmZeroInit (&ifreq, sizeof(NIMU_IF_REQ));

        /*
         *  We are interested in receiving the handle associated with 'index'
         *  Item
         */
        ifreq.index = pDS->IfIdx;
        if (NIMUIoctl(NIMU_GET_DEVICE_HANDLE, &ifreq, (void *)&pDS->hIF,
                sizeof(void *)) < 0)
            goto error_abort;
    }

    /* Search CFG for network with DHCPS flag set */
    i = 1;
    while(i)
    {
        rc = CfgGetImmediate( 0, CFGTAG_IPNET, pNTA->IfIdx, i,
                              sizeof(ci_net), (unsigned char *)&ci_net );
        if( rc <= 0 )
            goto error_abort;

        if( ci_net.NetType & CFG_NETTYPE_DHCPS )
            break;

        i++;
    }

    /* Get the server IP and the mask */
    pDS->IPServer = ci_net.IPAddr;
    pDS->IPMask   = ci_net.IPMask;

    /* Record the IP of the local name server if we're to use it */
    if( pNTP->Flags & DHCPS_FLG_LOCALDNS )
        pDS->IPNameServer = pDS->IPServer;

    /* Copy the domain name if in VIRTUAL mode else get the public */
    /* domain name. */
    if( pNTP->Flags & DHCPS_FLG_LOCALDOMAIN )
        strcpy( (char *)(pDS->DomainName), ci_net.Domain );
    else
        NtGetPublicHost( 0, CFG_DOMAIN_MAX, pDS->DomainName );
    pDS->DomainLength = strlen( (char *)(pDS->DomainName) );

    /* Save the Callback Information */
    pDS->hCb = pNTA->hCallback;
    pDS->pCb = pNTA->pCb;

    /* Convert optional pool to host format */
    pDS->PoolBase   = NDK_htonl( pNTP->PoolBase );
    pDS->PoolSize   = pNTP->PoolCount;

    /* Verify server address can serve the supplied pool */
    if( pDS->IPServer && pNTP->PoolBase )
        if( (pDS->IPServer & pDS->IPMask) != (pNTP->PoolBase & pDS->IPMask) )
            goto error_abort;

    /* Init the socket value */
    pDS->s = INVALID_SOCKET;

    /* Create the DHCPS task */
    pDS->hTask = TaskCreate( (void(*)())dhcpsMain, "DHCPserver",
                             OS_TASKPRINORM, OS_TASKSTKLOW,
                             (uintptr_t)pDS, 0, 0 );

    if( !pDS->hTask )
        goto error_abort;

    /* Return the handle to our instance */
    return( pDS );

error_abort:
    mmBulkFree( pDS );
    return(0);
}

/*---------------------------------------------------------------- */
/* DHCPSClose() */

/* This function terminates DHCPS control and free the supplied */
/* instance handle. */
/*---------------------------------------------------------------- */
void DHCPSClose( void *hDHCPS )
{
    DHCPSRV     *pDS = (DHCPSRV *)hDHCPS;

    /* Kill the server task */
    if( pDS->hTask )
    {
        fdCloseSession( pDS->hTask );
        TaskDestroy( pDS->hTask );
    }

    /* Close the socket */
    if( pDS->s != INVALID_SOCKET )
        fdClose( pDS->s );

    /* Free any held record */
    iprSync( pDS );

    /* Free the instance */
    mmBulkFree( pDS );
}

/*---------------------------------------------------------------- */
/* dhcpsMain() */

/* This routine listens for packets and processes requests */
/*---------------------------------------------------------------- */
static void dhcpsMain( DHCPSRV *pDS )
{
    int tmp;

    fdOpenSession( TaskSelf() );

    /* Create the main socket */
    pDS->s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if( pDS->s == INVALID_SOCKET )
        goto leave;

    /* Bind to the DHCP Server port */
    memset( &pDS->sin1, 0, sizeof(struct sockaddr_in) );
    pDS->sin1.sin_family      = AF_INET;
    pDS->sin1.sin_addr.s_addr = 0;
    pDS->sin1.sin_port        = NDK_htons(NDHCPS);

    /*
     * Note64: sizeof returns size_t, which is 64 bit unsigned for
     * LP64. May need to cast to int32_t to avoid implicit type conversions
     */
    if( bind( pDS->s, (struct sockaddr *)&pDS->sin1, sizeof( pDS->sin1 ) ) < 0 )
        goto leave;

    /* Set socket option to allow us to broadcast */
    tmp = 1;
    if( setsockopt( pDS->s, SOL_SOCKET, SO_BROADCAST, &tmp, sizeof(int) ) )
        goto leave;

    /* Set socket option to specifiy packet broadcast device */
    /* The SO_IFDEVICE option is specific to this stack. */
    if( setsockopt(pDS->s, SOL_SOCKET, SO_IFDEVICE, &pDS->IfIdx,
        sizeof(uint32_t)) ) {
        goto leave;
    }

    /* Inform that all is well with the world */
    if( pDS->pCb )
        (*pDS->pCb)( pDS->hCb, NETTOOLS_STAT_RUNNING );

    /* Listen for incoming requests */
    /* Run until task is destroyed by the system */
    for(;;)
    {
        NDK_fd_set ibits;

        /* Initialize the select flags */
        NDK_FD_ZERO(&ibits);
        NDK_FD_SET(pDS->s, &ibits);

        /*
         *  Wait for socket activity (1st param of fdSelect is a don't care,
         *  pass 0 for 64 bit compatibility)
         */
        if( fdSelect( 0, &ibits, 0, 0, 0 ) < 0 )
            goto leave;

        /* Check for new data on UDP socket */
        if( NDK_FD_ISSET(pDS->s, &ibits) )
        {
            /*
             * Note64: sizeof returns size_t, which is 64 bit unsigned for
             * LP64. May need to cast to int32_t to avoid implicit type conversions
             */
            tmp  = sizeof( pDS->sin1 );
            pDS->size = recvfrom( pDS->s, pDS->buf,
                                  sizeof(pDS->buf), 0,(struct sockaddr *)&pDS->sin1, &tmp);

            /* Process the request */
            if( pDS->size > 0 )
                ProcessMain( pDS );
        }
    }

leave:
    if( pDS->s != INVALID_SOCKET )
    {
        fdClose( pDS->s );
        pDS->s = INVALID_SOCKET;
    }

    /* Update status */
    if( pDS->pCb )
        (*pDS->pCb)( pDS->hCb, NETTOOLS_STAT_FAULT );

    /* We don't return - we're killed by the system */
    TaskBlock( TaskSelf() );
}

/*---------------------------------------------------------------- */
/*---------------------------------------------------------------- */
/* DHCPS - IP Database Functions */
/* The IP database is used to track MAC/IP address relationships. */
/* These functions are used to create / lookup address entries, */
/* and to age out invalid or long-pending entries. */
/*---------------------------------------------------------------- */
/*---------------------------------------------------------------- */

/*---------------------------------------------------------------- */
/* iprMacCmp - Hawrdware Address Compare */
/* Returns NULL if addresses are identical */
/*---------------------------------------------------------------- */
static int iprMacCmp( unsigned char *pMAC1, unsigned char *pMAC2 )
{
    int i;
    for( i=1; i<=6; i++ )
        if( *pMAC1++ != *pMAC2++ )
            return( i );
    return(0);
}

/*---------------------------------------------------------------- */
/* iprSync - Free any held IPR */
/*---------------------------------------------------------------- */
static void iprSync( DHCPSRV *pDS )
{
    if( pDS->hIPR )
    {
        CfgEntryDeRef( pDS->hIPR );
        pDS->hIPR = 0;
    }
}

/*---------------------------------------------------------------- */
/* iprGetByIndex - Get an IPR by Index */
/*---------------------------------------------------------------- */
static int iprGetByIndex( DHCPSRV *pDS, uint32_t Index )
{
    void *hCfgEntry;
    int    rc,size;

    /* Just to make sure... */
    iprSync( pDS );

    /* Get entry */
    rc = CfgGetEntry( 0, CFGTAG_CLIENT, pDS->IfIdx, Index, &hCfgEntry );
    if( rc < 1 )
        return(0);

    /* Get data */
    size = sizeof( DHCP_RECORD );
    CfgEntryGetData( hCfgEntry, &size, (unsigned char *)&pDS->ipr );

    /* Remember that we're holding this entry */
    pDS->hIPR = hCfgEntry;

    return(1);
}

/*---------------------------------------------------------------- */
/* iprNew - Find or create a new address entry */
/*---------------------------------------------------------------- */
static int iprNew( DHCPSRV *pDS, uint32_t mode, uint32_t IPReqAddr, unsigned char *pMAC )
{
    uint32_t    i,j=0;
    int         rc;

    /* Sanity check the mode */
    if( mode < IPR_MODE_FIND || mode > IPR_MODE_CREATEFIXED )
        return(0);

    /* First, try and find an entry */
    if( iprFindByMAC( pDS, pMAC ) )
    {
        if( mode == IPR_MODE_CREATEFIXED && IPReqAddr != pDS->ipr.IPAddr )
            return( 0 );
        return( 1 );
    }

    /* Here we must create an entry. If not set to create, return NULL */
    if( mode == IPR_MODE_FIND )
        return( 0 );

    /* See if we can use the preferred IP */
    i = NDK_htonl( IPReqAddr );

    /* If the address is out of range, we can't use it */
    /* else, see if the address has already been allocated */
    if( i<pDS->PoolBase || i>=(pDS->PoolBase+pDS->PoolSize) ||
                                          (j=iprFindByIP(pDS, IPReqAddr)) )
    {
        /* Requested IP is invalid or in use. If in use and marked */
        /* INVALID, we'll may allow use of the IP. This is because another */
        /* client could have claimed it invalid, although its "OK" for */
        /* this client - after all, it is specifically requesting it. */
        if( i && j )
        {
            if( pDS->ipr.Status == CFG_CLIENTSTATUS_INVALID )
            {
                /* The MAC address of this client can not match that */
                /* of the client that marked it invalid */
                for(j=0; j<6; j++)
                    if( *(pMAC+j) != *(pDS->ipr.MacAddr+j) )
                        break;
                if( j != 6 )
                {
                    /* We will attempt to reuse this IP addr. First, */
                    /* remove the INVALID record. */
                    iprRemove( pDS );
                    goto IPADDROK;
                }
            }
        }

        /* Requested IP is invalid or in use - get a new one */

        /* If the user only wanted the requested IP, then we're done */
        if( mode == IPR_MODE_CREATEFIXED )
            return(0);

        /* First try and time out any old entries */
        iprTimeoutCheck( pDS );

        /* Find a valid IP */
        for( i=0; i<pDS->PoolSize; i++ )
        {
            j = pDS->PoolBase + i;
            j = NDK_htonl( j );
            if( !iprFindByIP(pDS, j) )
                break;
        }

        /* If we're full, its an error */
        if( i == pDS->PoolSize )
            return( 0 );

        /* Change the requested IP to the free value we've decided to use */
        IPReqAddr = j;
    }

IPADDROK:
    /* If we're holding an IPR, ditch it */
    iprSync( pDS );

    /* Here, IPReqAddr contains a legal IP address. We just need to */
    /* intialize the entry. */
    pDS->ipr.ClientType = CFG_CLIENTTYPE_DYNAMIC;
    pDS->ipr.IPAddr     = IPReqAddr;
    pDS->ipr.Status     = CFG_CLIENTSTATUS_PENDING;
    pDS->ipr.TimeExpire = EXPIRATION_TIME;
    pDS->ipr.TimeStatus = llTimerGetTime(0);
    memmove( pDS->ipr.MacAddr, pMAC, 6 );

    /* Add it to the configuration */
    rc = CfgAddEntry( 0, CFGTAG_CLIENT, pDS->IfIdx, CFG_ADDMODE_NOSAVE,
                      sizeof( DHCP_RECORD ), (unsigned char *)&(pDS->ipr), &pDS->hIPR );
    if( rc < 0 )
        return(0);

    return( 1 );
}

/*---------------------------------------------------------------- */
/* iprRemove - Remove the current ipr */
/*---------------------------------------------------------------- */
static void iprRemove( DHCPSRV *pDS )
{
    if( pDS->hIPR )
    {
        /* If entry is one of ours, remove it. */
        if( pDS->ipr.ClientType == CFG_CLIENTTYPE_DYNAMIC )
        {
            CfgRemoveEntry( 0, pDS->hIPR );
            pDS->hIPR = 0;
        }
        /* Else just clear its state and deref */
        else
        {
            iprUpdate( pDS, 0 );
            iprSync( pDS );
        }
    }
}

/*---------------------------------------------------------------- */
/* iprUpdate - Set status and update IP record */
/*---------------------------------------------------------------- */
static void iprUpdate( DHCPSRV *pDS, uint32_t Status )
{
    if( pDS->hIPR )
    {
        pDS->ipr.Status = Status;
        pDS->ipr.TimeStatus = llTimerGetTime(0);
        CfgEntrySetData( pDS->hIPR, sizeof( DHCP_RECORD ), (unsigned char *)&pDS->ipr );
    }
}

/*---------------------------------------------------------------- */
/* iprTimeoutCheck - Scan list and timeout entries */
/*---------------------------------------------------------------- */
static void iprTimeoutCheck( DHCPSRV *pDS )
{
    uint32_t TimeNow;
    int    i=1;

    /* Get the current time */
    TimeNow = llTimerGetTime(0);

    for(;;)
    {
        /* Get the next entry */
        if( !iprGetByIndex( pDS, i ) )
            return;

        /* Remove any expired entry */

        /* STATIC entires are not subject to timeout */
        /* INVALID and PENDING timeout on a fixed TEMPREC timeout */
        /* VALID entries use the supplied timout */
        if( pDS->ipr.Status != CFG_CLIENTSTATUS_STATIC )
        {
            if( pDS->ipr.Status != CFG_CLIENTSTATUS_VALID )
            {
                if( (pDS->ipr.TimeStatus + TEMPREC_TIME) <= TimeNow )
                    iprRemove( pDS );
            }
            else
            {
                if( (pDS->ipr.TimeStatus + pDS->ipr.TimeExpire) <= TimeNow )
                    iprRemove( pDS );
            }
        }
        i++;
    }
}

/*---------------------------------------------------------------- */
/* iprFindByIP - Find Record by IP Address */
/*---------------------------------------------------------------- */
static int iprFindByIP( DHCPSRV *pDS, uint32_t IPReqAddr )
{
    int i=1;

    for(;;)
    {
        /* Get the next entry */
        if( !iprGetByIndex( pDS, i ) )
            return(0);

        if( pDS->ipr.IPAddr == IPReqAddr )
            return(1);

        i++;
    }
}

/*---------------------------------------------------------------- */
/* iprFindByMAC - Find Record by MAC Address */
/*---------------------------------------------------------------- */
static int iprFindByMAC( DHCPSRV *pDS, unsigned char *pMAC )
{
    int i=1;

    for(;;)
    {
        /* Get the next entry */
        if( !iprGetByIndex( pDS, i ) )
            return(0);

        /* Only care about VALID entries */
        if( pDS->ipr.Status != CFG_CLIENTSTATUS_INVALID )
        {
            if( !iprMacCmp( (unsigned char *)(pDS->ipr.MacAddr), pMAC ) )
                return(1);
        }

        i++;
    }
}

/*---------------------------------------------------------------- */
/*---------------------------------------------------------------- */
/* DHCPS - Network Packet Processing */
/*---------------------------------------------------------------- */
/*---------------------------------------------------------------- */

/*---------------------------------------------------------------- */
/* ProcessMain() */
/*---------------------------------------------------------------- */
static void ProcessMain( DHCPSRV *pDS )
{
    DHCPS       *pHdr = (DHCPS *)pDS->buf;
    unsigned char     Hostname[CFG_HOSTNAME_MAX];
    unsigned char     *pv;
    uint16_t    MessageType;
    int         TagLength,TmpSize;
    uint32_t    IPRequested = 0;
    uint32_t    IPClaimed = 0;
    uint32_t    tmp;
    int         ipr=0;

    MessageType = 0;
    memset(&Hostname, 0, sizeof(Hostname));

    /*  If packet not for us, then just return */
    if( (pHdr->op != BOOTREQUEST) ||
                     (pHdr->htype != ETHERNET) || (pHdr->hlen != ELEN ) )
        return;

    /* PV is our options pointer */
    pv = (unsigned char *)pHdr->options + 4;
    /*
     * Note64: sizeof returns size_t, which is 64 bit unsigned for
     * LP64. May need to cast to int32_t to avoid implicit type conversions
     */
    TmpSize = pDS->size - (sizeof(DHCPS) - (SZOPTIONS-4));

    if( TmpSize < 0 )
        return;

    /* Reset any requested tag count */
    pDS->tagsize = 0;


    /* Scan through the option tags and perform the following: */

    /* - Get the Message Type */
    /* - Record any requested IP */
    /* - If there's a server ID supplied, be sure it matches */
    /* - Record the list of requested options */

    while( TmpSize )
    {
        switch( *pv++ )
        {
        case DHCPOPT_END:
            TmpSize = 0;
            break;

        case DHCPOPT_HOSTNAME:
            TagLength = *pv;
            if( TagLength > (CFG_HOSTNAME_MAX-1) )
                TagLength = CFG_HOSTNAME_MAX-1;
            strncpy( (char *)Hostname, (char *)(pv+1), TagLength );
            Hostname[TagLength] = 0;
            TagLength = *pv++;
            goto continue_scan;

        case DHCPOPT_DHCP_MESSAGE_TYPE:
            TagLength = *pv++;
            MessageType = *pv;
            goto continue_scan;

        case DHCPOPT_REQUESTED_IP_ADDRESS:
            TagLength = *pv++;
            if( TagLength == 4 )
                memmove( &IPRequested, pv, 4 );
            goto continue_scan;

        case DHCPOPT_PARAMETER_REQUEST_LIST:
            TagLength = *pv++;
            if( TagLength <= TAGBUF_LENGTH )
            {
                memmove( pDS->tags, pv, TagLength );
                pDS->tagsize = TagLength;
            }
            goto continue_scan;

        case DHCPOPT_SERVER_IDENTIFIER:
            TagLength = *pv++;
            if( TagLength == 4 )
            {
                memmove( &tmp, pv, 4 );
                if( tmp == pDS->IPServer )
                    goto continue_scan;
            }
            /* If here, then the tag is illegal or doesn't match */
            return;

        default:
            /* skip over by using the length field */
            TagLength = *pv++;
        continue_scan:
            pv += TagLength;
            TmpSize -= TagLength+2;
            break;
        }
    }

    /* If the client is claiming an IP address, record it. */
    memmove( &IPClaimed, &pHdr->ciaddr, 4 );

    /* Process the message */
    switch( MessageType )
    {
    case DHCPDISCOVER:
        ipr = iprNew(pDS, IPR_MODE_CREATE, IPRequested, (unsigned char *)pHdr->chaddr);
        if( ipr )
            SendDHCPReply( pDS, pDS->ipr.IPAddr, DHCPOFFER, 0 );
        break;

    case DHCPREQUEST:
        /* Get the requested IP from the header or the options, but we */
        /* prefer the header */
        if( !IPClaimed )
            IPClaimed = IPRequested;

        /* If the client claimed an IP address, we must use it */
        /* Else, use any IP address we can get */
        if( IPClaimed )
            ipr = iprNew(pDS, IPR_MODE_CREATEFIXED, IPClaimed,
                                                    (unsigned char *)pHdr->chaddr);
        else
            ipr = iprNew(pDS, IPR_MODE_CREATE, 0, (unsigned char *)pHdr->chaddr);

        /* If we got an address, use it, else send a NAK */
        if( !ipr )
        {
            /* Note: We don't get here if there was a ServerID in the packet */
            /* and it wasn't us. */
            SendDHCPReply( pDS, IPClaimed, DHCPNAK, 0 );
        }
        else
        {
            strcpy( pDS->ipr.Hostname, (char *)Hostname );
            iprUpdate( pDS, CFG_CLIENTSTATUS_VALID );
            SendDHCPReply( pDS, pDS->ipr.IPAddr, DHCPACK, SENDFLG_LEASETIME );
        }
        break;

    case DHCPDECLINE:
        /* Here, just mark the client's entry as invalid */
        ipr = iprFindByMAC( pDS, (unsigned char *)pHdr->chaddr );
        if( ipr )
            iprUpdate( pDS, CFG_CLIENTSTATUS_INVALID );
        break;

    case DHCPRELEASE:
        /* Here, free the client's entry */
        ipr = iprFindByMAC( pDS, (unsigned char *)pHdr->chaddr );
        if( ipr )
            iprRemove( pDS );
        break;

    case DHCPINFORM:
        /* Here we create the entry if we can. If we can, we ACK the packet */
        if( IPClaimed )
        {
            ipr = iprNew(pDS, IPR_MODE_CREATEFIXED, IPClaimed,
                                                    (unsigned char *)pHdr->chaddr);

            /* If we agree on address, send a reply, else ignore it */
            if( ipr )
            {
                iprUpdate( pDS, CFG_CLIENTSTATUS_STATIC );
                SendDHCPReply( pDS, pDS->ipr.IPAddr, DHCPACK, 0 );
            }
        }
        break;

    default:
        break;
    }

    return;
}


/*---------------------------------------------------------------- */
/* SendDHCPReply() */
/* Called to send a reply to the specifed IP address */
/*---------------------------------------------------------------- */
static void SendDHCPReply( DHCPSRV *pDS, uint32_t IPAddr, unsigned char message,
                           uint32_t flags )
{
    DHCPS   *pHdr = (DHCPS *)pDS->buf;
    void *hRt;
    unsigned char *pv;
    uint32_t  Magic = NDK_htonl(RFC1084);
    int     OptSize,MsgSize;
    uint32_t IPReturn;                   /* IP Address for DHCP message */
    int     tagSize,i;
    int     idx,rc,multi;
    unsigned char TagData[64];

    /* Adjust the header */
    pHdr->op   = BOOTREPLY;
    pHdr->secs = 0;
    pHdr->hops = 0;

    /* Fill in yiaddr unless this is a NAK */
    if( message != DHCPNAK )
        memmove( &pHdr->yiaddr, &IPAddr, 4);
    else
        memset( &pHdr->yiaddr, 0, 4);

    /* Clear siaddr */
    memset( &pHdr->siaddr, 0, 4);

    /* Clear SNAME, FILE, and OPTIONS */
    memset(pHdr->sname, 0, SZSNAME);
    memset(pHdr->file, 0, SZFNAME);
    memset(pHdr->options, 0, SZOPTIONS);

    /* Construct new options section */
    memmove( pHdr->options, &Magic, 4);

    pv = (unsigned char *)pHdr->options + 4;
    OptSize = 4;

    /* We always supply a message */
    *pv++ = DHCPOPT_DHCP_MESSAGE_TYPE;
    *pv++ = 1;
    *pv++ = message;
    OptSize += 3;

    /* We always supply a server ID */
    *pv++ = DHCPOPT_SERVER_IDENTIFIER;
    *pv++ = 4;
    memmove( pv, &pDS->IPServer, 4);
    pv+=4;
    OptSize += 6;

    /* Force lease time under some circumstances */
    if( flags & SENDFLG_LEASETIME )
    {
        *pv++ = DHCPOPT_IP_ADDRESS_LEASE_TIME;
        *pv++ = 4;
        Magic = NDK_htonl( pDS->ipr.TimeExpire );
        memmove( pv, &Magic, 4);
        pv+=4;
        OptSize += 6;
    }

    /* Now add the optional tags if not sending a NAK */
    if( message != DHCPNAK )
    {
        for( i=0; i<pDS->tagsize; i++ )
        {
            switch( pDS->tags[i] )
            {
            case DHCPOPT_SUBNET_MASK:
                *pv++ = DHCPOPT_SUBNET_MASK;
                *pv++ = 4;
                memmove( pv, &pDS->IPMask, 4);
                pv+=4;
                tagSize = 6;
                break;

            case DHCPOPT_ROUTER:
                *pv++ = DHCPOPT_ROUTER;
                *pv++ = 4;
                memmove( pv, &pDS->IPServer, 4);
                pv+=4;
                tagSize = 6;
                break;

            case DHCPOPT_DOMAIN_NAME:
                *pv++ = DHCPOPT_DOMAIN_NAME;
                tagSize = pDS->DomainLength;
                *pv++ = (unsigned char)tagSize;
                memmove( pv, &pDS->DomainName, tagSize);
                pv += tagSize;
                tagSize += 2;
                break;

            case DHCPOPT_DOMAIN_NAME_SERVERS:
                /* If we aren't running our own name server, */
                /* then pass-through the external servers we use. */
                if( !pDS->IPNameServer )
                {
                    multi = 1;
                    goto config_tag;
                }
                *pv++ = DHCPOPT_DOMAIN_NAME_SERVERS;
                *pv++ = 4;
                memmove( pv, &pDS->IPServer, 4);
                pv+=4;
                tagSize = 6;
                break;

            /* Generic Tag Handling */

            /* Single Instance Tags */
            case DHCPOPT_NETBIOS_NODE_TYPE:
            case DHCPOPT_NETBIOS_SCOPE:
                multi = 0;
                goto config_tag;

            /* Multi Instance Tags */
            case DHCPOPT_NETBIOS_NAME_SERVER:
                multi = 1;
config_tag:
                idx     = 1;
                tagSize = 0;
                do
                {
                    rc = CfgGetImmediate( 0, CFGTAG_SYSINFO,
                                          pDS->tags[i], idx,
                                          sizeof(TagData), TagData);
                    if( rc <= 0 )
                        break;

                    /* Copy Tag Data */
                    memmove( pv+tagSize+2, TagData, rc );
                    tagSize += rc;

                    /* Do next index (if any) */
                    idx++;

                } while( multi );

                if( tagSize )
                {
                    *pv++ = pDS->tags[i];
                    *pv++ = (unsigned char)tagSize;
                    pv += tagSize;
                    tagSize += 2;
                }
                break;

            case DHCPOPT_IP_ADDRESS_LEASE_TIME:
                /* Don't send it twice */
                if( flags & SENDFLG_LEASETIME )
                    tagSize = 0;
                else
                {
                    *pv++ = DHCPOPT_IP_ADDRESS_LEASE_TIME;
                    *pv++ = 4;
                    Magic = NDK_htonl( pDS->ipr.TimeExpire );
                    memmove( pv, &Magic, 4);
                    tagSize = 6;
                }
                break;

            default:
                tagSize = 0;
                break;
            }

            /* Track the option size - bound by max value */
            if( tagSize )
            {
                OptSize += tagSize;
                if( OptSize > (MAX_OPTION_SIZE-1) )
                {
                    OptSize -= tagSize;
                    pv -= tagSize;
                    break;
                }
            }
        }
    }

    /* Add the final tag */
    *pv++ = DHCPOPT_END;
    OptSize++;

    /* Bound OptSize by its min value */
    if( OptSize < SZOPTIONS )
        OptSize = SZOPTIONS;

    /* Calculate message size */
    /*
     * Note64: sizeof returns size_t, which is 64 bit unsigned for
     * LP64. May need to cast to int32_t to avoid implicit type conversions
     */
    MsgSize = sizeof(DHCPS) - SZOPTIONS + OptSize;

    /* Now send the packet via broadcast or unicast */

    /* Get the return address (if any) */
    memmove( &IPReturn, &pHdr->giaddr, 4);

    /* If no return address in giaddr, try the ciaddr */
    if( !IPReturn )
    {
        /* NAK messages are always broadcast when giaddr is NULL */
        if( message == DHCPNAK )
            goto broadcast_reply;

        /* Get the contents of ciaddr */
        memmove( &IPReturn, &pHdr->ciaddr, 4);
    }

    /* If still no return address, then use yiaddr unless the broadcast */
    /* flag is set */
    if( !IPReturn )
    {
        /* If the broadcast flag is set, then broadcast the reply */
        if( pHdr->flags & 1 )
            goto broadcast_reply;

        /* If we got here, we know IPAddr == yiaddr */
        IPReturn = IPAddr;
    }

    /* Use UNICAST addressing */

    /* Configure sin1 to point back to client */
    pDS->sin1.sin_family      = AF_INET;
    pDS->sin1.sin_addr.s_addr = IPReturn;
    pDS->sin1.sin_port        = NDK_htons(NDHCPC);

    /* If the client is expecting a UNICAST reply, its possible that it */
    /* can not yet respond to an ARP request. For this reason, if we're not */
    /* sending a NAK, but are sending direct to the client, then we'll manually */
    /* validate the route in our ARP table. */
    if( (message != DHCPNAK) && (IPReturn == IPAddr) )
    {
        llEnter();
        hRt = LLIValidateRoute( pDS->hIF, IPReturn, (unsigned char *)pHdr->chaddr );
        /* Note: The function returns a reference handle which we don't want */
        if( hRt )
            RtDeRef( hRt );
        llExit();
    }

    /* Send the packet */
    sendto( pDS->s, pDS->buf, MsgSize, 0,(struct sockaddr *)&pDS->sin1, sizeof(pDS->sin1) );
    return;

broadcast_reply:
    /* Use BROADCAST addressing */

    /* Configure sin1 to point back to client */
    pDS->sin1.sin_family      = AF_INET;
    pDS->sin1.sin_addr.s_addr = INADDR_BROADCAST;
    pDS->sin1.sin_port        = NDK_htons(NDHCPC);

    /* Send the packet */
    sendto( pDS->s, pDS->buf, MsgSize, 0,(struct sockaddr *)&pDS->sin1, sizeof(pDS->sin1) );
}




