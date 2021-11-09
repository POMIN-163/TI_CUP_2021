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
 * ======== dhcp.h ========
 *
 * Simple DHCP Client Utility
 *
 */

#ifndef _DHCP_H
#define _DHCP_H

#include <netmain.h>
#include <_stack.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NDHCPS              67
#define NDHCPC              68

#define SZCHADDR            16          /* size of client haddr field */
#define SZSNAME             64          /* size of server name field */
#define SZFNAME             128         /* size of file name field */

/*
 *  Size of DHCP options field.
 */
#define SZOPTIONS           312

/* values for op */
#define BOOTREQUEST         1
#define BOOTREPLY           2

/* Debug Flag(s) */
#define DEBUGON             0

/* values for htype and hlen */
#define ETHERNET            1
#define ELEN                6

/*
 *  Maximum size for DHCP client host name.
 *
 *  *** NOTE: changing this value requires a rebuild of NETTOOLS! ***
 *
 *  NOTE: The client hostname size is actually governed by 2 definitions:
 *      1) CFG_HOSTNAME_MAX (defined in netcfg.h)
 *      2) HOSTNAME_LENGTH (defined in dhcp.h)
 *  These two values must be consistent!  If one is changed, both must be
 *  changed.
 *
 *  RFC 2131 and 2132 do not clearly specify what the maximum size for this
 *  value should be.  255 was determined based on MSDN "Host Name Resolution for
 *  IPv4" on Microsoft's website (the size is [255 + 1] to make room for "\0"). 
 *  See notes for fix of CQ15114 for more details.
 *  
 */
#define HOSTNAME_LENGTH     256

#define BUFFER_SIZE         1024
#define MAX_DISCOVER_TRIES  12          /*  Maximum attempts in INIT State */

/* structure of a DHCP message */
typedef struct _dhcp
{
    char op;                  /* request or reply */
    char htype;               /* hardware type */
    char hlen;                /* hardware address length */
    char hops;                /* set to zero */
    uint32_t  xid;                 /* transaction id */
    uint16_t secs;                /* time client has been trying */
    uint16_t flags;               /* Flags */
    int32_t  ciaddr;              /* client IP address */
    int32_t  yiaddr;              /* your (client) IP address */
    int32_t  siaddr;              /* server IP address */
    int32_t  giaddr;              /* gateway IP address */
    char chaddr[SZCHADDR];    /* client hardware address */
    char sname[SZSNAME];      /* Optional server host name, Null terminated */
    char file[SZFNAME];       /* boot file name */
    char options[SZOPTIONS];  /* Optional Parameter Field */
} DHCP;

/* DHCP Message Types */
#define     DHCPDISCOVER        1
#define     DHCPOFFER           2
#define     DHCPREQUEST         3
#define     DHCPDECLINE         4
#define     DHCPACK             5
#define     DHCPNAK             6
#define     DHCPRELEASE         7
#define     DHCPINFORM          8

/* structure of a DHCP lease information */
typedef struct _dhcpLEASE
{
    uint16_t StateInitial;        /* Starting State */
    uint16_t State;               /* Current State */
    uint16_t StateNext;           /* Next State */
    uint32_t IPAddress;           /* IP Address from DHCP Server */
    uint32_t IPAddressOld;        /* IP Address being used */
    uint32_t IPSubnetMask;        /* IP SubnetMask */
    uint32_t IPGate;              /* IP Gateway Address */
    uint32_t LeaseExpires;        /* Time to moved to Init State */
    uint32_t LeaseTime;           /* Lease Time from Server */
    uint32_t RenewalT1Time;       /* Renewal T1 Time Fron Server */
    uint32_t RenewalT2Time;       /* Renewal T2 Time from Server */
    uint32_t IPServer;            /* Server Address */
    uint32_t LeaseStartTime;      /* Time REQUEST was sent */
    uint32_t T1;                  /* Time to moved to Renewing State */
    uint32_t T2;                  /* Time to moved to Rebinding State */
    uint32_t T3;                  /* Time to moved to Init State (Lease Expired) */
    uint32_t Xid;                 /* transaction Id */
    int16_t  MaxDiscoverTries;
    char   MacAddress[ELEN];    /* The MAC address of the target interface */
    void *hCb;                 /* Handle for callback function */
    void   (*pCb)(void *,uint32_t); /* Callback function for results */
    uint32_t IfIdx;             /* Index to pysical DHCP interface */
    void *dhcpTASK;            /* HANDLE to the task for this instance */
    void *hCE_IPAddr;          /* HANDLE to CfgEntry of IP Network */
    void *hCE_IPGate;          /* HANDLE to CfgEntry of installed IP Gateway */
    int    ReceivedSize;
    int    SendSize;
    SOCKET Sock;
    struct sockaddr_in sin1;
    char DomainName[SZFNAME]; /* domain name */
    char   HostName[HOSTNAME_LENGTH];
    unsigned char  ClientID[HOSTNAME_LENGTH];	/* Client Identifier to be used. */
    char   Buffer[BUFFER_SIZE]; /* Transmit/Receive  buffer */
    unsigned char  *pOptions;           /* options to request */
    int    options_len;         /* length of options */
} DHCPLEASE;


#define RFC1084 0x63825363      /* vendor magic cookie from 1084 */

/* States */
enum DhcpState
{
    STATEZERO,
    BOUND,
    INIT,
    REBINDING,
    REBOOTING,
    RENEWING,
    REQUESTING,
    SELECTING,
    DHCPEND
};

/* Errors */
enum DhcpErrors
{
    DHCP_NOERROR=0,
    DHCP_ERR_ISSET,
    DHCP_ERR_NAK,
    DHCP_ERR_OFFER,
    DHCP_ERR_RECV,
    DHCP_ERR_REQUEST,
    DHCP_ERR_SELECT,
    DHCP_ERR_SEND,
    DHCP_ERR_END
};


/* Global Functions used only by DHCP */
extern int    dhcpSocketOpen(DHCPLEASE *pLease);
extern void   dhcpSocketClose(DHCPLEASE *pLease);
extern int    dhcpPacketSend( DHCPLEASE *pLease, uint32_t IPServer );
extern int    dhcpPacketReceive(DHCPLEASE *pLease);
extern void   dhcpState(DHCPLEASE *pLease);
extern int    dhcpBuildRequest(DHCPLEASE *pLease);
extern int    dhcpBuildDecline(DHCPLEASE *pLease);
extern int    dhcpBuildDiscover(DHCPLEASE *pLease);
extern uint16_t dhcpVerifyMessage(DHCPLEASE *pLease,uint32_t *pIPAddr,uint32_t *pIPServer);
extern void   dhcpPacketProcess(DHCPLEASE *pLease);
extern void   dhcpBuildOptions(unsigned char **pBuf, DHCPLEASE *pLease);
extern void   dhcpDecodeType( unsigned char tag, int length, unsigned char *data );
extern void   dhcpOptionsClear( void );

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /*  _DHCP_H */
