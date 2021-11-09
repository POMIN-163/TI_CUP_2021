/*
 * Copyright (c) 2012-2016, Texas Instruments Incorporated
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
 * ======== dhcps.h ========
 *
 * DHCP Server
 *
 */

#ifndef _DHCPS_H
#define _DHCPS_H

#include <netmain.h>
#include <_stack.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NDHCPS          67
#define NDHCPC          68

#define SZCHADDR        16              /* size of client haddr field */
#define SZSNAME         64              /* size of server name field */
#define SZFNAME         128             /* size of file name field */
#define SZOPTIONS       312             /* size of options field */

/* values for op */
#define BOOTREQUEST     1
#define BOOTREPLY       2

/* values for htype and hlen */
#define ETHERNET                1
#define ELEN                    6
#define BUFFER_SIZE             1024

/* structure of a DHCPS message */
typedef struct _dhcps
{
    char op;                  /* request or reply */
    char htype;               /* hardware type */
    char hlen;                /* hardware address length */
    char hops;                /* set to zero */
    uint32_t xid;                 /* transaction id */
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
} DHCPS;

#define RFC1084         0x63825363      /* vendor magic cookie from 1084 */

/* DHCP Message Types */
#define     DHCPDISCOVER        1
#define     DHCPOFFER           2
#define     DHCPREQUEST         3
#define     DHCPDECLINE         4
#define     DHCPACK             5
#define     DHCPNAK             6
#define     DHCPRELEASE         7
#define     DHCPINFORM          8

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /*  _DHCPS_H */

