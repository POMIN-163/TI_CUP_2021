/*
 * Copyright (c) 2012-2019, Texas Instruments Incorporated
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
 * ======== pppoe.h ========
 *
 * Basic PPPoE definitions
 *
 */

/*----------------------------------------------- */
/* PPPOE Protocol Header */

#ifndef _PPPOE_H
#define _PPPOE_H

#ifdef __cplusplus
extern "C" {
#endif

#define PPPOEHDR_SIZE   6

typedef struct {
                unsigned char   VerType;
                unsigned char   Code;
#define PPPOE_CODE_INITIATION   0x9
#define PPPOE_CODE_OFFER        0x7
#define PPPOE_CODE_REQUEST      0x19
#define PPPOE_CODE_CONFIRM      0x65
#define PPPOE_CODE_TERMINATE    0xa7
                uint16_t  SessionId;
                uint16_t  Length;
                unsigned char Data[];
#define PPPOE_TAG_EOL           0
#define PPPOE_TAG_SNAME         0x0101
#define PPPOE_TAG_ACNAME        0x0102
#define PPPOE_TAG_HOSTUNIQUE    0x0103
#define PPPOE_TAG_ACCOOKIE      0x0104
#define PPPOE_TAG_SNAMEERROR    0x0201
#define PPPOE_TAG_ACNAMEERROR   0x0202
#define PPPOE_TAG_ERROR         0x0203
               } PPPOEHDR;

/*----------------------------------------------- */
/* Shared Functions */
extern void pppoeSI( void *hSI, uint32_t Msg, uint32_t Aux, PBM_Pkt *pPkt );

/*----------------------------------------------- */
/* PPPOE Object Structures */
/*----------------------------------------------- */

/*----------------------------------------------- */
/* Max count for service list and max name length */
#define PPPOE_NAMESIZE          32

/* Generic Instance */
typedef struct _pppoe_instance {
    void      *hParent;                       /* Parent structure */
    uint32_t     Status;                        /* Call status */
    uint32_t     iType;                         /* Instance TYPE */
#define PPPOE_INST_SERVER       1
#define PPPOE_INST_CLIENT       2
    void      *hEther;                        /* Host Ethernet */
    uint16_t     SessionId;                     /* Session Index */
    void      *hPPP;                          /* PPP Interface */
    unsigned char      MacAddr[6];                    /* Our MAC address */
    unsigned char      PeerMac[6];                    /* Client's MAC address */
    } PPPOE_INST;

/* Server */
typedef struct _pppoe_server {
    uint32_t     Type;                          /* Set to HTYPE_PPPOE_SERVER */
    void      *hTimer;                        /* Session Timer */
    void      *hEther;                        /* Ether Interface */
    uint32_t     pppFlags;                      /* PPP Flags */
    uint32_t     SessionMax;                    /* Max active sessions */
    uint32_t     IPClientBase;                  /* New client base IP address */
    uint32_t     IPServer;                      /* Our IP address */
    uint32_t     IPMask;                        /* Our IP mask */
    unsigned char      MacAddr[6];                    /* Our MAC address */
    char       ServerName[PPPOE_NAMESIZE];    /* Our Server's Name */
    char       ServiceName[PPPOE_NAMESIZE];   /* Name of service provided */
    PPPOE_INST   ppi[1];                        /* Array of instances */
    } PPPOE_SERVER;

/* Client */
typedef struct _pppoe_client {
    uint32_t     Type;                          /* Set to HTYPE_PPPOE_SERVER */
    void      *hTimer;                        /* Session Timer */
    uint32_t     pppFlags;                      /* PPP Flags */
    char       Username[PPPOE_NAMESIZE];      /* Specified Username */
    char       Password[PPPOE_NAMESIZE];      /* Specifies Password */
    PPPOE_INST   ppi;                           /* Our instance */
    int          State;                         /* Current state */
    int          Timeout;                       /* Timeout for this state */
    int          Retry;                         /* Retry count for timeout */
    char       ServerName[PPPOE_NAMESIZE];    /* Obtained from server */
    char       ServiceName[PPPOE_NAMESIZE];   /* Obtained from server */
    uint32_t     CookieSize;                    /* Obtained from server */
    unsigned char      Cookie[PPPOE_NAMESIZE];        /* Obtained from server */
    } PPPOE_CLIENT;

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* _PPPOE_H */
