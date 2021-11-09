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
 * ======== rawethif.h ========
 *
 * The file contains defintions and structures which describe the Raw
 * Ethernet Channel Management Object.
 *
 */

#ifndef _C_RAWETHIF_INC
#define _C_RAWETHIF_INC

#ifdef __cplusplus
extern "C" {
#endif

/*  Well Known Ethernet Types. These cannot be overriden by
 *  the application.
 */
#define     ETHERTYPE_IP            0x800
#define     ETHERTYPE_IPv6          0x86DD
#define     ETHERTYPE_VLAN          0x8100
#define     ETHERTYPE_PPPOECTL      0x8863
#define     ETHERTYPE_PPPOEDATA     0x8864

/**
 * @brief
 *  The structure describes the RAW Ethernet Statistics block.
 *
 * @details
 *  This structure is used to hold various stats and
 *  counters by the Raw ethernet channel management module.
 */
typedef struct _rawethstat {
    /**
     * @brief   Total Raw Eth packets received by the Raw Eth
     *          Channel Mgmt Module.
     */
    uint32_t    RcvTotal;

    /**
     * @brief   Number of Raw Eth packets dropped by this module because
     *          of errors.
     */
    uint32_t    RcvDrops;

    /**
     * @brief   Total packets sent successfully by Raw Eth Module.
     */
    uint32_t    SndTotal;

    /**
     * @brief   Number of Sends failed due to memory allocation errors
     */
    uint32_t    SndNoPacket;

} RAWETHSTATS;

/* Raw ethernet stats instance */
extern RAWETHSTATS NDK_raweths;

/**********************************************************************
 * Exported API (KERNEL MODE):
 *  These functions are exported by the Raw Ethernet Module and
 *  are available only for NDK stack internal usage.
 ***********************************************************************/
extern int RawEthTxPacket (void *hRawEthSock, char *pBuffer, int len);
extern int RawEthTxPacketNC (void *hRawEthSock, char *pBuffer, int len, void *hPkt);
extern int RawEthRxPacket (PBM_Handle hPkt);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* _C_RAWETHIF_INC */
