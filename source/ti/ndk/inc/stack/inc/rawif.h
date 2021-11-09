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
 * ======== rawif.h ========
 *
 * Raw socket interface layer definitions
 *
 */


#ifndef _RAWIF_INC_
#define _RAWIF_INC_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *  The structure describes the RAW Statistics block.
 *
 * @details
 *  This structure is used to hold various stats and
 *  counters by the RAW module. A separate copy of this
 *  stats is maintained by the IPv4 and IPv6 stacks.
 */
typedef struct _rawstat {
    /**
     * @brief   Total packets received.
     */
    uint32_t  RcvTotal;

    /**
     * @brief   Unable to take more data.
     */
    uint32_t  RcvFull;

    /**
     * @brief   Total packets sent.
     */
    uint32_t  SndTotal;

    /**
     * @brief   Unable to allocate packet.
     */
    uint32_t  SndNoPacket;

} RAWSTATS;

extern RAWSTATS NDK_raws;

#ifdef _INCLUDE_IPv6_CODE
extern RAWSTATS NDK_raw6_stats;
#endif

/* RAW Functions */
extern void  RawInput( PBM_Pkt *pPkt );
extern int   RawOutput( void *h, unsigned char *pBuf, int32_t sz, int32_t *prsz );

#ifdef _INCLUDE_IPv6_CODE
extern void Raw6Input (PBM_Pkt* pPkt, IPV6HDR* ptr_ipv6hdr, unsigned char Protocol);
extern int  Raw6Output( void *hSock, unsigned char *buf, int32_t size, int32_t *pRetSize);
#endif

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif


