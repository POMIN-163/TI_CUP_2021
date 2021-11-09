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
 * ======== igmpif.h ========
 *
 */


#ifndef _C_IGMPIF_INC
#define _C_IGMPIF_INC  /* #defined if this .h file has been included */

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------------------------------------------- */
/* Global Task Information */

/* Defined Messages */
#define MSG_IGMP_TIMER                    (ID_IGMP*MSG_BLOCK + 0)
#define MSG_IGMP_NEEDTIMER                (ID_IGMP*MSG_BLOCK + 1)

/* Global Statistics */
extern uint32_t _IGMPInDiscard;
extern uint32_t _IGMPInQuery;
extern uint32_t _IGMPInResponse;
extern uint32_t _IGMPOutResponse;

/* Access Functions (Kernel) */
extern void IGMPMsg( uint32_t Msg );
extern void IGMPInput( PBM_Pkt *pPkt );
extern uint32_t IGMPTestGroup( uint32_t IpAddr, uint32_t IfIdx );
extern int IGMPJoin (void *hSock, struct ip_mreq* ptr_ipmreq);
extern int IGMPLeave (void *hSock, struct ip_mreq* ptr_ipmreq);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif

