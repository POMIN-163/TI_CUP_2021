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
 * ======== pppoeif.h ========
 *
 */


#ifndef _C_PPPOEIF_INC
#define _C_PPPOEIF_INC  /* #defined if this .h file has been included */

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------------------------------------------- */
/* Global Task Information */

#define ETHERTYPE_PPPOE_CTRL    0x8863
#define ETHERTYPE_PPPOE_DATA    0x8864

#ifdef _INCLUDE_PPPOE_CODE

/* PPPOE Client Functions */
extern void *pppoeNew( void *hEth, uint32_t pppFlags,
                         char *Username, char *Password );
extern void   pppoeFree( void *hPPPOE );
extern uint32_t   pppoeGetStatus( void *hPPPOE );
extern void   pppoeInput( PBM_Pkt *pPkt );

/* PPPOE Server Functions */
extern void *pppoesNew( void *hEth, uint32_t pppFlags, uint32_t SessionMax,
                          uint32_t IPServer, uint32_t IPMask, uint32_t IPClientBase,
                          char *ServerName, char *ServiceName );
extern void   pppoesFree( void *hPPPOES );

#endif

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif


