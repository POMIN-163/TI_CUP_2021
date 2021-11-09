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
 * ======== bindif.h ========
 *
 * IPv4 Binding table access functions and definitions
 *
 */


#ifndef _C_BINDIF_INC
#define _C_BINDIF_INC  /* #defined if this .h file has been included */

#ifdef __cplusplus
extern "C" {
#endif

/* Bind Create Functions */
extern void *BindNew( void *hIF, uint32_t IPHost, uint32_t IPMask );
extern void    BindFree( void *h );

/* Bind Access Functions */
extern void *BindGetFirst();
extern void *BindGetNext( void *);
extern void *BindGetIF( void *);
extern void *BindGetIFByDBCast( uint32_t IPHost );
extern void    BindGetIP(void *,uint32_t *pIPHost,uint32_t *pIPNet,uint32_t *pIPMask);

/* Bind Search Functions */
extern void *BindFindByIF( void *hIF );
extern void *BindFindByNet( void *hIF, uint32_t IP );
extern void *BindFindByHost( void *hIF, uint32_t IP );

/* IF to IP Matching Functions */
extern uint32_t BindIFNet2IPHost( void *hIF, uint32_t IP );
extern uint32_t BindIF2IPHost( void *hIF );
extern void *BindIPHost2IF( uint32_t IP );
extern void *BindIPNet2IF( uint32_t IP );

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
