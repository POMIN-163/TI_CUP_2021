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
 * ======== hal.h ========
 *
 * Include file for generic support layer
 *
 */

#ifndef _C_HAL_INC
#define _C_HAL_INC  /* #defined if this .h file has been included */

#ifdef __cplusplus
extern "C" {
#endif

/* Note: All functions define here with a leading underscore '_' */
/*       must only be called from outside of kernel mode (llEnter/llExit) */

/*----------------------------------------------------------------------- */
/*---[ LLPACKET ]-------------------------------------------------------- */
/*----------------------------------------------------------------------- */

/* Packet Driver Interface to the Operating System */
extern uint32_t _llPacketInit(STKEVENT_Handle h);     /* Init and enumerate */
extern void _llPacketShutdown();                        /* System shutdown */
extern void _llPacketServiceCheck(uint32_t fTimerTick); /* Polling Function */

/* Packet Driver Interface to the Stack */
extern uint32_t llPacketOpen( uint32_t dev, void *hEther );
extern void     llPacketClose( uint32_t dev );
extern void     llPacketSend( uint32_t dev, PBM_Handle hPkt );
extern void     llPacketSetRxFilter( uint32_t dev, uint32_t filter );
extern void     llPacketGetMacAddr( uint32_t dev, unsigned char *pbData );
extern uint32_t llPacketGetMCastMax( uint32_t dev );
extern void     llPacketSetMCast(uint32_t dev, uint32_t addrcnt, unsigned char *bAddr);
extern uint32_t llPacketGetMCast(uint32_t dev, uint32_t maxaddr, unsigned char *bAddr);
extern uint32_t llPacketIoctl( uint32_t dev, uint32_t cmd, void *arg);
extern void     llPacketService();


/*----------------------------------------------------------------------- */
/*---[ LLSERIAL ]-------------------------------------------------------- */
/*----------------------------------------------------------------------- */

/* Serial Driver Interface called from User Mode */
extern uint32_t _llSerialInit(STKEVENT_Handle h); /* Init and enumerate */
extern void     _llSerialShutdown();                  /* System shutdown */
extern void     _llSerialServiceCheck(uint32_t fTimerTick); /* Polling Fxn */
extern uint32_t _llSerialSend(uint32_t dev, unsigned char *pBuf, uint32_t len);

/* Serial Driver Interface called from Kernel Mode */
extern uint32_t llSerialOpen(uint32_t dev, void (*cbInput)(char c));
extern void     llSerialClose( uint32_t dev );

extern uint32_t llSerialOpenHDLC( uint32_t dev, void *hHDLC,
                                void (*cbTimer)(void *h),
                                void (*cbInput)(PBM_Handle hPkt) );
extern void     llSerialCloseHDLC( uint32_t dev );
extern void     llSerialSendPkt( uint32_t dev, PBM_Handle hPkt );
extern void     llSerialHDLCPeerMap( uint32_t dev, uint32_t peerMap );

extern void     llSerialService();
extern void     llSerialConfig( uint32_t dev, uint32_t baud, uint32_t mode,
                           uint32_t flowctrl );

/* Mode values for llSerialConfig() */
#define HAL_SERIAL_MODE_8N1             0
#define HAL_SERIAL_MODE_7E1             1
#define HAL_SERIAL_FLOWCTRL_NONE        0
#define HAL_SERIAL_FLOWCTRL_HARDWARE    1


/*----------------------------------------------------------------------- */
/*---[ LLTIMER ]--------------------------------------------------------- */
/*----------------------------------------------------------------------- */

/* Timer Driver Interface to the Operating System */
extern void   _llTimerInit( STKEVENT_Handle h, uint32_t ctime );
extern void   _llTimerShutdown();

/* Timer Driver Interface to the Stack */
extern uint32_t llTimerGetTime( uint32_t *pMSFrac );
extern uint32_t llTimerGetStartTime();



/*----------------------------------------------------------------------- */
/*---[ LLUSERLED ]----------------------------------------------------------- */
/*----------------------------------------------------------------------- */

/* User LED Driver Interface to the Operating System */
extern void   _llUserLedInit();
extern void   _llUserLedShutdown();
extern void   LED_ON(uint32_t ledId);
extern void   LED_OFF(uint32_t ledId);
extern void   LED_TOGGLE(uint32_t ledId);

#define USER_LED1   1
#define USER_LED2   2
#define USER_LED3   4
#define USER_LED4   8
#define USER_LED5   16
#define USER_LED6   32
#define USER_LED7   64
#define USER_LED8   128

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
