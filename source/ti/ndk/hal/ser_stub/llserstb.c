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
 * ======== llserstb.c ========
 *
 * Serial port stub driver
 *
 */

#include <stkmain.h>

/*-------------------------------------------------------------------- */
/* PUBLIC FUNCTIONS USED BY NETCTRL */
/*-------------------------------------------------------------------- */

/*-------------------------------------------------------------------- */
/* _llSerialInit() */
/* Opens the serial driver environment and enumerates devices */
/*-------------------------------------------------------------------- */
uint32_t _llSerialInit(STKEVENT_Handle hEvent)
{
    (void)hEvent;
    return( 0 );
}

/*-------------------------------------------------------------------- */
/* _llSerialShutdown() */
/* Called to shutdown serial driver environment */
/*-------------------------------------------------------------------- */
void _llSerialShutdown()
{
}

/*-------------------------------------------------------------------- */
/* _llSerialServiceCheck() */
/* Called to check for HDLC/Charmode activity */
/*-------------------------------------------------------------------- */
void _llSerialServiceCheck( uint32_t fTimerTick )
{
    (void)fTimerTick;
}

/*-------------------------------------------------------------------- */
/* _llSerialSend( uint32_t dev, unsigned char *pBuf, uint32_t len ) */
/* Send a block of data in character mode */
/* Returns the number of characters queued onto the send buffer */
/*-------------------------------------------------------------------- */
uint32_t  _llSerialSend(uint32_t dev, unsigned char *pBuf, uint32_t len)
{
    (void)dev;
    (void)pBuf;
    (void)len;
    return(0);
}

/*-------------------------------------------------------------------- */
/* PUBLIC FUNCTIONS USED BY THE STACK */
/*-------------------------------------------------------------------- */

/*-------------------------------------------------------------------- */
/* llSerialOpen(uint32_t dev, void (*cbInput)(char c)) */
/* Opens the device for charater mode. The device can be */
/* be open for both character and HDLC mode simultaneously */
/*-------------------------------------------------------------------- */
uint32_t llSerialOpen(uint32_t dev, void (*cbInput)(char c))
{
    (void)dev;
    (void)cbInput;
    return(0);
}

/*-------------------------------------------------------------------- */
/* llSerialClose( uint32_t dev ) */
/* Close the device */
/*-------------------------------------------------------------------- */
void llSerialClose( uint32_t dev )
{
    (void)dev;
}

/*-------------------------------------------------------------------- */
/* llSerialOpenHDLC() */
/* Opens the device for HDLC mode. The device can be */
/* be open for both character and HDLC mode simultaneously */
/*-------------------------------------------------------------------- */
uint32_t  llSerialOpenHDLC( uint32_t dev, void *hHDLC,
                        void (*cbTimer)(void *h),
                        void (*cbInput)(PBM_Handle hPkt) )
{
    (void)dev;
    (void)hHDLC;
    (void)cbTimer;
    (void)cbInput;
    return(0);
}

/*-------------------------------------------------------------------- */
/* llSerialCloseHDLC() */
/* Close the device for charater mode. */
/*-------------------------------------------------------------------- */
void llSerialCloseHDLC( uint32_t dev )
{
    (void)dev;
}

/*-------------------------------------------------------------------- */
/* llSerialSendPkt() */
/* Called to send data in packet form. */
/*-------------------------------------------------------------------- */
void  llSerialSendPkt( uint32_t dev, PBM_Handle hPkt )
{
    (void)dev;
    (void)hPkt;
}

/*-------------------------------------------------------------------- */
/* llSerialHDLCPeerMap( uint32_t dev, uint32_t peerMap ) */
/* Called to update the sending peer map. */
/*-------------------------------------------------------------------- */
void  llSerialHDLCPeerMap( uint32_t dev, uint32_t peerMap )
{
    (void)dev;
    (void)peerMap;
}

/*-------------------------------------------------------------------- */
/* void llSerialService() */
/* Called to service serial drivers and pass up packets to the */
/* HDLC module. Feed character mode data to application callback */
/*-------------------------------------------------------------------- */
void llSerialService()
{
}

/*-------------------------------------------------------------------- */
/* llSerialConfig() */
/* Called to configure the serial port baud rate and mode */
/*-------------------------------------------------------------------- */
void llSerialConfig( uint32_t dev, uint32_t baud, uint32_t mode,
        uint32_t flowctrl )
{
    (void)dev;
    (void)baud;
    (void)mode;
    (void)flowctrl;
}


