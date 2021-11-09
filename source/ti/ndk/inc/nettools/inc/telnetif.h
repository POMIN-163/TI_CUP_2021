/*
 * Copyright (c) 2012-2018, Texas Instruments Incorporated
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
 * ======== telnetif.h ========
 *
 */

/**
 *  @file  ti/ndk/inc/nettools/inc/telnetif.h
 *
 *  @addtogroup ti_ndk_inc_nettools_inc__Telnet Telnet Service
 *
 *  @brief      The Telnet Server service provides a mechanism for
 *              exposing a stream IO connection to any remote telnet
 *              client console.
 */

#ifndef _TELNETIF_H_
#define _TELNETIF_H_

/*! @ingroup ti_ndk_inc_nettools_inc__Telnet */
/*! @{ */

#ifdef __cplusplus
extern "C" {
#endif

/* TELNET SERVICE */

/* Telnet Parameter Structure */
typedef struct _ntparam_telnet {
    int     MaxCon;             /**< Max number of telnet connections */
    int     Port;               /**< Port (set to 0 for telnet default) */
    /*!
     *  Connect function that returns local pipe
     */
    SOCKET  (*Callback)(struct sockaddr *);
} NTPARAM_TELNET;

/*!
 *  @brief      Create an instance of the Telnet Server
 *
 *  @param[in]  pNTA    Pointer to common arg structure used for all services
 *  @param[in]  pNTP    Pointer to Telnet parameter structure
 *
 *  @remark     When a Telnet session is established, a telnet child task
 *              is spawned that will call the supplied callback
 *              function.  This callback function should return a
 *              local file descriptor of one end of a full duplex
 *              pipe. If the callback function returns -1, the
 *              connection is aborted.
 *
 *  @remark     When either the terminal or telnet connection end of the
 *              pipe is broken, the other connection is closed and
 *              the session is ended.
 *
 *  @return     Success: handle to new Telnet Server instance
 *  @return     Failure: NULL
 *
 *  @sa TelnetClose()
 */
extern void *TelnetOpen( NTARGS *pNTA, NTPARAM_TELNET *pNTP );

/*!
 *  @brief      Destroy an instance of the Telnet Server
 *
 *  @param[in]  hTelnet Handle to Telnet Server instance obtained from
 *                      TelnetOpen()
 *
 *  @remark     When called the server is shut down and no further
 *              Telnet sessions can be established. Also, all spawned
 *              connections are immediately terminated.
 *
 *  @sa TelnetOpen()
 */
extern void TelnetClose( void *hTelnet );

/** @cond INTERNAL */
/*
 * This is typically an internal fxn, so is not documented.  However,
 * IPv6 users do need access to this symbol for reasons described in
 * the API users guide.
 *
 * TODO: this needs to be simpler for IPv6 users.
 */

extern int   telnetClientProcess( SOCKET s, SOCKET (*cbfn)(struct sockaddr *) );

/** @endcond INTERNAL */

/*! @} */
#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
