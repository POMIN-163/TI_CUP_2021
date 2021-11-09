/*
 * Copyright (c) 2012-2020, Texas Instruments Incorporated
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
 * ======== netctrl.h ========
 *
 * Shell functions for simplified net startup and shutdown
 *
 * The idea behind this API is to hide the user callable HAL/STACK functions
 *
 */
/**
 *  @file  ti/ndk/inc/netctrl/netctrl.h
 *
 *  @addtogroup ti_ndk_inc_netctrl_NC Network Control Initialization Procedure
 *
 *  @brief      The stack library includes code to perform system
 *              initialization based on the configuration. Initialization
 *              of the scheduling routines is performed by a network
 *              control layer called NETCTRL.
 */

#ifndef _C_NETCTRL_INC
#define _C_NETCTRL_INC

/*! @ingroup ti_ndk_inc_netctrl_NC */
/*@{*/

#ifdef __cplusplus
extern "C" {
#endif

/* NETCTRL is used to initialize the stack and maintain services. To */
/* accomplish this, it makes use of the configuration manager provided */
/* in the NETTOOLS library. Note that the configuration definitions and */
/* structures defined here are specific to NETCTRL, not CONFIG. */

/* NETCTRL Access Functions */

/*!
 *  @brief      Initiate a system session
 *
 *  This is the first function that should be called when using the
 *  stack. It initializes the stack's memory manager, and the OS
 *  (or OS adaptation layer). It also configures the network event
 *  scheduler's task priority and operating mode.
 *
 *  @param[in]  Priority    Network event scheduler task priority
 *  @param[in]  OpMode      Network event scheduler operating mode
 *
 *  @remark     @c Priority must be set to either #NC_PRIORITY_LOW or
 *              #NC_PRIORITY_HIGH, and determines the scheduler task's
 *              priority relative to other networking tasks in the
 *              system.
 *
 *  @remark     @c OpMode must be set to either #NC_OPMODE_POLLING or
 *              #NC_OPMODE_INTERRUPT, and determines when the scheduler
 *              attempts to execute. The interrupt mode is used in the
 *              vast majority of applications.
 *
 *  @remark     Note that polling operating mode attempts to run
 *              continuously, so when polling is used, @c Priority
 *              must be set to #NC_PRIORITY_LOW.
 *
 *  @return     Success: 0
 *  @return     Failure:
 *                * #NC_OPEN_ILLEGAL_PRIORITY
 *                * #NC_OPEN_ILLEGAL_OPMODE
 *                * #NC_OPEN_MEMINIT_FAILED
 *                * #NC_OPEN_EVENTINIT_FAILED
 *
 *  @sa NC_SystemClose()
 */
extern int  NC_SystemOpen( int Priority, int OpMode );

#define NC_PRIORITY_LOW             OS_SCHEDULER_LOWPRI
#define NC_PRIORITY_HIGH            OS_SCHEDULER_HIGHPRI

#define NC_OPMODE_POLLING           1
#define NC_OPMODE_INTERRUPT         2

#define NC_OPEN_SUCCESS             0   /**< Success */
#define NC_OPEN_ILLEGAL_PRIORITY    -1  /**< Illegal priority */
#define NC_OPEN_ILLEGAL_OPMODE      -2  /**< Illegal operating mode */
#define NC_OPEN_MEMINIT_FAILED      -3  /**< Memory initialization failure */
#define NC_OPEN_EVENTINIT_FAILED    -4  /**< Event initialization failure */


/*!
 *  @brief      Shutdown the system
 *
 *  This is the last function that should be called when using the
 *  stack. It shuts down the memory manager and performs a final
 *  memory analysis.
 *
 *  @sa NC_SystemOpen()
 */
extern void NC_SystemClose();

/*!
 *  @brief      Start network
 *
 *  This function is called to boot up the network using the network
 *  configuration supplied in @c hCfg. Along with the network
 *  configuration, three callback function pointers are
 *  provided. These callback functions are called at distinct
 *  times.
 *    * @c NetStartCb is called when the system is first ready for
 *      the creation of application supplied network tasks.
 *    * @c NetStopCb is called when the network is about to shut down.
 *    * @c NetIPCb is called when an IP address is added or removed
 *      from the system.
 *
 *  If any of these callback functions are not required, the function
 *  pointers can be set to NULL.
 *
 *  @param[in]  hCfg        Handle to network configuration
 *  @param[in]  NetStartCb  Optional pointer to callback function called when
 *                          network stack is started
 *  @param[in]  NetStopCb   Optional pointer to callback function called when
 *                          network is stopped
 *  @param[in]  NetIPCb     Optional pointer to callback function called when
 *                          an IP address is added or removed from the
 *                          system
 *
 *  @remark     @c NC_NetStart() will not return until the entire network
 *              session has completed. Thus, all user supplied network
 *              code (creation of user tasks) should be included in
 *              the @c NetStartCb function.
 *
 *  @remark     When @c NetStartCb is called, the configuration handle
 *              supplied in @c hCfg is the default configuration
 *              handle for the system. The execution thread on which
 *              @c NetStartCb is called is not critical to event
 *              scheduling, but it should return eventually; i.e., the
 *              application should not take control of the thread. If
 *              system shutdown is initiated before this callback
 *              function returns, some resources may not be freed.
 *
 *  @remark     Excluding critical errors, NC_NetStart() will return only
 *              if an application calls the NC_NetStop() function. The
 *              parameter passed to NC_NetStop() becomes the return
 *              value returned by NC_NetStart().
 *
 *  @remark     Sometime after NC_NetStop() is called, but before
 *              NC_NetStart() returns, the NC_NetStart() thread will
 *              make a call to the application's NetStopCb callback
 *              function. In this callback function, the application
 *              should shut down any task initiated in its
 *              @c NetStartCb callback.
 *
 *  @remark     When an IP addressing change is made to the system, the
 *              @c NetIPCb function is called. The callback function is
 *              declared as:
 *  @code
 *      void NetIPCb(uint32_t IPAddr, uint32_t IfIndex, uint32_t fAdd);
 *  @endcode
 *
 *  @remark     where:
 *                * IPAddr: IP Address being added or removed
 *                * IfIndex: Index of physical interface gaining or
 *                      losing the IP address
 *                * fAdd: Set to 1 when adding an address, or 0 when
 *                      removing an address
 *
 *  @remark     The @c NetIPCb callback is purely informational, and no
 *              processing is necessary on the information provided.
 *
 *  @remark     There is an option for immediately calling NC_NetStart()
 *              again upon return, which provides a good stack reboot
 *              function. Optionally, the configuration can also be
 *              reloaded, which allows the stack to be restarted after
 *              a major configuration change.
 *
 *  @return     The integer passed to NC_NetStop()
 *
 *  @sa NC_NetStop()
 */
extern int NC_NetStart(void *hCfg, void (*NetStartCb)(),
        void (*NetStopCb)(), void (*NetIPCb)(uint32_t, uint32_t, uint32_t));

/*!
 *  @brief      Stop the network
 *
 *  This function is called to shut down a network initiated with
 *  NC_NetStart(). The return value supplied in the StopCode parameter
 *  becomes the return value for NC_NetStart(). See the description of
 *  NC_NetStart() for more detail.
 *
 *  @param[in]  rc      return code to be returned by NC_NetStart()
 *
 *  @sa NC_NetStart()
 */
extern void NC_NetStop( int rc );

/** @cond INTERNAL */

/* Called when Boot thread has completed */
extern void NC_BootComplete();

/* Called when IP address is added or removed */
extern void NC_IPUpdate( uint32_t IPAddr, uint32_t IfIdx, uint32_t fAdd );

/* Called to set the user function that's called when link goes up/down */
extern void NC_setLinkHook( void (*LinkHook)(int) );

/** @endcond */

/*! @} */
#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
