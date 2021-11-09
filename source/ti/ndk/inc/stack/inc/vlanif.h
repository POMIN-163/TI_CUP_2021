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
 * ======== vlanif.h ========
 *
 * The file contains defintions and structures which describe the VLAN
 * Network Interface unit. These definitions are available only with the
 * NIMU Packet Architecture.
 *
 */

#ifndef _C_VLANIF_INC
#define _C_VLANIF_INC

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************** 
 ************************* Local Definitions **************************
 **********************************************************************/

/* Limit Definitions: These are as per the IEEE802.1P specification. */
#define MAX_VLAN_ID     0xFFF
#define MAX_PRIO_VAL    8

/********************************************************************** 
 * Exported API (KERNEL MODE):
 *  These functions are exported by the VLAN and it is available for 
 *  internal NDK core stack usage only.
 ***********************************************************************/
extern void VLANInit (void);
extern void VLANDeinit (void);
extern uint32_t VLANReceivePacket (PBM_Handle hPkt);

/********************************************************************** 
 * Exported API (KERNEL MODE SAFE):
 *  These functions can be called from user-context to add and delete
 *  VLAN network nodes to the system.
 ***********************************************************************/ 

extern int VLANAddDevice (uint32_t index, uint16_t vlan_id, unsigned char dft_priority, unsigned char prio_mapping[MAX_PRIO_VAL]);
extern int VLANDelDevice (uint16_t dev_index);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* _C_VLANIF_INC */

