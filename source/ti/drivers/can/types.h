/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
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
 */

#ifndef ti_drivers_can_types__include
#define ti_drivers_can_types__include

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief   CAN identifier structure
 *
 * bit 0-28 : CAN identifier (11/29 bit)
 * bit 29   : error message frame flag (0 = data frame, 1 = error message)
 * bit 30   : remote transmission request flag (1 = rtr frame)
 * bit 31   : frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
 */
typedef uint32_t canid_t;

/*!
 * @brief   CAN frame structure
 *
 * The structure that makes up a CAN message. The unions are provided in order
 * for there to be structural naming compatibility with SocketCAN while
 * at the same time providing an alternative easier to use naming convention.
 * We diverge a bit with TI structural naming convention of the struct in order
 * to provide an option to be compatible with SocketCAN conventions.
 *
 * @sa       CAN_write()
 * @sa       CAN_read()
 */
struct can_frame {
    union {
        canid_t can_id; /*!< 11/29-bit CAN ID + EFF/RTR/ERR flags, SocketCAN */
        struct {
            uint32_t id  : 29; /*!< 11/29-bit CAN ID */
            uint32_t err :  1; /*!< error flag */
            uint32_t rtr :  1; /*!< remote frame flag */
            uint32_t eff :  1; /*!< extended frame format flag */
        };
    };
    union
    {
        uint8_t can_dlc; /*!< data length code, SocketCAN compatible */
        uint8_t dlc;     /*!< data length code */
    };
    uint8_t __pad;   /*!< alignment padding */
    uint8_t __res0;  /*!< reserved */
    uint8_t __res1; /*!< reserved */
    uint8_t data[8] __attribute__((aligned(8))); /*!< CAN frame payload data */
};

/*! SocketCAN compatible bit flag.
 *
 * @sa       CAN_frame
 * @sa       struct can_frame
 */
#define CAN_EFF_FLAG 0x80000000U

/*! SocketCAN compatible bit flag.
 *
 * @sa       CAN_frame
 * @sa       struct can_frame
 */
#define CAN_RTR_FLAG 0x40000000U

/*! SocketCAN compatible bit flag.
 *
 * @sa       CAN_frame
 * @sa       struct can_frame
 */
#define CAN_ERR_FLAG 0x20000000U

/*! SocketCAN compatible bit mask.
 *
 * @sa       CAN_frame
 * @sa       struct can_frame
 */
#define CAN_SFF_MASK 0x000007FFU

/*! SocketCAN compatible bit mask.
 *
 * @sa       CAN_frame
 * @sa       struct can_frame
 */
#define CAN_EFF_MASK 0x1FFFFFFFU

/*! SocketCAN compatible bit mask.
 *
 * @sa       CAN_frame
 * @sa       struct can_frame
 */
#define CAN_ERR_MASK 0x1FFFFFFFU

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_can_types__include */
