/*
 * Copyright (c) 2018-2019 Texas Instruments Incorporated - http://www.ti.com
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
 *
 */

/*
 *  ======== Bulk.syscfg.js ========
 */

"use strict";

let config = [
    {
        name: "$name",
        hidden: false
    },
    {
        name: "rxBufferHandler",
        displayName: "Receive Event Handler Function",
        default: "YourBulkReceiveEventCallback",
        placeholder: "YourBulkReceiveEventCallback",
        description: "Callback function called for events related to data receive channel.",
        longDescription:`
More information [__here__][1].

[1]: /usblib/msp432e4/users_guide/srcs/docs/usblib/msp432e4/users_guide/device_functions.html#receive-channel-events
`,
    },
    {
        name: "txBufferHandler",
        displayName: "Transmit Event Handler Function",
        default: "YourBulkTransmitEventCallback",
        placeholder: "YourBulkTransmitEventCallback",
        description: "Callback function called for events related to data transmit channel.",
        longDescription:`
More information [__here__][2].

[2]: /usblib/msp432e4/users_guide/srcs/docs/usblib/msp432e4/users_guide/device_functions.html#transmit-channel-events
`,
    }
];

let templates = {
    "/ti/usblib/ti_usblib_config.c.xdt": true,
    "/ti/usblib/ti_usblib_config.h.xdt": true,
};

/*
 *  ======== validate ========
 *  Validate this inst's configuration
 *
 *  @param inst       - instance to be validated
 *  @param validation - object to hold detected validation issues
 */
function validate(inst, validation) {
    if (inst.rxBufferHandler.toUpperCase() === "NULL"
        || inst.rxBufferHandler === "0" || inst.rxBufferHandler === "") {
        validation.logError("Must provide a Receive Event Handler Function that handles USB_EVENT_RX_AVAILABLE event.", inst);
    }
    if (inst.txBufferHandler.toUpperCase() === "NULL"
        || inst.txBufferHandler === "0" || inst.txBufferHandler === "") {
        validation.logInfo("Consider providing a Transmit Event Handler Function that handles USB_EVENT_TX_COMPLETE event.", inst);
    }
}

/*
 *  ======== base ========
 *  Module definition object
 */
let base = {
    displayName: "Bulk",
    defaultInstanceName: "usb_bulk",
    description: "Bulk Device Class",
    config,
    templates,
    validate,
};

/* export the module */
exports = base;

