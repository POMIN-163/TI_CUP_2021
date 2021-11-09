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
 *  ======== Composite.syscfg.js ========
 */

"use strict";

let config = [
    {
        name: "$name",
        hidden: false
    },
    {
        name: "usbHandler",
        displayName: "USB Callback Handler",
        default: "YourCompositeEventCallback",
        placeholder: "YourCompositeEventCallback",
        description: "Name of the callback function related to composite USB events.",
        longDescription:`
More information [__here__][1].

[1]: /usblib/msp432e4/users_guide/srcs/docs/usblib/msp432e4/users_guide/device_functions.html#composite-device-class-driver
`,
    },
    {
        name: "numHid",
        displayName: "# of HID classes",
        default: 0
    },
    {
        name: "numCdc",
        displayName: "# of CDC classes",
        default: 0
    },
    {
        name: "numBulk",
        displayName: "# of Bulk classes",
        default: 0
    },
    {
        name: "numMsc",
        displayName: "# of MSC classes",
        default: 0
    },
];

let templates = {
    "/ti/usblib/ti_usblib_config.c.xdt": true,
    "/ti/usblib/ti_usblib_config.h.xdt": true,
};

/*
 * Helps define the dependency chain and hierarchy.
 */
function moduleInstances(inst)
{
    let reqs = [];
    /*reqs.push(
        {
            name: "stringDescriptor",
            moduleName: "/ti/usblib/StringDescriptor",
            displayName: "String Descriptor",
            collapsed: true
        }
    )*/
    // There can only be 255 instances in 1 configuration, per USB standard
    // Catch this here otherwise GUI may crash for trying to add too many
    if (inst.numHid + inst.numCdc + inst.numBulk + inst.numMsc > 255) {
        return reqs;
    }
    for (let i = 0; i < inst.numHid; i++) {
        reqs.push(
            {
                name: "hid" + i,
                moduleName: "/ti/usblib/HID",
                displayName: "Human Interface Device " + i,
                collapsed: true
            }
        )
    }
    for (let i = 0; i < inst.numCdc; i++) {
        reqs.push(
            {
                name: "cdc" + i,
                moduleName: "/ti/usblib/CDC",
                displayName: "Communications Device Class " + i,
                collapsed: true
            }
        )
    }
    for (let i = 0; i < inst.numBulk; i++) {
        reqs.push(
            {
                name: "bulk" + i,
                moduleName: "/ti/usblib/Bulk",
                displayName: "Bulk Device Class " + i,
                collapsed: true
            }
        )
    }
    for (let i = 0; i < inst.numMsc; i++) {
        reqs.push(
            {
                name: "msc" + i,
                moduleName: "/ti/usblib/MSC",
                displayName: "Mass Storage Device Class " + i,
                collapsed: true
            }
        )
    }
    return reqs;
}

/*
 *  ======== validate ========
 *  Validate this inst's configuration
 *
 *  @param inst       - instance to be validated
 *  @param validation - object to hold detected validation issues
 */
function validate(inst, validation) {
    if (inst.usbHandler.toUpperCase() === "NULL"
        || inst.usbHandler === "0" || inst.usbHandler === "") {
        validation.logInfo("Consider providing a Composite Event Handler Function that handle USB_EVENT_CONNECTED and USB_EVENT_DISCONNECTED events.", inst, "usbHandler");
    }
    if (inst.numHid + inst.numCdc + inst.numBulk + inst.numMsc > 255) {
        validation.logError("A composite device can only have up to 255 interfaces.", inst);
    }
}

/*
 *  ======== base ========
 *  Module definition object
 */
let base = {
    displayName: "Composite",
    defaultInstanceName: "usb_composite",
    description: "Composite Device Class",
    config,
    moduleInstances,
    templates,
    validate,
};

/* export the module */
exports = base;

