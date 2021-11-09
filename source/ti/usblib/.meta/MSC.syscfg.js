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
 *  ======== MSC.syscfg.js ========
 */

"use strict";

let config = [
    {
        name: "$name",
        hidden: false
    },
    {
        displayName: "Device Name Information",
        config: [{
            name: "vendorName",
            displayName: "Vendor Name",
            default: "",
            placeholder: "8 characters",
            description: "Vendor Information."
        },
        {
            name: "productName",
            displayName: "Product Name",
            default: "",
            placeholder: "16 characters",
            description: "Product Identification."
        },
        {
            name: "version",
            displayName: "Version",
            default: "",
            placeholder: "4 characters",
            description: "Version Number."
        }]
    },
    {
        name: "usbHandler",
        displayName: "MSC Event Handler Function",
        default: "YourMSCEventCallback",
        placeholder: "YourMSCEventCallback",
        description: "Name of the callback function related to MSC events.",
        longDescription:`
More information [__here__][1].

[1]: /usblib/msp432e4/users_guide/srcs/docs/usblib/msp432e4/users_guide/device_functions.html#event-callbacks
`,
    },
    {
        displayName: "MSC Media Access Functions",
        config: [{
            name: "openFunction",
            displayName: "Open Function",
            default: "YourMSCOpenFunction",
            description: "Initializes and opens a physical drive number."
        },
        {
            name: "closeFunction",
            displayName: "Close Function",
            default: "YourMSCCloseFunction",
            description: "Closes the drive number used by the MSC device."
        },
        {
            name: "readFunction",
            displayName: "Read Function",
            default: "YourMSCReadFunction",
            description: "Reads from blocks."
        },
        {
            name: "writeFunction",
            displayName: "Write Function",
            default: "YourMSCWriteFunction",
            description: "Writes to blocks."
        },
        {
            name: "numBlocksFunction",
            displayName: "Number of Blocks Function",
            default: "YourMSCNumBlocksFunction",
            description: "Returns total number of blocks."
        },
        {
            name: "blockSizeFunction",
            displayName: "Block Size Function",
            default: "YourMSCBlockSizeFunction",
            description: "Returns block size."
        }]
    },
];

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
        validation.logError("Must provide an MSC Event Handler Function that handles USBD_MSC_EVENT_IDLE, USBD_MSC_EVENT_READING, and USBD_MSC_WRITING events.", inst, "usbHandler");
    }
    if (inst.openFunction.toUpperCase() === "NULL"
        || inst.openFunction === "0" || inst.openFunction === "") {
        validation.logError("Must provide an MSC Media Access Open Function for the physical media.", inst, "openFunction");
    }
    if (inst.closeFunction.toUpperCase() === "NULL"
        || inst.closeFunction === "0" || inst.closeFunction === "") {
        validation.logError("Must provide an MSC Media Access Close Function for the physical media.", inst, "closeFunction");
    }
    if (inst.readFunction.toUpperCase() === "NULL"
        || inst.readFunction === "0" || inst.readFunction === "") {
        validation.logError("Must provide an MSC Media Access Read Function for the physical media.", inst, "readFunction");
    }
    if (inst.writeFunction.toUpperCase() === "NULL"
        || inst.writeFunction === "0" || inst.writeFunction === "") {
        validation.logError("Must provide an MSC Media Access Write Function for the physical media.", inst, "writeFunction");
    }
    if (inst.numBlocksFunction.toUpperCase() === "NULL"
        || inst.numBlocksFunction === "0" || inst.numBlocksFunction === "") {
        validation.logError("Must provide an MSC Media Access Number of Blocks Function for the physical media.", inst, "numBlocksFunction");
    }
    if (inst.blockSizeFunction.toUpperCase() === "NULL"
        || inst.blockSizeFunction === "0" || inst.blockSizeFunction === "") {
        validation.logError("Must provide an MSC Media Access Block Size Function for the physical media.", inst, "blockSizeFunction");
    }
}

let templates = {
    "/ti/usblib/ti_usblib_config.c.xdt": true,
    "/ti/usblib/ti_usblib_config.h.xdt": true,
};

/*
 *  ======== base ========
 *  Module definition object
 */
let base = {
    displayName: "MSC",
    defaultInstanceName: "usb_msc",
    description: "Mass Storage Class",
    config,
    templates,
    validate,
};

/* export the module */
exports = base;

