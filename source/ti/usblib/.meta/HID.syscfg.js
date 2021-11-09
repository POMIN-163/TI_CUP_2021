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
 *  ======== HID.syscfg.js ========
 */

"use strict";

let config = [
    {
        name: "$name",
        hidden: false
    },
    {
        name: "hidType",
        displayName: "HID Type",
        default: "Generic",
        options: [
            { name: "Generic" },
            { name: "Keyboard"},
            { name: "Mouse"   },
            { name: "Gamepad" },
            { name: "Sensor"  }
        ],
        longDescription: "Although any HID device can be set up by the Generic type, choosing a more specific HID type can simplify your application.",
        onChange: updateConfigs
    },
    {
        name: "subclass",
        displayName: "Subclass",
        default: "USB_HID_SCLASS_NONE",
        options: [
            { name: "USB_HID_SCLASS_NONE", displayName: "None" },
            { name: "USB_HID_SCLASS_BOOT", displayName: "Boot" },
        ],
        description: "Follows standard USB definition.",
    },
    {
        name: "protocol",
        displayName: "Protocol",
        default: "USB_HID_PROTOCOL_NONE",
        options: [
            { name: "USB_HID_PROTOCOL_NONE", displayName: "None" },
            { name: "USB_HID_PROTOCOL_KEYB", displayName: "Keyboard" },
            { name: "USB_HID_PROTOCOL_MOUSE", displayName: "Mouse" },
            { name: "USB_HID_PROTOCOL_BOOT", displayName: "Boot" },
            { name: "USB_HID_PROTOCOL_REPORT", displayName: "Report" },
        ],
        description: "Follows standard USB definition.",
    },
    {
        displayName: "Callback Handler",
        config: [{
            name: "rxHandler",
            displayName: "Receive Event Handler Function",
            default: "YourHIDReceiveEventCallback",
            placeholder: "YourHIDReceiveEventCallback",
            description: "Name of the callback function called when receiving report transfers and Output/Feature reports."
        },
        {
            name: "rxHandlerData",
            displayName: "Receive Event Handler Data",
            default: "YourHIDReceiveInstanceData",
            placeholder: "YourHIDReceiveInstanceData",
            description: "Name of the struct passed to the receive callback alongside every event."
        },
        {
            name: "txHandler",
            displayName: "Transmit Event Handler Function",
            default: "YourHIDTransmitEventCallback",
            placeholder: "YourHIDTransmitEventCallback",
            description: "Name of the callback function called when transmitting Input reports."
        },
        {
            name: "txHandlerData",
            displayName: "Transmit Event Handler Data",
            default: "YourHIDTransmitInstanceData",
            placeholder: "YourHIDTransmitInstanceData",
            description: "Name of the struct passed to the transmit callback alongside every event."
        },
        {
            name: "usbHandler",
            displayName: "HID Event Handler Function",
            default: "YourHIDEventCallback",
            placeholder: "YourHIDEventCallback",
            hidden: true,
            description: "Name of the callback function called related to non-generic HID events."
        },
        {
            name: "usbHandlerData",
            displayName: "HID Event Handler Data",
            default: "YourHIDInstanceData",
            placeholder: "YourHIDInstanceData",
            hidden: true,
            description: "Name of the first parameter passed to the HID callback alongside every event."
        }]
    },
    {
        displayName: "Report Descriptor",
        config: [{
            name: "reportDescriptor",
            displayName: "Report Descriptor",
            default: "YourHIDReportDescriptor",
            placeholder: "YourHIDReportDescriptor",
            description: "Name of the HID device's report descriptor structure. Expected type is a (const uint8_t []).",
            longDescription:`
More information [__here__][1]. Due to the variability of HID report descriptors, it must be defined by the user and provided to the USB library. In the special case of gamepad, put 0 if application is using default gamepad descriptor.

[1]: /usblib/msp432e4/users_guide/srcs/docs/usblib/msp432e4/users_guide/device_functions.html#using-the-hid-device-class-driver
`,
        },
        {
            name: "reportIdleDescriptor",
            displayName: "Report Idle Descriptor",
            default: "YourHIDReportIdleDescriptor",
            placeholder: "YourHIDReportIdleDescriptor",
            description: "Name of the HID device's report idle descriptor structure. Expected type is a tHIDReportIdle struct.",
            longDescription:`
More information [__here__][2]. Not part of the standard report descriptor, but is used in the USB library. See the documentation for how to construct this report.

[2]: /usblib/msp432e4/users_guide/srcs/docs/usblib/msp432e4/users_guide/device_functions.html#using-the-hid-device-class-driver
`,
        },
        {
            name: "reportDescriptorSize",
            displayName: "Size of Report Descriptor",
            default: 0,
            description: "Size of the optional report descriptor override in gamepad. Used only in gamepad. A value of 0 invalidates the optional report descriptor."
        }]
    }
];

function updateConfigs (inst, ui) {
    switch (inst.hidType) {
        case "Generic":
            ui.subclass.hidden = false;
            ui.protocol.hidden = false;
            ui.txHandler.hidden = false;
            ui.txHandlerData.hidden = false;
            ui.rxHandler.hidden = false;
            ui.rxHandlerData.hidden = false;
            ui.usbHandler.hidden = true;
            inst.usbHandler = "YourHIDEventCallback";
            ui.usbHandlerData.hidden = true;
            inst.usbHandlerData = "YourHIDInstanceData";
            ui.reportDescriptor.hidden = false;
            ui.reportIdleDescriptor.hidden = false;
            ui.reportDescriptorSize.hidden = false;
            break;
        case "Gamepad":
            ui.subclass.hidden = true;
            ui.protocol.hidden = true;
            ui.txHandler.hidden = true;
            inst.txHandler = "YourHIDTransmitEventCallback";
            ui.txHandlerData.hidden = true;
            inst.txHandlerData = "YourHIDTransmitInstanceData";
            ui.rxHandler.hidden = true;
            inst.rxHandler = "YourHIDReceiveEventCallback";
            ui.rxHandlerData.hidden = true;
            inst.rxHandlerData = "YourHIDReceiveInstanceData";
            ui.usbHandler.hidden = false;
            ui.usbHandlerData.hidden = false;
            ui.reportDescriptor.hidden = false;
            ui.reportIdleDescriptor.hidden = true;
            inst.reportIdleDescriptor = "YourHIDReportIdleDescriptor";
            ui.reportDescriptorSize.hidden = false;
            break;
        default:
            ui.subclass.hidden = true;
            ui.protocol.hidden = true;
            ui.txHandler.hidden = true;
            inst.txHandler = "YourHIDTransmitEventCallback";
            ui.txHandlerData.hidden = true;
            inst.txHandlerData = "YourHIDTransmitInstanceData";
            ui.rxHandler.hidden = true;
            inst.rxHandler = "YourHIDReceiveEventCallback";
            ui.rxHandlerData.hidden = true;
            inst.rxHandlerData = "YourHIDReceiveInstanceData";
            ui.usbHandler.hidden = false;
            ui.usbHandlerData.hidden = false;
            ui.reportDescriptor.hidden = true;
            inst.reportDescriptor = "YourHIDReportDescriptor";
            ui.reportIdleDescriptor.hidden = true;
            inst.reportIdleDescriptor = "YourHIDReportIdleDescriptor";
            ui.reportDescriptorSize.hidden = true;
            break;
    }
}

/*
 * Helps define the dependency chain and hierarchy.
 */
function moduleInstances(inst) {
    let reqs = [];
    reqs.push(
        {
            name: "stringDescriptor",
            moduleName: "/ti/usblib/StringDescriptor",
            displayName: "String Descriptor",
            collapsed: true
        }
    )
    return reqs;
};

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
    if (inst.rxHandler.toUpperCase() === "NULL"
        || inst.rxHandler === "0" || inst.rxHandler === "") {
        validation.logError("Must provide a Receive Event Handler Function that handles USB_EVENT_RX_AVAILABLE, USBD_HID_EVENT_IDLE_TIMEOUT, USBD_HID_EVENT_GET_REPORT_BUFFER, USBD_HID_EVENT_GET_REPORT, USBD_HID_EVENT_SET_REPORT events.", inst, "rxHandler");
    }
    if (inst.txHandler.toUpperCase() === "NULL"
        || inst.txHandler === "0" || inst.txHandler === "") {
        validation.logError("Must provide a Transmit Event Handler Function that handles a USB_EVENT_TX_COMPLETE event.", inst, "txHandler");
    }
    if (inst.usbHandler.toUpperCase() === "NULL"
        || inst.usbHandler === "0" || inst.usbHandler === "") {
        validation.logError("Must provide a HID Event Handler Function.", inst, "usbHandler");
    }
    if (inst.reportDescriptor.toUpperCase() === "NULL" || inst.reportDescriptor === "") {
        validation.logError("Must provide a HID Report Descriptor.", inst, "reportDescriptor");
    }
    if (inst.reportIdleDescriptor.toUpperCase() === "NULL" || inst.reportIdleDescriptor === "") {
        validation.logError("Must provide a HID Report Idle Descriptor.", inst, "reportIdleDescriptor");
    }
}

/*
 *  ======== base ========
 *  Module definition object
 */
let base = {
    displayName: "HID",
    defaultInstanceName: "usb_hid",
    description: "Human Interface Device Class",
    config,
    templates,
    validate,
};

/* export the module */
exports = base;

