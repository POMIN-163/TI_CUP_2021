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
 *  ======== Configurations.syscfg.js ========
 */

"use strict";

let config = [
    {
        name: "maxPower",
        displayName: "Max Power (mA)",
        default: "500",
        description: "Max Power Consumption of Device in mA."
    },
    {
        name: "powerAttribute",
        displayName: "Power Attribute",
        default: "USB_CONF_ATTR_SELF_PWR",
        description: "How the device will draw power.",
        options: [
            { name: "USB_CONF_ATTR_SELF_PWR", displayName: "Self-Powered" },
            { name: "USB_CONF_ATTR_BUS_PWR", displayName: "Bus-Powered" }
        ]
    },
    {
        name: "remoteWakeUp",
        displayName: "Remote Wake-Up",
        default: false,
        description: "Ability for device to response to external wake signals while suspended."
    },
    {
        name: "deviceClass",
        displayName: "Device Class",
        default: "HID",
        longDescription:`
A device class roughly corresponds to an interface according to the USB standard.

To configure multiple interfaces, use the [__Composite Device Class__][1].
* Make sure there are enough endpoints to satisfy the number of devices included in the composite device.
* Extra function calls for the initialization are needed for composite devices in your application code. See the documentation for more information and an example.

[__Bulk Device Class__][2] (not to be confused with bulk endpoints) is not a standard USB device class, but it is a simpler method to set up USB communication, based off CDC.

The other listed classes are standard device classes. There may be some device classes the library supports that are not yet available in the SysConfig tool. To use them, users should generate the necessary structs on their own.

[1]: /usblib/msp432e4/users_guide/srcs/docs/usblib/msp432e4/users_guide/host_functions.html#composite-device-class-driver
[2]: /usblib/msp432e4/users_guide/srcs/docs/usblib/msp432e4/users_guide/host_functions.html#bulk-device-class-driver
`,
        options: [
            { name: "HID" },
            { name: "CDC" },
            { name: "Bulk" },
            { name: "MSC" },
            { name: "Composite" },
        ],
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
    switch (inst.deviceClass) {
        case "HID":
            reqs.push(
                {
                    name: "hid",
                    moduleName: "/ti/usblib/HID",
                    displayName: "Human Interface Device",
                    collapsed: true
                }
            )
        break;
        case "CDC":
            reqs.push(
                {
                    name: "cdc",
                    moduleName: "/ti/usblib/CDC",
                    displayName: "Communications Device Class",
                    collapsed: true
                }
            )
        break;
        case "Bulk":
            reqs.push(
                {
                    name: "bulk",
                    moduleName: "/ti/usblib/Bulk",
                    displayName: "Bulk Device Class",
                    collapsed: true
                }
            )
        break;
        case "MSC":
            reqs.push(
                {
                    name: "msc",
                    moduleName: "/ti/usblib/MSC",
                    displayName: "Mass Storage Class",
                    collapsed: true
                }
            )
        break;
        case "Composite":
            reqs.push(
                {
                    name: "composite",
                    moduleName: "/ti/usblib/Composite",
                    displayName: "Composite Device Class",
                    collapsed: true
                }
            )
        break;
    }   // switch inst.deviceClass
    return reqs;
}

/*
 *  ======== base ========
 *  Module definition object
 */
let base = {
    displayName: "Configurations",
    defaultInstanceName: "usb_configuration",
    description: "USBLib Configurations",
    config,
    maxInstances: 1,
    moduleInstances,
    templates,
    longDescription:`
This Configuration module is modelled after the standard USB Configuration Descriptor. The library supports multiple configurations, but the SysConfig tool for USB only supports one configuration for the sake of simplicity (and the overwhelmingly more common usecase).

In order to utilize multiple configurations, the user should manually create the structs that this tool would normally generate for them.
`,
};

/* export the module */
exports = base;

