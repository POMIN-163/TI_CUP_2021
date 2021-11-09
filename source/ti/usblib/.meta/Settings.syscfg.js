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
 *  ======== Settings.syscfg.js ========
 */

"use strict";

let config = [
    {
        name: "mode",
        displayName: "Mode",
        default: "Device",
        description: "Specifies USB mode.",
        options: [
            { name: "Device" },
            { name: "Host" }
        ],
        onChange: updateConfigs,
    },
    {
        name: "speed",
        displayName: "Speed",
        default: "Full Speed",
        description: "Specifies USB speed.",
        options: [
            { name: "Full Speed" },
            { name: "High Speed" }
        ]
    },
    {
        displayName: "VID/PID",
        config: [
            {
                name: "vendorID",
                displayName: "Vendor ID (VID)",
                default: "0x2047",
                description: "Specifies the USB device's vendor ID.",
                longDescription: "Assigned by USB Org.",
            },
            {
                name: "productID",
                displayName: "Product ID (PID)",
                default: "0x100",
                description: "Specifies the USB device's product ID.",
                longDescription: "Assigned by Manufacturer.",
            }
        ]
    },
    /*  Lock to 1 configuration for now
    {
        name: "numConfigurations",
        displayName: "Number of Configurations",
        default: 0,
    },*/
    {
        displayName: "Host Settings",
        // TODO: Custom HCD
        config: [
            {
                name: "hasHidHCD",
                displayName: "HID Host Class Driver",
                hidden: true,
                default: false,
                description: "Enable/disable Host Class Driver to communicate with HID device.",
                longDescription:`
More information about the [__HID__][1] class driver, as well as what application code still needs to be written.

[1]: /usblib/msp432e4/users_guide/srcs/docs/usblib/msp432e4/users_guide/host_functions.html#hid-class-driver
`,
            },
            {
                name: "hasMscHCD",
                displayName: "MSC Host Class Driver",
                hidden: true,
                default: false,
                description: "Enable/disable Host Class Driver to communicate with MSC device.",
                longDescription:`
More information about the [__MSC__][2] class driver, as well as what application code still needs to be written.

[2]: /usblib/msp432e4/users_guide/srcs/docs/usblib/msp432e4/users_guide/host_functions.html#mass-storage-class-driver
`,
            },
            {
                name: "hasCdcHCD",
                displayName: "CDC Host Class Driver",
                hidden: true,
                default: false,
                description: "Enable/disable Host Class Driver to communicate with CDC device.",
            },
            {
                name: "hasAudioHCD",
                displayName: "Audio Host Class Driver",
                hidden: true,
                default: false,
                description: "Enable/disable Host Class Driver to communicate with Audio device.",
                longDescription:`
More information about the [__Audio__][3] class driver, as well as what application code still needs to be written.

[3]: /usblib/msp432e4/users_guide/srcs/docs/usblib/msp432e4/users_guide/host_functions.html#audio-class-driver
`,
            },
            {
                name: "hasHubHCD",
                displayName: "Hub Class Driver",
                hidden: true,
                default: false,
                description: "Allows the USB controller to communicate with multiple USB devices.",
                longDescription:`
More information about [__Hub__][4] class driver, as well as what application code still needs to be written. Cascaded USB hubs are not supported by this USB library.

[4]: /usblib/msp432e4/users_guide/srcs/docs/usblib/msp432e4/users_guide/host_functions.html#hub-class-driver
`,
            },
            {
                name: "hasEventHCD",
                displayName: "Generic Event Host Controller Driver",
                hidden: true,
                default: true,
                description: "Enable/disable USB Event Driver to receive non-device class specific events.",
                longDescription:`
Can be used to notify users about insertion of an unsupported device or provide notification of power faults or shut off power.

A [__void USBHCDEvents(void *)__][5] function must be provided in the application to handle the different events.

[5]: /usblib/msp432e4/users_guide/srcs/docs/usblib/msp432e4/users_guide/host_functions.html#usb-events-driver
`,
            },
        ],
    },
];

function updateConfigs (inst, ui) {
    switch (inst.mode) {
        case "Device":
            // Device Configurables
            ui.vendorID.hidden = false;
            ui.productID.hidden = false;
            //ui.numConfigurations.hidden = false;
            // Host Configurables
            ui.hasHidHCD.hidden = true;
            ui.hasMscHCD.hidden = true;
            ui.hasCdcHCD.hidden = true;
            ui.hasAudioHCD.hidden = true;
            ui.hasHubHCD.hidden = true;
            ui.hasEventHCD.hidden = true;
            break;
        case "Host":
            // Device Configurables
            ui.vendorID.hidden = true;
            ui.productID.hidden = true;
            //ui.numConfigurations.hidden = true;
            // Host Configurables
            ui.hasHidHCD.hidden = false;
            ui.hasMscHCD.hidden = false;
            ui.hasCdcHCD.hidden = false;
            ui.hasAudioHCD.hidden = false;
            ui.hasHubHCD.hidden = false;
            ui.hasEventHCD.hidden = false;
            break;
        default:
            break;
    }
}

/*
 * Helps define the dependency chain and hierarchy.
 */
function moduleInstances(inst) {
    let reqs = [];
/*  Lock to 1 configuration for now
    for (let i = 0; i < inst.numConfigurations; i++) {
        reqs.push(
            {
                name: "configuration" + i,
                moduleName: "/ti/usblib/Configurations",
                displayName: "Configuration " + i,
                collapsed: true
            }
        )
    }
    */
    if (inst.mode === "Device") {
        reqs.push(
            {
                name: "stringDescriptor",
                moduleName: "/ti/usblib/StringDescriptor",
                displayName: "String Descriptor",
                collapsed: true,
            }
        )
        reqs.push(
            {
                name: "configuration",
                moduleName: "/ti/usblib/Configurations",
                displayName: "Configuration",
                collapsed: true,
            }
        )
    }
    return reqs;
}

/*
 *  ======== pinmuxRequirements ========
 *  Returns peripheral pin requirements of the specified instance
 *
 *  param inst    - a fully configured ADC instance
 *
 *  returns req[] - an array of pin requirements needed by inst
 */
function pinmuxRequirements(inst)
{
    let usb = {
        name: "usb",
        displayName: "USB Peripheral",
        interfaceName: "USB",
        resources: [
            {
                name: "dmPin",
                displayName: "D- Pin",
                interfaceNames: ["DM"]
            },
            {
                name: "dpPin",
                displayName: "D+ Pin",
                interfaceNames: ["DP"]
            },
            {
                name: "idPin",
                displayName: "Type Identification Pin",
                interfaceNames: ["ID"]
            },
            {
                name: "vbusPin",
                displayName: "VBUS Pin",
                interfaceNames: ["VBUS"]
            }
        ],
        signalTypes: {
            dmPin: ["USB_DM"],
            dpPin: ["USB_DP"],
            idPin: ["USB_ID"],
            vbusPin: ["USB_VBUS"]
        }
    };
    if (inst.mode === "Host") {
        usb.resources.push(
            {
                name: "epEnPin",
                displayName: "External Power Enable Pin",
                interfaceNames: ["EPEN"]
            },
            {
                name: "pFltPin",
                displayName: "Power Fault Pin",
                interfaceNames: ["PFLT"]
            }
        );
        usb.signalTypes.epEnPin = ["USB_EPEN"];
        usb.signalTypes.pFltPin = ["USB_PFLT"];
    }
    if (inst.speed === "High Speed") {
        usb.resources.push(
            {
                name: "clkPin",
                displayName: "Clock Pin",
                interfaceNames: ["CLK"]
            },
            {
                name: "d0Pin",
                displayName: "Data 0 Pin",
                interfaceNames: ["D0"]
            },
            {
                name: "d1Pin",
                displayName: "Data 1 Pin",
                interfaceNames: ["D1"]
            },
            {
                name: "d2Pin",
                displayName: "Data 2 Pin",
                interfaceNames: ["D2"]
            },
            {
                name: "d3Pin",
                displayName: "Data 3 Pin",
                interfaceNames: ["D3"]
            },
            {
                name: "d4Pin",
                displayName: "Data 4 Pin",
                interfaceNames: ["D4"]
            },
            {
                name: "d5Pin",
                displayName: "Data 5 Pin",
                interfaceNames: ["D5"]
            },
            {
                name: "d6Pin",
                displayName: "Data 6 Pin",
                interfaceNames: ["D6"]
            },
            {
                name: "d7Pin",
                displayName: "Data 7 Pin",
                interfaceNames: ["D7"]
            },
            {
                name: "dirPin",
                displayName: "Direction Pin",
                interfaceNames: ["DIR"]
            },
            {
                name: "nxtPin",
                displayName: "NXT Pin",
                interfaceNames: ["NXT"]
            },
            {
                name: "stpPin",
                displayName: "Stop Pin",
                interfaceNames: ["STP"]
            },
        );
        usb.signalTypes.clkPin = ["USB_CLK"];
        usb.signalTypes.d0Pin = ["USB_D0"];
        usb.signalTypes.d1Pin = ["USB_D1"];
        usb.signalTypes.d2Pin = ["USB_D2"];
        usb.signalTypes.d3Pin = ["USB_D3"];
        usb.signalTypes.d4Pin = ["USB_D4"];
        usb.signalTypes.d5Pin = ["USB_D5"];
        usb.signalTypes.d6Pin = ["USB_D6"];
        usb.signalTypes.d7Pin = ["USB_D7"];
        usb.signalTypes.dirPin = ["USB_DIR"];
        usb.signalTypes.nxtPin = ["USB_NXT"];
        usb.signalTypes.stpPin = ["USB_STP"];
    }

    // TODO: reserve PQ4 for host at some point
    /*let gpio = {
        name: "usbFaultHwiPin",
        displayName: "USB Fault HWI Pin",
        interfaceName: "GPIO",
        signalTypes: ["DIN", "DOUT"]
    };*/
    return ([usb]);
}

/*
 *  ======== filterHardware ========
 *
 *  param component - hardware object describing signals and
 *                     resources they're attached to
 *
 *  returns Boolean indicating whether or not to allow the component to
 *           be assigned to an instance's $hardware config
 */
function filterHardware(component)
{
    let options = {};

    if (component.type instanceof Array || typeof component.type == "object") {
        for (let i = 0; i < component.type.length; i++) {
            options[component.type[i]] = 1;
        }
    }
    else if (typeof component.type == "string" || component.type instanceof String) {
        options[component.type] = 1;
    }

    if ("USB" in options) {
        return (true);
    }

    return (false);
}

/*
 *  ======== validate ========
 *  Validate this inst's configuration
 *
 *  @param inst       - instance to be validated
 *  @param validation - object to hold detected validation issues
 */
function validate(inst, validation) {
    if (inst.$hardware == null) {  // is null when None is selected
        validation.logInfo("Recommended to change the Use Hardware field to USB Micro Connector.", inst, "$hardware");
    }
    if (inst.speed === "High Speed") {
        validation.logInfo("High Speed requires an external ULPI PHY.", inst, "$hardware");
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
exports = {
    displayName: "USB",
    defaultInstanceName: "CONFIG_USB_",
    description: "USBLib Settings",
    config,
    maxInstances: 1,
    moduleInstances,
    templates,
    pinmuxRequirements,
    filterHardware,
    validate,
};

