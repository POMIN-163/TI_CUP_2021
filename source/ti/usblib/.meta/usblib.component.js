/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated - http://www.ti.com
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
 *  ======== usblib.component.js ========
 */

"use strict";

let topModules;
let displayName = "USB Library (USBLib)";
let description = "USB System Configuration";
let deviceId = system.deviceData.deviceId;

if (deviceId.match(/MSP432E.*/)) {
    /* MSP432E */
    topModules = [
        {
            displayName: displayName,
            description: description,
            modules: [
                "/ti/usblib/Settings",
            ],
            /* Hide classes categories for now: too much complexity
            categories: [
            {
                displayName: "Classes",
                modules: [
                    "/ti/usblib/HID",
                    "/ti/usblib/CDC",
                    "/ti/usblib/Bulk",
                    "/ti/usblib/Composite",
                    "/ti/usblib/MSC",
                ],
            }
            ]
            */
        }
    ];
}

let templates = [
    {
        "name": "/ti/usblib/ti_usblib_config.c.xdt",
        "outputPath": "ti_usblib_config.c",
    },
    {
        "name": "/ti/usblib/ti_usblib_config.h.xdt",
        "outputPath": "ti_usblib_config.h",
    }
];

exports = {
    displayName: displayName,
    topModules: topModules,
    templates: templates
};
