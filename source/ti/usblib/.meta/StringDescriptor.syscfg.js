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
 *  ======== StringDescriptor.syscfg.js ========
 */

"use strict";

let config = [
    {
        name: "languageDescriptor",
        displayName: "Language Descriptor",
        default: ["ENGLISH_US"],
        description: "List of languages this device supports.",
        /* TODO: can support multiple languages (although a rare request) */
        options: [
            { name: "ENGLISH_US" }
        ]
    },
    {
        name: "manufacturerName",
        displayName: "Manufacturer Name",
        default: "Texas Instruments",
        placeholder: "Example: Texas Instruments",
        description: "Manufacturer name string."
    },
    {
        name: "productName",
        displayName: "Product Name",
        default: "Generic Device",
        placeholder: "Example: MSP432E401Y",
        description: "Product name string that appears upon enumeration."
    },
    {
        name: "serialNumber",
        displayName: "Serial Number",
        default: "12345678",
        placeholder: "Example: 12345678",
        description: "Serial number string."
    },
    {
        /* TODO: these should be per configuration */
        name: "configurationDescription",
        displayName: "Configuration Description",
        default: "Generic Configuration",
        description: "Configuration description string."
    },
    {
        /* TODO: these should be per interface */
        name: "interfaceDescription",
        displayName: "Interface Description",
        default: "Generic Interface",
        description: "Interface description string."
    },
];

/*
 *  ======== base ========
 *  Module definition object
 */
let base = {
    displayName: "String Descriptor",
    defaultInstanceName: "usb_string",
    description: "USBLib String Descriptor",
    config,
};

/* export the module */
exports = base;

