/*
 * Copyright (c) 2018-2020 Texas Instruments Incorporated - http://www.ti.com
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

/*
 *  ======== NAT.syscfg.js ========
 *  NAT configuration support
 */

"use strict";
/* global exports, system */

//console.log("load: ti/ndk/NAT");
let Common = system.getScript("/ti/utils/Common.js");

/*
 *  ======== modules ========
 *  Express dependencies for other modules
 *
 *  Invoked on any configuration change to the given instance.
 */
function modules(inst)
{
    let modules = new Array();

    modules.push({
        name: "general",
        moduleName: "/ti/ndk/General"
    });

    return (modules);
}

/*
 *  ======== validate_instance ========
 *  Validate given instance and report conflicts
 *
 *  This function is not allowed to modify the instance state.
 */
function validate_instance(inst, vo, getRef)
{
    /* servReportFxn */
    if ((inst.servReportFxn != "") && (!Common.isCName(inst.servReportFxn))) {
        vo["servReportFxn"].errors.push("Not a valid C identifier.");
    }
}

/*
 *  ======== longDescription ========
 *  Intro splash on GUI
 */
let longDescription =
    "Create and configure NAT Service. Only one NAT Service instance is " +
    "supported. The NAT Service needs to be associated with a unique " +
    "NDK Interface";

/*
 *  ======== config_module ========
 *  Define the config params of an instance
 */
let config_instance = [
    {
        name: "virtualIpAddr",
        displayName: "Virtual IP Address",
        textType: "ipv4_address",
        default: "0.0.0.0",
        readOnly: false,
        description: "Specifies the NAT group virtual network IP address.",
        longDescription: `
Sets the NAT Group virtual network address

[More ...](/ndk/ConfigDoc.html#ti_ndk_NAT_virtualIpAddr)`,
        documentation: `
More information on the NAT Service can be found in the
[NDK API Guide](NDK_API_Reference.html#network-address-translation-nat-service).
`
    },
    {
        name: "virtualIPMask",
        displayName: "Virtual IP Mask",
        textType: "ipv4_address",
        default: "255.255.255.0",
        readOnly: false,
        description: "Specifies the subnet mask of the NAT group virtual network.",
        longDescription: `
Sets the subnet mask of NAT Group virtual network

[More ...](/ndk/ConfigDoc.html#ti_ndk_NAT_virtualIPMask)`,
        documentation: `
More information on the NAT Service can be found in the
[NDK API Guide](NDK_API_Reference.html#network-address-translation-nat-service).
`
    },
    {
        name: "mtu",
        displayName: "MTU",
        default: "1500",
        readOnly: false,
        description: "Specifies the IP MTU limit.",
        longDescription: `
Sets the IP MTU Limit (1500 for Ethernet, 1492 for PPPoE, etc.)

[More ...](/ndk/ConfigDoc.html#ti_ndk_NAT_mtu)`,
        documentation: `
More information on the NAT Service can be found in the
[NDK API Guide](NDK_API_Reference.html#network-address-translation-nat-service).
`
    },
    {
        name: "servReportFxn",
        displayName: "Service Report Function",
        default: "",
        description: "NAT Server service reporting function.",
        longDescription: `
Optional, user defined service report function to handle NAT Server reports.
Note that multiple services (e.g. NAT Server and DHCP Server) can use the
same service report function.

If set, this service report function must be provided by the application, and
the function signature must match this prototype:

    extern void userNATServerServReportFxn(uint32_t item, uint32_t status,
            uint32_t report, void *h);

[More ...](/ndk/ConfigDoc.html#ti_ndk_NAT_servReportFxn)`,
        documentation: `
For more information on service report functions see the
[NDK User's Guide](ug/ug_ndk/03_network-application-development.html#adding-status-report-services).
`
    }
];

/*
 *  ======== moduleInstances ========
 */
function moduleInstances(inst)
{
    let modules = [];

    modules.push(
        {
            name: "NIMU",
            displayName: "NDK Interface",
            moduleName: "/ti/ndk/NIMU",
            description: "The NDK Interface to run this NAT instance from",
            collapsed: false,
            hidden: false
        }
    );

    return modules;
}

/*
 *  ======== base ========
 *  Module definition object
 */
let base = {
    displayName: "NAT",
    description: "NDK NAT Service Configuration",
    defaultInstanceName: "CONFIG_NAT_",
    longDescription: longDescription,
    config: config_instance,
    validate: validate_instance,
    sharedModuleInstances: moduleInstances,
    maxInstances: 1,
    moduleStatic: {
        modules: modules
    },
    templates: {
        "/ti/ndk/Config.c.xdt": "/ti/ndk/NAT.Config.c.xdt"
    }
};

/* export the module */
exports = base;
