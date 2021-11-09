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
 *  ======== DNS.syscfg.js ========
 *  DNS configuration support
 */

"use strict";
/* global exports, system */

//console.log("load: ti/ndk/DNS");

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

    /* Check for conflicting options across multiple instances */
    let thisNimu = "";
    if(inst.setDNSSIP == "By NDK Interface")
    {
        thisNimu = inst.NIMU.$name;
    }
    let instances = inst.$module.$instances;
    for(let i = 0; i < instances.length; i++)
    {
        if((instances[i].setDNSSIP == "By NDK Interface" &&
           instances[i].NIMU.$name == thisNimu) &&
           instances[i].$name != inst.$name)
        {
            vo.logError(instances[i].$name + " is already using the NDK " +
                        "Interface named " + thisNimu, inst);
        }

        if((instances[i].setDNSSIP == "Any IP Address" &&
           inst.setDNSSIP == "Any IP Address") &&
           instances[i].$name != inst.$name)
        {
            vo.logError(instances[i].$name +
                        " is already using the \"Any IP Address\" option",
                        inst);
        }
    }
}

/*
 *  ======== longDescription ========
 *  Intro splash on GUI
 */
let longDescription =
    "Create and configure DNS Server instances. Multiple DNS server " +
    "instances are supported, but each one must be configured to have " +
    "a unique IP address. This means only one instance can be bound to \"Any " +
    "IP Address\"";

/*
 *  ======== config_module ========
 *  Define the config params of an instance
 */
let config_instance = [
    {
        name: "setDNSSIP",
        displayName: "Set DNS Server IP Address:",
        hidden: false,
        default: "Any IP Address",
        options: [
            {
                name: "Any IP Address"
            },
            {
                name: "By NDK Interface"
            }
        ],
        longDescription: `
Use this option to choose which device IP address(es) the DNS server will
bind itself too.

Any IP Addresses - Binds the service to any IP address on the target.
Equivalent to setting IP Address to "0.0.0.0"

By Interface ID - Allows you to choose an interface ID, and the DNS Service
will bind to whatever IP that interface receives.

[More ...](/ndk/ConfigDoc.html#ti_ndk_DNS_setDNSSIP)`,
        documentation: `
This setting manipulates the common argument structure for NDK services.
It alters the Mode field documentated in
[NDK API Guide](NDK_API_Reference.html#common-argument-structure).
`
    },
    {
        name: "ndkInterface",
        displayName: "",
        config: []
    },
    {
        name: "servReportFxn",
        displayName: "Service Report Function",
        default: "",
        description: "DNS service reporting function.",
        longDescription: `
Optional, user defined service report function to handle DNS Server reports.
Note that multiple services (e.g. DNS Server and DHCP Server) can use the
same service report function.

If set, this service report function must be provided by the application, and
the function signature must match this prototype:

    extern void userDNSServerServReportFxn(uint32_t item, uint32_t status,
            uint32_t report, void *h);

[More ...](/ndk/ConfigDoc.html#ti_ndk_DNS_servReportFxn)`,
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

    if(inst.setDNSSIP == "By NDK Interface")
    {
        modules.push(
            {
                name: "NIMU",
                displayName: "NDK Interface",
                moduleName: "/ti/ndk/NIMU",
                description: "The NDK Interface to run this DNS Server from",
                collapsed: false,
                hidden: false,
                group: "ndkInterface"
            }
        );
    }

    return modules;
}

/*
 *  ======== base ========
 *  Module definition object
 */
let base = {
    displayName: "DNS Server",
    description: "NDK Domain Name System (DNS) configuration",
    defaultInstanceName: "CONFIG_DNSS_",
    longDescription: longDescription,
    config: config_instance,
    validate: validate_instance,
    sharedModuleInstances: moduleInstances,
    moduleStatic: {
        modules: modules
    },
    templates: {
        "/ti/ndk/Config.c.xdt": "/ti/ndk/DNS.Config.c.xdt"
    }
};

/* export the module */
exports = base;
