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
 *  ======== DHCP.syscfg.js ========
 *  DHCP configuration support
 */

"use strict";
/* global exports, system */

//console.log("load: ti/ndk/DHCP");

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
    /* Server interface ID */
    if(inst.ifIdx < 1 ) {
        vo["ifIdx"].errors.push("Must be greater than 0");
    }

    if ((inst.servReportFxn != "") && (!Common.isCName(inst.servReportFxn))) {
        vo["servReportFxn"].errors.push("Not a valid C identifier.");
    }

    let thisNimu = inst.NIMU.$name;
    let instances = inst.$module.$instances;
    for(let i = 0; i < instances.length; i++)
    {
        if(instances[i].NIMU.$name == thisNimu &&
           instances[i].$name != inst.$name)
        {
            vo.logError(instances[i].$name + " is already using the NDK " +
                        "Interface named " + thisNimu, inst);
        }
    }

}

/*
 *  ======== longDescription ========
 *  Intro splash on GUI
 */
let longDescription =
    "Create and configure DHCP server instances. Multiple DHCP Server " +
    "instances are supported, but each one must be configured to have " +
    "a unique NDK Interface.";

/*
 *  ======== config_module ========
 *  Define the config params of an instance
 */
let config_instance = [
    {
        displayName: "DHCP Server Flags",
        config: [
            {
                name: "reportLocalDomainName",
                displayName: "Report Local Domain Name",
                default: false,
                description: "Report the local domain name assigned to the " +
                    "virtual network to clients (DHCPS_FLG_LOCALDOMAIN)",
                longDescription: `
Causes DHCPS to report the local domain name assigned to the virtual network to
clients. If this flag is not set, DHCPS reports the public domain name to
clients

[More ...](/ndk/ConfigDoc.html#ti_ndk_DHCP_reportLocalDomainName)`,
                documentation: `
More information on the DHCP Server flags can be found in the
[NDK API Flag](NDK_API_Reference.html#dhcp-server-parameter-structure).
`
            },
            {
                name: "reportLocalDNSS",
                displayName: "Report Ourselves as the Local DNS Server",
                default: false,
                description: "Report ourselves as the local DNS server to " +
                    "clients (DHCPS_FLG_LOCALDNS)",
                longDescription: `
Causes DHCPS to report its own IP address as the local DNS server to clients.
If this flag is not set, DHCPS reports the DNS servers as contained in the
SYSINFO portion of the configuration.

[More ...](/ndk/ConfigDoc.html#ti_ndk_DHCP_reportLocalDNSS)`,
                documentation: `
More information on the DHCP Server flags can be found in the
[NDK API Flag](NDK_API_Reference.html#dhcp-server-parameter-structure).
`
            }
        ]
    },
    /*
     * This next menu item has no effect, but it could be used in the future
     * to support users who do not use syscfg to write "NDK Interface" code.
     * Currently these users (most likely a tiny minority) will be forced to
     * use the "NDK Interface" module if they pull in DHCPS (or any other
     * service that depends on "NDK Interface") as it is pulled in by DHCPS.
     *
     * We could support these users by adding a new menu item that gives them
     * the option of manually specifying the interface id DHCPS should use.
     * This is the only info that DHCPS actually needs from the "NDK Interface"
     * module.
     */
    {
        name: "ifIdx",
        displayName: "Interface ID",
        hidden: true,
        default: 1,
        description: "The physical device index on which the DHCP server " +
            "shall be executed. Must be greater than zero."
    },
    {
        name: "ipAddrPoolBase",
        displayName: "IP Address Pool Base",
        textType: "ipv4_address",
        default: "192.168.1.2",
        readOnly: false,
        description: "The first IP address of the DHCP server address pool.",
        longDescription: `
The first IP address (in dotted decimal notation) of the address pool.

[More ...](/ndk/ConfigDoc.html#ti_ndk_DHCP_ipAddrPoolBase)`,
        documentation: `
More information on the DHCP Server Service can be found in the
[NDK API Flag](NDK_API_Reference.html#dhcp-server-service).
`
    },
    {
        name: "ipAddrPoolCount",
        displayName: "IP Address Pool Count",
        default: "253",
        readOnly: false,
        description: "The number of IP addresses in the DHCP server address pool.",
        longDescription: `
The number of addresses in the address pool.

[More ...](/ndk/ConfigDoc.html#ti_ndk_DHCP_ipAddrPoolCount)`,
        documentation: `
More information on the DHCP Server Service can be found in the
[NDK API Flag](NDK_API_Reference.html#dhcp-server-service).
`
    },
    {
        name: "servReportFxn",
        displayName: "Service Report Function",
        default: "",
        description: "DHCP service reporting function.",
        longDescription: `
Optional, user defined service report function to handle DHCP Server reports.
Note that multiple services (e.g. Telnet Server and DHCP Server) can use the
same service report function.

If set, this service report function must be provided by the application, and
the function signature must match this prototype:

    extern void userDHCPServerServReportFxn(uint32_t item, uint32_t status,
            uint32_t report, void *h);

[More ...](/ndk/ConfigDoc.html#ti_ndk_DHCP_servReportFxn)`,
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
            description: "The NDK Interface to run this DHCP Server from",
            collapsed: false,
            hidden: false
        }
    );

    return modules;
}

/*
 *  ======== tfxn ========
 *  Template helper functions
 */
let tfxn = {
    cisargs_mode: function(inst)
    {
        let flags = [];

        if (inst.reportLocalDomainName) flags.push("DHCPS_FLG_LOCALDOMAIN");
        if (inst.reportLocalDNSS) flags.push("DHCPS_FLG_LOCALDNS");

        if (flags.length == 0) {
            return ("0");
        }
        else {
            return (flags.join(" | "));
        }
    }
};

/*
 *  ======== base ========
 *  Module definition object
 */
let base = {
    displayName: "DHCP Server",
    description: "NDK DHCP Server configuration",
    defaultInstanceName: "CONFIG_DHCPS_",
    longDescription: longDescription,
    config: config_instance,
    validate: validate_instance,
    sharedModuleInstances: moduleInstances,
    moduleStatic: {
        modules: modules
    },
    templates: {
        "/ti/ndk/Config.c.xdt": "/ti/ndk/DHCP.Config.c.xdt"
    },
    tfxn: tfxn
};

/* export the module */
exports = base;
