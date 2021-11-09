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
 *  ======== Telnet.syscfg.js ========
 *  Telnet configuration support
 */

"use strict";
/* global exports, system */

//console.log("load: ti/ndk/Telnet");
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

    /* max connections */
    if(inst.maxCon < 1 || inst.maxCon > 24) {
        vo["maxCon"].errors.push("Must be between 1 and 24");
    }

    /* port */
    let port = inst.port;
    let instances = inst.$module.$instances;
    for(let i = 0; i < instances.length; i++) {
        if(instances[i].port == port && instances[i].$name != inst.$name) {
            vo["port"].errors.push(instances[i].$name + " is already using this port number");
        }
    }
    if(!inst.port) {
        vo["port"].errors.push("Field cannot be empty");
    }

    /* callback function */
    if(inst.callbackFxn == "") {
        vo["callbackFxn"].errors.push("Field cannot be empty");
    }
}


/*
 *  ======== longDescription ========
 *  Intro splash on GUI
 */
let longDescription =
    "Create and configure Telnet server instances. Multiple Telnet server " +
    "instances are supported, but each one must be configured to have " +
    "a unique port number.";

/*
 *  ======== config_instance ========
 *  Define the config params of the module (module-wide)
 */
let config_instance = [
    {
        name: "setTelnetIP",
        displayName: "Set Telnet IP Address:",
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
Use this option to choose which device IP address(es) the Telnet server will
bind itself too.

Any IP Addresses - Binds the service to any IP address on the target.
Equivalent to setting the IP Address to "0.0.0.0"

By NDK Interface - Allows you to choose an interface ID, and the Telnet Service
will bind to whatever IP that interface receives.

[More ...](/ndk/ConfigDoc.html#ti_ndk_Telnet_setTelnetIP)`,
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
        name: "maxCon",
        displayName: "Maximum Connections",
        default: 8,  /* TODO - should this be 4?  Code uses 4 as default */
        description: "The maximum number of connections for the telnet " +
            "server (1 - 24).",
        longDescription: `
Sets the maximum number of connections allowed by the telnet server. Must fall
between 1 and 24.

[More ...](/ndk/ConfigDoc.html#ti_ndk_Telnet_maxCon)`,
        documentation: `
More information on the Telnet Server Service can be found in the
[NDK API Guide](NDK_API_Reference.html#telnet-server-service).
`
    },
    {
        name: "port",
        displayName: "Port",
        default: 23,
        description: "The port number which this telnet server will accept " +
            "connections.",
        longDescription: `
The port number that telnet will accept connections. Typically this port will
be 23.

[More ...](/ndk/ConfigDoc.html#ti_ndk_Telnet_port)`,
        documentation: `
More information on the Telnet Server Service can be found in the
[NDK API Guide](NDK_API_Reference.html#telnet-server-service).
`
    },
    {
        name: "servReportFxn",
        displayName: "Service Report Function",
        default: "",
        description: "Telnet Server service reporting function.",
        longDescription: `
Optional, user defined service report function to handle Telnet Server reports.
Note that multiple services (e.g. Telnet Server and DHCP Server) can use the
same service report function.

If set, this service report function must be provided by the application, and
the function signature must match this prototype:

    extern void userTelnetServiceServReportFxn(uint32_t item, uint32_t status,
            uint32_t report, void *h);

[More ...](/ndk/ConfigDoc.html#ti_ndk_Telnet_servReportFxn)`,
        documentation: `
For more information on service report functions see the
[NDK User's Guide](ug/ug_ndk/03_network-application-development.html#adding-status-report-services).
`
    },
    {
        name: "callbackFxn",
        displayName: "Callback Function",
        hidden: true,
        default: "ConsoleOpen",
        description: "Telnet callback function.  This is the function which " +
           "contains the telnet server code.",
        longDescription: `
Service report function used by the Telnet server. The default value will use the
report function generated by SysConfig.

[More ...](/ndk/ConfigDoc.html#ti_ndk_Telnet_servReportFxn)`,
        documentation: `
Information on adding your own service report function can be found in the
[NDK User's Guide](ug/ug_ndk/03_network-application-development.html#adding-status-report-services).
`
    }
];

/*
 *  ======== tfxn ========
 *  Template helper functions
 */
let tfxn = {
    cisargs_mode: function(inst)
    {
        let flags = [];

        if (inst.IfIdxValid) flags.push("CIS_FLG_IFIDXVALID");
        if (inst.ResolveIP) flags.push("CIS_FLG_RESOLVEIP");
        if (inst.CallByIP) flags.push("CIS_FLG_CALLBYIP");
        if (inst.RestartIPTerm) flags.push("CIS_FLG_RESTARTIPTERM");

        if (flags.length == 0) {
            return ("0");
        }
        else {
            return (flags.join(" | "));
        }
    }
};

/*
 *  ======== moduleInstances ========
 */
function moduleInstances(inst)
{
    let modules = [];

    if(inst.setTelnetIP == "By NDK Interface")
    {
        modules.push(
            {
                name: "NIMU",
                displayName: "NDK Interface",
                moduleName: "/ti/ndk/NIMU",
                description: "The NDK Interface to run this Telnet instance from",
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
    displayName: "Telnet Server",
    description: "NDK Telnet configuration",
    defaultInstanceName: "CONFIG_TELNETS_",
    longDescription: longDescription,
    config: config_instance,
    validate: validate_instance,
    sharedModuleInstances: moduleInstances,
    moduleStatic: {
        modules: modules
    },
    templates: {
        "/ti/ndk/Config.c.xdt": "/ti/ndk/Telnet.Config.c.xdt"
    },
    tfxn: tfxn
};

/* export the module */
exports = base;
