/*
 * Copyright (c) 2019-2020 Texas Instruments Incorporated - http://www.ti.com
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
 *  ======== NIMU.syscfg.js ========
 *  NIMU configuration support
 */

"use strict";
/* global exports, system */

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
    let general = system.modules["/ti/ndk/General"].$static;

    if (inst.device == "Custom") {
        if (!inst.custom) {
            vo["custom"].errors.push("A .init function must be specified");
        }
        else if (!Common.isCName(inst.custom)) {
            vo["custom"].errors.push("Not a valid C identifier.");
        }
    }

    if (inst.dhcpcServReportFxn == "" &&
        inst.localIPAddrConfig == "Use DHCP to Obtain IP Address" &&
        general.enableExtDNS == true)
    {
        vo.logWarning("A service report function is needed here in order " +
                      "for the External DNS Server configuration made in the " +
                      "NDK Stack module to function. For more information " +
                      "view the help menu for this config.", inst,
                      "dhcpcServReportFxn");
    }

    if ((inst.dhcpcServReportFxn != "") &&
            (!Common.isCName(inst.dhcpcServReportFxn))) {
        vo["dhcpcServReportFxn"].errors.push("Not a valid C identifier.");
    }
}

function onChange_device(inst, ui)
{
    if(inst.device == "Custom") {
        ui.custom.hidden = false;
    }
    else {
        ui.custom.hidden = true;
    }
}

/*
 *  ======== onChange_localIPAddrConfig ========
 */
function onChange_localIPAddrConfig(inst, ui)
{
    switch (inst.localIPAddrConfig) {
        case "Use DHCP to Obtain IP Address":
            ui.staticIPAddr.hidden = true;
            ui.ipMask.hidden = true;
            ui.gatewayIpAddr.hidden = true;
            ui.domainName.hidden = true;
            ui.dhcpcServReportFxn.hidden = false;
            break;

        case "Enable Static IP Address":
            ui.staticIPAddr.hidden = false;
            ui.ipMask.hidden = false;
            ui.gatewayIpAddr.hidden = false;
            ui.domainName.hidden = false;
            ui.dhcpcServReportFxn.hidden = true;
            break;

        case "Do Not Configure an IP Address":
            ui.staticIPAddr.hidden = true;
            ui.ipMask.hidden = true;
            ui.gatewayIpAddr.hidden = true;
            ui.domainName.hidden = true;
            ui.dhcpcServReportFxn.hidden = true;
            break;
    }
}

/*
 *  ======== defval ========
 */
let defval = {
    staticIPAddr: "0.0.0.0",
    ipMask: "255.255.255.0",
    gatewayIpAddr: "0.0.0.0",
    domainName: "demo.net"
};

let ipOptions = [
    {
        name: "localIPAddrConfig",
        displayName: "Local IP Address Configuration",
        onChange: onChange_localIPAddrConfig,
        default: "Use DHCP to Obtain IP Address",
        options: [
            {
                name: "Use DHCP to Obtain IP Address"
            },
            {
                name: "Enable Static IP Address"
            },
            {
                name: "Do Not Configure an IP Address"
            }
        ],
        longDescription: `
The method to obtain an IP address.

[More ...](/ndk/ConfigDoc.html#ti_ndk_NIMU_localIPAddrConfig)`
    },
    {
        name: "staticIPAddr",
        displayName: "Static IP Address",
        textType: "ipv4_address",
        default: defval.staticIPAddr,
        hidden: true,
        description: "Enter a valid address for static IP configuration",
        longDescription: `
The static IPv4 address to be used.

[More ...](/ndk/ConfigDoc.html#ti_ndk_NIMU_staticIPAddr)`,
        documentation: `
The IP Mask must be set in conjunction with this setting.
This translates directly into the following runtime call to.
[CfgAddEntry()](html/group__ti__ndk__inc__nettools__inc____Cfg.html#ga72795e289e7a5e347b54b94f40667dc2).

    char *LocalIPAddr = YOUR_CONFIGURED_ADDRESS_VALUE;
    char *LocalIPMask = YOUR_CONFIGURED_MASK_VALUE;
    char *DomainName  = YOUR_CONFIGURED_DOMAIN_VALUE;

    /* setup manual IP address */
    memset(&NA, 0, sizeof(NA));
    NA.IPAddr = inet_addr(LocalIPAddr);
    NA.IPMask = inet_addr(LocalIPMask);
    strcpy(NA.Domain, DomainName);
    NA.NetType = 0;

    CfgAddEntry(hCfg, CFGTAG_IPNET, 1, 0,
            sizeof(CI_IPNET), (unsigned char *)&NA, 0);
`
    },
    {
        name: "ipMask",
        displayName: "IP Mask",
        textType: "ipv4_address",
        default: defval.ipMask,
        hidden: true,
        description: "Must be specified when using a static IP address",
        longDescription: `
Used for manual/static IP configuration.  If configuring a static IP,
this must be set to a valid ipMask value.

[More ...](/ndk/ConfigDoc.html#ti_ndk_NIMU_ipMask)`,
        documentation: `
The IP Mask must be set in conjunction with the static IP address setting.
This translates directly into the following runtime call to.
[CfgAddEntry()](html/group__ti__ndk__inc__nettools__inc____Cfg.html#ga72795e289e7a5e347b54b94f40667dc2).

    char *LocalIPAddr = YOUR_CONFIGURED_ADDRESS_VALUE;
    char *LocalIPMask = YOUR_CONFIGURED_MASK_VALUE;
    char *DomainName  = YOUR_CONFIGURED_DOMAIN_VALUE;

    /* setup manual IP address */
    memset(&NA, 0, sizeof(NA));
    NA.IPAddr = inet_addr(LocalIPAddr);
    NA.IPMask = inet_addr(LocalIPMask);
    strcpy(NA.Domain, DomainName);
    NA.NetType = 0;

    CfgAddEntry(hCfg, CFGTAG_IPNET, 1, 0,
            sizeof(CI_IPNET), (unsigned char *)&NA, 0);
`
    },
    {
        name: "gatewayIpAddr",
        displayName: "Gateway IP Address",
        textType: "ipv4_address",
        default: defval.gatewayIpAddr,
        hidden: true,
        description: "Must be specified when using a static IP address",
        longDescription: `
Used for manual/static IP configuration.  If configuring a static IP,
this must be set to the IP address of the gateway.

[More ...](/ndk/ConfigDoc.html#ti_ndk_NIMU_gatewayIpAddr)`,
        documentation: `
The gateway IP address must be set in conjunction with the static IP address setting.
This translates directly into the following runtime call to.
[CfgAddEntry()](html/group__ti__ndk__inc__nettools__inc____Cfg.html#ga72795e289e7a5e347b54b94f40667dc2).

    char *GatewayIP = YOUR_CONFIGURED_VALUE;

    memset(&RT, 0, sizeof(RT));
    RT.IPDestAddr = 0;
    RT.IPDestMask = 0;
    RT.IPGateAddr = inet_addr(GatewayIP);

    CfgAddEntry(hCfg, CFGTAG_ROUTE, 0, 0,
            sizeof(CI_ROUTE), (unsigned char *)&RT, 0);
`
    },
    {
        name: "domainName",
        displayName: "Domain Name",
        default: defval.domainName,
        hidden: true,
        description: "Must be specified when using a static IP address",
        longDescription: `
Used for manual/static IP configuration.  If configuring a static IP,
this must be set to the IP address of the gateway.

[More ...](/ndk/ConfigDoc.html#ti_ndk_NIMU_domainName)`,
        documentation: `
The domain name must be set in conjunction with the static IP address setting.
This translates directly into the following runtime call to.
[CfgAddEntry()](html/group__ti__ndk__inc__nettools__inc____Cfg.html#ga72795e289e7a5e347b54b94f40667dc2).

    char *LocalIPAddr = YOUR_CONFIGURED_ADDRESS_VALUE;
    char *LocalIPMask = YOUR_CONFIGURED_MASK_VALUE;
    char *DomainName  = YOUR_CONFIGURED_DOMAIN_VALUE;

    /* setup manual IP address */
    memset(&NA, 0, sizeof(NA));
    NA.IPAddr = inet_addr(LocalIPAddr);
    NA.IPMask = inet_addr(LocalIPMask);
    strcpy(NA.Domain, DomainName);
    NA.NetType = 0;

    CfgAddEntry(hCfg, CFGTAG_IPNET, 1, 0,
            sizeof(CI_IPNET), (unsigned char *)&NA, 0);
`
    },
    {
        name: "dhcpcServReportFxn",
        displayName: "Service Report Function used by DHCP",
        default: "",
        description: "DHCP client service reporting function",
        longDescription: `
Optional, user defined service report function to handle DHCP client
reports. Note that multiple services (e.g. DNS Server and DHCP Client)
can use the same service report function.  This function is needed if
configuring an external DNS server.

If set, this service report function must be provided by the application, and
the function signature must match this prototype:

    extern void userDHCPClientServReportFxn(uint32_t item, uint32_t status,
            uint32_t report, void *h);

[More ...](/ndk/ConfigDoc.html#ti_ndk_NIMU_dhcpcServReportFxn)`,
        documentation: `
Information on adding your own service report report function can be found in the
[NDK User's Guide](ug/ug_ndk/03_network-application-development.html#adding-status-report-services).

If you are configuring an external DNS server you will need to add the code
detailed in the following
[section](ug/ug_ndk/03_network-application-development.html#using-a-statically-defined-dns-server)
of the NDK User's Guide to the service report function.
`
    },
    {
        name: "netType",
        hidden: true,
        default: 0
    }
];

/*
 *  ======== config_module ========
 *  Define the config params of an instance
 */
let config_instance = [
    {
        name: "$name",
        displayName: "Name",
        description: "NIMU Name",
        hidden: false
    },
    {
        name: "device",
        displayName: "Device",
        default: "EMAC",
        onChange: onChange_device,
        options: [
            {
                name: "EMAC"
            },
            {
                name: "Custom"
            }
        ],
        description: 'Choose the desired interface',
        longDescription: `
Choose the desired interface:

**EMAC** - Automatically selects the NIMU EMAC driver for your device

**Custom** - Allows you to manually specify the \`.init\` function for a
NIMU driver.

[More ...](/ndk/ConfigDoc.html#ti_ndk_NIMU_device)
    `,
        documentation: `
This option is equivalent to creating a NIMU Device Table entry and setting
its .init member.

For more information on NIMU Device Table entries see the
[NDK User's Guide](ug/ug_ndk/stack.html#nimu-device-table)
    `
    },
    {
        name: "custom",
        displayName: "Custom NIMU .init function",
        default: "",
        hidden: true,
        description: "Name of the user made .init function to place in the NIMU Device Table",
        longDescription: `
Name of the user made .init function to place in the NIMU Device Table.
The .init function must be defined in the application code.

[More ...](/ndk/ConfigDoc.html#ti_ndk_NIMU_custom)
    `,
        documentation: `
This option is equivalent to setting the .init member of a NIMU Device
Table entry. .init must point to a valid init function in a NIMU driver.

For more information on NIMU drivers visit the
[NDK User's Guide](ug/ug_ndk/stack.html#network-interface-manager-unit-nimu)
    `
    },
    {
        displayName: "IP Options",
        config: ipOptions,
        collapsed: false
    }
];

/*
 *  ======== moduleInstances ========
 */
function moduleInstances(inst)
{
    let modules = new Array();
    let devId = system.deviceData.deviceId;

    if(inst.device == "EMAC")
    {
        switch (devId) {
            case "MSP432E":
                modules.push({
                    name: "emac",
                    displayName: "EMAC",
                    moduleName: "/ti/drivers/EMAC"
                });
                break;

            case "dragon":
                /* TODO */
                break;

            default:
                break;
        }
    }

    return (modules);
}

let longDescription = `
This module configures the
[NIMU Device Table](/ndk/ug/ug_ndk/stack.html#nimu-device-table) by enumerating all
your desired network interfaces into the table. This table is required for any
NDK application.

You can also configure the IP Options for each NDK Interface here.

If you have already defined \`NIMU_DEVICE_TABLE_ENTRY  NIMUDeviceTable[]\` in
your application code you do not need to use this module, however you will also
have to manually configure each table entry's IP Options.
`;

/*
 *  ======== base ========
 *  Module definition object
 */
let base = {
    displayName: "NDK Interface",
    description: "NDK NIMU Config",
    longDescription: longDescription,
    defaultInstanceName: "CONFIG_NIMU_",
    moduleInstances: moduleInstances,
    config: config_instance,
    validate: validate_instance,
    moduleStatic: {
        modules: modules
    },
    templates: {
        "/ti/ndk/Config.c.xdt": "/ti/ndk/NIMU.Config.c.xdt"
    }
};

/* export the module */
exports = base;
