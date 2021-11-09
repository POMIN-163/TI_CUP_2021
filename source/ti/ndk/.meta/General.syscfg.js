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
 *  ======== General.syscfg.js ========
 *  General configuration support
 */

"use strict";
/* global exports, system */

//console.log("load: ti/ndk/General");

let Common = system.getScript("/ti/utils/Common.js");

/*
 *  ======== getLibs ========
 */
function getLibs(mod)
{
    /* get device ID and toolchain to select appropriate libs */
    let devId = system.deviceData.deviceId;
    let rtos = system.modules["/ti/drivers/RTOS"];
    let ns = system.modules["/ti/net/SlNet"];
    let Telnet = system.modules["/ti/ndk/Telnet"];
    let NAT = system.modules["/ti/ndk/NAT"];
    let DHCP = system.modules["/ti/ndk/DHCP"];
    let DNS = system.modules["/ti/ndk/DNS"];
    let GenLibs = system.getScript("/ti/utils/build/GenLibs.syscfg.js");

    var libs = [];
    var deps = ["/ti/drivers"];
    var isa = "";
    var ipv6 = mod.$static.ipv6;
    var suffix = "";
    var netctrl_type = "_full";  /* assume the worst; pare down if possible */

    switch (devId) {
        case "MSP432E":
            isa = "m4f";
            break;

        default:
            isa = devId;
            break;
    }

    switch (GenLibs.getToolchainDir()) {
        case "ccs": suffix = "e" + isa; break;
        case "gcc": suffix = isa + "g"; break;
        case "iar": suffix = "r" + isa; break;
        case "ticlang": suffix = isa; break;
        default: /* hmm... better to be quiet? */ break;
    }

    if (ns != null) {
        deps.push("/ti/net");
        if (devId == "MSP432E") {
            libs.push("ti/ndk/slnetif/lib/slnetifndk_msp432e4.a" + suffix);
        }
        else {
            libs.push("ti/ndk/slnetif/lib/slnetifndk.a" + suffix);
        }

        let instances = ns.$instances;
        for (let i = 0; i < instances.length; i++) {
            if (instances[i].networkIfFxnList == "NDK") {
                if (instances[i].enableSecureSocks) {
                    if (devId == "MSP432E") {
                        libs.push(GenLibs.libPath("third_party/mbedtls/ti",
                                                  "mbedtls_msp432e4.a"));
                    }
                    else {
                        libs.push(GenLibs.libPath("third_party/mbedtls/ti",
                                                  "mbedtls.a"));
                    }
                }
            }
        }
    }

    if ((NAT == null) && (DHCP == null) && (DNS == null)) {
        /* we have a lighter netctrl library */
        if (Telnet == null) {
            netctrl_type = "_min";
        }
        else {
            netctrl_type = "";
        }
    }

    libs.push("ti/ndk/hal/timer_bios/lib/hal_timer.a" + suffix);
    libs.push("ti/ndk/hal/eth_stub/lib/hal_eth_stub.a" + suffix);
    libs.push("ti/ndk/tools/hdlc/lib/hdlc.a" + suffix);

    if (ipv6) {
        libs.push("ti/ndk/tools/console/lib/console_min.a" + suffix);
        libs.push("ti/ndk/netctrl/lib/netctrl" + netctrl_type + ".a" + suffix);
        libs.push("ti/ndk/nettools/lib/nettool.a" + suffix);
        libs.push("ti/ndk/stack/lib/stk6.a" + suffix);
    }
    else {
        libs.push("ti/ndk/tools/console/lib/console_min_ipv4.a" + suffix);
        libs.push("ti/ndk/netctrl/lib/netctrl" + netctrl_type + "_ipv4.a" +
                  suffix);
        libs.push("ti/ndk/nettools/lib/nettool_ipv4.a" + suffix);
        libs.push("ti/ndk/stack/lib/stk.a" + suffix);
    }

    libs.push("ti/ndk/hal/ser_stub/lib/hal_ser_stub.a" + suffix);
    libs.push("ti/ndk/hal/userled_stub/lib/hal_userled_stub.a" + suffix);

    /* NDK OSAL library */
    switch (rtos.$static.name) {
        case "TI-RTOS":
            libs.push("ti/ndk/os/lib/os.a" + suffix);
            break;

        case "FreeRTOS":
            libs.push("ti/ndk/os/lib/os_freertos.a" + suffix);
            break;

        case "<none>":
        case "NoRTOS":
        default:
            /* this should never happen.  Assert? */
            break;
    }

    /* create a GenLibs input argument */
    var linkOpts = {
        name: "/ti/ndk",
        //vers: "1.0.0.0",
        deps: deps,
        libs: libs
    };

    return (linkOpts);
}

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
        name: "rtos",
        moduleName: "/ti/drivers/RTOS"
    });

    return (modules);
}

/*
 *  ======== onChange_ndkThrdCodeGen ========
 */
function onChange_ndkThrdCodeGen(inst, ui)
{
    let ndkStackThrd = (inst.stackThreadUser == "");
    let ndkThrdCodeGen = (ndkStackThrd && inst.enableCodeGeneration);

    ui.netTaskSchedulerTaskPri.readOnly = !ndkThrdCodeGen;
    ui.ndkTickPeriod.readOnly = !ndkStackThrd;
    ui.netSchedulerOpMode.readOnly = !ndkThrdCodeGen;

    /* stackThreadPriLevel always used */
    ui.lowPriTaskPriLevel.readOnly = !ndkThrdCodeGen;
    ui.normPriTaskPriLevel.readOnly = !ndkThrdCodeGen;
    ui.highPriTaskPriLevel.readOnly = !ndkThrdCodeGen;
    ui.kernPriLevel.readOnly = !ndkThrdCodeGen;

    /* ndkThreadStkSize always used */
    ui.lowPriTaskDefStkSize.readOnly = !ndkThrdCodeGen;
    ui.normPriTaskDefStkSize.readOnly = !ndkThrdCodeGen;
    ui.highPriTaskDefStkSize.readOnly = !ndkThrdCodeGen;

    ui.numPBMFrames.readOnly = !inst.enableCodeGeneration;
    ui.pbmFrameBufSize.readOnly = !inst.enableCodeGeneration;
    ui.pbmDataSection.readOnly = !inst.enableCodeGeneration;

    ui.pageSize.readOnly = !inst.enableCodeGeneration;
    ui.numPages.readOnly = !inst.enableCodeGeneration;
    ui.bufDataSection.readOnly = !inst.enableCodeGeneration;

    ui.beginHook.readOnly = !ndkThrdCodeGen;
    ui.initHook.readOnly = !ndkThrdCodeGen;
    ui.deleteHook.readOnly = !ndkThrdCodeGen;
    ui.rebootHook.readOnly = !ndkThrdCodeGen;

    ui.networkOpenHook.readOnly = !ndkThrdCodeGen;
    ui.networkCloseHook.readOnly = !ndkThrdCodeGen;
    ui.networkIPAddrHook.readOnly = !ndkThrdCodeGen;

    ui.hostname.readOnly = !inst.enableCodeGeneration;
}


/*
 *  ======== onChange_enableExtDNS ========
 */
function onChange_enableExtDNS(inst, ui)
{
    ui.externDNSServIPAddr.readOnly = !inst.enableExtDNS;
    if (!inst.enableExtDNS) {
        inst.externDNSServIPAddr = defval.externDNSServIPAddr;
    }
}

/*
 *  ======== validate ========
 *  Validate given instance and report conflicts
 *
 *  This function is not allowed to modify the instance state.
 */
function validate(inst, vo, getRef)
{
    let rtos = system.modules["/ti/drivers/RTOS"];

    if(!system.modules["/ti/ndk/NIMU"])
    {
        vo.logWarning("You have not added a NDK Interface instance, so you \
                      will be responsible for configuring the NIMUDeviceTable.",
                      system.modules["/ti/ndk/General"].$static);
    }

    switch (rtos.$static.name) {
        case "<none>":
        case "NoRTOS":
            vo.logError("Must select an RTOS (required by NDK)",
                        rtos.$static, "name");
            break;
    }

    /* scheduler's polling mode requires low-priority */
    if ((inst.netSchedulerOpMode == "NC_OPMODE_POLLING") &&
            (inst.netTaskSchedulerTaskPri != "NC_PRIORITY_LOW"))
    {
        vo.logError('When the Network Task Scheduler Operating Mode is set' +
                    ' to "Polling", the Task Priority must be "Low Priority"',
                    system.modules["/ti/ndk/General"].$static,
                    "netTaskSchedulerTaskPri");
    }

    /* Validate C symbol names */
    const NDKCParams = ["beginHook", "initHook", "rebootHook", "deleteHook",
        "networkOpenHook", "networkCloseHook", "networkIPAddrHook"];
    for (let i = 0; i < NDKCParams.length; i++) {
        if (!Common.isCName(inst[NDKCParams[i]])) {
            vo[NDKCParams[i]].errors.push("Not a valid C identifier.");
        }
    }

    /*
     * Handle the old IP options. These options have moved to the
     * NDK Interfaces/NIMU module, but are still present in this module to
     * prevent any syscfg errors.
     *
     * Users won't be able to see these settings in the GUI, but could
     * potentially write to the options in their *.syscfg scripts. Here we can
     * tell users these options have moved in case they do just that.
     */
    function logIPInfoMessage(name)
    {
        vo.logInfo("You are writing to this setting, but it has moved to " +
                    "the NDK Interface module. Edit your *.syscfg file in a " +
                    "text editor and delete the line setting " + name +
                    " to suppress this remark.", inst, name);
    }

    if(inst.localIPAddrConfig != defval.localIPAddrConfig)
    {
        logIPInfoMessage("localIPAddrConfig");
    }
    if(inst.staticIPAddr != defval.staticIPAddr)
    {
        logIPInfoMessage("staticIPAddr");
    }
    if(inst.ipMask != defval.ipMask)
    {
        logIPInfoMessage("ipMask");
    }
    if(inst.gatewayIpAddr != defval.gatewayIpAddr)
    {
        logIPInfoMessage("gatewayIpAddr");
    }
    if(inst.domainName != defval.domainName)
    {
        logIPInfoMessage("domainName");
    }
    if(inst.dhcpcServReportFxn != defval.dhcpcServReportFxn)
    {
        logIPInfoMessage("dhcpcServReportFxn");
    }
    if(inst.netType != defval.netType)
    {
        logIPInfoMessage("netType");
    }

    if(inst.enableExtDNS == true && inst.externDNSServIPAddr == "0.0.0.0")
    {
        vo.logError("Enter a valid DNS server address.",
                    inst, "externDNSServIPAddr");
    }
}

/*
 *  ======== defval ========
 */
let defval = {
    ifIdxValid: true,
    resolveIP: false,
    callUsingIP: false,
    restartServIPTerm: false,
    localIPAddrConfig: "Use DHCP to Obtain IP Address",
    staticIPAddr: "0.0.0.0",
    ipMask: "255.255.255.0",
    gatewayIpAddr: "0.0.0.0",
    domainName: "demo.net",
    hostname: "tisoc",
    dhcpcServReportFxn: "",
    netType: 0,

    externDNSServIPAddr: "0.0.0.0",

    netTaskSchedulerTaskPri:    "NC_PRIORITY_LOW",
    ndkTickPeriod:      100,
    netSchedulerOpMode: "NC_OPMODE_INTERRUPT",
    stackThreadPriLevel:       5, /* same as normPriTaskPriLevel */
    ndkThreadStkSize: 8192, /* same as ndkTaskStaskSize */

    /* network task priority levels */
    lowPriTaskPriLevel:  3,
    normPriTaskPriLevel: 5,
    highPriTaskPriLevel: 7,
    kernPriLevel: 9,

    /* network configuration defaults */
    lowPriTaskDefStkSize:  3072,
    normPriTaskDefStkSize: 4096,
    highPriTaskDefStkSize: 5120,

    /* TCP layer configuration defaults */
    tcpTxBufSize: 8192,
    tcpRxBufSize:  8192,
    tcpRxBufLimit: 8192,

    /* UDP layer configuration defaults */
    udpRxBufSize: 8192
};

/*
 *  ======== longDescription ========
 *  Intro splash on GUI
 */
let longDescription =
    "High level NDK stack configuration and settings. Use this module " +
    "to configure stack size, task priority, operating mode of the NDK " +
    "scheduler, and user hook functions.";

/*
 * Note these options have been moved to the NDK Interface/NIMU module. Copies
 * of them remain here to prevent syscfg from crashing for users who set
 * these setting in their *.syscfg file.
 */
let ipOptions = [
    {
        name: "localIPAddrConfig",
        displayName: "Local IP Address Configuration",
        default: defval.localIPAddrConfig,
        hidden: true,
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
        ]
    },
    {
        name: "staticIPAddr",
        displayName: "Static IP Address",
        textType: "ipv4_address",
        default: defval.staticIPAddr,
        hidden: true,
        description: "This setting has been moved to the NDK Interface module"
    },
    {
        name: "ipMask",
        displayName: "IP Mask",
        textType: "ipv4_address",
        default: defval.ipMask,
        hidden: true,
        description: "This setting has been moved to the NDK Interface module"
    },
    {
        name: "gatewayIpAddr",
        displayName: "Gateway IP Address",
        textType: "ipv4_address",
        default: defval.gatewayIpAddr,
        hidden: true,
        description: "This setting has been moved to the NDK Interface module"
    },
    {
        name: "domainName",
        displayName: "Domain Name",
        default: defval.domainName,
        hidden: true,
        description: "This setting has been moved to the NDK Interface module"
    },
    {
        name: "dhcpcServReportFxn",
        displayName: "Service Report Function Used by DHCP",
        default: "",
        hidden: true,
        description: "This setting has been moved to the NDK Interface module"
    },
    {
        name: "netType",
        hidden: true,
        default: defval.netType
    }
];

let layer4Options = [
    {
        name: "tcpTxBufSize",
        displayName: "TCP Transmit Buffer Size",
        default: defval.tcpTxBufSize,
        description: "Default TCP send buffer size (bytes)",
        longDescription: `
Sets the default size (in bytes) of the TCP send buffer

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_tcpTxBufSize)`,
        documentation: `
This translates directly into the following runtime call to
[CfgAddEntry()](html/group__ti__ndk__inc__nettools__inc____Cfg.html#ga72795e289e7a5e347b54b94f40667dc2).

    uint32_t transmitBufSize = YOUR_CONFIGURED_VALUE;

    CfgAddEntry(hCfg, CFGTAG_IP, CFGITEM_IP_SOCKTCPTXBUF, CFG_ADDMODE_UNIQUE,
            sizeof(uint32_t), (unsigned char *)&transmitBufSize, NULL);
`
    },
    {
        name: "tcpRxBufSize",
        displayName: "TCP Receive Buffer Size (Copy Mode)",
        default: defval.tcpRxBufSize,
        description: "Default TCP receive size (bytes)",
        longDescription: `
Sets the default size (in bytes) of the TCP receive buffer

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_tcpRxBufSize)`,
        documentation: `
This translates directly into the following runtime call to
[CfgAddEntry()](html/group__ti__ndk__inc__nettools__inc____Cfg.html#ga72795e289e7a5e347b54b94f40667dc2).

    uint32_t receiveBufSize = YOUR_CONFIGURED_VALUE;

    CfgAddEntry(hCfg, CFGTAG_IP, CFGITEM_IP_SOCKTCPRXBUF, CFG_ADDMODE_UNIQUE,
            sizeof(uint32_t), (unsigned char *)&receiveBufSize, NULL);
`
    },
    {
        name: "tcpRxBufLimit",
        displayName: "TCP Receive Size Maximum (Non-Copy Mode)",
        default: defval.tcpRxBufLimit,
        description: "Default maximum TCP receive size (bytes)",
        longDescription: `
Sets the default size (in bytes) of the maximum TCP receive buffer

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_tcpRxBufLimit)`,
        documentation: `
This translates directly into the following runtime call to
[CfgAddEntry()](html/group__ti__ndk__inc__nettools__inc____Cfg.html#ga72795e289e7a5e347b54b94f40667dc2).

    uint32_t receiveBufLimit = YOUR_CONFIGURED_VALUE;

    CfgAddEntry(hCfg, CFGTAG_IP, CFGITEM_IP_SOCKTCPRXLIMIT, CFG_ADDMODE_UNIQUE,
            sizeof(uint32_t), (unsigned char *)&receiveBufLimit, NULL);
`
    },
    {
        name: "udpRxBufSize",
        displayName: "UDP Receive Buffer Size",
        default: defval.udpRxBufSize,
        longDescription: `
Sets the default size (in bytes) of the maximum UDP receive buffer

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_udpRxBufSize)`,
        documentation: `
This translates directly into the following runtime call to
[CfgAddEntry()](html/group__ti__ndk__inc__nettools__inc____Cfg.html#ga72795e289e7a5e347b54b94f40667dc2).

    uint32_t udpRxBufSize = YOUR_CONFIGURED_VALUE;

    CfgAddEntry(hCfg, CFGTAG_IP, CFGITEM_IP_SOCKUDPRXLIMIT, CFG_ADDMODE_UNIQUE,
            sizeof(uint32_t), (unsigned char *)&udpRxBufSize, NULL);
`
    }
];

let externalDSNOptions = [
    {
        name: "enableExtDNS",
        displayName: "Enable External DNS Server",
        default: false,
        onChange: onChange_enableExtDNS,
        description: "Allow manually configuring the external DNS server",
        longDescription: `
Enable this option if you would like to manually configure an external DNS
server rather than receiving one from the DHCP client.

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_enableExtDNS)
    `,
        documentation: `
Selecting this option will unlock the
[External DNS Server IP Address](#ti_ndk_General_enableExtDNS) option.
    `
    },
    {
        name: "externDNSServIPAddr",
        displayName: "External DNS Server IP Address",
        textType: "ipv4_address",
        default: "0.0.0.0",
        readOnly: true,
        description: "Used to specify an external DNS Server.",
        longDescription: `
Use this to specify which external DNS server to use.

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_externDNSServIPAddr)`,
        documentation: `
This generates the code detailed in the Statically Defined DNS Server section
of the
[NDK User's Guide](ug/ug_ndk/03_network-application-development.html#using-a-statically-defined-dns-server).
Note that you are responsible for writing the code in the DHCP Client's Service
Report Function.
`
    }
];

let netSchedulerTaskOptions = [
    {
        name: "netTaskSchedulerTaskPri",
        displayName: "Network Task Scheduler Task Priority",
        default: defval.netTaskSchedulerTaskPri,
        options: [
            {
                name: "NC_PRIORITY_LOW",
                displayName: "Low Priority"
            },
            {
                name: "NC_PRIORITY_HIGH",
                displayName: "High Priority"
            }
        ],
        description: "The priority level at which the NDK net scheduler " +
            "task runs.",
        longDescription: `
Set the NDK scheduler Task's priority relative to other networking Tasks in
the system.

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_netTaskSchedulerTaskPri)`,
        documentation: `
This translates directly into the following runtime call to
[NC_SystemOpen()](html/group__ti__ndk__inc__netctrl__NC.html#ga07a431ba384014587a24666c8532f505).

    int priority = YOUR_CONFIGURED_VALUE;

    rc = NC_SystemOpen(priority, NC_OPMODE_INTERRUPT );
`
    },
    {
        name: "ndkTickPeriod",
        displayName: "NDK Tick Period",
        default: defval.ndkTickPeriod,
        description: "Tick period in clock ticks for the NDK heartbeat.",
        longDescription: `
Lets you adjust the NDK heartbeat rate

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_ndkTickPeriod)`,
        documentation: `
The default is 100ms, and is created with POSIX apis, so it will function the
same in both TIRTOS and FreeRTOS.
`
    },
    {
        name: "netSchedulerOpMode",
        displayName: "Network Scheduler Operating Mode",
        default: defval.netSchedulerOpMode,
        options: [
            {
                name: "NC_OPMODE_POLLING",
                displayName: "Polling"
            },
            {
                name: "NC_OPMODE_INTERRUPT",
                displayName: "Interrupt"
            }
        ],
        description: "The manner at which the NDK net scheduler task runs.",
        longDescription: `
Set to either Polling Mode (NC_OPMODE_POLLING) or Interrupt Mode
(NC_OPMODE_INTERRUPT), and determines when the scheduler attempts to execute.
Interrupt mode is used in the vast majority of applications. Note that
polling mode attempts to run continuously, so when polling is used, a low
priority must be used.

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_ndkTickPeriod)`,
        documentation: `
This translates directly into the following runtime call to
[NC_SystemOpen()](html/group__ti__ndk__inc__netctrl__NC.html#ga07a431ba384014587a24666c8532f505).

    int mode = YOUR_CONFIGURED_VALUE;

    rc = NC_SystemOpen(NC_PRIORITY_HIGH, mode);
`
    }
];

let netTask = [
    {
        name: "stackThreadPriLevel",
        displayName: "Stack Thread Priority Level",
        default: defval.stackThreadPriLevel,
        longDescription: `
Sets the priority of the generated NDK task ndkStackThread()

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_stackThreadPriLevel)`,
        documentation: `
This function will ultimately startup the stack
`
    },
    {
        name: "ndkThreadStkSize",
        displayName: "Stack Thread Stack Size",
        default: defval.ndkThreadStkSize,
        description: "Stack size, in bytes, of the generated NDK task ndkStackThread()",
        longDescription: `
Sets the stack size of the generated NDK task ndkStackThread()

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_ndkThreadStkSize)`,
        documentation: `
This function will ultimately startup the stack
`
    },
    {
        name: "lowPriTaskPriLevel",
        displayName: "Low Priority Tasks Priority Level",
        default: defval.lowPriTaskPriLevel,
        description: "Sets the priority value for low priority NDK tasks.",
        longDescription: `
Allows you to configure the priority level for network tasks

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_lowPriTaskPriLevel)`,
        documentation: `
Set the priority for the macro OS_TASKPRILOW that is used in the call to
[TaskCreate()](NDK_API_Reference.html#taskcreate-create-a-task-thread).

For more information on NDK task priorities see the
[NDK User's Guide](ug/ug_ndk/03_network-application-development.html#priority-levels-for-network-tasks).
`
    },
    {
        name: "lowPriTaskDefStkSize",
        displayName: "Low Priority Tasks Default Stack Size",
        default: defval.lowPriTaskDefStkSize,
        description: "Set the default stack size, in bytes, for low priority NDK tasks.",
        longDescription: `
Allows you to configure the stack size for network tasks

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_lowPriTaskDefStkSize)`,
        documentation: `
Set the priority for the macro OS_TASKSTKLOW that is used in the call to
[TaskCreate()](NDK_API_Reference.html#taskcreate-create-a-task-thread).
`
    },
    {
        name: "normPriTaskPriLevel",
        displayName: "Normal Priority Tasks Priority Level",
        default: defval.normPriTaskPriLevel,
        description: "Sets the priority value for normal priority NDK tasks.",
        longDescription: `
Allows you to configure the priority level for network tasks

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_normPriTaskPriLevel)`,
        documentation: `
Set the priority for the macro OS_TASKPRINORM that is used in the call to
[TaskCreate()](NDK_API_Reference.html#taskcreate-create-a-task-thread).

For more information on NDK task priorities see the
[NDK User's Guide](ug/ug_ndk/03_network-application-development.html#priority-levels-for-network-tasks).
`
    },
    {
        name: "normPriTaskDefStkSize",
        displayName: "Normal Priority Tasks Default Stack Size",
        default: defval.normPriTaskDefStkSize,
        description: "Set the default stack size, in bytes, for normal priority NDK tasks.",
        longDescription: `
Allows you to configure the stack size for network tasks

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_normPriTaskDefStkSize)`,
        documentation: `
Set the priority for the macro OS_TASKSTKNORM that is used in the call to
[TaskCreate()](NDK_API_Reference.html#taskcreate-create-a-task-thread).
`
    },
    {
        name: "highPriTaskPriLevel",
        displayName: "High Priority Tasks Priority Level",
        default: defval.highPriTaskPriLevel,
        description: "Sets the priority value for high priority NDK tasks.",
        longDescription: `
Allows you to configure the priority level for network tasks

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_highPriTaskPriLevel)`,
        documentation: `
Set the priority for the macro OS_TASKPRIHIGH that is used in the call to
[TaskCreate()](NDK_API_Reference.html#taskcreate-create-a-task-thread).

For more information on NDK task priorities see the
[NDK User's Guide](ug/ug_ndk/03_network-application-development.html#priority-levels-for-network-tasks).
`
    },
    {
        name: "highPriTaskDefStkSize",
        displayName: "High Priority Tasks Default Stack Size",
        default: defval.highPriTaskDefStkSize,
        description: "Set the default stack size, in bytes, for high priority NDK tasks.",
        longDescription: `
Allows you to configure the stack size for network tasks

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_highPriTaskDefStkSize)`,
        documentation: `
Set the priority for the macro OS_TASKSTKHIGH that is used in the call to
[TaskCreate()](NDK_API_Reference.html#taskcreate-create-a-task-thread).
`
    },
    {
        name: "kernPriLevel",
        displayName: "Kernel Priority Level",
        default: defval.kernPriLevel,
        description: "Sets the value for NDK kernel priority",
        longDescription: `
Allows you to configure the NDK kernel level priority.

Note that this only applies when the stack is configured to use priority
exclusion (not semaphores).

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_kernPriLevel)`,
        documentation: `
Tasks enter kernel mode during an llEnter()/llExit() block, during
which their priority will be raised to the level specified in this
configuration parameter.

For more information on llEnter()/llExit(). See the
[NDK User's Guide](ug/ug_ndk/05_os-adaptation-layer.html#choosing-the-llenter-llexit-exclusion-method).
`
    }
];

let pbmBuffers = [
    {
        name: "numPBMFrames",
        displayName: "Number of PBM Frames",
        default: 10,
        description: "Packet Buffer Manager (PBM) number of frames",
        longDescription: `
Sets the number of frames in the Packet Buffer Manager

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_numPBMFrames)`,
        documentation: `
For more information on the Packet Buffer Manager see the
[NDK User's Guide](ug/ug_ndk/stack.html#packet-buffer-manager-pbm-c).
`
    },
    {
        name: "pbmFrameBufSize",
        displayName: "PBM Frame Buffer Size",
        default: 1536,
        description: "Packet Buffer Manager (PBM) frame buffer size, in bytes.",
        longDescription: `
Sets the frame buffer size in the Packet Buffer Manager

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_pbmFrameBufSize)`,
        documentation: `
For more information on the Packet Buffer Manager see the
[NDK User's Guide](ug/ug_ndk/stack.html#packet-buffer-manager-pbm-c).
`
    },
    {
        name: "pbmDataSection",
        displayName: "PBM Data Section",
        default: ".bss:NDK_PACKETMEM",
        description: "Packet Buffer Manager (PBM) buffer data section",
        longDescription: `
Defines the memory section used to place the the PBM frame buffer array.

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_pbmDataSection)`,
        documentation: `
For more information on the Packet Buffer Manager see the
[NDK User's Guide](ug/ug_ndk/stack.html#packet-buffer-manager-pbm-c).
`
    }
];

let memManagerBuffs = [
    {
        name: "pageSize",
        displayName: "Page Size",
        default: 3072,
        description: "Memory Manager page size in bytes",
        longDescription: `
Sets the page size for the NDK's memory allocation system.

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_pageSize)`,
        documentation: `
For more information on the memory allocation system see the
[NDK User's Guide](ug/ug_ndk/05_os-adaptation-layer.html#memory-allocation-system-mem-c).
`
    },
    {
        name: "numPages",
        displayName: "Number of Pages",
        default: 6,
        description: "Memory Manager page count",
        longDescription: `
Sets the number of pages for the NDK's memory allocation system.

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_numPages)`,
        documentation: `
For more information on the memory allocation system see the
[NDK User's Guide](ug/ug_ndk/05_os-adaptation-layer.html#memory-allocation-system-mem-c).
`
    },
    {
        name: "bufDataSection",
        displayName: "Buffer Data Section",
        default: ".bss:NDK_MMBUFFER",
        description: "Memory Manager buffer data section",
        longDescription: `
Defines the memory section used to place the Memory Manager's allocation pool
buffer

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_pageSize)`,
        documentation: `
For more information on the memory allocation system see the
[NDK User's Guide](ug/ug_ndk/05_os-adaptation-layer.html#memory-allocation-system-mem-c).
`
    }
];

let userCallbacks = [
    {
        name: "beginHook",
        displayName: "Network Stack Begin Callback",
        default: "",
        description: "Callback run when the NDK stack thread begins",
        longDescription: `
Optional user defined callback function, called from the NDK stack
thread, at the very beginning of the stack thread's execution.  This
callback is called before the call to NC_SystemOpen(). It will not be
passed any arguments.

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_beginHook)`,
        documentation: `
For more information on stack thread callbacks see the
[NDK User's Guide](ug/ug_ndk/03_network-application-development.html#global-hook-configuration).
`
    },
    {
        name: "initHook",
        displayName: "Network Stack Initialization Callback",
        default: "",
        description: "Callback run when the NDK stack initializes",
        longDescription: `
Optional user defined callback function, called from the NDK stack
thread, as the stack thread is initializing.  This callback is called
immediately after the stack thread has created a new configuration,
CfgNew(), and will be passed the handle to that configuration

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_initHook)`,
        documentation: `
For more information on stack thread callbacks see the
[NDK User's Guide](ug/ug_ndk/03_network-application-development.html#global-hook-configuration).
`
    },
    {
        name: "rebootHook",
        displayName: "Network Stack Reboot Callback",
        default: "",
        description: "Callback run when the NDK stack is about to reboot",
        longDescription: `
Optional user defined callback function, called from the NDK stack
thread, when the stack is about to reboot. This function will run
immediately after the return from NC_NetStart() and within the while()
loop which contains the NC_NetStart() call. It will be passed a handle
to the configuration as well as the valued returned from NC_NetStart.

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_rebootHook)`,
        documentation: `
For more information on stack thread callbacks see the
[NDK User's Guide](ug/ug_ndk/03_network-application-development.html#global-hook-configuration).
`
    },
    {
        name: "deleteHook",
        displayName: "Network Stack Delete Callback",
        default: "",
        description: "Callback run when the stack is about to be deleted",
        longDescription: `
Optional user defined callback function, called from the NDK stack
thread, when the stack is about to be destroyed. This function will
run immediately after exiting from the while() loop which contains the
call to NC_NetStart(), but before the subsequent calls to
CfgFree(hCfg) and NC_SystemClose(). It will be passed a handle to the
configuration as well as the valued returned from NC_NetStart.

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_deleteHook)`,
        documentation: `
For more information on stack thread hooks see the
[NDK User's Guide](ug/ug_ndk/03_network-application-development.html#global-hook-configuration).
`
    },
    {
        name: "networkOpenHook",
        displayName: "Network Start Callback",
        default: "",
        description: "Callback run when the network starts",
        longDescription: `
Optional "Network Start" callback, called when the stack is ready to
begin the creation of any application supplied network tasks.

Note that this callback is called during the early stages of the stack
startup, and must return in order for the stack to resume operations. It is
not passed any arguments.

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_networkOpenHook)`,
        documentation: `
This callback is internally passed, as the NetStartCb argument, to
NC_NetStart() in the config-generated stack thread.  For more information,
see the
[NDK API Reference on NC_NetStart()](../ndk/html/group__ti__ndk__inc__netctrl__NC.html#gaa3e76836e265a4dfc7e7f58b9c9c5d85).
`
    },
    {
        name: "networkCloseHook",
        displayName: "Network Stop Callback",
        default: "",
        description: "Callback run when the network stops",
        longDescription: `
Optional "Network Stop" callback, called when the stack is about to shut down.
It is not passed any arguments.

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_networkCloseHook)`,
        documentation: `
This callback is internally passed, as the NetStopCb argument, to
NC_NetStart() in the config-generated stack thread.  For more information,
see the
[NDK API Reference on NC_NetStart()](../ndk/html/group__ti__ndk__inc__netctrl__NC.html#gaa3e76836e265a4dfc7e7f58b9c9c5d85).
`
    },
    {
        name: "networkIPAddrHook",
        displayName: "Network IP Address Callback",
        default: "",
        description: "Callback run for network IP address changes",
        longDescription: `
Optional "IP Address" callback, called when an IP address is added to,
or removed from, the system. It is passed IPAddr, IfIdx, fAdd arguments.

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_networkIPAddrHook)`,
        documentation: `
This callback is internally passed, as the NetIPCb argument, to
NC_NetStart() in the config-generated stack thread.  For more information,
see the
[NDK API Reference on NC_NetStart()](../ndk/html/group__ti__ndk__inc__netctrl__NC.html#gaa3e76836e265a4dfc7e7f58b9c9c5d85).
`
    }
];

/*
 *  ======== config ========
 *  Define the config params of the module instance
 */
let config = [
    {
        name: "ipv6",
        displayName: "Enable IPv6 support",
        hidden: true,
        default: false,
        description: "When selected, the IPv6 version of the NDK libraries " +
            "will be used, otherwise the IPv4 versions will be used."
    },
    {
        name: "hostname",
        displayName: "Hostname",
        default: "tisoc",
        longDescription: `
Add our device hostname to configuration (to be claimed in all connected domains)

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_hostname)`,
        documentation: `
This translates directly into the following runtime call to
[CfgAddEntry()](html/group__ti__ndk__inc__nettools__inc____Cfg.html#ga72795e289e7a5e347b54b94f40667dc2).

    unsigned char *hostname = YOUR_CONFIGURED_VALUE;

    CfgAddEntry(hCfg, CFGTAG_SYSINFO, CFGITEM_DHCP_HOSTNAME, 0,
                 strlen(hostname), hostname, 0);
`
    },
    {
        name: "maxNumSocks",
        displayName: "Maximum Number of Sockets",
        default: 10,
        description: "Determine the size of the socket file descriptor table"
    },
    {
        displayName: "Legacy IP Options",
        config: ipOptions
    },
    {
        displayName: "TCP & UDP Buffers",
        config: layer4Options
    },
    {
        displayName: "User Callbacks",
        config: userCallbacks
    },
    {
        displayName: "PBM Buffers",
        config: pbmBuffers
    },
    {
        displayName: "Memory Manager Buffers",
        config: memManagerBuffs
    },
    {
        displayName: "External DNS Server",
        config: externalDSNOptions
    },
    {
        displayName: "Network Scheduler Task",
        config: netSchedulerTaskOptions
    },
    {
        displayName: "NDK Created Threads",
        config: netTask
    },
    {
        name: "stackThreadUser",
        displayName: "User NDK thread function",
        hidden: true,
        default: "",
        onChange: onChange_ndkThrdCodeGen,
        description: "Example: User_stackThreadFxn",
        longDescription: `
User defined stack function that will run instead of the generated
ndkStackThread.

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_networkIPAddrHook)`,
        documentation: `
Only use your own stack thread if you do not want any of the generated SysConfig
content involved with starting up the stack.

If set, the user is responsible for defining the NDK stack thread, which has
no return value and has two parameters of type uintptr_t.

For example (C code):

  void MYMODULE_stackThreadUser(uintptr_t arg0, uintptr_t arg1);

The user is also responsible for creating the RTOS Clock
instance for the NDK 100ms heartbeat, calling appropriate NC_* APIs,
and adding the appropriate C run time configuration code that matches
the settings of the BIOS config file in the function. (e.g. if
configuring the Ip module, the stack thread must call NC_SystemOpen(),
'ti_ndk_config_ip_init(hCfg)', etc.).

For more information on the requirements for a user stack thread see the
[NDK User's Guide](ug/ug_ndk/03_network-application-development.html#constructing-a-configuration-for-a-static-ip-and-gateway)
`
    },
    {
        name: "enableCodeGeneration",
        displayName: "Enable Code Generation",
        hidden: true,
        default: true,
        onChange: onChange_ndkThrdCodeGen,
        description: "Generate NDK stack thread and C configuration code",
        longDescription: `
Only disable this if you know what you are doing. Disableing will prevent any
of the NDK stack thread from generating.

[More ...](/ndk/ConfigDoc.html#ti_ndk_General_networkIPAddrHook)`,
        documentation: `
This setting is similar to the "User NDK thread function" setting, except it
will not even generate the code to start a stack thread.
`
    }
];

/*
 *  ======== base ========
 *  Module definition object
 */
let base = {
    displayName: "NDK Stack",
    description: "General network configuration",
    longDescription: longDescription,
    moduleStatic: {
        modules: modules,
        config: config,
        validate: validate
    },
    templates: {
        /* contribute NDK libraries to linker command file */
        "/ti/utils/build/GenLibs.cmd.xdt"   :
            {modName: "/ti/ndk/General", getLibs: getLibs},

        /* trigger generation of ti_ndk_config.c */
        "/ti/ndk/Config.c.xdt": "/ti/ndk/General.Config.c.xdt",
        "/ti/utils/rov/syscfg_c.rov.xs.xdt":
            "/ti/ndk/ndk.rov.js"
    },
    defval: defval
};

/* export the module */
exports = base;
