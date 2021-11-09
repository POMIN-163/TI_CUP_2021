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
 *
 */

/*
 *  ======== Board.syscfg.js ========
 */

/* Module used to invoke Board.c and Board.h templates */

"use strict";

/* get ti/drivers common utility functions */
let Common = system.getScript("/ti/drivers/Common.js");

/*
 *  ======== getLibs ========
 */
function getLibs(mod)
{
    let FAMILY2LIBS = {
        CC26X2: {
            drivers: ["ti/drivers/lib/drivers_cc26x2"],
            dpl: {
                tirtos: ["ti/dpl/lib/dpl_cc26x2"],
                nortos: ["lib/nortos_cc26x2"],
                freertos: [""]
            }
        },
        CC13X2: {
            drivers: ["ti/drivers/lib/drivers_cc13x2"],
            dpl: {
                tirtos: ["ti/dpl/lib/dpl_cc13x2"],
                nortos: ["lib/nortos_cc13x2"],
                freertos: [""]
            }
        },
        CC26X0: {
            drivers: ["ti/drivers/lib/drivers_cc26x0"],
            dpl: {
                tirtos: ["ti/dpl/lib/dpl_cc26x0"],
                nortos: ["lib/nortos_cc26x0"],
                freertos: [""]
            }
        },
        CC26X0R2: {
            drivers: ["ti/drivers/lib/drivers_cc26x0r2"],
            dpl: {
                tirtos: ["ti/dpl/lib/dpl_cc26x0r2"],
                nortos: ["lib/nortos_cc26x0r2"],
                freertos: [""]
            }
        },
        CC13X0: {
            drivers: ["ti/drivers/lib/drivers_cc13x0"],
            dpl: {
                tirtos: ["ti/dpl/lib/dpl_cc13x0"],
                nortos: ["lib/nortos_cc13x0"],
                freertos: [""]
            }
        },
        CC32XX: {
            drivers: ["ti/drivers/lib/drivers_cc32xx"],
            dpl: {
                tirtos: ["ti/dpl/lib/dpl_cc32xx"],
                nortos: ["lib/nortos_cc32xx"],
                freertos: [""]
            }
        },
        MSP432E4: {
            drivers: ["ti/drivers/lib/drivers_msp432e4"],
            dpl: {
                tirtos: ["ti/dpl/lib/dpl_msp432e4"],
                nortos: ["lib/nortos_msp432e4"],
                freertos: [""]
            }
        },
        MTL: {
            drivers: ["ti/drivers/lib/drivers_mtxx"],
            dpl: {
                tirtos: ["ti/dpl/lib/dpl_mtxx"],
                nortos: ["lib/nortos_mtxx"],
                freertos: [""]
            }
        }
    };

    /* get device ID to select appropriate libs */
    let devId = system.deviceData.deviceId;

    /* get device information from DriverLib */
    var DriverLib = system.getScript("/ti/devices/DriverLib");
    let attrs = DriverLib.getAttrs(devId);

    /* Get current RTOS configuration information */
    var RTOS = system.modules["/ti/drivers/RTOS"];
    let rtos = "tirtos";
    if (RTOS != undefined) {
        rtos = RTOS.$static.name;

        switch (true) {
            case (/TI-RTOS/i).test(rtos): rtos = "tirtos";
                break;
            case (/FreeRTOS/i).test(rtos): rtos = "freertos";
                break;
            case (/NoRTOS/i).test(rtos): rtos = "nortos";
                break;
            default: rtos = "tirtos";
                break;
        }
    }

    /* Get toolchain specific information from GenLibs */
    let GenLibs = system.getScript("/ti/utils/build/GenLibs");
    let toolchain = GenLibs.getToolchainDir();
    let isa = GenLibs.getDeviceIsa();
    let suffix = Common.getLibSuffix(isa, toolchain);

    /* select a device family-specific set of libs */
    let libs;
    let family = attrs.deviceDefine;
    if (family != "") {
        family = family.replace(/^DeviceFamily_/, "");
        if (family.indexOf("MSP432E") == 0) {
            family = "MSP432E4";
        }
        else if (family.indexOf("CC32") == 0) {
            family = "CC32XX";
        }
        libs = FAMILY2LIBS[family].drivers;
        if (FAMILY2LIBS[family].dpl[rtos] != "") {
            libs.push(FAMILY2LIBS[family].dpl[rtos]);
        }
        libs = libs.map(function(lib) { return (lib += suffix); });
    }
    if (libs == null) {
        throw Error("device2LinkCmd: unknown device family ('"
            + family + "') for deviceId '" + devId + "'");
    }

    /* create a GenLibs input argument */
    var linkOpts = {
        name: "/ti/drivers",
        vers: "1.0.0.0",
        deps: ["/ti/devices/driverlib"],
        libs: libs
    };

    return (linkOpts);
}

/*
 *  ======== base ========
 */
let base = {
    displayName  : "Board",
    staticOnly   : true,

    templates    : {
        /* contribute TI-DRIVERS libraries to linker command file */
        "/ti/utils/build/GenLibs.cmd.xdt"   :
            {modName: "/ti/drivers/Board", getLibs: getLibs},

        /* trigger generation of ti_drivers_config.[ch] */
        "/ti/drivers/templates/Board.c.xdt" : true,
        "/ti/drivers/templates/Board.h.xdt" : true
    },

    moduleStatic : {
        /* ensure somthing supplies appropriate DriverLib library */
        modules  : Common.autoForceModules(["/ti/devices/DriverLib"]),
        config   : []
    }
};

/*
 *  ======== exports ========
 */
exports = base;
