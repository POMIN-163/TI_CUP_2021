/*
 * Copyright (c) 2019-2020, Texas Instruments Incorporated
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

/* global xdc */
var Program = xdc.module('xdc.rov.Program');
var Monitor = xdc.module("xdc.rov.runtime.Monitor");

// Bring in NDK's ROV utility functions and definitions
try {
    var NDK = xdc.loadCapsule("ti/ndk/ndk.rov.js");
}
catch (e) {
    throw new Error("Error: couldn't load NDK ROV capsule: " + e);
}

// Use a local variable to get at the NDK's PBM Packet definition
var PbmPacket = NDK.PbmPacket;

/* eslint-disable-next-line no-unused-vars */
var moduleName = "EMACMSP432E4";

/* eslint-disable-next-line no-unused-vars */
var viewMap = [
    {name: "PBM RX Queue", fxn: "getPbmRxQ", structName: "PbmPacket"},
    {name: "PBM TX Queue", fxn: "getPbmTxQ", structName: "PbmPacket"},
    {name: "DMA PBM RX Queue", fxn: "getDmaPbmRxQ", structName: "PbmPacket"},
    {name: "DMA PBM TX Queue", fxn: "getDmaPbmTxQ", structName: "PbmPacket"}
];

/*
 * ======== getPbmRxQ ========
 * Display the PBM RX queue
 */
/* eslint-disable-next-line no-unused-vars */
function getPbmRxQ()
{
    var view = new Array();

    try {
        var e4EmacPvtData = Program.fetchVariable("EMACMSP432E4_private");
    }
    catch (e) {
        Monitor.println(
            "Error: getPbmRxQ: couldn't fetch EMACMSP432E4_private: " + e);
    }

    NDK.fillPbmView(view, e4EmacPvtData.PBMQ_rx, false);

    return view;
}

/*
 * ======== getPbmTxQ ========
 * Display the PBM TX queue
 */
/* eslint-disable-next-line no-unused-vars */
function getPbmTxQ()
{
    var view = new Array();

    try {
        var e4EmacPvtData = Program.fetchVariable("EMACMSP432E4_private");
    }
    catch (e) {
        Monitor.println(
            "Error: getPbmTxQ: couldn't fetch EMACMSP432E4_private: " + e);
    }

    NDK.fillPbmView(view, e4EmacPvtData.PBMQ_tx, true);

    return view;
}

/*
 * ======== getDmaPbmRxQ ========
 * Display the PBMs in the DMA RX queue
 */
/* eslint-disable-next-line no-unused-vars */
function getDmaPbmRxQ()
{
    var view = new Array();

    try {
        var e4EmacDmaRxDescArray = Program.fetchVariable("g_pRxDescriptors");
    }
    catch (e) {
        Monitor.println(
            "Error: getDmaPbmRxQ: couldn't fetch g_pRxDescriptors: " + e);
    }

    fillDmaPbmView(view, e4EmacDmaRxDescArray, false);

    return view;
}

/*
 * ======== getDmaPbmTxQ ========
 * Display the PBMs in the DMA TX queue
 */
/* eslint-disable-next-line no-unused-vars */
function getDmaPbmTxQ()
{
    var view = new Array();

    try {
        var e4EmacDmaTxDescArray = Program.fetchVariable("g_pTxDescriptors");
    }
    catch (e) {
        Monitor.println(
            "Error: getDmaPbmTxQ: couldn't fetch g_pTxDescriptors: " + e);
    }

    fillDmaPbmView(view, e4EmacDmaTxDescArray, true);

    return view;
}

/*
 * ======== fillDmaPbmView ========
 */
function fillDmaPbmView(view, dmaDescArray, isTx)
{
    // loop over each DMA descriptor in the array and get its PBM buffer
    for (var i = 0; i < dmaDescArray.length; i++)
    {
        var currDesc = dmaDescArray[i];
        if (currDesc != 0x0 && currDesc.hPkt != 0x0)
        {
            /*
             * The following data is fetched directly from the
             * corresponding NDK symbol! As such, if this symbol name
             * changes in NDK code, this ROV code will break.
             */
            var PBM_Pkt = Program.fetchFromAddr(currDesc.hPkt, "PBM_Pkt");
            var pbmPkt = new PbmPacket();

            pbmPkt.pbmAddr = currDesc.hPkt;

            // translate the data from the PBM struct into the view object
            NDK.getPbmStructData(pbmPkt, PBM_Pkt, isTx);

            view.push(pbmPkt);
        }
    }
}
