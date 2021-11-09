/*
 * Copyright (c) 2018-2020, Texas Instruments Incorporated
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

var moduleName = "NDK";

var Program = xdc.module('xdc.rov.Program');

var viewMap = [
    {name: "UDP", fxn: "getUDP", structName: "UDPStats"},
    {name: "TCP", fxn: "getTCP", structName: "TCPStats"},
    {name: "Sockets", fxn: "getSockets", structName: "SocketsInfo"},
    {name: "Interfaces", fxn: "getInterfaces", structName: "Interface"},
    {name: "Memory", fxn: "getMemory", structName: "MemoryStats"},
    {name: "PBM Free Queue", fxn: "getPbmFreeQ", structName: "PbmPacket"}
];

/* Determine endianness. Assume LE if endian is set to null */
var Model = xdc.useModule("xdc.rov.Model");
var isLittleEndian =
    (Model.$private.recap.build.target.model.endian == "little" ||
    Model.$private.recap.build.target.model.endian == null) ? true : false;

/* Common structure sizes (in bytes) */
const ETH_HEADER_SIZE = 14;
const SIZE_OF_MAC_ADDR = 6;
const IP_HEADER_SIZE = 20;
const IPv6_HEADER_SIZE = 40;
const IPv6_ADDR_SIZE = 16;
const IPv6_SHORTS_PER_ADDR = 8;

const MAX_INTERFACE_NAME_LEN = 20;

// TODO: must adjust number of bytes per word to 8 for 64 bit
const BYTES_PER_WORD = 4;
const SIZE_OF_INT = 4;

/* ------------------------------- Memory Stats ------------------------------- */

function MemoryStats()
{
    this.pageStart = 0;
    this.pageSize = 0;
    this.blockSize = 0;
    this.blockSizeIndex = 0;
    this.blockCount = 0;
    this.blocksUsed = 0;
    this.blocksFree = 0;
}

function getMemory()
{
    var view = new Array();

    /*
     * The following data is fetched directly from the corresponding NDK
     * symbol! As such, if this symbol name changes in NDK code, this ROV code
     * will break.
     */
    var pit = Program.fetchVariable("ti_ndk_config_Global_pit");

    for (var i = 0; i < pit.length; i++) {
        var memInfo = new MemoryStats();

        memInfo.pageStart = pit[i].pPageStart;
        memInfo.pageSize = pit[i].PageSize;

        if (pit[i].BlockSize == 0) {
            memInfo.blockSize = "N/A";
            memInfo.blockSizeIndex = "N/A";
            memInfo.blockCount = "N/A";
            memInfo.blocksUsed = "N/A";
            memInfo.blocksFree = "N/A";
        }
        else {
            memInfo.blockSize = pit[i].BlockSize;
            memInfo.blockSizeIndex = pit[i].BlockSizeIdx;
            memInfo.blockCount = pit[i].BlocksPerPage;
            memInfo.blocksUsed = pit[i].AllocCount;
            memInfo.blocksFree = memInfo.blockCount - memInfo.blocksUsed;
        }
        view.push(memInfo);
    }

    return (view);
}

/* ------------------------------- UDP Stats ------------------------------- */

function UDPStats()
{
    this.Recv = 0;
    this.Sent = 0;
    this.Dropped = 0;
    this.NoPort = 0;
    this.NoMCPort = 0;
}

function getUDP()
{
    /*
     * The following data is fetched directly from the corresponding NDK
     * symbol! As such, if this symbol name changes in NDK code, this ROV code
     * will break.
     */
    var rawStats = Program.fetchVariable("NDK_udps");
    var udpStats = new UDPStats();

    udpStats.Recv = rawStats.RcvTotal;
    udpStats.Sent = rawStats.SndTotal;
    udpStats.Dropped = rawStats.RcvFull;
    udpStats.NoPort = rawStats.RcvNoPort;
    udpStats.NoMCPort = rawStats.RcvNoPortB;

    return udpStats;
}

/* ------------------------------- TCP Stats ------------------------------- */

function TCPStats()
{
    this.Recv = 0;
    this.Sent = 0;
}

function getTCP()
{
    /*
     * The following data is fetched directly from the corresponding NDK
     * symbol! As such, if this symbol name changes in NDK code, this ROV code
     * will break.
     */
    var rawStats = Program.fetchVariable("NDK_tcps");
    var tcpStats = new TCPStats();

    tcpStats.Recv = rawStats.RcvTotal;
    tcpStats.Sent = rawStats.SndTotal;

    return tcpStats;
}

/* ------------------------------- Sockets --------------------------------- */

function SocketsInfo()
{
    this.Address = null;
    this.Ctx = null;
    this.Protocol = null;
    this.RxBufSize = null;
    this.RxBufSizeUsed = null;
    this.RxBufSizeAvail = null;
    this.TxBufSize = null;
    this.TxBufSizeUsed = null;
    this.TxBufSizeAvail = null;
    this.LocalAddress = null;
    this.LocalPort = null;
    this.ForeignAddress = null;
    this.ForeignPort = null;
    this.State = null;
}

function getSockets()
{
    var view = new Array();

    var socketTable = Program.fetchVariable("pSockList");
    fillSocketView(view, socketTable, false);

    /*
     * Program.fetchVariable will return an error if the variable is not found
     * and this causes this whole CROV view to not work. This try catch block
     * handles this error state gracefully,
     * because non-ipv6 apps will not have the variable "pSock6List"
     *
     * Note also that  "pSock6List" the name of the corresponding NDK symbol!
     * As such, if this symbol name changes in NDK code, this ROV code
     * will break.
     */
    try
    {
        var socket6Table = Program.fetchVariable("pSock6List");
    }
    catch(err)
    {
        return view;
    }
    fillSocketView(view, socket6Table, true);


    return view;
}

function fillSocketView(view, socketTable, ipv6)
{
    var protocols = [
        "None",
        "TCP",
        "UDP",
        "RAW",
        "RAWETH"
    ];
    var tcpState = [
        "CLOSED",
        "LISTEN",
        "SYN SENT",
        "SYN RECVD",
        "ESTABLISHED",
        "CLOSE WAIT",
        "FIN WAIT1",
        "CLOSING",
        "LAST ACK",
        "FIN WAIT2",
        "TIME WAIT"
    ];

    /* loop through each protocol */
    for (var i = 0; i < (socketTable.length); i++)
    {
        /*
         * raweth socks use a different struct, so flag if this is a raweth sock
         * or not
         */
        var rawethsock;
        if(i == (socketTable.length - 1))
        {
            rawethsock = true;
        }
        else
        {
            rawethsock = false;
        }

        var socketAddress = socketTable[i];

        /* loop through each socket for the protocol */
        while(socketAddress != 0x0)
        {
            var socketsInfo = new SocketsInfo();

            var SOCK;
            /* raweth sock */
            if(rawethsock)
            {
                /*
                 * The following data is fetched directly from the
                 * corresponding NDK symbol! As such, if this symbol name
                 * changes in NDK code, this ROV code will break.
                 */
                SOCK = Program.fetchFromAddr(socketAddress, "SOCKRAWETH");
            }
            else
            {
                /* ipv6 sock */
                if(ipv6)
                {
                    /*
                     * The following data is fetched directly from the
                     * corresponding NDK symbol! As such, if this symbol name
                     * changes in NDK code, this ROV code will break.
                     */
                    SOCK = Program.fetchFromAddr(socketAddress, "SOCK6");
                }
                /* regular sock */
                else
                {
                    /*
                     * The following data is fetched directly from the
                     * corresponding NDK symbol! As such, if this symbol name
                     * changes in NDK code, this ROV code will break.
                     */
                    SOCK = Program.fetchFromAddr(socketAddress, "SOCK");
                }
            }

            socketsInfo.Address = socketAddress;
            socketsInfo.Ctx = SOCK.Ctx;
            socketsInfo.Protocol = protocols[i];

            /* ---------------- Local and Foreign Address -------------------*/

            if(rawethsock)
            {
                socketsInfo.LocalAddress = "NA";
                socketsInfo.LocalPort = "NA";
                socketsInfo.ForeignAddress = "NA";
                socketsInfo.ForeignPort = "NA";
            }
            else if(ipv6)
            {
                /*
                 * Currently CROV cannot access the LIP or FIP structs in the
                 * SOCK6 struct correctly.
                 */
                socketsInfo.LocalAddress = "Not Supported";
                socketsInfo.LocalPort = "Not Supported";
                socketsInfo.ForeignAddress = "Not Supported";
                socketsInfo.ForeignPort = "Not Supported";
            }
            else
            {
                socketsInfo.LocalAddress = addrToDots(SOCK.LIP, isLittleEndian);
                socketsInfo.LocalPort = ntohs(SOCK.LPort, isLittleEndian);
                socketsInfo.ForeignAddress = addrToDots(SOCK.FIP, isLittleEndian);
                socketsInfo.ForeignPort = ntohs(SOCK.FPort, isLittleEndian);
            }

            /* ---------------------- RX/TX Buffer --------------------------*/

            var rxBufPtr = SOCK.hSBRx;
            if(rxBufPtr != 0x0)
            {
                /*
                 * The following data is fetched directly from the
                 * corresponding NDK symbol! As such, if this symbol name
                 * changes in NDK code, this ROV code will break.
                 */
                var rxBuf = Program.fetchFromAddr(rxBufPtr, "SB");
                socketsInfo.RxBufSizeUsed = rxBuf.Total;
                socketsInfo.RxBufSizeAvail = rxBuf.Max - rxBuf.Total;
                socketsInfo.RxBufSize = SOCK.RxBufSize;
            }
            else
            {
                socketsInfo.RxBufSizeUsed = "No Buffer";
                socketsInfo.RxBufSize = "No Buffer";
            }

            var txBufPtr = SOCK.hSBTx;
            if(txBufPtr != 0x0)
            {
                /*
                 * The following data is fetched directly from the
                 * corresponding NDK symbol! As such, if this symbol name
                 * changes in NDK code, this ROV code will break.
                 */
                var txBuf = Program.fetchFromAddr(txBufPtr, "SB");
                socketsInfo.TxBufSizeUsed = txBuf.Total;
                socketsInfo.TxBufSizeAvail = txBuf.Max - txBuf.Total;
                socketsInfo.TxBufSize = SOCK.TxBufSize;
            }
            else
            {
                socketsInfo.TxBufSizeUsed = "No Buffer";
                socketsInfo.TxBufSize = "No Buffer";
            }

            /* -------------------------- TCP Stats -------------------------*/

            var hTP;
            /* rawethsock does not have a hTP, so manually set it to 0 */
            if(rawethsock)
            {
                hTP = 0x0;
            }
            else
            {
                hTP = SOCK.hTP;
            }

            if(i == 1 && hTP != 0x0)
            {
                /*
                 * The following data is fetched directly from the
                 * corresponding NDK symbol! As such, if this symbol name
                 * changes in NDK code, this ROV code will break.
                 */
                var TCPPROT = Program.fetchFromAddr(hTP, "TCPPROT");
                var t_state = TCPPROT.t_state;
                if (t_state >= 0 && t_state <= 10) {
                    socketsInfo.State = tcpState[t_state];
                }
                else
                {
                    socketsInfo.State = "UNKOWN";
                }
            }
            else
            {
                socketsInfo.State = "NA - TCP Only";
            }

            view.push(socketsInfo);

            socketAddress = SOCK.pProtNext;
        }
    }
}

/* ---------------------------- PBM Info ---------------------------- */
function PbmPacket()
{
    this.pbmAddr = 0;
    this.bufferLen = 0;
    this.bufferAddr = 0;
    this.validLen = 0;
    this.srcMAC = 0;
    this.dstMAC = 0;
    this.protocol = 0;
    this.srcIpAddr = 0;
    this.dstIpAddr = 0;
    this.IpPayloadType = 0;
    this.IpPayloadSrcPort = 0;
    this.IpPayloadDstPort = 0;
}

function getEthTypeStr(type)
{
    var typeStr = "";

    /*
     * Translate the values of this Ethernet frame's 'type' field to human
     * readable string format. The hex type numbers come from RFC 7042.
     */
    switch (type) {
        case 0x800:
            typeStr = "IPv4";
            break;
        case 0x86DD:
            typeStr = "IPv6";
            break;
        case 0x806:
            typeStr = "ARP";
            break;
        case 0x8100:
            typeStr = "VLAN";
            break;
        case 0x8863:
            typeStr = "PPPoE CTRL";
            break;
        case 0x8864:
            typeStr = "PPPoE DATA";
            break;
        default:
            typeStr = "Unknown";
            break;
    }

    return (typeStr);
}

function getIpTypeStr(type)
{
    var typeStr = "";

    switch (type) {
        case 1:
            typeStr = "ICMP";
            break;
        case 2:
            typeStr = "IGMP";
            break;
        case 6:
            typeStr = "TCP";
            break;
        case 17:
            typeStr = "UDP";
            break;
        case 41:
            typeStr = "IPv6 Payload";
            break;
        case 58:
            typeStr = "ICMPv6";
            break;
        default:
            typeStr = "Unknown";
            break;
    }

    return (typeStr);
}

/*
 * Utility function to read the MAC address stored in the ETHHDR struct. This
 * is necessary due to problems reading an array that's part of a struct
 * (XDCTOOLS-345)
 *
 * Returns a string containing the MAC address in colon separated hex.
 */
function readMacAddr(macAddrBytePos)
{
    var macByteVal;
    var macStr = "";
    var tmpStr = "";

    for (var i = 0; i < SIZE_OF_MAC_ADDR; i++) {
        // read this byte of the MAC address
        macByteVal = Program.fetchFromAddr(macAddrBytePos, "unsigned char");

        // convert to a string in hex format:
        tmpStr = macByteVal.toString(16);

        // ensure leading zero isn't dropped
        if (tmpStr.length == 1) {
            tmpStr = "0" + tmpStr;
        }

        // follow each byte with a ":", except for the final byt of the addr
        if (i < (SIZE_OF_MAC_ADDR - 1)) {
            tmpStr += ":";
        }

        macStr += tmpStr;

        // move to the next byte of the MAC address
        macAddrBytePos += 1;
    }

    return (macStr);
}

/*
 * Utility function to read the IPv6 address stored in the IP6N struct. Here's
 * an example IPv6 address:
 *
 *  a. long form:
 *
 *    fe80:0000:0000:0000:abcd:1234:fe1c:dcba
 *
 *  b. zero compressed form:
 *
 *    fe80::abcd:1234:fe1c:dcba
 *
 * This byte by byte processing is necessary due to problems reading an array
 * that's part of a struct (XDCTOOLS-345)
 *
 * Returns a string containing the (zero compressed) IPv6 address in colon
 * separated hex (e.g. example b. above).
 */
function readIPv6Addr(bytePos)
{
    var addrStr = "";
    var ipv6AddrWord = 0;
    var zeroComprAllowed = true;
    var zeroComprCount = 0;
    var ipv6AddrPos = bytePos;

    // process the IPv6 address one block at a time, where each block is 2 bytes
    for (var count = 0; count < IPv6_SHORTS_PER_ADDR; count++, ipv6AddrPos += 2) {
        // read current block of IPv6 addr and ensure correct byte order
        ipv6AddrWord = Program.fetchFromAddr(ipv6AddrPos, "uint_least16_t");
        ipv6AddrWord = ntohs(ipv6AddrWord, isLittleEndian);

        // check if this address block is zero, and zero compress it if we can
        if (ipv6AddrWord == 0) {
            if (zeroComprAllowed) {
                // these zeroes will be compressed. Skip to next block
                zeroComprCount++;

                if (zeroComprCount == IPv6_SHORTS_PER_ADDR) {
                    // each block of the IPv6 addr is 0. This is a special case:
                    addrStr = "::";
                }
                continue;
            }

        }
        else {
            // curr addr block is non zero; were we zero compressing prior?
            if (zeroComprAllowed && zeroComprCount > 0) {
                // zero compression end condition, append closing ':'
                zeroComprAllowed = false;
                zeroComprCount = 0;
                addrStr += ":";
            }
        }

        // append current addr block
        addrStr += ipv6AddrWord.toString(16);

        // only append ":" if this is not the final block of this addr
        if (count < (IPv6_SHORTS_PER_ADDR - 1)) {
            addrStr += ":";
        }
    }

    return (addrStr);
}

/*
 * Display the NDK's PBM packet free pool (queue)
 */
function getPbmFreeQ()
{
    var view = new Array();

    /*
     * The following data is fetched directly from the corresponding NDK
     * symbol! As such, if this symbol name changes in NDK code, this ROV code
     * will break.
     */
    var pbmqFreeQ = Program.fetchVariable("PBMQ_free");

    // The free Q will have both TX and RX packets, so omit last arg
    fillPbmView(view, pbmqFreeQ);

    return view;
}

/* Given an NDK PBMQ object, populate an ROV view displaying all its packets */
function fillPbmView(view, pbmQ, isTx)
{
    var pbmHead = pbmQ.pHead;
    var currPbmPkt = pbmHead;

    /* loop over each packet in the queue */
    while (currPbmPkt != 0x0)
    {
        /*
         * The following data is fetched directly from the
         * corresponding NDK symbol! As such, if this symbol name
         * changes in NDK code, this ROV code will break.
         */
        var PBM_Pkt = Program.fetchFromAddr(currPbmPkt, "PBM_Pkt");
        var pbmPkt = new PbmPacket();

        pbmPkt.pbmAddr = currPbmPkt;

        // translate the data from the PBM struct into the view object
        getPbmStructData(pbmPkt, PBM_Pkt, isTx);

        view.push(pbmPkt);

        // move to the next PBM packet
        currPbmPkt = PBM_Pkt.pNext;
    }
}

/* utility function to read data out of a PBM_Pkt struct & into a PbmPacket */
function getPbmStructData(pbmPkt, PBM_Pkt, isTx)
{
    var ethType = "";

    pbmPkt.bufferLen = PBM_Pkt.BufferLen;
    pbmPkt.bufferAddr = PBM_Pkt.pDataBuffer;
    pbmPkt.validLen = PBM_Pkt.ValidLen;

    /*
     * Read the Ethernet frame that's stored in this PBM buffer. By starting
     * at the Ethernet header, we can decode all of the encapsulated
     * protocols within and display their info in ROV.
     *
     * The Ethernet frame header is defined as follows:
     *
     *     typedef struct {
     *         unsigned char DstMac[6];
     *         unsigned char SrcMac[6];
     *         uint16_t  Type;
     *     } ETHHDR;
     *
     * For RX packets, the Ethernet header alway starts at the beginning
     * of the buffer (offset of zero), and decoding is easy. This is also the
     * case for RX packets whose PBM is reused and immediately TX'd back out
     * (e.g. for ping responses and some ARP packets).
     *
     * However, for newly created TX packets, the start of the Ethernet
     * header may not be at the beginning of the buffer. Rather, it depends
     * on the value of nimu_mcb.header_size. In this case, the correct
     * offset from the beginning of the buffer is as follows (see also:
     * NIMUAddEthernetHeader() and NIMUCreatePacket() fxns in nimu.c of NDK):
     *
     *     offset = nimu_mcb.header_size - ETH_HEADER_SIZE
     *
     * The Ethernet header is then located using this offset:
     *
     *    <start of Eth header> = <start of buffer> + offset
     */

    /*
     * Get the NIMU Master Control Block.
     *
     * This data is fetched directly from the corresponding NDK symbol! As
     * such, if this symbol name changes in NDK code, this ROV code will
     * break.
     */
    var nimu_mcb = Program.fetchVariable("nimu_mcb");

    // compute the expected TX offset (see above)
    var txOffSet = nimu_mcb.header_size - ETH_HEADER_SIZE;

    // adjust offset for PBMs created in the TX path. Offset is 0 for RX
    var dataOffset = ((isTx && PBM_Pkt.DataOffset != 0) ||
        PBM_Pkt.DataOffset == txOffSet) ? txOffSet : 0;

    var currBufferPos = PBM_Pkt.pDataBuffer + dataOffset;

    /*
     * The following data is fetched directly from the
     * corresponding NDK symbol! As such, if this symbol name
     * changes in NDK code, this ROV code will break.
     */
    var ETHHDR = Program.fetchFromAddr(currBufferPos, "ETHHDR");

    /*
     * Read the destination and source MAC addresses from the header. Due
     * to XDCTOOLS-345, must fetch each byte individually. Use a different
     * variable to track the byte position in the buffer for this MAC
     * address work
     */
    var macAddrBytePos = currBufferPos;
    pbmPkt.dstMAC = readMacAddr(macAddrBytePos);

    // move past the destination address and onto the source address
    macAddrBytePos += SIZE_OF_MAC_ADDR;

    pbmPkt.srcMAC = readMacAddr(macAddrBytePos);

    /*
     * Get the type, which tells us the protocol that's in the payload.
     * Since this is a 2 byte value and we're reading directly from the
     * packet, must convert from network byte order to host byte order
     */
    ethType = ntohs(ETHHDR.Type, isLittleEndian);
    pbmPkt.protocol = getEthTypeStr(ethType);

    // Skip over the Ethernet header to the start of the payload
    currBufferPos += ETH_HEADER_SIZE;

    // Decode the Ethernet payload for IP packets
    if (pbmPkt.protocol == "IPv4") {
        /*
         * get the source and destination addresses of this IP packet
         *
         * This data is fetched directly from the corresponding NDK symbol! As
         * such, if this symbol name changes in NDK code, this ROV code will
         * break.
         */
        var IPHDR = Program.fetchFromAddr(currBufferPos, "IPHDR");
        pbmPkt.srcIpAddr = addrToDots(IPHDR.IPSrc, isLittleEndian);
        pbmPkt.dstIpAddr = addrToDots(IPHDR.IPDst, isLittleEndian);

        /*
         * The IP header's protocol field tells us the protocol that's
         * contained in the IP packet's payload
         */
        pbmPkt.IpPayloadType = getIpTypeStr(IPHDR.Protocol);

        // skip over the IP header and onto start of the IP payload
        currBufferPos += IP_HEADER_SIZE;
    }
    else if (pbmPkt.protocol == "IPv6") {
        /*
         * The following data is fetched directly from the
         * corresponding NDK symbol! As such, if this symbol name
         * changes in NDK code, this ROV code will break.
         */
        var IPV6HDR = Program.fetchFromAddr(currBufferPos, "IPV6HDR");

        /*
         * Read the source and destination IPv6 addresses using the current
         * byte offset, instead of reading from IPV6HDR.SrcAddr[] directly.
         * Must read the IPv6 addresses in this byte by byte manner due
         * to XDCTOOLS-345. Use a different variable to track the byte
         * position in the buffer for this IPv6 address work.
         *
         * The IPv6 header is defined as follows, where the source and
         * destination addresses are each 16 byte arrays. The addresses are
         * preceeded by 2 words of data in the header:
         *
         *    typedef struct {
         *       < word 1 >
         *       < word 2 >
         *       IP6N     SrcAddr; // 16 byte array
         *       IP6N     DstAddr; // 16 byte array
         *      } IPV6HDR;
         */
        var ipv6AddrPos = currBufferPos;

        /*
         * Skip over first 2 words in the IPv6 header to get to the source
         * address
         */
        ipv6AddrPos += 8;

        // our current byte pos is now at the start of the src addr; get it
        pbmPkt.srcIpAddr = readIPv6Addr(ipv6AddrPos);

        // done w/ src addr, move past it and on to the dst address
        ipv6AddrPos += IPv6_ADDR_SIZE;

        // our current byte pos is now at the start of the dst addr; get it
        pbmPkt.dstIpAddr = readIPv6Addr(ipv6AddrPos);

        /*
         * The IP header's protocol field tells us the protocol that's
         * contained in the IP packet's payload
         */
        pbmPkt.IpPayloadType = getIpTypeStr(IPV6HDR.NextHeader);

        // skip over the IP header
        currBufferPos += IPv6_HEADER_SIZE;
    }
    else {
        // non-IP packets won't display any further data
        pbmPkt.IpPayloadType = "N/A";
    }

    // Decode the IP payload for UDP and TCP packets
    if (pbmPkt.IpPayloadType == "UDP") {
        /*
         * The following data is fetched directly from the
         * corresponding NDK symbol! As such, if this symbol name
         * changes in NDK code, this ROV code will break.
         */
        var UDPHDR = Program.fetchFromAddr(currBufferPos, "UDPHDR");

        pbmPkt.IpPayloadSrcPort = ntohs(UDPHDR.SrcPort, isLittleEndian);
        pbmPkt.IpPayloadDstPort = ntohs(UDPHDR.DstPort, isLittleEndian);
    }
    else if (pbmPkt.IpPayloadType == "TCP") {
        /*
         * The following data is fetched directly from the
         * corresponding NDK symbol! As such, if this symbol name
         * changes in NDK code, this ROV code will break.
         */
        var TCPHDR = Program.fetchFromAddr(currBufferPos, "TCPHDR");

        pbmPkt.IpPayloadSrcPort = ntohs(TCPHDR.SrcPort, isLittleEndian);
        pbmPkt.IpPayloadDstPort = ntohs(TCPHDR.DstPort, isLittleEndian);
    }
    else {
        // unsupported packet
        pbmPkt.IpPayloadSrcPort = "N/A";
        pbmPkt.IpPayloadDstPort = "N/A";
    }
}

/* ---------------------------- Interfaces Info ---------------------------- */

function Interface()
{
    this.Address = 0;
    this.Name = 0;
    this.IfId = 0;
    this.MTU = 0;
    this.MACaddr = 0;
    this.IPv4Addr = 0;
    this.IPv4Mask = 0;
    this.IPv6Addr = 0;
}

/*
 * Utility function to get the name of an interface.
 *
 * The byte by byte processing done within is necessary due to problems reading
 * an array that's part of a struct (XDCTOOLS-345)
 *
 * Parameter: A valid pointer to a NIMU device object
 *            (i.e. pointer to struct NETIF_DEVICE)
 *
 * Returns: a string containing the interface name
 */
function getNameFromIF(nimuDev)
{
    var ifNameStr = "N/A";

    if (nimuDev != 0) {
        // start from the address of the NIMU device
        var currBytePos = nimuDev;

        /*
         * Skip to the name field starting from beginning of the
         * NETIF_DEVICE struct's start address (workaround for XDCTOOLS-345)
         *
         * The NETIF_DEVICE struct is defined as follows. The name[] array
         * is preceeded by several fields whose size depends on the data model
         * (ILP32 or LP64):
         *
         *   typedef struct NETIF_DEVICE
         *   {
         *       *p_next        // 8 byte pointer for LP64!
         *       *p_prev        // 8 byte pointer for LP64!
         *        int RefCount; // 4 bytes in both data models
         *        int index;    // 4 bytes in both data models
         *        char name[];  // MAX_INTERFACE_NAME_LEN bytes
         *        ...
         *   }
         *
         * To get to the name array, must skip over all previous data. Refer to
         * the similar comment in the getIpv6LinkLocalAddrFromIF() function below,
         * which details this calculation for the BIND6_ENTRY struct, which is
         * similar.
         */

        // skip over preceeding struct fields to get to the MAC addr:
        var numPtrs = 2;
        var numInts = 2;
        currBytePos = currBytePos +
            (numPtrs * BYTES_PER_WORD) + (numInts * SIZE_OF_INT);

        // read the string stored at name[]
        ifNameStr = Program.fetchString(currBytePos, false);
    }

    return (ifNameStr);
}

/*
 * Utility function to get the MAC address of an interface.
 *
 * The byte by byte processing done within is necessary due to problems reading
 * an array that's part of a struct (XDCTOOLS-345)
 *
 * Parameter: A valid pointer to a NIMU device object
 *            (i.e. pointer to struct NETIF_DEVICE)
 *
 * Returns: a string containing the MAC address in colon separated hex.
 */
function getMacAddrFromIF(nimuDev)
{
    var macAddrStr = "N/A";

    if (nimuDev != 0) {
        // start from the address of the NIMU device
        var currBytePos = nimuDev;

        /*
         * Skip to the MAC addr starting from beginning of the
         * NETIF_DEVICE struct's start address (workaround for XDCTOOLS-345)
         *
         * The NETIF_DEVICE struct is defined as follows. The MAC address
         * is preceeded by several fields whose size depends on the data model
         * (ILP32 or LP64):
         *
         *   typedef struct NETIF_DEVICE
         *   {
         *       *p_next        // 8 byte pointer for LP64!
         *       *p_prev        // 8 byte pointer for LP64!
         *        int RefCount; // 4 bytes in both data models
         *        int index;    // 4 bytes in both data models
         *        char name[];  // MAX_INTERFACE_NAME_LEN bytes
         *        int flags     // 4 bytes in both data models
         *        int type      // 4 bytes in both data models
         *        int mtu       // 4 bytes in both data models
         *        char mac_address[]; // 6 bytes
         *        ...
         *   }
         *
         * To get to the mac_address, must skip over all previous data. Refer to
         * the similar comment in the getIpv6LinkLocalAddrFromIF() function below,
         * which details this calculation for the BIND6_ENTRY struct, which is
         * similar.
         */

        // skip over preceeding struct fields to get to the MAC addr:
        var numPtrs = 2;
        var numInts = 5;
        currBytePos = currBytePos + MAX_INTERFACE_NAME_LEN +
            (numPtrs * BYTES_PER_WORD) + (numInts * SIZE_OF_INT);

        macAddrStr = readMacAddr(currBytePos);
    }

    return (macAddrStr);
}

/*
 * Utility function to get the IPv6 Link Local address that's bound to an
 * interface.
 *
 * The byte by byte processing done within is necessary due to problems reading
 * an array that's part of a struct (XDCTOOLS-345)
 *
 * Parameter: A valid pointer to a NIMU device object's IPv6 device record
 *            (i.e. struct NETIF_DEVICE's ptr_ipv6device field)
 *
 * Returns a string containing the Link Local IPv6 address (zero compressed,
 * colon separated hex) that's bound to the provided interface.
 */
function getIpv6LinkLocalAddrFromIF(ipv6Dev)
{
    var ipv6AddrStr = "N/A"; // assume IPv6 disabled

    // check if there is an IPv6 record available for this IF
    if (ipv6Dev != 0) {
        // the IPv6 record contains the BIND6_ENTRY obj, which has the IPv6 LLA
        var IPV6_DEV_RECORD = Program.fetchFromAddr(ipv6Dev, "IPV6_DEV_RECORD");

        // get the start address of the BIND6_ENTRY
        var currBytePos = IPV6_DEV_RECORD.hLinkLocalBind;

        /*
         * Skip to IPv6 addr starting from beginning of BIND6_ENTRY struct's
         * start address (workaround for XDCTOOLS-345)
         *
         * The BIND6_ENTRY struct is defined as follows. The Link Local
         * IPv6 address is preceeded by 5 fields whose size depends on the data
         * model (ILP32 or LP64):
         *
         *   struct BIND6_ENTRY {
         *      *p_next     // 8 byte pointer for LP64!
         *      *p_prev     // 8 byte pointer for LP64!
         *      int flags   // 4 bytes in both data models
         *      *ptr_device // 8 byte pointer for LP64!
         *      *hRtNet     // 8 byte pointer for LP64!
         *      *hRtHost    // 8 byte pointer for LP64!
         *       IP6N IPHost; // 16 byte array holding LLA
         *       ...
         *   }
         *
         * To get to 'IPHost', must skip over all previous data in the
         * struct, but the amount to skip depends on the data model:
         *
         *     numPtrs = 5;
         *     offset = (numPtrs * BYTES_PER_WORD) + (SIZE_OF_INT)
         *
         *     A. ILP32: pointers are 4 bytes:
         *        - BYTES_PER_WORD = 4
         *        - SIZE_OF_INT = 4
         *        - offset = (5 * 4) + 4 = 24 bytes
         *
         *     B. LP64: pointers are 8 bytes:
         *        - BYTES_PER_WORD = 8
         *        - SIZE_OF_INT = 4
         *        - offset = (5 * 8) + 4 = 44 bytes
         */

        // skip over preceeding struct fields to get to the IPv6 addr:
        var numPtrs = 5;
        var numInts = 1;
        currBytePos = currBytePos +
            (numPtrs * BYTES_PER_WORD) + (numInts * SIZE_OF_INT);

        ipv6AddrStr = readIPv6Addr(currBytePos);
    }

    return (ipv6AddrStr);
}

function getInterfaces()
{
    var ifs = new Array();

    /*
     * The following data is fetched directly from the corresponding NDK
     * symbol! As such, if this symbol name changes in NDK code, this ROV code
     * will break.
     */

    var head = Program.lookupSymbolValue("pbindFirst");
    var next = Program.fetchFromAddr(head, "uintptr_t");
    while (next) {
        var bind = Program.fetchFromAddr(next, "BIND");
        if (bind.Type == 0x9) {
            var nextif = new Interface();
            var NETIF_DEVICE = Program.fetchFromAddr(bind.hIF, "NETIF_DEVICE");

            // build the IF view:
            nextif.Address = bind.hIF;
            nextif.Name = getNameFromIF(bind.hIF);
            nextif.MACaddr = getMacAddrFromIF(bind.hIF);
            nextif.IfId = NETIF_DEVICE.index;
            nextif.MTU = NETIF_DEVICE.mtu;
            nextif.IPv4Addr = addrToDots(bind.IPHost, isLittleEndian);
            nextif.IPv4Mask = addrToDots(bind.IPMask, isLittleEndian);
            nextif.IPv6Addr =
                getIpv6LinkLocalAddrFromIF(NETIF_DEVICE.ptr_ipv6device);

            ifs.push(nextif);
            next = bind.pNext;
        }
        else {
            next = 0;
        }
    }

    return (ifs);
}

function addrToDots(addr, little)
{
    if (little) {
        return (addr & 0xff) + "." + ((addr >> 8) & 0xff) + "." +
            ((addr >> 16) & 0xff) + "." + ((addr >> 24) & 0xff);
    }
    else {
        return (addr >> 24) + "." + ((addr >> 16) & 0xff) + "." +
            ((addr >> 8) & 0xff) + "." + (addr & 0xff);
    }
}

function ntohs(port, little)
{
    if (little) {
        return (((port << 8) & 0xff00) | ((port >> 8) & 0x00ff));
    }
    else {
        return (port);
    }
}
