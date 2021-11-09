/*
 * Copyright (c) 2012, Texas Instruments Incorporated
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
 * */
/*
 * ======== constat.c ========
 *
 * Console Stat Function
 *      ConCmdStat  -   Console Stat command function
 *
 */

#include <netmain.h>
#include <_stack.h>
#include "console.h"

static void DumpIPStats();
static void DumpICMPStats();
static void DumpTCPStats();
static void DumpUDPStats();
static void DumpNATStats();

/*------------------------------------------------------------------------- */
/* ConCmdStat() */
/* Function to print out stack statistics */
/*------------------------------------------------------------------------- */
void ConCmdStat( int ntok, char *tok1 )
{
    /* Check for 'stat ip' */
    if( ntok == 1 && !stricmp( tok1, "ip" ) )
        DumpIPStats();
    /* Check for 'stat icmp' */
    else if( ntok == 1 && !stricmp( tok1, "icmp" ) )
        DumpICMPStats();
    /* Check for 'stat tcp' */
    else if( ntok == 1 && !stricmp( tok1, "tcp" ) )
        DumpTCPStats();
    /* Check for 'stat udp' */
    else if( ntok == 1 && !stricmp( tok1, "udp" ) )
        DumpUDPStats();
    /* Check for 'stat nat' */
    else if( ntok == 1 && !stricmp( tok1, "nat" ) )
        DumpNATStats();
    else if( ntok == 0 )
    {
        ConPrintf("\n[Stat Command]\n");
        ConPrintf("\nCalled to dump out internal stack statistics.\n\n");
        ConPrintf("stat ip      - Print out IP statistics\n");
        ConPrintf("stat icmp    - Print out ICMP statistics\n");
        ConPrintf("stat tcp     - Print out TCP statistics\n");
        ConPrintf("stat udp     - Print out UDP and RAW statistics\n");
        ConPrintf("stat nat     - Print out NAT statistics\n\n");
    }
    else
        ConPrintf("\nIllegal argument. Type 'stat' for help\n");
}

/*------------------------------------------------------------------------- */
/* DumpIPStats() */
/* Dump out IP Statistics */
/*------------------------------------------------------------------------- */
static void DumpIPStats()
{
    ConPrintf("\nIP Statistics:\n");
    ConPrintf("   Total         = %010u  ", NDK_ips.Total        );
    ConPrintf("   Odropped      = %010u\n", NDK_ips.Odropped     );
    ConPrintf("   Badsum        = %010u  ", NDK_ips.Badsum       );
    ConPrintf("   Badhlen       = %010u\n", NDK_ips.Badhlen      );
    ConPrintf("   Badlen        = %010u  ", NDK_ips.Badlen       );
    ConPrintf("   Badoptions    = %010u\n", NDK_ips.Badoptions   );
    ConPrintf("   Badvers       = %010u  ", NDK_ips.Badvers      );
    ConPrintf("   Forward       = %010u\n", NDK_ips.Forward      );
    ConPrintf("   Noproto       = %010u  ", NDK_ips.Noproto      );
    ConPrintf("   Delivered     = %010u\n", NDK_ips.Delivered    );
    ConPrintf("   Cantforward   = %010u  ", NDK_ips.Cantforward  );
    ConPrintf("   CantforwardBA = %010u\n", NDK_ips.CantforwardBA);
    ConPrintf("   Expired       = %010u  ", NDK_ips.Expired      );
    ConPrintf("   Redirectsent  = %010u\n", NDK_ips.Redirectsent );
    ConPrintf("   Localout      = %010u  ", NDK_ips.Localout     );
    ConPrintf("   Localnoroute  = %010u\n", NDK_ips.Localnoroute );
    ConPrintf("   CacheHit      = %010u  ", NDK_ips.CacheHit     );
    ConPrintf("   CacheMiss     = %010u\n", NDK_ips.CacheMiss    );
    ConPrintf("   Fragments     = %010u  ", NDK_ips.Fragments    );
    ConPrintf("   Fragdropped   = %010u\n", NDK_ips.Fragdropped  );
    ConPrintf("   Fragtimeout   = %010u  ", NDK_ips.Fragtimeout  );
    ConPrintf("   Reassembled   = %010u\n", NDK_ips.Reassembled  );
    ConPrintf("   Ofragments    = %010u  ", NDK_ips.Ofragments   );
    ConPrintf("   Fragmented    = %010u\n", NDK_ips.Fragmented   );
    ConPrintf("   Cantfrag      = %010u  ", NDK_ips.Cantfrag     );
    ConPrintf("   Filtered      = %010u\n", NDK_ips.Filtered     );
}

/*------------------------------------------------------------------------- */
/* DumpICMPStats() */
/* Dump out ICMP Statistics */
/*------------------------------------------------------------------------- */
static void DumpICMPStats()
{
    ConPrintf("\nICMP Statistics:\n");
    ConPrintf("    ICMP Rx Errors: %u\n", NDK_ICMPInErrors);
    ConPrintf("    ICMP Tx Errors: %u\n", NDK_ICMPOutErrors);
    ConPrintf("    ECHO             In: %010u     Out: %010u\n",
           NDK_ICMPIn[ICMP_ECHO], NDK_ICMPOut[ICMP_ECHO] );
    ConPrintf("    ECHOREPLY        In: %010u     Out: %010u\n",
           NDK_ICMPIn[ICMP_ECHOREPLY], NDK_ICMPOut[ICMP_ECHOREPLY] );
    ConPrintf("    TSTAMP           In: %010u     Out: %010u\n",
           NDK_ICMPIn[ICMP_TSTAMP], NDK_ICMPOut[ICMP_TSTAMP] );
    ConPrintf("    TSTAMPREPLY      In: %010u     Out: %010u\n",
           NDK_ICMPIn[ICMP_TSTAMPREPLY], NDK_ICMPOut[ICMP_TSTAMPREPLY] );
    ConPrintf("    IREQ             In: %010u     Out: %010u\n",
           NDK_ICMPIn[ICMP_IREQ], NDK_ICMPOut[ICMP_IREQ] );
    ConPrintf("    IREQREPLY        In: %010u     Out: %010u\n",
           NDK_ICMPIn[ICMP_IREQREPLY], NDK_ICMPOut[ICMP_IREQREPLY] );
    ConPrintf("    MASKREQ          In: %010u     Out: %010u\n",
           NDK_ICMPIn[ICMP_MASKREQ], NDK_ICMPOut[ICMP_MASKREQ] );
    ConPrintf("    MASKREPLY        In: %010u     Out: %010u\n",
           NDK_ICMPIn[ICMP_MASKREPLY], NDK_ICMPOut[ICMP_MASKREPLY] );
    ConPrintf("    UNREACH          In: %010u     Out: %010u\n",
           NDK_ICMPIn[ICMP_UNREACH], NDK_ICMPOut[ICMP_UNREACH] );
    ConPrintf("    SOURCEQUENCH     In: %010u     Out: %010u\n",
           NDK_ICMPIn[ICMP_SOURCEQUENCH], NDK_ICMPOut[ICMP_SOURCEQUENCH] );
    ConPrintf("    REDIRECT         In: %010u     Out: %010u\n",
           NDK_ICMPIn[ICMP_REDIRECT], NDK_ICMPOut[ICMP_REDIRECT] );
    ConPrintf("    ROUTERADVERT     In: %010u     Out: %010u\n",
           NDK_ICMPIn[ICMP_ROUTERADVERT], NDK_ICMPOut[ICMP_ROUTERADVERT] );
    ConPrintf("    ROUTERSOLICIT    In: %010u     Out: %010u\n",
           NDK_ICMPIn[ICMP_ROUTERSOLICIT], NDK_ICMPOut[ICMP_ROUTERSOLICIT] );
    ConPrintf("    TIMXCEED         In: %010u     Out: %010u\n",
           NDK_ICMPIn[ICMP_TIMXCEED], NDK_ICMPOut[ICMP_TIMXCEED] );
    ConPrintf("    PARAMPROB        In: %010u     Out: %010u\n",
           NDK_ICMPIn[ICMP_PARAMPROB], NDK_ICMPOut[ICMP_PARAMPROB] );
}

/*------------------------------------------------------------------------- */
/* DumpTCPStats() */
/* Dump out TCP Statistics */
/*------------------------------------------------------------------------- */
static void DumpTCPStats()
{
    ConPrintf("\nTCP Statistics:\n");
    ConPrintf("  RcvTotal       = %010u  ", NDK_tcps.RcvTotal        );
    ConPrintf("  RcvShort       = %010u\n", NDK_tcps.RcvShort        );
    ConPrintf("  RcvHdrSize     = %010u  ", NDK_tcps.RcvHdrSize      );
    ConPrintf("  RcvBadSum      = %010u\n", NDK_tcps.RcvBadSum       );
    ConPrintf("  RcvAfterClose  = %010u  ", NDK_tcps.RcvAfterClose   );
    ConPrintf("  RcvDupAck      = %010u\n", NDK_tcps.RcvDupAck       );
    ConPrintf("  RcvPack        = %010u  ", NDK_tcps.RcvPack         );
    ConPrintf("  RcvByte        = %010u\n", NDK_tcps.RcvByte         );
    ConPrintf("  RcvAckPack     = %010u  ", NDK_tcps.RcvAckPack      );
    ConPrintf("  RcvAckByte     = %010u\n", NDK_tcps.RcvAckByte      );
    ConPrintf("  RcvDupPack     = %010u  ", NDK_tcps.RcvDupPack      );
    ConPrintf("  RcvDupByte     = %010u\n", NDK_tcps.RcvDupByte      );
    ConPrintf("  RcvPartDupPack = %010u  ", NDK_tcps.RcvPartDupPack  );
    ConPrintf("  RcvPartDupByte = %010u\n", NDK_tcps.RcvPartDupByte  );
    ConPrintf("  RcvAfterWinPack= %010u  ", NDK_tcps.RcvAfterWinPack );
    ConPrintf("  RcvAfterWinByte= %010u\n", NDK_tcps.RcvAfterWinByte );
    ConPrintf("  RcvOOPack      = %010u  ", NDK_tcps.RcvOOPack       );
    ConPrintf("  RcvOOByte      = %010u\n", NDK_tcps.RcvOOByte       );
    ConPrintf("  RcvWinUpd      = %010u  ", NDK_tcps.RcvWinUpd       );
    ConPrintf("  RcvWinProbe    = %010u\n", NDK_tcps.RcvWinProbe     );
    ConPrintf("  RcvAckTooMuch  = %010u  ", NDK_tcps.RcvAckTooMuch   );
    ConPrintf("  SndNoBufs      = %010u\n", NDK_tcps.SndNoBufs       );

    ConPrintf("  SndTotal       = %010u  ", NDK_tcps.SndTotal        );
    ConPrintf("  SndProbe       = %010u\n", NDK_tcps.SndProbe        );
    ConPrintf("  SndPack (data) = %010u  ", NDK_tcps.SndPack         );
    ConPrintf("  SndByte (data) = %010u\n", NDK_tcps.SndByte         );
    ConPrintf("  SndRexmitPack  = %010u  ", NDK_tcps.SndRexmitPack   );
    ConPrintf("  SndRexmitByte  = %010u\n", NDK_tcps.SndRexmitByte   );
    ConPrintf("  SndAcks        = %010u  ", NDK_tcps.SndAcks         );
    ConPrintf("  SndCtrl        = %010u\n", NDK_tcps.SndCtrl         );
    ConPrintf("  SndUrg         = %010u  ", NDK_tcps.SndUrg          );
    ConPrintf("  SndWinUp       = %010u\n", NDK_tcps.SndWinUp        );

    ConPrintf("  SegsTimed      = %010u  ", NDK_tcps.SegsTimed       );
    ConPrintf("  RttUpdated     = %010u\n", NDK_tcps.RttUpdated      );
    ConPrintf("  Connects       = %010u  ", NDK_tcps.Connects        );
    ConPrintf("  ConnAttempt    = %010u\n", NDK_tcps.ConnAttempt     );
    ConPrintf("  Drops          = %010u  ", NDK_tcps.Drops           );
    ConPrintf("  ConnDrops      = %010u\n", NDK_tcps.ConnDrops       );
    ConPrintf("  Accepts        = %010u  ", NDK_tcps.Accepts         );
    ConPrintf("  TimeoutDrops   = %010u\n", NDK_tcps.TimeoutDrops    );
    ConPrintf("  KeepDrops      = %010u  ", NDK_tcps.KeepDrops       );
    ConPrintf("  DelAck         = %010u\n", NDK_tcps.DelAck          );
    ConPrintf("  KeepProbe      = %010u  ", NDK_tcps.KeepProbe       );
    ConPrintf("  PersistTimeout = %010u\n", NDK_tcps.PersistTimeout  );
    ConPrintf("  KeepTimeout    = %010u  ", NDK_tcps.KeepTimeout     );
    ConPrintf("  RexmtTimeout   = %010u\n", NDK_tcps.RexmtTimeout    );
}

/*------------------------------------------------------------------------- */
/* DumpUDPStats() */
/* Dump out UDP/RAW Statistics */
/*------------------------------------------------------------------------- */
static void DumpUDPStats()
{
    ConPrintf("\nUDP Statistics:\n");
    ConPrintf("  RcvTotal       = %010u\n", NDK_udps.RcvTotal        );
    ConPrintf("  RcvShort       = %010u\n", NDK_udps.RcvShort        );
    ConPrintf("  RcvBadLen      = %010u\n", NDK_udps.RcvBadLen       );
    ConPrintf("  RcvBadSum      = %010u\n", NDK_udps.RcvBadSum       );
    ConPrintf("  RcvFull        = %010u\n", NDK_udps.RcvFull         );
    ConPrintf("  RcvNoPort      = %010u\n", NDK_udps.RcvNoPort       );
    ConPrintf("  RcvNoPortB     = %010u\n", NDK_udps.RcvNoPortB      );
    ConPrintf("  SndTotal       = %010u\n", NDK_udps.SndTotal        );
    ConPrintf("  SndNoPacket    = %010u\n", NDK_udps.SndNoPacket     );

    ConPrintf("\nRAW Statistics:\n");
    ConPrintf("  RcvTotal       = %010u\n", NDK_raws.RcvTotal        );
    ConPrintf("  RcvFull        = %010u\n", NDK_raws.RcvFull         );
    ConPrintf("  SndTotal       = %010u\n", NDK_raws.SndTotal        );
    ConPrintf("  SndNoPacket    = %010u\n", NDK_raws.SndNoPacket     );
}

/*------------------------------------------------------------------------- */
/* DumpNATStats() */
/* Dump out NAT Statistics */
/*------------------------------------------------------------------------- */
static void DumpNATStats()
{
#if NETSRV_ENABLE_NAT
    ConPrintf("\nNAT Statistics:\n");
    ConPrintf("  TxExamined     = %010u\n", NDK_nats.TxExamined      );
    ConPrintf("  TxQualified    = %010u\n", NDK_nats.TxQualified     );
    ConPrintf("  TxAltered      = %010u\n", NDK_nats.TxAltered       );
    ConPrintf("  RxExamined     = %010u\n", NDK_nats.RxExamined      );
    ConPrintf("  RxQualified    = %010u\n", NDK_nats.RxQualified     );
    ConPrintf("  RxAltered      = %010u\n", NDK_nats.RxAltered       );
    ConPrintf("  Entries        = %010u\n", NDK_nats.Entries         );
    ConPrintf("  MaxEntries     = %010u\n", NDK_nats.MaxEntries      );
    ConPrintf("  LongTerm       = %010u\n", NDK_nats.LongTerm        );
    ConPrintf("  MaxLongTerm    = %010u\n", NDK_nats.MaxLongTerm     );
#endif
}

