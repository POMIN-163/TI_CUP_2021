/*
 * Copyright (c) 2012-2019, Texas Instruments Incorporated
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
 * ======== netcfg.h ========
 *
 * Standard Configuration Structures
 *
 * Although the Configuration functions are generic, NETTOOLS expects
 * to be able to access a configuration with certain pre-defined
 * configuration tags.
 *
 * If a system needs to use the Service functions of NetTools, it must
 * provide service providers to implement the configuration specification
 * defined here.
 *
 */
/**
 *  @file  ti/ndk/inc/nettools/netcfg.h
 *
 *  @addtogroup ti_ndk_inc_nettools_inc__Cfg Configuration Manager
 */

#ifndef _C_NETCFG_INC
#define _C_NETCFG_INC

/*! @ingroup ti_ndk_inc_nettools_inc__Cfg */
/*! @{ */

#ifdef __cplusplus
extern "C" {
#endif

/* Note: *** Default Configuration Handle Required *** */
/* NetTools uses the default configuration handle for things like */
/* DHCP client, DHCP server, DNS, etc.. This handle should be */
/* set by calling CfgSetDefault() before using NetTools */

/*--------------------------------------------------------------------------- */
/* Defined Configuration Tags */
/*--------------------------------------------------------------------------- */
#define CFGTAG_OS               0x0001          /**< OS Configuration */
#define CFGTAG_IP               0x0002          /**< IP Stack Configuration */
#define CFGTAG_SERVICE          0x0003          /**< Service */
#define CFGTAG_IPNET            0x0004          /**< IP Network */
#define CFGTAG_ROUTE            0x0005          /**< Gateway Route */
#define CFGTAG_CLIENT           0x0006          /**< DHCPS Client */
#define CFGTAG_SYSINFO          0x0007          /**< System Information */
#define CFGTAG_ACCT             0x0008          /**< User Account */

/*!
 *  @brief      Maxiumum number of config tags
 *
 *  @remark     Users are allowed to add their own tags, which may require
 *              changing this value.  Note that changes to this value
 *              require rebuilding the NETTOOLS library.
 */
#define CFGTAG_MAX              0x0010

/*--------------------------------------------------------------------------- */
/* Configuration Item Values and Entry Data by Tag Value */
/*--------------------------------------------------------------------------- */

/*--------------------------------------------------------------------------- */
/* Config Tag: CFGTAG_SERVICE */
/*   List of active services */
/*     Item     = Service Type */
/*     Instance = Service Instance (1 to max) */
/* *** USERS CAN ADD THEIR OWN SERVICE ITEM VALUES *** */
#define CFGITEM_SERVICE_TELNET          0x0001
#define CFGITEM_SERVICE_RESERVED1       0x0002  /* was HTTP */
#define CFGITEM_SERVICE_NAT             0x0003
#define CFGITEM_SERVICE_DHCPSERVER      0x0004
#define CFGITEM_SERVICE_DHCPCLIENT      0x0005
#define CFGITEM_SERVICE_DNSSERVER       0x0006
#define CFGITEM_SERVICE_MAX             0x0006

/*!
 *  @brief      Common service arguments
 *
 *  All services share this common, base structure.
 */
typedef struct _ci_srvargs {
    /*!
     * This is a copy of the Item value used when the entry is added
     * to the configuration. Its initial value should be NULL, but it
     * is overwritten by the service provider callback. It is used so
     * that the status callback function can be provided with the
     * original Item value.
     */
    uint32_t    Item;

    /*!
     * This is the handle to the service as returned by the NETTOOLS
     * function corresponding to the type of service requested. Its
     * initial value should be NULL, and it is initialized by the
     * service callback function when the service is started. The
     * value is needed to shut down the service when the configuration
     * is unloaded.
     */
    void *hService;

    /*!
     * The mode parameter is a collection of flags representing the
     * desired execution behavior of the service. One or more of the
     * following flags can be set:
     *
     *    * #CIS_FLG_IFIDXVALID: Specifies the @c IfIdx field is valid.
     *    * #CIS_FLG_RESOLVEIP: Requests that @c IfIdx be resolved to an IP
     *      address before service execution is initiated.
     *    * #CIS_FLG_CALLBYIP: Specifies that the service should be invoked
     *      by IP address. (This is the default behavior when
     *      #CIS_FLG_IFIDXVALID is not set, but this flag can be set
     *      with #CIS_FLG_IFIDXVALID when #CIS_FLG_RESOLVEIP
     *      is also set. If #CIS_FLG_IFIDXVALID is set and this bit is not
     *      set, the service is invoked by physical device.)
     *    * #CIS_FLG_RESTARTIPTERM: A service that is dependent on a valid
     *      IP address (as determined by the #CIS_FLG_RESOLVEIP flag)
     *      is shut down if the IP address becomes invalid. When this
     *      flag is set, the service will be restarted when a new
     *      address becomes available. Otherwise; the service will
     *      not be restarted.
     */
    uint32_t    Mode;
#define CIS_FLG_IFIDXVALID      0x0001  /* IfIdx field is supplied to CONFIG */
#define CIS_FLG_RESOLVEIP       0x0002  /* Resolve If to IP before execution */
#define CIS_FLG_CALLBYIP        0x0004  /* Call using IP (set w/RESOLVEIP) */
#define CIS_FLG_RESTARTIPTERM   0x0008  /* Restart serivce on IPTERM */

    /*!
     *  The status parameter contains the service status as detected
     *  by the Net Control service callback function that initiates
     *  the service with NETTOOLS. The value of status should be
     *  initialized to NULL. Its defined values are:
     *     * #CIS_SRV_STATUS_DISABLED: Service not active (NULL state)
     *     * #CIS_SRV_STATUS_WAIT: Net Control is waiting on IP resolution
     *       to start service CIS_SRV_STATUS_IPTERM: Service was terminated
     *       because it lost its IP address
     *     * #CIS_SRV_STATUS_FAILED: Service failed to initialize via
     *       its NETTOOLS open function
     *     * #CIS_SRV_STATUS_ENABLED: Service enabled and initialized
     *       properly
    */
    uint32_t    Status;
#define CIS_SRV_STATUS_DISABLED 0x0000  /* Config not active */
#define CIS_SRV_STATUS_WAIT     0x0001  /* Waiting on IP resolve */
#define CIS_SRV_STATUS_IPTERM   0x0002  /* Service terminated via IP synch */
#define CIS_SRV_STATUS_FAILED   0x0003  /* Service failed to initialize */
#define CIS_SRV_STATUS_ENABLED  0x0004  /* Service enabled */

    /*!
     *  All the services available via the configuration can also be
     *  launched directly via a NETTOOLS API. The NETTOOLS service API
     *  has a standard service reporting callback function that is
     *  mirrored by the configuration system via the Net Control
     *  service provider callback. This variable holds the last report
     *  code reported by the NETTOOLS service invoked by this
     *  configuration entry.
     */
    uint32_t    ReportCode;

    /*!
     *  This is the physical device Index (1 to n) on which the
     *  service is to be executed. For example, when launching a DHCP
     *  server service, the physical interface is that connected to
     *  the home network. For more generic services (like Telnet), the
     *  service can be launched by a pre-defined IP address (or
     *  INADDR_ANY as a wildcard). When launching by IP address only,
     *  this field is left NULL. If the field is valid, the
     *  #CIS_FLG_IFIDXVALID flag should be set in @c Mode.
     */
    uint32_t    IfIdx;

    /*!
     *  This is the IP address (in network format) on which to
     *  initiate the service. This IP address can specify the wildcard
     *  INADDR_ANY, in which case the service will accept connections
     *  to any valid IP address on any device. Note that some services
     *  (like DHCP server) do not support being launched by an IP
     *  address and require a device Index (supplied in @c IfIdx) on
     *  which to execute.
     */
    uint32_t IPAddr;

    /*!
     *  Callback for status changes
     *
     *  @param  Item        Item value of entry changed
     *  @param  Status      New status
     *  @param  Code        Report code (if any)
     *  @param  hCfgEntry   Non-referenced pointer to entry with status change
     *
     *  @note   @c Status is the same as the @c Status field described
     *          in the CISARGS structure.
     *
     *  @note   @c Code is that returned by the NETTOOLS service
     *          callback, which is a lower-level status callback
     *          function used by Net Control.
     */
    void(*pCbSrv)(uint32_t Item, uint32_t Status, uint32_t Code,
            void *hCfgEntry);
} CISARGS;
/*! @} */

/*! @addtogroup ti_ndk_inc_nettools_inc__Telnet Telnet Service */
/*! @{ */
typedef struct _ci_service_telnet {
        CISARGS        cisargs;         /**< Common arguments */
        NTPARAM_TELNET param;           /**< Telnet parameters */
} CI_SERVICE_TELNET;
/*! @} */

/*! @addtogroup ti_ndk_inc_nettools_inc__NAT NAT Service */
/*! @{ */
typedef struct _ci_service_nat {
        CISARGS         cisargs;        /**< Common arguments */
        NTPARAM_NAT     param;          /**< NAT parameters */
} CI_SERVICE_NAT;
/*! @} */

/*! @addtogroup ti_ndk_inc_nettools_inc__DHCPS DHCP Server Service */
/*! @{ */
typedef struct _ci_service_dhcps {
        CISARGS         cisargs;        /* Common arguments */
        NTPARAM_DHCPS   param;          /* DHCPS parameters */
} CI_SERVICE_DHCPS;
/*! @} */

/*! @addtogroup ti_ndk_inc_nettools_inc__DHCPC DHCP Client Service*/
/*! @{ */
typedef struct _ci_service_dhcpc {
        CISARGS         cisargs;        /* Common arguments */
        NTPARAM_DHCP    param;          /* DHCP parameters */
} CI_SERVICE_DHCPC;
/*! @} */

/*! @addtogroup ti_ndk_inc_nettools_inc__DNSS DNS Server Service*/
/*! @{ */
typedef struct _ci_service_dnss {
        CISARGS         cisargs;        /* Common arguments */
} CI_SERVICE_DNSSERVER;
/*! @} */

/*! @addtogroup ti_ndk_inc_nettools_inc__Cfg Configuration Manager */
/*! @{ */

/*--------------------------------------------------------------------------- */
/* Config Tag: CFGTAG_IPNET */
/*   IP networks assigned to physical devices */
/*     Item     = Physical Interface Idx (1 to n) */
/*     Instance = Address Instance (1 to n) */

/* Max IPNet Domain name Length - Change requires NETTOOLS rebuild */
#define CFG_DOMAIN_MAX  64

/* IPNet Instance */
typedef struct _ci_ipnet {
        uint32_t NetType;               /* Network address type flags */
        uint32_t IPAddr;                 /* IP Address */
        uint32_t IPMask;                 /* Subnet Mask */
        void *hBind;                    /* Binding handle (resets to NULL) */
        char    Domain[CFG_DOMAIN_MAX]; /* IPNet Domain Name */
        } CI_IPNET;

/* NetType consists of flags. One or more of the following can be set... */
/* Note: VIRTUAL and non-VIRTUAL networks can not appear on the same interface */
#define CFG_NETTYPE_DYNAMIC     0x0001  /* Address created by DHCP CLIENT */
#define CFG_NETTYPE_VIRTUAL     0x0002  /* Virtual (one per IF) */
#define CFG_NETTYPE_DHCPS       0x0004  /* DHCPS Server IP */

/*--------------------------------------------------------------------------- */
/* Config Tag: CFGTAG_ROUTE */
/*   Static Gateway routes for hosts and networks */
/*     Item     = 0 */
/*     Instance = Route instance index (1 to n) */

/* Route Instance */
typedef struct _ci_route {
        uint32_t IPDestAddr;             /* Destination Network Address */
        uint32_t IPDestMask;             /* Subnet Mask of Destination */
        uint32_t IPGateAddr;             /* Gateway IP Address */
        void *hRoute;                   /* Route handle (resets to NULL) */
        } CI_ROUTE;

/*--------------------------------------------------------------------------- */
/* Config Tag: CFGTAG_CLIENT */
/*   List of DHCPS clients by manual entry or allocation from pool */
/*     Item     = Physical Interface Idx (1 to n) */
/*     Instance = Client Address Instance (1 to n) */
/*   Used by DHCPS and DNS server for local name resolution */

/* Client Instance */

/*
 *  Maximum size for DHCP client host name.
 *
 *  *** NOTE: changing this value requires a rebuild of NETTOOLS! ***
 *
 *  NOTE: The client hostname size is actually governed by 2 definitions:
 *      1) CFG_HOSTNAME_MAX (defined in netcfg.h)
 *      2) HOSTNAME_LENGTH (defined in dhcp.h)
 *  These two values must be consistent!  If one is changed, both must be
 *  changed.
 *
 *  RFC 2131 and 2132 do not clearly specify what the maximum size for this
 *  value should be.  255 was determined based on MSDN "Host Name Resolution for
 *  IPv4" on Microsoft's website (the size is 255 + 1 to make room for "\0").
 *  See notes for fix of CQ15114 for more details.
 *
 */
#define CFG_HOSTNAME_MAX     256

typedef struct _ci_client {
        uint32_t    ClientType;             /* Entry Status */
        uint32_t    Status;                 /* DHCPS Status (init to ZERO) */
        uint32_t IPAddr;                 /* Client IP Address */
        char    MacAddr[6];             /* Client Physical Address */
        char    Hostname[CFG_HOSTNAME_MAX]; /* Client Hostname */
        uint32_t  TimeStatus;             /* Time of last status msg (REQ/DEC/INF) */
        uint32_t  TimeExpire;             /* Expiration Time from TimeStatus */
        } CI_CLIENT;

/* ClientType and ClientStatus are values and can be one of the following... */
#define CFG_CLIENTTYPE_DYNAMIC     1    /* Entry created via DHCPS */
#define CFG_CLIENTTYPE_STATIC      2    /* Create manually */

#define CFG_CLIENTSTATUS_PENDING   1    /* Supplied by OFFER */
#define CFG_CLIENTSTATUS_VALID     2    /* Validated by REQUEST */
#define CFG_CLIENTSTATUS_STATIC    3    /* Given by a INFORM */
#define CFG_CLIENTSTATUS_INVALID   4    /* Invalidated by DECLINE */

/*--------------------------------------------------------------------------- */
/* Config Tag: CFGTAG_ACCT */
/*   List of user accounts for PPP or similar login */
/*     Item     = 1 for PPP or REALM */
/*     Instance = Client Address Instance (1 to n) */
/*   Used by PPP in server mode for authentication */
/*   Used by file system for REALM authentication */
/*   * Note all NDK accounts use the the same "item" value * */

/* Account Item Types */
#define CFGITEM_ACCT_SYSTEM     1
#define CFGITEM_ACCT_PPP        1
#define CFGITEM_ACCT_REALM      1

#define CFG_ACCTSTR_MAX         32

/* Account Instance */
typedef struct _ci_acct {
        uint32_t    Flags;                     /* Account Flags */
        char    Username[CFG_ACCTSTR_MAX]; /* Username */
        char    Password[CFG_ACCTSTR_MAX]; /* Password */
        } CI_ACCT;

/* Authority consists of flags. One or more of the following can be set... */
#define CFG_ACCTFLG_CH1         0x1000
#define CFG_ACCTFLG_CH2         0x2000
#define CFG_ACCTFLG_CH3         0x4000
#define CFG_ACCTFLG_CH4         0x8000
#define CFG_ACCTFLG_CHALL       0xF000

/*--------------------------------------------------------------------------- */
/* Config Tag: CFGTAG_SYSINFO */
/*   System information */
/*   This tag is for system information that is common for all system */
/*   tasks (DNS server, hostname, domainname, etc.). */
/*   Tag values less than 256 are reserved for DHCP */
/*   ( Item < 256 ) */
/*     (DHCP Info: Item numbers are reserved to match DHCP info tags) */
/*     Item     = DHCP Compatible Tag */
/*     Instance = Data Item Instance (1-n) */
/*   ( Item >= 256 ) */
/*     Item     = System Info Tag */
/*     Instance = Data Item Instance (1-n) */
/* *** USERS CAN ADD THEIR OWN ITEM VALUES *** */
/* For information, there is no structure - purely data */

/* Currently Used DHCP Compatible Items */
/* Multiple instances are always to be stored as multiple */
/* config entries, not a concatenated byte string in a */
/* single config entry. */
#define CFGITEM_DHCP_DOMAINNAMESERVER   6       /* Stack's DNS servers */
#define CFGITEM_DHCP_HOSTNAME           12      /* Stack's host name */
#define CFGITEM_DHCP_NBNS               44      /* Stack's NBNS servers */
#define CFGITEM_DHCP_CLIENT_OPTION		61		/* Stack DHCP Client Identifier */

#define CFGITEM_SYSINFO_REALM1          256     /* Realm Name 1 (max 31 chars) */
#define CFGITEM_SYSINFO_REALM2          257     /* Realm Name 2 (max 31 chars) */
#define CFGITEM_SYSINFO_REALM3          258     /* Realm Name 3 (max 31 chars) */
#define CFGITEM_SYSINFO_REALM4          259     /* Realm Name 4 (max 31 chars) */
#define CFGITEM_SYSINFO_REALMPPP        260     /* Realm Name PPP (max 31 chars) */

/*--------------------------------------------------------------------------- */
/* Config Tag: CFGTAG_IP, CFGTAG_OS */
/*   IP Stack Configuration / OS Configuration */
/*   The CFGTAG_IP tag is for setting configuration values in the */
/*   TCP/IP stack. */
/*   The CFGTAG_OS tag is for setting configuration values in the */
/*   system OS (or OS shell). */
/*   Write to either tag results in writes the the internal configuration */
/*   structure of the stack or OS, but being part of the configuration, the */
/*   entry can be stored off in as part of the CfgSave() functionality. */
/*   Removing an entry restores the default value to the configuration. */
/*   Entries that are not present can not be read, but an error on */
/*   read implies the entry is in its default state. */
/*   All items are of type "int" or "uint". */

/* When Tag = CFGTAG_IP, Item values are */
#define CFGITEM_IP_ICMPDOREDIRECT  1   /* Add route on ICMP redirect (1=Yes) */
#define CFGITEM_IP_ICMPTTL         2   /* TTL for ICMP msgs (RFC1700 says 64) */
#define CFGITEM_IP_ICMPTTLECHO     3   /* TTL for ICMP echo (RFC1700 says 64) */
#define CFGITEM_IP_IPINDEXSTART    4   /* IP Protocol Start Index */
#define CFGITEM_IP_IPFORWARDING    5   /* IP Forwarding Enable (1=Yes) */
#define CFGITEM_IP_IPNATENABLE     6   /* IP NAT Translation Enable (1=Yes) */
#define CFGITEM_IP_IPFILTERENABLE  7   /* IP Filtering Enable (1=Yes) */
#define CFGITEM_IP_IPREASMMAXTIME  8   /* Max IP reassembly time in seconds */
#define CFGITEM_IP_IPREASMMAXSIZE  9   /* Max IP reassembly packet size */
#define CFGITEM_IP_DIRECTEDBCAST   10  /* Directed BCast IP addresses (1=Yes) */
#define CFGITEM_IP_TCPREASMMAXPKT  11  /* Out of order pkts held by TCP socket */
#define CFGITEM_IP_RTCENABLEDEBUG  12  /* Route control dbg messages (1=Yes) */
#define CFGITEM_IP_RTCADVTIME      13  /* Seconds to send Router Adv. (0=don't) */
#define CFGITEM_IP_RTCADVLIFE      14  /* Lifetime of route in RtAdv if active */
#define CFGITEM_IP_RTCADVPREF      15  /* Preference of route in RvAdv if active */
#define CFGITEM_IP_RTARPDOWNTIME   16  /* Time 5 failed ARPs keeps route down */
#define CFGITEM_IP_RTKEEPALIVETIME 17  /* Timeout of validated route in seconds */
#define CFGITEM_IP_RTARPINACTIVITY 18  /* Time in seconds beyond which a route if */
                                       /* unused considered "inactive" and is cleaned up. */
#define CFGITEM_IP_RTCLONETIMEOUT  19  /* Timeout of new cloned route in seconds */
#define CFGITEM_IP_RTDEFAULTMTU    20  /* MTU for internal routes */
#define CFGITEM_IP_SOCKTTLDEFAULT  21  /* Default IP TTL for Sockets */
#define CFGITEM_IP_SOCKTOSDEFAULT  22  /* Default IP TOS for Sockets */
#define CFGITEM_IP_SOCKMAXCONNECT  23  /* Max connections on listening socket */
#define CFGITEM_IP_SOCKTIMECONNECT 24  /* Max time for connect socket */
#define CFGITEM_IP_SOCKTIMEIO      25  /* Default Max time for socket send/rcv */
#define CFGITEM_IP_SOCKTCPTXBUF    26  /* TCP Transmit buffer size */
#define CFGITEM_IP_SOCKTCPRXBUF    27  /* TCP Receive buffer size (copy mode) */
#define CFGITEM_IP_SOCKTCPRXLIMIT  28  /* TCP Receive limit (non-copy mode) */
#define CFGITEM_IP_SOCKUDPRXLIMIT  29  /* UDP Receive limit */
#define CFGITEM_IP_SOCKMINTX       30  /* Default min space for "able to write" */
#define CFGITEM_IP_SOCKMINRX       31  /* Default min data for "able to read" */
#define CFGITEM_IP_PIPETIMEIO      32  /* Max time for pipe send/rcv call */
#define CFGITEM_IP_PIPEBUFMAX      33  /* Pipe internal buffer size */
#define CFGITEM_IP_PIPEMINTX       34  /* Pipe min tx space for "able to write" */
#define CFGITEM_IP_PIPEMINRX       35  /* Pipe min rx data for "able to read" */
#define CFGITEM_IP_TCPKEEPIDLE     36  /* Idle time before 1st TCP keep probe */
#define CFGITEM_IP_TCPKEEPINTVL    37  /* TCP keep probe interval */
#define CFGITEM_IP_TCPKEEPMAXIDLE  38  /* Max TCP keep probing time before drop */
#define CFGITEM_IP_ICMPDONTREPLYBCAST  39  /* Dont Reply To ICMP ECHO REQ  */
                                       /* packets sent to BCast/Directed BCast  */
#define CFGITEM_IP_ICMPDONTREPLYMCAST  40  /* Dont Reply To ICMP ECHO REQ */
                                       /* packets sent to Multi-Cast */

#define CFGITEM_IP_RTGARP          41  /* How to handle received gratuitous ARP  */
                                       /* 0 : discard them. (Default)  */
                                       /* 1 : update MAC address of HOST, if it */
                                       /*     is already in the routing table. */
                                       /* 2 : add HOST to routing table, if it */
                                       /*     does NOT exist. If exists, update */
                                       /*     MAC address. */

#define CFGITEM_IP_ICMPDONTREPLYECHO  42  /* Don't Reply To ICMP ECHO REQ */
#define CFGITEM_IP_UDPSENDICMPPORTUNREACH 43  /* Send ICMP Port Unreachable */
                                       /* packet if UDP port is opened or not  */
#define CFGITEM_IP_TCPSENDRST      44  /* Send RST if TCP port is not    */
                                       /* opened or not.   */
#define CFGITEM_IP_SOCKRAWETHRXLIMIT  45  /* Raw Eth Receive limit */

#define CFGITEM_IP_MAX             46  /* Max CFGTAG_IP item */


/* When Tag = CFGTAG_OS, Item values are */
#define CFGITEM_OS_DBGPRINTLEVEL   1   /* Debug msg print threshhold */
#define CFGITEM_OS_DBGABORTLEVEL   2   /* Debug msg sys abort theshhold */
#define CFGITEM_OS_TASKPRILOW      3   /* Lowest priority for stack task */
#define CFGITEM_OS_TASKPRINORM     4   /* Normal priority for stack task */
#define CFGITEM_OS_TASKPRIHIGH     5   /* High priority for stack task */
#define CFGITEM_OS_TASKPRIKERN     6   /* Kernel-level priority (highest) */
#define CFGITEM_OS_TASKSTKLOW      7   /* Minimum stack size */
#define CFGITEM_OS_TASKSTKNORM     8   /* Normal stack size */
#define CFGITEM_OS_TASKSTKHIGH     9   /* Stack size for high volume tasks */
#define CFGITEM_OS_TASKSTKBOOT     10  /* Stack size for NS_BootTask */
#define CFGITEM_OS_MAX             10  /* Max CFGTAG_OS item */

/*! @} */
#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
