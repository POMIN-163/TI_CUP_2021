/*
 * Copyright (c) 2017-2019 Texas Instruments Incorporated
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
 *  ======== EMACMSP432E4.c ========
 *  This driver currently supports only 1 EMACMSP432E4 port. In the future
 *  when multiple ports are required, this driver needs to move all the
 *  EMACMSP432E4_Data fields into the EMACMSP432E4_Object structure. The APIs
 *  need to get to the fields via the pvt_data field in the NETIF_DEVICE that
 *  is passed in. ROV must be changed to support the change also.
 *  The NETIF_DEVICE structure should go into the EMACMSP432E4_Object also.
 *
 *  These changes are not being done at this time because of the impact on
 *  the complexity of the code.
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <ti/devices/msp432e4/inc/msp432.h>

#include <ti/devices/msp432e4/driverlib/emac.h>
#include <ti/devices/msp432e4/driverlib/flash.h>
#include <ti/devices/msp432e4/driverlib/gpio.h>
#include <ti/devices/msp432e4/driverlib/pin_map.h>
#include <ti/devices/msp432e4/driverlib/sysctl.h>

#include <ti/drivers/dpl/ClockP.h>
#include <ti/drivers/dpl/HwiP.h>

/* NDK pre-processor guard to avoid a POSIX dependency */
#define NDK_NOUSERAPIS 1

#include <ti/drivers/emac/EMACMSP432E4.h>
#include <ti/drivers/gpio/GPIOMSP432E4.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432E4.h>

#include <ti/ndk/inc/stkmain.h>

/*
 * Communicate using the internal PHY (address zero)
 *
 * This driver assumes an MSP432E4 board with the 128 pin QFP package (DID1
 * register's pin count field has value 0x6), which can only communicate with
 * the internal PHY.
 */
#define PHY_PHYS_ADDR 0

#define NUM_RX_DESCRIPTORS 4
#define NUM_TX_DESCRIPTORS 4
#define EMAC_PHY_CONFIG         (EMAC_PHY_TYPE_INTERNAL |                     \
                                 EMAC_PHY_INT_MDIX_EN |                       \
                                 EMAC_PHY_AN_100B_T_FULL_DUPLEX)

/* The size of the CRC stored at the end of the received frames */
#define CRC_SIZE_BYTES 4

/* The receive descriptor buffer size must be a multiple of 4 bytes */
#define RX_BUFFER_SIZE_MULTIPLE 4

#define RX_BUFFER_SIZE_ROUNDUP(X) ((((X) + \
        (RX_BUFFER_SIZE_MULTIPLE - 1)) / RX_BUFFER_SIZE_MULTIPLE) * \
        RX_BUFFER_SIZE_MULTIPLE)

/*
 * Define checksum related macros that are missing from driverlib
 *
 * The following bits in the RX DES0 descriptor have different meanings when
 * h/w checksum calculations are enabled. Define them here based on the default
 * macro values (default == h/w checksums disabled).
 */
/* RX DES0 bit 0 has payload checksum error status if EMACCFG IPC bit set */
#define EMAC_DES0_RX_STAT_PAYLOAD_CHKSM_ERR  DES0_RX_STAT_MAC_ADDR

/* RX DES0 bit 7 has IP header checksum error status if EMACCFG IPC bit set */
#define EMAC_DES0_RX_STAT_IPHDR_CHKSM_ERR  DES0_RX_STAT_TS_AVAILABLE

/*
 *  The buffer size for receive descriptors to allow for receipt of a maximum
 *  length Ethernet payload (ETH_MAX_PAYLOAD) allowing for:
 *
 *  - The CRC also being stored by the EMACMSP432E4 port
 *  - Rounding up the size to the multiple required by the EMACMSP432E4 port
 */
#define RX_BUFFER_SIZE RX_BUFFER_SIZE_ROUNDUP (ETH_MAX_PAYLOAD + CRC_SIZE_BYTES)

/*
 *  Helper struct holding a DMA descriptor and the pbuf it currently refers
 *  to.
 */
typedef struct {
    tEMACDMADescriptor Desc;
    PBM_Handle hPkt;
} tDescriptor;

typedef struct {
    tDescriptor *pDescriptors;
    uint32_t     ulNumDescs;
    uint32_t     ulWrite;
    uint32_t     ulRead;
} tDescriptorList;

/*
 * The struct is used to store the private data for the EMACMSP432E4 controller.
 */
typedef struct {
    STKEVENT_Handle  hEvent;
    PBMQ             PBMQ_tx;
    PBMQ             PBMQ_rx;
    uint32_t         rxCount;
    uint32_t         rxDropped;
    uint32_t         rxIpHdrChksmErrors;
    uint32_t         rxPayloadChksmErrors;
    uint32_t         txSent;
    uint32_t         txDropped;
    uint32_t         txIpHdrChksmErrors;
    uint32_t         txPayloadChksmErrors;
    uint32_t         pbmAllocErrors;
    uint32_t         descriptorLoopCount[NUM_RX_DESCRIPTORS];
    uint32_t         abnormalInts;
    uint32_t         isrCount;
    uint32_t         linkUp;
    tDescriptorList *pTxDescList;
    tDescriptorList *pRxDescList;
    HwiP_Handle      hwi;
} EMACMSP432E4_Data;

/*
 * Static global variables for this interface's private data.
 */
static tDescriptor g_pTxDescriptors[NUM_TX_DESCRIPTORS];
static tDescriptor g_pRxDescriptors[NUM_RX_DESCRIPTORS];

static tDescriptorList g_TxDescList = {
    g_pTxDescriptors, NUM_TX_DESCRIPTORS, 0, 0
};

static tDescriptorList g_RxDescList = {
    g_pRxDescriptors, NUM_RX_DESCRIPTORS, 0, 0
};

/* Application is required to provide this variable */
extern const EMACMSP432E4_HWAttrs EMACMSP432E4_hwAttrs;

/* Only supporting one EMACMSP432E4 device */
static EMACMSP432E4_Data EMACMSP432E4_private;

/* Funtion prototypes */
static void EMACMSP432E4_handleRx();
static void EMACMSP432E4_processTransmitted();
static int EMACMSP432E4_initDMADescriptors(void);
static int EMACMSP432E4_emacStart(struct NETIF_DEVICE* ptr_net_device);
static int EMACMSP432E4_emacStop(struct NETIF_DEVICE* ptr_net_device);

/*
 *  ======== signalLinkChange ========
 *  Signal the stack based on linkUp parameter.
 */
static void signalLinkChange(STKEVENT_Handle hEvent, uint32_t linkUp,
        uint32_t flag)
{
    if (linkUp) {
        /* Signal the stack that the link is up */
        STKEVENT_signal(hEvent, STKEVENT_LINKUP, flag);
    }
    else {
        /* Signal the stack that the link is down */
        STKEVENT_signal(hEvent, STKEVENT_LINKDOWN, flag);
    }
}

/*
 *  ======== EMACMSP432E4_processPendingTx ========
 */
static void EMACMSP432E4_processPendingTx()
{
    uint8_t     *pBuffer;
    uint32_t     len;
    PBM_Handle   hPkt;
    tDescriptor *pDesc;

    /*
     *  If there are pending packets, send one.
     *  Otherwise quit the loop.
     */
    hPkt = PBMQ_deq(&EMACMSP432E4_private.PBMQ_tx);
    if (hPkt != NULL) {

        pDesc = &(EMACMSP432E4_private.pTxDescList->pDescriptors[EMACMSP432E4_private.pTxDescList->ulWrite]);
        if (pDesc->hPkt) {
            PBM_free(hPkt);
            EMACMSP432E4_private.txDropped++;
            return;
        }

        /* Get the pointer to the buffer and the length */
        pBuffer = PBM_getDataBuffer(hPkt) + PBM_getDataOffset(hPkt);
        len = PBM_getValidLen(hPkt);

        /* Fill in the buffer pointer and length */
        pDesc->Desc.ui32Count = len;
        pDesc->Desc.pvBuffer1 = pBuffer;
        pDesc->Desc.ui32CtrlStatus = DES0_TX_CTRL_FIRST_SEG;

        pDesc->Desc.ui32CtrlStatus |=
                (DES0_TX_CTRL_IP_ALL_CKHSUMS | DES0_TX_CTRL_CHAINED);
        pDesc->Desc.ui32CtrlStatus |= (DES0_TX_CTRL_LAST_SEG |
                                       DES0_TX_CTRL_INTERRUPT);
        EMACMSP432E4_private.pTxDescList->ulWrite++;
        if (EMACMSP432E4_private.pTxDescList->ulWrite == NUM_TX_DESCRIPTORS) {
            EMACMSP432E4_private.pTxDescList->ulWrite = 0;
        }
        pDesc->hPkt = hPkt;
        pDesc->Desc.ui32CtrlStatus |= DES0_TX_CTRL_OWN;

        EMACMSP432E4_private.txSent++;

        EMACTxDMAPollDemand(EMAC0_BASE);
    }

    return;
}

/*
 *  ======== EMACMSP432E4_processTransmitted ========
 */
static void EMACMSP432E4_processTransmitted()
{
    tDescriptor *pDesc;
    uint32_t     ulNumDescs;

    /*
     * Walk the list until we have checked all descriptors or we reach the
     * write pointer or find a descriptor that the hardware is still working
     * on.
     */
    for (ulNumDescs = 0; ulNumDescs < NUM_TX_DESCRIPTORS; ulNumDescs++) {
        pDesc = &(EMACMSP432E4_private.pTxDescList->pDescriptors[EMACMSP432E4_private.pTxDescList->ulRead]);
        /* Has the buffer attached to this descriptor been transmitted? */
        if (pDesc->Desc.ui32CtrlStatus & DES0_TX_CTRL_OWN) {
            /* No - we're finished. */
            break;
        }

        /*
         * Check this descriptor for transmit errors
         *
         * First, check the error summary to see if there was any error and if
         * so, check to see what type of error it was.
         */
        if (pDesc->Desc.ui32CtrlStatus & DES0_TX_STAT_ERR) {
            /* An error occurred - now look for errors of interest */

            /*
             * Ensure TX Checksum Offload is enabled before checking for IP
             * header error status (this status bit is reserved when TX CS
             * offload is disabled)
             */
            if (((pDesc->Desc.ui32CtrlStatus & DES0_TX_CTRL_IP_HDR_CHKSUM) ||
                (pDesc->Desc.ui32CtrlStatus & DES0_TX_CTRL_IP_ALL_CKHSUMS)) &&
                (pDesc->Desc.ui32CtrlStatus & DES0_TX_STAT_IPH_ERR)) {
                /* Error inserting IP header checksum */
                EMACMSP432E4_private.txIpHdrChksmErrors++;
            }

            if (pDesc->Desc.ui32CtrlStatus & DES0_TX_STAT_PAYLOAD_ERR) {
                /* Error in IP payload checksum (i.e. in UDP, TCP or ICMP) */
                EMACMSP432E4_private.txPayloadChksmErrors++;
            }
        }

        /* Does this descriptor have a buffer attached to it? */
        if (pDesc->hPkt) {
            /* Yes - free it if it's not marked as an intermediate pbuf */
            if (!((uint32_t)(pDesc->hPkt) & 1)) {
                PBM_free(pDesc->hPkt);
            }
            pDesc->hPkt = NULL;
        }
        else {
            /* If the descriptor has no buffer, we are finished. */
            break;
        }

        /* Move on to the next descriptor. */
        EMACMSP432E4_private.pTxDescList->ulRead++;
        if (EMACMSP432E4_private.pTxDescList->ulRead == NUM_TX_DESCRIPTORS) {
            EMACMSP432E4_private.pTxDescList->ulRead = 0;
        }
    }
}

/*
 *  ======== EMACMSP432E4_primeRx ========
 */
static void EMACMSP432E4_primeRx(PBM_Handle hPkt, tDescriptor *desc)
{
    desc->hPkt = hPkt;
    desc->Desc.ui32Count = DES1_RX_CTRL_CHAINED;

    /* We got a buffer so fill in the payload pointer and size. */
    desc->Desc.pvBuffer1 = PBM_getDataBuffer(hPkt) + PBM_getDataOffset(hPkt);
    desc->Desc.ui32Count |= (RX_BUFFER_SIZE << DES1_RX_CTRL_BUFF1_SIZE_S);

    /* Give this descriptor back to the hardware */
    desc->Desc.ui32CtrlStatus = DES0_RX_CTRL_OWN;
}

/*
 *  ======== EMACMSP432E4_handleRx ========
 */
static void EMACMSP432E4_handleRx()
{
    PBM_Handle       hPkt;
    PBM_Handle       hPktNew;
    int32_t          len;
    tDescriptorList *pDescList;
    uint32_t         ulDescEnd;
    int32_t          descCount = -1;
    uint32_t         ui32Config;
    uint32_t         ui32Mode;
    uint32_t         ui32FrameSz;
    uint32_t         ui32CtrlStatus;

    /* Get a pointer to the receive descriptor list. */
    pDescList = EMACMSP432E4_private.pRxDescList;

    /* Determine where we start and end our walk of the descriptor list */
    ulDescEnd = pDescList->ulRead ?
            (pDescList->ulRead - 1) : (pDescList->ulNumDescs - 1);

    /* Step through the descriptors that are marked for CPU attention. */
    while (pDescList->ulRead != ulDescEnd) {
        descCount++;

        /* Does the current descriptor have a buffer attached to it? */
        hPkt = pDescList->pDescriptors[pDescList->ulRead].hPkt;
        if (hPkt) {
            ui32CtrlStatus = pDescList->pDescriptors[pDescList->ulRead].Desc.ui32CtrlStatus; 

            /* Determine if the host has filled it yet. */
            if (ui32CtrlStatus & DES0_RX_CTRL_OWN) {
                /* The DMA engine still owns the descriptor so we are finished. */
                break;
            }

            /* Yes - does the frame contain errors? */
            if (ui32CtrlStatus & DES0_RX_STAT_ERR) {
                /*
                 *  This is a bad frame. Update the relevant statistics and
                 *  then discard it.
                 */

                /*
                 * Check the EMAC configuration to see if RX h/w checksums are
                 * enabled. (The last 2 parameters are don't cares here)
                 */
                ui32Config = 0;
                EMACConfigGet(EMAC0_BASE, &ui32Config, &ui32Mode, &ui32FrameSz);
                if (ui32Config & EMAC_CONFIG_CHECKSUM_OFFLOAD) {
                    /* RX h/w checksums are enabled, look for checksum errors */

                    /* First check if the Frame Type bit is set */
                    if (ui32CtrlStatus & DES0_RX_STAT_FRAME_TYPE) {
                         /* Now, if bit 7 is reset and bit 0 is set: */
                         if (!(ui32CtrlStatus & EMAC_DES0_RX_STAT_IPHDR_CHKSM_ERR) &&
                             (ui32CtrlStatus & EMAC_DES0_RX_STAT_PAYLOAD_CHKSM_ERR)) {
                             /* Checksum error detected in the IP payload */
                             EMACMSP432E4_private.rxPayloadChksmErrors++;
                         }
                         /* Else if bit 7 and bit 0 are both set */
                         else if ((ui32CtrlStatus & EMAC_DES0_RX_STAT_IPHDR_CHKSM_ERR) &&
                             (ui32CtrlStatus & EMAC_DES0_RX_STAT_PAYLOAD_CHKSM_ERR)) {
                             /* Checksum error in both IP header & payload */
                             EMACMSP432E4_private.rxIpHdrChksmErrors++;
                             EMACMSP432E4_private.rxPayloadChksmErrors++;
                        }
                         
                    }
                }

                EMACMSP432E4_private.rxDropped++;
                EMACMSP432E4_primeRx(hPkt,
                        &(pDescList->pDescriptors[pDescList->ulRead]));
                pDescList->ulRead++;
                break;
            }
            else {
                /* Allocate a new buffer for this descriptor */
                hPktNew = PBM_alloc(RX_BUFFER_SIZE);
                if (hPktNew == NULL) {
                    /*
                     *  Leave the packet in the descriptor and owned by the
                     *  driver. Process when the next interrupt occurs.
                     */
                    EMACMSP432E4_private.pbmAllocErrors++;
                    break;
                }

                /* This is a good frame so pass it up the stack. */
                len = (pDescList->pDescriptors[pDescList->ulRead].Desc.ui32CtrlStatus &
                    DES0_RX_STAT_FRAME_LENGTH_M) >> DES0_RX_STAT_FRAME_LENGTH_S;

                /* Remove the CRC */
                PBM_setValidLen(hPkt, len - CRC_SIZE_BYTES);

                /*
                 *  Place the packet onto the receive queue to be handled in the
                 *  EMACMSP432E4_pkt_service function (which is called by the
                 *  NDK stack).
                 */
                PBMQ_enq(&EMACMSP432E4_private.PBMQ_rx, hPkt);

                /* Update internal statistic */
                EMACMSP432E4_private.rxCount++;

                /*
                 *  Notify NDK stack of pending Rx Ethernet packet and
                 *  that it was triggered by an external event.
                 */
                STKEVENT_signal(EMACMSP432E4_private.hEvent, STKEVENT_ETHERNET,
                        1);

                /* Prime the receive descriptor back up for future packets */
                EMACMSP432E4_primeRx(hPktNew,
                        &(pDescList->pDescriptors[pDescList->ulRead]));
            }
        }

        /* Move on to the next descriptor in the chain, taking care to wrap. */
        pDescList->ulRead++;
        if (pDescList->ulRead == pDescList->ulNumDescs) {
            pDescList->ulRead = 0;
        }
    }

    /*
     * Update the desciptorLoopCount. This shows how frequently we are cycling
     * through x DMA descriptors, where x is the index of this array. So if
     * descriptorLoopcount[1] = 32, we had to cycle through 2 descriptors here
     * 32 times.
     */
    if(descCount >= 0 && descCount < NUM_RX_DESCRIPTORS) {
        EMACMSP432E4_private.descriptorLoopCount[descCount]++;
    }
}

/*
 *  ======== EMACMSP432E4_processPhyInterrupt ========
 */
static void EMACMSP432E4_processPhyInterrupt()
{
    uint16_t value;
    uint16_t status;
    uint32_t config;
    uint32_t mode;
    uint32_t rxMaxFrameSize;

    /*
     * Read the PHY interrupt status.  This clears all interrupt sources.
     * Note that we are only enabling sources in EPHY_MISR1 so we don't
     * read EPHY_MISR2.
     */
    value = EMACPHYRead(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_MISR1);

    /* Read the current PHY status. */
    status = EMACPHYRead(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_STS);

    /* Has the link status changed? */
    if (value & EPHY_MISR1_LINKSTAT) {
        /* Is link up or down now? */
        if (status & EPHY_STS_LINK) {
            EMACMSP432E4_private.linkUp = true;
        }
        else {
            EMACMSP432E4_private.linkUp = false;
        }
        /* Signal the stack for this link status change (from ISR) */
        signalLinkChange(EMACMSP432E4_private.hEvent,
                EMACMSP432E4_private.linkUp, 1);
    }

    /* Has the speed or duplex status changed? */
    if (value & (EPHY_MISR1_SPEED | EPHY_MISR1_DUPLEXM | EPHY_MISR1_ANC)) {
        /* Get the current MAC configuration. */
        EMACConfigGet(EMAC0_BASE, (uint32_t *)&config, (uint32_t *)&mode,
                        (uint32_t *)&rxMaxFrameSize);

        /* What speed is the interface running at now? */
        if (status & EPHY_STS_SPEED) {
            /* 10Mbps is selected */
            config &= ~EMAC_CONFIG_100MBPS;
        }
        else {
            /* 100Mbps is selected */
            config |= EMAC_CONFIG_100MBPS;
        }

        /* Are we in full- or half-duplex mode? */
        if (status & EPHY_STS_DUPLEX) {
            /* Full duplex. */
            config |= EMAC_CONFIG_FULL_DUPLEX;
        }
        else {
            /* Half duplex. */
            config &= ~EMAC_CONFIG_FULL_DUPLEX;
        }

        /* Reconfigure the MAC */
        EMACConfigSet(EMAC0_BASE, config, mode, rxMaxFrameSize);
    }
}

/*
 *  ======== EMACMSP432E4_hwiIntFxn ========
 */
static void EMACMSP432E4_hwiIntFxn(uintptr_t callbacks)
{
    uint32_t status;
    bool newLinkStatus;

    /* Check link status */
    newLinkStatus = (EMACPHYRead(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_BMSR) &
        EPHY_BMSR_LINKSTAT) ? true : false;

    /* Signal the stack if link status changed */
    if (newLinkStatus != EMACMSP432E4_private.linkUp) {
        signalLinkChange(EMACMSP432E4_private.hEvent, newLinkStatus, 1);
    }

    /* Set the link status */
    EMACMSP432E4_private.linkUp = newLinkStatus;

    EMACMSP432E4_private.isrCount++;

    /* Read and Clear the interrupt. */
    status = EMACIntStatus(EMAC0_BASE, true);
    EMACIntClear(EMAC0_BASE, status);

    /*
     *  Disable the Ethernet interrupts.  Since the interrupts have not been
     *  handled, they are not asserted.  Once they are handled by the Ethernet
     *  interrupt, it will re-enable the interrupts.
     */
    EMACIntDisable(EMAC0_BASE, (EMAC_INT_RECEIVE | EMAC_INT_TRANSMIT |
                     EMAC_INT_TX_STOPPED | EMAC_INT_RX_NO_BUFFER |
                     EMAC_INT_RX_STOPPED | EMAC_INT_PHY));

    if (status & EMAC_INT_ABNORMAL_INT) {
        EMACMSP432E4_private.abnormalInts++;
    }

    if (status & EMAC_INT_PHY) {
        EMACMSP432E4_processPhyInterrupt();
    }

    /* Process the transmit DMA list, freeing any buffers that have been
     * transmitted since our last interrupt.
     */
    if (status & EMAC_INT_TRANSMIT) {
        EMACMSP432E4_processTransmitted();
    }

    /*
     * Process the receive DMA list and pass all successfully received packets
     * up the stack.  We also call this function in cases where the receiver has
     * stalled due to missing buffers since the receive function will attempt to
     * allocate new pbufs for descriptor entries which have none.
     */
    if (status & (EMAC_INT_RECEIVE | EMAC_INT_RX_NO_BUFFER |
        EMAC_INT_RX_STOPPED)) {
        EMACMSP432E4_handleRx();
    }

    EMACIntEnable(EMAC0_BASE, (EMAC_INT_RECEIVE | EMAC_INT_TRANSMIT |
                        EMAC_INT_TX_STOPPED | EMAC_INT_RX_NO_BUFFER |
                        EMAC_INT_RX_STOPPED | EMAC_INT_PHY));
}

/*
 *  ======== EMACMSP432E4_emacStart ========
 *  The function is used to initialize and start the EMACMSP432E4
 *  controller and device.
 */
static int EMACMSP432E4_emacStart(struct NETIF_DEVICE* ptr_net_device)
{
    uint16_t value;
    EMACMSP432E4_HWAttrs const *hwAttrs = &EMACMSP432E4_hwAttrs;
    HwiP_Params hwiParams;
    ClockP_FreqHz freq;
    uint32_t ui32FlashConf, key;
    uint32_t pinMap;
    uint8_t  port;
    uint8_t  pin;
    bool enablePrefetch = false;

    /* set power dependency on peripherals being used */
    Power_setDependency(PowerMSP432E4_PERIPH_EMAC0);
    Power_setDependency(PowerMSP432E4_PERIPH_EPHY0);

    /* Configure the appropriate pins for ethernet led0 */
    pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->led0Pin);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->led0Pin);
    pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->led0Pin);

    Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
    GPIOPinConfigure(pinMap);
    GPIOPinTypeEthernetLED(GPIOMSP432E4_getGpioBaseAddr(port), pin);

    /* Configure the appropriate pins for ethernet led1 */
    pin = GPIOMSP432E4_getPinFromPinConfig(hwAttrs->led1Pin);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->led1Pin);
    pinMap = GPIOMSP432E4_getPinMapFromPinConfig(hwAttrs->led1Pin);

    Power_setDependency(GPIOMSP432E4_getPowerResourceId(port));
    GPIOPinConfigure(pinMap);
    GPIOPinTypeEthernetLED(GPIOMSP432E4_getGpioBaseAddr(port), pin);

    ClockP_getCpuFreq(&freq);

    key = HwiP_disable();

    /*
     *  This is a work-around for the EMAC initialization issue found
     *  on the TM4C129 devices. This can be found in the silicon errata
     *  documentation spmz850g.pdf as ETH#02.
     *
     *  The following disables the flash pre-fetch (if it is not already
     *  disabled).
     */
    ui32FlashConf = FLASH_CTRL->CONF;
    if ((ui32FlashConf & (FLASH_CONF_FPFOFF)) == false) {
        enablePrefetch = true;
        ui32FlashConf &= ~(FLASH_CONF_FPFON);
        ui32FlashConf |= FLASH_CONF_FPFOFF;
        FLASH_CTRL->CONF = ui32FlashConf;
    }

    EMACPHYConfigSet(EMAC0_BASE, EMAC_PHY_CONFIG);

    EMACInit(EMAC0_BASE, freq.lo,
             EMAC_BCONFIG_MIXED_BURST | EMAC_BCONFIG_PRIORITY_FIXED, 4, 4, 0);

    /* Set MAC configuration options. */
    EMACConfigSet(EMAC0_BASE, (EMAC_CONFIG_FULL_DUPLEX |
                               EMAC_CONFIG_7BYTE_PREAMBLE |
                               EMAC_CONFIG_IF_GAP_96BITS |
                               EMAC_CONFIG_USE_MACADDR0 |
                               EMAC_CONFIG_SA_FROM_DESCRIPTOR |
                               /* Enable RX Checksum Offload: */
                               EMAC_CONFIG_CHECKSUM_OFFLOAD |
                               EMAC_CONFIG_BO_LIMIT_1024),
                  (EMAC_MODE_RX_STORE_FORWARD |
                   EMAC_MODE_TX_STORE_FORWARD |
                   EMAC_MODE_TX_THRESHOLD_64_BYTES |
                   EMAC_MODE_RX_THRESHOLD_64_BYTES), 0);

    /* Program the MAC address into the Ethernet controller. */
    EMACAddrSet(EMAC0_BASE, 0, (uint8_t *)hwAttrs->macAddress);

    /* Initialize the DMA descriptors. */
    if (EMACMSP432E4_initDMADescriptors() < 0) {
        /*
         *  If fail to initialize DMA descriptor lists:
         *  1. Turns ON the prefetch buffer if it was disabled above
         *  2. call HwiP_restore
         *  3. call emacStop to clean up
         */
        if (enablePrefetch) {
            ui32FlashConf = FLASH_CTRL->CONF;
            ui32FlashConf &= ~(FLASH_CONF_FPFOFF);
            ui32FlashConf |= FLASH_CONF_FPFON;
            FLASH_CTRL->CONF = ui32FlashConf;
        }

        HwiP_restore(key);
        EMACMSP432E4_emacStop(ptr_net_device);

        return (-1);
    }

    /* Clear any stray PHY interrupts that may be set. */
    value = EMACPHYRead(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_MISR1);
    value = EMACPHYRead(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_MISR2);

    /* Configure and enable the link status change interrupt in the PHY. */
    value = EMACPHYRead(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_SCR);
    value |= (EPHY_SCR_INTEN_EXT | EPHY_SCR_INTOE_EXT);
    EMACPHYWrite(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_SCR, value);
    EMACPHYWrite(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_MISR1, (EPHY_MISR1_LINKSTATEN |
                 EPHY_MISR1_SPEEDEN | EPHY_MISR1_DUPLEXMEN | EPHY_MISR1_ANCEN));

    /* Read the PHY interrupt status to clear any stray events. */
    value = EMACPHYRead(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_MISR1);

    /*
     *  Set MAC filtering options.  We receive all broadcast and multicast
     *  packets along with those addressed specifically for us.
     */
    EMACFrameFilterSet(EMAC0_BASE, (EMAC_FRMFILTER_HASH_AND_PERFECT |
                       EMAC_FRMFILTER_PASS_MULTICAST));

    /* Clear any pending interrupts. */
    EMACIntClear(EMAC0_BASE, EMACIntStatus(EMAC0_BASE, false));

    /* Enable the Ethernet MAC transmitter and receiver. */
    EMACTxEnable(EMAC0_BASE);
    EMACRxEnable(EMAC0_BASE);

    /* Enable the Ethernet RX and TX interrupt source. */
    EMACIntEnable(EMAC0_BASE, (EMAC_INT_RECEIVE | EMAC_INT_TRANSMIT |
                  EMAC_INT_TX_STOPPED | EMAC_INT_RX_NO_BUFFER |
                  EMAC_INT_RX_STOPPED | EMAC_INT_PHY));

    EMACPHYWrite(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_BMCR, (EPHY_BMCR_ANEN |
                 EPHY_BMCR_RESTARTAN));

    /*
     * Turns ON the prefetch buffer if it was disabled above
     */
    if (enablePrefetch) {
        ui32FlashConf = FLASH_CTRL->CONF;
        ui32FlashConf &= ~(FLASH_CONF_FPFOFF);
        ui32FlashConf |= FLASH_CONF_FPFON;
        FLASH_CTRL->CONF = ui32FlashConf;
    }

    HwiP_restore(key);

    /* Create the hardware interrupt */
    HwiP_Params_init(&hwiParams);
    hwiParams.priority = hwAttrs->intPriority;

    EMACMSP432E4_private.hwi = HwiP_create(hwAttrs->intNum,
                                   EMACMSP432E4_hwiIntFxn,
                                   &hwiParams);

    if (EMACMSP432E4_private.hwi == NULL) {
        EMACMSP432E4_emacStop(ptr_net_device);
        return (-1);
    }

    return (0);
}

/*
 *  ======== EMACMSP432E4_emacStop ========
 *  The function is used to de-initialize and stop the EMACMSP432E4
 *  controller and device.
 */
static int EMACMSP432E4_emacStop(struct NETIF_DEVICE* ptr_net_device)
{
    EMACMSP432E4_HWAttrs const *hwAttrs = &EMACMSP432E4_hwAttrs;
    PBM_Handle hPkt;
    uint8_t  port;

    int i = 0;

    EMACIntDisable(EMAC0_BASE, (EMAC_INT_RECEIVE | EMAC_INT_TRANSMIT |
                     EMAC_INT_TX_STOPPED | EMAC_INT_RX_NO_BUFFER |
                     EMAC_INT_RX_STOPPED | EMAC_INT_PHY));

    if (EMACMSP432E4_private.hwi != NULL) {
        HwiP_delete(EMACMSP432E4_private.hwi);
    }

    while (PBMQ_count(&EMACMSP432E4_private.PBMQ_rx)) {
        /* Dequeue a packet from the driver receive queue. */
        hPkt = PBMQ_deq(&EMACMSP432E4_private.PBMQ_rx);
        PBM_free(hPkt);
    }

    while (PBMQ_count(&EMACMSP432E4_private.PBMQ_tx)) {
        /* Dequeue a packet from the driver receive queue. */
        hPkt = PBMQ_deq(&EMACMSP432E4_private.PBMQ_tx);
        PBM_free(hPkt);
    }

    for (i = 0; i < NUM_RX_DESCRIPTORS; i++) {
        if (g_pRxDescriptors[i].hPkt != NULL) {
            PBM_free(g_pRxDescriptors[i].hPkt);
        }
    }

    GPIOMSP432E4_undoPinConfig(hwAttrs->led0Pin);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->led0Pin);
    Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));

    GPIOMSP432E4_undoPinConfig(hwAttrs->led1Pin);
    port = GPIOMSP432E4_getPortFromPinConfig(hwAttrs->led1Pin);
    Power_releaseDependency(GPIOMSP432E4_getPowerResourceId(port));

    Power_releaseDependency(PowerMSP432E4_PERIPH_EPHY0);
    Power_releaseDependency(PowerMSP432E4_PERIPH_EMAC0);

    return (0);
}

/*
 *  ======== EMACMSP432E4_emacPoll ========
 *  The function is used to poll the EMACMSP432E4 controller to check
 *  if there has been any activity
 */
static void EMACMSP432E4_emacPoll(struct NETIF_DEVICE* ptr_net_device,
        uint32_t timer_tick)
{
    bool newLinkStatus;

    /* Send pending Tx packets */
    EMACMSP432E4_processPendingTx();

    /* Check link status */
    newLinkStatus = (EMACPHYRead(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_BMSR) &
        EPHY_BMSR_LINKSTAT) ? true : false;

    /* Signal the stack if link status changed */
    if (newLinkStatus != EMACMSP432E4_private.linkUp) {
        signalLinkChange(EMACMSP432E4_private.hEvent, newLinkStatus, 0);
    }

    /* Set the link status */
    EMACMSP432E4_private.linkUp = newLinkStatus;
}

/*
 *  ======== EMACMSP432E4_emacSend ========
 *  The function is the interface routine invoked by the NDK stack to
 *  pass packets to the driver.
 */
static int EMACMSP432E4_emacSend(struct NETIF_DEVICE* ptr_net_device, PBM_Handle hPkt)
{
    /*
     *  Enqueue the packet onto the end of the transmit queue.
     *  This is done to ensure that the packets are sent in order.
     */
    PBMQ_enq(&EMACMSP432E4_private.PBMQ_tx, hPkt);

    /* Transmit pending packets */
    EMACMSP432E4_processPendingTx();

    return (0);
}

/*
 *  ======== EMACMSP432E4_emacioctl ========
 *  The function is called by the NDK core stack to configure the driver
 */
static int EMACMSP432E4_emacioctl(struct NETIF_DEVICE* ptr_net_device,
        uint32_t cmd, void* pbuf, uint32_t size)
{
    int retval;

    switch (cmd) {
        case NIMU_ADD_MULTICAST_ADDRESS:
        case NIMU_DEL_MULTICAST_ADDRESS:
            retval = 0;
            break;

        case NIMU_GET_DEVICE_ISLINKUP:
            if (size >= sizeof(uint32_t)) {
                if (EMACMSP432E4_isLinkUp()) {
                    *(uint32_t *)pbuf = 1;
                }
                else {
                    *(uint32_t *)pbuf = 0;
                }
                retval = 0;
            }
            else {
                /* user-provided buffer is too small */
                retval = -(NDK_EINVAL);
            }

            break;

        default:
            retval = -(NDK_EINVAL);
            break;
    }

    return (retval);
}

/*
 *  ======== EMACMSP432E4_pkt_service ========
 *  The function is called by the NDK core stack to receive any packets
 *  from the driver.
 */
static void EMACMSP432E4_pkt_service(NETIF_DEVICE* ptr_net_device)
{
    PBM_Handle  hPkt;

    /* Give all queued packets to the stack */
    while (PBMQ_count(&EMACMSP432E4_private.PBMQ_rx)) {

        /* Dequeue a packet from the driver receive queue. */
        hPkt = PBMQ_deq(&EMACMSP432E4_private.PBMQ_rx);

        /*
         *  Prepare the packet so that it can be passed up the networking stack.
         *  If this 'step' is not done the fields in the packet are not correct
         *  and the packet will eventually be dropped.
         */
        PBM_setIFRx(hPkt, ptr_net_device);

        /* Pass the packet to the NDK Core stack. */
        NIMUReceivePacket(hPkt);
    }

    /* Work has been completed; the receive queue is empty. */
    return;
}

/*
 *  ======== EMACMSP432E4_initDMADescriptors ========
 * Initialize the transmit and receive DMA descriptor lists.
 */
static int EMACMSP432E4_initDMADescriptors(void)
{
    int32_t     i;
    PBM_Handle  hPkt;

    /* Reset DMA descriptor lists' indexes to 0 */
    EMACMSP432E4_private.pTxDescList->ulRead = 0;
    EMACMSP432E4_private.pTxDescList->ulWrite = 0;
    EMACMSP432E4_private.pRxDescList->ulRead = 0;
    EMACMSP432E4_private.pRxDescList->ulWrite = 0;

    /* Transmit list -  mark all descriptors as not owned by the hardware */
    for (i = 0; i < NUM_TX_DESCRIPTORS; i++) {
        g_pTxDescriptors[i].hPkt = NULL;
        g_pTxDescriptors[i].Desc.ui32Count = 0;
        g_pTxDescriptors[i].Desc.pvBuffer1 = 0;
        g_pTxDescriptors[i].Desc.DES3.pLink = ((i == (NUM_TX_DESCRIPTORS - 1)) ?
               &g_pTxDescriptors[0].Desc : &g_pTxDescriptors[i + 1].Desc);
        g_pTxDescriptors[i].Desc.ui32CtrlStatus = DES0_TX_CTRL_INTERRUPT |
                DES0_TX_CTRL_IP_ALL_CKHSUMS |
                DES0_TX_CTRL_CHAINED;
    }

    /*
     * Receive list - tag each descriptor with a buffer and set all fields to
     * allow packets to be received.
     */
    for (i = 0; i < NUM_RX_DESCRIPTORS; i++) {
        hPkt = PBM_alloc(RX_BUFFER_SIZE);
        if (hPkt) {
            EMACMSP432E4_primeRx(hPkt, &(g_pRxDescriptors[i]));
        }
        else {
            /*
             *  This is a failing scenario return -1.
             *  emacStop will do the PBM_free for any allocated packet.
             */
            g_pRxDescriptors[i].Desc.pvBuffer1 = 0;
            g_pRxDescriptors[i].Desc.ui32CtrlStatus = 0;
            return (-1);
        }
        g_pRxDescriptors[i].Desc.DES3.pLink =
                ((i == (NUM_RX_DESCRIPTORS - 1)) ?
                &g_pRxDescriptors[0].Desc : &g_pRxDescriptors[i + 1].Desc);
    }

    /* Set the descriptor pointers in the hardware. */
    EMACRxDMADescriptorListSet(EMAC0_BASE, &g_pRxDescriptors[0].Desc);
    EMACTxDMADescriptorListSet(EMAC0_BASE, &g_pTxDescriptors[0].Desc);

    return (0);
}

/*
 *  ======== EMACMSP432E4_NIMUInit ========
 *  The function is used to initialize and register the EMACMSP432E4
 *  with the Network Interface Management Unit (NIMU)
 */
int EMACMSP432E4_NIMUInit(STKEVENT_Handle hEvent)
{
    EMACMSP432E4_HWAttrs const *hwAttrs = &EMACMSP432E4_hwAttrs;
    NETIF_DEVICE *device;
    uint32_t ulUser0, ulUser1;

    if (hwAttrs->macAddress[0] == 0xff && hwAttrs->macAddress[1] == 0xff &&
        hwAttrs->macAddress[2] == 0xff && hwAttrs->macAddress[3] == 0xff &&
        hwAttrs->macAddress[4] == 0xff && hwAttrs->macAddress[5] == 0xff) {

        /* Get the MAC address from flash */
        FlashUserGet(&ulUser0, &ulUser1);
        if ((ulUser0 != 0xffffffff) && (ulUser1 != 0xffffffff)) {
            /*
             *  Convert the 24/24 split MAC address from NV ram into a 32/16
             *  split MAC address needed to program the hardware registers, then
             *  program the MAC address into the Ethernet Controller registers.
             */
            hwAttrs->macAddress[0] = ((ulUser0 >>  0) & 0xff);
            hwAttrs->macAddress[1] = ((ulUser0 >>  8) & 0xff);
            hwAttrs->macAddress[2] = ((ulUser0 >> 16) & 0xff);
            hwAttrs->macAddress[3] = ((ulUser1 >>  0) & 0xff);
            hwAttrs->macAddress[4] = ((ulUser1 >>  8) & 0xff);
            hwAttrs->macAddress[5] = ((ulUser1 >> 16) & 0xff);
        }
        else {
            /* MAC address was not specified */
            return (-1);
        }
    } /* else, use the MAC address set by the user in board.c  */

    /* Initialize the global structures */
    memset(&EMACMSP432E4_private, 0, sizeof(EMACMSP432E4_Data));

    /* Allocate memory for the EMAC. Memory freed in the NDK stack shutdown */
    device = mmAlloc(sizeof(NETIF_DEVICE));
    if (device == NULL) {
        return (-1);
    }

    /* Initialize the allocated memory block. */
    mmZeroInit(device, sizeof(NETIF_DEVICE));

    device->mac_address[0] = hwAttrs->macAddress[0];
    device->mac_address[1] = hwAttrs->macAddress[1];
    device->mac_address[2] = hwAttrs->macAddress[2];
    device->mac_address[3] = hwAttrs->macAddress[3];
    device->mac_address[4] = hwAttrs->macAddress[4];
    device->mac_address[5] = hwAttrs->macAddress[5];

    /* Initialize the Packet Device Information struct */
    PBMQ_init(&EMACMSP432E4_private.PBMQ_rx);
    PBMQ_init(&EMACMSP432E4_private.PBMQ_tx);
    EMACMSP432E4_private.hEvent       = hEvent;
    EMACMSP432E4_private.pTxDescList  = &g_TxDescList;
    EMACMSP432E4_private.pRxDescList  = &g_RxDescList;
    EMACMSP432E4_private.rxCount      = 0;
    EMACMSP432E4_private.rxDropped    = 0;
    EMACMSP432E4_private.rxIpHdrChksmErrors = 0;
    EMACMSP432E4_private.rxPayloadChksmErrors = 0;
    EMACMSP432E4_private.txSent       = 0;
    EMACMSP432E4_private.txDropped    = 0;
    EMACMSP432E4_private.txIpHdrChksmErrors = 0;
    EMACMSP432E4_private.txPayloadChksmErrors = 0;
    EMACMSP432E4_private.pbmAllocErrors = 0;
    EMACMSP432E4_private.abnormalInts = 0;
    EMACMSP432E4_private.isrCount = 0;
    EMACMSP432E4_private.linkUp       = false;
    memset(EMACMSP432E4_private.descriptorLoopCount, 0,
            sizeof(EMACMSP432E4_private.descriptorLoopCount));

    /* Populate the Network Interface Object. */
    strcpy(device->name, EMACMSP432E4_ETHERNET_NAME);
    device->mtu         = ETH_MAX_PAYLOAD - ETHHDR_SIZE;
    device->pvt_data    = (void *)&EMACMSP432E4_private;

    /* Inform upper layers that this driver enables Checksum Offloading */
    device->flags = NIMU_DEVICE_ENABLE_HW_CHKSM_TX_ALL |
                    NIMU_DEVICE_ENABLE_HW_CHKSM_RX_ALL;

    /* Populate the Driver Interface Functions. */
    device->start       = EMACMSP432E4_emacStart;
    device->stop        = EMACMSP432E4_emacStop;
    device->poll        = EMACMSP432E4_emacPoll;
    device->send        = EMACMSP432E4_emacSend;
    device->pkt_service = EMACMSP432E4_pkt_service;
    device->ioctl       = EMACMSP432E4_emacioctl;
    device->add_header  = NIMUAddEthernetHeader;

    /* Register the device with NIMU */
    if (NIMURegister(device) < 0) {
        return (-1);
    }

    return (0);
}

/*
 *  ======== EMACMSP432E4_linkUp ========
 */
bool EMACMSP432E4_isLinkUp()
{
    bool newLinkStatus;

    /* Check link status */
    newLinkStatus = (EMACPHYRead(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_BMSR) &
        EPHY_BMSR_LINKSTAT) ? true : false;

    /* Signal the stack if link status changed */
    if (newLinkStatus != EMACMSP432E4_private.linkUp) {
        signalLinkChange(EMACMSP432E4_private.hEvent, newLinkStatus, 0);
    }

    /* Set the link status */
    EMACMSP432E4_private.linkUp = newLinkStatus;

    if (EMACMSP432E4_private.linkUp) {
        return (true);
    }
    else {
        return (false);
    }
}
