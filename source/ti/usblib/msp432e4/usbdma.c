//*****************************************************************************
//
// usbdma.c - USB Library DMA handling functions.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "ti/devices/msp432e4/driverlib/driverlib.h"
#include "ti/devices/msp432e4/driverlib/rtos_bindings.h"
#include "usblib.h"
#include "usblibpriv.h"

//*****************************************************************************
//
//! \addtogroup usblib_dma_api Internal USB DMA functions
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// There are two sets of functions in this file, one is used with uDMA while
// the other is used with USB controllers with an integrated DMA controller.
// The functions with the IDMA prefix are for the integrated DMA controller and
// the functions that are specific to the uDMA controller are prefixed with
// uDMA.  Any common functions are have just the DMA prefix.
//
//*****************************************************************************
static tUSBDMAInstance g_psUSBDMAInst[1];

//*****************************************************************************
//
// Macros used to determine if a uDMA endpoint configuration is used for
// receive or transmit.
//
//*****************************************************************************
#define UDMAConfigIsRx(ui32Config)                                            \
        ((ui32Config & UDMA_SRC_INC_NONE) == UDMA_SRC_INC_NONE)
#define UDMAConfigIsTx(ui32Config)                                            \
        ((ui32Config & UDMA_DEST_INC_NONE) == UDMA_DEST_INC_NONE)



//*****************************************************************************
//
// USBLibDMAChannelStatus() for USB controllers with an integrated DMA
// controller.
//
//*****************************************************************************
static uint32_t
iDMAUSBChannelStatus(tUSBDMAInstance *psUSBDMAInst, uint32_t ui32Channel)
{
    uint32_t ui32Status;

    //
    // Initialize the current status to no events.
    //
    ui32Status = USBLIBSTATUS_DMA_IDLE;

    //
    // Check if an error has occurred.
    //
    if(USBDMAChannelStatus(psUSBDMAInst->ui32Base, ui32Channel) ==
       USB_DMA_STATUS_ERROR)
    {
        ui32Status = USBLIBSTATUS_DMA_ERROR;
    }
    //
    // Otherwise check if there a pending DMA transfer has completed.
    //
    else if(psUSBDMAInst->ui32Complete & (1 << (ui32Channel - 1)))
    {
        //
        // Return that the DMA transfer has completed and clear the
        // DMA pending flag.
        //
        ui32Status = USBLIBSTATUS_DMA_COMPLETE;
    }
    else if(psUSBDMAInst->ui32Pending & (1 << (ui32Channel - 1)))
    {
        //
        // DMA transfer is still pending.
        //
        ui32Status = USBLIBSTATUS_DMA_PENDING;
    }
    else
    {
        //
        // DMA Channel is idle.
        //
        ui32Status = USBLIBSTATUS_DMA_IDLE;
    }

    return(ui32Status);
}



//*****************************************************************************
//
// USBLibDMAIntStatus() for USB controllers with an integrated DMA controller.
//
//*****************************************************************************
static uint32_t
iDMAUSBIntStatus(tUSBDMAInstance *psUSBDMAInst)
{
    //
    // Read the current DMA status, unfortunately this clears the
    // pending interrupt status.
    //
    return(USBDMAChannelIntStatus(psUSBDMAInst->ui32Base));
}

//*****************************************************************************
//
// USBLibDMAIntStatusClear() for USB controllers that use uDMA for DMA or have
// an integrated DMA controller.
//
//*****************************************************************************
static void
DMAUSBIntStatusClear(tUSBDMAInstance *psUSBDMAInst, uint32_t ui32Status)
{
    //
    // Clear out the requested interrupts.  Since the USB interface does not
    // have a true interrupt clear, this clears the current completed
    // status for the requested channels.
    //
    psUSBDMAInst->ui32Complete &= ~ui32Status;

    return;
}

//*****************************************************************************
//
// USBLibDMAIntHandler() for USB controllers that use uDMA for DMA or have an
// integrated DMA controller.
//
//*****************************************************************************
static void
DMAUSBIntHandler(tUSBDMAInstance *psUSBDMAInst, uint32_t ui32DMAIntStatus)
{
    uint32_t ui32Channel;

    if(ui32DMAIntStatus == 0)
    {
        return;
    }

    //
    // Determine if the uDMA is used or the USB DMA controller.
    //
    for(ui32Channel = 0; ui32Channel < USB_MAX_DMA_CHANNELS; ui32Channel++)
    {
        //
        // Mark any pending interrupts as completed.
        //
        if(ui32DMAIntStatus & 1)
        {
            psUSBDMAInst->ui32Pending &= ~(1 << ui32Channel);
            psUSBDMAInst->ui32Complete |= (1 << ui32Channel);
        }

        //
        // Check the next channel.
        //
        ui32DMAIntStatus >>= 1;

        //
        // Break if there are no more pending DMA interrupts.
        //
        if(ui32DMAIntStatus == 0)
        {
            break;
        }
    }
}


//*****************************************************************************
//
// USBLibDMAChannelEnable() for USB controllers with an integrated DMA
// controller.
//
//*****************************************************************************
static void
iDMAUSBChannelEnable(tUSBDMAInstance *psUSBDMAInst, uint32_t ui32Channel)
{
    uint32_t ui32IntEnabled;

    //
    // Save if the interrupt was enabled or not.
    //
    ui32IntEnabled = IntIsEnabled(psUSBDMAInst->ui32IntNum);

    //
    // Disable the USB interrupt if it was enabled.
    //
    if(ui32IntEnabled)
    {
        OS_INT_DISABLE(psUSBDMAInst->ui32IntNum);
    }

    //
    // Mark this channel as pending and not complete.
    //
    psUSBDMAInst->ui32Pending |= (1 << (ui32Channel - 1));
    psUSBDMAInst->ui32Complete &= ~(1 << (ui32Channel - 1));

    //
    // Enable the interrupt for this DMA channel.
    //
    USBDMAChannelIntEnable(psUSBDMAInst->ui32Base, ui32Channel - 1);

    //
    // Enable the DMA channel.
    //
    USBDMAChannelEnable(psUSBDMAInst->ui32Base, ui32Channel - 1);

    //
    // Enable the USB interrupt if it was enabled before.
    //
    if(ui32IntEnabled)
    {
        OS_INT_ENABLE(psUSBDMAInst->ui32IntNum);
    }
}


//*****************************************************************************
//
// USBLibDMAChannelDisable() for USB controllers with an integrated DMA
// controller.
//
//*****************************************************************************
static void
iDMAUSBChannelDisable(tUSBDMAInstance *psUSBDMAInst, uint32_t ui32Channel)
{
    //
    // Disable the DMA channel.
    //
    USBDMAChannelDisable(psUSBDMAInst->ui32Base, ui32Channel - 1);

    //
    // Disable the interrupt for this DMA channel.
    //
    USBDMAChannelIntDisable(psUSBDMAInst->ui32Base, ui32Channel - 1);

    //
    // Clear out any pending or complete flag set for this DMA channel.
    //
    psUSBDMAInst->ui32Pending &= ~(1 << (ui32Channel - 1));
    psUSBDMAInst->ui32Complete &= ~(1 << (ui32Channel - 1));
}


//*****************************************************************************
//
// USBLibDMAChannelIntEnable() for USB controllers with an integrated DMA
// controller.
//
//*****************************************************************************
static void
iDMAUSBChannelIntEnable(tUSBDMAInstance *psUSBDMAInst, uint32_t ui32Channel)
{
    //
    // Enable the interrupt for this DMA channel.
    //
    USBDMAChannelIntEnable(psUSBDMAInst->ui32Base, ui32Channel - 1);
}



//*****************************************************************************
//
// USBLibDMAChannelIntDisable() for USB controllers with an integrated DMA
// controller.
//
//*****************************************************************************
static void
iDMAUSBChannelIntDisable(tUSBDMAInstance *psUSBDMAInst, uint32_t ui32Channel)
{
    //
    // Disable the interrupt for this DMA channel.
    //
    USBDMAChannelIntDisable(psUSBDMAInst->ui32Base, ui32Channel - 1);
}


//*****************************************************************************
//
// USBLibDMATransfer() for USB controllers with an integrated DMA controller.
//
//*****************************************************************************
static uint32_t
iDMAUSBTransfer(tUSBDMAInstance *psUSBDMAInst, uint32_t ui32Channel,
                void *pvBuffer, uint32_t ui32Size)
{
    uint32_t ui32PacketCount;

    if((uint32_t)pvBuffer & 0x3)
    {
        return(0);
    }

    //
    // Mark this channel as pending and not complete.
    //
    psUSBDMAInst->ui32Pending |= (1 << (ui32Channel - 1));
    psUSBDMAInst->ui32Complete &= ~(1 << (ui32Channel - 1));

    //
    // Save the pointer to the data and the byte count.
    //
    psUSBDMAInst->ppui32Data[ui32Channel - 1] = pvBuffer;
    psUSBDMAInst->pui32Count[ui32Channel - 1] = ui32Size;

    //
    // Set the address.
    //
    USBDMAChannelAddressSet(psUSBDMAInst->ui32Base, ui32Channel - 1, pvBuffer);

    //
    // Set the number of transfers.
    //
    USBDMAChannelCountSet(psUSBDMAInst->ui32Base, ui32Channel - 1, ui32Size);

    //
    // Set the mode based on the size of the transfer.  More than one
    // packet requires mode 1.
    //
    if(ui32Size >= psUSBDMAInst->pui32MaxPacketSize[ui32Channel - 1])
    {
        //
        // Calculate the number of packets required for this transfer.
        //
        ui32PacketCount = ui32Size /
                          psUSBDMAInst->pui32MaxPacketSize[ui32Channel - 1];

        if(ui32Size % psUSBDMAInst->pui32MaxPacketSize[ui32Channel - 1])
        {
            ui32PacketCount += 1;
        }

        USBEndpointPacketCountSet(psUSBDMAInst->ui32Base,
                                  psUSBDMAInst->pui8Endpoint[ui32Channel - 1],
                                  ui32PacketCount);

        //
        // Configure the USB DMA controller for mode 1.
        //
        USBEndpointDMAConfigSet(psUSBDMAInst->ui32Base,
                                psUSBDMAInst->pui8Endpoint[ui32Channel - 1],
                                psUSBDMAInst->pui32EPDMAMode1[ui32Channel - 1]);

        USBDMAChannelConfigSet(psUSBDMAInst->ui32Base, ui32Channel - 1,
                               psUSBDMAInst->pui8Endpoint[ui32Channel - 1],
                               psUSBDMAInst->pui32Config[ui32Channel - 1] |
                               USB_DMA_CFG_MODE_1);

        //
        // Enable DMA on the endpoint.
        //
        if(psUSBDMAInst->pui32Config[ui32Channel - 1] & USB_DMA_CFG_DIR_TX)
        {
            //
            // Make sure that DMA is enabled on the endpoint.
            //
            MAP_USBEndpointDMAEnable(
                                psUSBDMAInst->ui32Base,
                                psUSBDMAInst->pui8Endpoint[ui32Channel - 1],
                                USB_EP_HOST_OUT);
        }
        else
        {
            //
            // Make sure that DMA is enabled on the endpoint.
            //
            MAP_USBEndpointDMAEnable(
                                   psUSBDMAInst->ui32Base,
                                   psUSBDMAInst->pui8Endpoint[ui32Channel - 1],
                                   USB_EP_HOST_IN);
        }

        //
        // Enable the DMA channel.
        //
        USBDMAChannelEnable(psUSBDMAInst->ui32Base, ui32Channel - 1);
    }
    else
    {
        //
        // Configure the USB DMA controller for mode 0.
        //
        USBEndpointDMAConfigSet(psUSBDMAInst->ui32Base,
                                psUSBDMAInst->pui8Endpoint[ui32Channel - 1],
                                psUSBDMAInst->pui32EPDMAMode0[ui32Channel -1]);

        USBDMAChannelConfigSet(psUSBDMAInst->ui32Base, ui32Channel -1,
                               psUSBDMAInst->pui8Endpoint[ui32Channel - 1],
                               psUSBDMAInst->pui32Config[ui32Channel - 1] |
                               USB_DMA_CFG_MODE_0);

        //
        // In mode 0 only enable DMA transfer for mode 0.
        //
        if(psUSBDMAInst->pui32Config[ui32Channel - 1] & USB_DMA_CFG_DIR_TX)
        {
            //
            // Make sure that DMA is enabled on the endpoint.
            //
            MAP_USBEndpointDMAEnable(
                                psUSBDMAInst->ui32Base,
                                psUSBDMAInst->pui8Endpoint[ui32Channel - 1],
                                USB_EP_HOST_OUT);
            //
            // Enable the DMA channel.
            //
            USBDMAChannelEnable(psUSBDMAInst->ui32Base, ui32Channel - 1);
        }
        else
        {
            //
            // Make sure that DMA is disabled on the endpoint, it will
            // be enabled when the endpoint interrupt occurs.
            //
            MAP_USBEndpointDMADisable(
                                   psUSBDMAInst->ui32Base,
                                   psUSBDMAInst->pui8Endpoint[ui32Channel - 1],
                                   USB_EP_HOST_IN);

            //
            // Returns 0 so that when DMA is not used, the bUseDMA flag in the calling function
            // is set to false.  Jira Bug number: MSP432SDK-276
            //
            return(0);  

        }
    }

    return(ui32Size);
}


//*****************************************************************************
//
// USBLibDMAChannelAllocate() for USB controllers with an integrated DMA
// controller.
//
//*****************************************************************************
static uint32_t
iDMAUSBChannelAllocate(tUSBDMAInstance *psUSBDMAInst, uint8_t ui8Endpoint,
                       uint32_t ui32MaxPacketSize, uint32_t ui32Config)
{
    uint32_t ui32Channel;

    //
    // Search for an available DMA channel to use.
    //
    for(ui32Channel = 0; ui32Channel < USB_MAX_DMA_CHANNELS_0; ui32Channel++)
    {
        //
        // If the current endpoint value is zero then this channel is
        // available.
        //
        if(psUSBDMAInst->pui8Endpoint[ui32Channel] == 0)
        {
            //
            // Clear out the attributes on this channel.
            //
            USBDMAChannelDisable(psUSBDMAInst->ui32Base, ui32Channel);

            //
            // Save the endpoint for this DMA channel.
            //
            psUSBDMAInst->pui8Endpoint[ui32Channel] = ui8Endpoint;

            //
            // Save the maximum packet size for the endpoint.
            //
            psUSBDMAInst->pui32MaxPacketSize[ui32Channel] = ui32MaxPacketSize;

            //
            // Assign the endpoint to the channel and set the direction.
            //
            if(ui32Config & USB_DMA_EP_RX)
            {
                psUSBDMAInst->pui32Config[ui32Channel] =
                                        USB_DMA_CFG_DIR_RX |
                                        USB_DMA_CFG_BURST_NONE |
                                        USB_DMA_CFG_INT_EN;

                //
                // If in device mode and Isochronous.
                //
                if(((ui32Config & USB_DMA_EP_HOST) == 0) &&
                   ((ui32Config & USB_DMA_EP_TYPE_M) == USB_DMA_EP_TYPE_ISOC))
                {
                    //
                    // USB_EP_AUTO_REQUEST is required for device
                    // Isochronous endpoints.
                    //
                    psUSBDMAInst->pui32EPDMAMode0[ui32Channel] =
                                                    USB_EP_DMA_MODE_0 |
                                                    USB_EP_AUTO_REQUEST |
                                                    USB_EP_HOST_IN;
                }
                else
                {
                    psUSBDMAInst->pui32EPDMAMode0[ui32Channel] =
                                                    USB_EP_DMA_MODE_0 |
                                                    USB_EP_AUTO_CLEAR |
                                                    USB_EP_HOST_IN;
                }

                //
                // Do not set auto request in device mode unless it is an
                // isochronous endpoint.
                //
                if(((ui32Config & USB_DMA_EP_HOST) == 0) &&
                   ((ui32Config & USB_DMA_EP_TYPE_M) != USB_DMA_EP_TYPE_ISOC))
                {
                    psUSBDMAInst->pui32EPDMAMode1[ui32Channel] =
                                                USB_EP_DMA_MODE_1 |
                                                USB_EP_HOST_IN |
                                                USB_EP_AUTO_CLEAR;
                }
                else
                {
                    psUSBDMAInst->pui32EPDMAMode1[ui32Channel] =
                                                USB_EP_DMA_MODE_1 |
                                                USB_EP_HOST_IN |
                                                USB_EP_AUTO_REQUEST |
                                                USB_EP_AUTO_CLEAR;
                }
            }
            else
            {
                psUSBDMAInst->pui32Config[ui32Channel] =
                                                USB_DMA_CFG_DIR_TX |
                                                USB_DMA_CFG_BURST_NONE |
                                                USB_DMA_CFG_INT_EN;

                psUSBDMAInst->pui32EPDMAMode0[ui32Channel] =
                                                USB_EP_DMA_MODE_0 |
                                                USB_EP_HOST_OUT;
                psUSBDMAInst->pui32EPDMAMode1[ui32Channel] =
                                                USB_EP_DMA_MODE_1 |
                                                USB_EP_HOST_OUT |
                                                USB_EP_AUTO_SET;
            }

            //
            // Outside of this function all channels are 1 based as
            // zero is not a valid channel.
            //
            return(ui32Channel + 1);
        }
    }
    return(0);
}



//*****************************************************************************
//
// DMAChannelRelease() for USB controllers with an integrated DMA controller.
//
//*****************************************************************************
static void
iDMAUSBChannelRelease(tUSBDMAInstance *psUSBDMAInst, uint8_t ui32Channel)
{
    ASSERT(ui32Channel < USB_MAX_DMA_CHANNELS);

    //
    // Clear out the attributes on this channel.
    //
    USBDMAChannelDisable(psUSBDMAInst->ui32Base, ui32Channel);

    //
    // Clear out the state for this endpoint.
    //
    psUSBDMAInst->pui8Endpoint[ui32Channel - 1] = 0;
    psUSBDMAInst->pui32Config[ui32Channel - 1] = 0;
    psUSBDMAInst->ui32Pending &= ~(1 << (ui32Channel - 1));
    psUSBDMAInst->ui32Complete &= ~(1 << (ui32Channel - 1));
    psUSBDMAInst->pui32MaxPacketSize[ui32Channel - 1] = 0;
}


//*****************************************************************************
//
// USBLibDMAUnitSizeSet() for USB controllers that have an integrated DMA
// controller.
//
//*****************************************************************************
static void
iDMAUSBUnitSizeSet(tUSBDMAInstance *psUSBDMAInst, uint32_t ui32Channel,
                   uint32_t ui32BitSize)
{
}


//*****************************************************************************
//
// USBLibDMAArbSizeSet() for USB controllers that have an integrated DMA
// controller.
//
//*****************************************************************************
static void
iDMAUSBArbSizeSet(tUSBDMAInstance *psUSBDMAInst, uint32_t ui32Channel,
                  uint32_t ui32ArbSize)
{
}

//*****************************************************************************
//
// USBLibDMAStatus() for USB controllers that use uDMA for DMA or have an
// integrated DMA controller.
//
//*****************************************************************************
static uint32_t
DMAUSBStatus(tUSBDMAInstance *psUSBDMAInst)
{
    return(0);
}

//*****************************************************************************
//
//! This function is used to return the current DMA pointer for a given
//! DMA channel.
//!
//! \param psUSBDMAInst is a generic instance pointer that can be used to
//! distinguish between different hardware instances.
//! \param ui32Channel is the DMA channel number for this request.
//!
//! This function returns the address that is in use by the DMA channel passed
//! in via the \e ui32Channel parameter.  This is not the real-time pointer,
//! but the starting address of the DMA transfer for this DMA channel.
//!
//! \return The current DMA address for the given DMA channel.
//
//*****************************************************************************
void *
USBLibDMAAddrGet(tUSBDMAInstance *psUSBDMAInst, uint32_t ui32Channel)
{
    return(psUSBDMAInst->ppui32Data[ui32Channel - 1]);
}

//*****************************************************************************
//
//! This function is used to return the current DMA transfer size for a given
//! DMA channel.
//!
//! \param psUSBDMAInst is a generic instance pointer that can be used to
//! distinguish between different hardware instances.
//! \param ui32Channel is the DMA channel number for this request.
//!
//! This function returns the DMA transfer size that is in use by the DMA
//! channel passed in via the \e ui32Channel parameter.
//!
//! \return The current DMA transfer size for the given DMA channel.
//
//*****************************************************************************
uint32_t
USBLibDMASizeGet(tUSBDMAInstance *psUSBDMAInst, uint32_t ui32Channel)
{
    return(psUSBDMAInst->pui32Count[ui32Channel - 1]);
}

//*****************************************************************************
//
//! This function is used to initialize the DMA interface for a USB instance.
//!
//! \param ui32Index is the index of the USB controller for this instance.
//!
//! This function performs any initialization and configuration of the DMA
//! portions of the USB controller.  This function returns a pointer that
//! is used with the remaining USBLibDMA APIs or the function returns zero
//! if the requested controller cannot support DMA.  If this function is called
//! when already initialized it will not reinitialize the DMA controller and
//! will instead return the previously initialized DMA instance.
//!
//! \return A pointer to use with USBLibDMA APIs.
//
//*****************************************************************************
tUSBDMAInstance *
USBLibDMAInit(uint32_t ui32Index)
{
    uint32_t ui32Channel;

    ASSERT(ui32Index == 0);

    //
    // Make sure that the DMA has not already been initialized.
    //
    if(g_psUSBDMAInst[0].ui32Base == USB0_BASE)
    {
        return(&g_psUSBDMAInst[0]);
    }

    //
    // Save the base address of the USB controller.
    //
    g_psUSBDMAInst[0].ui32Base = USB0_BASE;

    //
    // Save the interrupt number for the USB controller.
    //
    g_psUSBDMAInst[0].ui32IntNum = INT_USB0;

    //
    // Initialize the function pointers.
    //
    g_psUSBDMAInst[0].pfnArbSizeSet = iDMAUSBArbSizeSet;
    g_psUSBDMAInst[0].pfnChannelAllocate = iDMAUSBChannelAllocate;
    g_psUSBDMAInst[0].pfnChannelDisable = iDMAUSBChannelDisable;
    g_psUSBDMAInst[0].pfnChannelEnable = iDMAUSBChannelEnable;
    g_psUSBDMAInst[0].pfnChannelIntEnable = iDMAUSBChannelIntEnable;
    g_psUSBDMAInst[0].pfnChannelIntDisable = iDMAUSBChannelIntDisable;
    g_psUSBDMAInst[0].pfnChannelRelease = iDMAUSBChannelRelease;
    g_psUSBDMAInst[0].pfnChannelStatus = iDMAUSBChannelStatus;
    g_psUSBDMAInst[0].pfnIntHandler = DMAUSBIntHandler;
    g_psUSBDMAInst[0].pfnIntStatus = iDMAUSBIntStatus;
    g_psUSBDMAInst[0].pfnIntStatusClear = DMAUSBIntStatusClear;
    g_psUSBDMAInst[0].pfnStatus = DMAUSBStatus;
    g_psUSBDMAInst[0].pfnTransfer = iDMAUSBTransfer;
    g_psUSBDMAInst[0].pfnUnitSizeSet = iDMAUSBUnitSizeSet;

    //
    // Clear out the endpoint and the current configuration.
    //
    for(ui32Channel = 0; ui32Channel < USB_MAX_DMA_CHANNELS; ui32Channel++)
    {
        g_psUSBDMAInst[0].pui8Endpoint[ui32Channel] = 0;
        g_psUSBDMAInst[0].pui32Config[ui32Channel] = 0;
        g_psUSBDMAInst[0].ui32Pending = 0;
        g_psUSBDMAInst[0].ui32Complete = 0;
    }
    return(&g_psUSBDMAInst[0]);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
