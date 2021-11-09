//*****************************************************************************
//
// usbhcdc.c - USB CDC host class driver.
//
// Copyright (c) 2008-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
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
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "ti/devices/msp432e4/driverlib/driverlib.h"
#include "ti/usblib/msp432e4/usblib.h"
#include "ti/usblib/msp432e4/usblibpriv.h"
#include "ti/usblib/msp432e4/usbcdc.h"
#include "usbhost.h"
#include "usbhostpriv.h"
#include "usbhcdc.h"

static void *CDCDriverOpen(tUSBHostDevice *psDevice);
static void CDCDriverClose(void *pvInstance);

//*****************************************************************************
//
//! \addtogroup usblib_host_class_cdc
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// If the user has not explicitly stated the maximum number of CDC devices to
// support, we assume that we need to support up to the maximum number of USB
// devices that the build is configured for.
//
//*****************************************************************************
#ifndef MAX_CDC_DEVICES
#define MAX_CDC_DEVICES MAX_USB_DEVICES
#endif

//*****************************************************************************
//
// This is the structure that holds all of the data for a given instance of
// a CDC device.
//
//*****************************************************************************
struct tCDCInstance
{
    //
    // Save the device instance.
    //
    tUSBHostDevice *psDevice;

    //
    // Used to save the callback.
    //
    tUSBCallback pfnCallback;

    //
    // Callback data provided by caller.
    //
    void *pvCBData;

    //
    // Used to remember what type of device was registered.
    //
    tCDCSubClassProtocol iDeviceType;

    //
    // Bulk IN pipe.
    //
    uint32_t ui32CDCBulkInPipe;

    //
    // Bulk OUT pipe.
    //
    uint32_t ui32CDCBulkOutPipe;

    //
    // Interrupt IN pipe.
    //
    uint32_t ui32CDCIntInPipe;  

};

//*****************************************************************************
//
// The instance data storage for attached CDC devices.
//
//*****************************************************************************
static tCDCInstance g_psCDCDevice[MAX_CDC_DEVICES];

//*****************************************************************************
//
//! This constant global structure defines the CDC Class Driver that is
//! provided with the USB library.
//
//*****************************************************************************
const tUSBHostClassDriver g_sUSBCDCClassDriver =
{
    USB_CLASS_CDC,
    CDCDriverOpen,
    CDCDriverClose,
    0
};

//*****************************************************************************
//
//! This function is used to open an instance of a CDC device.
//!
//! \param iDeviceType is the type of device that should be loaded for this
//! instance of the CDC device.
//! \param pfnCallback is the function that will be called whenever changes
//! are detected for this device.
//! \param pvCBData is the data that will be returned in when the
//! \e pfnCallback function is called.
//!
//! This function creates an instance of an specific type of CDC device.  The
//! \e iDeviceType parameter is one subclass/protocol values of the types
//! specified in enumerated types tCDCSubClassProtocol.  Only devices that
//! enumerate with this type will be called back via the \e pfnCallback
//! function.  The \e pfnCallback parameter is the callback function for any
//! events that occur for this device type.  The \e pfnCallback function must
//! point to a valid function of type \e tUSBCallback for this call to complete
//! successfully.  To release this device instance the caller of USBHCDCOpen()
//! should call USBHCDCClose() and pass in the value returned from the
//! USBHCDCOpen() call.
//!
//! \return tCDCInstance is the instance value.  If a value of 0 is returned
//! then the device instance could not be created.
//
//*****************************************************************************
tCDCInstance *
USBHCDCOpen(tCDCSubClassProtocol iDeviceType, tUSBCallback pfnCallback,
            void *pvCBData)
{
    uint32_t ui32Loop;

    //
    // Find a free device instance structure.
    //
    for(ui32Loop = 0; ui32Loop < MAX_CDC_DEVICES; ui32Loop++)
    {
        if(g_psCDCDevice[ui32Loop].iDeviceType == eUSBHCDCClassNone)
        {
            //
            // Save the instance data for this device.
            //
            g_psCDCDevice[ui32Loop].pfnCallback = pfnCallback;
            g_psCDCDevice[ui32Loop].iDeviceType = iDeviceType;
            g_psCDCDevice[ui32Loop].pvCBData = pvCBData;

            //
            // Return the device instance pointer.
            //
            return(&g_psCDCDevice[ui32Loop]);
        }
    }

    //
    // If we get here, there are no space device slots so return NULL to
    // indicate a problem.
    //
    return(0);
}

//*****************************************************************************
//
//! This function is used to release an instance of a CDC device.
//!
//! \param psCDCInstance is the instance value for a CDC device to release.
//!
//! This function releases an instance of a CDC device that was created by a
//! call to USBHCDCOpen().  This call is required to allow other CDC devices
//! to be enumerated after another CDC device has been disconnected.  The
//! \e psCDCInstance parameter should hold the value that was returned from
//! the previous call to USBHCDCOpen().
//!
//! \return None.
//
//*****************************************************************************
void
USBHCDCClose(tCDCInstance *psCDCInstance)
{
    //
    // Disable any more notifications from the CDC layer.
    //
    psCDCInstance->pfnCallback = 0;

    //
    // Mark this device slot as free.
    //
    psCDCInstance->iDeviceType = eUSBHCDCClassNone;
}

//*****************************************************************************
//
//! This function handles callbacks for the interrupt IN endpoint.
//!
//! \param ui32Pipe is the instance value for a CDC device to release.
//! \param ui32Event is the event value for a CDC device
//!
//! \return None.
//
//*****************************************************************************
static void
CDCIntINCallback(uint32_t ui32Pipe, uint32_t ui32Event)
{
    int32_t i32Dev;

    switch (ui32Event)
    {
        //
        // Handles a request to schedule a new request on the interrupt IN
        // pipe.
        //
        case USB_EVENT_SCHEDULER:
        {
            USBHCDPipeSchedule(ui32Pipe, 0, 1);
            break;
        }
        //
        // Called when new data is available on the bulk IN pipe.
        //
        case USB_EVENT_RX_AVAILABLE:
        {
            //
            // Determine which device this notification is intended for.
            //
            for(i32Dev = 0; i32Dev < MAX_CDC_DEVICES; i32Dev++)
            {
                //
                // Does this device own the pipe we have been passed?
                //
                if(g_psCDCDevice[i32Dev].ui32CDCIntInPipe == ui32Pipe)
                {
                    //
                    // Yes - send the data to the USB host CDC device
                    // class driver.
                    //
                    g_psCDCDevice[i32Dev].pfnCallback(
                            g_psCDCDevice[i32Dev].pvCBData,
                                    USB_EVENT_RX_AVAILABLE, ui32Pipe, 0);
                }
            }

            break;
        }
    }
}

//*****************************************************************************
//
//! This function is used to open an instance of the CDC driver.
//!
//! \param psDevice is a pointer to the device information structure.
//!
//! This function will attempt to open an instance of the CDC driver based on
//! the information contained in the \e psDevice structure.  This call can fail
//! if there are not sufficient resources to open the device.  The function
//! returns a value that should be passed back into USBCDCClose() when the
//! driver is no longer needed.
//!
//! \return &g_psCDCDevice[i32Dev] is a pointer to a CDC driver instance.
//! \return 0 indicates that no user has registered an interest in 
//! this particular CDC device.
//
//*****************************************************************************
static void *
CDCDriverOpen(tUSBHostDevice *psDevice)
{
    int32_t i32Idx, i32Dev;
    uint8_t NumOfInterfaces, i;
    tEndpointDescriptor *psEndpointDescriptor;
    tInterfaceDescriptor *psInterface;

    NumOfInterfaces = psDevice->psConfigDescriptor->bNumInterfaces;

    psInterface = USBDescGetInterface(psDevice->psConfigDescriptor, 0, 0);
    //
    // Search the currently open instances for one that supports the protocol
    // of this device.
    //
    for(i32Dev = 0; i32Dev < MAX_CDC_DEVICES; i32Dev++)
    {
        if(g_psCDCDevice[i32Dev].iDeviceType ==
           psInterface->bInterfaceProtocol)
        {
            //
            // Save the device pointer.
            //
            g_psCDCDevice[i32Dev].psDevice = psDevice;

            for (i = 0; i < NumOfInterfaces; i++)
            {
                //
                // Get the interface descriptor for each interface
                //
                psInterface = USBDescGetInterface(psDevice->psConfigDescriptor, i, 0);

                //
                // Loop through the endpoints of the device.
                //
                for(i32Idx = 0; i32Idx < 3; i32Idx++)
                {
                    //
                    // Get the interrupt endpoint descriptor
                    //
                    psEndpointDescriptor =
                             USBDescGetInterfaceEndpoint(psInterface, i32Idx,
                                        psDevice->ui32ConfigDescriptorSize);

                    //
                    // If no more endpoints then break out.
                    //
                    if(psEndpointDescriptor == 0)
                    {
                        break;
                    }

                    //
                    // See if this is a bulk endpoint.
                    //
                    if((psEndpointDescriptor->bmAttributes & USB_EP_ATTR_TYPE_M) ==
                                                       USB_EP_ATTR_BULK)
                    {
                        //
                        // See if this is bulk IN or bulk OUT.
                        //
                        if(psEndpointDescriptor->bEndpointAddress & USB_EP_DESC_IN)
                        {
                            //
                            // Allocate the USB Pipe for this Bulk IN endpoint.
                            //
                            g_psCDCDevice[i32Dev].ui32CDCBulkInPipe =
                                    USBHCDPipeAllocSize(0, USBHCD_PIPE_BULK_IN_DMA,
                                        psDevice,
                                        psEndpointDescriptor->wMaxPacketSize, 0);
                            //
                            // Configure the USB pipe as a Bulk IN endpoint.
                            //
                            USBHCDPipeConfig(g_psCDCDevice[i32Dev].ui32CDCBulkInPipe,
                                 psEndpointDescriptor->wMaxPacketSize,
                                 0,
                                 (psEndpointDescriptor->bEndpointAddress &
                                  USB_EP_DESC_NUM_M));
                        }
                        else
                        {
                            //
                            // Allocate the USB Pipe for this Bulk OUT endpoint.
                            //
                            g_psCDCDevice[i32Dev].ui32CDCBulkOutPipe =
                                   USBHCDPipeAllocSize(0, USBHCD_PIPE_BULK_OUT_DMA,
                                   psDevice,
                                   psEndpointDescriptor->wMaxPacketSize, 0);
                            //
                            // Configure the USB pipe as a Bulk OUT endpoint.
                            //
                            USBHCDPipeConfig(g_psCDCDevice[i32Dev].ui32CDCBulkOutPipe,
                                 psEndpointDescriptor->wMaxPacketSize,
                                 0, (psEndpointDescriptor->bEndpointAddress &
                                  USB_EP_DESC_NUM_M));
                        }
                    }
                    //
                    // See if this is an interrupt endpoint.
                    //
                    if((psEndpointDescriptor->bmAttributes & USB_EP_ATTR_TYPE_M) ==
                                                   USB_EP_ATTR_INT)
                    {
                        //
                        // See if this is interrupt IN endpoint.
                        //
                        if(psEndpointDescriptor->bEndpointAddress & USB_EP_DESC_IN)
                        {
                            //
                            // Allocate the USB Pipe for this Interrupt IN endpoint.
                            //
                            g_psCDCDevice[i32Dev].ui32CDCIntInPipe =
                                         USBHCDPipeAlloc(0, USBHCD_PIPE_INTR_IN,
                                         psDevice, CDCIntINCallback);

                            //
                            // Configure the USB pipe as a Interrupt IN endpoint.
                            //
                            USBHCDPipeConfig(g_psCDCDevice[i32Dev].ui32CDCIntInPipe,
                                                    psEndpointDescriptor->wMaxPacketSize,
                                                    psEndpointDescriptor->bInterval,
                                                    (psEndpointDescriptor->bEndpointAddress &
                                                     USB_EP_DESC_NUM_M));
                        }
                   }
                }
            }
           //
           // If there is a callback function call it to inform the application that
           // the device has been enumerated.
           //
           if(g_psCDCDevice[i32Dev].pfnCallback != 0)
           {
               g_psCDCDevice[i32Dev].pfnCallback(
                       g_psCDCDevice[i32Dev].pvCBData,
                                   USB_EVENT_CONNECTED,
                                   (uint32_t)&g_psCDCDevice[i32Dev], 0);
           }

           //
           // Save the device pointer.
           //
           g_psCDCDevice[i32Dev].psDevice = psDevice;
           return (&g_psCDCDevice[i32Dev]);
        }
    }
    //
    // If we get here, no user has registered an interest in this particular
    // CDC device so we return an error.
    //
    return(0);
}

//*****************************************************************************
//
//! This function is used to release an instance of the CDC driver.
//!
//! \param pvInstance is an instance pointer that needs to be released.
//!
//! This function will free up any resources in use by the CDC driver instance
//! that is passed in.  The \e pvInstance pointer should be a valid value that
//! was returned from a call to USBCDCOpen().
//!
//! \return None.
//
//*****************************************************************************
static void
CDCDriverClose(void *pvInstance)
{
    tCDCInstance *psInst;

    //
    // Get our instance pointer.
    //
    psInst = (tCDCInstance *)pvInstance;

    //
    // Reset the device pointer.
    //
    psInst->psDevice = 0;

    //
    // Free the Interrupt IN pipe.
    //
    if(psInst->ui32CDCIntInPipe != 0)
    {
        USBHCDPipeFree(psInst->ui32CDCIntInPipe);
    }

    //
    // Free the Bulk IN pipe.
    //
    if(psInst->ui32CDCBulkInPipe != 0)
    {
        USBHCDPipeFree(psInst->ui32CDCBulkInPipe);
    }

    //
    // Free the Bulk OUT pipe.
    //
    if(psInst->ui32CDCBulkOutPipe != 0)
    {
        USBHCDPipeFree(psInst->ui32CDCBulkOutPipe);
    }

    //
    // If the callback exists, call it with a DISCONNECTED event.
    //
    if(psInst->pfnCallback != 0)
    {
        psInst->pfnCallback(psInst->pvCBData, USB_EVENT_DISCONNECTED,
                            (uint32_t)pvInstance, 0);
    }
}

//*****************************************************************************
//
//! This function is used to get line coding information for a CDC device.
//!
//! \param psCDCInstance is the instance value of a CDC device
//! USBHCDCOpen().
//! \param pui8Buffer is buffer that holds the CDC line coding parameter data
//! \param ui32Size is the line coding size
//!
//! This function is used to retrieve line parameter information from a device.
//! The parameters retrieved are DTR rate, stop bit, partity and data bits
//! This request is sent when a CDC device is connected to the host.
//!
//! \return ui32Bytes is the number of bytes read into the \e pui8Buffer.
//
//*****************************************************************************
uint32_t
USBHCDCGetLineCoding(tCDCInstance *psCDCInstance, uint8_t *pui8Buffer,
                     uint32_t ui32Size)
{
    tUSBRequest sSetupPacket;
    uint32_t ui32Bytes;

    //
    // This is a Class specific interface OUT request.
    //
    sSetupPacket.bmRequestType = USB_RTYPE_DIR_IN | USB_RTYPE_CLASS |
                                 USB_RTYPE_INTERFACE;

    //
    // Request a Device Descriptor.
    //
    sSetupPacket.bRequest = USBREQ_GET_LINE_CODING;
    sSetupPacket.wValue = 0;

    //
    // Set this on interface 0.
    //
    sSetupPacket.wIndex = 0;

    //
    // This is always 7 for this request.
    //
    sSetupPacket.wLength = 0x07;

    //
    // Put the setup packet in the buffer.
    //
    ui32Bytes = (USBHCDControlTransfer(0, &sSetupPacket, psCDCInstance->psDevice,
                                       pui8Buffer, 0x07 , MAX_PACKET_SIZE_EP0));

    return ui32Bytes;  
}

//*****************************************************************************
//
//! This function is used to set the line coding information for a CDC device.
//!
//! \param psCDCInstance is the value that was returned from the call to
//! USBHCDCOpen().
//! \param pui8Data is the line parameter values
//!
//! This function is used to set the line parameter information from a device.
//! The parameters are baud rate, stop bit, partiy and data bits
//! This request is sent when a CDC device is connected to the host.
//!
//! \return 0 since this is a status message
//
//*****************************************************************************
uint32_t
USBHCDCSetLineCoding(tCDCInstance *psCDCInstance,  uint8_t *pui8Data)
{
    tUSBRequest sSetupPacket;

    //
    // This is a Class specific interface OUT request.
    //
    sSetupPacket.bmRequestType = USB_RTYPE_DIR_OUT | USB_RTYPE_CLASS |
                                 USB_RTYPE_INTERFACE;

    //
    // Request a Device Descriptor.
    //
    sSetupPacket.bRequest = USBREQ_SET_LINE_CODING;
    sSetupPacket.wValue = 0;

    //
    // Set this on interface 0.
    //
    sSetupPacket.wIndex = 0;

    //
    // This is always 7 for this request.
    //
    sSetupPacket.wLength = 0x07;  

    //
    // Put the setup packet in the buffer.
    // This request includes an OUT transaction and an IN transaction. The OUT transaction is the line
    // coding structure of length 7 bytes
    //
    USBHCDControlTransfer(0, &sSetupPacket, psCDCInstance->psDevice,
                                 pui8Data, 0x07, MAX_PACKET_SIZE_EP0);

    return (0);  
}

//*****************************************************************************
//
//! This function is used to set the line state of a CDC device.
//!
//! \param psCDCInstance is the value that was returned from the call to
//! USBHCDCOpen().
//! \param carrierValue is the indicator that deactivates or activates the 
//! Control Line
//!
//! This function is used to set the line parameter information from a device.
//! The parameters are baud rate, stop bit, partiy and data bits
//! This request is sent when a CDC device is connected to the host.
//!
//! \return Returns 0 since this is a status message.
//
//*****************************************************************************
uint32_t
USBHCDCSetControlLineState(tCDCInstance *psCDCInstance, uint16_t carrierValue)
{
    tUSBRequest sSetupPacket;

    //
    // This is a Class specific interface OUT request (from host to device)
    //
    sSetupPacket.bmRequestType = USB_RTYPE_DIR_OUT | USB_RTYPE_CLASS |
                                 USB_RTYPE_INTERFACE;

    //
    // Request a Device Descriptor.
    //
    sSetupPacket.bRequest = USBREQ_SET_CONTROL_LINE_STATE;
    if(carrierValue == 0x03)
    {
      sSetupPacket.wValue = 0x03;  //Activate carrier, DTE present
    }
    else
    {

        sSetupPacket.wValue = 0x00;  //Deactivate carrier, DTE not present
    }

    //
    // Set this on interface 0.
    //
    sSetupPacket.wIndex = 0;

    //
    // This is always 0 for this request.
    //
    sSetupPacket.wLength = 0x0;  //no OUT transaction for this request.  Status only

    //
    // Put the setup packet in the buffer.
    // This request includes an OUT transaction and an IN transaction. The OUT transaction is the line
    // coding structure of length 7 bytes
    //
    USBHCDControlTransfer(0, &sSetupPacket, psCDCInstance->psDevice,
                                 0, 0, MAX_PACKET_SIZE_EP0);

    return (0);  
}

//*****************************************************************************
//
//! This function is used to read data from bulk IN endpoint.
//!
//! \param psCDCInstance is the value that was returned from the call to
//! USBHCDCOpen().
//! \param ui32Interface is the interface to retrieve the data from.
//! \param pui8Data is the memory buffer to use to store the data.
//! \param ui32Size is the size in bytes of the buffer pointed to by
//! \e pui8Buffer.
//!
//! This function is used to read data sent from a connected device.  The
//! function specifically reads data from a USB Bulk IN pipe.  The data is
//! sent every time the main loop enters \b STATE_CDC_DEVICE_CONNECTED
//! state.
//!
//! \return ui32Size is the number of bytes read from buffer.
//
//*****************************************************************************
uint32_t
USBHCDCReadData(tCDCInstance *psCDCInstance, uint32_t ui32Interface,
                 uint8_t *pui8Data, uint32_t ui32Size)
{

     //
     // read data from bulk IN pipe
    //
     ui32Size = USBHCDPipeRead(psCDCInstance->ui32CDCBulkInPipe,
                                             pui8Data, ui32Size);

    //
    // Return the number of bytes read from the IN pipe.
    //
    return(ui32Size);
}

//*****************************************************************************
//
//! This function writes data to bulk OUT endpoint.
//!
//! \param psCDCInstance is the value that was returned from the call to
//! USBHCDCOpen().
//! \param ui32Interface is the interface to write the data to.
//! \param pui8Data is the memory buffer storing the data.
//! \param ui32Size is the size in bytes of the buffer pointed to by
//! \e pui8Buffer.
//!
//! This function is used to send data to a connected device by writing the
//! data stored in a memory buffer to a USB Bulk OUT pipe.  The data is
//! polled every time the main loop enters \b STATE_CDC_DEVICE_CONNECTED
//! state.
//!
//! \return ui32Size is the number of bytes read from buffer.
//
//*****************************************************************************
uint32_t
USBHCDCWriteData(tCDCInstance *psCDCInstance, uint32_t ui32Interface,
                 uint8_t *pui8Data, uint32_t ui32Size)
{
     //
     // send data to device via bulk OUT pipe
     //
     ui32Size = USBHCDPipeWrite(psCDCInstance->ui32CDCBulkOutPipe,
                                             pui8Data, ui32Size);

    //
    // Return the number of bytes sent to OUT pipe.
    //
    return(ui32Size);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
