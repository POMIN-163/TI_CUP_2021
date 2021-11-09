//*****************************************************************************
//
// usbhcdcserial.c - USB CDC host class driver.
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
#include "ti/usblib/msp432e4/usblib.h"
#include "usbhost.h"
#include "ti/usblib/msp432e4/usbcdc.h"
#include "usbhcdc.h"
#include "usbhcdcserial.h"

//*****************************************************************************
//
//! \addtogroup usblib_host_cdc
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Prototypes for local functions.
//
//*****************************************************************************
static uint32_t USBHSerialCallback(void *pvCdc, uint32_t ui32Event,
                                     uint32_t ui32MsgParam, void *pvMsgData);

//*****************************************************************************
//
// The size of a USB CDC serial data size packets.
//
//*****************************************************************************
#define USBHCDC_DATA_SIZE    64

//*****************************************************************************
//
// This is the structure definition for a CDC device instance.
//
//*****************************************************************************
struct tUSBHCDCSerial
{
    //
    // Global flags for an instance of the device.
    //
    uint32_t ui32CDCFlags;

    //
    // The applications registered callback.
    //
    tUSBHCDCSerialCallback pfnCallback;

    //
    // The CDC instance pointer for this CDC instance.
    //
    tCDCInstance *psCDCInstance;

    //
    // This is a local buffer to hold the CDC data
    // from the CDC driver layer.
    //
    uint8_t pui8Buffer[64];
    uint8_t pui8OutBuffer[64];

};

//*****************************************************************************
//
// This is the per instance information for a CDC device.
//
//*****************************************************************************
static tUSBHCDCSerial g_sUSBHCDCSerial =
{
    0
};

//*****************************************************************************
//
//! This function is used open an instance of a CDC device.
//!
//! \param pfnCallback is the callback function to call when new events occur
//! with the CDC device returned.
//!
//! This function is used to open an instance of the CDC device.  The value
//! returned from this function should be used as the instance identifier for
//! all other USBHCDCSerial calls.
//!
//! \return &g_sUSBHCDCSerial is the pointer to the instance identifier for 
//! the device that is attached. If there is no device present this will 
//! return 0.
//
//*****************************************************************************
tUSBHCDCSerial *
USBHCDCSerialOpen(tUSBHCDCSerialCallback pfnCallback)
{
    //
    // Save the callback and data pointers.
    //
    g_sUSBHCDCSerial.pfnCallback = pfnCallback;

    //
    // Save the instance pointer for the CDC device that was opened.
    //
    g_sUSBHCDCSerial.psCDCInstance =
        USBHCDCOpen(eUSBHCDCClassDirectLineControl, USBHSerialCallback,
                    (void *)&g_sUSBHCDCSerial);


    return(&g_sUSBHCDCSerial);
}

//*****************************************************************************
//
//! This function is used close an instance of a CDC device.
//!
//! \param psCdcSerialInstance is the instance value for this device.
//!
//! This function is used to close an instance of the device that was opened
//! with a call to USBHCDCSerialOpen().  The \e psCdcSerialInstance value is the
//! value that was returned when the application called USBHCDCSerialOpen().
//!
//! \return 0 to indicate success of closing the device
//
//*****************************************************************************
uint32_t
USBHCDCSerialClose(tUSBHCDCSerial *psCdcSerialInstance)
{
    //
    // Reset the callback to null.
    //
    psCdcSerialInstance->pfnCallback = 0;

    //
    // Call the CDC driver layer to close out this instance.
    //
    USBHCDCClose(psCdcSerialInstance->psCDCInstance);

    return(0);
}

//*****************************************************************************
//
//! This function handles event callbacks from the USB CDC driver layer.
//!
//! \param pvCdc is the pointer that was passed in to the USBHCDCOpen()
//! call.
//! \param ui32Event is the event that has been passed up from the CDC driver.
//! \param ui32MsgParam has meaning related to the \e ui32Event that occurred.
//! \param pvMsgData has meaning related to the \e ui32Event that occurred.
//!
//! This function will receive all event updates from the CDC driver layer.
//!
//! \return Non-zero values should be assumed to indicate an error condition.
//
//*****************************************************************************
static uint32_t
USBHSerialCallback(void *pvCdc, uint32_t ui32Event,
                     uint32_t ui32MsgParam, void *pvMsgData)
{
    tUSBHCDCSerial *psCdcSerialInstance;

    //
    // Recover the pointer to the instance data.
    //
    psCdcSerialInstance = (tUSBHCDCSerial *)pvCdc;

    switch (ui32Event)
    {
        //
        // New CDC serial device has been connected so notify the application.
        //
        case USB_EVENT_CONNECTED:
        {
            //
            // Remember that a CDC serial device is present.
            //
            psCdcSerialInstance->ui32CDCFlags |= USBHCDC_DEVICE_PRESENT;

            //
            // Notify the application about the connected device.
            //
            psCdcSerialInstance->pfnCallback(0, ui32Event, ui32MsgParam, pvMsgData);

            break;
        }
        case USB_EVENT_DISCONNECTED:
        {
            //
            // No CDC is present.
            //
            psCdcSerialInstance->ui32CDCFlags &= ~USBHCDC_DEVICE_PRESENT;

            //
            // Notify the application that the CDC serial device was disconnected.
            //
            psCdcSerialInstance->pfnCallback(0, ui32Event, ui32MsgParam, pvMsgData);

            break;
        }
    }
    return(0);
}

//*****************************************************************************
//
//! This function is used to initialize a CDC interface after a CDC device
//! is detected.
//!
//! \param psCdcInstance is the instance value for this device.
//!
//! This function should be called after receiving a \b STATE_CDC_DEVICE_INIT
//! event in the callback function provided by USBHCDCOpen(), however this
//! function should only be called outside the callback function.  This will
//! initialize the CDC interface.  The
//! \e psCdcInstance value is the value that was returned when the application
//! called USBHCDCOpen().  This function only needs to be called once
//! per connection event but it should be called every time a
//! \b STATE_CDC_DEVICE_INIT event occurs.
//!
//! \return This function returns 0 to indicate success any non-zero value
//! indicates an error condition.
//
//*****************************************************************************
uint32_t
USBHCDCSerialInit(tUSBHCDCSerial *psCdcInstance)
{
    uint8_t parmArray[7] = {0x00, 0xc2, 01, 00, 00, 00, 0x08};

    //
    // Get Line coding data.  size of data is 7 bytes
    //
    USBHCDCGetLineCoding(psCdcInstance->psCDCInstance, psCdcInstance->pui8Buffer, USB_GET_LINE_CODING_SIZE);

    //
    // Set Control line state - deactivate the carrier if it is active
    //
    USBHCDCSetControlLineState(psCdcInstance->psCDCInstance, CDC_DEACTIVATE_CARRIER);

    //
    // Set the parameters of the line (baud rate = 0, stop bits = 1, partity = none, databits = 8)
    //
    //
    USBHCDCSetLineCoding(psCdcInstance->psCDCInstance, parmArray);

    //
    // Get Line coding data.  size of data is 7 bytes
    //
    USBHCDCGetLineCoding(psCdcInstance->psCDCInstance, psCdcInstance->pui8Buffer, USB_GET_LINE_CODING_SIZE);



    return(0);

    }

//*****************************************************************************
//
//! This function calls the api that gets the data at the CDC bulk endpoint sent
//! by the CDC device.
//!
//! \param psCdcSerialInstance is the instance value for this device.
//! \param ui32Interface is the bulk endpoint interface
//!
//! This function is called after receiving a \b USB_EVENT_CONNECTED
//! event in the callback function provided by USBHCDCOpen().
//!
//! \return This function returns 0 to indicate success any non-zero value
//! indicates an error condition.
//
//*****************************************************************************

uint32_t
USBHCDCGetDataFromDevice(tUSBHCDCSerial *psCdcSerialInstance, uint32_t ui32Interface)
{

     uint8_t i;
     //
     // Read data sent from device into input buffer pui8Buffer
     //
    USBHCDCReadData(psCdcSerialInstance->psCDCInstance, ui32Interface,
                                     psCdcSerialInstance->pui8Buffer,
                                     USBHCDC_DATA_SIZE);

    //
    // Pass the Data Received event to main application
    //
    psCdcSerialInstance->pfnCallback( 0, USBH_EVENT_RX_CDC_DATA,
                                      psCdcSerialInstance->pui8Buffer[0],
                                           0);

     //
     // Clean out the buffer for receiving new data
     //
    for(i = 0; i < USBHCDC_DATA_SIZE;i++)
    {

        psCdcSerialInstance->pui8Buffer[i] = 0;
    }
    return(0);

}
//*****************************************************************************
//
//! This function calls the api that sends the bulk data to the CDC device
//!
//! \param psCdcSerialInstance is the instance value for this device.
//! \param ui32Interface is the bulk endpoint interface
//! \param *ui8SendBuffer is pointer of array
//! \param byteSize  is number of data bytes to be sent to device
//!
//! This function is called after receiving a \b USB_EVENT_CONNECTED
//! event in the callback function provided by USBHCDCOpen().  And it
//! is called after receiving 74 bytes of data from the device
//!
//! \return This function returns 0 to indicate success. Any non-zero value
//! indicates an error condition.
//
//*****************************************************************************

uint32_t
USBHCDCSendDataToDevice(tUSBHCDCSerial *psCdcSerialInstance, uint32_t ui32Interface,
                        uint8_t *ui8SendBuffer, uint8_t byteSize)
{
    uint8_t i;

    //
    // Copy data to the outbuffer
    //
    for (i = 0; i<byteSize; i++)
    {
        psCdcSerialInstance->pui8OutBuffer[i] = ui8SendBuffer[i];
    }

    //
    // Send data to device
    //
    USBHCDCWriteData(psCdcSerialInstance->psCDCInstance, ui32Interface,
                                         psCdcSerialInstance->pui8OutBuffer,
                                         byteSize);

    //
    // Pass the Data Transmitted event to main application
    //
    psCdcSerialInstance->pfnCallback(0, USBH_EVENT_TX_CDC_DATA,
                                     psCdcSerialInstance->pui8OutBuffer[0],
                                     0);

    return(0);
}


//*****************************************************************************
//
//! This function processes the data contained in the USB buffer.
//!
//! \param psCdcSerialInstance is the instance value for this device.
//! \param ui8ArrayIndex index into the buffer array
//!
//! This function is called after receiving a \b USB_EVENT_CONNECTED
//! event in the callback function provided by USBHCDCOpen().
//!
//! \return ui32Value The value pointed to by ui8ArrayIndex.
//
//*****************************************************************************
uint32_t
USBHCDCProcessData(tUSBHCDCSerial *psCdcSerialInstance, uint8_t ui8ArrayIndex)
{
   uint32_t ui32Value;

    //
    // Get value at specified array index.
    //
    ui32Value = psCdcSerialInstance->pui8Buffer[ui8ArrayIndex];

    return(ui32Value);

}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

