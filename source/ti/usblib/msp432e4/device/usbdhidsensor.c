//*****************************************************************************
//
// usbdhidsensor.c - USB HID Sensor device class driver.
//
// Copyright (c) 2008-2017 Texas Instruments Incorporated.  All rights reserved.
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


#include "ti/usblib/msp432e4/usblib.h"
#include "ti/usblib/msp432e4/usblibpriv.h"
#include "usbdevice.h"
#include "ti/usblib/msp432e4/usbhid.h"
#include "usbdhid.h"
#include "usbdhidsensor.h"

//*****************************************************************************
//
//! \addtogroup hid_sensor_device_class_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// HID sensor device configuration descriptor.
//
// It is vital that the configuration descriptor bConfigurationValue field
// (byte 6) is 1 for the first configuration and increments by 1 for each
// additional configuration defined here.  This relationship is assumed in the
// device stack for simplicity even though the USB 2.0 specification imposes
// no such restriction on the bConfigurationValue values.
//
// Note that this structure is deliberately located in RAM since we need to
// be able to patch some values in it based on client requirements.
//
//*****************************************************************************
static uint8_t g_pui8SensorDescriptor[] =
{
    //
    // Configuration descriptor header.
    //
    9,                          // Size of the configuration descriptor.
    USB_DTYPE_CONFIGURATION,    // Type of this descriptor.
    USBShort(24),               // The total size of this full structure.
    1,                          // The number of interfaces in this
                                // configuration.  bNumInterface
    1,                          // The unique value for this configuration. bConfigurationValue
    5,                          // The string identifier that describes this
                                // configuration. iConfiguration
    USB_CONF_ATTR_BUS_PWR,     // Bus Powered, Self Powered, remote wake up.
    10,                        // The maximum power in 2mA increments.
};

//*****************************************************************************
//
// The remainder of the configuration descriptor is stored in flash since we
// don't need to modify anything in it at runtime.
//
//*****************************************************************************
static uint8_t g_pui8HIDInterface[HIDINTERFACE_SIZE] =
{
    //
    // HID Device Class Interface Descriptor.
    //
    9,                          // Size of the interface descriptor.
    USB_DTYPE_INTERFACE,        // Type of this descriptor.
    0,                          // The index for this interface.
    0,                          // The alternate setting for this interface.
    1,                          // The number of endpoints used by this
                                // interface.
    USB_CLASS_HID,              // The interface class
    USB_HID_SCLASS_NONE,        // The interface sub-class.
    USB_HID_PROTOCOL_NONE,      // The interface protocol for the sub-class
                                // specified above.
    4,                          // The string index for this interface.
};

//*****************************************************************************
//
// This is the HID IN endpoint descriptor for the sensor device.
//
//*****************************************************************************
static const uint8_t g_pui8HIDInEndpoint[HIDINENDPOINT_SIZE] =
{
    //
    // Interrupt IN endpoint descriptor
    //
    7,                          // The size of the endpoint descriptor.
    USB_DTYPE_ENDPOINT,         // Descriptor type is an endpoint.
    USB_EP_DESC_IN | USBEPToIndex(USB_EP_1),
    USB_EP_ATTR_INT,            // Endpoint is an interrupt endpoint.
    USBShort(USBFIFOSizeToBytes(USB_FIFO_SZ_64)),
                                // The maximum packet size.
    1,                         // The polling interval for this endpoint.
};

//*****************************************************************************
//
// The following is the HID report structure definition that is passed back
// to the host.
//
//*****************************************************************************
static const uint8_t g_pui8HIDOutEndpoint[HIDOUTENDPOINT_SIZE] =
{
    //
    // Interrupt OUT endpoint descriptor
    //
    7,                          // The size of the endpoint descriptor.
    USB_DTYPE_ENDPOINT,         // Descriptor type is an endpoint.
    USB_EP_DESC_OUT | USBEPToIndex(USB_EP_2),
    USB_EP_ATTR_INT,            // Endpoint is an interrupt endpoint.
    USBShort(USBFIFOSizeToBytes(USB_FIFO_SZ_64)),
                                // The maximum packet size.
    16,                         // The polling interval for this endpoint.
};


//*****************************************************************************
//
// The following is the HID report structure definition that is passed back
// to the host.
// UsagePage(Vendor Defined) is selected here in order for the HIDDemo_Tool
// to interact with the device connected to any Windows machine (7 and 10).
// If all instances of UsagePage(Vendor Defined) in the report descriptor is
// replaced with: UsagePage(USB_HID_SENSOR),  HidDemo_Tool is not able to
// interact with the device on a Windows 10 machine since Windows 10 classifies the
// device as a Sensor and associates a SensorHid.dll driver to the device.
// The generic HID API calls used to create the HidDemo_Tool GUI will need to
// be updated with SensorHID API calls in order to communicate with the device.
//
//*****************************************************************************


static const uint8_t g_pui8SensorReportDescriptor[] =
{
     //UsagePage(USB_HID_SENSOR),               // uncomment this line and comment
                                                // the vendor defined usage page to
                                                // identify the device as a sensor
                                                // in Windows 10.

     0x06, 0x00, 0xff,                          //UsagePage(Vendor Defined)
     Usage(USB_HID_ENVIRONMENTAL_TEMPERATURE),
     Collection(USB_HID_PHYSICAL),
//     0x85, 0x01,    // Report ID (Vendor Defined)

     UsagePage(USB_HID_SENSOR),
     HidUsageSensorPropertyReportingState,           //HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE,
     LogicalMinimum(0),
     LogicalMaximum(5),
     ReportSize(8),
     ReportCount(1),
     Feature(USB_HID_INPUT_DATA | USB_HID_INPUT_ARRAY |USB_HID_INPUT_ABS ),   //Data_Arr_Abs

     HidUsageSensorPropertySensorStatus,                            //HID_USAGE_SENSOR_PROPERTY_SENSOR_STATUS,
     LogicalMinimum(0),
     LogicalMaximum32(0xFF,0xFF,0xFF,0xFF),
     ReportSize(32),
     ReportCount(1),
     Feature(USB_HID_INPUT_DATA | USB_HID_INPUT_VARIABLE | USB_HID_INPUT_ABS),   // Data_Var_Abs up to VT_UI4 worth of status info

     HidUsageSensorPropertyReportInterval,
     LogicalMinimum(0),
     LogicalMaximum32(0xFF,0xFF,0xFF,0xFF),
     ReportSize(32),
     ReportCount(1),
     UnitExponent(0),
     Feature(USB_HID_INPUT_DATA | USB_HID_INPUT_VARIABLE | USB_HID_INPUT_ABS),

     0x0A,0x0F,0x03,                            //HID_USAGE_SENSOR_PROPERTY_CHANGE_SENSITIVITY_ABS,
     LogicalMinimum(0),
     LogicalMinimum16(0xFF,0xFF),
     ReportSize(16),
     ReportCount(1),
     UnitExponent(0x0E), // scale default unit “Celsius” to provide 2 digits past the decimal point
     Feature(USB_HID_INPUT_DATA | USB_HID_INPUT_VARIABLE | USB_HID_INPUT_ABS),    //Data_Var_Abs

     //UsagePage(USB_HID_SENSOR),                //uncomment this line and comment
                                                 // the vendor defined usage page to
                                                 // identify the device as a sensor
                                                 // in Windows 10.
     0x06, 0x00, 0xff,                           //UsagePage(Vendor Defined)
     HidUsageSensorState,
     LogicalMinimum(0),
     LogicalMaximum(6),
     ReportSize(8),
     ReportCount(1),
     Input(USB_HID_INPUT_CONSTANT | USB_HID_INPUT_ARRAY | USB_HID_INPUT_ABS),

     HidUsageSensorEvent,
     LogicalMinimum(0),
     LogicalMaximum(16),
     ReportSize(8),
     ReportCount(1),
     Input(USB_HID_INPUT_CONSTANT | USB_HID_INPUT_ARRAY | USB_HID_INPUT_ABS),

     HidUsageSensorDataEnvironmentalTemperature,
     LogicalMinimum16(01, 0x80),  // LOGICAL_MINIMUM (-32767)
     LogicalMaximum16(0xFF,0x7F),   // LOGICAL_MAXIMUM (32767)
     ReportSize(16),
     ReportCount(1),
     UnitExponent(0x0E),  // scale default unit “Celsius” to provide 2 digits past the decimal point
     Input(USB_HID_INPUT_CONSTANT | USB_HID_INPUT_VARIABLE | USB_HID_INPUT_ABS),
     EndCollection


};


//*****************************************************************************
//
// The HID descriptor for the sensor device.
//
//*****************************************************************************
static const tHIDDescriptor g_sSensorHIDDescriptor =
{
    9,                              // bLength
    USB_HID_DTYPE_HID,              // bDescriptorType
    0x111,                          // bcdHID (version 1.11 compliant)
    0,                              // bCountryCode (not localized)
    1,                              // bNumDescriptors
    {
        {
            USB_HID_DTYPE_REPORT,   // Report descriptor
            sizeof(g_pui8SensorReportDescriptor)
                                    // Size of report descriptor
        }
    }
};

//*****************************************************************************
//
// The HID configuration descriptor is defined as four or five sections
// depending upon the client's configuration choice.  These sections are:
//
// 1.  The 9 byte configuration descriptor (RAM).
// 2.  The interface descriptor (RAM).
// 3.  The HID report and physical descriptors (provided by the client)
//     (FLASH).
// 4.  The mandatory interrupt IN endpoint descriptor (FLASH).
// 5.  The optional interrupt OUT endpoint descriptor (FLASH).
//
//*****************************************************************************
static const tConfigSection g_sHIDConfigSection =
{
    sizeof(g_pui8SensorDescriptor),
    g_pui8SensorDescriptor
};

static const tConfigSection g_sHIDInterfaceSection =
{
    sizeof(g_pui8HIDInterface),
    g_pui8HIDInterface
};

static const tConfigSection g_sHIDInEndpointSection =
{
    sizeof(g_pui8HIDInEndpoint),
    g_pui8HIDInEndpoint
};

static const tConfigSection g_sHIDOutEndpointSection =
{
    sizeof(g_pui8HIDOutEndpoint),
    g_pui8HIDOutEndpoint
};

//*****************************************************************************
//
// Place holder for the user's HID descriptor block.
//
//*****************************************************************************
static tConfigSection g_sHIDDescriptorSection =
{
   sizeof(g_sSensorHIDDescriptor),
   (const uint8_t *)&g_sSensorHIDDescriptor
};

//*****************************************************************************
//
// This array lists all the sections that must be concatenated to make a
// single, complete HID configuration descriptor.
//
//*****************************************************************************
static const tConfigSection *g_psHIDSections[] =
{
    &g_sHIDConfigSection,
    &g_sHIDInterfaceSection,
    &g_sHIDDescriptorSection,
    &g_sHIDInEndpointSection,
    &g_sHIDOutEndpointSection
};

#define NUM_HID_SECTIONS        ((sizeof(g_psHIDSections) /                   \
                                  sizeof(g_psHIDSections[0])) - 1)

//*****************************************************************************
//
// The header for the single configuration we support.  This is the root of
// the data structure that defines all the bits and pieces that are pulled
// together to generate the configuration descriptor.  Note that this must be
// in RAM since we need to include or exclude the final section based on
// client supplied initialization parameters.
//
//*****************************************************************************
static tConfigHeader g_sHIDConfigHeader =
{
    NUM_HID_SECTIONS,
    g_psHIDSections
};

//*****************************************************************************
//
// Configuration Descriptor.
//
//*****************************************************************************
static const tConfigHeader * const g_ppsHIDConfigDescriptors[] =
{
    &g_sHIDConfigHeader
};

//*****************************************************************************
//
// The HID class descriptor table.  For the HID sensor, we have only a
// single report descriptor.
//
//*****************************************************************************
static const uint8_t * const g_pui8SensorClassDescriptors[] =
{
      g_pui8SensorReportDescriptor
};


//*****************************************************************************
//
// Forward references for sensor device callback functions.
//
//*****************************************************************************
static uint32_t HIDSensorRxHandler(void *pvSensorDevice, uint32_t ui32Event,
                                     uint32_t ui32MsgData, void *pvMsgData);
static uint32_t HIDSensorTxHandler(void *pvSensorDevice, uint32_t ui32Event,
                                     uint32_t ui32MsgData, void *pvMsgData);


//*****************************************************************************
//
// HID Sensor transmit channel event handler function.
//
// \param pvGameDevice is the event callback pointer provided during
// USBDHIDInit().  This is a pointer to the HID Sensor device structure
// of the type tUSBDHIDSensorDevice.
// \param ui32Event identifies the event we are being called back for.
// \param ui32MsgData is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the lower level HID device class driver to inform
// the application of particular asynchronous events related to report events
// related to using the interrupt IN endpoint.
//
// \return Returns a value which is event-specific.
//
//*****************************************************************************
static uint32_t
HIDSensorTxHandler(void *pvSensorDevice, uint32_t ui32Event,
                    uint32_t ui32MsgData, void *pvMsgData)
{
    tUSBDSensorInstance *psInst;
    tUSBDHIDSensorDevice *psSensor;

    //
    // Make sure we did not get a NULL pointer.
    //
    ASSERT(pvSensorDevice);

    //
    // Get a pointer to our instance data
    //
    psSensor = (tUSBDHIDSensorDevice *)pvSensorDevice;
    psInst = &psSensor->sPrivateData;

    //
    // Which event were we sent?
    //
    switch (ui32Event)
    {
        //
        // A report transmitted via the interrupt IN endpoint was acknowledged
        // by the host.
        //
        case USB_EVENT_TX_COMPLETE:
        {
            //
            // The last transmission is complete so return to the idle state.
            //
            psInst->iState = eHIDSensorStateIdle;

            //
            // Pass the event on to the application.
            //
            psSensor->pfnCallback(psSensor->pvCBData, USB_EVENT_TX_COMPLETE,
                                   ui32MsgData, (void *)0);

            break;
        }

        //
        // Ignore all other events related to transmission of reports via
        // the interrupt IN endpoint.
        //
        default:
        {
            break;
        }
    }

    return(0);
}


//*****************************************************************************
//
// Main HID device class event handler function.
//
// \param pvSensorDevice is the event callback pointer provided during
//  USBDHIDInit().This is a pointer to our HID device structure
// (&g_sHIDSensorDevice).
// \param ui32Event identifies the event we are being called back for.
// \param ui32MsgData is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the HID device class driver to inform the
// application of particular asynchronous events related to operation of the
// sensor HID device.
//
// \return Returns a value which is event-specific.
//
//*****************************************************************************
static uint32_t
HIDSensorRxHandler(void *pvSensorDevice, uint32_t ui32Event,
                     uint32_t ui32MsgData, void *pvMsgData)
{
    tUSBDSensorInstance *psInst;
    tUSBDHIDSensorDevice *psSensorDevice;
    uint32_t ui32Ret;

    //
    // Make sure we did not get a NULL pointer.
    //
    ASSERT(pvSensorDevice);

    //
    // Return zero by default.
    //
    ui32Ret = 0;

    //
    // Get a pointer to our instance data
    //
    psSensorDevice = (tUSBDHIDSensorDevice *)pvSensorDevice;
    psInst = &psSensorDevice->sPrivateData;

    //
    // Which event were we sent?
    //
    switch (ui32Event)
    {
        //
        // The host has connected to us and configured the device.
        //
        case USB_EVENT_CONNECTED:
        {
            //
            // Now in the idle state.
            //
            psInst->iState = eHIDSensorStateIdle;

            //
            // Pass the information on to the client.
            //
            psSensorDevice->pfnCallback(psSensorDevice->pvCBData,
                                          USB_EVENT_CONNECTED, 0, (void *)0);

            break;
        }

        //
        // The host has disconnected from us.
        //
        case USB_EVENT_DISCONNECTED:
        {
            psInst->iState = eHIDSensorStateNotConnected;

            //
            // Pass the information on to the applicatio.
            //
            ui32Ret = psSensorDevice->pfnCallback(psSensorDevice->pvCBData,
                                          USB_EVENT_DISCONNECTED, 0,
                                          (void *)0);

            break;
        }

        //
        // The host is polling us for a particular report and the HID driver
        // is asking for the latest version to transmit.
        //
        case USBD_HID_EVENT_IDLE_TIMEOUT:
        {
            //
            // Give the pointer to the idle report structure.
            //
            *(void **)pvMsgData = (void *)&psInst->sReportIdle;

            ui32Ret = sizeof(psInst->sReportIdle);

            break;
        }
        case USBD_HID_EVENT_GET_REPORT:
        {

             ui32Ret = psSensorDevice->pfnCallback(psSensorDevice->pvCBData,
                                                  USBD_HID_EVENT_GET_REPORT, 0,
                                                  pvMsgData);

        }

        //
        // The device class driver has completed sending a report to the
        // host in response to a Get_Report request.
        //
        case USBD_HID_EVENT_REPORT_SENT:
        {
            //
            // We have nothing to do here.
            //
            break;
        }

        //
        // Pass ERROR, SUSPEND and RESUME to the client unchanged.
        //
        case USB_EVENT_ERROR:
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
        case USB_EVENT_LPM_RESUME:
        case USB_EVENT_LPM_SLEEP:
        case USB_EVENT_LPM_ERROR:
        {
            ui32Ret = psSensorDevice->pfnCallback(psSensorDevice->pvCBData, ui32Event,
                                                         ui32MsgData, pvMsgData);
            break;
        }

        // This event is sent in response to a host Set_Report request.  We
        // must return a pointer to a buffer large enough to receive the
        // report into.
        //
        case USBD_HID_EVENT_GET_REPORT_BUFFER:
            break;

        // This event indicates that the host has sent us an Output or
        // Feature report.  Sensor report supports Input reports only
        // as listed in HID report descriptor
        //
        case USBD_HID_EVENT_SET_REPORT:
            //not implemented

            break;

        //
        // We ignore all other events.
        //
        default:
        {
            break;
        }
    }
    return(ui32Ret);
}


//*****************************************************************************
//
//! Initializes HID sensor device operation for a given USB controller.
//!
//! \param ui32Index is the index of the USB controller which is to be
//! initialized for HID sensor device operation.
//! \param psHIDSensorDevice points to a structure containing parameters
//! customizing the operation of the HID Sensor device.
//!
//! An application wishing to offer a USB HID sensor interface to a USB host
//! must call this function to initialize the USB controller and attach the
//! sensor device to the USB bus.  This function performs all required USB
//! initialization.
//!
//! On successful completion, this function returns the \e psHIDSensorDevice
//! pointer passed to it.  This must be passed on all future calls to the HID
//! sensor device driver.
//!
//! When a host connects and configures the device, the application callback
//! receives \b USB_EVENT_CONNECTED after which sensor temp, state and event
//! are reported back to the host.
//!
//! \note The application must not make any calls to the lower level USB device
//! interfaces if interacting with USB via the USB HID sensor device class
//! API.  Doing so causes unpredictable (though almost certainly
//! unpleasant) behavior.
//!
//! \return Returns NULL on failure or the \e psHIDSensorDevice pointer on success.
//
//*****************************************************************************
tUSBDHIDSensorDevice *
USBDHIDSensorInit(uint32_t ui32Index, tUSBDHIDSensorDevice *psHIDSensorDevice)
{
    void *pvRetcode;
    tUSBDHIDDevice *psHIDDevice;
    tConfigDescriptor *pConfigDesc;

    //
    // Check parameter validity.
    //
    ASSERT(psHIDSensorDevice);
    ASSERT(psHIDSensorDevice->ppui8StringDescriptors);
    ASSERT(psHIDSensorDevice->pfnCallback);

    //
    // Get a pointer to the HID device data.
    //
    psHIDDevice = &psHIDSensorDevice->sPrivateData.sHIDDevice;

    //
    // Call the common initialization routine.
    //
    pvRetcode = USBDHIDSensorCompositeInit(ui32Index, psHIDSensorDevice, 0);

    pConfigDesc = (tConfigDescriptor *)g_pui8SensorDescriptor;
    pConfigDesc->bmAttributes = psHIDSensorDevice->ui8PwrAttributes;
    pConfigDesc->bMaxPower =  (uint8_t)(psHIDSensorDevice->ui16MaxPowermA / 2);

    //
    // If we initialized the HID layer successfully, pass our device pointer
    // back as the return code, otherwise return NULL to indicate an error.
    //
    if(pvRetcode)
    {
        //
        // Initialize the lower layer HID driver and pass it the various
        // structures and descriptors necessary to declare that we are a
        // sensor.
        //
        pvRetcode = USBDHIDInit(ui32Index, psHIDDevice);

        return(psHIDSensorDevice);

    }
    else
    {
        return((tUSBDHIDSensorDevice *)0);
    }
}

//*****************************************************************************
//
//! Initializes HID sensor device operation for a given USB controller.
//!
//! \param ui32Index is the index of the USB controller which is to be
//! initialized for HID sensor device operation.
//! \param psHIDSensorDevice points to a structure containing parameters
//! customizing the operation of the HID sensor device.
//! \param psCompEntry is the composite device entry to initialize when
//! creating a composite device.
//!
//! This call is very similar to USBDHIDSensorInit() except that it is used
//! for initializing an instance of the HID sensor device for use in a
//! composite device.  If this HID sensor is part of a composite device, then
//! the \e psCompEntry should point to the composite device entry to
//! initialize. This is part of the array that is passed to the
//! USBDCompositeInit() function.
//!
//! \return Returns zero on failure or a non-zero instance value that should be
//! used with the remaining USB HID sensor APIs.
//
//*****************************************************************************
tUSBDHIDSensorDevice *
USBDHIDSensorCompositeInit(uint32_t ui32Index,
                             tUSBDHIDSensorDevice *psHIDSensorDevice,
                             tCompositeEntry *psCompEntry)
{
    tUSBDSensorInstance *psInst;
    tUSBDHIDDevice *psHIDDevice;

    //
    // Check parameter validity.
    //
    ASSERT(psHIDSensorDevice);
    ASSERT(psHIDSensorDevice->ppui8StringDescriptors);
    ASSERT(psHIDSensorDevice->pfnCallback);

    //
    // Get a pointer to our instance data
    //
    psInst = &psHIDSensorDevice->sPrivateData;

    //
    // Initialize the various fields in our instance structure.
    //
    psInst->sReportIdle.ui8Duration4mS = 125;
    psInst->sReportIdle.ui8ReportID = 0x00;
    psInst->sReportIdle.ui32TimeSinceReportmS = 0;
    psInst->sReportIdle.ui16TimeTillNextmS = 0;

    //
    // Initialize the various fields in our instance structure.
    //
    psInst->iState = eHIDSensorStateNotConnected;

    //
    // Get a pointer to the HID device data.
    //
    psHIDDevice = &psInst->sHIDDevice;

    //
    // Initialize the HID device class instance structure based on input from
    // the caller.
    //
    psHIDDevice->ui16PID = psHIDSensorDevice->ui16PID;
    psHIDDevice->ui16VID = psHIDSensorDevice->ui16VID;
    psHIDDevice->ui16MaxPowermA = psHIDSensorDevice->ui16MaxPowermA;
    psHIDDevice->ui8PwrAttributes = psHIDSensorDevice->ui8PwrAttributes;
    psHIDDevice->ui8Subclass = USB_HID_SCLASS_NONE;
    psHIDDevice->ui8Protocol = USB_HID_PROTOCOL_NONE;
    psHIDDevice->ui8NumInputReports = 1;
    psHIDDevice->psReportIdle = 0;
    psHIDDevice->pfnRxCallback = HIDSensorRxHandler;
    psHIDDevice->pvRxCBData = (void *)psHIDSensorDevice;
    psHIDDevice->pfnTxCallback = HIDSensorTxHandler;
    psHIDDevice->pvTxCBData = (void *)psHIDSensorDevice;
    psHIDDevice->bUseOutEndpoint = false,

    psHIDDevice->psHIDDescriptor = &g_sSensorHIDDescriptor;
    psHIDDevice->ppui8ClassDescriptors = g_pui8SensorClassDescriptors;
    psHIDDevice->ppui8StringDescriptors =
                                        psHIDSensorDevice->ppui8StringDescriptors;
    psHIDDevice->ui32NumStringDescriptors =
                                        psHIDSensorDevice->ui32NumStringDescriptors;
    psHIDDevice->ppsConfigDescriptor = g_ppsHIDConfigDescriptors;

    psHIDDevice->psReportIdle = &psInst->sReportIdle;

    //
    // Initialize the lower layer HID driver and pass it the various structures
    // and descriptors necessary to declare that we are a sensor.
    //
    return(USBDHIDCompositeInit(ui32Index, psHIDDevice, psCompEntry));
}

//*****************************************************************************
//
//! Schedules a report to be sent once the host requests more data.
//!
//! \param psHIDSensor is the structure pointer that is returned from the
//! USBDHIDSensorCompositeInit() or USBDHIDSensorInit() functions.
//! \param pvReport is the data to send to the host.
//! \param ui32Size is the number of bytes in the \e pvReport buffer.
//!
//! This call is made by an application to schedule data to be sent to the
//! host when the host requests an update from the device.  The application
//! must then wait for a \b USB_EVENT_TX_COMPLETE event in the function
//! provided in the \e pfnCallback pointer in the tUSBDHIDSensorDevice
//! structure before being able to send more data with this function.  The
//! pointer passed in the \e pvReport can be updated once this call returns as
//! the data has been copied from the buffer.  The function returns
//! \b USBDSENSOR_SUCCESS if the transmission was successfully scheduled or
//! \b USBDSENSOR_TX_ERROR if the report could not be sent at this time.
//! If the call is made before the device is connected or ready to communicate
//! with the host, then the function can return \b USBDSENSOR_NOT_CONFIGURED.
//!
//! \return The function returns one of the \b USBDSENSOR_* values.
//
//*****************************************************************************
uint32_t
USBDHIDSensorSendReport(tUSBDHIDSensorDevice *psHIDSensor, void *pvReport,
                         uint32_t ui32Size)
{
    uint32_t ui32Retcode, ui32Count;
    tUSBDSensorInstance *psInst;
    tUSBDHIDDevice *psHIDDevice;

    //
    // Get a pointer to the HID device data.
    //
    psHIDDevice = &psHIDSensor->sPrivateData.sHIDDevice;

    //
    // Get a pointer to our instance data
    //
    psInst = &psHIDSensor->sPrivateData;

    //
    // If we are not configured, return an error here before trying to send
    // anything.
    //
    if(psInst->iState == eHIDSensorStateNotConnected)
    {
        return(USBDSENSOR_NOT_CONFIGURED);
    }

    //
    // Only send a report if the transmitter is currently free.
    //
    if(USBDHIDTxPacketAvailable((void *)psHIDDevice))
    {
        //
        // Send the report to the host.
        //
        psInst->iState = eHIDSensorStateSending;
        ui32Count = USBDHIDReportWrite((void *)psHIDDevice, pvReport, ui32Size,
                                       true);

        //
        // Did we schedule a packet for transmission correctly?
        //
        if(ui32Count == 0)
        {
            //
            // No - report the error to the caller.
            //
            ui32Retcode = USBDSENSOR_TX_ERROR;
        }
        else
        {
            ui32Retcode = USBDSENSOR_SUCCESS;
        }
    }
    else
    {
        ui32Retcode = USBDSENSOR_TX_ERROR;
    }

    //
    // Return the relevant error code to the caller.
    //
    return(ui32Retcode);
}

//*****************************************************************************
//
//! Shuts down the HID Sensor device.
//!
//! \param psSensor is the pointer to the device instance structure
//! as returned by USBDHIDSensorInit() or USBDHIDSensorCompositeInit().
//!
//! This function terminates HID Sensor operation for the instance supplied
//! and removes the device from the USB bus.  Following this call, the
//! \e psSensor instance may not me used in any other call to the HID
//! Sensor device other than to reinitialize by calling USBDHIDSensorInit()
//! or USBDHIDSensorCompositeInit().
//!
//! \return None.
//
//*****************************************************************************
void
USBDHIDSensorTerm(tUSBDHIDSensorDevice *psSensor)
{
    tUSBDHIDDevice *psHIDDevice;

    ASSERT(psSensor);

    //
    // Get a pointer to the HID device data.
    //
    psHIDDevice = &psSensor->sPrivateData.sHIDDevice;

    //
    // Mark the device as no longer connected.
    //
    psSensor->sPrivateData.iState = eHIDSensorStateNotConnected;

    //
    // Terminate the low level HID driver.
    //
    USBDHIDTerm(psHIDDevice);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


