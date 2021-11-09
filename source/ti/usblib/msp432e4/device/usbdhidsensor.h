//*****************************************************************************
//
// usbdhidsensor.h - Definitions used by HID sensor class devices.
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
//
//*****************************************************************************
#ifndef __USBDHIDSENSOR_H__
#define __USBDHIDSENSOR_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \addtogroup hid_sensor_device_class_api
//! @{
//
//*****************************************************************************
//*****************************************************************************
//
// PRIVATE
//
// The first few sections of this header are private defines that are used by
// the USB HID sensor code and are here only to help with the application
// allocating the correct amount of memory for the USB HID Sensor device
// code.
//
//*****************************************************************************


//*****************************************************************************
//
// PRIVATE
//
// This enumeration holds the various states that the sensor can be in during
// normal operation.
//
//*****************************************************************************
typedef enum
{
    //
    // Not yet configured.
    //
    eHIDSensorStateNotConnected,

    //
    // Nothing to transmit and not waiting on data to be sent.
    //
    eHIDSensorStateIdle,

    //
    // Waiting on data to be sent.
    //
    eHIDSensorStateSending
}
tSensorState;


//*****************************************************************************
//
// PRIVATE
//
// This structure defines the private instance data structure for the USB HID
// sensor device.  This structure forms the RAM workspace used by each
// instance of the sensor.
//
//*****************************************************************************
typedef struct
{
    //
    // This is needed for the lower level HID driver.
    //
    tUSBDHIDDevice sHIDDevice;

     //
    // The current state of the game pad device.
    //
    tSensorState iState;

    //
    // The idle timeout control structure for our input report.  This is
    // required by the lower level HID driver.
    //
    tHIDReportIdle sReportIdle;
} tUSBDSensorInstance;

//*****************************************************************************
//
//! This structure is used by the application to define operating parameters
//! for the HID sensor device.
//
//*****************************************************************************
typedef struct
{
    //
    //! The vendor ID that this device is to present in the device descriptor.
    //
    const uint16_t ui16VID;

    //
    //! The product ID that this device is to present in the device descriptor.
    //
    const uint16_t ui16PID;

    //
    //! The maximum power consumption of the device, expressed in milliamps.
    //
    const uint16_t ui16MaxPowermA;

    //
    //! Indicates whether the device is self- or bus-powered and whether or not
    //! it supports remote wakeup.  Valid values are \b USB_CONF_ATTR_SELF_PWR
    //! or \b USB_CONF_ATTR_BUS_PWR, optionally ORed with
    //! \b USB_CONF_ATTR_RWAKE.
    //
    const uint8_t ui8PwrAttributes;

    //! A pointer to the callback function which is called to notify
    //! the application of general events and those related to reception of
    //! Output and Feature reports via the (optional) interrupt OUT endpoint.
    //
    const tUSBCallback pfnCallback;

    //
    //! A client-supplied pointer which is sent as the first
    //! parameter in all calls made to the sensor callback,
    //! pfnCallback.
    //
    void *pvCBData;

    //
    //! A pointer to the string descriptor array for this device.  This array
    //! must contain the following string descriptor pointers in this order.
    //! Language descriptor, Manufacturer name string (language 1), Product
    //! name string (language 1), Serial number string (language 1),HID
    //! Interface description string (language 1), Configuration description
    //! string (language 1).
    //!
    //! If supporting more than 1 language, the descriptor block (except for
    //! string descriptor 0) must be repeated for each language defined in the
    //! language descriptor.
    //
    const uint8_t * const *ppui8StringDescriptors;

    //
    //! The number of descriptors provided in the ppStringDescriptors
    //! array.  This must be (1 + (5 * (num languages))).
    //
    const uint32_t ui32NumStringDescriptors;

    //
    //! The private instance data for this device.  This memory must
    //! remain accessible for as long as the sensor device is in use and
    //! must not be modified by any code outside the HID sensor driver.
    //
    tUSBDSensorInstance sPrivateData;
}
tUSBDHIDSensorDevice;

///*****************************************************************************
//
//! The USBDHIDSensorSendReport() call successfully scheduled the report.
//
//*****************************************************************************
#define USBDSENSOR_SUCCESS     0

//*****************************************************************************
//
//! The USBDHIDSensorSendReport() function could not send the report at this
//! time.
//
//*****************************************************************************
#define USBDSENSOR_TX_ERROR    1

//*****************************************************************************
//
//! The device is not currently configured and cannot perform any operations.
//
//*****************************************************************************
#define USBDSENSOR_NOT_CONFIGURED    2


//*****************************************************************************
//
//! This structure is the default packed report structure that is sent to the
//! host.   The data sent to the host that is part of this structure are
//! specified in Input report of the Report Descriptor.
//! This structure is passed to the USBDHIDSensorSendReport
//! function to send sensor updates to the host.
//
//*****************************************************************************
typedef struct
{

    //
    //! 8-bit value.  Sensor state
    //
    uint8_t ui8SensorState;

    //
    //! 8-bit value.  Sensor Event
    //
    uint8_t ui8SensorEvent;

    //
    //! Signed 16-bit temperature value (-32767 to 32767).
    //
    int16_t i16Temp;

}PACKED tSensorTemperatureReport;


//*****************************************************************************
//
// API Function Prototypes
//
//*****************************************************************************
extern tUSBDHIDSensorDevice *USBDHIDSensorInit(uint32_t ui32Index,
                                        tUSBDHIDSensorDevice *psHIDSensor);
extern tUSBDHIDSensorDevice *USBDHIDSensorCompositeInit(uint32_t ui32Index,
                                         tUSBDHIDSensorDevice *psHIDSensor,
                                         tCompositeEntry *psCompEntry);
extern void USBDHIDSensorTerm(tUSBDHIDSensorDevice *psCompEntry);

extern uint32_t USBDHIDSensorSendReport(tUSBDHIDSensorDevice *psHIDSensor,
                                         void *pvReport, uint32_t ui32Size);

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************



//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __USBDHIDSENSOR_H__


