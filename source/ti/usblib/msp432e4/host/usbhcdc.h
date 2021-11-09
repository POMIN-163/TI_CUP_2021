//*****************************************************************************
//
// usbhcdc.h - This hold the host driver for CDC class.
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

#ifndef __USBHCDC_H__
#define __USBHCDC_H__

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
//! \addtogroup usblib_host_class_cdc
//! @{
//
//*****************************************************************************

typedef struct tCDCInstance tCDCInstance;

//*****************************************************************************
//
// These defines are the the events that will be passed in the \e ui32Event
// parameter of the callback from the driver.
//
//*****************************************************************************
#define CDC_EVENT_OPEN          1
#define CDC_EVENT_CLOSE         2

//*****************************************************************************
//
// CDC USB requests.
//
//*****************************************************************************
#define USBREQ_GET_LINE_CODING  0x21
#define USBREQ_SET_LINE_CODING  0x20
#define USBREQ_SET_CONTROL_LINE_STATE  0x22

#define USB_GET_LINE_CODING_SIZE   0x07
#define CDC_DEACTIVATE_CARRIER   0x00
#define CDC_ACTIVATE_CARRIER     0x03

//*****************************************************************************
//
// These defines are the the events that will be passed in the ui32Event
// parameter of the callback from the driver.
//
//*****************************************************************************

//
//! The CDC data detected.
//
#define USBH_EVENT_RX_CDC_DATA     USBH_CDC_EVENT_BASE + 16

//
// CDC data to send
//
#define USBH_EVENT_TX_CDC_DATA     USBH_CDC_EVENT_BASE + 17

//*****************************************************************************
//
//! The following values are used to register callbacks to the USB HOST CDC
//! device class layer.
//
//*****************************************************************************
typedef enum
{
    //
    //! No device should be used.  This value should not be used by
    //! applications.
    //
    eUSBHCDCClassNone = 0,

    //
    //! This is a direct line control device.
    //
    eUSBHCDCClassDirectLineControl,

    //
    //! This is a abstract control device.
    //
    eUSBHCDCClassAbstractContol,

    //
    //! This is telephone control device.
    //
    eUSBHCDCClassTelephoneControl,

    //
    //! This is multi-channel control device.
    //
    eUSBHCDCClassMultichannelControl,

    //
    //! This is CAPI control device.
    //
    eUSBHCDCClassCapiControl,

    //
    //! This is Ethernet Networking Control device.
    //
    eUSBHCDCClassEthernetNetworkingControl,

    //
    //! This is ATM Networking control device.
    //
    eUSBHCDCClassATMNetworkingControl,

    //
    //! This is a vendor specific device.
    //
    eUSBHCDCClassVendor
}
tCDCSubClassProtocol;

//*****************************************************************************
//
// Prototypes.
//
//*****************************************************************************
extern tCDCInstance * USBHCDCOpen(tCDCSubClassProtocol iDeviceType,
                                  tUSBCallback pfnCallback,
                                  void *pvCBData);
extern void USBHCDCClose(tCDCInstance *psCDCInstance);

extern uint32_t USBHCDCSetControlLineState(tCDCInstance *psCDCInstance, uint16_t carrierValue);
extern uint32_t USBHCDCSetLineCoding(tCDCInstance *psCDCInstance,  uint8_t *pui8Data);
extern uint32_t USBHCDCGetLineCoding(tCDCInstance *psCDCInstance, uint8_t *pui8Buffer,
                     uint32_t ui32Size);
extern uint32_t USBHCDCReadData(tCDCInstance *psCDCInstance, uint32_t ui32Interface,
                 uint8_t *pui8Data, uint32_t ui32Size);

extern const tUSBHostClassDriver g_sUSBCDCClassDriver;
extern uint32_t USBHCDCWriteData(tCDCInstance *psCDCInstance, uint32_t ui32Interface,
                 uint8_t *pui8Data, uint32_t ui32Size);
				 
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
#endif
