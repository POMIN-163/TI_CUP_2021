//*****************************************************************************
//
// usbhcdcserial.h - USB CDC host class driver.
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
#ifndef __USBHCDCSERIAL_H__
#define __USBHCDCSERIAL_H__

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
//! \addtogroup usblib_host_cdc
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// These are the flags for the ui32CDCFlags member variable.
//
//*****************************************************************************
#define USBHCDC_DEVICE_PRESENT 0x00000001


typedef struct tUSBHCDCSerial tUSBHCDCSerial;

//*****************************************************************************
//
// The prototype for the host USB CDC driver callback function.
//
//*****************************************************************************
typedef void (*tUSBHCDCSerialCallback)(tUSBHCDCSerial *psCdcSerialInstance,
                                        uint32_t ui32Event,
                                        uint32_t ui32MsgParam,
                                        void *pvMsgData);

extern tUSBHCDCSerial * USBHCDCSerialOpen(tUSBHCDCSerialCallback pfnCallback);

extern uint32_t USBHCDCSerialClose(tUSBHCDCSerial *psCdcInstance);

extern uint32_t USBHCDCSerialInit(tUSBHCDCSerial *psCdcInstance);

extern uint32_t USBHCDCGetDataFromDevice(tUSBHCDCSerial *psCdcSerialInstance, uint32_t ui32Interface);
extern uint32_t USBHCDCProcessData(tUSBHCDCSerial *psCdcSerialInstance, uint8_t ui8ArrayIndex);
extern uint32_t USBHCDCSendDataToDevice(tUSBHCDCSerial *psCdcSerialInstance, uint32_t ui32Interface,
                        uint8_t *ui8SendBuffer, uint8_t byteSize);
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
