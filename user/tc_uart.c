/*
 * @Author: Pomin
 * @Date: 2021-11-04 14:48:03
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-06 21:53:30
 * @Description:
 */
#include "tc_uart.h"

/* UART0 -- PC   UART6 -- LCD   UART7 -- BLUETOOTH */
void uart_init(void) {
    /* Enable the clock to GPIO port A and UART 0 */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    /* Configure the GPIO Port A for UART 0 */
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);

    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Configure the UART for 115200 bps 8-N-1 format */
    UARTStdioConfig(0, 115200, g_ui32SysClock);
}

uint32_t uart_read(void) {
    return 0;
}

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void uart_write(const uint8_t * pui8Buffer) {
    // Loop while there are more characters to send.
//     while (*pui8Buffer != '\0') {
//         // Write the next character to the UART.
//         MAP_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer);
//         pui8Buffer++;
//     }
}
//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
// void UART0_IRQHandler(void) {
//     uint32_t ui32Status;

//     // Get the interrrupt status.
//     ui32Status = MAP_UARTIntStatus(UART0_BASE, true);

//     // Clear the asserted interrupts.
//     MAP_UARTIntClear(UART0_BASE, ui32Status);

//     // Loop while there are characters in the receive FIFO.
//     while (MAP_UARTCharsAvail(UART0_BASE)) {
//         // Read the next character from the UART and write it back to the UART.
//         MAP_UARTCharPutNonBlocking(UART0_BASE,
//                                    MAP_UARTCharGetNonBlocking(UART0_BASE));
//     }
// }
