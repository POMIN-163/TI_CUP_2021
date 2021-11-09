/*
 * @Author: Pomin
 * @Date: 2021-11-04 14:48:03
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-07 20:21:26
 * @Description:
 */
#include "tc_bluetooth.h"

void bluetooth_init(void) {
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);

    MAP_GPIOPinConfigure(GPIO_PA4_U3RX);
    MAP_GPIOPinConfigure(GPIO_PA5_U3TX);

    MAP_UARTConfigSetExpClk(UART3_BASE, g_ui32SysClock, 9600,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                UART_CONFIG_PAR_NONE));
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5);
}
void uart_bluetooth_put_asciis(char* buff, uint8_t len) {
    for (size_t i = 0; i < len; i++) {
        MAP_UARTCharPutNonBlocking(UART3_BASE, buff[i]);
        SysCtlDelay(120000);
    }
}
uint32_t bluetooth_read(void) {
    return 0;
}
void bluetooth_write(uint16_t data_arr[], uint16_t len) {
    uint16_t temp = 0;
    uint8_t buff[7] = { 0xa5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5a, };
    for (size_t i = 0; i < len; i++) {
        temp = data_arr[i];
        buff[3] = temp >> 8;
        buff[4] = temp & 0xff;
        temp = buff[3] + buff[4];
        buff[5] = (uint8_t)(temp);
        uart_bluetooth_put_asciis(buff, 7);
    }
}

