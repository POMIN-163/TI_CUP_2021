/*
 * @Author: Pomin
 * @Date: 2021-11-04 14:48:03
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-07 20:02:34
 * @Description:
 */
#include "tc_lcd.h"

void lcd_init(void) {
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);

    MAP_GPIOPinConfigure(GPIO_PC4_U7RX);
    MAP_GPIOPinConfigure(GPIO_PC5_U7TX);

    MAP_UARTConfigSetExpClk(UART7_BASE, g_ui32SysClock, 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                UART_CONFIG_PAR_NONE));

    MAP_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
}

uint32_t lcd_read(void) {
    return 0;
}
unsigned char lcd_buff[4];
void Float2Char(float fSend, unsigned char* chSend) {
    chSend[3] = *((char*)(&fSend));
    chSend[2] = *((char*)(&fSend) + 1);
    chSend[1] = *((char*)(&fSend) + 2);
    chSend[0] = *((char*)(&fSend) + 3);
}
void uart_lcd_put_ascii(uint8_t asc) {
    MAP_UARTCharPutNonBlocking(UART7_BASE, asc);
}
void uart_lcd_put_asciis(char* buff, uint8_t len) {
    for (size_t i = 0; i < len; i++) {
        MAP_UARTCharPutNonBlocking(UART7_BASE, buff[i]);
        SysCtlDelay(9600);
    }
}
void uart_lcd_put_asciis_test(char* buff, uint8_t len) {
    for (size_t i = 0; i < len; i++) {
        MAP_UARTCharPutNonBlocking(UART0_BASE, buff[i]);
        SysCtlDelay(1200);
    }
}
char uart_lcd_ctr_wave[] = {
    0x5A, 0xA5, 0x37, 0x82, 0x03, 0x10, 0x5A, 0xA5, 0x01, 0x00, 0x02, 0x01
};
char uart_lcd_ctr_wave_mid[] = {
    0x5A, 0xA5, 0x05, 0x82, 0x80, 0x06, 0x08, 0x00,
    0x5A, 0xA5, 0x05, 0x82, 0x80, 0x09, 0x02, 0x02,
    0x5A, 0xA5, 0x05, 0x82, 0x80, 0x08, 0x00, 0x20,
};
char uart_lcd_ctr_wave_cls[] = {
    0x5A, 0xA5, 0x05, 0x82, 0x03, 0x05, 0x00, 0x00,
};
char uart_lcd_ctr_float[] = {
    0x5A, 0xA5, 0x07, 0x82,
};
char uart_lcd_ctr_rst[] = {
    0x5A, 0xA5, 0x07, 0x82, 0x00, 0x04, 0x55, 0xaa, 0x5A, 0xA5,
};
char uart_lcd_ctr_beep[] = {
    0x5A, 0xA5, 0x05, 0x82, 0x00, 0xa0, 0x00, 0x3e,
};
char uart_lcd_ctr_thd[] = {
    0x5A, 0xA5, 0x07, 0x82, 0x00, 0x10,
};
/* 写单精度浮点数 */
uint8_t u8arr[2];
void u16tou8(uint16_t val) {
    u8arr[0] = val >> 8;
    u8arr[1] = val;
}
void lcd_write(uint16_t addr, float value) {
    u16tou8(addr);
    Float2Char(value, lcd_buff);
    uart_lcd_put_asciis(uart_lcd_ctr_float, 4);
    uart_lcd_put_asciis(u8arr, 2);
    uart_lcd_put_asciis(lcd_buff, 4);
}
void lcd_beep(void) {
    uart_lcd_put_asciis(uart_lcd_ctr_beep, 8);
}
void lcd_rst(void) {
    uart_lcd_put_asciis(uart_lcd_ctr_rst, 10);
}
void lcd_disp_thd(float value) {
    Float2Char(value, lcd_buff);
    uart_lcd_put_asciis(uart_lcd_ctr_thd, 6);
    uart_lcd_put_asciis(lcd_buff, 4);
}
uint16_t wave_max = 0;
uint16_t wave_min = 0;
void lcd_disp_wave_point(uint16_t val) {
    u16tou8(val);
    uart_lcd_put_asciis(uart_lcd_ctr_wave, 12);
    // uart_lcd_put_ascii(1); // 单次数据大小
    uart_lcd_put_asciis(u8arr, 2);
}
/**
 * 1k --
 *
 *
 *
 * 10k -- 08 40
 * 20k --
 *
**/
void lcd_disp_wave(uint16_t arr[]) {
    extern uint16_t MAX;
    // uart_lcd_ctr_wave_mid[7] = 0x80;
    // uart_lcd_ctr_wave_mid[6] = 0x00;
    /* 上下位置 */
    if (measure_freq < 7000.0f) {
        uart_lcd_ctr_wave_mid[15] = (uint8_t)(measure_freq / 1000.0f);
    } else if (measure_freq < 20000.0f) {
        uart_lcd_ctr_wave_mid[15] = (uint8_t)(measure_freq / 1000.0f );
    } else if (measure_freq < 20000.0f) {
        uart_lcd_ctr_wave_mid[15] = (uint8_t)(measure_freq / 1000.0f * 0.6);
    } else if (measure_freq < 40000.0f) {
        uart_lcd_ctr_wave_mid[15] = (uint8_t)(measure_freq / 1000.0f * 0.6);
    } else if (measure_freq < 60000.0f) {
        uart_lcd_ctr_wave_mid[15] = (uint8_t)(measure_freq / 1000.0f * 0.6);
    } else {
        uart_lcd_ctr_wave_mid[15] = (uint8_t)(measure_freq / 1000.0f * 0.5);
    }

    /* X 轴放大 */
    // uart_lcd_ctr_wave_mid[23] = 0x01;
    /* Y 轴放大 */
    uart_lcd_put_asciis(uart_lcd_ctr_wave_mid, 16);
    uart_lcd_put_asciis(uart_lcd_ctr_wave_mid, 16);
    uart_lcd_put_asciis(uart_lcd_ctr_wave_mid, 16);
    for (size_t i = 0; i < 1024; i++) {
        extern uint16_t lowV(uint16_t com);
        if (i > 50)
            lcd_disp_wave_point(lowV(arr[i]) );
        else
            lowV(arr[i]);
        // uart_lcd_put_asciis_test(uart_lcd_ctr_wave, 12);
        // uart_lcd_put_asciis_test(u8arr, 2);
        // SysCtlDelay(12000);
    }
}
void lcd_disp_wave_cls(void) {
    uart_lcd_put_asciis(uart_lcd_ctr_wave_cls, 8);
}
