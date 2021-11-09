/*
 * @Author: Pomin
 * @Date: 2021-11-04 14:48:03
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-04 19:28:40
 * @Description:
 */
#include "tc_key.h"

typedef struct Button {
    uint8_t last_status;
    uint8_t curr_status;
    uint8_t (*scan)(void);
    void (*event)(void);
} Button;

#define BUTTON_NUM 2
Button key_arr[BUTTON_NUM];

uint8_t sw1_get(void) { return GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0); }
uint8_t sw2_get(void) { return GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1) >> 1; }

void key_init(void) {
    /* Enable the clock to the GPIO Port J and wait for it to be ready */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    MAP_GPIODirModeSet(GPIO_PORTJ_BASE, GPIO_PIN_0|GPIO_PIN_1, GPIO_DIR_MODE_IN);
    MAP_GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0|GPIO_PIN_1,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    for (int i = 0; i < BUTTON_NUM; i++) {
        key_arr[i].curr_status = key_arr[i].last_status = 1;
    }
    key_arr[0].scan = sw1_get;
    key_arr[1].scan = sw2_get;
}

void key_scan(uint8_t *msg) {
    for (int i = 0; i < BUTTON_NUM; i++) {
        key_arr[i].last_status = key_arr[i].curr_status;
        key_arr[i].curr_status = key_arr[i].scan();  // 从寄存器获取当前电平
        if (!key_arr[i].last_status && key_arr[i].curr_status) {
            *msg = i + 1;
            // printf("key %d press\r\n", i);
        }
    }
}
