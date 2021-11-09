/*
 * @Author: Pomin
 * @Date: 2021-11-04 14:48:03
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-04 18:59:19
 * @Description:
 */
#include "tc_led.h"

void led_init(void) {
    // Enable the GPIO port that is used for the on-board LED.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    /* Configure the GPIO PN0-PN1 as output */
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, (GPIO_PIN_0 | GPIO_PIN_1));
    /* Configure the GPIO PF0 PF4 as output */
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (GPIO_PIN_0 | GPIO_PIN_4));

    GPIO_OUT(F, 0, 1);
    GPIO_OUT(F, 4, 1);
    GPIO_OUT(N, 0, 1);
    GPIO_OUT(N, 1, 1);

}

uint32_t led_read(void) {
    return 0;
}

void led_write(uint32_t value) {

}
