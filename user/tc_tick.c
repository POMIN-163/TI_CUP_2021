/*
 * @Author: Pomin
 * @Date: 2021-11-04 14:48:03
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-06 09:43:17
 * @Description:
 */
#include "tc_tick.h"

//****************************************************************************
//
// System clock rate in Hz.
//
//****************************************************************************
uint32_t g_ui32SysClock;

void tick_init(void) {
    // Set the clocking to run directly from the crystal at 120MHz.
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
        SYSCTL_OSC_MAIN |
        SYSCTL_USE_PLL |
        SYSCTL_CFG_VCO_480), 120000000);

    /* Enable the SysTick timer to generate an interrupt every 1 ms */
    MAP_SysTickPeriodSet(g_ui32SysClock);
    MAP_SysTickIntEnable();
    MAP_SysTickEnable();
}
void delay_ms(uint32_t xms) {
    SysCtlDelay(g_ui32SysClock / (1000 * 3 * xms));
}
void SysTick_Handler(void) {


}
