/*
 * @Author: Pomin
 * @Date: 2021-11-04 14:48:03
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-05 15:10:22
 * @Description:
 */
#include "tc_timer.h"
#include "scheduler.h"

// tc_timer_t tc_timer;

//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the UART.
//
//*****************************************************************************
uint32_t g_ui32Flags;
/* timer3 -- tick, timer1 -- leds, timer0 -- dma adc, timer2 -- freq */
void timer_init(void) {
    // Enable the peripherals used by this example.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    // Enable processor interrupts.
    MAP_IntMasterEnable();

    // Configure the two 32-bit periodic timers.
    MAP_TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(TIMER3_BASE, TIMER_A, g_ui32SysClock / 1000);
    MAP_TimerLoadSet(TIMER1_BASE, TIMER_A, g_ui32SysClock / 2);

    // Setup the interrupts for the timer timeouts.
    MAP_IntEnable(INT_TIMER3A);
    MAP_IntEnable(INT_TIMER1A);
    MAP_TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    // Enable the timers.
    MAP_TimerEnable(TIMER3_BASE, TIMER_A);
    MAP_TimerEnable(TIMER1_BASE, TIMER_A);
}

uint32_t timer_read(void) {
    return 0;
}

void timer_write(uint32_t value) {

}

//*****************************************************************************
//
// The interrupt handler for the first timer interrupt.
//
//*****************************************************************************
void
TIMER3A_IRQHandler(void) {
    // Clear the timer interrupt.
    MAP_TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

    schedulerTick();

    MAP_IntMasterDisable();
    MAP_IntMasterEnable();
}

//*****************************************************************************
//
// The interrupt handler for the second timer interrupt.
//
//*****************************************************************************
volatile bool led_blink = 1;
void
TIMER1A_IRQHandler(void) {
    //
    // Clear the timer interrupt.
    //
    MAP_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    led_blink ^= 1;

    GPIO_OUT(N, 0, led_blink);
    GPIO_OUT(N, 1, led_blink);
}
