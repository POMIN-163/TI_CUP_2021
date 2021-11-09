/*
 * @Author: Pomin
 * @Date: 2021-11-04 14:48:03
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-07 11:20:56
 * @Description:
 */
#include "tc_freq.h"
#include "uartstdio.h"

char _buff[500];
#define _LOG(fmt, ...) \
    do { \
        sprintf(_buff, fmt, ##__VA_ARGS__); \
        UARTprintf(_buff); \
        UARTprintf("\n"); \
    } while (0)

extern volatile struct FFT_DATA FFTDATA[Sample_Num];

void freq_init(void) {
    int i, j = 0;
    uint16_t max;

    // MAP_TimerDisable(TIMER0_BASE, TIMER_A);
    // MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, (g_ui32SysClock / 1024000));
    // MAP_TimerEnable(TIMER0_BASE, TIMER_A);
    adc_read();
    for (i = 0; i < Sample_Num; i++) {
        FFTDATA[i].dataR = adc_buff[i];
    }
    InitForFFT();
    FFT_N();
    max = 0;
    for (i = 1;i < Sample_Num / 2;i++) {
        if (result[i].U > max) {
            max = result[i].U;
            j = i;
        }
    }
    measure_freq = _Fs * j / _N;
}

uint32_t freq_read(void) {
    return 0;
}

void freq_write(uint32_t value) {

}
