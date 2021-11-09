/*
 * @Author: Pomin
 * @Date: 2021-11-04 12:27:57
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-09 11:27:47
 * @Description: 2021-电赛 A 题，失真度测量仪
 */
#include "tc_uart.h"
#include "tc_tick.h"
#include "tc_lcd.h"
#include "tc_led.h"
#include "tc_key.h"
#include "tc_freq.h"
#include "tc_timer.h"
#include "scheduler.h"
#include "tc_bluetooth.h"
#include "cm_backtrace/cm_backtrace.h"

uint16_t adc_buff[BUFF_SIZE];

char dbg_buff[500];
#define DBG_LOG(fmt, ...) \
    do { \
        sprintf(dbg_buff, fmt, ##__VA_ARGS__); \
        UARTprintf(dbg_buff); \
        UARTprintf("\n"); \
    } while (0)
// #define DBG_LOG(fmt, ...) UARTprintf(fmt, ##__VA_ARGS__)
void measure_main(void);

/**
 * @brief 单次测量 (启动 -> 测量频率 -> 设置采样率 -> 采样 -> FFT -> THD计算)
 *
**/
typedef enum {
    STATUS_NONE = 0,
    STATUS_FREQ,
    STATUS_FREQ_OK,
    STATUS_SAMP,
    STATUS_SAMP_OK,
    STATUS_FFT,
    STATUS_FFT_OK,
    STATUS_THD,
} measure_status_t;
uint16_t disp_adc_buff[4096] = { 0 };
void measure_main(void) {
    size_t i = 0;
    float thd_result = 0.0f;
    extern float xiebo_u[5];
    measure_status = 1;

    while (measure_status) {
        if (measure_status == STATUS_NONE) {
            // measure_status = STATUS_FREQ_OK;
            // break;
            measure_status++;
        } else if (measure_status == STATUS_FREQ) {
            DBG_LOG("初始化测频率");
            for (i = 0; i < 4; i++) {
                lcd_disp_thd(0);
                lcd_write(0x0110, 0.0f);
                lcd_write(0x0120, 0.0f);
                lcd_write(0x0130, 0.0f);
                lcd_write(0x0140, 0.0f);
                lcd_write(0x0150, 0.0f);
                lcd_write(0x0160, 0.0f);
                lcd_disp_wave_cls();
            }
            lcd_beep();
            freq_init();
            for (i = 0; i < BUFF_SIZE; i++) {
                disp_adc_buff[i] = adc_buff[i];
            }
            // DBG_LOG("测频率中");
            // freq_read();
            measure_status++;
        } else if (measure_status == STATUS_FREQ_OK) {
            DBG_LOG("FREQ: %f Hz", measure_freq);
            DBG_LOG("获取频率完成，准备采样");
            measure_status++;
        } else if (measure_status == STATUS_SAMP) {
            if (measure_freq < 3200.0f) {
                adc_fs = 32000;
            } else if (measure_freq < 6400.0f) {
                adc_fs = 64000;
            } else if (measure_freq < 12800.0f) {
                adc_fs = 128000;
            } else if (measure_freq < 25600.0f) {
                adc_fs = 256000;
            } else if (measure_freq < 51200.0f) {
                adc_fs = 512000;
            } else if (measure_freq < 102400.0f) {
                adc_fs = 1024000;
            } else {
                adc_fs = 1536000;
            }
            DBG_LOG("开始采样 采样率: %ld 采样点数: %d", adc_fs, BUFF_SIZE);

            MAP_TimerDisable(TIMER0_BASE, TIMER_A);
            MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, (g_ui32SysClock / adc_fs));
            MAP_TimerEnable(TIMER0_BASE, TIMER_A);
            measure_status++;
        } else if (measure_status == STATUS_SAMP_OK) {
            // DBG_LOG("采样完毕");
            // adc_read();
            // for (size_t i = 0; i < Sample_Num;i++) {
            //     DBG_LOG("ADC: %d,", adc_buff[i]);
            //     delay_ms(5000);
            // }
            measure_status++;
        } else if (measure_status == STATUS_FFT) {
            // for (size_t i = 0; i < 5; i++) {
            // }
            fft_read_useless(); /* 第一次舍弃掉 */
            DBG_LOG("开始傅里叶变换");
            fft_read();
            measure_status++;
        } else if (measure_status == STATUS_FFT_OK) {
            // DBG_LOG("傅里叶变换完成");
            // extern kiss_fft_cpx   FFT_256PointOut[_N / 2];
            // for (size_t i = 0; i < BUFF_SIZE; i++) {
            //     DBG_LOG("F: %f,", FFT_256PointOut[i].r);
            //     delay_ms(5000);
            // }
            measure_status++;
        } else if (measure_status == STATUS_THD) {
            for (i = 1; i < 5; i++) {
                thd_result += xiebo_u[i] * xiebo_u[i];
            }
            thd_result = sqrt(thd_result);
            thd_result = thd_result / xiebo_u[0];
            DBG_LOG("计算 THD: %f", thd_result);
            measure_status++;
        } else {
            extern float Freq;
            lcd_beep();
            lcd_disp_thd(thd_result);
            lcd_write(0x0110, 1.0f);
            lcd_write(0x0120, xiebo_u[1] / xiebo_u[0]);
            lcd_write(0x0130, xiebo_u[2] / xiebo_u[0]);
            lcd_write(0x0140, xiebo_u[3] / xiebo_u[0]);
            lcd_write(0x0150, xiebo_u[4] / xiebo_u[0]);
            lcd_write(0x0160, Freq / 1000.0f);
            // for (i = 0; i < 1024;i++) {
            //     DBG_LOG("ADC: %d,", adc_buff[i]);
            //     delay_ms(5000);
            // }
            // if (measure_freq <= 3000.0f) {
            //     lcd_disp_wave(adc_buff);
            // }
            lcd_disp_wave(disp_adc_buff);
            bluetooth_write(disp_adc_buff, 1024);
            DBG_LOG("屏幕显示&蓝牙显示");

            measure_status = 0;
        }
    }
}

/**
 * @brief 按键扫描任务
 *
**/
uint8_t key_msg = 0;
taskTypeDef task_20ms_handle;
void task_20ms(void) {
    key_scan(&key_msg);
    if (key_msg) {
        // DBG_LOG("KEY : %d", key_msg);
        if (key_msg == 1) {
            adc_read();
            for (size_t i = 0; i < BUFF_SIZE; i++) {
                DBG_LOG("ADC: %d,", adc_buff[i]);
                delay_ms(5000);
            }
        }
        // DBG_LOG("SW1 : %ld SW2 : %ld", GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0), GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1) >> 1);
        measure_main();
        key_msg = 0;
    }
};
/**
 * @brief 闪灯任务
 *
**/
taskTypeDef task_500ms_handle;
void task_500ms(void) {
    static bool is_light = 1;
    is_light ^= 1;
    GPIO_OUT(F, 0, is_light);
    GPIO_OUT(F, 4, is_light);
};
/**
 * @brief 刷屏任务
 *
**/
taskTypeDef task_200ms_handle;
void task_200ms(void) {
};

/**
 * @brief
 *
**/
void dev_init(void) {
    led_init();
    key_init();
    tick_init();
    uart_init();
    timer_init();
    lcd_init();
    adc_init();
    bluetooth_init();
    // DBG_LOG("CLK : %d MHz", g_ui32SysClock / 1000000);
    DBG_LOG("这是一个失真的失真度测量装置");
}

int main(void) {
    dev_init();
    /* 加载任务进程 */
    schedulerLoad(&task_20ms_handle, 19, task_20ms);
    schedulerLoad(&task_500ms_handle, 499, task_500ms);
    schedulerLoad(&task_200ms_handle, 199, task_200ms);
    key_msg = 2;
    while (1) {
        schedulerRun();
    }
}
