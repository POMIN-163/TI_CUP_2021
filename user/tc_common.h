/*
 * @Author: Pomin
 * @Date: 2021-11-04 14:26:04
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-07 15:26:49
 * @Description:
 */
#if !defined(TC_COMMON)
#define TC_COMMON
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "ti/devices/msp432e4/driverlib/driverlib.h"
#include "hw_gpio.h"
#include "math.h"
extern uint32_t g_ui32SysClock;
extern uint32_t measure_status;
extern float measure_freq;
extern volatile bool bgetConvStatus;

extern uint32_t adc_fs;
#define  _N    1024         // 采样点数
#define  _Fs   (adc_fs)     // 采样频率
#define  _F    (_Fs / _N)   // 分辨率

#define BUFF_SIZE _N
extern uint32_t adc_activity;
extern uint16_t adc_buff[BUFF_SIZE];

extern uint32_t GPIO_OUT_TEM;
#define GPIO_OUT(port, pin, status) \
do { \
    GPIO_OUT_TEM = status; \
    HWREGBITW(&GPIO_OUT_TEM, pin) = status; \
    HWREG(GPIO_PORT##port##_BASE + (GPIO_O_DATA + (GPIO_PIN_##pin << 2))) = GPIO_OUT_TEM; \
} while(0)



#define BIT_REG(Reg, Bit) (*((uint32_t volatile*)                     \
    (0x42000000u + (((uint32_t) & (Reg) - (uint32_t)0x40000000u) << 5) +  \
    (((uint32_t)(Bit)) << 2))))

#define PAout(n) BIT_REG (GPIOA->ODR, n)
#define PBout(n) BIT_REG (GPIOB->ODR, n)
#define PCout(n) BIT_REG (GPIOC->ODR, n)
#define PDout(n) BIT_REG (GPIOD->ODR, n)
#define PEout(n) BIT_REG (GPIOE->ODR, n)
#define PFout(n) BIT_REG (GPIOF->ODR, n)
#define PGout(n) BIT_REG (GPIOG->ODR, n)
#define PHout(n) BIT_REG (GPIOH->ODR, n)

#define PAin(n) BIT_REG (GPIOA->IDR, n)
#define PBin(n) BIT_REG (GPIOB->IDR, n)
#define PCin(n) BIT_REG (GPIOC->IDR, n)
#define PDin(n) BIT_REG (GPIOD->IDR, n)
#define PEin(n) BIT_REG (GPIOE->IDR, n)
#define PFin(n) BIT_REG (GPIOF->IDR, n)
#define PGin(n) BIT_REG (GPIOG->IDR, n)
#define PHin(n) BIT_REG (GPIOH->IDR, n)


#endif // TC_COMMON
