/*
 * @Author: Pomin
 * @Date: 2021-11-04 14:48:10
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-07 15:25:27
 * @Description:
 */
#if !defined(TC_ADC)
#define TC_ADC
#include "tc_common.h"

extern uint16_t srcBuffer[1024];
#define signal_in srcBuffer[0]

void adc_init(void);
uint32_t adc_read(void);
void adc_write(uint32_t value);
uint16_t lowV(uint16_t com);

#endif // TC_ADC
