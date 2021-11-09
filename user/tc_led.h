/*
 * @Author: Pomin
 * @Date: 2021-11-04 14:48:10
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-04 16:01:26
 * @Description:
 */
#if !defined(TC_LED)
#define TC_LED
#include "tc_common.h"

// typedef struct
// {
//     __IO uint32_t value;
//     __IO uint8_t  status;
// } tc_led_t;

// extern tc_led_t tc_led;

void led_init(void);
uint32_t led_read(void);
void led_write(uint32_t value);

#endif // TC_LED
