/*
 * @Author: Pomin
 * @Date: 2021-11-04 14:48:10
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-04 16:01:26
 * @Description:
 */
#if !defined(TC_TIMER)
#define TC_TIMER
#include "tc_common.h"

// typedef struct
// {
//     __IO uint32_t value;
//     __IO uint8_t  status;
// } tc_timer_t;

// extern tc_timer_t tc_timer;

void timer_init(void);
uint32_t timer_read(void);
void timer_write(uint32_t value);

#endif // TC_TIMER
