/*
 * @Author: Pomin
 * @Date: 2021-11-04 14:48:10
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-04 15:57:41
 * @Description:
 */
#if !defined(tc_uart)
#define tc_uart
#include "tc_common.h"
#include "uartstdio.h"

void uart_init(void);
uint32_t uart_read(void);
void uart_write(const uint8_t * pui8Buffer);

#endif // tc_uart
