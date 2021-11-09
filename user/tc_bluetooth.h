/*
 * @Author: Pomin
 * @Date: 2021-11-04 14:48:10
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-07 17:54:31
 * @Description:
 */
#if !defined(TC_BLUETOOTH)
#define TC_BLUETOOTH
#include "tc_common.h"

void bluetooth_init(void);
uint32_t bluetooth_read(void);
void bluetooth_write_arr(uint16_t dat_arr[], uint16_t len);
void bluetooth_write(uint16_t data_arr[], uint16_t len);

#endif // TC_BLUETOOTH
