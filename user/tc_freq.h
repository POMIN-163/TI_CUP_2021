/*
 * @Author: Pomin
 * @Date: 2021-11-04 14:48:10
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-04 16:01:26
 * @Description:
 */
#if !defined(TC_FREQ)
#define TC_FREQ
#include "tc_common.h"
#include "tc_fft.h"

void freq_init(void);
uint32_t freq_read(void);
void freq_write(uint32_t value);

#endif // TC_FREQ
