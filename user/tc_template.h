/*
 * @Author: Pomin
 * @Date: 2021-11-04 14:48:10
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-04 16:01:26
 * @Description:
 */
#if !defined(TC_DEF)
#define TC_DEF
#include "tc_common.h"

// typedef struct
// {
//     __IO uint32_t value;
//     __IO uint8_t  status;
// } tc_template_t;

// extern tc_template_t tc_template;

void template_init(void);
uint32_t template_read(void);
void template_write(uint32_t value);

#endif // TC_DEF
