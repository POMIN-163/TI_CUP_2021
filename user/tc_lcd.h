/*
 * @Author: Pomin
 * @Date: 2021-11-04 14:48:10
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-07 05:27:36
 * @Description:
 */
#if !defined(TC_LCD)
#define TC_LCD
#include "tc_common.h"

void lcd_init(void);
void lcd_beep(void);
void lcd_rst(void);
uint32_t lcd_read(void);
void lcd_write(uint16_t addr, float value);
void lcd_disp_thd(float value);
void lcd_disp_wave(uint16_t arr64[]);
void lcd_disp_wave_cls(void);

#endif // TC_LCD
