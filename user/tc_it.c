/*
 * @Author: Pomin
 * @Date: 2021-11-04 15:49:03
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-09 11:25:51
 * @Description:
 */

/* 关开中断 */
// MAP_IntMasterDisable();
// MAP_IntMasterEnable();

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char* pcFilename, uint32_t ui32Line) { }
#endif
