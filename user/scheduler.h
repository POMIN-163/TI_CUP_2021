/*
 * @Description:
 * @Version: 2.0
 * @Autor: Pomin
 * @Date: 2021-09-10 16:59:58
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2021-11-04 16:38:12
 */
#ifndef __SCHEDULER_
#define __SCHEDULER_
#include "./tc_common.h"

typedef void(* Func)(void);

typedef struct TaskType {
    void(* task)(void);
    uint32_t cycle;
    uint32_t lastRun;
    uint32_t runTime;
    struct TaskType *next;
} taskTypeDef;

#define CLK_TYPE volatile uint32_t    // 定义调度器时钟依赖的计时变量数据类型
#define CLK_TICK gTime1      // 定义调度器时钟到一个计时变量上面

extern CLK_TYPE CLK_TICK;

uint8_t schedulerLoad(taskTypeDef* handle, uint32_t cycleLength, Func timeoutTask);
uint8_t schedulerDel(taskTypeDef* handle);
void schedulerRun (void);
void schedulerTick(void);


#endif

