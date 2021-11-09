/*
 * Copyright (c) 2012-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * */
/*
 * ======== task.c ========
 *
 * Task Management functions
 *
 */

#include <stdbool.h>
#include <stdlib.h>

#include <netmain.h>

#include <xdc/std.h>
#include <xdc/runtime/Error.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

extern void ti_ndk_config_global_taskCreateHook(Task_Handle h);
extern void ti_ndk_config_global_taskExitHook(Task_Handle h);

/*
 * Hook ID
 * The Hook ID is used to set our environment pointer for
 * slot zero. It must be valid!! (Check in SetEnv function)
 */
static int32_t hookId = 0xffffffff;
static bool hookOK = false;

/*-------------------------------------------------------------------- */
/* TaskInit() - Called from NETCTRL init */
/*-------------------------------------------------------------------- */
void _TaskInit()
{
    hookOK = true;
}

/*-------------------------------------------------------------------- */
/* TaskShutdown() - Called from NETCTRL shutdown */
/*-------------------------------------------------------------------- */
void _TaskShutdown()
{
    hookOK = false;
}

/*-------------------------------------------------------------------- */
/* TaskBlock() */
/* Block a task */
/*-------------------------------------------------------------------- */
void TaskBlock(void *h)
{
    Task_setPri((Task_Handle)h, -1);
}

/*
 * ======== ti_ndk_os_threadStub ========
 * Wrapper function used by TaskCreate(). This function conforms to the
 * SYS/BIOS Task function signature:
 *
 *     void taskFunction(uintptr_t arg0, uintptr_t arg1)
 *
 * This function will call fdOpenSession/fdCloseSession automatically for the
 * user, calling the user's thread function in between.
 */
static void ti_ndk_os_threadStub(uintptr_t arg0, uintptr_t arg1)
{
    ti_ndk_os_TaskArgs *args = (ti_ndk_os_TaskArgs *)arg0;

    /*
     * Copy args out of TaskArgs object into our Task's stack. This way we can
     * free args before the user thread function is called.
     */
    void (*taskFunction)() = args->arg0;
    uintptr_t argument1 = args->arg1;
    uintptr_t argument2 = args->arg2;

    free(args);

    /* open the file descriptor session for the user here */
    fdOpenSession(TaskSelf());

    /* call user thread function */
    taskFunction(argument1, argument2);

    /* close the file descriptor session for the user here */
    fdCloseSession(TaskSelf());
}

/*-------------------------------------------------------------------- */
/* TaskCreate() */
/* Create a task */
/*-------------------------------------------------------------------- */
/* ARGSUSED */
void *TaskCreate(void (*pFun)(), char *Name, int Priority, uint32_t StackSize,
        uintptr_t Arg1, uintptr_t Arg2, uintptr_t argReserved)
{
    Task_Params params;
    Task_Handle task;
    Error_Block eb;
    ti_ndk_os_TaskArgs *taskArgs;

    taskArgs = malloc(sizeof(ti_ndk_os_TaskArgs));
    if (!taskArgs) {
        /* Error: couldn't alloc task function args */
        return (NULL);
    }
    taskArgs->arg0 = pFun;
    taskArgs->arg1 = Arg1;
    taskArgs->arg2 = Arg2;

    Error_init(&eb);
    Task_Params_init(&params);
    params.instance->name = Name;
    params.priority = Priority;
    params.stackSize = StackSize;
    params.arg0 = (uintptr_t)taskArgs;

    task = Task_create((Task_FuncPtr)ti_ndk_os_threadStub, &params, &eb);

    if (!task) {
        free(taskArgs);
        return (NULL);
    }

    return ((void *)task);
}

/*-------------------------------------------------------------------- */
/* TaskDestroy() */
/* Destroy a task */
/*-------------------------------------------------------------------- */
void TaskDestroy( void *h )
{
    if (h == TaskSelf()) {
        TaskExit();
    }
    else {
        Task_delete((Task_Handle *)&h);
    }
}

/*-------------------------------------------------------------------- */
/* TaskSetEnv */
/* Sets the task's Environment pointer */
/*-------------------------------------------------------------------- */
void TaskSetEnv( void *h, int Slot, void *hEnv )
{
    if (Slot) {
        return;
    }

    if (hookId == 0xffffffff) {
        DbgPrintf(DBG_ERROR, "TaskSetEnv: FATAL: NDK_hookInit() must be set in BIOS Task module config");
        return;
    }

    if (!hookOK) {
        DbgPrintf(DBG_ERROR,"TaskSetEnv: FATAL: NDK not initialized");
        return;
    }

    /* Use Task hook context (set up in the NDK configuration) for TLS */
    Task_setHookContext((Task_Handle)h, (int32_t)hookId, hEnv);
}

/*-------------------------------------------------------------------- */
/* TaskGetEnv */
/* Gets the task's Environment pointer */
/*-------------------------------------------------------------------- */
void *TaskGetEnv( void *h, int Slot )
{
    if (Slot) {
        return (0);
    }

    if (hookId == 0xffffffff) {
        DbgPrintf(DBG_ERROR, "TaskGetEnv: FATAL: NDK_hookInit() must be set in BIOS Task module config");
        return (0);
    }

    if (!hookOK) {
        DbgPrintf(DBG_ERROR,"TaskGetEnv: FATAL: NDK not initialized");
        return (0);
    }

    /* Use Task hook context (set up in the NDK configuration) for TLS */
    return ((void *)Task_getHookContext((Task_Handle)h, (int32_t)hookId));
}

/*-------------------------------------------------------------------- */
/* TaskExit() */
/* Exits and destroys a task */
/*-------------------------------------------------------------------- */
void TaskExit()
{
    Task_exit();
}

/*-------------------------------------------------------------------- */
/* TaskGetPri() */
/* Get a task's priority */
/*-------------------------------------------------------------------- */
int TaskGetPri(void *h)
{
    return ((int)Task_getPri((Task_Handle)h));
}

/*-------------------------------------------------------------------- */
/* TaskSetPri() */
/* Set a task's priority */
/*-------------------------------------------------------------------- */
int TaskSetPri(void *h, int priority)
{
    return (Task_setPri((Task_Handle)h, (int32_t)priority));
}

/*-------------------------------------------------------------------- */
/* TaskSelf() */
/* Return handle of task itself  */
/*-------------------------------------------------------------------- */
void *TaskSelf()
{
    return ((void *)Task_self());
}

/*-------------------------------------------------------------------- */
/* TaskSleep() */
/* Put a task into sleep  */
/*-------------------------------------------------------------------- */
void TaskSleep(uint32_t delay)
{
     Task_sleep(delay);
}

/*-------------------------------------------------------------------- */
/* TaskYield() */
/* Yield task  */
/*-------------------------------------------------------------------- */
void TaskYield()
{
     Task_yield();
}

/*
 * This hookInit() function is required for Thread Local Storage when using BIOS
 */
void NDK_hookInit(int32_t id)
{
    hookId = id;
}

/*
 * This hookCreate() function is only used by BIOS applications using RTSC
 * config. It is needed to init the environment to zero.
 *
 * If configured to, call fdOpenSession via Task create hook to allow BSD
 * sockets code to work "as is."  Note that this function will only be run in
 * Task context for dynamically created Tasks.
 */
void NDK_hookCreate(Task_Handle h)
{
    if (hookId != 0xffffffff) {
        Task_setHookContext(h, (int32_t)hookId, 0);

        /* open the file descriptor session automatically (if configured to) */
        ti_ndk_config_global_taskCreateHook(h);
    }
}

/*
 * This hookExit() function is only used by BIOS applications using RTSC config
 *
 * If configured to, call fdCloseSession via Task exit hook to allow BSD
 * sockets code to work "as is."
 *
 * The Task exit hook should always run in the context of the Task that is
 * exiting (i.e. Task context).
 */
void NDK_hookExit(Task_Handle h)
{
    if (hookId != 0xffffffff) {
        /* close the file descriptor session automatically (if configured to) */
        ti_ndk_config_global_taskExitHook(h);
    }
}
