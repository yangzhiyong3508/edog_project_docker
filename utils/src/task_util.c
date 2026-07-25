#include "los_task.h"
#include <stdio.h>

UINT32 CreateTask(void (*taskFunc)(void), const char *taskName, UINT32 stackSize, UINT16 priority)
{
    UINT32 taskID;
    TSK_INIT_PARAM_S taskParam = {0};

    if (taskFunc == NULL || taskName == NULL) {
        printf("CreateTask: Invalid parameter\n");
        return LOS_NOK;
    }

    taskParam.pfnTaskEntry = (TSK_ENTRY_FUNC)taskFunc;
    taskParam.uwStackSize  = stackSize;
    taskParam.pcName       = (char *)taskName;
    taskParam.usTaskPrio   = priority;

    UINT32 ret = LOS_TaskCreate(&taskID, &taskParam);
    if (ret == LOS_OK) {
        return LOS_OK;
    } else {
        printf("Failed to create task [%s], error code: %u\n", taskName, ret);
        return ret;
    }
}
