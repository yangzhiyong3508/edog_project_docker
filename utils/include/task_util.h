#ifndef TASK_UTIL_H
#define TASK_UTIL_H

#include "los_task.h"
#include <stdio.h>


/**
 * @brief 通用任务创建函数
 *
 * @param taskFunc     任务函数指针（函数原型：void func(void)）
 * @param taskName     任务名（字符串）
 * @param stackSize    栈大小（单位：字节）
 * @param priority     任务优先级（数值越小优先级越高）
 * @return UINT32      返回 LOS_OK 表示创建成功，否则返回 LiteOS 错误码
 *
 * 示例：
 *     CreateTask(MqttTask, "MqttTask", 8192, 25);
 */
UINT32 CreateTask(void (*taskFunc)(void), const char *taskName, UINT32 stackSize, UINT16 priority);


#endif // TASK_UTIL_H
