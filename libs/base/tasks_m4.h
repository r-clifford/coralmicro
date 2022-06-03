#ifndef LIBS_BASE_TASKS_M4_H_
#define LIBS_BASE_TASKS_M4_H_

#include "libs/FreeRTOS/FreeRTOSConfig.h"

#define IPC_TASK_PRIORITY (configMAX_PRIORITIES - 1)
#define CONSOLE_TASK_PRIORITY (configMAX_PRIORITIES - 1)
#define APP_TASK_PRIORITY (configMAX_PRIORITIES - 1)
#define CAMERA_TASK_PRIORITY (configMAX_PRIORITIES - 1)

#if defined(__cplusplus)
#define VALIDATE_TASK_PRIORITY(prio)                            \
    static_assert(0 <= (prio) && (prio) < configMAX_PRIORITIES, \
                  "Invalid value for task priority")

VALIDATE_TASK_PRIORITY(IPC_TASK_PRIORITY);
VALIDATE_TASK_PRIORITY(CONSOLE_TASK_PRIORITY);
VALIDATE_TASK_PRIORITY(CONSOLE_TASK_PRIORITY);
VALIDATE_TASK_PRIORITY(CAMERA_TASK_PRIORITY);
#endif  // __cplusplus

#endif  // LIBS_BASE_TASKS_M4_H_
