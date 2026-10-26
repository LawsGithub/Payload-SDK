/**
 * @file wt_osal_mono.c
 * @brief OSAL 计时函数的单调时钟实现（为什么见头文件）。
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE /* sem_clockwait（PSDK 应用的编译参数里通常已经带了） */
#endif

#include "wt_osal_mono.h"

#include <dji_error.h>

#include <semaphore.h>
#include <time.h>

static uint64_t WtOsal_MonoUs(void)
{
    struct timespec ts;

    (void)clock_gettime(CLOCK_MONOTONIC, &ts);

    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

T_DjiReturnCode WtOsal_GetTimeMs(uint32_t *ms)
{
    if (ms == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    *ms = (uint32_t)(WtOsal_MonoUs() / 1000u);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode WtOsal_GetTimeUs(uint64_t *us)
{
    if (us == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    *us = WtOsal_MonoUs();

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode WtOsal_SemaphoreTimedWait(T_DjiSemaHandle semaphore, uint32_t waitTimeMs)
{
    struct timespec deadline;

    if (semaphore == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    (void)clock_gettime(CLOCK_MONOTONIC, &deadline);
    deadline.tv_sec += (time_t)(waitTimeMs / 1000u);
    deadline.tv_nsec += (long)(waitTimeMs % 1000u) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_sec += 1;
        deadline.tv_nsec -= 1000000000L;
    }

    if (sem_clockwait((sem_t *)semaphore, CLOCK_MONOTONIC, &deadline) != 0) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
