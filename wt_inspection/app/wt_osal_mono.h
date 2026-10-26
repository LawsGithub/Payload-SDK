/**
 * @file wt_osal_mono.h
 * @brief OSAL 里三个与时间有关的函数，换成单调时钟版：GetTimeMs / GetTimeUs /
 *        SemaphoreTimedWait。其余 OSAL 函数照用官方样例。
 *
 * ## 为什么
 *
 * 官方样例（`samples/sample_c/platform/linux/common/osal/osal.c`）三者都用
 * `gettimeofday`，也就是墙上时钟。妙算3 的 RTC 没有电池：开机先回到上次保存的
 * 时间，进 Pilot 2 校时后一下跳到真实时间。2026-10-26 实测，应用运行中跳了约
 * 5 小时：一次编码耗时被算成 18267601 ms。
 *
 * 跳变那一刻会出两类问题：
 *
 * - 用 GetTimeMs 算的超时一齐误触发：行人会话上限、按键 5 s 有效、航线进度判定……
 * - SDK 内部的带超时等待（SemaphoreTimedWait → `sem_timedwait`，绝对截止时间按
 *   墙上时钟算）：往前跳立刻超时；往回跳则可能一等就是几个小时
 *
 * `CLOCK_MONOTONIC` 不受 settimeofday / 校时跳变影响（clock_gettime(2)）。
 * 信号量仍由样例的 `Osal_SemaphoreCreate` 创建（`sem_t`），这里只换等待用的时钟
 * （`sem_clockwait`，glibc ≥ 2.30；设备是 2.31）。
 */

#ifndef WT_OSAL_MONO_H
#define WT_OSAL_MONO_H

#include <stdint.h>

#include <dji_platform.h>
#include <dji_typedef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 单调时钟毫秒数（开机起算；uint32 约 49.7 天回绕，调用方一律用无符号差） */
T_DjiReturnCode WtOsal_GetTimeMs(uint32_t *ms);

/** @brief 单调时钟微秒数（开机起算） */
T_DjiReturnCode WtOsal_GetTimeUs(uint64_t *us);

/** @brief 带超时等待信号量，截止时间按单调时钟算。语义同样例：超时或出错返回 SYSTEM_ERROR */
T_DjiReturnCode WtOsal_SemaphoreTimedWait(T_DjiSemaHandle semaphore, uint32_t waitTimeMs);

#ifdef __cplusplus
}
#endif

#endif /* WT_OSAL_MONO_H */
