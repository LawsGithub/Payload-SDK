/**
 * @file wt_square_mission.h
 * @brief 绕行人正方形航线的上传、启动、监控与中止（PSDK Waypoint V3）。
 *
 * 规划与 KMZ 在 wt_core（`wt_ped_square` + `WtBridge_ExportPedSquareKmz`），
 * 进度判定也在 wt_core（`wt_mission_watch`）。本文件只做有副作用的事：
 * 调 PSDK、把任务状态回调的结果缓存起来交给判定逻辑。
 *
 * ## 用法（都在主循环线程里调，不能放进 PSDK 回调线程）
 *
 * ```
 * WtSquareMission_Begin(kmz, start)   Init → 注册状态回调 → 上传 →（START）
 * 循环：WtSquareMission_Poll(&watch, now)   把最新状态喂给判定
 *       中止时 WtSquareMission_Stop()       STOP：飞机退出航线、原地悬停
 * WtSquareMission_End()               DeInit
 * ```
 *
 * ## 为什么每次都 Init / DeInit
 *
 * 风机作业（`WtRunner_ExecuteTurbine`）也是「用时 Init、用完 DeInit」。两边都
 * 这样做，谁先谁后都不会把对方的回调注册冲掉。
 */

#ifndef WT_SQUARE_MISSION_H
#define WT_SQUARE_MISSION_H

#include <stdbool.h>
#include <stdint.h>

#include "wt_mission_watch.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化航点模块、上传 KMZ，start 为 true 时紧接着 START。
 *
 * 失败时内部已 DeInit，调用方不要再调 End。成功后必须配对调用 End。
 */
bool WtSquareMission_Begin(const char *kmzPath, bool start);

/** @brief 把回调缓存里的最新任务状态交给判定逻辑（每个控制周期调一次） */
void WtSquareMission_Poll(WtMissionWatch *watch, uint32_t nowMs);

/** @brief 中止：下发 STOP。飞机退出航线、原地悬停，由操作员接管 */
bool WtSquareMission_Stop(void);

/** @brief 收尾：DeInit。任务已结束或已 STOP 之后调 */
void WtSquareMission_End(void);

#ifdef __cplusplus
}
#endif

#endif /* WT_SQUARE_MISSION_H */
