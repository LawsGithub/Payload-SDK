/**
 * @file lz_mission.h
 * @brief 绕飞作业的执行编排：从"操作员拨开关"到"飞机跑完航线"。
 *
 * 本模块是**作业决策的唯一集中地**。控件回调只记录意图，真正的动作
 * （规划 → 生成 KMZ → 上传 → 启动 → 监控 → 收尾）全部发生在这里。
 */

#ifndef LZ_MISSION_H
#define LZ_MISSION_H

#include "dji_typedef.h"

/**
 * @brief 初始化任务模块（注册航点状态回调）
 *
 * 在 `DjiCore_ApplicationStart()` 之前调用。
 */
T_DjiReturnCode LzMission_Init(void);

/** @brief 反初始化 */
T_DjiReturnCode LzMission_DeInit(void);

/**
 * @brief 主循环的一拍：检查操作员意图并推进状态机
 *
 * 由 main 的业务循环周期调用（建议 100 ms 一次）。
 *
 * 状态机：
 *   IDLE  --操作员拨 ON-->  规划并上传  --成功-->  RUNNING
 *   RUNNING --航点结束/出错/操作员拨 OFF--> 收尾，回 IDLE
 */
void LzMission_Tick(void);

/** @brief 当前是否在执行绕飞（供外部查询） */
bool LzMission_IsRunning(void);

#endif /* LZ_MISSION_H */
