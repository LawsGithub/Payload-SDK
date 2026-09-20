/**
 * @file lz_widget.h
 * @brief Pilot 2 自定义控件：操作员在这里控制绕飞。
 *
 * ## 为什么控件是基础功能，而航点是高级功能
 *
 * 自定义控件被归在 API Reference 的 `basic-function/widget.html` 下，
 * M4T + 妙算3 的基础功能全支持 —— **没有"是否需要申请高级权限"那层不确定性**，
 * 比航点那条路好走。
 *
 * ## 三个控件（对应 widget_config.json 的 index）
 *
 * | index | 类型   | 名称   | 作用 |
 * |---|---|---|---|
 * | 0 | switch | 绕飞   | **0 = 停止（急停），1 = 上传航线并开始** |
 * | 1 | scale  | 半径m  | 0–100 百分比 → 映射到 [半径下限, 上限] |
 * | 2 | scale  | 高度m  | 同上，映射到高度范围 |
 *
 * ## 安全语义（本项目的约定）
 *
 * - **拨到 ON**：生成航线 → 上传 → 立即开始
 * - **拨到 OFF**：**立即 STOP，不是 PAUSE**
 *
 * 选 STOP 而非 PAUSE 的理由：绕飞中操作员想停，绝大多数情况是发现了异常
 * （有人进入场地、杆上有异物、飞机姿态不对）。此时需要的是"停下来"，而不是
 * "悬停在那里等我决定" —— 后者会让飞机继续停在杆附近，未必更安全。
 *
 * ⚠️ 但 STOP 之后飞机停在原地，**不会自动返航**。若操作员不接管，飞机就悬停在
 * 那里耗电。这是刻意的：把"接下来怎么办"的决定权交给人，而不是程序替他决定
 * （自动返航可能穿越操作员正想避开的区域）。
 */

#ifndef LZ_WIDGET_H
#define LZ_WIDGET_H

#include <stdbool.h>

#include "dji_typedef.h"

/**
 * @brief 初始化控件模块（配置 + handler 注册），并创建状态推送线程
 *
 * 调用顺序：必须在 `DjiCore_Init()` **之后**、
 * 与其他模块的 Init/Reg 一起，且在 `DjiCore_ApplicationStart()` 之前。
 */
T_DjiReturnCode LzWidget_Init(void);

/**
 * @brief 停掉状态推送线程
 *
 * ⚠️ 模块里**没有** `DjiWidget_DeInit()` —— 核实过 `dji_widget.h` 全部 9 个
 * 导出函数，只有 `DjiWidget_Init`，没有反初始化。官方样例也从不停它。
 * 所以这里只能停自己创建的线程，SDK 侧的注销由 `DjiCore_DeInit()` 统一处理。
 */
void LzWidget_Stop(void);

/**
 * @brief 往 Pilot 2 浮窗推一条状态消息
 *
 * 这是**室外看不到日志时的唯一反馈通道** —— 操作员在遥控器上就能看到
 * 程序在干什么。约束：单条长度有限，带宽 2 KB/s。
 */
void LzWidget_PostMessage(const char *fmt, ...);

/** @brief 操作员是否已请求绕飞（拨到 ON 且已被主循环消费） */
bool LzWidget_IsOrbitRequested(void);

/** @brief 取当前控件上的半径/高度设定（已从百分比映射为实际值） */
double LzWidget_GetRadiusM(void);
double LzWidget_GetAltitudeM(void);

/**
 * @brief 报告绕飞已结束，把开关**程序化地**拨回 OFF
 *
 * 用途：航点执行完毕或出错时调用，让界面上的开关与实际状态一致。
 * 若不回弹，操作员会看到"开关是 ON 但飞机已经停了"，容易误判。
 */
void LzWidget_ReportOrbitFinished(const char *reason);

#endif /* LZ_WIDGET_H */
