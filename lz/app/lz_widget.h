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
 * ## 六个控件（对应 widget_config.json 的 index）
 *
 * | index | 类型          | 名称     | 作用 |
 * |---|---|---|---|
 * | 0 | switch        | 绕飞     | **0 = 停止（急停），1 = 上传航线并开始** |
 * | 1 | scale         | 半径m    | 0–100 百分比 → 映射到 [半径下限, 上限] |
 * | 2 | scale         | 高度m    | 同上，映射到高度范围 |
 * | 3 | button        | 记录飞机位 | 把当前位置记为绕飞圆心 |
 * | 4 | button        | 记录激光点 | 激光测距点记为绕飞圆心 |
 * | 5 | int_input_box | 航点数   | **直接填个数**，夹到 [MIN, MAX] |
 *
 * ⚠️ 航点数用输入框而不是滑杆：滑杆只有 0–100 的整数档，而航点数是有界的
 * 小整数（3–64），输入框能让操作员直接填 16、24，不必心算档位。
 * 代价是**输入框可以填任意整数**，所以取值必须过
 * `LzPlan_ClampWaypointCount()` 夹取 —— 见 `LzWidget_GetWaypointCount()`。
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
 *
 * ⚠️ **任务正常结束也一样不返航** —— `finishAction=gotoFirstWaypoint`，
 * 飞机停在航线起始点（杆旁边）悬停。两条路径的收尾行为一致，
 * 操作员无论怎样结束都需要手动接管。
 */

#ifndef LZ_WIDGET_H
#define LZ_WIDGET_H

#include <stdbool.h>

#include "dji_typedef.h"

/* LzPoleRecordKind —— 记录请求的来源（飞机位 / 激光点） */
#include "lz_pole_source.h"

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
 * @brief 取操作员填的航点数，**已夹到 [LZ_PLAN_WAYPOINT_MIN, MAX]**
 *
 * 返回 int 而不是 double —— 航点数是计数量，不是连续量，
 * 与半径/高度那两个"百分比映射出的物理量"性质不同。
 *
 * ⚠️ 本函数保证返回值合法，但**调用方仍需把剖面的 waypointCount 交给
 * `LzPlan_Validate`** —— 夹取是"替操作员改成一个能用的值",
 * 校验是"起飞前最后一道关",两者职责不同。
 */
int LzWidget_GetWaypointCount(void);

/**
 * @brief 取走"操作员按了记录按钮"的请求（取走即清，只返回一次 true）
 *
 * ## 为什么记录动作要经过这个函数，而不是在回调里直接做
 *
 * ⚠️ **控件回调跑在 PSDK 的工作线程上，不能做耗时/阻塞操作。**
 *
 * 本项目为此踩过一次闪退（2026-09-22 实测）：`LzPole_RecordLaser()` 内部调
 * `DjiCameraManager_GetLaserRangingInfo()`，那是个**同步阻塞调用**
 * （要过 cmd 通道等相机回包，官方注明"Max execution time slightly larger
 * than 1200ms"）。在回调线程里调它会把 SDK 的链路线程卡住：
 *
 *     dji_msgq.c:227    semaphore wait timeout
 *     dji_linker.c:309  send msg to queue error
 *
 * 刷屏之后进程死掉 —— 日志里连 `LzPole_RecordLaser()` 的第一条日志
 * 都没打出来。**这也解释了为什么"记录飞机位"没事**：它只读订阅回调写好的
 * 静态缓存，不碰任何阻塞接口。
 *
 * ## 模式：回调只置标志，主循环干活
 *
 *     回调（PSDK 线程）        主循环 LzMission_Tick（我们的线程）
 *     ────────────────        ──────────────────────────────
 *     置 pending 标志    →    调阻塞接口、记录、落盘、回执
 *
 * 与绕飞开关的 `LzWidget_IsOrbitRequested()` 是同一个模式 ——
 * 只是激光那条路我一开始没照做。
 *
 * @param kind [out] 操作员按的是哪个按钮
 * @return true 表示取到了一次请求（此时 `kind` 有效）
 */
bool LzWidget_TakeRecordRequest(LzPoleRecordKind *kind);

/**
 * @brief 报告绕飞已结束，并提醒操作员开关仍在 ON 位
 *
 * ## ⚠️ 它**做不到**把开关程序化拨回 OFF —— 别再照旧描述写
 *
 * 旧版本这句注释写的是"把开关**程序化地**拨回 OFF"，而实现里明确说明做不到：
 * `LzWidget_SetWidgetValue` 是**我们自己**注册给 PSDK 的回调，被 Pilot 调用时
 * 才生效 —— 直接调它只改本地变量，**不会**改变 Pilot 界面上的开关位置。
 * PSDK 没有反向推控件状态的接口（`dji_widget.h` 9 个导出函数全是
 * Init / Reg* / FloatingWindow*，无 setter；`dji_widget_manager.h` 的
 * `SetWidgetState` 目标是机上挂载的负载，不是本应用的 UI）。
 *
 * 所以实际行为是：**开关保持 ON（持续可见），浮窗飘过一行"绕飞结束：…"**。
 * 两条信息互相矛盾，而持续可见的那条是**错的** —— 因此实现改用
 * "开关仍在 ON 位，请手动拨回"这种明确措辞。
 *
 * ## 收尾后飞机在哪
 *
 * `finishAction = gotoFirstWaypoint`（见 `lz_wpml.h`）—— 飞完最后一个航点
 * （与首点重合）即退出航线模式，**停在航线起始点悬停，不返航、不降落**。
 * 与拨 OFF 急停的行为一致：把"接下来怎么办"交给操作员。
 *
 * @param reason 结束原因，会写进浮窗消息
 */
void LzWidget_ReportOrbitFinished(const char *reason);

#endif /* LZ_WIDGET_H */
