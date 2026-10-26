/**
 * @file wt_widget.h
 * @brief Pilot 2 自定义控件：操作员在遥控器上控制作业，无需 SSH 改配置。
 *
 * ## 为什么需要它
 *
 * 停机位相位（`park_phase`）是**现场实测值**，而作业中改配置只能靠 SSH ——
 * 飞机在天上时这条路根本不存在。控件让操作员在 Pilot 2 界面上直接改，
 * 并且能在起飞前就设好，不必下机改文件重启。
 *
 * ## 五个控件，都在 main_interface
 *
 * `main_interface` → Pilot 相机视图左侧「PSDK」菜单，**飞行中操作**。
 *
 * | index | 类型   | 名称     | 作用 |
 * |---|---|---|---|
 * | 0 | switch | 开始作业 | **0 = 中止，1 = 开始** |
 * | 1 | button | 中止作业 | 立即请求中止当前作业（也中止行人瞄准） |
 * | 2 | switch | 干跑     | 1 = 只上传不启动（dry_run） |
 * | 3 | scale  | 停用角   | 0–100 映射到 0–359° |
 * | 4 | switch | 行人瞄准 | **1 = 开始一次瞄准打点会话，0 = 中止** |
 *
 * ⚠️ **数值输入用 `scale` 滑块，不用 `int_input_box`。** 输入框放 main 界面时
 * SDK 收下但 Pilot 不渲染（liangzhourenwu 实测）；放 config 界面（Payload
 * Settings）同样不渲染（本项目 2026-10-20 实测）。这条是踩出来的，不要
 * "优化"回去。
 *
 * ## 回调纪律：只置标志，主循环干活
 *
 * ⚠️ **控件回调跑在 PSDK 的工作线程上，不能做耗时/阻塞操作。**
 * liangzhourenwu 项目为此踩过一次闪退（2026-09-22）：在回调里调了
 * `DjiCameraManager_GetLaserRangingInfo()`（同步阻塞约 1.2 s），
 * 把 SDK 的链路线程卡死，日志刷 `semaphore wait timeout` 后进程死掉。
 *
 * 本模块所有回调都只写一个 volatile 标志或一个整数，动作由主循环取走执行。
 * 上传 KMZ、规划航线这些都要几秒，绝不能塞进回调。
 */

#ifndef WT_WIDGET_H
#define WT_WIDGET_H

#include <stdbool.h>

#include "dji_typedef.h"

/**
 * @brief 初始化控件模块（配置 + handler 注册），并创建状态推送线程
 *
 * 调用顺序：必须在 `DjiCore_Init()` **之后**、`DjiCore_ApplicationStart()`
 * **之前**。晚于 ApplicationStart 的话 Pilot 拉不到控件列表。
 */
T_DjiReturnCode WtWidget_Init(void);

/**
 * @brief 停掉状态推送线程
 *
 * ⚠️ 模块里**没有** `DjiWidget_DeInit()` —— `dji_widget.h` 全部导出函数
 * 只有 `DjiWidget_Init`，没有反初始化。官方样例也从不停它。
 * 所以这里只能停自己创建的线程，SDK 侧的注销由 `DjiCore_DeInit()` 统一处理。
 */
void WtWidget_Stop(void);

/**
 * @brief 往 Pilot 2 浮窗推一条状态消息
 *
 * 这是**室外看不到日志时的唯一反馈通道** —— 操作员在遥控器上就能看到
 * 程序在干什么。约束：单条长度有限，带宽 2 KB/s。
 */
void WtWidget_PostMessage(const char *fmt, ...);

/**
 * @brief 操作员是否已请求开始作业（开关拨到 ON 且尚未被消费）
 *
 * 与 `WtWidget_TakeAbortRequest()` 同为"回调只置标志"模式的读端。
 */
bool WtWidget_IsStartRequested(void);

/**
 * @brief 取走"操作员按了中止"的请求（取走即清，只返回一次 true）
 */
bool WtWidget_TakeAbortRequest(void);

/**
 * @brief 取停用角（度）
 *
 * 返回操作员在 Payload Settings 里填的值，**已夹到 [0, 359]**。
 *
 * ⚠️ 未填过时返回**配置文件的初值**；配置里也没写（负数）则返回负值，
 * 表示"未知"，沿用运行器既有的 12 点方向假设 + 拒绝自动启动逻辑。
 *
 * @return 停用角；负值表示未知
 */
double WtWidget_GetParkPhaseDeg(void);

/**
 * @brief 设置停用角的初值（由 main.c 在载入配置后调用）
 *
 * 让控件初始显示与配置文件一致 —— 否则操作员打开界面看到 0，
 * 而程序实际用配置里的 45，两边对不上。
 */
void WtWidget_SetParkPhaseInitial(double deg);

/**
 * @brief 干跑开关是否打开（true = 只上传不启动）
 *
 * 初值同样来自配置文件的 `dry_run`，操作员可在飞行中改。
 */
bool WtWidget_IsDryRun(void);

/** @brief 设置干跑的初值（由 main.c 在载入配置后调用） */
void WtWidget_SetDryRunInitial(bool dryRun);

/**
 * @brief 操作员是否请求了行人瞄准（「行人瞄准」开关在 ON）
 *
 * 会话进行中主循环每拍都查它：拨回 OFF 即中止。
 */
bool WtWidget_IsPedRequested(void);

/**
 * @brief 清除行人瞄准请求（会话结束后调用，避免立刻又开一轮）
 *
 * ⚠️ 与 `WtWidget_ClearStartRequest` 同理：只改本地状态，**界面上的开关
 * 仍在 ON**，要在浮窗里提示操作员拨回。
 */
void WtWidget_ClearPedRequest(void);

/**
 * @brief 清除"开始作业"请求
 *
 * 作业结束后调用，避免那个仍停在 ON 位的开关让状态机重新起飞。
 *
 * ⚠️ 它**做不到**把 Pilot 上的开关拨回 OFF —— `WtWidget_SetWidgetValue`
 * 是我们自己注册给 PSDK 的回调，直接调它只改本地变量，不会改变界面。
 * PSDK 没有反向推控件状态的接口（`dji_widget.h` 全是 Init / Reg* /
 * FloatingWindow*，无 setter）。所以界面上开关会保持 ON，必须在浮窗里
 * 明确告诉操作员"请手动拨回"。
 */
void WtWidget_ClearStartRequest(void);

#endif /* WT_WIDGET_H */
