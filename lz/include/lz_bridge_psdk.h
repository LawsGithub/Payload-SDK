/**
 * @file lz_bridge_psdk.h
 * @brief 桥接层的 PSDK 侧：把 LzRoute 翻译成航点 V2 的任务结构体。
 *
 * ⚠️ 与 lz_bridge.h 分开是有意的 —— 这个头文件 include 了 PSDK 类型，
 * 而 lz_bridge.h 属于零依赖的 lz_core。**不要让 lz_core 的源文件包含本文件**，
 * 否则"lz_core 不依赖 PSDK"这条分界线就破了，PC 侧测试也就跑不起来了。
 *
 * ## 本文件里有两套东西，别搞混
 *
 * 1. **`LzBridge_UploadKmzV3()` —— 本项目走的路**
 *    M4T 上 Waypoint 用的就是 V3（V2 只支持 M300/M350，见 API-MAP §〇）。
 *
 * 2. **`LzBridge_FillWaypointV2()` —— M300/M350 的备用实现**
 *    M4T 上**用不了**（官方文档：Waypoint 2.0 currently only supports
 *    Matrice 300 RTK and Matrice 350 RTK）。保留它是因为：换机型时能直接用上，
 *    而且它示范了"逐点云台绝对方位角"在结构体接口里怎么表达
 *    （`T_DJIGimbalRotation.absYawModeRef`），与 wpml 的
 *    `gimbalYawRotateEnable` 是同一个概念的两种写法。
 *
 * ⚠️ 该实现早期版本的注释曾断言"KMZ 对第三方负载没意义"—— **该断言已证伪**
 * （`gimbalRotate` 是独立于相机的动作，见 API-MAP §一）。
 */

#ifndef LZ_BRIDGE_PSDK_H
#define LZ_BRIDGE_PSDK_H

#include <stdbool.h>

#include "dji_waypoint_v2.h"
#include "dji_waypoint_v2_type.h"
#include "dji_waypoint_v3.h"

#include "lz_plan.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 由调用方持有、由本模块分配的两块内存。
 *
 * 为什么不让调用方自己 malloc：`T_DjiWayPointV2MissionSettings` 里有
 * 两个**裸指针**（`mission` 与 `actionList.actions`），它们的内存必须活到
 * UploadMission 返回之后。把分配与释放配对放在一处，比让每个调用点
 * 各自记得 malloc/free 可靠。
 */
typedef struct {
    T_DjiWaypointV2 *waypoints;
    T_DJIWaypointV2Action *actions;
    uint16_t waypointCount;
    uint16_t actionCount;
} LzWaypointV2Buffers;

void LzWaypointV2Buffers_Init(LzWaypointV2Buffers *buffers);
void LzWaypointV2Buffers_Free(LzWaypointV2Buffers *buffers);

/**
 * @brief 把航线填进航点 V2 的任务结构体
 *
 * 每个航点生成一个**到达该点时触发**的云台动作，把云台 yaw 旋到
 * `wp->gimbalYawDeg`（**相对正北的绝对方位角**）。这是绕飞的核心：
 * 飞机在圆周上跑，光轴始终盯着杆心。
 *
 * @param route    规划好的航线
 * @param profile  绕飞剖面（提供全局速度与结束动作等）
 * @param takeoff  起飞点（提供航点坐标的参考点）
 * @param settings [out] 任务结构体；其内部指针指向 buffers 里的内存
 * @param buffers  [out] 由本函数分配，用完调 LzWaypointV2Buffers_Free
 */
LzStatus LzBridge_FillWaypointV2(const LzRoute *route,
                                 const LzOrbitProfile *profile,
                                 const LzGeo *takeoff,
                                 T_DjiWayPointV2MissionSettings *settings,
                                 LzWaypointV2Buffers *buffers);

/**
 * @brief 把 KMZ 文件上传给飞机并开始执行（**本项目实际使用的入口**）
 *
 * KMZ 由 `LzBridge_ExportKmz()`（零依赖侧）生成到磁盘，
 * 这里只负责读文件与调 PSDK —— 生成与上传分开，
 * 意味着 KMZ 的正确性可以在桌面上用解包器验证，不必上飞机。
 *
 * ## 返回值刻意分成两段
 *
 * 本函数内部是**两个语义完全不同的步骤**，它们的失败必须能被区分：
 *
 * | 返回 | 含义 | 该去查什么 |
 * |---|---|---|
 * | `LZ_ERR_IO` | 本地文件读不出来 | 磁盘、路径 |
 * | `LZ_ERR_UPLOAD` | 文件读到了，但**飞机拒收** | KMZ 内容、链路 |
 * | `LZ_ERR_START` | 上传成功了，但**任务启动被拒** | 飞行状态、RC 档位、GPS… |
 *
 * ⚠️ 早期版本这三种情况都返回 `LZ_ERR_IO`，于是调用方只能笼统地说
 * "上传失败：文件读写失败" —— **而实际死因往往是启动被拒**。这与
 * `.dpk` 安装器把"应用提前退出"误报成"凭据错误"是同一个形状：
 * 一个返回值承载多种失败，调用方必然误报。
 *
 * 想拿到**启动失败的具体原因**（如 `0x302` = 当前 RC 模式下无法启动），
 * 在返回 `LZ_ERR_START` 后调用 `LzBridge_LogStartPreconditions()`。
 *
 * @param kmzPath            KMZ 文件路径
 * @param startImmediately   true = 上传后立即 DjiWaypointV3_Action(START)
 */
LzStatus LzBridge_UploadKmzV3(const char *kmzPath, bool startImmediately);

/**
 * @brief 订阅"能否启动航点"所需的那几个飞机状态话题（只需调一次）
 *
 * ⚠️ **必须在 `DjiCore_ApplicationStart()` 之后调用。**
 * 官方文档（`.psdk-apiref/docs/cn/20.basic-function/50.fc-subscription.md`）原文：
 *
 *   "请勿在 main() 函数中调用本接口，请在用户线程中调用本接口，
 *    启动调度器后，该接口将正常运行。"
 *
 * 实测 2026-09-20：放在 `ApplicationStart()` **之前**调会让它在错误时机初始化。
 * **返回 SUCCESS 不代表调用合法** —— 这与下面那条经验是同一件事的两面。
 *
 * ## ⚠️ 数据靠**回调**收，不要用 `DjiFcSubscription_GetLatestValueOfTopic`
 *
 * 实测（2026-09-20，M4T + 妙算3 + PSDK 3.16.0-beta）：
 *
 *     gdb:  SIGSEGV
 *     #0  DjiDataSubscriptionDds_v3_GetLastValueOfTopic ()
 *     #1  DjiDataSubscription_GetLastValueOfTopic ()
 *     #2  DjiFcSubscription_GetLatestValueOfTopic ()
 *
 * **崩在 SDK 内部。** 已排除的嫌疑：读太快（sleep 3s 仍崩）、传 NULL 回调
 * （传了也崩）、没数据（回调 10 秒 518 次，数据一直在到）、初始化时机
 * （已在 ApplicationStart 之后）。⇒ 这条路在本组合上**不可用**，
 * 改为回调里自己缓存。别再回头试 getter。
 *
 * 订阅是一次性的（头文件明写 "one topic can not be subscribed repeatedly"），
 * 所以不能放进每次现调的诊断函数里。
 */
LzStatus LzBridge_InitStartDiagnostics(void);

/**
 * @brief 把与"能否启动航点"相关的飞机状态打进日志
 *
 * 在 `DjiWaypointV3_Action(START)` **失败之后**调用，输出 RC 档位、飞行状态、
 * GPS 卫星数、起飞点等 —— 这些正是官方错误码表里 `0x0301`~`0x0309` 那一段
 * 各自对应的前置条件。
 *
 * ## ⚠️ 为什么是"打状态"而不是"取错误码"—— 能力边界，别在这里加假接口
 *
 * 飞机给出的具体原因（如 `770 = 0x302`"当前 RC 模式下无法启动"）
 * **PSDK 只写进它自己的日志，不通过任何 API 暴露** `[V]` 已核实：
 *
 *   - `dji_waypoint_v3.h` 全部 **6 个**导出函数，没有任何错误码查询接口
 *   - 状态回调 `T_DjiWaypointV3MissionState` 只有 state / wayLineId / index
 *   - `dji_error.h` 连"错误码→字符串"的运行时函数都没有，只有编译期宏表
 *   - 那行 `error_code: 770` 出自 `dji_waypoint_v3.c:497` 的 USER_LOG_ERROR，
 *     是 SDK 内部打印的，没有配套 getter
 *
 * 它就在 SDK 日志里，**读日志就能看到**；本函数解决的是另一件事：
 * 操作员在室外看的是 Pilot 2 浮窗，不是 SDK 日志。
 */
void LzBridge_LogStartPreconditions(void);

/**
 * @brief 把关键前置条件压成一行短消息，供 Pilot 2 浮窗显示
 *
 * 操作员在室外看不到 SDK 日志，而且浮窗有 2KB/s 的上限。
 * 只放"飞行状态 / RC 值 / GPS fixState / 卫星数"四项 —— 这几项
 * 恰好对应官方错误码表 0x0301~0x0309 里最可能的那几个。
 *
 * @return 静态缓冲里的字符串，调用方不需释放，下次调用会被覆盖
 */
const char *LzBridge_StartDiagSummary(void);

/** @brief 航点任务的启停（转发 DjiWaypointV3_Action，避免调用方直接依赖 PSDK） */
LzStatus LzBridge_StopMissionV3(void);

/**
 * @brief 取飞机当前位置（供"记录飞机位"按钮用）
 *
 * @param out [out] 单位是**度**的 WGS84 坐标
 * @return `LZ_OK`；`LZ_ERR_NOT_READY` = 还没收到过融合位置数据
 *
 * ## 为什么必须走这个函数，而不是让调用方自己去读话题
 *
 * 两个理由，都是踩过的坑：
 *
 * 1. **`DjiFcSubscription_GetLatestValueOfTopic` 在本组合上必崩**
 *    （SIGSEGV，栈在 `DjiDataSubscriptionDds_v3_GetLastValueOfTopic` 内部）。
 *    数据靠订阅回调写进静态缓存，这里读的就是那个缓存。
 * 2. **`TOPIC_POSITION_FUSED` 的经纬度单位是 `rad`，不是度**
 *    （头文件 `dji_fc_subscription.h:1015-1016` 原文 `unit: rad`）。
 *    换算收在这一处，避免每个调用点各自记得乘 `180/π` ——
 *    忘了乘的后果是"记录的杆位跑到几内亚湾"，而坐标看上去完全合法，
 *    直到飞机飞过去才发现。
 */
LzStatus LzBridge_GetCurrentPosition(LzGeo *out);

/**
 * @brief 飞机当前位置是否已就绪（收到过数据）
 *
 * 单独一个查询而不是让调用方看 `LzBridge_GetCurrentPosition` 的返回码：
 * 按钮的**回执文案**要能区分"没定位"与"读失败"，而返回码只有一种。
 */
bool LzBridge_HasCurrentPosition(void);

#ifdef __cplusplus
}
#endif

#endif /* LZ_BRIDGE_PSDK_H */
