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
#include <stddef.h>   /* size_t —— LzBridge_GimbalStatusStr 用 */

#include "lz_gimbal_status.h"   /* LZ_GIMBAL_STATUS_BUF —— 缓冲区尺寸的唯一真值 */

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

/**
 * @brief 起飞点海拔（米）；无数据时返回 **NAN**
 *
 * ## ⚠️ 参考面没有权威来源 —— 用之前先看这一段
 *
 * 头文件对这个话题的原文是 "Provides the altitude from sea level when the
 * aircraft last took off" + "also uses the ICAO model"，即**气压高度**。
 * 而 `LaserRangingInfo.altitude` 的头文件只写 `Unit: 0.1m`，
 * **没说参考面**。两者相减在参考面不一致时会系统性偏掉。
 *
 * ⇒ 调用方（`lz_pole_source.c`）把三个高程**都打进日志**做自洽校验，
 * 不靠猜。详见该文件的 `LzPole_RecordLaser()`。
 */
double LzBridge_GetHomeAltitudeM(void);

/**
 * @brief 飞机当前的椭球高（米）；无数据时返回 **NAN**
 *
 * 与 `LzBridge_GetCurrentPosition()` 同源，单位与参考面都是 WGS84 椭球。
 * 单独开一个是因为激光记点要**同一时刻**同时拿到位置与高度做自洽校验。
 */
double LzBridge_GetFusedAltitudeM(void);

/**
 * @brief 机头偏航（真北 0°、顺时针为正），度；无数据时返回 **NAN**
 *
 * ## 为什么需要它
 *
 * M4T 的云台 pan 软限位 ±60° 是**相对机头**的，不是绝对方位角
 * （见 `lz_plan.h` 的 `LZ_GIMBAL_YAW_MIN/MAX_DEG`）。所以横向照准判
 * "够不够得到"时必须拿「目标偏航角 − 机头偏航角」来比。
 *
 * ## 怎么算的
 *
 * 订阅 `TOPIC_QUATERNION`（body FRD → ground NED 的旋转，Hamilton 约定）：
 *
 *     yaw = atan2( 2(q0·q3 + q1·q2), 1 − 2(q2² + q3²) )
 *
 * ⚠️ NED 下 `atan2(y, x)` 给出的就是"从北起向**东**为正"的角，
 * 与 `LzGeo_BearingDeg()` **同约定**，所以不做翻转。
 *
 * 头文件给的精度：yaw `<3°`（校准良好的罗盘）—— 对"有没有超 ±60°"
 * 这个粗判足够，**不足以**用来做精细的偏航闭环。
 */
double LzBridge_GetBodyYawDeg(void);

/**
 * @brief 云台状态里**值得报警的那几位**，格式化成一行；无数据返回 false
 *
 * ## 为什么要有它（2026-10-01 上机实测逼出来的）
 *
 * 现场报的是「无人机在绕飞的时候云台显示**偏航角达到限位**，然后显示
 * **云台电机异常**，结束绕飞航线后又正常了」。
 *
 * 这两件事**飞机一直在报**（`TOPIC_GIMBAL_STATUS` 的 `yawLimited` 与
 * `escYawStatus` 位），但此前我们没订阅那条话题 —— 于是日志里一个字都没有，
 * 排查只能靠操作员口述现象。而"偏航顶限位"与"电机异常"是**两个不同的位**，
 * 口述里分不开，混在一起就定位不到成因。
 *
 * ⇒ 订阅 `GIMBAL_STATUS`（10Hz）并在日志里报出来。判据全在飞机侧，
 * 这里只做翻译，不做判断。
 *
 * ## 为什么是"变化时调用"而不是"每拍调用"
 *
 * 调用方只在**这一行内容变了**时报一次 —— 浮窗带宽上限 2 KB/s，
 * 每拍报一条会把它灌满（本项目已因此踩过两次：716 条刷屏堵死控件回执、
 * 去重表发送失败也标记已发）。
 *
 * ⚠️ **缓冲区尺寸用 `LZ_GIMBAL_STATUS_BUF`，不要自己写数字**：七项全报警时
 * 最长的一行是 `俯仰限位 横滚限位 偏航限位 俯仰电机异常 横滚电机异常
 * 偏航电机异常 陀螺故障` = **108 字节 + NUL**，而 128 是留了余量的取值。
 * 给 96 会在最坏情况下**丢掉最后一项**（`陀螺故障`）—— 而"全报警"恰恰是
 * 最需要完整一行的时候。少一项的诊断信息同样会把人引偏。
 *
 * @param buf  [out] 至少 `LZ_GIMBAL_STATUS_BUF` 字节
 * @param size 缓冲大小
 * @return true = 填好了；false = 还没收到过云台状态（**不是**"云台正常"）
 */
bool LzBridge_GimbalStatusStr(char *buf, size_t size);

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
