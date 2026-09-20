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
 * @param kmzPath            KMZ 文件路径
 * @param startImmediately   true = 上传后立即 DjiWaypointV3_Action(START)
 */
LzStatus LzBridge_UploadKmzV3(const char *kmzPath, bool startImmediately);

/** @brief 航点任务的启停（转发 DjiWaypointV3_Action，避免调用方直接依赖 PSDK） */
LzStatus LzBridge_StopMissionV3(void);

#ifdef __cplusplus
}
#endif

#endif /* LZ_BRIDGE_PSDK_H */
