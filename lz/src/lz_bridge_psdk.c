/**
 * @file lz_bridge_psdk.c
 * @brief 桥接层 PSDK 侧：LzRoute → T_DjiWayPointV2MissionSettings。
 *
 * 本文件有两套实现，见头文件里的说明：
 *
 *   1. LzBridge_UploadKmzV3()   ← **本项目走的路**（M4T 用 Waypoint V3）
 *   2. LzBridge_FillWaypointV2() ← M300/M350 的备用（M4T 上不可用）
 *
 * ️ 早期版本在这里断言过"KMZ 对第三方负载没意义"，**该断言已证伪**：
 * wpml 的 `gimbalRotate` 是**独立于相机**的动作，且有
 * `gimbalYawRotateEnable` + `gimbalRotateMode=absoluteAngle` —— 正是绕飞所需。
 * 当时只看了一个动作实例（takePhoto）就下了全局结论。
 * 核实：解包 samples/.../waypoint_v3_test_file.kmz 看 gimbalRotate 的参数块。
 */

#include "lz_bridge_psdk.h"
#include "lz_types.h"
#include "platform/lz_sdk_log_watch.h"

#include <dji_fc_subscription.h>
#include <dji_logger.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 云台角在协议里是 int16，单位 0.1 度 */
#define LZ_GIMBAL_DEG_SCALE 10.0

/* 云台俯仰角的硬件限位，转成协议单位 */
#define LZ_GIMBAL_PITCH_MIN_DEG (-90.0)
#define LZ_GIMBAL_PITCH_MAX_DEG (30.0)

void LzWaypointV2Buffers_Init(LzWaypointV2Buffers *buffers)
{
    if (buffers == NULL) {
        return;
    }
    buffers->waypoints = NULL;
    buffers->actions = NULL;
    buffers->waypointCount = 0;
    buffers->actionCount = 0;
}

void LzWaypointV2Buffers_Free(LzWaypointV2Buffers *buffers)
{
    if (buffers == NULL) {
        return;
    }
    free(buffers->waypoints);
    free(buffers->actions);
    LzWaypointV2Buffers_Init(buffers);
}

/** 把角度转成协议单位并夹到 int16 范围内 */
static int16_t lz_deg_to_int16(double deg)
{
    double scaled = deg * LZ_GIMBAL_DEG_SCALE;
    if (scaled > 32767.0) {
        scaled = 32767.0;
    }
    if (scaled < -32768.0) {
        scaled = -32768.0;
    }
    /* 四舍五入到最近整数，避免 -0.04° 被截断成 0 而丢掉符号 */
    return (int16_t)lround(scaled);
}

LzStatus LzBridge_FillWaypointV2(const LzRoute *route,
                                 const LzOrbitProfile *profile,
                                 const LzGeo *takeoff,
                                 T_DjiWayPointV2MissionSettings *settings,
                                 LzWaypointV2Buffers *buffers)
{
    if (route == NULL || profile == NULL || takeoff == NULL ||
        settings == NULL || buffers == NULL) {
        return LZ_ERR_PARAM;
    }
    if (route->count == 0 || route->count > 65535) {
        /* missTotalLen 是 uint16，上限 65535 */
        return LZ_ERR_RANGE;
    }
    if (!LzGeo_IsValid(takeoff)) {
        return LZ_ERR_PARAM;
    }

    LzWaypointV2Buffers_Init(buffers);

    const uint16_t n = (uint16_t)route->count;

    /* 每个航点一个云台动作 —— 触发类型是"到达该点" */
    buffers->waypoints = calloc(n, sizeof(*buffers->waypoints));
    buffers->actions = calloc(n, sizeof(*buffers->actions));
    if (buffers->waypoints == NULL || buffers->actions == NULL) {
        LzWaypointV2Buffers_Free(buffers);
        return LZ_ERR_IO;
    }
    buffers->waypointCount = n;
    buffers->actionCount = n;

    memset(settings, 0, sizeof(*settings));

    /* ---- 任务级 ---- */
    settings->missionID = 1;
    settings->repeatTimes = 0;   /* 不重复，绕一圈就结束 */
    settings->finishedAction = DJI_WAYPOINT_V2_FINISHED_GO_HOME;
    /* maxFlightSpeed 是遥控器杆量能给的速度增量上限，范围 [2,15] */
    settings->maxFlightSpeed = 10.0f;
    settings->autoFlightSpeed = (float)profile->speedMs;
    settings->actionWhenRcLost = DJI_WAYPOINT_V2_MISSION_KEEP_EXECUTE_WAYPOINT_V2;
    settings->gotoFirstWaypointMode =
        DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_POINT_TO_POINT;
    settings->mission = buffers->waypoints;
    settings->missTotalLen = n;
    settings->actionList.actions = buffers->actions;
    settings->actionList.actionNum = n;

    for (uint16_t i = 0; i < n; ++i) {
        const LzWaypoint *wp = &route->points[i];

        /* ---- 航点本体 ---- */
        T_DjiWaypointV2 *pt = &buffers->waypoints[i];
        pt->longitude = wp->geo.longitudeDeg;
        pt->latitude = wp->geo.latitudeDeg;
        pt->relativeHeight = (float)wp->relativeAltM;
        /* 直线段飞行；航点间不做曲线过渡，绕飞是规则多边形，曲线过渡
         * 反而会让实际轨迹偏离算好的圆 */
        pt->waypointType = DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_IN_STRAIGHT_AND_STOP;
        /* 机头朝向：应指向杆心 —— 与 V3 路径保持一致。
         *
         * ⚠️ 这条 V2 路径**在 M4T 上不可用**（Waypoint 2.0 只支持
         * M300/M350），保留它是为换机型时能直接用。但**注释里的结论
         * 不能留错**：原注释写"云台 yaw 与机头朝向无关，机头怎么转
         * 都不影响光轴"，那条断言在 M4T 上已被证伪 —— 规范要求
         * gimbalYawRotateAngle 与 aircraftHeading 一致（M4 系列在列）。
         * 换到 M300/M350 时（那两个机型云台 yaw 可独立），
         * HEADING_MODE_AUTO + absYawModeRef=1 才是成立的组合。 */
        pt->headingMode = DJI_WAYPOINT_V2_HEADING_MODE_AUTO;
        /* T_DjiWaypointV2Config **只有** useLocalCruiseVel / useLocalMaxVel
         * 两个开关，没有配套的 local* 数值字段（核实：
         *   grep -n "useLocalCruiseVel\|localCruiseVel" dji_waypoint_v2_type.h
         * 只命中 useLocal* 两处）。逐点速度走 T_DjiWaypointV2.autoFlightSpeed，
         * 官方样例同样把这两个开关设为 0。 */
        pt->config.useLocalCruiseVel = 0;
        pt->config.useLocalMaxVel = 0;
        /* dampingDistance 只在 COORDINATE_TURN 且 >0 时参与提前转弯。
         * 绕飞要的是贴着算好的圆飞，不提前切角，故置 0。 */
        pt->dampingDistance = 0;
        pt->heading = 0.0f;
        /* turnMode 描述的是**机头**转向方向（不是航线方向，航线方向由航点
         * 顺序决定）。机头沿切线飞，转向方向自然跟着绕行方向。 */
        pt->turnMode = profile->clockwise
                           ? DJI_WAYPOINT_V2_TURN_MODE_CLOCK_WISE
                           : DJI_WAYPOINT_V2_TURN_MODE_COUNTER_CLOCK_WISE;
        pt->maxFlightSpeed = 10.0f;
        pt->autoFlightSpeed = (float)wp->speedMs;

        /* ---- 云台动作：到达该点时把光轴旋到杆心 ---- */
        T_DJIWaypointV2Action *act = &buffers->actions[i];
        act->actionId = i;
        act->trigger.actionTriggerType =
            DJI_WAYPOINT_V2_ACTION_TRIGGER_TYPE_SAMPLE_REACH_POINT;
        act->trigger.sampleReachPointTriggerParam.waypointIndex = i;
        /* terminateNum = 0 表示该动作只由这一个航点触发，不被别处终止 */
        act->trigger.sampleReachPointTriggerParam.terminateNum = 0;

        act->actuator.actuatorType = DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_GIMBAL;
        act->actuator.actuatorIndex = 0;
        act->actuator.gimbalActuatorParam.operationType =
            DJI_WAYPOINT_V2_ACTION_ACTUATOR_GIMBAL_OPERATION_TYPE_ROTATE_GIMBAL;

        T_DJIGimbalRotation *rot = &act->actuator.gimbalActuatorParam.rotation;
        rot->x = 0;
        rot->y = lz_deg_to_int16(wp->gimbalPitchDeg);
        rot->z = lz_deg_to_int16(wp->gimbalYawDeg);
        rot->ctrl_mode = 0;       /* 绝对位置控制 */
        rot->rollCmdIgnore = 1;   /* 绕飞不关心横滚 */
        rot->pitchCmdIgnore = 0;
        rot->yawCmdIgnore = 0;
        /* ★ 关键位：绝对 yaw 相对正北，而不是相对机头。
         *   绕飞中机头朝向一直在变，用相对机头的 yaw 需要每点重算，
         *   且机头一抖光轴就偏；相对正北则直接就是我们要的方位角。 */
        rot->absYawModeRef = 1;
        rot->durationTime = 0;
    }

    return LZ_OK;
}

/* ===========================================================================
 * V3：上传 KMZ 并执行 —— 本项目实际使用的入口
 *
 * 为什么不在这里顺便生成 KMZ：生成是零依赖的（lz_wpml + lz_kmz），
 * 放在 lz_core 里就能在桌面上用**独立解包器**验证。这里的职责只有
 * "读文件 + 调 PSDK"，越薄越好 —— 它是唯一无法在桌面上验证的一环。
 *
 * ## 返回值必须区分「上传失败」与「启动失败」
 *
 * 这两件事的排查方向完全不同：上传失败要查文件与链路，启动失败要查
 * 飞行状态与 RC 档位。早期版本两者都返回 LZ_ERR_IO，上层只能说
 * "上传航线失败：文件读写失败" —— 而实测 2026-09-20 的真实死因是
 * 上传成功、启动被拒（error_code 770 = 0x302），日志与报错完全对不上。
 * =========================================================================== */

/* ===========================================================================
 * 启动诊断：订阅飞机状态话题
 *
 * ## ⚠️ 为什么不用 DjiFcSubscription_GetLatestValueOfTopic —— 它必崩
 *
 * 2026-09-20 用独立探针（`lz_mission_probe`）在设备上定位到：
 *
 *     gdb:  Program received signal SIGSEGV
 *     #0  DjiDataSubscriptionDds_v3_GetLastValueOfTopic ()
 *     #1  DjiDataSubscription_GetLastValueOfTopic ()
 *     #2  DjiFcSubscription_GetLatestValueOfTopic ()
 *     #3  read_topic ()
 *     #4  main ()
 *
 * **崩在 SDK 内部，不是我们的代码。** 而且排查过程排除了以下所有嫌疑：
 *
 *   - 不是"订阅后读太快"：读之前 sleep 3 秒仍崩
 *   - 不是"格式化字符串 %u 对 uint8_t"：那是无害的栈读取
 *   - 不是"传 NULL 回调"：传了回调照样崩
 *   - 不是"没有数据"：回调计数 10 秒内 518 次，**数据一直在到**
 *   - 不是"初始化时机"：FcSubscription_Init 已在 ApplicationStart 之后
 *
 * 结论：`GetLatestValueOfTopic` 这条路在 M4T + 妙算3 + PSDK 3.16.0-beta 上
 * **不可用**。不再尝试绕过它 —— 换一条确定能走的路：
 *
 * ## 改用的办法：**回调里自己缓存**
 *
 * 回调能收到数据（上面 518 次是实证），那就让回调把值存进我们自己的
 * 静态变量。诊断时直接读自己的缓存，完全不碰 SDK 的 getter。
 *
 * 代价：缓存是回调线程写的，诊断是主线程读的 —— 有数据竞争。用
 * `volatile` + 每个话题一个"已收到过"标志处理：读到的可能是某一帧的
 * 快照，但每个字段单独看都是飞机真实报过的一个值。对"排查启动被拒"
 * 这个用途，够用。
 *
 * 之前那个版本（直接调 getter）会在启动失败时**把主程序带崩** ——
 * 那比没有诊断糟得多。这次改了之后，诊断再也不会拖垮主流程。
 * =========================================================================== */

/* 订阅是一次性的（头文件："one topic can not be subscribed repeatedly"） */
static bool s_startDiagSubscribed = false;

/* 回调写入 / 诊断读取的缓存 */
static volatile bool s_gotFlightStatus = false;
static volatile uint8_t s_flightStatus = 0;
static volatile bool s_gotRc = false;
static volatile T_DjiFcSubscriptionRC s_rc = {0};
static volatile bool s_gotGps = false;
static volatile T_DjiFcSubscriptionGpsDetails s_gps = {0};
static volatile bool s_gotFused = false;
static volatile T_DjiFcSubscriptionPositionFused s_fused = {0};
static volatile bool s_gotHome = false;
static volatile uint8_t s_home = 0;

/* 每个话题一个专用回调：长度校验 + 拷贝，写自己的缓存。
 * 分开写而不是用一个带 topic 参数的函数，是因为回调签名里没有 topic。 */
#define DEFINE_TOPIC_CB(name, dst, flag, type)                                    \
    static T_DjiReturnCode name(const uint8_t *data, uint16_t size,               \
                                const T_DjiDataTimestamp *timestamp)              \
    {                                                                             \
        (void)timestamp;                                                          \
        if (data == NULL || size < sizeof(type)) {                                \
            return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;                \
        }                                                                         \
        memcpy((void *)&(dst), data, sizeof(type));                               \
        (flag) = true;                                                            \
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;                              \
    }

DEFINE_TOPIC_CB(lz_cb_flight_status, s_flightStatus, s_gotFlightStatus, T_DjiFcSubscriptionFlightStatus)
DEFINE_TOPIC_CB(lz_cb_rc, s_rc, s_gotRc, T_DjiFcSubscriptionRC)
DEFINE_TOPIC_CB(lz_cb_gps, s_gps, s_gotGps, T_DjiFcSubscriptionGpsDetails)
DEFINE_TOPIC_CB(lz_cb_fused, s_fused, s_gotFused, T_DjiFcSubscriptionPositionFused)
DEFINE_TOPIC_CB(lz_cb_home, s_home, s_gotHome, T_DjiFcSubscriptionHomePointSetStatus)

LzStatus LzBridge_InitStartDiagnostics(void)
{
    if (s_startDiagSubscribed) {
        return LZ_OK;
    }

    const T_DjiReturnCode rc = DjiFcSubscription_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiFcSubscription_Init 失败 rc=0x%08X", (unsigned)rc);
        return LZ_ERR_IO;
    }

    /* 5 个话题、统一 10Hz。看着少是刻意的 —— 头文件两条硬上限：
     *   "types of subscription frequency ... less than or equal to 4"
     *   "data length sum of all topics of the same frequency ... <= 242"
     * 同一频率既省额度，也让各值是同一时刻的快照。
     *
     * ⚠️ 回调**必须传**（不能传 NULL）—— 我们靠它拿数据，
     * 因为 getter 不可用（见文件顶部说明）。 */
    struct {
        E_DjiFcSubscriptionTopic topic;
        DjiReceiveDataOfTopicCallback cb;
        const char *name;
    } items[] = {
        {DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, lz_cb_flight_status, "飞行状态"},
        {DJI_FC_SUBSCRIPTION_TOPIC_RC, lz_cb_rc, "RC"},
        {DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS, lz_cb_gps, "GPS 详情"},
        {DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, lz_cb_fused, "融合位置"},
        {DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_SET_STATUS, lz_cb_home, "起飞点状态"},
    };

    int okCount = 0;
    for (size_t i = 0; i < sizeof(items) / sizeof(items[0]); ++i) {
        const T_DjiReturnCode r =
            DjiFcSubscription_SubscribeTopic(items[i].topic, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                             items[i].cb);
        if (r == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            okCount++;
        } else {
            /* 订阅失败不阻断启动 —— 这个模块只服务于诊断 */
            USER_LOG_WARN("订阅 %s 失败 rc=0x%08X（诊断会缺这一项）",
                          items[i].name, (unsigned)r);
        }
    }

    s_startDiagSubscribed = (okCount > 0);
    USER_LOG_INFO("启动诊断已就绪（%d/5 个飞机状态话题订阅成功，10Hz，走回调缓存）",
                  okCount);
    return LZ_OK;
}

/* 浮窗摘要缓冲：回调写、这里格式化。静态的，避免每次失败都 malloc */
static char s_diagSummary[200];

const char *LzBridge_StartDiagSummary(void)
{
    if (!s_startDiagSubscribed) {
        snprintf(s_diagSummary, sizeof(s_diagSummary), "诊断未就绪");
        return s_diagSummary;
    }

    /* 只放最关键的几项。浮窗带宽 2KB/s，且操作员一眼要能看懂。 */
    const char *flight = "?";
    if (s_gotFlightStatus) {
        flight = (s_flightStatus == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) ? "空中" :
                 (s_flightStatus == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_ON_GROUND) ? "地面·电机转" :
                 (s_flightStatus == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_STOPED) ? "地面·停" : "?";
    }

    if (s_gotRc && s_gotGps) {
        snprintf(s_diagSummary, sizeof(s_diagSummary),
                 "%s RC=%d GPS状态=%.0f 卫星=%u",
                 flight, s_rc.mode, s_gps.fixState,
                 (unsigned)s_gps.totalSatelliteNumberUsed);
    } else if (s_gotRc) {
        snprintf(s_diagSummary, sizeof(s_diagSummary), "%s RC=%d GPS=无数据", flight, s_rc.mode);
    } else {
        snprintf(s_diagSummary, sizeof(s_diagSummary), "%s（RC/GPS 无数据）", flight);
    }
    return s_diagSummary;
}

void LzBridge_LogStartPreconditions(void)
{
    if (!s_startDiagSubscribed) {
        USER_LOG_WARN("启动诊断未就绪（订阅未成功），无法输出飞机状态");
        return;
    }

    USER_LOG_WARN("=== 航点启动被拒：以下是启动所需的前置条件 ===");

    /* 1. 飞行状态 —— 对应错误码 0x309(正在移动) / 0x30a(在地面但电机已启动) */
    if (s_gotFlightStatus) {
        const char *s = "未知";
        if (s_flightStatus == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_STOPED) {
            s = "地面·电机停转";
        } else if (s_flightStatus == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_ON_GROUND) {
            s = "地面·电机已转";
        } else if (s_flightStatus == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
            s = "空中";
        }
        USER_LOG_WARN("  飞行状态 = %u (%s)", (unsigned)s_flightStatus, s);
    } else {
        USER_LOG_WARN("  飞行状态 = 尚未收到数据");
    }

    /* 2. RC 档位 —— 对应错误码 0x302(当前 RC 模式下无法启动)
     *
     * ⚠️ 只打原始值，**不做档位名映射**。头文件只给了 M100 的参考
     *    [P: -8000, A: 0, F: 8000]，M4T 的取值没有文档。猜一个映射
     *    比不猜更糟：会把排查引到错误方向。 */
    if (s_gotRc) {
        USER_LOG_WARN("  RC mode = %d（请与遥控器档位对照：N/S/F 哪一个）", s_rc.mode);
    } else {
        USER_LOG_WARN("  RC = 尚未收到数据");
    }

    /* 3. 定位质量 —— 对应错误码 0x301(GPS 无效) / 0x308(RTK 未就绪) */
    if (s_gotGps) {
        /* fixState / hdop 单位是 0.01（头文件："unit: 0.01, eg: 100 = 1.00"） */
        USER_LOG_WARN("  GPS 定位状态 = %.0f (需 3 = 3D Fix)，HDOP = %.2f，卫星 = %u",
                      s_gps.fixState, s_gps.hdop / 100.0,
                      (unsigned)s_gps.totalSatelliteNumberUsed);
    } else {
        USER_LOG_WARN("  GPS 详情 = 尚未收到数据");
    }

    if (s_gotFused) {
        /* 经纬度单位是 **rad**（头文件原文），转成度再打，否则读的人会以为坏了 */
        USER_LOG_WARN("  融合位置 = %.7f, %.7f（卫星 %u）",
                      s_fused.latitude * 180.0 / M_PI,
                      s_fused.longitude * 180.0 / M_PI,
                      (unsigned)s_fused.visibleSatelliteNumber);
    }

    /* 4. 起飞点 —— 对应错误码 0x303(没有记录起飞点) */
    if (s_gotHome) {
        USER_LOG_WARN("  起飞点 = %s",
                      (s_home == DJI_FC_SUBSCRIPTION_HOME_POINT_SET_STATUS_SUCCESS)
                          ? "已记录" : "未记录");
    } else {
        USER_LOG_WARN("  起飞点状态 = 尚未收到数据");
    }

    USER_LOG_WARN("=== 对照官方错误码表 0x0301~0x0309 逐项排查 ===");
}

LzStatus LzBridge_UploadKmzV3(const char *kmzPath, bool startImmediately)
{
    if (kmzPath == NULL) {
        return LZ_ERR_PARAM;
    }

    /* ---- 第一段：本地读文件。失败一律是 LZ_ERR_IO ---- */
    FILE *fp = fopen(kmzPath, "rb");
    if (fp == NULL) {
        USER_LOG_ERROR("打不开 KMZ 文件: %s", kmzPath);
        return LZ_ERR_IO;
    }

    if (fseek(fp, 0, SEEK_END) != 0) {
        fclose(fp);
        return LZ_ERR_IO;
    }
    const long size = ftell(fp);
    if (size <= 0) {
        fclose(fp);
        return LZ_ERR_IO;
    }
    rewind(fp);

    /* `DjiWaypointV3_UploadKmzFile(const uint8_t *data, uint32_t dataLen)`
     * 的 dataLen 是 uint32。KMZ 只有几 KB，但别假设 —— 超了直接拒绝，
     * 截断上传只会得到一个含糊的失败。
     * 核实命令（豁免规则 2，签名必查）：
     *   grep -n -B8 "DjiWaypointV3_UploadKmzFile" psdk_lib/include/dji_waypoint_v3.h */
    if ((unsigned long)size > 0xFFFFFFFFUL) {
        fclose(fp);
        return LZ_ERR_RANGE;
    }

    uint8_t *buf = malloc((size_t)size);
    if (buf == NULL) {
        fclose(fp);
        return LZ_ERR_IO;
    }
    const size_t read = fread(buf, 1, (size_t)size, fp);
    fclose(fp);
    if (read != (size_t)size) {
        free(buf);
        USER_LOG_ERROR("KMZ 读取不完整：%zu / %ld 字节", read, size);
        return LZ_ERR_IO;
    }
    USER_LOG_INFO("KMZ 已读入内存：%ld 字节", size);

    /* ---- 第二段：交给飞机。从这里开始的失败都是 LZ_ERR_UPLOAD ---- */
    T_DjiReturnCode rc = DjiWaypointV3_UploadKmzFile(buf, (uint32_t)size);
    free(buf);

    /* 按仓库约定，`== DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS` 这类判断属
     * 豁免范围（见 .psdk-apiref/LEARNING-PROTOCOL.md §7），不必查原文 */
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("飞机拒收 KMZ rc=0x%08X（上传阶段失败）", (unsigned)rc);
        return LZ_ERR_UPLOAD;
    }
    USER_LOG_INFO("KMZ 上传完成（MD5 校验由 PSDK 在内部完成）");

    /* ---- 第三段：启动。失败是 LZ_ERR_START —— 与上面两段都不同 ---- */
    if (startImmediately) {
        rc = DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_START);
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            /* 这里是本项目最需要可观测性的一处：上传明明成功了，飞机却拒绝启动。
             *
             * ⚠️ 打印必须用 %llX 配 64 位 —— `T_DjiReturnCode` 是 uint64_t
             * （dji_typedef.h:80），PSDK 的返回码把「模块号」放在高 32 位
             * （DJI_ERROR_MODULE_INDEX_OFFSET = 32）。用 (unsigned) 截成 32 位
             * 就把模块号整个丢掉了，只剩一个光秃秃的 raw code，
             * 看起来像"小错误码"而实际含义完全不同。 */
            USER_LOG_ERROR("启动航点任务被拒 rc=0x%08llX（上传是成功的，问题在启动条件）",
                           (unsigned long long)rc);

            /* 先看能不能从 SDK 自己的日志流里捞到飞机给的真实原因
             * （如 error_code: 770 = 当前 RC 模式下无法启动）。捞不到不算错，
             * 那只是少一条信息 —— 下面还有状态摘要兜底。 */
            const char *real = LzSdkLogWatch_LastStartReject();
            if (real != NULL) {
                USER_LOG_ERROR("  ↑ 飞机给出的原因（摘自 SDK 日志）：%s", real);
            }
            const char *act = LzSdkLogWatch_LastActionError();
            if (act != NULL) {
                USER_LOG_ERROR("  ↑ SDK 把同一失败转成的返回值：%s", act);
            }

            LzBridge_LogStartPreconditions();
            return LZ_ERR_START;
        }
        USER_LOG_INFO("航点任务已启动");
    }

    return LZ_OK;
}

/**
 * @brief 停止正在执行的航点任务
 *
 * ⚠️ **失败必须让调用方看见。** 早先这里一行日志都没有，调用方又用
 * `(void)` 丢掉返回值 —— 于是"操作员拨 OFF 但飞机没停"这件事
 * 在日志和界面上**完全不可见**，程序照样报"绕飞结束"。
 *
 * 后果是物理的：飞机可能在杆旁边继续绕，而操作员以为已经停了。
 * 这与 START 失败那条是同一形状的问题（一个返回值承载多种失败、
 * 且没人看），只是这次没人看的是 STOP。
 *
 * 返回码说明：`DjiWaypointV3_Action(STOP)` 在任务没在跑时会返回
 * `CANNOT_STOP_WAYLINE_WHEN_WAYLINE_NOT_RUNNING`（raw 259）——
 * 那其实不是失败。但我们无从区分"没在跑"与"拒绝停止"，
 * 因为 `T_DjiReturnCode` 只给 `0x000000FF`（SYSTEM·UNKNOWN），
 * 真实原因只在 SDK 日志里（同 START 那条，见文件顶部的说明）。
 * 所以这里**如实上报失败**，把判断留给调用方。
 */
LzStatus LzBridge_StopMissionV3(void)
{
    const T_DjiReturnCode rc = DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_STOP);
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_INFO("已下发停止航点任务");
        return LZ_OK;
    }

    /* 用 %llX 配 64 位 —— 与 START 那条同样的理由：
     * T_DjiReturnCode 是 uint64_t，模块号在高 32 位，用 (unsigned) 会截掉。 */
    USER_LOG_ERROR("停止航点任务被拒 rc=0x%08llX", (unsigned long long)rc);

    /* 顺手把 SDK 日志里同一失败的那行捞出来 —— 它才是带原因的那条。
     * 抓不到不算错（那只是少一条信息），这里刻意不 fallback 造消息。 */
    const char *act = LzSdkLogWatch_LastActionError();
    if (act != NULL) {
        USER_LOG_ERROR("  ↑ SDK 把同一失败转成的返回值：%s", act);
    }
    return LZ_ERR_IO;
}

/* ===========================================================================
 * 飞机当前位置 —— 供"记录飞机位"按钮使用
 *
 * ⚠️ 数据来自订阅回调写进 s_fused 的缓存，**不是** getter。
 * `DjiFcSubscription_GetLatestValueOfTopic` 在本组合上必崩
 * （见本文件顶部那段说明与头文件里的警告）。
 *
 * ⚠️ 单位：`TOPIC_POSITION_FUSED` 的经纬度是 **rad**
 * （`dji_fc_subscription.h:1015-1016` 原文 `unit: rad`），
 * 这里换成度再交给调用方。换算只在这一处做。
 * =========================================================================== */

bool LzBridge_HasCurrentPosition(void)
{
    return s_gotFused;
}

LzStatus LzBridge_GetCurrentPosition(LzGeo *out)
{
    if (out == NULL) {
        return LZ_ERR_PARAM;
    }
    if (!s_gotFused) {
        /* 与"读失败"区分开：这里只是还没有数据到过。
         * 订阅成功但飞机没上报时就是这样 —— 属于**正常状态**。 */
        return LZ_ERR_NOT_READY;
    }

    LzGeo g = {
        .latitudeDeg = s_fused.latitude * 180.0 / M_PI,
        .longitudeDeg = s_fused.longitude * 180.0 / M_PI,
        .altitudeM = s_fused.altitude,
    };

    /* 零解在这里就拦掉，不留给调用方。判据走 `LzGeo_IsNullSolution`
     * （给邻域，不比 0）—— 实测无定位时给的是 `0.0000003, 0.0000004`，
     * 早先 `== 0.0` 的写法放行了它。详见 lz_types.h 里的说明。 */
    if (LzGeo_IsNullSolution(&g)) {
        USER_LOG_WARN("融合位置在零解邻域内（%.7f, %.7f）—— 当前没有定位",
                      g.latitudeDeg, g.longitudeDeg);
        return LZ_ERR_NO_TARGET;
    }
    if (!LzGeo_IsValid(&g)) {
        USER_LOG_WARN("融合位置非法（%.7f, %.7f）", g.latitudeDeg, g.longitudeDeg);
        return LZ_ERR_NO_TARGET;
    }

    *out = g;
    return LZ_OK;
}
