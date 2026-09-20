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
        /* 机头朝向：绕飞时机头沿切线最自然，这里用"沿航线"。
         * 云台 yaw 是绝对方位角，与机头朝向无关，所以机头怎么转
         * 都不影响光轴指向 —— 这也是选 absYawModeRef=1 的好处。 */
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
 * =========================================================================== */

LzStatus LzBridge_UploadKmzV3(const char *kmzPath, bool startImmediately)
{
    if (kmzPath == NULL) {
        return LZ_ERR_PARAM;
    }

    FILE *fp = fopen(kmzPath, "rb");
    if (fp == NULL) {
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
        return LZ_ERR_IO;
    }

    T_DjiReturnCode rc = DjiWaypointV3_UploadKmzFile(buf, (uint32_t)size);
    free(buf);

    /* 按仓库约定，`== DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS` 这类判断属
     * 豁免范围（见 .psdk-apiref/LEARNING-PROTOCOL.md §7），不必查原文 */
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return LZ_ERR_IO;
    }

    if (startImmediately) {
        rc = DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_START);
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            return LZ_ERR_IO;
        }
    }

    return LZ_OK;
}

LzStatus LzBridge_StopMissionV3(void)
{
    const T_DjiReturnCode rc = DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_STOP);
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? LZ_OK : LZ_ERR_IO;
}
