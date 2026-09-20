/**
 * @file lz_mission.c
 * @brief 绕飞作业的执行编排。
 *
 * ## 为什么要有状态机，而不是"回调里直接开飞"
 *
 * 上传 KMZ、启动航点、处理结束事件，这三件事发生在**不同线程**上：
 *   - 控件回调在 PSDK 工作线程
 *   - 航点状态回调在 PSDK 的工作线程
 *   - 主循环在 main 线程
 * 用一个显式状态机在单一位置（LzMission_Tick）串起来，
 * 比让三个线程各自改共享变量可靠得多。
 *
 * ## 与 lz_core 的分工
 *
 * lz_core 负责**算**（航线、KMZ），本模块负责**做**（上传、启动、收尾）。
 * 算的部分能在桌面上测，做的部分只能上机 —— 所以边界划得越干净越好。
 */

#include "lz_mission.h"

#include <dji_logger.h>
#include <dji_platform.h>
#include <dji_waypoint_v3.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "lz_bridge.h"
#include "lz_bridge_psdk.h"
#include "lz_geo.h"
#include "lz_plan.h"
#include "lz_pole_source.h"
#include "lz_widget.h"

typedef enum {
    LZ_MISSION_STATE_IDLE = 0,
    LZ_MISSION_STATE_RUNNING,
} LzMissionState;

static LzMissionState s_state = LZ_MISSION_STATE_IDLE;
static volatile bool s_missionEnded = false;
static char s_endReason[64];

/* ------------------------------------------------------------------ */
/* 航点状态回调（PSDK 工作线程）                                        */
/* ------------------------------------------------------------------ */

static T_DjiReturnCode LzMission_OnWaypointState(T_DjiWaypointV3MissionState state)
{
    /* 只做"记下事件"，动作交给主循环 —— 回调里不能做耗时操作 */
    switch (state.state) {
    case DJI_WAYPOINT_V3_MISSION_STATE_MISSION:
        /* 正在执行，更新进度消息 */
        LzWidget_PostMessage("绕飞中：航点 %u", (unsigned)state.currentWaypointIndex);
        break;

    case DJI_WAYPOINT_V3_MISSION_STATE_IDLE:
        /* 回到空闲 = 任务结束（正常跑完或被打断）。
         * 注意这个回调在 RUNNING 期间也可能收到 IDLE，所以用标志位
         * 而不是直接改状态 —— 由主循环统一裁决。 */
        if (s_state == LZ_MISSION_STATE_RUNNING) {
            s_missionEnded = true;
            snprintf(s_endReason, sizeof(s_endReason), "已结束（回到 IDLE）");
        }
        break;

    default:
        break;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* ------------------------------------------------------------------ */
/* 生命周期                                                            */
/* ------------------------------------------------------------------ */

T_DjiReturnCode LzMission_Init(void)
{
    const T_DjiReturnCode rc = DjiWaypointV3_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiWaypointV3_Init 失败 rc=0x%08X", (unsigned)rc);
        return rc;
    }

    const T_DjiReturnCode rc2 = DjiWaypointV3_RegMissionStateCallback(LzMission_OnWaypointState);
    if (rc2 != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("注册航点状态回调失败 rc=0x%08X", (unsigned)rc2);
        return rc2;
    }

    s_state = LZ_MISSION_STATE_IDLE;
    s_missionEnded = false;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode LzMission_DeInit(void)
{
    if (s_state == LZ_MISSION_STATE_RUNNING) {
        (void)LzBridge_StopMissionV3();
    }
    s_state = LZ_MISSION_STATE_IDLE;
    return DjiWaypointV3_DeInit();
}

/* ------------------------------------------------------------------ */
/* 一次完整的绕飞：规划 → 生成 → 上传 → 启动                            */
/* ------------------------------------------------------------------ */

/**
 * @brief 按控件上的设定生成航线并上传启动
 * @return 成功返回 true
 */
static bool lz_mission_start_orbit(void)
{
    /* 先取圆心 —— 这是整条链路上唯一的外部输入。取不到就不起飞。 */
    LzTarget pole;
    LzStatus st = LzPole_Acquire(&pole);
    if (st != LZ_OK) {
        LzWidget_PostMessage("取杆位失败（%s）：%s", LzPole_SourceName(), LzStatus_Str(st));
        return false;
    }

    LzOrbitProfile profile = {
        .radiusM = LzWidget_GetRadiusM(),
        .altitudeM = LzWidget_GetAltitudeM(),
        .speedMs = 3.0,
        .waypointCount = 8,
        .startBearingDeg = 0.0,
        .clockwise = true,
        .gimbalPitchDeg = -15.0,
    };

    const LzGeo takeoff = pole.geo;   /* 相对高度的参考点 */

    /* ---- 1. 算：纯算法，能在桌面上测 ---- */
    LzRoute route;
    LzRoute_Init(&route);
    st = LzPlan_BuildOrbit(&pole, &takeoff, &profile, &route);
    if (st != LZ_OK) {
        LzWidget_PostMessage("规划失败：%s", LzStatus_Str(st));
        LzRoute_Free(&route);
        return false;
    }

    st = LzPlan_Validate(&route, &profile);
    if (st != LZ_OK) {
        /* 安全校验不通过 —— 宁可不起飞，也不要带着已知问题起飞 */
        LzWidget_PostMessage("安全校验不通过：%s，已中止", LzStatus_Str(st));
        LzRoute_Free(&route);
        return false;
    }

    /* ---- 2. 生成 KMZ 到应用目录下的 data/ ---- */
    const char *kmzPath = "data/orbit.kmz";
    st = LzBridge_ExportKmz(&route, &pole, &profile, kmzPath);
    LzRoute_Free(&route);
    if (st != LZ_OK) {
        LzWidget_PostMessage("生成航线文件失败：%s", LzStatus_Str(st));
        return false;
    }

    /* ---- 3. 做：上传并启动（只能上机验证）---- */
    st = LzBridge_UploadKmzV3(kmzPath, true);
    if (st != LZ_OK) {
        LzWidget_PostMessage("上传航线失败：%s", LzStatus_Str(st));
        return false;
    }

    LzWidget_PostMessage("绕飞已启动（杆位取自%s）：半径 %.1f m，高度 %.1f m，%d 个航点",
                         LzPole_SourceName(),
                         profile.radiusM, profile.altitudeM, profile.waypointCount);
    return true;
}

/* ------------------------------------------------------------------ */
/* 状态机                                                              */
/* ------------------------------------------------------------------ */

void LzMission_Tick(void)
{
    switch (s_state) {
    case LZ_MISSION_STATE_IDLE: {
        if (!LzWidget_IsOrbitRequested()) {
            break;   /* 操作员没请求，什么都不做 */
        }
        if (lz_mission_start_orbit()) {
            s_state = LZ_MISSION_STATE_RUNNING;
            s_missionEnded = false;
        } else {
            /* 启动失败：把开关拨回去，让界面反映真实状态。
             * 不拨回的话操作员会以为飞机在飞。 */
            LzWidget_ReportOrbitFinished("启动失败");
        }
        break;
    }

    case LZ_MISSION_STATE_RUNNING: {
        /* 操作员中途拨 OFF → 立即停（见 lz_widget.h 的安全语义） */
        if (!LzWidget_IsOrbitRequested()) {
            (void)LzBridge_StopMissionV3();
            s_state = LZ_MISSION_STATE_IDLE;
            LzWidget_ReportOrbitFinished("操作员停止");
            break;
        }
        /* 飞机侧报告结束 */
        if (s_missionEnded) {
            s_missionEnded = false;
            s_state = LZ_MISSION_STATE_IDLE;
            LzWidget_ReportOrbitFinished(s_endReason);
        }
        break;
    }
    }
}

bool LzMission_IsRunning(void)
{
    return s_state == LZ_MISSION_STATE_RUNNING;
}
