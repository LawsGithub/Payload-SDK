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

/* 停止被拒的告警只报一次的闸。
 *
 * 为什么需要：`LzMission_Tick()` 由主循环每 100 ms 调一次，而拨 OFF 后
 * 开关会一直是 OFF —— 不加闸的话，STOP 每被拒一次就发一条浮窗，
 * 每秒 10 条。那既刷屏，也可能把 SDK 的浮窗带宽（2 KB/s）吃光，
 * 反而盖掉别的消息。
 *
 * 复位时机：只在"操作员重新拨 ON 又拨 OFF"后才复位。
 * 不复位的话，第二次失败会被当成重复而静默 ——
 * 而重复失败恰恰说明重试也没用，更需要报。 */
static bool s_stopRejectReported = false;

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

    /* 启动诊断**不能**在这里订阅 —— 见 LzMission_StartPostApp() 的说明。
     * 这里只做注册航点回调，那是确实要在 ApplicationStart 之前完成的。 */

    s_state = LZ_MISSION_STATE_IDLE;
    s_missionEnded = false;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode LzMission_StartPostApp(void)
{
    /* 订阅飞机状态话题，供"启动被拒"时排除法定位。
     *
     * ⚠️ 官方文档（.psdk-apiref/docs/cn/20.basic-function/50.fc-subscription.md）：
     *   "请勿在 main() 函数中调用本接口，请在用户线程中调用本接口，
     *    启动调度器后，该接口将正常运行。"
     * 实测 2026-09-20：在 ApplicationStart 之前调它 → 返回 SUCCESS，
     * 但随后读话题时 SIGSEGV。**返回成功不代表调用合法。**
     *
     * 失败不阻断启动 —— 诊断只服务于可观测性，不该拖累主流程。 */
    const LzStatus st = LzBridge_InitStartDiagnostics();
    if (st != LZ_OK) {
        USER_LOG_WARN("启动诊断订阅失败（%s）—— 不影响作业，仅日志信息会少", LzStatus_Str(st));
    }
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
        /* 上传失败与启动失败要分开说 —— 两者的排查方向完全不同。
         * 早期版本这里只有一句"上传航线失败：文件读写失败"，
         * 而实测（2026-09-20）真实死因是"上传成功、启动被拒"。 */
        if (st == LZ_ERR_START) {
            /* 浮窗带宽上限 2KB/s，且操作员在室外看不到 SDK 日志 ——
             * 把最关键的几项塞进一条短消息里。 */
            LzWidget_PostMessage("启动被拒：%s", LzBridge_StartDiagSummary());
        } else if (st == LZ_ERR_UPLOAD) {
            LzWidget_PostMessage("上传被拒：%s", LzStatus_Str(st));
        } else {
            LzWidget_PostMessage("航线文件处理失败：%s", LzStatus_Str(st));
        }
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
            s_stopRejectReported = false;   /* 新一轮作业，告警闸归零 */
        } else {
            /* 启动失败：清本地意图并发结束消息。
             * ⚠️ 注意这**不会**把 Pilot 上的开关拨回去（PSDK 没有那个接口，
             * 详见 lz_widget.h 的 LzWidget_ReportOrbitFinished 说明）——
             * 实际是"开关保持 ON 而浮窗说启动失败"，所以消息里带上
             * "请手动拨回"，不假装界面已经一致了。 */
            LzWidget_ReportOrbitFinished("启动失败");
        }
        break;
    }

    case LZ_MISSION_STATE_RUNNING: {
        /* 操作员中途拨 OFF → 立即停（见 lz_widget.h 的安全语义） */
        if (!LzWidget_IsOrbitRequested()) {
            /* ⚠️ 这里**不能**丢弃返回值。
             *
             * 早先写成 `(void)LzBridge_StopMissionV3();` 然后无条件置 IDLE、
             * 无条件报"绕飞结束：操作员停止" —— 于是 STOP 被拒时，
             * 飞机还在杆旁边绕，而日志与界面都说已经停了。
             * **静默的失败等于假装成功**，而在飞控语境里这直接关系到安全。
             *
             * 停止失败时**保持 RUNNING**：让状态与事实一致
             * （任务确实还在跑），操作员再拨一次 OFF 就能重试。
             * 若在这里置 IDLE，开关还在 ON 位而状态机认为空闲 ——
             * 那个组合会让下一次 Tick 把它当成"新的绕飞请求"重新上传启动。 */
            const LzStatus st = LzBridge_StopMissionV3();
            if (st != LZ_OK) {
                /* 只报一次：Tick 是 100 ms 一拍，而开关会一直保持 OFF，
                 * 不加闸就会每秒刷 10 条浮窗。 */
                if (!s_stopRejectReported) {
                    s_stopRejectReported = true;
                    /* 措辞要准确：此刻开关**已经在 OFF 位**，只拨 OFF 不会再
                     * 触发回调（值没变化）。要重试必须走 OFF→ON→OFF，
                     * 让控件值产生变化 —— 所以提示里要写清楚。 */
                    LzWidget_PostMessage("停止指令被拒（%s）—— 任务可能仍在执行。"
                                         "重试需把开关拨回 ON 再拨 OFF，"
                                         "或直接用遥控器接管",
                                         LzStatus_Str(st));
                }
                break;   /* 状态不动，留在 RUNNING */
            }
            s_stopRejectReported = false;
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
