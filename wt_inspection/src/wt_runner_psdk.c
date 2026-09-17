/**
 * @file wt_runner_psdk.c
 * @brief 运行器的执行部分：航线下发、任务监控、云台闭环
 *
 * 本文件只在妙算3 目标上编译。所有危险动作（解锁、起飞、返航）都集中在
 * 这里，且受三道闸门约束：航线已通过安全校验、操作员已确认、中止标志未置位。
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <dji_camera_manager.h>
#include <dji_fc_subscription.h>
#include <dji_flight_controller.h>
#include <dji_gimbal_manager.h>
#include <dji_logger.h>
#include <dji_typedef.h>
#include <dji_waypoint_v3.h>

#include "wt_runner.h"

#define WT_RUNNER_LOOP_PERIOD_MS 100   /*!< 主循环 10Hz */
#define WT_RUNNER_MISSION_TIMEOUT_S 1800 /*!< 单台风机任务超时 30 分钟 */
#define WT_RUNNER_KMZ_MAX_BYTES (4 * 1024 * 1024)

static volatile bool s_missionFinished = false;
static volatile bool s_missionBroken = false;

static T_DjiReturnCode WtRunnerPsdk_MissionStateCallback(T_DjiWaypointV3MissionState state)
{
    /*
     * 回调运行在 PSDK 的根线程上，只允许做最轻量的动作：置位标志。
     * 任何阻塞调用（日志落盘、网络）都会拖垮整个 PSDK 运行环境。
     */
    switch (state.state) {
    case DJI_WAYPOINT_V3_MISSION_STATE_IDLE:
    case DJI_WAYPOINT_V3_MISSION_STATE_RETURN_FIRSTPOINT:
        s_missionFinished = true;
        break;
    case DJI_WAYPOINT_V3_MISSION_STATE_BREAK:
        s_missionBroken = true;
        break;
    default:
        break;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode WtRunnerPsdk_ActionStateCallback(T_DjiWaypointV3ActionState state)
{
    (void)state;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/** 读取 KMZ 文件到内存 */
static bool WtRunnerPsdk_ReadFile(const char *path, uint8_t **outBuf, uint32_t *outLen)
{
    FILE *fp = fopen(path, "rb");
    long size;
    uint8_t *buf;

    if (fp == NULL) {
        USER_LOG_ERROR("无法打开航线文件 %s", path);
        return false;
    }

    fseek(fp, 0, SEEK_END);
    size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    if (size <= 0 || size > WT_RUNNER_KMZ_MAX_BYTES) {
        USER_LOG_ERROR("航线文件大小异常：%ld", size);
        fclose(fp);
        return false;
    }

    buf = malloc((size_t)size);
    if (buf == NULL) {
        fclose(fp);
        return false;
    }
    if (fread(buf, 1, (size_t)size, fp) != (size_t)size) {
        free(buf);
        fclose(fp);
        return false;
    }

    fclose(fp);
    *outBuf = buf;
    *outLen = (uint32_t)size;

    return true;
}

/**
 * @brief 云台闭环：把云台指向当前应在的位置
 *
 * 两个输入叠加：
 *   1) 航点自带的目标角（静态，来自航线文件）；
 *   2) 风轮相位外推带来的修正量（动态，叶尖追踪时才有意义）。
 *
 * 用绝对角模式而不是速度模式：绝对角指令是幂等的，即使丢了一帧或
 * 命令延迟，下一帧也会把云台拉回正确位置；速度模式则会累积误差。
 */
static void WtRunnerPsdk_TrackGimbal(WtRunner *runner, const WtPlanPoint *target)
{
    const WtAppConfig *cfg = &runner->config;
    WtRotorFrame predicted;
    T_DjiGimbalManagerRotation rot;
    double wantYaw;
    double wantPitch;
    double deadband = cfg->trackDeadbandDeg;

    if (cfg->gimbalTrackGain <= 0.0 || target == NULL) {
        return;
    }

    wantYaw = target->gimbalYawDeg;
    wantPitch = target->gimbalPitchDeg;

    /*
     * 叶尖追踪：用当前相位外推 0.3s 后风轮的姿态，再让云台指向那时
     * 叶尖所在的位置。外推时间取云台指令的典型执行延迟，使相机在快门
     * 开启的瞬间恰好对准叶尖 —— 对准"此刻"的叶尖是来不及的。
     */
    if (cfg->profile.rotorMayRotate && runner->telemetry.rotor.valid &&
        target->bladeIndex >= 0) {
        WtEnu tip;
        WtEnu look;

        predicted = WtTelemetry_PredictRotorFrame(&runner->telemetry, 0.3);
        tip = WtTurbine_BladeTip(&predicted, &runner->job.turbine->spec,
                                 target->bladeIndex);
        look = WtEnu_Sub(tip, runner->telemetry.aircraft.enu);

        wantYaw = WtEnu_Azimuth(look);
        wantPitch = WtEnu_Elevation(look);
    }

    /* 死区：小于阈值不动作，避免云台在稳态附近高频微调而引入抖动 */
    if (fabs(Wt_AngleDiff(wantYaw, runner->telemetry.gimbal.yawDeg)) < deadband &&
        fabs(wantPitch - runner->telemetry.gimbal.pitchDeg) < deadband) {
        return;
    }

    memset(&rot, 0, sizeof(rot));
    rot.rotationMode = DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE;
    rot.pitch = (dji_f32_t)wantPitch;
    rot.roll = 0.0f;
    rot.yaw = (dji_f32_t)wantYaw;
    rot.time = 0.2;

    DjiGimbalManager_Rotate(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, rot);
}

/** 触发一次单张拍照 */
static void WtRunnerPsdk_TriggerPhoto(void)
{
    DjiCameraManager_StartShootPhoto(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1,
                                     DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
}

/**
 * @brief 按航点序号推进相机动作
 *
 * 用「当前航点序号」与上一次处理的序号比较来触发一次性动作，而不是依赖
 * 定时器：航点到达事件本身就是最可靠的时序基准。
 */
static void WtRunnerPsdk_HandleWaypointActions(WtRunner *runner,
                                               const T_DjiWaypointV3MissionState *state,
                                               int *lastHandledIndex)
{
    const WtTurbineJob *job = &runner->job;
    int idx = (int)state->currentWaypointIndex;

    if (idx == *lastHandledIndex) {
        return;
    }
    if (idx < 0 || (size_t)idx >= job->mission.count) {
        *lastHandledIndex = idx;
        return;
    }

    {
        const WtPlanPoint *p = &job->mission.points[idx];

        WtRunnerPsdk_TrackGimbal(runner, p);

        if (WtPlanPoint_TakesPhoto(p)) {
            /*
             * 悬停稳定后再拍：运动模糊是叶片缺陷漏检的主要原因之一，
             * 而无人机在到点瞬间仍有残余速度。
             */
            usleep((useconds_t)(runner->config.profile.photoDwellSec * 1e6));
            WtRunnerPsdk_TriggerPhoto();
            USER_LOG_INFO("拍照 @ 航点 %d (%s, GSD %.2f mm/px)", idx, p->tag, p->gsdMmPerPx);
        }
    }

    *lastHandledIndex = idx;
}

WtRunResult WtRunner_ExecuteTurbine(WtRunner *runner)
{
    WtTurbineJob *job = &runner->job;
    T_DjiReturnCode rc;
    uint8_t *kmzBuf = NULL;
    uint32_t kmzLen = 0;
    int lastHandledIndex = -1;
    double elapsedS = 0.0;
    WtRunResult result = WT_RUN_OK;

    if (job->turbine == NULL) {
        return WT_RUN_ERR_PLAN;
    }
    if (!job->safety.ok) {
        USER_LOG_ERROR("拒绝执行：安全校验未通过");
        return WT_RUN_ERR_SAFETY;
    }

    if (!WtRunnerPsdk_ReadFile(job->kmzPath, &kmzBuf, &kmzLen)) {
        return WT_RUN_ERR_UPLOAD;
    }

    rc = DjiWaypointV3_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiWaypointV3_Init 失败: 0x%08X", rc);
        free(kmzBuf);
        return WT_RUN_ERR_UPLOAD;
    }

    /*
     * 回调必须在 UploadKmzFile 之前注册：任务可能在启动的瞬间就推送状态，
     * 注册晚了会丢掉最初几个状态，导致监控循环误判任务已结束。
     */
    DjiWaypointV3_RegMissionStateCallback(WtRunnerPsdk_MissionStateCallback);
    DjiWaypointV3_RegActionStateCallback(WtRunnerPsdk_ActionStateCallback);

    rc = DjiWaypointV3_UploadKmzFile(kmzBuf, kmzLen);
    free(kmzBuf);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("航线上传失败: 0x%08X", rc);
        DjiWaypointV3_DeInit();
        return WT_RUN_ERR_UPLOAD;
    }

    USER_LOG_INFO("航线已上传，启动任务：%s", job->turbine->name);

    s_missionFinished = false;
    s_missionBroken = false;

    if (DjiCameraManager_SetMode(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1,
                                 DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO) !=
        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_WARN("相机切到拍照模式失败，继续执行，航点动作可能无法触发");
    }
    DjiCameraManager_SetShootPhotoMode(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1,
                                       DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);

    rc = DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_START);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("任务启动失败: 0x%08X", rc);
        DjiWaypointV3_DeInit();
        return WT_RUN_ERR_EXECUTE;
    }

    /* 主监控循环 */
    while (!s_missionFinished && !s_missionBroken) {
        T_DjiWaypointV3MissionState state;

        usleep(WT_RUNNER_LOOP_PERIOD_MS * 1000);
        elapsedS += WT_RUNNER_LOOP_PERIOD_MS / 1000.0;

        if (runner->abortRequested) {
            USER_LOG_WARN("收到中止请求，暂停任务并返航");
            DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_PAUSE);
            DjiFlightController_StartGoHome();
            result = WT_RUN_ERR_ABORTED;
            break;
        }

        WtTelemetry_Poll(&runner->telemetry);

        /*
         * 位置不可信是危险信号：贴近叶片飞行完全依赖 RTK 精度，
         * 一旦退化为单点解，必须立即中止而不是"再飞几个航点看看"。
         */
        if (runner->telemetry.aircraft.validPosition &&
            !runner->telemetry.aircraft.rtkFixed &&
            runner->config.profile.minSafeDistM < 10.0) {
            USER_LOG_ERROR("RTK 失去固定解，贴近巡检不再安全，中止任务");
            DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_PAUSE);
            DjiFlightController_StartGoHome();
            result = WT_RUN_ERR_EXECUTE;
            break;
        }

        /*
         * 航点状态通过订阅话题获取，而不是从回调里读：回调只置位，
         * 真正的状态消费统一在主循环，避免跨线程共享可变状态。
         */
        memset(&state, 0, sizeof(state));
        {
            T_DjiFcSubscriptionFlightStatus flightStatus;

            if (DjiFcSubscription_GetLatestValueOfTopic(
                    DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                    (uint8_t *)&flightStatus, sizeof(flightStatus), NULL) ==
                    DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
                flightStatus == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_ON_GROUND &&
                elapsedS > 30.0) {
                USER_LOG_ERROR("飞机已落地但任务未正常结束，判定为异常");
                result = WT_RUN_ERR_EXECUTE;
                break;
            }
        }

        WtRunnerPsdk_HandleWaypointActions(runner, &state, &lastHandledIndex);

        if (elapsedS > WT_RUNNER_MISSION_TIMEOUT_S) {
            USER_LOG_ERROR("任务超时 %.0f s，中止", elapsedS);
            DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_STOP);
            result = WT_RUN_ERR_EXECUTE;
            break;
        }

        if (((int)elapsedS % 30) == 0) {
            WtTelemetry_LogSnapshot(&runner->telemetry);
        }
    }

    if (s_missionBroken && result == WT_RUN_OK) {
        USER_LOG_ERROR("任务被飞机中断（BREAK 状态）");
        result = WT_RUN_ERR_EXECUTE;
    }

    DjiWaypointV3_DeInit();
    USER_LOG_INFO("任务结束：%s，耗时 %.1f s", job->turbine->name, elapsedS);

    return result;
}

WtRunResult WtRunner_RunTurbine(WtRunner *runner, const char *turbineName,
                                double parkPhaseDeg)
{
    WtRunResult rc;

    runner->abortRequested = false;

    rc = WtRunner_PlanTurbine(runner, turbineName, parkPhaseDeg);
    if (rc != WT_RUN_OK) {
        WtRunner_WriteReport(runner); /* 失败也要留痕，便于事后复盘 */
        return rc;
    }

    rc = WtRunner_ExecuteTurbine(runner);
    WtRunner_WriteReport(runner);

    return rc;
}

void WtRunner_RequestAbort(WtRunner *runner)
{
    runner->abortRequested = true;
}