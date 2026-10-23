/**
 * @file wt_laser.c
 * @brief 激光测距与云台俯仰下发实现。
 */

#include "wt_laser.h"

#include <dji_camera_manager.h>
#include <dji_gimbal.h>          /* E_DjiGimbalResetMode 在这里，不在 dji_typedef.h */
#include <dji_gimbal_manager.h>
#include <dji_logger.h>
#include <dji_platform.h>

#include <math.h>
#include <string.h>

/*
 * 相机/云台挂载位置：M4T 的内置三摄挂在 PAYLOAD_PORT_NO1。
 * 与 wt_ai_stream_probe.c 里的 DJI_LIVEVIEW_CAMERA_POSITION_NO_1 是同一个值
 * （后者就 typedef 成前者），保持两边指向同一个物理相机。
 */
#define WT_LASER_MOUNT DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1

/* 头文件原文：Maximum data update frequency: 5Hz */
#define WT_LASER_MIN_INTERVAL_MS 250u

static bool s_cameraInited = false;
static bool s_gimbalInited = false;
static uint32_t s_lastReadMs = 0;
static WtLaserSample s_lastSample;
static bool s_haveSample = false;

static uint32_t WtLaser_NowMs(void)
{
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    uint32_t ms = 0;

    if (osal != NULL && osal->GetTimeMs != NULL) {
        osal->GetTimeMs(&ms);
    }

    return ms;
}

bool WtLaser_Init(void)
{
    T_DjiReturnCode rc;

    if (!s_cameraInited) {
        rc = DjiCameraManager_Init();
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("激光：DjiCameraManager_Init 失败 rc=0x%08X",
                           (unsigned)rc);
            return false;
        }
        s_cameraInited = true;
    }

    if (!s_gimbalInited) {
        rc = DjiGimbalManager_Init();
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("激光：DjiGimbalManager_Init 失败 rc=0x%08X",
                           (unsigned)rc);
            return false;
        }
        s_gimbalInited = true;

        /*
         * FREE 模式：云台姿态锁定在地面坐标系，忽略飞机自身的运动。
         * 对准一个固定目标（叶尖）必须用这个模式 —— YAW_FOLLOW 会让云台
         * 跟着机头转，刚对准的目标会被机头的微小偏转带走。
         *
         * ⚠️ 这是在改飞机上的全局状态，而 PSDK 没有 GetGimbalMode，
         * 无法读回原值、也就无法真正归还（见头文件的 @warning）。
         */
        rc = DjiGimbalManager_SetMode(WT_LASER_MOUNT, DJI_GIMBAL_MODE_FREE);
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_WARN("激光：云台 SetMode(FREE) 失败 rc=0x%08X，"
                          "对准可能被机头运动带偏", (unsigned)rc);
        } else {
            USER_LOG_INFO("激光：云台已设为 FREE 模式（注意：此为全局状态，"
                          "PSDK 无法读回原值，作业后不会自动还原）");
        }
    }

    s_haveSample = false;
    s_lastReadMs = 0;

    return true;
}

void WtLaser_DeInit(void)
{
    if (s_gimbalInited) {
        /*
         * 只复位 pitch —— 我们只动过这一个轴（M4T 的 yaw 本就不可控）。
         * 用 PITCH_AND_YAW 会连带复位一个我们没碰过的轴，属于"还多了"，
         * 同样是在改别人的状态。
         *
         * 这是能做到的最接近"归还"的动作：它不恢复之前的**模式**
         * （PSDK 没有接口可查原模式，见头文件 @warning）。
         */
        (void)DjiGimbalManager_Reset(WT_LASER_MOUNT, DJI_GIMBAL_RESET_MODE_PITCH_ONLY);
        /* 注意拼写：云台是 Deinit（小写 i），相机是 DeInit（大写 I）——
         * PSDK 自身命名不一致，这里按头文件原文写。 */
        (void)DjiGimbalManager_Deinit();
        s_gimbalInited = false;
    }

    if (s_cameraInited) {
        (void)DjiCameraManager_DeInit();
        s_cameraInited = false;
    }
}

bool WtLaser_Read(WtLaserSample *out)
{
    T_DjiCameraManagerLaserRangingInfo info;
    T_DjiReturnCode rc;
    uint32_t now;

    if (out == NULL) {
        return false;
    }
    if (!s_cameraInited) {
        memset(out, 0, sizeof(*out));
        return false;
    }

    now = WtLaser_NowMs();

    /*
     * 限频：不足 250 ms 就直接返回上一次的结果。
     *
     * 这里刻意返回"旧值"而不是失败 —— 调用方若在一个控制周期里多问几次，
     * 拿到一致的值比拿到一次成功一次失败更容易写对。但要清楚它**不是新采的**，
     * 所以"多次采样取中位数"这类做法必须自己控制间隔。
     */
    if (s_haveSample && (now - s_lastReadMs) < WT_LASER_MIN_INTERVAL_MS) {
        *out = s_lastSample;
        return s_lastSample.valid;
    }

    memset(&info, 0, sizeof(info));
    rc = DjiCameraManager_GetLaserRangingInfo(WT_LASER_MOUNT, &info);
    s_lastReadMs = now;

    memset(out, 0, sizeof(*out));

    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_WARN("激光：GetLaserRangingInfo 失败 rc=0x%08X", (unsigned)rc);
        s_lastSample = *out;
        s_haveSample = true;
        return false;
    }

    /*
     * 单位换算（头文件原文）：
     *   longitude / latitude : deg（直接可用）
     *   altitude             : 0.1 m
     *   distance             : 0.1 m
     *   screenX / screenY    : 0.1 %
     * 漏掉这些除法会得到 10 倍的距离与海拔，而数值看起来仍然"像个高度"。
     */
    out->point.lon = info.longitude;
    out->point.lat = info.latitude;
    out->point.alt = (double)info.altitude / 10.0;
    out->distanceM = (double)info.distance / 10.0;
    out->screenXPct = (double)info.screenX / 10.0;
    out->screenYPct = (double)info.screenY / 10.0;
    out->lidarEnabled = info.enable_lidar;
    out->exception = info.exception;

    /*
     * (0,0) 哨兵：与 wt_telemetry_psdk.c 里对 POSITION_FUSED 的处理同一个
     * 理由 —— 模块在线但没有有效解时会给出全零，而 (0,0) 是几内亚湾，
     * 当成真实坐标会让后续 ENU 转换得出天文数字。
     */
    if (info.exception != 0) {
        USER_LOG_WARN("激光：exception=%u（打空/超量程/被遮挡？）",
                      (unsigned)info.exception);
    } else if (out->distanceM <= 0.0) {
        USER_LOG_WARN("激光：距离为 %.1f m，视为无效", out->distanceM);
    } else if (fabs(out->point.lat) < 1e-9 && fabs(out->point.lon) < 1e-9) {
        USER_LOG_WARN("激光：经纬度为 (0,0)，视为无效解");
    } else {
        out->valid = true;
    }

    s_lastSample = *out;
    s_haveSample = true;

    return out->valid;
}

bool WtLaser_SetGimbalPitchAbs(double pitchDeg, double seconds)
{
    T_DjiGimbalManagerRotation rot;
    T_DjiReturnCode rc;

    if (!s_gimbalInited) {
        return false;
    }

    memset(&rot, 0, sizeof(rot));
    rot.rotationMode = DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE;
    rot.pitch = (dji_f32_t)pitchDeg;
    rot.roll = 0.0f;
    /*
     * yaw 置 0 并不是"转到正北"，而是 ABSOLUTE_ANGLE 下 M4T 会忽略它
     * （Pan 轴 Not controllable）。这里显式写 0 并加注释，是为了避免
     * 后来者"顺手把 yaw 填上目标方位"—— 那样不会报错、也不会生效，
     * 只会让人以为云台已经对准了水平方向。
     */
    rot.yaw = 0.0f;
    rot.time = seconds;

    rc = DjiGimbalManager_Rotate(WT_LASER_MOUNT, rot);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_WARN("激光：云台 Rotate(pitch=%.1f) 失败 rc=0x%08X",
                      pitchDeg, (unsigned)rc);
        return false;
    }

    return true;
}
