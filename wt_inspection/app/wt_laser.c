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
         * **这里不碰云台模式。** 借与还由 BorrowFreeMode/ReturnFollowMode
         * 成对管理，作用域是一次采集会话 —— 因为 dji_app_ctl stop 直接杀
         * 进程，放在 DeInit 里归还等于永远不还（见头文件说明）。
         */
    }

    s_haveSample = false;
    s_lastReadMs = 0;

    return true;
}

void WtLaser_DeInit(void)
{
    if (s_gimbalInited) {
        /*
         * 兜底归还。正常路径上会话结束时已经还过了（ReturnFollowMode 可
         * 重复调用），这里只是防"会话异常退出"。
         *
         * ⚠️ 但**不能依赖这里** —— dji_app_ctl stop 直接杀进程，本函数
         * 根本不会被执行（2026-10-22 实测：收尾日志一行都没有）。
         */
        WtLaser_ReturnFollowMode();

        /*
         * 只复位 pitch —— 用 PITCH_AND_YAW 会连带复位 yaw，而此刻 yaw 已经
         * 交回 YAW_FOLLOW 跟随机头，再去复位它属于"还多了"。
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

/* 当前是否借着 FREE —— 用于让归还可重复调用而不刷日志 */
static bool s_freeModeBorrowed = false;

bool WtLaser_BorrowFreeMode(void)
{
    T_DjiReturnCode rc;

    if (!s_gimbalInited) {
        return false;
    }
    if (s_freeModeBorrowed) {
        return true;
    }

    rc = DjiGimbalManager_SetMode(WT_LASER_MOUNT, DJI_GIMBAL_MODE_FREE);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_WARN("激光：云台 SetMode(FREE) 失败 rc=0x%08X，"
                      "对准可能被机头运动带偏", (unsigned)rc);
        return false;
    }

    s_freeModeBorrowed = true;
    USER_LOG_INFO("激光：已借用云台 FREE 模式（会话结束即归还）");

    return true;
}

void WtLaser_ReturnFollowMode(void)
{
    T_DjiReturnCode rc;

    if (!s_gimbalInited || !s_freeModeBorrowed) {
        return;
    }

    rc = DjiGimbalManager_SetMode(WT_LASER_MOUNT, DJI_GIMBAL_MODE_YAW_FOLLOW);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        /*
         * 还不回去是**需要人工介入**的状态，不是普通告警：FREE 残留会让
         * 后续机头连续转动时 pan 顶限位、电机持续给力。所以报 ERROR 并
         * 明确告诉操作员怎么办。
         */
        USER_LOG_ERROR("激光：云台模式未能还成 YAW_FOLLOW rc=0x%08X —— "
                       "FREE 残留会让后续绕飞顶 pan 限位，请用遥控器拨一下"
                       "云台轮拿回控制权，或重启飞机", (unsigned)rc);
        return;
    }

    s_freeModeBorrowed = false;
    USER_LOG_INFO("激光：云台模式已还成 YAW_FOLLOW");
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

bool WtLaser_SetGimbalAbs(double pitchDeg, double yawDeg)
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
     * yaw 原样下发调用方给的值。**绝不能填 0** —— 那是"转到正北"，
     * 机头不朝北时够不到，整条命令会被拒（见头文件说明）。
     * 不想动 yaw 的调用方应该传入当前云台 yaw。
     */
    rot.yaw = (dji_f32_t)yawDeg;
    /*
     * time = 0：让云台自己决定速度。
     *
     * 不自己指定时长，是因为 Payload-SDK issue #563 实测
     * 「Rotate 的实际速度只有下发的 10–20%，差距非常大」—— 既然下发的
     * 速度本来就不准，指定时长只会给出一个假的"预计到位时间"，
     * 让调用方以为可以按它来等。到位与否只能靠**读回角度**判断。
     */
    rot.time = 0.0;

    rc = DjiGimbalManager_Rotate(WT_LASER_MOUNT, rot);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        /*
         * 分病因 —— 两者的处置完全不同，混在一起报会把排查引向错误方向：
         *   NON_CONTROL_AUTHORITY 是**权限**（有别的控制源在抢）
         *   YAW_REACH_*_LIMIT     是**参数**（yaw 超出相对机头 ±60°）
         */
        if (rc == DJI_ERROR_GIMBAL_MODULE_CODE_NON_CONTROL_AUTHORITY) {
            USER_LOG_ERROR("激光：云台控制权被占（勿同时在 Pilot 里操作云台）"
                           " rc=0x%08X", (unsigned)rc);
        } else {
            USER_LOG_ERROR("激光：云台 Rotate(pitch=%.1f, yaw=%.1f) 被拒 rc=0x%08X"
                           "（0x600000004 = yaw 超出相对机头 ±60°）",
                           pitchDeg, yawDeg, (unsigned)rc);
        }

        return false;
    }

    return true;
}
