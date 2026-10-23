/**
 * @file wt_survey.c
 * @brief 激光采集会话状态机实现。
 */

#include "wt_survey.h"

#include "wt_aim.h"
#include "wt_camera.h"
#include "wt_laser.h"
#include "wt_vision.h"

#include <dji_logger.h>
#include <dji_platform.h>

#include <stdio.h>
#include <string.h>

static uint32_t WtSurvey_NowMs(void)
{
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    uint32_t ms = 0;

    if (osal != NULL && osal->GetTimeMs != NULL) {
        osal->GetTimeMs(&ms);
    }

    return ms;
}

WtSurveyConfig WtSurvey_DefaultConfig(void)
{
    WtSurveyConfig c;

    /*
     * 2.5° 容差：激光光斑在 100 m 外约有几十厘米张角，而叶尖是细长体 ——
     * 容差太小会反复重试却永远"不够准"，太大则可能打到叶片中段而不是尖端
     * （后者会被 wt_rotor_fit 的径向散布判据抓住，但那时已经白跑一趟）。
     */
    c.aimToleranceDeg = 2.5;

    /*
     * 云台停稳时间。下发后立刻读激光会读到转动过程中的方向，
     * 而返回值不会有任何"还在动"的标志 —— 拿到的是一个完全合法但错误的点。
     */
    c.settleMs = 1200;

    /*
     * 检测快照的最大年龄。用一帧 1 秒前的检测去下发云台角，等于按目标
     * 已经不在的位置去转 —— 风机在转或飞机在动时尤其危险。
     */
    c.maxSnapshotAgeMs = 800;

    c.maxAttemptsPerTarget = 12;
    c.gimbalMoveSeconds = 1.0;

    return c;
}

const char *WtSurvey_TargetName(int targetIndex)
{
    switch (targetIndex) {
    case WT_SURVEY_TARGET_HUB:
        return "轮毂中心";
    case WT_SURVEY_TARGET_TIP1:
        return "叶片 NO.1";
    case WT_SURVEY_TARGET_TIP2:
        return "叶片 NO.2";
    case WT_SURVEY_TARGET_TIP3:
        return "叶片 NO.3";
    default:
        return "(未知)";
    }
}

static void WtSurvey_Enter(WtSurvey *s, E_WtSurveyState state, const char *msg)
{
    s->state = state;
    s->stateEnteredMs = WtSurvey_NowMs();
    if (msg != NULL) {
        s->message = msg;
    }
}

bool WtSurvey_Begin(WtSurvey *s, const WtTurbineSpec *spec,
                    const WtSurveyConfig *cfg)
{
    WtValidateResult v;

    if (s == NULL || spec == NULL) {
        return false;
    }

    v = WtTurbine_Validate(spec);
    if (!v.ok) {
        USER_LOG_ERROR("采集：台账非法（%s），无法建立坐标系", v.message);
        return false;
    }

    memset(s, 0, sizeof(*s));
    s->spec = *spec;
    /*
     * 坐标系原点取塔基 —— 与 wt_plan.c / wt_telemetry_psdk.c 全部一致。
     * 激光给的是 WGS84，必须转到这个系里才能和规划链对上。
     */
    s->frame = WtLocalFrame_Init(&spec->base);
    s->cfg = (cfg != NULL) ? *cfg : WtSurvey_DefaultConfig();
    s->limits = WtRotorFit_DefaultLimits();
    s->targetIndex = WT_SURVEY_TARGET_HUB;
    s->attempts = 0;
    s->fitAccepted = false;

    WtSurvey_Enter(s, WT_SURVEY_AIMING, "开始采集：请把轮毂对进画面");
    USER_LOG_INFO("采集：会话开始，目标顺序 轮毂 → NO.1 → NO.2 → NO.3");

    return true;
}

void WtSurvey_Abort(WtSurvey *s)
{
    if (s == NULL) {
        return;
    }
    WtSurvey_Enter(s, WT_SURVEY_FAILED, "已中止");
}

bool WtSurvey_IsFinished(const WtSurvey *s)
{
    return s != NULL &&
           (s->state == WT_SURVEY_DONE || s->state == WT_SURVEY_FAILED);
}

bool WtSurvey_GetRotorFrame(const WtSurvey *s, WtRotorFrame *out)
{
    if (s == NULL || out == NULL || !s->fitAccepted) {
        return false;
    }
    *out = WtRotorFit_ToRotorFrame(&s->fit);

    return true;
}

/** 取当前目标在画面里的像素坐标 */
static bool WtSurvey_TargetPixel(const WtVisionSnapshot *snap, int targetIndex,
                                 double *outU, double *outV)
{
    if (targetIndex == WT_SURVEY_TARGET_HUB) {
        if (!snap->det.hubValid) {
            return false;
        }
        *outU = snap->det.hubU;
        *outV = snap->det.hubV;

        return true;
    }

    {
        /*
         * 叶尖下标 = 目标序号 − 1。这里依赖 wt_vision 已经按图像极角做过
         * 稳定排序 —— 否则"NO.1"每帧都可能指向不同的叶片。
         */
        const int tip = targetIndex - WT_SURVEY_TARGET_TIP1;

        if (tip < 0 || tip >= snap->det.tipCount || !snap->det.tipValid[tip]) {
            return false;
        }
        *outU = snap->det.tipU[tip];
        *outV = snap->det.tipV[tip];
    }

    return true;
}

static void WtSurvey_DoFit(WtSurvey *s)
{
    WtEnu tips[3];
    int i;
    const char *reason = NULL;

    for (i = 0; i < 3; i++) {
        tips[i] = s->pointEnu[WT_SURVEY_TARGET_TIP1 + i];
    }

    if (!WtRotorFit_FromPoints(&s->spec, s->pointEnu[WT_SURVEY_TARGET_HUB],
                               tips, 3, &s->fit)) {
        WtSurvey_Enter(s, WT_SURVEY_FAILED, s->fit.message);
        USER_LOG_ERROR("采集：拟合失败 —— %s", s->fit.message);
        return;
    }

    s->fitAccepted = WtRotorFit_CheckLimits(&s->fit, &s->limits, &reason);

    {
        char buf[512];

        WtRotorFit_Format(&s->fit, buf, sizeof(buf));
        USER_LOG_INFO("采集：拟合结果\n%s", buf);
    }

    if (!s->fitAccepted) {
        WtSurvey_Enter(s, WT_SURVEY_FAILED, reason);
        USER_LOG_ERROR("采集：拟合未通过采信阈值 —— %s", reason);
        return;
    }

    WtSurvey_Enter(s, WT_SURVEY_DONE, "采集完成");
    USER_LOG_INFO("采集：完成。实测停用角 %.1f°，叶片半径 %.1f m",
                  s->fit.parkPhaseDeg, s->fit.rotorRadiusM);
}

void WtSurvey_Tick(WtSurvey *s, const WtGimbalState *gimbal)
{
    const uint32_t now = WtSurvey_NowMs();

    if (s == NULL || WtSurvey_IsFinished(s) || s->state == WT_SURVEY_IDLE) {
        return;
    }

    switch (s->state) {
    case WT_SURVEY_AIMING: {
        WtVisionSnapshot snap;
        WtCameraGeometry geom;
        WtCameraPose pose;
        WtAimResult aim;
        double u = 0.0;
        double v = 0.0;

        if (gimbal == NULL || !gimbal->valid) {
            s->message = "等待云台姿态";
            return;
        }
        if (!WtVision_GetSnapshot(&snap)) {
            s->message = "等待首次识别结果";
            return;
        }
        if (snap.ageMs > s->cfg.maxSnapshotAgeMs) {
            s->message = "识别结果过旧，等待新帧";
            return;
        }
        if (!WtSurvey_TargetPixel(&snap, s->targetIndex, &u, &v)) {
            s->message = "当前目标未被识别到";
            return;
        }

        /*
         * FOV 必须由相机模型算出，不能写字面量。
         * DJI 规格页标的 84° 是**对角线** FOV，水平值是 71.59° ——
         * 把 84 当水平用会把视场算宽约 17%，于是每个像素对应的角度被高估，
         * 云台会转过头，而画面看起来"总也对不准"。
         */
        geom = WtCamera_ComputeGeometry(&WT_CAMERA_M4T_WIDE);

        memset(&pose, 0, sizeof(pose));
        pose.yawDeg = gimbal->yawDeg;
        pose.pitchDeg = gimbal->pitchDeg;
        pose.rollDeg = gimbal->rollDeg;

        if (!WtAim_FromPixel(&pose, u, v, snap.imageWidthPx, snap.imageHeightPx,
                             geom.fovHorizDeg, geom.fovVertDeg, &aim)) {
            s->message = "照准解算失败";
            return;
        }

        s->yawErrorDeg = aim.yawErrorDeg;
        s->pitchErrorDeg = aim.pitchErrorDeg;
        s->angularErrorDeg = aim.angularErrorDeg;

        if (WtAim_IsOnTarget(&aim, s->cfg.aimToleranceDeg)) {
            WtSurvey_Enter(s, WT_SURVEY_SETTLING, "已对准，等待云台停稳");
            return;
        }

        if (s->attempts >= s->cfg.maxAttemptsPerTarget) {
            WtSurvey_Enter(s, WT_SURVEY_FAILED, "对准重试超次数");
            USER_LOG_ERROR("采集：%s 对准失败，残余夹角 %.1f°（机头还需转 %.1f°）",
                           WtSurvey_TargetName(s->targetIndex),
                           aim.angularErrorDeg, aim.yawErrorDeg);
            return;
        }

        /*
         * 只下发俯仰。水平偏差交给操作员转机头 —— M4T 云台 Pan 不可控，
         * 这里若把 yaw 一并下发，不会报错也不会生效。
         */
        (void)WtLaser_SetGimbalPitchAbs(aim.targetPitchDeg, s->cfg.gimbalMoveSeconds);
        s->attempts++;

        USER_LOG_INFO("采集：对准 %s —— 云台俯仰 → %.1f°，机头还需转 %+.1f°（夹角 %.1f°）",
                      WtSurvey_TargetName(s->targetIndex), aim.targetPitchDeg,
                      aim.yawErrorDeg, aim.angularErrorDeg);

        WtSurvey_Enter(s, WT_SURVEY_SETTLING, "云台转动中");
        return;
    }

    case WT_SURVEY_SETTLING:
        if ((now - s->stateEnteredMs) < s->cfg.settleMs) {
            return;
        }
        WtSurvey_Enter(s, WT_SURVEY_LASING, "读取激光");
        return;

    case WT_SURVEY_LASING: {
        WtLaserSample sample;

        if (!WtLaser_Read(&sample) || !sample.valid) {
            s->attempts++;
            if (s->attempts >= s->cfg.maxAttemptsPerTarget) {
                WtSurvey_Enter(s, WT_SURVEY_FAILED, "激光反复无有效回波");
                USER_LOG_ERROR("采集：%s 激光失败（exception=%u）",
                               WtSurvey_TargetName(s->targetIndex),
                               (unsigned)sample.exception);
                return;
            }
            /* 回到对准：目标可能已经飘出容差 */
            WtSurvey_Enter(s, WT_SURVEY_AIMING, "激光无回波，重新对准");
            return;
        }

        s->pointGeo[s->targetIndex] = sample.point;
        /*
         * 转到塔基 ENU —— 与规划链同一个系。
         *
         * ⚠️ 激光 alt 的基准没有文档承诺（姊妹项目实测与气压高差约 38 m），
         * 所以这里转出的 u 分量可能整体偏移几十米。这不影响拟合：
         * wt_rotor_fit 只用点与点之间的相对量，整体平移被抵消。
         * 但**不要**拿它去和飞机高度相减。
         */
        s->pointEnu[s->targetIndex] = WtGeo_ToEnu(&s->frame, &sample.point);
        s->pointDistanceM[s->targetIndex] = sample.distanceM;
        s->pointValid[s->targetIndex] = true;

        USER_LOG_INFO("采集：%s 已记录 —— 距离 %.1f m，ENU (%.1f, %.1f, %.1f)",
                      WtSurvey_TargetName(s->targetIndex), sample.distanceM,
                      s->pointEnu[s->targetIndex].e,
                      s->pointEnu[s->targetIndex].n,
                      s->pointEnu[s->targetIndex].u);

        s->targetIndex++;
        s->attempts = 0;

        if (s->targetIndex >= WT_SURVEY_TARGET_COUNT) {
            WtSurvey_Enter(s, WT_SURVEY_FITTING, "四点采齐，开始拟合");
        } else {
            WtSurvey_Enter(s, WT_SURVEY_AIMING, "采下一个目标");
        }
        return;
    }

    case WT_SURVEY_FITTING:
        WtSurvey_DoFit(s);
        return;

    case WT_SURVEY_IDLE:
    case WT_SURVEY_DONE:
    case WT_SURVEY_FAILED:
    default:
        return;
    }
}

void WtSurvey_FormatProgress(const WtSurvey *s, char *buf, size_t bufLen)
{
    if (buf == NULL || bufLen == 0) {
        return;
    }
    if (s == NULL) {
        snprintf(buf, bufLen, "采集：(空)");
        return;
    }

    switch (s->state) {
    case WT_SURVEY_DONE:
        snprintf(buf, bufLen, "采集完成 停用角%.0f° 半径%.0fm 散布%.1f%%",
                 s->fit.parkPhaseDeg, s->fit.rotorRadiusM, s->fit.radiusSpreadPct);
        break;
    case WT_SURVEY_FAILED:
        snprintf(buf, bufLen, "采集失败 %s", s->message ? s->message : "");
        break;
    case WT_SURVEY_IDLE:
        snprintf(buf, bufLen, "采集未开始");
        break;
    default:
        /*
         * 机头偏差放在最前面 —— 这是操作员唯一需要动手的量，
         * 浮窗带宽有限（2 KB/s），把它排在后面容易被截断看不见。
         */
        snprintf(buf, bufLen, "机头%+.0f° %s(%d/4) 夹角%.1f°",
                 s->yawErrorDeg, WtSurvey_TargetName(s->targetIndex),
                 s->targetIndex + 1, s->angularErrorDeg);
        break;
    }
}
