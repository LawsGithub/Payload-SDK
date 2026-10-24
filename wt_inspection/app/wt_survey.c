/**
 * @file wt_survey.c
 * @brief 采集会话的接线层：采观测 → 调决策层 → 执行动作。
 *
 * 判据与状态转移都在 `wt_survey_logic`（wt_core），本文件只做副作用。
 */

#include "wt_survey.h"

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
    s->limits = WtRotorFit_DefaultLimits();

    WtSurveyLogic_Begin(&s->logic, cfg, WtSurvey_NowMs());

    /*
     * 借云台 FREE 模式 —— 作用域是**这次会话**，不是进程生命周期。
     * 失败不阻断：对准会被机头运动带偏，但仍可能对上（而且失败本身已告警）。
     */
    (void)WtLaser_BorrowFreeMode();

    USER_LOG_INFO("采集：会话开始，目标顺序 轮毂 → NO.1 → NO.2 → NO.3");

    return true;
}

void WtSurvey_Abort(WtSurvey *s)
{
    if (s == NULL) {
        return;
    }
    WtSurveyLogic_Abort(&s->logic, "已中止");
}

bool WtSurvey_IsFinished(const WtSurvey *s)
{
    return s != NULL && WtSurveyLogic_IsFinished(&s->logic);
}

bool WtSurvey_GetRotorFrame(const WtSurvey *s, WtRotorFrame *out)
{
    if (s == NULL || out == NULL || !s->fitAccepted) {
        return false;
    }
    *out = WtRotorFit_ToRotorFrame(&s->fit);

    return true;
}

/** 从检测快照里取出当前目标的像素 */
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
         * 叶尖下标 = 目标序号 − 1。依赖 wt_vision 已按图像极角稳定排序，
         * 否则"NO.1"每帧可能指向不同叶片。
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

/** 组装决策层需要的观测 */
static void WtSurvey_BuildObs(const WtSurvey *s, const WtTelemetry *tele,
                              WtSurveyObs *obs)
{
    WtVisionSnapshot snap;
    WtCameraGeometry geom;

    memset(obs, 0, sizeof(*obs));
    obs->nowMs = WtSurvey_NowMs();

    if (tele != NULL && tele->gimbal.valid) {
        obs->haveGimbal = true;
        obs->gimbal.yawDeg = tele->gimbal.yawDeg;
        obs->gimbal.pitchDeg = tele->gimbal.pitchDeg;
        obs->gimbal.rollDeg = tele->gimbal.rollDeg;
    }

    /*
     * 机头朝向：判 pan 可达性必须有它（限位是相对机头的 ±60°）。
     * 拿不到时决策层会退化成"只调 pitch、yaw 保持当前值"。
     */
    if (tele != NULL && tele->aircraft.validAttitude) {
        obs->haveBodyYaw = true;
        obs->bodyYawDeg = tele->aircraft.yawDeg;
    }

    /*
     * FOV 由相机模型算出，**不写字面量**。DJI 规格页标的 84° 是**对角线**
     * FOV，水平值是 71.59°。把 84 当水平用会把视场算宽约 17%，于是每像素
     * 对应的角度被高估、云台转过头，而画面看起来"总也对不准"。
     */
    geom = WtCamera_ComputeGeometry(&WT_CAMERA_M4T_WIDE);
    obs->fovHorizDeg = geom.fovHorizDeg;
    obs->fovVertDeg = geom.fovVertDeg;

    if (WtVision_GetSnapshot(&snap)) {
        obs->haveDetection = true;
        obs->detectionAgeMs = snap.ageMs;
        obs->imageWidthPx = snap.imageWidthPx;
        obs->imageHeightPx = snap.imageHeightPx;
        obs->targetVisible = WtSurvey_TargetPixel(&snap, s->logic.targetIndex,
                                                  &obs->targetU, &obs->targetV);
    }
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
        s->fitAccepted = false;
        WtSurveyLogic_FitDone(&s->logic, false, s->fit.message);
        USER_LOG_ERROR("采集：拟合失败 —— %s", s->fit.message);

        return;
    }

    s->fitAccepted = WtRotorFit_CheckLimits(&s->fit, &s->limits, &reason);

    {
        char buf[512];

        WtRotorFit_Format(&s->fit, buf, sizeof(buf));
        USER_LOG_INFO("采集：拟合结果\n%s", buf);
    }

    WtSurveyLogic_FitDone(&s->logic, s->fitAccepted, reason);

    if (s->fitAccepted) {
        USER_LOG_INFO("采集：完成。实测停用角 %.1f°，叶片半径 %.1f m",
                      s->fit.parkPhaseDeg, s->fit.rotorRadiusM);
    } else {
        USER_LOG_ERROR("采集：拟合未通过采信阈值 —— %s", reason);
    }
}

static void WtSurvey_DoLaser(WtSurvey *s, uint32_t nowMs)
{
    WtLaserSample sample;
    const int idx = s->logic.targetIndex;

    if (idx < 0 || idx >= WT_SURVEY_TARGET_COUNT) {
        return;
    }

    if (!WtLaser_Read(&sample) || !sample.valid) {
        USER_LOG_WARN("采集：%s 激光无有效回波（exception=%u）",
                      WtSurveyLogic_TargetName(idx), (unsigned)sample.exception);
        WtSurveyLogic_LaserFailed(&s->logic, nowMs);

        return;
    }

    s->pointGeo[idx] = sample.point;
    /*
     * 转到塔基 ENU —— 与规划链同一个系。
     *
     * ⚠️ 激光 alt 的基准没有文档承诺（姊妹项目实测与气压高差约 38 m），
     * 所以转出的 u 分量可能整体偏移几十米。这不影响拟合：wt_rotor_fit 只用
     * 点与点之间的相对量，整体平移被抵消。但**不要**拿它去和飞机高度相减。
     */
    s->pointEnu[idx] = WtGeo_ToEnu(&s->frame, &sample.point);
    s->pointDistanceM[idx] = sample.distanceM;
    s->pointValid[idx] = true;

    USER_LOG_INFO("采集：%s 已记录 —— 距离 %.1f m，ENU (%.1f, %.1f, %.1f)",
                  WtSurveyLogic_TargetName(idx), sample.distanceM,
                  s->pointEnu[idx].e, s->pointEnu[idx].n, s->pointEnu[idx].u);

    WtSurveyLogic_RecordPoint(&s->logic, nowMs);
}

void WtSurvey_Tick(WtSurvey *s, const WtTelemetry *tele)
{
    WtSurveyObs obs;
    WtSurveyDecision d;

    if (s == NULL || WtSurvey_IsFinished(s)) {
        return;
    }

    WtSurvey_BuildObs(s, tele, &obs);
    d = WtSurveyLogic_Step(&s->logic, &obs);

    s->yawErrorDeg = d.yawErrorDeg;
    s->pitchErrorDeg = d.pitchErrorDeg;
    s->angularErrorDeg = d.angularErrorDeg;

    switch (d.action) {
    case WT_SURVEY_ACT_ROTATE_GIMBAL:
        /*
         * pitch 与 yaw 一起下发。yaw 由决策层给出：可达时是目标方位角，
         * 拿不到机头朝向时是"当前云台 yaw"（绝不是 0，否则整条命令被拒）。
         */
        (void)WtLaser_SetGimbalAbs(d.gimbalPitchDeg, d.gimbalYawDeg);
        USER_LOG_INFO("采集：对准 %s —— 云台 pitch → %.1f°、yaw → %.1f°"
                      "（夹角 %.1f°）",
                      WtSurveyLogic_TargetName(d.targetIndex),
                      d.gimbalPitchDeg, d.gimbalYawDeg, d.angularErrorDeg);
        break;

    case WT_SURVEY_ACT_NEED_BODY_YAW:
        /*
         * 云台够不到，只能靠机头。这里只提示，**不操纵飞机** ——
         * 让采集模块去转机身需要飞控控制权，风险与收益不成比例。
         */
        USER_LOG_WARN("采集：%s 超出云台范围 —— 请把机头转 %+.0f°（%s）",
                      WtSurveyLogic_TargetName(d.targetIndex),
                      d.yawErrorDeg, d.message ? d.message : "");
        break;

    case WT_SURVEY_ACT_READ_LASER:
        WtSurvey_DoLaser(s, obs.nowMs);
        break;

    case WT_SURVEY_ACT_FIT:
        WtSurvey_DoFit(s);
        break;

    case WT_SURVEY_ACT_FINISH:
    case WT_SURVEY_ACT_WAIT:
    case WT_SURVEY_ACT_NONE:
    default:
        break;
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

    switch (s->logic.state) {
    case WT_SURVEY_DONE:
        snprintf(buf, bufLen, "采集完成 停用角%.0f° 半径%.0fm 散布%.1f%%",
                 s->fit.parkPhaseDeg, s->fit.rotorRadiusM, s->fit.radiusSpreadPct);
        break;
    case WT_SURVEY_FAILED:
        snprintf(buf, bufLen, "采集失败 %s",
                 s->logic.message ? s->logic.message : "");
        break;
    case WT_SURVEY_IDLE:
        snprintf(buf, bufLen, "采集未开始");
        break;
    default:
        /*
         * 机头偏差放最前面 —— 这是操作员唯一需要动手的量，浮窗带宽有限
         * （2 KB/s），排在后面容易被截断看不见。
         */
        snprintf(buf, bufLen, "机头%+.0f° %s(%d/4) 夹角%.1f°",
                 s->yawErrorDeg, WtSurveyLogic_TargetName(s->logic.targetIndex),
                 s->logic.targetIndex + 1, s->angularErrorDeg);
        break;
    }
}
