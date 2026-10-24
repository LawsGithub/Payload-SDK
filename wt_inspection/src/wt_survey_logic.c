/**
 * @file wt_survey_logic.c
 * @brief 激光采集决策层实现。纯算法，桌面可测。
 */

#include "wt_survey_logic.h"

#include <stddef.h>
#include <string.h>

WtSurveyConfig WtSurveyLogic_DefaultConfig(void)
{
    WtSurveyConfig c;

    /*
     * 2.5°：激光光斑在百米外有几十厘米张角，而叶尖是细长体。
     * 再小会反复重试却永远"不够准"；再大则可能打到叶片中段而非尖端
     * （那会被 wt_rotor_fit 的径向散布判据抓住，但已经白跑一趟）。
     */
    c.aimToleranceDeg = 2.5;

    /*
     * 云台停稳时间。下发后立刻读激光会读到转动过程中的指向，
     * 而返回值里**没有任何"还在动"的标志** —— 拿到的是一个完全合法
     * 但错误的点。
     */
    c.settleMs = 1200;

    /*
     * 检测快照的最大年龄。拿一帧 1 秒前的检测去下发云台角，等于按目标
     * 已经不在的位置去转。
     */
    c.maxSnapshotAgeMs = 800;

    c.maxAttemptsPerTarget = 12;

    /*
     * 60 秒：够操作员把机头转过去，又不至于让"人走了"的情况永久挂住。
     */
    c.outOfRangeTimeoutMs = 60000;

    return c;
}

const char *WtSurveyLogic_TargetName(int targetIndex)
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

static void WtSurveyLogic_Enter(WtSurveyLogic *s, E_WtSurveyState state,
                                uint32_t nowMs, const char *msg)
{
    s->state = state;
    s->stateEnteredMs = nowMs;
    if (msg != NULL) {
        s->message = msg;
    }
}

void WtSurveyLogic_Begin(WtSurveyLogic *s, const WtSurveyConfig *cfg, uint32_t nowMs)
{
    if (s == NULL) {
        return;
    }

    memset(s, 0, sizeof(*s));
    s->cfg = (cfg != NULL) ? *cfg : WtSurveyLogic_DefaultConfig();
    s->targetIndex = WT_SURVEY_TARGET_HUB;
    s->attempts = 0;
    WtSurveyLogic_Enter(s, WT_SURVEY_AIMING, nowMs, "开始采集：请把轮毂对进画面");
}

bool WtSurveyLogic_IsFinished(const WtSurveyLogic *s)
{
    return s != NULL &&
           (s->state == WT_SURVEY_DONE || s->state == WT_SURVEY_FAILED);
}

void WtSurveyLogic_Abort(WtSurveyLogic *s, const char *why)
{
    if (s == NULL) {
        return;
    }
    s->state = WT_SURVEY_FAILED;
    s->message = (why != NULL) ? why : "已中止";
}

void WtSurveyLogic_RecordPoint(WtSurveyLogic *s, uint32_t nowMs)
{
    if (s == NULL || WtSurveyLogic_IsFinished(s)) {
        return;
    }

    s->targetIndex++;
    s->attempts = 0;

    if (s->targetIndex >= WT_SURVEY_TARGET_COUNT) {
        WtSurveyLogic_Enter(s, WT_SURVEY_FITTING, nowMs, "四点采齐，开始拟合");
    } else {
        WtSurveyLogic_Enter(s, WT_SURVEY_AIMING, nowMs, "采下一个目标");
    }
}

void WtSurveyLogic_LaserFailed(WtSurveyLogic *s, uint32_t nowMs)
{
    if (s == NULL || WtSurveyLogic_IsFinished(s)) {
        return;
    }

    s->attempts++;
    if (s->attempts >= s->cfg.maxAttemptsPerTarget) {
        WtSurveyLogic_Enter(s, WT_SURVEY_FAILED, nowMs, "激光反复无有效回波");
        return;
    }

    /*
     * 退回对准而不是原地重读：激光打空通常意味着指向已经偏了
     * （飞机在动、或风轮微转），原地重读大概率继续打空。
     */
    WtSurveyLogic_Enter(s, WT_SURVEY_AIMING, nowMs, "激光无回波，重新对准");
}

void WtSurveyLogic_FitDone(WtSurveyLogic *s, bool accepted, const char *reason)
{
    if (s == NULL) {
        return;
    }
    s->state = accepted ? WT_SURVEY_DONE : WT_SURVEY_FAILED;
    s->message = accepted ? "采集完成"
                          : ((reason != NULL) ? reason : "拟合未通过采信阈值");
}

WtSurveyDecision WtSurveyLogic_Step(WtSurveyLogic *s, const WtSurveyObs *obs)
{
    WtSurveyDecision d;

    memset(&d, 0, sizeof(d));
    d.action = WT_SURVEY_ACT_NONE;
    d.message = "";

    if (s == NULL || obs == NULL) {
        return d;
    }

    d.state = s->state;
    d.targetIndex = s->targetIndex;
    d.message = (s->message != NULL) ? s->message : "";

    switch (s->state) {
    case WT_SURVEY_AIMING: {
        WtAimResult aim;

        if (!obs->haveGimbal) {
            d.action = WT_SURVEY_ACT_WAIT;
            d.message = "等待云台姿态";
            return d;
        }
        if (!obs->haveDetection) {
            d.action = WT_SURVEY_ACT_WAIT;
            d.message = "等待首次识别结果";
            return d;
        }
        if (obs->detectionAgeMs > s->cfg.maxSnapshotAgeMs) {
            d.action = WT_SURVEY_ACT_WAIT;
            d.message = "识别结果过旧，等待新帧";
            return d;
        }
        if (!obs->targetVisible) {
            d.action = WT_SURVEY_ACT_WAIT;
            d.message = "当前目标未被识别到";
            return d;
        }

        if (!WtAim_FromPixel(&obs->gimbal, obs->targetU, obs->targetV,
                             obs->imageWidthPx, obs->imageHeightPx,
                             obs->fovHorizDeg, obs->fovVertDeg, &aim)) {
            d.action = WT_SURVEY_ACT_WAIT;
            d.message = "照准解算失败";
            return d;
        }

        d.yawErrorDeg = aim.yawErrorDeg;
        d.pitchErrorDeg = aim.pitchErrorDeg;
        d.angularErrorDeg = aim.angularErrorDeg;
        d.gimbalPitchDeg = aim.targetPitchDeg;

        /*
         * yaw 的目标值。
         *
         * 拿不到机头朝向时无法判 pan 可达性（限位是相对机头的），于是退化成
         * "只调 pitch" —— 而那必须表达成「yaw 保持当前值」，**不能填 0**：
         * 绝对角模式下 0 是"转到正北"，机头不朝北就够不到，整条命令会被拒，
         * 连 pitch 都不转。
         */
        d.gimbalYawDeg = obs->haveBodyYaw ? aim.targetYawDeg : obs->gimbal.yawDeg;

        /*
         * 可达性先判、再谈对准。
         *
         * 顺序不能反：若目标超出 pan 的 ±60°，下发会被整条拒掉
         * （0x600000004），而现象是"云台没动"，看起来和"对不准"一样。
         * 先判就能把它报成"请转机头"，这是操作员真正能处置的信息。
         */
        if (obs->haveBodyYaw) {
            const char *why = NULL;

            if (!WtAim_IsReachable(&aim, obs->bodyYawDeg, &why)) {
                if (s->outOfRangeSinceMs == 0) {
                    s->outOfRangeSinceMs = obs->nowMs;
                }
                if ((obs->nowMs - s->outOfRangeSinceMs) > s->cfg.outOfRangeTimeoutMs) {
                    WtSurveyLogic_Enter(s, WT_SURVEY_FAILED, obs->nowMs,
                                        "目标长时间超出云台可达范围");
                    d.state = s->state;
                    d.action = WT_SURVEY_ACT_FINISH;
                    d.message = s->message;

                    return d;
                }

                d.action = WT_SURVEY_ACT_NEED_BODY_YAW;
                d.message = why;
                /* 不消耗 attempts —— 这不是程序重试，是在等人转机头 */

                return d;
            }
            s->outOfRangeSinceMs = 0;
        }

        if (WtAim_IsOnTarget(&aim, s->cfg.aimToleranceDeg)) {
            /*
             * 已经对准：直接进停稳等待，**不再下发云台**。
             * 多下发一次会让云台重新微动，于是"刚对准"又变成"正在动"，
             * 永远等不到稳定。
             */
            WtSurveyLogic_Enter(s, WT_SURVEY_SETTLING, obs->nowMs,
                                "已对准，等待云台停稳");
            d.state = s->state;
            d.action = WT_SURVEY_ACT_WAIT;

            return d;
        }

        if (s->attempts >= s->cfg.maxAttemptsPerTarget) {
            WtSurveyLogic_Enter(s, WT_SURVEY_FAILED, obs->nowMs, "对准重试超次数");
            d.state = s->state;
            d.action = WT_SURVEY_ACT_FINISH;
            d.message = s->message;

            return d;
        }

        s->attempts++;
        WtSurveyLogic_Enter(s, WT_SURVEY_SETTLING, obs->nowMs, "云台转动中");
        d.state = s->state;
        d.action = WT_SURVEY_ACT_ROTATE_GIMBAL;

        return d;
    }

    case WT_SURVEY_SETTLING:
        /*
         * 用减法比较而不是 `now > entered + settle` —— 后者在 uint32
         * 回绕时会把"已经等够了"判成"还没到"，于是采集永久卡住。
         */
        if ((obs->nowMs - s->stateEnteredMs) < s->cfg.settleMs) {
            d.action = WT_SURVEY_ACT_WAIT;
            d.message = "等待云台停稳";

            return d;
        }
        WtSurveyLogic_Enter(s, WT_SURVEY_LASING, obs->nowMs, "读取激光");
        d.state = s->state;
        d.action = WT_SURVEY_ACT_READ_LASER;

        return d;

    case WT_SURVEY_LASING:
        /*
         * 停在这个状态等 app 层回调 RecordPoint 或 LaserFailed。
         * 再次 Step 时仍然要求读激光 —— 这样即使 app 层漏了回调，
         * 行为是"重试"而不是"静默卡死"。
         */
        d.action = WT_SURVEY_ACT_READ_LASER;

        return d;

    case WT_SURVEY_FITTING:
        d.action = WT_SURVEY_ACT_FIT;

        return d;

    case WT_SURVEY_DONE:
    case WT_SURVEY_FAILED:
        d.action = WT_SURVEY_ACT_FINISH;

        return d;

    case WT_SURVEY_IDLE:
    default:
        d.action = WT_SURVEY_ACT_NONE;

        return d;
    }
}
