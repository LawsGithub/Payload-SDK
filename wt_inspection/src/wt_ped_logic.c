/**
 * @file wt_ped_logic.c
 * @brief 高空行人瞄准打点决策层实现。纯算法，桌面可测。
 */

#include "wt_ped_logic.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * 判"在动"时两次采样的最小间隔。间隔太短时，读回噪声除以一个很小的 dt
 * 会被放大成很大的角速度 —— 静止的云台被误判成在动，永远等不到停稳。
 */
#define WT_PED_MOTION_MIN_DT_MS 50u

/* a 不早于 b（抗 uint32 回绕：OSAL 毫秒计时约 49.7 天溢出一次） */
static bool WtPed_AtOrAfter(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) >= 0;
}

WtPedConfig WtPedLogic_DefaultConfig(void)
{
    WtPedConfig c;

    memset(&c, 0, sizeof(c));

    /*
     * 锁定确认：停稳后连续 3 帧在容差内就打（推理 10 Hz，约 0.3 s）。
     *
     * 原先是「锁定满 3 s 再打」—— 3 s 来自用户最初的描述。2026-10-25 用户同意
     * 改成按证据确认：只要求打一次、拿到位置，固定等待不提供额外的证据。
     * 需要停顿时用配置 ped_dwell_ms 加回去（例如 3000），两个条件同时满足才打。
     */
    c.confirmFrames = 3;
    c.dwellMs = 0;

    /*
     * 激光没打成后的再确认时间。没打成多半是光轴擦着人过去了（无回波，或几次
     * 读数一会儿打到人一会儿打到地面），锁定本身已经确认过；但也不能立刻重打 ——
     * 5 Hz 限频下拿到的是同一个值。
     */
    c.retryDwellMs = 1000;

    /*
     * 新捕获要求的置信度。比推理阈值（0.25）高：捕获决定了接下来要打谁，
     * 误检的代价是对着一块地面打半天；而一旦跟上，偶尔一帧置信度掉下来
     * 不该就此丢掉目标。
     */
    c.acquireConf = 0.40;

    /*
     * 锁定容差 = 0.5 × 目标半尺寸：光轴落在人身中间那一半。
     * 下限 0.2°（广角下约 4 px）—— 再小就是在和检测框的逐帧抖动较劲；
     * 上限 1.0° —— 人离得很近、框很大时，也不该在离中心 1° 以外就开打。
     */
    c.lockFrac = 0.5;
    c.lockMinDeg = 0.2;
    c.lockMaxDeg = 1.0;
    /* 进锁定用 1×、出锁定用 1.6× —— 没有滞回的话，框在边界上抖一下就重新计时 */
    c.breakFactor = 1.6;

    /*
     * 关联门限 5°。60 m 外步行（1.4 m/s）约 1.3°/s，一次对准循环一两秒，
     * 走出去也就两三度；而两个人并排站（相距 2 m）在 60 m 外约 1.9° ——
     * 门限再大就容易换人，所以取"最近的那个"而不是"门限内第一个"。
     */
    c.gateDeg = 5.0;
    c.lostTimeoutMs = 2500;

    /*
     * 停稳：下发后至少 400 ms（云台从收到命令到开始转有延迟，这段时间读回
     * 是静止的，不能据此判"已停稳"），且读回静止 300 ms（覆盖取流延迟 ——
     * 停稳之后拍到的帧才算数）。1.5°/s 以上视为在动。
     */
    c.minSettleMs = 400;
    c.stableWindowMs = 300;
    c.motionDegPerS = 1.5;

    c.maxObsAgeMs = 800;

    /*
     * 12 次：静止目标两三次就锁上；人在走的话每次都要追，12 次没锁上
     * 说明它动得太快，继续追也打不准。
     */
    c.maxAimAttempts = 12;
    c.maxLaserAttempts = 3;

    /* 2 分钟：够操作员找人、转机头；又不至于"人走了"时永久挂住 */
    c.sessionTimeoutMs = 120000;

    return c;
}

WtPedConfig WtPedLogic_ConfigFrom(int dwellMs, double acquireConf)
{
    WtPedConfig c = WtPedLogic_DefaultConfig();

    /* 取值范围由配置解析把关（越界即拒绝启动），这里只防御负值；0 = 不额外停顿 */
    if (dwellMs >= 0) {
        c.dwellMs = (uint32_t)dwellMs;
    }
    if (acquireConf > 0.0 && acquireConf < 1.0) {
        c.acquireConf = acquireConf;
    }

    return c;
}

double WtPedLogic_AngleBetweenDeg(double az1Deg, double el1Deg, double az2Deg, double el2Deg)
{
    const WtEnu a = WtEnu_FromAzEl(az1Deg, el1Deg);
    const WtEnu b = WtEnu_FromAzEl(az2Deg, el2Deg);
    const double dot = Wt_Clamp(WtEnu_Dot(a, b), -1.0, 1.0);

    return Wt_RadToDeg(acos(dot));
}

double WtPedLogic_LockToleranceDeg(const WtPedConfig *cfg, double widthPx, double heightPx,
                                   int imageWidthPx, int imageHeightPx,
                                   double fovHorizDeg, double fovVertDeg)
{
    double halfW;
    double halfH;
    double tol;

    if (cfg == NULL) {
        return 0.0;
    }
    if (widthPx <= 0.0 || heightPx <= 0.0 || imageWidthPx <= 0 || imageHeightPx <= 0 ||
        fovHorizDeg <= 0.0 || fovVertDeg <= 0.0) {
        return cfg->lockMinDeg;
    }

    /* 小角度近似：目标框只有画面的几十分之一，误差可忽略 */
    halfW = 0.5 * widthPx * fovHorizDeg / (double)imageWidthPx;
    halfH = 0.5 * heightPx * fovVertDeg / (double)imageHeightPx;

    tol = cfg->lockFrac * ((halfW < halfH) ? halfW : halfH);

    return Wt_Clamp(tol, cfg->lockMinDeg, cfg->lockMaxDeg);
}

static void WtPed_Enter(WtPedLogic *s, E_WtPedState state, uint32_t nowMs, const char *msg)
{
    s->state = state;
    s->stateEnteredMs = nowMs;
    if (msg != NULL) {
        s->message = msg;
    }
}

void WtPedLogic_Begin(WtPedLogic *s, const WtPedConfig *cfg, uint32_t nowMs)
{
    if (s == NULL) {
        return;
    }

    memset(s, 0, sizeof(*s));
    s->cfg = (cfg != NULL) ? *cfg : WtPedLogic_DefaultConfig();
    s->beginMs = nowMs;
    /*
     * 开始时云台可能正被操作员拨动 —— 先要求一个完整的静止窗口，
     * 再采信画面。
     */
    s->lastMotionMs = nowMs;
    WtPed_Enter(s, WT_PED_SEARCHING, nowMs, "搜索行人：把目标放到画面中间附近");
}

bool WtPedLogic_IsFinished(const WtPedLogic *s)
{
    return s != NULL && (s->state == WT_PED_DONE || s->state == WT_PED_FAILED);
}

void WtPedLogic_Abort(WtPedLogic *s, const char *why)
{
    if (s == NULL || WtPedLogic_IsFinished(s)) {
        return;
    }
    s->state = WT_PED_FAILED;
    s->message = (why != NULL) ? why : "已中止";
}

void WtPedLogic_RecordPoint(WtPedLogic *s, uint32_t nowMs)
{
    if (s == NULL || WtPedLogic_IsFinished(s)) {
        return;
    }
    WtPed_Enter(s, WT_PED_DONE, nowMs, "打点完成");
}

void WtPedLogic_LaserFailed(WtPedLogic *s, uint32_t nowMs)
{
    if (s == NULL || WtPedLogic_IsFinished(s)) {
        return;
    }

    s->laserAttempts++;
    if (s->laserAttempts >= s->cfg.maxLaserAttempts) {
        WtPed_Enter(s, WT_PED_FAILED, nowMs,
                    "激光反复无有效回波（超出量程？目标太小？）");
        return;
    }

    /*
     * 回到锁定而不是对准：锁定已经确认过，只是这一下没打着。仍在锁定里就
     * 短暂再确认后重打；目标若已偏出，锁定那一支会把它送回对准。
     * 确认帧从 0 重新数：重打凭的是**之后**拍到的画面，不是打之前那几帧。
     */
    s->lockSinceMs = nowMs;
    s->dwellRequiredMs = s->cfg.retryDwellMs;
    s->confirmCount = 0;
    WtPed_Enter(s, WT_PED_LOCKED, nowMs, "激光没打成，稍后重打");
}

/** 由读回角速度判断云台是否停稳 */
static void WtPed_UpdateStability(WtPedLogic *s, const WtPedObs *obs)
{
    bool nowSettled;

    if (!obs->haveGimbal) {
        s->settled = false;
        s->havePrevGimbal = false;
        return;
    }

    if (!s->havePrevGimbal) {
        s->havePrevGimbal = true;
        s->prevYawDeg = obs->gimbal.yawDeg;
        s->prevPitchDeg = obs->gimbal.pitchDeg;
        s->prevGimbalMs = obs->nowMs;
    } else {
        const uint32_t dt = obs->nowMs - s->prevGimbalMs;

        if (dt >= WT_PED_MOTION_MIN_DT_MS) {
            const double ang = WtPedLogic_AngleBetweenDeg(s->prevYawDeg, s->prevPitchDeg,
                                                          obs->gimbal.yawDeg,
                                                          obs->gimbal.pitchDeg);

            if (ang * 1000.0 / (double)dt > s->cfg.motionDegPerS) {
                s->lastMotionMs = obs->nowMs;
            }
            s->prevYawDeg = obs->gimbal.yawDeg;
            s->prevPitchDeg = obs->gimbal.pitchDeg;
            s->prevGimbalMs = obs->nowMs;
        }
    }

    nowSettled = (!s->commanded || (obs->nowMs - s->lastCommandMs) >= s->cfg.minSettleMs) &&
                 (obs->nowMs - s->lastMotionMs) >= s->cfg.stableWindowMs;

    if (nowSettled && !s->settled) {
        s->settled = true;
        s->settledAtMs = obs->nowMs;
    } else if (!nowSettled) {
        /* 包括"停稳后又被拨动"—— 操作员动了云台轮，之前的判断都作废 */
        s->settled = false;
    }
}

/**
 * 处理一帧：已有目标则按绝对方向关联，没有则捕获离光轴最近的。
 *
 * 只在「已停稳、且这帧是停稳之后拍的」时调用（见头文件「三处不同」第 2 条）。
 *
 * @return 这一帧里是否找到了（或捕获了）目标
 */
static bool WtPed_ProcessFrame(WtPedLogic *s, const WtPedObs *obs)
{
    int n = obs->candidateCount;
    int best = -1;
    double bestScore = 1e18;
    WtAimResult bestAim;
    int i;

    memset(&bestAim, 0, sizeof(bestAim));

    if (n > WT_PED_MAX_CANDIDATES) {
        n = WT_PED_MAX_CANDIDATES;
    }

    for (i = 0; i < n; i++) {
        const WtPedCandidate *c = &obs->cand[i];
        WtAimResult aim;
        double score;

        if (!WtAim_FromPixel(&obs->gimbal, c->u, c->v, obs->imageWidthPx, obs->imageHeightPx,
                             obs->fovHorizDeg, obs->fovVertDeg, &aim)) {
            continue;
        }

        if (s->track.active) {
            /* 同一个人 = 世界里方向最接近上一次的那个（不是像素最近） */
            score = WtPedLogic_AngleBetweenDeg(s->track.azDeg, s->track.elDeg,
                                               aim.targetYawDeg, aim.targetPitchDeg);
            if (score > s->cfg.gateDeg) {
                continue;
            }
        } else {
            if (c->conf < s->cfg.acquireConf) {
                continue;
            }
            /* 捕获：离光轴最近的 —— 操作员把谁放在画面中间就打谁 */
            score = aim.angularErrorDeg;
        }

        if (score < bestScore) {
            bestScore = score;
            best = i;
            bestAim = aim;
        }
    }

    if (best < 0) {
        return false;   /* 本帧没看到它（或没有够格捕获的）；丢失由超时判 */
    }

    if (!s->track.active) {
        s->aimAttempts = 0;
    }

    s->track.active = true;
    s->track.azDeg = bestAim.targetYawDeg;
    s->track.elDeg = bestAim.targetPitchDeg;
    s->track.u = obs->cand[best].u;
    s->track.v = obs->cand[best].v;
    s->track.widthPx = obs->cand[best].widthPx;
    s->track.heightPx = obs->cand[best].heightPx;
    s->track.conf = obs->cand[best].conf;
    s->track.errDeg = bestAim.angularErrorDeg;
    s->track.yawErrDeg = bestAim.yawErrorDeg;
    s->track.pitchErrDeg = bestAim.pitchErrorDeg;
    s->track.lockTolDeg = WtPedLogic_LockToleranceDeg(&s->cfg, obs->cand[best].widthPx,
                                                      obs->cand[best].heightPx,
                                                      obs->imageWidthPx, obs->imageHeightPx,
                                                      obs->fovHorizDeg, obs->fovVertDeg);
    s->track.seenAtMs = obs->frameCapturedMs;

    return true;
}

/** 对准：停稳 → 有停稳后的新观测 → 已在容差内就锁定，否则可达性检查后下发 */
static WtPedDecision WtPed_StepAiming(WtPedLogic *s, const WtPedObs *obs, WtPedDecision d)
{
    WtAimResult aim;
    bool pitchOk;

    if (!obs->haveGimbal) {
        d.action = WT_PED_ACT_WAIT;
        d.message = "等待云台姿态";
        return d;
    }
    if (!s->settled) {
        d.action = WT_PED_ACT_WAIT;
        d.message = s->commanded ? "云台转动中，等待停稳" : "等待云台停稳";
        return d;
    }
    if (!WtPed_AtOrAfter(s->track.seenAtMs, s->settledAtMs)) {
        d.action = WT_PED_ACT_WAIT;
        d.message = "等待停稳后的新画面";
        return d;
    }
    if ((obs->nowMs - s->track.seenAtMs) > s->cfg.maxObsAgeMs) {
        d.action = WT_PED_ACT_WAIT;
        d.message = "识别结果过旧，等待新帧";
        return d;
    }

    if (s->track.errDeg <= s->track.lockTolDeg) {
        /*
         * 已在容差内：直接锁定计时，**不再下发云台** —— 多下发一次会让云台
         * 重新微动，"已对准"又变成"正在动"。
         */
        s->aimAttempts = 0;
        s->lockSinceMs = obs->nowMs;
        s->dwellRequiredMs = s->cfg.dwellMs;
        s->confirmCount = 1;    /* 触发锁定的这一帧就是第 1 帧 */
        WtPed_Enter(s, WT_PED_LOCKED, obs->nowMs, "已锁定，逐帧确认");
        d.state = s->state;
        d.action = WT_PED_ACT_WAIT;
        d.dwellRequiredMs = s->dwellRequiredMs;
        d.message = s->message;
        return d;
    }

    /*
     * 可达性先判、再下发：超出 pan 的 ±60° 时整条命令会被拒（连 pitch 都不转），
     * 现象是"云台不动"，和"对不准"长得一样。先判就能报成操作员能处置的
     * "请转机头"。
     */
    memset(&aim, 0, sizeof(aim));
    aim.ok = true;
    aim.targetYawDeg = s->track.azDeg;
    aim.targetPitchDeg = s->track.elDeg;
    aim.yawErrorDeg = s->track.yawErrDeg;
    aim.pitchErrorDeg = s->track.pitchErrDeg;
    aim.angularErrorDeg = s->track.errDeg;

    pitchOk = s->track.elDeg >= WT_GIMBAL_PITCH_MIN_DEG &&
              s->track.elDeg <= WT_GIMBAL_PITCH_MAX_DEG;

    if (obs->haveBodyYaw) {
        const char *why = NULL;

        if (!WtAim_IsReachable(&aim, obs->bodyYawDeg, &why)) {
            d.action = WT_PED_ACT_NEED_BODY_YAW;
            /* pan 超限时报"机头要转多少"；俯仰够不到时转机头没用，报 0 */
            d.yawErrorDeg = pitchOk ? Wt_AngleDiff(s->track.azDeg, obs->bodyYawDeg) : 0.0;
            d.message = why;
            return d;
        }
        d.gimbalYawDeg = s->track.azDeg;
    } else {
        if (!pitchOk) {
            d.action = WT_PED_ACT_NEED_BODY_YAW;
            d.yawErrorDeg = 0.0;
            d.message = "目标超出云台俯仰范围（-90°~+70°）";
            return d;
        }
        /*
         * 拿不到机头朝向就判不了 pan 可达性，只能"只调 pitch、yaw 保持当前值"。
         * **不能填 0**：绝对角模式下 0 是"转到正北"，够不到时整条命令被拒。
         */
        d.gimbalYawDeg = obs->gimbal.yawDeg;
    }

    if (s->aimAttempts >= s->cfg.maxAimAttempts) {
        WtPed_Enter(s, WT_PED_FAILED, obs->nowMs, "多次修正仍未锁定（目标在快速移动？）");
        d.state = s->state;
        d.action = WT_PED_ACT_FINISH;
        d.message = s->message;
        return d;
    }

    s->aimAttempts++;
    s->commanded = true;
    s->lastCommandMs = obs->nowMs;
    s->settled = false;     /* 这之后拍到的帧，要等重新停稳才采信 */

    d.gimbalPitchDeg = s->track.elDeg;
    d.action = WT_PED_ACT_ROTATE_GIMBAL;
    d.message = "转动云台对准目标";

    return d;
}

WtPedDecision WtPedLogic_Step(WtPedLogic *s, const WtPedObs *obs)
{
    WtPedDecision d;

    memset(&d, 0, sizeof(d));
    d.action = WT_PED_ACT_NONE;
    d.message = "";

    if (s == NULL || obs == NULL) {
        return d;
    }

    d.state = s->state;
    d.candidateCount = obs->haveFrame ? obs->candidateCount : 0;
    d.message = (s->message != NULL) ? s->message : "";

    if (WtPedLogic_IsFinished(s)) {
        d.action = WT_PED_ACT_FINISH;
        return d;
    }
    if (s->state == WT_PED_IDLE) {
        return d;
    }

    /* ---- 会话超时（读激光时不判：那一步本身就阻塞） ---- */
    if (s->state != WT_PED_LASING &&
        (obs->nowMs - s->beginMs) > s->cfg.sessionTimeoutMs) {
        WtPed_Enter(s, WT_PED_FAILED, obs->nowMs, "超时：会话时限内未完成打点");
        d.state = s->state;
        d.action = WT_PED_ACT_FINISH;
        d.message = s->message;
        return d;
    }

    if (s->state == WT_PED_LASING) {
        /*
         * 停在这里等接线层回调 RecordPoint / LaserFailed。再次 Step 仍要求读
         * 激光 —— 接线层漏了回调时，行为是"重读"而不是"静默卡死"。
         */
        d.action = WT_PED_ACT_READ_LASER;
        return d;
    }

    /* ---- 停稳判定 + 新帧（只采信停稳之后拍到的） ---- */
    WtPed_UpdateStability(s, obs);

    if (obs->haveFrame && (!s->haveLastFrame || obs->frameId != s->lastFrameId)) {
        s->haveLastFrame = true;
        s->lastFrameId = obs->frameId;

        if (s->settled && WtPed_AtOrAfter(obs->frameCapturedMs, s->settledAtMs) &&
            WtPed_ProcessFrame(s, obs) && s->state == WT_PED_LOCKED) {
            /*
             * 锁定中的每一张新画面都是一次确认：在容差内就累加，落到容差外
             * （还没到脱锁线）就从头数 —— 「连续」的意思。到脱锁线的由下面
             * LOCKED 分支送回对准。
             */
            if (s->track.errDeg <= s->track.lockTolDeg) {
                s->confirmCount++;
            } else {
                s->confirmCount = 0;
            }
        }
    }

    /*
     * ---- 丢失 ----
     * 从「最近一次看到」与「最近一次停稳」中较晚的那个算起：云台转动期间
     * 本来就看不到（不采信），那段时间不该算作丢失。
     */
    if (s->track.active && s->settled) {
        const uint32_t ref = WtPed_AtOrAfter(s->track.seenAtMs, s->settledAtMs)
                                 ? s->track.seenAtMs
                                 : s->settledAtMs;

        if ((obs->nowMs - ref) > s->cfg.lostTimeoutMs) {
            s->track.active = false;
            WtPed_Enter(s, WT_PED_SEARCHING, obs->nowMs, "目标丢失，重新搜索");
        }
    }

    if (s->state == WT_PED_SEARCHING && s->track.active) {
        WtPed_Enter(s, WT_PED_AIMING, obs->nowMs, "已捕获目标，开始对准");
    }

    d.state = s->state;
    if (s->track.active) {
        d.yawErrorDeg = s->track.yawErrDeg;
        d.pitchErrorDeg = s->track.pitchErrDeg;
        d.angularErrorDeg = s->track.errDeg;
        d.lockTolDeg = s->track.lockTolDeg;
    }

    switch (s->state) {
    case WT_PED_SEARCHING:
        d.action = WT_PED_ACT_WAIT;
        if (!obs->haveFrame) {
            d.message = "等待识别结果";
        } else if (!s->settled) {
            d.message = "等待云台停稳";
        } else if (obs->candidateCount <= 0) {
            d.message = "画面中没有行人";
        } else {
            d.message = "行人置信度不足，等待更清楚的画面";
        }
        return d;

    case WT_PED_AIMING:
        d.message = s->message;
        return WtPed_StepAiming(s, obs, d);

    case WT_PED_LOCKED: {
        uint32_t elapsed;

        if (!obs->haveGimbal || !s->settled) {
            WtPed_Enter(s, WT_PED_AIMING, obs->nowMs, "云台被扰动，重新对准");
            d.state = s->state;
            d.action = WT_PED_ACT_WAIT;
            d.message = s->message;
            return d;
        }
        if (s->track.errDeg > s->track.lockTolDeg * s->cfg.breakFactor) {
            WtPed_Enter(s, WT_PED_AIMING, obs->nowMs, "目标偏出锁定范围，重新对准");
            d.state = s->state;
            d.action = WT_PED_ACT_WAIT;
            d.message = s->message;
            return d;
        }

        elapsed = obs->nowMs - s->lockSinceMs;
        d.dwellElapsedMs = elapsed;
        d.dwellRequiredMs = s->dwellRequiredMs;
        d.confirmCount = s->confirmCount;
        d.confirmRequired = s->cfg.confirmFrames;

        if (s->confirmCount < s->cfg.confirmFrames) {
            d.action = WT_PED_ACT_WAIT;
            d.message = "锁定中，逐帧确认";
            return d;
        }
        if (elapsed < s->dwellRequiredMs) {
            d.action = WT_PED_ACT_WAIT;
            d.message = "锁定中";
            return d;
        }
        /* 打之前要有一帧足够新的观测确认它还在 —— 不能凭很久以前的印象开打 */
        if ((obs->nowMs - s->track.seenAtMs) > s->cfg.maxObsAgeMs) {
            d.action = WT_PED_ACT_WAIT;
            d.message = "等待新画面确认锁定";
            return d;
        }

        WtPed_Enter(s, WT_PED_LASING, obs->nowMs, "锁定已确认，读取激光");
        d.state = s->state;
        d.action = WT_PED_ACT_READ_LASER;
        d.message = s->message;
        return d;
    }

    default:
        d.action = WT_PED_ACT_NONE;
        return d;
    }
}

int WtPedLogic_PickLaserSample(const double *distM, int n, double *outSpreadM)
{
    int order[WT_PED_LASER_SAMPLES];
    double lo;
    double hi;
    double tol;
    int i;
    int j;

    if (outSpreadM != NULL) {
        *outSpreadM = 0.0;
    }
    if (distM == NULL || n < 2 || n > WT_PED_LASER_SAMPLES) {
        return -1;
    }

    for (i = 0; i < n; i++) {
        if (!isfinite(distM[i]) || distM[i] <= 0.0) {
            return -1;
        }
        order[i] = i;
    }
    /* 按距离从近到远排下标（n ≤ 3，插入排序足够） */
    for (i = 1; i < n; i++) {
        const int k = order[i];

        for (j = i - 1; j >= 0 && distM[order[j]] > distM[k]; j--) {
            order[j + 1] = order[j];
        }
        order[j + 1] = k;
    }

    lo = distM[order[0]];
    hi = distM[order[n - 1]];
    if (outSpreadM != NULL) {
        *outSpreadM = hi - lo;
    }
    tol = distM[order[(n - 1) / 2]] * WT_PED_LASER_SPREAD_FRAC;
    if (tol < WT_PED_LASER_SPREAD_ABS_M) {
        tol = WT_PED_LASER_SPREAD_ABS_M;
    }
    if (hi - lo > tol) {
        return -1;
    }

    /* 3 次取中间那次；2 次取 (n-1)/2 = 0，即较近的那次 */
    return order[(n - 1) / 2];
}
