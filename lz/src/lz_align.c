/**
 * @file lz_align.c
 * @brief 视觉照准的决策逻辑。设计依据（为什么抽出这一层、守哪三件事）
 *        见 `include/lz_align.h`。
 *
 * 本文件**不依赖 PSDK、不依赖图像、不依赖时间** —— 纯函数，桌面上可完整
 * 回归。凡是需要"等一会儿""读云台""开流"的东西都在 `app/lz_visual_align.c`。
 */

#include "lz_align.h"

#include <math.h>

#include "lz_plan.h"   /* LZ_GIMBAL_PITCH_MIN/MAX_DEG —— 唯一真值处 */

/* ------------------------------------------------------------------ */

LzAlignPolicy LzAlign_DefaultPolicy(void)
{
    LzAlignPolicy p;
    /* 死区 1.5°：12.5 m 外约 0.33 m —— 与"半个杆"同量级。
     * 单轮 30° 上限：检测读错一个数时不让云台猛甩。
     * 20 轮上限：实测单轮约 1.5 s，20 轮 ≈ 30 s —— 超过就说明
     * 目标在动或者检测不稳定，继续试下去只是耗电。 */
    p.deadzoneDeg = 1.5;
    p.maxStepDeg  = 30.0;
    p.maxRounds   = 20;
    return p;
}

/** 参数是否可用：`NaN` 的比较全为 false，所以这里显式用 `isfinite` 判 */
static bool policy_ok(const LzAlignPolicy *p)
{
    return isfinite(p->deadzoneDeg) && p->deadzoneDeg > 0.0 &&
           isfinite(p->maxStepDeg)  && p->maxStepDeg  > 0.0 &&
           p->maxRounds > 0;
}

LzAlignDecision LzAlign_DecideStep(double deltaDeg, double curPitchDeg,
                                   int round, bool awaitingConfirm,
                                   const LzAlignPolicy *policy)
{
    LzAlignDecision d;
    d.action = LZ_ALIGN_ACT_ROTATE;
    d.stepDeg = 0.0;
    d.targetDeg = curPitchDeg;

    const LzAlignPolicy def = LzAlign_DefaultPolicy();
    const LzAlignPolicy *p = (policy != NULL && policy_ok(policy)) ? policy : &def;

    /* ---- 1. 非法测量值：**必须最先拦** ----
     *
     * 为什么不能靠下面的阈值判据兜底：C 里 `NaN < x` 与 `NaN > x`
     * **都是 false**，所以
     *   `fabs(NaN) <= deadzone`  → false（不进死区，看起来"还需要转"）
     *   `target < MIN || target > MAX` → false（**不拒绝**）
     * 两处都放行，NaN 就被原样发给云台。**"检查没反对"不等于"检查通过"。**
     *
     * `round < 0` 同属"上游算错了"，一并归到这里 —— 让它走正常流程只会
     * 得到"轮数用尽"这种与病因不符的结论。 */
    if (!isfinite(deltaDeg) || !isfinite(curPitchDeg) || round < 0) {
        d.action = LZ_ALIGN_ACT_BAD_MEASURE;
        return d;
    }

    /* ---- 2. 死区：连续两轮才算收敛 ---- */
    if (fabs(deltaDeg) <= p->deadzoneDeg) {
        d.action = awaitingConfirm ? LZ_ALIGN_ACT_DONE : LZ_ALIGN_ACT_CONFIRM;
        return d;
    }

    /* ---- 3. 轮数用尽 ---- */
    if (round >= p->maxRounds) {
        d.action = LZ_ALIGN_ACT_ROUNDS_EXHAUSTED;
        return d;
    }

    /* ---- 4. 单轮限幅 ---- */
    double step = deltaDeg;
    if (step > p->maxStepDeg) {
        step = p->maxStepDeg;
    } else if (step < -p->maxStepDeg) {
        step = -p->maxStepDeg;
    }

    const double target = curPitchDeg + step;
    d.stepDeg = step;
    d.targetDeg = target;

    /* ---- 5. 包线：越界如实拒绝，**不钳位** ----
     * 钳位会把"物理上做不到"伪装成"做得到"（见头文件）。 */
    if (target < LZ_GIMBAL_PITCH_MIN_DEG || target > LZ_GIMBAL_PITCH_MAX_DEG) {
        d.action = LZ_ALIGN_ACT_OUT_OF_RANGE;
        return d;
    }

    d.action = LZ_ALIGN_ACT_ROTATE;
    return d;
}

/* ------------------------------------------------------------------ */

LzToggleAction LzAlign_DecideToggle(bool canStop, bool inGrace)
{
    /* 「刚按下、还没真正跑起来」的窗口里，重复按压**一律吞掉**。
     *
     * 现场（2026-09-28）实测的那条因果链：按下 → `do_init()` 阻塞约 1.5 秒
     * → 操作员以为没反应又按一次 → 这一拍被判成"再按一次 = 停止"
     * ⇒ **照准在启动后 100 ms 自杀**，浮窗两条相隔 100 ms，
     * 单缓冲让第一条被覆盖，操作员只看到"已停止"。
     *
     * `inGrace` 必须先判：`canStop` 在启动窗口里本来就是 false 之外，
     * 更关键的是**顺序**——两个都为真时若先判 `canStop`，
     * 那么"启动窗口内恰好已经能停"（真值组合）就会被判成停止，
     * 而那正是要拦的情形。 */
    if (inGrace) {
        return LZ_TOGGLE_IGNORED;
    }
    return canStop ? LZ_TOGGLE_STOP : LZ_TOGGLE_START;
}

LzConflict LzAlign_CheckConflict(bool orbitRunning, bool alignRunning)
{
    /* 两个都在跑时**绕飞优先** —— 它是安全关键的作业，让已经在跑的那个
     * 继续跑完，照准（可重来的动作）让路。这个顺序是定义了的，
     * 而不是碰巧由这里的 if 次序决定：`lz_test_align` 有一条用例
     * 专门断言"两个都为真时返回 ORBIT_ACTIVE"。 */
    if (orbitRunning) {
        return LZ_CONFLICT_ORBIT_ACTIVE;
    }
    if (alignRunning) {
        return LZ_CONFLICT_ALIGN_ACTIVE;
    }
    return LZ_CONFLICT_NONE;
}

const char *LzAlign_ConflictStr(LzConflict c)
{
    switch (c) {
    case LZ_CONFLICT_ORBIT_ACTIVE:
        return "绕飞正在执行 —— 请先拨回开关结束它，再按「识别目标」";
    case LZ_CONFLICT_ALIGN_ACTIVE:
        return "识别正在执行 —— 请等它结束（或再按一次「识别目标」停止）再拨开关";
    case LZ_CONFLICT_NONE:
    default:
        return NULL;
    }
}

/* ------------------------------------------------------------------ */

LzAlignRetryPolicy LzAlign_DefaultRetryPolicy(void)
{
    LzAlignRetryPolicy p;
    /* 3 秒（30 拍 × 100 ms）还没帧 ⇒ 取图那一路有问题，不是"再等等"。
     * 检测失败给 5 次：单帧糊了/被挡了是可能的，连着 5 次就不是偶然了。 */
    p.maxFrameWaits   = 30;
    p.maxDetectMisses = 5;
    return p;
}

void LzAlign_Retry_Reset(LzAlignRetry *r)
{
    if (r == NULL) {
        return;
    }
    r->frameWaits = 0;
    r->detectMisses = 0;
}

/** 上限判据：**非正值一律当作 0**（即"一次都不容忍"）。
 *
 * 为什么不把 0 当"无上限"：那会让一个写错的配置变成"永不放弃"，
 * 而"永不放弃"在飞行器上是危险方向。宁可早停。 */
static bool over(int count, int max)
{
    return count >= (max > 0 ? max : 0);
}

bool LzAlign_Retry_NoteNoFrame(LzAlignRetry *r, const LzAlignRetryPolicy *policy)
{
    if (r == NULL) {
        return true;   /* 没有状态就没有"再试一次"可言 */
    }
    const LzAlignRetryPolicy def = LzAlign_DefaultRetryPolicy();
    const LzAlignRetryPolicy *p = (policy != NULL) ? policy : &def;
    r->frameWaits++;
    return over(r->frameWaits, p->maxFrameWaits);
}

void LzAlign_Retry_NoteFrameOk(LzAlignRetry *r)
{
    if (r == NULL) {
        return;
    }
    /* ⚠️ **只清 frameWaits。** 这里曾经是 `s_waitTicks = 0` 一处清两个，
     * 后果见 `lz_align.h` 第 2 条：检测失败的上限永远到不了。 */
    r->frameWaits = 0;
}

bool LzAlign_Retry_NoteDetectMiss(LzAlignRetry *r, const LzAlignRetryPolicy *policy)
{
    if (r == NULL) {
        return true;
    }
    const LzAlignRetryPolicy def = LzAlign_DefaultRetryPolicy();
    const LzAlignRetryPolicy *p = (policy != NULL) ? policy : &def;
    r->detectMisses++;
    return over(r->detectMisses, p->maxDetectMisses);
}

void LzAlign_Retry_NoteDetectHit(LzAlignRetry *r)
{
    if (r == NULL) {
        return;
    }
    r->detectMisses = 0;
}
