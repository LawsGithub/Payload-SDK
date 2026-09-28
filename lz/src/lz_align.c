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

#include "lz_geo.h"    /* LzGeo_NormalizeDeg —— 偏航相对角折算 */
#include "lz_plan.h"   /* LZ_GIMBAL_*_DEG —— 唯一真值处 */

/* ------------------------------------------------------------------ */

LzAlignPolicy LzAlign_DefaultPolicy(void)
{
    LzAlignPolicy p;
    /* 死区 **3% 画面高度** —— 权威值是比例，不是角度。
     *
     * 为什么是 3%：1080 行上约 32 px。现场那面带星的红旗在画面里
     * 高 110 px（广角）~ 517 px（变焦 7×），3% 都远小于旗本身的高度，
     * 所以"画面中心落在旗面上"这个目标一定满足；同时又比检测本身的
     * 抖动（实测同一静止场景下杆列只动 0~2 px）大两个数量级，
     * 不会因为噪声永远收敛不了。
     *
     * `deadzoneDeg` 给一个**广角端的等价初值**，供"还不知道 VFOV 时"
     * 使用（如单元测试）。运行时由调用方按当前视场角重算，见
     * `LzAlign_DeadzoneDegFromFrac()`。 */
    p.deadzoneFrac = 0.03;
    p.deadzoneDeg  = 1.79;   /* = atan(0.03 · 2 · tan(55.09°/2))，广角端 */
    p.maxStepDeg  = 30.0;
    p.maxRounds   = 20;
    return p;
}

double LzAlign_DeadzoneDegFromFrac(double frac, double vfovDeg)
{
    /* NaN 的比较全为 false，所以这里显式判有限性 —— 与 DecideStep 里
     * 那道"NaN 会穿过阈值判据"的守卫同一个理由。 */
    if (!isfinite(frac) || !isfinite(vfovDeg)) {
        return NAN;
    }
    /* frac 取 (0, 0.5]：0.5 = 半个画面，超过它"死区"就大得没有意义了 */
    if (!(frac > 0.0) || frac > 0.5) {
        return NAN;
    }
    if (!(vfovDeg > 0.0) || vfovDeg >= 180.0) {
        return NAN;
    }

    const double halfRad = vfovDeg * 0.5 * (M_PI / 180.0);
    const double d = atan(frac * 2.0 * tan(halfRad)) * (180.0 / M_PI);
    return isfinite(d) ? d : NAN;
}

/** 参数是否可用：`NaN` 的比较全为 false，所以这里显式用 `isfinite` 判 */
static bool policy_ok(const LzAlignPolicy *p)
{
    /* `deadzoneFrac` 允许为 0（= 用 `deadzoneDeg`），但若给了就必须合法 ——
     * 一个非法的比例（负数/NaN）静默退回角度值会让"按画面比例"的意图落空，
     * 而那种落空在画面上看不出来（与 `policy_ok` 拦 NaN 是同一个理由）。 */
    if (!(isfinite(p->deadzoneFrac) && p->deadzoneFrac >= 0.0 && p->deadzoneFrac <= 0.5)) {
        return false;
    }
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

/* ------------------------------------------------------------------ */
/* 双轴版：横向（偏航）+ 纵向（俯仰）                                    */
/* ------------------------------------------------------------------ */

/** 单轴限幅 */
static double clamp_step(double v, double maxAbs)
{
    if (v > maxAbs)  { return maxAbs; }
    if (v < -maxAbs) { return -maxAbs; }
    return v;
}

LzAlignDecision LzAlign_DecideStepXY(double deltaPitchDeg, double deltaYawDeg,
                                     double curPitchDeg, double curYawDeg,
                                     double bodyYawDeg,
                                     int round, bool awaitingConfirm,
                                     const LzAlignPolicy *policy)
{
    LzAlignDecision d;
    d.action = LZ_ALIGN_ACT_ROTATE;
    d.stepDeg = 0.0;
    d.targetDeg = curPitchDeg;
    d.stepYawDeg = 0.0;
    d.targetYawDeg = curYawDeg;

    const LzAlignPolicy def = LzAlign_DefaultPolicy();
    const LzAlignPolicy *p = (policy != NULL && policy_ok(policy)) ? policy : &def;

    /* 本次是否做横向闭环。NaN 是一个**有意义的哨兵**（"这一轮不算横向"），
     * 不是"上游算错了" —— 所以它不能落进下面的 BAD_MEASURE。 */
    const bool wantYaw = isfinite(deltaYawDeg) && isfinite(curYawDeg) && isfinite(bodyYawDeg);

    /* ---- 1. 非法测量值：**必须最先拦**（同单轴版，理由见那里） ---- */
    if (!isfinite(deltaPitchDeg) || !isfinite(curPitchDeg) || round < 0) {
        d.action = LZ_ALIGN_ACT_BAD_MEASURE;
        return d;
    }
    /* ⚠️ 请求了横向却读不到云台 yaw ⇒ 数据不可信。**但 `NaN` 的 deltaYaw
     * 是"不做横向"，不是错** —— 两者必须分开，否则"退回单轴"这条正常路径
     * 会被误报成"测量数据不可信"（本项目在 read_pitch 的启动竞态上踩过
     * 同一个形状：**"还不知道"与"读到坏值"是两回事**）。 */
    if (!isnan(deltaYawDeg)) {
        if (!isfinite(curYawDeg)) {
            d.action = LZ_ALIGN_ACT_BAD_MEASURE;
            return d;
        }
        if (isfinite(deltaYawDeg) && !isfinite(bodyYawDeg)) {
            /* 请求了横向、但不知道机头朝向 ⇒ 可达性无从判定。
             * 报 OUT_OF_RANGE 而不是 BAD_MEASURE：数据是好的，
             * 缺的是"判可达性所需的那一项"。 */
            d.action = LZ_ALIGN_ACT_OUT_OF_RANGE;
            return d;
        }
    }

    /* ---- 2. 死区：**两个方向都**进死区才算收敛 ---- */
    const bool pitchIn = fabs(deltaPitchDeg) <= p->deadzoneDeg;
    const bool yawIn   = !wantYaw || (fabs(deltaYawDeg) <= p->deadzoneDeg);
    if (pitchIn && yawIn) {
        d.action = awaitingConfirm ? LZ_ALIGN_ACT_DONE : LZ_ALIGN_ACT_CONFIRM;
        return d;
    }

    /* ---- 3. 轮数用尽 ---- */
    if (round >= p->maxRounds) {
        d.action = LZ_ALIGN_ACT_ROUNDS_EXHAUSTED;
        return d;
    }

    /* ---- 4. 双轴限幅 ---- */
    double stepP = clamp_step(deltaPitchDeg, p->maxStepDeg);
    double stepY = wantYaw ? clamp_step(deltaYawDeg, p->maxStepDeg) : 0.0;

    const double targetP = curPitchDeg + stepP;
    const double targetY = curYawDeg + stepY;

    /* ⚠️ **先填进决策再判包线**，而不是只在成功路径上填。理由是两个轴
     * 的越界要能分辨是**哪一个**越了 —— 只报 `OUT_OF_RANGE` 而把
     * `target*Deg` 留成初值的话，日志与测试都看不出病因
     * （本项目在云台权限那处踩过：`NON_CONTROL_AUTHORITY` 与
     *  `YAW_REACH_POSITIVE_LIMIT` 病因完全不同，光看"失败"会混）。 */
    d.stepDeg = stepP;
    d.targetDeg = targetP;
    d.stepYawDeg = stepY;
    d.targetYawDeg = targetY;

    /* ---- 5. 包线：越界如实拒绝，**不钳位** ----
     *
     * 两个轴的判据**互相独立**，一个越界不该让另一个也失败：
     * 俯仰的可达性只取决于云台自身限位，与机头朝向无关。 */
    if (targetP < LZ_GIMBAL_PITCH_MIN_DEG || targetP > LZ_GIMBAL_PITCH_MAX_DEG) {
        d.action = LZ_ALIGN_ACT_OUT_OF_RANGE;
        return d;
    }
    if (wantYaw) {
        /* ⚠️ 比的是**相对机头**的角，不是绝对方位角 —— 见头文件。
         * 折到 [-180, 180] 再比，否则跨正北时会算出一个 350° 的巨大偏差。 */
        const double rel = LzGeo_NormalizeDeg(targetY - bodyYawDeg + 180.0) - 180.0;
        if (rel < LZ_GIMBAL_YAW_MIN_DEG || rel > LZ_GIMBAL_YAW_MAX_DEG) {
            d.action = LZ_ALIGN_ACT_OUT_OF_RANGE;
            return d;
        }
    }

    d.action = LZ_ALIGN_ACT_ROTATE;
    return d;
}
