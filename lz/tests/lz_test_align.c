/**
 * @file lz_test_align.c
 * @brief 视觉照准**决策逻辑**的回归测试（零依赖，桌面可跑）。
 *
 * ## 本文件守什么，以及每条用例对应的真实缺陷
 *
 * | 用例组 | 守的东西 | 写错的表现 |
 * |---|---|---|
 * | A 非法测量值 | `NaN` 不能穿过包线检查 | NaN 被原样发给云台 |
 * | B 死区与收敛 | 必须连续两轮在死区里 | 某帧抖一下就算"对准了" |
 * | C 限幅与包线 | 单轮限幅、越界拒绝而非钳位 | 检测读错一个数 → 云台猛甩 |
 * | D 轮数上限 | 不无限试 | 目标在动时永远不结束 |
 * | E 互斥判定 | 两方向都拦、且绕飞优先 | 云台被两处同时抢 |
 * | F 重试计数 | 两个计数器彼此独立 | **照准永不结束**（见下） |
 *
 * ⚠️ **F 组是本次修的真实缺陷**：`do_grab()` 里一句 `s_waitTicks = 0`
 * 同时清掉了"检测失败"的计数，于是"失败 5 次就放弃"这条判据**永远到不了**
 * —— 每轮都是"先取到帧（清零）→ 再检测失败一次"。判据在、注释在、不生效，
 * 与 `LZ_CHECK_ANGLE_NEAR` 漏计 failures 是同一个形状。
 *
 * 反向验证（逐条回退，确认对应断言变红）见文件末尾注释。
 */

#include "lz_align.h"
#include "lz_plan.h"   /* 云台俯仰包线 —— 断言用的期望值从这里取，不另写一份 */
#include "lz_test.h"

#include <math.h>

/**
 * 构造一个**只给绝对角度**的策略。
 *
 * ⚠️ `deadzoneFrac = 0` 是必须的：缺省策略里 `deadzoneFrac = 0.03` 是
 * **权威值**（会按视场角折算覆盖 `deadzoneDeg`）。被测用例大多是
 * "给定角度下判据对不对"，必须显式声明"用角度、别折算"，
 * 否则用例测的是折算后的值而不是它写的那个 —— 而且看起来照样能过。
 */
static LzAlignPolicy pol(double dead, double step, int rounds)
{
    LzAlignPolicy p;
    p.deadzoneFrac = 0.0;
    p.deadzoneDeg = dead;
    p.maxStepDeg  = step;
    p.maxRounds   = rounds;
    return p;
}

int main(void)
{
    const LzAlignPolicy P = LzAlign_DefaultPolicy();

    /* ================= A. 非法测量值必须最先被拦 ================= */

    LZ_CASE("A1 缺省参数：死区的**权威值是画面比例**，角度只是广角端等价");
    {
        /* ⚠️ 这条断言在 2026-09-29 **方向被改过**：原先是
         * `deadzoneDeg == 1.5`（固定角度）。固定角度在变焦下会失真 ——
         * 7× 时 1.5° 等于画面高度的 17.6%（见 B4 组）。
         * 现在比例是权威值，角度由它折算而来。
         *
         * 断言值本身而不只是"大于 0"：这些数散落在两处（本层与 app 层）
         * 时会各自漂移。 */
        LZ_CHECK_NEAR(P.deadzoneFrac, 0.03, 1e-12);
        LZ_CHECK_NEAR(P.deadzoneDeg, 1.79, 0.01);   /* 广角 55.09° 下的等价角度 */
        LZ_CHECK_NEAR(P.maxStepDeg, 30.0, 1e-12);
        LZ_CHECK(P.maxRounds == 20);
    }

    LZ_CASE("A2 NaN 的 Δθ 必须判 BAD_MEASURE，不能落进『还需要转』");
    {
        /* 这是本组存在的理由。若没有最前面那道 isfinite 检查，
         * NaN 会一路走到 `fabs(NaN) <= 1.5`（false）→ 不进死区，
         * 再走到 `target < MIN || target > MAX`（两个都 false）
         * → **不拒绝** → 被下发。 */
        LZ_CHECK(LzAlign_DecideStep(NAN, -40.0, 0, false, &P).action
                 == LZ_ALIGN_ACT_BAD_MEASURE);
        LZ_CHECK(LzAlign_DecideStep(5.0, NAN, 0, false, &P).action
                 == LZ_ALIGN_ACT_BAD_MEASURE);
        LZ_CHECK(LzAlign_DecideStep(INFINITY, -40.0, 0, false, &P).action
                 == LZ_ALIGN_ACT_BAD_MEASURE);
        LZ_CHECK(LzAlign_DecideStep(-INFINITY, -40.0, 0, false, &P).action
                 == LZ_ALIGN_ACT_BAD_MEASURE);
        LZ_CHECK(LzAlign_DecideStep(5.0, -INFINITY, 0, false, &P).action
                 == LZ_ALIGN_ACT_BAD_MEASURE);
    }

    LZ_CASE("A2b 负轮数与 NaN 同属『上游算错了』，不混进正常的轮数逻辑");
    {
        /* 负轮数若走正常流程，它会先判"未用尽"（-1 < 20）然后去转云台 ——
         * 报出来的现象与病因完全对不上。 */
        LZ_CHECK(LzAlign_DecideStep(5.0, -40.0, -1, false, &P).action
                 == LZ_ALIGN_ACT_BAD_MEASURE);
    }

    LZ_CASE("A3 判定顺序：非法值优先于死区、优先于轮数用尽");
    {
        /* 顺序本身是规格。同时满足多个条件时，必须先报"数据不可信" ——
         * 现在恰好轮数用尽、值又合法，那报 ROUNDS_EXHAUSTED 才是对的；
         * 但只要值非法，无论轮数如何都该报 BAD_MEASURE。 */
        /* ⚠️ 这里要用**死区外**的 Δθ：死区检查在轮数检查之前（见 D2），
         * 拿 0.0 去测会得到 CONFIRM —— 那是对的，但测不出本组要测的东西。 */
        LZ_CHECK(LzAlign_DecideStep(5.0, -40.0, 999, false, &P).action
                 == LZ_ALIGN_ACT_ROUNDS_EXHAUSTED);
        LZ_CHECK(LzAlign_DecideStep(NAN, NAN, 999, false, &P).action
                 == LZ_ALIGN_ACT_BAD_MEASURE);
        LZ_CHECK(LzAlign_DecideStep(0.1, -40.0, 0, false, &P).action
                 == LZ_ALIGN_ACT_CONFIRM);
    }

    LZ_CASE("A4 非法参数退回缺省，而不是让非法参数把判据全废掉");
    {
        const LzAlignPolicy bad = pol(NAN, NAN, 0);
        const LzAlignDecision d = LzAlign_DecideStep(5.0, -40.0, 0, false, &bad);
        /* 缺省 maxStep 是 30 ⇒ 5° 应当原样通过（限幅后仍是 5） */
        LZ_CHECK(d.action == LZ_ALIGN_ACT_ROTATE);
        LZ_CHECK_NEAR(d.stepDeg, 5.0, 1e-12);

        const LzAlignDecision d2 = LzAlign_DecideStep(5.0, -40.0, 0, false, NULL);
        LZ_CHECK(d2.action == LZ_ALIGN_ACT_ROTATE);
        LZ_CHECK_NEAR(d2.targetDeg, -35.0, 1e-12);
    }

    /* ================= B. 死区与收敛 ================= */

    LZ_CASE("B1 死区边界：恰好等于死区算『进』，略大算『没进』");
    {
        /* 取"≤"而不是"<"是刻意的：死区是一个承诺（偏差在这个范围内就算
         * 对准了），边界值当然属于这个承诺。这条断言把边界钉住 ——
         * 改成 `<` 会在这里变红。 */
        /* 用"只给绝对角度"的策略（`deadzoneFrac = 0`），
         * 否则缺省的 0.03 会按视场角折算覆盖掉这里的 1.5 —— 见 `pol()`。 */
        const LzAlignPolicy ang = pol(1.5, 30.0, 20);
        LZ_CHECK(LzAlign_DecideStep(1.5, -40.0, 0, false, &ang).action
                 == LZ_ALIGN_ACT_CONFIRM);
        LZ_CHECK(LzAlign_DecideStep(-1.5, -40.0, 0, false, &ang).action
                 == LZ_ALIGN_ACT_CONFIRM);
        LZ_CHECK(LzAlign_DecideStep(1.5000001, -40.0, 0, false, &ang).action
                 == LZ_ALIGN_ACT_ROTATE);
    }

    LZ_CASE("B2 第一次进死区只发 CONFIRM，第二次才 DONE");
    {
        /* 只判一轮的话，某一帧检测抖一下就宣布"对准了"，而云台还没停稳。
         * 两拍的语义不同：CONFIRM 不转云台，只再取一帧。 */
        const LzAlignDecision first = LzAlign_DecideStep(0.4, -40.0, 3, false, &P);
        LZ_CHECK(first.action == LZ_ALIGN_ACT_CONFIRM);

        const LzAlignDecision second = LzAlign_DecideStep(0.4, -40.0, 3, true, &P);
        LZ_CHECK(second.action == LZ_ALIGN_ACT_DONE);
    }

    LZ_CASE("B3 复核那一帧若又跑出死区，回到正常迭代（不是直接 DONE）");
    {
        const LzAlignDecision d = LzAlign_DecideStep(8.0, -40.0, 3, true, &P);
        LZ_CHECK(d.action == LZ_ALIGN_ACT_ROTATE);
        /* `awaitingConfirm` 为真时要能把"复核失败"如实反映成一次正常转向，
         * 而不是仍然报 CONFIRM（那会陷在复核里出不来）。 */
        LZ_CHECK(d.action != LZ_ALIGN_ACT_CONFIRM);
        LZ_CHECK(d.action != LZ_ALIGN_ACT_DONE);
    }

    /* ================= C. 限幅与包线 ================= */

    LZ_CASE("C1 单轮限幅：超过 30° 的 Δθ 只走 30°");
    {
        const LzAlignDecision big = LzAlign_DecideStep(85.0, -40.0, 0, false, &P);
        LZ_CHECK(big.action == LZ_ALIGN_ACT_ROTATE);
        LZ_CHECK_NEAR(big.stepDeg, 30.0, 1e-12);
        LZ_CHECK_NEAR(big.targetDeg, -10.0, 1e-12);

        const LzAlignDecision neg = LzAlign_DecideStep(-85.0, 0.0, 0, false, &P);
        LZ_CHECK_NEAR(neg.stepDeg, -30.0, 1e-12);
        LZ_CHECK_NEAR(neg.targetDeg, -30.0, 1e-12);

        /* 正好等于限幅值时不该被改（边界用 > 而不是 >=） */
        const LzAlignDecision eq = LzAlign_DecideStep(30.0, -40.0, 0, false, &P);
        LZ_CHECK_NEAR(eq.stepDeg, 30.0, 1e-12);
    }

    LZ_CASE("C2 包线：目标角越界要如实拒绝，不能钳位");
    {
        /* 期望值取自 lz_plan.h 的**唯一定义处**，这里不另写一份数字 ——
         * 包线改了，本用例跟着走；写死 -90/70 的话两处会打架。 */
        const double lo = LZ_GIMBAL_PITCH_MIN_DEG;
        const double hi = LZ_GIMBAL_PITCH_MAX_DEG;

        /* 从 lo+5 再往下压 30° ⇒ 越过下限 */
        const LzAlignDecision low = LzAlign_DecideStep(-30.0, lo + 5.0, 0, false, &P);
        LZ_CHECK(low.action == LZ_ALIGN_ACT_OUT_OF_RANGE);
        LZ_CHECK(low.targetDeg < lo);

        /* 从 hi-5 再往上仰 30° ⇒ 越过上限 */
        const LzAlignDecision high = LzAlign_DecideStep(30.0, hi - 5.0, 0, false, &P);
        LZ_CHECK(high.action == LZ_ALIGN_ACT_OUT_OF_RANGE);
        LZ_CHECK(high.targetDeg > hi);

        /* 恰好落在边界上必须**放行** —— 钳位版与拒绝版在这里都通过，
         * 所以真正区分两者的是上面两条（拒绝版报 OUT_OF_RANGE，
         * 钳位版会报 ROTATE 且 targetDeg 被改成 ±边界）。 */
        const LzAlignDecision onHi = LzAlign_DecideStep(5.0, hi - 5.0, 0, false, &P);
        LZ_CHECK(onHi.action == LZ_ALIGN_ACT_ROTATE);
        LZ_CHECK_NEAR(onHi.targetDeg, hi, 1e-12);

        const LzAlignDecision onLo = LzAlign_DecideStep(-5.0, lo + 5.0, 0, false, &P);
        LZ_CHECK(onLo.action == LZ_ALIGN_ACT_ROTATE);
        LZ_CHECK_NEAR(onLo.targetDeg, lo, 1e-12);
    }

    LZ_CASE("C3 越界判定用的是**目标角**，不是当前角或步进量");
    {
        /* 当前角已经在包线外（理论上不该发生，但上游读数可能越界）时，
         * 只要这一步能把角带回范围内，就该允许 —— 否则会陷在越界状态
         * 里永远出不来。 */
        const double hi = LZ_GIMBAL_PITCH_MAX_DEG;
        const LzAlignDecision back = LzAlign_DecideStep(-30.0, hi + 10.0, 0, false, &P);
        LZ_CHECK(back.action == LZ_ALIGN_ACT_ROTATE);
        LZ_CHECK(back.targetDeg <= hi);
    }

    /* ================= D. 轮数上限 ================= */

    LZ_CASE("D1 轮数用尽要判失败，不能继续转");
    {
        LZ_CHECK(LzAlign_DecideStep(20.0, -40.0, 19, false, &P).action
                 == LZ_ALIGN_ACT_ROTATE);
        LZ_CHECK(LzAlign_DecideStep(20.0, -40.0, 20, false, &P).action
                 == LZ_ALIGN_ACT_ROUNDS_EXHAUSTED);
        LZ_CHECK(LzAlign_DecideStep(20.0, -40.0, 999, false, &P).action
                 == LZ_ALIGN_ACT_ROUNDS_EXHAUSTED);
    }

    LZ_CASE("D2 轮数用尽**不覆盖**『已对准』—— 收敛优先");
    {
        /* 顺序反了会得到很坏的行为：已经对准了，却因为"轮数用尽"报失败。
         * 而这条恰好会在现场表现为"明明屏幕上已经居中，程序却说失败"。 */
        LZ_CHECK(LzAlign_DecideStep(0.2, -40.0, 999, true, &P).action
                 == LZ_ALIGN_ACT_DONE);
        LZ_CHECK(LzAlign_DecideStep(0.2, -40.0, 999, false, &P).action
                 == LZ_ALIGN_ACT_CONFIRM);
    }

    /* ========== E0. 按钮按压：「开始 / 停止 / 吞掉」三选一 ========== */

    LZ_CASE("E0a 空闲时按下 = 开始；正在跑时按下 = 停止");
    {
        LZ_CHECK(LzAlign_DecideToggle(false, false) == LZ_TOGGLE_START);
        LZ_CHECK(LzAlign_DecideToggle(true,  false) == LZ_TOGGLE_STOP);
    }

    LZ_CASE("E0b ⚠️ 启动保护期内一律吞掉 —— 不管能不能停");
    {
        /* ---- 本组是现场缺陷的守门断言 ----
         *
         * 现场形态（2026-09-28 实测日志）：按下 → `do_init()` 阻塞约 1.6 秒
         * → 操作员再按一次 → 那一拍被判成"再按一次 = 停止"
         * ⇒ **照准启动后 100 ms 自杀**。
         *
         * 所以保护期内**两个方向都不许走**：既不能停（还没起来），
         * 也不能再启一次（会 restart 掉刚起来的那个）。 */
        LZ_CHECK(LzAlign_DecideToggle(false, true) == LZ_TOGGLE_IGNORED);
        LZ_CHECK(LzAlign_DecideToggle(true,  true) == LZ_TOGGLE_IGNORED);
    }

    LZ_CASE("E0c 保护期优先于 canStop —— 顺序错了这条会红");
    {
        /* `inGrace` 与 `canStop` 同时为真时的行为**是规格**：
         * 若把 `canStop` 判在前面，`(true, true)` 会走 STOP，
         * 也就是"云台刚转起来就被下一次按压停掉" —— 正是要防的那件事。
         * 这条断言把顺序钉死。 */
        LZ_CHECK(LzAlign_DecideToggle(true, true) != LZ_TOGGLE_STOP);
        LZ_CHECK(LzAlign_DecideToggle(true, true) == LZ_TOGGLE_IGNORED);
    }

    /* ================= E. 照准与绕飞的互斥 ================= */

    LZ_CASE("E1 两个方向都要拦");
    {
        LZ_CHECK(LzAlign_CheckConflict(false, false) == LZ_CONFLICT_NONE);
        LZ_CHECK(LzAlign_CheckConflict(true,  false) == LZ_CONFLICT_ORBIT_ACTIVE);
        LZ_CHECK(LzAlign_CheckConflict(false, true)  == LZ_CONFLICT_ALIGN_ACTIVE);
    }

    LZ_CASE("E2 两个都在跑时绕飞优先 —— 这是定义了的优先级");
    {
        /* 不是"碰巧由 if 的次序决定"：绕飞是安全关键的作业，且它已经在跑；
         * 照准是可重来的动作。反过来判会让一条正在执行的航线被一个按钮
         * 弄成"我该让路"的语义。 */
        LZ_CHECK(LzAlign_CheckConflict(true, true) == LZ_CONFLICT_ORBIT_ACTIVE);
    }

    LZ_CASE("E3 冲突说明：NONE 必须是 NULL（调用方据此决定报不报）");
    {
        LZ_CHECK(LzAlign_ConflictStr(LZ_CONFLICT_NONE) == NULL);
        LZ_CHECK(LzAlign_ConflictStr(LZ_CONFLICT_ORBIT_ACTIVE) != NULL);
        LZ_CHECK(LzAlign_ConflictStr(LZ_CONFLICT_ALIGN_ACTIVE) != NULL);
        /* 两条文案必须不同 —— 相同的话操作员看到"绕飞在跑"而实际是
         * 照准在跑，照着他看到的去做会得到"拨开关没反应"。 */
        LZ_CHECK(strcmp(LzAlign_ConflictStr(LZ_CONFLICT_ORBIT_ACTIVE),
                        LzAlign_ConflictStr(LZ_CONFLICT_ALIGN_ACTIVE)) != 0);
    }

    /* ================= F. 两个彼此独立的失败计数器 ================= */

    const LzAlignRetryPolicy RP = LzAlign_DefaultRetryPolicy();

    LZ_CASE("F0 缺省重试上限就是文档写的 30 拍 / 5 次");
    {
        LZ_CHECK(RP.maxFrameWaits == 30);
        LZ_CHECK(RP.maxDetectMisses == 5);
    }

    LZ_CASE("F1 等帧：第 30 次才超限，第 29 次还不算");
    {
        LzAlignRetry r;
        LzAlign_Retry_Reset(&r);
        bool over = false;
        for (int i = 1; i <= 29; ++i) {
            over = LzAlign_Retry_NoteNoFrame(&r, &RP);
            LZ_CHECK(!over);   /* 前 29 次都不能判死 */
        }
        LZ_CHECK(LzAlign_Retry_NoteNoFrame(&r, &RP));   /* 第 30 次 */
    }

    LZ_CASE("F2 ⚠️ 取到帧**只清等帧计数**，绝不能清检测失败计数");
    {
        /* ---- 本文件最重要的一条 ----
         *
         * 这就是"照准永不结束"那个缺陷的守门断言。
         * 旧写法里 `do_grab()` 成功时一句 `s_waitTicks = 0` 把两个计数
         * 一起清了，于是"检测失败 5 次就放弃"永远到不了上限：
         *
         *     GRAB → 失败 ×5（misses=5，**本该判死**）→ 取到帧
         *          → s_waitTicks=0（misses 也被清零）
         *          → DETECT 又失败一次（misses=1）→ 回 GRAB …
         *
         * 循环里 misses 永远在 1~5 之间打转，判据不生效。 */
        LzAlignRetry r;
        LzAlign_Retry_Reset(&r);

        for (int i = 0; i < 4; ++i) {
            LZ_CHECK(!LzAlign_Retry_NoteDetectMiss(&r, &RP));
        }
        LZ_CHECK(r.detectMisses == 4);

        LzAlign_Retry_NoteFrameOk(&r);
        LZ_CHECK(r.detectMisses == 4);   /* ← 回退成"s_waitTicks = 0"时这里变红 */

        /* 第 5 次失败必须判死 */
        LZ_CHECK(LzAlign_Retry_NoteDetectMiss(&r, &RP));
    }

    LZ_CASE("F3 检测成功只清检测失败计数，不动等帧计数");
    {
        LzAlignRetry r;
        LzAlign_Retry_Reset(&r);
        for (int i = 0; i < 10; ++i) {
            (void)LzAlign_Retry_NoteNoFrame(&r, &RP);
        }
        for (int i = 0; i < 3; ++i) {
            (void)LzAlign_Retry_NoteDetectMiss(&r, &RP);
        }
        LZ_CHECK(r.frameWaits == 10);
        LZ_CHECK(r.detectMisses == 3);

        LzAlign_Retry_NoteDetectHit(&r);
        LZ_CHECK(r.detectMisses == 0);
        LZ_CHECK(r.frameWaits == 10);   /* ← 若一并清零则变红 */
    }

    LZ_CASE("F4 等帧成功只清等帧计数，不动检测失败计数");
    {
        LzAlignRetry r;
        LzAlign_Retry_Reset(&r);
        for (int i = 0; i < 4; ++i) {
            (void)LzAlign_Retry_NoteDetectMiss(&r, &RP);
        }
        for (int i = 0; i < 7; ++i) {
            (void)LzAlign_Retry_NoteNoFrame(&r, &RP);
        }
        LzAlign_Retry_NoteFrameOk(&r);
        LZ_CHECK(r.frameWaits == 0);
        LZ_CHECK(r.detectMisses == 4);
    }

    LZ_CASE("F5 上限非正值按 0 处理（一次都不容忍），不当『无上限』");
    {
        /* 把 0 当"无上限"会让一个写错的配置变成"永不放弃" ——
         * 而"永不放弃"在飞行器上是危险方向。宁可早停。 */
        LzAlignRetryPolicy zero;
        zero.maxFrameWaits = 0;
        zero.maxDetectMisses = -3;

        LzAlignRetry r;
        LzAlign_Retry_Reset(&r);
        LZ_CHECK(LzAlign_Retry_NoteNoFrame(&r, &zero));      /* 第 1 次就超 */
        LZ_CHECK(LzAlign_Retry_NoteDetectMiss(&r, &zero));
    }

    LZ_CASE("F6 NULL 状态视为『已超限』，不崩也不继续试");
    {
        LZ_CHECK(LzAlign_Retry_NoteNoFrame(NULL, &RP));
        LZ_CHECK(LzAlign_Retry_NoteDetectMiss(NULL, &RP));
        LzAlign_Retry_NoteFrameOk(NULL);      /* 不应崩 */
        LzAlign_Retry_NoteDetectHit(NULL);
        LzAlign_Retry_Reset(NULL);
    }

    LZ_CASE("死区必须按画面比例算 —— 固定角度在变焦下会失真");
    {
        /* ★ 这条守的是 2026-09-29 发现的形状：
         * 死区原先是固定的 1.5°，而"对准了没有"看在**画面上**，
         * 不在**角度**上 —— 两者只在广角端接近：
         *
         * | 倍率 | VFOV | 1.5° 等于画面高度的 |
         * |---|---|---|
         * | 1.00× | 55.09° | 2.5% |
         * | 7.00× |  8.52° | **17.6%** |
         *
         * ⇒ 与 `poleWinBelowRatioQ`、置信度分母是同一个形状。 */

        /* 1. 正切折算（**不是**线性近似 frac·VFOV） */
        const double d30_wide = LzAlign_DeadzoneDegFromFrac(0.03, 55.09);
        LZ_CHECK_NEAR(d30_wide, 1.79, 0.01);
        LZ_CHECK(fabs(d30_wide - 0.03 * 55.09) > 0.1);   /* 线性给 1.65 */

        /* 2. 同一个比例在变焦下得到**更小**的角度 —— 这是重点 */
        const double d30_zoom = LzAlign_DeadzoneDegFromFrac(0.03, 8.52);
        LZ_CHECK_NEAR(d30_zoom, 0.26, 0.01);
        LZ_CHECK(d30_zoom < d30_wide / 6.0);

        /* 3. 单调：VFOV 越小 → 角度越小 */
        double prev = 1e9;
        for (double vf = 60.0; vf >= 5.0; vf -= 5.0) {
            const double d = LzAlign_DeadzoneDegFromFrac(0.03, vf);
            LZ_CHECK(isfinite(d) && d > 0.0 && d < prev);
            prev = d;
        }

        /* 4. 比例越大 → 角度越大 */
        LZ_CHECK(LzAlign_DeadzoneDegFromFrac(0.06, 55.09) >
                 LzAlign_DeadzoneDegFromFrac(0.03, 55.09));

        /* 5. 退化输入返回 NAN —— 返回 0 会静默变成"必须完全对准" */
        LZ_CHECK(isnan(LzAlign_DeadzoneDegFromFrac(0.0, 55.0)));
        LZ_CHECK(isnan(LzAlign_DeadzoneDegFromFrac(-0.1, 55.0)));
        LZ_CHECK(isnan(LzAlign_DeadzoneDegFromFrac(0.6, 55.0)));
        LZ_CHECK(isnan(LzAlign_DeadzoneDegFromFrac(NAN, 55.0)));
        LZ_CHECK(isnan(LzAlign_DeadzoneDegFromFrac(0.03, 0.0)));
        LZ_CHECK(isnan(LzAlign_DeadzoneDegFromFrac(0.03, 180.0)));
        LZ_CHECK(isnan(LzAlign_DeadzoneDegFromFrac(0.03, NAN)));

        /* 6. 缺省策略里比例是权威值，角度只是广角端的等价初值 */
        const LzAlignPolicy P = LzAlign_DefaultPolicy();
        LZ_CHECK_NEAR(P.deadzoneFrac, 0.03, 1e-12);
        LZ_CHECK_NEAR(P.deadzoneDeg,
                      LzAlign_DeadzoneDegFromFrac(P.deadzoneFrac, 55.09), 0.02);

        /* 7. 非法的 deadzoneFrac 必须被 policy_ok 拦掉（否则静默退回角度值，
         *    而"按画面比例"的意图落空 —— 那种落空在画面上看不出来） */
        {
            LzAlignPolicy bad = LzAlign_DefaultPolicy();
            bad.deadzoneFrac = -0.5;
            const LzAlignDecision d =
                LzAlign_DecideStep(10.0, -20.0, 0, false, &bad);
            /* 退回缺省策略：10° 远超缺省死区 ⇒ 仍是 ROTATE 而不是 CONFIRM */
            LZ_CHECK(d.action == LZ_ALIGN_ACT_ROTATE);
        }
    }

    return LZ_TEST_SUMMARY();
}

/* ------------------------------------------------------------------
 * 反向验证（逐条回退本层实现，确认对应用例变红 —— 已实测）
 *
 * | 回退的改动 | 变红 |
 * |---|---|
 * | 删掉 DecideStep 最前面的 isfinite 检查 | A2/A3（NaN 落到 ROTATE） |
 * | 死区判定 `<=` 改成 `<` | B1（边界那条） |
 * | 死区里直接返回 DONE（去掉复核） | B2、D2 |
 * | 越界改成钳位而非拒绝 | C2（两条 OUT_OF_RANGE） |
 * | 轮数检查挪到死区检查**之前** | D2 |
 * | CheckConflict 两个都为真时返回 ALIGN_ACTIVE | E2 |
 * | NoteFrameOk 改成同时清 detectMisses | F2 |
 * | NoteDetectHit 改成同时清 frameWaits | F3 |
 * | over() 把 max<=0 当无上限 | F5 |
 * | ConflictStr 两条文案写成同一句 | E3 |
 * ------------------------------------------------------------------ */
