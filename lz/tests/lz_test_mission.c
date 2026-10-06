/**
 * @file lz_test_mission.c
 * @brief 绕飞状态机**决策逻辑**的回归测试（零依赖，桌面可跑）。
 *
 * ## 为什么这一层最该被测
 *
 * 状态机此前整个在 `app/lz_mission.c` 里 ⇒ 桌面测不到 ⇒ **整张转移表
 * 一条断言都没有**。而走错的后果是**安全相关的谎话**：
 * "飞机还在杆旁边绕，而日志与界面都说已经停了"。
 *
 * ## 本文件守什么
 *
 * | 用例组 | 守的东西 | 写错的表现 |
 * |---|---|---|
 * | A 空闲态 | 开关 OFF 什么都不做；ON 才启动 | 一直在启动/上传 |
 * | B 互斥 | 照准在跑时不启动，**但不清开关** | 逼操作员再拨一次 ON |
 * | C 启动结果 | 失败要报且**不清开关** | 界面与事实不一致 |
 * | D 停止 | 失败**留在 RUNNING** | 下一拍重新上传启动（危险） |
 * | E 结束 | 只在 RUNNING 时认 IDLE 事件 | 任务刚启动就被判结束 |
 * | F 去重闸 | 三条各自独立、各自只报一次 | 浮窗被刷屏，控件回执被挤掉 |
 *
 * ⚠️ **D 组是这一层的核心**。见 `LzMission_Apply()` 的注释。
 */

#include "lz_mission_logic.h"
#include "lz_test.h"

#include <string.h>

/** 一个"什么都没发生"的输入：空闲、开关 OFF、没照准、没结束 */
static LzMissionInput idle_input(void)
{
    LzMissionInput in;
    memset(&in, 0, sizeof(in));
    in.state = LZ_MISSION_STATE_IDLE;
    return in;
}

/** 一个"绕飞正在跑、开关 ON"的输入 */
static LzMissionInput running_input(void)
{
    LzMissionInput in;
    memset(&in, 0, sizeof(in));
    in.state = LZ_MISSION_STATE_RUNNING;
    in.orbitRequested = true;
    return in;
}

int main(void)
{
    /* ---------------- A 空闲态 ---------------- */
    LZ_CASE("A1 开关 OFF 时什么都不做");
    {
        const LzMissionInput in = idle_input();
        const LzMissionDecision d = LzMission_Decide(&in);
        LZ_CHECK(d.step == LZ_MISSION_STEP_NONE);
        LZ_CHECK(!d.reportOrbitDeferred);
    }

    LZ_CASE("A2 开关 ON 且没有照准 ⇒ 启动绕飞");
    {
        LzMissionInput in = idle_input();
        in.orbitRequested = true;
        const LzMissionDecision d = LzMission_Decide(&in);
        LZ_CHECK(d.step == LZ_MISSION_STEP_START_ORBIT);
    }

    /* ---------------- B 互斥 ---------------- */
    LZ_CASE("B1 照准在跑 ⇒ **不**启动绕飞");
    {
        LzMissionInput in = idle_input();
        in.orbitRequested = true;
        in.conflictAlignActive = true;
        const LzMissionDecision d = LzMission_Decide(&in);
        LZ_CHECK(d.step == LZ_MISSION_STEP_NONE);
    }

    LZ_CASE("B2 照准在跑时**报一次**提示，且**不清开关**（step 是 NONE 不是别的）");
    {
        /* ⚠️ **"不清开关"是这条的核心**：操作员的意思多半是"先识别、再绕飞"，
         * 清掉开关会逼他再拨一次 ON —— 而他记得自己已经拨过了。
         *
         * 判据：`Decide` 给出的 step 必须是 NONE（即"什么都别做"）。
         * 若哪天有人加了一个 `LZ_MISSION_STEP_CLEAR_SWITCH` 之类，
         * 这条会红 —— 那正是它该红的时候。 */
        LzMissionInput in = idle_input();
        in.orbitRequested = true;
        in.conflictAlignActive = true;
        const LzMissionDecision d = LzMission_Decide(&in);
        LZ_CHECK(d.step == LZ_MISSION_STEP_NONE);
        LZ_CHECK(d.reportOrbitDeferred);
        LZ_CHECK(d.orbitDeferredReported);   /* 闸已置 */
    }

    LZ_CASE("B3 照准期间**只报一次** —— 第二拍不再报");
    {
        LzMissionInput in = idle_input();
        in.orbitRequested = true;
        in.conflictAlignActive = true;
        LzMissionDecision d = LzMission_Decide(&in);
        LZ_CHECK(d.reportOrbitDeferred);
        in.orbitDeferredReported = d.orbitDeferredReported;   /* 存回 */
        d = LzMission_Decide(&in);
        LZ_CHECK(!d.reportOrbitDeferred);      /* 第二拍静默 */
        LZ_CHECK(d.step == LZ_MISSION_STEP_NONE);
    }

    LZ_CASE("B4 照准结束后**自然启动**，不需要操作员再拨一次");
    {
        /* 这条是 B2"不清开关"的**目的**：开关一直 ON，照准一结束下一拍
         * 就该启动。若 B2 改成清开关，这条会红。 */
        LzMissionInput in = idle_input();
        in.orbitRequested = true;              /* 开关还在 ON */
        in.conflictAlignActive = false;        /* 照准结束了 */
        in.orbitDeferredReported = true;       /* 之前报过提示 */
        const LzMissionDecision d = LzMission_Decide(&in);
        LZ_CHECK(d.step == LZ_MISSION_STEP_START_ORBIT);
        LZ_CHECK(!d.reportOrbitDeferred);
    }

    LZ_CASE("B5 操作员在照准期间把开关拨回 OFF ⇒ 闸归零（下次能再报）");
    {
        LzMissionInput in = idle_input();
        in.orbitRequested = false;
        in.orbitDeferredReported = true;
        const LzMissionDecision d = LzMission_Decide(&in);
        LZ_CHECK(!d.orbitDeferredReported);
        LZ_CHECK(d.step == LZ_MISSION_STEP_NONE);
    }

    /* ---------------- C 启动结果 ---------------- */
    LZ_CASE("C1 启动成功 ⇒ 进 RUNNING，不报任何消息");
    {
        LzMissionApplyInput ai;
        memset(&ai, 0, sizeof(ai));
        ai.state = LZ_MISSION_STATE_IDLE;
        ai.decision.step = LZ_MISSION_STEP_START_ORBIT;
        ai.sdkResult = LZ_OK;
        const LzMissionApplyOutput o = LzMission_Apply(&ai);
        LZ_CHECK(o.state == LZ_MISSION_STATE_RUNNING);
        LZ_CHECK(!o.reportStartFailed);
        LZ_CHECK(!o.reportOperatorStop);
    }

    LZ_CASE("C2 启动失败 ⇒ 回 IDLE 并报「启动失败」");
    {
        LzMissionApplyInput ai;
        memset(&ai, 0, sizeof(ai));
        ai.state = LZ_MISSION_STATE_IDLE;
        ai.decision.step = LZ_MISSION_STEP_START_ORBIT;
        ai.sdkResult = LZ_ERR_UPLOAD;
        const LzMissionApplyOutput o = LzMission_Apply(&ai);
        LZ_CHECK(o.state == LZ_MISSION_STATE_IDLE);
        LZ_CHECK(o.reportStartFailed);
    }

    /* ---------------- D 停止（核心） ---------------- */
    LZ_CASE("D1 停止成功 ⇒ 回 IDLE 并报「操作员停止」");
    {
        LzMissionApplyInput ai;
        memset(&ai, 0, sizeof(ai));
        ai.state = LZ_MISSION_STATE_RUNNING;
        ai.decision.step = LZ_MISSION_STEP_STOP_MISSION;
        ai.sdkResult = LZ_OK;
        const LzMissionApplyOutput o = LzMission_Apply(&ai);
        LZ_CHECK(o.state == LZ_MISSION_STATE_IDLE);
        LZ_CHECK(o.reportOperatorStop);
        LZ_CHECK(!o.reportStopRejected);
    }

    LZ_CASE("D2 ⚠️ 停止**失败** ⇒ 必须留在 RUNNING（状态与事实一致）");
    {
        /* ⚠️ **这是本层最重要的一条。**
         *
         * 早先写成 `(void)LzBridge_StopMissionV3();` 然后**无条件**置 IDLE、
         * 无条件报"绕飞结束：操作员停止" —— 于是 STOP 被拒时飞机还在杆旁边
         * 绕，而日志与界面都说已经停了。**静默的失败等于假装成功**，
         * 而在飞控语境里这直接关系到安全。
         *
         * 更糟的第二步：置 IDLE 之后开关还在 ON 位 ⇒ 下一拍
         * `Decide` 看到"IDLE + 开关 ON" ⇒ 当成**新的绕飞请求**重新上传启动。
         * 所以 D2 与 D3 是一对，缺一不可。 */
        LzMissionApplyInput ai;
        memset(&ai, 0, sizeof(ai));
        ai.state = LZ_MISSION_STATE_RUNNING;
        ai.decision.step = LZ_MISSION_STEP_STOP_MISSION;
        ai.sdkResult = LZ_ERR_START;   /* 被拒 */
        const LzMissionApplyOutput o = LzMission_Apply(&ai);
        LZ_CHECK(o.state == LZ_MISSION_STATE_RUNNING);   /* ← 不是 IDLE */
        LZ_CHECK(o.reportStopRejected);
        LZ_CHECK(!o.reportOperatorStop);   /* **不能**报"已结束" */
    }

    LZ_CASE("D3 ⚠️ 停止失败后下一拍**不许**重新启动（D2 的后果）");
    {
        /* 串起两相：D2 的输出喂给下一拍的 `Decide`。
         * 若 D2 置了 IDLE，这里就会得到 START_ORBIT —— 飞机被重新上传启动。 */
        LzMissionApplyInput ai;
        memset(&ai, 0, sizeof(ai));
        ai.state = LZ_MISSION_STATE_RUNNING;
        ai.decision.step = LZ_MISSION_STEP_STOP_MISSION;
        ai.sdkResult = LZ_ERR_START;
        const LzMissionApplyOutput o = LzMission_Apply(&ai);

        LzMissionInput next;
        memset(&next, 0, sizeof(next));
        next.state = o.state;             /* 采纳新状态 */
        next.orbitRequested = false;      /* 开关仍在 OFF 位（操作员拨过了） */
        const LzMissionDecision d = LzMission_Decide(&next);
        LZ_CHECK(d.step != LZ_MISSION_STEP_START_ORBIT);
    }

    LZ_CASE("D4 停止被拒**只报一次**（Tick 是 100 ms 一拍）");
    {
        LzMissionApplyInput ai;
        memset(&ai, 0, sizeof(ai));
        ai.state = LZ_MISSION_STATE_RUNNING;
        ai.decision.step = LZ_MISSION_STEP_STOP_MISSION;
        ai.sdkResult = LZ_ERR_START;
        LzMissionApplyOutput o = LzMission_Apply(&ai);
        LZ_CHECK(o.reportStopRejected);
        LZ_CHECK(o.stopRejectReported);

        /* 第二拍：闸已经置上 */
        ai.stopRejectReported = o.stopRejectReported;
        o = LzMission_Apply(&ai);
        LZ_CHECK(!o.reportStopRejected);          /* 静默 */
        LZ_CHECK(o.state == LZ_MISSION_STATE_RUNNING);
    }

    LZ_CASE("D5 停成功时把闸归零 —— 下一轮作业还能再报一次");
    {
        LzMissionApplyInput ai;
        memset(&ai, 0, sizeof(ai));
        ai.state = LZ_MISSION_STATE_RUNNING;
        ai.decision.step = LZ_MISSION_STEP_STOP_MISSION;
        ai.sdkResult = LZ_OK;
        ai.stopRejectReported = true;    /* 上一轮留下的 */
        const LzMissionApplyOutput o = LzMission_Apply(&ai);
        LZ_CHECK(!o.stopRejectReported);
    }

    /* ---------------- E 结束 ---------------- */
    LZ_CASE("E1 只在 RUNNING 时认「航点 IDLE」事件");
    {
        /* ⚠️ **RUNNING 期间也会收到 IDLE** —— 见到就置标志的话，
         * 任务刚启动就会被判成结束。 */
        LZ_CHECK(LzMission_ShouldNoteEnded(LZ_MISSION_STATE_RUNNING));
        LZ_CHECK(!LzMission_ShouldNoteEnded(LZ_MISSION_STATE_IDLE));
    }

    LZ_CASE("E2 飞机侧报结束 ⇒ 专门的 step（**不是** NONE）");
    {
        /* ⚠️ 第一版把这条写成 `NONE` + "调用方自己收尾"，理由是"没有判据"。
         * **那是错的** —— "RUNNING + ended ⇒ 收尾"本身就是判据，
         * 写成 NONE 等于把它留在 app 层测不到。
         * 现在它有专门的 step，收尾逻辑在 `Apply()` 里被断言。 */
        LzMissionInput in = running_input();
        in.missionEnded = true;
        const LzMissionDecision d = LzMission_Decide(&in);
        LZ_CHECK(d.step == LZ_MISSION_STEP_MISSION_ENDED);

        LzMissionApplyInput ai;
        memset(&ai, 0, sizeof(ai));
        ai.state = LZ_MISSION_STATE_RUNNING;
        ai.decision = d;
        const LzMissionApplyOutput o = LzMission_Apply(&ai);
        LZ_CHECK(o.state == LZ_MISSION_STATE_IDLE);
        LZ_CHECK(o.reportMissionEnded);
    }

    LZ_CASE("E2b IDLE 态收到结束事件 ⇒ 什么都不做（不该报「绕飞结束」）");
    {
        /* 状态本来就是 IDLE，"结束"这件事无从谈起。 */
        LzMissionInput in = idle_input();
        in.missionEnded = true;
        const LzMissionDecision d = LzMission_Decide(&in);
        LZ_CHECK(d.step != LZ_MISSION_STEP_MISSION_ENDED);
    }

    LZ_CASE("E3 没结束、开关还在 ON ⇒ 什么都不做，状态保持 RUNNING");
    {
        const LzMissionInput in = running_input();
        const LzMissionDecision d = LzMission_Decide(&in);
        LZ_CHECK(d.step == LZ_MISSION_STEP_NONE);

        /* ⚠️ **`Apply` 的输出状态在 step == NONE 时必须是传入的当前状态** ——
         * 第一版默认给 IDLE，靠"调用方记得别采纳"兜，而那种约定一定会有人忘，
         * 忘了的表现是 **RUNNING 被悄悄打回 IDLE**（开关还在 ON ⇒ 下一拍
         * 重新上传启动）。 */
        LzMissionApplyInput ai;
        memset(&ai, 0, sizeof(ai));
        ai.state = LZ_MISSION_STATE_RUNNING;
        ai.decision = d;
        const LzMissionApplyOutput o = LzMission_Apply(&ai);
        LZ_CHECK(o.state == LZ_MISSION_STATE_RUNNING);
    }

    /* ---------------- F 去重闸与航点号 ---------------- */
    LZ_CASE("F1 航点号没变就不报");
    {
        int last = -1;
        LZ_CHECK(LzMission_ShouldReportWaypoint(1, &last));
        LZ_CHECK(last == 1);
        LZ_CHECK(!LzMission_ShouldReportWaypoint(1, &last));
        LZ_CHECK(!LzMission_ShouldReportWaypoint(1, &last));
        LZ_CHECK(LzMission_ShouldReportWaypoint(2, &last));
        LZ_CHECK(last == 2);
    }

    LZ_CASE("F2 新一轮作业从 -1 复位后，第一个航点必须报");
    {
        int last = -1;
        LZ_CHECK(LzMission_ShouldReportWaypoint(0, &last));   /* 索引 0 也要报 */
        LZ_CHECK(last == 0);
    }

    LZ_CASE("F3 NULL 的 lastIdx 必须安全返回 false");
    {
        LZ_CHECK(!LzMission_ShouldReportWaypoint(3, NULL));
    }

    LZ_CASE("F4 三个去重闸互不影响（照准那条与停止那条各走各的）");
    {
        /* 合并成一个闸的话，照准期间报过一次就会把"停止被拒"那条吃掉 ——
         * 而后者是安全相关的。这条守的是"它们确实是三个独立变量"。 */
        LzMissionInput in = idle_input();
        in.orbitRequested = true;
        in.conflictAlignActive = true;
        in.orbitDeferredReported = true;   /* 照准那条已报 */
        const LzMissionDecision d = LzMission_Decide(&in);
        LZ_CHECK(!d.reportOrbitDeferred);

        LzMissionApplyInput ai;
        memset(&ai, 0, sizeof(ai));
        ai.state = LZ_MISSION_STATE_RUNNING;
        ai.decision.step = LZ_MISSION_STEP_STOP_MISSION;
        ai.sdkResult = LZ_ERR_START;
        ai.stopRejectReported = false;     /* 停止那条**没**报过 */
        const LzMissionApplyOutput o = LzMission_Apply(&ai);
        LZ_CHECK(o.reportStopRejected);    /* 照样要报 */
    }

    LZ_CASE("F5 入参 NULL 不许崩");
    {
        const LzMissionDecision d = LzMission_Decide(NULL);
        LZ_CHECK(d.step == LZ_MISSION_STEP_NONE);
        const LzMissionApplyOutput o = LzMission_Apply(NULL);
        LZ_CHECK(o.state == LZ_MISSION_STATE_IDLE);
        LZ_CHECK(!o.reportStartFailed && !o.reportOperatorStop);
    }

    /* ---------------- G 穷举：所有输入组合的不变式 ---------------- */
    LZ_CASE("G1 穷举 32 组输入：四条不变式一条都不许破");
    {
        /* 逐条用例只能覆盖想到的组合；穷举覆盖**全部**。
         * 2(状态) × 2(开关) × 2(照准) × 2(结束) × 2(去重闸) = 32 组。
         *
         * 四条不变式：
         *   1. `step == NONE` ⇒ 状态**原样保持**（否则 RUNNING 会被悄悄打回）
         *   2. 最多报**一条**消息（浮窗带宽 2 KB/s）
         *   3. IDLE 且开关 OFF ⇒ 什么都不做
         *   4. 照准在跑 ⇒ **绝不**启动绕飞 */
        int violations = 0;
        for (int st = 0; st < 2; ++st)
        for (int req = 0; req < 2; ++req)
        for (int cf = 0; cf < 2; ++cf)
        for (int end = 0; end < 2; ++end)
        for (int gate = 0; gate < 2; ++gate) {
            LzMissionInput in;
            memset(&in, 0, sizeof(in));
            in.state = (LzMissionState)st;
            in.orbitRequested = (req != 0);
            in.conflictAlignActive = (cf != 0);
            in.missionEnded = (end != 0);
            in.orbitDeferredReported = (gate != 0);

            const LzMissionDecision d = LzMission_Decide(&in);

            LzMissionApplyInput ai;
            memset(&ai, 0, sizeof(ai));
            ai.decision = d;
            ai.state = in.state;
            ai.sdkResult = LZ_OK;
            ai.stopRejectReported = (gate != 0);
            const LzMissionApplyOutput o = LzMission_Apply(&ai);

            if (d.step == LZ_MISSION_STEP_NONE && o.state != in.state) {
                violations++;   /* 不变式 1 */
            }
            const int msgs = (int)o.reportOrbitDeferred + (int)o.reportStartFailed
                           + (int)o.reportStopRejected + (int)o.reportOperatorStop
                           + (int)o.reportMissionEnded;
            if (msgs > 1) {
                violations++;   /* 不变式 2 */
            }
            if (in.state == LZ_MISSION_STATE_IDLE && !in.orbitRequested
                && d.step != LZ_MISSION_STEP_NONE) {
                violations++;   /* 不变式 3 */
            }
            if (in.conflictAlignActive && d.step == LZ_MISSION_STEP_START_ORBIT) {
                violations++;   /* 不变式 4 */
            }
        }
        LZ_CHECK(violations == 0);
        if (violations != 0) {
            printf("    32 组输入里违反不变式 %d 次\n", violations);
        }
    }

    return LZ_TEST_SUMMARY();
}

/* ------------------------------------------------------------------
 * 反向验证（逐条回退本层实现，确认对应用例变红 —— 已实测）
 *
 * | 回退的改动 | 结果 |
 * |---|---|
 * | 停止失败时置 IDLE 而非 RUNNING | **3 项失败**（D2、D3、D4 的 state） |
 * | 停止失败也报 `reportOperatorStop` | D2 |
 * | 照准在跑时改成"清开关"（step 换成 START 或别的） | B2、B3、B4 |
 * | 照准那条去掉去重闸（每拍都报） | B3 |
 * | 操作员拨回 OFF 时不归零去重闸 | B5 |
 * | `Apply` 在 `step == NONE` 时返回 IDLE | E3 |
 * | `ShouldNoteEnded` 不看状态直接返回 true | E1 |
 * | `ShouldReportWaypoint` 去掉"变了才报" | F1 |
 * | `Decide` 在 RUNNING+ended 时给 NONE（第一版的写法） | E2 |
 * | `Apply` 的 MISSION_ENDED 分支不置 IDLE | E2 |
 * | IDLE 态收到 ended 也进 MISSION_ENDED | E2b |
 * ------------------------------------------------------------------ */
