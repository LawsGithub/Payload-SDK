/**
 * @file lz_mission_logic.c
 * @brief 绕飞状态机的决策逻辑 —— 见头文件里的说明与三条"刻意如此"。
 */

#include "lz_mission_logic.h"

LzMissionDecision LzMission_Decide(const LzMissionInput *in)
{
    LzMissionDecision d;
    d.step = LZ_MISSION_STEP_NONE;
    d.orbitDeferredReported = false;
    d.reportOrbitDeferred = false;
    if (in == NULL) {
        return d;
    }
    d.orbitDeferredReported = in->orbitDeferredReported;

    switch (in->state) {
    case LZ_MISSION_STATE_IDLE:
        if (!in->orbitRequested) {
            /* 操作员没请求 —— 顺手把去重闸归零：下次拨 ON 时
             * "照准在跑"那条应该能再报一次。 */
            d.orbitDeferredReported = false;
            break;
        }
        /* ⚠️ **互斥**：照准正在转云台时不许启动航线 —— 航线里的
         * `gimbalRotate` 与手动云台控制会抢同一个云台，而"抢"的表现是
         * 命令被拒或姿态诡异，都不指向真实病因。
         *
         * 刻意**不在这里把开关清掉**：操作员的意思很可能是
         * "先识别、再绕飞"，此刻清掉开关会让他在照准结束后**还得再拨
         * 一次 ON** —— 而他记得自己已经拨过了。
         * 所以状态停在 IDLE、开关保持 ON：照准一结束，下一拍就自然启动。 */
        if (in->conflictAlignActive) {
            if (!in->orbitDeferredReported) {
                d.orbitDeferredReported = true;
                d.reportOrbitDeferred = true;
            }
            break;
        }
        d.orbitDeferredReported = false;
        d.step = LZ_MISSION_STEP_START_ORBIT;
        break;

    case LZ_MISSION_STATE_RUNNING:
        /* 操作员中途拨 OFF → 立即停（见 lz_widget.h 的安全语义） */
        if (!in->orbitRequested) {
            d.step = LZ_MISSION_STEP_STOP_MISSION;
            break;
        }
        /* 飞机侧报告结束 ⇒ 收尾（无 SDK 调用，但**是一条判据**） */
        if (in->missionEnded) {
            d.step = LZ_MISSION_STEP_MISSION_ENDED;
        }
        break;
    }
    return d;
}

LzMissionApplyOutput LzMission_Apply(const LzMissionApplyInput *in)
{
    LzMissionApplyOutput o;
    o.state = LZ_MISSION_STATE_IDLE;
    o.stopRejectReported = false;
    o.reportOrbitDeferred = false;
    o.reportStartFailed = false;
    o.reportStopRejected = false;
    o.reportOperatorStop = false;
    o.reportMissionEnded = false;
    if (in == NULL) {
        return o;
    }
    /* ⚠️ 默认**原样带回当前状态**，不是 IDLE —— 见头文件里那段说明。 */
    o.state = in->state;
    o.stopRejectReported = in->stopRejectReported;
    o.reportOrbitDeferred = in->decision.reportOrbitDeferred;

    switch (in->decision.step) {
    case LZ_MISSION_STEP_NONE:
        /* 没有 SDK 调用 ⇒ 状态原样保持（上面已经设好）。 */
        break;

    case LZ_MISSION_STEP_MISSION_ENDED:
        /* 收尾：回 IDLE 并报一条。**原因文本**由调用方给（读 `s_endReason`）。 */
        o.state = LZ_MISSION_STATE_IDLE;
        o.reportMissionEnded = true;
        break;

    case LZ_MISSION_STEP_START_ORBIT:
        if (in->sdkResult == LZ_OK) {
            o.state = LZ_MISSION_STATE_RUNNING;
        } else {
            /* 启动失败：清本地意图并发结束消息。
             * ⚠️ 这**不会**把 Pilot 上的开关拨回去（PSDK 没有那个接口）——
             * 实际是"开关保持 ON 而浮窗说启动失败"，所以消息里带上
             * "请手动拨回"，不假装界面已经一致了。 */
            o.state = LZ_MISSION_STATE_IDLE;
            o.reportStartFailed = true;
        }
        break;

    case LZ_MISSION_STEP_STOP_MISSION:
        if (in->sdkResult == LZ_OK) {
            o.state = LZ_MISSION_STATE_IDLE;
            o.reportOperatorStop = true;
            o.stopRejectReported = false;   /* 停了，闸归零 */
        } else {
            /* ⚠️ **留在 RUNNING**：状态必须与事实一致（任务确实还在跑）。
             *
             * 若在这里置 IDLE，开关还在 ON 位而状态机认为空闲 ——
             * 那个组合会让下一拍把它当成"新的绕飞请求"重新上传启动。
             * 早先还写成 `(void)LzBridge_StopMissionV3();` 然后无条件
             * 报"绕飞结束" —— 于是飞机还在杆旁边绕，而日志与界面都说停了。 */
            o.state = LZ_MISSION_STATE_RUNNING;
            if (!in->stopRejectReported) {
                o.stopRejectReported = true;
                o.reportStopRejected = true;
            }
        }
        break;
    }
    return o;
}

bool LzMission_ShouldReportWaypoint(int idx, int *lastIdx)
{
    if (lastIdx == NULL) {
        return false;
    }
    if (idx == *lastIdx) {
        return false;
    }
    *lastIdx = idx;
    return true;
}

bool LzMission_ShouldNoteEnded(LzMissionState state)
{
    return state == LZ_MISSION_STATE_RUNNING;
}
