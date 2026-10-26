/**
 * @file wt_mission_watch.c
 * @brief 航点任务执行进度判定的实现
 */

#include "wt_mission_watch.h"

#include <stddef.h>
#include <string.h>

void WtMissionWatch_Begin(WtMissionWatch *w, uint32_t nowMs, uint32_t startTimeoutMs)
{
    memset(w, 0, sizeof(*w));
    w->phase = WT_MW_WAITING;
    w->startedMs = nowMs;
    w->startTimeoutMs = startTimeoutMs;
    w->lastRawState = -1;
    w->waypointIndex = -1;
    w->maxWaypointIndex = -1;
    w->lastUpdateMs = nowMs;
}

static bool WtMissionWatch_IsActiveState(int rawState)
{
    switch (rawState) {
    case WT_MW_STATE_PREPARE:
    case WT_MW_STATE_TRANS_MISSION:
    case WT_MW_STATE_MISSION:
    case WT_MW_STATE_BREAK:
    case WT_MW_STATE_RESUME:
    case WT_MW_STATE_RETURN_FIRSTPOINT:
        return true;
    default:
        return false;
    }
}

void WtMissionWatch_OnState(WtMissionWatch *w, int rawState, int waypointIndex, uint32_t nowMs)
{
    if (w == NULL || WtMissionWatch_IsOver(w)) {
        return;
    }

    w->lastRawState = rawState;
    w->lastUpdateMs = nowMs;

    if (WtMissionWatch_IsActiveState(rawState)) {
        w->seenActive = true;
        /* 航点号只在真正执行（含暂停）时才有意义；准备阶段报的是 0 之类的占位 */
        if (rawState == WT_MW_STATE_MISSION || rawState == WT_MW_STATE_BREAK) {
            w->waypointIndex = waypointIndex;
            if (waypointIndex > w->maxWaypointIndex) {
                w->maxWaypointIndex = waypointIndex;
            }
        }
        w->phase = (rawState == WT_MW_STATE_BREAK) ? WT_MW_PAUSED : WT_MW_RUNNING;
        return;
    }

    /*
     * IDLE（及未知值）：只有见到过执行状态之后才算结束。
     * 刚 START 时推来的 IDLE 是旧状态，忽略。
     */
    if (rawState == WT_MW_STATE_IDLE && w->seenActive) {
        w->phase = WT_MW_FINISHED;
    }
}

E_WtMissionWatchPhase WtMissionWatch_Tick(WtMissionWatch *w, uint32_t nowMs)
{
    if (w->phase == WT_MW_WAITING && !w->seenActive &&
        (uint32_t)(nowMs - w->startedMs) > w->startTimeoutMs) {
        w->phase = WT_MW_NOT_STARTED;
    }

    return w->phase;
}

bool WtMissionWatch_IsOver(const WtMissionWatch *w)
{
    return w->phase == WT_MW_FINISHED || w->phase == WT_MW_NOT_STARTED;
}

const char *WtMissionWatch_PhaseName(E_WtMissionWatchPhase phase)
{
    switch (phase) {
    case WT_MW_WAITING:
        return "等待飞机进入航线";
    case WT_MW_RUNNING:
        return "执行中";
    case WT_MW_PAUSED:
        return "已暂停";
    case WT_MW_FINISHED:
        return "已结束";
    case WT_MW_NOT_STARTED:
        return "未能启动";
    default:
        return "?";
    }
}
