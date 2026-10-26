/**
 * @file wt_test_mission_watch.c
 * @brief 航点任务进度判定的回归测试
 *
 * 重点钉住一条踩过的坑：START 之后推来的 IDLE 不能当成「已结束」。
 */

#include "wt_mission_watch.h"
#include "wt_test.h"

static void TestStaleIdleIgnored(void)
{
    WtMissionWatch w;

    WT_CASE("刚 START 时推来的 IDLE 不算结束");
    WtMissionWatch_Begin(&w, 1000, 30000);
    WtMissionWatch_OnState(&w, WT_MW_STATE_IDLE, 0, 1100);
    WT_CHECK_EQ_INT(WtMissionWatch_Tick(&w, 1200), WT_MW_WAITING);
    WT_CHECK(!WtMissionWatch_IsOver(&w));
}

static void TestNormalRun(void)
{
    WtMissionWatch w;

    WT_CASE("准备 → 执行 → 回到 IDLE = 结束，记下走到的航点");
    WtMissionWatch_Begin(&w, 0, 30000);
    WtMissionWatch_OnState(&w, WT_MW_STATE_PREPARE, 0, 500);
    WT_CHECK_EQ_INT(WtMissionWatch_Tick(&w, 500), WT_MW_RUNNING);
    WT_CHECK_EQ_INT(w.waypointIndex, -1); /* 准备阶段的航点号不采信 */
    WtMissionWatch_OnState(&w, WT_MW_STATE_TRANS_MISSION, 0, 1000);
    WtMissionWatch_OnState(&w, WT_MW_STATE_MISSION, 3, 9000);
    WT_CHECK_EQ_INT(w.waypointIndex, 3);
    WtMissionWatch_OnState(&w, WT_MW_STATE_MISSION, 12, 60000);
    WT_CHECK_EQ_INT(WtMissionWatch_Tick(&w, 60000), WT_MW_RUNNING);
    WtMissionWatch_OnState(&w, WT_MW_STATE_IDLE, 0, 61000);
    WT_CHECK_EQ_INT(WtMissionWatch_Tick(&w, 61000), WT_MW_FINISHED);
    WT_CHECK(WtMissionWatch_IsOver(&w));
    WT_CHECK_EQ_INT(w.maxWaypointIndex, 12);
    /* 结束后再来的状态不改判定 */
    WtMissionWatch_OnState(&w, WT_MW_STATE_MISSION, 5, 62000);
    WT_CHECK_EQ_INT(WtMissionWatch_Tick(&w, 62000), WT_MW_FINISHED);
    WT_CHECK_EQ_INT(w.maxWaypointIndex, 12);
}

static void TestPauseResume(void)
{
    WtMissionWatch w;

    WT_CASE("BREAK = 暂停，不是结束；恢复后继续算执行中");
    WtMissionWatch_Begin(&w, 0, 30000);
    WtMissionWatch_OnState(&w, WT_MW_STATE_MISSION, 4, 1000);
    WtMissionWatch_OnState(&w, WT_MW_STATE_BREAK, 4, 2000);
    WT_CHECK_EQ_INT(WtMissionWatch_Tick(&w, 2000), WT_MW_PAUSED);
    WT_CHECK(!WtMissionWatch_IsOver(&w));
    WtMissionWatch_OnState(&w, WT_MW_STATE_RESUME, 4, 3000);
    WT_CHECK_EQ_INT(WtMissionWatch_Tick(&w, 3000), WT_MW_RUNNING);
    WtMissionWatch_OnState(&w, WT_MW_STATE_BREAK, 6, 4000);
    WtMissionWatch_OnState(&w, WT_MW_STATE_IDLE, 0, 5000); /* 暂停中被停掉 */
    WT_CHECK_EQ_INT(WtMissionWatch_Tick(&w, 5000), WT_MW_FINISHED);
    WT_CHECK_EQ_INT(w.maxWaypointIndex, 6);
}

static void TestNeverStarted(void)
{
    WtMissionWatch w;

    WT_CASE("START 后一直见不到执行状态：超时判为未能启动");
    WtMissionWatch_Begin(&w, 0xFFFFF000u, 30000); /* 起点贴着 32 位回绕 */
    WtMissionWatch_OnState(&w, WT_MW_STATE_IDLE, 0, 0xFFFFF100u);
    WT_CHECK_EQ_INT(WtMissionWatch_Tick(&w, 0xFFFFF000u + 29999u), WT_MW_WAITING);
    WT_CHECK_EQ_INT(WtMissionWatch_Tick(&w, 0xFFFFF000u + 30001u), WT_MW_NOT_STARTED);
    WT_CHECK(WtMissionWatch_IsOver(&w));

    WT_CASE("见到过执行状态就不会再判未启动（哪怕很久没新状态）");
    WtMissionWatch_Begin(&w, 0, 30000);
    WtMissionWatch_OnState(&w, WT_MW_STATE_MISSION, 1, 1000);
    WT_CHECK_EQ_INT(WtMissionWatch_Tick(&w, 900000), WT_MW_RUNNING);
}

int main(void)
{
    TestStaleIdleIgnored();
    TestNormalRun();
    TestPauseResume();
    TestNeverStarted();

    return WT_TEST_SUMMARY();
}
