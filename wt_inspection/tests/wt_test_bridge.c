/**
 * @file wt_test_bridge.c
 * @brief 桥接层回归测试：KMZ 下发的速度必须与「预计耗时」同源
 *
 * 这是本次修复的收口用例。此前规划器与桥接层各写了一份「哪些航段算巡检段」
 * 的判据，方向还相反（一个按入边、一个按出边），于是报告上的预计耗时与飞机
 * 真正怎么飞是两回事。这里用一条人工航线把两条路径的输出直接对账。
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wt_bridge.h"
#include "wt_plan.h"
#include "wt_test.h"
#include "wt_test_fixture.h"

#define WT_TEST_CSV_FILE "wt_test_bridge_actions.csv"
#define WT_TEST_KMZ_FILE "wt_test_bridge_wayline.kmz"

/**
 * 造一条覆盖全部航段类型的航线：
 *   entry -> b0p1 -> b0p2 -> b1p1 -> detour -> b1p2 -> exit
 * 期望的速度：只有 b0p1->b0p2、detour->b1p2 之外的同片同面相邻段走巡检速度。
 */
static void BuildMixedMission(WtMission *m)
{
    WtPlanPoint entry = WtTest_MakePoint(0, 0, 0, -1, -1.0, -1, WT_PHOTO_NONE);
    WtPlanPoint b0p1 = WtTest_MakePoint(10, 0, 0, 0, 0.2, 0, WT_PHOTO_DEFECT);
    WtPlanPoint b0p2 = WtTest_MakePoint(12, 0, 0, 0, 0.3, 0, WT_PHOTO_DEFECT);
    WtPlanPoint b1p1 = WtTest_MakePoint(80, 0, 0, 1, 0.2, 0, WT_PHOTO_DEFECT);
    WtPlanPoint detour = WtTest_MakePoint(90, 0, 0, -1, -1.0, -1, WT_PHOTO_NONE);
    WtPlanPoint b1p2 = WtTest_MakePoint(92, 0, 0, 1, 0.3, 0, WT_PHOTO_DEFECT);
    WtPlanPoint exit = WtTest_MakePoint(200, 0, 0, -1, -1.0, -1, WT_PHOTO_NONE);

    WtMission_Init(m);
    WtMission_Append(m, &entry);
    WtMission_Append(m, &b0p1);
    WtMission_Append(m, &b0p2);
    WtMission_Append(m, &b1p1);
    WtMission_Append(m, &detour);
    WtMission_Append(m, &b1p2);
    WtMission_Append(m, &exit);
}

static void TestFlightParamsSpeedRule(void)
{
    WtInspectionProfile profile = WtTest_ProfileTiny();
    WtMission m;
    WtMissionPlan plan;
    WtGeo takeoff;

    WT_CASE("flightParams 的速度按「入边」判定，首点与转场点走巡航");

    BuildMixedMission(&m);
    memset(&takeoff, 0, sizeof(takeoff));
    takeoff.alt = 1450.0;

    WtBridge_InitPlan(&plan);
    WT_CHECK_EQ_INT(WtBridge_BuildFlightParams(&m, &takeoff, &profile, &plan), WT_PLAN_OK);

    if (plan.flightParams != NULL) {
        /* 下标：0 entry, 1 b0p1, 2 b0p2, 3 b1p1, 4 detour, 5 b1p2, 6 exit */
        WT_CHECK_NEAR(plan.flightParams[0].speedMs, 8.0, 1e-9); /* 首点无入边 */
        WT_CHECK_NEAR(plan.flightParams[1].speedMs, 8.0, 1e-9); /* entry->b0p1 */
        WT_CHECK_NEAR(plan.flightParams[2].speedMs, 2.0, 1e-9); /* b0p1->b0p2 巡检 */
        WT_CHECK_NEAR(plan.flightParams[3].speedMs, 8.0, 1e-9); /* 叶片间转移 */
        WT_CHECK_NEAR(plan.flightParams[4].speedMs, 8.0, 1e-9); /* 进绕行点 */
        WT_CHECK_NEAR(plan.flightParams[5].speedMs, 8.0, 1e-9); /* 出绕行点 */
        WT_CHECK_NEAR(plan.flightParams[6].speedMs, 8.0, 1e-9); /* 退场 */
    }

    WtBridge_FreePlan(&plan);
    WtMission_Free(&m);
}

static void TestStatsAndFlightParamsAgree(void)
{
    WtInspectionProfile profile = WtTest_ProfileTiny();
    WtMission m;
    WtMissionPlan plan;
    WtGeo takeoff;
    double sum = 0.0;
    size_t i;

    WT_CASE("预计耗时与下发速度同源：逐段用 flightParams 的速度重算，应与 durationSec 相等");

    BuildMixedMission(&m);
    memset(&takeoff, 0, sizeof(takeoff));
    takeoff.alt = 1450.0;

    WtBridge_InitPlan(&plan);
    WT_CHECK_EQ_INT(WtBridge_BuildFlightParams(&m, &takeoff, &profile, &plan), WT_PLAN_OK);
    WtMission_ComputeStats(&m, &profile);

    if (plan.flightParams != NULL) {
        for (i = 1; i < m.count; i++) {
            double d = WtEnu_Distance(m.points[i - 1].enu, m.points[i].enu);

            sum += d / plan.flightParams[i].speedMs;
        }
        for (i = 0; i < m.count; i++) {
            if (WtPlanPoint_TakesPhoto(&m.points[i])) {
                sum += profile.photoDwellSec;
            }
        }

        /* 这里相等是「同源」的可执行证据：两处若各写一套判据，必然对不上 */
        WT_CHECK_NEAR(sum, m.durationSec, 1e-9);
    }

    WtBridge_FreePlan(&plan);
    WtMission_Free(&m);
}

static void TestActionMapping(void)
{
    WtMission m;
    WtMissionPlan plan;

    WT_CASE("动作映射：转场与绕行点不触发快门");

    BuildMixedMission(&m);

    WtBridge_InitPlan(&plan);
    WT_CHECK_EQ_INT(WtBridge_BuildActions(&m, WT_GIMBAL_FREE_YAW, &plan), WT_PLAN_OK);

    if (plan.actions.items != NULL) {
        WT_CHECK_EQ_INT(plan.actions.items[0].camera, WT_ACTION_NONE);  /* entry */
        WT_CHECK_EQ_INT(plan.actions.items[1].camera, WT_ACTION_TAKE_PHOTO);
        WT_CHECK_EQ_INT(plan.actions.items[2].camera, WT_ACTION_TAKE_PHOTO);
        WT_CHECK_EQ_INT(plan.actions.items[3].camera, WT_ACTION_TAKE_PHOTO);
        WT_CHECK_EQ_INT(plan.actions.items[4].camera, WT_ACTION_NONE);  /* detour */
        WT_CHECK_EQ_INT(plan.actions.items[5].camera, WT_ACTION_TAKE_PHOTO);
        WT_CHECK_EQ_INT(plan.actions.items[6].camera, WT_ACTION_NONE);  /* exit */
    }

    WtBridge_FreePlan(&plan);
    WtMission_Free(&m);
}

/**
 * @brief 查找 KMZ 里的文段
 *
 * 不能直接用 strstr：KMZ 是 zip，正文之前有 CRC 等二进制字段，其中含 0 字节，
 * strstr 会在那里提前结束。必须按显式长度搜索。
 */
static const char *FindBytes(const char *hay, size_t hayLen, const char *needle)
{
    size_t nLen = strlen(needle);
    size_t i;

    if (nLen == 0 || hayLen < nLen) {
        return NULL;
    }

    for (i = 0; i + nLen <= hayLen; i++) {
        if (memcmp(hay + i, needle, nLen) == 0) {
            return hay + i;
        }
    }

    return NULL;
}

static void TestKmzSpeedTags(void)
{
    WtTurbineSpec spec = WtTest_Spec155();
    WtInspectionProfile profile = WtInspectionProfile_Default();
    WtRotorFrame frame = WtTurbine_BuildRotorFrame(&spec, 0.0);
    WtMission m;
    WtMissionPlan plan;
    WtGeo takeoff;
    FILE *fp;
    char *buf = NULL;
    long size;
    size_t len;
    size_t pos;
    int inspectTags = 0;
    int cruiseTags = 0;
    int inspectSegs = 0;
    size_t i;

    WT_CASE("KMZ 里 waypointSpeed 的分布与判据一致");

    WtMission_Init(&m);
    WT_CHECK_EQ_INT(WtPlan_BuildMission(&spec, &frame, &profile, &m), WT_PLAN_OK);

    takeoff = spec.base;
    takeoff.alt = spec.base.alt + 5.0;

    WtBridge_InitPlan(&plan);
    WT_CHECK_EQ_INT(WtBridge_BuildActions(&m, WT_GIMBAL_FREE_YAW, &plan), WT_PLAN_OK);
    WT_CHECK_EQ_INT(WtBridge_BuildFlightParams(&m, &takeoff, &profile, &plan), WT_PLAN_OK);
    WT_CHECK_EQ_INT(WtBridge_ExportKmz(&m, &plan, &profile, &takeoff, WT_TEST_KMZ_FILE),
                    WT_PLAN_OK);

    for (i = 1; i < m.count; i++) {
        if (WtPlan_SegmentUsesInspectSpeed(&m.points[i - 1], &m.points[i])) {
            inspectSegs++;
        }
    }

    /* KMZ 采用 store 方式（未压缩），可以按字节直接检索 */
    fp = fopen(WT_TEST_KMZ_FILE, "rb");
    if (fp == NULL) {
        WT_CHECK(0);
        goto done;
    }
    fseek(fp, 0, SEEK_END);
    size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    buf = (char *)malloc((size_t)size);
    if (buf == NULL || fread(buf, 1, (size_t)size, fp) != (size_t)size) {
        WT_CHECK(0);
        fclose(fp);
        goto done;
    }
    fclose(fp);
    len = (size_t)size;

    pos = 0;
    while (pos + 20 <= len) {
        const char *hit = FindBytes(buf + pos, len - pos, "<wpml:waypointSpeed>");

        if (hit == NULL) {
            break;
        }
        {
            double v = 0.0;

            if (sscanf(hit, "<wpml:waypointSpeed>%lf<", &v) == 1) {
                if (fabs(v - profile.inspectSpeedMs) < 0.01) {
                    inspectTags++;
                } else if (fabs(v - profile.cruiseSpeedMs) < 0.01) {
                    cruiseTags++;
                }
            }
        }
        pos = (size_t)(hit - buf) + 1;
    }

    /*
     * KMZ 里有两个 wpml 文档（template.kml 与 waylines.wpml），速度逐点各写
     * 一份，所以整包里的速度标签数是航点数的两倍。这个 2 是有意写死的：
     * 哪天导出结构变了，这里会立刻报出来。
     */
    WT_CHECK_EQ_INT(inspectTags, 2 * inspectSegs);
    WT_CHECK_EQ_INT(inspectTags + cruiseTags, 2 * (int)m.count);

done:
    free(buf);
    remove(WT_TEST_KMZ_FILE);
    WtBridge_FreePlan(&plan);
    WtMission_Free(&m);
}

static void TestActionCsvExport(void)
{
    WtTurbineSpec spec = WtTest_Spec155();
    WtInspectionProfile profile = WtInspectionProfile_Default();
    WtRotorFrame frame = WtTurbine_BuildRotorFrame(&spec, 0.0);
    WtMission m;
    WtMissionPlan plan;
    WtGeo takeoff;
    FILE *fp;
    char line[512];
    int rows = 0;

    WT_CASE("动作 CSV 可导出，行数与航点数一致");

    WtMission_Init(&m);
    WT_CHECK_EQ_INT(WtPlan_BuildMission(&spec, &frame, &profile, &m), WT_PLAN_OK);

    takeoff = spec.base;
    takeoff.alt = spec.base.alt + 5.0;

    WtBridge_InitPlan(&plan);
    WT_CHECK_EQ_INT(WtBridge_BuildActions(&m, WT_GIMBAL_FREE_YAW, &plan), WT_PLAN_OK);
    WT_CHECK_EQ_INT(WtBridge_BuildFlightParams(&m, &takeoff, &profile, &plan), WT_PLAN_OK);
    WT_CHECK_EQ_INT(WtBridge_ExportActionCsv(&m, &plan, WT_TEST_CSV_FILE), WT_PLAN_OK);

    fp = fopen(WT_TEST_CSV_FILE, "r");
    if (fp == NULL) {
        WT_CHECK(0);
    } else {
        while (fgets(line, sizeof(line), fp) != NULL) {
            rows++;
        }
        fclose(fp);
        WT_CHECK_EQ_INT(rows, (int)m.count + 1); /* 含表头 */
    }

    remove(WT_TEST_CSV_FILE);
    WtBridge_FreePlan(&plan);
    WtMission_Free(&m);
}

/* ------------------------------------------------------------------ */

int main(void)
{
    TestFlightParamsSpeedRule();
    TestStatsAndFlightParamsAgree();
    TestActionMapping();
    TestKmzSpeedTags();
    TestActionCsvExport();

    return WT_TEST_SUMMARY();
}