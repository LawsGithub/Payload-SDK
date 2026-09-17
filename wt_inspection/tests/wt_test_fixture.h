/**
 * @file wt_test_fixture.h
 * @brief 测试夹具：与 PC 自检程序同源的样例风机
 *
 * 这里刻意复制一份而不是做成 wt_core 的公共函数 —— 样例风机是测试与演示的
 * 素材，不属于产品库的接口。两处的默认值必须一致，改一处要同时改：
 *   本文件、tools/wt_plan_demo.c 的 DemoSpec()。
 */

#ifndef WT_TEST_FIXTURE_H
#define WT_TEST_FIXTURE_H

#include <string.h>

#include "wt_plan.h"
#include "wt_turbine.h"

/**
 * @brief 陆上大型机组：叶轮直径 155m、轮毂中心高 100m
 *
 * 选择这组参数是因为它代表陆上大型机组的主流规格（155m 叶轮 / 100m 轮毂），
 * 同时被 tools/wt_plan_demo.c 的 DemoSpec() 用作演示样例 —— 测试里的期望值
 * 可以直接与 `wt_plan_demo` 的输出对照。
 */
static WtTurbineSpec WtTest_Spec155(void)
{
    WtTurbineSpec s;

    memset(&s, 0, sizeof(s));
    s.base.lat = 41.5236000;
    s.base.lon = 111.7461000;
    s.base.alt = 1450.0;
    s.hubHeight = 100.0;
    s.rotorDiameter = 155.0;
    s.hubRadius = 2.5;
    s.towerBottomDia = 6.5;
    s.towerTopDia = 4.0;
    s.nacelleOffset = 4.5;
    s.coneAngleDeg = 4.0;
    s.tiltDeg = 5.0;
    s.headingDeg = 30.0;
    s.prebendM = 3.0;
    s.bladeCount = 3;

    return s;
}

/** @brief 叶轮直径 191m 的机组，用于验证点数上限对不同展向长度的作用 */
static WtTurbineSpec WtTest_Spec191(void) __attribute__((unused));

static WtTurbineSpec WtTest_Spec191(void)
{
    WtTurbineSpec s = WtTest_Spec155();

    s.rotorDiameter = 191.0;
    s.hubHeight = 110.0;

    return s;
}

/**
 * @brief 数值整洁的剖面：关掉粗模与塔筒，速度取整
 *
 * 用于人工构造航点、精确核算耗时的用例 —— 默认剖面里混着粗模环绕与塔筒，
 * 想断言「某一段该按哪个速度计费」会被无关航段干扰。
 */
static WtInspectionProfile WtTest_ProfileTiny(void)
{
    WtInspectionProfile p = WtInspectionProfile_Default();

    p.includeTower = false;
    p.doCoarseSurvey = false;
    p.cruiseSpeedMs = 8.0;
    p.inspectSpeedMs = 2.0;
    p.photoDwellSec = 1.5;

    return p;
}

/**
 * @brief 造一个用于计时断言的航点
 *
 * 只填计时与判据会用到的字段。距离由 enu 决定，速度由 bladeIndex / side /
 * purpose 三项决定。
 */
static WtPlanPoint WtTest_MakePoint(double e, double n, double u, int bladeIndex,
                                    double radialFrac, int side, WtPhotoPurpose purpose)
{
    WtPlanPoint p;

    memset(&p, 0, sizeof(p));
    p.enu.e = e;
    p.enu.n = n;
    p.enu.u = u;
    p.bladeIndex = bladeIndex;
    p.radialFrac = radialFrac;
    p.side = side;
    p.purpose = purpose;
    p.tag = "test";

    return p;
}

#endif /* WT_TEST_FIXTURE_H */