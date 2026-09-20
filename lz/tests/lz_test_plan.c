/**
 * @file lz_test_plan.c
 * @brief 绕飞规划的回归测试。
 *
 * 这些用例是绕飞几何的**规格说明**，比注释可靠：
 * 尤其是方位角递增方向与绕行方向的对应关系，肉眼极难看出对错 ——
 * 这里做了逐点断言，另有独立程序用多边形有向面积（鞋带公式）复核。
 */

#include "lz_plan.h"
#include "lz_geo.h"
#include "lz_test.h"

#include <stdlib.h>

static LzTarget pole(void)
{
    LzTarget t = {
        .id = 1,
        .geo = { .latitudeDeg = 30.500500, .longitudeDeg = 114.300300, .altitudeM = 30.0 },
        .heightM = 15.0,
        .radiusM = 0.1,
        .pixel = { .u = 0.5, .v = 0.6, .topV = 0.2, .bottomV = 0.6 },
        .confidence = 0.92,
    };
    return t;
}

static LzOrbitProfile profile(void)
{
    LzOrbitProfile p = {
        .radiusM = 20.0,
        .altitudeM = 12.0,
        .speedMs = 3.0,
        .waypointCount = 8,
        .startBearingDeg = 0.0,
        .clockwise = true,
        .gimbalPitchDeg = -15.0,
    };
    return p;
}

int main(void)
{
    /* ---------------- 这部分现在就该全绿 ---------------- */

    LZ_CASE("空入参必须被拒绝");
    {
        LzTarget p = pole();
        LzGeo to = p.geo;
        LzOrbitProfile pr = profile();
        LzRoute route;
        LzRoute_Init(&route);

        LZ_CHECK(LzPlan_BuildOrbit(NULL, &to, &pr, &route) == LZ_ERR_PARAM);
        LZ_CHECK(LzPlan_BuildOrbit(&p, NULL, &pr, &route) == LZ_ERR_PARAM);
        LZ_CHECK(LzPlan_BuildOrbit(&p, &to, NULL, &route) == LZ_ERR_PARAM);
        LZ_CHECK(LzPlan_BuildOrbit(&p, &to, &pr, NULL) == LZ_ERR_PARAM);
        LzRoute_Free(&route);
    }

    LZ_CASE("位置非法的目标判为无目标");
    {
        LzTarget bad = pole();
        bad.geo.latitudeDeg = NAN;
        LzGeo to = pole().geo;
        LzOrbitProfile pr = profile();
        LzRoute route;
        LzRoute_Init(&route);
        LZ_CHECK(LzPlan_BuildOrbit(&bad, &to, &pr, &route) == LZ_ERR_NO_TARGET);
        LzRoute_Free(&route);
    }

    LZ_CASE("航点数少于 3 个无法构成圆周");
    {
        LzTarget p = pole();
        LzGeo to = p.geo;
        LzOrbitProfile pr = profile();
        LzRoute route;
        LzRoute_Init(&route);

        pr.waypointCount = 2;
        LZ_CHECK(LzPlan_BuildOrbit(&p, &to, &pr, &route) == LZ_ERR_PARAM);
        pr.waypointCount = 4;
        /* 4 个是合法的（入参校验不该拦它）—— 结果可能是 UNSUPPORTED，
         * 但不该再是 PARAM */
        LZ_CHECK(LzPlan_BuildOrbit(&p, &to, &pr, &route) != LZ_ERR_PARAM);

        LzRoute_Free(&route);
    }

    LZ_CASE("Validate：空航线判为无目标");
    {
        LzRoute route;
        LzRoute_Init(&route);
        LzOrbitProfile pr = profile();
        LZ_CHECK(LzPlan_Validate(&route, &pr) == LZ_ERR_NO_TARGET);
        LzRoute_Free(&route);
    }

    LZ_CASE("Validate：高度非正判为不安全，云台超限判为越界");
    {
        LzRoute route;
        LzRoute_Init(&route);
        LzOrbitProfile pr = profile();

        route.points = malloc(sizeof(LzWaypoint));
        LZ_CHECK(route.points != NULL);
        route.points[0] = (LzWaypoint){
            .geo = { .latitudeDeg = 30.5, .longitudeDeg = 114.3, .altitudeM = 30.0 },
            .relativeAltM = 12.0,
            .speedMs = 3.0,
            .gimbalYawDeg = 180.0,
            .gimbalPitchDeg = -15.0,
        };
        route.count = 1;
        route.capacity = 1;

        LZ_CHECK(LzPlan_Validate(&route, &pr) == LZ_OK);

        route.points[0].relativeAltM = 0.0;
        LZ_CHECK(LzPlan_Validate(&route, &pr) == LZ_ERR_UNSAFE);

        route.points[0].relativeAltM = 12.0;
        route.points[0].gimbalPitchDeg = -120.0;
        LZ_CHECK(LzPlan_Validate(&route, &pr) == LZ_ERR_RANGE);

        route.points[0].gimbalPitchDeg = -15.0;
        route.points[0].speedMs = 0.0;
        LZ_CHECK(LzPlan_Validate(&route, &pr) == LZ_ERR_RANGE);

        LzRoute_Free(&route);
    }

    LZ_CASE("目标可用性：位置合法即可用，杆高半径允许未知");
    {
        LzTarget t = pole();
        LZ_CHECK(LzTarget_IsUsable(&t, 0.5));

        t.confidence = 0.3;
        LZ_CHECK(!LzTarget_IsUsable(&t, 0.5));

        t.confidence = 0.9;
        t.heightM = -1.0;    /* 未知 —— 仍可用于绕飞 */
        t.radiusM = -1.0;
        LZ_CHECK(LzTarget_IsUsable(&t, 0.5));

        t.geo.latitudeDeg = 999.0;  /* 位置非法 —— 不可用 */
        LZ_CHECK(!LzTarget_IsUsable(&t, 0.5));

        LZ_CHECK(!LzTarget_IsUsable(NULL, 0.5));
    }

    /* ---------------- 以下是绕飞的规格用例 ---------------- */

#if 1
    LZ_CASE("航点数与半径正确");
    {
        LzTarget p = pole();
        LzGeo to = p.geo;
        LzOrbitProfile pr = profile();
        LzRoute route;
        LzRoute_Init(&route);

        LZ_CHECK(LzPlan_BuildOrbit(&p, &to, &pr, &route) == LZ_OK);
        LZ_CHECK(route.count == (size_t)pr.waypointCount);

        for (size_t i = 0; i < route.count; ++i) {
            /* 到杆心的水平距离 —— 用 LzGeo_DistanceM（它算的是球面距离，
             * 高度不参与，正合此处所需） */
            LZ_CHECK_NEAR(LzGeo_DistanceM(&route.points[i].geo, &p.geo), pr.radiusM, 0.5);
            LZ_CHECK_NEAR(route.points[i].relativeAltM, pr.altitudeM, 1e-6);
        }

        LZ_CHECK(LzPlan_Validate(&route, &pr) == LZ_OK);
        LzRoute_Free(&route);
    }

    LZ_CASE("云台偏航必须指向杆心");
    {
        LzTarget p = pole();
        LzGeo to = p.geo;
        LzOrbitProfile pr = profile();
        LzRoute route;
        LzRoute_Init(&route);

        LZ_CHECK(LzPlan_BuildOrbit(&p, &to, &pr, &route) == LZ_OK);

        for (size_t i = 0; i < route.count; ++i) {
            /* 从航点看杆心的方位角，应与下发的云台偏航一致 */
            double toPole = LzGeo_BearingDeg(&route.points[i].geo, &p.geo);
            LZ_CHECK_ANGLE_NEAR(toPole, route.points[i].gimbalYawDeg, 0.5);
        }

        LzRoute_Free(&route);
    }

    LZ_CASE("相邻航点间距是弦长，不是弧长");
    {
        /* 这条守的是 wpml 里 distance/duration 的计算前提：
         * 飞机飞的是相邻点之间的直线，所以总长是 (n-1) 段**弦**之和，
         * 而不是整圈周长 2πr。两者差了 17%，写错了 Pilot 显示的里程就是错的。 */
        LzTarget p = pole();
        LzGeo to = p.geo;
        LzOrbitProfile pr = profile();
        LzRoute route;
        LzRoute_Init(&route);
        LZ_CHECK(LzPlan_BuildOrbit(&p, &to, &pr, &route) == LZ_OK);

        const double stepRad = 2.0 * M_PI / pr.waypointCount;
        const double chord = 2.0 * pr.radiusM * sin(stepRad / 2.0);

        for (size_t i = 1; i < route.count; ++i) {
            LZ_CHECK_NEAR(LzGeo_DistanceM(&route.points[i - 1].geo,
                                          &route.points[i].geo), chord, 0.01);
        }
        /* 也顺带确认它确实不是整圈周长 */
        LZ_CHECK(fabs(chord - 2.0 * M_PI * pr.radiusM / (pr.waypointCount - 1)) > 1.0);

        LzRoute_Free(&route);
    }

    LZ_CASE("顺逆时针的航点顺序应互为逆序");
    {
        LzTarget p = pole();
        LzGeo to = p.geo;
        LzOrbitProfile pr = profile();
        LzRoute cw, ccw;
        LzRoute_Init(&cw);
        LzRoute_Init(&ccw);

        pr.clockwise = true;
        LZ_CHECK(LzPlan_BuildOrbit(&p, &to, &pr, &cw) == LZ_OK);
        pr.clockwise = false;
        LZ_CHECK(LzPlan_BuildOrbit(&p, &to, &pr, &ccw) == LZ_OK);

        LZ_CHECK(cw.count == ccw.count);
        if (cw.count == ccw.count && cw.count > 2) {
            /* 首点都从 startBearing 出发，方向相反。
             *
             * 逐点断言方位角的**递增/递减**，而不是只看第二个点不同 ——
             * 后者在"符号反了"时也可能碰巧通过。约定：方位角正北 0°、
             * 顺时针为正，所以"站点方位角递增"= 俯视顺时针。 */
            const double step = 360.0 / pr.waypointCount;
            for (size_t i = 0; i < cw.count; ++i) {
                double bcw = LzGeo_BearingDeg(&p.geo, &cw.points[i].geo);
                double bccw = LzGeo_BearingDeg(&p.geo, &ccw.points[i].geo);
                double expectCw = LzGeo_NormalizeDeg(pr.startBearingDeg + step * (double)i);
                double expectCcw = LzGeo_NormalizeDeg(pr.startBearingDeg - step * (double)i);
                LZ_CHECK_ANGLE_NEAR(bcw, expectCw, 0.5);
                LZ_CHECK_ANGLE_NEAR(bccw, expectCcw, 0.5);
            }
        }

        LzRoute_Free(&cw);
        LzRoute_Free(&ccw);
    }
#endif

    return LZ_TEST_SUMMARY();
}
