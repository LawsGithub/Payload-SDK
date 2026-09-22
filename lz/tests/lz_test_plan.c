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

    LZ_CASE("航点数夹取：边界与越界");
    {
        /* 夹取是**操作员输入的第一道过滤**（Pilot 输入框能敲任意整数）。
         * 它必须：区间内原样通过、越界夹到端点、非法小值抬到下限。
         * 注意 MIN 侧的行为与 MAX 侧同为"夹取" —— 0 和负数不是"拒绝"，
         * 而是抬到 3，因为两个点构不成圆这件事在夹取层表现为"下限"。 */
        LZ_CHECK(LzPlan_ClampWaypointCount(LZ_PLAN_WAYPOINT_MIN) == LZ_PLAN_WAYPOINT_MIN);
        LZ_CHECK(LzPlan_ClampWaypointCount(LZ_PLAN_WAYPOINT_MAX) == LZ_PLAN_WAYPOINT_MAX);
        LZ_CHECK(LzPlan_ClampWaypointCount(8) == 8);
        LZ_CHECK(LzPlan_ClampWaypointCount(16) == 16);
        LZ_CHECK(LzPlan_ClampWaypointCount(24) == 24);

        LZ_CHECK(LzPlan_ClampWaypointCount(LZ_PLAN_WAYPOINT_MAX + 1) == LZ_PLAN_WAYPOINT_MAX);
        LZ_CHECK(LzPlan_ClampWaypointCount(200) == LZ_PLAN_WAYPOINT_MAX);
        LZ_CHECK(LzPlan_ClampWaypointCount(100000) == LZ_PLAN_WAYPOINT_MAX);

        /* 输入框里敲 0 / 负数 / 1 / 2 —— 都抬到 MIN */
        LZ_CHECK(LzPlan_ClampWaypointCount(2) == LZ_PLAN_WAYPOINT_MIN);
        LZ_CHECK(LzPlan_ClampWaypointCount(1) == LZ_PLAN_WAYPOINT_MIN);
        LZ_CHECK(LzPlan_ClampWaypointCount(0) == LZ_PLAN_WAYPOINT_MIN);
        LZ_CHECK(LzPlan_ClampWaypointCount(-1) == LZ_PLAN_WAYPOINT_MIN);
        LZ_CHECK(LzPlan_ClampWaypointCount(-9999) == LZ_PLAN_WAYPOINT_MIN);

        /* 夹取必须是**幂等**的 —— 否则"读时再夹一次"那道防御会变成抖动源 */
        for (int v = -100; v <= 300; v += 7) {
            const int once = LzPlan_ClampWaypointCount(v);
            LZ_CHECK(LzPlan_ClampWaypointCount(once) == once);
        }
    }

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
        /* route.count = waypointCount + 1 —— 末尾那个是与首点重合的收尾点。
         * `waypointCount` 的语义是"圆周上均分几个方位"，不是"几个点"。 */
        LZ_CHECK(route.count == (size_t)pr.waypointCount + 1);

        for (size_t i = 0; i < route.count; ++i) {
            /* 到杆心的水平距离 —— 用 LzGeo_DistanceM（它算的是球面距离，
             * 高度不参与，正合此处所需） */
            LZ_CHECK_NEAR(LzGeo_DistanceM(&route.points[i].geo, &p.geo), pr.radiusM, 0.5);
            LZ_CHECK_NEAR(route.points[i].relativeAltM, pr.altitudeM, 1e-6);
        }

        LZ_CHECK(LzPlan_Validate(&route, &pr) == LZ_OK);
        LzRoute_Free(&route);
    }

    LZ_CASE("看向杆心的方位角必须指向杆心");
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
         * 飞机走的是相邻点之间的**直线**，所以总长是 n 段**弦**之和，
         * 而不是整圈周长 2πr。闭合之后段数正好是 n（含收尾段），
         * 但每段仍是弦 —— 写错的话 Pilot 显示的里程就是错的。 */
        LzTarget p = pole();
        LzGeo to = p.geo;
        LzOrbitProfile pr = profile();
        LzRoute route;
        LzRoute_Init(&route);
        LZ_CHECK(LzPlan_BuildOrbit(&p, &to, &pr, &route) == LZ_OK);

        const double stepRad = 2.0 * M_PI / pr.waypointCount;
        const double chord = 2.0 * pr.radiusM * sin(stepRad / 2.0);

        /* 除收尾段（末点→首点，长度应为 ~0）外，其余每段都等于弦长 */
        for (size_t i = 1; i + 1 < route.count; ++i) {
            LZ_CHECK_NEAR(LzGeo_DistanceM(&route.points[i - 1].geo,
                                          &route.points[i].geo), chord, 0.01);
        }
        /* 收尾段：末点与首点重合 —— 它不是一个"航段"，是同一个位置的重复点 */
        const double closing =
            LzGeo_DistanceM(&route.points[route.count - 1].geo,
                            &route.points[0].geo);
        LZ_CHECK(closing < 0.01);

        /* 总长应是 n 段弦，明显小于整圈周长（差 2πr - n·chord ≈ 7.6% @ n=8） */
        double total = 0.0;
        for (size_t i = 1; i < route.count; ++i) {
            total += LzGeo_DistanceM(&route.points[i - 1].geo,
                                     &route.points[i].geo);
        }
        LZ_CHECK_NEAR(total, chord * (double)pr.waypointCount, 0.05);
        LZ_CHECK(total < 2.0 * M_PI * pr.radiusM);

        LzRoute_Free(&route);
    }

    LZ_CASE("严格闭合：末点与首点坐标完全相同");
    {
        /* 用户 2026-09-22 明确要求"补一个与首点重合的收尾点"。
         *
         * 为什么收尾点必须是**独立的一个航点**：航线逐点执行，飞完最后
         * 一个"真实的"方位点就按 finishAction=goHome 走了 ——
         * 回起点那段弧不在航线里。补上它才真的闭合整圈。
         *
         * 顺带守一个容易忽略的语义：towardPOI 下，一个航点的朝向作用于
         * "飞向**下一个**航段"。收尾点让倒数第二个点的朝向有了归宿，
         * 否则最后半段机头会停在更早给的方向上、**收尾处朝错方向**。 */
        LzTarget p = pole();
        LzGeo to = p.geo;

        for (int dir = 0; dir < 2; ++dir) {
            for (int n = 3; n <= 16; ++n) {
                LzOrbitProfile pr = profile();
                pr.waypointCount = n;
                pr.clockwise = (dir == 0);
                LzRoute route;
                LzRoute_Init(&route);
                LZ_CHECK(LzPlan_BuildOrbit(&p, &to, &pr, &route) == LZ_OK);

                LZ_CHECK(route.count == (size_t)n + 1);
                const LzWaypoint *first = &route.points[0];
                const LzWaypoint *last = &route.points[route.count - 1];

                /* 经纬度必须**完全**相同 —— 收尾点由同一个
                 * LzGeo_Destination 公式生成，浮点结果应逐位一致 */
                LZ_CHECK(last->geo.latitudeDeg == first->geo.latitudeDeg);
                LZ_CHECK(last->geo.longitudeDeg == first->geo.longitudeDeg);
                /* 高度、速度、朝向也必须一致，否则闭合处会有一次跳变 */
                LZ_CHECK_NEAR(last->relativeAltM, first->relativeAltM, 1e-9);
                LZ_CHECK_NEAR(last->speedMs, first->speedMs, 1e-9);
                LZ_CHECK_ANGLE_NEAR(last->gimbalYawDeg, first->gimbalYawDeg, 1e-9);
                LZ_CHECK_NEAR(last->gimbalPitchDeg, first->gimbalPitchDeg, 1e-9);

                /* 所有点（含首尾）都必须在圆上 */
                for (size_t i = 0; i < route.count; ++i) {
                    LZ_CHECK_NEAR(LzGeo_DistanceM(&route.points[i].geo, &p.geo),
                                  pr.radiusM, 0.5);
                }
                LzRoute_Free(&route);
            }
        }
    }

    LZ_CASE("相邻方位角均匀，末点绕行整圈后回到起点方位");
    {
        LzTarget p = pole();
        LzGeo to = p.geo;
        LzOrbitProfile pr = profile();
        pr.waypointCount = 8;
        pr.clockwise = true;
        pr.startBearingDeg = 0.0;
        LzRoute route;
        LzRoute_Init(&route);
        LZ_CHECK(LzPlan_BuildOrbit(&p, &to, &pr, &route) == LZ_OK);

        /* 首点应在 startBearing 上、末点应回到同一方位（相差整 360°） */
        const double firstB = LzGeo_BearingDeg(&p.geo, &route.points[0].geo);
        const double lastB  = LzGeo_BearingDeg(&p.geo,
                                               &route.points[route.count - 1].geo);
        LZ_CHECK_ANGLE_NEAR(firstB, pr.startBearingDeg, 0.5);
        LZ_CHECK_ANGLE_NEAR(lastB, pr.startBearingDeg, 0.5);

        /* 相邻方位角差应恒为 360/n（含最后一段） */
        for (size_t i = 1; i < route.count; ++i) {
            const double b0 = LzGeo_BearingDeg(&p.geo, &route.points[i - 1].geo);
            const double b1 = LzGeo_BearingDeg(&p.geo, &route.points[i].geo);
            double delta = b1 - b0;
            while (delta < 0.0)    { delta += 360.0; }
            while (delta >= 360.0) { delta -= 360.0; }
            LZ_CHECK_NEAR(delta, 360.0 / pr.waypointCount, 0.5);
        }
        LzRoute_Free(&route);
    }

    LZ_CASE("收尾行为的前置条件：飞完最后一点即已在起点上");
    {
        /* finishAction = gotoFirstWaypoint 的含义是"完成后飞向航线起始点"。
         * 我们额外补了与首点**坐标重合**的收尾点，所以飞完最后一个点时
         * 飞机**已经**在 p0 上了 —— 本动作几乎立即结束。
         *
         * 这两件事合起来才成立：wpml 侧保证指令对（lz_test_wpml 有专门用例），
         * 这里保证指令**下有意义**。若哪天有人去掉收尾点，
         * gotoFirstWaypoint 就变成"再飞一段回 p0"—— 语义变了，
         * 而且那一段在航线之外、朝向不受控。本用例就是守这个。 */
        LzTarget p = pole();
        LzGeo to = p.geo;

        for (int n = 3; n <= 16; n += 1) {
            LzOrbitProfile pr = profile();
            pr.waypointCount = n;
            LzRoute route;
            LzRoute_Init(&route);
            LZ_CHECK(LzPlan_BuildOrbit(&p, &to, &pr, &route) == LZ_OK);

            /* 点数：n 个方位 + 1 个重合收尾点 */
            LZ_CHECK(route.count == (size_t)n + 1);

            /* 末点即起点 —— 飞机到达"起始点"时任务已自然走完 */
            const LzWaypoint *first = &route.points[0];
            const LzWaypoint *last = &route.points[route.count - 1];
            LZ_CHECK(LzGeo_DistanceM(&last->geo, &first->geo) < 0.01);
            LZ_CHECK(last->geo.latitudeDeg == first->geo.latitudeDeg);
            LZ_CHECK(last->geo.longitudeDeg == first->geo.longitudeDeg);
            /* 末点是**真实航点**，不是占位 —— 高度/速度必须和别处一样有效 */
            LZ_CHECK(last->relativeAltM > 0.0);
            LZ_CHECK(last->speedMs > 0.0);
            LZ_CHECK(isfinite(last->gimbalYawDeg));
            LZ_CHECK(isfinite(last->gimbalPitchDeg));

            /* 起始点必须在圆周上（不是误取成圆心或起飞点） */
            LZ_CHECK_NEAR(LzGeo_DistanceM(&first->geo, &p.geo), pr.radiusM, 0.5);

            LzRoute_Free(&route);
        }
    }

    LZ_CASE("提前转弯截距：由真实几何反算，满足规范两条约束");
    {
        /* 规范对 waypointTurnDampingDist 有两条硬约束：
         *   1. 取值域 (0, 航段最大长度]
         *   2. 段长必须 > 2×截距
         * 这两条都是相对于**航段长度**的。截距由 route 反算，
         * 因此任何半径/点数组合下都必须自动满足。 */
        LzTarget p = pole();
        LzGeo to = p.geo;

        for (double r = LZ_PLAN_RADIUS_MIN_M; r <= LZ_PLAN_RADIUS_MAX_M; r += 2.5) {
            for (int n = 3; n <= 64; n += 7) {
                LzOrbitProfile pr = profile();
                pr.radiusM = r;
                pr.waypointCount = n;
                LzRoute route;
                LzRoute_Init(&route);
                LZ_CHECK(LzPlan_BuildOrbit(&p, &to, &pr, &route) == LZ_OK);

                const double d = LzPlan_SuggestDampingM(&route);
                LZ_CHECK(d > 0.0);

                /* 逐段检查两条约束。跳过收尾段（长度 ~0，不是真航段）。 */
                double shortest = 1e9, longest = 0.0;
                for (size_t i = 1; i + 1 < route.count; ++i) {
                    const double seg = LzGeo_DistanceM(&route.points[i - 1].geo,
                                                       &route.points[i].geo);
                    if (seg < shortest) { shortest = seg; }
                    if (seg > longest)  { longest  = seg; }
                }
                /* 约束 1：截距 <= 最长段 */
                LZ_CHECK(d <= longest);
                /* 约束 2：最短段 > 2×截距（严格不等式） */
                LZ_CHECK(2.0 * d < shortest);
                LzRoute_Free(&route);
            }
        }

        /* 退化输入返回 0，而不是 NaN 或负数 */
        LZ_CHECK(LzPlan_SuggestDampingM(NULL) == 0.0);
        LzRoute empty;
        LzRoute_Init(&empty);
        LZ_CHECK(LzPlan_SuggestDampingM(&empty) == 0.0);
        LzRoute_Free(&empty);
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
