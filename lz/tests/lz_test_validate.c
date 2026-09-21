/**
 * @file lz_test_validate.c
 * @brief 安全包线的回归测试。
 *
 * ## 为什么单列一个文件
 *
 * `lz_test_plan.c` 测的是**几何算得对不对**（圆周位置、方位角方向、弦长），
 * 这里测的是**什么样的输入该被拒绝**。两类失败的原因完全不同：
 * 前者是算法错，后者是判据错 —— 报错指向混在一起会浪费时间。
 *
 * ## 这一组断言守的是什么
 *
 * 安全包线的上/下限（`lz_plan.h` 的 `LZ_PLAN_*`）原先只活在控件层
 * （`app/lz_widget.c` 的滑杆映射宏），`LzPlan_Validate` 看不到它们。
 * 也就是说：**控件能拨出来的值有上限，但校验层没有** ——
 * 换个调用点（探针、demo、将来的自动规划）就能构造出超限剖面并通过校验。
 *
 * 现在判据收在规划层，所以断言要覆盖三件事：
 *   1. 超上限 → 拒（且错误码是 `LZ_ERR_UNSAFE` 而非 `LZ_ERR_PARAM`）
 *   2. 边界值 → 通过（闭区间，20 m 是可用值不是越界值）
 *   3. 数值非法（0/NaN）→ `LZ_ERR_PARAM`，与"合法但危险"分开
 */

#include "lz_plan.h"
#include "lz_geo.h"
#include "lz_test.h"

#include <math.h>
#include <stdlib.h>

static LzGeo takeoff_geo(void)
{
    const LzGeo g = { .latitudeDeg = 30.5005, .longitudeDeg = 114.3003, .altitudeM = 30.0 };
    return g;
}

static LzTarget pole(void)
{
    LzTarget t = {
        .id = 1,
        .geo = { .latitudeDeg = 30.5005, .longitudeDeg = 114.3003, .altitudeM = 30.0 },
        .heightM = 15.0,
        .radiusM = 0.1,
        .confidence = 0.9,
    };
    return t;
}

/* 造一份"除了待测字段以外全部合法"的剖面。
 * 每个用例只改一个字段 —— 否则断言失败时不知道是哪个字段的问题。 */
static LzOrbitProfile base_profile(void)
{
    LzOrbitProfile p = {
        .radiusM = 15.0,
        .altitudeM = 50.0,
        .speedMs = 3.0,
        .waypointCount = 8,
        .startBearingDeg = 0.0,
        .clockwise = true,
        .gimbalPitchDeg = -15.0,
    };
    return p;
}

/** 按剖面建一条航线并校验；返回校验结果（建图失败则返回那个错误） */
static LzStatus validate_with(LzOrbitProfile pr)
{
    LzTarget p = pole();
    const LzGeo takeoff = takeoff_geo();
    LzRoute route;
    LzRoute_Init(&route);

    const LzStatus built = LzPlan_BuildOrbit(&p, &takeoff, &pr, &route);
    if (built != LZ_OK) {
        LzRoute_Free(&route);
        return built;
    }
    const LzStatus st = LzPlan_Validate(&route, &pr);
    LzRoute_Free(&route);
    return st;
}

int main(void)
{
    /* ---------- 半径 ---------- */

    LZ_CASE("半径：区间内通过");
    {
        LzOrbitProfile pr = base_profile();
        pr.radiusM = 15.0;
        LZ_CHECK(validate_with(pr) == LZ_OK);
    }

    LZ_CASE("半径：恰好等于上/下限应通过（闭区间）");
    {
        LzOrbitProfile pr = base_profile();

        pr.radiusM = LZ_PLAN_RADIUS_MAX_M;      /* 20.0 —— 场地约束的边界值 */
        LZ_CHECK(validate_with(pr) == LZ_OK);

        pr.radiusM = LZ_PLAN_RADIUS_MIN_M;      /* 5.0 */
        LZ_CHECK(validate_with(pr) == LZ_OK);
    }

    LZ_CASE("半径：超出上限判为不安全，而不是参数非法");
    {
        LzOrbitProfile pr = base_profile();
        pr.radiusM = LZ_PLAN_RADIUS_MAX_M + 1.0;   /* 21.0 */

        /* 与"数值非法"分开是刻意的：21 m 是个语法上完全正常的数，
         * 它越界是因为现场约束，不是编程错误。
         * 排查方向不同 —— 前者去查现场条件与控件设定，后者去查代码。 */
        LZ_CHECK(validate_with(pr) == LZ_ERR_UNSAFE);
    }

    LZ_CASE("半径：低于下限判为不安全");
    {
        LzOrbitProfile pr = base_profile();
        pr.radiusM = LZ_PLAN_RADIUS_MIN_M - 1.0;   /* 4.0 */
        LZ_CHECK(validate_with(pr) == LZ_ERR_UNSAFE);
    }

    LZ_CASE("半径：0 与负数仍是参数非法（比越界更早一层）");
    {
        LzOrbitProfile pr = base_profile();

        /* 0 / 负数在 BuildOrbit 就被拦下，详见 lz_plan.c 的入参校验 */
        pr.radiusM = 0.0;
        LZ_CHECK(validate_with(pr) == LZ_ERR_PARAM);
        pr.radiusM = -5.0;
        LZ_CHECK(validate_with(pr) == LZ_ERR_PARAM);
    }

    /* ---------- 高度 ---------- */

    LZ_CASE("高度：区间内通过");
    {
        LzOrbitProfile pr = base_profile();
        pr.altitudeM = 80.0;
        LZ_CHECK(validate_with(pr) == LZ_OK);
    }

    LZ_CASE("高度：恰好等于上/下限应通过（闭区间）");
    {
        LzOrbitProfile pr = base_profile();

        pr.altitudeM = LZ_PLAN_ALTITUDE_MAX_M;   /* 120.0 */
        LZ_CHECK(validate_with(pr) == LZ_OK);

        pr.altitudeM = LZ_PLAN_ALTITUDE_MIN_M;   /* 5.0 */
        LZ_CHECK(validate_with(pr) == LZ_OK);
    }

    LZ_CASE("高度：超出上限判为不安全");
    {
        LzOrbitProfile pr = base_profile();
        pr.altitudeM = LZ_PLAN_ALTITUDE_MAX_M + 1.0;   /* 121.0 */
        LZ_CHECK(validate_with(pr) == LZ_ERR_UNSAFE);
    }

    LZ_CASE("高度：低于下限判为不安全");
    {
        LzOrbitProfile pr = base_profile();
        pr.altitudeM = LZ_PLAN_ALTITUDE_MIN_M - 1.0;   /* 4.0 */
        LZ_CHECK(validate_with(pr) == LZ_ERR_UNSAFE);
    }

    LZ_CASE("高度：非正仍是参数非法或更早被拦下");
    {
        LzOrbitProfile pr = base_profile();
        pr.altitudeM = 0.0;
        /* BuildOrbit 不校验高度符号，交由 Validate 判 —— 无论哪一层拦下，
         * 都**不能**是 LZ_OK。这里只断言"被拒"，不锁定具体码，
         * 因为两层都可能拒它，锁定会让断言与实现细节耦合。 */
        LZ_CHECK(validate_with(pr) != LZ_OK);

        pr.altitudeM = -1.0;
        LZ_CHECK(validate_with(pr) != LZ_OK);
    }

    LZ_CASE("高度：NaN 属参数非法，不属不安全");
    {
        LzOrbitProfile pr = base_profile();
        pr.altitudeM = NAN;
        /* NaN 参与任何比较都是 false —— 只写范围判断会把它悄悄放过去。
         * lz_plan.c 里先做 isfinite 正是为了挡它。 */
        LZ_CHECK(validate_with(pr) == LZ_ERR_PARAM);

        pr = base_profile();
        pr.radiusM = NAN;
        LZ_CHECK(validate_with(pr) == LZ_ERR_PARAM);
    }

    /* ---------- 逐点高度也要受包线约束 ---------- */

    LZ_CASE("逐点高度越限：即使剖面本身合法也要拒");
    {
        LzTarget p = pole();
        const LzGeo takeoff = takeoff_geo();
        LzOrbitProfile pr = base_profile();

        LzRoute route;
        LzRoute_Init(&route);
        LZ_CHECK(LzPlan_BuildOrbit(&p, &takeoff, &pr, &route) == LZ_OK);
        LZ_CHECK(LzPlan_Validate(&route, &pr) == LZ_OK);

        /* 手改一个航点的高度到上限之外 —— 模拟"航线不是这个剖面生成的"。
         * 校验的对象是**航线内容**，不是"生成它的那份剖面"：
         * route 由调用方给，探针 / demo / 将来的自动规划都能构造它。 */
        if (route.count > 0) {
            route.points[0].relativeAltM = LZ_PLAN_ALTITUDE_MAX_M + 10.0;
            LZ_CHECK(LzPlan_Validate(&route, &pr) == LZ_ERR_UNSAFE);

            route.points[0].relativeAltM = 0.5;
            LZ_CHECK(LzPlan_Validate(&route, &pr) == LZ_ERR_UNSAFE);

            route.points[0].relativeAltM = 50.0;   /* 改回合法值 */
            LZ_CHECK(LzPlan_Validate(&route, &pr) == LZ_OK);
        }
        LzRoute_Free(&route);
    }

    /* ---------- 与控件层的一致性 ---------- */

    LZ_CASE("包线常量本身是自洽的");
    {
        /* 下限必须小于上限，否则任何值都非法 —— 这类"常量改坏了"的
         * 错误不会有编译错误，只会让所有航线都飞不起来。 */
        LZ_CHECK(LZ_PLAN_RADIUS_MIN_M < LZ_PLAN_RADIUS_MAX_M);
        LZ_CHECK(LZ_PLAN_ALTITUDE_MIN_M < LZ_PLAN_ALTITUDE_MAX_M);

        /* 都是正数（高度是相对起飞点，为负意味着在地下） */
        LZ_CHECK(LZ_PLAN_RADIUS_MIN_M > 0.0);
        LZ_CHECK(LZ_PLAN_ALTITUDE_MIN_M > 0.0);
    }

    /* ---------- 已有的规则不能被新规则盖掉 ---------- */

    LZ_CASE("回归：云台俯仰与速度的旧规则仍然生效");
    {
        LZ_CASE("旧规则：云台俯仰超限判为越界");
        {
            LzTarget p = pole();
            const LzGeo takeoff = takeoff_geo();
            LzOrbitProfile pr = base_profile();

            LzRoute route;
            LzRoute_Init(&route);
            LZ_CHECK(LzPlan_BuildOrbit(&p, &takeoff, &pr, &route) == LZ_OK);

            if (route.count > 0) {
                route.points[0].gimbalPitchDeg = -120.0;   /* 物理限位 [-90, 30] */
                LZ_CHECK(LzPlan_Validate(&route, &pr) == LZ_ERR_RANGE);

                route.points[0].gimbalPitchDeg = -15.0;
                route.points[0].speedMs = 0.0;
                LZ_CHECK(LzPlan_Validate(&route, &pr) == LZ_ERR_RANGE);

                route.points[0].speedMs = 3.0;
                LZ_CHECK(LzPlan_Validate(&route, &pr) == LZ_OK);
            }
            LzRoute_Free(&route);
        }
    }

    return LZ_TEST_SUMMARY();
}
