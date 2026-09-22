/**
 * @file lz_plan.c
 * @brief 规划层实现。
 *
 * 本文件是整个工程的**算法核心**：输入是视觉看到的几个目标，输出是一条
 * 可执行的航线。它不碰 PSDK、不碰 OpenCV，因此可以在桌面上用写死的
 * 目标数组反复跑（见 tests/lz_test_plan.c）。
 */

#include "lz_plan.h"
#include "lz_geo.h"

#include <math.h>
#include <stdlib.h>

#define LZ_ROUTE_INIT_CAP 32

void LzRoute_Init(LzRoute *route)
{
    if (route == NULL) {
        return;
    }
    route->points = NULL;
    route->count = 0;
    route->capacity = 0;
}

void LzRoute_Free(LzRoute *route)
{
    if (route == NULL) {
        return;
    }
    free(route->points);
    route->points = NULL;
    route->count = 0;
    route->capacity = 0;
}

static LzStatus lz_route_push(LzRoute *route, const LzWaypoint *wp)
{
    if (route->count == route->capacity) {
        size_t newCap = (route->capacity == 0) ? LZ_ROUTE_INIT_CAP : route->capacity * 2;
        LzWaypoint *grown = realloc(route->points, newCap * sizeof(*grown));
        if (grown == NULL) {
            return LZ_ERR_IO;
        }
        route->points = grown;
        route->capacity = newCap;
    }
    route->points[route->count++] = *wp;
    return LZ_OK;
}

/* ===========================================================================
 * 绕飞圆周的航点生成
 *
 * ## 几何：三种"方位角"不要混
 *
 *   stationBearing   从**杆**出发指向站点的方位角 —— 决定站点在圆的哪一点
 *   dir              绕行方向对 bearing 的符号
 *   gimbalYawDeg     从**站点**出发指向杆心的方位角 —— 决定机头/云台朝哪
 *                    （M4T 上机头承担这个偏转，见 lz_wpml.c 文件头）
 *
 * stationBearing 与 gimbalYawDeg 相差 180°，但**不要用 +180 去算后者**：
 * 直接用 LzGeo_BearingDeg(站点, 杆心) 反算。两者数学上等价，但反算少一处
 * mod 360、且万一将来圆心不是 target->geo 也不会错。
 *
 * ## 方向符号：靠推理定，然后靠测试守
 *
 * 方位角的约定是**正北 0°、顺时针为正**（0=N, 90=E, 180=S, 270=W）。
 * 站点由 LzGeo_Destination(杆, stationBearing, r) 得到，所以：
 *
 *     stationBearing = 0   → 站点在杆的**正北**
 *     stationBearing = 90  → 站点在杆的**正东**
 *
 * 把地图摊开（北在上、东在右），站点从"上"移到"右"再移到"下"再移到"左"，
 * 即 N→E→S→W —— 这**俯视看就是顺时针**。
 *
 * 所以：**stationBearing 递增 = 俯视顺时针**。
 *     clockwise  → bearing 递增（dir = +1）
 *     counter    → bearing 递减（dir = -1）
 *
 * 这类"符号反了但看起来完全对"的错误，肉眼抓不住 ——
 * 所以 tests/lz_test_plan.c 里专门有一条断言守它
 * （"顺逆时针的航点顺序应互为逆序"）。
 *
 * ## 关于闭合
 *
 * 本函数生成 waypointCount 个点，均分 360°。飞机依次飞过它们，
 * 覆盖的是 (waypointCount-1) × (360/waypointCount) 度 —— **差一段没绕完**，
 * 随后按 finishAction=goHome 返航。若要严格闭合整圈，
 * 需要额外补一个与首点重合的收尾点（见 README 的说明）。
 * =========================================================================== */

LzStatus LzPlan_BuildOrbit(const LzTarget *target,
                           const LzGeo *takeoff,
                           const LzOrbitProfile *profile,
                           LzRoute *route)
{
    if (target == NULL || takeoff == NULL || profile == NULL || route == NULL) {
        return LZ_ERR_PARAM;
    }
    /* 位置非法或置信度不足 ⇒ 圆心不可信，整个圆都不可信，直接拒绝 */
    if (!LzTarget_IsUsable(target, 0.0)) {
        return LZ_ERR_NO_TARGET;
    }
    /* 少于 3 个点构不成圆 */
    if (profile->waypointCount < 3) {
        return LZ_ERR_PARAM;
    }
    if (!(profile->radiusM > 0.0) || !isfinite(profile->radiusM) ||
        !isfinite(profile->startBearingDeg) || !isfinite(profile->altitudeM) ||
        !isfinite(profile->speedMs)) {
        return LZ_ERR_PARAM;
    }

    const double stepDeg = 360.0 / (double)profile->waypointCount;
    const double dir = profile->clockwise ? 1.0 : -1.0;

    for (int i = 0; i < profile->waypointCount; ++i) {
        const double stationBearing =
            LzGeo_NormalizeDeg(profile->startBearingDeg + dir * stepDeg * (double)i);

        LzWaypoint wp;
        LzStatus st = LzGeo_Destination(&target->geo, stationBearing,
                                        profile->radiusM, &wp.geo);
        if (st != LZ_OK) {
            return st;
        }

        wp.relativeAltM = profile->altitudeM;
        wp.speedMs = profile->speedMs;
        /* 看向杆心的方位角 —— 绕飞的全部意义就在这一行。
         * 它同时是云台 yaw 与机头目标角（M4T 的 towardPOI 要求两者一致）。 */
        wp.gimbalYawDeg = LzGeo_BearingDeg(&wp.geo, &target->geo);
        wp.gimbalPitchDeg = profile->gimbalPitchDeg;

        st = lz_route_push(route, &wp);
        if (st != LZ_OK) {
            return st;
        }
    }

    return LZ_OK;
}

/* ===========================================================================
 * 安全校验：规则相对固定，与上面的生成策略分开演进
 * =========================================================================== */

LzStatus LzPlan_Validate(const LzRoute *route, const LzOrbitProfile *profile)
{
    if (route == NULL || profile == NULL) {
        return LZ_ERR_PARAM;
    }
    if (route->count == 0) {
        return LZ_ERR_NO_TARGET;
    }
    if (profile->radiusM <= 0.0 || profile->waypointCount < 3) {
        return LZ_ERR_PARAM;
    }
    if (!isfinite(profile->radiusM) || !isfinite(profile->altitudeM)) {
        return LZ_ERR_PARAM;
    }

    /* ---- 安全包线：合法但危险的数值 ----
     *
     * 与上面的 `<= 0` / 非有限数**分开判**，且返回**不同的**错误码：
     *   `LZ_ERR_PARAM`  = 数值本身非法（0、负数、NaN）—— 是编程错误
     *   `LZ_ERR_UNSAFE` = 数值合法但超出安全范围 —— 是**操作员可修正**的输入
     * 两者排查方向完全不同：前者去查代码，后者去查现场条件与控件设定。
     *
     * 边界取**闭区间**（恰好等于上限算通过）—— 上限本身就是从现场条件
     * 推出来的可用值，没有理由把"正好 20 m"判为越界。
     *
     * 上限只认 lz_plan.h 的 `LZ_PLAN_*`。**不再接受调用方自带的上限**：
     * 原先这些边界只活在控件层（`app/lz_widget.c` 的滑杆映射宏），
     * 规划层看不见 —— 换个调用点（探针、demo、将来的自动规划）就能绕过，
     * 而 `LzPlan_Validate` 是起飞前最后一道关卡，它的判据不该取决于谁在调用。 */
    if (profile->radiusM < LZ_PLAN_RADIUS_MIN_M ||
        profile->radiusM > LZ_PLAN_RADIUS_MAX_M) {
        return LZ_ERR_UNSAFE;
    }
    if (profile->altitudeM < LZ_PLAN_ALTITUDE_MIN_M ||
        profile->altitudeM > LZ_PLAN_ALTITUDE_MAX_M) {
        return LZ_ERR_UNSAFE;
    }

    for (size_t i = 0; i < route->count; ++i) {
        const LzWaypoint *wp = &route->points[i];

        if (!LzGeo_IsValid(&wp->geo)) {
            return LZ_ERR_PARAM;
        }
        if (!isfinite(wp->relativeAltM) || !isfinite(wp->speedMs) ||
            !isfinite(wp->gimbalYawDeg) || !isfinite(wp->gimbalPitchDeg)) {
            return LZ_ERR_PARAM;
        }
        /* 高度必须为正 —— 相对起飞点的高度为负意味着在地下。
         * 不做自动抬升：自动"修正"会让航线悄悄偏离操作员的意图，
         * 现场更难判断发生了什么。 */
        if (wp->relativeAltM <= 0.0) {
            return LZ_ERR_UNSAFE;
        }
        /* 逐点也受安全包线约束。
         * 为什么不只靠上面的 profile 检查：`route` 是调用方给的，
         * 不一定出自 `LzPlan_BuildOrbit`（探针、demo、将来的自动规划都可能
         * 构造它）。校验的对象是**航线内容**，不是"生成它的那份剖面"。 */
        if (wp->relativeAltM < LZ_PLAN_ALTITUDE_MIN_M ||
            wp->relativeAltM > LZ_PLAN_ALTITUDE_MAX_M) {
            return LZ_ERR_UNSAFE;
        }
        if (wp->speedMs <= 0.0) {
            return LZ_ERR_RANGE;
        }
        /* 云台俯仰的物理限位，超出范围的指令会被飞机拒绝执行 */
        if (wp->gimbalPitchDeg < -90.0 || wp->gimbalPitchDeg > 30.0) {
            return LZ_ERR_RANGE;
        }
    }

    return LZ_OK;
}
