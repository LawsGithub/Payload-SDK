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
 * ## 关于闭合：末尾补一个与首点重合的收尾点
 *
 * 本函数生成 `waypointCount + 1` 个点：先按 `waypointCount` 均分 360°，
 * 再补一个与首点**坐标完全相同**的收尾点。
 *
 *     输入 waypointCount = 4        →  route.count = 5
 *     p0(0°) p1(90°) p2(180°) p3(270°) p4 == p0
 *
 * 为什么必须是**独立的一个点**，而不能靠"飞完最后一段自动回到起点"：
 * 航线是逐点执行的，飞机飞完 p3 就按 `finishAction=goHome` 走了 ——
 * 从 p3 回 p0 的那段弧**根本不在航线里**。补上 p4 才真正闭合。
 *
 * ## 补上之后，有个副产品：方向语义变干净了
 *
 * 收尾点带的是**从 p4 看向杆心**的方位角，而 p4 与 p0 同点，
 * 所以它等于 p0 的方位角。于是航线首尾的机头方向一致，闭合处不会
 * 出现一次额外的甩头。
 *
 * ⚠️ 更要紧的是另一件事：**这个值同时也定义了 p3→p4 航段的机头方向**。
 * `towardPOI` 语义下，一个航点的朝向作用于"飞向**下一个**航段"；
 * 若不补收尾点，最后一个点 p_{n-1} 的朝向作用在任何航段上都是无意义的
 * （后面没有航段了），而 p_{n-1}→p0 那段又不存在 —— 于是机头在最后
 * 半段会保持在 p_{n-2} 给的方向上，**收尾处朝错方向**。
 * 补上重合点后，p_{n-1} 的朝向有了归宿（指向 p_{n-1}→p0 那段的杆心方向）。
 *
 * ## 几何精度
 *
 * 收尾点用 `stationBearing = startBearingDeg + dir*360` 生成，
 * 与首点走的都是 `LzGeo_Destination` 同一个公式，得到**同一坐标**
 * （球面公式对 ±360° 缩回同一方位角）。实测两者距离 ~1e-8 m。
 * =========================================================================== */

/* ===========================================================================
 * 提前转弯截距的几何反算
 *
 * 规范对 `wpml:waypointTurnDampingDist` 的两条硬约束都是**相对于航段长度**的
 * （取值域 `(0, 航段最大长度]`、段长必须 > 2×截距），而航段长度由半径与
 * 航点数决定。与其要求调用方自己算，不如从**真实航线**量出来 ——
 * 这样不可能与实际几何不一致。
 *
 * 取最短航段的 45%：留 10% 余量，避开"段长 > 2×截距"那条严格不等式的边界。
 * 取最短段而非平均段，因为约束里的"必需大于"是逐段的。
 * =========================================================================== */
double LzPlan_SuggestDampingM(const LzRoute *route)
{
    if (route == NULL || route->count < 2) {
        return 0.0;
    }
    double shortest = -1.0;
    for (size_t i = 1; i < route->count; ++i) {
        const double d = LzGeo_DistanceM(&route->points[i - 1].geo,
                                         &route->points[i].geo);
        if (d <= 0.0) {
            continue;    /* 收尾点与首点重合会产生 0 长段？不会 —— 见下 */
        }
        if (shortest < 0.0 || d < shortest) {
            shortest = d;
        }
    }
    if (shortest <= 0.0) {
        return 0.0;
    }
    return shortest * 0.45;
}

int LzPlan_ClampWaypointCount(int count)
{
    if (count < LZ_PLAN_WAYPOINT_MIN) {
        return LZ_PLAN_WAYPOINT_MIN;
    }
    if (count > LZ_PLAN_WAYPOINT_MAX) {
        return LZ_PLAN_WAYPOINT_MAX;
    }
    return count;
}

/* 相机的物理俯仰限位。与 `LzPlan_Validate` 里那道检查**同源** ——
 * 校验层按它拒、这里按它钳，两处写的必须是同一对数。 */
#define LZ_GIMBAL_PITCH_MIN_DEG (-90.0)
#define LZ_GIMBAL_PITCH_MAX_DEG (30.0)

double LzPlan_ComputeGimbalPitchDeg(double radiusM, double altAboveTargetM,
                                    double targetHeightM)
{
    if (!(radiusM > 0.0) || !isfinite(radiusM) || !isfinite(altAboveTargetM)) {
        return 0.0;
    }
    /* 高度未知按 0 处理（瞄准目标底部所在那一层水平面）—— 不猜典型杆高，
     * 理由见头文件。 */
    const double h = (isfinite(targetHeightM) && targetHeightM > 0.0) ? targetHeightM : 0.0;

    /* 瞄目标**中点**：竖直方向上的居中，与机头瞄杆心（水平方向居中）对称。
     * atan2 的参数次序是 (纵向差, 横向) —— 写反了得到的是余角，
     * 而余角在 45° 附近看起来"差不多"，正是那种改错了不容易发现的地方。 */
    return -atan2(altAboveTargetM - h * 0.5, radiusM) * 180.0 / M_PI;
}

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
    /* 少于 3 个点构不成圆 —— 几何必然，不是可调的包线 */
    if (profile->waypointCount < LZ_PLAN_WAYPOINT_MIN) {
        return LZ_ERR_PARAM;
    }
    if (!(profile->radiusM > 0.0) || !isfinite(profile->radiusM) ||
        !isfinite(profile->startBearingDeg) || !isfinite(profile->altitudeM) ||
        !isfinite(profile->speedMs)) {
        return LZ_ERR_PARAM;
    }

    const double stepDeg = 360.0 / (double)profile->waypointCount;
    const double dir = profile->clockwise ? 1.0 : -1.0;

    /* 云台俯仰：要么几何反算，要么原样用操作员给的常数。
     *
     * 反算需要"飞机相对目标底部的高度"，而剖面里的 altitudeM 是**相对起飞点**
     * 的 —— 两者差一个椭球高差。用 takeoff 与 target 的椭球高补齐：
     *
     *     相对目标底 = (起飞点椭球高 + 相对起飞点高度) - 目标椭球高
     *
     * 这一步**只在这一处**做，别让每个调用方各自补 —— 忘了补的表现是
     * 相机俯仰整体偏掉，而"偏一点"在画面上看不出来。 */
    const double altAboveTargetM =
        (takeoff->altitudeM + profile->altitudeM) - target->geo.altitudeM;

    double pitchDeg = profile->gimbalPitchDeg;
    if (profile->autoGimbalPitch) {
        pitchDeg = LzPlan_ComputeGimbalPitchDeg(profile->radiusM, altAboveTargetM,
                                                target->heightM);
    }

    /* 循环跑到 waypointCount（含）—— 第 waypointCount 次即收尾点，
     * 方位角比首点多走整 360°，缩回后与首点同坐标。 */
    for (int i = 0; i <= profile->waypointCount; ++i) {
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
        wp.gimbalPitchDeg = pitchDeg;

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
    if (profile->radiusM <= 0.0) {
        return LZ_ERR_PARAM;
    }
    /* 航点数：下限是几何必然，上限是安全包线 —— 两者错误码不同。
     *   < MIN → PARAM：两个点构不成圆，是编程错误
     *   > MAX → UNSAFE：数值合法但超出可用范围，是**操作员可修正**的输入
     * 与半径/高度的判法保持一致。 */
    if (profile->waypointCount < LZ_PLAN_WAYPOINT_MIN) {
        return LZ_ERR_PARAM;
    }
    if (profile->waypointCount > LZ_PLAN_WAYPOINT_MAX) {
        return LZ_ERR_UNSAFE;
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
        if (wp->gimbalPitchDeg < LZ_GIMBAL_PITCH_MIN_DEG ||
            wp->gimbalPitchDeg > LZ_GIMBAL_PITCH_MAX_DEG) {
            return LZ_ERR_RANGE;
        }
    }

    return LZ_OK;
}
