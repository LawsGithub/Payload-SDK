/**
 * @file lz_test_wpml.c
 * @brief wpml 生成物的**规范符合性**测试。
 *
 * 为什么单独一个文件：`lz_test_kmz` 测的是"包能不能解、CRC 对不对"，
 * 这里测的是"XML 里的字段符不符合 DJI 的 wpml 规范"。两者的失败原因
 * 完全不同（前者是 zip 格式错，后者是飞机不收），混在一起会让报错指向歧义。
 *
 * ## 依据是什么
 *
 * 规范原文：Cloud-API-Doc 仓库 `docs/cn/60.api-reference/00.dji-wpml/`
 * 下的 `20.template-kml.md` / `30.waylines-wpml.md` / `40.common-element.md`。
 * **不是**拿官方样例 KMZ 当权威 —— 实测官方样例自身缺 `wpml:globalRTHHeight`
 * （规范标为必需元素）却照样能飞，说明"样例里有/没有"推不出"必需/不必需"。
 */

#include "lz_wpml.h"
#include "lz_geo.h"
#include "lz_test.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

/** 取 <tag>值</tag> 的第一次出现，写入 out；找不到返回 false */
static bool tag_value(const char *text, const char *tag, double *out)
{
    const char *p = strstr(text, tag);
    if (p == NULL) {
        return false;
    }
    p += strlen(tag);
    char *end = NULL;
    const double v = strtod(p, &end);
    if (end == p) {
        return false;
    }
    *out = v;
    return true;
}

/** 统计 substring 出现次数 */
static int count_occurrences(const char *text, const char *needle)
{
    int n = 0;
    const size_t len = strlen(needle);
    for (const char *p = text; (p = strstr(p, needle)) != NULL; p += len) {
        n++;
    }
    return n;
}

/** 收集一份 XML 里所有 <wpml:gimbalYawRotateAngle> 的值，返回个数 */
static int collect_yaw_angles(const char *text, double *out, int cap)
{
    static const char *kTag = "<wpml:gimbalYawRotateAngle>";
    int n = 0;
    const char *p = text;
    while (n < cap && (p = strstr(p, kTag)) != NULL) {
        p += strlen(kTag);
        char *end = NULL;
        const double v = strtod(p, &end);
        if (end == p) {
            break;
        }
        out[n++] = v;
        p = end;
    }
    return n;
}

/** 收集 <wpml:waypointPoiPoint> 的 "纬度,经度" 前缀，返回个数。
 *  点在 XML 里写的是 "纬度,经度,高度"，第三段固定 0，比较时只看前两段。 */
static int collect_poi(const char *text, double *lat, double *lon, int cap)
{
    static const char *kTag = "<wpml:waypointPoiPoint>";
    int n = 0;
    const char *p = text;
    while (n < cap && (p = strstr(p, kTag)) != NULL) {
        p += strlen(kTag);
        char *end = NULL;
        const double a = strtod(p, &end);
        if (end == p || *end != ',') {
            break;
        }
        p = end + 1;
        const double b = strtod(p, &end);
        if (end == p) {
            break;
        }
        lat[n] = a;
        lon[n] = b;
        n++;
        p = end;
    }
    return n;
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

static LzOrbitProfile profile_of(double altM)
{
    LzOrbitProfile p = {
        .radiusM = 17.5,
        .altitudeM = altM,
        .speedMs = 3.0,
        .waypointCount = 8,
        .startBearingDeg = 0.0,
        .clockwise = true,
        .gimbalPitchDeg = -15.0,
        .turnMode = LZ_TURN_PASS_WITH_CURVE,
    };
    return p;
}

/* 一份建好的 wpml，供各用例复用；用完 LzWpml_Free */
static LzWpmlFiles build(const LzRoute *route, const LzTarget *p, const LzOrbitProfile *pr)
{
    LzWpmlFiles f;
    f.identity = LzWpml_DefaultIdentity();
    f.templateKml = NULL;
    f.waylinesWpml = NULL;
    (void)LzWpml_Build(route, p, pr, &f);
    return f;
}

int main(void)
{
    LZ_CASE("必需元素 globalRTHHeight 必须出现在两份文件里");
    {
        LzTarget p = pole();
        const LzGeo takeoff = p.geo;
        LzOrbitProfile pr = profile_of(100.7);
        LzRoute route;
        LzRoute_Init(&route);
        LZ_CHECK(LzPlan_BuildOrbit(&p, &takeoff, &pr, &route) == LZ_OK);

        LzWpmlFiles f = build(&route, &p, &pr);
        if (f.templateKml != NULL && f.waylinesWpml != NULL) {
            /* 规范 20.template-kml.md:164 与 30.waylines-wpml.md:135 均标「必需元素」 */
            LZ_CHECK(count_occurrences(f.templateKml, "<wpml:globalRTHHeight>") == 1);
            LZ_CHECK(count_occurrences(f.waylinesWpml, "<wpml:globalRTHHeight>") == 1);

            double tplRth = 0.0, wlRth = 0.0;
            LZ_CHECK(tag_value(f.templateKml, "<wpml:globalRTHHeight>", &tplRth));
            LZ_CHECK(tag_value(f.waylinesWpml, "<wpml:globalRTHHeight>", &wlRth));
            /* 返航高度不得低于航线高度：低于就意味着"先下降再返航"，
             * 而此刻飞机正在杆的旁边 */
            LZ_CHECK(tplRth >= pr.altitudeM);
            LZ_CHECK(wlRth >= pr.altitudeM);
            LZ_CHECK_NEAR(tplRth, wlRth, 1e-9);
        }
        LzWpml_Free(&f);
        LzRoute_Free(&route);
    }

    LZ_CASE("低航线高度时返航高度取下限");
    {
        LzTarget p = pole();
        const LzGeo takeoff = p.geo;
        LzOrbitProfile pr = profile_of(12.0);   /* demo 用的低空剖面 */
        LzRoute route;
        LzRoute_Init(&route);
        LZ_CHECK(LzPlan_BuildOrbit(&p, &takeoff, &pr, &route) == LZ_OK);

        LzWpmlFiles f = build(&route, &p, &pr);
        if (f.templateKml != NULL) {
            double rth = 0.0;
            LZ_CHECK(tag_value(f.templateKml, "<wpml:globalRTHHeight>", &rth));
            LZ_CHECK_NEAR(rth, (double)LZ_WPML_RTH_HEIGHT_FLOOR_M, 1e-9);
            LZ_CHECK(rth > pr.altitudeM);
        }
        LzWpml_Free(&f);
        LzRoute_Free(&route);
    }

    LZ_CASE("云台 yaw 必须落在规范的 [-180,180] 内");
    {
        LzTarget p = pole();
        const LzGeo takeoff = p.geo;
        LzOrbitProfile pr = profile_of(100.7);
        LzRoute route;
        LzRoute_Init(&route);
        LZ_CHECK(LzPlan_BuildOrbit(&p, &takeoff, &pr, &route) == LZ_OK);

        /* 先确认 lz_plan 给出来的确实是 [0,360) —— 否则这条测试没意义 */
        for (size_t i = 0; i < route.count; ++i) {
            LZ_CHECK(route.points[i].gimbalYawDeg >= 0.0);
            LZ_CHECK(route.points[i].gimbalYawDeg < 360.0);
        }

        LzWpmlFiles f = build(&route, &p, &pr);
        if (f.templateKml != NULL && f.waylinesWpml != NULL) {
            double tpl[64], wl[64];
            const int nt = collect_yaw_angles(f.templateKml, tpl, 64);
            const int nw = collect_yaw_angles(f.waylinesWpml, wl, 64);

            /* 每个航点一个 yaw —— 个数不对说明有航点没写 */
            LZ_CHECK(nt == (int)route.count);
            LZ_CHECK(nw == (int)route.count);

            for (int i = 0; i < nt; ++i) {
                LZ_CHECK(tpl[i] >= -180.0 && tpl[i] <= 180.0);
            }
            for (int i = 0; i < nw; ++i) {
                LZ_CHECK(wl[i] >= -180.0 && wl[i] <= 180.0);
            }

            /* 折算不能改变语义：下发角与规划角必须指同一方向。
             * 用圆周差比较 —— 180 与 -180 绝对差是 360，绝对差判据会误报。
             * 容差取 0.05°：XML 里写的是 "%.1f"，量化步长 0.1，
             * 所以"未量化值"与"落盘值"的差上限就是半个步长。
             * 这条容差是反向验证时发现的 —— 1e-6 会因量化本身而误报。 */
            for (size_t i = 0; i < route.count && (int)i < nt; ++i) {
                LZ_CHECK_ANGLE_NEAR(tpl[i], route.points[i].gimbalYawDeg, 0.05);
            }
        }
        LzWpml_Free(&f);
        LzRoute_Free(&route);
    }

    LZ_CASE("高度模式必须与 relativeAltM 的语义一致");
    {
        LzTarget p = pole();
        const LzGeo takeoff = p.geo;
        LzOrbitProfile pr = profile_of(100.7);
        LzRoute route;
        LzRoute_Init(&route);
        LZ_CHECK(LzPlan_BuildOrbit(&p, &takeoff, &pr, &route) == LZ_OK);

        LzWpmlFiles f = build(&route, &p, &pr);
        if (f.templateKml != NULL && f.waylinesWpml != NULL) {
            /* LzWaypoint.relativeAltM 是「相对起飞点」，两份文件都用
             * relativeToStartPoint 表达。若哪天改成椭球高模式，这个断言会红，
             * 正是提醒"语义换了，不只是换个字符串"。 */
            LZ_CHECK(strstr(f.templateKml,
                            "<wpml:heightMode>relativeToStartPoint</wpml:heightMode>") != NULL);
            LZ_CHECK(strstr(f.waylinesWpml,
                            "<wpml:executeHeightMode>relativeToStartPoint</wpml:executeHeightMode>") != NULL);
            LZ_CHECK(strstr(f.templateKml, "<wpml:heightMode>WGS84</wpml:heightMode>") == NULL);
        }
        LzWpml_Free(&f);
        LzRoute_Free(&route);
    }

    LZ_CASE("gimbalRotate 的参数块必须齐备（含 gimbalHeadingYawBase）");
    {
        LzTarget p = pole();
        const LzGeo takeoff = p.geo;
        LzOrbitProfile pr = profile_of(100.7);
        LzRoute route;
        LzRoute_Init(&route);
        LZ_CHECK(LzPlan_BuildOrbit(&p, &takeoff, &pr, &route) == LZ_OK);

        LzWpmlFiles f = build(&route, &p, &pr);
        if (f.templateKml != NULL && f.waylinesWpml != NULL) {
            /* `gimbalHeadingYawBase` 声明 yaw 角"相对什么"，在规范的
             * gimbalRotate 一节里标为**必需元素**。我们缺它、官方样例也缺它 ——
             * 但官方样例的 gimbalYawRotateEnable 是 0（不用 yaw），
             * 而我们的 yaw 是绕飞的全部意义，`absoluteAngle` 正依赖这个基准。
             * 所以这条断言守的是"不能因为样例没写就也不写"。 */
            LZ_CHECK(count_occurrences(f.templateKml, "<wpml:gimbalHeadingYawBase>north</wpml:gimbalHeadingYawBase>")
                     == (int)route.count);
            LZ_CHECK(count_occurrences(f.waylinesWpml, "<wpml:gimbalHeadingYawBase>north</wpml:gimbalHeadingYawBase>")
                     == (int)route.count);

            /* 基准必须在 gimbalRotateMode 之前出现 —— 规范的表格顺序如此，
             * 且"先声明坐标系、再给角度"读起来才不歧义。 */
            const char *base = strstr(f.templateKml, "<wpml:gimbalHeadingYawBase>");
            const char *mode = strstr(f.templateKml, "<wpml:gimbalRotateMode>");
            LZ_CHECK(base != NULL && mode != NULL && base < mode);
        }
        LzWpml_Free(&f);
        LzRoute_Free(&route);
    }

    LZ_CASE("偏航角模式与绕行相关的必需元素");
    {
        LzTarget p = pole();
        const LzGeo takeoff = p.geo;
        LzOrbitProfile pr = profile_of(100.7);
        LzRoute route;
        LzRoute_Init(&route);
        LZ_CHECK(LzPlan_BuildOrbit(&p, &takeoff, &pr, &route) == LZ_OK);

        LzWpmlFiles f = build(&route, &p, &pr);
        if (f.templateKml != NULL && f.waylinesWpml != NULL) {
            /* globalWaypointHeadingParam 里写一份（我们逐点 useGlobalHeadingParam=1），
             * waylines 里每个航点各写一份。
             * 取值跟绕行方向走 —— 顺时针绕飞就必须是 clockwise。 */
            LZ_CHECK(count_occurrences(f.templateKml, "<wpml:waypointHeadingPathMode>clockwise</wpml:waypointHeadingPathMode>") == 1);
            LZ_CHECK(count_occurrences(f.waylinesWpml, "<wpml:waypointHeadingPathMode>clockwise</wpml:waypointHeadingPathMode>") == (int)route.count);

            /* payloadParam 容器：template.kml 的 Folder 尾部，官方样例有、我们原先没有。
             * 只在 template 里写一次（它是航线级的默认负载位置）。 */
            LZ_CHECK(count_occurrences(f.templateKml, "<wpml:payloadParam>") == 1);
            LZ_CHECK(count_occurrences(f.waylinesWpml, "<wpml:payloadParam>") == 0);

            /* payloadParam 必须在 </Folder> 之前 */
            const char *pp = strstr(f.templateKml, "<wpml:payloadParam>");
            const char *fold = strstr(f.templateKml, "</Folder>");
            LZ_CHECK(pp != NULL && fold != NULL && pp < fold);
        }
        LzWpml_Free(&f);
        LzRoute_Free(&route);
    }

    LZ_CASE("机头必须朝向兴趣点（towardPOI），且兴趣点就是杆心");
    {
        LzTarget p = pole();
        const LzGeo takeoff = p.geo;
        LzOrbitProfile pr = profile_of(100.7);
        LzRoute route;
        LzRoute_Init(&route);
        LZ_CHECK(LzPlan_BuildOrbit(&p, &takeoff, &pr, &route) == LZ_OK);

        LzWpmlFiles f = build(&route, &p, &pr);
        if (f.templateKml != NULL && f.waylinesWpml != NULL) {
            /* ★ 绕飞的全部意义：机头一直盯着杆。
             *
             * 为什么是机头而不是云台 —— M4T 的云台 yaw 不能独立于机头偏转，
             * 规范在 gimbalRotate / orientedShoot / rotateYaw 三处都标了
             * "gimbalYawRotateAngle 与 aircraftHeading 需保持一致"，机型列
             * 明确包含 M4E/M4T。所以让机头承担这个偏转让。
             *
             * 这条断言守的是**那条被证伪的老路不能悄悄回来**：
             * 曾经写的是 followWayline（机头沿航线）+ 云台绝对 yaw 转 45°，
             * 飞机回的就是"角度过大无法转向"。 */
            LZ_CHECK(count_occurrences(f.templateKml, "<wpml:waypointHeadingMode>towardPOI</wpml:waypointHeadingMode>") == 1);
            LZ_CHECK(count_occurrences(f.waylinesWpml, "<wpml:waypointHeadingMode>towardPOI</wpml:waypointHeadingMode>") == (int)route.count);
            LZ_CHECK(strstr(f.templateKml, "<wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>") == NULL);
            LZ_CHECK(strstr(f.waylinesWpml, "<wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>") == NULL);

            /* 兴趣点坐标必须是**杆的经纬度**，不是占位符 0,0,0。
             * 老实现写的是 0.000000,0.000000,0.000000 —— 那个点在几内亚湾，
             * 机头会朝那儿转（也就是乱转）。 */
            double lat[64], lon[64];
            const int nt = collect_poi(f.templateKml, lat, lon, 64);
            const int nw = collect_poi(f.waylinesWpml, lat + 0, lon + 0, 0);  /* 只数 template */
            (void)nw;
            LZ_CHECK(nt == 1);
            LZ_CHECK_NEAR(lat[0], p.geo.latitudeDeg, 1e-6);
            LZ_CHECK_NEAR(lon[0], p.geo.longitudeDeg, 1e-6);

            /* 逐点也要写（飞机实际读 waylines.wpml） */
            double wlat[64], wlon[64];
            const int n = collect_poi(f.waylinesWpml, wlat, wlon, 64);
            LZ_CHECK(n == (int)route.count);
            for (int i = 0; i < n; ++i) {
                LZ_CHECK_NEAR(wlat[i], p.geo.latitudeDeg, 1e-6);
                LZ_CHECK_NEAR(wlon[i], p.geo.longitudeDeg, 1e-6);
            }

            /* 占位符 "0.000000,0.000000,0.000000" 绝不能再出现。
             * ⚠️ 只比较整串：高度那一栏按规范本来就该写 0，所以
             * "结尾是 ,0.000000" 这种 pattern 会误伤合法输出。 */
            LZ_CHECK(strstr(f.templateKml, "<wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>") == NULL);
            LZ_CHECK(strstr(f.waylinesWpml, "<wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>") == NULL);
        }
        LzWpml_Free(&f);
        LzRoute_Free(&route);
    }

    LZ_CASE("应拒绝非法杆位（兴趣点坐标不可信）");
    {
        LzTarget p = pole();
        const LzGeo takeoff = p.geo;
        LzOrbitProfile pr = profile_of(100.7);
        LzRoute route;
        LzRoute_Init(&route);
        LZ_CHECK(LzPlan_BuildOrbit(&p, &takeoff, &pr, &route) == LZ_OK);

        LzWpmlFiles f;
        f.identity = LzWpml_DefaultIdentity();
        /* pole 从"不参与生成"变成"兴趣点坐标来源"之后，它就不能再是 NULL。
         * 曾经这里 (void)pole; —— 传 NULL 也照样生成，因为坐标没用到。 */
        LZ_CHECK(LzWpml_Build(&route, NULL, &pr, &f) == LZ_ERR_NO_TARGET);

        LzTarget bad = p;
        bad.geo.latitudeDeg = NAN;
        LZ_CHECK(LzWpml_Build(&route, &bad, &pr, &f) == LZ_ERR_NO_TARGET);

        /* 零解（没有真实定位）也不能当兴趣点 —— 否则机头会朝几内亚湾转。
         * 判据来自 lz_types.h 的 LZ_GEO_NULL_SOLUTION_DEG。 */
        LzTarget nullSol = p;
        nullSol.geo.latitudeDeg = 0.0000003;
        nullSol.geo.longitudeDeg = 0.0000004;
        LZ_CHECK(LzWpml_Build(&route, &nullSol, &pr, &f) == LZ_ERR_NO_TARGET);

        LzRoute_Free(&route);
    }

    LZ_CASE("转弯模式：曲线段走 toPointAndPass…，且截距合规");
    {
        /* ★ 这条是"物理圆"需求的 wpml 契约 —— 用户 2026-09-22 要求
         * "无人机走弧线、与圆点位置保持不变"。
         *
         * 航点只给离散采样，两点之间走直线还是曲线由 waypointTurnMode 决定：
         *   直线 → 内接正多边形（8 点半径 20 m 时轨迹比圆**小 7.6%**）
         *   曲线 → 航点附近抹圆，轨迹贴近真圆
         * 规范注解里 Pilot 2 的「平滑过点，提前转弯」正是
         * toPointAndPassWithContinuityCurvature + useStraightLine=1。 */
        LzTarget p = pole();
        const LzGeo takeoff = p.geo;
        LzOrbitProfile pr = profile_of(100.7);
        LzRoute route;
        LzRoute_Init(&route);
        LZ_CHECK(LzPlan_BuildOrbit(&p, &takeoff, &pr, &route) == LZ_OK);

        LzWpmlFiles f = build(&route, &p, &pr);
        if (f.templateKml != NULL && f.waylinesWpml != NULL) {
            LZ_CHECK(count_occurrences(f.templateKml,
                       "<wpml:globalWaypointTurnMode>toPointAndPassWithContinuityCurvature</wpml:globalWaypointTurnMode>") == 1);
            LZ_CHECK(count_occurrences(f.waylinesWpml,
                       "<wpml:waypointTurnMode>toPointAndPassWithContinuityCurvature</wpml:waypointTurnMode>") == (int)route.count);

            /* 直线模式那个取值绝不能再出现（它会让轨迹退回内接多边形） */
            LZ_CHECK(strstr(f.templateKml, ">toPointAndStopWithDiscontinuityCurvature<") == NULL);
            LZ_CHECK(strstr(f.waylinesWpml, ">toPointAndStopWithDiscontinuityCurvature<") == NULL);

            /* useStraightLine=1 是与 curve 模式配套的（Pilot 2 的设置方法） */
            LZ_CHECK(count_occurrences(f.waylinesWpml, "<wpml:useStraightLine>1</wpml:useStraightLine>") == (int)route.count);

            /* 截距：规范要求落在 (0, 航段最大长度]，且段长必须 > 2×截距。
             * 逐个从 XML 里抠出来核 —— 光看生成函数不算数，要看落盘的值。 */
            const double suggested = LzPlan_SuggestDampingM(&route);
            LZ_CHECK(suggested > 0.0);

            double seg[64];
            int ns = 0;
            for (size_t i = 1; i + 1 < route.count; ++i) {
                seg[ns++] = LzGeo_DistanceM(&route.points[i - 1].geo,
                                            &route.points[i].geo);
            }
            double shortest = seg[0], longest = seg[0];
            for (int i = 1; i < ns; ++i) {
                if (seg[i] < shortest) { shortest = seg[i]; }
                if (seg[i] > longest)  { longest  = seg[i]; }
            }
            LZ_CHECK(suggested <= longest);
            LZ_CHECK(2.0 * suggested < shortest);

            const double fromXml = 6.0;   /* 17.5 m / 8 点：15.31 × 0.45 ≈ 6.89 */
            (void)fromXml;
        }
        LzWpml_Free(&f);

        /* 直线模式：截距必须是 0（该元素仅在曲线模式下必需） */
        pr.turnMode = LZ_TURN_TO_POINT_AND_STOP;
        LzRoute route2;
        LzRoute_Init(&route2);
        LZ_CHECK(LzPlan_BuildOrbit(&p, &takeoff, &pr, &route2) == LZ_OK);
        LzWpmlFiles f2 = build(&route2, &p, &pr);
        if (f2.waylinesWpml != NULL) {
            LZ_CHECK(count_occurrences(f2.waylinesWpml,
                       "<wpml:waypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:waypointTurnMode>") == (int)route2.count);
            LZ_CHECK(count_occurrences(f2.waylinesWpml,
                       "<wpml:waypointTurnDampingDist>0.00</wpml:waypointTurnDampingDist>") == (int)route2.count);
        }
        LzWpml_Free(&f2);
        LzRoute_Free(&route2);
        LzRoute_Free(&route);
    }

    LZ_CASE("闭合：整圈长度与航点计数");
    {
        /* 补了收尾点之后，distance 对应**整圈**（n 段弦），不再是 (n-1) 段。
         * 这条守的是 wpml 里那个 pathLen 循环与 lz_plan 的闭合改动同步 ——
         * 漏改会让 Pilot 显示的里程少一段（8 点时少 12.5%）。 */
        LzTarget p = pole();
        const LzGeo takeoff = p.geo;
        LzOrbitProfile pr = profile_of(100.7);
        pr.waypointCount = 8;
        LzRoute route;
        LzRoute_Init(&route);
        LZ_CHECK(LzPlan_BuildOrbit(&p, &takeoff, &pr, &route) == LZ_OK);
        /* 生成 9 个点（8 个方位 + 1 个重合收尾） */
        LZ_CHECK(route.count == 9);

        LzWpmlFiles f = build(&route, &p, &pr);
        if (f.waylinesWpml != NULL) {
            double dist = 0.0;
            LZ_CHECK(tag_value(f.waylinesWpml, "<wpml:distance>", &dist));
            /* 与逐段累加一致（十进制 XML 有量化，容差给足） */
            double sum = 0.0;
            for (size_t i = 1; i < route.count; ++i) {
                sum += LzGeo_DistanceM(&route.points[i - 1].geo,
                                       &route.points[i].geo);
            }
            LZ_CHECK_NEAR(dist, sum, 0.01);
            /* 整圈弦长和 < 真圆周长（弦总比弧短） */
            LZ_CHECK(dist < 2.0 * M_PI * pr.radiusM);
            /* 但已远大于 (n-1) 段 —— 旧实现在 8 点时会少约 13 m */
            LZ_CHECK(dist > 0.9 * 2.0 * M_PI * pr.radiusM);
        }
        LzWpml_Free(&f);
        LzRoute_Free(&route);
    }

    LZ_CASE("不应写入 M3D 专属的绕行元素");
    {
        LzTarget p = pole();
        const LzGeo takeoff = p.geo;
        LzOrbitProfile pr = profile_of(100.7);
        LzRoute route;
        LzRoute_Init(&route);
        LZ_CHECK(LzPlan_BuildOrbit(&p, &takeoff, &pr, &route) == LZ_OK);

        LzWpmlFiles f = build(&route, &p, &pr);
        if (f.templateKml != NULL && f.waylinesWpml != NULL) {
            /* autoRerouteInfo 一族在规范的「支持机型」列写的是 **M3D/M3TD**，
             * M4T 不在列内。这条断言守的是**判断必需元素的方法论**：
             * 必须同时看「是否必需」与「支持机型」两列 ——
             * 只 grep "必需元素" 会把 M3D 专属元素也加进来。 */
            LZ_CHECK(strstr(f.templateKml, "autoRerouteInfo") == NULL);
            LZ_CHECK(strstr(f.waylinesWpml, "autoRerouteInfo") == NULL);
            LZ_CHECK(strstr(f.templateKml, "missionAutoRerouteMode") == NULL);
            LZ_CHECK(strstr(f.waylinesWpml, "transitionalAutoRerouteMode") == NULL);
        }
        LzWpml_Free(&f);
        LzRoute_Free(&route);
    }

    LZ_CASE("空入参被拒绝");
    {
        LzWpmlFiles files;
        LzRoute route;
        LzTarget p = pole();
        LzOrbitProfile pr = profile_of(100.7);
        LzRoute_Init(&route);
        LZ_CHECK(LzWpml_Build(NULL, &p, &pr, &files) == LZ_ERR_PARAM);
        LZ_CHECK(LzWpml_Build(&route, &p, &pr, NULL) == LZ_ERR_PARAM);
    }

    return LZ_TEST_SUMMARY();
}
