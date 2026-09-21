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
