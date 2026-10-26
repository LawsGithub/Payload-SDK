/**
 * @file wt_test_ped_square.c
 * @brief 绕行人正方形航线：规划几何 + KMZ 正文的回归测试
 *
 * 守的是用户原话里的每一条（2026-10-25）：
 *   - 以基点为几何中心的正方形，边长 20 m
 *   - 从第一个顶点 10 m（相对起飞点）开始，垂直爬升到 20 m
 *   - 顺时针沿边飞到下一顶点，再垂直降到 10 m，依次类推
 *   - 最后回到第一个顶点 10 m 处
 *   - 顶点到顶点的水平飞行中机头时刻朝向基点（KMZ：towardPOI + 兴趣点 = 基点）
 *
 * KMZ 部分查的是**生成出来的文本**，不是结构体 —— 飞机读的是文本。
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wt_bridge.h"
#include "wt_geometry.h"
#include "wt_ped_square.h"
#include "wt_test.h"

#define WT_TEST_SQUARE_KMZ "wt_test_ped_square.kmz"

static const WtGeo s_base = {41.5236000, 111.7461000, 1452.0};

/** 在基点的局部系里取一个点（东 e、北 n 米） */
static WtGeo GeoAt(double e, double n)
{
    WtLocalFrame f;
    WtEnu d = {e, n, 0.0};
    WtGeo flat = s_base;

    flat.alt = 0.0;
    f = WtLocalFrame_Init(&flat);
    return WtEnu_ToGeo(&f, &d);
}

static WtEnu EnuOf(const WtGeo *g)
{
    WtLocalFrame f;
    WtGeo flat = s_base;
    WtGeo p = *g;

    flat.alt = 0.0;
    p.alt = 0.0;
    f = WtLocalFrame_Init(&flat);
    return WtGeo_ToEnu(&f, &p);
}

static double Horiz(WtEnu a, WtEnu b)
{
    return hypot(a.e - b.e, a.n - b.n);
}

/** 统计 needle 在 hay 里出现几次 */
static int CountOf(const char *hay, const char *needle)
{
    int n = 0;
    size_t len = strlen(needle);
    const char *p = hay;

    while ((p = strstr(p, needle)) != NULL) {
        n++;
        p += len;
    }
    return n;
}

static void TestDefaultsAreUserNumbers(void)
{
    WtPedSquareConfig c = WtPedSquare_DefaultConfig();

    WT_CASE("默认参数就是用户给的 20 m / 10 m / 20 m");
    WT_CHECK_NEAR(c.sideM, 20.0, 1e-12);
    WT_CHECK_NEAR(c.lowHeightM, 10.0, 1e-12);
    WT_CHECK_NEAR(c.highHeightM, 20.0, 1e-12);
    WT_CHECK(WtPedSquare_ConfigValid(&c));
}

static void TestSequence(void)
{
    static const double heights[WT_PED_SQUARE_WAYPOINTS] = {10, 20, 20, 10, 20, 20, 10,
                                                            20, 20, 10, 20, 20, 10};
    static const int corners[WT_PED_SQUARE_WAYPOINTS] = {0, 0, 1, 1, 1, 2, 2,
                                                         2, 3, 3, 3, 0, 0};
    static const E_WtPedSquareStep steps[WT_PED_SQUARE_WAYPOINTS] = {
        WT_PED_SQUARE_STEP_START,
        WT_PED_SQUARE_STEP_CLIMB, WT_PED_SQUARE_STEP_CRUISE, WT_PED_SQUARE_STEP_DESCEND,
        WT_PED_SQUARE_STEP_CLIMB, WT_PED_SQUARE_STEP_CRUISE, WT_PED_SQUARE_STEP_DESCEND,
        WT_PED_SQUARE_STEP_CLIMB, WT_PED_SQUARE_STEP_CRUISE, WT_PED_SQUARE_STEP_DESCEND,
        WT_PED_SQUARE_STEP_CLIMB, WT_PED_SQUARE_STEP_CRUISE, WT_PED_SQUARE_STEP_DESCEND,
    };
    WtGeo aircraft = GeoAt(-25.0, -25.0);
    WtPedSquarePlan p;
    int i;

    WT_CASE("13 个航点：低 → 高 → 平飞 → 低，最后回到起点低点");
    WT_CHECK_EQ_INT(WtPedSquare_Build(NULL, &s_base, &aircraft, &p), WT_PED_SQUARE_OK);
    WT_CHECK_EQ_INT(p.count, WT_PED_SQUARE_WAYPOINTS);
    for (i = 0; i < p.count; i++) {
        WT_CHECK_NEAR(p.wp[i].relHeightM, heights[i], 1e-12);
        WT_CHECK_NEAR(p.wp[i].enu.u, heights[i], 1e-12);
        WT_CHECK_EQ_INT(p.wp[i].corner, corners[i]);
        WT_CHECK_EQ_INT(p.wp[i].step, steps[i]);
    }
    /* 终点与起点同一位置、同一高度 */
    WT_CHECK_NEAR(Horiz(p.wp[0].enu, p.wp[12].enu), 0.0, 1e-9);
    WT_CHECK_NEAR(p.wp[0].relHeightM, p.wp[12].relHeightM, 1e-12);
}

static void TestGeometry(void)
{
    WtGeo aircraft = GeoAt(30.0, 5.0);
    WtPedSquarePlan p;
    WtEnu centroid = {0.0, 0.0, 0.0};
    double area2 = 0.0;
    int i;

    WT_CASE("几何中心是基点、边长 20 m、垂直段纯垂直、平飞段等于边长");
    WT_CHECK_EQ_INT(WtPedSquare_Build(NULL, &s_base, &aircraft, &p), WT_PED_SQUARE_OK);

    for (i = 0; i < WT_PED_SQUARE_CORNERS; i++) {
        const WtEnu a = p.cornerEnu[i];
        const WtEnu b = p.cornerEnu[(i + 1) % WT_PED_SQUARE_CORNERS];

        centroid.e += a.e / WT_PED_SQUARE_CORNERS;
        centroid.n += a.n / WT_PED_SQUARE_CORNERS;
        WT_CHECK_NEAR(Horiz(a, b), 20.0, 1e-9);           /* 相邻顶点 = 边长 */
        WT_CHECK_NEAR(hypot(a.e, a.n), 10.0 * sqrt(2.0), 1e-9); /* 到中心 = 半对角线 */
        area2 += a.e * b.n - b.e * a.n;                   /* 鞋带公式 */
    }
    WT_CHECK_NEAR(centroid.e, 0.0, 1e-9);
    WT_CHECK_NEAR(centroid.n, 0.0, 1e-9);
    /* ENU（东为 x、北为 y）里有向面积为负 = 俯视顺时针；|面积| = 400 m² */
    WT_CHECK_NEAR(area2 / 2.0, -400.0, 1e-6);

    for (i = 1; i < p.count; i++) {
        const WtPedSquareWaypoint *a = &p.wp[i - 1];
        const WtPedSquareWaypoint *b = &p.wp[i];
        const double h = Horiz(a->enu, b->enu);
        const double v = fabs(b->relHeightM - a->relHeightM);

        if (b->step == WT_PED_SQUARE_STEP_CRUISE) {
            WT_CHECK_NEAR(h, 20.0, 1e-9);   /* 沿边长直线，不走对角线 */
            WT_CHECK_NEAR(v, 0.0, 1e-12);   /* 平飞：高度不变 */
            WT_CHECK_NEAR(a->relHeightM, 20.0, 1e-12);
        } else {
            WT_CHECK_NEAR(h, 0.0, 1e-9);    /* 爬升/下降：水平不动 */
            WT_CHECK_NEAR(v, 10.0, 1e-12);
        }
    }

    WT_CHECK_NEAR(p.pathLengthM, 4 * 20.0 + 8 * 10.0, 1e-9);
    WT_CHECK_NEAR(p.durationS, p.pathLengthM / 3.0, 1e-9);
}

static void TestClockwiseAndHeading(void)
{
    WtGeo aircraft = GeoAt(-3.0, 40.0);
    WtPedSquarePlan p;
    int i;

    WT_CASE("顶点方位每步 +90°（顺时针），每个航点的朝向都指着基点");
    WT_CHECK_EQ_INT(WtPedSquare_Build(NULL, &s_base, &aircraft, &p), WT_PED_SQUARE_OK);

    for (i = 0; i + 1 < WT_PED_SQUARE_CORNERS; i++) {
        WT_CHECK_NEAR(Wt_AngleDiff(p.cornerBearingDeg[i + 1], p.cornerBearingDeg[i]), 90.0, 1e-9);
    }
    WT_CHECK_NEAR(Wt_AngleDiff(p.cornerBearingDeg[0], p.cornerBearingDeg[3]), 90.0, 1e-9);

    for (i = 0; i < p.count; i++) {
        const WtPedSquareWaypoint *w = &p.wp[i];
        WtEnu toBase = {-w->enu.e, -w->enu.n, 0.0};

        WT_CHECK_NEAR(Wt_AngleDiff(w->headingToBaseDeg, WtEnu_Azimuth(toBase)), 0.0, 1e-9);
        /* 朝基点 = 顶点方位 + 180° */
        WT_CHECK_NEAR(Wt_AngleDiff(w->headingToBaseDeg, p.cornerBearingDeg[w->corner] + 180.0),
                      0.0, 1e-9);
    }
}

static void TestFirstCornerIsNearest(void)
{
    /* 飞机在四个象限 → 首个顶点就在那个象限；方位 45/135/225/315 */
    static const double where[4][2] = {{30, 30}, {30, -30}, {-30, -30}, {-30, 30}};
    static const double expectBearing[4] = {45.0, 135.0, 225.0, 315.0};
    int q;

    WT_CASE("第一个顶点取离飞机最近的那个（不横穿正方形、不从人头顶过）");
    for (q = 0; q < 4; q++) {
        WtGeo aircraft = GeoAt(where[q][0], where[q][1]);
        WtPedSquarePlan p;
        int k;
        double best = 1e9;

        WT_CHECK_EQ_INT(WtPedSquare_Build(NULL, &s_base, &aircraft, &p), WT_PED_SQUARE_OK);
        WT_CHECK_NEAR(Wt_AngleDiff(p.cornerBearingDeg[0], expectBearing[q]), 0.0, 1e-9);
        for (k = 0; k < WT_PED_SQUARE_CORNERS; k++) {
            WtEnu ac = EnuOf(&aircraft);
            double d = Horiz(p.cornerEnu[k], ac);

            if (d < best) {
                best = d;
            }
        }
        WT_CHECK_NEAR(p.transitM, best, 1e-6);
        WT_CHECK_NEAR(Horiz(p.wp[0].enu, EnuOf(&aircraft)), best, 1e-6);
    }
}

static void TestGeoRoundTrip(void)
{
    WtGeo aircraft = GeoAt(10.0, -50.0);
    WtPedSquarePlan p;
    int i;

    WT_CASE("航点经纬度换回局部坐标与规划一致（毫米级）");
    WT_CHECK_EQ_INT(WtPedSquare_Build(NULL, &s_base, &aircraft, &p), WT_PED_SQUARE_OK);
    for (i = 0; i < p.count; i++) {
        WtEnu back = EnuOf(&p.wp[i].geo);

        WT_CHECK_NEAR(back.e, p.wp[i].enu.e, 1e-3);
        WT_CHECK_NEAR(back.n, p.wp[i].enu.n, 1e-3);
    }
}

static void TestRejects(void)
{
    WtGeo aircraft = GeoAt(-20.0, -20.0);
    WtGeo bad;
    WtPedSquareConfig c;
    WtPedSquarePlan p;

    WT_CASE("非法参数、非法坐标、基点太远都拒绝");

    c = WtPedSquare_DefaultConfig();
    c.lowHeightM = 4.0; /* 低于 5 m 下限 */
    WT_CHECK_EQ_INT(WtPedSquare_Build(&c, &s_base, &aircraft, &p), WT_PED_SQUARE_ERR_CONFIG);
    c = WtPedSquare_DefaultConfig();
    c.highHeightM = 11.0; /* 爬升不足 2 m */
    WT_CHECK_EQ_INT(WtPedSquare_Build(&c, &s_base, &aircraft, &p), WT_PED_SQUARE_ERR_CONFIG);
    c = WtPedSquare_DefaultConfig();
    c.speedMs = 16.0; /* wpml 速度上限 15 */
    WT_CHECK_EQ_INT(WtPedSquare_Build(&c, &s_base, &aircraft, &p), WT_PED_SQUARE_ERR_CONFIG);
    c = WtPedSquare_DefaultConfig();
    c.transitSpeedMs = 0.5;
    WT_CHECK_EQ_INT(WtPedSquare_Build(&c, &s_base, &aircraft, &p), WT_PED_SQUARE_ERR_CONFIG);
    c = WtPedSquare_DefaultConfig();
    c.sideM = NAN;
    WT_CHECK_EQ_INT(WtPedSquare_Build(&c, &s_base, &aircraft, &p), WT_PED_SQUARE_ERR_CONFIG);
    c = WtPedSquare_DefaultConfig();
    c.sideM = 2.0;
    WT_CHECK_EQ_INT(WtPedSquare_Build(&c, &s_base, &aircraft, &p), WT_PED_SQUARE_ERR_CONFIG);

    bad = s_base;
    bad.lat = NAN;
    WT_CHECK_EQ_INT(WtPedSquare_Build(NULL, &bad, &aircraft, &p), WT_PED_SQUARE_ERR_BASE);
    bad.lat = 0.0;
    bad.lon = 0.0; /* 零解 */
    WT_CHECK_EQ_INT(WtPedSquare_Build(NULL, &bad, &aircraft, &p), WT_PED_SQUARE_ERR_BASE);
    bad.lat = 91.0;
    bad.lon = 111.0;
    WT_CHECK_EQ_INT(WtPedSquare_Build(NULL, &bad, &aircraft, &p), WT_PED_SQUARE_ERR_BASE);
    WT_CHECK_EQ_INT(WtPedSquare_Build(NULL, &s_base, NULL, &p), WT_PED_SQUARE_ERR_AIRCRAFT);
    bad.lat = 0.0;
    bad.lon = 0.0;
    WT_CHECK_EQ_INT(WtPedSquare_Build(NULL, &s_base, &bad, &p), WT_PED_SQUARE_ERR_AIRCRAFT);

    /* 上限 150 m：149 m 可以，151 m 拒绝 */
    aircraft = GeoAt(149.0, 0.0);
    WT_CHECK_EQ_INT(WtPedSquare_Build(NULL, &s_base, &aircraft, &p), WT_PED_SQUARE_OK);
    aircraft = GeoAt(0.0, -151.0);
    WT_CHECK_EQ_INT(WtPedSquare_Build(NULL, &s_base, &aircraft, &p), WT_PED_SQUARE_ERR_TOO_FAR);
    WT_CHECK_NEAR(p.aircraftToBaseM, 151.0, 0.01);   /* 失败时也给出距离，供浮窗提示 */
    WT_CHECK_EQ_INT(p.count, 0);                      /* 但不留半成品航点 */
}

static void TestRthHeight(void)
{
    WtGeo aircraft = GeoAt(-20.0, -20.0);
    WtPedSquareConfig c = WtPedSquare_DefaultConfig();
    WtPedSquarePlan p;

    WT_CASE("返航高度 = max(下限, 高点)，不会比航线还低");
    WT_CHECK_EQ_INT(WtPedSquare_Build(&c, &s_base, &aircraft, &p), WT_PED_SQUARE_OK);
    WT_CHECK_NEAR(p.rthHeightM, 30.0, 1e-12);
    c.highHeightM = 45.0;
    WT_CHECK_EQ_INT(WtPedSquare_Build(&c, &s_base, &aircraft, &p), WT_PED_SQUARE_OK);
    WT_CHECK_NEAR(p.rthHeightM, 45.0, 1e-12);
}

/** 从 waylines 正文里按顺序抠出每个航点的经纬度与高度 */
static int ParseWaylines(const char *wpml, double lat[], double lon[], double h[], int maxN)
{
    const char *p = wpml;
    int n = 0;

    while (n < maxN && (p = strstr(p, "<Placemark>")) != NULL) {
        const char *c = strstr(p, "<coordinates>");
        const char *e = strstr(p, "<wpml:executeHeight>");

        if (c == NULL || e == NULL) {
            break;
        }
        if (sscanf(c + strlen("<coordinates>"), "%lf,%lf", &lon[n], &lat[n]) != 2 ||
            sscanf(e + strlen("<wpml:executeHeight>"), "%lf", &h[n]) != 1) {
            break;
        }
        n++;
        p = e;
    }
    return n;
}

static void TestWpmlText(void)
{
    WtGeo aircraft = GeoAt(-30.0, -10.0);
    WtPedSquareConfig cfg = WtPedSquare_DefaultConfig();
    WtPedSquarePlan p;
    char *tpl = NULL;
    char *wl = NULL;
    char poi[96];
    char idx[64];
    double lat[WT_PED_SQUARE_WAYPOINTS + 1];
    double lon[WT_PED_SQUARE_WAYPOINTS + 1];
    double h[WT_PED_SQUARE_WAYPOINTS + 1];
    int n;
    int i;

    WT_CASE("waylines.wpml：13 个航点，每点 towardPOI 指向基点、到点停，高度与规划一致");
    WT_CHECK_EQ_INT(WtPedSquare_Build(&cfg, &s_base, &aircraft, &p), WT_PED_SQUARE_OK);
    WT_CHECK_EQ_INT(WtBridge_BuildPedSquareWpml(&p, &cfg, &tpl, &wl), WT_PLAN_OK);
    if (tpl == NULL || wl == NULL) {
        WT_CHECK(0);
        free(tpl);
        free(wl);
        return;
    }

    snprintf(poi, sizeof(poi), "<wpml:waypointPoiPoint>%.8f,%.8f,0.000000</wpml:waypointPoiPoint>",
             s_base.lat, s_base.lon);
    WT_CHECK_EQ_INT(CountOf(wl, "<Placemark>"), WT_PED_SQUARE_WAYPOINTS);
    WT_CHECK_EQ_INT(CountOf(wl, "<wpml:waypointHeadingMode>towardPOI</wpml:waypointHeadingMode>"),
                    WT_PED_SQUARE_WAYPOINTS);
    WT_CHECK_EQ_INT(CountOf(wl, poi), WT_PED_SQUARE_WAYPOINTS);
    WT_CHECK_EQ_INT(CountOf(wl, "<wpml:waypointTurnMode>toPointAndStopWithDiscontinuityCurvature"
                                "</wpml:waypointTurnMode>"),
                    WT_PED_SQUARE_WAYPOINTS);
    for (i = 0; i < WT_PED_SQUARE_WAYPOINTS; i++) {
        snprintf(idx, sizeof(idx), "<wpml:index>%d</wpml:index>", i);
        WT_CHECK_EQ_INT(CountOf(wl, idx), 1); /* 从 0 起连续递增，各一次 */
    }

    n = ParseWaylines(wl, lat, lon, h, WT_PED_SQUARE_WAYPOINTS + 1);
    WT_CHECK_EQ_INT(n, WT_PED_SQUARE_WAYPOINTS);
    for (i = 0; i < n; i++) {
        WtGeo g = {lat[i], lon[i], 0.0};
        WtEnu e = EnuOf(&g);

        WT_CHECK_NEAR(h[i], p.wp[i].relHeightM, 1e-9);
        /* %.8f 的经纬度约 1 mm 分辨率 */
        WT_CHECK_NEAR(e.e, p.wp[i].enu.e, 2e-3);
        WT_CHECK_NEAR(e.n, p.wp[i].enu.n, 2e-3);
    }

    /* 执行文件该有的元素 */
    WT_CHECK_EQ_INT(CountOf(wl, "<wpml:executeHeightMode>relativeToStartPoint</wpml:executeHeightMode>"), 1);
    WT_CHECK_EQ_INT(CountOf(wl, "<wpml:waylineId>0</wpml:waylineId>"), 1);
    WT_CHECK_EQ_INT(CountOf(wl, "<wpml:finishAction>noAction</wpml:finishAction>"), 1);
    WT_CHECK_EQ_INT(CountOf(wl, "<wpml:globalRTHHeight>30.0</wpml:globalRTHHeight>"), 1);
    WT_CHECK_EQ_INT(CountOf(wl, "<wpml:droneEnumValue>99</wpml:droneEnumValue>"), 1);
    WT_CHECK_EQ_INT(CountOf(wl, "<wpml:droneSubEnumValue>1</wpml:droneSubEnumValue>"), 1);
    WT_CHECK_EQ_INT(CountOf(wl, "<wpml:payloadEnumValue>89</wpml:payloadEnumValue>"), 1);
    /* 不带任何动作：没有云台转动、没有拍照 */
    WT_CHECK_EQ_INT(CountOf(wl, "actionGroup"), 0);
    /* 模板专用元素不该出现在执行文件里 */
    WT_CHECK_EQ_INT(CountOf(wl, "templateType"), 0);
    WT_CHECK_EQ_INT(CountOf(wl, "waylineCoordinateSysParam"), 0);

    WT_CASE("template.kml：模板元素齐全，且不混进执行文件专用元素");
    WT_CHECK_EQ_INT(CountOf(tpl, "<Placemark>"), WT_PED_SQUARE_WAYPOINTS);
    WT_CHECK_EQ_INT(CountOf(tpl, "<wpml:templateType>waypoint</wpml:templateType>"), 1);
    WT_CHECK_EQ_INT(CountOf(tpl, "<wpml:heightMode>relativeToStartPoint</wpml:heightMode>"), 1);
    WT_CHECK_EQ_INT(CountOf(tpl, "<wpml:waypointHeadingMode>towardPOI</wpml:waypointHeadingMode>"), 1);
    WT_CHECK_EQ_INT(CountOf(tpl, poi), 1);
    WT_CHECK_EQ_INT(CountOf(tpl, "<wpml:useGlobalHeadingParam>1</wpml:useGlobalHeadingParam>"),
                    WT_PED_SQUARE_WAYPOINTS);
    WT_CHECK_EQ_INT(CountOf(tpl, "executeHeight"), 0);
    WT_CHECK_EQ_INT(CountOf(tpl, "waylineId"), 0);
    WT_CHECK_EQ_INT(CountOf(tpl, "<wpml:finishAction>noAction</wpml:finishAction>"), 1);
    WT_CHECK(strcmp(tpl, wl) != 0);

    free(tpl);
    free(wl);
}

static void TestKmzFile(void)
{
    WtGeo aircraft = GeoAt(-30.0, -10.0);
    WtPedSquareConfig cfg = WtPedSquare_DefaultConfig();
    WtPedSquarePlan p;
    char *tpl = NULL;
    char *wl = NULL;
    char *buf = NULL;
    long size = 0;
    FILE *fp;

    WT_CASE("KMZ 文件：两个条目各放各的正文（不再是同一份）");
    WT_CHECK_EQ_INT(WtPedSquare_Build(&cfg, &s_base, &aircraft, &p), WT_PED_SQUARE_OK);
    WT_CHECK_EQ_INT(WtBridge_ExportPedSquareKmz(&p, &cfg, WT_TEST_SQUARE_KMZ), WT_PLAN_OK);
    WT_CHECK_EQ_INT(WtBridge_BuildPedSquareWpml(&p, &cfg, &tpl, &wl), WT_PLAN_OK);

    fp = fopen(WT_TEST_SQUARE_KMZ, "rb");
    if (fp != NULL) {
        fseek(fp, 0, SEEK_END);
        size = ftell(fp);
        fseek(fp, 0, SEEK_SET);
        buf = malloc((size_t)size + 1);
        if (buf != NULL && fread(buf, 1, (size_t)size, fp) == (size_t)size) {
            buf[size] = '\0';
        } else {
            free(buf);
            buf = NULL;
        }
        fclose(fp);
    }
    WT_CHECK(buf != NULL && tpl != NULL && wl != NULL);
    if (buf != NULL && tpl != NULL && wl != NULL) {
        const size_t hdr = 30;
        const size_t n1 = strlen("wpmz/template.kml");
        const size_t n2 = strlen("wpmz/waylines.wpml");
        const size_t off2 = hdr + n1 + strlen(tpl);

        /* store 方式：正文原样落在本地文件头 + 文件名之后 */
        WT_CHECK(memcmp(buf, "PK\x03\x04", 4) == 0);
        WT_CHECK(memcmp(buf + hdr, "wpmz/template.kml", n1) == 0);
        WT_CHECK(memcmp(buf + hdr + n1, tpl, strlen(tpl)) == 0);
        WT_CHECK(memcmp(buf + off2, "PK\x03\x04", 4) == 0);
        WT_CHECK(memcmp(buf + off2 + hdr, "wpmz/waylines.wpml", n2) == 0);
        WT_CHECK(memcmp(buf + off2 + hdr + n2, wl, strlen(wl)) == 0);
        /* 中央目录紧跟在第二个条目之后 */
        WT_CHECK(memcmp(buf + off2 + hdr + n2 + strlen(wl), "PK\x01\x02", 4) == 0);
    }

    free(buf);
    free(tpl);
    free(wl);
    remove(WT_TEST_SQUARE_KMZ);
}

int main(void)
{
    TestDefaultsAreUserNumbers();
    TestSequence();
    TestGeometry();
    TestClockwiseAndHeading();
    TestFirstCornerIsNearest();
    TestGeoRoundTrip();
    TestRejects();
    TestRthHeight();
    TestWpmlText();
    TestKmzFile();

    return WT_TEST_SUMMARY();
}
