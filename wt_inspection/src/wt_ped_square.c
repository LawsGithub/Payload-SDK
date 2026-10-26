/**
 * @file wt_ped_square.c
 * @brief 绕行人正方形航线的规划实现
 */

#include "wt_ped_square.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* wpml 对 autoFlightSpeed / waypointSpeed / globalTransitionalSpeed 的取值域 */
#define WT_PED_SQUARE_SPEED_MIN 1.0
#define WT_PED_SQUARE_SPEED_MAX 15.0

/*
 * 两高度之差的下限 m。垂直段是两个经纬度相同、只差高度的航点，差太小的话
 * 两个航点几乎重合，飞控怎么处理没有文档；用户要的是 10 m，留足余量即可。
 */
#define WT_PED_SQUARE_MIN_CLIMB_M 2.0

/* 边长的合理范围 m：太小两顶点重合，太大已经不是"绕着人飞"了 */
#define WT_PED_SQUARE_SIDE_MIN_M 4.0
#define WT_PED_SQUARE_SIDE_MAX_M 200.0

/*
 * 顶点按**俯视顺时针**排列：东北 → 东南 → 西南 → 西北。
 *
 * 方位角约定是正北 0°、顺时针为正（见 wt_geometry.h），四个顶点相对基点的
 * 方位依次是 45° / 135° / 225° / 315° —— 方位递增 = 俯视顺时针。
 * 每个元素是 (东向符号, 北向符号)。
 */
static const int s_cornerSign[WT_PED_SQUARE_CORNERS][2] = {
    {+1, +1}, /* 东北 */
    {+1, -1}, /* 东南 */
    {-1, -1}, /* 西南 */
    {-1, +1}, /* 西北 */
};

WtPedSquareConfig WtPedSquare_DefaultConfig(void)
{
    WtPedSquareConfig c;

    memset(&c, 0, sizeof(c));
    c.sideM = 20.0;
    c.lowHeightM = 10.0;
    c.highHeightM = 20.0;
    /*
     * 3 m/s：离人 10 m、离地 10 m 的航线，慢一点让操作员有反应时间。
     * 20 m 一条边约 7 s，整圈约 1 分钟（每点刹停，实际再长一些）。
     */
    c.speedMs = 3.0;
    c.transitSpeedMs = 5.0;
    c.rthHeightM = 30.0;
    c.maxBaseDistanceM = 150.0;

    return c;
}

static bool WtPedSquare_InRange(double v, double lo, double hi)
{
    return isfinite(v) && v >= lo && v <= hi;
}

bool WtPedSquare_ConfigValid(const WtPedSquareConfig *cfg)
{
    if (cfg == NULL) {
        return false;
    }
    if (!WtPedSquare_InRange(cfg->sideM, WT_PED_SQUARE_SIDE_MIN_M, WT_PED_SQUARE_SIDE_MAX_M)) {
        return false;
    }
    if (!isfinite(cfg->lowHeightM) || cfg->lowHeightM < WT_PED_SQUARE_MIN_HEIGHT_M) {
        return false;
    }
    if (!isfinite(cfg->highHeightM) ||
        cfg->highHeightM < cfg->lowHeightM + WT_PED_SQUARE_MIN_CLIMB_M) {
        return false;
    }
    if (!WtPedSquare_InRange(cfg->speedMs, WT_PED_SQUARE_SPEED_MIN, WT_PED_SQUARE_SPEED_MAX) ||
        !WtPedSquare_InRange(cfg->transitSpeedMs, WT_PED_SQUARE_SPEED_MIN,
                             WT_PED_SQUARE_SPEED_MAX)) {
        return false;
    }
    if (!isfinite(cfg->rthHeightM) || cfg->rthHeightM < 0.0) {
        return false;
    }
    if (!isfinite(cfg->maxBaseDistanceM) || cfg->maxBaseDistanceM <= 0.0) {
        return false;
    }

    return true;
}

bool WtPedSquare_GeoLooksValid(const WtGeo *g)
{
    if (g == NULL || !isfinite(g->lat) || !isfinite(g->lon)) {
        return false;
    }
    if (fabs(g->lat) > 90.0 || fabs(g->lon) > 180.0) {
        return false;
    }
    /* 零解：两个都贴着 0。约 0.1 m 量级，真实定位不可能同时落在这里 */
    if (fabs(g->lat) < 1e-6 && fabs(g->lon) < 1e-6) {
        return false;
    }

    return true;
}

static double WtPedSquare_Horizontal(WtEnu v)
{
    return sqrt(v.e * v.e + v.n * v.n);
}

static void WtPedSquare_Put(WtPedSquarePlan *out, const WtLocalFrame *frame, int corner,
                            double relHeightM, E_WtPedSquareStep step)
{
    WtPedSquareWaypoint *w = &out->wp[out->count];
    WtEnu flat = out->cornerEnu[corner];
    WtEnu toBase;

    flat.u = 0.0;
    w->geo = WtEnu_ToGeo(frame, &flat);
    w->enu = flat;
    w->enu.u = relHeightM;
    w->relHeightM = relHeightM;
    w->corner = corner;
    w->step = step;

    toBase.e = -flat.e;
    toBase.n = -flat.n;
    toBase.u = 0.0;
    w->headingToBaseDeg = WtEnu_Azimuth(toBase);

    out->count++;
}

E_WtPedSquareResult WtPedSquare_Build(const WtPedSquareConfig *cfg, const WtGeo *base,
                                      const WtGeo *aircraft, WtPedSquarePlan *out)
{
    WtPedSquareConfig def = WtPedSquare_DefaultConfig();
    WtLocalFrame frame;
    WtGeo baseFlat;
    WtGeo aircraftFlat;
    WtEnu ac;
    double half;
    double bestDist = 0.0;
    int first = 0;
    int k;

    if (cfg == NULL) {
        cfg = &def;
    }
    if (out == NULL) {
        return WT_PED_SQUARE_ERR_CONFIG;
    }
    /* 先清零：失败时调用方仍可安全地读字段（例如打印距离），不会读到垃圾 */
    memset(out, 0, sizeof(*out));
    if (!WtPedSquare_ConfigValid(cfg)) {
        return WT_PED_SQUARE_ERR_CONFIG;
    }
    if (!WtPedSquare_GeoLooksValid(base)) {
        return WT_PED_SQUARE_ERR_BASE;
    }
    if (!WtPedSquare_GeoLooksValid(aircraft)) {
        return WT_PED_SQUARE_ERR_AIRCRAFT;
    }

    out->base = *base;

    /*
     * 只做水平几何：高度全部是「相对起飞点」，由 relHeightM 承载。
     * 把两个点的 alt 拉平，免得基准不明的激光海拔（见 wt_laser.h）混进来。
     */
    baseFlat = *base;
    baseFlat.alt = 0.0;
    aircraftFlat = *aircraft;
    aircraftFlat.alt = 0.0;
    frame = WtLocalFrame_Init(&baseFlat);
    ac = WtGeo_ToEnu(&frame, &aircraftFlat);

    out->aircraftToBaseM = WtPedSquare_Horizontal(ac);
    if (out->aircraftToBaseM > cfg->maxBaseDistanceM) {
        return WT_PED_SQUARE_ERR_TOO_FAR;
    }

    /*
     * 第一个顶点 = 离飞机最近的那个。距离相等（飞机正好在基点上方）时取
     * 列表里靠前的，结果确定、可复现。
     */
    half = cfg->sideM / 2.0;
    for (k = 0; k < WT_PED_SQUARE_CORNERS; k++) {
        WtEnu c;
        double d;

        c.e = s_cornerSign[k][0] * half;
        c.n = s_cornerSign[k][1] * half;
        c.u = 0.0;
        d = WtPedSquare_Horizontal(WtEnu_Sub(c, ac));
        if (k == 0 || d < bestDist - 1e-9) {
            bestDist = d;
            first = k;
        }
    }
    out->transitM = bestDist;

    /* 按飞行顺序（从第一个顶点起顺时针）排好四个顶点 */
    for (k = 0; k < WT_PED_SQUARE_CORNERS; k++) {
        int src = (first + k) % WT_PED_SQUARE_CORNERS;

        out->cornerEnu[k].e = s_cornerSign[src][0] * half;
        out->cornerEnu[k].n = s_cornerSign[src][1] * half;
        out->cornerEnu[k].u = 0.0;
        out->cornerBearingDeg[k] = WtEnu_Azimuth(out->cornerEnu[k]);
    }

    /* c0 低 →（每条边）爬升 → 平飞到下一顶点 → 下降。最后一条边回到 c0 */
    WtPedSquare_Put(out, &frame, 0, cfg->lowHeightM, WT_PED_SQUARE_STEP_START);
    for (k = 0; k < WT_PED_SQUARE_CORNERS; k++) {
        int next = (k + 1) % WT_PED_SQUARE_CORNERS;

        WtPedSquare_Put(out, &frame, k, cfg->highHeightM, WT_PED_SQUARE_STEP_CLIMB);
        WtPedSquare_Put(out, &frame, next, cfg->highHeightM, WT_PED_SQUARE_STEP_CRUISE);
        WtPedSquare_Put(out, &frame, next, cfg->lowHeightM, WT_PED_SQUARE_STEP_DESCEND);
    }

    for (k = 1; k < out->count; k++) {
        out->pathLengthM += WtEnu_Distance(out->wp[k].enu, out->wp[k - 1].enu);
    }
    out->durationS = out->pathLengthM / cfg->speedMs;
    out->rthHeightM = (cfg->rthHeightM > cfg->highHeightM) ? cfg->rthHeightM : cfg->highHeightM;

    return WT_PED_SQUARE_OK;
}

const char *WtPedSquare_ResultText(E_WtPedSquareResult r)
{
    switch (r) {
    case WT_PED_SQUARE_OK:
        return "成功";
    case WT_PED_SQUARE_ERR_CONFIG:
        return "航线参数非法";
    case WT_PED_SQUARE_ERR_BASE:
        return "基点坐标非法";
    case WT_PED_SQUARE_ERR_AIRCRAFT:
        return "飞机位置非法";
    case WT_PED_SQUARE_ERR_TOO_FAR:
        return "基点离飞机太远";
    default:
        return "未知错误";
    }
}

const char *WtPedSquare_StepName(E_WtPedSquareStep step)
{
    switch (step) {
    case WT_PED_SQUARE_STEP_START:
        return "起点";
    case WT_PED_SQUARE_STEP_CLIMB:
        return "爬升";
    case WT_PED_SQUARE_STEP_CRUISE:
        return "平飞";
    case WT_PED_SQUARE_STEP_DESCEND:
        return "下降";
    default:
        return "?";
    }
}
