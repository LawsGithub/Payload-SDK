/**
 * @file lz_geo.c
 * @brief 大地几何工具的实现（球面近似）。
 */

#include "lz_geo.h"

#include <math.h>

#define LZ_DEG2RAD (M_PI / 180.0)
#define LZ_RAD2DEG (180.0 / M_PI)

/* 工程上常见的球面公式都用到了 atan2/asin 这类对定义域敏感的数学函数，
 * 浮点误差可能把参数推出 [-1,1]，所以处处夹紧。 */

static double lz_clamp_unit(double x)
{
    if (x > 1.0) {
        return 1.0;
    }
    if (x < -1.0) {
        return -1.0;
    }
    return x;
}

double LzGeo_DistanceM(const LzGeo *a, const LzGeo *b)
{
    if (a == NULL || b == NULL) {
        return 0.0;
    }

    const double lat1 = a->latitudeDeg * LZ_DEG2RAD;
    const double lat2 = b->latitudeDeg * LZ_DEG2RAD;
    const double dLat = lat2 - lat1;
    const double dLon = (b->longitudeDeg - a->longitudeDeg) * LZ_DEG2RAD;

    /* haversine：小距离下比余弦公式数值稳定得多 */
    const double sLat = sin(dLat * 0.5);
    const double sLon = sin(dLon * 0.5);
    const double h = sLat * sLat + cos(lat1) * cos(lat2) * sLon * sLon;

    return 2.0 * LZ_EARTH_RADIUS_M * asin(lz_clamp_unit(sqrt(h)));
}

double LzGeo_BearingDeg(const LzGeo *a, const LzGeo *b)
{
    if (a == NULL || b == NULL) {
        return 0.0;
    }

    const double lat1 = a->latitudeDeg * LZ_DEG2RAD;
    const double lat2 = b->latitudeDeg * LZ_DEG2RAD;
    const double dLon = (b->longitudeDeg - a->longitudeDeg) * LZ_DEG2RAD;

    const double y = sin(dLon) * cos(lat2);
    const double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

    return LzGeo_NormalizeDeg(atan2(y, x) * LZ_RAD2DEG);
}

LzStatus LzGeo_Destination(const LzGeo *from, double bearingDeg, double distanceM, LzGeo *out)
{
    if (from == NULL || out == NULL) {
        return LZ_ERR_PARAM;
    }
    if (!LzGeo_IsValid(from) || !isfinite(bearingDeg) || !isfinite(distanceM)) {
        return LZ_ERR_PARAM;
    }

    const double lat1 = from->latitudeDeg * LZ_DEG2RAD;
    const double lon1 = from->longitudeDeg * LZ_DEG2RAD;
    const double theta = bearingDeg * LZ_DEG2RAD;
    const double delta = distanceM / LZ_EARTH_RADIUS_M;

    const double sinLat1 = sin(lat1);
    const double cosLat1 = cos(lat1);
    const double sinDelta = sin(delta);
    const double cosDelta = cos(delta);

    const double sinLat2 = sinLat1 * cosDelta + cosLat1 * sinDelta * cos(theta);
    const double lat2 = asin(lz_clamp_unit(sinLat2));
    const double lon2 = lon1 + atan2(sin(theta) * sinDelta * cosLat1,
                                     cosDelta - sinLat1 * sinLat2);

    out->latitudeDeg = lat2 * LZ_RAD2DEG;
    /* 经度归一化到 (-180,180]，跨越 180° 经线时不做这一步会得到 190° 这种值 */
    out->longitudeDeg = LzGeo_NormalizeDeg(lon2 * LZ_RAD2DEG + 540.0) - 180.0;
    out->altitudeM = from->altitudeM;

    return LZ_OK;
}

double LzGeo_NormalizeDeg(double deg)
{
    if (!isfinite(deg)) {
        return 0.0;
    }
    double r = fmod(deg, 360.0);
    if (r < 0.0) {
        r += 360.0;
    }
    /* 浮点残留会让 r 恰好落在 360.0 上：当 deg 是一个极小的负数（例如
     * atan2 在正北方向返回的 -1e-17），r += 360.0 会舍入成 360.0 而不是
     * 359.999…，于是区间变成左闭右闭。归一化的契约是 [0,360)，
     * 这里把右端点收回来，否则调用方拿 360.0 去算夹角会得到偏差 360°。 */
    if (r >= 360.0) {
        r -= 360.0;
    }
    return r;
}

double LzGeo_AngleDiffDeg(double a, double b)
{
    double d = LzGeo_NormalizeDeg(a - b);
    if (d > 180.0) {
        d -= 360.0;
    }
    return d;
}
