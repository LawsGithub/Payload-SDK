/**
 * @file wt_geometry.c
 * @brief 坐标与几何基础库实现
 */

#include "wt_geometry.h"

#include <math.h>
#include <stddef.h>

WtLocalFrame WtLocalFrame_Init(const WtGeo *origin)
{
    WtLocalFrame frame;
    double sinLat;
    double w;

    frame.origin = *origin;

    /*
     * 卯酉圈半径 N = a / sqrt(1 - e^2 sin^2(lat))
     * 子午圈半径 M = a(1 - e^2) / (1 - e^2 sin^2(lat))^1.5
     */
    sinLat = sin(origin->lat * WT_DEG2RAD);
    w = sqrt(1.0 - WT_EARTH_E2 * sinLat * sinLat);
    frame.radiusN = WT_EARTH_A / w;
    frame.radiusM = WT_EARTH_A * (1.0 - WT_EARTH_E2) / (w * w * w);
    frame.cosLat = cos(origin->lat * WT_DEG2RAD);

    return frame;
}

WtEnu WtGeo_ToEnu(const WtLocalFrame *frame, const WtGeo *geo)
{
    WtEnu out;

    out.e = (geo->lon - frame->origin.lon) * WT_DEG2RAD * frame->radiusN * frame->cosLat;
    out.n = (geo->lat - frame->origin.lat) * WT_DEG2RAD * frame->radiusM;
    out.u = geo->alt - frame->origin.alt;

    return out;
}

WtGeo WtEnu_ToGeo(const WtLocalFrame *frame, const WtEnu *enu)
{
    WtGeo out;
    double lat;

    /* 一次迭代：先用原点纬度反算，再用新纬度修正东向换算 */
    lat = frame->origin.lat + (enu->n / frame->radiusM) * WT_RAD2DEG;
    out.lat = lat;
    out.lon = frame->origin.lon +
              (enu->e / (frame->radiusN * cos(lat * WT_DEG2RAD))) * WT_RAD2DEG;
    out.alt = frame->origin.alt + enu->u;

    return out;
}

double WtGeo_Distance(const WtGeo *a, const WtGeo *b)
{
    WtLocalFrame frame = WtLocalFrame_Init(a);
    WtEnu d = WtGeo_ToEnu(&frame, b);

    return sqrt(d.e * d.e + d.n * d.n + d.u * d.u);
}

WtEnu WtEnu_Add(WtEnu a, WtEnu b)
{
    WtEnu r = {a.e + b.e, a.n + b.n, a.u + b.u};
    return r;
}

WtEnu WtEnu_Sub(WtEnu a, WtEnu b)
{
    WtEnu r = {a.e - b.e, a.n - b.n, a.u - b.u};
    return r;
}

WtEnu WtEnu_Scale(WtEnu a, double k)
{
    WtEnu r = {a.e * k, a.n * k, a.u * k};
    return r;
}

double WtEnu_Dot(WtEnu a, WtEnu b)
{
    return a.e * b.e + a.n * b.n + a.u * b.u;
}

WtEnu WtEnu_Cross(WtEnu a, WtEnu b)
{
    WtEnu r;
    r.e = a.n * b.u - a.u * b.n;
    r.n = a.u * b.e - a.e * b.u;
    r.u = a.e * b.n - a.n * b.e;
    return r;
}

double WtEnu_Length(WtEnu a)
{
    return sqrt(WtEnu_Dot(a, a));
}

double WtEnu_Distance(WtEnu a, WtEnu b)
{
    return WtEnu_Length(WtEnu_Sub(a, b));
}

WtEnu WtEnu_Normalize(WtEnu a)
{
    double len = WtEnu_Length(a);
    WtEnu r = {0.0, 0.0, 0.0};

    if (len > 1e-12) {
        r.e = a.e / len;
        r.n = a.n / len;
        r.u = a.u / len;
    }

    return r;
}

WtEnu WtEnu_RotateAround(WtEnu v, WtEnu axisUnit, double angleRad)
{
    /* 罗德里格斯： v' = v cosθ + (k × v) sinθ + k (k·v)(1 - cosθ) */
    double c = cos(angleRad);
    double s = sin(angleRad);
    WtEnu kv = WtEnu_Cross(axisUnit, v);
    double kd = WtEnu_Dot(axisUnit, v);
    WtEnu r;

    r.e = v.e * c + kv.e * s + axisUnit.e * kd * (1.0 - c);
    r.n = v.n * c + kv.n * s + axisUnit.n * kd * (1.0 - c);
    r.u = v.u * c + kv.u * s + axisUnit.u * kd * (1.0 - c);

    return r;
}

WtEnu WtEnu_FromAzEl(double azimuthDeg, double elevationDeg)
{
    double az = azimuthDeg * WT_DEG2RAD;
    double el = elevationDeg * WT_DEG2RAD;
    double cosEl = cos(el);
    WtEnu r;

    r.e = sin(az) * cosEl;
    r.n = cos(az) * cosEl;
    r.u = sin(el);

    return r;
}

double WtEnu_Azimuth(WtEnu v)
{
    return Wt_Wrap360(atan2(v.e, v.n) * WT_RAD2DEG);
}

double WtEnu_Elevation(WtEnu v)
{
    double horiz = sqrt(v.e * v.e + v.n * v.n);

    return atan2(v.u, horiz) * WT_RAD2DEG;
}

double Wt_Clamp(double v, double lo, double hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }

    return v;
}

double Wt_DegToRad(double deg)
{
    return deg * WT_DEG2RAD;
}

double Wt_RadToDeg(double rad)
{
    return rad * WT_RAD2DEG;
}

double Wt_Wrap360(double deg)
{
    double r = fmod(deg, 360.0);

    if (r < 0.0) {
        r += 360.0;
    }

    return r;
}

double Wt_AngleDiff(double a, double b)
{
    double d = Wt_Wrap360(a - b);

    if (d > 180.0) {
        d -= 360.0;
    }

    return d;
}

double Wt_Lerp(double a, double b, double t)
{
    return a + (b - a) * t;
}