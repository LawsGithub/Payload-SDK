/**
 * @file wt_turbine.c
 * @brief 风机几何模型与风轮运动学实现
 */

#include "wt_turbine.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define WT_TURBINE_MIN_HUB_MARGIN 2.0 /*!< 轮毂半径需比叶轮半径小多少 m 以上 */

WtValidateResult WtTurbine_Validate(const WtTurbineSpec *spec)
{
    WtValidateResult r = {true, "ok"};

    if (spec == NULL) {
        r.ok = false;
        r.message = "spec is null";
        return r;
    }
    if (spec->bladeCount < 1 || spec->bladeCount > WT_MAX_BLADES) {
        r.ok = false;
        r.message = "bladeCount out of range";
        return r;
    }
    if (spec->rotorDiameter <= 0.0 || spec->hubHeight <= 0.0) {
        r.ok = false;
        r.message = "rotorDiameter / hubHeight must be positive";
        return r;
    }
    if (spec->hubRadius <= 0.0 ||
        spec->hubRadius > spec->rotorDiameter * 0.5 - WT_TURBINE_MIN_HUB_MARGIN) {
        r.ok = false;
        r.message = "hubRadius inconsistent with rotorDiameter";
        return r;
    }
    if (spec->towerBottomDia <= 0.0 || spec->towerTopDia <= 0.0) {
        r.ok = false;
        r.message = "tower diameter must be positive";
        return r;
    }
    if (spec->coneAngleDeg < 0.0 || spec->coneAngleDeg >= 45.0) {
        r.ok = false;
        r.message = "coneAngleDeg out of range";
        return r;
    }
    if (spec->tiltDeg < -30.0 || spec->tiltDeg > 30.0) {
        r.ok = false;
        r.message = "tiltDeg out of range";
        return r;
    }
    if (spec->prebendM < 0.0) {
        r.ok = false;
        r.message = "prebendM must not be negative";
        return r;
    }

    return r;
}

WtEnu WtTurbine_GetRotorCenter(const WtTurbineSpec *spec)
{
    WtLocalFrame frame = WtLocalFrame_Init(&spec->base);
    WtGeo baseEnu = {spec->base.lat, spec->base.lon, spec->base.alt};
    WtEnu center = WtGeo_ToEnu(&frame, &baseEnu);
    double h = spec->headingDeg * WT_DEG2RAD;

    /* 塔筒轴线上 hubHeight 高处，再沿机舱朝向水平前伸 */
    center.u += spec->hubHeight;
    center.e += sin(h) * spec->nacelleOffset;
    center.n += cos(h) * spec->nacelleOffset;

    return center;
}

WtRotorFrame WtTurbine_BuildRotorFrame(const WtTurbineSpec *spec, double phaseDeg)
{
    WtRotorFrame frame;
    WtEnu up = {0.0, 0.0, 1.0};
    WtEnu forward;   /* 机舱朝向（来流来向）水平单位矢量 */
    WtEnu axisHoriz; /* 未加仰角时的主轴方向，指向下游 */
    WtEnu pivot;     /* 施加仰角所用的水平旋转轴 */
    double h = spec->headingDeg * WT_DEG2RAD;
    double upDotAxis;

    frame.phaseDeg = phaseDeg;
    frame.rotorCenter = WtTurbine_GetRotorCenter(spec);

    forward.e = sin(h);
    forward.n = cos(h);
    forward.u = 0.0;
    axisHoriz = WtEnu_Scale(forward, -1.0);

    /*
     * 主轴仰角：绕「竖直方向 × 机舱朝向」所得的水平轴旋转。
     * 该旋转轴与机舱朝向正交，因此旋转只改变俯仰、不改变偏航。
     * 取 up × forward 使 tiltDeg > 0 时主轴向上抬起。
     */
    pivot = WtEnu_Normalize(WtEnu_Cross(up, forward));
    frame.rotAxis = WtEnu_RotateAround(axisHoriz, pivot, spec->tiltDeg * WT_DEG2RAD);

    /*
     * 风轮平面内的「12 点钟」方向：把全局竖直方向对旋转轴做正交化。
     * tiltDeg = 0 时恰好等于全局 up；有仰角时仍在风轮平面内且指向最高点。
     */
    upDotAxis = WtEnu_Dot(up, frame.rotAxis);
    frame.upRef = WtEnu_Normalize(WtEnu_Sub(up, WtEnu_Scale(frame.rotAxis, upDotAxis)));

    /*
     * 相位角增大方向：自上游（迎风侧）观察时的顺时针方向。
     * 观察者视线方向即 rotAxis，其右手边为 rotAxis × upRef。
     */
    frame.rotRef = WtEnu_Normalize(WtEnu_Cross(frame.rotAxis, frame.upRef));

    return frame;
}

WtEnu WtTurbine_BladeRadialDir(const WtRotorFrame *frame, int bladeIndex, const WtTurbineSpec *spec)
{
    double ang = frame->phaseDeg + (double)bladeIndex * (360.0 / (double)spec->bladeCount);
    double rad = ang * WT_DEG2RAD;

    return WtEnu_Add(WtEnu_Scale(frame->upRef, cos(rad)),
                     WtEnu_Scale(frame->rotRef, sin(rad)));
}

WtEnu WtTurbine_BladeAxisDir(const WtRotorFrame *frame, int bladeIndex, const WtTurbineSpec *spec)
{
    WtEnu radial = WtTurbine_BladeRadialDir(frame, bladeIndex, spec);
    double cone = spec->coneAngleDeg * WT_DEG2RAD;

    /* 锥角把叶片拉向迎风侧，即 -rotAxis 方向 */
    return WtEnu_Sub(WtEnu_Scale(radial, cos(cone)),
                     WtEnu_Scale(frame->rotAxis, sin(cone)));
}

WtEnu WtTurbine_BladePoint(const WtRotorFrame *frame, const WtTurbineSpec *spec,
                           int bladeIndex, double radialFrac)
{
    WtEnu axisDir = WtTurbine_BladeAxisDir(frame, bladeIndex, spec);
    double span = spec->rotorDiameter * 0.5 - spec->hubRadius; /* 叶根到叶尖的展向长度 */
    double t = Wt_Clamp(radialFrac, 0.0, 1.0);
    WtEnu p = frame->rotorCenter;

    /* 沿叶片轴线伸出 */
    p = WtEnu_Add(p, WtEnu_Scale(axisDir, spec->hubRadius + span * t));

    /* 预弯：叶尖朝迎风侧偏出，用二次分布近似（叶根处无偏移） */
    p = WtEnu_Sub(p, WtEnu_Scale(frame->rotAxis, spec->prebendM * t * t));

    return p;
}

WtEnu WtTurbine_BladeTip(const WtRotorFrame *frame, const WtTurbineSpec *spec, int bladeIndex)
{
    return WtTurbine_BladePoint(frame, spec, bladeIndex, 1.0);
}

double WtTurbine_TowerRadiusAt(const WtTurbineSpec *spec, double heightAboveBase)
{
    double t = Wt_Clamp(heightAboveBase / spec->hubHeight, 0.0, 1.0);

    return 0.5 * Wt_Lerp(spec->towerBottomDia, spec->towerTopDia, t);
}

bool WtTurbine_PhaseFromTip(const WtTurbineSpec *spec, WtEnu tipEnu, int bladeIndex, double *outPhase)
{
    WtRotorFrame frame;
    WtEnu v;
    WtEnu vPlane;
    double alongAxis;
    double cosC;
    double sinC;
    double psiBlade;

    if (outPhase == NULL) {
        return false;
    }

    /* 参考系的方向部分与相位角无关，可任取一个相位角构建 */
    frame = WtTurbine_BuildRotorFrame(spec, 0.0);

    v = WtEnu_Sub(tipEnu, frame.rotorCenter);
    alongAxis = WtEnu_Dot(v, frame.rotAxis);
    vPlane = WtEnu_Sub(v, WtEnu_Scale(frame.rotAxis, alongAxis));

    if (WtEnu_Length(vPlane) < 1e-6) {
        return false; /* 点退化到风轮轴上，无法定相位 */
    }

    /* 把平面内矢量分解到 (upRef, rotRef) 基上，直接得到叶片方位角 */
    cosC = WtEnu_Dot(vPlane, frame.upRef);
    sinC = WtEnu_Dot(vPlane, frame.rotRef);
    psiBlade = Wt_Wrap360(atan2(sinC, cosC) * WT_RAD2DEG);

    *outPhase = Wt_Wrap360(psiBlade - (double)bladeIndex * (360.0 / (double)spec->bladeCount));

    return true;
}

double WtTurbine_AdvancePhase(const WtTurbineSpec *spec, double phaseDeg, double rpm, double dtSec)
{
    (void)spec;

    return Wt_Wrap360(phaseDeg + rpm * 360.0 * dtSec / 60.0);
}