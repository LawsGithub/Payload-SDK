/**
 * @file wt_camera.c
 * @brief 相机成像模型实现
 */

#include "wt_camera.h"

#include <math.h>
#include <stddef.h>

#include "wt_geometry.h" /* 复用全局的度/弧度换算常数，保持全工程一致 */

const WtCameraModel WT_CAMERA_M4T_WIDE = {"M4T Wide 24mm", 24.0, 8000, 6000};
const WtCameraModel WT_CAMERA_M4T_MID = {"M4T Mid-Tele 70mm", 70.0, 8000, 6000};
const WtCameraModel WT_CAMERA_M4T_TELE = {"M4T Tele 168mm", 168.0, 8000, 6000};

WtCameraGeometry WtCamera_ComputeGeometry(const WtCameraModel *cam)
{
    WtCameraGeometry g = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double diagPx;

    if (cam == NULL || cam->eflMm <= 0.0 || cam->imageWidthPx <= 0 || cam->imageHeightPx <= 0) {
        return g;
    }

    g.sensorDiagMm = WT_CAMERA_FF_DIAG_MM;
    g.sensorWidthMm = WT_CAMERA_FF_DIAG_MM * (double)cam->imageWidthPx /
                      sqrt((double)cam->imageWidthPx * cam->imageWidthPx +
                           (double)cam->imageHeightPx * cam->imageHeightPx);
    g.sensorHeightMm = g.sensorWidthMm * (double)cam->imageHeightPx / (double)cam->imageWidthPx;

    g.fovDiagDeg = 2.0 * atan(g.sensorDiagMm / (2.0 * cam->eflMm)) * WT_RAD2DEG;
    g.fovHorizDeg = 2.0 * atan(g.sensorWidthMm / (2.0 * cam->eflMm)) * WT_RAD2DEG;
    g.fovVertDeg = 2.0 * atan(g.sensorHeightMm / (2.0 * cam->eflMm)) * WT_RAD2DEG;

    /* 对角线上共有 sqrt(w^2 + h^2) 个像素 */
    diagPx = sqrt((double)cam->imageWidthPx * cam->imageWidthPx +
                  (double)cam->imageHeightPx * cam->imageHeightPx);
    g.pixelPitchUm = g.sensorDiagMm * 1000.0 / diagPx;

    return g;
}

double WtCamera_GsdAtDistance(const WtCameraModel *cam, double distanceM)
{
    WtCameraGeometry g;

    if (cam == NULL || cam->eflMm <= 0.0 || distanceM <= 0.0) {
        return 0.0;
    }

    g = WtCamera_ComputeGeometry(cam);

    /*
     * GSD = 像元尺寸 × 距离 / 焦距。
     * 像元与焦距用 mm 表达（像元为 µm，故乘 1e-3），距离由 m 换算为 mm。
     */
    return (g.pixelPitchUm * 0.001) * (distanceM * 1000.0) / cam->eflMm;
}

double WtCamera_MaxDistanceForGsd(const WtCameraModel *cam, double targetGsdMmPerPx)
{
    WtCameraGeometry g;

    if (cam == NULL || cam->eflMm <= 0.0 || targetGsdMmPerPx <= 0.0) {
        return 0.0;
    }

    g = WtCamera_ComputeGeometry(cam);

    /* 上式的反解，结果单位为 m */
    return targetGsdMmPerPx * cam->eflMm / (g.pixelPitchUm * 0.001) / 1000.0;
}

double WtCamera_CoverageHeightAt(const WtCameraModel *cam, double distanceM)
{
    WtCameraGeometry g = WtCamera_ComputeGeometry(cam);

    if (g.fovVertDeg <= 0.0) {
        return 0.0;
    }

    return 2.0 * distanceM * tan(g.fovVertDeg * 0.5 / WT_RAD2DEG);
}

double WtCamera_CoverageWidthAt(const WtCameraModel *cam, double distanceM)
{
    WtCameraGeometry g = WtCamera_ComputeGeometry(cam);

    if (g.fovHorizDeg <= 0.0) {
        return 0.0;
    }

    return 2.0 * distanceM * tan(g.fovHorizDeg * 0.5 / WT_RAD2DEG);
}

double WtCamera_SpacingForOverlap(const WtCameraModel *cam, double distanceM,
                                  double overlapPct, bool alongHorizontal)
{
    double coverage;

    if (overlapPct < 0.0 || overlapPct >= 100.0) {
        return 0.0;
    }

    coverage = alongHorizontal ? WtCamera_CoverageWidthAt(cam, distanceM)
                               : WtCamera_CoverageHeightAt(cam, distanceM);
    if (coverage <= 0.0) {
        return 0.0;
    }

    /* 重叠率 o：相邻画幅搭接长度为 o×coverage，间距即剩余部分 */
    return coverage * (1.0 - overlapPct / 100.0);
}

double WtCamera_OverlapForSpacing(const WtCameraModel *cam, double distanceM,
                                  double spacingM, bool alongHorizontal)
{
    double coverage = alongHorizontal ? WtCamera_CoverageWidthAt(cam, distanceM)
                                      : WtCamera_CoverageHeightAt(cam, distanceM);

    if (coverage <= 0.0 || spacingM < 0.0) {
        return 0.0;
    }

    return (1.0 - spacingM / coverage) * 100.0;
}