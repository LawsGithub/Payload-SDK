/**
 * @file wt_projection.c
 * @brief 像素 ↔ 三维的反投影实现。
 *
 * 纯数学，不依赖 PSDK，可在桌面上用已知答案的用例验证。
 */

#include "wt_projection.h"

#include <math.h>
#include <stddef.h>

/* ------------------------------------------------------------------ */
/* 内参                                                                */
/* ------------------------------------------------------------------ */

bool WtProjection_SetupIntrinsics(int imageWidthPx, int imageHeightPx,
                                  double fovHorizDeg, double fovVertDeg,
                                  double *outTanHalfH, double *outTanHalfV)
{
    if (outTanHalfH == NULL || outTanHalfV == NULL) {
        return false;
    }
    if (imageWidthPx <= 0 || imageHeightPx <= 0) {
        return false;
    }
    if (fovHorizDeg <= 0.0 || fovHorizDeg >= 180.0 ||
        fovVertDeg <= 0.0 || fovVertDeg >= 180.0) {
        return false;
    }

    *outTanHalfH = tan(Wt_DegToRad(fovHorizDeg) * 0.5);
    *outTanHalfV = tan(Wt_DegToRad(fovVertDeg) * 0.5);

    return true;
}

bool WtProjection_PixelToCameraRay(double u, double v,
                                   int widthPx, int heightPx,
                                   double tanHalfH, double tanHalfV,
                                   WtEnu *out)
{
    double nx;
    double ny;
    double len;

    if (out == NULL || widthPx <= 0 || heightPx <= 0) {
        return false;
    }

    /*
     * 归一化到 [-1, 1]：图像中心对应 0。
     * 注意 v 轴向下，而相机系 y 也向下，所以这里不取负 ——
     * 坐标系约定在 WtCameraPose 的注释里写明了。
     */
    nx = (u - (double)widthPx * 0.5) / ((double)widthPx * 0.5) * tanHalfH;
    ny = (v - (double)heightPx * 0.5) / ((double)heightPx * 0.5) * tanHalfV;

    out->e = nx;   /* 相机系 x：右 */
    out->n = ny;   /* 相机系 y：下 */
    out->u = 1.0;  /* 相机系 z：前（光轴） */

    len = WtEnu_Length(*out);
    if (len <= 0.0) {
        return false;
    }
    *out = WtEnu_Scale(*out, 1.0 / len);

    return true;
}

/* ------------------------------------------------------------------ */
/* 姿态变换                                                            */
/* ------------------------------------------------------------------ */

WtEnu WtProjection_CameraToEnu(const WtCameraPose *pose, WtEnu dirCam)
{
    WtEnu v;
    WtEnu right;
    WtEnu down;
    WtEnu forward;
    const double yaw = Wt_DegToRad(pose->yawDeg);
    const double pitch = Wt_DegToRad(pose->pitchDeg);
    const double roll = Wt_DegToRad(pose->rollDeg);
    const double cp = cos(pitch);
    const double sp = sin(pitch);
    const double cy = cos(yaw);
    const double sy = sin(yaw);

    /*
     * 先构造「无滚转」时的三个基矢量（ENU 分量）：
     *
     *   forward = (sy*cp, cy*cp, sp)        光轴，yaw 自北顺时针
     *   right   = (cy,   -sy,   0)          水平右向
     *   down    = forward × right 的负向 …  这里直接由叉积得到，保证正交
     *
     * 再绕 forward 转 roll 角，把 right/down 转到位。
     */
    forward.e = sy * cp;
    forward.n = cy * cp;
    forward.u = sp;

    right.e = cy;
    right.n = -sy;
    right.u = 0.0;

    /* down = right × forward（右手系：x右, y下, z前 → y = z × x） */
    down = WtEnu_Cross(forward, right);

    /* 应用滚转：绕光轴旋转 right 与 down */
    if (fabs(roll) > 1e-12) {
        right = WtEnu_RotateAround(right, forward, roll);
        down = WtEnu_RotateAround(down, forward, roll);
    }

    /* dirCam 的分量：e→right, n→down, u→forward */
    v = WtEnu_Add(WtEnu_Scale(right, dirCam.e),
                  WtEnu_Add(WtEnu_Scale(down, dirCam.n),
                            WtEnu_Scale(forward, dirCam.u)));

    return WtEnu_Normalize(v);
}

/* ------------------------------------------------------------------ */
/* 求交                                                                */
/* ------------------------------------------------------------------ */

bool WtProjection_RayPlaneIntersect(WtEnu rayOrigin, WtEnu rayDir,
                                    WtEnu planePoint, WtEnu planeNormal,
                                    WtProjectionResult *out)
{
    const double denom = WtEnu_Dot(rayDir, planeNormal);
    WtEnu diff;
    double t;

    if (out == NULL) {
        return false;
    }
    out->hit = false;
    out->distanceM = 0.0;

    /*
     * 射线与平面近乎平行时无解。阈值取 1e-6：对应夹角约 0.00006°，
     * 比任何实际姿态误差都小得多，不会误杀正常情况。
     */
    if (fabs(denom) < 1e-6) {
        return false;
    }

    diff = WtEnu_Sub(planePoint, rayOrigin);
    t = WtEnu_Dot(diff, planeNormal) / denom;

    /* 交点在射线反方向 → 目标在相机背后，视为未命中 */
    if (t <= 0.0) {
        return false;
    }

    out->point = WtEnu_Add(rayOrigin, WtEnu_Scale(rayDir, t));
    out->distanceM = t * WtEnu_Length(rayDir);
    out->hit = true;

    return true;
}

bool WtProjection_PixelToRotorPlane(const WtCameraPose *pose,
                                    double u, double v,
                                    int widthPx, int heightPx,
                                    double fovHorizDeg, double fovVertDeg,
                                    const WtRotorFrame *rotorFrame,
                                    WtProjectionResult *out)
{
    double tanH = 0.0;
    double tanV = 0.0;
    WtEnu dirCam;
    WtEnu dirEnu;

    if (pose == NULL || rotorFrame == NULL || out == NULL) {
        return false;
    }
    if (!WtProjection_SetupIntrinsics(widthPx, heightPx, fovHorizDeg, fovVertDeg,
                                      &tanH, &tanV)) {
        return false;
    }
    if (!WtProjection_PixelToCameraRay(u, v, widthPx, heightPx, tanH, tanV, &dirCam)) {
        return false;
    }

    dirEnu = WtProjection_CameraToEnu(pose, dirCam);

    return WtProjection_RayPlaneIntersect(pose->position, dirEnu,
                                          rotorFrame->rotorCenter,
                                          rotorFrame->rotAxis, out);
}

/* ------------------------------------------------------------------ */
/* 盘面内坐标                                                          */
/* ------------------------------------------------------------------ */

bool WtProjection_PointToRotorCoords(const WtRotorFrame *rotorFrame,
                                     WtEnu point,
                                     double *outRadialM,
                                     double *outAngleDeg)
{
    WtEnu rel;
    double alongUp;
    double alongRot;

    if (rotorFrame == NULL || outRadialM == NULL || outAngleDeg == NULL) {
        return false;
    }

    rel = WtEnu_Sub(point, rotorFrame->rotorCenter);

    /*
     * 投影到盘面内的两个基矢量上：
     *   upRef  = 12 点钟方向
     *   rotRef = 切向，ψ 增大方向
     *
     * 相位角从 upRef 起、沿 rotRef 方向量 —— 与 WtRotorFrame 的约定一致。
     */
    alongUp = WtEnu_Dot(rel, rotorFrame->upRef);
    alongRot = WtEnu_Dot(rel, rotorFrame->rotRef);

    *outRadialM = sqrt(alongUp * alongUp + alongRot * alongRot);
    *outAngleDeg = Wt_Wrap360(Wt_RadToDeg(atan2(alongRot, alongUp)));

    return true;
}
