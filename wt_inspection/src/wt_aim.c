/**
 * @file wt_aim.c
 * @brief 像素 → 云台绝对指向的换算实现。
 */

#include "wt_aim.h"

#include <math.h>
#include <stddef.h>

bool WtAim_FromPixel(const WtCameraPose *gimbal,
                     double u, double v,
                     int widthPx, int heightPx,
                     double fovHorizDeg, double fovVertDeg,
                     WtAimResult *out)
{
    double tanH = 0.0;
    double tanV = 0.0;
    WtEnu dirCam;
    WtEnu dirEnu;
    WtEnu axisEnu;
    double dot;

    if (out == NULL) {
        return false;
    }
    out->ok = false;
    out->targetYawDeg = 0.0;
    out->targetPitchDeg = 0.0;
    out->yawErrorDeg = 0.0;
    out->pitchErrorDeg = 0.0;
    out->angularErrorDeg = 0.0;

    if (gimbal == NULL) {
        return false;
    }
    if (!WtProjection_SetupIntrinsics(widthPx, heightPx, fovHorizDeg, fovVertDeg,
                                      &tanH, &tanV)) {
        return false;
    }
    if (!WtProjection_PixelToCameraRay(u, v, widthPx, heightPx, tanH, tanV, &dirCam)) {
        return false;
    }

    /*
     * 走完整的方向矢量变换，而不是把画面内偏角加到云台角上。
     * 复用 WtProjection_CameraToEnu —— 相机系到 ENU 的约定全工程只有这一份
     * 实现，两处各写一遍必然漂移（而症状是"云台总差一点点"）。
     */
    dirEnu = WtProjection_CameraToEnu(gimbal, dirCam);
    if (WtEnu_Length(dirEnu) < 1e-9) {
        return false;
    }

    out->targetYawDeg = WtEnu_Azimuth(dirEnu);
    out->targetPitchDeg = WtEnu_Elevation(dirEnu);

    /*
     * 两轴的误差分开报：yaw 要机头承担（M4T 云台 Pan 不可控），
     * pitch 才是云台能吃的。
     *
     * yaw 用 Wt_AngleDiff 取最短转向（±180 内），否则从 350° 转到 10°
     * 会被算成 -340° 这种没人会去执行的值。
     */
    out->yawErrorDeg = Wt_AngleDiff(out->targetYawDeg, gimbal->yawDeg);
    out->pitchErrorDeg = out->targetPitchDeg - gimbal->pitchDeg;

    /*
     * 总夹角用方向矢量的点积算，而不是 sqrt(yawErr² + pitchErr²)。
     *
     * 后者在高俯仰角下会严重高估：云台已经俯视 60° 时，10° 的方位偏差对应的
     * 实际视线夹角只有约 5°。用它当"是否对准"的判据，会在俯视叶尖时
     * 永远判为未对准、反复重试到超次数 —— 而画面里目标其实已经在中心了。
     */
    axisEnu = WtEnu_FromAzEl(gimbal->yawDeg, gimbal->pitchDeg);
    dot = Wt_Clamp(WtEnu_Dot(WtEnu_Normalize(dirEnu), axisEnu), -1.0, 1.0);
    out->angularErrorDeg = Wt_RadToDeg(acos(dot));

    out->ok = true;

    return true;
}

bool WtAim_IsOnTarget(const WtAimResult *aim, double toleranceDeg)
{
    if (aim == NULL || !aim->ok || toleranceDeg <= 0.0) {
        return false;
    }

    return aim->angularErrorDeg <= toleranceDeg;
}

bool WtAim_SortTipsByImageAngle(WtAiDetection *det, double *outAngles)
{
    double angles[WT_BLADE_ID_MAX];
    int i;
    int j;

    if (det == NULL || det->tipCount < 2 || det->tipCount > WT_BLADE_ID_MAX) {
        return false;
    }

    for (i = 0; i < det->tipCount; i++) {
        const double du = det->tipU[i] - det->hubU;
        const double dv = det->tipV[i] - det->hubV;

        /*
         * 图像正上方为 0°、顺时针递增：atan2(du, -dv)。
         *   正上方 (du=0, dv<0) → atan2(0, +) = 0°
         *   正右方 (du>0, dv=0) → atan2(+, 0) = 90°
         * 取 -dv 是因为图像 v 轴向下。
         */
        angles[i] = Wt_Wrap360(Wt_RadToDeg(atan2(du, -dv)));
    }

    /* 选择排序：n 最多 6，与 wt_blade_id.c 同一写法，便于对照 */
    for (i = 0; i < det->tipCount - 1; i++) {
        for (j = i + 1; j < det->tipCount; j++) {
            if (angles[j] < angles[i]) {
                const double ta = angles[i];
                const double tu = det->tipU[i];
                const double tv = det->tipV[i];
                const bool tval = det->tipValid[i];

                angles[i] = angles[j];
                det->tipU[i] = det->tipU[j];
                det->tipV[i] = det->tipV[j];
                det->tipValid[i] = det->tipValid[j];

                angles[j] = ta;
                det->tipU[j] = tu;
                det->tipV[j] = tv;
                det->tipValid[j] = tval;
            }
        }
    }

    if (outAngles != NULL) {
        for (i = 0; i < det->tipCount; i++) {
            outAngles[i] = angles[i];
        }
    }

    return true;
}
