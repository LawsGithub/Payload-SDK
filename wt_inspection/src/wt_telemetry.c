/**
 * @file wt_telemetry.c
 * @brief 遥测解算的纯算法部分（不依赖 PSDK）
 *
 * 与 PSDK 的胶水代码放在 wt_telemetry_psdk.c，两者以 WtTelemetry 上下文
 * 对接。这样相位反解、低通滤波、相位外推这些真正容易出错的部分可以在
 * PC 上单测，不必每次都上机验证。
 */

#include "wt_telemetry.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/*
 * 相位低通系数。视觉反解的逐帧噪声约在 ±3°，而风轮的真实转动是低频的，
 * 因此用较强的一阶低通：系数越小越平滑，但跟随真实转动的滞后越大。
 * 0.25 对应约 4 帧的时间常数，在 10Hz 下滞后约 0.4s，对停机巡检足够，
 * 对叶尖追踪则需要按 profile 调高。
 */
#define WT_PHASE_LPF_ALPHA 0.25

/*
 * 由两次观测估计转速时，两次采样的相位差需超过该阈值才认为可信。
 * 太小的差值会被噪声淹没，算出的转速毫无意义。
 */
#define WT_PHASE_MIN_DELTA_DEG 1.0

/** 相位差归一到 (-180, 180] 并转为标量转角 */
static double WtTelemetry_PhaseDelta(double a, double b)
{
    return Wt_AngleDiff(a, b);
}

void WtTelemetry_UpdatePhaseEstimate(WtTelemetry *telemetry, double measuredPhaseDeg,
                                     uint64_t sampleTimeMs)
{
    WtRotorState *r = &telemetry->rotor;

    if (!r->valid) {
        r->phaseDeg = Wt_Wrap360(measuredPhaseDeg);
        r->rpm = 0.0;
        r->valid = true;
        r->updatedAtMs = sampleTimeMs;
        return;
    }

    {
        double delta = WtTelemetry_PhaseDelta(measuredPhaseDeg, r->phaseDeg);
        uint64_t dtMs = (sampleTimeMs > r->updatedAtMs) ? (sampleTimeMs - r->updatedAtMs) : 0;

        /* 一阶低通：用增量而不是直接赋值，避免相位在 0/360 边界跳变 */
        r->phaseDeg = Wt_Wrap360(r->phaseDeg + WT_PHASE_LPF_ALPHA * delta);

        /*
         * 转速由「本次测量的相位增量 / 时间」估计，而不是用滤波后的相位：
         * 滤波会吃掉真实的转动，用它反推转速会系统性偏小。
         */
        if (dtMs > 0 && fabs(delta) > WT_PHASE_MIN_DELTA_DEG) {
            double rpm = delta * 60.0 / (360.0 * ((double)dtMs / 1000.0));
            double prevRpm = r->rpm;

            /* 转速再做一次低通，单帧估计的方差很大 */
            r->rpm = (prevRpm == 0.0) ? rpm : (prevRpm * 0.7 + rpm * 0.3);
        } else if (dtMs > 2000) {
            /* 长时间没有有效增量，判定为静止，避免残留转速导致外推发散 */
            r->rpm = 0.0;
        }

        r->updatedAtMs = sampleTimeMs;
    }
}

bool WtTelemetry_SolvePhaseFromVision(WtTelemetry *telemetry,
                                       double azOfQuadrantDeg,
                                       double pitchOfQuadrantDeg,
                                       WtRotorState *outRotor)
{
    const WtGimbalState *g = &telemetry->gimbal;
    WtEnu tipDir;
    WtEnu tipPoint;
    double phase;
    double halfSpan;

    if (!g->valid || !telemetry->aircraft.validPosition) {
        return false;
    }

    /*
     * 视线方向 = 云台姿态。图像的横纵坐标换算成相对光轴的方位/俯仰偏角，
     * 叠加到云台指向即得叶尖方向。这里把输入的 az/pitch 视为「相对光轴的
     * 偏角」，由图像处理侧按内参换算好后传入 —— 内参与像素的换算属于
     * 视觉模块，不在本层重复实现。
     */
    tipDir = WtEnu_FromAzEl(g->yawDeg + azOfQuadrantDeg, g->pitchDeg + pitchOfQuadrantDeg);

    /* 沿视线方向取到叶尖的距离：用叶轮半径作为距离的初值足够反解方向 */
    halfSpan = telemetry->turbineSpec.rotorDiameter * 0.5;
    tipPoint = WtEnu_Add(telemetry->aircraft.enu, WtEnu_Scale(tipDir, halfSpan));

    /* 逐片叶片试反解，取与预测相位最接近的那个解，避免叶片序号歧义 */
    {
        double bestPhase = 0.0;
        double bestErr = 1e18;
        int i;

        for (i = 0; i < telemetry->turbineSpec.bladeCount; i++) {
            double p;

            if (!WtTurbine_PhaseFromTip(&telemetry->turbineSpec, tipPoint, i, &p)) {
                continue;
            }

            {
                double err = fabs(WtTelemetry_PhaseDelta(p, telemetry->rotor.phaseDeg));

                if (err < bestErr) {
                    bestErr = err;
                    bestPhase = p;
                }
            }
        }

        if (bestErr > 90.0) {
            /* 三个候选解都离预测很远，说明视觉输入本身不可信，拒绝更新 */
            return false;
        }
        phase = bestPhase;
    }

    WtTelemetry_UpdatePhaseEstimate(telemetry, phase, telemetry->aircraft.timestampMs);

    if (outRotor != NULL) {
        *outRotor = telemetry->rotor;
    }

    return true;
}

void WtTelemetry_RefreshEnu(WtTelemetry *telemetry)
{
    if (!telemetry->aircraft.validPosition) {
        return;
    }

    telemetry->aircraft.enu = WtGeo_ToEnu(&telemetry->frame, &telemetry->aircraft.position);
}

WtRotorFrame WtTelemetry_PredictRotorFrame(const WtTelemetry *telemetry, double dtSec)
{
    double predicted = telemetry->rotor.valid
                           ? WtTurbine_AdvancePhase(&telemetry->turbineSpec,
                                                    telemetry->rotor.phaseDeg,
                                                    telemetry->rotor.rpm, dtSec)
                           : 0.0;

    return WtTurbine_BuildRotorFrame(&telemetry->turbineSpec, predicted);
}

void WtTelemetry_LogSnapshot(const WtTelemetry *telemetry)
{
    const WtAircraftState *a = &telemetry->aircraft;
    const WtRotorState *r = &telemetry->rotor;
    const WtGimbalState *g = &telemetry->gimbal;

    printf("[tele] pos=(%.7f, %.7f, %.1f) enu=(%.1f, %.1f, %.1f) "
           "yaw=%.1f vel=%.1f alt=%.1f sat=%u rtk=%d flight=%u\n",
           a->position.lat, a->position.lon, a->position.alt,
           a->enu.e, a->enu.n, a->enu.u,
           a->yawDeg, a->velocityMs, a->heightAboveGroundM,
           a->visibleSatellites, a->rtkFixed ? 1 : 0, a->flightStatus);

    printf("[tele] rotor phase=%.1f° rpm=%.2f valid=%d | gimbal yaw=%.1f pitch=%.1f valid=%d\n",
           r->phaseDeg, r->rpm, r->valid ? 1 : 0,
           g->yawDeg, g->pitchDeg, g->valid ? 1 : 0);
}