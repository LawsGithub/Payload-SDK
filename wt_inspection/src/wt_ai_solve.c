/**
 * @file wt_ai_solve.c
 * @brief 端到端解算实现：AI 关键点 → 三维位置 → 叶片编号。
 */

#include "wt_ai_solve.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* 反投影后三片叶片径向距离的相对散布超过这个比例就判为不可信。
 * 真实叶片等长，5% 已经是很宽松的容忍（对应 100 m 叶片 5 m 误差）。 */
#define WT_AI_RADIAL_SPREAD_LIMIT_PCT 5.0

bool WtAiSolve_FromDetection(const WtAiSolveContext *ctx,
                             const WtAiDetection *det,
                             WtAiSolveResult *out)
{
    WtBladeIdInput idIn;
    WtBladeIdResult idOut;
    double angles[WT_BLADE_ID_MAX];
    double radials[WT_BLADE_ID_MAX];
    int solved = 0;
    int i;
    double minR;
    double maxR;

    if (ctx == NULL || det == NULL || out == NULL) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    memset(&idIn, 0, sizeof(idIn));

    /* ---- ① 轮毂反投影 ---- */
    if (det->hubValid) {
        WtProjectionResult hubRes;

        if (WtProjection_PixelToRotorPlane(&ctx->pose, det->hubU, det->hubV,
                                           ctx->imageWidthPx, ctx->imageHeightPx,
                                           ctx->fovHorizDeg, ctx->fovVertDeg,
                                           &ctx->rotorFrame, &hubRes) &&
            hubRes.hit) {
            out->hubHit = true;
            out->hubPoint = hubRes.point;
            out->hubDistanceM = hubRes.distanceM;
        }
    }

    /*
     * 轮毂反投影失败时仍可继续给叶片编号 —— 编号只依赖角度，
     * 不依赖轮毂的三维位置。但径向距离会失去参考，所以标记为不可信。
     */

    /* ---- ② 各叶尖反投影 + 盘面内坐标 ---- */
    for (i = 0; i < det->tipCount && i < WT_BLADE_ID_MAX; i++) {
        WtProjectionResult tipRes;
        double radial = 0.0;
        double phase = 0.0;

        if (!det->tipValid[i]) {
            continue;
        }

        if (!WtProjection_PixelToRotorPlane(&ctx->pose, det->tipU[i], det->tipV[i],
                                            ctx->imageWidthPx, ctx->imageHeightPx,
                                            ctx->fovHorizDeg, ctx->fovVertDeg,
                                            &ctx->rotorFrame, &tipRes) ||
            !tipRes.hit) {
            continue;
        }

        if (!WtProjection_PointToRotorCoords(&ctx->rotorFrame, tipRes.point,
                                             &radial, &phase)) {
            continue;
        }

        angles[solved] = phase;
        radials[solved] = radial;
        out->blades[solved].point = tipRes.point;
        out->blades[solved].radialM = radial;
        out->blades[solved].phaseDeg = phase;
        out->blades[solved].distanceM = tipRes.distanceM;
        out->blades[solved].valid = true;
        solved++;
    }

    out->bladeCount = solved;

    if (solved == 0) {
        return false;
    }

    /* ---- ③ 编号 ---- */
    idIn.count = solved;
    idIn.parkPhaseDeg = ctx->parkPhaseDeg;
    for (i = 0; i < solved; i++) {
        idIn.anglesDeg[i] = angles[i];
    }

    if (WtBladeId_AssignFromAngles(&idIn, &idOut)) {
        for (i = 0; i < solved; i++) {
            out->blades[i].bladeNo = idOut.bladeNoOf[i];
        }
        out->angleGapDevDeg = idOut.worstGapDeg;
    }

    /* ---- ④ 质量评估：径向距离的一致性 ---- */
    minR = radials[0];
    maxR = radials[0];
    for (i = 1; i < solved; i++) {
        if (radials[i] < minR) {
            minR = radials[i];
        }
        if (radials[i] > maxR) {
            maxR = radials[i];
        }
    }
    out->radialSpreadM = maxR - minR;
    out->radialSpreadPct = (maxR > 1e-9) ? (out->radialSpreadM / maxR * 100.0) : 0.0;

    /*
     * 可信判据：至少解出 2 片，且径向散布在容忍范围内。
     *
     * 只解出 1 片时 radialSpreadPct 恒为 0，看似完美但毫无意义 ——
     * 所以把 solved >= 2 也作为必要条件。
     */
    out->ok = (solved >= 2) &&
              (out->radialSpreadPct <= WT_AI_RADIAL_SPREAD_LIMIT_PCT) &&
              out->hubHit;

    return true;
}
