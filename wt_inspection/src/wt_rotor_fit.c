/**
 * @file wt_rotor_fit.c
 * @brief 由激光实测点反推叶轮盘面几何的实现。
 *
 * 纯几何，不依赖 PSDK，可在桌面用构造的已知答案验证。
 */

#include "wt_rotor_fit.h"

#include "wt_projection.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* 点退化判据：两点间距小于此值视为重合，叉积不可用 */
#define WT_FIT_MIN_SEPARATION_M 0.5

/* 法向长度下限：叉积模长过小说明三点近乎共线 */
#define WT_FIT_MIN_NORMAL_LEN 1e-6

WtRotorFitLimits WtRotorFit_DefaultLimits(void)
{
    WtRotorFitLimits l;

    /*
     * 径向散布 5%：与 wt_ai_solve.c 的 WT_AI_RADIAL_SPREAD_LIMIT_PCT 取同一个
     * 值，理由也相同 —— 真实叶片等长，40 m 叶片 5% 就是 2 m，已经很宽松。
     */
    l.maxRadiusSpreadPct = 5.0;

    /*
     * 相位间隔偏差 8°：激光打在叶尖附近而非正尖端会产生这个量级的相位误差
     * （40 m 处横向偏 5 m 约 7°）。再放宽就分不出"打偏了"与"叶片真的不均"。
     */
    l.maxGapDevDeg = 8.0;

    /*
     * 实测中心与台账中心的水平偏差 15 m：台账的 nacelleOffset 与机舱实际
     * 偏航共同造成的偏移就有几米量级，而认错风机通常差几十米以上 ——
     * 这个阈值的作用是分辨这两种情况，不是追求精度。
     */
    l.maxCenterOffsetM = 15.0;

    l.maxRadiusErrPct = 15.0;

    /*
     * 法向夹角 25°：机舱随风偏航，实测朝向与台账安装朝向本来就会差很多，
     * 所以这个阈值只用来拦"法向整体解错了"（例如符号反了会差 180°）。
     *
     * ⚠️ 两叶降级路径本身就会贡献约 11.6°（见 wt_rotor_fit.h 的 @warning），
     * 即吃掉这里将近一半余量。收紧这个值之前先确认不会把两叶采集一律判死。
     */
    l.maxAxisAngleErrDeg = 25.0;

    return l;
}

/**
 * 相位间隔偏差：每个间隔都应接近「理想间隔的整数倍」。
 *
 * 为什么是整数倍而不是直接比 360/n：漏测一片叶尖时，剩下两片的间隔是
 * 240° 而不是 120°。拿 120° 直接比会得出 dev=120°，把一次完全正常的
 * 降级采集判成"质量极差"。按整数倍比，240° 对 2×120° 偏差为 0，
 * 既不误杀，也仍能抓住真正打偏的点。
 *
 * （`wt_blade_id.c` 用的是 `360/实际检出数`，在漏检时会落进上面那个坑。
 *  新代码不沿用那个写法。）
 */
static double WtRotorFit_GapDeviation(const double *phases, int n, int bladeCount)
{
    double sorted[WT_ROTOR_FIT_MAX_TIPS];
    double ideal;
    double worst = 0.0;
    int i;
    int j;

    if (n < 2 || bladeCount < 1) {
        return 0.0;
    }

    for (i = 0; i < n; i++) {
        sorted[i] = Wt_Wrap360(phases[i]);
    }
    for (i = 0; i < n - 1; i++) {
        for (j = i + 1; j < n; j++) {
            if (sorted[j] < sorted[i]) {
                const double t = sorted[i];

                sorted[i] = sorted[j];
                sorted[j] = t;
            }
        }
    }

    ideal = 360.0 / (double)bladeCount;

    for (i = 0; i < n; i++) {
        const double gap = Wt_Wrap360(sorted[(i + 1) % n] - sorted[i]);
        double k = floor(gap / ideal + 0.5);
        double dev;

        if (k < 1.0) {
            k = 1.0;
        }
        dev = fabs(gap - k * ideal);

        if (dev > worst) {
            worst = dev;
        }
    }

    return worst;
}

bool WtRotorFit_FromPoints(const WtTurbineSpec *spec,
                           WtEnu hubEnu,
                           const WtEnu *tipEnu, int tipCount,
                           WtRotorFitResult *out)
{
    WtRotorFrame nominal;
    WtEnu normal;
    WtEnu up = {0.0, 0.0, 1.0};
    double normalLen;
    double upDotAxis;
    double axialSum = 0.0;
    double radialMin = 0.0;
    double radialMax = 0.0;
    double radialSum = 0.0;
    int i;

    if (out == NULL) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->ok = false;
    out->message = "not evaluated";

    if (spec == NULL || tipEnu == NULL) {
        out->message = "spec / tipEnu is null";
        return false;
    }
    if (tipCount < 2 || tipCount > WT_ROTOR_FIT_MAX_TIPS) {
        /*
         * 单个叶尖时平面不唯一 —— 轮毂点与一个叶尖只定出一条直线，绕它旋转
         * 的任意平面都"过这两点"。此时宁可拒绝，也不要退回台账法向凑一个
         * 看起来完整的结果：那样调用方分不出哪部分是实测、哪部分是假设。
         */
        out->message = "tipCount must be in [2, WT_ROTOR_FIT_MAX_TIPS]";
        return false;
    }

    /* 台账参考系：提供法向定向依据与比对基准 */
    nominal = WtTurbine_BuildRotorFrame(spec, 0.0);

    /* ---- ① 盘面法向 ---- */
    if (tipCount >= 3) {
        /*
         * 三个叶尖都在「叶尖平面」上（同一半径、同一锥角），它们定出的法向
         * 就是盘面法向。这条路**不用轮毂点** —— 轮毂并不在叶尖平面上
         * （锥角与预弯把叶尖拉向上游），把它拉进来拟合反而会把法向带偏。
         */
        const WtEnu a = WtEnu_Sub(tipEnu[1], tipEnu[0]);
        const WtEnu b = WtEnu_Sub(tipEnu[2], tipEnu[0]);

        if (WtEnu_Length(a) < WT_FIT_MIN_SEPARATION_M ||
            WtEnu_Length(b) < WT_FIT_MIN_SEPARATION_M) {
            out->message = "tip points too close to each other";
            return false;
        }
        normal = WtEnu_Cross(a, b);
    } else {
        /*
         * 只测到两个叶尖时退回「轮毂 + 两叶尖」定平面。
         *
         * ⚠️ 这条路的法向含**锥角量级的系统误差**（约 3°）：轮毂不在叶尖
         * 平面上，三点定出的平面相对真实盘面是倾斜的。结果仍然可用于
         * 粗定位，但 axisAngleErrDeg 会偏大，message 里标明来源。
         */
        const WtEnu a = WtEnu_Sub(tipEnu[0], hubEnu);
        const WtEnu b = WtEnu_Sub(tipEnu[1], hubEnu);

        if (WtEnu_Length(a) < WT_FIT_MIN_SEPARATION_M ||
            WtEnu_Length(b) < WT_FIT_MIN_SEPARATION_M) {
            out->message = "tip points too close to hub";
            return false;
        }
        normal = WtEnu_Cross(a, b);
    }

    normalLen = WtEnu_Length(normal);
    if (normalLen < WT_FIT_MIN_NORMAL_LEN) {
        out->message = "points are collinear, plane undefined";
        return false;
    }
    normal = WtEnu_Scale(normal, 1.0 / normalLen);

    /*
     * 定向：叉积的符号取决于采集顺序（NO.1→NO.2→NO.3 是顺时针还是逆时针看
     * 从哪一侧观察），所以必须用外部信息定向。台账法向指向下游，取与它同向
     * 的那一支。
     *
     * 这里只用台账的**符号**，不用它的数值 —— 即使机舱偏航了 90°，点积的
     * 正负仍然可靠（偏到 >90° 才会翻，那种情况 axisAngleErrDeg 会直接报警）。
     */
    if (WtEnu_Dot(normal, nominal.rotAxis) < 0.0) {
        normal = WtEnu_Scale(normal, -1.0);
    }

    out->rotorCenter = hubEnu;
    out->rotAxis = normal;

    /* ---- ② 盘面内基矢量：与 WtTurbine_BuildRotorFrame 同一套约定 ---- */
    upDotAxis = WtEnu_Dot(up, out->rotAxis);
    out->upRef = WtEnu_Normalize(WtEnu_Sub(up, WtEnu_Scale(out->rotAxis, upDotAxis)));
    if (WtEnu_Length(out->upRef) < WT_FIT_MIN_NORMAL_LEN) {
        /* 主轴竖直（盘面水平）—— 12 点方向无定义。真实风机不会这样。 */
        out->message = "rotor axis is vertical, upRef undefined";
        return false;
    }
    out->rotRef = WtEnu_Normalize(WtEnu_Cross(out->rotAxis, out->upRef));

    /* ---- ③ 各叶尖的盘面内坐标 ---- */
    {
        /*
         * 此刻 out->parkPhaseDeg 还没算出来（它要等相位算完才知道），所以这个
         * frame 的 phaseDeg 字段是 0。这里成立的前提是
         * WtProjection_PointToRotorCoords 只读 rotorCenter / upRef / rotRef ——
         * 相位角是相对 upRef 量出来的，与 frame 自己记录的 phaseDeg 无关。
         */
        WtRotorFrame fitted = WtRotorFit_ToRotorFrame(out);

        out->tipCount = tipCount;

        for (i = 0; i < tipCount; i++) {
            double radial = 0.0;
            double phase = 0.0;

            /*
             * 复用 WtProjection_PointToRotorCoords 而不是在这里重算：
             * 「相位角从 upRef 起、沿 rotRef 量」这个约定必须全工程只有一份
             * 实现，否则两处会各自漂移，而症状是编号整体偏一片。
             */
            if (!WtProjection_PointToRotorCoords(&fitted, tipEnu[i], &radial, &phase)) {
                out->message = "failed to project tip into rotor plane";
                return false;
            }

            out->tipRadialM[i] = radial;
            out->tipPhaseDeg[i] = phase;

            /* 轴向偏移：叶尖相对轮毂沿法向的分量，锥角与预弯的体现（应为负） */
            axialSum += WtEnu_Dot(WtEnu_Sub(tipEnu[i], hubEnu), out->rotAxis);

            radialSum += radial;
            if (i == 0 || radial < radialMin) {
                radialMin = radial;
            }
            if (i == 0 || radial > radialMax) {
                radialMax = radial;
            }
        }
    }

    if (radialMax < WT_FIT_MIN_SEPARATION_M) {
        out->message = "all tips collapsed onto the hub";
        return false;
    }

    out->rotorRadiusM = radialSum / (double)tipCount;
    out->radiusSpreadPct = (radialMax - radialMin) / radialMax * 100.0;
    out->axialOffsetM = axialSum / (double)tipCount;

    /* 停用角 = 采集顺序第一个叶尖（NO.1）的相位 */
    out->parkPhaseDeg = out->tipPhaseDeg[0];

    out->gapDevDeg = WtRotorFit_GapDeviation(out->tipPhaseDeg, tipCount,
                                             spec->bladeCount);

    /* ---- ④ 与台账交叉校验 ---- */
    {
        const WtEnu nominalCenter = nominal.rotorCenter;
        const WtEnu d = WtEnu_Sub(hubEnu, nominalCenter);
        double dot;

        out->centerOffsetHorizM = sqrt(d.e * d.e + d.n * d.n);

        /*
         * 垂直分量单独报：激光返回的海拔基准没有文档承诺（姊妹项目实测与
         * 气压高差约 38 m），所以这一项偏大**不一定**意味着位置错了。
         * 把它和水平偏差混成一个三维距离，会让基准问题伪装成定位问题。
         */
        out->centerOffsetVerticalM = d.u;

        if (spec->rotorDiameter > 1e-9) {
            const double specRadius = spec->rotorDiameter * 0.5;

            out->radiusErrPct = (out->rotorRadiusM - specRadius) / specRadius * 100.0;
        }

        dot = Wt_Clamp(WtEnu_Dot(out->rotAxis, nominal.rotAxis), -1.0, 1.0);
        out->axisAngleErrDeg = Wt_RadToDeg(acos(dot));
    }

    out->ok = true;
    out->message = (tipCount >= 3) ? "ok"
                                   : "ok (2 tips: normal carries cone-angle bias)";

    return true;
}

bool WtRotorFit_CheckLimits(const WtRotorFitResult *fit,
                            const WtRotorFitLimits *limits,
                            const char **outReason)
{
    const char *reason = "ok";
    bool pass = true;

    if (fit == NULL || limits == NULL) {
        reason = "null argument";
        pass = false;
    } else if (!fit->ok) {
        reason = fit->message;
        pass = false;
    } else if (fit->radiusSpreadPct > limits->maxRadiusSpreadPct) {
        reason = "radius spread too large (a tip was likely mis-aimed)";
        pass = false;
    } else if (fit->gapDevDeg > limits->maxGapDevDeg) {
        reason = "phase gap deviation too large";
        pass = false;
    } else if (fit->centerOffsetHorizM > limits->maxCenterOffsetM) {
        reason = "measured hub too far from spec (wrong turbine or stale spec?)";
        pass = false;
    } else if (fabs(fit->radiusErrPct) > limits->maxRadiusErrPct) {
        reason = "measured radius disagrees with spec rotorDiameter";
        pass = false;
    } else if (fit->axisAngleErrDeg > limits->maxAxisAngleErrDeg) {
        reason = "measured rotor axis disagrees with spec heading";
        pass = false;
    }

    if (outReason != NULL) {
        *outReason = reason;
    }

    return pass;
}

WtRotorFrame WtRotorFit_ToRotorFrame(const WtRotorFitResult *fit)
{
    WtRotorFrame frame;

    memset(&frame, 0, sizeof(frame));

    if (fit == NULL) {
        return frame;
    }

    frame.rotorCenter = fit->rotorCenter;
    frame.rotAxis = fit->rotAxis;
    frame.upRef = fit->upRef;
    frame.rotRef = fit->rotRef;
    frame.phaseDeg = fit->parkPhaseDeg;
    frame.rpm = 0.0; /* 实测前提是停机 */

    return frame;
}

void WtRotorFit_Format(const WtRotorFitResult *fit, char *buf, size_t bufLen)
{
    if (buf == NULL || bufLen == 0) {
        return;
    }
    if (fit == NULL) {
        snprintf(buf, bufLen, "rotor fit: (null)\n");
        return;
    }

    snprintf(buf, bufLen,
             "rotor fit : %s (%s)\n"
             "  center   : E %.2f N %.2f U %.2f\n"
             "  axis     : E %.4f N %.4f U %.4f\n"
             "  radius   : %.2f m (spread %.2f%%, vs spec %+.1f%%)\n"
             "  park     : %.1f deg (gap dev %.2f deg)\n"
             "  axial off: %.2f m (cone + prebend, expected negative)\n"
             "  vs spec  : horiz %.2f m, vert %+.2f m, axis %.1f deg\n",
             fit->ok ? "OK" : "FAILED", fit->message,
             fit->rotorCenter.e, fit->rotorCenter.n, fit->rotorCenter.u,
             fit->rotAxis.e, fit->rotAxis.n, fit->rotAxis.u,
             fit->rotorRadiusM, fit->radiusSpreadPct, fit->radiusErrPct,
             fit->parkPhaseDeg, fit->gapDevDeg,
             fit->axialOffsetM,
             fit->centerOffsetHorizM, fit->centerOffsetVerticalM,
             fit->axisAngleErrDeg);
}
