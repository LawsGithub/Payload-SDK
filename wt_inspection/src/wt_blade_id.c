/**
 * @file wt_blade_id.c
 * @brief 叶片编号解算实现。
 *
 * 纯算法，不依赖 PSDK —— 与 wt_core 的其余部分同一条界。
 */

#include "wt_blade_id.h"

#include "wt_geometry.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* 角度工具                                                            */
/* ------------------------------------------------------------------ */

/** 归一化到 [0,360) */
static double WtBladeId_Wrap360(double deg)
{
    double r = fmod(deg, 360.0);

    if (r < 0.0) {
        r += 360.0;
    }
    /* fmod 对 -0.0 也返回 -0.0，统一成 +0 */
    return (r == 0.0) ? 0.0 : r;
}

/** 两个角度之间的最小夹角，范围 [0,180] */
static double WtBladeId_AngleGap(double a, double b)
{
    double d = fabs(WtBladeId_Wrap360(a) - WtBladeId_Wrap360(b));

    if (d > 180.0) {
        d = 360.0 - d;
    }
    return d;
}

/* ------------------------------------------------------------------ */
/* 核心：角度 → 编号                                                    */
/* ------------------------------------------------------------------ */

bool WtBladeId_AssignFromAngles(const WtBladeIdInput *in, WtBladeIdResult *out)
{
    int n;
    int i;
    int j;
    int oneIdx = -1;
    double bestGap = 1e9;
    double sorted[WT_BLADE_ID_MAX];
    int sortedIdx[WT_BLADE_ID_MAX];

    if (out == NULL) {
        return false;
    }

    /*
     * 先把输出置成明确的"失败"状态，再往下算。
     *
     * 原先 early-return 的几条路径完全不碰 out —— 调用方拿到的是栈上的
     * 残留值，而头文件却声明「oneBasedIndex = -1 表示失败」。更糟的是
     * worstGapDeg 可能恰好是 0，而 0 在质量表里是"最可信"那一档。
     *
     * 360.0 而不是 NaN：NaN 参与任何比较都返回 false，会让下游的
     * `if (dev > 阈值) reject` 静默失效。
     */
    memset(out, 0, sizeof(*out));
    out->oneBasedIndex = -1;
    out->worstGapDeg = 360.0;

    if (in == NULL) {
        return false;
    }
    if (in->count <= 0 || in->count > WT_BLADE_ID_MAX) {
        return false;
    }

    n = in->count;

    /* 输入角度先归一化，后面统一用 [0,360) */
    for (i = 0; i < n; i++) {
        sorted[i] = WtBladeId_Wrap360(in->anglesDeg[i]);
        sortedIdx[i] = i;
    }

    /*
     * 找与 park_phase 最接近的那片 —— 它就是 NO.1。
     *
     * 用「最小夹角」而不是「最小差值」：角度是环形量，
     * 350° 与 10° 相差 20° 而不是 340°。
     */
    for (i = 0; i < n; i++) {
        const double gap = WtBladeId_AngleGap(sorted[i], in->parkPhaseDeg);

        if (gap < bestGap) {
            bestGap = gap;
            oneIdx = i;
        }
    }
    if (oneIdx < 0) {
        return false;
    }

    /*
     * 按角度升序排序，编号从 NO.1 起沿角度增大方向依次递增。
     *
     * 这里假定叶片按角度均匀分布（风机必然如此）。若实际分布不均，
     * `worstGapDeg` 会暴露出来 —— 调用方据此决定是否采信。
     */
    for (i = 0; i < n - 1; i++) {
        for (j = i + 1; j < n; j++) {
            if (sorted[j] < sorted[i]) {
                const double tmpA = sorted[i];
                const int tmpI = sortedIdx[i];

                sorted[i] = sorted[j];
                sortedIdx[i] = sortedIdx[j];
                sorted[j] = tmpA;
                sortedIdx[j] = tmpI;
            }
        }
    }

    /*
     * 排序后，从 NO.1 所在位置开始绕一圈编号。
     * 注意编号是**绕环**的：NO.1 不一定在数组首位。
     */
    {
        int posOfOne = -1;

        for (i = 0; i < n; i++) {
            if (sortedIdx[i] == oneIdx) {
                posOfOne = i;
                break;
            }
        }
        if (posOfOne < 0) {
            return false;
        }

        for (i = 0; i < n; i++) {
            const int ringIdx = (posOfOne + i) % n;
            const int origIdx = sortedIdx[ringIdx];

            out->bladeNoOf[origIdx] = i + 1;
        }
    }

    /*
     * 相邻编号间的角度间隔：理想值 360/n。偏差越大说明检测越不可信
     * （叶尖检出位置有噪声，或某片叶片漏检后由别的点顶上）。
     */
    {
        double worst = 0.0;

        for (i = 0; i < n; i++) {
            const int ringIdx = (i + 1) % n;
            double d = WtBladeId_Wrap360(sorted[ringIdx] - sorted[i]);
            const double ideal = 360.0 / (double)n;
            const double dev = fabs(d - ideal);

            if (dev > worst) {
                worst = dev;
            }
        }
        out->worstGapDeg = worst;
    }

    out->oneBasedIndex = oneIdx;

    return true;
}

/* ------------------------------------------------------------------ */
/* 像素 → 角度 → 编号                                                   */
/* ------------------------------------------------------------------ */

bool WtBladeId_AssignFromPixels(double hubU, double hubV,
                                const double *tipU, const double *tipV, int count,
                                double imageAngleSign, double imageAngleOffsetDeg,
                                double parkPhaseDeg,
                                WtBladeIdResult *out,
                                double *outImageAngles)
{
    WtBladeIdInput in;
    int i;

    if (tipU == NULL || tipV == NULL || out == NULL) {
        return false;
    }
    if (count <= 0 || count > WT_BLADE_ID_MAX) {
        return false;
    }

    memset(&in, 0, sizeof(in));
    in.count = count;
    in.parkPhaseDeg = parkPhaseDeg;

    for (i = 0; i < count; i++) {
        const double du = tipU[i] - hubU;
        const double dv = tipV[i] - hubV;

        /*
         * 图像极角：atan2 的 y 轴在图像里是向下的，所以取 -dv
         * 让「屏幕上方」对应 +90°，与常规数学约定一致。
         * 之后再由 imageAngleSign 处理相机朝向带来的翻转。
         */
        const double imgDeg = Wt_RadToDeg(atan2(-dv, du));

        if (outImageAngles != NULL) {
            outImageAngles[i] = imgDeg;
        }

        in.anglesDeg[i] = WtBladeId_Wrap360(
            imgDeg * imageAngleSign + imageAngleOffsetDeg);
    }

    return WtBladeId_AssignFromAngles(&in, out);
}
