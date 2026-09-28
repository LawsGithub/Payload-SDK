/**
 * @file lz_vision_shared.c
 * @brief 视觉层里**两个后端都要有**的东西：默认配置 + 像素↔角度的纯数学。
 *
 * ## 为什么单独一个文件，而不是放进 `lz_vision.c`
 *
 * 这两个函数是"视觉调俯仰"链路里唯一**完全可桌面验证**的部分 ——
 * 不依赖 PSDK、不依赖图像、不依赖机型。而 `lz_vision.c` 与
 * `lz_vision_stub.c` 是**互斥的两个后端**（都定义 `LzVision_Detect`，
 * 同时编会符号冲突），把纯数学放进任一个都会：
 *
 *   1. 在另一个后端下**编不进来**（于是 `lz_test_vision` 只在 hsv 后端注册，
 *      纯数学的测试也跟着丢了）
 *   2. 要么就得在两处各写一份 —— 那就是两份真值，与
 *      `LZ_GIMBAL_PITCH_*` / 半径包线那两次踩的坑同一个形状
 *
 * 所以单独成文件，**两个后端共用**，且**无论选哪个后端都参与编译**。
 *
 * ## 与"取到的是哪个镜头"的关系
 *
 * 无关 —— 对角视场角是**入参**。所以在取图探针答出"广角 82° / 中长焦 35° /
 * 长焦 15°"之前，本文件就能用规格值在桌面上跑测试。
 *
 * 设计依据与全部待定项见 [`doc/VISION-GIMBAL-PITCH.md`](../../doc/VISION-GIMBAL-PITCH.md) §7 步骤 2/4。
 *
 * ## ⚠️ 为什么 `LzVision_DefaultConfig` 也在这里（2026-09-27 挪过来）
 *
 * 它原先只定义在 `lz_vision.c`（hsv 后端）里，而**头文件声明了它**。
 * 后果：切到 `stub` 后端时，任何调用它的代码都**链接失败**
 * （2026-09-27 写 `lz_vision_probe` 时踩到 —— 那个探针只是想打印一下
 * 阈值，就在 stub 下编不过）。
 *
 * 这与 `LzVision_BackendName` 是**同一个形状**：两个后端实现的函数集
 * 不一致，而头文件按"全集"声明。凡是头文件声明的东西，
 * **两个后端都必须提供** —— 要么各自实现，要么都从本文件取。
 */

#include "lz_vision.h"

#include <math.h>

/* ------------------------------------------------------------------ */
/* 像素 ↔ 角度（实现在这里，两个后端共用 —— 它们与检测算法无关）          */
/* ------------------------------------------------------------------ */

LzVisionConfig LzVision_DefaultConfig(void)
{
    /* 取值依据：6 张真实俯拍照片的红色像素分布（16966 个样本）
     *   H: p5=5.6  p50=172.9  p95=176.6   —— 双峰清晰
     *   S: p5=129  p50=224    p95=249
     *   V: p5=170  p50=228    p95=254
     * 低段(H<=10) 占 8.2%，高段(H>=170) 占 91.8%。
     * 阈值取得比 p5 更宽松（S=100 / V=60）是**刻意的**：留出阴天的余量，
     * 代价是偶尔多几个小红块 —— 而"取面积最大"会把它滤掉。 */
    LzVisionConfig c;
    c.redHueLowMax = 10;
    c.redHueHighMin = 170;
    c.minSaturation = 100;
    c.minValue = 60;
    c.minBlobArea = 200;
    c.poleRoiMarginX = 70;
    c.poleContrastOffset = 9;
    c.poleContrastThreshold = 34;
    c.poleHalfWidth = 1;
    c.poleGap = 40;
    c.poleWinAbove = 20;
    /* ⚠️ 下界**按旗高缩放**，不是固定像素 —— 见 lz_vision.h 的说明。
     * 2600/1000 = 2.6 倍旗高。6 张黄金图 + 现场广角/变焦两张截图实测：
     * 从 2.0 到 3.5 倍之间，杆列位置不变、8 张全部检出。 */
    c.poleWinBelowRatioQ = 2600;
    c.minConfidence = 0.5;
    return c;
}

double LzVision_VerticalFovDeg(int frameW, int frameH, double diagFovDeg)
{
    if (frameW <= 0 || frameH <= 0) {
        return NAN;
    }
    if (!isfinite(diagFovDeg) || !(diagFovDeg > 0.0) || !(diagFovDeg < 180.0)) {
        return NAN;
    }

    /* 对角像素长度。用 double 累加：W、H 各自上到 8K 时 W² 会到 6.7e7，
     * 平方在 double 里毫无压力，但这一点值得写下来 —— 它是"为什么不用
     * 整数开方"的答案。 */
    const double w = (double)frameW;
    const double h = (double)frameH;
    const double diag = sqrt(w * w + h * h);
    if (!(diag > 0.0)) {
        return NAN;
    }

    /* tan(DFOV/2) · H/D —— 见头文件的推导。
     * ⚠️ 这里**不是** tan(DFOV/2)·H/W（那就等价于把 DFOV 当水平 FOV 用，
     * 差 11°），也不是 tan(DFOV/2)·W/D（那是 HFOV）。 */
    const double halfDiagRad = diagFovDeg * 0.5 * (M_PI / 180.0);
    const double tanHalfV = tan(halfDiagRad) * (h / diag);

    const double vfovDeg = 2.0 * atan(tanHalfV) * (180.0 / M_PI);
    if (!isfinite(vfovDeg) || !(vfovDeg > 0.0) || !(vfovDeg < 180.0)) {
        return NAN;
    }
    return vfovDeg;
}

double LzVision_ZoomedDiagFovDeg(double wideDiagFovDeg, double zoomFactor)
{
    if (!isfinite(wideDiagFovDeg) || !(wideDiagFovDeg > 0.0) || !(wideDiagFovDeg < 180.0)) {
        return NAN;
    }
    if (!isfinite(zoomFactor) || !(zoomFactor > 0.0)) {
        return NAN;
    }

    /* 小孔成像：焦距 f 越大视场越小，而"变焦倍数"就是焦距之比 ——
     * 于是半角的正切按倍数**缩小**：
     *
     *     tan(DFOV_zoomed / 2) = tan(DFOV_wide / 2) / zoomFactor
     *
     * ⚠️ **不能直接对角度做除法**（`DFOV / z`）。视场角与焦距是
     * **正切**关系而非线性关系：82° 除以 7 得 11.7°，而按正切算是 15.9°，
     * 差 4° —— 在 12.5 m 外是 0.9 m，比一根杆还粗。
     * 这与 `LzVision_VerticalFovDeg` 里"不能把 DFOV 当 HFOV 用"同源：
     * **凡是视场角，变换都发生在正切上。**
     *
     * 用 double 直接算即可：zoomFactor 到 100 时半角仍在 (0, 45°) 内，
     * 不存在精度问题。 */
    const double halfWide = wideDiagFovDeg * 0.5 * (M_PI / 180.0);
    const double tanHalfZoomed = tan(halfWide) / zoomFactor;
    const double zoomedDeg = 2.0 * atan(tanHalfZoomed) * (180.0 / M_PI);
    if (!isfinite(zoomedDeg) || !(zoomedDeg > 0.0) || !(zoomedDeg < 180.0)) {
        return NAN;
    }
    return zoomedDeg;
}

double LzVision_PixelOffsetToDeg(double vMid, double vfovDeg)
{
    if (!isfinite(vMid) || !isfinite(vfovDeg)) {
        return NAN;
    }
    if (vfovDeg <= 0.0 || vfovDeg >= 180.0) {
        return NAN;
    }

    /* 焦平面上的偏移量 = (v - 0.5) · 2 · tan(VFOV/2)（以"半高"为单位）。
     * v == 0.5 时恰为 0 —— 目标已居中，不需要转。 */
    const double halfVRad = vfovDeg * 0.5 * (M_PI / 180.0);
    const double offset = (vMid - 0.5) * 2.0 * tan(halfVRad);

    /* 符号见头文件：v 向下为正、俯仰向下为负，所以 Δθ = −atan(offset)。 */
    const double d = -atan(offset) * (180.0 / M_PI);
    if (!isfinite(d)) {
        return NAN;
    }
    return d;
}
