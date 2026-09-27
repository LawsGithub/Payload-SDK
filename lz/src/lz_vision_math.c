/**
 * @file lz_vision_math.c
 * @brief 视觉层里与**检测算法无关**的纯数学：像素 ↔ 角度。
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
 */

#include "lz_vision.h"

#include <math.h>

/* ------------------------------------------------------------------ */
/* 像素 ↔ 角度（实现在这里，两个后端共用 —— 它们与检测算法无关）          */
/* ------------------------------------------------------------------ */

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
