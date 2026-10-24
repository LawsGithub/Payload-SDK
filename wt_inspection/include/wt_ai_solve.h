/**
 * @file wt_ai_solve.h
 * @brief 端到端解算：AI 关键点 → 叶片编号 + 三维位置。
 *
 * ## 为什么要反投影修正编号
 *
 * 直接用**图像极角**给叶片编号，在完美正对盘面时够用，但实际拍摄总有透视：
 *
 *   2026-10-21 实测（`img1.jpg`，正面特写，仍非完美正对）：
 *     三片叶尖的径向距离  874 / 1069 / 848 px  —— 差 22%
 *     编号解算的间隔偏差  24.74°              —— 理想值应接近 0
 *
 * 径向差 22% 意味着三片叶片在图像里的投影长度不同，极角间隔自然不均。
 * 这个误差**不是检测不准**（关键点肉眼看着很准），是**透视本身**。
 *
 * 修正办法：把每个叶尖按射线反投影到**已知的叶轮盘面**上，得到三维位置，
 * 再在盘面内算相位角。透视被几何关系抵消，三片的径向距离会重新趋于一致
 * （真实的叶片长度相等）。
 *
 * ## 分层
 *
 *   关键点像素
 *     ↓ WtProjection_PixelToRotorPlane   （每片叶尖各自反投影）
 *   盘面三维点
 *     ↓ WtProjection_PointToRotorCoords  （径向 + 相位）
 *   相位角数组
 *     ↓ WtBladeId_AssignFromAngles       （编号）
 *   NO.1 / NO.2 / NO.3
 *
 * 每一层都已单独测试过，本模块只做串联与结果校验。
 */

#ifndef WT_AI_SOLVE_H
#define WT_AI_SOLVE_H

#include <stdbool.h>

#include "wt_blade_id.h"
#include "wt_geometry.h"
#include "wt_projection.h"
#include "wt_turbine.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 单张图里一个风机的 AI 检测结果 */
typedef struct {
    bool hubValid;              /*!< 轮毂关键点是否可信 */
    double hubU;                /*!< 轮毂像素 x */
    double hubV;                /*!< 轮毂像素 y */

    int tipCount;               /*!< 实际检出的叶尖数（≤ WT_BLADE_ID_MAX） */
    double tipU[WT_BLADE_ID_MAX]; /*!< 叶尖像素 x */
    double tipV[WT_BLADE_ID_MAX]; /*!< 叶尖像素 y */
    bool tipValid[WT_BLADE_ID_MAX]; /*!< 各叶尖是否可信 */
} WtAiDetection;

/** 解算上下文：相机与风机几何 */
typedef struct {
    WtCameraPose pose;          /*!< 相机位姿 */
    int imageWidthPx;
    int imageHeightPx;
    double fovHorizDeg;
    double fovVertDeg;
    WtRotorFrame rotorFrame;    /*!< 叶轮盘面（提供中心与法向） */
    double parkPhaseDeg;        /*!< 1 号叶片物理相位 */
} WtAiSolveContext;

/** 单片叶片的解算结果 */
typedef struct {
    int bladeNo;            /*!< 叶片编号 1..N，0 表示未解出 */
    WtEnu point;            /*!< 盘面上的三维位置（ENU） */
    double radialM;         /*!< 距轮毂中心的径向距离 m */
    double phaseDeg;        /*!< 盘面内相位角 度 */
    double distanceM;       /*!< 相机到该点的斜距 m */
    bool valid;
} WtAiBladeResult;

/** 整体解算结果 */
typedef struct {
    bool hubHit;                /*!< 轮毂是否成功反投影 */
    WtEnu hubPoint;             /*!< 轮毂三维位置 */
    double hubDistanceM;        /*!< 到轮毂的斜距 m */

    WtAiBladeResult blades[WT_BLADE_ID_MAX];
    int bladeCount;             /*!< 参与解算的叶片数 */

    /*
     * 解算质量：三片叶片反投影后的径向距离彼此相差多少。
     *
     * 真实风机三片叶片等长，所以这个值**应该很小**（几 % 以内）。
     * 它偏大说明：
     *   - 姿态/内参不准（反投影有系统误差）
     *   - 盘面假设错（park_phase 或 rotAxis 不对）
     *   - 某个关键点检测偏了
     *
     * 这是判断"这次解算能不能用"的核心指标。
     */
    double radialSpreadM;
    double radialSpreadPct;

    /*
     * 编号解算的间隔偏差（度），理想接近 0。
     *
     * ⚠️ **编号失败时这里是 360.0，不是 0。**
     *
     * 曾经的写法是「成功才赋值，失败保持 memset 的 0」—— 而 0 在 README
     * 的质量表里恰好是"基本正对盘面、编号可信"这一档。于是"编号彻底失败"
     * 会被下游读成"质量最好"，而按 README 建议加的 `dev < 15°` 采信阈值
     * 会直接放行。
     *
     * 失败时也**不能**用 NaN：`NaN > 15.0` 求值为 false，阈值判据会静默
     * 失效（fail-open）。360.0 则必然超过任何合理阈值，是 fail-closed 的。
     *
     * 判断编号是否成功请看 `idAssigned`，不要靠这个值反推。
     */
    double angleGapDevDeg;

    /*
     * 编号是否真的解出来了。为 false 时 `blades[].bladeNo` 全为 0，
     * `angleGapDevDeg` 为 360.0。
     */
    bool idAssigned;

    bool ok;                    /*!< 整体是否可信 */
} WtAiSolveResult;

/**
 * @brief 执行端到端解算。
 *
 * @param ctx 相机与风机几何
 * @param det AI 检测结果（像素坐标）
 * @param out 解算结果
 * @return true 表示至少解出一片叶片
 */
bool WtAiSolve_FromDetection(const WtAiSolveContext *ctx,
                             const WtAiDetection *det,
                             WtAiSolveResult *out);

#ifdef __cplusplus
}
#endif

#endif /* WT_AI_SOLVE_H */
