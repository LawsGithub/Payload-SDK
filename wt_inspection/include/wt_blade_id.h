/**
 * @file wt_blade_id.h
 * @brief 从 AI 关键点结果解算叶片编号与三维位置。
 *
 * ## 要解决什么问题
 *
 * 开源模型（rwth-irt/Wind-Turbine-Synthetic-Vision）输出 7 个关键点：
 *
 *   0 机舱后部   1 塔顶   2 塔底   3 机舱前部(=轮毂)
 *   4 叶尖 A     5 叶尖 B  6 叶尖 C
 *
 * **但 4/5/6 是无序的。** 模型用「排列不变损失」训练（`loss.py` 里对
 * 三个叶尖枚举 6 种排列取误差最小的那个），所以它学到的是"三个点的集合"，
 * 而不是"1 号叶尖在哪"。三片叶片旋转对称，视觉上本就无法区分。
 *
 * ## 编号必须由几何决定
 *
 * 1 号叶片的物理定义是**相位角 = `park_phase` 的那片**。所以：
 *
 *   ① 以轮毂为原点，算三个叶尖的图像极角
 *   ② 图像极角 → 风轮平面相位角（正对盘面时近似相等，符号取决于相机朝向）
 *   ③ 与 park_phase 最接近的 → NO.1，按旋向依次 NO.2 / NO.3
 *
 * ## 本模块的分层
 *
 * - `WtBladeId_AssignFromAngles`：纯几何，输入角度，输出编号。**可单元测试**
 * - `WtBladeId_AssignFromPixels`：从像素算角度再调上面那个
 *
 * 之所以拆开：角度→编号这段是精度和正确性的核心，不该被像素坐标的噪声掩盖。
 */

#ifndef WT_BLADE_ID_H
#define WT_BLADE_ID_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 叶片数量上限（与 WtTurbineSpec 一致） */
#define WT_BLADE_ID_MAX 6

/** 单次解算的输入 */
typedef struct {
    /*
     * 三片（或 bladeCount 片）叶片相对轮毂的角度，单位度，范围 [0,360)。
     *
     * 定义：在风轮平面内，从「12 点钟」方向起、沿 ψ 增大方向量的角度 ——
     * 与 `WtRotorFrame` 的相位角同一约定。调用方负责把图像极角换算到这个
     * 约定上（见 `WtBladeId_AssignFromPixels` 的说明）。
     */
    double anglesDeg[WT_BLADE_ID_MAX];
    int count;              /*!< 实际叶片数，典型 3 */
    double parkPhaseDeg;    /*!< 1 号叶片的物理相位角（配置里的 park_phase） */
} WtBladeIdInput;

/** 单次解算的输出 */
typedef struct {
    /*
     * `bladeNoOf[i]` = 第 i 个输入角度对应的叶片编号（1..count）。
     * 编号 1 对应 park_phase，其后按角度增大方向依次 2、3…
     */
    int bladeNoOf[WT_BLADE_ID_MAX];
    int oneBasedIndex;      /*!< 被判为 NO.1 的输入索引，-1 表示失败 */
    double worstGapDeg;     /*!< 相邻编号间的最大角度偏差，用于判断解算是否可信 */
} WtBladeIdResult;

/**
 * @brief 由角度解算叶片编号。
 *
 * 纯几何、无副作用，可对任意输入反复调用 —— 这是本模块最该被测的部分。
 *
 * @return true 表示解算成功（输入合法且至少 1 片）
 */
bool WtBladeId_AssignFromAngles(const WtBladeIdInput *in, WtBladeIdResult *out);

/**
 * @brief 由图像像素解算叶片编号，并给出图像极角。
 *
 * 输入为轮毂与各叶尖的像素坐标。内部按 `imageAngleSign` 把图像极角换算成
 * 风轮平面相位角：
 *
 *   phaseDeg = wrap360(imageAngleDeg * imageAngleSign + imageAngleOffsetDeg)
 *
 * 两个标定量的来历：正对盘面时图像极角与相位角近似相等，但
 *   - 相机倒装 / 镜像 → `imageAngleSign = -1`
 *   - 轮毂不在画面正中、或 12 点钟方向与图像 +Y 不一致 → 用 offset 校正
 *
 * 现场标定方法：让风机停在**已知 park_phase**（如 0°：一片竖直向上），
 * 拍一张，读出图像里那片叶片的极角 θ，则
 *   offset = 0 - θ * sign
 *
 * @param hubU, hubV      轮毂像素坐标
 * @param tipU, tipV      各叶尖像素坐标数组，长度 count
 * @param count           叶片数
 * @param imageAngleSign  +1 或 -1
 * @param imageAngleOffsetDeg 角度偏移
 * @param parkPhaseDeg    1 号叶片物理相位
 * @param out             解算结果
 * @param outImageAngles  可选，输出各叶片的图像极角（用于调试）
 * @return true 表示解算成功
 */
bool WtBladeId_AssignFromPixels(double hubU, double hubV,
                                const double *tipU, const double *tipV, int count,
                                double imageAngleSign, double imageAngleOffsetDeg,
                                double parkPhaseDeg,
                                WtBladeIdResult *out,
                                double *outImageAngles);

#ifdef __cplusplus
}
#endif

#endif /* WT_BLADE_ID_H */
