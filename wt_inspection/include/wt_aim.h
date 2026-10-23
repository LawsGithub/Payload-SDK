/**
 * @file wt_aim.h
 * @brief 把「目标在画面的哪个像素」换算成「云台该转到哪个角度」。
 *
 * ## 为什么这一层单独存在
 *
 * 激光测距打的是**光轴方向**（`T_DjiCameraManagerLaserRangingInfo` 里的
 * `screenX/screenY` 只是报告激光落点，不能指定），所以要测某个叶尖，必须先
 * 把那个叶尖转到画面中心。这个换算是整条采集链里最容易算错、且错了**不会
 * 报错**的一环 —— 云台会老老实实转到一个错误的角度，激光会老老实实测回一个
 * 错误的点，拟合会老老实实给出一个看起来合理的盘面。
 *
 * 所以它必须在桌面上可测，不能埋在 PSDK 胶水里。
 *
 * ## 不能把偏角直接加到云台角上
 *
 * 直觉做法是「当前云台 yaw + 画面内水平偏角」。**球面上不成立** ——
 * 方位角与俯仰角不是可交换的平移量，误差随偏角与俯仰角增大，pitch 接近
 * ±90° 时完全失效。1440×1080、水平视场 71.6° 的画面，角点偏角就有 ±36°。
 *
 * （`wt_telemetry.c:96` 的 `WtTelemetry_SolvePhaseFromVision` 正是这么写的，
 *  已知有偏差；本模块不沿用那个做法。）
 *
 * 正确路径是走一遍完整的方向矢量变换：
 *
 *   像素 →（内参）→ 相机系射线 →（云台姿态）→ ENU 方向 → 绝对方位/俯仰
 *
 * 得到的是目标方向的**绝对角**，可以直接用
 * `DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE` 下发 —— 比下发相对增量更稳，
 * 不累积误差，重试也不会越转越偏。
 *
 * ## M4T 的硬约束：偏航要靠机头
 *
 * M4T 的云台 Pan 轴标着 `Not controllable`（README「航向锁定」一节、
 * `wt_bridge.c:431`），云台 yaw 无法独立于机头偏转。所以本模块把两个轴
 * **分开报**：`targetPitchDeg` 可以下发给云台，`yawErrorDeg` 只能交给
 * 机头（航点任务的 heading，或操作员手动）。
 *
 * 把它们合成一个「云台姿态」返回会诱导调用方去下发一个根本不会生效的 yaw，
 * 而现象是「云台转了但画面没对准」—— 极难定位。
 */

#ifndef WT_AIM_H
#define WT_AIM_H

#include <stdbool.h>

#include "wt_ai_solve.h"
#include "wt_geometry.h"
#include "wt_projection.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 照准解算结果 */
typedef struct {
    bool ok;

    /** 目标方向的绝对方位角（度，自正北顺时针） */
    double targetYawDeg;
    /** 目标方向的绝对俯仰角（度，水平 0，向上为正） */
    double targetPitchDeg;

    /**
     * 还需要机头转多少度才能把目标摆到光轴上（带符号，正 = 需要顺时针转）。
     * M4T 云台 yaw 不可控，这一项**只能**由机头承担。
     */
    double yawErrorDeg;
    /** 还需要云台俯仰转多少度（带符号，正 = 需要抬头） */
    double pitchErrorDeg;

    /** 目标方向与当前光轴的总夹角（度），用于判断"是否已对准" */
    double angularErrorDeg;
} WtAimResult;

/**
 * @brief 由目标像素解出云台该指向的绝对角度。
 *
 * @param gimbal   当前云台指向（作为相机位姿；position 不参与方向解算，可为零）
 * @param u, v     目标像素坐标（像素原点左上，+u 右，+v 下）
 * @param widthPx, heightPx 图像尺寸
 * @param fovHorizDeg, fovVertDeg 视场角
 *        ⚠️ 必须是**水平/垂直** FOV，不是对角线 FOV。DJI 规格页标的 84°
 *        是 M4T 广角的**对角线**值，水平值是 71.59°
 *        （由 `WtCamera_ComputeGeometry()` 算出，勿手填字面量）。
 * @param out      输出
 * @return true 表示解算成功
 */
bool WtAim_FromPixel(const WtCameraPose *gimbal,
                     double u, double v,
                     int widthPx, int heightPx,
                     double fovHorizDeg, double fovVertDeg,
                     WtAimResult *out);

/**
 * @brief 判断是否已对准到可以打激光的程度。
 *
 * @param toleranceDeg 容差（度）。激光光斑本身有张角，且叶尖是细长体，
 *        容差太小会导致永远对不准、反复重试；太大则可能打到叶片中段而非叶尖。
 */
bool WtAim_IsOnTarget(const WtAimResult *aim, double toleranceDeg);

/**
 * @brief 把检出的叶尖按图像极角排成稳定顺序。
 *
 * ## 为什么必须排序
 *
 * 模型输出的三个叶尖是**无序的** —— 它用排列不变损失训练（对 3 个点枚举
 * 6 种排列取误差最小者），因为三片叶片旋转对称、视觉上本就无法区分。
 * 所以 `WtAiDetection.tipU[0]` **不是** NO.1，它只是"模型这一帧恰好先输出
 * 的那个"，下一帧可能就换了。
 *
 * 而激光采集的编号是**由采集顺序定义**的（见 `wt_rotor_fit.h`：
 * `parkPhaseDeg` 取第一个采集点的相位）。两者对接需要一个稳定的排序，
 * 否则后果是：操作员以为在采 NO.1，程序以为在采 NO.2，而**没有任何报错** ——
 * 最终 parkPhase 偏 120°，整条贴叶航线绕着错误的叶片画。
 *
 * ## 排序基准
 *
 * 以轮毂为原点、图像「正上方」为 0°、顺时针递增，取角度最小的为第一片。
 * 选图像极角而不是三维相位：此刻还没有任何三维信息（激光一个点都没打），
 * 图像极角是唯一可用的量。它只需要"稳定"，不需要"物理正确" ——
 * 物理编号由后续拟合出的相位给出。
 *
 * @param det       检测结果，其 tip 数组会被**原地重排**
 * @param outAngles 可选，写回排序后各叶尖的图像极角（度），至少 tipCount 个
 * @return true 表示已排序；tipCount < 2 时无需排序，返回 false
 */
bool WtAim_SortTipsByImageAngle(WtAiDetection *det, double *outAngles);

#ifdef __cplusplus
}
#endif

#endif /* WT_AIM_H */
