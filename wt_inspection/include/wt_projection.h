/**
 * @file wt_projection.h
 * @brief 像素 ↔ 三维：相机反投影与叶轮盘面求交。
 *
 * ## 这条链路在做什么
 *
 * AI 给出的是**像素坐标**（轮毂中心、叶尖在哪一格）。要变成航线规划能用的
 * **经纬度**，必须走完：
 *
 *   像素 (u,v)
 *     ↓ 相机内参（焦距、主点）
 *   相机坐标系方向矢量
 *     ↓ 云台姿态
 *   机体坐标系方向矢量
 *     ↓ 飞机姿态（yaw/pitch/roll）
 *   ENU 方向矢量
 *     ↓ 与叶轮盘面求交
 *   盘面上的三维点
 *     ↓ WtEnu_ToGeo
 *   经纬度
 *
 * ## 为什么用「与盘面求交」而不是「从像素猜距离」
 *
 * 单张图的一个像素只确定一条射线，缺一个自由度。常规做法是假设目标在地面
 * 然后与地形求交，但风机叶片在几十米高处，这个假设不成立。
 *
 * 而**叶轮盘面的位置是已知几何**：中心 = `WtTurbine_GetRotorCenter()`，
 * 法向 = `rotorFrame.rotAxis`。所以射线与这个已知平面求交，距离就确定了，
 * 不需要任何猜测。这也是为什么本模块要求先有 `WtRotorFrame`。
 *
 * ## 精度瓶颈在哪
 *
 * 不是求交运算，是**内参**。当前 `WtCameraGeometry` 由规格页的等效焦距推得，
 * 没有经过标定。棋盘格标定能把重投影误差从"未知"压到 1 px 以内，
 * 对 50 m 外的目标意味着米级 vs 分米级的差别。现场作业前必须标定。
 */

#ifndef WT_PROJECTION_H
#define WT_PROJECTION_H

#include <stdbool.h>

#include "wt_geometry.h"
#include "wt_turbine.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 相机位姿：把相机坐标系转到 ENU。
 *
 * 相机坐标系约定（与 DJI 一致）：
 *   x → 右（图像 +u 方向）
 *   y → 下（图像 +v 方向）
 *   z → 前（光轴，指向被摄物体）
 */
typedef struct {
    WtEnu position;     /*!< 相机光心在 ENU 中的位置 */
    double yawDeg;      /*!< 光轴方位角，自正北顺时针 */
    double pitchDeg;    /*!< 光轴俯仰角，水平 0，向上为正 */
    double rollDeg;     /*!< 相机绕光轴的滚转角 */
} WtCameraPose;

/** 反投影结果 */
typedef struct {
    WtEnu point;        /*!< 交点在 ENU 中的位置 */
    double distanceM;   /*!< 光心到交点的斜距 m */
    bool hit;           /*!< 是否命中（射线与盘面近乎平行时为 false） */
} WtProjectionResult;

/**
 * @brief 由相机内参构造「像素 → 相机坐标系方向」的换算参数。
 *
 * @param imageWidthPx, imageHeightPx 图像尺寸
 * @param fovHorizDeg, fovVertDeg     水平/垂直视场角（由 WtCamera_ComputeGeometry 得到）
 * @param outTanHalfH 输出 tan(fovH/2)
 * @param outTanHalfV 输出 tan(fovV/2)
 * @return true 表示参数合法
 */
bool WtProjection_SetupIntrinsics(int imageWidthPx, int imageHeightPx,
                                  double fovHorizDeg, double fovVertDeg,
                                  double *outTanHalfH, double *outTanHalfV);

/**
 * @brief 像素坐标 → 相机坐标系单位方向矢量。
 *
 * 像素原点在左上角，+u 向右，+v 向下。返回的方向矢量已归一化。
 *
 * @param u, v         像素坐标（可用亚像素）
 * @param widthPx, heightPx 图像尺寸
 * @param tanHalfH, tanHalfV 由 WtProjection_SetupIntrinsics 得到
 * @param out 输出方向（相机坐标系）
 * @return true 表示成功
 */
bool WtProjection_PixelToCameraRay(double u, double v,
                                   int widthPx, int heightPx,
                                   double tanHalfH, double tanHalfV,
                                   WtEnu *out);

/**
 * @brief 相机坐标系方向 → ENU 方向。
 *
 * 按 roll → pitch → yaw 的顺序把相机系转到 ENU。
 */
WtEnu WtProjection_CameraToEnu(const WtCameraPose *pose, WtEnu dirCam);

/**
 * @brief 射线与平面求交。
 *
 * 平面由「过一点 + 法向」定义。射线为 origin + t*dir（t>0）。
 *
 * @param rayOrigin 射线起点
 * @param rayDir    射线方向（不必归一化）
 * @param planePoint 平面上一点
 * @param planeNormal 平面法向
 * @param out 输出结果
 * @return true 表示求交成功（射线不与平面平行，且交点在 t>0 侧）
 */
bool WtProjection_RayPlaneIntersect(WtEnu rayOrigin, WtEnu rayDir,
                                    WtEnu planePoint, WtEnu planeNormal,
                                    WtProjectionResult *out);

/**
 * @brief 完整链路：像素 → 与叶轮盘面交点。
 *
 * @param pose        相机位姿
 * @param u, v        像素坐标
 * @param widthPx, heightPx 图像尺寸
 * @param fovHorizDeg, fovVertDeg 视场角
 * @param rotorFrame  叶轮盘面（提供中心与法向）
 * @param out         输出
 * @return true 表示成功命中
 */
bool WtProjection_PixelToRotorPlane(const WtCameraPose *pose,
                                    double u, double v,
                                    int widthPx, int heightPx,
                                    double fovHorizDeg, double fovVertDeg,
                                    const WtRotorFrame *rotorFrame,
                                    WtProjectionResult *out);

/**
 * @brief 把交点换算成「叶片径向位置」。
 *
 * 用于回答"这个点落在叶片的哪个径向位置"—— 是编号与后续规划都要的信息。
 *
 * @param rotorFrame 叶轮盘面
 * @param point      盘面上的点（ENU）
 * @param outRadialM 输出：距轮毂中心的径向距离 m
 * @param outAngleDeg 输出：该点在盘面内的相位角（度，0~360，与 park_phase 同一约定）
 * @return true 表示成功
 */
bool WtProjection_PointToRotorCoords(const WtRotorFrame *rotorFrame,
                                     WtEnu point,
                                     double *outRadialM,
                                     double *outAngleDeg);

#ifdef __cplusplus
}
#endif

#endif /* WT_PROJECTION_H */
