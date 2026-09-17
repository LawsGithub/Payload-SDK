/**
 * @file wt_turbine.h
 * @brief 风机几何模型与风轮运动学
 *
 * 把一台风机抽象成「塔筒 + 机舱 + N 片叶片」的解析几何体，并给出
 * 叶片上任意点在任意相位角下的三维坐标。航线规划只需在该模型上
 * 采样即可，无需预先存在三维重建结果。
 *
 * 角度约定（全部为「度」）：
 *   - headingDeg      机舱头部方位角，即来流来向。0=正北，顺时针为正。
 *   - tiltDeg         主轴仰角，风轮盘法线相对水平面抬起的角度。
 *   - coneAngleDeg    锥角，叶片离开风轮盘的张角，叶尖偏向上游侧。
 *   - bladePhaseDeg   相位角 ψ，自 12 点钟方向起、沿叶片旋转方向为正。
 *
 * 相位角方向：本模块约定 ψ 增大方向为「从上游看过去的顺时针」，
 * 与主流上风向三叶片风机停机/运行时的旋转方向一致。
 */

#ifndef WT_TURBINE_H
#define WT_TURBINE_H

#include "wt_geometry.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WT_MAX_BLADES 6

/** 风机静态几何参数（来自风机台账 / 机组手册） */
typedef struct {
    WtGeo base;             /*!< 塔基中心 WGS84 坐标 */
    double hubHeight;       /*!< 风轮中心离地高度 m */
    double rotorDiameter;   /*!< 叶轮直径 m（含轮毂，= 2R） */
    double hubRadius;       /*!< 轮毂半径 m，叶片从该半径处开始 */
    double towerBottomDia;  /*!< 塔底外径 m */
    double towerTopDia;     /*!< 塔顶外径 m */
    double nacelleOffset;   /*!< 风轮中心相对塔筒轴线的水平前伸距离 m */
    double coneAngleDeg;    /*!< 锥角 度，典型 0~8 */
    double tiltDeg;         /*!< 主轴仰角 度，典型 3~6 */
    double headingDeg;      /*!< 机舱头部方位角 度 */
    double prebendM;        /*!< 叶尖最大预弯量 m，沿上游方向 */
    int bladeCount;         /*!< 叶片数量，典型 3 */
} WtTurbineSpec;

/** 风轮运动状态 */
typedef struct {
    double phaseDeg;    /*!< 1 号叶片当前相位角 度 */
    double rpm;         /*!< 转速 转/分，0 表示停机 */
    WtEnu rotorCenter;  /*!< 风轮中心（ENU，由 WtTurbine_GetRotorCenter 得到） */
    WtEnu rotAxis;      /*!< 风轮旋转轴单位矢量，指向机舱（下游） */
    WtEnu upRef;        /*!< 风轮平面内「12 点钟」方向单位矢量 */
    WtEnu rotRef;       /*!< 风轮平面内切向单位矢量，ψ 增大方向 */
} WtRotorFrame;

/** 校验结果 */
typedef struct {
    bool ok;              /*!< 参数是否可用 */
    const char *message;  /*!< 不通过时的原因 */
} WtValidateResult;

/**
 * @brief 校验风机几何参数是否自洽
 */
WtValidateResult WtTurbine_Validate(const WtTurbineSpec *spec);

/**
 * @brief 计算风轮中心的 ENU 坐标
 *
 * 风轮盘中心位于塔筒轴线高度 hubHeight 处，沿机舱朝向水平前伸 nacelleOffset。
 */
WtEnu WtTurbine_GetRotorCenter(const WtTurbineSpec *spec);

/**
 * @brief 构建风轮参考系（旋转轴、12 点钟方向、切向方向）
 * @param spec  风机参数
 * @param phaseDeg 1 号叶片相位角
 */
WtRotorFrame WtTurbine_BuildRotorFrame(const WtTurbineSpec *spec, double phaseDeg);

/**
 * @brief 给定相位角，计算某片叶片在风轮平面内的径向单位矢量
 *
 * 注意：这是「未加锥角」的纯径向方向，位于风轮平面内。
 */
WtEnu WtTurbine_BladeRadialDir(const WtRotorFrame *frame, int bladeIndex, const WtTurbineSpec *spec);

/**
 * @brief 计算叶片真实伸出方向（已包含锥角偏置）
 */
WtEnu WtTurbine_BladeAxisDir(const WtRotorFrame *frame, int bladeIndex, const WtTurbineSpec *spec);

/**
 * @brief 叶片上一点的三维坐标
 * @param radialFrac 叶片展向位置，0=叶根（轮毂边缘），1=叶尖
 */
WtEnu WtTurbine_BladePoint(const WtRotorFrame *frame, const WtTurbineSpec *spec,
                           int bladeIndex, double radialFrac);

/**
 * @brief 叶尖三维坐标，等价于 radialFrac = 1
 */
WtEnu WtTurbine_BladeTip(const WtRotorFrame *frame, const WtTurbineSpec *spec, int bladeIndex);

/**
 * @brief 塔筒在指定高度处的外半径（沿高度线性收缩）
 * @param heightAboveBase 相对塔基的高度 m
 */
double WtTurbine_TowerRadiusAt(const WtTurbineSpec *spec, double heightAboveBase);

/**
 * @brief 由实测叶尖位置反解相位角（粗模/视觉识别阶段使用）
 *
 * 做法：把实测叶尖投影到风轮平面，取其相对 12 点钟方向的夹角。
 * 该值已消除锥角带来的平面外分量，因此可直接作为 ψ 使用。
 *
 * @param tipEnu    实测叶尖 ENU 坐标
 * @param bladeIndex 该叶尖对应的叶片序号
 * @param outPhase  输出相位角（度）
 * @return true 表示反解成功（点未退化到风轮轴上）
 */
bool WtTurbine_PhaseFromTip(const WtTurbineSpec *spec, WtEnu tipEnu, int bladeIndex, double *outPhase);

/**
 * @brief 按转速推算 dt 秒之后的相位角
 */
double WtTurbine_AdvancePhase(const WtTurbineSpec *spec, double phaseDeg, double rpm, double dtSec);

#ifdef __cplusplus
}
#endif

#endif /* WT_TURBINE_H */