/**
 * @file wt_geometry.h
 * @brief 风机叶片无人机巡检 —— 坐标与几何基础库
 *
 * 本模块为纯算法实现，不依赖 PSDK / 任何硬件，可在 PC 上直接单元测试。
 * 所有角度输入输出均为「度」，内部换算为弧度。
 *
 * 坐标约定：
 *   - WtGeo   : WGS84 大地坐标 (lat, lon, alt)，alt 为椭球高（单位 m）
 *   - WtEnu   : 以风机塔基为原点的局部切平面直角坐标 (东, 北, 天)，单位 m
 *   - 方位角  : 自正北起、顺时针为正，单位度
 */

#ifndef WT_GEOMETRY_H
#define WT_GEOMETRY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* WGS84 椭球基本参数 */
#define WT_EARTH_A 6378137.0                     /*!< 长半轴 m */
#define WT_EARTH_INV_F 298.257223563             /*!< 扁率倒数 */
#define WT_EARTH_F (1.0 / WT_EARTH_INV_F)        /*!< 扁率 */
#define WT_EARTH_E2 (WT_EARTH_F * (2.0 - WT_EARTH_F)) /*!< 第一偏心率平方 */

#define WT_DEG2RAD 0.017453292519943295
#define WT_RAD2DEG 57.29577951308232

/** 局部 ENU 矢量 */
typedef struct {
    double e; /*!< 东向分量 m */
    double n; /*!< 北向分量 m */
    double u; /*!< 天向分量 m */
} WtEnu;

/** WGS84 大地坐标 */
typedef struct {
    double lat; /*!< 纬度 度 */
    double lon; /*!< 经度 度 */
    double alt; /*!< 椭球高 m */
} WtGeo;

/**
 * @brief 局部切平面参考系
 *
 * 以某一 WGS84 点为原点建立 ENU 切平面。为把投影误差压到毫米级，
 * 不使用「固定地球半径」的简化模型，而是采用该纬度处的卯酉圈半径
 * radiusN 与子午圈半径 radiusM 分别换算东向与北向位移。
 */
typedef struct {
    WtGeo origin;  /*!< 切平面原点 */
    double radiusN; /*!< 卯酉圈曲率半径 m */
    double radiusM; /*!< 子午圈曲率半径 m */
    double cosLat;  /*!< 原点纬度余弦，预计算 */
} WtLocalFrame;

/* ------------------------------------------------------------------ */
/* 参考系                                                              */
/* ------------------------------------------------------------------ */

/**
 * @brief 以 origin 为原点初始化局部切平面参考系
 * @return 初始化后的参考系
 */
WtLocalFrame WtLocalFrame_Init(const WtGeo *origin);

/** @brief 大地坐标 -> 局部 ENU */
WtEnu WtGeo_ToEnu(const WtLocalFrame *frame, const WtGeo *geo);

/** @brief 局部 ENU -> 大地坐标（反算采用一次迭代，风机尺度下足以收敛到毫米级） */
WtGeo WtEnu_ToGeo(const WtLocalFrame *frame, const WtEnu *enu);

/** @brief 两个大地坐标之间的水平距离 m */
double WtGeo_Distance(const WtGeo *a, const WtGeo *b);

/* ------------------------------------------------------------------ */
/* 矢量运算                                                            */
/* ------------------------------------------------------------------ */

WtEnu WtEnu_Add(WtEnu a, WtEnu b);
WtEnu WtEnu_Sub(WtEnu a, WtEnu b);
WtEnu WtEnu_Scale(WtEnu a, double k);
double WtEnu_Dot(WtEnu a, WtEnu b);
WtEnu WtEnu_Cross(WtEnu a, WtEnu b);
double WtEnu_Length(WtEnu a);
double WtEnu_Distance(WtEnu a, WtEnu b);
WtEnu WtEnu_Normalize(WtEnu a);
/** @brief 绕任意单位轴旋转（罗德里格斯公式） */
WtEnu WtEnu_RotateAround(WtEnu v, WtEnu axisUnit, double angleRad);

/**
 * @brief 由方位角、俯仰角构造单位矢量
 * @param azimuthDeg 方位角，自正北顺时针为正
 * @param elevationDeg 俯仰角，水平为 0，向上为正
 */
WtEnu WtEnu_FromAzEl(double azimuthDeg, double elevationDeg);

/** @brief 矢量水平投影的方位角（度，0~360） */
double WtEnu_Azimuth(WtEnu v);

/** @brief 矢量相对水平面的俯仰角（度，向上为正，即 atan2(u, horiz)） */
double WtEnu_Elevation(WtEnu v);

/* ------------------------------------------------------------------ */
/* 实用工具                                                            */
/* ------------------------------------------------------------------ */

double Wt_Clamp(double v, double lo, double hi);
double Wt_DegToRad(double deg);
double Wt_RadToDeg(double rad);
/** @brief 把角度规整到 [0, 360) */
double Wt_Wrap360(double deg);
/** @brief 角度差，结果规整到 (-180, 180] */
double Wt_AngleDiff(double a, double b);
/** @brief 线性插值 */
double Wt_Lerp(double a, double b, double t);

#ifdef __cplusplus
}
#endif

#endif /* WT_GEOMETRY_H */