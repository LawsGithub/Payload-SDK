/**
 * @file lz_geo.h
 * @brief 大地几何工具：距离、方位角、按方位角推算坐标。
 *
 * 这些是**标准公式**，实现固定、无设计余地，所以直接写好。
 * 「怎么用它们排航线」才是设计决策，那部分在 lz_plan.c。
 *
 * 采用球面近似（地球半径 6371008.8 m）而非 WGS84 椭球：本项目的作业
 * 尺度是几十到几百米，球面近似在该尺度上的误差远小于 GPS 自身的抖动。
 */

#ifndef LZ_GEO_H
#define LZ_GEO_H

#include "lz_types.h"

/** 地球平均半径 m */
#define LZ_EARTH_RADIUS_M 6371008.8

/** @brief 两点间的球面距离 m */
double LzGeo_DistanceM(const LzGeo *a, const LzGeo *b);

/** @brief a → b 的初始方位角，单位度，正北为 0，顺时针为正，范围 [0,360) */
double LzGeo_BearingDeg(const LzGeo *a, const LzGeo *b);

/**
 * @brief 从 from 出发，沿 bearingDeg 走 distanceM 后的坐标
 * @note altitudeM 原样保留（本函数只处理水平位置）
 */
LzStatus LzGeo_Destination(const LzGeo *from, double bearingDeg, double distanceM, LzGeo *out);

/** @brief 把方位角归一化到 [0,360) */
double LzGeo_NormalizeDeg(double deg);

/** @brief 返回两个方位角之间的最小夹角（带符号，范围 (-180,180]） */
double LzGeo_AngleDiffDeg(double a, double b);

#endif /* LZ_GEO_H */
