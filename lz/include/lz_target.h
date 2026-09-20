/**
 * @file lz_target.h
 * @brief 视觉层与规划层之间的**唯一契约**。
 *
 * 这个结构体是整个工程最关键的接缝：一边是视觉（"画面里的杆在哪"），
 * 一边是规划（"圆怎么画、往哪飞"）。它只描述**几何事实**，不含任何
 * 视觉库或 PSDK 类型，因此两件事同时成立：
 *
 *   1. 换视觉算法（HSV 阈值 → 模板匹配 → 深度学习）不必动规划器一行
 *   2. 规划器能用**手写死的**目标数组跑回归测试，不必有图、不必有飞机
 *
 * 本项目的目标固定为**杆状物（国旗杆）**，绕飞需要的几何量只有三个：
 * 杆在哪（WGS84）、杆多高、杆多粗。所以结构体就只装这三个。
 */

#ifndef LZ_TARGET_H
#define LZ_TARGET_H

#include "lz_types.h"

/** 目标在画面中的像素位置（已归一化，与分辨率解耦） */
typedef struct {
    double u;          /*!< 归一化横坐标 [0,1]，0 = 左边缘 */
    double v;          /*!< 归一化纵坐标 [0,1]，0 = 上边缘 */
    double topV;       /*!< 杆顶的归一化纵坐标，用于估高 */
    double bottomV;    /*!< 杆底的归一化纵坐标，用于估高 */
} LzPixelBox;

/** 一根杆（国旗杆） */
typedef struct {
    int id;                /*!< 目标编号，由视觉层分配，同一目标跨帧应一致 */
    LzGeo geo;             /*!< 杆底的 WGS84 位置 —— 视觉 + 机载定位解算得出 */
    double heightM;        /*!< 杆高 m，从杆底到杆顶。<= 0 表示未知 */
    double radiusM;        /*!< 杆半径 m。<= 0 表示未知；绕飞取中心，此值仅用于避让 */
    LzPixelBox pixel;      /*!< 像素位置，主要用于调试与跨帧关联 */
    double confidence;     /*!< 置信度 [0,1]；低于阈值的不应出现在列表里 */
} LzTarget;

/** 目标列表 */
typedef struct {
    LzTarget *items;
    size_t count;
    size_t capacity;
} LzTargetList;

void LzTargetList_Init(LzTargetList *list);
LzStatus LzTargetList_Push(LzTargetList *list, const LzTarget *target);
void LzTargetList_Free(LzTargetList *list);

/** @brief 目标是否可用于规划（位置合法、置信度达标） */
bool LzTarget_IsUsable(const LzTarget *target, double minConfidence);

#endif /* LZ_TARGET_H */
