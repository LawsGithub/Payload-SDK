/**
 * @file lz_plan.h
 * @brief 规划层：把「杆在哪」变成「绕飞的圆怎么画」。
 *
 * 本层**不依赖 PSDK、不依赖任何视觉库**，输入全是纯几何量。
 * 这是刻意的分界线：航线一旦算错，代价是撞杆或漏拍，而这类错误
 * 只有在桌面上能反复复现才有机会被发现。
 */

#ifndef LZ_PLAN_H
#define LZ_PLAN_H

#include "lz_target.h"

/** 绕飞剖面：一次绕飞的全局参数 */
typedef struct {
    double radiusM;         /*!< 环绕半径：飞机到杆的水平距离 m */
    double altitudeM;       /*!< 环绕高度：相对起飞点 m */
    double speedMs;         /*!< 环绕线速度 m/s */
    int waypointCount;      /*!< 圆周上均分多少个航点，>= 3 */
    double startBearingDeg; /*!< 起始方位角（正北 0°，顺时针），决定从圆的哪一点切入 */
    bool clockwise;         /*!< 绕行方向。true = 俯视顺时针 */
    double gimbalPitchDeg;  /*!< 拍摄时云台俯仰，向下为负 */
} LzOrbitProfile;

/** 一个航点 */
typedef struct {
    LzGeo geo;              /*!< WGS84 位置（高度字段不含相对高，相对高见下） */
    double relativeAltM;    /*!< 相对起飞点高度 m */
    double speedMs;         /*!< 到达该点的速度 m/s */
    double gimbalYawDeg;    /*!< 该点云台偏航（绝对方位角，指向杆心） */
    double gimbalPitchDeg;  /*!< 该点云台俯仰 */
} LzWaypoint;

/** 航线 */
typedef struct {
    LzWaypoint *points;
    size_t count;
    size_t capacity;
} LzRoute;

void LzRoute_Init(LzRoute *route);
void LzRoute_Free(LzRoute *route);

/**
 * @brief 为**一根杆**生成绕飞航线（实现在 lz_plan.c，含方向约定的推导）
 *
 * 每个航点的云台偏航应指向杆心 —— 这是绕飞的核心：飞机在动，光轴要一直盯着杆。
 *
 * @param target   目标杆
 * @param takeoff  起飞点 WGS84（提供经度/纬度基准；高度用 relativeAltM 表达）
 * @param profile  绕飞剖面
 * @param route    [out] 生成的航线；调用前需 LzRoute_Init
 */
LzStatus LzPlan_BuildOrbit(const LzTarget *target,
                           const LzGeo *takeoff,
                           const LzOrbitProfile *profile,
                           LzRoute *route);

/**
 * @brief 安全校验：高度、半径、航点数、云台限位是否合理
 *
 * 与 BuildOrbit 分开：校验规则会随现场飞行安全要求变化，
 * 而航线生成逻辑相对稳定，两者独立演进、独立测试。
 */
LzStatus LzPlan_Validate(const LzRoute *route, const LzOrbitProfile *profile);

#endif /* LZ_PLAN_H */
