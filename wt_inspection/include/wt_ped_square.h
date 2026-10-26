/**
 * @file wt_ped_square.h
 * @brief 绕行人正方形航线：以激光打到的行人坐标为基点（几何中心），在四个顶点上
 *        「低 → 高 → 平飞到下一顶点 → 低」，顺时针绕一圈回到起点。
 *
 * 纯算法，桌面可测。KMZ 导出在 `wt_bridge`（`WtBridge_ExportPedSquareKmz`），
 * 上传与执行在 `app/wt_square_mission.c`。
 *
 * ## 用户要的飞法（2026-10-25 原话整理）
 *
 * 以基点为几何中心建正方形，边长 20 m。从第一个顶点开始，高度 10 m（相对起飞点），
 * 垂直爬升到 20 m；顺时针沿边长直线飞到下一顶点，再垂直降到 10 m；依次类推，
 * 最后回到第一个顶点 10 m 处。顶点到顶点的水平飞行中，机头始终朝向基点。
 *
 * ## 航点序列（13 个）
 *
 * ```
 * c0 低 → c0 高 → c1 高 → c1 低 → c1 高 → c2 高 → c2 低
 *       → c2 高 → c3 高 → c3 低 → c3 高 → c0 高 → c0 低
 * ```
 *
 * 每个航点都「到点停」（直线航段，见 KMZ 的 turnMode），所以爬升与下降是
 * 纯垂直的 —— 换成过点不停的模式，拐角会被抹圆，"垂直"就不成立了。
 *
 * ## 用户没指定、这里定下的两条约定（网页核对页上明示，可改）
 *
 * 1. 正方形的边沿正东西、正南北。
 * 2. 「第一个顶点」取**离飞机当前位置最近**的那个：从飞机过去不必横穿正方形，
 *    也就不会从行人头顶飞过。
 *
 * ## 机头朝向不在这里算
 *
 * KMZ 用 `towardPOI`（兴趣点 = 基点），由飞控在航段中**连续**计算朝向 ——
 * 规范原文：「飞行器从该航点飞向下一航点途中机头都将朝向兴趣点」。
 * 这里的 `headingToBaseDeg` 只是各航点上朝向基点的方位，供核对与显示。
 * 若改用 `smoothTransition` 在两顶点之间线性插值角度，沿直线边飞时机头会
 * 偏离基点（直线上指向中心的方位不是线性变化的），不满足「时刻朝向」。
 */

#ifndef WT_PED_SQUARE_H
#define WT_PED_SQUARE_H

#include <stdbool.h>

#include "wt_geometry.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WT_PED_SQUARE_CORNERS   4
/** 起点 1 个 + 每条边「爬升、平飞、下降」3 个 */
#define WT_PED_SQUARE_WAYPOINTS (1 + 3 * WT_PED_SQUARE_CORNERS)

/**
 * 低点高度下限 m。用户要的是 10 m；这条只挡写错的参数（例如写成 0，
 * 飞机会去贴地）。注意它**不是**地形保证：高度相对的是起飞点，行人脚下若
 * 比起飞点高，实际离地会更低 —— 现场要自己看。
 */
#define WT_PED_SQUARE_MIN_HEIGHT_M 5.0

/** 航线参数 */
typedef struct {
    double sideM;            /*!< 正方形边长 m（用户：20） */
    double lowHeightM;       /*!< 低点高度，相对起飞点 m（用户：10） */
    double highHeightM;      /*!< 高点高度，相对起飞点 m（用户：20） */
    double speedMs;          /*!< 航线速度 m/s，wpml 允许 [1, 15] */
    double transitSpeedMs;   /*!< 从当前位置飞往首个航点的速度 m/s，[1, 15] */
    /*
     * 返航高度下限 m。KMZ 的 `globalRTHHeight` 是必需元素：失控返航时飞机
     * 先爬到这个高度再回家，取 max(它, highHeightM)。比航线还低的话返航
     * 第一步就是下降，而飞机正在人旁边。
     */
    double rthHeightM;
    /*
     * 飞机到基点的水平距离上限 m。超过就拒绝 —— 行人是在 60 m 内打的点，
     * 基点远到上百米，多半是激光打到了别处，照着飞就是低空飞去一个错地方。
     */
    double maxBaseDistanceM;
} WtPedSquareConfig;

/** 航点在流程里的角色 */
typedef enum {
    WT_PED_SQUARE_STEP_START = 0, /*!< 首个航点：第一个顶点的低点 */
    WT_PED_SQUARE_STEP_CLIMB,     /*!< 在顶点上垂直爬升到高点 */
    WT_PED_SQUARE_STEP_CRUISE,    /*!< 高点平飞到下一个顶点（机头朝基点） */
    WT_PED_SQUARE_STEP_DESCEND,   /*!< 在顶点上垂直下降到低点 */
} E_WtPedSquareStep;

/** 一个航点 */
typedef struct {
    WtGeo geo;               /*!< 经纬度。alt 不用 —— 高度看 relHeightM */
    WtEnu enu;               /*!< 相对基点的东、北；u = relHeightM */
    double relHeightM;       /*!< 相对起飞点高度（KMZ 的 executeHeight） */
    int corner;              /*!< 所在顶点，按飞行顺序 0..3 */
    E_WtPedSquareStep step;
    double headingToBaseDeg; /*!< 该点朝向基点的方位角 [0, 360)，自正北顺时针 */
} WtPedSquareWaypoint;

/** 规划结果 */
typedef struct {
    WtGeo base;                                 /*!< 基点（兴趣点） */
    WtEnu cornerEnu[WT_PED_SQUARE_CORNERS];     /*!< 各顶点相对基点，按飞行顺序 */
    double cornerBearingDeg[WT_PED_SQUARE_CORNERS]; /*!< 各顶点相对基点的方位 */
    WtPedSquareWaypoint wp[WT_PED_SQUARE_WAYPOINTS];
    int count;

    double pathLengthM;      /*!< 航线内总长（含垂直段，不含飞往首点那段） */
    /*
     * 按 pathLengthM / speedMs 估的耗时 s。每个航点都要刹停再起步，
     * 实际会更长 —— 这是下界，只供显示。
     */
    double durationS;
    double rthHeightM;       /*!< 实际写进 KMZ 的返航高度 */
    double aircraftToBaseM;  /*!< 飞机当前位置到基点的水平距离 */
    double transitM;         /*!< 飞机当前位置到首个航点的水平距离 */
} WtPedSquarePlan;

typedef enum {
    WT_PED_SQUARE_OK = 0,
    WT_PED_SQUARE_ERR_CONFIG,   /*!< 参数非法 */
    WT_PED_SQUARE_ERR_BASE,     /*!< 基点非法（非有限数、越界、零解） */
    WT_PED_SQUARE_ERR_AIRCRAFT, /*!< 飞机位置非法 */
    WT_PED_SQUARE_ERR_TOO_FAR,  /*!< 基点离飞机太远 */
} E_WtPedSquareResult;

/** @brief 默认参数：用户给定的 20 m / 10 m / 20 m */
WtPedSquareConfig WtPedSquare_DefaultConfig(void);

/** @brief 参数是否合法（Build 内部也会查，单独暴露给测试） */
bool WtPedSquare_ConfigValid(const WtPedSquareConfig *cfg);

/**
 * @brief 规划航线。
 *
 * @param cfg      参数，NULL 用默认
 * @param base     基点（激光打到的行人坐标）。只用经纬度
 * @param aircraft 飞机当前位置。用来选第一个顶点、算距离；只用经纬度
 * @param out      结果。失败时已清零；基点太远时 aircraftToBaseM 有效
 */
E_WtPedSquareResult WtPedSquare_Build(const WtPedSquareConfig *cfg, const WtGeo *base,
                                      const WtGeo *aircraft, WtPedSquarePlan *out);

/** @brief 结果码的中文说明（日志与浮窗用） */
const char *WtPedSquare_ResultText(E_WtPedSquareResult r);

/** @brief 航点角色的中文名 */
const char *WtPedSquare_StepName(E_WtPedSquareStep step);

/**
 * @brief 坐标是否像一个真实定位。
 *
 * 除了有限、不越界，还要排除「零解」：没有定位时的经纬度是 0 附近的浮点残差，
 * 能过范围检查却毫无意义 —— 拿它当兴趣点，机头会朝几内亚湾转
 * （姊妹项目 liangzhourenwu 的教训）。
 */
bool WtPedSquare_GeoLooksValid(const WtGeo *g);

#ifdef __cplusplus
}
#endif

#endif /* WT_PED_SQUARE_H */
