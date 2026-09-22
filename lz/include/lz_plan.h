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

/** @name 绕飞的安全包线
 *
 * 这组值是**硬约束**，`LzPlan_Validate` 按它拒绝超限的剖面。
 *
 * ⚠️ **唯一真值在这里**。控件层（`app/lz_widget.c` 的滑杆映射）必须引用
 * 同一组常量，不得自备一份 —— 否则两处独立演进，会出现"控件允许 30 m
 * 但校验拒绝 25 m"这种界面与校验打架的情况，而那种矛盾在起飞前
 * 才暴露，操作员看到的是"拨了开关但飞机不动"。
 *
 * 取值的来历（不是技术推导，是现场约束，改动前先确认现场条件）：
 *   - 半径上限 20 m —— 场地约束是"杆周围 20 m 内无建筑物"。
 *     再大就出到这个范围之外了。
 *   - 半径下限 5 m —— 杆高 15 m（`lz_pole_source.c` 的 heightM），
 *     半径过小会让绕飞退化成围绕杆顶盘旋，且 8 点圆周的相邻弦长太短。
 *   - 高度上限 120 m（**相对起飞点**）。注意不是绝对海拔：
 *     KMZ 用 `executeHeightMode=relativeToStartPoint`。
 *   - 高度下限 5 m。
 */
/** @{ */
#define LZ_PLAN_RADIUS_MIN_M     5.0
#define LZ_PLAN_RADIUS_MAX_M     20.0
#define LZ_PLAN_ALTITUDE_MIN_M   5.0
#define LZ_PLAN_ALTITUDE_MAX_M   120.0
/** @} */

/** @name 航点数的允许区间
 *
 * 操作员可在 Pilot 2 的「航点数」输入框里直接填，控件层用
 * `LzPlan_ClampWaypointCount()` 夹到这个区间，`LzPlan_Validate` 用同一组
 * 常量复核 —— **两处必须同源**，理由与上面那组包线相同。
 *
 * 取值的来历：
 *   - 下限 3 —— **几何必然**，两个点连不成圆（原本就写死的判据，搬到这里）。
 *   - 上限 64 —— **现场判断，不是规范限制**。KMZ 协议本身能收 200 个
 *     （DJI Fly 文档"新增航点数量最多 200 个"），但本项目的转弯模式是
 *     `toPointAndStopWithDiscontinuityCurvature`（**到点停**），
 *     n 个点就是 n 次起停。5 m 半径（下限）配 64 点时相邻弦长只剩 0.49 m，
 *     飞机会一直在"停—起步"之间抖；再往上没有实际收益。
 *     若将来改走曲线过点（`toPointAndPassWithContinuityCurvature`），
 *     这个上限应当重新评估。
 */
/** @{ */
#define LZ_PLAN_WAYPOINT_MIN 3
#define LZ_PLAN_WAYPOINT_MAX 64
/** @} */

/**
 * @brief 把操作员填的航点数夹到合法区间
 *
 * ## 为什么是零依赖的独立函数，而不是在控件回调里写两句 if
 *
 * 与 `LzPole_JudgeLaserReading()` 同一个理由：控件层（`app/lz_widget.c`）
 * 依赖 PSDK，在桌面上**编译不了也测不了** —— 夹取逻辑写在那里就等于没有测试。
 * 抽到这里之后 `tests/lz_test_plan.c` 能直接断言边界行为。
 *
 * 夹取而不拒绝：操作员输入框里敲了 100，正确的反应是"按 64 飞并告诉操作员"，
 * 而不是"拨开关没反应"。真正的拒绝留给 `LzPlan_Validate`（起飞前最后一道关）。
 */
int LzPlan_ClampWaypointCount(int count);

/**
 * @brief 航段转弯方式：飞机在航点之间走直线还是走曲线
 *
 * ## 为什么这个开关决定"正多边形"还是"圆"
 *
 * 航点只给出**离散的采样点**，两点之间怎么飞由转弯模式决定：
 *
 *   `LZ_TURN_TO_POINT_AND_STOP`  → 直线飞行、到点停
 *       轨迹是圆周的**内接正多边形**。n 点半径 r 时，边心距 = r·cos(π/n)，
 *       8 点时轨迹比圆**小 7.6%** —— 飞机实际离杆比设定半径近 1.3 m。
 *
 *   `LZ_TURN_PASS_WITH_CURVE`    → 曲线飞行、过点不停、提前转弯
 *       轨迹在航点附近被"抹圆"，**近似圆弧**，边心距回落到接近半径。
 *       这就是 DJI Pilot 2 里「平滑过点，提前转弯」那个组合
 *       （规范 40.common-element.md 的 waypointTurnMode 一行有明确注解）。
 *
 * ⚠️ **走多边形时"航点在圆上"这件事本身是对的** —— 错的只是"飞机贴着两点
 * 连线飞"。所以修法是改转弯模式，不是把航点往里挪。
 *
 * ⚠️ 但注意：本项目生成的首尾**重合**（见 `LzPlan_BuildOrbit`），
 * 多边形是**闭合**的，不是缺一段的开口折线。
 */
typedef enum {
    LZ_TURN_TO_POINT_AND_STOP = 0,  /*!< 直线段、到点停（内接多边形，最保守） */
    LZ_TURN_PASS_WITH_CURVE   = 1,  /*!< 曲线段、过点不停、提前转弯（近似圆） */
} LzTurnMode;

/** 绕飞剖面：一次绕飞的全局参数 */
typedef struct {
    double radiusM;         /*!< 环绕半径：飞机到杆的水平距离 m */
    double altitudeM;       /*!< 环绕高度：相对起飞点 m */
    double speedMs;         /*!< 环绕线速度 m/s */
    int waypointCount;      /*!< 圆周上均分多少个航点，>= 3。
                                 * 实际生成的航点数是它 **+1** —— 末尾补一个与
                                 * 首点重合的收尾点，见 `LzPlan_BuildOrbit`。 */
    double startBearingDeg; /*!< 起始方位角（正北 0°，顺时针），决定从圆的哪一点切入 */
    bool clockwise;         /*!< 绕行方向。true = 俯视顺时针 */
    double gimbalPitchDeg;  /*!< 拍摄时云台俯仰，向下为负 */
    LzTurnMode turnMode;    /*!< 航段走直线还是走曲线。默认走曲线（近似圆） */
} LzOrbitProfile;

/** 一个航点 */
typedef struct {
    LzGeo geo;              /*!< WGS84 位置（高度字段不含相对高，相对高见下） */
    double relativeAltM;    /*!< 相对起飞点高度 m */
    double speedMs;         /*!< 到达该点的速度 m/s */
    double gimbalYawDeg;    /*!< 该点看向杆心的绝对方位角 [0,360)
                                 *
                                 * 同时是两个东西的取值：
                                 *  - wpml 的 `gimbalYawRotateAngle`（云台 yaw）
                                 *  - 机头的目标偏航角 —— M4T 上 `towardPOI`
                                 *    让机头指向杆心，其目标角正是这个值。
                                 *    规范要求两者一致，所以用同一个数。
                                 * 详见 lz_wpml.c 文件头。 */
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
 * 每个航点都带着「看向杆心」的方位角 —— 这是绕飞的核心：飞机在动，
 * 相机要一直盯着杆。M4T 上由**机头**承担这个偏转（`towardPOI`），
 * 因为它的云台 yaw 不能独立于机头转动（见 lz_wpml.c 文件头）。
 *
 * ⚠️ **生成的航点数是 `waypointCount + 1`** —— 末尾补一个与首点**坐标完全
 * 相同**的收尾点，把圆周闭合。`waypointCount` 是"圆周上均分几个方位"，
 * 不是"route 里有几个点"。见下方「关于闭合」一节。
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

/**
 * @brief 建议的"提前转弯截距" m（供 `LZ_TURN_PASS_WITH_CURVE` 用）
 *
 * ## 为什么由几何反算，而不是让调用方填一个值
 *
 * 规范对 `wpml:waypointTurnDampingDist` 有**两条硬约束**
 * （40.common-element.md 的 `<wpml:waypointTurnParam>` 一节）：
 *
 *   1. 取值域 `(0, 航段最大长度]`
 *   2. "两航点间航段长度必需大于两航点转弯截距之和"（即段长 > 2·damping）
 *
 * 这两条都是**相对于航段长度**的，而航段长度由半径和航点数决定 ——
 * 让调用方自己填，就等于要求它同时算出这两个数再倒推，而它多半不会。
 * 这里从 `route` 的真实几何反算，**不可能与实际航段长度不一致**。
 *
 * 取最短航段的 45%：留出 10% 余量避开第 2 条的严格不等式边界。
 *
 * @return 建议截距 m；航点少于 2 个时返回 0（调用方应按 0 处理，即不提前转弯）
 */
double LzPlan_SuggestDampingM(const LzRoute *route);

#endif /* LZ_PLAN_H */
