/**
 * @file lz_wpml.h
 * @brief 生成 wpml 航点文件（template.kml + waylines.wpml）。
 */

#ifndef LZ_WPML_H
#define LZ_WPML_H

#include "lz_plan.h"
#include "lz_types.h"

/** 起飞安全高度 m（相对起飞点）
 *
 * 官方取值域（遥控器场景）：[1.2, 1500]。核实自 Cloud API 文档
 * `60.api-reference/00.dji-wpml/30.waylines-wpml.md` 的 `wpml:takeOffSecurityHeight` 行。 */
#define LZ_WPML_TAKEOFF_SECURITY_HEIGHT 20

/** 返航高度的下限 m
 *
 * `wpml:globalRTHHeight` 是**必需元素**，官方说明是"飞行器返航时，
 * 先爬升至该高度，再进行返航"。若它低于航线高度，返航就变成先下降再返航 ——
 * 所以在 `LzWpml_Build` 里取 `max(航线高度, 本值)`，保证返航高度不会低于航线。 */
#define LZ_WPML_RTH_HEIGHT_FLOOR_M 30

/** 航线结束动作
 *
 * 规范取值域（`30.waylines-wpml.md:130` / `20.template-kml.md:159`，
 * 两份都标**必需元素**）：
 *
 *   `goHome`             完成航线后退出航线模式并**返航**
 *   `noAction`           完成航线后退出航线模式（就地悬停）
 *   `autoLand`           完成航线后退出航线模式并原地降落
 *   `gotoFirstWaypoint`  完成航线后**立即飞向航线起始点**，到达后退出航线模式
 *
 * 本项目用 `gotoFirstWaypoint` —— 用户 2026-09-22 明确要求"结束后回到起点"。
 *
 * ⚠️ 规范里的"航线起始点"是**航线第一个航点**（圆周上 startBearingDeg
 * 那一点，在杆旁边 17.5 m 处），**不是起飞点**。
 *
 * ⚠️ **后果：任务正常结束后飞机停在杆旁边悬停，不返航、不降落。**
 * 与拨 OFF 急停的行为一致（见 `lz_widget.h` 的安全语义），
 * 操作员必须手动接管。这是刻意的：把"接下来怎么办"交给现场的人决定。
 *
 * ⚠️ 配合 `LzPlan_BuildOrbit` 补的那个**与首点重合的收尾点**使用时，
 * 飞机飞完最后一个航点时**已经在起始点上了**，本动作几乎立即结束 ——
 * 既做到了"停在起点"，收尾那一段弧线与机头朝向又都受航线控制。 */
#define LZ_WPML_FINISH_ACTION "gotoFirstWaypoint"

/**
 * @brief 航段是否"贴着两点连线飞"—— **走弧线必须置 0**
 *
 * ## ⚠️ 这个值原先写反了（2026-09-28 修正）
 *
 * 规范原文（`40.common-element.md` 的 `<wpml:waypointTurnParam>` 一节，
 * 与 `30.waylines-wpml.md` 的 `wpml:useStraightLine` 一节一致）：
 *
 * ```text
 * 0：航段轨迹**全程为曲线**
 * 1：航段轨迹**尽量贴合两点连线**
 * ```
 *
 * 原实现写 1，并在注释里称它"与 curve 模式搭配正是 Pilot 2 的
 * 「平滑过点，提前转弯」"—— **那是把两件事混成了一件**：
 *
 * | | 控制什么 | 取值 |
 * |---|---|---|
 * | `waypointTurnMode` | **过点停不停**（要不要减速到点） | `toPointAndPassWithContinuityCurvature` |
 * | `useStraightLine` | **两点之间走直线还是走曲线** | **0 = 曲线** |
 *
 * ⇒ 「平滑过点，提前转弯」= 前者取 `Pass` + 后者取 **0**。
 * 写 1 等于明确要求"尽量贴着两点连线飞" —— 那就是**内接正多边形**，
 * 正是用户反馈的"航点之间机械地折一个角度"。
 *
 * 用户的现场观察佐证了这一点（2026-09-28）：「从一个航点到下一个航点的
 * 瞬间，运动突然很机械地调整小角度」。
 *
 * ## 为什么不能只靠"把转弯截距调大"来解决
 *
 * 截距（`waypointTurnDampingDist`）只在**曲线模式**下有意义
 * （规范标注：该元素仅在 `coordinateTurn` 或
 * `toPointAndPassWithContinuityCurvature` + `useStraightLine=1` 时必需）。
 * 在"贴直线"模式下，截距再大也只是把拐点提前一点，轨迹仍是折线。
 */
#define LZ_WPML_USE_STRAIGHT_LINE 0

/**
 * @brief 逐点 `gimbalRotate` 动作里是否使能 **yaw**
 *
 * ## 为什么置 0（2026-09-28 修正）
 *
 * 逐点下发 `gimbalRotate` 是**为了让云台俯仰在任务开始时到位** ——
 * 这是必需的，因为 WPML 的 `towardPOI` **只管水平方向**（规范原文：
 * "目前不支持 Z 方向朝向兴趣点，高度可设置为0"），垂直方向没有
 * 别的元素能替代它。
 *
 * 但 **yaw 不该由这个动作下发**：
 *
 * 1. **机头已经负责了这件事。** 每个航点都写了
 *    `waypointHeadingMode=towardPOI` + `waypointPoiPoint` 指向杆心，
 *    机头（因而光轴）在**整个航段上连续**地朝向圆心。
 * 2. **逐点下发 yaw 会把那个连续性打断。** `gimbalRotate` 的触发器是
 *    `reachPoint` —— 到点才执行。于是飞行途中云台 yaw 被"停在"上一个
 *    航点的角度，每次到点才**跳**一次。用户现场看到的「到点瞬间突然
 *    机械地调整一个小角度」正是这个。
 * 3. **规范对本机型有硬要求**（`40.common-element.md` 的 `gimbalRotate` /
 *    `orientedShoot` / `rotateYaw` 三处都标）：
 *    `wpml:gimbalYawRotateAngle` 与 `wpml:aircraftHeading` 需保持一致，
 *    而 M4T 那一条注明确列在机型栏里。
 *    `towardPOI` 算出的 `aircraftHeading` 是**连续变化**的，任何
 *    "到点才更新"的离散值都必然与它不一致 —— 这一条本身就说明
 *    逐点写 yaw 是错的路子。
 *
 * ⇒ 只保留 pitch：云台 yaw 由 `towardPOI` 的机头跟随带着走（M4T 的 pan
 * 轴本来就不能独立于机头偏转，见 lz/doc/WPML-ORBIT.md「绕飞怎么让相机盯着杆」）。
 *
 * ⚠️ `gimbalYawRotateAngle` 元素**仍然照写**（规范标为必需元素），
 * 只是 `Enable` 置 0 —— 它的值仍是该点看向杆心的方位角，
 * 与飞机自己算的 `aircraftHeading` 一致。
 */
#define LZ_WPML_GIMBAL_YAW_IN_ACTION 0

/**
 * 机型 / 负载身份。
 *
 * ⚠️ 这两个值**必须与实际机型负载匹配**，否则飞机可能拒绝执行航线。
 * 取值见 `psdk_lib/include/dji_typedef.h` 的 `E_DjiAircraftType` /
 * `E_DjiCameraType`（核实命令：
 *   grep -n "DJI_AIRCRAFT_TYPE_M4T\|DJI_CAMERA_TYPE_M4T" dji_typedef.h
 * ）
 * 默认给 M4T / M4T 相机（99 / 1 与 89 / 0）。
 */
typedef struct {
    int droneEnumValue;      /*!< 默认 99  = DJI_AIRCRAFT_TYPE_M4T */
    int droneSubEnumValue;   /*!< 默认 1   = M4T 子类型 */
    int payloadEnumValue;    /*!< 默认 89  = DJI_CAMERA_TYPE_M4T */
    int payloadSubEnumValue; /*!< 默认 0 */
} LzWpmlIdentity;

/** 生成结果：两份 XML，由调用方 free */
typedef struct {
    char *templateKml;
    char *waylinesWpml;
    LzWpmlIdentity identity;   /*!< 调用前填好；用 LzWpml_DefaultIdentity 取默认 */
} LzWpmlFiles;

/** @brief M4T 的默认身份 */
LzWpmlIdentity LzWpml_DefaultIdentity(void);

/**
 * @brief 由航线生成两份 wpml XML
 *
 * 每个航点写入一个 `gimbalRotate` 动作：`gimbalRotateMode=absoluteAngle`、
 * `gimbalYawRotateEnable=1`、角度取 `wp->gimbalYawDeg`（绝对方位角指向杆心）。
 */
LzStatus LzWpml_Build(const LzRoute *route,
                      const LzTarget *pole,
                      const LzOrbitProfile *profile,
                      LzWpmlFiles *out);

void LzWpml_Free(LzWpmlFiles *files);

#endif /* LZ_WPML_H */
