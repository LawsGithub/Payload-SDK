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
