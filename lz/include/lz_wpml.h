/**
 * @file lz_wpml.h
 * @brief 生成 wpml 航点文件（template.kml + waylines.wpml）。
 */

#ifndef LZ_WPML_H
#define LZ_WPML_H

#include "lz_plan.h"
#include "lz_types.h"

/** 起飞安全高度 m（相对起飞点） */
#define LZ_WPML_TAKEOFF_SECURITY_HEIGHT 20

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
