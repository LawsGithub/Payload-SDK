/**
 * @file lz_bridge.h
 * @brief 桥接层：规划结果 → PSDK 可执行的航点文件。
 *
 * 本层只做**格式转换**，不含任何几何计算（与 wt_inspection 的 wt_bridge 同构）。
 * 换算集中在一处的理由：航点文件里写的是"相对坐标 + 动作"，规划器给出的是
 * "绝对方位角/俯仰角"，换算固定但容易错，散落各处就无法核对。
 */

#ifndef LZ_BRIDGE_H
#define LZ_BRIDGE_H

#include "lz_geo.h"
#include "lz_plan.h"

/**
 * @brief 把航线导出为 DJI 航点 KMZ
 *
 * 产物遵循 wpml 约定，供 DjiWaypointV3_UploadKmzFile() 消费：
 *     wpmz/template.kml   航线与航点（含相对高度、速度、云台动作）
 *     wpmz/waylines.wpml  可执行航线
 *
 * @note 本项目用 Waypoint **V3**（KMZ 上传）而非 V2（结构体上传）——
 *       V3 的逐点云台动作表达力更强，且与 Pilot 的航线文件互通，
 *       现场可以用 Pilot 手工复核我们生成的航线。核实命令：
 *       grep -n "DjiWaypointV3_UploadKmzFile" psdk_lib/include/dji_waypoint_v3.h
 */
LzStatus LzBridge_ExportKmz(const LzRoute *route,
                            const LzTarget *pole,
                            const LzOrbitProfile *profile,
                            const char *outPath);

/** @brief 导出 CSV 便于人工核对航线 */
LzStatus LzBridge_ExportCsv(const LzRoute *route, const char *path);

#endif /* LZ_BRIDGE_H */
