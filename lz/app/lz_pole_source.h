/**
 * @file lz_pole_source.h
 * @brief 绕飞圆心（杆的 WGS84 坐标）从哪来。
 *
 * 这是整个工程**最后一处未接通的环节**。前面所有部分都已就绪：
 * 几何、规划、KMZ 生成、控件入口、航点上传，全都验证过了；
 * 缺的只是"杆在哪"这一个输入。
 *
 * ## 为什么单列一个模块，而不是直接写在 lz_mission.c 里
 *
 * 因为它**至少有三条完全不同的实现路径**，而选哪条取决于现场条件：
 *
 * | 方式 | 前提 | 精度 | 现状 |
 * |---|---|---|---|
 * | **激光测距** | 激光打中杆 + GPS 锁定 | 高（直出经纬度） | 硬件**已实测可用**，见 API-MAP §八 |
 * | **视觉识别 + 定位** | `lz_vision` 连通域实现 | 中 | 算法未实现 |
 * | **固定坐标** | 无（人工输入） | 取决于输入 | **当前默认** |
 *
 * 把"取坐标"这件事收在一个接口后面，换方式时只动这一个文件，
 * 规划/上传/控件全都不受影响。
 *
 * ## 激光测距那条路的关键约束
 *
 * `[V]` 2026-09-19 实测：测距硬件在 **位置 1**（`PAYLOAD_PORT_NO1`，
 * 即 M4T 自带云台相机），`enable_lidar=1`。但 `exception` 字段的取值
 * 官方未公开，实测反推出：**1 = 无回波，3 = 正常读数，2 = 过渡态**。
 * 取坐标前必须判 `exception`，否则会把 `lat/lon = 0,0` 当成真坐标。
 */

#ifndef LZ_POLE_SOURCE_H
#define LZ_POLE_SOURCE_H

#include "lz_target.h"
#include "lz_types.h"

/**
 * @brief 取得绕飞圆心
 *
 * @param out [out] 目标杆；`geo` 必须被填成合法坐标
 * @return LZ_OK 表示取到了可用的坐标
 */
LzStatus LzPole_Acquire(LzTarget *out);

/** @brief 当前用的是什么方式，用于日志与浮窗消息 */
const char *LzPole_SourceName(void);

#endif /* LZ_POLE_SOURCE_H */