/**
 * @file lz_types.h
 * @brief 基础类型：坐标系与返回值。
 *
 * 本工程有三层，依赖严格单向：
 *
 *     lz_core（纯算法，不依赖 PSDK、不依赖 OpenCV）
 *        ↑                    ↑
 *     lz_vision（视觉）      lz_app（PSDK 应用）
 *
 * lz_core 不依赖 lz_vision 是刻意的：规划与安全校验是"出错代价最大"的部分，
 * 必须能在桌面上不装 OpenCV、不接飞机就反复验证。视觉只负责把图像变成
 * LzTarget，怎么变的与规划器无关。
 */

#ifndef LZ_TYPES_H
#define LZ_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** WGS84 地理坐标 */
typedef struct {
    double latitudeDeg;  /*!< 纬度，北正 */
    double longitudeDeg; /*!< 经度，东正 */
    double altitudeM;    /*!< 椭球高 m。注意不是相对起飞点的高度 */
} LzGeo;

/** 相对起飞点的 ENU 局部坐标（东-北-天） */
typedef struct {
    double eastM;
    double northM;
    double upM;
} LzEnu;

/** 统一返回值。与 PSDK 的 T_DjiReturnCode 无关 —— lz_core 不认识 PSDK */
typedef enum {
    LZ_OK = 0,
    LZ_ERR_PARAM,       /*!< 入参非法：空指针、越界、非有限数 */
    LZ_ERR_RANGE,       /*!< 数值超出可安全规划的范围 */
    LZ_ERR_UNSAFE,      /*!< 安全校验不通过 */
    LZ_ERR_NO_TARGET,   /*!< 没有可用目标 */
    LZ_ERR_IO,          /*!< 文件读写失败 */
    LZ_ERR_UNSUPPORTED, /*!< 当前机型/相机不支持该能力 */
    LZ_ERR_UPLOAD,      /*!< 文件已生成但**上传被拒**（数据、校验、连接问题） */
    LZ_ERR_START,       /*!< 上传成功但**任务启动被拒**（飞行状态、RC 档位、GPS…） */
    LZ_ERR_NOT_READY,   /*!< 前置条件未就绪：操作员还没做某件事（如未记录杆位） */
} LzStatus;

/** @brief 返回值的可读名字，用于日志 */
const char *LzStatus_Str(LzStatus status);

/** @brief 判断地理坐标是否在合法范围内（含有限性检查） */
bool LzGeo_IsValid(const LzGeo *geo);

/** 判定"零解"的阈值：经纬度绝对值都小于这个值，就认为不是真实定位。
 *
 * ## 为什么需要它 —— `LzGeo_IsValid` 拦不住零解
 *
 * 无定位时融合位置给的是坐标原点附近的**浮点残差**，不是精确的 `(0,0)`。
 * 实测（2026-09-22，M4T 室内）：
 *
 *     lon=0.0000004  lat=0.0000003
 *
 * 这个值在经纬度范围内完全"合法"，`LzGeo_IsValid` 会放行 ——
 * 但它毫无意义，记下来的话圆会画在几内亚湾。
 *
 * ## 阈值为什么取 0.5°
 *
 * 0.5° ≈ 55 km。真实作业点不可能同时距本初子午线与赤道都不到 55 km，
 * 而零解残差是 1e-7 量级 —— 两者之间隔着六个数量级，取值不敏感。
 *
 * ⚠️ **判据必须给邻域，不能写 `== 0.0`。** 这是本项目第二次踩同一形状的坑：
 * 激光在 `distance=0` 时给出的也是"机身位置附近的精确错误值"而非 0。
 * 退化情形总带着一层浮点皮。 */
#define LZ_GEO_NULL_SOLUTION_DEG 0.5

/** @brief 坐标是否落在"零解"邻域内（即当前没有真实定位） */
bool LzGeo_IsNullSolution(const LzGeo *geo);

#endif /* LZ_TYPES_H */
