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
#define LZ_GEO_NULL_SOLUTION_DEG 0.5

/**
 * 目标自身的最大合理高度（m）。
 *
 * ## 它拦的是什么
 *
 * `LzPole_RecordLaser()` 用「激光点海拔 − 起飞点海拔」当**目标离地高度**，
 * 而这个差有两个已知的污染源：
 *
 * 1. **两个高程的参考面可能不一致** —— `LaserRangingInfo.altitude` 的头文件
 *    只写 `Unit: 0.1m` 没说参考面，而 `ALTITUDE_OF_HOMEPOINT` 原文是
 *    "altitude from sea level ... also uses the ICAO model"（气压高）。
 * 2. **激光飘走打到远处地面**（用户指出的风险）—— 那时量到的是地面，
 *    但离地高度会算出一个不真实的值。
 *
 * ## 为什么给上限而不是给判据
 *
 * 二者都能让差变**大**，而"多大算不合理"没有干净的分界：旗面飘动本身
 * 就有 ±0.5 m，而一个 50 m 的差既可能是参考面错、也可能是真打到了
 * 一栋楼顶上。⇒ **只拦明显荒谬的量**，超限**如实报出并按 0 处理**
 * （不静默取一个看起来合理的数）。这与 `LzPlan_Validate` 拒绝而不钳位
 * 是同一条纪律：**做不到就说做不到，别伪装成做得到。**
 *
 * 取值 60 m：比常见国旗杆（15–30 m）留一倍余量，又远小于参考面
 * 不一致时可能出现的几百米量级偏差。
 */
#define LZ_POLE_TARGET_HEIGHT_MAX_M 60.0

/** @brief 坐标是否落在"零解"邻域内（即当前没有真实定位） */
bool LzGeo_IsNullSolution(const LzGeo *geo);

#endif /* LZ_TYPES_H */
