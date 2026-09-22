/**
 * @file lz_pole_source.h
 * @brief 绕飞圆心（杆的 WGS84 坐标）从哪来。
 *
 * 这是整条链路上**唯一的外部输入**。前面所有部分都已就绪：
 * 几何、规划、KMZ 生成、控件入口、航点上传。
 *
 * ## 为什么单列一个模块，而不是直接写在 lz_mission.c 里
 *
 * 因为它**有几条完全不同的实现路径**，而选哪条取决于现场条件：
 *
 * | 方式 | 前提 | 精度 | 现状 |
 * |---|---|---|---|
 * | **操作员记录**（当前默认） | 有定位；激光那条还要打中目标 | 取决于记录时瞄准哪 | 本文件 |
 * | 固定坐标 | 无（人工输入） | 取决于输入 | 保留为兜底，见下 |
 * | 视觉识别 + 定位 | `lz_vision` 连通域实现 | 中 | 算法未实现 |
 *
 * 把"取坐标"这件事收在一个接口后面，换方式时只动这一个文件，
 * 规划/上传/控件全都不受影响。
 *
 * ## 操作员记录：两个来源，不是二选一
 *
 * 操作员在 Pilot 2 控件上按按钮记录，两个按钮语义**不同**：
 *
 * - **记录飞机位** —— 存飞机当前所在位置（`TOPIC_POSITION_FUSED`）。
 *   含义是"我就在圆心正上方"。室内有 GPS fix 也能用。
 * - **记录激光点** —— 存激光瞄准点。含义是"我瞄的就是杆"。
 *   精度高（直出经纬度），但**必须打中实物**，无回波时拒绝记录。
 *
 * 两者都写进同一个槽位 —— 因为圆心只有一个，后来的记录覆盖先前的。
 * 记录成功会落盘（`data/pole.txt`），掉电/重启后仍在。
 *
 * ## 未记录时**不**回落固定坐标
 *
 * `LzPole_Acquire()` 在未记录时返回 `LZ_ERR_NOT_READY`，由上层拒绝启动绕飞。
 * 刻意不静默回落到那个写死的坐标：**回落到一个几十公里外的点会让飞机飞过去**，
 * 而操作员以为自己只是在原地绕圈。宁可不起飞 —— 这与
 * "取不到就不起飞"是同一条原则。
 *
 * 固定坐标那套仍然留在代码里，但只在编译时用
 * `-DLZ_POLE_SOURCE_FIXED` 显式选用，运行时不再自动回落。
 */

#ifndef LZ_POLE_SOURCE_H
#define LZ_POLE_SOURCE_H

#include "lz_target.h"
#include "lz_types.h"

/** 记录来源 —— 决定 `LzPole_SourceName()` 的字样与记录时的取数方式 */
typedef enum {
    LZ_POLE_RECORD_AIRCRAFT = 0, /*!< 记录飞机当前位置 */
    LZ_POLE_RECORD_LASER,        /*!< 记录激光瞄准点 */
} LzPoleRecordKind;

/**
 * @brief 取得绕飞圆心
 *
 * @param out [out] 目标杆；`geo` 被填成已记录的坐标
 * @return `LZ_OK` = 取到可用坐标；
 *         `LZ_ERR_NOT_READY` = **还没有记录过**（上层应拒绝启动并提示操作员）
 */
LzStatus LzPole_Acquire(LzTarget *out);

/**
 * @brief 记录飞机当前位置为圆心
 *
 * @param curPos 飞机当前的 WGS84 位置（由调用方从 `TOPIC_POSITION_FUSED` 取，
 *               注意那个话题的经纬度单位是 **rad**，要先转成度）
 * @return `LZ_OK` = 已记录并落盘；`LZ_ERR_NO_TARGET` = 坐标非法（无定位）
 */
LzStatus LzPole_RecordAircraft(const LzGeo *curPos);

/**
 * @brief 记录激光瞄准点为圆心
 *
 * @return `LZ_OK` = 已记录并落盘；
 *         `LZ_ERR_UNSUPPORTED` = 未以 `-DLZ_POLE_SOURCE_LASER` 编译；
 *         `LZ_ERR_NO_TARGET` = 激光无回波或距离为 0（瞄准点会退化成机身位置）
 */
LzStatus LzPole_RecordLaser(void);

/**
 * @brief 从磁盘读回上次记录的圆心（进程启动时调一次）
 *
 * 失败不是错误 —— 首次运行本来就没有文件。返回 `LZ_ERR_NOT_READY`
 * 表示"没有已记录的点"，与"读失败"（`LZ_ERR_IO`）区分开：前者是正常状态。
 */
LzStatus LzPole_LoadRecorded(void);

/** @brief 当前状态下，圆心会来自哪里（用于日志与浮窗消息） */
const char *LzPole_SourceName(void);

/**
 * @brief 已记录的圆心，供界面显示
 * @return `LZ_OK` 时 `out` 被填好；未记录时返回 `LZ_ERR_NOT_READY`
 */
LzStatus LzPole_GetRecorded(LzGeo *out);

#endif /* LZ_POLE_SOURCE_H */
