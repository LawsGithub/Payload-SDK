/**
 * @file wt_ped.h
 * @brief 高空行人瞄准打点的**接线层**：把决策层接到视觉、云台、激光上。
 *        （ped = pedestrian，行人）
 *
 * ## 整条链路
 *
 * ```
 * 取流(wt_frame_pump) → 推理(wt_vision，VisDrone 检测模型) → 行人框
 *      ↓
 * 决策(wt_ped_logic)：选人 → 转云台(wt_laser) → 等停稳 → 锁定后连续 3 帧确认
 *      ↓
 * 连读 3 次激光(wt_laser)取中位 → WGS84 落点 → 日志 + data/ped_points.csv + Pilot 浮窗
 * ```
 *
 * ## 分层
 *
 * 判据与状态转移**不在这里**，在 `wt_ped_logic`（wt_core，桌面可测）。
 * 本文件只做有副作用的事：采观测、下发云台、读激光、记结果。
 *
 * ## 云台模式的借与还
 *
 * `WtPed_Begin` 借 FREE（锁定地面姿态，飞机晃动时光轴不跟着晃）；
 * **归还由调用方在会话结束时做**（`WtLaser_ReturnFollowMode`），和激光采集
 * 同一个约定 —— `dji_app_ctl stop` 直接杀进程，挂在 DeInit 上等于永远不还。
 */

#ifndef WT_PED_H
#define WT_PED_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "wt_laser.h"
#include "wt_ped_logic.h"
#include "wt_telemetry.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WT_PED_PATH_MAX 256

/** 一次瞄准打点会话 */
typedef struct {
    WtPedLogic logic;           /*!< 决策层状态（判据都在它那边） */

    double zoomFactor;          /*!< 最近一次读到的变焦倍率（读不到按 1） */
    bool zoomEverRead;
    uint32_t lastZoomPollMs;

    WtPedDecision last;         /*!< 最近一次决策，供浮窗 */
    E_WtPedState loggedState;   /*!< 已经打过日志的状态，用于只在变化时打 */
    uint32_t lastStatusLogMs;

    /* 打点结果 */
    bool havePoint;
    WtLaserSample sample;       /*!< 几次读数里选中的那一次（距离居中） */
    int laserCount;             /*!< 这一轮有效读数的次数 */
    double laserSpreadM;        /*!< 这一轮读数的离散度（最大 − 最小距离） */
    WtPedTrack fireTrack;       /*!< 开打那一刻的目标 */
    double fireGimbalPitchDeg;
    double fireGimbalYawDeg;
    double fireBodyYawDeg;
    char csvPath[WT_PED_PATH_MAX];
} WtPedSession;

/**
 * @brief 开始会话：借云台 FREE 模式、读一次变焦。
 *
 * @param cfg       决策参数，NULL 用默认（锁定后连续 3 帧确认即打点）
 * @param outputDir 结果 CSV 的目录（相对应用目录，例如 "data"）
 */
bool WtPed_Begin(WtPedSession *s, const WtPedConfig *cfg, const char *outputDir);

/**
 * @brief 推进一步，每个控制周期调一次（10 Hz）。
 *
 * @note 打激光那一步内部调 `WtLaser_Read()`，是**阻塞调用**（实测最多 1.2 s）。
 *       只能在主循环里调，**绝不能在 PSDK 回调线程里调**。
 */
void WtPed_Tick(WtPedSession *s, const WtTelemetry *tele);

/** @brief 中止会话 */
void WtPed_Abort(WtPedSession *s, const char *why);

/** @brief 会话是否已结束（成功或失败） */
bool WtPed_IsFinished(const WtPedSession *s);

/** @brief 把会话进度格式化成一行，供浮窗显示（带宽上限 2 KB/s，要短） */
void WtPed_FormatProgress(const WtPedSession *s, char *buf, size_t bufLen);

#ifdef __cplusplus
}
#endif

#endif /* WT_PED_H */
