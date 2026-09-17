/**
 * @file wt_telemetry.h
 * @brief 机载遥测采集与解算
 *
 * 把 PSDK 的订阅话题收敛成巡检业务需要的几个量：飞机位姿、风轮相位、
 * 电池状态。上层运行器只消费解算后的结果，不直接接触 PSDK 话题，
 * 这样在 PC 上也能用录制的数据回放整条控制逻辑。
 */

#ifndef WT_TELEMETRY_H
#define WT_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "wt_geometry.h"
#include "wt_turbine.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 一帧解算后的飞机状态 */
typedef struct {
    uint64_t timestampMs;   /*!< 采样时刻（本地单调时钟） */
    bool validPosition;     /*!< 位置是否已收到且可信 */
    bool validAttitude;     /*!< 姿态是否已收到 */
    WtGeo position;         /*!< WGS84 位置（若 RTK 已固定则为 RTK 解） */
    WtEnu enu;              /*!< 相对塔基的 ENU 坐标，由上层按当前目标风机填充 */
    double yawDeg;          /*!< 机头方位角，自正北顺时针 */
    double pitchDeg;        /*!< 机体俯仰角 */
    double rollDeg;         /*!< 机体横滚角 */
    double velocityMs;      /*!< 合速度 */
    double heightAboveGroundM; /*!< 相对起飞点高度 */
    uint16_t visibleSatellites;
    uint8_t flightStatus;   /*!< 0=停桨 1=地面转桨 2=空中 */
    bool rtkFixed;          /*!< RTK 是否处于固定解 */
} WtAircraftState;

/** 一帧解算后的风轮状态 */
typedef struct {
    bool valid;             /*!< 相位是否已解算出来 */
    double phaseDeg;        /*!< 1 号叶片相位角 */
    double rpm;             /*!< 估计转速，0 表示静止 */
    uint64_t updatedAtMs;   /*!< 相位解算时刻 */
} WtRotorState;

/** 云台当前指向 */
typedef struct {
    bool valid;
    double yawDeg;
    double pitchDeg;
    double rollDeg;
} WtGimbalState;

/** 遥测上下文 */
typedef struct {
    WtAircraftState aircraft;
    WtRotorState rotor;
    WtGimbalState gimbal;
    WtTurbineSpec turbineSpec;   /*!< 当前作业风机，用于把位置转 ENU、解算相位 */
    WtLocalFrame frame;
    bool initialized;
} WtTelemetry;

/**
 * @brief 初始化遥测：订阅 PSDK 话题并注册回调
 * @note 必须在 DjiCore_Init 之后、且处于用户任务中调用
 * @param telemetry 遥测上下文
 * @param spec      当前作业风机参数（用于相位反解）
 */
bool WtTelemetry_Init(WtTelemetry *telemetry, const WtTurbineSpec *spec);

/** @brief 停止订阅并释放资源 */
void WtTelemetry_DeInit(WtTelemetry *telemetry);

/**
 * @brief 拉取一次最新值，刷新 aircraft / gimbal 字段
 *
 * 采用「主动拉取」而非纯回调：控制循环本身是周期性的，
 * 拉取能保证每一帧用到的都是同一时刻的一致快照。
 */
void WtTelemetry_Poll(WtTelemetry *telemetry);

/**
 * @brief 由当前飞机的相机视线反解风轮相位
 *
 * 思路：云台指向已知，若此刻画面中的叶片被识别出叶尖方向，
 * 即可由「云台方位角 + 图像内叶尖相对光轴的角度」推得叶尖方位，
 * 再调用 WtTurbine_PhaseFromTip 反解出相位角。
 *
 * @param azOfQuadrantDeg 图像内叶尖相对画面中心的方向（方位角，度）
 * @param pitchOfQuadrantDeg 图像内叶尖相对画面中心的俯仰（度）
 * @param outRotor 输出风轮状态
 * @return true 表示反解成功
 */
bool WtTelemetry_SolvePhaseFromVision(WtTelemetry *telemetry,
                                       double azOfQuadrantDeg,
                                       double pitchOfQuadrantDeg,
                                       WtRotorState *outRotor);

/**
 * @brief 平滑更新相位估计
 *
 * 视觉反解是逐帧带噪的，直接使用会让云台抖动。这里做一阶低通，
 * 并按两次观测的时间差与相位差估计转速，供后续帧外推。
 */
void WtTelemetry_UpdatePhaseEstimate(WtTelemetry *telemetry, double measuredPhaseDeg,
                                     uint64_t sampleTimeMs);

/** @brief 把当前飞机位置换算为相对塔基的 ENU 并写回 aircraft.enu */
void WtTelemetry_RefreshEnu(WtTelemetry *telemetry);

/** @brief 用当前相位外推出 dtSec 之后的风轮参考系，供云台提前指向 */
WtRotorFrame WtTelemetry_PredictRotorFrame(const WtTelemetry *telemetry, double dtSec);

/** @brief 打印一帧遥测摘要 */
void WtTelemetry_LogSnapshot(const WtTelemetry *telemetry);

#ifdef __cplusplus
}
#endif

#endif /* WT_TELEMETRY_H */