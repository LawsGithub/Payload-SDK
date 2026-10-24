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

/**
 * @brief 高度基准。
 *
 * ## 为什么必须显式区分
 *
 * PSDK 的两个定位话题给的是**不同基准**的高度，而头文件里只有一句英文说明：
 *
 * ```
 * PositionFused.altitude : "Altitude, WGS 84 reference ellipsoid"   ← 椭球高
 * RtkPosition.hfsl       : "Height above mean sea level"            ← 海拔高
 * ```
 *
 * 中国境内大地水准面相对椭球面的起伏是 **−10 ~ −40 m**。而 ENU 的天向分量
 * 是 `geo->alt − frame->origin.alt` 这样一个**减法** —— 只要两边同基准，
 * 差值就对，基准偏移被抵消；一旦不同基准，就凭空多出几十米误差。
 *
 * 原先两条路径的值都写进同一个 `position.alt` 字段，注释写着"视为一致"。
 * 姊妹项目 `liangzhourenwu` 为完全同类的问题（气压高 vs 椭球高差 38 m）
 * 付出过代价：算出的杆高 50 多米而真值约 16 m，俯仰角方向都反了。
 *
 * ⚠️ **PSDK 不提供 geoid 高**，所以两个基准无法自动换算 —— 只能现场测
 * 或查表，通过 `WtTelemetry_SetGeoidOffset()` 注入。
 */
typedef enum {
    WT_ALT_DATUM_UNKNOWN = 0,
    WT_ALT_DATUM_MSL,        /*!< 海平面（大地水准面）。RTK 的 hfsl、风机台账通常用这个 */
    WT_ALT_DATUM_ELLIPSOID   /*!< WGS84 椭球面。POSITION_FUSED 的 altitude */
} E_WtAltDatum;

/** 一帧解算后的飞机状态 */
typedef struct {
    uint64_t timestampMs;   /*!< 采样时刻（本地单调时钟，已处理 32 位回绕） */
    bool validPosition;     /*!< 位置是否已收到且可信 */
    bool validAttitude;     /*!< 姿态是否已收到 */
    WtGeo position;         /*!< WGS84 位置（若 RTK 已固定则为 RTK 解） */
    E_WtAltDatum altDatum;  /*!< position.alt 的基准，**不要假定** */
    /*
     * enu.u 是否可信。为 false 的典型情形：只有融合定位（椭球高）而
     * geoid 偏移未注入 —— 此时水平分量仍然可用，只有天向分量有几十米偏差。
     */
    bool enuVerticalTrusted;
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

    /*
     * 台账 `turbineSpec.base.alt` 的基准。风机台账通常给海拔，故默认 MSL。
     * 若你的台账用的是椭球高，必须改这里 —— 否则 ENU 天向分量会整体偏。
     */
    E_WtAltDatum specAltDatum;

    /*
     * geoid 偏移 = MSL − 椭球高。中国境内为负（约 −10 ~ −40 m）。
     * 0 表示未知：此时只有融合定位可用时，enu.u 会被标记为不可信。
     *
     * 怎么测：让飞机同时拿到 RTK 与融合定位，两个 alt 相减即为本地 geoid 偏移。
     */
    double geoidOffsetM;

    /* 基准不一致的告警只打一次，避免 10Hz 刷屏 */
    bool datumWarned;

    /*
     * 转速估计的内部状态。
     *
     * 必须保存上次的**测量值**而不是滤波后的值：原先用
     * `measured − rotor.phaseDeg` 算增量，而 rotor.phaseDeg 是低通输出、
     * 滞后于真实值，于是滤波滞后被算进了增量里，rpm **偏大** ——
     * 与那段代码的注释声称的"避免偏小"恰好相反。
     *
     * 偏大的倍数不是小数量级，而是 **1/α**：稳态下低通输出的滞后量正好
     * 使 `measured − filtered = Δ_true / α`。α = 0.25 ⇒ **rpm 偏大 4 倍**
     * （`wt_test_telemetry` 的变异验证实测 3.9998 vs 真值 1.0）。
     */
    double lastMeasuredPhaseDeg;
    uint64_t lastMeasuredAtMs;
    bool haveMeasurement;

    /*
     * 上次检测到有效转动的时刻。
     *
     * 原先的静止判据写的是 `else if (dtMs > 2000)`，而那个 dtMs 是**相邻两次
     * 调用**的间隔 —— 以 10Hz 调用时恒为 100 ms，永远不可能大于 2000，
     * 于是"长时间静止则把转速归零"这段是**死代码**，残留转速会一直外推。
     */
    uint64_t lastMotionAtMs;

    bool initialized;
} WtTelemetry;

/**
 * @brief 注入本地 geoid 偏移（MSL − 椭球高，中国为负值）。
 *
 * 不注入的后果：只有融合定位时 `aircraft.enuVerticalTrusted` 为 false。
 */
void WtTelemetry_SetGeoidOffset(WtTelemetry *telemetry, double offsetM);

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