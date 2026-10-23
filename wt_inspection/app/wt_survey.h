/**
 * @file wt_survey.h
 * @brief 激光采集会话：依次对轮毂与三个叶尖打点，拟合出可用于规划的盘面。
 *
 * ## 整条链路
 *
 * ```
 *   取流(wt_frame_pump) → 推理(wt_vision) → 稳定排序的 4 个像素
 *        ↓
 *   逐个目标：算目标指向(wt_aim) → 转云台 pitch(wt_laser) → 等稳 → 打激光
 *        ↓
 *   4 个 WGS84 点 → 转 ENU → 拟合(wt_rotor_fit) → WtRotorFrame
 *        ↓
 *   喂给既有规划链 WtPlan_BladeTrace，生成贴叶 KMZ
 * ```
 *
 * ## 为什么是「半自动」而不是全自动
 *
 * M4T 的云台 Pan 轴 `Not controllable`，水平对准只能靠机头。而转机头需要
 * 飞控控制权 —— 让一个采集模块去操纵飞机的朝向，风险与收益不成比例。
 *
 * 所以分工是：
 *   - **俯仰**：程序自动转云台（这个轴可控，且是误差的主要来源）
 *   - **水平**：程序算出还差多少度，交给操作员转机头（`yawErrorDeg`）
 *
 * 这与姊妹项目「记录激光点」按钮的模式一致：程序负责判定与采集，
 * 操作员负责把飞机摆对。
 *
 * ## 编号由采集顺序定义
 *
 * 模型输出的叶尖无序，所以 `wt_vision` 先按图像极角做了稳定排序；
 * 本模块按那个顺序依次采集，`wt_rotor_fit` 又把第一个采集点的相位当作
 * 停用角。三者是同一个约定的三段，改任何一段都要同时改另外两段。
 *
 * ## 这里产出的停用角，正是既有流程拒绝执行的那个前提
 *
 * `WtRunner_PlanTurbine` 在停用角未知时**无条件拒绝规划**（贴叶航线按
 * 「叶片停在 12 点」假设画出来，假设不成立就是撞机）。本模块把它测出来，
 * 让那条路走得通。
 */

#ifndef WT_SURVEY_H
#define WT_SURVEY_H

#include <stdbool.h>
#include <stdint.h>

#include "wt_geometry.h"
#include "wt_rotor_fit.h"
#include "wt_telemetry.h"
#include "wt_turbine.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 采集目标，顺序即编号 */
typedef enum {
    WT_SURVEY_TARGET_HUB = 0,
    WT_SURVEY_TARGET_TIP1,
    WT_SURVEY_TARGET_TIP2,
    WT_SURVEY_TARGET_TIP3,
    WT_SURVEY_TARGET_COUNT
} E_WtSurveyTarget;

/** 会话状态 */
typedef enum {
    WT_SURVEY_IDLE = 0,
    WT_SURVEY_AIMING,    /*!< 正在对准当前目标 */
    WT_SURVEY_SETTLING,  /*!< 云台已下发，等它停稳 */
    WT_SURVEY_LASING,    /*!< 读激光 */
    WT_SURVEY_FITTING,   /*!< 点采齐了，拟合 */
    WT_SURVEY_DONE,
    WT_SURVEY_FAILED
} E_WtSurveyState;

/** 会话参数 */
typedef struct {
    double aimToleranceDeg;  /*!< 视线夹角在此范围内才允许打激光 */
    uint32_t settleMs;       /*!< 云台下发后的等待时间 */
    uint32_t maxSnapshotAgeMs; /*!< 超过这个年龄的检测快照不采信 */
    int maxAttemptsPerTarget;  /*!< 单个目标的最大重试次数 */
    double gimbalMoveSeconds;  /*!< 单次云台转动的期望时长 */
} WtSurveyConfig;

/** 会话状态机 */
typedef struct {
    E_WtSurveyState state;
    const char *message;

    WtTurbineSpec spec;       /*!< 台账副本：提供坐标系原点与比对基准 */
    WtLocalFrame frame;       /*!< 以塔基为原点，与规划链同一坐标系 */
    WtSurveyConfig cfg;

    int targetIndex;          /*!< 当前目标（E_WtSurveyTarget） */
    int attempts;             /*!< 当前目标已尝试次数 */
    uint32_t stateEnteredMs;

    WtGeo pointGeo[WT_SURVEY_TARGET_COUNT];
    WtEnu pointEnu[WT_SURVEY_TARGET_COUNT];
    bool pointValid[WT_SURVEY_TARGET_COUNT];
    double pointDistanceM[WT_SURVEY_TARGET_COUNT];

    /*
     * 还需要机头转多少度。M4T 云台 yaw 不可控，这一项程序动不了,
     * 必须呈现给操作员。符号：正 = 需要顺时针（向右）转。
     */
    double yawErrorDeg;
    double pitchErrorDeg;
    double angularErrorDeg;

    WtRotorFitResult fit;
    WtRotorFitLimits limits;
    bool fitAccepted;
} WtSurvey;

/** @brief 默认会话参数 */
WtSurveyConfig WtSurvey_DefaultConfig(void);

/**
 * @brief 开始一次采集会话。
 *
 * @param spec 台账。必须含有效的 `base`（提供 ENU 原点）与 `bladeCount`
 * @return false 表示参数非法或视觉链路未就绪
 */
bool WtSurvey_Begin(WtSurvey *s, const WtTurbineSpec *spec,
                    const WtSurveyConfig *cfg);

/**
 * @brief 推进状态机，每个控制周期调一次。
 *
 * @param gimbal 当前云台指向（来自 `WtTelemetry.gimbal`）
 *
 * @note 内部会调 `WtLaser_Read()`，而那是**阻塞调用**（实测最多 1.2 s）。
 *       所以本函数只能在主循环或采集线程里调，**绝不能在 PSDK 回调线程里调**。
 */
void WtSurvey_Tick(WtSurvey *s, const WtGimbalState *gimbal);

/** @brief 中止会话 */
void WtSurvey_Abort(WtSurvey *s);

/** @brief 会话是否已结束（成功或失败） */
bool WtSurvey_IsFinished(const WtSurvey *s);

/**
 * @brief 取拟合出的盘面参考系，可直接喂给 `WtPlan_BladeTrace`。
 * @return false 表示尚未成功拟合
 */
bool WtSurvey_GetRotorFrame(const WtSurvey *s, WtRotorFrame *out);

/** @brief 当前目标的人类可读名字，用于日志与 Pilot 浮窗 */
const char *WtSurvey_TargetName(int targetIndex);

/** @brief 把会话进度格式化成一行，供浮窗显示（浮窗带宽上限 2 KB/s） */
void WtSurvey_FormatProgress(const WtSurvey *s, char *buf, size_t bufLen);

#ifdef __cplusplus
}
#endif

#endif /* WT_SURVEY_H */
