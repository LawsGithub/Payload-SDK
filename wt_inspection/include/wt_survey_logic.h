/**
 * @file wt_survey_logic.h
 * @brief 激光采集的**决策层**：状态转移、对准判据、重试与超时。纯算法。
 *
 * ## 为什么要和 PSDK 接线分开
 *
 * 本项目（以及姊妹项目）反复吃过同一个教训：**判据写在 PSDK 侧 = 桌面测不到
 * = 没有测试**。采集状态机里容易错的恰恰全是判据类的东西：
 *
 *   - 什么时候算"已对准"（用夹角还是用两轴误差？俯视时会不会永远判不准？）
 *   - 重试计到第几次该放弃
 *   - 云台停稳等多久、检测快照多旧就不能用
 *   - 激光失败后该退回对准还是直接判死
 *
 * 这些一行写错都不会报错，只会让采集卡死或采到错误的点。所以它们留在
 * `wt_core` 里，由 `wt_test_survey_logic` 覆盖；`app/wt_survey.c` 只剩
 * 「采观测 → 调 Step → 执行动作」的接线。
 *
 * ## 用法
 *
 * ```c
 * WtSurveyLogic logic;
 * WtSurveyLogic_Begin(&logic, NULL, nowMs);
 *
 * for (;;) {
 *     WtSurveyObs obs = { ... };            // app 层采集观测
 *     WtSurveyDecision d = WtSurveyLogic_Step(&logic, &obs);
 *
 *     switch (d.action) {
 *     case WT_SURVEY_ACT_ROTATE_GIMBAL: 下发 d.gimbalPitchDeg; break;
 *     case WT_SURVEY_ACT_READ_LASER:    读激光；成功则 RecordPoint(); break;
 *     case WT_SURVEY_ACT_FIT:           做拟合，再 FitDone(); break;
 *     ...
 *     }
 * }
 * ```
 *
 * 决策层**不碰**激光与云台，也不持有采到的点 —— 点由 app 层存放，
 * 因为那涉及坐标转换与台账，属于接线的职责。
 */

#ifndef WT_SURVEY_LOGIC_H
#define WT_SURVEY_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

#include "wt_aim.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 采集目标，顺序即编号（第一个采集点的相位就是停用角） */
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
    WT_SURVEY_AIMING,
    WT_SURVEY_SETTLING,
    WT_SURVEY_LASING,
    WT_SURVEY_FITTING,
    WT_SURVEY_DONE,
    WT_SURVEY_FAILED
} E_WtSurveyState;

/** app 层该执行的动作 */
typedef enum {
    WT_SURVEY_ACT_NONE = 0,
    WT_SURVEY_ACT_WAIT,           /*!< 条件不满足，什么也别做，下个周期再来 */
    WT_SURVEY_ACT_ROTATE_GIMBAL,  /*!< 下发 decision.gimbalPitchDeg */
    WT_SURVEY_ACT_READ_LASER,     /*!< 读激光，成功后调 RecordPoint */
    WT_SURVEY_ACT_FIT,            /*!< 四点齐了，去做拟合，再调 FitDone */
    WT_SURVEY_ACT_FINISH          /*!< 会话已结束（成功或失败） */
} E_WtSurveyAction;

/** 会话参数 */
typedef struct {
    double aimToleranceDeg;     /*!< 视线夹角在此范围内才允许打激光 */
    uint32_t settleMs;          /*!< 云台下发后的停稳等待 */
    uint32_t maxSnapshotAgeMs;  /*!< 超过此年龄的检测快照不采信 */
    int maxAttemptsPerTarget;   /*!< 单目标最大重试次数 */
    double gimbalMoveSeconds;   /*!< 单次云台转动的期望时长 */
} WtSurveyConfig;

/** 一次 Step 的输入观测，由 app 层填好 */
typedef struct {
    uint32_t nowMs;

    bool haveGimbal;            /*!< 云台姿态是否可用 */
    WtCameraPose gimbal;        /*!< 当前云台指向（position 不参与方向解算） */

    bool haveDetection;         /*!< 是否有过成功的识别 */
    uint32_t detectionAgeMs;
    bool targetVisible;         /*!< 当前目标在画面里有有效像素 */
    double targetU;
    double targetV;

    int imageWidthPx;
    int imageHeightPx;
    double fovHorizDeg;         /*!< 必须是水平 FOV，不是对角线值 */
    double fovVertDeg;
} WtSurveyObs;

/** 一次 Step 的输出 */
typedef struct {
    E_WtSurveyAction action;
    E_WtSurveyState state;
    int targetIndex;

    double gimbalPitchDeg;      /*!< ROTATE_GIMBAL 时要下发的绝对俯仰角 */

    /*
     * 两轴误差分开报。M4T 云台 Pan 不可控 ⇒ yawErrorDeg 只能由机头承担，
     * 程序动不了，必须呈现给操作员。
     */
    double yawErrorDeg;
    double pitchErrorDeg;
    double angularErrorDeg;

    const char *message;
} WtSurveyDecision;

/** 决策层状态（纯数据，可直接 memcpy / 存档） */
typedef struct {
    E_WtSurveyState state;
    int targetIndex;
    int attempts;
    uint32_t stateEnteredMs;
    WtSurveyConfig cfg;
    const char *message;
} WtSurveyLogic;

/** @brief 默认会话参数 */
WtSurveyConfig WtSurveyLogic_DefaultConfig(void);

/**
 * @brief 开始会话。
 * @param cfg 为 NULL 时用默认参数
 */
void WtSurveyLogic_Begin(WtSurveyLogic *s, const WtSurveyConfig *cfg, uint32_t nowMs);

/**
 * @brief 推进一步，返回 app 层该做什么。
 *
 * 不产生任何副作用 —— 同一组输入反复调用会得到同一个决策（除了内部的
 * 状态与计数推进），这正是它可被测试穷举的前提。
 */
WtSurveyDecision WtSurveyLogic_Step(WtSurveyLogic *s, const WtSurveyObs *obs);

/**
 * @brief 告知决策层「当前目标的点已采到」，推进到下一个目标。
 *
 * 由 app 层在激光读取成功后调用。四个点都采齐时状态会进入 FITTING。
 */
void WtSurveyLogic_RecordPoint(WtSurveyLogic *s, uint32_t nowMs);

/**
 * @brief 告知决策层「本次激光无有效回波」。
 *
 * 会累加重试计数并退回对准（目标可能已飘出容差）；超次数则判死。
 */
void WtSurveyLogic_LaserFailed(WtSurveyLogic *s, uint32_t nowMs);

/** @brief 告知决策层拟合结果，结束会话 */
void WtSurveyLogic_FitDone(WtSurveyLogic *s, bool accepted, const char *reason);

/** @brief 中止会话 */
void WtSurveyLogic_Abort(WtSurveyLogic *s, const char *why);

/** @brief 是否已结束（成功或失败） */
bool WtSurveyLogic_IsFinished(const WtSurveyLogic *s);

/** @brief 目标的人类可读名字 */
const char *WtSurveyLogic_TargetName(int targetIndex);

#ifdef __cplusplus
}
#endif

#endif /* WT_SURVEY_LOGIC_H */
