/**
 * @file wt_survey.h
 * @brief 激光采集会话的**接线层**：把决策层接到视觉、激光、云台与拟合上。
 *
 * ## 整条链路
 *
 * ```
 *   取流(wt_frame_pump) → 推理(wt_vision) → 稳定排序的 4 个像素
 *        ↓
 *   逐个目标：决策(wt_survey_logic) → 转云台 pitch(wt_laser) → 等稳 → 打激光
 *        ↓
 *   4 个 WGS84 点 → 转塔基 ENU → 拟合(wt_rotor_fit) → WtRotorFrame
 *        ↓
 *   喂给既有规划链 WtPlan_BladeTrace，生成贴叶 KMZ
 * ```
 *
 * ## 分层
 *
 * **判据与状态转移不在这里**，在 `wt_survey_logic`（`wt_core`，桌面可测）。
 * 本文件只做三件有副作用的事：采观测、执行动作、存点与做拟合。
 *
 * 这条分界线和项目里其他几处是同一条 —— 判据写在 PSDK 侧就等于没有测试。
 *
 * ## 为什么是半自动
 *
 * M4T 的云台 Pan 轴 `Not controllable`，水平对准只能靠机头。让采集模块去
 * 操纵飞机朝向，风险与收益不成比例。所以：**俯仰**由程序转云台，
 * **水平**把还差多少度（`yawErrorDeg`）交给操作员。
 *
 * ## 编号由采集顺序定义
 *
 * 模型输出的叶尖无序，所以 `wt_vision` 先按图像极角稳定排序，本模块按那个
 * 顺序依次采集，`wt_rotor_fit` 再把第一个采集点的相位当作停用角。
 * 三者是同一约定的三段，改一段要同时改另外两段。
 *
 * ## 产出的停用角正是既有流程拒绝执行的那个前提
 *
 * `WtRunner_PlanTurbine` 在停用角未知时**无条件拒绝规划**（贴叶航线按
 * 「叶片停在 12 点」假设画出来，假设不成立就是撞机）。本模块把它测出来。
 */

#ifndef WT_SURVEY_H
#define WT_SURVEY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "wt_geometry.h"
#include "wt_rotor_fit.h"
#include "wt_survey_logic.h"
#include "wt_telemetry.h"
#include "wt_turbine.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 采集会话 */
typedef struct {
    WtSurveyLogic logic;      /*!< 决策层状态（判据都在它那边） */

    WtTurbineSpec spec;       /*!< 台账副本：提供坐标系原点与比对基准 */
    WtLocalFrame frame;       /*!< 以塔基为原点，与规划链同一坐标系 */

    WtGeo pointGeo[WT_SURVEY_TARGET_COUNT];
    WtEnu pointEnu[WT_SURVEY_TARGET_COUNT];
    bool pointValid[WT_SURVEY_TARGET_COUNT];
    double pointDistanceM[WT_SURVEY_TARGET_COUNT];

    /* 最近一次决策的误差量，供日志与 Pilot 浮窗显示 */
    double yawErrorDeg;
    double pitchErrorDeg;
    double angularErrorDeg;

    WtRotorFitResult fit;
    WtRotorFitLimits limits;
    bool fitAccepted;
} WtSurvey;

/**
 * @brief 开始一次采集会话。
 *
 * @param spec 台账。必须含有效的 `base`（提供 ENU 原点）与 `bladeCount`
 * @param cfg  会话参数，NULL 用默认
 * @return false 表示台账非法
 */
bool WtSurvey_Begin(WtSurvey *s, const WtTurbineSpec *spec,
                    const WtSurveyConfig *cfg);

/**
 * @brief 推进状态机，每个控制周期调一次。
 *
 * @param tele 遥测上下文。需要两样：`gimbal`（当前云台指向）与
 *        `aircraft.yawDeg`（**机头朝向**）—— 后者是判 pan 可达性的前提，
 *        因为云台 pan 的限位是相对机头的 ±60°，不是绝对方位角。
 *
 * @note 内部会调 `WtLaser_Read()`，而那是**阻塞调用**（实测最多 1.2 s）。
 *       所以本函数只能在主循环或采集线程里调，**绝不能在 PSDK 回调线程里调**。
 */
void WtSurvey_Tick(WtSurvey *s, const WtTelemetry *tele);

/** @brief 中止会话 */
void WtSurvey_Abort(WtSurvey *s);

/** @brief 会话是否已结束（成功或失败） */
bool WtSurvey_IsFinished(const WtSurvey *s);

/**
 * @brief 取拟合出的盘面参考系，可直接喂给 `WtPlan_BladeTrace`。
 * @return false 表示尚未成功拟合
 */
bool WtSurvey_GetRotorFrame(const WtSurvey *s, WtRotorFrame *out);

/** @brief 把会话进度格式化成一行，供浮窗显示（浮窗带宽上限 2 KB/s） */
void WtSurvey_FormatProgress(const WtSurvey *s, char *buf, size_t bufLen);

#ifdef __cplusplus
}
#endif

#endif /* WT_SURVEY_H */
