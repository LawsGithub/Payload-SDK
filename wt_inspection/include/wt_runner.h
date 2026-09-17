/**
 * @file wt_runner.h
 * @brief 巡检任务的执行编排（机载）
 *
 * 把「载入配置 -> 规划 -> 安全校验 -> 下发 -> 执行监控」串成一条完整的
 * 作业流水线。任何一步不通过就中止，绝不把未通过校验的航线送上飞机。
 */

#ifndef WT_RUNNER_H
#define WT_RUNNER_H

#include <stdbool.h>

#include "wt_app_config.h"
#include "wt_bridge.h"
#include "wt_plan.h"
#include "wt_telemetry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 任务执行结果 */
typedef enum {
    WT_RUN_OK = 0,
    WT_RUN_ERR_CONFIG,     /*!< 配置载入或校验失败 */
    WT_RUN_ERR_PLAN,       /*!< 航线规划失败 */
    WT_RUN_ERR_SAFETY,     /*!< 安全校验未通过，航线被拒绝下发 */
    WT_RUN_ERR_UPLOAD,     /*!< 航线文件上传失败 */
    WT_RUN_ERR_EXECUTE,    /*!< 执行阶段出错（失控、超时等） */
    WT_RUN_ERR_ABORTED,    /*!< 被操作员中止 */
} WtRunResult;

/** 单台风机的一次巡检作业上下文 */
typedef struct {
    const WtTurbineEntry *turbine;
    WtMission mission;
    WtMissionPlan plan;
    WtSafetyReport safety;
    WtRotorFrame frame;
    double parkPhaseDeg;   /*!< 停用角（停机位相位），来自现场或视觉反解 */
    bool parkPhaseKnown;   /*!< 该角度是实测值还是"未知时的默认假设" */
    char kmzPath[WT_CONFIG_MAX_PATH];
    char csvPath[WT_CONFIG_MAX_PATH];          /*!< 动作与飞行参数表 */
    char waypointsCsvPath[WT_CONFIG_MAX_PATH]; /*!< 航点外参表，供缺陷三维定位 */
    char reportPath[WT_CONFIG_MAX_PATH];
} WtTurbineJob;

/** 运行器全局上下文 */
typedef struct {
    WtAppConfig config;
    WtTelemetry telemetry;
    WtTurbineJob job;
    bool abortRequested;
} WtRunner;

/**
 * @brief 初始化运行器：载入配置、初始化核心模块与遥测
 */
WtRunResult WtRunner_Init(WtRunner *runner, const char *configPath);

/** @brief 释放运行器占用的资源 */
void WtRunner_DeInit(WtRunner *runner);

/**
 * @brief 为指定风机规划航线并做安全校验
 *
 * @param parkPhaseDeg 停用角；若现场无法提供，传入负值表示"未知"，
 *                     此时采用保守假设（叶片停在 12 点方向）并给出告警，
 *                     同时把 rotorMayRotate 视为需要现场确认。
 */
WtRunResult WtRunner_PlanTurbine(WtRunner *runner, const char *turbineName,
                                 double parkPhaseDeg);

/**
 * @brief 执行已规划好的航线
 *
 * 流程：上传 KMZ -> 启动任务 -> 监控状态 -> 云台闭环跟踪 -> 处理中止。
 * 云台跟踪只在 profile.gimbalTrackGain > 0 且风轮可能转动时启用。
 */
WtRunResult WtRunner_ExecuteTurbine(WtRunner *runner);

/**
 * @brief 一键作业：规划 + 校验 + 执行
 */
WtRunResult WtRunner_RunTurbine(WtRunner *runner, const char *turbineName,
                                double parkPhaseDeg);

/**
 * @brief 请求中止当前任务
 *
 * 置位中止标志；执行循环在下一个周期内响应，先暂停航线任务再切换返航。
 * 用标志位而不是直接调 PSDK 接口，是为了保证中止动作发生在主循环线程，
 * 避免与状态回调并发调用 PSDK。
 */
void WtRunner_RequestAbort(WtRunner *runner);

/**
 * @brief 写出本次作业的报告（含安全校验结果与统计）
 */
bool WtRunner_WriteReport(const WtRunner *runner);

/* ------------------------------------------------------------------ */
/* 文件路径工具                                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief 递归创建目录（仅支持绝对路径，层级不深）
 *
 * 部署时真正会用到的一个场景：全新设备上 /data/wt_inspection/ 并不存在，
 * 而配置模板要先落进这个目录才谈得上"填好重启"。fopen 不会替你建目录。
 *
 * @return true 表示目录已存在或创建成功
 */
bool WtRunner_MakeDirs(const char *path);

/**
 * @brief 取路径的目录部分
 * @return true 表示 out 已写入父目录；路径里没有 '/' 时返回 false
 */
bool WtRunner_DirName(const char *path, char *out, size_t outLen);

#ifdef __cplusplus
}
#endif

#endif /* WT_RUNNER_H */