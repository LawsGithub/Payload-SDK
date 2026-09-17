/**
 * @file wt_bridge.h
 * @brief 规划器与 PSDK 航点任务之间的桥接层
 *
 * 本层只做「格式转换」与「任务编排」，不含任何几何计算：
 *   - 把 WtMission 的点序列翻译成 PSDK 航点文件（KMZ，航点 V3 接口用）
 *   - 把 WtMission 翻译成 PSDK 航点 V2 的任务结构体
 *   - 描述每个航点需要执行的云台/相机动作，供执行层绑定
 *
 * 之所以把动作描述抽出来：航点文件里写的是「相对坐标 + 动作」，而规划器
 * 给出的是「绝对方位角/俯仰角」，两者的换算固定且容易出错，集中在一处
 * 便于核对。
 */

#ifndef WT_BRIDGE_H
#define WT_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>

#include "wt_plan.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 云台控制模式 */
typedef enum {
    WT_GIMBAL_ABSOLUTE = 0, /*!< 云台偏航跟随航向，俯仰按绝对值下发 */
    WT_GIMBAL_FREE_YAW,     /*!< 云台偏航独立于机头，按绝对方位角下发 */
} WtGimbalMode;

/** 相机动作 */
typedef enum {
    WT_ACTION_NONE = 0,
    WT_ACTION_TAKE_PHOTO,   /*!< 单张拍照 */
    WT_ACTION_START_RECORD, /*!< 开始录像 */
    WT_ACTION_STOP_RECORD,  /*!< 停止录像 */
} WtCameraActionType;

/** 一个航点要执行的动作 */
typedef struct {
    int waypointIndex;       /*!< 航点序号，与 WtMission.points 下标一致 */
    WtCameraActionType camera;
    bool setGimbal;          /*!< 是否需要设置云台姿态 */
    double gimbalYawDeg;     /*!< 云台偏航：绝对方位角或相对机头的偏角 */
    double gimbalPitchDeg;   /*!< 云台俯仰，向下为负 */
    WtGimbalMode mode;
} WtWaypointAction;

/** 动作序列 */
typedef struct {
    WtWaypointAction *items;
    size_t count;
    size_t capacity;
} WtActionList;

/** 一个航点的相对高度设定 */
typedef struct {
    int waypointIndex;
    double relativeAltitudeM; /*!< 相对起飞点的高度 m */
    double speedMs;           /*!< 到达该点的速度 m/s */
} WtWaypointFlightParam;

/** 执行层需要的完整任务描述 */
typedef struct {
    WtActionList actions;
    WtWaypointFlightParam *flightParams;
    size_t flightParamCount;
} WtMissionPlan;

/* ------------------------------------------------------------------ */
/* 动作生成                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief 由航线点序列生成动作列表
 *
 * 规则：
 *   - 缺陷/粗模拍照点 -> 到达即触发一次快门
 *   - 转场点         -> 不触发任何动作
 *   - 每个需要拍照或指向的航点都附带云台设定，保证到点瞬间光轴已就位
 *
 * @param mode 云台模式；FREE_YAW 时 yaw 为绝对方位角，ABSOLUTE 时换算为
 *             相对机头的偏角（机头默认沿航线切向，见 WtBridge_HeadingAt）
 */
WtPlanResult WtBridge_BuildActions(const WtMission *mission,
                                   WtGimbalMode mode,
                                   WtMissionPlan *plan);

/**
 * @brief 取第 index 个航点的机头朝向（沿航线的切线方位角）
 *
 * 首末点退化时分别沿用相邻段的朝向。
 */
double WtBridge_HeadingAt(const WtMission *mission, size_t index);

/**
 * @brief 取第 index 个航点的相对起飞高度
 */
double WtBridge_RelativeAltitudeAt(const WtMission *mission, const WtGeo *takeoffGeo, size_t index);

/** @brief 由航线与起飞点填充 flightParams */
WtPlanResult WtBridge_BuildFlightParams(const WtMission *mission,
                                        const WtGeo *takeoffGeo,
                                        const WtInspectionProfile *profile,
                                        WtMissionPlan *plan);

void WtBridge_InitPlan(WtMissionPlan *plan);
void WtBridge_FreePlan(WtMissionPlan *plan);

/* ------------------------------------------------------------------ */
/* 导出                                                                */
/* ------------------------------------------------------------------ */

/**
 * @brief 把航线导出为 PSDK 航点 KMZ 文件
 *
 * 产物结构遵循 DJI Pilot 的 wpml 约定：
 *     wpmz/template.kml   航线与各航点（含相对高度、速度、云台动作）
 *     wpmz/waylines.wpml  可执行航线（执行层主要消费该文件）
 *
 * 之所以自建生成器而不复用 Pilot 导出的模板：规划结果是逐点计算的，
 * 需要把「每点独立的云台角与速度」原样带进航线文件，Pilot 的固定模板
 * 无法表达这种逐点差异。
 *
 * @param mission    规划好的航线
 * @param plan       动作与飞行参数
 * @param profile     作业剖面（提供安全起飞高度与全局速度）
 * @param takeoffGeo 起飞点 WGS84 坐标，用于换算相对高度
 * @param outPath    输出 .kmz 路径
 */
WtPlanResult WtBridge_ExportKmz(const WtMission *mission,
                                const WtMissionPlan *plan,
                                const WtInspectionProfile *profile,
                                const WtGeo *takeoffGeo,
                                const char *outPath);

/** @brief 把动作列表与飞行参数导出为 CSV，便于人工核对 */
WtPlanResult WtBridge_ExportActionCsv(const WtMission *mission,
                                      const WtMissionPlan *plan,
                                      const char *path);

#ifdef __cplusplus
}
#endif

#endif /* WT_BRIDGE_H */