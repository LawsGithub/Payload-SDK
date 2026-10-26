/**
 * @file wt_mission_watch.h
 * @brief 航点任务（Waypoint V3）执行进度的判定：开始了没有、结束了没有、停在哪。
 *
 * 纯逻辑，桌面可测。PSDK 的任务状态回调只把原始状态值和航点号交过来，
 * 判定在这里做。
 *
 * ## 为什么要单独一层
 *
 * 「收到 IDLE 就算结束」是错的：任务刚 START 时飞机仍可能推送 IDLE
 * （姊妹项目 liangzhourenwu 实测踩过），按那个判据，航线一启动就会被判成
 * 「已结束」，监控循环随即退出，后面的中止按钮也就没人响应了。
 * 正确的判据是**先见到过执行状态，再回到 IDLE** 才算结束；START 之后
 * 迟迟见不到执行状态，单独判为「没启动起来」。
 *
 * 状态值与 `dji_waypoint_v3.h` 的 `E_DjiWaypointV3MissionState` 一一对应
 * （wt_core 不依赖 PSDK，所以在这里重抄一份；机载侧有编译期断言保证一致）。
 */

#ifndef WT_MISSION_WATCH_H
#define WT_MISSION_WATCH_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* E_DjiWaypointV3MissionState 的取值（dji_waypoint_v3.h:53-61） */
#define WT_MW_STATE_IDLE              0
#define WT_MW_STATE_PREPARE           16
#define WT_MW_STATE_TRANS_MISSION     32
#define WT_MW_STATE_MISSION           48
#define WT_MW_STATE_BREAK             64
#define WT_MW_STATE_RESUME            80
#define WT_MW_STATE_RETURN_FIRSTPOINT 98

/** 判定结果 */
typedef enum {
    WT_MW_WAITING = 0,  /*!< 已 START，还没见到执行状态 */
    WT_MW_RUNNING,      /*!< 执行中 */
    WT_MW_PAUSED,       /*!< BREAK：飞机暂停（遥控器暂停、避障刹停等），等恢复或人工接管 */
    WT_MW_FINISHED,     /*!< 见到过执行状态后回到 IDLE：跑完了，或被外部停止 */
    WT_MW_NOT_STARTED,  /*!< START 之后超时仍未进入执行状态 */
} E_WtMissionWatchPhase;

typedef struct {
    E_WtMissionWatchPhase phase;
    uint32_t startedMs;
    uint32_t startTimeoutMs;
    bool seenActive;
    int lastRawState;       /*!< 最近一次收到的原始状态值，-1 = 还没收到过 */
    int waypointIndex;      /*!< 最近一次报告的航点号，-1 = 还没报告过 */
    int maxWaypointIndex;   /*!< 报告过的最大航点号 */
    uint32_t lastUpdateMs;
} WtMissionWatch;

/** @brief START 成功后立刻调用 */
void WtMissionWatch_Begin(WtMissionWatch *w, uint32_t nowMs, uint32_t startTimeoutMs);

/** @brief 喂一条状态（来自任务状态回调的缓存） */
void WtMissionWatch_OnState(WtMissionWatch *w, int rawState, int waypointIndex, uint32_t nowMs);

/** @brief 每个控制周期调一次：处理「迟迟没启动」的超时，返回当前判定 */
E_WtMissionWatchPhase WtMissionWatch_Tick(WtMissionWatch *w, uint32_t nowMs);

/** @brief 是否已终结（FINISHED 或 NOT_STARTED） */
bool WtMissionWatch_IsOver(const WtMissionWatch *w);

/** @brief 判定结果的中文名 */
const char *WtMissionWatch_PhaseName(E_WtMissionWatchPhase phase);

#ifdef __cplusplus
}
#endif

#endif /* WT_MISSION_WATCH_H */
