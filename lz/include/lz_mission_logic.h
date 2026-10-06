/**
 * @file lz_mission_logic.h
 * @brief 绕飞状态机的**决策逻辑** —— 零依赖，桌面上可完整验证。
 *
 * ## 为什么抽出来（与 `lz_align.h`、`lz_gimbal_status.h` 同一条理由）
 *
 * 状态机在 `app/lz_mission.c` 里，而那个文件依赖 PSDK ⇒ **桌面测不到**。
 * 于是整张转移表**一条断言都没有** —— 而它是本模块最贵的部分：
 * 走错的后果是"飞机还在绕而界面说停了"这类**安全相关**的谎话。
 *
 * 实际拆下来，`LzMission_Tick()` 里只有 **2 个** SDK 调用
 * （启动航线 / 停止航线），其余全是判据。所以决策可以整个搬到这边。
 *
 * ## 两相式：为什么不能一个函数搞定
 *
 * SDK 调用夹在中间，所以必须分两步：
 *
 * ```text
 *   Decide(输入)          → 决策{该做什么 + 去重闸的新值}
 *   （调用方调 SDK，拿到结果）
 *   Apply(决策, 结果)     → {新状态 + 要不要报哪条消息}
 * ```
 *
 * 两步都是**纯函数**：不读全局、不调 SDK、不打日志。整张转移表因此
 * 可以在桌面上穷举。
 *
 * ## 三条"刻意如此"的规则（都有测试守着，别顺手改）
 *
 * 1. **照准在跑时不启动绕飞，但也不清开关** —— 操作员的意思多半是
 *    "先识别、再绕飞"，清掉开关会逼他再拨一次 ON（而他记得自己拨过了）。
 * 2. **停止失败时留在 RUNNING** —— 状态必须与事实一致（任务确实还在跑）。
 *    若置 IDLE，开关还在 ON 位而状态机认为空闲，下一拍会把它当成
 *    **新的绕飞请求**重新上传启动。
 * 3. **三个去重闸各自独立** —— 它们是"浮窗带宽 2 KB/s"那条纪律的产物，
 *    合并成一个会让某一条消息被另一条吃掉。
 */

#ifndef LZ_MISSION_LOGIC_H
#define LZ_MISSION_LOGIC_H

#include <stdbool.h>

#include "lz_types.h"

/** 作业状态机的状态 */
typedef enum {
    LZ_MISSION_STATE_IDLE = 0,
    LZ_MISSION_STATE_RUNNING,
} LzMissionState;

/** 一拍该做什么 —— 由 `LzMission_Decide()` 给出，由调用方执行 */
typedef enum {
    LZ_MISSION_STEP_NONE = 0,   /*!< 什么都不做 */
    LZ_MISSION_STEP_START_ORBIT,/*!< 规划 + 生成 KMZ + 上传 + 启动（**唯一**的启动路径） */
    LZ_MISSION_STEP_STOP_MISSION,/*!< 调 STOP —— 结果要回传给 `Apply()` */
    LZ_MISSION_STEP_MISSION_ENDED,/*!< 飞机侧报结束 ⇒ 收尾（**无 SDK 调用**）
                                   *
                                   * ⚠️ 它是一条**判据**（"RUNNING + ended ⇒ 收尾"），
                                   * 不是"没判据所以留在 app 层" —— 第一版就是这么
                                   * 想错的，于是最后一条转移留在外面测不到。
                                   * 消息里的**原因文本**（`s_endReason`）仍由调用方给，
                                   * 那是回调写的静态缓冲，不进纯函数。 */
} LzMissionStep;

/** `Decide()` 的输入 —— 全部来自当前状态与外部观测 */
typedef struct {
    LzMissionState state;      /*!< 当前状态 */
    bool orbitRequested;       /*!< 操作员开关是否在 ON 位 */
    bool missionEnded;         /*!< 飞机侧报告结束（回调置的标志） */
    /* ---- 互斥判定的结果 ----
     * ⚠️ **由调用方传入、不在本函数里重算**：判据的唯一真值在
     * `LzAlign_CheckConflict()`（`lz_core/src/lz_align.c`，含"两个都在跑时
     * 绕飞优先"的用例）。在这里再算一遍就是**两份判据** ——
     * 本项目已经因为"两边各写一份、对不上也不报错"踩过多次。
     *
     * ⚠️ 刻意**不另设一个 `alignRunning` 字段**：那个字段与这个布尔
     * 说的是同一件事，两个都在就会漂移（改一个忘一个，而两者都不报错）。 */
    bool conflictAlignActive;  /*!< `LzAlign_CheckConflict(...) == LZ_CONFLICT_ALIGN_ACTIVE` */
    /* ---- 去重闸的当前值（由调用方持有，`Decide` 只算出新值）---- */
    bool orbitDeferredReported;/*!< "照准期间开关保持 ON"那条报过没有 */
} LzMissionInput;

/** `Decide()` 的输出 */
typedef struct {
    LzMissionStep step;
    /* ---- 去重闸的新值，调用方负责存回 ---- */
    bool orbitDeferredReported;
    /* ---- 要不要报"绕飞暂缓（照准在跑）"这条 ----
     * 单独一个标志而不是让调用方比去重闸的前后值：那样调用方要知道
     * "哪个闸对应哪条消息"，而那是判据的一部分。 */
    bool reportOrbitDeferred;
} LzMissionDecision;

/**
 * @brief 第一相：这一拍该做什么（纯函数）
 *
 * 不读全局、不调 SDK、不打日志。
 */
LzMissionDecision LzMission_Decide(const LzMissionInput *in);

/** `Apply()` 的输入 —— `Decide` 的结果 + SDK 调用（若有）的结果 */
typedef struct {
    LzMissionDecision decision;
    /* ---- 当前状态 ----
     * ⚠️ **必须传**：`Apply` 的输出状态是**权威的**，`step == NONE` 时
     * 它要原样带回当前状态。
     *
     * 第一版没这个字段，靠"调用方在 `step == NONE` 时**别采纳** `o.state`"
     * 来兜 —— 那种"记得别用"的约定一定会有人忘，而忘了的表现是
     * **RUNNING 被悄悄打回 IDLE**（开关还在 ON ⇒ 下一拍重新上传启动）。 */
    LzMissionState state;
    /* ---- 上一拍的 SDK 调用结果 ----
     * ⚠️ 只有 `step == START_ORBIT` / `STOP_MISSION` 时有意义；
     * 其余情况**调用方必须传 `LZ_OK`**（那两相之间没有 SDK 调用）。 */
    LzStatus sdkResult;
    /* ---- 去重闸：停止被拒那条 ----
     * 只有 `step == STOP_MISSION` 且失败时会被用到。 */
    bool stopRejectReported;
} LzMissionApplyInput;

/** `Apply()` 的输出 —— `state` **永远是权威的**，调用方直接采纳 */
typedef struct {
    LzMissionState state;      /*!< 新状态（`step == NONE` 时 = 传入的当前状态） */
    bool stopRejectReported;   /*!< 去重闸的新值 */
    /* ---- 要报哪条消息（互斥，最多一条）---- */
    bool reportOrbitDeferred;  /*!< "照准在跑，开关保持 ON" */
    bool reportStartFailed;    /*!< "启动失败"（**不清开关**，见 .c 说明） */
    bool reportStopRejected;   /*!< "停止指令被拒，任务可能仍在执行" */
    bool reportOperatorStop;   /*!< "绕飞结束：操作员停止" */
    bool reportMissionEnded;   /*!< "绕飞结束：<飞机侧给的原因>" */
} LzMissionApplyOutput;

/**
 * @brief 第二相：按 SDK 调用的结果定新状态与要报的消息（纯函数）
 */
LzMissionApplyOutput LzMission_Apply(const LzMissionApplyInput *in);

/**
 * @brief 航点号变了才报 —— 回调里用（纯函数）
 *
 * ## 为什么要有它（2026-09-28 实测）
 *
 * 「绕飞中：航点 N」原先**每次 SDK 推航点状态就发一条**，实测一次
 * 3 分钟绕飞发了 **716 条**。浮窗带宽只有 **2 KB/s**，被灌满后
 * **控件回执与状态推送全被挤掉** —— 操作员看到的是"点了没反应"。
 *
 * ⇒ 只在航点号变化时发。实测航点号去重后是 1..9 递增不回头 ⇒ 716 → 9 条。
 *
 * @param idx       本次回调给的航点号
 * @param lastIdx   上次报过的（调用方持有，`-1` = 还没报过）
 * @return true = 该报（并把 `lastIdx` 更新成 `idx`）
 */
bool LzMission_ShouldReportWaypoint(int idx, int *lastIdx);

/**
 * @brief 收到航点 IDLE 事件时，该不该置"任务结束"（纯函数）
 *
 * ⚠️ **RUNNING 期间也会收到 IDLE** —— 所以不能见到 IDLE 就置标志，
 * 必须看当前状态。早先没这个判断的话，任务刚启动就会被判成结束。
 *
 * @param state 当前状态
 * @return true = 该置结束标志
 */
bool LzMission_ShouldNoteEnded(LzMissionState state);

#endif /* LZ_MISSION_LOGIC_H */
