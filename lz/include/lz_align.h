/**
 * @file lz_align.h
 * @brief 视觉照准的**决策逻辑** —— 零依赖，桌面上可完整验证。
 *
 * ## 为什么单独一层，而不是写在 `app/lz_visual_align.c` 里
 *
 * `app/` 下的东西依赖 PSDK，**在桌面上编不了也测不了**。而这里放的三组判据
 * 恰好全是"写错了不会报错、只在画面上表现为‘相机越调越偏’或‘程序不结束’"
 * 的那一类 —— 与 `LzPlan_ClampWaypointCount()`、`LzPole_JudgeLaserReading()`
 * 被抽出来的理由完全相同（见各自的头文件）。
 *
 * 放在 `lz_core` 而不是新开一层：它引用的云台俯仰包线
 * （`LZ_GIMBAL_PITCH_MIN/MAX_DEG`）本来就在 `lz_plan.h` 里，**同层才谈得上
 * "判据只有一处真值"**。依赖方向仍是单向：`lz_visual_align` → `lz_align`。
 *
 * ## 这一层守的三件事（每一件都对应一个真实缺陷）
 *
 * ### 1. 非法测量值不能穿过包线检查
 *
 * C 里 `NaN` 与任何数的比较**全是 false**，所以
 *
 * ```c
 * if (fabs(delta) <= DEADZONE) { ... }          // NaN → false，不进死区
 * if (target < MIN || target > MAX) { 拒绝 }     // NaN → false，**不拒绝**
 * ```
 *
 * 两处都放过它，NaN 于是被原样发给云台。**"两个检查都没反对"不等于"检查
 * 通过了"** —— 这是本项目记过的「静默的成功更要命」的又一个入口。
 * 所以非法值必须在**最前面**被显式拦下，而不是指望后面的阈值判据。
 *
 * ### 2. 两个失败计数器必须彼此独立
 *
 * "等不到帧"与"看不到目标"是**两种不同的失败**，各自有各自的容忍次数。
 * 一旦共用一个计数器、或让其中一个的成功路径把另一个清零，那条判据就
 * **永远到不了上限** —— 判据还在、注释还在、就是不生效。
 * 这与 `LZ_CHECK_ANGLE_NEAR` 漏计 failures 是同一个形状。
 *
 * ### 3. 照准与绕飞的互斥是**双向**的
 *
 * 绕飞时飞机姿态在变，而照准算的是"静止观测位下该瞄哪"；反过来，照准在
 * 转云台时启动航线，航线里的 `gimbalRotate` 与手动云台控制会抢同一个云台。
 * ⇒ 两个方向都要拦，且**两处引用同一个判定函数**（判据不该取决于谁在调用）。
 */

#ifndef LZ_ALIGN_H
#define LZ_ALIGN_H

#include <stdbool.h>

/* ------------------------------------------------------------------ */
/* ① 每一步该做什么                                                     */
/* ------------------------------------------------------------------ */

/** 照准迭代的参数。都是现场判断，不是规范值 —— 见 `lz_visual_align.c` 的取值说明 */
typedef struct {
    double deadzoneDeg;   /*!< 死区：|Δθ| 小于它就算对准了 */
    double maxStepDeg;    /*!< 单轮最大转角 —— 检测读错一个数时不让云台猛甩 */
    int    maxRounds;     /*!< 总轮数上限 —— 超过就判"未收敛"，不无限试 */
} LzAlignPolicy;

/** 一步的结论 */
typedef enum {
    LZ_ALIGN_ACT_ROTATE = 0,   /*!< 转云台到 `targetDeg`（`stepDeg`/`targetDeg` 有效） */
    LZ_ALIGN_ACT_CONFIRM,      /*!< 刚进死区，再取一帧复核（不转） */
    LZ_ALIGN_ACT_DONE,         /*!< 复核仍在死区 ⇒ 收敛 */
    LZ_ALIGN_ACT_BAD_MEASURE,  /*!< Δθ 或当前俯仰角非法（NaN/Inf）—— **必须最先拦** */
    LZ_ALIGN_ACT_ROUNDS_EXHAUSTED, /*!< 轮数用尽 */
    LZ_ALIGN_ACT_OUT_OF_RANGE, /*!< 目标角超出云台俯仰范围 —— 如实拒绝，**不钳位** */
} LzAlignAction;

typedef struct {
    LzAlignAction action;
    double stepDeg;    /*!< 仅 `ROTATE` 时有效：限幅后的本步转角 */
    double targetDeg;  /*!< 仅 `ROTATE` 时有效：`curPitchDeg + stepDeg` */
} LzAlignDecision;

/**
 * @brief 决定照准的下一步：转 / 复核 / 收工 / 拒绝
 *
 * @param deltaDeg        目标点在画面里的角度偏差（`LzVision_PixelOffsetToDeg` 的输出）
 * @param curPitchDeg     云台**当前**俯仰角（读回值，不是上一轮下发的目标值）
 * @param round           已经用掉的轮数（首次调用传 0）
 * @param awaitingConfirm 上一轮是否已判"在死区里"、正等这次复核
 * @param policy          参数；为 NULL 时用 `LzAlign_DefaultPolicy()`
 *
 * ⚠️ **判定的顺序本身是规格**，不是实现细节：
 *
 * ```text
 * 1. Δθ / curPitch 非法          → BAD_MEASURE      ← 必须最先，否则后面全是空判
 * 2. |Δθ| ≤ 死区                 → CONFIRM / DONE
 * 3. 轮数用尽                    → ROUNDS_EXHAUSTED
 * 4. 限幅                        → step = clamp(Δθ, ±maxStep)
 * 5. curPitch + step 越界        → OUT_OF_RANGE
 * 6. 否则                        → ROTATE
 * ```
 *
 * 为什么越界是"拒绝"而不是"钳到边界"：钳位会把「飞机低于杆时需要上仰，
 * 而相机上仰能力有限，超出就是**物理上做不到**」伪装成"做得到"。
 * 与 `LzPlan_ComputeGimbalPitchDeg` 照实返回、由 `LzPlan_Validate` 拒绝
 * 是同一条纪律。
 */
LzAlignDecision LzAlign_DecideStep(double deltaDeg, double curPitchDeg,
                                   int round, bool awaitingConfirm,
                                   const LzAlignPolicy *policy);

/** @brief 缺省参数（与 `lz_visual_align.c` 的历史取值一致） */
LzAlignPolicy LzAlign_DefaultPolicy(void);

/* ------------------------------------------------------------------ */
/* ①b 按钮的"开始/停止"该判成哪一个                                      */
/* ------------------------------------------------------------------ */

/** 一次按钮按压的处置 */
typedef enum {
    LZ_TOGGLE_START = 0,   /*!< 开始照准 */
    LZ_TOGGLE_STOP,        /*!< 停止照准 */
    LZ_TOGGLE_IGNORED,     /*!< **吞掉这一次按压**（并告诉操作员为什么） */
} LzToggleAction;

/**
 * @brief 决定一次按钮按压是"开始"还是"停止"
 *
 * ## 这个函数存在的理由是一个真实的现场缺陷
 *
 * 现场现象：「点了按钮没反应」→ 操作员再点一次 → **照准启动后立刻自己停了**。
 * 日志（2026-09-28 实测）：
 *
 * ```text
 * 01:51:47.359  index=6, value=1     ← 第一次按下
 * 01:51:48.533  index=6, value=1     ← 又一次按下（因为"没反应"）
 * 01:51:49.004  取图：已开流           ← 照准真的启动了
 * 01:51:49.025  [widget] 照准开始…
 * 01:51:49.125  [widget] 照准已按操作员要求停止   ← 100 ms 后被自己停掉
 * ```
 *
 * 成因是一条完整的因果链：
 *
 * 1. 按下后主循环要跑 `do_init()`（开图流 + 初始化检测器 + 订阅云台角），
 *    里面有两个**阻塞的 SDK 调用**（`StartImageStream` / `FcSubscription_Init`），
 *    合计约 1.5 秒。这期间主循环完全没响应。
 * 2. 操作员看到没反应，**又按了一次**；这次按压排在队列里。
 * 3. `do_init()` 返回后，下一拍把排队的按压当成"再按一次 = 停止" ⇒
 *    **照准在启动后 100 ms 自杀**。
 *
 * ⇒ 判据必须是「**能不能停**」，而不是「`s_step` 是不是 IDLE」。
 * 「刚按下、还没真正跑起来」那段窗口里，重复按压应当**被吞掉并说明原因**，
 * 不能被解读成停止 —— 操作员的意图几乎必然是"怎么没反应"，
 * 而不是"我要停一个我还没看到启动的东西"。
 *
 * @param canStop   照准是否已经真正在跑（**启动窗口内为 false**）
 * @param inGrace   是否处在"刚按下、还在启动"的窗口内
 */
LzToggleAction LzAlign_DecideToggle(bool canStop, bool inGrace);

/* ------------------------------------------------------------------ */
/* ② 照准与绕飞的互斥                                                   */
/* ------------------------------------------------------------------ */

typedef enum {
    LZ_CONFLICT_NONE = 0,
    LZ_CONFLICT_ORBIT_ACTIVE,  /*!< 绕飞在跑 ⇒ 不许启动照准 */
    LZ_CONFLICT_ALIGN_ACTIVE,  /*!< 照准在跑 ⇒ 不许启动绕飞 */
} LzConflict;

/**
 * @brief 两个动作能否并存的**唯一判据**
 *
 * 两处调用方（`lz_mission.c` 的绕飞启动路径与照准请求路径）都打这个函数，
 * 不得各写一份 —— 否则会出现"一个方向拦、另一个方向不拦"，而那种不对称
 * 在代码里看是两段各自都合理的 `if`。
 *
 * ⚠️ 两个都为真时返回 `LZ_CONFLICT_ORBIT_ACTIVE`：绕飞是安全关键的作业，
 * 已经在跑的那个优先，照准让路。这个优先级是**定义了并测了**的，不是巧合。
 */
LzConflict LzAlign_CheckConflict(bool orbitRunning, bool alignRunning);

/** @brief 冲突的人话说明（供浮窗），`NONE` 时返回 NULL */
const char *LzAlign_ConflictStr(LzConflict c);

/* ------------------------------------------------------------------ */
/* ③ 两个彼此独立的失败计数器                                           */
/* ------------------------------------------------------------------ */

/** 计数状态 */
typedef struct {
    int frameWaits;    /*!< 连续"还没收到帧"的拍数 */
    int detectMisses;  /*!< 连续"检测不到目标"的次数 */
} LzAlignRetry;

typedef struct {
    int maxFrameWaits;     /*!< 等帧的拍数上限 */
    int maxDetectMisses;   /*!< 检测失败的次数上限 */
} LzAlignRetryPolicy;

void LzAlign_Retry_Reset(LzAlignRetry *r);

/**
 * @brief 记一次"还没收到帧"
 * @return true = 已超过上限，应当放弃
 */
bool LzAlign_Retry_NoteNoFrame(LzAlignRetry *r, const LzAlignRetryPolicy *policy);

/**
 * @brief 记一次"帧到手了"
 *
 * ⚠️ **只清 `frameWaits`，绝不动 `detectMisses`。**
 *
 * 这一条是缺陷的修法本身：早先 `do_grab()` 里一句 `s_waitTicks = 0` 同时
 * 清掉了检测失败的计数，于是"检测失败 5 次就放弃"这条判据**永远到不了**
 * —— 每轮都会先取到帧、把计数清零、再失败一次。表现是**照准永不结束**。
 * 反向验证时把本函数改回"清两个"，`lz_test_align` 必须变红。
 */
void LzAlign_Retry_NoteFrameOk(LzAlignRetry *r);

/** @brief 记一次"检测不到目标"；@return true = 已超过上限，应当放弃 */
bool LzAlign_Retry_NoteDetectMiss(LzAlignRetry *r, const LzAlignRetryPolicy *policy);

/** @brief 记一次"检测成功" —— 只清 `detectMisses` */
void LzAlign_Retry_NoteDetectHit(LzAlignRetry *r);

/** @brief 缺省上限（与 `lz_visual_align.c` 的历史取值一致） */
LzAlignRetryPolicy LzAlign_DefaultRetryPolicy(void);

#endif /* LZ_ALIGN_H */
