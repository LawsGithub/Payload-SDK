/**
 * @file wt_ped_logic.h
 * @brief 高空行人瞄准打点的**决策层**：选目标、跨帧关联、判停稳、锁定计时、
 *        发起激光。纯算法，桌面可测。（ped = pedestrian，行人）
 *
 * ## 流程
 *
 * ```
 * 搜索：画面里出现行人 → 取离光轴最近的那个（操作员把谁放在画面中间就打谁）
 *   ↓
 * 对准：云台转到目标方向 → 等读回停稳 → 看停稳后的新一帧 → 还差就再转
 *   ↓
 * 锁定：停稳后连续 confirmFrames 帧（默认 3，约 0.3 s）都在锁定容差内
 *       （可再加一段固定停顿 dwellMs，默认 0；配置 ped_dwell_ms = 3000 即旧的「3 s 后打」）
 *   ↓
 * 打激光（接线层连读 3 次取中位数）→ 完成（读数不一致或无回波则再确认后重打，超次数判失败）
 * ```
 *
 * ## 为什么不再固定等 3 s（2026-10-25 用户同意）
 *
 * 只要求打一次、拿到位置，3 s 本身不提供任何证据：真正说明「对准了」的是
 * **停稳之后拍到的几帧都落在容差内**。按帧数确认，静止的人约 0.3 s 就能打；
 * 老师若确实要求停顿，用配置项把 dwellMs 加回去，两个条件同时满足才打。
 *
 * ## 和 wt_survey_logic 的三处不同（都因为目标是人，会动）
 *
 * 1. **目标跨帧关联**。画面里可能有好几个人。按目标的**绝对方向**（方位/俯仰）
 *    关联，不按像素：云台一转，同一个人的像素位置全变了，但他在世界里的方向
 *    不变。按像素关联会在转动后认错人。
 *
 * 2. **停稳靠读回判，不靠固定等待；且只用停稳之后拍到的帧。**
 *    取流有延迟（帧到达比曝光晚），云台转动中拍到的帧配上「现在」的云台角，
 *    算出的目标方向是错的 —— 错的方向与转动方向相同，会被当成「还差一点」
 *    继续转，形成过冲。只用停稳后拍到的帧，延迟多大都无关紧要。
 *    （`wt_survey_logic` 转完云台、等够固定时长就直接读激光，不再看一帧确认；
 *    对静止的叶尖问题不大，对人不行。）
 *
 * 3. **锁定容差随目标大小**。取目标框角尺寸（半宽、半高中较小者）的一个比例，
 *    并夹在 [lockMinDeg, lockMaxDeg]。高空看行人只有十几个像素，固定的 2.5°
 *    容差会让光轴落在人旁边一大片地面上。
 *
 * ## 激光打的点在不在人身上
 *
 * 从高处斜看，行人脚边地面与人到飞机的距离只差约一个身高 × 俯角的余弦。
 * 所以即使激光擦着人打到脚边，得到的 WGS84 点也只偏一两米 —— 「行人位置」
 * 这个用途对它不敏感。容差下限因此不用压得极小（那会让云台反复微调）。
 *
 * 决策层**不碰**云台与激光，也不持有激光结果 —— 那是接线层（`app/wt_ped.c`）
 * 的事。
 */

#ifndef WT_PED_LOGIC_H
#define WT_PED_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

#include "wt_aim.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 一帧最多处理的候选数（与推理封装 WT_TRT_MAX_DETECTIONS 相同） */
#define WT_PED_MAX_CANDIDATES 64

/** 一个行人候选（像素坐标，原图） */
typedef struct {
    double u;           /*!< 瞄准像素：框中心 */
    double v;
    double widthPx;     /*!< 框宽高，用于定锁定容差 */
    double heightPx;
    double conf;
} WtPedCandidate;

/** 会话状态 */
typedef enum {
    WT_PED_IDLE = 0,
    WT_PED_SEARCHING,   /*!< 还没有目标 */
    WT_PED_AIMING,      /*!< 有目标，未锁定（转动、等停稳、等新帧、评估） */
    WT_PED_LOCKED,      /*!< 在容差内，驻留计时 */
    WT_PED_LASING,      /*!< 等接线层读激光 */
    WT_PED_DONE,
    WT_PED_FAILED
} E_WtPedState;

/** 接线层该执行的动作 */
typedef enum {
    WT_PED_ACT_NONE = 0,
    WT_PED_ACT_WAIT,            /*!< 条件不满足，下个周期再来 */
    WT_PED_ACT_ROTATE_GIMBAL,   /*!< 下发 gimbalPitchDeg + gimbalYawDeg（绝对角） */
    /*
     * 目标超出云台相对机头的 ±60°（或俯仰够不到）—— 需要操作员转机头。
     * 与 WAIT 分开：给操作员的提示完全不同，而且不消耗对准次数。
     */
    WT_PED_ACT_NEED_BODY_YAW,
    WT_PED_ACT_READ_LASER,      /*!< 读激光，成功调 RecordPoint，失败调 LaserFailed */
    WT_PED_ACT_FINISH           /*!< 会话已结束 */
} E_WtPedAction;

/** 会话参数 */
typedef struct {
    /*
     * 锁定后要连续几帧（停稳后拍到、且在锁定容差内）确认才打激光。
     * 锁定那一帧算第 1 帧。中途有一帧落在容差外（但没到脱锁线）就从头数。
     */
    int confirmFrames;
    uint32_t dwellMs;           /*!< 锁定后至少再停多久（默认 0；配置 ped_dwell_ms） */
    uint32_t retryDwellMs;      /*!< 激光没打成后，再确认多久重打 */
    double acquireConf;         /*!< 新捕获目标所需的置信度（跟踪中不再要求） */

    double lockFrac;            /*!< 锁定容差 = lockFrac × 目标框半尺寸（角度） */
    double lockMinDeg;
    double lockMaxDeg;
    double breakFactor;         /*!< 脱锁容差 = breakFactor × 锁定容差（滞回） */

    double gateDeg;             /*!< 跨帧关联门限：离上一帧方向超过它就不算同一个人 */
    uint32_t lostTimeoutMs;     /*!< 云台停稳后多久没看到目标就放弃它、重新搜索 */

    uint32_t minSettleMs;       /*!< 下发后至少等多久（云台响应本身有延迟） */
    uint32_t stableWindowMs;    /*!< 读回静止多久才算停稳（应 ≥ 取流延迟） */
    double motionDegPerS;       /*!< 读回角速度超过它视为在动 */

    uint32_t maxObsAgeMs;       /*!< 判锁定 / 打激光所用观测的最大年龄 */
    int maxAimAttempts;         /*!< 每次锁定前最多下发几次 */
    int maxLaserAttempts;       /*!< 激光最多打几次 */
    uint32_t sessionTimeoutMs;  /*!< 整个会话上限（操作员走开了也不会永久挂住） */
} WtPedConfig;

/** 一次 Step 的输入观测，由接线层填好 */
typedef struct {
    uint32_t nowMs;

    bool haveGimbal;
    WtCameraPose gimbal;        /*!< 当前云台指向（position 不参与方向解算） */

    /* 机头朝向。判 pan 可达性必须有它（限位是相对机头的 ±60°） */
    bool haveBodyYaw;
    double bodyYawDeg;

    /* 推理结果：同一帧会被反复给到，决策层按 frameId 去重 */
    bool haveFrame;
    uint32_t frameId;
    uint32_t frameCapturedMs;   /*!< 与 nowMs 同一个时钟 */
    int candidateCount;
    WtPedCandidate cand[WT_PED_MAX_CANDIDATES];

    int imageWidthPx;
    int imageHeightPx;
    double fovHorizDeg;         /*!< 当前变焦下的水平 FOV（不是对角线值） */
    double fovVertDeg;
} WtPedObs;

/** 当前跟踪的目标 */
typedef struct {
    bool active;
    double azDeg;               /*!< 目标的绝对方向 */
    double elDeg;
    /* 最近一次观测 */
    double u;
    double v;
    double widthPx;
    double heightPx;
    double conf;
    double errDeg;              /*!< 当时光轴与目标的夹角 */
    double yawErrDeg;
    double pitchErrDeg;
    double lockTolDeg;
    uint32_t seenAtMs;          /*!< 最近一次关联上的帧的采集时刻 */
} WtPedTrack;

/** 决策层状态（纯数据） */
typedef struct {
    E_WtPedState state;
    WtPedConfig cfg;
    uint32_t beginMs;
    uint32_t stateEnteredMs;
    const char *message;

    WtPedTrack track;
    bool haveLastFrame;
    uint32_t lastFrameId;

    /* 停稳判定 */
    bool havePrevGimbal;
    double prevYawDeg;
    double prevPitchDeg;
    uint32_t prevGimbalMs;
    uint32_t lastMotionMs;
    bool commanded;
    uint32_t lastCommandMs;
    bool settled;
    uint32_t settledAtMs;

    /* 锁定 */
    uint32_t lockSinceMs;
    uint32_t dwellRequiredMs;
    int confirmCount;           /*!< 锁定以来连续在容差内的帧数（锁定那一帧算 1） */

    int aimAttempts;
    int laserAttempts;
} WtPedLogic;

/** 一次 Step 的输出 */
typedef struct {
    E_WtPedAction action;
    E_WtPedState state;

    double gimbalPitchDeg;      /*!< ROTATE_GIMBAL 时下发的绝对俯仰 */
    /*
     * ROTATE_GIMBAL 时下发的绝对方位。
     * ⚠️ 拿不到机头朝向时这里是「当前云台 yaw」而不是 0 —— 绝对角模式下 0 是
     * "转到正北"，够不到时整条命令被拒、连 pitch 都不转。
     */
    double gimbalYawDeg;

    double yawErrorDeg;         /*!< NEED_BODY_YAW 时 = 机头需要转的角度 */
    double pitchErrorDeg;
    double angularErrorDeg;
    double lockTolDeg;

    uint32_t dwellElapsedMs;
    uint32_t dwellRequiredMs;
    int confirmCount;           /*!< 锁定中：已确认的帧数 */
    int confirmRequired;        /*!< 锁定中：需要的帧数 */
    int candidateCount;

    const char *message;
} WtPedDecision;

/** @brief 默认会话参数 */
WtPedConfig WtPedLogic_DefaultConfig(void);

/**
 * @brief 默认参数 + 配置文件里的两项（`ped_dwell_ms`、`ped_conf`）。
 *
 * dwellMs 为 0 是合法值（不额外停顿，只靠确认帧）；负值不该出现（解析已拦），
 * 防御性地按默认处理。
 *
 * 配置项必须是这两个量的**唯一**来源：main.c 只经由它构造会话参数，
 * 不另传 —— 另传的那一处迟早会把配置架空（停用角就这样被架空过一次）。
 */
WtPedConfig WtPedLogic_ConfigFrom(int dwellMs, double acquireConf);

/** @brief 开始会话。cfg 为 NULL 时用默认参数 */
void WtPedLogic_Begin(WtPedLogic *s, const WtPedConfig *cfg, uint32_t nowMs);

/** @brief 推进一步，返回接线层该做什么 */
WtPedDecision WtPedLogic_Step(WtPedLogic *s, const WtPedObs *obs);

/** @brief 激光读到有效回波，会话完成 */
void WtPedLogic_RecordPoint(WtPedLogic *s, uint32_t nowMs);

/** @brief 激光没打成（无回波或几次读数不一致）：再确认后重打，超次数判失败 */
void WtPedLogic_LaserFailed(WtPedLogic *s, uint32_t nowMs);

/** 一次打点连读几次激光（读数间隔要超过 5 Hz 限频，见 wt_laser.h） */
#define WT_PED_LASER_SAMPLES 3

/**
 * 几次激光读数允许的离散度：max(WT_PED_LASER_SPREAD_ABS_M, 距离 × WT_PED_LASER_SPREAD_FRAC)。
 * 激光标称精度约 ±(0.2 m + 0.15% × 距离)；行人只有半米宽，光斑压在人边缘时
 * 一会儿打到人、一会儿打到身后的地面，读数会差出米级 —— 那样的一组不能用。
 */
#define WT_PED_LASER_SPREAD_ABS_M 0.5
#define WT_PED_LASER_SPREAD_FRAC  0.01

/**
 * @brief 从几次激光读数里挑出代表那一次。
 *
 * 要求至少 2 次有效读数，且离散度（最大 − 最小距离）不超过上面的容差。
 * 3 次取距离居中的那次；2 次取较近的那次（俯视时人比身后的地面近）。
 * 返回的是**某一次真实读数**的下标，不对经纬度做平均 —— 平均出来的点
 * 可能哪次都没打到。
 *
 * @param distM       各次读数的距离（只传有效的）
 * @param n           个数
 * @param outSpreadM  输出离散度，可为 NULL
 * @return 选中的下标；-1 表示读数不够或不一致
 */
int WtPedLogic_PickLaserSample(const double *distM, int n, double *outSpreadM);

/** @brief 中止会话 */
void WtPedLogic_Abort(WtPedLogic *s, const char *why);

/** @brief 是否已结束（成功或失败） */
bool WtPedLogic_IsFinished(const WtPedLogic *s);

/**
 * @brief 由目标框大小算锁定容差（度）。
 *
 * 半宽/半高按「像素数 × 每像素角度」近似（小角度下足够）。框大小非法时
 * 返回下限。单独暴露是为了让测试能直接钉住这条换算。
 */
double WtPedLogic_LockToleranceDeg(const WtPedConfig *cfg, double widthPx, double heightPx,
                                   int imageWidthPx, int imageHeightPx,
                                   double fovHorizDeg, double fovVertDeg);

/** @brief 两个方向（方位/俯仰，度）之间的夹角（度），用方向矢量点积算 */
double WtPedLogic_AngleBetweenDeg(double az1Deg, double el1Deg, double az2Deg, double el2Deg);

#ifdef __cplusplus
}
#endif

#endif /* WT_PED_LOGIC_H */
