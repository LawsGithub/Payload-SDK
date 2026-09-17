/**
 * @file wt_plan.h
 * @brief 风机叶片巡检 —— 航线规划引擎
 *
 * 规划器输入「风机几何 + 风轮相位 + 作业剖面」，输出一串带相机指向的
 * 三维站位点（WtPlanPoint）。输出点同时携带 ENU 与 WGS84 坐标，
 * 前者供本机校验与可视化，后者可直接喂给 PSDK 航点接口。
 *
 * 规划器不依赖 PSDK，可在 PC 上单元测试。
 */

#ifndef WT_PLAN_H
#define WT_PLAN_H

#include <stddef.h>

#include "wt_camera.h"
#include "wt_geometry.h"
#include "wt_turbine.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 巡检模式 */
typedef enum {
    WT_MODE_COARSE_SURVEY = 0, /*!< 粗模环绕建模（第一阶段） */
    WT_MODE_BLADE_PRECISE,     /*!< 叶片精细巡检（停机） */
    WT_MODE_BLADE_ROTATING,    /*!< 叶片叶尖追踪（不停机） */
    WT_MODE_TOWER,             /*!< 塔筒垂直巡检 */
} WtInspectionMode;

/** 叶片巡检覆盖的面 */
typedef enum {
    WT_BLADE_SIDE_UPSTREAM = 0, /*!< 迎风面（-rotAxis 侧），前缘腐蚀高发面 */
    WT_BLADE_SIDE_DOWNSTREAM,   /*!< 背风面（+rotAxis 侧） */
    WT_BLADE_SIDE_BOTH,         /*!< 双面全覆盖 */
} WtBladeSide;

/** 作业剖面 —— 全部可在地面站调整的作业参数 */
typedef struct {
    /* ---- 作业模式 ---- */
    WtInspectionMode mode;       /*!< 本次任务的作业模式 */

    /* ---- 相机 ---- */
    WtCameraModel camera;        /*!< 使用的相机 */
    double targetGsdMmPerPx;     /*!< 缺陷检测目标 GSD mm/px，用于反算最大拍摄距离 */

    /* ---- 安全约束 ---- */
    double bladeStandoffM;       /*!< 叶片巡检的拍摄距离 m，0 = 由目标 GSD 自动反算 */
    double towerStandoffM;       /*!< 塔筒巡检时相机到塔壁的水平距离 m，0 = 自动反算 */
    double minSafeDistM;         /*!< 硬下限：任何站位到任一叶片轴线不得小于该值 m */
    double groundClearanceM;     /*!< 最低离地高度余量 m */
    bool rotorMayRotate;         /*!< 风轮是否可能转动（决定采用哪种安全包络） */

    /* ---- 采样密度 ---- */
    WtBladeSide bladeSide;       /*!< 叶片巡检面 */
    double overlapPct;           /*!< 期望展向重叠率 % */
    double bladeSpacingM;        /*!< 展向站位间距 m，>0 时优先于 overlapPct */
    int maxSamplesPerBlade;      /*!< 单片叶片单面采样点上限，防止误配参数炸点数 */
    double bladeAxialShiftM;     /*!< 站位沿叶片轴向的偏移量，取「拍摄斜距的比例」，
                                      0=正对拍摄；正=朝下游(背风侧)。典型 0.3（约 17°斜视角） */

    /* ---- 塔筒 ---- */
    int towerRingCount;          /*!< 塔筒环绕圈数 */
    int towerPerRingCount;       /*!< 每圈拍摄点数 */
    bool includeTower;           /*!< 是否规划塔筒巡检段 */

    /* ---- 粗模环绕 ---- */
    double coarseRadiusM;        /*!< 环绕半径 m，0=按叶轮半径自动推算 */
    int coarsePhotoCount;        /*!< 环绕拍摄张数 */
    double coarsePitchDeg;       /*!< 环绕时的云台俯仰角 */

    /* ---- 飞行 ---- */
    double cruiseSpeedMs;        /*!< 转场巡航速度 m/s */
    double inspectSpeedMs;       /*!< 巡检段速度 m/s */
    double photoDwellSec;        /*!< 每个拍照点的悬停稳定时间 s */
    bool doCoarseSurvey;         /*!< 是否在精细巡检前插入粗模环绕 */
} WtInspectionProfile;

/** 拍照用途 —— 两类图像的用途与质量指标完全不同，必须分开统计 */
typedef enum {
    WT_PHOTO_NONE = 0,       /*!< 纯转场点，不拍照 */
    WT_PHOTO_COARSE_MODEL,   /*!< 粗模重建用图，追求覆盖率而非分辨率 */
    WT_PHOTO_DEFECT,         /*!< 缺陷检测用图，必须满足目标 GSD */
} WtPhotoPurpose;

/** 单个航线点 */
typedef struct {
    WtEnu enu;              /*!< 局部 ENU 坐标，供校验与可视化 */
    WtGeo geo;              /*!< WGS84 坐标，可直接下发 PSDK */
    int bladeIndex;         /*!< 归属叶片序号，-1 表示非叶片点 */
    double radialFrac;      /*!< 展向归一化位置 0~1，非叶片点为 -1 */
    int side;               /*!< 对应 WtBladeSide，塔筒/环绕点为 -1 */
    double gimbalYawDeg;    /*!< 云台偏航角（绝对方位角） */
    double gimbalPitchDeg;  /*!< 云台俯仰角（向下为负） */
    double distanceToTargetM; /*!< 到拍摄目标的斜距 m */
    double gsdMmPerPx;      /*!< 该点预计 GSD */
    WtPhotoPurpose purpose; /*!< 该点拍照用途 */
    const char *tag;        /*!< 阶段标签，用于日志与航线文件注释 */
} WtPlanPoint;

/** 该点是否需要触发快门 */
bool WtPlanPoint_TakesPhoto(const WtPlanPoint *point);

/** 轴向偏移比例的上界（约合 26.6° 斜视角），见 WtPlan_BladeSkewAngleDeg */
#define WT_PLAN_AXIAL_SHIFT_MAX_FRAC 0.50

/**
 * @brief 该点是否属于「叶片精细巡检工位」
 *
 * 判据是三个字段同时成立：归属某片叶片、属于某个巡检面、且用途是缺陷检测。
 *
 * 为什么不能只看 bladeIndex >= 0：塔筒点、转场点的 bladeIndex 本来也是 -1，
 * 今天看起来够用；但绕行修补与任何新增的阶段只要给合成点补上 bladeIndex，
 * 仅凭一个字段就会把几十米的大跨度转移段误判成巡检段。把 side 与 purpose
 * 一并纳入，等于给这个判据多留一道闸。
 */
bool WtPlanPoint_IsBladeInspection(const WtPlanPoint *point);

/**
 * @brief 航段 a->b 是否应按巡检速度（inspectSpeedMs）计费与下发
 *
 * 「估算耗时」与「KMZ 里实际下发的 waypointSpeed」必须用同一份判据，否则
 * 报告上的耗时与飞机真正的飞法会对不上。本函数是这两处唯一的判据来源。
 *
 * 四项同时成立才算巡检段：
 *   - 两端都是叶片巡检工位（见 WtPlanPoint_IsBladeInspection）
 *   - 两端 bladeIndex 相等 —— 排除叶片之间的转移段
 *   - 两端 side 相等      —— 排除同一片叶片的迎风面/背风面换面段
 *
 * 注意判据里没有 profile：它只做分类，选哪个速度由调用方决定，
 * 这样它才能被单元测试直接覆盖。
 */
bool WtPlan_SegmentUsesInspectSpeed(const WtPlanPoint *a, const WtPlanPoint *b);

/** 一条完整航线 */
typedef struct {
    WtPlanPoint *points;   /*!< 点数组，堆分配 */
    size_t count;
    size_t capacity;
    double pathLengthM;    /*!< 折线总长 m */
    double durationSec;    /*!< 按速度与悬停时间估算的总耗时 s */
    int defectPhotoCount;  /*!< 缺陷检测用图数量 */
    int modelPhotoCount;   /*!< 粗模重建用图数量 */
} WtMission;

/** 规划结果码 */
typedef enum {
    WT_PLAN_OK = 0,
    WT_PLAN_ERR_PARAM = -1,     /*!< 参数非法 */
    WT_PLAN_ERR_NOMEM = -2,     /*!< 内存不足 */
    WT_PLAN_ERR_OVERFLOW = -3, /*!< 点数超出上限 */
} WtPlanResult;

/* ------------------------------------------------------------------ */
/* 剖面                                                                */
/* ------------------------------------------------------------------ */

/**
 * @brief 生成针对 M4T + 停机精细巡检的默认剖面
 *
 * 默认值基于 M4T 中焦 70mm 相机、1.5mm/px 目标 GSD、70% 展向重叠率推导，
 * 与 docs/风机叶片巡检方案.md §6.1 的默认剖面表一一对应。
 */
WtInspectionProfile WtInspectionProfile_Default(void);

/** @brief 校验剖面参数 */
WtValidateResult WtInspectionProfile_Validate(const WtInspectionProfile *profile);

/**
 * @brief 由 bladeAxialShiftM 推出的实际斜视角（度）
 *
 * 取 atan(比例)：偏移量与面法向垂距构成直角三角形的两直角边，正视时为 0°。
 * 用于报告与日志中核对「填进去的比例究竟对应多大的斜视角」。
 */
double WtPlan_BladeSkewAngleDeg(const WtInspectionProfile *profile);

/* ------------------------------------------------------------------ */
/* 任务容器                                                            */
/* ------------------------------------------------------------------ */

void WtMission_Init(WtMission *mission);
void WtMission_Free(WtMission *mission);
/** @brief 追加一个点（内部按需扩容） */
WtPlanResult WtMission_Append(WtMission *mission, const WtPlanPoint *point);
/** @brief 基于各点间距与速度，重算折线长度与预计耗时 */
void WtMission_ComputeStats(WtMission *mission, const WtInspectionProfile *profile);

/* ------------------------------------------------------------------ */
/* 规划器                                                              */
/* ------------------------------------------------------------------ */

/**
 * @brief 粗模环绕建模航线（第一阶段）
 *
 * 在风轮中心高度水平环绕一周，云台按 profile->coarsePitchDeg 俯视，
 * 用于本地重建风机三维模型、反解停机相位与机舱朝向。
 */
WtPlanResult WtPlan_CoarseSurvey(const WtTurbineSpec *spec,
                                 const WtInspectionProfile *profile,
                                 WtMission *mission);

/**
 * @brief 一次叶片站位规划实际采用的采样参数
 *
 * 站位之间的距离由「画面覆盖高度 × 重叠率」与「maxSamplesPerBlade 上限」
 * 共同决定，取两者中较大的那个 —— 所以用户填的重叠率未必真能达成。
 * 这里把推导过程的全套中间量都暴露出来，供报告与 PC 自检打印：
 * 只看最终站位数无法判断究竟是哪一项在起作用。
 */
typedef struct {
    double standoffM;        /*!< 实际采用的垂距 = 拍摄距离 m */
    double axialShiftM;      /*!< 实际采用的轴向偏移 m（已按上界削减） */
    double coverageM;        /*!< 该距离下展向画面覆盖高度 m */
    double idealStepM;       /*!< 由重叠率推算的理想间距 m */
    double spanStepM;        /*!< 受 maxSamplesPerBlade 约束的最小间距 m */
    double stepM;            /*!< 实际采用间距 m */
    double actualOverlapPct; /*!< 由实际间距反算的达成重叠率 % */
    int samplesPerBlade;     /*!< 单片叶片单面的站位数 */
} WtBladeSampling;

/**
 * @brief 推导一次叶片站位规划的完整采样参数
 *
 * 本函数是「采样密度」的唯一事实来源：站位间距、实际重叠率、站位数都从
 * 这里取。PC 自检程序与文档曾经各写一套公式，结果与规划器实际采用的参数
 * 不一致 —— 参数是否合理的判断依据因此失真。要改公式，只改这一处。
 */
WtPlanResult WtPlan_ResolveBladeSampling(const WtTurbineSpec *spec,
                                        const WtInspectionProfile *profile,
                                        WtBladeSampling *out);

/** @brief 把采样参数格式化为多行文本（调用方提供缓冲） */
void WtBladeSampling_Format(const WtBladeSampling *sampling, char *buf, size_t bufLen);

/**
 * @brief 单片叶片的巡检站位序列（规划器核心）
 *
 * 把叶片自叶根向叶尖离散成若干站位，每个站位给出无人机的空间位置与
 * 云台指向。站位相对叶片点的偏移方向与距离决定了成像角度、遮挡情况
 * 与飞行安全性，是本模块唯一需要权衡的设计点。
 *
 * 偏移由两项构成：沿巡检面法向的拍摄距离 standoff，以及沿叶片轴向的
 * 斜视分量（profile->bladeAxialShiftM，正=朝下游）。后者与叶片轴线平行，
 * 只改变观察方位、不改变成像距离的量级，因此不参与安全距离校验，可以
 * 自由用来换取斜视角；代价是斜距略增，目标 GSD 反算的余量要能覆盖。
 *
 * @param spec      风机参数
 * @param frame     风轮参考系（含相位角）
 * @param bladeIndex 叶片序号
 * @param side      巡检面（单面）
 * @param profile   作业剖面
 * @param outPoints 输出数组
 * @param maxPoints 输出数组容量
 * @param outCount  实际输出点数
 */
WtPlanResult WtPlan_BladeSamples(const WtTurbineSpec *spec,
                                 const WtRotorFrame *frame,
                                 int bladeIndex,
                                 WtBladeSide side,
                                 const WtInspectionProfile *profile,
                                 WtPlanPoint *outPoints,
                                 size_t maxPoints,
                                 size_t *outCount);

/**
 * @brief 塔筒垂直巡检航线
 *
 * 自塔基上方至轮毂高度螺旋环绕，逐层拍摄塔筒环焊缝与涂层。
 * 顶部高度会被自动限制在当前相位下最低叶片之下，避免在叶片扫掠
 * 区内贴塔飞行。
 *
 * @param frame 当前风轮参考系（塔筒段的安全高度依赖相位）
 */
WtPlanResult WtPlan_Tower(const WtTurbineSpec *spec,
                          const WtRotorFrame *frame,
                          const WtInspectionProfile *profile,
                          WtMission *mission);

/**
 * @brief 组装一条完整的单机巡检航线
 *
 * 顺序：起始悬停点 → 粗模环绕（可选） → 逐叶片精细巡检 → 塔筒（可选）
 *       → 退场点。返回的点序列可直接映射为 PSDK 航点。
 * 塔筒段是否包含由 profile->includeTower 决定。
 */
WtPlanResult WtPlan_BuildMission(const WtTurbineSpec *spec,
                                 const WtRotorFrame *frame,
                                 const WtInspectionProfile *profile,
                                 WtMission *mission);

/* ------------------------------------------------------------------ */
/* 校验                                                                */
/* ------------------------------------------------------------------ */

/** 安全校验统计 */
typedef struct {
    bool ok;                 /*!< 全部点均通过校验 */
    int minSafeDistViolations; /*!< 到叶片轴线距离小于 minSafeDistM 的点数 */
    int groundViolations;      /*!< 低于地面余量的点数 */
    int bladeHazardViolations; /*!< 落入叶片扫掠危险区的点数与航段数 */
    int gsdViolations;         /*!< 缺陷检测用图超出目标 GSD 的点数 */
    double requiredSafeDistM;  /*!< 本剖面要求的最近安全距离 m */
    double targetGsdMmPerPx;   /*!< 本剖面要求的成像分辨率 */
    double minSafeDistM;       /*!< 实测最小安全距离 m */
    double requiredGsdM;       /*!< 目标 GSD 对应的最大拍摄距离 m */
    double maxDefectDistanceM; /*!< 叶片缺陷用图的实际最大拍摄距离 m */
    double maxDefectGsdMmPerPx;/*!< 缺陷用图的最差 GSD（唯一有质量含义的指标） */
    double avgDefectGsdMmPerPx;/*!< 缺陷用图的平均 GSD */
    double avgModelGsdMmPerPx; /*!< 粗模用图的平均 GSD，仅供记录 */
} WtSafetyReport;

/**
 * @brief 对整条航线做安全与成像质量校验
 *
 * 逐点计算：到各叶片轴线的最近距离、离地高度、是否越界、GSD。
 * 该函数是「航线能否下发」的守门人，必须在飞行前执行。
 */
WtSafetyReport WtPlan_ValidateMission(const WtTurbineSpec *spec,
                                      const WtRotorFrame *frame,
                                      const WtInspectionProfile *profile,
                                      const WtMission *mission);

/** @brief 把校验报告格式化为多行文本（调用方提供缓冲） */
void WtSafetyReport_Format(const WtSafetyReport *report, char *buf, size_t bufLen);

/** @brief 把航线导出为 CSV，便于在 PC 上核对与在 GIS 中预览 */
WtPlanResult WtMission_ExportCsv(const WtMission *mission, const char *path);

#ifdef __cplusplus
}
#endif

#endif /* WT_PLAN_H */