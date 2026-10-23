/**
 * @file wt_rotor_fit.h
 * @brief 由激光实测的轮毂 + 三个叶尖反推叶轮盘面几何。
 *
 * ## 这一层解决什么
 *
 * 既有规划链（`WtPlan_BladeTrace`）需要一个 `WtRotorFrame`：盘面中心、
 * 法向、以及 1 号叶片的相位角。台账能给出中心与法向的**名义值**，但给不了
 * 两样东西：
 *
 *   - **停用角**：叶片停在哪，每次停机都不同
 *   - **盘面实际朝向**：机舱随风偏航，台账里的 headingDeg 只是安装朝向
 *
 * 而这两样正是贴叶航线的安全前提 —— `WtRunner_PlanTurbine` 在停用角未知时
 * 无条件拒绝执行，就是因为「叶片停在 12 点」这个假设一旦不成立，贴近叶片的
 * 航线就是撞机。本模块用 4 次激光测距把这两样测出来。
 *
 * ## 为什么不反推塔基坐标
 *
 * 看起来更彻底的做法是从实测点反推整个 `WtTurbineSpec`（含 `base`）。
 * 但那不可解：已知轮毂中心，要分解出「塔基在下方多少（hubHeight）」与
 * 「水平前伸多少（nacelleOffset）」需要额外观测，4 个点给不出这个自由度。
 *
 * 所以分工是：**台账提供坐标系与名义尺寸，实测修正盘面的姿态与相位**。
 * 实测中心同时与台账中心比对，差太多说明台账错了或认错了风机。
 *
 * ## 两个几何陷阱
 *
 * 1. **三个叶尖所在的平面不过轮毂中心。** 锥角把叶尖拉向上游，预弯又加一截，
 *    所以叶尖平面相对轮毂平行偏移约 `R·sin(cone) + prebend`（40 m 叶片、
 *    3° 锥角约 2.1 m）。把「轮毂到叶尖平面的距离」当拟合误差会得出
 *    「实测全是错的」这种结论。本模块把它作为 `axialOffsetM` **报出来**，
 *    不计入残差。
 *
 * 2. **叶尖到轮毂的直线距离不是盘面内半径。** 它是斜距，含锥角造成的轴向
 *    分量。半径必须取叶尖在盘面内的**投影**到中心的距离。
 *
 * ## 对高度基准的免疫
 *
 * 激光返回的海拔基准没有文档承诺（姊妹项目实测与气压高差约 38 m）。
 * 但本模块只用**相对轮毂点**的量算法向、半径与相位，整体平移被抵消 ——
 * 所以基准偏差不影响拟合结果。它只会出现在与台账比对的
 * `centerOffsetVerticalM` 里，该字段因此单独报，不与水平偏差混在一起。
 */

#ifndef WT_ROTOR_FIT_H
#define WT_ROTOR_FIT_H

#include <stdbool.h>
#include <stddef.h>

#include "wt_geometry.h"
#include "wt_turbine.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 一次采集里最多的叶尖数，与 WT_MAX_BLADES 对齐 */
#define WT_ROTOR_FIT_MAX_TIPS WT_MAX_BLADES

/** 拟合结果 */
typedef struct {
    bool ok;                  /*!< 几何是否解出（质量是否达标另看下面的指标） */
    const char *message;      /*!< 失败原因；成功时为 "ok" */

    WtEnu rotorCenter;        /*!< 轮毂中心，直接取自激光实测点 */
    WtEnu rotAxis;            /*!< 盘面法向单位矢量，指向下游（机舱侧） */
    WtEnu upRef;              /*!< 盘面内「12 点钟」方向单位矢量 */
    WtEnu rotRef;             /*!< 盘面内切向单位矢量，相位增大方向 */

    int tipCount;
    double tipPhaseDeg[WT_ROTOR_FIT_MAX_TIPS];  /*!< 各叶尖实测相位角（度） */
    double tipRadialM[WT_ROTOR_FIT_MAX_TIPS];   /*!< 各叶尖盘面内径向距离 m */

    /*
     * 停用角 = 采集顺序里第一个叶尖的相位角。
     *
     * 这是「依次采集」流程的一个附带好处：操作员按 NO.1/NO.2/NO.3 的顺序
     * 打激光，编号就由采集顺序确定了，不需要 WtBladeId 那套按 park_phase
     * 匹配角度的反推 —— 而那套反推本身就要先有 park_phase，是个循环依赖。
     */
    double parkPhaseDeg;

    double rotorRadiusM;      /*!< 叶片半径 = 各叶尖径向距离的均值 */
    double radiusSpreadPct;   /*!< 径向距离散布 %，真实叶片等长，应当很小 */
    double gapDevDeg;         /*!< 相邻相位间隔与 360/n 的最大偏差（度） */
    double axialOffsetM;      /*!< 叶尖平面相对轮毂的轴向偏移（锥角+预弯） */

    /* 与台账的交叉校验 */
    double centerOffsetHorizM;    /*!< 实测中心与台账中心的水平距离 m */
    double centerOffsetVerticalM; /*!< 垂直差 m —— 可能含激光海拔基准偏差，单独看 */
    double radiusErrPct;          /*!< 实测半径相对台账 rotorDiameter/2 的偏差 % */
    double axisAngleErrDeg;       /*!< 实测法向与台账法向的夹角（度） */
} WtRotorFitResult;

/** 采信阈值 */
typedef struct {
    double maxRadiusSpreadPct;  /*!< 径向散布上限 %，超过视为某个叶尖打偏了 */
    double maxGapDevDeg;        /*!< 相位间隔偏差上限 度 */
    double maxCenterOffsetM;    /*!< 实测中心与台账中心的水平偏差上限 m */
    double maxRadiusErrPct;     /*!< 实测半径与台账偏差上限 % */
    double maxAxisAngleErrDeg;  /*!< 实测法向与台账法向夹角上限 度 */
} WtRotorFitLimits;

/** @brief 默认阈值 */
WtRotorFitLimits WtRotorFit_DefaultLimits(void);

/**
 * @brief 由实测点拟合盘面几何。
 *
 * @param spec     风机台账：提供名义法向与尺寸作为比对基准和法向定向依据
 * @param hubEnu   轮毂中心实测点（ENU，与 spec->base 同一坐标系）
 * @param tipEnu   各叶尖实测点，**顺序即编号**（[0] = NO.1）
 * @param tipCount 叶尖数，需 >= 2 且 <= WT_ROTOR_FIT_MAX_TIPS
 * @param out      拟合结果
 * @return true 表示几何解出；质量是否达标用 WtRotorFit_CheckLimits 判
 *
 * @note 至少需要 2 个叶尖才能定出平面（连同轮毂点共 3 点）。只有 1 个叶尖时
 *       平面不唯一，直接拒绝 —— 宁可不出结果，也不要出一个看起来合理的。
 *
 * @warning **两叶降级路径的法向误差远大于直觉。** 三叶时法向由三个叶尖
 *          自己定出，精确；两叶时只能用「轮毂 + 两叶尖」，而轮毂不在叶尖
 *          平面上，法向因此偏斜：
 *
 *            偏角 = atan( c·|u₀−u₁| / (r·|sin Δφ|) )
 *                   c = R·sinθ + prebend，r = R·cosθ，Δφ = 两叶相位差
 *
 *          80 m 叶轮 / 3° 锥角 / 2 m 预弯、两叶相差 240° 时是 **11.6°**，
 *          不是锥角的 3°（已由 `wt_test_rotor_fit` 按闭式解钉住）。
 *          它会吃掉 `maxAxisAngleErrDeg` 默认余量的将近一半，所以两叶结果
 *          只适合粗定位，要贴叶飞请补齐第三个叶尖。
 */
bool WtRotorFit_FromPoints(const WtTurbineSpec *spec,
                           WtEnu hubEnu,
                           const WtEnu *tipEnu, int tipCount,
                           WtRotorFitResult *out);

/**
 * @brief 按阈值判定拟合结果是否可用于规划。
 *
 * 分开成两个函数而不是在 FromPoints 里直接判：几何解出与质量达标是两件事，
 * 前者失败说明输入退化（点重合、共线），后者失败说明某个点打偏了或台账不对 ——
 * 现场要据此做不同处置，所以必须能分辨。
 *
 * @param outReason 不通过时写入原因（可为 NULL）
 */
bool WtRotorFit_CheckLimits(const WtRotorFitResult *fit,
                            const WtRotorFitLimits *limits,
                            const char **outReason);

/**
 * @brief 把拟合结果转成既有规划链吃的 WtRotorFrame。
 *
 * 转出来的 frame 可以直接喂给 `WtPlan_BladeTrace` / `WtPlan_BuildMission`，
 * 替代 `WtTurbine_BuildRotorFrame` 的台账推算版本。
 */
WtRotorFrame WtRotorFit_ToRotorFrame(const WtRotorFitResult *fit);

/** @brief 把拟合摘要格式化成一行行文本，供日志与报告使用 */
void WtRotorFit_Format(const WtRotorFitResult *fit, char *buf, size_t bufLen);

#ifdef __cplusplus
}
#endif

#endif /* WT_ROTOR_FIT_H */
