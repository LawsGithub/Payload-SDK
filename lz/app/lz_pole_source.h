/**
 * @file lz_pole_source.h
 * @brief 绕飞圆心（杆的 WGS84 坐标）从哪来。
 *
 * 这是整条链路上**唯一的外部输入**。前面所有部分都已就绪：
 * 几何、规划、KMZ 生成、控件入口、航点上传。
 *
 * ## 为什么单列一个模块，而不是直接写在 lz_mission.c 里
 *
 * 因为它**有几条完全不同的实现路径**，而选哪条取决于现场条件：
 *
 * | 方式 | 前提 | 精度 | 现状 |
 * |---|---|---|---|
 * | **操作员记录**（当前默认） | 有定位；激光那条还要打中目标 | 取决于记录时瞄准哪 | 本文件 |
 * | 固定坐标 | 无（人工输入） | 取决于输入 | 保留为兜底，见下 |
 * | 视觉识别 + 定位 | `lz_vision` 连通域实现 | 中 | 算法未实现 |
 *
 * 把"取坐标"这件事收在一个接口后面，换方式时只动这一个文件，
 * 规划/上传/控件全都不受影响。
 *
 * ## 操作员记录：两个来源，不是二选一
 *
 * 操作员在 Pilot 2 控件上按按钮记录，两个按钮语义**不同**：
 *
 * - **记录飞机位** —— 存飞机当前所在位置（`TOPIC_POSITION_FUSED`）。
 *   含义是"我就在圆心正上方"。室内有 GPS fix 也能用。
 * - **记录激光点** —— 存激光瞄准点。含义是"我瞄的就是杆"。
 *   精度高（直出经纬度），但两个前提：**必须打中实物**（有距离），
 *   且**飞机自身要有定位**（瞄准点要靠机身位置解算 ——
 *   飞机没定位时激光给的是零解，哪怕距离读数是有效的）。
 *
 * 两者都写进同一个槽位 —— 因为圆心只有一个，后来的记录覆盖先前的。
 * 记录成功会落盘（`data/pole.txt`），掉电/重启后仍在。
 *
 * ## 未记录时**不**回落固定坐标
 *
 * `LzPole_Acquire()` 在未记录时返回 `LZ_ERR_NOT_READY`，由上层拒绝启动绕飞。
 * 刻意不静默回落到那个写死的坐标：**回落到一个几十公里外的点会让飞机飞过去**，
 * 而操作员以为自己只是在原地绕圈。宁可不起飞 —— 这与
 * "取不到就不起飞"是同一条原则。
 *
 * 固定坐标那套仍然留在代码里，但只在编译时用
 * `-DLZ_POLE_SOURCE_FIXED` 显式选用，运行时不再自动回落。
 */

#ifndef LZ_POLE_SOURCE_H
#define LZ_POLE_SOURCE_H

#include "lz_target.h"
#include "lz_types.h"

/** 记录来源 —— 决定 `LzPole_SourceName()` 的字样与记录时的取数方式 */
typedef enum {
    LZ_POLE_RECORD_AIRCRAFT = 0, /*!< 记录飞机当前位置 */
    LZ_POLE_RECORD_LASER,        /*!< 记录激光瞄准点 */
} LzPoleRecordKind;

/**
 * @brief 取得绕飞圆心
 *
 * @param out [out] 目标杆；`geo` 被填成已记录的坐标
 * @return `LZ_OK` = 取到可用坐标；
 *         `LZ_ERR_NOT_READY` = **还没有记录过**（上层应拒绝启动并提示操作员）
 */
LzStatus LzPole_Acquire(LzTarget *out);

/**
 * @brief 记录飞机当前位置为圆心
 *
 * @param curPos 飞机当前的 WGS84 位置（由调用方从 `TOPIC_POSITION_FUSED` 取，
 *               注意那个话题的经纬度单位是 **rad**，要先转成度）
 * @return `LZ_OK` = 已记录并落盘；`LZ_ERR_NO_TARGET` = 坐标非法（无定位）
 */
LzStatus LzPole_RecordAircraft(const LzGeo *curPos);

/**
 * @brief 记录激光瞄准点为圆心
 *
 * 判据是 **`distance`**，不是 `exception` 白名单 —— 后者取值官方未公开、
 * 实测还在增加（0/1/2/3 都见过），而距离是可自证的量。详见 .c 里的说明。
 *
 * @return `LZ_OK` = 已记录并落盘；
 *         `LZ_ERR_UNSUPPORTED` = 未以 `-DLZ_POLE_SOURCE_LASER` 编译；
 *         `LZ_ERR_NO_TARGET` = 以下三者之一：
 *             - 测不到距离（`distance == 0`，无回波）
 *             - 坐标非法（超出经纬度范围）
 *             - **坐标是零解**（飞机自身没有定位，瞄准点解算退化）
 */
LzStatus LzPole_RecordLaser(void);

/**
 * @brief 判定一次激光读数能不能当圆心（**纯逻辑，零依赖**）
 *
 * 三道闸各自独立、不能互相担保：
 *   ① 必须有距离（`distance == 0` 时坐标会退化成机身位置）
 *   ② 坐标必须合法（经纬度范围内）
 *   ③ **坐标不能是零解**（瞄准点靠机身位置解算，飞机没定位时解算退化）
 *
 * ⚠️ 实测（2026-09-22）出现过 `distance=2.0m` 有效而坐标是零解的情况 ——
 * **"有距离"推不出"坐标有效"**。
 *
 * @param latDeg/lonDeg/altM 激光给出的坐标（**度**）
 * @param distanceM          激光距离（**米**）
 * @return `LZ_OK` / `LZ_ERR_NO_TARGET`
 */
LzStatus LzPole_JudgeLaserReading(double latDeg, double lonDeg, double altM,
                                  double distanceM);

/**
 * @brief 一次激光读数**为什么**不可用
 *
 * ## 为什么要有它（而不是只返回一个 `LZ_ERR_NO_TARGET`）
 *
 * 三种成因的**处置完全不同**：
 *
 * | 值 | 成因 | 操作员该做什么 |
 * |---|---|---|
 * | `LZ_LASER_MISS_NO_DISTANCE` | 无回波 | **重新瞄准** |
 * | `LZ_LASER_MISS_BAD_COORD` | 坐标越界 | 报 bug（飞机给了非法值） |
 * | `LZ_LASER_MISS_NULL_SOLUTION` | 零解（飞机没定位） | **等定位**，别调瞄准 |
 *
 * 合并成一个错误码、只靠日志文案区分的话，**"分得开"这件事本身没有断言
 * 守着** —— 文案改一个字就静默退化了。本项目在 `LzVisionMiss` 上踩过
 * 同一个形状（"没红块"与"置信度不足"混在一起，主应用只报了后者，
 * 把操作员引去反复调瞄准而病因在代码里）。
 *
 * ⇒ 与 `LzVisionMiss` 同一条纪律：**返回值要能区分病因，文案只是它的呈现。**
 */
typedef enum {
    LZ_LASER_MISS_NONE = 0,      /*!< 可用 */
    LZ_LASER_MISS_NO_DISTANCE,   /*!< `distance == 0`，无回波 */
    LZ_LASER_MISS_BAD_COORD,     /*!< 经纬度超出合法范围 */
    LZ_LASER_MISS_NULL_SOLUTION, /*!< 零解邻域 —— 飞机自身没有定位 */
} LzLaserMiss;

/**
 * @brief 目标高度那一步走到了哪个出口
 *
 * 与 `LzLaserMiss` 同一个理由：四个出口的处置不同（"打地面"是**合法用法**，
 * 而"超上限"要操作员核对参考面），合并成"算出来是 0"就分不开了。
 */
typedef enum {
    /* ⚠️ 0 是"这次读数不可用、目标高没算" —— **不是** NORMAL。
     * 原先 0 是 NORMAL，于是"miss 非 NONE"的早退路径会把
     * `heightDiag` 留成 NORMAL，读起来像"算过了，正常"。 */
    LZ_LASER_TH_NA = 0,      /*!< 不适用：读数不可用，目标高没算 */
    LZ_LASER_TH_NORMAL,      /*!< 差落在 (0, MAX]，正常 */
    LZ_LASER_TH_GROUND,      /*!< 差 ≤ 0 —— 打的是地面，**合法** */
    LZ_LASER_TH_OVER_MAX,    /*!< 差 > MAX —— 参考面不一致 / 打到远处 */
    LZ_LASER_TH_NO_HOME,     /*!< 拿不到起飞点海拔 */
    /* ⚠️ **刻意没有"激光海拔拿不到"这一项**：它不可达。
     * 实测（2026-10-06）`altitudeDm = NaN` 时 `LzGeo_IsValid()` 的
     * **有限性检查**在更前面就把它拦成 `LZ_LASER_MISS_BAD_COORD`，
     * 函数早退，根本走不到算目标高那一步。
     * 加一个到不了的分支 = 加一段没人能验的代码 —— 删掉。 */
} LzLaserHeightDiag;

/**
 * @brief 激光测距的**原始读数**（单位照 SDK，未换算）
 *
 * 单独一个结构体是为了让"原始读数 → 该记什么"这段判据**能上桌面**：
 * PSDK 的 `T_DjiCameraManagerLaserRangingInfo` 在桌面上找不到，
 * 而这个结构体只依赖 `double`。
 */
typedef struct {
    double latitudeDeg;  /*!< 度，SDK 直出 */
    double longitudeDeg; /*!< 度 */
    double altitudeDm;   /*!< **0.1 m** —— 与 SDK 一致，不在这里换算 */
    double distanceDm;   /*!< **0.1 m** */
} LzLaserRawReading;

/** @brief 一次激光记录该记什么（判据的输出） */
typedef struct {
    LzGeo  geo;             /*!< 圆心坐标（海拔已换算成米） */
    double targetHeightM;   /*!< 目标离地高；0 = 点目标 */
    double distanceM;       /*!< 距离（米），供日志 */
    double laserAltM;       /*!< 激光点海拔（米），供日志 */
    LzLaserMiss miss;       /*!< 为什么不可用（`LZ_OK` 时是 `_NONE`） */
    LzLaserHeightDiag heightDiag; /*!< 目标高走到了哪个出口 */
} LzPoleLaserRecord;

/**
 * @brief 三道闸的**分类**结果 —— 纯逻辑，不打日志
 *
 * 抽出来是为了让 `LzPole_JudgeLaserReading()`（打日志）与
 * `LzPole_PrepareLaserRecord()`（填结果）**共用同一份判据**。
 * 两处各写一遍的话，改了一处另一处不变，而两者都不报错 ——
 * 这正是本项目反复记的"两边各写一份、对不上也不报错"。
 *
 * ⚠️ 调用方传进来的 `altM`/`distanceM` **必须是米**（调用方负责换算）。
 * 换算本身在 `LzPole_PrepareLaserRecord()` 里，那是它存在的理由之一。
 */
LzLaserMiss LzPole_ClassifyLaserReading(double latDeg, double lonDeg,
                                        double altM, double distanceM);

/**
 * @brief 把一次激光原始读数变成"该记什么" —— **纯逻辑，零依赖**
 *
 * 这一步包含的全部内容（此前散在 `LzPole_RecordLaser()` 里，**一行都测不到**）：
 *
 * 1. **单位换算**：`altitude` / `distance` 是 **0.1 m**（结构体注释明写）
 * 2. **三道闸**：无距离 / 坐标非法 / 零解 —— 分类结果进 `miss`
 * 3. **目标高**：激光海拔 − 起飞点海拔 —— 分类结果进 `heightDiag`
 *
 * ## 为什么必须抽出来
 *
 * 这三样此前全在 `#ifdef LZ_POLE_SOURCE_LASER` 里面 —— 桌面构建不编它，
 * 于是**写成 `distance / 100.0` 也不会有任何测试变红**（实测确认）。
 * 而单位写错的表现是"距离小十倍"，看起来像个合理的读数。
 *
 * ⚠️ 抽出来之后，`LzPole_RecordLaser()` 只剩两件事：**取数**与**打日志** ——
 * 一个 SDK 调用 + 一个按 `miss`/`heightDiag` 分派的 switch。判据全在这里。
 *
 * ⚠️ **本函数不做任何 I/O、不落盘、不打日志**（`USER_LOG_*` 都不调）——
 * 它的输出全在返回值里，调用方负责呈现。这样测试才断言得到"分得开"。
 *
 * ⚠️ **刻意不接收「飞机融合海拔」** —— 那个量只进日志、不参与任何判定。
 * 传进来却不用的参数会让下一个读代码的人以为它参与判定，
 * 进而以为"改了它会影响结果"。日志由调用方自己打。
 *
 * @param raw       原始读数（单位 0.1 m）
 * @param homeAltM  起飞点海拔（米）；NaN = 拿不到
 * @param out       [out] 结果；`NULL` 时返回 `LZ_ERR_PARAM`
 * @return `LZ_OK` = 可用；`LZ_ERR_NO_TARGET` = 不可用（**成因看 `out->miss`**）；
 *         `LZ_ERR_PARAM` = `out == NULL`
 */
LzStatus LzPole_PrepareLaserRecord(const LzLaserRawReading *raw,
                                   double homeAltM, LzPoleLaserRecord *out);

/**
 * @brief 由「激光点海拔 − 起飞点海拔」算**目标离地高度**（**纯逻辑，零依赖**）
 *
 * ## 用法与来历（用户 2026-09-28 的方案）
 *
 * 把激光打在**旗面**上时，`LaserRangingInfo.altitude` 就是旗面那个点的
 * 高程；减去起飞点海拔就是旗面离地多高。这条比"距离 × 云台俯仰角"
 * 少两个误差源：俯仰角有噪声（实测 Rotate 实速只有下发的 10–20%），
 * 经纬度解算还要机身自身定位参与。
 *
 * ## 三个出口，都不猜
 *
 * | 情形 | 返回 |
 * |---|---|
 * | 两值有限且差落在 `(0, LZ_POLE_TARGET_HEIGHT_MAX_M]` | 那个差 |
 * | 差 ≤ 0（激光打的是地面） | **0** —— 合法用法，瞄它自身 |
 * | 差 > 上限（参考面不一致 / 打到远处） | **0**，并在日志里报出原始值 |
 * | 任一值拿不到（NaN） | **0** |
 *
 * ⚠️ **超限不"钳位到上限"而取 0**：钳位会把一个明显错的量伪装成
 * "一个很高的目标"，而取 0 至少是已知合法的那种用法。
 *
 * ⚠️ 本函数**放在 `#ifdef LZ_POLE_SOURCE_LASER` 之外** —— 与
 * `LzPole_JudgeLaserReading` 同一条纪律：判据要能在桌面上被测试打到。
 * 本项目已经因为"判据在 ifdef 里、测试编不到"踩过一次（激光零解闸门）。
 *
 * @param laserAltM 激光点海拔（米）；NaN = 拿不到
 * @param homeAltM  起飞点海拔（米）；NaN = 拿不到
 * @return 目标离地高度（米）；不可用时 0
 */
double LzPole_ComputeTargetHeight(double laserAltM, double homeAltM);

/**
 * @brief 从磁盘读回上次记录的圆心（进程启动时调一次）
 *
 * 失败不是错误 —— 首次运行本来就没有文件。返回 `LZ_ERR_NOT_READY`
 * 表示"没有已记录的点"，与"读失败"（`LZ_ERR_IO`）区分开：前者是正常状态。
 */
LzStatus LzPole_LoadRecorded(void);

/** @brief 当前状态下，圆心会来自哪里（用于日志与浮窗消息） */
const char *LzPole_SourceName(void);

/**
 * @brief 已记录的圆心，供界面显示
 * @return `LZ_OK` 时 `out` 被填好；未记录时返回 `LZ_ERR_NOT_READY`
 */
LzStatus LzPole_GetRecorded(LzGeo *out);

#endif /* LZ_POLE_SOURCE_H */
