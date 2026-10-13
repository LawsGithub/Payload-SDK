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
 * @brief 标定**地面基准**（激光海拔）—— 目标高的参考零点
 *
 * ## 操作时机（**必须是起飞前、飞机还在起降点**）
 *
 * 飞机在地面时，把激光打向**飞机附近的平整地面**，按「标定地面」。
 * 那一刻的激光海拔就是"地面"这个参考面。
 *
 * ⚠️ **标定与测量必须同一架次、同一场地** —— 它存的是**绝对海拔**，
 * 换了场地或重新上电（定位重收敛）后可能失效，须重标。
 *
 * ## 为什么需要它
 *
 * 见 `LzPole_ComputeTargetHeight()` 的长注释：原先用"起飞点海拔"当零点，
 * 而它与激光海拔**参考面不同**（气压高 vs 椭球高），相减系统性偏约 38 m。
 * 用激光自己标自己的地面，参考面问题被彻底绕开。
 *
 * ## 落盘
 *
 * 写 `data/ground.txt`（与 `pole.txt` 分开 —— 两者的生命周期不同：
 * 杆位每次作业重记，地面基准一个场地标一次）。掉电后仍在。
 *
 * @return `LZ_OK` = 标定成功；
 *         `LZ_ERR_IO` = 读不到激光；
 *         `LZ_ERR_NO_TARGET` = 读数不可用（无回波 / 坐标非法 / 零解）
 */
LzStatus LzPole_CalibrateGround(void);

/**
 * @brief 地面基准海拔（米）；**没标定过返回 NaN**
 *
 * 返回 NaN 而不是 0：0 是一个**合法的海拔**（海平面附近），
 * 用它当"没标定"的哨兵会让"标定过、且地面正好在海平面"与"没标定"
 * 撞在一起 —— 本项目在"零解判据写 `== 0.0`"上踩过同一个形状。
 */
double LzPole_GetGroundAltM(void);

/**
 * @brief 从磁盘读回地面基准（进程启动时调一次）
 *
 * 与 `LzPole_LoadRecorded()` 同一个模式：没有文件是**正常状态**
 * （还没标定过），返回 `LZ_ERR_NOT_READY`，不是错误。
 */
LzStatus LzPole_LoadGround(void);

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
 * 而"超上限"要操作员核对），合并成"算出来是 0"就分不开了。
 *
 * ⚠️ **2026-10-13 起，"起飞点海拔"换成了"地面基准"**（标定得来）——
 * 因为实测发现前者与激光海拔**参考面不同**（气压高 vs 椭球高），
 * 相减会系统性偏掉约 38 m。详见 `LzPole_ComputeTargetHeight()` 的说明。
 */
typedef enum {
    /* ⚠️ 0 是"这次读数不可用、目标高没算" —— **不是** NORMAL。
     * 原先 0 是 NORMAL，于是"miss 非 NONE"的早退路径会把
     * `heightDiag` 留成 NORMAL，读起来像"算过了，正常"。 */
    LZ_LASER_TH_NA = 0,      /*!< 不适用：读数不可用，目标高没算 */
    LZ_LASER_TH_NORMAL,      /*!< 差落在 (0, MAX]，正常 */
    LZ_LASER_TH_GROUND,      /*!< 差 ≤ 0 —— 激光打在**基准那一层**（地面），合法 */
    LZ_LASER_TH_OVER_MAX,    /*!< 差 > MAX —— 打到远处 / 基准标错 */
    LZ_LASER_TH_NO_GROUND,   /*!< **还没标定地面基准** —— 操作员须先按「标定地面」 */
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
 * 3. **目标高**：激光海拔 − 地面基准 —— 分类结果进 `heightDiag`
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
 * @param raw        原始读数（单位 0.1 m）
 * @param groundAltM 地面基准海拔（米，标定得来）；**NaN = 没标定过**
 * @param out        [out] 结果；`NULL` 时返回 `LZ_ERR_PARAM`
 * @return `LZ_OK` = 可用；`LZ_ERR_NO_TARGET` = 不可用（**成因看 `out->miss`**）；
 *         `LZ_ERR_PARAM` = `out == NULL`
 */
LzStatus LzPole_PrepareLaserRecord(const LzLaserRawReading *raw,
                                   double groundAltM, LzPoleLaserRecord *out);

/**
 * @brief 由「激光点海拔 − 地面基准海拔」算**目标离地高度**（**纯逻辑，零依赖**）
 *
 * ## ⚠️ 2026-10-13：基准从「起飞点海拔」改成「标定的地面海拔」
 *
 * ### 实测暴露的问题
 *
 * 原方案（用户 2026-09-28）是 `激光海拔 − 起飞点海拔`。2026-10-13 现场实测
 * 证明**这两个量的参考面不同**：
 *
 * | 场景 | 激光点海拔 | 起飞点海拔 | 飞机椭球高 | 距离 | 反算目标高 |
 * |---|---|---|---|---|---|
 * | **瞄地面** | 34.10 | −4.15 | 61.71 | 30.9 m | **38.25** ❌ |
 * | **瞄杆顶** | 49.60 | −4.15 | 61.72 | 18.8 m | 53.75 |
 *
 * 三条独立证据：
 * 1. 地面读数稳定在 33.2–34.3、杆顶在 48.5–51.6 ⇒ 真实杆高约 **16 m**
 * 2. `飞机椭球高 61.72 − 地面 33.5 = 28.2 m`，而同一时刻的飞行高度是
 *    **29.1 m** ⇒ **激光海拔与飞机椭球高同一参考面（WGS84 椭球高）**
 * 3. `ALTITUDE_OF_HOMEPOINT` 头文件原文：`"altitude from sea level ...
 *    also uses the ICAO model"` ⇒ 它是**气压高**
 *
 * ⇒ 相减等于把椭球高与气压高混在一起，**系统性偏掉约 38 m**
 * （本场地 37.6 m，且**随场地地面高程变**）。
 *
 * ### 后果（为什么必须修，不是"偏一点"）
 *
 * `targetHeightM` 直接进俯仰公式的 `-h` 项。取 11:40 那次（半径 14.4、
 * 高度 29.1）：正确杆高 16 ⇒ **−42.3°**（俯视），错值 53.75 ⇒ **+59.7°**
 * （上仰）—— **方向都反了，差 102°**。相机在瞄天空。
 *
 * 还有一个潜伏风险：`LZ_POLE_TARGET_HEIGHT_MAX_M` 是 60 m，而错误偏移
 * 随场地变。本例 33.5 + 37.6 = 53.65 勉强没超；**换个地面更高的场地，
 * 杆高还没超、和就已经超过 60，会被整个判成 0**（静默失效）。
 *
 * ### 新方案：地面标定
 *
 * 操作员在**起飞前、飞机还在起降点**时按「标定地面」，程序把那一刻的
 * 激光海拔记为**地面基准**（落盘 `data/ground.txt`）。之后
 * `targetHeight = 激光海拔 − 地面基准`。
 *
 * 为什么这样对：标定与测量**来自同一个传感器、同一个坐标系**，
 * 参考面问题被彻底绕开 —— 而飞机自身的绝对高度不参与任何一步。
 *
 * ## 出口，都不猜
 *
 * | 情形 | 返回 |
 * |---|---|
 * | 两值有限且差落在 `(0, LZ_POLE_TARGET_HEIGHT_MAX_M]` | 那个差 |
 * | 差 ≤ 0（激光打在基准那一层） | **0** —— 合法用法，瞄它自身 |
 * | 差 > 上限（打到远处 / 基准标错） | **0**，并在日志里报出原始值 |
 * | **没标定过**（`groundAltM` 是 NaN） | **0** |
 *
 * ⚠️ **超限不"钳位到上限"而取 0**：钳位会把一个明显错的量伪装成
 * "一个很高的目标"，而取 0 至少是已知合法的那种用法。
 *
 * ⚠️ 本函数**放在 `#ifdef LZ_POLE_SOURCE_LASER` 之外** —— 与
 * `LzPole_JudgeLaserReading` 同一条纪律：判据要能在桌面上被测试打到。
 * 本项目已经因为"判据在 ifdef 里、测试编不到"踩过一次（激光零解闸门）。
 *
 * @param laserAltM 激光点海拔（米）；NaN = 拿不到
 * @param groundAltM 地面基准海拔（米，标定得来）；**NaN = 没标定过**
 * @return 目标离地高度（米）；不可用时 0
 */
double LzPole_ComputeTargetHeight(double laserAltM, double groundAltM);


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
