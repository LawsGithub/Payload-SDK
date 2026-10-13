/**
 * @file lz_pole_source.c
 * @brief 绕飞圆心的取得方式 —— 见头文件里对各条路径的说明。
 *
 * ## 这个文件里的三条路径不是并列的
 *
 * ```
 * 运行时：  LzPole_Acquire() ──> 已记录的点（操作员在 Pilot 2 控件上记的）
 *                              └> 未记录 ⇒ LZ_ERR_NOT_READY，上层拒绝启动
 *
 * 编译期：  -DLZ_POLE_SOURCE_LASER  ⇒ 额外启用"记录激光点"按钮
 *           -DLZ_POLE_SOURCE_FIXED  ⇒ 忽略记录、恒定用写死的坐标（调试用）
 * ```
 *
 * **运行时不再自动回落固定坐标。** 理由：回落到一个几十公里外的坐标会让
 * 飞机飞过去，而操作员以为在原地绕圈 —— 这种"静默换了一个完全不同的位置"
 * 比直接拒绝起飞危险得多。固定坐标只在显式加 `-DLZ_POLE_SOURCE_FIXED`
 * 时才生效，那是"我知道我在干什么"的调试场景。
 */

#include "lz_pole_source.h"

#include <dji_logger.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#ifdef LZ_POLE_SOURCE_LASER
/* ⚠️ 这两个 include **必须在 ifdef 内** —— `lz_test_pole` 会在桌面上编本文件
 * （用 tests/stub/ 的替身头文件替换 dji_logger.h），而 PSDK 的头文件在
 * 桌面上找不到。放进 ifdef 里，桌面构建就只编到纯逻辑那部分，
 * 与"判据与取数分离"是同一条纪律。 */
#include <dji_camera_manager.h>

#include "lz_bridge_psdk.h"   /* LzBridge_GetFusedAltitudeM（诊断日志用） */
#endif

/* ------------------------------------------------------------------ */
/* 落盘                                                               */
/* ------------------------------------------------------------------ */

/* 存到应用目录下的 data/。
 *
 * ⚠️ 不能写绝对路径 `/data/...` —— 应用以 uid=1000(dji) 运行，`/data` 是
 * root:root 755，写不进去。官方约定是**相对应用目录**（dpk 包里已建好
 * data/，属主 dji:dji）。这也是 build_dpk.sh 在包内建 data/ 的原因。
 *
 * CWD 在两种运行方式下不同（dpk 运行=包根、源码树调试=lz/），
 * 与 lz_widget.c 解析控件配置目录是同一个问题。这里用同样的做法：
 * 写第一个能打开的路径。 */
#define LZ_POLE_RECORD_PATH "data/pole.txt"

/* 已记录的圆心。静态而非每次读盘：`LzPole_Acquire()` 在启动绕飞时调用，
 * 不应引入文件 I/O 的不确定性。 */
static bool s_haveRecord = false;
static LzGeo s_recorded;
static LzPoleRecordKind s_recordKind = LZ_POLE_RECORD_AIRCRAFT;

/* ---- 地面基准（目标高的参考零点）----
 *
 * ⚠️ **2026-10-13 新增** —— 起因是现场实测发现"激光海拔 − 起飞点海拔"
 * 把两个不同参考面的量相减（椭球高 vs 气压高），系统性偏约 38 m。
 * 完整证据链见 `LzPole_ComputeTargetHeight()` 的说明。
 *
 * 值由「标定地面」按钮在**起飞前、飞机还在起降点**时记录：
 * 把激光打向附近平整地面，那一刻的激光海拔就是"地面"这个参考面。
 *
 * ⚠️ 存 **NaN** 表示"没标定过"，不存 0 —— 0 是合法海拔（海平面附近），
 * 用它当哨兵会让"标定过、且地面正好在海平面"与"没标定"撞在一起。
 * 本项目在"零解判据写 `== 0.0`"上踩过同一个形状。 */
static double s_groundAltM = NAN;
static bool s_haveGround = false;

/** 地面基准的落盘文件。与 `pole.txt` 分开 —— 生命周期不同：
 * 杆位每次作业重记，地面基准一个场地标一次。 */
#define LZ_GROUND_RECORD_PATH "data/ground.txt"

/**
 * @brief 把坐标写成一行文本
 *
 * 格式刻意用人类可读的十进制 + 一行注释头，方便现场 `cat` 出来核对：
 *
 *     lon=112.9210020 lat=28.1788480 alt=60.0 src=laser
 *
 * 不用二进制：这个文件是给人和给程序共用的，可读性的收益远大于几字节。
 *
 * ⚠️ **2026-10-13 起不再写 `th=`**（目标高）。原因：`alt=` 本身就是
 * **激光点海拔**（圆心即激光点），而地面基准在同一坐标系里 ——
 * 目标高 = `alt − groundAlt` 可以**用时现算**，不必存第二份。
 *
 * 为什么这比存 `th=` 好：
 *   1. **单一真值** —— 存了 `th=` 就有两个量描述同一件事，标定后两者会打架
 *   2. **标定后旧记录自动修正** —— 设备上那份 `th=53.75`（错的）不必重记，
 *      标定完立刻变正确值。若存死，错误会一直粘着
 *   3. 少一个要同步的字段 */
static LzStatus lz_record_save(void)
{
    /* 目录可能不存在（首次运行、或调试时从源码树跑）。
     * 不在这里 mkdir —— 一旦引入建目录，就得处理权限、已存在等各种分支，
     * 而 dpk 包里 data/ 本来就存在。失败时明确返回 IO 错误，
     * 由调用方报给操作员，不静默。 */
    FILE *fp = fopen(LZ_POLE_RECORD_PATH, "w");
    if (fp == NULL) {
        USER_LOG_ERROR("无法写入杆位记录文件 %s（目录不存在或不可写）",
                       LZ_POLE_RECORD_PATH);
        return LZ_ERR_IO;
    }

    fprintf(fp, "lon=%.7f lat=%.7f alt=%.1f src=%s\n",
            s_recorded.longitudeDeg, s_recorded.latitudeDeg, s_recorded.altitudeM,
            (s_recordKind == LZ_POLE_RECORD_LASER) ? "laser" : "aircraft");
    fclose(fp);
    return LZ_OK;
}

/**
 * @brief 把地面基准写盘（一行文本，便于现场 `cat` 核对）
 *
 * 与 `lz_record_save()` 分开：两者的**生命周期不同** —— 杆位每次作业重记，
 * 地面基准一个场地标一次。混在一个文件里的话，重记杆位会把基准覆盖掉。
 *
 * ⚠️ **只在 `#ifdef LZ_POLE_SOURCE_LASER` 里有调用者**（标定要靠激光），
 * 所以函数体也放在那个 `#ifdef` 里 —— 否则桌面构建（不启用激光）会报
 * `defined but not used`。放外面再套 `(void)` 抑制是下策：
 * 那会让"这个函数在桌面构建里是死的"这件事从编译输出里消失。
 */
#ifdef LZ_POLE_SOURCE_LASER
static LzStatus lz_ground_save(void)
{
    FILE *fp = fopen(LZ_GROUND_RECORD_PATH, "w");
    if (fp == NULL) {
        USER_LOG_ERROR("无法写入地面基准文件 %s（目录不存在或不可写）",
                       LZ_GROUND_RECORD_PATH);
        return LZ_ERR_IO;
    }
    fprintf(fp, "groundAlt=%.2f\n", s_groundAltM);
    fclose(fp);
    return LZ_OK;
}

#endif /* LZ_POLE_SOURCE_LASER —— 仅为 lz_ground_save 而开的块 */

LzStatus LzPole_LoadGround(void)
{
    FILE *fp = fopen(LZ_GROUND_RECORD_PATH, "r");
    if (fp == NULL) {
        /* 首次运行没有这个文件是**正常状态**（还没标定过），不是错误。
         *
         * ⚠️ **但仍要清掉内存里的旧值** —— 与 `LzPole_LoadRecorded()` 同一条
         * 纪律：本函数的语义是"让内存与磁盘一致"，盘上没有基准时内存里
         * 也不该有。不清的话，调用方看到"当作未标定"紧接着又拿到一个
         * 上次的基准 —— 说法与行为相反。
         *
         * 这条是被 `lz_test_pole` 当场抓出来的：用例先标定、再删文件、
         * 期望目标高变 0，而内存里那个基准还在。 */
        s_haveGround = false;
        s_groundAltM = NAN;
        USER_LOG_INFO("尚无地面基准（%s 不存在）—— 目标高需先按「标定地面」",
                      LZ_GROUND_RECORD_PATH);
        return LZ_ERR_NOT_READY;
    }

    char line[128] = {0};
    if (fgets(line, sizeof(line), fp) == NULL) {
        fclose(fp);
        USER_LOG_ERROR("地面基准文件读不出内容（%s）", LZ_GROUND_RECORD_PATH);
        s_haveGround = false;
        s_groundAltM = NAN;
        return LZ_ERR_IO;
    }
    fclose(fp);

    double alt = 0.0;
    if (sscanf(line, "groundAlt=%lf", &alt) != 1 || !isfinite(alt)) {
        USER_LOG_ERROR("地面基准文件格式不对（%s），当作未标定处理",
                       LZ_GROUND_RECORD_PATH);
        s_haveGround = false;
        s_groundAltM = NAN;
        return LZ_ERR_IO;
    }

    s_groundAltM = alt;
    s_haveGround = true;
    USER_LOG_INFO("已从 %s 读回地面基准：%.2f m", LZ_GROUND_RECORD_PATH, alt);
    return LZ_OK;
}

double LzPole_GetGroundAltM(void)
{
    /* 没标定过返回 NaN —— 见 `s_groundAltM` 的说明（0 是合法海拔，
     * 不能当"没标定"的哨兵）。 */
    return s_haveGround ? s_groundAltM : NAN;
}

LzStatus LzPole_LoadRecorded(void)
{
    FILE *fp = fopen(LZ_POLE_RECORD_PATH, "r");
    if (fp == NULL) {
        /* 首次运行没有这个文件是**正常状态**，不是错误 */
        USER_LOG_INFO("尚无已记录的杆位（%s 不存在）—— 需操作员在 Pilot 2 控件上记录",
                      LZ_POLE_RECORD_PATH);
        return LZ_ERR_NOT_READY;
    }

    char src[16] = {0};
    LzGeo g;
    memset(&g, 0, sizeof(g));

    /* ⚠️ 用 fgets + sscanf，**不能用「一次 fscanf 读全部字段再看返回值」**
     * （第一版那么写，被 `lz_test_pole` 的"重启后能读回"用例当场抓红）。
     *
     * 原因：格式串里带可选字段时，`%lf` 会去解析 `src=laser` 里的 's'，
     * **在中间某个字段处停下** —— 于是"旧格式"与"文件被截断"两者的返回值
     * 撞在一起，分不开。读成一整行再各自 sscanf，两种格式的成功与否
     * 就是**两个独立的布尔**，不会有这种歧义。
     *
     * ⚠️ **2026-10-13：`th=` 字段已退役**（目标高改为用时现算，见
     * `lz_record_save()` 的说明）。但仍要**接受带 `th=` 的旧文件** ——
     * 设备上现存的 `pole.txt` 就是那个格式，读不回来会让操作员
     * 莫名其妙丢掉一条本来可用的记录。 */
    char line[256] = {0};
    if (fgets(line, sizeof(line), fp) == NULL) {
        fclose(fp);
        USER_LOG_ERROR("杆位记录文件读不出内容（%s）", LZ_POLE_RECORD_PATH);
        s_haveRecord = false;
        memset(&s_recorded, 0, sizeof(s_recorded));
        return LZ_ERR_IO;
    }
    fclose(fp);

    /* 当前格式（4 字段）优先；不匹配再试带 `th=` 的旧格式（5 字段）。
     * 旧格式的 `th` **读出来就丢** —— 它的真值现在是 `alt − groundAlt`。 */
    int fields = 0;
    if (sscanf(line, "lon=%lf lat=%lf alt=%lf src=%15s",
               &g.longitudeDeg, &g.latitudeDeg, &g.altitudeM, src) == 4) {
        fields = 4;
    } else {
        memset(&g, 0, sizeof(g));
        memset(src, 0, sizeof(src));
        double thIgnored = 0.0;
        if (sscanf(line, "lon=%lf lat=%lf alt=%lf th=%lf src=%15s",
                   &g.longitudeDeg, &g.latitudeDeg, &g.altitudeM,
                   &thIgnored, src) == 5) {
            fields = 5;
        }
    }

    /* 字段数不对（文件被截断/手改坏）与坐标非法都拒绝。
     * 半个坐标比没有坐标更危险 —— 它会看起来"有记录"，实际是垃圾。
     *
     * ⚠️ **必须同时清掉内存里的状态**，不能只返回错误码。
     *
     * 这个函数的名字是"从磁盘读回"，它的语义就是**让内存与磁盘一致**：
     * 盘上没有可用点时，内存里也不该有。早先只 `return LZ_ERR_IO`
     * 而没清 `s_haveRecord`，于是出现"日志说当作未记录、紧接着
     * `LzPole_Acquire()` 却成功返回上一次的记录" —— 说法与行为相反。
     * 这是被 `lz_test_pole` 的"文件被改坏"用例抓出来的。 */
    /* 零解也要拒 —— 旧版本可能已经把零解写进过文件，
     * 或者文件被手改成 0,0。读回来等于"有一个看起来有效的圆心"。 */
    if ((fields != 4 && fields != 5) || !LzGeo_IsValid(&g) || LzGeo_IsNullSolution(&g)) {
        USER_LOG_ERROR("杆位记录文件格式不对（读到 %d 个字段），当作未记录处理", fields);
        s_haveRecord = false;
        memset(&s_recorded, 0, sizeof(s_recorded));
        return LZ_ERR_IO;
    }

    s_recorded = g;
    s_haveRecord = true;
    s_recordKind = (strcmp(src, "laser") == 0) ? LZ_POLE_RECORD_LASER
                                               : LZ_POLE_RECORD_AIRCRAFT;

    /* 目标高**不在这里算** —— 它是 `alt − groundAlt`，而 groundAlt 可能
     * 还没读回来（两个文件各自加载）。真值由 `lz_pole_target_height()`
     * 在取用时现算。这里只报坐标与来源。 */
    USER_LOG_INFO("已从 %s 读回杆位：%.7f, %.7f（%s）",
                  LZ_POLE_RECORD_PATH, g.latitudeDeg, g.longitudeDeg, src);
    return LZ_OK;
}

/* ------------------------------------------------------------------ */
/* 记录                                                               */
/* ------------------------------------------------------------------ */

static void lz_record_common(const LzGeo *g, LzPoleRecordKind kind)
{
    s_recorded = *g;
    s_recordKind = kind;
    s_haveRecord = true;

    /* 落盘失败**不回退**内存里的记录：内存中的坐标是可用的（本次绕飞能用），
     * 只是掉电后会丢。把"落盘失败"降级成一条 WARN，而不是让记录整个失败 ——
     * 否则一次磁盘问题会让操作员连当下的绕飞也做不成。 */
    const LzStatus st = lz_record_save();
    if (st != LZ_OK) {
        USER_LOG_WARN("杆位已记在内存中，但落盘失败（%s）—— 重启后会丢失",
                      LzStatus_Str(st));
    }
}

LzStatus LzPole_RecordAircraft(const LzGeo *curPos)
{
    if (curPos == NULL) {
        return LZ_ERR_PARAM;
    }
    if (!LzGeo_IsValid(curPos)) {
        USER_LOG_WARN("无法记录：飞机位置非法（%.7f, %.7f）",
                      curPos->latitudeDeg, curPos->longitudeDeg);
        return LZ_ERR_NO_TARGET;
    }
    /* 零解判据走 `LzGeo_IsNullSolution` —— 关键点是**给邻域而不是比 0**。
     *
     * 实测（2026-09-22，M4T 室内无 GPS）融合位置给的是
     * `lon=0.0000004, lat=0.0000003`，不是精确的 `(0,0)`：
     * 早先写 `== 0.0` 的判据**放行了它**，于是"记录圆心"成功返回了一个
     * 几内亚湾附近的坐标。判据与现实的差距只有一层浮点皮。 */
    if (LzGeo_IsNullSolution(curPos)) {
        USER_LOG_WARN("无法记录：飞机位置在零解邻域内（%.7f, %.7f）—— 当前没有定位",
                      curPos->latitudeDeg, curPos->longitudeDeg);
        return LZ_ERR_NO_TARGET;
    }

    lz_record_common(curPos, LZ_POLE_RECORD_AIRCRAFT);
    USER_LOG_INFO("★ 已记录飞机位置为绕飞圆心：%.7f, %.7f",
                  curPos->latitudeDeg, curPos->longitudeDeg);
    return LZ_OK;
}

/* ------------------------------------------------------------------ */
/* 激光读数的判定 —— 纯逻辑，**不依赖 PSDK**                             */
/* ------------------------------------------------------------------ */

/**
 * @brief 判定一次激光读数能不能当圆心
 *
 * ## 为什么这段单独抽出来、且放在 `#ifdef LZ_POLE_SOURCE_LASER` 之前
 *
 * 判定是**纯逻辑**（几个数值比较），取数才依赖相机接口。分开之后：
 *
 *   - 判定部分可以上桌面测试（`lz_test_pole` 会编到这个函数）
 *   - 取数部分只能在设备上验
 *
 * ⚠️ 这个拆分是被一次**空转的测试**逼出来的：原先的测试只断言
 * `LzGeo_IsNullSolution()` 本身对零解有效，却没有断言
 * `LzPole_RecordLaser()` 真的调用它 —— 反向验证时把闸门删掉，
 * 测试**照样全绿**。原因是未定义 `LZ_POLE_SOURCE_LASER` 时
 * 整个激光实现分支不参与编译，测试根本看不见它。
 *
 * **断言"判据对"不等于断言"接线对"。**
 *
 * ## 三道闸各自独立，不能互相担保
 *
 * 实测（2026-09-22，M4T 室内无 GPS）出现过 `distance=2.0m`（有效！）
 * 而坐标是零解的情况 —— 激光的经纬度 = 机身位置 + 云台朝向 + 距离
 * **解算**出来的，机身位置是零解时解算结果自然是零解。
 *
 * @param latDeg/lonDeg/altM  激光给出的坐标（**度**，不是 rad）
 * @param distanceM           激光距离（**米**，调用方已从 0.1m 换算）
 * @return `LZ_OK` = 可用；`LZ_ERR_NO_TARGET` = 不可用（三种成因，日志里区分）
 */
LzLaserMiss LzPole_ClassifyLaserReading(double latDeg, double lonDeg,
                                        double altM, double distanceM)
{
    /* 闸门 1：必须有距离。
     * 距离为 0 时坐标会退化成机身自身的位置 —— 不是垃圾值，
     * 是个精确可预测的退化情形（实测模拟器上给出飞机初始位置）。 */
    if (!(distanceM > 0.0)) {
        return LZ_LASER_MISS_NO_DISTANCE;
    }

    const LzGeo g = { .latitudeDeg = latDeg, .longitudeDeg = lonDeg, .altitudeM = altM };

    /* 闸门 2：坐标必须合法 */
    if (!LzGeo_IsValid(&g)) {
        return LZ_LASER_MISS_BAD_COORD;
    }

    /* 闸门 3：坐标不能是零解 —— 见本函数上方关于"三闸独立"的说明 */
    if (LzGeo_IsNullSolution(&g)) {
        return LZ_LASER_MISS_NULL_SOLUTION;
    }

    return LZ_LASER_MISS_NONE;
}

/** 把分类结果翻成操作员看得懂的一句话 —— **文案的唯一定义处**。
 *
 * 三种成因的处置不同（重新瞄准 / 报 bug / 等定位），所以文案必须分开。
 * 放在一个函数里而不是散在调用点：`LzPole_RecordLaser()` 与将来别的
 * 调用方都要说同一句话，各写一份就会漂移。 */
static void lz_laser_miss_report(LzLaserMiss miss, double latDeg, double lonDeg,
                                 double distanceM)
{
    switch (miss) {
    case LZ_LASER_MISS_NO_DISTANCE:
        USER_LOG_WARN("激光测不到距离（distance=%.1f m）—— 请对准目标再按", distanceM);
        break;
    case LZ_LASER_MISS_BAD_COORD:
        USER_LOG_WARN("激光给出的坐标非法（%.7f, %.7f，距离 %.1f m）",
                      latDeg, lonDeg, distanceM);
        break;
    case LZ_LASER_MISS_NULL_SOLUTION:
        USER_LOG_WARN("激光坐标是零解（%.7f, %.7f，距离 %.1f m）—— "
                      "瞄准点要靠飞机自身定位解算，当前飞机没有定位",
                      latDeg, lonDeg, distanceM);
        break;
    case LZ_LASER_MISS_NONE:
    default:
        break;
    }
}

LzStatus LzPole_JudgeLaserReading(double latDeg, double lonDeg, double altM,
                                  double distanceM)
{
    const LzLaserMiss miss = LzPole_ClassifyLaserReading(latDeg, lonDeg, altM, distanceM);
    if (miss != LZ_LASER_MISS_NONE) {
        lz_laser_miss_report(miss, latDeg, lonDeg, distanceM);
        return LZ_ERR_NO_TARGET;
    }
    return LZ_OK;
}

/**
 * @brief 由「激光点海拔 − 地面基准海拔」算**目标离地高度**（纯逻辑，零依赖）
 *
 * ## 为什么单独抽出来，且放在 `#ifdef LZ_POLE_SOURCE_LASER` **之前**
 *
 * 与 `LzPole_JudgeLaserReading()` 完全同一个理由（那条注释写得更细）：
 * 判定是纯逻辑、取数才依赖相机接口。放在 `#ifdef` 里面的话，
 * 桌面测试**根本编不到它** —— 于是"改了判定逻辑，测试照样绿"，
 * 而本项目已经在完全相同的地方踩过一次（激光零解闸门，
 * 见 lz/doc/CORE-TESTING.md「判据对 ≠ 接线对」）。
 *
 * ## 量的是什么（2026-10-13 改）
 *
 * 激光打在**旗面**上时 `altitude` 是旗面那个点的高程；减去**地面基准**
 * 就是旗面离地多高。这条比"距离 × 俯仰角"少两个误差源 ——
 * 俯仰角有噪声（实测 Rotate 实速只有下发的 10–20%），
 * 经纬度解算还要机身自身定位参与。
 *
 * ⚠️ **基准从"起飞点海拔"改成了"标定的地面海拔"** —— 2026-10-13 现场
 * 实测证明前者的参考面与激光不同（气压高 vs 椭球高），相减系统性偏
 * 约 38 m。完整证据链见头文件 `LzPole_ComputeTargetHeight()` 的说明。
 *
 * ## 出口，都不猜
 *
 * | 情形 | 返回 | 为什么 |
 * |---|---|---|
 * | 两值都有限且差在 (0, MAX] | 那个差 | 正常 |
 * | 差 ≤ 0（打在基准那一层） | 0 | 点目标，瞄它自身 —— **合法**用法 |
 * | 差 > MAX | 0 | 打到远处 / 基准标错 —— 荒谬量，不采用 |
 * | 地面基准拿不到（NaN） | 0 | 没标定就不猜（用户须先按「标定地面」） |
 *
 * ⚠️ **上限那一条不"钳位到 MAX"而取 0**：钳位会把一个明显错的量
 * 伪装成"一个很高的目标"，而取 0 至少是**已知合法**的那种用法。
 * 与 `LzPlan_ComputeGimbalPitchDeg` 照实返回、由 `LzPlan_Validate`
 * 拒绝是同一条纪律：**别把"做不到"或"不知道"伪装成"做得到"。**
 *
 * @param laserAltM  激光点海拔（米）；NaN 表示拿不到
 * @param groundAltM 地面基准海拔（米，标定得来）；**NaN 表示没标定过**
 * @return 目标离地高度（米），失败时 0
 */
double LzPole_ComputeTargetHeight(double laserAltM, double groundAltM)
{
    if (!isfinite(laserAltM) || !isfinite(groundAltM)) {
        return 0.0;
    }
    const double raw = laserAltM - groundAltM;
    if (!isfinite(raw) || raw <= 0.0 || raw >= LZ_POLE_TARGET_HEIGHT_MAX_M) {
        return 0.0;
    }
    return raw;
}

/**
 * @brief 把一次激光原始读数变成"该记什么" —— 纯逻辑，零依赖
 *
 * 见头文件里的说明。**这里不做 I/O、不打日志**：输出全在返回值里，
 * 调用方负责呈现 —— 测试才断言得到"三种成因分得开"。
 */
LzStatus LzPole_PrepareLaserRecord(const LzLaserRawReading *raw,
                                   double groundAltM, LzPoleLaserRecord *out)
{
    if (raw == NULL || out == NULL) {
        return LZ_ERR_PARAM;
    }
    memset(out, 0, sizeof(*out));

    /* ---- ① 单位换算：SDK 的 `altitude` / `distance` 都是 **0.1 m** ----
     *
     * ⚠️ **这一步此前在 `#ifdef` 里，一行都测不到** —— 实测确认过：
     * 写成 `/ 100.0` 不会有任何测试变红，而表现是"距离小十倍"，
     * 看起来仍像个合理的读数。抽出来之后 `lz_test_pole` 直接钉住它。
     *
     * 经度/纬度是**度**，SDK 直出，不换算（`LzPole_JudgeLaserReading()`
     * 的头文件注释专门写了"不是 rad"，那是踩过的坑）。 */
    const double altM = raw->altitudeDm / 10.0;
    const double distanceM = raw->distanceDm / 10.0;

    out->laserAltM = altM;
    out->distanceM = distanceM;
    out->geo.latitudeDeg = raw->latitudeDeg;
    out->geo.longitudeDeg = raw->longitudeDeg;
    out->geo.altitudeM = altM;

    /* ---- ② 三道闸 ---- */
    out->miss = LzPole_ClassifyLaserReading(raw->latitudeDeg, raw->longitudeDeg,
                                            altM, distanceM);
    if (out->miss != LZ_LASER_MISS_NONE) {
        /* 读数不可用 ⇒ 目标高**没算**。显式置 NA 而不是让它留着 memset 的 0
         * —— 那个 0 曾经等于 NORMAL，读起来像"算过了，正常"。 */
        out->heightDiag = LZ_LASER_TH_NA;
        return LZ_ERR_NO_TARGET;
    }

    /* ---- ③ 目标高：激光海拔 − 地面基准 ----
     *
     * 判据在 `LzPole_ComputeTargetHeight()`（同样零依赖）。这里**只做分类**，
     * 让调用方能按出口说不同的话 —— "打地面"是合法用法（INFO），
     * 而"超上限"要操作员核对（WARN），两者不能合并成同一句。 */
    out->targetHeightM = LzPole_ComputeTargetHeight(altM, groundAltM);

    if (!isfinite(groundAltM)) {
        out->heightDiag = LZ_LASER_TH_NO_GROUND;
    } else {
        const double rawDiff = altM - groundAltM;
        if (rawDiff <= 0.0) {
            out->heightDiag = LZ_LASER_TH_GROUND;
        } else if (out->targetHeightM <= 0.0) {
            /* 差为正却被判 0 ⇒ 必然是超上限那一条 */
            out->heightDiag = LZ_LASER_TH_OVER_MAX;
        } else {
            out->heightDiag = LZ_LASER_TH_NORMAL;
        }
    }
    return LZ_OK;
}

#ifdef LZ_POLE_SOURCE_LASER

/* `[V]` 实测：M4T 的激光测距在位置 1（E1，自带云台相机那一路） */
#define LZ_LASER_MOUNT_POSITION DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1

/* ## 为什么判据是 `distance` 而不是白名单枚举 `exception`
 *
 * `exception` 的取值**官方从未公开**（头文件与中英文档都只有"异常标志"四字），
 * 只能靠实测对照反推。实测见过的值：
 *
 * | exc | 伴随的 distance | 结论 |
 * |---|---|---|
 * | 0 | **4.5 ~ 14.2 m（有实数、且在变）** | **2026-09-22 新观察到** |
 * | 1 | 0（恒为 0） | 无回波 |
 * | 2 | 0 | 过渡态，含义不明 |
 * | 3 | 有实数 | 早先认为的"正常读数" |
 *
 * ⚠️ 早先的实现只认 `3`，于是 **`exception=0` 且距离有效时被误判成"无回波"**
 * —— 操作员看到"激光无回波"，而实际测距工作正常、只是我们的闸门关着。
 * 那次现场排查浪费了一轮，因为**文案把病因指反了**。
 *
 * ⇒ 判据改成**只看 `distance`**：
 *
 *   `distance` 是三个字段里唯一**可自证**的量 —— 它由激光独立测量，
 *   不参与坐标解算，无回波时恒为 0。而 `exception` 是飞机给的语义标签，
 *   语义未定义、取值还在增加，**不能当白名单用**。
 *
 * 这条经验值得推广：**当一个字段的取值域没有权威来源时，不要用枚举白名单，
 * 改用有物理意义、能自证的量。** 白名单会随着新观测不断打补丁，
 * 而漏掉的那一个恰恰会在最需要它的时候出现。
 */

/**
 * @brief 读一次激光原始读数并过三道闸 —— 两条路（记录 / 标定）共用
 *
 * 抽出来的理由与"判据抽进 lz_core"同源：**取数与呈现各写一遍必然漂移**，
 * 而漂移了不会报错。这里只做"取数 + 过闸"，单位换算与判定都在
 * `LzPole_PrepareLaserRecord()`（纯逻辑、桌面可测）里。
 *
 * @param rec [out] 过闸后的结果（`miss == NONE` 才有效）
 * @return `LZ_OK` / `LZ_ERR_IO`（读不到）/ `LZ_ERR_NO_TARGET`（读数不可用）
 */
static LzStatus lz_read_laser_reading(LzPoleLaserRecord *rec)
{
    T_DjiCameraManagerLaserRangingInfo info;
    memset(&info, 0, sizeof(info));

    const T_DjiReturnCode rc =
        DjiCameraManager_GetLaserRangingInfo(LZ_LASER_MOUNT_POSITION, &info);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("读取激光测距失败 rc=0x%08llX", (unsigned long long)rc);
        return LZ_ERR_IO;
    }

    const LzLaserRawReading raw = {
        .latitudeDeg = info.latitude,
        .longitudeDeg = info.longitude,
        .altitudeDm = (double)info.altitude,
        .distanceDm = (double)info.distance,
    };
    /* ⚠️ 这里传的是**地面基准**（可能是 NaN）—— 本函数只用来过前两道闸
     * （距离 / 坐标），目标高那一步的结果由调用方按自己的需要重算。
     * 传 NaN 会让 `heightDiag` 变成 `NO_GROUND`，调用方**不要**读它。 */
    const LzStatus st = LzPole_PrepareLaserRecord(&raw, NAN, rec);
    if (st != LZ_OK) {
        /* 成因由 `rec->miss` 给 —— 三种处置不同（重新瞄准 / 报 bug / 等定位），
         * 文案的唯一定义处在 `lz_laser_miss_report()` 里。 */
        lz_laser_miss_report(rec->miss, raw.latitudeDeg, raw.longitudeDeg,
                             rec->distanceM);
        return st;
    }
    return LZ_OK;
}

LzStatus LzPole_CalibrateGround(void)
{
    LzPoleLaserRecord rec;
    const LzStatus st = lz_read_laser_reading(&rec);
    if (st != LZ_OK) {
        return st;
    }

    /* 标定只取**海拔**，不关心目标高（此时激光就打在脚下地面上）。
     * ⚠️ 三道闸仍然要过 —— 零解 / 无回波时那个 altitude 是垃圾，
     * 存进去会让之后所有目标高都错。 */
    s_groundAltM = rec.laserAltM;
    s_haveGround = true;

    const LzStatus saveSt = lz_ground_save();
    if (saveSt != LZ_OK) {
        /* 与杆位落盘同一条纪律：内存里已生效，只是掉电会丢 ——
         * 降级成 WARN，不让一次磁盘问题废掉本次作业。 */
        USER_LOG_WARN("地面基准已记在内存中，但落盘失败（%s）—— 重启后会丢失",
                      LzStatus_Str(saveSt));
    }

    USER_LOG_INFO("★ 已标定地面基准：激光海拔 %.2f m（距离 %.1f m）—— "
                  "之后的目标高都相对这个面算",
                  rec.laserAltM, rec.distanceM);
    return LZ_OK;
}

LzStatus LzPole_RecordLaser(void)
{
    T_DjiCameraManagerLaserRangingInfo info;
    memset(&info, 0, sizeof(info));

    const T_DjiReturnCode rc =
        DjiCameraManager_GetLaserRangingInfo(LZ_LASER_MOUNT_POSITION, &info);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("读取激光测距失败 rc=0x%08llX", (unsigned long long)rc);
        return LZ_ERR_IO;
    }

    /* ★ 判据全部在 `LzPole_PrepareLaserRecord()` 里（纯逻辑、零依赖、
     * 在 `#ifdef` 之外）—— 单位换算、三道闸、目标高分类都在那边，
     * 所以那条路能在桌面上被测到。本函数只剩**取数**与**呈现**。
     *
     * ⚠️ 这个拆分是被"一行都测不到"逼出来的：原先换算与判定都在这个
     * `#ifdef` 里，实测确认把 `distance / 10.0` 写成 `/ 100.0`
     * **不会有任何测试变红** —— 而表现是"距离小十倍"，看着仍像个合理读数。 */
    const LzLaserRawReading raw = {
        .latitudeDeg = info.latitude,
        .longitudeDeg = info.longitude,
        .altitudeDm = (double)info.altitude,
        .distanceDm = (double)info.distance,
    };
    /* ⚠️ **地面基准**（不是起飞点海拔）—— 2026-10-13 改，见
     * `LzPole_ComputeTargetHeight()` 的证据链。没标定过时是 NaN，
     * 判据会给 0 并把 `heightDiag` 标成 `NO_GROUND`。 */
    const double groundAlt = LzPole_GetGroundAltM();

    LzPoleLaserRecord rec;
    const LzStatus st = LzPole_PrepareLaserRecord(&raw, groundAlt, &rec);
    if (st != LZ_OK) {
        /* 成因由 `rec.miss` 给 —— 三种处置不同（重新瞄准 / 报 bug / 等定位），
         * 文案的唯一定义处在 `lz_laser_miss_report()` 里。 */
        lz_laser_miss_report(rec.miss, raw.latitudeDeg, raw.longitudeDeg,
                             rec.distanceM);
        return st;
    }

    const double altM = rec.laserAltM;
    const double distanceM = rec.distanceM;
    const double targetH = rec.targetHeightM;

    /* ---- 目标高度：激光海拔 − 地面基准 ----
     *
     * ## 为什么走这条路，而不是"距离 × 俯仰角"
     *
     * 把激光打在旗面上，用**激光点自己的高程**减去**地面基准高程**就是
     * 旗面离地多高。这条比"距离 + 云台俯仰角"少两个误差源：
     *   · 云台俯仰角有噪声（实测 Rotate 的实速只有下发的 10–20%，
     *     到位前读到的角不一定是最终角）
     *   · 经纬度解算还要机身自身定位参与（零解就整个废掉）
     * 而 `LaserRangingInfo.altitude` 是激光**直接给出**的第三个独立量。
     *
     * ## ⚠️ 为什么基准必须是"标定的地面"而不是"起飞点海拔"（2026-10-13）
     *
     * `LaserRangingInfo.altitude` 的头文件只写 `Unit: 0.1m`，**没说参考面**；
     * `ALTITUDE_OF_HOMEPOINT` 的原文是 "altitude from sea level ... also uses
     * the ICAO model"，即**气压高**。**两者参考面不同**，相减系统性偏
     * 约 38 m —— 2026-10-13 现场实测确认（瞄地面得 38.25 m）。
     * 完整证据链见 `LzPole_ComputeTargetHeight()` 的说明。
     *
     * ⇒ 改用激光自己标自己脚下那块地面，参考面问题被彻底绕开。
     *
     * ⚠️ 顺带一个**必须显式记录的坑**：激光飘走后打到后面的地面时，
     * `distance` 会突然变大（实测 25 → 60 m 都见过），于是量到的是地面
     * 而不是旗面。这里不试图自动判它 —— 判据不足（旗面飘动 0.5 m 与
     * "打到地面"之间没有干净的分界），硬判会把正常读数误拒。
     * ⇒ 只**如实报出量到的值**，由操作员核对。 */
    /* ★ 目标高那一步的**分类**已经由 `LzPole_PrepareLaserRecord()` 给在
     * `rec.heightDiag` 里 —— 这里只负责**按出口说不同的话**。
     *
     * 为什么不在这里重算：出口的处置不同（"打地面"是合法用法、
     * "没标定"要操作员去按标定按钮），重算一遍就等于**判据有两份**，
     * 改一处另一处不变，而两者都不报错。 */
    const double fusedAlt = LzBridge_GetFusedAltitudeM();

    switch (rec.heightDiag) {
    case LZ_LASER_TH_NO_GROUND:
        USER_LOG_WARN("尚未标定地面基准 —— 目标高按 0 处理，"
                      "俯仰将瞄**地面那一层**而不是杆顶。"
                      "请在起飞前（飞机还在起降点）先按「标定地面」");
        break;
    case LZ_LASER_TH_OVER_MAX:
        USER_LOG_WARN("激光目标高算出来 %.2f m（激光海拔 %.2f − 地面基准 %.2f）"
                      "—— 超出 %.0f m 上限，按点目标（0）处理。"
                      "请核对地面基准是否标错，或激光是不是打到远处地面上去了",
                      altM - groundAlt, altM, groundAlt, LZ_POLE_TARGET_HEIGHT_MAX_M);
        break;
    case LZ_LASER_TH_GROUND:
        USER_LOG_INFO("激光目标高 %.2f m ≤ 0 —— 按点目标处理"
                      "（激光打在地面基准那一层）", altM - groundAlt);
        break;
    case LZ_LASER_TH_NORMAL:
    case LZ_LASER_TH_NA:   /* 走不到这里：miss 非 NONE 时上面已经 return 了 */
    default:
        break;
    }

    /* 自洽校验：几个高程同框。
     *
     * ⚠️ **2026-10-13 起这条日志的用途变了** —— 从前它是"判断哪一对参考面
     * 自洽"的现场手段（当时不知道激光海拔是什么参考面）。现在参考面问题
     * 已经解决（用标定绕开），这条留着是为了**核对标定本身**：
     * 激光海拔若与地面基准接近，说明激光确实打在地面上。
     *
     * 飞机椭球高（`fusedAlt`）仍然打出来 —— 它是判断"激光海拔是什么参考面"
     * 的独立佐证，将来换机型时还要用它。 */
    USER_LOG_INFO("高程自洽校验：激光点海拔 %.2f m ｜ 地面基准 %.2f m ｜ "
                  "飞机椭球高 %.2f m ｜ 反算目标高 %.2f m（距离 %.1f m）",
                  altM, groundAlt, fusedAlt, targetH, distanceM);

    const LzGeo g = {
        .latitudeDeg = info.latitude,
        .longitudeDeg = info.longitude,
        .altitudeM = altM,
    };
    lz_record_common(&g, LZ_POLE_RECORD_LASER);
    USER_LOG_INFO("★ 已记录激光瞄准点为绕飞圆心：%.7f, %.7f（距离 %.1f m，"
                  "目标高 %.2f m，exception=%u）",
                  g.latitudeDeg, g.longitudeDeg, distanceM, targetH,
                  (unsigned)info.exception);
    return LZ_OK;
}

#else  /* !LZ_POLE_SOURCE_LASER */

LzStatus LzPole_RecordLaser(void)
{
    /* 未编译进激光支持时，明确说"不支持"，而不是返回一个含糊的 IO 错误 ——
     * 操作员按了按钮什么都没发生，至少要知道是"这个包没带激光功能"。 */
    USER_LOG_WARN("本包未启用激光记录（需以 -DLZ_POLE_SOURCE_LASER 编译）");
    return LZ_ERR_UNSUPPORTED;
}

LzStatus LzPole_CalibrateGround(void)
{
    /* 地面基准也要靠激光，所以未启用激光时同样不可用 ——
     * 文案与「记录激光点」保持一致，别让操作员以为"只是标定坏了"。 */
    USER_LOG_WARN("本包未启用激光（需以 -DLZ_POLE_SOURCE_LASER 编译）—— "
                  "无法标定地面");
    return LZ_ERR_UNSUPPORTED;
}

#endif /* LZ_POLE_SOURCE_LASER */

/* ------------------------------------------------------------------ */
/* 取圆心                                                              */
/* ------------------------------------------------------------------ */

#ifdef LZ_POLE_SOURCE_FIXED

/* 固定杆位：**2026-09-20 用户提供的场地坐标**。
 *
 * 只在显式 `-DLZ_POLE_SOURCE_FIXED` 时生效，用于"没有操作员、只想验证链路"
 * 的调试场景。运行时默认走操作员记录那条路。 */
#define LZ_FIXED_POLE_LAT 28.1788480
#define LZ_FIXED_POLE_LON 112.9210020
#define LZ_FIXED_POLE_ALT 60.0

const char *LzPole_SourceName(void)
{
    return "固定坐标（调试）";
}

LzStatus LzPole_Acquire(LzTarget *out)
{
    if (out == NULL) {
        return LZ_ERR_PARAM;
    }
    memset(out, 0, sizeof(*out));
    out->id = 1;
    out->geo.latitudeDeg = LZ_FIXED_POLE_LAT;
    out->geo.longitudeDeg = LZ_FIXED_POLE_LON;
    out->geo.altitudeM = LZ_FIXED_POLE_ALT;
    out->heightM = 15.0;
    out->radiusM = 0.1;
    out->confidence = 0.9;

    USER_LOG_INFO("杆位取自固定坐标（调试模式）：%.7f, %.7f",
                  out->geo.latitudeDeg, out->geo.longitudeDeg);
    return LZ_OK;
}

LzStatus LzPole_GetRecorded(LzGeo *out)
{
    (void)out;
    return LZ_ERR_UNSUPPORTED;   /* 调试模式下没有"已记录的点"这个概念 */
}

#else  /* 运行时：操作员记录 */

const char *LzPole_SourceName(void)
{
    if (!s_haveRecord) {
        return "尚未记录";
    }
    return (s_recordKind == LZ_POLE_RECORD_LASER) ? "已记录（激光点）"
                                                  : "已记录（飞机位）";
}


/**
 * @brief 目标高度由来源决定 —— 见 `LzPole_Acquire()` 里那段说明
 *
 * 单独成函数（而不是在 `LzPole_Acquire()` 里写个三元表达式）的理由：
 * 它是"目标有没有高度"这个判据的**唯一落点**，将来视觉那条路接进来时
 * 只改这里。写成表达式的话，下一个改的人会在调用点加分支。
 */
/**
 * @brief 目标高度 = 圆心海拔 − 地面基准（**用时现算**）
 *
 * 单独成函数（而不是在 `LzPole_Acquire()` 里写个表达式）的理由：
 * 它是"目标有没有高度"这个判据的**唯一落点**，将来视觉那条路接进来时
 * 只改这里。写成表达式的话，下一个改的人会在调用点加分支。
 *
 * ## 为什么是"现算"而不是"记录时存死"（2026-10-13 改）
 *
 * 圆心坐标来自激光，它的 `altitudeM` **就是激光点海拔**；地面基准也在
 * 同一坐标系里。两者相减即目标高 —— 不必再存一份。
 *
 * 存死会带来两个问题：
 *   1. **两个真值**：`th=` 与 `alt − ground` 描述同一件事，标定后互相打架
 *   2. **标定后旧记录不会自动修正**：设备上那份 `th=53.75`（错误参考面
 *      算出来的）会一直粘着，操作员必须重记一次才能变对。现算则标定完
 *      立刻生效 —— 这正是本次修复想要的效果。
 *
 * ## ⚠️ 为什么"飞机位"来源也要现算
 *
 * 记录飞机位时圆心是**飞机坐标**，它的 `altitudeM` 是飞机当时的海拔
 * （也是椭球高）。`飞机海拔 − 地面基准` = 飞机离地高度 —— 那**不是**杆高，
 * 但在"用飞机位置当圆心"的用法下，靶子本来就是飞机下方那个点，
 * 语义自洽。⚠️ 而**绕杆飞时应当用「记录激光点」**（圆心才是杆）。
 */
static double lz_pole_target_height(void)
{
    /* ⚠️ **2026-10-13：零点从"起飞点海拔"改成"标定的地面基准"**。
     *
     * 旧实现是 `LzPole_RecordLaser()` 里算好 `th=` 存下来；现场实测证明
     * 那条路把两个不同参考面的量相减（激光=椭球高、起飞点海拔=气压高），
     * 系统性偏约 38 m。完整证据链见 `LzPole_ComputeTargetHeight()` 的说明。
     *
     * 判据仍是 `LzPole_ComputeTargetHeight()`（纯逻辑、零依赖、桌面可测）——
     * 这里只负责**取两个数**（圆心海拔、地面基准）喂给它。
     *
     * 没标定过时 `LzPole_GetGroundAltM()` 返回 NaN，判据给 0 ⇒
     * 按点目标处理。操作员会在浮窗看到"未标定地面"的提示（见
     * `LzPole_Acquire()` 的日志与 `lz_mission.c` 的消息）。 */
    return LzPole_ComputeTargetHeight(s_recorded.altitudeM,
                                      LzPole_GetGroundAltM());
}

LzStatus LzPole_Acquire(LzTarget *out)
{
    if (out == NULL) {
        return LZ_ERR_PARAM;
    }

    if (!s_haveRecord) {
        /* **刻意不回落固定坐标** —— 见文件头注释。
         * 上层会把这个错误码翻成"请先在控件上记录杆位"的浮窗提示。 */
        USER_LOG_WARN("尚未记录杆位，拒绝启动绕飞");
        return LZ_ERR_NOT_READY;
    }

    memset(out, 0, sizeof(*out));
    out->id = 1;
    out->geo = s_recorded;
    out->radiusM = 0.1;
    out->confidence = 0.9;

    /* ⚠️ **`heightM` 由"记录来源"决定，不再一律写死 15 m**（2026-09-28 改）。
     *
     * ## 原先为什么是 15
     *
     * 题目是"绕国旗杆"，而国旗杆常见规格约 15 m，所以写死一个典型值。
     * 在那时的用法下它几乎不影响结果 —— 绕飞只需要"杆在哪儿"，
     * 高度只参与云台俯仰，而俯仰当时写死成 −15°（**本身就已经错得离谱**）。
     *
     * ## 为什么现在必须区分
     *
     * 俯仰改成几何反算之后，`heightM` **直接进入公式**：
     *
     *     俯仰 = -atan2(飞机相对目标底的高度 - heightM, 半径)
     *
     * 那个 `-heightM` 的物理含义是「**瞄目标的顶端**」（2026-10-12 起；
     * 早先是 `-heightM/2` = 瞄中点）——对一根杆是对的。
     * 但现场会用**激光打任意地面点当圆心**（2026-09-28 实测：打的是
     * **足球场中心**）。那种目标没有"高度"，公式里的 `-15 m` 纯粹是错的：
     *
     *     半径 20 m、高度 40 m 时
     *       按 h=15：−51.34°   按 h=0（真实）：−63.43°   **差 12.09°**
     *       20 m 外是 4.28 m，画面里 287 px（画面高度的 27%）
     *
     * ⇒ 判据：**记录的是"一个点"还是"一根杆"**，由来源给：
     *   - 激光点 ⇒ 靶子就是**那个点**，heightM = 0（瞄它自身）
     *   - 飞机位 ⇒ 同样是一个点，heightM = 0
     *
     * ⚠️ **两个来源当前都是"点"**，所以都填 0。写成本函数而不是常量，
     * 是为了将来视觉那条路（识别到**杆**）接进来时**只改这一处**：
     * 那时它会填实测杆高，而"瞄杆顶"的语义自动生效。
     *
     * ⚠️ `heightM = 0` 在 `LzPlan_ComputeGimbalPitchDeg` 里走的是
     * "高度未知按 0 处理"分支 —— 语义正确（瞄目标所在的那层水平面），
     * 与"未知⇒不猜"的既有约定一致。 */
    out->heightM = lz_pole_target_height();

    USER_LOG_INFO("杆位取自%s：%.7f, %.7f（目标高 %.2f m —— %s）",
                  LzPole_SourceName(),
                  out->geo.latitudeDeg, out->geo.longitudeDeg,
                  out->heightM,
                  (out->heightM > 0.0) ? "瞄杆顶" : "点目标，瞄它自身");
    return LZ_OK;
}

LzStatus LzPole_GetRecorded(LzGeo *out)
{
    if (out == NULL) {
        return LZ_ERR_PARAM;
    }
    if (!s_haveRecord) {
        return LZ_ERR_NOT_READY;
    }
    *out = s_recorded;
    return LZ_OK;
}

#endif /* LZ_POLE_SOURCE_FIXED */
