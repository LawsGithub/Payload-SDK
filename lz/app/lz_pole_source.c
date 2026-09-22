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

#include <stdio.h>
#include <string.h>

#ifdef LZ_POLE_SOURCE_LASER
#include <dji_camera_manager.h>
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

/**
 * @brief 把坐标写成一行文本
 *
 * 格式刻意用人类可读的十进制 + 一行注释头，方便现场 `cat` 出来核对：
 *
 *     lon=112.9210020 lat=28.1788480 alt=60.0 src=aircraft
 *
 * 不用二进制：这个文件是给人和给程序共用的，可读性的收益远大于几字节。
 */
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
    const int n = fscanf(fp, "lon=%lf lat=%lf alt=%lf src=%15s",
                         &g.longitudeDeg, &g.latitudeDeg, &g.altitudeM, src);
    fclose(fp);

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
    if (n != 4 || !LzGeo_IsValid(&g) || LzGeo_IsNullSolution(&g)) {
        USER_LOG_ERROR("杆位记录文件格式不对（读到 %d 个字段），当作未记录处理", n);
        s_haveRecord = false;
        memset(&s_recorded, 0, sizeof(s_recorded));
        return LZ_ERR_IO;
    }

    s_recorded = g;
    s_haveRecord = true;
    s_recordKind = (strcmp(src, "laser") == 0) ? LZ_POLE_RECORD_LASER
                                               : LZ_POLE_RECORD_AIRCRAFT;

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
LzStatus LzPole_JudgeLaserReading(double latDeg, double lonDeg, double altM,
                                  double distanceM)
{
    /* 闸门 1：必须有距离。
     * 距离为 0 时坐标会退化成机身自身的位置 —— 不是垃圾值，
     * 是个精确可预测的退化情形（实测模拟器上给出飞机初始位置）。 */
    if (!(distanceM > 0.0)) {
        USER_LOG_WARN("激光测不到距离（distance=%.1f m）—— 请对准目标再按", distanceM);
        return LZ_ERR_NO_TARGET;
    }

    const LzGeo g = { .latitudeDeg = latDeg, .longitudeDeg = lonDeg, .altitudeM = altM };

    /* 闸门 2：坐标必须合法 */
    if (!LzGeo_IsValid(&g)) {
        USER_LOG_WARN("激光给出的坐标非法（%.7f, %.7f，距离 %.1f m）",
                      latDeg, lonDeg, distanceM);
        return LZ_ERR_NO_TARGET;
    }

    /* 闸门 3：坐标不能是零解 —— 见本函数上方关于"三闸独立"的说明 */
    if (LzGeo_IsNullSolution(&g)) {
        USER_LOG_WARN("激光坐标是零解（%.7f, %.7f，距离 %.1f m）—— "
                      "瞄准点要靠飞机自身定位解算，当前飞机没有定位",
                      latDeg, lonDeg, distanceM);
        return LZ_ERR_NO_TARGET;
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

    /* 判定逻辑在 `LzPole_JudgeLaserReading()` 里 —— 它是纯逻辑、零依赖，
     * 所以那条路能在桌面上被测到。这里只负责取数与落盘。
     * 拆分的理由见该函数的注释（一次"空转的测试"逼出来的）。 */
    const double distanceM = info.distance / 10.0;   /* 结构体注释：unit 0.1m */
    const double altM = info.altitude / 10.0;

    const LzStatus st = LzPole_JudgeLaserReading(info.latitude, info.longitude,
                                                altM, distanceM);
    if (st != LZ_OK) {
        return st;
    }

    const LzGeo g = {
        .latitudeDeg = info.latitude,
        .longitudeDeg = info.longitude,
        .altitudeM = altM,
    };
    lz_record_common(&g, LZ_POLE_RECORD_LASER);
    USER_LOG_INFO("★ 已记录激光瞄准点为绕飞圆心：%.7f, %.7f（距离 %.1f m，exception=%u）",
                  g.latitudeDeg, g.longitudeDeg, distanceM, (unsigned)info.exception);
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
    out->heightM = 15.0;   /* 杆高：国旗杆常见规格，绕飞不需要精确值 */
    out->radiusM = 0.1;
    out->confidence = 0.9;

    USER_LOG_INFO("杆位取自%s：%.7f, %.7f", LzPole_SourceName(),
                  out->geo.latitudeDeg, out->geo.longitudeDeg);
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
