/**
 * @file lz_test_pole.c
 * @brief 绕飞圆心的「记录 → 落盘 → 读回」往返测试。
 *
 * ## 为什么这个测试不在 foreach 那个循环里
 *
 * 其余 6 个测试只链接 `lz_core`（纯算法，零依赖）。这个测试要编译
 * **`app/lz_pole_source.c`** —— 它 include 了 `dji_logger.h`，属于 PSDK 侧。
 *
 * 用一个**替身头文件**（`tests/stub/dji_logger.h`）把 `USER_LOG_*` 换成
 * `printf` 就能上桌面。这是刻意的：被测的是"记录 → 写文件 → 读回"这段
 * **与 PSDK 无关**的逻辑，不该为了测它去接飞机。
 *
 * ⚠️ 替身只替换日志宏，**不替换任何被测逻辑** —— `lz_pole_source.c` 是
 * 原封不动编译进来的。如果哪天它把落盘逻辑挪进 PSDK 依赖里，这个测试
 * 会编译失败，那正是提醒"这段逻辑变得只能在设备上验了"。
 *
 * ## 这一组断言守的是四件事
 *
 * 1. **未记录时不回落** —— 这是本功能的核心安全约定。回落到写死的坐标会
 *    让飞机飞到几十公里外，而操作员以为在原地绕圈。
 * 2. **零解被拒** —— 无定位时融合位置给约 `(0,0)`，它在经纬度范围内
 *    "合法"，但毫无意义。`LzGeo_IsValid` 拦不住它，必须显式判。
 * 3. **拒绝不破坏既有记录** —— 一次失败的记录尝试不该把上次的好数据抹掉。
 * 4. **文件坏掉时当作未记录** —— 半个坐标比没有坐标危险：它会看起来
 *    "有记录"，实际是垃圾。
 */

#include "lz_pole_source.h"
#include "lz_types.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "lz_test.h"

/* 落盘路径由 lz_pole_source.c 决定（`data/pole.txt`，相对 CWD）。
 * 测试的 CWD 是构建目录，先确保 `data/` 存在 —— 与 dpk 包里
 * build_dpk.sh 建好 data/ 是同一个前提。 */
#define LZ_TEST_POLE_DIR "data"
#define LZ_TEST_POLE_FILE "data/pole.txt"

/** 直接写一个文件，用来构造"上次记录的"或"被改坏的"输入 */
static void write_file(const char *content)
{
    FILE *fp = fopen(LZ_TEST_POLE_FILE, "w");
    if (fp == NULL) {
        return;
    }
    fputs(content, fp);
    fclose(fp);
}

/** 读回文件首行，用于断言落盘内容 */
static bool read_first_line(char *buf, size_t cap)
{
    FILE *fp = fopen(LZ_TEST_POLE_FILE, "r");
    if (fp == NULL) {
        return false;
    }
    const bool ok = (fgets(buf, (int)cap, fp) != NULL);
    fclose(fp);
    return ok;
}

int main(void)
{
    /* 测试替身里没有 osal，用 libc 建目录即可 */
    (void)mkdir(LZ_TEST_POLE_DIR, 0755);
    remove(LZ_TEST_POLE_FILE);

    LzTarget t;
    LzGeo g;

    LZ_CASE("未记录时：取圆心应被拒，而不是回落到某个坐标");
    {
        /* 这条是本功能的核心安全约定。断言"返回值是 NOT_READY"而不是
         * "返回值非 OK" —— 后者在"回落到固定坐标"时也会通过，
         * 那正是我们要防的行为。 */
        LZ_CHECK(LzPole_LoadRecorded() == LZ_ERR_NOT_READY);
        LZ_CHECK(LzPole_Acquire(&t) == LZ_ERR_NOT_READY);
        LZ_CHECK(LzPole_GetRecorded(&g) == LZ_ERR_NOT_READY);
        LZ_CHECK(strcmp(LzPole_SourceName(), "尚未记录") == 0);
    }

    LZ_CASE("记录飞机位：取圆心应成功，且坐标逐位一致");
    {
        const LzGeo pos = {
            .latitudeDeg = 28.1788480,
            .longitudeDeg = 112.9210020,
            .altitudeM = 61.5,
        };
        LZ_CHECK(LzPole_RecordAircraft(&pos) == LZ_OK);

        LZ_CHECK(LzPole_Acquire(&t) == LZ_OK);
        LZ_CHECK_NEAR(t.geo.latitudeDeg, 28.1788480, 1e-7);
        LZ_CHECK_NEAR(t.geo.longitudeDeg, 112.9210020, 1e-7);
        LZ_CHECK(LzPole_GetRecorded(&g) == LZ_OK);
        LZ_CHECK_NEAR(g.altitudeM, 61.5, 1e-6);
    }

    LZ_CASE("⚠️ 目标高度由来源决定：点目标必须是 0，不是写死的 15");
    {
        /* ---- 这条守的是一个**实测到的 5° 偏差**（2026-09-28）----
         *
         * 现场用激光打**足球场中心**当圆心（半径 20 m、高度 40 m），
         * 而 `heightM` 写死 15 ⇒ 俯仰按 `-atan2(40 - 7.5, 20)` = -58.39°，
         * 正确值是 `-atan2(40 - 0, 20)` = **-63.43°**，**差 5.04°**。
         * 20 m 外是 1.76 m，画面里 118 px（画面高度的 11%）。
         *
         * `heightM` 里那个 `-h/2` 的物理含义是「瞄**目标的中点**」——
         * 对一根杆对，对**一个地面点**纯粹是错的。
         *
         * ⇒ 记录来源是"点"，高度就必须是 0。
         * ⚠️ 断言的是**具体值 0** 而不是"小于 15" —— 后者在填 7 时也会过，
         * 而 7 同样错得没道理。 */
        const LzGeo pos = {
            .latitudeDeg = 28.1788480,
            .longitudeDeg = 112.9210020,
            .altitudeM = 61.5,
        };
        LZ_CHECK(LzPole_RecordAircraft(&pos) == LZ_OK);
        LZ_CHECK(LzPole_Acquire(&t) == LZ_OK);
        LZ_CHECK_NEAR(t.heightM, 0.0, 1e-12);

        /* 换成激光来源，同样必须是 0 —— 两个来源记录的**都是一个点**，
         * 语义相同。若将来视觉（识别到**杆**）接进来，这里会多一条
         * "填实测杆高"的分支，而那时本条应当跟着改。 */
        /* ⚠️ 这里曾有一段 `#ifdef LZ_POLE_SOURCE_LASER` 包着的
         * `LzPole_RecordLaser(&pos)` —— **它是死代码，从未被编译过**
         * （2026-10-06 发现）：`LzPole_RecordLaser` 的签名是 `(void)`，
         * 传 `&pos` 会编译失败。之所以一直没暴露，是因为桌面构建
         * **从不定义** `LZ_POLE_SOURCE_LASER`，那个块整个被预处理掉了。
         *
         * ⇒ 已删除。激光那条路现在由下面的
         * 「原始读数 → 该记什么」一组用例覆盖（走 `LzPole_PrepareLaserRecord()`，
         * 那个函数**在 ifdef 之外**，桌面编得到）。 */
    }

    LZ_CASE("落盘：文件应可被外部工具直接 cat 出来核对");
    {
        /* 格式刻意是人类可读的一行 —— 现场排查时 `cat data/pole.txt`
         * 就能确认记的是哪个点，不必写解包工具。 */
        char line[160] = {0};
        LZ_CHECK(read_first_line(line, sizeof(line)));
        LZ_CHECK(strstr(line, "lon=112.9210020") != NULL);
        LZ_CHECK(strstr(line, "lat=28.1788480") != NULL);
        LZ_CHECK(strstr(line, "src=aircraft") != NULL);
    }

    LZ_CASE("零解**邻域**必须被拒 —— 不只是精确的 (0,0)");
    {
        /* 这条是设备实测逼出来的：2026-09-22 在 M4T 室内无 GPS 时，
         * 融合位置给的是 `lon=0.0000004, lat=0.0000003`，**不是** (0,0)。
         * 早先写 `== 0.0` 的判据放行了它，于是"记录圆心"成功返回了一个
         * 几内亚湾附近的坐标 —— 界面显示成功，圆画在半个地球之外。 */
        const LzGeo residual = {
            .latitudeDeg = 0.0000003,
            .longitudeDeg = 0.0000004,
            .altitudeM = 60.8,
        };
        LZ_CHECK(LzPole_RecordAircraft(&residual) == LZ_ERR_NO_TARGET);

        /* 负的残差也要拦（fabs 而不是与 0 比） */
        const LzGeo negResidual = {
            .latitudeDeg = -0.0000007,
            .longitudeDeg = -0.0000002,
            .altitudeM = 60.0,
        };
        LZ_CHECK(LzPole_RecordAircraft(&negResidual) == LZ_ERR_NO_TARGET);

        /* 边界附近：阈值内拒、阈值外放行。用阈值本身做判据，
         * 而不是硬编码数字 —— 阈值改了这条测试仍然自洽。 */
        LzGeo edgeIn = { .latitudeDeg = 0.0, .longitudeDeg = LZ_GEO_NULL_SOLUTION_DEG * 0.9, .altitudeM = 0.0 };
        LZ_CHECK(LzPole_RecordAircraft(&edgeIn) == LZ_ERR_NO_TARGET);
    }

    LZ_CASE("精确 (0,0) 也必须被拒，且不破坏既有记录");
    {
        /* 先确保有一条好记录，再试零解 —— 断言"失败不影响已有记录" */
        const LzGeo good = { .latitudeDeg = 30.5, .longitudeDeg = 114.3, .altitudeM = 30.0 };
        LZ_CHECK(LzPole_RecordAircraft(&good) == LZ_OK);

        const LzGeo zero = { .latitudeDeg = 0.0, .longitudeDeg = 0.0, .altitudeM = 0.0 };
        LZ_CHECK(LzPole_RecordAircraft(&zero) == LZ_ERR_NO_TARGET);

        /* 关键：失败不能把上一次的好记录抹掉 ——
         * 否则操作员在室内误按一次，到室外就发现"记录没了"。 */
        LZ_CHECK(LzPole_GetRecorded(&g) == LZ_OK);
        LZ_CHECK_NEAR(g.latitudeDeg, 30.5, 1e-7);
        LZ_CHECK_NEAR(g.longitudeDeg, 114.3, 1e-7);
    }

    LZ_CASE("非法坐标必须被拒");
    {
        const LzGeo bad = { .latitudeDeg = 999.0, .longitudeDeg = 0.0, .altitudeM = 0.0 };
        LZ_CHECK(LzPole_RecordAircraft(&bad) == LZ_ERR_NO_TARGET);

        const LzGeo nan = { .latitudeDeg = 0.0 / 0.0, .longitudeDeg = 112.0, .altitudeM = 0.0 };
        LZ_CHECK(LzPole_RecordAircraft(&nan) == LZ_ERR_NO_TARGET);

        LZ_CHECK(LzPole_RecordAircraft(NULL) == LZ_ERR_PARAM);
    }

    LZ_CASE("重启后能读回：模拟进程重启，从文件恢复");
    {
        /* 写一份"上次记录的"文件，然后当作新进程读它。
         * 这里不能复用上面的内存状态 —— 必须先清掉才有资格说"这是从盘上来的"。 */
        remove(LZ_TEST_POLE_FILE);
        write_file("lon=112.9500000 lat=28.1800000 alt=55.0 src=laser\n");

        LZ_CHECK(LzPole_LoadRecorded() == LZ_OK);
        LZ_CHECK(LzPole_Acquire(&t) == LZ_OK);
        LZ_CHECK_NEAR(t.geo.latitudeDeg, 28.1800000, 1e-7);
        LZ_CHECK_NEAR(t.geo.longitudeDeg, 112.9500000, 1e-7);

        /* 来源标记要跟着回来 —— 浮窗靠它告诉操作员"现在用的是激光点还是飞机位" */
        LZ_CHECK(strcmp(LzPole_SourceName(), "已记录（激光点）") == 0);
    }

    LZ_CASE("飞机位来源的标记也要能读回");
    {
        remove(LZ_TEST_POLE_FILE);
        write_file("lon=112.9210020 lat=28.1788480 alt=60.0 src=aircraft\n");
        LZ_CHECK(LzPole_LoadRecorded() == LZ_OK);
        LZ_CHECK(strcmp(LzPole_SourceName(), "已记录（飞机位）") == 0);
    }

    LZ_CASE("文件被改坏：当作未记录，不给出半个坐标");
    {
        /* 半个坐标比没有坐标更危险：它会看起来"有记录"，实际是垃圾，
         * 飞机照它飞过去。所以必须降级成"未记录"。 */
        remove(LZ_TEST_POLE_FILE);
        write_file("lon=112.95\n");          /* 截断 */
        LZ_CHECK(LzPole_LoadRecorded() != LZ_OK);
        LZ_CHECK(LzPole_Acquire(&t) == LZ_ERR_NOT_READY);

        remove(LZ_TEST_POLE_FILE);
        write_file("lon=abc lat=xyz alt=0 src=aircraft\n");   /* 字段非法 */
        LZ_CHECK(LzPole_LoadRecorded() != LZ_OK);
        LZ_CHECK(LzPole_Acquire(&t) == LZ_ERR_NOT_READY);

        remove(LZ_TEST_POLE_FILE);
        write_file("lon=999.0 lat=999.0 alt=0.0 src=aircraft\n");  /* 越界但可解析 */
        LZ_CHECK(LzPole_LoadRecorded() != LZ_OK);
        LZ_CHECK(LzPole_Acquire(&t) == LZ_ERR_NOT_READY);

        /* 盘上存着零解也要拒 —— 旧版本可能写进去过，或文件被手改 */
        remove(LZ_TEST_POLE_FILE);
        write_file("lon=0.0000004 lat=0.0000003 alt=60.8 src=aircraft\n");
        LZ_CHECK(LzPole_LoadRecorded() != LZ_OK);
        LZ_CHECK(LzPole_Acquire(&t) == LZ_ERR_NOT_READY);
    }

    LZ_CASE("空文件也要当作未记录");
    {
        remove(LZ_TEST_POLE_FILE);
        write_file("");
        LZ_CHECK(LzPole_LoadRecorded() != LZ_OK);
        LZ_CHECK(LzPole_Acquire(&t) == LZ_ERR_NOT_READY);
    }

    LZ_CASE("激光判定闸门 ①：必须有距离");
    {
        /* 距离为 0 时坐标会退化成机身位置 —— 必须拒 */
        LZ_CHECK(LzPole_JudgeLaserReading(28.1788480, 112.9210020, 60.0, 0.0) == LZ_ERR_NO_TARGET);
        /* 负距离是不可能的输入，也要拒 */
        LZ_CHECK(LzPole_JudgeLaserReading(28.1788480, 112.9210020, 60.0, -1.0) == LZ_ERR_NO_TARGET);

        /* 有距离 + 真实坐标 → 通过 */
        LZ_CHECK(LzPole_JudgeLaserReading(28.1788480, 112.9210020, 60.0, 14.2) == LZ_OK);
    }

    LZ_CASE("激光判定闸门 ②：坐标必须合法");
    {
        LZ_CHECK(LzPole_JudgeLaserReading(999.0, 112.0, 60.0, 5.0) == LZ_ERR_NO_TARGET);
        LZ_CHECK(LzPole_JudgeLaserReading(28.0, 999.0, 60.0, 5.0) == LZ_ERR_NO_TARGET);
        LZ_CHECK(LzPole_JudgeLaserReading(0.0 / 0.0, 112.0, 60.0, 5.0) == LZ_ERR_NO_TARGET);
    }

    LZ_CASE("激光判定闸门 ③：坐标不能是零解（**即使距离有效**）");
    {
        /* 这条是设备实测逼出来的核心用例（2026-09-22，M4T 室内无 GPS）：
         * `distance=2.0m` **有效**，而 `lat/lon = 0.0, 0.0`。
         *
         * 激光的经纬度 = 机身位置 + 云台朝向 + 距离 **解算**出来的 ——
         * 飞机自身没定位时，机身位置是零解，解算结果自然是零解。
         * **"有距离"推不出"坐标有效"**，所以闸门 ① 过了也要过 ③。 */
        LZ_CHECK(LzPole_JudgeLaserReading(0.0, 0.0, 2.0, 2.0) == LZ_ERR_NO_TARGET);

        /* 残差（不是精确 0）也要拦 —— 与融合位置那个坑同形 */
        LZ_CHECK(LzPole_JudgeLaserReading(0.0000001, -0.0000002, 2.0, 2.0) == LZ_ERR_NO_TARGET);

        /* 阈值内拒 */
        LZ_CHECK(LzPole_JudgeLaserReading(0.4, 0.4, 2.0, 2.0) == LZ_ERR_NO_TARGET);

        /* 越过阈值就放行 —— 否则激光永远记不下来 */
        LZ_CHECK(LzPole_JudgeLaserReading(0.6, 112.0, 2.0, 2.0) == LZ_OK);
    }

    LZ_CASE("激光记录在未启用编译时应报 UNSUPPORTED，而不是含糊的失败");
    {
        /* ⚠️ 这条测的是**本测试目标**的编译配置，不是主应用的。
         *
         * `lz_test_pole` 直接编 `app/lz_pole_source.c` 而**没有**传
         * `-DLZ_POLE_SOURCE_LASER`（它只链接 lz_types，没有 PSDK 的
         * camera_manager 头文件），所以走 `#else` 分支。
         * 主应用那边 `LZ_POLE_SOURCE_LASER` 默认是 **ON**（见 CMakeLists）。
         *
         * 测这条的价值：断言"没编进去时给出的是**明确的 UNSUPPORTED**，
         * 不是一个看不出所以然的 IO 错误" —— 操作员按了按钮要知道
         * 该怎么办。 */
        LZ_CHECK(LzPole_RecordLaser() == LZ_ERR_UNSUPPORTED);
    }

    remove(LZ_TEST_POLE_FILE);

    LZ_CASE("目标高度：由「激光海拔 − 起飞点海拔」算，三个出口都不猜");
    {
        /* ★ 这条守的是**问题 3 的修法本身**。
         *
         * 用户 2026-09-28 指出：把激光打在**旗面**上时，靶子不是"地面上的
         * 一个点"，而是离地 h 米的一个点 —— 俯仰公式里的 `-h/2`
         * 必须真的用上。而 h 可以从两个高程之差得到（比"距离 × 俯仰角"
         * 少两个误差源）。
         *
         * ⚠️ 本函数**必须能在桌面上被测到**。放在 `#ifdef LZ_POLE_SOURCE_LASER`
         * 里面的话，桌面构建根本不编它 —— 改了判定逻辑测试照样绿。
         * 本项目已经在完全相同的地方踩过一次（激光零解闸门）。 */

        /* 1. 正常：打旗面。10 m 高的旗面，起飞点在 0 m 那层 */
        LZ_CHECK_NEAR(LzPole_ComputeTargetHeight(50.0, 40.0), 10.0, 1e-9);
        LZ_CHECK_NEAR(LzPole_ComputeTargetHeight(40.02, 40.0), 0.02, 1e-9);

        /* 2. 打地面：差 ≈ 0 或略负 ⇒ 0（**合法用法**，不是失败） */
        LZ_CHECK_NEAR(LzPole_ComputeTargetHeight(40.0, 40.0), 0.0, 1e-9);
        LZ_CHECK_NEAR(LzPole_ComputeTargetHeight(39.7, 40.0), 0.0, 1e-9);
        LZ_CHECK_NEAR(LzPole_ComputeTargetHeight(0.0, 100.0), 0.0, 1e-9);

        /* 3. 超上限 ⇒ 0（参考面不一致 / 打到远处）。
         *    ⚠️ **不是钳位到上限** —— 钳位会把明显错的量伪装成
         *    "一个很高的目标"。取 0 至少是已知合法的那种用法。 */
        LZ_CHECK_NEAR(LzPole_ComputeTargetHeight(40.0 + 1000.0, 40.0), 0.0, 1e-9);
        LZ_CHECK_NEAR(LzPole_ComputeTargetHeight(40.0 + LZ_POLE_TARGET_HEIGHT_MAX_M,
                                                 40.0), 0.0, 1e-9);   /* 边界：开区间下端 */
        LZ_CHECK(LzPole_ComputeTargetHeight(
                     40.0 + LZ_POLE_TARGET_HEIGHT_MAX_M - 0.01, 40.0) > 0.0);

        /* 4. 拿不到起飞点海拔 ⇒ 0，**不猜一个典型杆高** */
        LZ_CHECK_NEAR(LzPole_ComputeTargetHeight(50.0, 0.0 / 0.0), 0.0, 1e-9);
        LZ_CHECK_NEAR(LzPole_ComputeTargetHeight(0.0 / 0.0, 40.0), 0.0, 1e-9);
        LZ_CHECK_NEAR(LzPole_ComputeTargetHeight(0.0 / 0.0, 0.0 / 0.0), 0.0, 1e-9);
        LZ_CHECK_NEAR(LzPole_ComputeTargetHeight(1.0 / 0.0, 40.0), 0.0, 1e-9);
    }

    LZ_CASE("目标高度要能落盘并读回（th= 字段；旧文件缺它时按 0）");
    {
        /* ⚠️ **本条只覆盖"读"这一侧** —— 写那一侧由下面
         * 「原始读数 → 该记什么」一组用例覆盖（走
         * `LzPole_PrepareLaserRecord()`，它在 `#ifdef` 之外）。
         *
         * 本条 2026-10-06 之前写的是"写侧在桌面上测不到，只能靠上机看日志"。
         * 那句话**当时是对的**（判据全在 `#ifdef` 里），现在**不再成立** ——
         * 单位换算、三道闸、目标高分类都已抽出来。
         *
         * 仍然留在设备上验的只剩：SDK 的取数本身（`GetLaserRangingInfo`
         * 返回什么）与两个海拔话题就绪与否。那是观测，不是判据。 */
        /* ⚠️ `th=` 是**后加的**字段。旧文件（现场设备上那份）没有它 ——
         * 读旧文件必须**当作目标高 0**，而不是判成坏文件。
         * 判错会让操作员莫名其妙地丢掉一条本来可用的记录。 */
        remove(LZ_TEST_POLE_FILE);
        write_file("lon=112.9500000 lat=28.1800000 alt=55.0 th=9.75 src=laser\n");
        LZ_CHECK(LzPole_LoadRecorded() == LZ_OK);
        LZ_CHECK(LzPole_Acquire(&t) == LZ_OK);
        LZ_CHECK_NEAR(t.heightM, 9.75, 1e-9);

        /* 旧格式（4 字段，无 th=）⇒ 目标高 0，仍要能读回 */
        remove(LZ_TEST_POLE_FILE);
        write_file("lon=112.9500000 lat=28.1800000 alt=55.0 src=laser\n");
        LZ_CHECK(LzPole_LoadRecorded() == LZ_OK);
        LZ_CHECK(LzPole_Acquire(&t) == LZ_OK);
        LZ_CHECK_NEAR(t.heightM, 0.0, 1e-9);
        LZ_CHECK_NEAR(t.geo.latitudeDeg, 28.1800000, 1e-7);

        /* th=0（打地面）与"没有 th="等价 —— 都是点目标 */
        remove(LZ_TEST_POLE_FILE);
        write_file("lon=112.9500000 lat=28.1800000 alt=55.0 th=0.00 src=laser\n");
        LZ_CHECK(LzPole_LoadRecorded() == LZ_OK);
        LZ_CHECK(LzPole_Acquire(&t) == LZ_OK);
        LZ_CHECK_NEAR(t.heightM, 0.0, 1e-9);
    }

    /* ================================================================
     * 「原始读数 → 该记什么」—— 2026-10-06 新增
     *
     * 这一段此前**一行都测不到**：单位换算、三道闸、目标高分类全在
     * `#ifdef LZ_POLE_SOURCE_LASER` 里，桌面构建不编它。实测确认把
     * `distanceDm / 10.0` 写成 `/ 100.0`，**没有任何测试变红** ——
     * 而表现是"距离小十倍"，看起来仍像个合理的读数。
     *
     * ⇒ 判据抽到 `LzPole_PrepareLaserRecord()`（在 `#ifdef` 之外），
     * 本组用例直接打它。
     * ================================================================ */

    LZ_CASE("单位换算：SDK 的 altitude / distance 是 0.1 m，必须除 10 而不是 100");
    {
        /* ⚠️ **这条用例的存在本身就是修法**：它守的是"换算发生在
         * 桌面上能测到的地方"。
         *
         * 构造：SDK 给 distance=44（= 4.4 m，2026-09-22 实测过的真实读数）、
         * altitude=504（= 50.4 m，现场 pole.txt 里那个值）。
         * 断言**具体米数**而不是"大于 0" —— 后者在除错十倍时也通过。 */
        const LzLaserRawReading raw = {
            .latitudeDeg = 28.1788176, .longitudeDeg = 112.9210889,
            .altitudeDm = 504.0,        /* 50.4 m */
            .distanceDm = 44.0,         /* 4.4 m */
        };
        LzPoleLaserRecord rec;
        LZ_CHECK(LzPole_PrepareLaserRecord(&raw, 40.4, &rec) == LZ_OK);
        LZ_CHECK_NEAR(rec.distanceM, 4.4, 1e-9);
        LZ_CHECK_NEAR(rec.laserAltM, 50.4, 1e-9);
        LZ_CHECK_NEAR(rec.geo.altitudeM, 50.4, 1e-9);   /* geo 里也必须是米 */

        /* 经纬度**不换算** —— 它们是度，SDK 直出。
         * 这是踩过的坑（头文件注释专门写了"不是 rad"），所以显式断言。 */
        LZ_CHECK_NEAR(rec.geo.latitudeDeg, 28.1788176, 1e-12);
        LZ_CHECK_NEAR(rec.geo.longitudeDeg, 112.9210889, 1e-12);
    }

    LZ_CASE("三道闸的**分类**：三种成因必须分得开，不是一个 NO_TARGET");
    {
        /* 为什么这条重要：三种成因的处置完全不同 ——
         *   无距离   ⇒ 重新瞄准
         *   坐标越界 ⇒ 报 bug（飞机给了非法值）
         *   零解     ⇒ 等定位，**别调瞄准**
         * 合并成一个错误码、只靠日志文案区分的话，"分得开"这件事
         * 本身就没有断言守着 —— 文案改一个字就静默退化了。
         * 本项目在 `LzVisionMiss` 上踩过同一个形状。 */
        LzPoleLaserRecord rec;
        const LzLaserRawReading base = {
            .latitudeDeg = 28.1788176, .longitudeDeg = 112.9210889,
            .altitudeDm = 504.0, .distanceDm = 44.0,
        };

        /* ① 无回波：distance == 0 */
        LzLaserRawReading r = base;
        r.distanceDm = 0.0;
        LZ_CHECK(LzPole_PrepareLaserRecord(&r, 40.0, &rec) == LZ_ERR_NO_TARGET);
        LZ_CHECK(rec.miss == LZ_LASER_MISS_NO_DISTANCE);

        /* ② 坐标越界 */
        r = base;
        r.latitudeDeg = 95.0;      /* 纬度合法域 ±90 */
        LZ_CHECK(LzPole_PrepareLaserRecord(&r, 40.0, &rec) == LZ_ERR_NO_TARGET);
        LZ_CHECK(rec.miss == LZ_LASER_MISS_BAD_COORD);

        /* ③ 零解邻域：飞机没定位时瞄准点解算退化。
         *    ⚠️ 用**带残差**的值，不是精确 (0,0) —— 实测给的是
         *    `lon=0.0000004, lat=0.0000003`，写 `== 0.0` 的判据会放行它。 */
        r = base;
        r.latitudeDeg = 0.0000003;
        r.longitudeDeg = 0.0000004;
        LZ_CHECK(LzPole_PrepareLaserRecord(&r, 40.0, &rec) == LZ_ERR_NO_TARGET);
        LZ_CHECK(rec.miss == LZ_LASER_MISS_NULL_SOLUTION);

        /* ④ 可用的那一条：miss 必须是 NONE（否则"分得开"没有对照） */
        LZ_CHECK(LzPole_PrepareLaserRecord(&base, 40.0, &rec) == LZ_OK);
        LZ_CHECK(rec.miss == LZ_LASER_MISS_NONE);

        /* ⑤ **顺序**也要对：距离为 0 且坐标也非法时，报的是"无距离"。
         *    这不是吹毛求疵 —— 距离为 0 时坐标必然退化成机身位置，
         *    先报"坐标非法"会把操作员引去查飞机定位，而实际该做的是瞄准。 */
        r = base;
        r.distanceDm = 0.0;
        r.latitudeDeg = 95.0;
        (void)LzPole_PrepareLaserRecord(&r, 40.0, &rec);
        LZ_CHECK(rec.miss == LZ_LASER_MISS_NO_DISTANCE);
    }

    LZ_CASE("目标高的四个出口必须分得开 —— 「打地面」是合法用法，不是失败");
    {
        /* 处置不同：
         *   打地面   ⇒ INFO，**正常**（点目标，瞄它自身）
         *   超上限   ⇒ WARN，请核对参考面 / 激光是否打到远处地面
         *   无起飞点 ⇒ WARN，按 0 处理
         * 全合并成"算出来是 0"就分不开了。 */
        LzPoleLaserRecord rec;
        const LzLaserRawReading raw = {
            .latitudeDeg = 28.1788176, .longitudeDeg = 112.9210889,
            .altitudeDm = 504.0,       /* 50.4 m */
            .distanceDm = 44.0,
        };

        /* 正常：旗面在 10 m 高（起飞点 40.4 m） */
        LZ_CHECK(LzPole_PrepareLaserRecord(&raw, 40.4, &rec) == LZ_OK);
        LZ_CHECK(rec.heightDiag == LZ_LASER_TH_NORMAL);
        LZ_CHECK_NEAR(rec.targetHeightM, 10.0, 1e-9);

        /* 打地面：差 ≤ 0 ⇒ 0，且分类是 GROUND（**不是** OVER_MAX） */
        LZ_CHECK(LzPole_PrepareLaserRecord(&raw, 50.4, &rec) == LZ_OK);
        LZ_CHECK(rec.heightDiag == LZ_LASER_TH_GROUND);
        LZ_CHECK_NEAR(rec.targetHeightM, 0.0, 1e-9);

        /* 超上限：差 100 m > LZ_POLE_TARGET_HEIGHT_MAX_M ⇒ 0，分类是 OVER_MAX。
         * ⚠️ 这条与上一条**都返回 0** —— 若只断言返回值，两条一模一样，
         * 用例就退化成测了个寂寞。断言的是 `heightDiag`。 */
        LZ_CHECK(LzPole_PrepareLaserRecord(&raw, -49.6, &rec) == LZ_OK);
        LZ_CHECK(rec.heightDiag == LZ_LASER_TH_OVER_MAX);
        LZ_CHECK_NEAR(rec.targetHeightM, 0.0, 1e-9);

        /* 拿不到起飞点海拔 ⇒ 分类是 NO_HOME（与"打地面"同样返回 0） */
        LZ_CHECK(LzPole_PrepareLaserRecord(&raw, 0.0 / 0.0, &rec) == LZ_OK);
        LZ_CHECK(rec.heightDiag == LZ_LASER_TH_NO_HOME);
        LZ_CHECK_NEAR(rec.targetHeightM, 0.0, 1e-9);

        /* 边界：正好等于上限 ⇒ 0（开区间下端）；差一点点 ⇒ 正常。
         * 这一对来自 `LzPole_ComputeTargetHeight` 的规格，这里复核分类没走偏。 */
        LZ_CHECK(LzPole_PrepareLaserRecord(&raw, 50.4 - LZ_POLE_TARGET_HEIGHT_MAX_M,
                                           &rec) == LZ_OK);
        LZ_CHECK(rec.heightDiag == LZ_LASER_TH_OVER_MAX);
        LZ_CHECK(LzPole_PrepareLaserRecord(&raw, 50.4 - LZ_POLE_TARGET_HEIGHT_MAX_M + 0.01,
                                           &rec) == LZ_OK);
        LZ_CHECK(rec.heightDiag == LZ_LASER_TH_NORMAL);
    }

    LZ_CASE("PrepareLaserRecord 的入参校验：NULL 必须被拒，不写半个结果");
    {
        LzPoleLaserRecord rec;
        const LzLaserRawReading raw = {
            .latitudeDeg = 28.0, .longitudeDeg = 112.0,
            .altitudeDm = 500.0, .distanceDm = 40.0,
        };
        LZ_CHECK(LzPole_PrepareLaserRecord(NULL, 40.0, &rec) == LZ_ERR_PARAM);
        LZ_CHECK(LzPole_PrepareLaserRecord(&raw, 40.0, NULL) == LZ_ERR_PARAM);

        /* `Acquire` / `GetRecorded` 的 NULL 校验也要有断言 ——
         * 它们此前从未被覆盖过（gcov 实测）。 */
        LZ_CHECK(LzPole_Acquire(NULL) == LZ_ERR_PARAM);
        LZ_CHECK(LzPole_GetRecorded(NULL) == LZ_ERR_PARAM);
    }

    LZ_CASE("读数不可用时 heightDiag 必须是 NA —— 不能读成「算过了，正常」");
    {
        /* ⚠️ 这条是被**实测**逼出来的：`altitudeDm = NaN` 时，
         * `LzGeo_IsValid()` 的**有限性检查**在更前面就把它拦成
         * `LZ_LASER_MISS_BAD_COORD`，函数早退。
         *
         * 于是问题变成：早退时 `heightDiag` 是什么？原先枚举里 0 是 NORMAL，
         * 而 `memset` 之后它就是 0 —— **读起来像"目标高算过了，正常"**。
         * 那是一句假话：根本没算。
         *
         * ⇒ 枚举改成 `LZ_LASER_TH_NA = 0`，并在早退路径上显式置它。
         * 与"`LzVisionMiss` 要分得开"同一条纪律：**返回值不许含糊。** */
        LzPoleLaserRecord rec;
        const LzLaserRawReading raw = {
            .latitudeDeg = 28.1788176, .longitudeDeg = 112.9210889,
            .altitudeDm = 0.0 / 0.0,   /* NaN ⇒ 被有限性检查拦下 */
            .distanceDm = 44.0,
        };
        LZ_CHECK(LzPole_PrepareLaserRecord(&raw, 40.4, &rec) == LZ_ERR_NO_TARGET);
        LZ_CHECK(rec.miss == LZ_LASER_MISS_BAD_COORD);
        LZ_CHECK(rec.heightDiag == LZ_LASER_TH_NA);
        LZ_CHECK(rec.heightDiag != LZ_LASER_TH_NORMAL);
    }

    LZ_CASE("落盘失败时记录仍留在内存 —— 一次磁盘问题不该毁掉当下的绕飞");
    {
        /* 这条守的是 `lz_record_common()` 的**降级策略**：
         * 落盘失败 → WARN，但内存里的记录**保留**（本次绕飞照常能用），
         * 而不是让整个记录失败。
         *
         * 构造：把 `data/` 目录**去掉**（`remove` 对目录无效，用改名），
         * 于是 fopen("data/pole.txt","w") 失败。 */
        remove(LZ_TEST_POLE_FILE);
        LZ_CHECK(rename(LZ_TEST_POLE_DIR, LZ_TEST_POLE_DIR ".hidden") == 0);

        const LzGeo pos = {
            .latitudeDeg = 28.1888000, .longitudeDeg = 112.9333000, .altitudeM = 62.0,
        };
        LZ_CHECK(LzPole_RecordAircraft(&pos) == LZ_OK);   /* 记录本身成功 */
        LzGeo back;
        LZ_CHECK(LzPole_GetRecorded(&back) == LZ_OK);     /* 内存里在 */
        LZ_CHECK_NEAR(back.latitudeDeg, 28.1888000, 1e-7);

        /* 恢复目录，否则后面所有用例都写不了盘 */
        LZ_CHECK(rename(LZ_TEST_POLE_DIR ".hidden", LZ_TEST_POLE_DIR) == 0);
    }

    LZ_CASE("读文件：空文件 / 只有表头之外的垃圾，都当作未记录");
    {
        /* `fgets` 返回 NULL 那一条（空文件）与"字段数不对"是两个不同的出口，
         * gcov 实测前者此前从未被走到过。 */
        remove(LZ_TEST_POLE_FILE);
        write_file("");                       /* 空文件 ⇒ fgets 返回 NULL */
        LZ_CHECK(LzPole_LoadRecorded() != LZ_OK);
        LzTarget t2;
        LZ_CHECK(LzPole_Acquire(&t2) == LZ_ERR_NOT_READY);

        write_file("garbage without any fields\n");
        LZ_CHECK(LzPole_LoadRecorded() != LZ_OK);
        LZ_CHECK(LzPole_Acquire(&t2) == LZ_ERR_NOT_READY);
    }

    return LZ_TEST_SUMMARY();
}
