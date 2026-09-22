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
    return LZ_TEST_SUMMARY();
}
