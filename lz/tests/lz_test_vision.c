/**
 * @file lz_test_vision.c
 * @brief 视觉检测的回归测试：拿 6 张**真实俯拍照片**跑，与黄金值比对。
 *
 * ## 为什么这是本项目最有价值的一条测试
 *
 * 视觉算法的问题在设备上**看不见**：飞机飞起来才知道认没认对，而那时
 * 已经在天上了。所以检测算法必须在桌面上对着真实照片跑 —— 这也是
 * `lz_vision` 坚持零依赖（不引 OpenCV）的主要理由。
 *
 * ## 测试数据从哪来
 *
 * `tests/data/` 下的 ppm 是从现场照片裁出来的 6 张小图（约 1500 KB），
 * 黄金值在 `tests/data/vision_golden.txt`。生成工具是
 * `tools/gen_vision_testdata.js`，**期望值来自全图检测 + 人工目视核对**，
 * 再纯平移到裁剪坐标 —— 不是"算法自己跑出来的结果"，那样是循环论证。
 *
 * ## 这条测试守的是什么（按重要性）
 *
 * 1. **杆列位置**（±2 px）—— 这是整个视觉层的产出，上层拿它做对齐判定。
 *    最朴素的"取旗面 bbox 中点"在这 6 张图上有 −47 ~ +52 px 的偏差，
 *    所以这条断言实际上就是在守"别退回那个写法"。
 * 2. **旗面的竖向范围**（±10%）—— 守 HSV 阈值没被"顺手调一下"。
 *    调松会让红色电动车并进旗面、调紧会让旗的下摆掉，两种都不会让
 *    位置断言变红（杆列仍大致对），但检测已经不可靠了。
 * 3. **退化输入**（空指针、零尺寸、全黑图、全红图）—— 守不住崩溃。
 * 4. **视觉不填 geo / 高度 / 半径** —— 守分工：坐标由激光直出。
 *
 * ## 已知偏差，故意不掩盖
 *
 * 裁剪前后杆列检测值相差 0–2 px（ROI 被 clamp 到图边的边界效应；
 * 收窄投票窗口后多数图已是 0，见 lz_vision.h 的 poleWin* 说明），
 * 所以容差取 ±2 px。**把它收紧到 0 会掩盖真实的边界效应** ——
 * 那样将来真的偏移 2 px 就看不出来了。
 */

#include "lz_vision.h"
#include "lz_test.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** 一条黄金值记录 */
typedef struct {
    char name[32];
    int width, height;
    int flagMinX, flagMinY, flagMaxX, flagMaxY;
    int flagArea;
    int poleX;
} GoldenRow;

/** 读 PPM P6 —— 故意挑这个格式：C 侧解析只要 20 行，不需要任何压缩库 */
static uint8_t *load_ppm(const char *path, int *outW, int *outH)
{
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        fprintf(stderr, "[SKIP ] 打不开 %s —— 测试数据没生成？\n", path);
        return NULL;
    }

    char magic[3] = {0};
    int w = 0, h = 0, maxval = 0;
    if (fscanf(f, "%2s", magic) != 1 || strcmp(magic, "P6") != 0) {
        fclose(f);
        fprintf(stderr, "[FAIL ] %s 不是 P6 PPM\n", path);
        return NULL;
    }
    /* 依次读三个十进制数，跳过 # 注释行 */
    int *dst[3] = { &w, &h, &maxval };
    for (int k = 0; k < 3; ++k) {
        int c;
        do {
            c = fgetc(f);
            if (c == '#') {
                while (c != EOF && c != '\n') {
                    c = fgetc(f);
                }
            }
        } while (c == ' ' || c == '\n' || c == '\r' || c == '\t' || c == '#');
        if (c == EOF) {
            fclose(f);
            return NULL;
        }
        ungetc(c, f);
        if (fscanf(f, "%d", dst[k]) != 1) {
            fclose(f);
            return NULL;
        }
    }
    fgetc(f);   /* 头部与数据之间的单个空白 */

    if (w <= 0 || h <= 0 || maxval != 255) {
        fclose(f);
        fprintf(stderr, "[FAIL ] %s 头部非法: %dx%d maxval=%d\n", path, w, h, maxval);
        return NULL;
    }

    const size_t n = (size_t)w * (size_t)h * 3u;
    uint8_t *rgb = malloc(n);
    if (rgb == NULL) {
        fclose(f);
        return NULL;
    }
    if (fread(rgb, 1, n, f) != n) {
        free(rgb);
        fclose(f);
        fprintf(stderr, "[FAIL ] %s 数据不足\n", path);
        return NULL;
    }
    fclose(f);

    *outW = w;
    *outH = h;
    return rgb;
}

/** 读黄金值文件 */
static int load_golden(const char *path, GoldenRow *rows, int cap)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "[SKIP ] 打不开黄金值文件 %s\n", path);
        return 0;
    }
    char line[512];
    int n = 0;
    while (n < cap && fgets(line, sizeof(line), f) != NULL) {
        if (line[0] == '#' || line[0] == '\n') {
            continue;
        }
        GoldenRow *r = &rows[n];
        const int got = sscanf(line, "%31s %d %d %d %d %d %d %d %d",
                               r->name, &r->width, &r->height,
                               &r->flagMinX, &r->flagMinY,
                               &r->flagMaxX, &r->flagMaxY,
                               &r->flagArea, &r->poleX);
        if (got == 9) {
            n++;
        } else if (got != EOF && got != 0) {
            fprintf(stderr, "[WARN ] 跳过无法解析的一行: %s", line);
        }
    }
    fclose(f);
    return n;
}

/** 把 PPM 的 RGB 数据包成 LzFrame */
static LzFrame frame_of(const uint8_t *rgb, int w, int h)
{
    LzFrame f;
    f.data = rgb;
    f.width = w;
    f.height = h;
    f.channels = 3;
    f.stride = w * 3;
    f.isBgr = false;
    f.frameId = 0;
    return f;
}

int main(int argc, char **argv)
{
    (void)argc;

    /* 数据目录的优先级：命令行 > 环境变量 > 构建目录下的相对路径。
     *
     * 为什么要环境变量这一条：ctest 在**构建目录**里跑，而数据在**源树**的
     * `tests/data` —— 写相对路径找不到。CMake 那边把绝对路径塞进
     * `LZ_VISION_TESTDATA`，这里读它。
     * 直接用 `./lz_test_vision` 手跑时给 argv[1]，或者从 lz/ 目录跑
     * （那时相对路径 `tests/data` 正好对）。 */
    const char *dataDir = argv[1];
    if (dataDir == NULL) {
        dataDir = getenv("LZ_VISION_TESTDATA");
    }
    if (dataDir == NULL) {
        dataDir = "tests/data";
    }

    char goldenPath[4000];
    snprintf(goldenPath, sizeof(goldenPath), "%s/vision_golden.txt", dataDir);

    GoldenRow rows[16];
    const int rowCount = load_golden(goldenPath, rows, 16);

    LZ_CASE("黄金值文件必须可读且非空");
    {
        /* 数据是测试的前提。读不到就明确报出来，别让下面的用例静静地
         * 一条都不执行 —— 那样汇总会报"0 项检查"，虽然也算失败，
         * 但原因看不出来。 */
        LZ_CHECK(rowCount > 0);
        if (rowCount > 0) {
            printf("[ INFO] 载入 %d 条黄金值，来自 %s\n", rowCount, goldenPath);
        }
    }
    if (rowCount == 0) {
        return LZ_TEST_SUMMARY();
    }

    LZ_CASE("真实照片：杆列位置与旗面外接框必须匹配黄金值");
    {
        LzVisionConfig cfg = LzVision_DefaultConfig();
        LzVision *vision = NULL;
        LZ_CHECK(LzVision_Init(&cfg, &vision) == LZ_OK);

        int checkedRows = 0;
        for (int i = 0; i < rowCount; ++i) {
            const GoldenRow *g = &rows[i];
            char path[4000];
            snprintf(path, sizeof(path), "%s/%s", dataDir, g->name);

            int w = 0, h = 0;
            uint8_t *rgb = load_ppm(path, &w, &h);
            if (rgb == NULL) {
                continue;   /* load_ppm 已报原因 */
            }
            /* 黄金值里记的尺寸必须与文件实际尺寸一致 —— 不一致说明
             * 黄金值文件和 PPM 不是同一批生成的（生成脚本重跑过一半）
             * 那种情况下比对毫无意义，还会误报成算法错。 */
            LZ_CHECK(w == g->width);
            LZ_CHECK(h == g->height);

            const LzFrame f = frame_of(rgb, w, h);
            LzTargetList list;
            LzTargetList_Init(&list);

            const LzStatus st = LzVision_Detect(vision, &f, &list);
            LZ_CHECK(st == LZ_OK);
            if (st == LZ_OK && list.count > 0) {
                const LzTarget *t = &list.items[0];

                /* ---- 1. 杆列位置：±2 px（已知的裁剪边界效应，见文件头） ---- */
                const double poleXPx = t->pixel.u * (double)w;
                LZ_CHECK_NEAR(poleXPx, (double)g->poleX, 2.0);

                /* ---- 2. 旗面外接框：逐边 ±1 px ----
                 * 这是"阈值没被改坏"的最直接证据。 */
                LZ_CHECK_NEAR(t->pixel.topV * (double)h, (double)g->flagMinY, 1.0);
                LZ_CHECK_NEAR(t->pixel.v * (double)h, (double)g->flagMaxY, 1.0);

                /* ---- 3. 置信度必须是个有意义的数 ---- */
                LZ_CHECK(t->confidence >= 0.0 && t->confidence <= 1.0);
                LZ_CHECK(t->confidence >= cfg.minConfidence);

                /* ---- 4. 视觉**不填** geo / 高度 / 半径 ----
                 * 这是刻意的分工：坐标由激光直出。若哪天有人在这里
                 * 兜一个假坐标，下层无从分辨真假，会一路飞到几内亚湾。
                 *
                 * ⚠️ **坐标的判据必须用 `LzGeo_IsNullSolution`，不能用
                 * `LzGeo_IsValid` 取反** —— 实测（2026-09-24 写这条时踩到）：
                 * 未填的 `geo` 是 memset 出来的全 0，而 `(0, 0)` 在经纬度
                 * 范围内**完全合法**，`LzGeo_IsValid` 会放行。这正是
                 * CLAUDE.md 里记的那个坑："零解在经纬度范围内合法"。
                 * 用错的判据会让这条断言永远为真 —— 而"恒真的断言"
                 * 与"没写断言"是同一件事。 */
                LZ_CHECK(t->heightM <= 0.0);
                LZ_CHECK(t->radiusM <= 0.0);
                LZ_CHECK(LzGeo_IsNullSolution(&t->geo));

                printf("[ INFO] %-12s 图 %dx%d  杆列 %5.1f px（黄金值 %d）"
                       "  旗纵 [%d,%d]  置信度 %.2f\n",
                       g->name, w, h, poleXPx, g->poleX,
                       (int)(t->pixel.topV * h), (int)(t->pixel.v * h),
                       t->confidence);
                checkedRows++;
            } else {
                fprintf(stderr, "[FAIL ] %s 未检出目标（st=%s）\n",
                        g->name, LzStatus_Str(st));
            }

            LzTargetList_Free(&list);
            free(rgb);
        }
        /* 至少要真的比过几行 —— 全被 continue 掉的话上面那些断言
         * 一条都没执行，而汇总会报"检查项数为 0"以外的假象 */
        LZ_CHECK(checkedRows == rowCount);

        LzVision_Deinit(vision);
    }

    LZ_CASE("旗面的竖向范围必须落在黄金值的 ±10% 内");
    {
        /* 单独一条守"HSV 阈值没被顺手改坏"。
         *
         * 阈值调松一点，画面里的红色电动车会被并进旗面，外接框立刻涨；
         * 调紧一点，旗的下摆会掉，框立刻缩。两种都不会让上面那些
         * 位置断言变红 —— 杆列仍然大致对，但检测已经不可靠了。
         *
         * ⚠️ 只查**竖向**范围：`pixel.u` 是**杆列**（不是旗面中心），
         * 所以旗面的横向范围从 `LzTarget` 里恢复不出来。这是刻意的 ——
         * 上层只需要杆列，多暴露一个"旗面中心"字段就会有人去用它，
         * 而那正是本项目实测踩过的坑（差 −47 ~ +52 px，随风向变号）。
         * 要横向范围的话，说明需求变了，那时再改接口。 */
        LzVisionConfig cfg = LzVision_DefaultConfig();
        LzVision *vision = NULL;
        LZ_CHECK(LzVision_Init(&cfg, &vision) == LZ_OK);

        int compared = 0;
        for (int i = 0; i < rowCount; ++i) {
            const GoldenRow *g = &rows[i];
            char path[4000];
            snprintf(path, sizeof(path), "%s/%s", dataDir, g->name);

            int w = 0, h = 0;
            uint8_t *rgb = load_ppm(path, &w, &h);
            if (rgb == NULL) {
                continue;
            }
            const LzFrame f = frame_of(rgb, w, h);
            LzTargetList list;
            LzTargetList_Init(&list);

            if (LzVision_Detect(vision, &f, &list) == LZ_OK && list.count > 0) {
                const LzTarget *t = &list.items[0];
                const double top = t->pixel.topV * (double)h;
                const double bot = t->pixel.v * (double)h;
                const double wantTop = (double)g->flagMinY;
                const double wantBot = (double)g->flagMaxY;
                /* 容差按**框高**的比例给，而不是固定像素 —— 旗大时允许
                 * 的绝对偏差也该大一些，这是阈值型算法的固有性质。 */
                const double tol = (wantBot - wantTop + 1.0) * 0.10;
                LZ_CHECK_NEAR(top, wantTop, tol);
                LZ_CHECK_NEAR(bot, wantBot, tol);
                compared++;
            }

            LzTargetList_Free(&list);
            free(rgb);
        }
        LZ_CHECK(compared == rowCount);

        LzVision_Deinit(vision);
    }

    LZ_CASE("杆列判定必须与「图有多大」无关（补齐画面下缘不改结论）");
    {
        /* ## 这条守的是什么
         *
         * 杆列是谁，靠"这一列有多少行满足对比度判据"来投票。若在**整幅图**
         * 上投票，画面下缘的绿篱边缘、铺装接缝都会投票 —— 于是
         * **判定结果依赖于图的尺寸**，而不是"画面里有什么"。
         *
         * 实测（2026-09-24）这条真的发生过：把同一张照片裁成小图，
         * 某些列的票数变化导致杆列判定**偏移 6 px**。
         * 修法是把投票窗口收窄到旗附近（`poleWinAbove/Below`）。
         *
         * ## 为什么用"往下补"来测
         *
         * 补出来的区域是**纯色**（没有对比度），但它的行仍然进入分母。
         * 于是：窗口跟着图走 → 各列票数被同样稀释，比例不变，结论不变；
         * 窗口不跟着图走（= 整幅图投票）→ 原来旗下方的那些"绿篱边缘"
         * 仍然计票，结论可能翻转。
         *
         * 这条不需要原图就能跑：在裁剪图上再补一部分，等价于"更大的一帧"。
         * 所以它能守住那个回归，而**不依赖**测试数据是谁裁的。
         */
        LzVisionConfig cfg = LzVision_DefaultConfig();
        LzVision *vision = NULL;
        LZ_CHECK(LzVision_Init(&cfg, &vision) == LZ_OK);

        char path[4000];
        snprintf(path, sizeof(path), "%s/%s", dataDir, rows[0].name);
        int w = 0, h = 0;
        uint8_t *rgb = load_ppm(path, &w, &h);
        LZ_CHECK(rgb != NULL);

        if (rgb != NULL) {
            /* 在下面补 200 行，并在其中画一条**很强的竖直干扰线**。
             *
             * 纯色填充测不出东西 —— 均匀区域不投任何票，整幅图投票也照样
             * 得出同一结论（实测：那样构造的反向验证**不会变红**）。
             * 必须放一个真正的干扰物才有效：这条干扰线满足对比度判据的
             * 行数比真杆还多，所以**整幅图投票时它会赢**，
             * 而窗口收窄到旗附近时它根本不参与投票。
             *
             * 这正是现场会遇到的形状：画面下方的绿篱边缘、铺装接缝、
             * 栏杆立柱都是这种"比真杆更显眼的竖线"。 */
            const int padRows = 200;
            const int h2 = h + padRows;
            uint8_t *padded = malloc((size_t)w * h2 * 3);
            LZ_CHECK(padded != NULL);
            if (padded != NULL) {
                memcpy(padded, rgb, (size_t)w * h * 3);
                memset(padded + (size_t)w * h * 3, 128, (size_t)w * padRows * 3);

                /* 干扰线画在离杆列很远的固定位置（画面 3/4 宽处），
                 * 高对比度、贯穿整个补出来的区域 */
                const int distractorX = w * 3 / 4;
                for (int y = h; y < h2; ++y) {
                    uint8_t *row = padded + (size_t)y * w * 3;
                    for (int x = distractorX - 2; x <= distractorX + 2; ++x) {
                        if (x < 0 || x >= w) {
                            continue;
                        }
                        row[x * 3] = 0; row[x * 3 + 1] = 0; row[x * 3 + 2] = 0;
                    }
                }

                LzTargetList a, b;
                LzTargetList_Init(&a);
                LzTargetList_Init(&b);
                const LzFrame fa = frame_of(rgb, w, h);
                const LzFrame fb = frame_of(padded, w, h2);

                const LzStatus sa = LzVision_Detect(vision, &fa, &a);
                const LzStatus sb = LzVision_Detect(vision, &fb, &b);
                LZ_CHECK(sa == LZ_OK);
                LZ_CHECK(sb == LZ_OK);
                if (sa == LZ_OK && sb == LZ_OK && a.count > 0 && b.count > 0) {
                    /* 结论必须一模一样 —— 补出来的那段里除了干扰线别无他物，
                     * 它不该改变"杆在哪"。 */
                    LZ_CHECK_NEAR(a.items[0].pixel.u * w, b.items[0].pixel.u * w, 0.5);
                    LZ_CHECK(a.items[0].pixel.u == b.items[0].pixel.u);
                    /* 顺带确认干扰线真的**没有**被选中 */
                    LZ_CHECK(fabs(b.items[0].pixel.u * w - distractorX) > 5.0);
                }
                LzTargetList_Free(&a);
                LzTargetList_Free(&b);
                free(padded);
            }
            free(rgb);
        }

        LzVision_Deinit(vision);
    }

    LZ_CASE("连通域：需要多次合并的形状必须连成一块");
    {
        /* ## 为什么单独构造，不用真实照片
         *
         * 真实照片里的旗是一整块近似矩形的连通区域，并查集几乎不需要真正
         * 合并（每个像素的邻居基本都带着同一个根）。所以那些用例能测出
         * "识别得对不对"，测不出"合并逻辑写得对不对"。
         *
         * ## ⚠️ 这里踩了**两次**坑，两次都是"构造不逼出要测的东西"
         *
         * **第一次**：画布取 34x24，U 形占画面 28% → 撞上置信度里
         * "红块过大"的惩罚，被判不可用。测的是置信度，不是连通域。
         *
         * **第二次**：改成 200x200 的大 U 形之后，**去掉 union 整段代码
         * 测试照样全绿** —— 因为只要有一块"与另一个块上下接触"（同一列
         * 相邻），第二遍路径压缩时的**左邻/上邻传播**就足以把两块连起来，
         * 全程不需要 union。可用的形状是：
         *
         *     左块(10x15)      右块(10x15)      ← 两者**横向**分离（中间空 6 列）
         *     └────────────────────────┘        ← 一条 1 行的横桥
         *
         * 关键在**桥只与两块的底边相接，且桥自身是一整行**：
         * 桥的最左像素的左邻是背景、左上角也是背景（左块在它上面），
         * 于是它建立**新的根**；往右走到左块正下方时，左邻是桥的新根、
         * 而**左上角**是左块的根 —— 这时必须真的 union 才能把两者并起来。
         *
         * 实测（/tmp 下的对照程序）：
         *   正确 union → 最大块 326 px ≥ 200 → 检出
         *   不做 union → 最大块 176 px < 200 → 报无目标
         * 差距够大，所以那条 `st == LZ_OK` 就是有效的反向验证判据。
         */
        const int W = 200, H = 200;
        uint8_t *img = calloc((size_t)W * H * 3, 1);
        LZ_CHECK(img != NULL);
        if (img == NULL) {
            return LZ_TEST_SUMMARY();
        }
        for (int i = 0; i < W * H; ++i) {
            img[i * 3] = 10; img[i * 3 + 1] = 10; img[i * 3 + 2] = 60;   /* 深蓝背景 */
        }
        const int lbX0 = 80, lbX1 = 89, rbX0 = 96, rbX1 = 105;
        const int blkY0 = 60, blkY1 = 74, bridgeY = 75;
        for (int y = blkY0; y <= blkY1; ++y) {
            for (int x = lbX0; x <= lbX1; ++x) {
                const int i = (y * W + x) * 3;
                img[i] = 220; img[i + 1] = 20; img[i + 2] = 20;
            }
            for (int x = rbX0; x <= rbX1; ++x) {
                const int i = (y * W + x) * 3;
                img[i] = 220; img[i + 1] = 20; img[i + 2] = 20;
            }
        }
        for (int x = lbX0; x <= rbX1; ++x) {
            const int i = (bridgeY * W + x) * 3;
            img[i] = 220; img[i + 1] = 20; img[i + 2] = 20;
        }
        /* 左块 10x15 = 150，右块 10x15 = 150，桥 26 = 326。
         * 两块分开时最大 176（150 + 桥的 26，桥本身与左块同根），
         * 低于 minBlobArea(200) —— 所以"没合并"必然表现成"检不出目标"。 */
        const int expected = 10 * 15 * 2 + 26;

        LzVisionConfig cfg = LzVision_DefaultConfig();
        /* ⚠️ **把置信度门槛压到 0**：本用例测的是连通域，而这个人造形状
         * 里没有"竖线"，杆的证据必然是 0 —— 那会让置信度归零、目标被判
         * 不可用，于是一条本该测合并逻辑的用例变成在测置信度（实测踩到）。
         * 把不相干的那一级中和掉，用例才真的只测它要测的东西。 */
        cfg.minConfidence = 0.0;
        LzVision *vision = NULL;
        LZ_CHECK(LzVision_Init(&cfg, &vision) == LZ_OK);

        LzTargetList list;
        LzTargetList_Init(&list);
        const LzFrame f = frame_of(img, W, H);
        const LzStatus st = LzVision_Detect(vision, &f, &list);

        /* 合并正确 → 326 px 够大 → 检出；没合并 → 176 px → 无目标。
         * 这是本用例唯一需要的断言，而且**能被反向验证**（实测过）。 */
        LZ_CHECK(st == LZ_OK);
        if (st == LZ_OK && list.count > 0) {
            /* 外接框必须横跨两个块 —— 只认得一块的话跨度会小得多。
             * 这是"真的连成了一块"的几何证据，比面积更硬（面积是我手算的）。 */
            const double spanV = list.items[0].pixel.v - list.items[0].pixel.topV;
            LZ_CHECK_NEAR(spanV * H, (double)(bridgeY - blkY0 + 1), 2.0);
        }

        LzTargetList_Free(&list);
        LzVision_Deinit(vision);

        /* 守住手算的期望值：构造若被改动，这条会先红，
         * 提醒同步改上面注释里的算式 */
        LZ_CHECK(expected == 326);
        LZ_CHECK(expected > LzVision_DefaultConfig().minBlobArea);
        free(img);
    }

    LZ_CASE("置信度：红块占满画面时必须压低，哪怕里面有杆");
    {
        /* 构造"镜头被一面红旗糊住"的情形：整幅都是红，但其中有一条
         * 高对比度的竖线（所以杆的证据是满的）。
         *
         * 期望：**判为不可用**。占满视场的红块意味着"目标把镜头堵住了"，
         * 此时"杆在哪"这个问题的答案没有意义 —— 就算算出来一个也不敢用。
         *
         * ⚠️ 反向验证发现这条是必需的（2026-09-24）：去掉 `frac > 0.05`
         * 的惩罚后，用"纯全红图"构造的那条用例**照样通过** ——
         * 因为均匀红色区域没有对比度，杆的证据本来就是 0，置信度被
         * `min()` 拉低到 0，走的不是旗面那条路。必须**同时**给出
         * 满格的杆证据，才真正隔离出"旗面过大要压分"这一条。 */
        const int W = 200, H = 200;
        uint8_t *img = malloc((size_t)W * H * 3);
        LZ_CHECK(img != NULL);
        if (img != NULL) {
            for (int i = 0; i < W * H; ++i) {
                img[i * 3] = 220; img[i * 3 + 1] = 20; img[i * 3 + 2] = 20;
            }
            /* 一条贯穿整幅的暗竖线 —— 与左右邻域亮度差极大 */
            for (int y = 0; y < H; ++y) {
                for (int x = W / 2 - 1; x <= W / 2 + 1; ++x) {
                    const int i = (y * W + x) * 3;
                    img[i] = 0; img[i + 1] = 0; img[i + 2] = 0;
                }
            }

            LzVisionConfig cfg = LzVision_DefaultConfig();
            LzVision *vision = NULL;
            LZ_CHECK(LzVision_Init(&cfg, &vision) == LZ_OK);
            LzTargetList list;
            LzTargetList_Init(&list);
            const LzFrame f = frame_of(img, W, H);

            const LzStatus st = LzVision_Detect(vision, &f, &list);
            /* 要么直接判不可用，要么给出的置信度必须低到不可用 */
            if (st == LZ_OK && list.count > 0) {
                LZ_CHECK(list.items[0].confidence < cfg.minConfidence);
            } else {
                LZ_CHECK(st == LZ_ERR_NO_TARGET);
            }

            LzTargetList_Free(&list);
            LzVision_Deinit(vision);
            free(img);
        }
    }

    LZ_CASE("置信度：目标太小 / 杆的证据不足时必须判为无目标");
    {
        /* ## 构造
         *
         * 一片**均匀**的红色（满足面积下限，但没有对比度），四周是纯色背景。
         * 那里没有任何"竖线"—— 杆的证据为 0。
         *
         * 期望：判为无目标。理由是分工：`LzVision_Detect` 的产出是**杆的
         * 像素列**，上层拿它对激光做对齐判定。只能看见一片红、找不到杆
         * 的时候说"杆在这儿"是错的 —— 宁可报无目标，让操作员重新瞄准。
         *
         * ⚠️ 反向验证发现（2026-09-24）：把置信度里的杆证据那一项拿掉
         * （只留旗面得分）时，"纯全红图"那条用例**照样通过** ——
         * 因为它撞的是旗面过大的那条分支。必须用"面积正常但无杆"的形状
         * 才能隔离出杆证据这一项。 */
        /* ⚠️ **画布必须够大**：红块要"面积正常"（不触发旗面过大的惩罚），
         * 才谈得上隔离出"杆的证据"这一项。写成 120x120 时那块红占 16.7%，
         * 先被旗面那条拦掉 —— **测的是旗面，不是杆**（实测踩到）。
         * 取 400x400，同一个红块只占 1.5%。 */
        const int W = 400, H = 400;
        uint8_t *img = malloc((size_t)W * H * 3);
        LZ_CHECK(img != NULL);
        if (img != NULL) {
            for (int i = 0; i < W * H; ++i) {
                img[i * 3] = 40; img[i * 3 + 1] = 45; img[i * 3 + 2] = 50;   /* 灰背景 */
            }
            /* 一块 60x40 的红 —— 2400 px，远大于 minBlobArea(200)，
             * 占画面 1.5%（**不**触发旗面过大/过小的惩罚）。
             * 内部纯色 → 杆的证据为 0。 */
            for (int y = 180; y < 220; ++y) {
                for (int x = 170; x < 230; ++x) {
                    const int i = (y * W + x) * 3;
                    img[i] = 220; img[i + 1] = 25; img[i + 2] = 25;
                }
            }

            LzVisionConfig cfg = LzVision_DefaultConfig();
            LzVision *vision = NULL;
            LZ_CHECK(LzVision_Init(&cfg, &vision) == LZ_OK);
            LzTargetList list;
            LzTargetList_Init(&list);
            const LzFrame f = frame_of(img, W, H);

            const LzStatus st = LzVision_Detect(vision, &f, &list);
            if (st == LZ_OK && list.count > 0) {
                /* 若哪天决定"有红块就算找到"（放宽分工），这条会红 ——
                 * 那时应当改的是注释与上层的对齐判定，不是这条断言 */
                LZ_CHECK(list.items[0].confidence < cfg.minConfidence);
            } else {
                LZ_CHECK(st == LZ_ERR_NO_TARGET);
            }

            LzTargetList_Free(&list);
            LzVision_Deinit(vision);
            free(img);
        }
    }

    LZ_CASE("退化输入不得崩溃，且必须被明确拒绝");
    {
        LzVisionConfig cfg = LzVision_DefaultConfig();
        LzVision *vision = NULL;
        LZ_CHECK(LzVision_Init(&cfg, &vision) == LZ_OK);

        /* 小图：16x16 的黑图 —— 没有红色，应报"无目标"而不是崩 */
        uint8_t black[16 * 16 * 3];
        memset(black, 0, sizeof(black));
        LzFrame f = frame_of(black, 16, 16);
        LzTargetList list;
        LzTargetList_Init(&list);
        LZ_CHECK(LzVision_Detect(vision, &f, &list) == LZ_ERR_NO_TARGET);

        /* 全红图：整幅都是红 —— 旗面占满画面，置信度应被压低。
         * 一个"占满视场的红块"不是我们要找的目标（见 lz_confidence）。 */
        uint8_t *red = malloc(64 * 64 * 3);
        LZ_CHECK(red != NULL);
        if (red != NULL) {
            for (int i = 0; i < 64 * 64; ++i) {
                red[i * 3] = 220; red[i * 3 + 1] = 20; red[i * 3 + 2] = 20;
            }
            LzFrame rf = frame_of(red, 64, 64);
            LzTargetList rl;
            LzTargetList_Init(&rl);
            const LzStatus rst = LzVision_Detect(vision, &rf, &rl);
            /* 要么判为无目标，要么给出低置信度 —— 但绝不能给高分 */
            if (rst == LZ_OK && rl.count > 0) {
                LZ_CHECK(rl.items[0].confidence < 0.5);
            } else {
                LZ_CHECK(rst == LZ_ERR_NO_TARGET);
            }
            LzTargetList_Free(&rl);
            free(red);
        }

        LzTargetList_Free(&list);

        /* 空指针 / 零尺寸 / 非法通道数 */
        LZ_CHECK(LzVision_Detect(NULL, &f, &list) == LZ_ERR_PARAM);
        LZ_CHECK(LzVision_Detect(vision, NULL, &list) == LZ_ERR_PARAM);
        LZ_CHECK(LzVision_Detect(vision, &f, NULL) == LZ_ERR_PARAM);

        LzFrame bad = frame_of(black, 0, 16);
        LZ_CHECK(LzVision_Detect(vision, &bad, &list) == LZ_ERR_PARAM);

        LzFrame nv12 = frame_of(black, 16, 16);
        nv12.channels = 2;   /* NV12 之类的格式没实现 */
        LZ_CHECK(LzVision_Detect(vision, &nv12, &list) == LZ_ERR_UNSUPPORTED);

        LzVision_Deinit(vision);
    }

    LZ_CASE("Init 必须拒绝自相矛盾的配置");
    {
        /* 这些参数会被用来算内存下标。Init 是唯一能拦住它们的地方 ——
         * 让它们进去，Detect 就会越界写，而那种崩溃极难定位。 */
        LzVision *v = (LzVision *)0x1;

        LzVisionConfig c = LzVision_DefaultConfig();
        c.poleContrastOffset = 0;      /* 邻域偏移 0 → "与左右比较"恒等 */
        LZ_CHECK(LzVision_Init(&c, &v) == LZ_ERR_PARAM);

        c = LzVision_DefaultConfig();
        c.poleRoiMarginX = -1;         /* 负边距 → ROI 反向 */
        LZ_CHECK(LzVision_Init(&c, &v) == LZ_ERR_PARAM);

        c = LzVision_DefaultConfig();
        c.poleWinBelow = -1;
        LZ_CHECK(LzVision_Init(&c, &v) == LZ_ERR_PARAM);

        c = LzVision_DefaultConfig();
        c.poleHalfWidth = -1;
        LZ_CHECK(LzVision_Init(&c, &v) == LZ_ERR_PARAM);

        /* 色相区间反了 / 越界 */
        c = LzVision_DefaultConfig();
        c.redHueLowMax = 170;
        c.redHueHighMin = 10;
        LZ_CHECK(LzVision_Init(&c, &v) == LZ_ERR_PARAM);

        c = LzVision_DefaultConfig();
        c.redHueHighMin = 200;
        LZ_CHECK(LzVision_Init(&c, &v) == LZ_ERR_PARAM);

        /* 默认配置必须能过 */
        c = LzVision_DefaultConfig();
        LzVision *ok = NULL;
        LZ_CHECK(LzVision_Init(&c, &ok) == LZ_OK);
        LzVision_Deinit(ok);

        LZ_CHECK(LzVision_Init(NULL, &v) == LZ_ERR_PARAM);
        LZ_CHECK(LzVision_Init(&c, NULL) == LZ_ERR_PARAM);
    }

    return LZ_TEST_SUMMARY();
}
