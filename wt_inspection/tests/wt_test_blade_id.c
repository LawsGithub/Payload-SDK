/**
 * @file wt_test_blade_id.c
 * @brief 叶片编号解算的回归测试。
 *
 * 这个模块的失败方式很隐蔽：编号错了不会崩，只会让飞机去修错的那片叶片。
 * 所以用例要覆盖「角度噪声」「绕环」「park_phase 落在边界」这些真实会遇到
 * 的情况，而不只是理想输入。
 */

#include "wt_blade_id.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int s_failures = 0;

#define CHECK(cond, fmt, ...)                                              \
    do {                                                                   \
        if (!(cond)) {                                                     \
            printf("  [FAIL] " fmt "\n", ##__VA_ARGS__);                   \
            s_failures++;                                                  \
        }                                                                  \
    } while (0)

#define CHECK_NEAR(a, b, tol, fmt, ...)                                    \
    do {                                                                   \
        const double _a = (a);                                             \
        const double _b = (b);                                             \
        if (fabs(_a - _b) > (tol)) {                                       \
            printf("  [FAIL] " fmt " (got %.3f, want %.3f)\n",             \
                   ##__VA_ARGS__, _a, _b);                                 \
            s_failures++;                                                  \
        }                                                                  \
    } while (0)

/* ------------------------------------------------------------------ */

/**
 * 理想情况：三片叶片 0/120/240，park_phase = 0。
 * 期望：角度 0 的那片是 NO.1，120 是 NO.2，240 是 NO.3。
 */
static void Test_Ideal(void)
{
    WtBladeIdInput in;
    WtBladeIdResult out;

    printf("Test_Ideal\n");

    memset(&in, 0, sizeof(in));
    in.count = 3;
    in.anglesDeg[0] = 0.0;
    in.anglesDeg[1] = 120.0;
    in.anglesDeg[2] = 240.0;
    in.parkPhaseDeg = 0.0;

    CHECK(WtBladeId_AssignFromAngles(&in, &out), "解算应成功");
    CHECK(out.bladeNoOf[0] == 1, "0° 应为 NO.1（得到 %d）", out.bladeNoOf[0]);
    CHECK(out.bladeNoOf[1] == 2, "120° 应为 NO.2（得到 %d）", out.bladeNoOf[1]);
    CHECK(out.bladeNoOf[2] == 3, "240° 应为 NO.3（得到 %d）", out.bladeNoOf[2]);
    CHECK_NEAR(out.worstGapDeg, 0.0, 0.01, "理想分布间隔偏差应为 0");
}

/**
 * 输入顺序打乱 —— 编号必须跟角度走，不能跟数组下标走。
 */
static void Test_ShuffledInput(void)
{
    WtBladeIdInput in;
    WtBladeIdResult out;

    printf("Test_ShuffledInput\n");

    memset(&in, 0, sizeof(in));
    in.count = 3;
    in.anglesDeg[0] = 240.0; /* 应为 NO.3 */
    in.anglesDeg[1] = 0.0;   /* 应为 NO.1 */
    in.anglesDeg[2] = 120.0; /* 应为 NO.2 */
    in.parkPhaseDeg = 0.0;

    CHECK(WtBladeId_AssignFromAngles(&in, &out), "解算应成功");
    CHECK(out.bladeNoOf[0] == 3, "索引0(240°) 应为 NO.3（得到 %d）", out.bladeNoOf[0]);
    CHECK(out.bladeNoOf[1] == 1, "索引1(0°) 应为 NO.1（得到 %d）", out.bladeNoOf[1]);
    CHECK(out.bladeNoOf[2] == 2, "索引2(120°) 应为 NO.2（得到 %d）", out.bladeNoOf[2]);
    CHECK(out.oneBasedIndex == 1, "NO.1 应在输入索引 1（得到 %d）", out.oneBasedIndex);
}

/**
 * 绕环：park_phase 在 350°，叶片在 350/110/230。
 * 编号沿角度增大方向递增，跨越 0° 边界不能出错。
 */
static void Test_Wraparound(void)
{
    WtBladeIdInput in;
    WtBladeIdResult out;

    printf("Test_Wraparound\n");

    memset(&in, 0, sizeof(in));
    in.count = 3;
    in.anglesDeg[0] = 350.0;
    in.anglesDeg[1] = 110.0;
    in.anglesDeg[2] = 230.0;
    in.parkPhaseDeg = 350.0;

    CHECK(WtBladeId_AssignFromAngles(&in, &out), "解算应成功");
    CHECK(out.bladeNoOf[0] == 1, "350° 应为 NO.1（得到 %d）", out.bladeNoOf[0]);
    CHECK(out.bladeNoOf[1] == 2, "110° 应为 NO.2（得到 %d）", out.bladeNoOf[1]);
    CHECK(out.bladeNoOf[2] == 3, "230° 应为 NO.3（得到 %d）", out.bladeNoOf[2]);
}

/**
 * park_phase 落在两片叶片中间时，选「夹角最小」的那片。
 * 这里是 10°，与 0° 差 10、与 120° 差 110 → 应选 0°。
 */
static void Test_NearestWins(void)
{
    WtBladeIdInput in;
    WtBladeIdResult out;

    printf("Test_NearestWins\n");

    memset(&in, 0, sizeof(in));
    in.count = 3;
    in.anglesDeg[0] = 0.0;
    in.anglesDeg[1] = 120.0;
    in.anglesDeg[2] = 240.0;
    in.parkPhaseDeg = 10.0;

    CHECK(WtBladeId_AssignFromAngles(&in, &out), "解算应成功");
    CHECK(out.bladeNoOf[0] == 1, "0° 应与 park 10° 最近，为 NO.1（得到 %d）",
          out.bladeNoOf[0]);
}

/**
 * 角度噪声：检测有 ±3° 误差，编号仍应正确。
 * 同时 worstGapDeg 应该反映出这个噪声量级。
 */
static void Test_NoisyAngles(void)
{
    WtBladeIdInput in;
    WtBladeIdResult out;

    printf("Test_NoisyAngles\n");

    memset(&in, 0, sizeof(in));
    in.count = 3;
    in.anglesDeg[0] = 2.0;
    in.anglesDeg[1] = 118.0;
    in.anglesDeg[2] = 243.0;
    in.parkPhaseDeg = 0.0;

    CHECK(WtBladeId_AssignFromAngles(&in, &out), "解算应成功");
    CHECK(out.bladeNoOf[0] == 1, "2° 应为 NO.1（得到 %d）", out.bladeNoOf[0]);
    CHECK(out.bladeNoOf[1] == 2, "118° 应为 NO.2（得到 %d）", out.bladeNoOf[1]);
    CHECK(out.bladeNoOf[2] == 3, "243° 应为 NO.3（得到 %d）", out.bladeNoOf[2]);
    /* 偏差最大约 3° —— 超过 6° 说明间隔解算有问题 */
    CHECK(out.worstGapDeg > 1.0 && out.worstGapDeg < 6.0,
          "间隔偏差应反映噪声量级（得到 %.2f）", out.worstGapDeg);
}

/**
 * 非法输入必须拒绝，不能静默返回错误编号。
 */
static void Test_RejectInvalid(void)
{
    WtBladeIdInput in;
    WtBladeIdResult out;

    printf("Test_RejectInvalid\n");

    memset(&in, 0, sizeof(in));
    in.count = 0;
    CHECK(!WtBladeId_AssignFromAngles(&in, &out), "count=0 应被拒绝");

    in.count = WT_BLADE_ID_MAX + 1;
    CHECK(!WtBladeId_AssignFromAngles(&in, &out), "count 超限应被拒绝");

    CHECK(!WtBladeId_AssignFromAngles(NULL, &out), "NULL 输入应被拒绝");
    CHECK(!WtBladeId_AssignFromAngles(&in, NULL), "NULL 输出应被拒绝");
}

/**
 * 失败时输出必须是**明确的失败值**，不能留着调用方栈上的残留。
 *
 * 原先 early-return 的几条路径完全不碰 out：
 *   - 头文件声明「oneBasedIndex = -1 表示失败」，而代码从不写它
 *   - worstGapDeg 可能恰好是 0，而 0 在 README 的质量表里是"编号可信"
 *     那一档 —— 于是"编号失败"会被下游读成"质量最好"
 *
 * 这里刻意把 out 预填成"看起来成功"的值再调用，验证失败路径会覆盖它们。
 * 另外验证**不用 NaN**：NaN 参与任何比较都返回 false，下游
 * `if (dev > 阈值) reject` 会静默失效（fail-open）。360 则必然被拒。
 */
static void Test_FailureLeavesExplicitValues(void)
{
    WtBladeIdInput in;
    WtBladeIdResult out;
    int i;

    printf("Test_FailureLeavesExplicitValues\n");

    memset(&in, 0, sizeof(in));
    in.count = 0; /* 非法 */

    /* 预填成"成功且质量完美"的样子 */
    memset(&out, 0, sizeof(out));
    out.oneBasedIndex = 2;
    out.worstGapDeg = 0.0;
    for (i = 0; i < WT_BLADE_ID_MAX; i++) {
        out.bladeNoOf[i] = 7;
    }

    CHECK(!WtBladeId_AssignFromAngles(&in, &out), "应返回失败");

    CHECK(out.oneBasedIndex == -1,
          "失败时 oneBasedIndex 应为 -1（得到 %d）", out.oneBasedIndex);
    CHECK(out.worstGapDeg > 180.0,
          "失败时 worstGapDeg 应是必然被拒的大值，而不是 0（得到 %.1f）",
          out.worstGapDeg);
    CHECK(out.worstGapDeg == out.worstGapDeg,
          "不应是 NaN —— NaN 会让下游的阈值比较静默失效");
    for (i = 0; i < WT_BLADE_ID_MAX; i++) {
        CHECK(out.bladeNoOf[i] == 0,
              "失败时 bladeNoOf[%d] 应清零（得到 %d）", i, out.bladeNoOf[i]);
    }
}

/**
 * 像素路径：轮毂在 (720,540)，三个叶尖分别在正上、右下、左下。
 * 正上 = 图像角度 90°，右下 = -30°，左下 = 210°。
 * sign=+1, offset=0, park=90 → 正上那片应为 NO.1。
 */
static void Test_PixelPath(void)
{
    const double tipU[3] = {720.0, 720.0 + 300.0, 720.0 - 300.0};
    const double tipV[3] = {540.0 - 300.0, 540.0 + 173.0, 540.0 + 173.0};
    double imgAngles[3] = {0};
    WtBladeIdResult out;

    printf("Test_PixelPath\n");

    CHECK(WtBladeId_AssignFromPixels(720.0, 540.0, tipU, tipV, 3,
                                     1.0, 0.0, 90.0, &out, imgAngles),
          "像素路径解算应成功");

    /* 正上方：du=0, dv=-300 → atan2(300,0) = 90° */
    CHECK_NEAR(imgAngles[0], 90.0, 1.0, "正上方叶尖图像角应为 90°");
    /* 右下：du=300, dv=+173 → atan2(-173,300) ≈ -30° */
    CHECK_NEAR(imgAngles[1], -30.0, 2.0, "右下叶尖图像角应约 -30°");
    /* 左下：du=-300, dv=+173 → atan2(-173,-300) ≈ -150° */
    CHECK_NEAR(imgAngles[2], -150.0, 2.0, "左下叶尖图像角应约 -150°");

    CHECK(out.bladeNoOf[0] == 1, "正上方应为 NO.1（得到 %d）", out.bladeNoOf[0]);
}

/**
 * 相机翻转（sign=-1）时，编号要跟着翻转，不能沿用正装的结果。
 */
static void Test_PixelSignFlip(void)
{
    const double tipU[3] = {720.0, 720.0 + 300.0, 720.0 - 300.0};
    const double tipV[3] = {540.0 - 300.0, 540.0 + 173.0, 540.0 + 173.0};
    WtBladeIdResult outFlipped;

    printf("Test_PixelSignFlip\n");

    /* sign=-1 时，图像 90° 映射到相位 -90° = 270°；park=270 → 正上那片仍应是 NO.1 */
    CHECK(WtBladeId_AssignFromPixels(720.0, 540.0, tipU, tipV, 3,
                                     -1.0, 0.0, 270.0, &outFlipped, NULL),
          "翻转解算应成功");
    CHECK(outFlipped.bladeNoOf[0] == 1,
          "翻转后正上方仍应为 NO.1（得到 %d）", outFlipped.bladeNoOf[0]);
}

/* ------------------------------------------------------------------ */

int main(void)
{
    printf("=== 叶片编号解算测试 ===\n");

    Test_Ideal();
    Test_ShuffledInput();
    Test_Wraparound();
    Test_NearestWins();
    Test_NoisyAngles();
    Test_RejectInvalid();
    Test_FailureLeavesExplicitValues();
    Test_PixelPath();
    Test_PixelSignFlip();

    if (s_failures == 0) {
        printf("全部通过\n");
        return 0;
    }

    printf("%d 项失败\n", s_failures);
    return 1;
}
