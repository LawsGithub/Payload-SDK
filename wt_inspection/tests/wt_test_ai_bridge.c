/**
 * @file wt_test_ai_bridge.c
 * @brief WtAiBridge_FromTrtResult 的回归测试。
 *
 * 核心要验证的是**索引映射没有错位**——hub/tip 的下标搬运一旦弄混
 * （比如把某个叶尖当成轮毂），几何解算侧不会报错，只会算出一个
 * 看起来合理但语义错误的结果，光靠肉眼很难发现。所以每个关键点
 * 都填一个彼此不同、可追溯的坐标值。
 */

#include "wt_ai_bridge.h"

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
            printf("  [FAIL] " fmt " (got %.4f, want %.4f)\n",             \
                   ##__VA_ARGS__, _a, _b);                                 \
            s_failures++;                                                  \
        }                                                                  \
    } while (0)

/** 按关键点索引 k 生成一个唯一、可追溯的坐标值：x=100*(k+1), y=100*(k+1)+1 */
static void FillDetAt(WtTrtDetection *d, float conf)
{
    int k;

    memset(d, 0, sizeof(*d));
    d->bboxX1 = 0.0f;
    d->bboxY1 = 0.0f;
    d->bboxX2 = 10.0f;
    d->bboxY2 = 10.0f;
    d->conf = conf;
    for (k = 0; k < WT_TRT_KPT_COUNT; k++) {
        d->kptX[k] = (float)(100 * (k + 1));
        d->kptY[k] = (float)(100 * (k + 1) + 1);
    }
}

/**
 * 单目标：hub 必须取自索引 3，三个 tip 必须依次取自索引 4/5/6，
 * 不能互相搞混，也不能偏移。
 */
static void Test_SingleDetection_IndexMapping(void)
{
    WtTrtResult trt;
    WtAiDetection out;

    printf("Test_SingleDetection_IndexMapping\n");

    memset(&trt, 0, sizeof(trt));
    trt.count = 1;
    FillDetAt(&trt.items[0], 0.9f);

    CHECK(WtAiBridge_FromTrtResult(&trt, &out), "应转换成功");
    CHECK(out.hubValid, "hub 应标记为 valid");
    CHECK_NEAR(out.hubU, 400.0, 1e-6, "hubU 应取自 kpt[3]");
    CHECK_NEAR(out.hubV, 401.0, 1e-6, "hubV 应取自 kpt[3]");
    CHECK(out.tipCount == 3, "tipCount 应为 3（得到 %d）", out.tipCount);

    CHECK_NEAR(out.tipU[0], 500.0, 1e-6, "tip[0] 应取自 kpt[4]");
    CHECK_NEAR(out.tipU[1], 600.0, 1e-6, "tip[1] 应取自 kpt[5]");
    CHECK_NEAR(out.tipU[2], 700.0, 1e-6, "tip[2] 应取自 kpt[6]");
    CHECK(out.tipValid[0] && out.tipValid[1] && out.tipValid[2],
          "三个 tip 都应标记为 valid");
}

/**
 * 多目标：必须取 conf 最高的那个（items[0]），不能被其他目标污染。
 * items[] 按 conf 降序排列是 wt_trt_pose.cpp 的 NMS 产出约定，这里不
 * 自己再排序，只验证"确实用了 items[0]"。
 */
static void Test_MultiDetection_PicksFirst(void)
{
    WtTrtResult trt;
    WtAiDetection out;

    printf("Test_MultiDetection_PicksFirst\n");

    memset(&trt, 0, sizeof(trt));
    trt.count = 2;
    FillDetAt(&trt.items[0], 0.95f);   /* 最可信：坐标按 100*(k+1) 生成 */
    FillDetAt(&trt.items[1], 0.30f);
    /* 第二个目标坐标整体偏移，若被误选，下面的断言会精确抓到。 */
    {
        int k;

        for (k = 0; k < WT_TRT_KPT_COUNT; k++) {
            trt.items[1].kptX[k] += 9000.0f;
            trt.items[1].kptY[k] += 9000.0f;
        }
    }

    CHECK(WtAiBridge_FromTrtResult(&trt, &out), "应转换成功");
    CHECK_NEAR(out.hubU, 400.0, 1e-6, "应取 items[0] 的 hub，不是 items[1]");
    CHECK_NEAR(out.tipU[0], 500.0, 1e-6, "应取 items[0] 的 tip[0]");
}

/** count<=0 时必须拒绝，不能读出未定义的 items[0]。 */
static void Test_EmptyResult_Rejected(void)
{
    WtTrtResult trt;
    WtAiDetection out;

    printf("Test_EmptyResult_Rejected\n");

    memset(&trt, 0, sizeof(trt));
    trt.count = 0;

    CHECK(!WtAiBridge_FromTrtResult(&trt, &out), "count=0 应返回 false");
    CHECK(!WtAiBridge_FromTrtResult(NULL, &out), "NULL trt 应被拒绝");
    CHECK(!WtAiBridge_FromTrtResult(&trt, NULL), "NULL out 应被拒绝");
}

int main(void)
{
    printf("=== WtAiBridge 转换测试 ===\n");

    Test_SingleDetection_IndexMapping();
    Test_MultiDetection_PicksFirst();
    Test_EmptyResult_Rejected();

    if (s_failures == 0) {
        printf("全部通过\n");
        return 0;
    }
    printf("%d 项失败\n", s_failures);
    return 1;
}
