/**
 * @file wt_test_yolo_decode.c
 * @brief YOLO 预处理与解码的回归测试（`app/wt_yolo_decode.c`）。
 *
 * 守的都是「错了不报错」的约定：
 *
 *   - 通道下标错一位 → 框照样画得出来，只是落在别处
 *   - letterbox 漏还原 / 填充量取反 → 目标系统性偏移，且随长宽比变化
 *   - 类别先筛后取 argmax → 一个明显更像自行车的框被当成人
 *   - NMS 按类别分开做 → 同一个人被报成 pedestrian + people 两个目标，
 *     跟踪在两者之间来回跳
 *   - 容量写死 32 → 人群里排在后面的目标被无故截掉
 */

#include "wt_yolo_decode.h"

#include "wt_test.h"

#include <stdlib.h>
#include <string.h>

/* 合成张量：通道优先，channels × n */
static float *NewTensor(int channels, int n)
{
    float *t = (float *)calloc((size_t)channels * (size_t)n, sizeof(float));

    return t;
}

static void SetBox(float *t, int n, int i, float cx, float cy, float w, float h)
{
    t[0 * n + i] = cx;
    t[1 * n + i] = cy;
    t[2 * n + i] = w;
    t[3 * n + i] = h;
}

static void TestChannels(void)
{
    WtYoloLayout turbine = {1, 7, 2};
    WtYoloLayout visdrone = {11, 0, 0};   /* 实际权重：VisDrone 10 类 + others */
    WtYoloLayout bad = {0, 0, 0};
    WtYoloLayout tooManyKpt = {1, WT_TRT_KPT_COUNT + 1, 2};
    WtYoloLayout l;

    WT_CASE("通道数：风机 19、VisDrone（11 类）15，非法布局为 0");

    WT_CHECK_EQ_INT(WtYolo_Channels(&turbine), 19);
    WT_CHECK_EQ_INT(WtYolo_Channels(&visdrone), 15);
    WT_CHECK_EQ_INT(WtYolo_Channels(&bad), 0);
    WT_CHECK_EQ_INT(WtYolo_Channels(&tooManyKpt), 0);
    WT_CHECK_EQ_INT(WtYolo_Channels(NULL), 0);

    WT_CASE("由 engine 的通道数反推类别数");

    WT_CHECK(WtYolo_LayoutFromChannels(15, 0, 0, &l));
    WT_CHECK_EQ_INT(l.numClasses, 11);
    WT_CHECK(WtYolo_LayoutFromChannels(19, 7, 2, &l));
    WT_CHECK_EQ_INT(l.numClasses, 1);
    WT_CHECK(WtYolo_LayoutFromChannels(5, 0, 0, &l));
    WT_CHECK_EQ_INT(l.numClasses, 1);
    /* 4 通道 = 没有类别得分，不是合法的 YOLO 输出 */
    WT_CHECK(!WtYolo_LayoutFromChannels(4, 0, 0, &l));
    /* 关键点约定与通道数对不上（18 = 4 + 0 类 + 7×2）*/
    WT_CHECK(!WtYolo_LayoutFromChannels(18, 7, 2, &l));
}

/**
 * 1440×1080 → 1280×1280：缩放 0.8889、上下各留 160 行灰边。
 * 用一个独一无二的像素值追踪它落在画布的哪里。
 */
static void TestLetterboxGeometry(void)
{
    const int sw = 1440;
    const int sh = 1080;
    const int dw = 1280;
    const int dh = 1280;
    const int plane = dw * dh;
    unsigned char *rgb = (unsigned char *)calloc((size_t)sw * sh * 3, 1);
    float *dst = (float *)malloc(sizeof(float) * 3u * (size_t)plane);
    WtYoloLetterbox lb;
    const float gray = 114.0f / 255.0f;

    WT_CASE("letterbox：缩放比、填充量与像素落点");

    if (rgb == NULL || dst == NULL) {
        WT_CHECK(0);
        free(rgb);
        free(dst);
        return;
    }

    /*
     * 原图 (720, 540) 周围放一块 5×5 的特征色。用一块而不是一个像素：
     * 缩小时最近邻会跳过部分源列，单个像素是否被采到取决于浮点舍入，
     * 测的就不再是几何关系了。
     */
    {
        int x;
        int y;

        for (y = 538; y <= 542; y++) {
            for (x = 718; x <= 722; x++) {
                rgb[(y * sw + x) * 3 + 0] = 200;
                rgb[(y * sw + x) * 3 + 1] = 10;
                rgb[(y * sw + x) * 3 + 2] = 77;
            }
        }
    }

    WT_CHECK(WtYolo_Letterbox(rgb, sw, sh, dw, dh, WT_YOLO_RESIZE_NEAREST, dst, &lb));
    WT_CHECK_NEAR(lb.scale, 1280.0 / 1440.0, 1e-6);
    WT_CHECK_NEAR(lb.padX, 0.0, 1e-9);
    WT_CHECK_NEAR(lb.padY, 160.0, 1e-9);

    /* 灰边：第 0 行与最后一行 */
    WT_CHECK_NEAR(dst[0], gray, 1e-6);
    WT_CHECK_NEAR(dst[(dh - 1) * dw + 5], gray, 1e-6);
    WT_CHECK_NEAR(dst[2 * plane + 159 * dw + 640], gray, 1e-6);
    /* 图像区第一行不是灰边 */
    WT_CHECK_NEAR(dst[160 * dw + 0], 0.0, 1e-6);

    /*
     * 特征块：dst(dx,dy) 取 src(floor(dx/scale), floor(dy/scale))。
     * 画布 (640, 480) 对应原图约 (720, 540)，再加上 160 行顶部灰边。
     * 三个通道各自在自己的平面上。
     */
    WT_CHECK_NEAR(dst[(480 + 160) * dw + 640], 200.0 / 255.0, 1e-6);
    WT_CHECK_NEAR(dst[plane + (480 + 160) * dw + 640], 10.0 / 255.0, 1e-6);
    WT_CHECK_NEAR(dst[2 * plane + (480 + 160) * dw + 640], 77.0 / 255.0, 1e-6);

    WT_CASE("letterbox：640 输入（风机模型）与拆分前同一组参数");

    WT_CHECK(WtYolo_Letterbox(rgb, sw, sh, 640, 640, WT_YOLO_RESIZE_NEAREST, dst, &lb));
    WT_CHECK_NEAR(lb.scale, 640.0 / 1440.0, 1e-6);
    WT_CHECK_NEAR(lb.padX, 0.0, 1e-9);
    WT_CHECK_NEAR(lb.padY, 80.0, 1e-9);

    WT_CASE("letterbox：画布超过列下标表上限必须拒绝，而不是越界写");

    WT_CHECK(!WtYolo_Letterbox(rgb, sw, sh, WT_YOLO_MAX_INPUT + 1, 640,
                               WT_YOLO_RESIZE_NEAREST, dst, &lb));

    free(rgb);
    free(dst);
}

/**
 * 双线性：下标映射必须与 OpenCV INTER_LINEAR 一致（像素中心对齐），
 * 否则整张图会有半个像素的系统偏移 —— 对十几像素的行人就是好几个百分点
 * 的定位误差，而且看不出来。
 */
static void TestLetterboxBilinear(void)
{
    unsigned char src[8 * 4 * 3];
    unsigned char up[2 * 2 * 3];
    float dst[3 * 4 * 4];
    WtYoloLetterbox lb;
    int x;
    int y;
    int c;

    WT_CASE("双线性缩小：8×4 水平斜坡 → 4 宽，取值 = 20·dx + 5");

    /* 每列值 10·x，三通道相同 */
    for (y = 0; y < 4; y++) {
        for (x = 0; x < 8; x++) {
            for (c = 0; c < 3; c++) {
                src[(y * 8 + x) * 3 + c] = (unsigned char)(10 * x);
            }
        }
    }
    WT_CHECK(WtYolo_Letterbox(src, 8, 4, 4, 4, WT_YOLO_RESIZE_BILINEAR, dst, &lb));
    WT_CHECK_NEAR(lb.scale, 0.5, 1e-9);
    WT_CHECK_NEAR(lb.padY, 1.0, 1e-9);
    /* 源坐标 (dx+0.5)·2−0.5 = 2dx+0.5 → 列 2dx 与 2dx+1 各一半 */
    for (x = 0; x < 4; x++) {
        WT_CHECK_NEAR(dst[1 * 4 + x], (20.0 * x + 5.0) / 255.0, 1e-6);
        WT_CHECK_NEAR(dst[2 * 16 + 2 * 4 + x], (20.0 * x + 5.0) / 255.0, 1e-6);
    }
    /* 上下灰边不受插值影响 */
    WT_CHECK_NEAR(dst[0], 114.0 / 255.0, 1e-6);
    WT_CHECK_NEAR(dst[3 * 4 + 2], 114.0 / 255.0, 1e-6);

    WT_CASE("双线性放大：2×2 → 4×4，两端夹住、中间按 0.25/0.75 插值");

    for (y = 0; y < 2; y++) {
        for (c = 0; c < 3; c++) {
            up[(y * 2 + 0) * 3 + c] = 0;
            up[(y * 2 + 1) * 3 + c] = 100;
        }
    }
    WT_CHECK(WtYolo_Letterbox(up, 2, 2, 4, 4, WT_YOLO_RESIZE_BILINEAR, dst, &lb));
    WT_CHECK_NEAR(dst[0], 0.0, 1e-6);
    WT_CHECK_NEAR(dst[1], 25.0 / 255.0, 1e-6);
    WT_CHECK_NEAR(dst[2], 75.0 / 255.0, 1e-6);
    WT_CHECK_NEAR(dst[3], 100.0 / 255.0, 1e-6);

    WT_CASE("双线性：常数图缩放后仍是常数");

    memset(src, 50, sizeof(src));
    WT_CHECK(WtYolo_Letterbox(src, 8, 4, 4, 4, WT_YOLO_RESIZE_BILINEAR, dst, &lb));
    for (x = 0; x < 4; x++) {
        WT_CHECK_NEAR(dst[1 * 4 + x], 50.0 / 255.0, 1e-6);
        WT_CHECK_NEAR(dst[2 * 4 + x], 50.0 / 255.0, 1e-6);
    }
}

/**
 * VisDrone（11 类）：只要行人（0）与人群（1）。
 *
 *   候选 3：people 0.90               → 留下（还原后坐标逐项核对）
 *   候选 5：car 0.95                  → 类别不在接受集，丢
 *   候选 6：pedestrian 0.20           → 低于阈值，丢
 *   候选 7：pedestrian 0.80，与 3 重叠 → 不分类别的 NMS 压掉
 *   候选 2：pedestrian 0.60、bicycle 0.70 → argmax 是自行车，丢
 */
static void TestDecodeVisDrone(void)
{
    const int nc = 11;
    const int n = 8;
    const int ch = 4 + nc;
    float *t = NewTensor(ch, n);
    WtYoloLayout layout = {nc, 0, 0};
    WtYoloLetterbox lb = {0.5f, 0.0f, 160.0f};
    WtYoloFilter filter = {0.25f, 0.45f, (1u << 0) | (1u << 1)};
    WtYoloCand work[8];
    WtTrtResult res;

    WT_CASE("VisDrone：类别筛选、阈值、不分类别 NMS、坐标还原");

    if (t == NULL) {
        WT_CHECK(0);
        return;
    }

    SetBox(t, n, 3, 400.0f, 300.0f, 20.0f, 40.0f);
    t[(4 + 1) * n + 3] = 0.90f;

    SetBox(t, n, 5, 100.0f, 100.0f, 30.0f, 30.0f);
    t[(4 + 3) * n + 5] = 0.95f;

    SetBox(t, n, 6, 600.0f, 600.0f, 10.0f, 20.0f);
    t[(4 + 0) * n + 6] = 0.20f;

    SetBox(t, n, 7, 401.0f, 301.0f, 20.0f, 40.0f);
    t[(4 + 0) * n + 7] = 0.80f;

    SetBox(t, n, 2, 800.0f, 200.0f, 16.0f, 32.0f);
    t[(4 + 0) * n + 2] = 0.60f;
    t[(4 + 2) * n + 2] = 0.70f;

    WT_CHECK(WtYolo_Decode(t, ch, n, &layout, &lb, &filter, work, n, &res));
    WT_CHECK_EQ_INT(res.count, 1);
    if (res.count >= 1) {
        WT_CHECK_EQ_INT(res.items[0].classId, 1);
        WT_CHECK_NEAR(res.items[0].conf, 0.90, 1e-6);
        /* x1 = (400 − 10 − 0) / 0.5 = 780；y1 = (300 − 20 − 160) / 0.5 = 240 */
        WT_CHECK_NEAR(res.items[0].bboxX1, 780.0, 1e-3);
        WT_CHECK_NEAR(res.items[0].bboxY1, 240.0, 1e-3);
        WT_CHECK_NEAR(res.items[0].bboxX2, 820.0, 1e-3);
        WT_CHECK_NEAR(res.items[0].bboxY2, 320.0, 1e-3);
    }

    WT_CASE("classMask = 0 时全部类别都收");

    filter.classMask = 0u;
    WT_CHECK(WtYolo_Decode(t, ch, n, &layout, &lb, &filter, work, n, &res));
    /* 车 0.95、人群 0.90、自行车 0.70（候选 7 仍被候选 3 压掉） */
    WT_CHECK_EQ_INT(res.count, 3);
    if (res.count >= 3) {
        WT_CHECK_EQ_INT(res.items[0].classId, 3);
        WT_CHECK_EQ_INT(res.items[1].classId, 1);
        WT_CHECK_EQ_INT(res.items[2].classId, 2);
    }

    free(t);
}

/**
 * 风机模型（1 类 + 7 关键点 × 2）：关键点通道从 4 + nc = 5 开始。
 *
 * 这条是拆分前的硬编码 `5 + 2k`，拆分后由布局算出 —— 写成 `4 + 2k`
 * 或漏掉 nc 都会让每个关键点读到相邻通道。
 */
static void TestDecodeTurbineKeypoints(void)
{
    const int n = 4;
    const int ch = 19;
    float *t = NewTensor(ch, n);
    WtYoloLayout layout = {1, 7, 2};
    WtYoloLetterbox lb = {640.0f / 1440.0f, 0.0f, 80.0f};
    WtYoloFilter filter = {0.25f, 0.45f, 0u};
    WtYoloCand work[4];
    WtTrtResult res;
    int k;

    WT_CASE("风机：关键点通道 5+2k / 6+2k，并按 letterbox 还原");

    if (t == NULL) {
        WT_CHECK(0);
        return;
    }

    SetBox(t, n, 1, 320.0f, 320.0f, 200.0f, 200.0f);
    t[4 * n + 1] = 0.9f;
    for (k = 0; k < 7; k++) {
        t[(5 + 2 * k) * n + 1] = 100.0f + 10.0f * (float)k;
        t[(6 + 2 * k) * n + 1] = 200.0f + 10.0f * (float)k;
    }

    WT_CHECK(WtYolo_Decode(t, ch, n, &layout, &lb, &filter, work, n, &res));
    WT_CHECK_EQ_INT(res.count, 1);
    if (res.count == 1) {
        for (k = 0; k < 7; k++) {
            WT_CHECK_NEAR(res.items[0].kptX[k],
                          (100.0 + 10.0 * k - 0.0) / (640.0 / 1440.0), 1e-2);
            WT_CHECK_NEAR(res.items[0].kptY[k],
                          (200.0 + 10.0 * k - 80.0) / (640.0 / 1440.0), 1e-2);
        }
    }

    free(t);
}

/** 人群：超过容量时留下得分最高的那些，按得分降序 */
static void TestCapacity(void)
{
    const int nc = 1;
    const int n = WT_TRT_MAX_DETECTIONS + 20;
    const int ch = 4 + nc;
    float *t = NewTensor(ch, n);
    WtYoloCand *work = (WtYoloCand *)malloc(sizeof(WtYoloCand) * (size_t)n);
    WtYoloLayout layout = {nc, 0, 0};
    WtYoloLetterbox lb = {1.0f, 0.0f, 0.0f};
    WtYoloFilter filter = {0.25f, 0.45f, 0u};
    WtTrtResult res;
    int i;

    WT_CASE("容量：人群超过上限时按得分截取，不越界");

    if (t == NULL || work == NULL) {
        WT_CHECK(0);
        free(t);
        free(work);
        return;
    }

    /* 互不重叠的小框，得分随下标递增 */
    for (i = 0; i < n; i++) {
        SetBox(t, n, i, 20.0f * (float)i + 5.0f, 50.0f, 8.0f, 16.0f);
        t[4 * n + i] = 0.30f + 0.005f * (float)i;
    }

    WT_CHECK(WtYolo_Decode(t, ch, n, &layout, &lb, &filter, work, n, &res));
    WT_CHECK_EQ_INT(res.count, WT_TRT_MAX_DETECTIONS);
    WT_CHECK(res.items[0].conf > res.items[res.count - 1].conf);
    /* 最高分的是最后一个候选 */
    WT_CHECK_NEAR(res.items[0].bboxX1, 20.0 * (n - 1) + 5.0 - 4.0, 1e-3);

    WT_CASE("参数非法必须拒绝：通道数与布局不符、工作区不够");

    WT_CHECK(!WtYolo_Decode(t, ch + 1, n, &layout, &lb, &filter, work, n, &res));
    WT_CHECK_EQ_INT(res.count, 0);
    WT_CHECK(!WtYolo_Decode(t, ch, n, &layout, &lb, &filter, work, n - 1, &res));

    free(t);
    free(work);
}

int main(void)
{
    TestChannels();
    TestLetterboxGeometry();
    TestLetterboxBilinear();
    TestDecodeVisDrone();
    TestDecodeTurbineKeypoints();
    TestCapacity();

    return WT_TEST_SUMMARY();
}
