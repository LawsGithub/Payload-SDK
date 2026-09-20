/**
 * @file lz_vision.c
 * @brief 视觉层：HSV 阈值分割找国旗杆（**纯 C，零依赖**）。
 *
 * 管线（三步，每步都可独立测试）：
 *   1. RGB → HSV，按双区间红色阈值生成二值掩码
 *   2. 形态学开运算去噪（可关）
 *   3. 连通域标记，取面积最大的若干净块
 *
 * 拆成三步而不是写成一个大函数，是为了让中间结果可以被测试直接断言 ——
 * 阈值分割这类算法，肉眼调参是常态，能单独把掩码存下来看会省很多时间。
 */

#include "lz_vision.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

struct LzVision {
    LzVisionConfig config;
    int frameW;
    int frameH;
    uint8_t *mask;      /*!< 二值掩码，1 字节/像素 */
    size_t maskBytes;
};

/* ------------------------------------------------------------------ */
/* 色彩转换                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief RGB → HSV
 * @param h [0,180)（OpenCV 的约定：色相折半以塞进 1 字节）
 * @param s [0,255]，v [0,255]
 */
static void lz_rgb2hsv(int r, int g, int b, int *h, int *s, int *v)
{
    const int max = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
    const int min = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
    const int delta = max - min;

    *v = max;
    *s = (max == 0) ? 0 : (int)(255.0 * delta / max);

    if (delta == 0) {
        *h = 0;
        return;
    }

    double hue;
    if (max == r) {
        hue = 60.0 * ((g - b) / (double)delta);
    } else if (max == g) {
        hue = 60.0 * (2.0 + (b - r) / (double)delta);
    } else {
        hue = 60.0 * (4.0 + (r - g) / (double)delta);
    }
    if (hue < 0.0) {
        hue += 360.0;
    }
    /* 折半到 [0,180) */
    int hh = (int)(hue * 0.5);
    if (hh >= 180) {
        hh = 179;
    }
    *h = hh;
}

/* ------------------------------------------------------------------ */
/* 生命周期                                                            */
/* ------------------------------------------------------------------ */

LzStatus LzVision_Init(const LzVisionConfig *config, LzVision **out)
{
    if (config == NULL || out == NULL) {
        return LZ_ERR_PARAM;
    }
    if (config->redHueLowMax < 0 || config->redHueHighMin > 180 ||
        config->redHueLowMax >= config->redHueHighMin) {
        return LZ_ERR_PARAM;
    }

    LzVision *vision = calloc(1, sizeof(*vision));
    if (vision == NULL) {
        return LZ_ERR_IO;
    }
    vision->config = *config;
    *out = vision;
    return LZ_OK;
}

void LzVision_Deinit(LzVision *vision)
{
    if (vision == NULL) {
        return;
    }
    free(vision->mask);
    free(vision);
}

/* ------------------------------------------------------------------ */
/* 步骤 1：阈值分割                                                    */
/* ------------------------------------------------------------------ */

static bool lz_is_red(const LzVision *vision, int h, int s, int v)
{
    const LzVisionConfig *c = &vision->config;
    if (s < c->minSaturation || v < c->minValue) {
        return false;
    }
    /* 双区间：红色在色环上跨 0°，低段与高段要分别判 */
    return (h <= c->redHueLowMax) || (h >= c->redHueHighMin);
}

static LzStatus lz_ensure_mask(LzVision *vision, int w, int h)
{
    const size_t need = (size_t)w * (size_t)h;
    if (vision->maskBytes < need) {
        uint8_t *grown = realloc(vision->mask, need);
        if (grown == NULL) {
            return LZ_ERR_IO;
        }
        vision->mask = grown;
        vision->maskBytes = need;
    }
    vision->frameW = w;
    vision->frameH = h;
    return LZ_OK;
}

static LzStatus lz_build_mask(LzVision *vision, const LzFrame *frame)
{
    LzStatus st = lz_ensure_mask(vision, frame->width, frame->height);
    if (st != LZ_OK) {
        return st;
    }

    for (int y = 0; y < frame->height; ++y) {
        const uint8_t *row = frame->data + (size_t)y * frame->stride;
        uint8_t *m = vision->mask + (size_t)y * frame->width;

        for (int x = 0; x < frame->width; ++x) {
            int r, g, b;
            if (frame->channels == 1) {
                r = g = b = row[x];
            } else {
                const uint8_t *px = row + (size_t)x * frame->channels;
                if (frame->isBgr) {
                    b = px[0]; g = px[1]; r = px[2];
                } else {
                    r = px[0]; g = px[1]; b = px[2];
                }
            }
            int hh, ss, vv;
            lz_rgb2hsv(r, g, b, &hh, &ss, &vv);
            m[x] = lz_is_red(vision, hh, ss, vv) ? 1 : 0;
        }
    }
    return LZ_OK;
}

/* ------------------------------------------------------------------ */
/* 步骤 2/3 与检测主流程 —— 待实现                                      */
/* ------------------------------------------------------------------ */

LzStatus LzVision_Detect(LzVision *vision, const LzFrame *frame, LzTargetList *targets)
{
    if (vision == NULL || frame == NULL || targets == NULL) {
        return LZ_ERR_PARAM;
    }
    if (frame->data == NULL || frame->width <= 0 || frame->height <= 0) {
        return LZ_ERR_PARAM;
    }

    LzStatus st = lz_build_mask(vision, frame);
    if (st != LZ_OK) {
        return st;
    }

    /* -----------------------------------------------------------------------
     * 从掩码到目标 —— 实现要点（本轮先留空，待 lz_plan 的算法定下后再做）
     *
     * 掩码已经在 vision->mask 里（1 字节/像素，1 = 红色）。接下来：
     *
     *   步骤 2（可选）：形态学开运算去噪 —— 先腐蚀后膨胀，核尺寸
     *     config->openKernel，去掉零散噪点。0 表示跳过。
     *
     *   步骤 3：连通域标记（两遍扫描 + 并查集，或 BFS 泛洪），
     *     对每个连通域算：面积、外接框（minX/maxX/minY/maxY）、质心。
     *     面积 < config->minBlobArea 的丢弃。
     *
     *   然后判断哪些连通域是"旗面"：
     *     —— 旗面是一块大致成矩形的红色区域，其**左下角附近**连着细长的杆。
     *     本项目只要**一根杆的方位**，所以：取面积最大的连通域作为旗面，
     *     用它外接框的**下边中点**作为杆的像素位置。
     *
     *   填充 LzTarget：
     *     pixel.u      = 外接框水平中心 / frame->width
     *     pixel.v      = 外接框下边 / frame->height
     *     pixel.topV   = 外接框上边 / frame->height
     *     pixel.bottomV= 外接框下边 / frame->height
     *     confidence   = 由面积与矩形度（面积/外接框面积）组合出的 [0,1] 值
     *     geo / heightM / radiusM **留空**（由 lz_localize 与标定填）
     *     id           = 依次递增
     *
     * 用 lz_target.c 的 LzTargetList_Push() 追加（已有测试覆盖）。
     *
     * 注意：confidence 要真的能区分好坏 —— 一个占满画面的巨大红块
     * （比如红旗占满视场）不该得高分。想想什么指标最能反映"这是一面
     * 挂在杆上的旗"，而不是"画面里有一片红"。
     * ----------------------------------------------------------------------- */

    (void)lz_is_red;

    return LZ_ERR_UNSUPPORTED;
}

LzStatus LzVision_EstimateSize(const LzPixelBox *pixel, int frameW, int frameH,
                               double focalPx, double distanceM, double knownDiameterM,
                               double *outHeightM, double *outRadiusM)
{
    if (pixel == NULL || outHeightM == NULL || outRadiusM == NULL) {
        return LZ_ERR_PARAM;
    }
    if (frameW <= 0 || frameH <= 0 || !(focalPx > 0.0) || !(distanceM > 0.0)) {
        return LZ_ERR_PARAM;
    }
    if (!isfinite(pixel->topV) || !isfinite(pixel->bottomV)) {
        return LZ_ERR_PARAM;
    }

    /* 小孔成像：物体在画面上的像素高度 h_px 与实物高度 H 的关系是
     *     h_px / focalPx = H / distance
     * 于是 H = h_px * distance / focalPx。
     * 像素高度取归一化框高乘以画面高 —— 用归一化量而非像素量，
     * 是为了让调用方不必关心画面的绝对分辨率。 */
    const double pixelHeight = fabs(pixel->bottomV - pixel->topV) * (double)frameH;

    *outHeightM = pixelHeight * distanceM / focalPx;

    /* 半径由先验直径给出；未知则记 0（绕飞取中心，半径仅用于避让） */
    *outRadiusM = (knownDiameterM > 0.0) ? (knownDiameterM * 0.5) : 0.0;

    return LZ_OK;
}
