/**
 * @file wt_yolo_decode.c
 * @brief YOLO 预处理与输出解码实现。
 *
 * letterbox 与解码公式与拆分前 `wt_trt_pose.cpp` 里的实现逐项一致
 * （缩放比、lround、整除取填充量、最近邻下标），风机模型的结果因此不变；
 * 新增的只有「类别数可变」与「输入边长从 engine 读」。
 */

#include "wt_yolo_decode.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

int WtYolo_Channels(const WtYoloLayout *layout)
{
    if (layout == NULL || layout->numClasses < 1 ||
        layout->kptCount < 0 || layout->kptCount > WT_TRT_KPT_COUNT) {
        return 0;
    }
    if (layout->kptCount > 0 && layout->kptDims < 2) {
        return 0;
    }

    return 4 + layout->numClasses +
           (layout->kptCount > 0 ? layout->kptCount * layout->kptDims : 0);
}

bool WtYolo_LayoutFromChannels(int channels, int kptCount, int kptDims,
                               WtYoloLayout *out)
{
    WtYoloLayout l;

    if (out == NULL || kptCount < 0 || kptCount > WT_TRT_KPT_COUNT) {
        return false;
    }
    if (kptCount > 0 && kptDims < 2) {
        return false;
    }

    l.kptCount = kptCount;
    l.kptDims = kptDims;
    l.numClasses = channels - 4 - (kptCount > 0 ? kptCount * kptDims : 0);

    if (WtYolo_Channels(&l) != channels) {
        return false;
    }
    *out = l;

    return true;
}

/** 最近邻：下标公式与拆分前的 wt_trt_pose.cpp 完全相同 */
static void WtYolo_ResizeNearest(const unsigned char *rgb, int srcW, int srcH, float scale,
                                 int newW, int newH, int dstW, int padX, int padY,
                                 int plane, float *dst)
{
    int sxTable[WT_YOLO_MAX_INPUT];
    int dx;
    int dy;

    /*
     * 列下标每行都一样，先算一次 —— 逐像素做一次浮点除法在 1280 输入下是
     * 上百万次，这张表把它降到每帧一行。
     */
    for (dx = 0; dx < newW; dx++) {
        int sx = (int)((float)dx / scale);

        sxTable[dx] = (sx < srcW - 1) ? sx : (srcW - 1);
    }

    for (dy = 0; dy < newH; dy++) {
        int sy = (int)((float)dy / scale);
        int rowOut;
        const unsigned char *srcRow;

        if (sy > srcH - 1) {
            sy = srcH - 1;
        }
        rowOut = (dy + padY) * dstW + padX;
        srcRow = rgb + (size_t)sy * (size_t)srcW * 3u;

        for (dx = 0; dx < newW; dx++) {
            const unsigned char *px = srcRow + (size_t)sxTable[dx] * 3u;
            const int outIdx = rowOut + dx;

            dst[outIdx] = (float)px[0] / 255.0f;
            dst[plane + outIdx] = (float)px[1] / 255.0f;
            dst[2 * plane + outIdx] = (float)px[2] / 255.0f;
        }
    }
}

/**
 * 双线性：源坐标 = (目标下标 + 0.5) × 源宽 / 新宽 − 0.5，与 OpenCV INTER_LINEAR
 * 相同（ultralytics 的 letterbox 就是 cv2.resize 默认插值）。比例按各轴实际的
 * newW/srcW 取，而不是统一的 scale —— cv2 就是这么算的，取整后两者差一点点。
 */
static void WtYolo_ResizeBilinear(const unsigned char *rgb, int srcW, int srcH,
                                  int newW, int newH, int dstW, int padX, int padY,
                                  int plane, float *dst)
{
    int x0Table[WT_YOLO_MAX_INPUT];
    int x1Table[WT_YOLO_MAX_INPUT];
    float wxTable[WT_YOLO_MAX_INPUT];
    const float ratioX = (float)srcW / (float)newW;
    const float ratioY = (float)srcH / (float)newH;
    int dx;
    int dy;

    for (dx = 0; dx < newW; dx++) {
        float sx = ((float)dx + 0.5f) * ratioX - 0.5f;
        int x0;

        if (sx < 0.0f) {
            sx = 0.0f;
        }
        x0 = (int)sx;
        if (x0 >= srcW - 1) {
            x0Table[dx] = srcW - 1;
            x1Table[dx] = srcW - 1;
            wxTable[dx] = 0.0f;
        } else {
            x0Table[dx] = x0;
            x1Table[dx] = x0 + 1;
            wxTable[dx] = sx - (float)x0;
        }
    }

    for (dy = 0; dy < newH; dy++) {
        float sy = ((float)dy + 0.5f) * ratioY - 0.5f;
        int y0;
        int y1;
        float wy;
        const unsigned char *r0;
        const unsigned char *r1;
        const int rowOut = (dy + padY) * dstW + padX;

        if (sy < 0.0f) {
            sy = 0.0f;
        }
        y0 = (int)sy;
        if (y0 >= srcH - 1) {
            y0 = srcH - 1;
            y1 = srcH - 1;
            wy = 0.0f;
        } else {
            y1 = y0 + 1;
            wy = sy - (float)y0;
        }
        r0 = rgb + (size_t)y0 * (size_t)srcW * 3u;
        r1 = rgb + (size_t)y1 * (size_t)srcW * 3u;

        for (dx = 0; dx < newW; dx++) {
            const unsigned char *p00 = r0 + (size_t)x0Table[dx] * 3u;
            const unsigned char *p01 = r0 + (size_t)x1Table[dx] * 3u;
            const unsigned char *p10 = r1 + (size_t)x0Table[dx] * 3u;
            const unsigned char *p11 = r1 + (size_t)x1Table[dx] * 3u;
            const float wx = wxTable[dx];
            const int outIdx = rowOut + dx;
            int c;

            for (c = 0; c < 3; c++) {
                const float top = (float)p00[c] + ((float)p01[c] - (float)p00[c]) * wx;
                const float bot = (float)p10[c] + ((float)p11[c] - (float)p10[c]) * wx;

                dst[c * plane + outIdx] = (top + (bot - top) * wy) / 255.0f;
            }
        }
    }
}

bool WtYolo_Letterbox(const unsigned char *rgb, int srcW, int srcH,
                      int dstW, int dstH, E_WtYoloResize resize,
                      float *dst, WtYoloLetterbox *lb)
{
    const float fill = 114.0f / 255.0f;
    const int plane = dstW * dstH;
    float scale;
    int newW;
    int newH;
    int padX;
    int padY;
    int i;

    if (rgb == NULL || dst == NULL || lb == NULL || srcW <= 0 || srcH <= 0 ||
        dstW <= 0 || dstH <= 0 || dstW > WT_YOLO_MAX_INPUT || dstH > WT_YOLO_MAX_INPUT) {
        return false;
    }

    scale = (float)dstW / (float)srcW;
    if ((float)dstH / (float)srcH < scale) {
        scale = (float)dstH / (float)srcH;
    }
    newW = (int)lroundf((float)srcW * scale);
    newH = (int)lroundf((float)srcH * scale);
    /* 浮点误差不应让缩放后的图比画布还大 —— 那会越界写 */
    if (newW > dstW) {
        newW = dstW;
    }
    if (newH > dstH) {
        newH = dstH;
    }
    padX = (dstW - newW) / 2;
    padY = (dstH - newH) / 2;

    lb->scale = scale;
    lb->padX = (float)padX;
    lb->padY = (float)padY;

    /* 先填灰边（YOLO 训练时用 114/255） */
    for (i = 0; i < plane; i++) {
        dst[i] = fill;
        dst[plane + i] = fill;
        dst[2 * plane + i] = fill;
    }

    if (newW <= 0 || newH <= 0) {
        return true;    /* 源图极小，缩完不足一个像素：整张都是灰边 */
    }

    if (resize == WT_YOLO_RESIZE_BILINEAR) {
        WtYolo_ResizeBilinear(rgb, srcW, srcH, newW, newH, dstW, padX, padY, plane, dst);
    } else {
        WtYolo_ResizeNearest(rgb, srcW, srcH, scale, newW, newH, dstW, padX, padY, plane, dst);
    }

    return true;
}

static float WtYolo_Iou(const WtTrtDetection *a, const WtTrtDetection *b)
{
    const float x1 = (a->bboxX1 > b->bboxX1) ? a->bboxX1 : b->bboxX1;
    const float y1 = (a->bboxY1 > b->bboxY1) ? a->bboxY1 : b->bboxY1;
    const float x2 = (a->bboxX2 < b->bboxX2) ? a->bboxX2 : b->bboxX2;
    const float y2 = (a->bboxY2 < b->bboxY2) ? a->bboxY2 : b->bboxY2;
    const float w = x2 - x1;
    const float h = y2 - y1;
    float inter;
    float areaA;
    float areaB;
    float uni;

    if (w <= 0.0f || h <= 0.0f) {
        return 0.0f;
    }

    inter = w * h;
    areaA = (a->bboxX2 - a->bboxX1) * (a->bboxY2 - a->bboxY1);
    areaB = (b->bboxX2 - b->bboxX1) * (b->bboxY2 - b->bboxY1);
    uni = areaA + areaB - inter;

    return (uni > 0.0f) ? (inter / uni) : 0.0f;
}

/* 得分降序；同分按候选下标升序，让结果与平台的 qsort 实现无关 */
static int WtYolo_CandCompare(const void *pa, const void *pb)
{
    const WtYoloCand *a = (const WtYoloCand *)pa;
    const WtYoloCand *b = (const WtYoloCand *)pb;

    if (a->score > b->score) {
        return -1;
    }
    if (a->score < b->score) {
        return 1;
    }

    return (a->index > b->index) - (a->index < b->index);
}

bool WtYolo_Decode(const float *data, int channels, int numCand,
                   const WtYoloLayout *layout, const WtYoloLetterbox *lb,
                   const WtYoloFilter *filter,
                   WtYoloCand *work, int workCap, WtTrtResult *out)
{
    const float *ch;
    int nc;
    int n = 0;
    int i;
    int j;

    if (out != NULL) {
        out->count = 0;
    }
    if (data == NULL || layout == NULL || lb == NULL || filter == NULL ||
        work == NULL || out == NULL || numCand <= 0 || workCap < numCand ||
        lb->scale <= 0.0f) {
        return false;
    }
    if (WtYolo_Channels(layout) != channels) {
        return false;
    }

    nc = layout->numClasses;

    /* ---- 阈值 + 类别筛选 ---- */
    for (i = 0; i < numCand; i++) {
        float best = data[4 * numCand + i];
        int cls = 0;
        int c;

        for (c = 1; c < nc; c++) {
            const float s = data[(4 + c) * numCand + i];

            if (s > best) {
                best = s;
                cls = c;
            }
        }

        if (best < filter->confThresh) {
            continue;
        }
        /* 先 argmax 再筛类别（与 ultralytics 相同），见 WtYoloFilter 的说明 */
        if (filter->classMask != 0u &&
            (cls >= 32 || (filter->classMask & (1u << (unsigned)cls)) == 0u)) {
            continue;
        }

        work[n].score = best;
        work[n].index = i;
        work[n].classId = cls;
        n++;
    }

    qsort(work, (size_t)n, sizeof(work[0]), WtYolo_CandCompare);

    /* ---- 贪心 NMS（不分类别），只为留下的目标解关键点 ---- */
    for (j = 0; j < n && out->count < WT_TRT_MAX_DETECTIONS; j++) {
        const int idx = work[j].index;
        WtTrtDetection d;
        bool suppressed = false;
        float cx;
        float cy;
        float bw;
        float bh;
        int k;

        cx = data[0 * numCand + idx];
        cy = data[1 * numCand + idx];
        bw = data[2 * numCand + idx];
        bh = data[3 * numCand + idx];

        /* 从 letterbox 坐标还原到原图 */
        d.bboxX1 = (cx - bw * 0.5f - lb->padX) / lb->scale;
        d.bboxY1 = (cy - bh * 0.5f - lb->padY) / lb->scale;
        d.bboxX2 = (cx + bw * 0.5f - lb->padX) / lb->scale;
        d.bboxY2 = (cy + bh * 0.5f - lb->padY) / lb->scale;

        for (k = 0; k < out->count; k++) {
            if (WtYolo_Iou(&d, &out->items[k]) > filter->iouThresh) {
                suppressed = true;
                break;
            }
        }
        if (suppressed) {
            continue;
        }

        d.conf = work[j].score;
        d.classId = work[j].classId;

        for (k = 0; k < WT_TRT_KPT_COUNT; k++) {
            d.kptX[k] = 0.0f;
            d.kptY[k] = 0.0f;
        }
        for (k = 0; k < layout->kptCount; k++) {
            ch = data + (size_t)(4 + nc + k * layout->kptDims) * (size_t)numCand;
            d.kptX[k] = (ch[idx] - lb->padX) / lb->scale;
            d.kptY[k] = (ch[numCand + idx] - lb->padY) / lb->scale;
        }

        out->items[out->count++] = d;
    }

    return true;
}
