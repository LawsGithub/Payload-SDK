/**
 * @file lz_vision.c
 * @brief 视觉层：HSV 阈值 + 连通域找旗面，再用竖线对比度滤波定杆列（**纯 C，零依赖**）。
 *
 * ## 三级管线，每级都可独立测试
 *
 *   1. RGB → HSV，按双区间红色阈值生成二值掩码
 *   2. 连通域标记（两遍扫描 + 并查集，8 邻域），取面积最大的合格块 = 旗面
 *   3. 在旗面附近的 ROI 内做**竖线对比度滤波**，定出杆的像素列
 *
 * 拆成三级而不是一个大函数，是为了让中间结果能被测试直接断言 ——
 * 阈值分割这类算法，肉眼调参是常态，能单独把掩码存下来看会省很多时间。
 *
 * ## 为什么第 3 级是必需的（这一步是实测逼出来的）
 *
 * 最朴素的做法是"取旗面外接框的中点当杆位"。**在真实照片上这是错的**：
 * 实测 6 张俯拍照片（2026-09-24），旗 bbox 中心与真实杆列相差
 * **−3.7% ~ +4.1% 画面宽**，而且**符号随风向翻转** —— 风把旗吹向一侧时
 * 杆在旗的另一侧。半个旗宽随风向变化，因此**这不是常数偏差，标定不掉**，
 * 半径越大放得越大。
 *
 * 杆本身是可以直接检测的：它是一根细长竖线，压在绿篱/水面上时比周围**亮**，
 * 压在浅色铺装上时比周围**暗**。两个方向都算，就不依赖背景明暗，也不需要
 * 知道杆是什么颜色（实测现场杆是浅灰色）。实测跨 54 组参数
 * （3 种邻域 × 3 种阈值 × 6 张图）杆列位置只动 0–2 px。
 *
 * ## 两个前提，都是现场确认过的
 *
 * - 现场形态是**旗在杆顶、迎风向水平展开**（不是"垂下来贴着杆"）
 * - 天气良好、俯拍 —— 阴天与逆光**未覆盖**。HSV 阈值在阴天的表现是这套
 *   算法的公认弱点，好在失败可解释（调 `minSaturation`），
 *   这正是选它而不用模型的原因
 */

#include "lz_vision.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/** 一个连通域的统计量。**内部类型**，不出现在 `lz_vision.h` 里 ——
 *  外部只该拿到 `LzTarget`，不该看见"连通域"这个概念。 */
typedef struct {
    int area;
    int minX, maxX, minY, maxY;
} LzBlobStat;

struct LzVision {
    LzVisionConfig config;
    int frameW;
    int frameH;
    uint8_t *mask;      /*!< 二值掩码，1 字节/像素 */
    size_t maskBytes;
    int32_t *label;     /*!< 连通域等价类，-1 = 非目标 */
    int32_t *parent;    /*!< 并查集父指针（下标 0 是哨兵，不用） */
    LzBlobStat *stat;   /*!< 按根编号索引的面积统计 */
};

LzVisionConfig LzVision_DefaultConfig(void)
{
    /* 取值依据：6 张真实俯拍照片的红色像素分布（16966 个样本）
     *   H: p5=5.6  p50=172.9  p95=176.6   —— 双峰清晰
     *   S: p5=129  p50=224    p95=249
     *   V: p5=170  p50=228    p95=254
     * 低段(H<=10) 占 8.2%，高段(H>=170) 占 91.8%。
     * 阈值取得比 p5 更宽松（S=100 / V=60）是**刻意的**：留出阴天的余量，
     * 代价是偶尔多几个小红块 —— 而"取面积最大"会把它滤掉。 */
    LzVisionConfig c;
    c.redHueLowMax = 10;
    c.redHueHighMin = 170;
    c.minSaturation = 100;
    c.minValue = 60;
    c.minBlobArea = 200;
    c.poleRoiMarginX = 70;
    c.poleContrastOffset = 9;
    c.poleContrastThreshold = 34;
    c.poleHalfWidth = 1;
    c.poleGap = 40;
    c.poleWinAbove = 20;
    c.poleWinBelow = 300;
    c.minConfidence = 0.5;
    return c;
}

/* ------------------------------------------------------------------ */
/* 色彩转换                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief RGB → HSV
 * @param h [0,180)（色相折半以塞进 1 字节，与 OpenCV 同约定）
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
    /* 杆列检测的参数必须自洽，否则会算出越界的内存下标：
     *   poleContrastOffset >= 1    邻域偏移为 0 时"与左右比较"恒等
     *   poleHalfWidth / poleGap >= 0
     *   poleRoiMarginX >= 0        ROI 边距为负会让 ROI 反向
     *   poleWin* >= 0              投票窗口为负会让窗口反向
     * 这些是**入参校验，不是防御性编程** —— Init 是唯一能拦住它们的
     * 地方，之后每次 Detect 都会拿这些值去算下标。 */
    if (config->poleContrastOffset < 1 || config->poleHalfWidth < 0 ||
        config->poleGap < 0 || config->poleRoiMarginX < 0 ||
        config->poleWinAbove < 0 || config->poleWinBelow < 0 ||
        config->poleContrastThreshold < 0) {
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
    free(vision->label);
    free(vision->parent);
    free(vision->stat);
    free(vision);
}

/**
 * @brief 保证所有中间缓冲都能装下 w×h 的帧
 *
 * 缓冲放进上下文复用，而不是每次 Detect 现 malloc：取图回调以 30 fps 的频率
 * 调它，每帧几块 w×h 的分配释放会让堆抖动，而设备（妙算3）内存并不宽裕。
 */
static LzStatus lz_ensure_buffers(LzVision *vision, int w, int h)
{
    const size_t need = (size_t)w * (size_t)h;
    if (vision->maskBytes >= need) {
        vision->frameW = w;
        vision->frameH = h;
        return LZ_OK;
    }
    /* 三块一起长：任一块失败就整批清空，避免出现"mask 有了但 label 没有"
     * 这种半新半旧的状态 —— 那会让下一次 Detect 越界写。
     * need+1 是给并查集的父指针留的哨兵位（下标 0 不使用）。 */
    void *m  = realloc(vision->mask, need);
    void *l  = realloc(vision->label, need * sizeof(int32_t));
    void *p  = realloc(vision->parent, (need + 1) * sizeof(int32_t));
    void *st = realloc(vision->stat, (need + 1) * sizeof(LzBlobStat));
    if (m == NULL || l == NULL || p == NULL || st == NULL) {
        free(m); free(l); free(p); free(st);
        vision->mask = NULL; vision->label = NULL;
        vision->parent = NULL; vision->stat = NULL;
        vision->maskBytes = 0;
        return LZ_ERR_IO;
    }
    vision->mask = m;
    vision->label = l;
    vision->parent = p;
    vision->stat = st;
    vision->maskBytes = need;
    vision->frameW = w;
    vision->frameH = h;
    return LZ_OK;
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

static LzStatus lz_build_mask(LzVision *vision, const LzFrame *frame)
{
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
/* 步骤 2：连通域标记                                                  */
/* ------------------------------------------------------------------ */

/**
 * @brief 两遍扫描 + 并查集，8 邻域
 *
 * ## 为什么取"面积最大"而不是"最像旗"
 *
 * 实测这一条足够稳：6 张真实照片上第一名与第二名的面积比是
 * **7.9x ~ 103x**（第二名全是红色电动车、红衣路人）。用"最大"而不是
 * 引入矩形度之类的形状判据，是因为**少一个自由度就少一处会在现场失配的
 * 地方** —— 形状判据需要定阈值，而阈值在阴天/逆光下要不要跟着变没人知道。
 *
 * ## 平局怎么定
 *
 * 面积相同时取**根编号小的**（= 更早遇到的）。JS 参考实现用稳定排序，
 * 效果等价。这不是随便定的：不确定的平局会让同一张图两次跑出不同结果，
 * 而黄金值测试恰恰靠"结果确定"才有意义。
 */
static void lz_label_and_measure(LzVision *vision, LzBlobStat *outBest)
{
    const int w = vision->frameW, h = vision->frameH;
    const size_t n = (size_t)w * (size_t)h;
    const uint8_t *m = vision->mask;
    int32_t *label = vision->label;
    int32_t *parent = vision->parent;

    for (size_t i = 0; i < n; ++i) {
        label[i] = -1;
    }
    parent[0] = 0;
    int32_t next = 1;

    /* ---- 第一遍：给每个红色像素打标并合并等价类 ---- */
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const size_t i = (size_t)y * w + x;
            if (!m[i]) {
                continue;
            }
            /* 8 邻域里"已处理过"的四个：左、上、左上、右上 */
            int32_t nb[4];
            int nbCount = 0;
            if (x > 0 && m[i - 1])             { nb[nbCount++] = label[i - 1]; }
            if (y > 0) {
                if (m[i - w])                  { nb[nbCount++] = label[i - w]; }
                if (x > 0 && m[i - w - 1])     { nb[nbCount++] = label[i - w - 1]; }
                if (x < w - 1 && m[i - w + 1]) { nb[nbCount++] = label[i - w + 1]; }
            }
            if (nbCount == 0) {
                parent[next] = next;
                label[i] = next++;
            } else {
                int32_t root = nb[0];
                for (int k = 1; k < nbCount; ++k) {
                    if (nb[k] < root) {
                        root = nb[k];
                    }
                }
                label[i] = root;
                /* 把其余等价类的根都挂到 root 的根下。
                 * ⚠️ 必须先各自 find 到**各自的根**再 union —— 直接写
                 * parent[nb[k]] = root 是错的：nb[k] 可能不是根，
                 * 那样会切断它下面的一串像素，产生虚假的小连通域。 */
                int32_t rb = root;
                while (parent[rb] != rb) { rb = parent[rb]; }
                for (int k = 0; k < nbCount; ++k) {
                    int32_t a = nb[k];
                    while (parent[a] != a) { a = parent[a]; }
                    if (a != rb) {
                        parent[a] = rb;
                    }
                }
            }
        }
    }

    /* ---- 路径压缩：把每个像素的 label 换成它所属根的编号 ---- */
    for (size_t i = 0; i < n; ++i) {
        if (!m[i]) {
            continue;
        }
        int32_t r = label[i];
        while (parent[r] != r) {
            r = parent[r];
        }
        label[i] = r;
    }

    /* ---- 归并统计：按根累加面积与外接框 ---- */
    for (int32_t r = 0; r <= next; ++r) {
        vision->stat[r].area = 0;
        vision->stat[r].minX = w;
        vision->stat[r].maxX = -1;
        vision->stat[r].minY = h;
        vision->stat[r].maxY = -1;
    }
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const size_t i = (size_t)y * w + x;
            if (!m[i]) {
                continue;
            }
            LzBlobStat *s = &vision->stat[label[i]];
            s->area++;
            if (x < s->minX) { s->minX = x; }
            if (x > s->maxX) { s->maxX = x; }
            if (y < s->minY) { s->minY = y; }
            if (y > s->maxY) { s->maxY = y; }
        }
    }

    /* ---- 取面积最大者。平局取根编号小的（= 更早遇到的）——
     * 用严格大于，所以先出现的胜出。确定性的平局规则是黄金值测试的前提。 */
    LzBlobStat best = { 0, 0, 0, 0, 0 };
    for (int32_t r = 1; r < next; ++r) {
        if (vision->stat[r].area > best.area) {
            best = vision->stat[r];
        }
    }
    *outBest = best;
}

/* ------------------------------------------------------------------ */
/* 步骤 3：杆列检测（竖线对比度滤波）                                   */
/* ------------------------------------------------------------------ */

/** 三通道里的最大分量（用来做与颜色无关的亮度代理） */
static int lz_max3(const uint8_t *px, int channels)
{
    if (channels == 1) {
        return px[0];
    }
    int v = px[0];
    if (px[1] > v) { v = px[1]; }
    if (channels >= 3 && px[2] > v) { v = px[2]; }
    return v;
}

/**
 * @brief 在旗面附近的 ROI 内找出杆的像素列
 *
 * ## 判据
 *
 * 杆是一根细长竖线，与左右各 `poleContrastOffset` 像素处的亮度都差
 * `poleContrastThreshold` 以上 —— 压在绿篱/水面上时比周围亮，压在浅色
 * 铺装上时比周围暗。**两个方向都算**，所以不依赖背景明暗。
 *
 * ## 投票窗口为什么不能取整幅图
 *
 * 一列是不是杆，靠"这一列有多少行满足判据"来投票。若在整幅图上投票，
 * 画面底部的绿篱边缘、铺装接缝都会投票，而且**结果依赖于图的尺寸**：
 * 实测把同一张照片裁成小图会让某些列的票数变化，杆列判定因此偏移 6 px。
 * 收窄到旗附近后，全图与裁剪图给出同一答案（0 px）——
 * **判定应当依赖于"画面里有什么"，而不是"图有多大"。**
 *
 * @param outVotes [out] 最高票数；调用方拿它折算置信度
 * @return 杆列 x。找不到时返回旗 bbox 中心（退化值，由 confidence 反映）
 */
static int lz_detect_pole_column(const LzVision *vision, const LzFrame *frame,
                                 const LzBlobStat *flag, int *outVotes)
{
    const LzVisionConfig *c = &vision->config;
    const int w = frame->width, h = frame->height;
    const int off = c->poleContrastOffset;

    /* ROI 与投票窗口都要留出 ±off 的余量，否则读邻域时越界 */
    int x0 = flag->minX - c->poleRoiMarginX;
    int x1 = flag->maxX + c->poleRoiMarginX;
    if (x0 < off)          { x0 = off; }
    if (x1 > w - 1 - off)  { x1 = w - 1 - off; }
    int wy0 = flag->minY - c->poleWinAbove;
    int wy1 = flag->minY + c->poleWinBelow;
    if (wy0 < 0)           { wy0 = 0; }
    if (wy1 > h - 1 - off) { wy1 = h - 1 - off; }

    if (x0 > x1 || wy0 > wy1) {
        *outVotes = 0;
        return (flag->minX + flag->maxX) / 2;
    }

    int bestX = x0, bestVotes = -1;
    for (int x = x0; x <= x1; ++x) {
        int votes = 0;
        for (int y = wy0; y <= wy1; ++y) {
            const uint8_t *row = frame->data + (size_t)y * frame->stride;
            const int v = lz_max3(row + (size_t)x * frame->channels, frame->channels);
            const int l = lz_max3(row + (size_t)(x - off) * frame->channels, frame->channels);
            const int r = lz_max3(row + (size_t)(x + off) * frame->channels, frame->channels);
            if ((v - l > c->poleContrastThreshold && v - r > c->poleContrastThreshold) ||
                (l - v > c->poleContrastThreshold && r - v > c->poleContrastThreshold)) {
                votes++;
            }
        }
        /* 严格大于：平局取更靠左的列。确定性的平局规则是黄金值测试的前提 */
        if (votes > bestVotes) {
            bestVotes = votes;
            bestX = x;
        }
    }

    *outVotes = bestVotes;
    return bestX;
}

/**
 * @brief 杆的竖向延伸：从杆列往下找最长的一段连续命中
 *
 * 这个值**不参与定位**（杆列的 x 才是定位量），它只有两个用途：
 * 一是算置信度（只命中几行的"杆"多半是绿篱边缘），
 * 二是给上层一个"杆有多长"的直观量，用于判断检测是否合理。
 */
static void lz_pole_extent(const LzVision *vision, const LzFrame *frame,
                           int poleX, int *outTop, int *outBottom)
{
    const LzVisionConfig *c = &vision->config;
    const int w = frame->width, h = frame->height;
    const int off = c->poleContrastOffset;

    int bestTop = -1, bestBottom = -1, bestLen = 0;
    int start = -1, last = -1;

    for (int y = 0; y < h; ++y) {
        bool hit = false;
        for (int k = -c->poleHalfWidth; k <= c->poleHalfWidth && !hit; ++k) {
            const int xx = poleX + k;
            if (xx < off || xx >= w - off) {
                continue;
            }
            const uint8_t *row = frame->data + (size_t)y * frame->stride;
            const int v = lz_max3(row + (size_t)xx * frame->channels, frame->channels);
            const int l = lz_max3(row + (size_t)(xx - off) * frame->channels, frame->channels);
            const int r = lz_max3(row + (size_t)(xx + off) * frame->channels, frame->channels);
            if ((v - l > c->poleContrastThreshold && v - r > c->poleContrastThreshold) ||
                (l - v > c->poleContrastThreshold && r - v > c->poleContrastThreshold)) {
                hit = true;
            }
        }
        if (hit) {
            if (start < 0) {
                start = y;
            }
            last = y;
        } else if (start >= 0 && y - last > c->poleGap) {
            if (last - start > bestLen) {
                bestLen = last - start;
                bestTop = start;
                bestBottom = last;
            }
            start = -1;
        }
    }
    if (start >= 0 && last - start > bestLen) {
        bestTop = start;
        bestBottom = last;
    }

    *outTop = bestTop;
    *outBottom = bestBottom;
}

/* ------------------------------------------------------------------ */
/* 检测主流程                                                          */
/* ------------------------------------------------------------------ */

/**
 * @brief 由两级检测的证据折算置信度 [0,1]
 *
 * ## 为什么要真的能区分好坏
 *
 * 一个占满画面的巨大红块（比如镜头被一面红旗糊住）**不该得高分** ——
 * 那时连杆在哪都无从谈起。反过来，旗小但杆清楚，是完全可以用的。
 *
 * 所以置信度取两者的**较小值**，任一项差就压低整体：
 *
 *   旗面证据 = 面积落在 [minBlobArea, 5% 画面] 内 —— 太大太小都扣分
 *   杆的证据 = 杆列票数 / 投票窗口行数（命中率）
 *
 * ⚠️ **不做形状判据**（矩形度之类）：那需要再定一批阈值，而阈值在
 * 阴天/逆光下该不该跟着变没有依据。宁可少一个自由度。
 */
static double lz_confidence(const LzVision *vision, const LzFrame *frame,
                            const LzBlobStat *flag, int poleVotes)
{
    (void)vision;
    const double frameArea = (double)frame->width * (double)frame->height;
    const double frac = (double)flag->area / frameArea;

    /* 旗面：小于 minBlobArea 的已经在候选里滤过，所以这里只惩罚"过大"。
     * 5% 是现场尺度下的经验值：画面里那面旗大约占 0.14%–0.44%
     * （实测 6 张图），留一个数量级余量。 */
    double flagScore = 1.0;
    if (frac > 0.05) {
        flagScore = 0.05 / frac;      /* 糊满画面 → 迅速掉分 */
    } else if (frac < 0.0005) {
        flagScore = frac / 0.0005;    /* 太小 → 线性掉分 */
    }

    /* 杆的证据：**这根竖线有多长**。
     *
     * ## 分母为什么是"画面高度的 25%"而不是投票窗口的行数
     *
     * 投票窗口总是比杆长 —— 它要为旗**下方**那段杆留余量（现场实测杆的
     * 可见长度是旗高的两倍以上）。拿窗口行数当分母，量到的是
     * "杆长 / 窗口长"，那是个与取景方式有关的量，不是"这是不是一根杆"。
     *
     * 更具移植性的判据是"杆至少占了画面四分之一的高度"：那是一条
     * **看得见的杆**的物理下限，与分辨率、裁剪方式都无关。
     *
     * ⚠️ 本函数早期版本用的是"票数 / 配置里的名义窗口行数"再乘 2 ——
     * 那个 2 其实就是在把分母折半（321 → 160），只是没有说出来。
     * 现在把"满分行数"写成显式的量，理由才看得出来。 */
    const double fullMarkRows = (double)frame->height * 0.25;
    double poleScore = (fullMarkRows > 0.0) ? (double)poleVotes / fullMarkRows : 0.0;
    if (poleScore > 1.0) {
        poleScore = 1.0;
    }

    return (flagScore < poleScore) ? flagScore : poleScore;
}

LzStatus LzVision_Detect(LzVision *vision, const LzFrame *frame, LzTargetList *targets)
{
    if (vision == NULL || frame == NULL || targets == NULL) {
        return LZ_ERR_PARAM;
    }
    if (frame->data == NULL || frame->width <= 0 || frame->height <= 0) {
        return LZ_ERR_PARAM;
    }
    if (frame->channels != 1 && frame->channels != 3) {
        return LZ_ERR_UNSUPPORTED;   /* NV12 等格式请先转换，见文件头 */
    }

    LzStatus st = lz_ensure_buffers(vision, frame->width, frame->height);
    if (st != LZ_OK) {
        return st;
    }
    st = lz_build_mask(vision, frame);
    if (st != LZ_OK) {
        return st;
    }

    /* ---- 步骤 2：连通域 ---- */
    LzBlobStat flag;
    lz_label_and_measure(vision, &flag);
    if (flag.area < vision->config.minBlobArea) {
        /* 没有够大的红色块 —— 可能根本没有旗，也可能阈值把它滤掉了。
         * 两种情况上层都只能"重新瞄准"，所以不细分。 */
        return LZ_ERR_NO_TARGET;
    }

    /* ---- 步骤 3：杆列 ---- */
    int poleVotes = 0;
    const int poleX = lz_detect_pole_column(vision, frame, &flag, &poleVotes);

    int poleTop = -1, poleBottom = -1;
    lz_pole_extent(vision, frame, poleX, &poleTop, &poleBottom);
    (void)poleTop;
    (void)poleBottom;

    const double conf = lz_confidence(vision, frame, &flag, poleVotes);
    if (conf < vision->config.minConfidence) {
        return LZ_ERR_NO_TARGET;
    }

    LzTarget t;
    memset(&t, 0, sizeof(t));
    t.id = 1;
    /* geo / heightM / radiusM 留空 —— 本项目由激光直出经纬度，
     * 视觉只回答"杆在画面里的哪里"。填一个假坐标比留空更危险：
     * 下游无从分辨它是真解算出来的还是占位值。 */
    t.heightM = -1.0;
    t.radiusM = -1.0;
    /* ⚠️ pixel.u 是**杆列**的归一化横坐标，不是旗面中心。
     * 实测两者相差 −3.7% ~ +4.1% 画面宽且随风向变号，见文件头。 */
    t.pixel.u = (double)poleX / (double)frame->width;
    t.pixel.v = (double)flag.maxY / (double)frame->height;
    t.pixel.topV = (double)flag.minY / (double)frame->height;
    t.pixel.bottomV = (double)flag.maxY / (double)frame->height;
    t.confidence = conf;

    return LzTargetList_Push(targets, &t);
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
