/**
 * @file lz_vision.h
 * @brief 视觉层：从一帧画面里找出国旗杆，给出**杆轴**的像素位置。
 *
 * 本层的**唯一职责**是回答"杆在画面里的哪里"，不做任何航线决策，
 * 也不做定位解算（那是把像素变成 WGS84 的事，本项目走激光直出经纬度，
 * 不需要它）。
 *
 * ⚠️ 本层**刻意不依赖 OpenCV**。国旗是极易分割的目标：高饱和的红色块
 * （旗面）配一根细长的杆，HSV 阈值 + 连通域即可，纯 C 约 400 行。
 * 保持零依赖有两层好处：
 *   1. WSL 与妙算3 都不必装 OpenCV（设备上**确实装了** 4.2，但结论不变）
 *   2. 视觉层与 lz_core 一样能在桌面上直接跑回归测试 ——
 *      测试数据是 6 张真实俯拍照片（`tests/data/` 下的 ppm）
 * 真需要特征匹配/深度模型时再引入 OpenCV 不迟。
 *
 * ## ⚠️ 检测的是**杆**，不是旗面的中心
 *
 * 早先的实现按"旗面外接框下边中点"给杆位。**这条规则在真实照片上是错的**
 * —— 实测 6 张俯拍照片（2026-09-24）：
 *
 * | 照片 | 旗 bbox 中心 x | 真实杆列 x | 偏差 |
 * |---|---|---|---|
 * | 1 | 668 | 647 | −21 px |
 * | 2 | 604 | 657 | **+52 px** |
 * | 3 | 676 | 629 | −47 px |
 * | 4 | 661 | 632 | −29 px |
 * | 5 | 667 | 626 | −41 px |
 * | 6 | 665 | 631 | −34 px |
 *
 * 偏差 **−3.7% ~ +4.1% 画面宽**（FOV 82° ⇒ 约 ±3.3°），而且**符号随风向翻转**
 * （风把旗吹向一侧时杆在另一侧）。这是"半个旗宽取决于风向"的必然结果 ——
 * **不是常数偏差，标定不掉**，半径越大放得越大。
 *
 * 所以现在是两级检测（见 `lz_vision.c` 的实现说明）：
 *   ① 红色连通域取最大块 = 旗面（高置信度的"这里有个杆状目标"标记）
 *   ② 在旗面附近的 ROI 内做**竖线对比度滤波** = 杆的精确像素列
 *
 * ⚠️ 另一个前提也变了（2026-09-24）：现场是"旗在杆顶、迎风向水平展开"，
 * 而原注释描述的"旗垂下来贴着杆"是另一种形态。两者红色块的形状与相对杆的
 * 位置完全不同 —— 本实现在前者上验证过。
 */

#ifndef LZ_VISION_H
#define LZ_VISION_H

#include "lz_target.h"

/** 一帧图像（本层自己的表示，不暴露任何视觉库类型给外部） */
typedef struct {
    const uint8_t *data;    /*!< 像素数据 */
    int width;
    int height;
    int channels;           /*!< 1 = 灰度，3 = RGB（顺序见下） */
    int stride;             /*!< 每行字节数，可能 != width * channels */
    bool isBgr;             /*!< 3 通道时的字节顺序：true = BGR，false = RGB */
    uint64_t frameId;       /*!< 供跨帧关联使用 */
} LzFrame;

/**
 * 视觉配置
 *
 * ⚠️ 这些默认值**必须与 `tools/gen_vision_testdata.js` 的 `CFG` 逐项一致**。
 * 不一致的话，那边生成的黄金值和这边跑出来的就对不上，而那种失配会被
 * 误读成"算法写错了"。
 */
typedef struct {
    /* ---- HSV 阈值：旗面的红色在色环两端各有一段 ---- */
    int redHueLowMax;       /*!< 低段红：H ∈ [0, redHueLowMax]，默认 10 */
    int redHueHighMin;      /*!< 高段红：H ∈ [redHueHighMin, 180]，默认 170 */
    int minSaturation;      /*!< 饱和度下限，默认 100（滤掉灰白背景） */
    int minValue;           /*!< 明度下限，默认 60（滤掉阴影） */

    /* ---- 连通域 ---- */
    int minBlobArea;        /*!< 连通域最小面积（像素），默认 200 */

    /* ---- 杆列检测（竖线对比度滤波）---- */
    int poleRoiMarginX;     /*!< 杆列 ROI：旗 bbox 左右各扩这么多像素，默认 70 */
    int poleContrastOffset; /*!< 对比度滤波的邻域偏移，默认 9 */
    int poleContrastThreshold; /*!< 与左右邻域的最小亮度差，默认 34 */
    int poleHalfWidth;      /*!< 判定"该行属于杆"时允许的 x 偏移，默认 1 */
    int poleGap;            /*!< 竖向延伸允许的最大间断（行），默认 40 */

    /* ---- 打分行窗（**裁剪不变性的关键**）----
     *
     * 杆列是谁，靠"这一列有多少行满足对比度判据"来投票。**投票窗口必须
     * 只取旗附近这一段**，不能取整幅图：
     *
     * 实测（2026-09-24）整幅图投票时，把同一张照片裁成小图会让某些列的
     * 票数变化，杆列判定因此**偏移 6 px** —— 判定结果依赖于"图有多大"，
     * 而不是"画面里有什么"。窗口收窄后，全图与裁剪图给出同一个答案（0 px）。
     *
     * 业务上也更对：杆的证据应当来自目标附近，画面底部的绿篱边缘、
     * 铺装接缝不该参与投票。
     */
    int poleWinAbove;       /*!< 投票窗口上界 = 旗 bbox 顶 - 这个值，默认 20 */
    int poleWinBelow;       /*!< 投票窗口下界 = 旗 bbox 顶 + 这个值，默认 300 */

    double minConfidence;   /*!< 低于此置信度的结果丢弃 */
} LzVisionConfig;

/** @brief 一套可用的默认配置（现场的实测参数） */
LzVisionConfig LzVision_DefaultConfig(void);

/** 视觉上下文（不透明；内部持有中间缓冲，故不放进头文件） */
typedef struct LzVision LzVision;

LzStatus LzVision_Init(const LzVisionConfig *config, LzVision **out);
void     LzVision_Deinit(LzVision *vision);

/**
 * @brief 在一帧里找出候选杆
 *
 * 输出填入 `pixel`（**杆轴的像素位置**）与 `confidence`；
 * `geo` / `heightM` / `radiusM` **留空**（本项目由激光给经纬度）。
 *
 * `pixel.u` 是**杆列**的归一化横坐标，不是旗面中心 —— 理由见文件头。
 * `pixel.topV` / `bottomV` 是**旗面**外接框的上下边（杆比旗长得多，
 * 用旗的框去估杆高会偏，故这个值只作调试用）。
 *
 * @param frame   [in]  一帧图像
 * @param targets [out] 检测结果；调用前需 LzTargetList_Init
 * @return `LZ_OK`；`LZ_ERR_NO_TARGET` = 没找到红色目标
 */
LzStatus LzVision_Detect(LzVision *vision, const LzFrame *frame, LzTargetList *targets);

/**
 * @brief 由像素框与标定参数估算杆高与半径
 *
 * 单独一个函数：它依赖相机内参与距离，与"怎么找到杆"是两回事。
 * 距离可由杆的像素宽度与先验直径反推，或由激光/双目给出。
 */
LzStatus LzVision_EstimateSize(const LzPixelBox *pixel, int frameW, int frameH,
                               double focalPx, double distanceM, double knownDiameterM,
                               double *outHeightM, double *outRadiusM);

#endif /* LZ_VISION_H */
