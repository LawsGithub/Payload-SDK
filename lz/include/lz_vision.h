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

/**
 * @brief 当前编译进来的是哪个后端（`"hsv"` 或 `"stub"`）
 *
 * ⚠️ **两个后端都必须实现它**（返回各自的字符串）。
 *
 * 它原先只在 `lz_vision_stub.c` 里定义、也只在那里就地声明，
 * 而 `lz_vision.c` **根本没有这个函数**、头文件里也没有原型。
 * 后果：任何想打印后端名的调用方**只在 stub 后端下编得过**，
 * 切到 hsv 就是 `implicit declaration` + 链接失败
 * （2026-09-27 写 `lz_vision_probe` 时踩到）。
 *
 * 与 `LzVision_DefaultConfig` 是**同一个形状**：头文件按"全集"声明，
 * 而两个后端实现的函数集不一致。**凡是头文件声明的，两个后端都要给。**
 */
const char *LzVision_BackendName(void);

/** 视觉上下文（不透明；内部持有中间缓冲，故不放进头文件） */
typedef struct LzVision LzVision;

LzStatus LzVision_Init(const LzVisionConfig *config, LzVision **out);
void     LzVision_Deinit(LzVision *vision);

/* ------------------------------------------------------------------ */
/* 像素 ↔ 角度：视觉调俯仰用的纯数学（与检测算法无关，故独立成函数）      */
/* ------------------------------------------------------------------ */

/**
 * @brief 由**对角**视场角算出垂直视场角
 *
 * ## 为什么需要单独一个函数，而不能直接把规格值当垂直 FOV 用
 *
 * 规格页写的 `FOV: 82°` 是**对角**视场角（DFOV），不是水平的、更不是垂直的。
 * 拿它当水平 FOV 用会错 **11°**（下面有算式）。这类"看着差不多"的错误
 * 正是本项目反复强调要防的形状 —— 而它在画面上只表现为"杆偏了一点"。
 *
 * 小孔成像下三条视场角满足同一个焦距：
 *
 *     tan(HFOV/2) = (W/2)/f
 *     tan(VFOV/2) = (H/2)/f
 *     tan(DFOV/2) = (D/2)/f        D = √(W² + H²)
 *  ⇒ tan(VFOV/2) = tan(DFOV/2) · H / D
 *
 * ## 实例（4:3，正好是 M4T 的 8064×6048）
 *
 * | 输入 | 结果 |
 * |---|---|
 * | DFOV 82°、4:3 | **55.1°** ✅ |
 * | 把 82° 当水平 FOV：`tan(41°)·3/4` | 66.2° ❌ 差 11° |
 *
 * ⚠️ **宽高比必须按实测的 `frameW/frameH` 现算**，不能用规格里的 4:3 ——
 * 取到的图像流未必是 4:3（`DjiLiveview_StartImageStream` 的分辨率
 * 未见文档说明，见 `doc/VISION-GIMBAL-PITCH.md` §9 #5）。
 *
 * ⚠️ **本函数与"取到的是哪个镜头"无关** —— DFOV 是入参。
 * 所以在探针答出"广角 82° / 中长焦 35° / 长焦 15°"之前，
 * 本函数就能用规格值在桌面上测。
 *
 * @param frameW/frameH 画面像素尺寸，都必须 > 0
 * @param diagFovDeg    对角视场角，度。必须落在 (0, 180)
 * @return 垂直视场角（度）；入参非法时返回 NAN
 */
double LzVision_VerticalFovDeg(int frameW, int frameH, double diagFovDeg);

/**
 * @brief 把"目标在画面里的纵向偏差"折算成"云台需要转多少度"
 *
 * ## 用法：从"现在指向哪"推到"该指向哪"
 *
 *     thetaStar = thetaNow + LzVision_PixelOffsetToDeg(vMid, VFOV)
 *
 * 其中 `thetaNow` 必须是**测这一帧时云台的实际俯仰**（读回的值，
 * 不是我们下发的值 —— issue #555/#563 说明"下发了 ≠ 执行了"）。
 *
 * ## 符号约定（画面坐标 → 俯仰角）
 *
 * 画面 `v` 向下为正（0 = 上缘，1 = 下缘），俯仰角向下为负。
 * 于是 `vMid > 0.5`（目标在画面**偏下**）意味着"光轴比目标**低**了"，
 * 需要**下压**，即俯仰更负 ⇒ 本函数返回**负值**。反之目标偏上 ⇒ 返回正值（上仰）。
 *
 * ## 为什么用 `atan` 而不是线性近似
 *
 *     deltaDeg = -atan( (v - 0.5) · 2 · tan(VFOV/2) ) · 180/π
 *
 * 匀角展开（`(v-0.5)·VFOV`）只在中心附近成立。⚠️ **实测误差不是"边缘最大"
 * 而是出现在 |v−0.5| ≈ 0.28 处**（即 v ≈ 0.22 与 v ≈ 0.78 两个对称点，
 * 因为误差只取决于 `|2v−1|`）—— 在 **v = 1.0（画面边缘）两者恰好相等**，
 * 都等于半个 VFOV。这一点反直觉，所以写下来：
 *
 * | 垂直 FOV | 线性近似最大误差 | 出现在 | 12.5 m 外的偏移 |
 * |---|---|---|---|
 * | 46.2°（16:9 + 82°） | **0.50°** | **±0.28** | 0.11 m |
 * | 55.1°（4:3 + 82°）  | **0.86°** | **±0.28** | 0.19 m |
 * | 63.2°（1:1 + 82°）  | 1.31° | ±0.28 | 0.29 m |
 * | 29.3°（16:9 + 35°） | 0.13° | ±0.28 | 0.03 m |
 * | 12.5°（16:9 + 15°） | 0.01° | ±0.28 | 0.002 m |
 *
 * （⚠️ 早期本文档与 `doc/VISION-GIMBAL-PITCH.md` 都写"边缘差约 3°"，
 * **那是错的**，2026-09-27 实测更正。真实误差比 3° 小一个量级。）
 *
 * 半个杆的半径约 0.05 m，所以对**广角**镜头，线性近似的 0.11–0.19 m 已经
 * 与"瞄错半个杆"同量级；而对**长焦**它小到无关紧要。⇒ 统一用 `atan`，
 * 不为省一次反正切而在广角档留下一个系统性偏差。
 *
 * @param vMid     目标在画面里的归一化纵坐标 [0,1]
 * @param vfovDeg  垂直视场角（度），由 `LzVision_VerticalFovDeg()` 给
 * @return 需要施加的角度增量（度，正 = 上仰）；入参非法时返回 NAN
 */
double LzVision_PixelOffsetToDeg(double vMid, double vfovDeg);

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
