/**
 * @file lz_vision.h
 * @brief 视觉层：从一帧画面里找出国旗杆，给出像素位置。
 *
 * 本层的**唯一职责**是回答"杆在画面里的哪里、多粗多高"，
 * 不做任何航线决策，也不做定位解算（那是 lz_localize 的事）。
 *
 * ⚠️ 本层**刻意不依赖 OpenCV**。国旗是极易分割的目标：高饱和的红色块
 * （旗面）配一根细长的杆，HSV 阈值 + 连通域即可，纯 C 约 200–400 行。
 * 保持零依赖有两层好处：
 *   1. WSL 与妙算3 都不必装 OpenCV（设备上有没有尚未确认）
 *   2. 视觉层与 lz_core 一样能在桌面上直接跑回归测试
 * 真需要特征匹配/深度模型时再引入 OpenCV 不迟。
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

/** 视觉配置 */
typedef struct {
    /* ---- HSV 阈值：旗面的红色在色环两端各有一段 ---- */
    int redHueLowMax;       /*!< 低段红：H ∈ [0, redHueLowMax]，典型 10 */
    int redHueHighMin;      /*!< 高段红：H ∈ [redHueHighMin, 180]，典型 170 */
    int minSaturation;      /*!< 饱和度下限，典型 100（滤掉灰白背景） */
    int minValue;           /*!< 明度下限，典型 60（滤掉阴影） */

    /* ---- 形态学与连通域 ---- */
    int openKernel;         /*!< 开运算核尺寸（像素），去噪点；0 = 不做 */
    int minBlobArea;        /*!< 连通域最小面积（像素），滤掉小色块 */

    double minConfidence;   /*!< 低于此置信度的结果丢弃 */
} LzVisionConfig;

/** 视觉上下文（不透明；内部持有中间缓冲，故不放进头文件） */
typedef struct LzVision LzVision;

LzStatus LzVision_Init(const LzVisionConfig *config, LzVision **out);
void     LzVision_Deinit(LzVision *vision);

/**
 * @brief 在一帧里找出所有候选杆（实现待补，见 lz_vision.c 的要点说明）
 *
 * 输出填入 targets 的 pixel 与 confidence 字段；
 * geo / heightM / radiusM **留空**（分别由 lz_localize 与标定得出）。
 *
 * @param frame   [in]  一帧图像
 * @param targets [out] 检测结果；调用前需 LzTargetList_Init
 */
LzStatus LzVision_Detect(LzVision *vision, const LzFrame *frame, LzTargetList *targets);

/**
 * @brief 由像素框与标定参数估算杆高与半径
 *
 * 单独一个函数：它依赖相机内参与距离，与"怎么找到杆"是两回事。
 * 距离可由杆的像素宽度与先验直径反推，或由激光/双目给出。
 *
 * @param pixel     杆的像素框
 * @param frameW     画面宽（像素）
 * @param frameH     画面高（像素）
 * @param focalPx    焦距（像素）
 * @param distanceM  相机到杆的距离 m
 * @param knownDiameterM 杆的先验直径 m；<= 0 表示未知，此时半径按 0 处理
 * @param outHeightM [out] 估算的杆高
 * @param outRadiusM [out] 估算的杆半径
 */
LzStatus LzVision_EstimateSize(const LzPixelBox *pixel, int frameW, int frameH,
                               double focalPx, double distanceM, double knownDiameterM,
                               double *outHeightM, double *outRadiusM);

#endif /* LZ_VISION_H */
