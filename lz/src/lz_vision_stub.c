/**
 * @file lz_vision_stub.c
 * @brief 视觉层的**占位实现** —— 让整条链路先能跑通，视觉算法后补。
 *
 * ## 为什么要有这个文件
 *
 * `lz_vision.c`（HSV 阈值分割）的连通域部分尚未实现，`LzVision_Detect()`
 * 会返回 `LZ_ERR_UNSUPPORTED`。但主应用的目标不是"检测得多准"，而是
 * **"操作员拨开关 → 飞机绕一圈"这条链路完整可用**。
 *
 * 而且现在还多了一个前提变化：激光测距已实测可用（见 API-MAP §八），
 * 视觉层的职责可能从"解算杆的绝对坐标"降级为"确认激光瞄准点是不是杆"
 * —— 那根本用不着连通域。**在职责定下来之前把视觉算法做深，是白做。**
 *
 * 所以本文件提供两种"能跑"的模式：
 *
 *   - **固定杆位**：用一个写死的坐标当作杆，用于室内把链路走通
 *   - **激光取位**：读激光测距的经纬度当杆位（真实路径，需室外有 GPS）
 *
 * ## 与 lz_vision.c 的关系
 *
 * 两者是**互斥的替代实现**，CMake 里二选一（见 `LZ_VISION_BACKEND`）。
 * 不要把两个都编进来 —— `LzVision_Detect` 会符号冲突。
 */

#include "lz_vision.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/** 后端标识，编译期可见 */
const char *LzVision_BackendName(void);

const char *LzVision_BackendName(void)
{
    return "stub";
}

struct LzVision {
    LzVisionConfig config;
    /** 写死的杆位；由 LzVision_SetFixedPole 设置 */
    LzGeo fixedPole;
    bool hasFixedPole;
};

LzStatus LzVision_Init(const LzVisionConfig *config, LzVision **out)
{
    if (config == NULL || out == NULL) {
        return LZ_ERR_PARAM;
    }
    LzVision *v = calloc(1, sizeof(*v));
    if (v == NULL) {
        return LZ_ERR_IO;
    }
    v->config = *config;
    *out = v;
    return LZ_OK;
}

void LzVision_Deinit(LzVision *vision)
{
    free(vision);
}

/**
 * @brief 设置固定的杆位（占位后端的专用接口）
 *
 * 不属于 `lz_vision.h` 的公共契约 —— 只有占位后端有这个能力。
 * 主应用通过弱符号探测它是否存在，因此换个后端不会导致链接失败。
 */
LzStatus LzVision_SetFixedPole(LzVision *vision, const LzGeo *pole, double confidence)
{
    if (vision == NULL || pole == NULL) {
        return LZ_ERR_PARAM;
    }
    if (!LzGeo_IsValid(pole)) {
        return LZ_ERR_PARAM;
    }
    vision->fixedPole = *pole;
    vision->hasFixedPole = true;

    (void)confidence;
    return LZ_OK;
}

/**
 * @brief 占位检测：不分析图像，直接产出预设的杆
 *
 * 刻意**不看 frame 的内容** —— 真去看反而会让人误以为"视觉在工作"。
 * 它的用途是让下游（规划 → KMZ → 上传）能拿到一个合法的 LzTarget。
 */
LzStatus LzVision_Detect(LzVision *vision, const LzFrame *frame, LzTargetList *targets)
{
    if (vision == NULL || frame == NULL || targets == NULL) {
        return LZ_ERR_PARAM;
    }
    if (!vision->hasFixedPole) {
        return LZ_ERR_NO_TARGET;
    }

    LzTarget t;
    memset(&t, 0, sizeof(t));
    t.id = 1;
    t.geo = vision->fixedPole;
    /* 杆高与半径按国旗杆的常见规格给个保守值；绕飞取中心，半径仅用于避让 */
    t.heightM = 15.0;
    t.radiusM = 0.1;
    /* 像素位置给画面正中 —— 占位值，真视觉会填实测值 */
    t.pixel.u = 0.5;
    t.pixel.v = 0.6;
    t.pixel.topV = 0.2;
    t.pixel.bottomV = 0.6;
    t.confidence = 0.9;

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

    /* 小孔成像：h_px / focalPx = H / distance */
    const double pixelHeight = fabs(pixel->bottomV - pixel->topV) * (double)frameH;
    *outHeightM = pixelHeight * distanceM / focalPx;
    *outRadiusM = (knownDiameterM > 0.0) ? (knownDiameterM * 0.5) : 0.0;
    return LZ_OK;
}