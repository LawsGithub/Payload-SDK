/**
 * @file lz_vision_source.c
 * @brief 取图：把 PSDK 的图像流变成 LzFrame。
 *
 * 两条路径，按机型支持情况择一：
 *
 *   A. DjiLiveview_StartImageStream()  —— **妙算3 支持**（头文件 @note 明确写了），
 *      直接给解码后的像素帧，视觉侧最省事。首选。
 *   B. DjiLiveview_StartH264Stream()   —— 拿到的是编码流，需自行解码。
 *      仅在 A 不可用时才走（@note: "This interface support on DJI manifold3"）。
 *
 * 核实签名（豁免规则 2，签名必查）：
 *   grep -n -B14 "DjiLiveview_StartImageStream" \
 *        ~/projects/Payload-SDK/psdk_lib/include/dji_liveview.h
 *   回调类型 DjiLiveview_ImageCallback 与像素格式 E_DjiLiveViewPixFormate
 *   在同一头文件，注意 pixFmt 的选择 —— 选错不是报错，是花屏。
 */

#include "lz_vision.h"

/* 骨架：待联调。 */
LzStatus LzVisionSource_Start(void);
LzStatus LzVisionSource_Stop(void);
