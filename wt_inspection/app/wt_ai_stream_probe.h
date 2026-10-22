/**
 * @file wt_ai_stream_probe.h
 * @brief 最小取流探针：验证 DjiLiveview_StartImageStream 能否拿到相机帧。
 *
 * 用于判定「持续跟踪」方案是否可行 —— 如果拿不到实时画面，整个
 * 「取流 → 推理 → 定位 → 规划」链路就不成立。
 */

#ifndef WT_AI_STREAM_PROBE_H
#define WT_AI_STREAM_PROBE_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 启动取流探针。
 *
 * 失败不阻断启动（与 AI 叠加同一原则）。
 *
 * @param enabled 是否启用（来自配置）
 * @return true 表示已开始接收
 */
bool WtAiStreamProbe_Start(bool enabled);

/**
 * @brief 停止取流并打印统计。
 */
void WtAiStreamProbe_Stop(void);

/**
 * @brief 已收到的帧数，用于外部判断链路是否活着。
 */
uint32_t WtAiStreamProbe_FrameCount(void);

#endif /* WT_AI_STREAM_PROBE_H */
