/**
 * @file wt_ai_bridge.h
 * @brief 把 WtTrtPose 的推理结果转换成 WtAiSolve_FromDetection 的输入。
 *
 * 两边语义已经一致（同一套 7 关键点约定，见 wt_trt_pose.h / wt_ai_solve.h），
 * 这里只做字段搬运，不含任何几何计算。
 */

#ifndef WT_AI_BRIDGE_H
#define WT_AI_BRIDGE_H

#include <stdbool.h>

#include "wt_ai_solve.h"
#include "wt_trt_pose.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 取置信度最高的检测目标，转换为几何解算的输入。
 *
 * WtAiSolve_FromDetection 只处理单张图里一个风机；一帧可能检出多个目标时
 * 取 conf 最高的一个——WtTrtResult.items 已按 conf 降序排列（NMS 的贪心
 * 顺序，见 wt_trt_pose.cpp）。
 *
 * 关键点没有逐点置信度（kpt_shape:[7,2]，只有 x,y），所以 hub/tip 的有效性
 * 直接继承整个 detection 的置信度门槛：WtTrtPose_Infer 内部已用 confThresh
 * 过滤过，能进 items[] 的目标，其全部关键点在这里都标记为 valid。
 *
 * @param trt 推理结果
 * @param out 转换后的检测结果
 * @return true 表示 trt 至少有一个检测目标；false 时 *out 不作保证
 */
bool WtAiBridge_FromTrtResult(const WtTrtResult *trt, WtAiDetection *out);

#ifdef __cplusplus
}
#endif

#endif /* WT_AI_BRIDGE_H */
