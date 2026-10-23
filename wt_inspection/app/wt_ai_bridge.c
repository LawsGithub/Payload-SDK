/**
 * @file wt_ai_bridge.c
 * @brief WtTrtPose → WtAiSolve 的字段转换实现。
 */

#include "wt_ai_bridge.h"

#include <string.h>

/*
 * 7 个关键点的语义（README.md「模型与关键点」/ wt_trt_pose.h 头部注释，
 * 来源 generator.py:472-486）：索引 3 = 轮毂，4/5/6 = 三个叶尖（无序，
 * 由 wt_blade_id.c 按几何编号）。两处约定分散在注释里，这里锚成常量，
 * 避免将来改模型时有一处漏改。
 */
#define WT_AI_BRIDGE_HUB_KPT_IDX  3
#define WT_AI_BRIDGE_TIP_KPT_IDX0 4
#define WT_AI_BRIDGE_TIP_N        3

bool WtAiBridge_FromTrtResult(const WtTrtResult *trt, WtAiDetection *out)
{
    const WtTrtDetection *best;
    int i;

    if (trt == NULL || out == NULL || trt->count <= 0) {
        return false;
    }

    /* items[] 已按 conf 降序排列，第一个就是最可信的目标。 */
    best = &trt->items[0];

    memset(out, 0, sizeof(*out));

    out->hubValid = true;
    out->hubU = (double)best->kptX[WT_AI_BRIDGE_HUB_KPT_IDX];
    out->hubV = (double)best->kptY[WT_AI_BRIDGE_HUB_KPT_IDX];

    out->tipCount = WT_AI_BRIDGE_TIP_N;
    for (i = 0; i < WT_AI_BRIDGE_TIP_N; i++) {
        out->tipU[i] = (double)best->kptX[WT_AI_BRIDGE_TIP_KPT_IDX0 + i];
        out->tipV[i] = (double)best->kptY[WT_AI_BRIDGE_TIP_KPT_IDX0 + i];
        out->tipValid[i] = true;
    }

    return true;
}
