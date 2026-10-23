/**
 * @file wt_vision.h
 * @brief 独立推理线程：拉帧 → TensorRT 关键点 → 稳定排序的检测快照。
 *
 * ## 为什么必须是独立线程
 *
 * `WtTrtPose_Infer` 单帧端到端约 16 ms，而取流回调跑在 SDK 线程上 ——
 * 在回调里推理会把取流链路拖死（姊妹项目实测：回调里调阻塞接口，日志刷
 * `semaphore wait timeout` 后进程崩）。而且 `WtTrtPose_Infer` 本身
 * **不是线程安全的**（同一 engine 上下文不能并发），所以推理必须集中在
 * 单一线程里。
 *
 * ## 快照语义
 *
 * 对外只暴露「最新一次成功推理的结果」，带年龄（ageMs）。调用方必须检查
 * 年龄 —— 用一帧 2 秒前的检测去下发云台角，会把云台转到目标已经不在的
 * 位置，而且**不会报任何错**。
 *
 * ## 叶尖顺序已被钉死
 *
 * 模型输出的三个叶尖是无序的（排列不变损失训练）。本模块在存快照前调用
 * `WtAim_SortTipsByImageAngle()` 做稳定排序，所以 `tipU/tipV[0]` 始终是
 * 图像上方起顺时针第一片。采集编号就建立在这个顺序上。
 */

#ifndef WT_VISION_H
#define WT_VISION_H

#include <stdbool.h>
#include <stdint.h>

#include "wt_ai_solve.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 一次推理的快照 */
typedef struct {
    WtAiDetection det;      /*!< 已按图像极角排序的检测结果 */
    int imageWidthPx;
    int imageHeightPx;
    uint32_t frameId;
    uint32_t ageMs;         /*!< 距这帧被采集的时间 */
    int rawTargetCount;     /*!< 该帧原始检出的目标数（>1 说明画面里有多台风机） */
    float conf;             /*!< 被采用的那个目标的框置信度 */
} WtVisionSnapshot;

/**
 * @brief 启动推理线程。
 *
 * @param enginePath  .engine 文件路径。**必须是设备上用 trtexec 构建的**，
 *                    engine 与 GPU 架构和 TensorRT 版本强绑定，x86 上转的
 *                    在 Orin 上加载会直接失败。放持久目录，别放 /tmp
 *                    （设备重启会清空，重建要 13 分钟）。
 * @param confThresh  框置信度阈值，典型 0.25
 * @param iouThresh   NMS 的 IoU 阈值，典型 0.45
 * @param intervalMs  两次推理之间的最小间隔。采集激光点时不需要 30fps ——
 *                    推理每秒几次就够，留出 GPU 和内存带宽给别的事。
 */
bool WtVision_Start(const char *enginePath, float confThresh, float iouThresh,
                    uint32_t intervalMs);

/** @brief 停止推理线程并释放 engine */
void WtVision_Stop(void);

/**
 * @brief 取最新的检测快照。
 *
 * @return true 表示拿到了快照（可能很旧，看 ageMs）；false 表示还没有任何
 *         成功的推理结果
 */
bool WtVision_GetSnapshot(WtVisionSnapshot *out);

/** @brief 累计成功推理的帧数 */
uint32_t WtVision_InferCount(void);

/** @brief 累计推理失败次数 */
uint32_t WtVision_FailCount(void);

#ifdef __cplusplus
}
#endif

#endif /* WT_VISION_H */
