/**
 * @file wt_vision.h
 * @brief 独立推理线程：拉帧 → TensorRT → 检测快照。风机与行人两种模型共用。
 *
 * ## 为什么必须是独立线程
 *
 * `WtTrtPose_Infer` 单帧端到端约 16 ms（风机模型），而取流回调跑在 SDK 线程上 ——
 * 在回调里推理会把取流链路拖死（姊妹项目实测：回调里调阻塞接口，日志刷
 * `semaphore wait timeout` 后进程崩）。而且 `WtTrtPose_Infer` 本身
 * **不是线程安全的**（同一 engine 上下文不能并发），所以推理必须集中在
 * 单一线程里。
 *
 * ## 两种模型，快照语义不同
 *
 * | 模型 | 快照里看什么 | 没检出时 |
 * |---|---|---|
 * | 风机（`WT_VISION_MODEL_TURBINE`） | `det`：轮毂 + 已排序的叶尖 | **不更新**快照，让它自然变旧 |
 * | 行人（`WT_VISION_MODEL_PEDESTRIAN`） | `raw`：NMS 后的全部行人框 | **照样更新**（空结果） |
 *
 * 风机只有一台，"多久没看到它了"比"这一帧有没有"更有用；行人要跨帧跟踪，
 * 必须分得清「这一帧确实没人」和「推理线程卡住了没出新帧」—— 前者是新帧号
 * + 空结果，后者是帧号不变。
 *
 * ## 快照年龄
 *
 * 对外只暴露「最新一次推理的结果」，带年龄（ageMs）与采集时刻。调用方必须
 * 检查 —— 用一帧 2 秒前的检测去下发云台角，会把云台转到目标已经不在的
 * 位置，而且**不会报任何错**。
 *
 * ## 叶尖顺序已被钉死（风机模型）
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
#include "wt_trt_pose.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 加载哪种模型 —— 决定输出布局、类别筛选、缩放方式与快照语义 */
typedef enum {
    /* RWTH 风机整机关键点模型：1 类 + 7 关键点，640 输入 */
    WT_VISION_MODEL_TURBINE = 0,
    /*
     * 航拍行人检测（VisDrone 系）：类别数由 engine 反推，只收类别 0/1
     * （pedestrian / people）—— 这是 VisDrone 的类别顺序，换模型时必须
     * 保持（tools/ped_model/export_onnx.py 导出时会核对类别名）。
     */
    WT_VISION_MODEL_PEDESTRIAN
} E_WtVisionModel;

/** 一次推理的快照 */
typedef struct {
    WtAiDetection det;      /*!< 风机模型：已按图像极角排序的检测结果 */
    WtTrtResult raw;        /*!< 行人模型：NMS 后的全部行人框（原图像素） */
    int imageWidthPx;
    int imageHeightPx;
    uint32_t frameId;       /*!< 相机给的帧号（只用于日志，语义无文档承诺） */
    /*
     * 快照序号：每存一次 +1。判「是不是新的一帧」用它，不用 frameId ——
     * 相机帧号是否单调、会不会恒为 0，头文件里都没有承诺；万一不变，
     * 下游就永远等不到"新帧"，而且不报错。
     */
    uint32_t seq;
    uint32_t capturedAtMs;  /*!< 帧到达时刻（OSAL 毫秒，与云台读回同一时钟） */
    uint32_t ageMs;         /*!< 读取时刻 − capturedAtMs */
    int rawTargetCount;     /*!< 该帧原始检出的目标数 */
    float conf;             /*!< 最可信那个目标的置信度 */
} WtVisionSnapshot;

/**
 * @brief 启动推理线程。
 *
 * @param enginePath  .engine 文件路径。**必须是设备上用 trtexec 构建的**，
 *                    engine 与 GPU 架构和 TensorRT 版本强绑定，x86 上转的
 *                    在 Orin 上加载会直接失败。放持久目录，别放 /tmp
 *                    （设备重启会清空，重建要十几分钟）。
 * @param model       模型种类，见 E_WtVisionModel
 * @param confThresh  框置信度阈值，典型 0.25
 * @param iouThresh   NMS 的 IoU 阈值，典型 0.45
 * @param intervalMs  两次推理之间的最小间隔。采集激光点时不需要 30fps ——
 *                    推理每秒几次就够，留出 GPU 和内存带宽给别的事。
 */
bool WtVision_Start(const char *enginePath, E_WtVisionModel model,
                    float confThresh, float iouThresh, uint32_t intervalMs);

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
