/**
 * @file wt_trt_pose.h
 * @brief TensorRT YOLOv8-pose 推理封装（妙算3 / Orin NX）。
 *
 * ## 模型约定（2026-10-22 实测，`trtexec` 输出确认）
 *
 * ```
 * 输入  images   1 × 3 × 640 × 640   float32 NCHW
 * 输出  output0  1 × 19 × 8400       float32
 * ```
 *
 * `19 = 4 (cx,cy,w,h) + 1 (conf) + 14 (7 关键点 × 2)`。
 * **关键点没有逐点置信度** —— 这是 `kpt_shape: [7,2]` 决定的，不是封装漏了。
 * 下游要置信度门槛时用框的 conf 代替。
 *
 * 7 个关键点的语义见 `wt_ai_solve.h`：索引 3 是轮毂，4/5/6 是三个叶尖
 * （无序，编号由 `wt_blade_id.c` 按几何决定）。
 *
 * ## 性能实测（Orin NX，FP16，TensorRT 8.5.2）
 *
 * ```
 * GPU 计算   8.60 ms（中位）
 * 端到端     9.18 ms
 * 吞吐       110~114 QPS
 * ```
 *
 * 30fps 每帧预算 33.3 ms，推理只占约 9 ms。
 *
 * ## 线程模型
 *
 * ⚠️ **`WtTrtPose_Infer` 不是线程安全的**，同一个 engine 上下文不能并发调用。
 * 调用方（取流回调）应当只把帧塞进队列，由**单一推理线程**消费 ——
 * 在 SDK 回调线程里直接推理会阻塞取流。
 *
 * ## 生命周期
 *
 * `Init` → 反复 `Infer` → `Deinit`。engine 文件加载约 80 ms，
 * 不要每帧重载。
 */

#ifndef WT_TRT_POSE_H
#define WT_TRT_POSE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 模型输入边长（固定 640，导出时 dynamic=False） */
#define WT_TRT_INPUT_SIZE 640

/** 关键点数量 */
#define WT_TRT_KPT_COUNT 7

/** 单帧最多返回的目标数 */
#define WT_TRT_MAX_DETECTIONS 32

/** 一个检测结果（像素坐标，已还原到原图） */
typedef struct {
    float bboxX1;
    float bboxY1;
    float bboxX2;
    float bboxY2;
    float conf;
    /* 关键点像素坐标，顺序与 wt_ai_solve.h 的 KPT 语义一致 */
    float kptX[WT_TRT_KPT_COUNT];
    float kptY[WT_TRT_KPT_COUNT];
} WtTrtDetection;

/** 一次推理的结果集 */
typedef struct {
    int count;
    WtTrtDetection items[WT_TRT_MAX_DETECTIONS];
} WtTrtResult;

/** 推理上下文（不透明） */
typedef struct WtTrtPose WtTrtPose;

/**
 * @brief 加载 engine 并初始化推理上下文。
 *
 * @param enginePath  .engine 文件路径
 * @param confThresh  框置信度阈值（典型 0.25）
 * @param iouThresh   NMS 的 IoU 阈值（典型 0.45）
 * @return 上下文指针；失败返回 NULL
 */
WtTrtPose *WtTrtPose_Init(const char *enginePath,
                          float confThresh, float iouThresh);

/**
 * @brief 对一帧 RGB 图像做推理。
 *
 * 输入可以是任意分辨率；内部做 letterbox 到 640×640，输出坐标已还原到
 * **输入图像的像素坐标系**。
 *
 * @param ctx   上下文
 * @param rgb    输入图像，RGB 三通道交错（与 `PIXFMT_RGB_PACKED` 一致）
 * @param width  输入宽
 * @param height 输入高
 * @param out    输出结果
 * @return true 表示推理成功（结果可能为空，看 out->count）
 */
bool WtTrtPose_Infer(WtTrtPose *ctx, const unsigned char *rgb,
                     int width, int height, WtTrtResult *out);

/** @brief 释放资源 */
void WtTrtPose_Deinit(WtTrtPose *ctx);

#ifdef __cplusplus
}
#endif

#endif /* WT_TRT_POSE_H */
