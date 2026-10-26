/**
 * @file wt_trt_pose.h
 * @brief TensorRT YOLO 推理封装（妙算3 / Orin NX）：关键点模型与检测模型共用。
 *
 * ## 支持的两种模型（输出布局见 `wt_yolo_decode.h`）
 *
 * | 用途 | 模型 | 输入 | 输出 |
 * |---|---|---|---|
 * | 风机整机 | YOLO11-pose，7 关键点 | 1×3×640×640 | 1×19×8400 |
 * | 高空行人 | YOLO11 检测，VisDrone 11 类 | 1×3×1280×1280 | 1×15×33600 |
 *
 * 风机那行是 2026-10-22 `trtexec` 实测确认的。**关键点没有逐点置信度** ——
 * `kpt_shape: [7,2]` 决定的，不是封装漏了；下游要门槛时用框的 conf 代替。
 *
 * ## 形状从 engine 读，并在 Init 时核对
 *
 * 拆分前这里把 `19 × 8400` 写死，engine 换了也照读 —— 装错 engine 时读到的
 * 是一片错位的数字，而**不报任何错**。现在输入边长、候选数、通道数都从
 * engine 读出，通道数与调用方声明的布局对不上就拒绝初始化。
 *
 * ## 性能实测（Orin NX，FP16，TensorRT 8.5.2，风机模型 640 输入）
 *
 * ```
 * GPU 计算   8.60 ms（中位）
 * 端到端     9.18 ms
 * 吞吐       110~114 QPS
 * ```
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

#include "wt_yolo_decode.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 推理上下文（不透明） */
typedef struct WtTrtPose WtTrtPose;

/**
 * @brief 加载 engine 并初始化推理上下文。
 *
 * @param enginePath  .engine 文件路径
 * @param layout      输出布局。`numClasses = 0` 表示**由 engine 的输出通道数
 *                    反推**（检测模型换数据集时类别数会变）；关键点约定
 *                    （kptCount / kptDims）必须由调用方给出
 * @param filter      置信度 / NMS / 类别筛选
 * @param resize      letterbox 缩放方式（风机最近邻、行人双线性，见 E_WtYoloResize）
 * @return 上下文指针；失败返回 NULL（原因打到 stderr）
 */
WtTrtPose *WtTrtPose_Init(const char *enginePath, const WtYoloLayout *layout,
                          const WtYoloFilter *filter, E_WtYoloResize resize);

/**
 * @brief 取 engine 的实际布局与形状（Init 成功后才有意义）。
 *
 * 任一输出指针可为 NULL。
 */
void WtTrtPose_GetShape(const WtTrtPose *ctx, WtYoloLayout *layout,
                        int *inputW, int *inputH, int *numCand);

/**
 * @brief 对一帧 RGB 图像做推理。
 *
 * 输入可以是任意分辨率；内部做 letterbox 到 engine 的输入尺寸，输出坐标
 * 已还原到**输入图像的像素坐标系**。
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
