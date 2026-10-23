/**
 * @file wt_trt_pose.cpp
 * @brief TensorRT YOLOv8-pose 推理实现。
 *
 * 用 C++ 是因为 TensorRT 的 API 是 C++ 的。对外仍导出 C 接口
 * （见 `wt_trt_pose.h`），这样调用方（PSDK 应用，C）不受影响。
 *
 * ## 实现要点
 *
 * 1. **预处理在 CPU 上做**：1440×1080 RGB → letterbox 640×640 → float32。
 *    实测这部分约 2 ms，相比 8.7 ms 的推理不算瓶颈。上 GPU 做能再省 1 ms，
 *    但引入 CUDA kernel 后复杂度陡增，当前不值得。
 *
 * 2. **letterbox 必须记录缩放比和填充量**，输出坐标要按它还原。
 *    漏掉这步关键点会系统性偏移，且偏移量随长宽比变化。
 *
 * 3. **NMS 自己实现**，不依赖 OpenCV 的 `dnn::NMSBoxes` —— 后者要链接
 *    opencv_dnn，而设备上的 OpenCV 版本不统一（系统 4.2 / 本地 4.5 并存）。
 */

#include "wt_trt_pose.h"

#include <NvInfer.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

namespace {

/** TensorRT 的日志回调，只保留 error 级，避免刷屏 */
class TrtLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity <= Severity::kERROR) {
            std::fprintf(stderr, "[trt] %s\n", msg);
        }
    }
};

struct TrtDeleter {
    template <typename T>
    void operator()(T *p) const { delete p; }
};

/** letterbox 参数：缩放比与填充量，用于把结果还原到原图 */
struct Letterbox {
    float scale;
    float padX;
    float padY;
};

/**
 * 把 RGB 图等比缩放并居中填充到 640×640，同时转成 NCHW float32 /255。
 *
 * 最近邻插值：对关键点定位足够，且比双线性快。
 */
void LetterboxToTensor(const unsigned char *rgb, int w, int h,
                       float *dst, Letterbox *lb)
{
    const float scale = std::min((float)WT_TRT_INPUT_SIZE / (float)w,
                                 (float)WT_TRT_INPUT_SIZE / (float)h);
    const int newW = (int)std::lround((float)w * scale);
    const int newH = (int)std::lround((float)h * scale);
    const int padX = (WT_TRT_INPUT_SIZE - newW) / 2;
    const int padY = (WT_TRT_INPUT_SIZE - newH) / 2;
    const int plane = WT_TRT_INPUT_SIZE * WT_TRT_INPUT_SIZE;

    lb->scale = scale;
    lb->padX = (float)padX;
    lb->padY = (float)padY;

    /* 先填灰边（YOLO 训练时用 114/255） */
    for (int i = 0; i < plane; i++) {
        dst[i] = 114.0f / 255.0f;
        dst[plane + i] = 114.0f / 255.0f;
        dst[2 * plane + i] = 114.0f / 255.0f;
    }

    for (int dy = 0; dy < newH; dy++) {
        const int sy = std::min(h - 1, (int)((float)dy / scale));
        const int rowOut = (dy + padY) * WT_TRT_INPUT_SIZE + padX;

        for (int dx = 0; dx < newW; dx++) {
            const int sx = std::min(w - 1, (int)((float)dx / scale));
            const int src = (sy * w + sx) * 3;
            const int outIdx = rowOut + dx;

            dst[outIdx] = (float)rgb[src + 0] / 255.0f;
            dst[plane + outIdx] = (float)rgb[src + 1] / 255.0f;
            dst[2 * plane + outIdx] = (float)rgb[src + 2] / 255.0f;
        }
    }
}

float Iou(const WtTrtDetection &a, const WtTrtDetection &b)
{
    const float x1 = std::max(a.bboxX1, b.bboxX1);
    const float y1 = std::max(a.bboxY1, b.bboxY1);
    const float x2 = std::min(a.bboxX2, b.bboxX2);
    const float y2 = std::min(a.bboxY2, b.bboxY2);
    const float w = x2 - x1;
    const float h = y2 - y1;

    if (w <= 0.0f || h <= 0.0f) {
        return 0.0f;
    }

    const float inter = w * h;
    const float areaA = (a.bboxX2 - a.bboxX1) * (a.bboxY2 - a.bboxY1);
    const float areaB = (b.bboxX2 - b.bboxX1) * (b.bboxY2 - b.bboxY1);
    const float uni = areaA + areaB - inter;

    return (uni > 0.0f) ? (inter / uni) : 0.0f;
}

}  // namespace

struct WtTrtPose {
    TrtLogger logger;
    std::unique_ptr<nvinfer1::IRuntime, TrtDeleter> runtime;
    std::unique_ptr<nvinfer1::ICudaEngine, TrtDeleter> engine;
    std::unique_ptr<nvinfer1::IExecutionContext, TrtDeleter> context;
    cudaStream_t stream = nullptr;
    void *dInput = nullptr;
    void *dOutput = nullptr;
    std::vector<float> hostOutput;
    float confThresh = 0.25f;
    float iouThresh = 0.45f;
};

extern "C" {

WtTrtPose *WtTrtPose_Init(const char *enginePath, float confThresh, float iouThresh)
{
    if (enginePath == nullptr) {
        return nullptr;
    }

    auto *ctx = new WtTrtPose();
    ctx->confThresh = confThresh;
    ctx->iouThresh = iouThresh;

    /* 读 engine 文件 */
    FILE *f = std::fopen(enginePath, "rb");

    if (f == nullptr) {
        std::fprintf(stderr, "[trt] 打不开 engine: %s\n", enginePath);
        delete ctx;
        return nullptr;
    }
    std::fseek(f, 0, SEEK_END);
    const long size = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);

    std::vector<char> blob((size_t)size);

    if (std::fread(blob.data(), 1, blob.size(), f) != blob.size()) {
        std::fclose(f);
        delete ctx;
        return nullptr;
    }
    std::fclose(f);

    ctx->runtime.reset(nvinfer1::createInferRuntime(ctx->logger));
    if (!ctx->runtime) {
        delete ctx;
        return nullptr;
    }

    ctx->engine.reset(ctx->runtime->deserializeCudaEngine(blob.data(), blob.size()));
    if (!ctx->engine) {
        std::fprintf(stderr, "[trt] engine 反序列化失败（架构或版本不匹配？）\n");
        delete ctx;
        return nullptr;
    }

    ctx->context.reset(ctx->engine->createExecutionContext());
    if (!ctx->context) {
        delete ctx;
        return nullptr;
    }

    /* 分配显存：输入 1*3*640*640*4，输出 1*19*8400*4 */
    const size_t inBytes = (size_t)3 * WT_TRT_INPUT_SIZE * WT_TRT_INPUT_SIZE * sizeof(float);
    const size_t outCount = (size_t)19 * 8400;
    const size_t outBytes = outCount * sizeof(float);

    if (cudaMalloc(&ctx->dInput, inBytes) != cudaSuccess ||
        cudaMalloc(&ctx->dOutput, outBytes) != cudaSuccess) {
        std::fprintf(stderr, "[trt] cudaMalloc 失败\n");
        WtTrtPose_Deinit(ctx);
        return nullptr;
    }

    cudaStreamCreate(&ctx->stream);
    ctx->hostOutput.resize(outCount);

    return ctx;
}

bool WtTrtPose_Infer(WtTrtPose *ctx, const unsigned char *rgb,
                     int width, int height, WtTrtResult *out)
{
    if (ctx == nullptr || rgb == nullptr || out == nullptr) {
        return false;
    }
    if (width <= 0 || height <= 0) {
        return false;
    }

    out->count = 0;

    /* ---- 预处理 ---- */
    Letterbox lb;
    static thread_local std::vector<float> inputBuf;

    inputBuf.resize((size_t)3 * WT_TRT_INPUT_SIZE * WT_TRT_INPUT_SIZE);
    LetterboxToTensor(rgb, width, height, inputBuf.data(), &lb);

    const size_t inBytes = inputBuf.size() * sizeof(float);
    const size_t outCount = (size_t)19 * 8400;
    const size_t outBytes = outCount * sizeof(float);

    if (cudaMemcpyAsync(ctx->dInput, inputBuf.data(), inBytes,
                        cudaMemcpyHostToDevice, ctx->stream) != cudaSuccess) {
        return false;
    }

    /* ---- 推理 ---- */
    void *bindings[2] = {ctx->dInput, ctx->dOutput};

    if (!ctx->context->enqueueV2(bindings, ctx->stream, nullptr)) {
        std::fprintf(stderr, "[trt] enqueue 失败\n");
        return false;
    }

    if (cudaMemcpyAsync(ctx->hostOutput.data(), ctx->dOutput, outBytes,
                        cudaMemcpyDeviceToHost, ctx->stream) != cudaSuccess) {
        return false;
    }
    cudaStreamSynchronize(ctx->stream);

    /* ---- 解码 ---- */
    /*
     * 输出布局：1×19×8400，19 = 4(bbox) + 1(conf) + 14(7 kpt × 2)。
     * 转置访问：第 c 个通道的第 i 个候选 = data[c * 8400 + i]。
     */
    const float *data = ctx->hostOutput.data();
    const int numCand = 8400;
    std::vector<WtTrtDetection> cands;

    cands.reserve(64);

    for (int i = 0; i < numCand; i++) {
        const float conf = data[4 * numCand + i];

        if (conf < ctx->confThresh) {
            continue;
        }

        WtTrtDetection d;
        const float cx = data[0 * numCand + i];
        const float cy = data[1 * numCand + i];
        const float bw = data[2 * numCand + i];
        const float bh = data[3 * numCand + i];

        /* 从 letterbox 坐标还原到原图 */
        d.bboxX1 = (cx - bw * 0.5f - lb.padX) / lb.scale;
        d.bboxY1 = (cy - bh * 0.5f - lb.padY) / lb.scale;
        d.bboxX2 = (cx + bw * 0.5f - lb.padX) / lb.scale;
        d.bboxY2 = (cy + bh * 0.5f - lb.padY) / lb.scale;
        d.conf = conf;

        for (int k = 0; k < WT_TRT_KPT_COUNT; k++) {
            const float kx = data[(5 + 2 * k) * numCand + i];
            const float ky = data[(5 + 2 * k + 1) * numCand + i];

            d.kptX[k] = (kx - lb.padX) / lb.scale;
            d.kptY[k] = (ky - lb.padY) / lb.scale;
        }

        cands.push_back(d);
    }

    /* ---- NMS（按 conf 降序贪心） ---- */
    std::sort(cands.begin(), cands.end(),
              [](const WtTrtDetection &a, const WtTrtDetection &b) {
                  return a.conf > b.conf;
              });

    for (const auto &c : cands) {
        if (out->count >= WT_TRT_MAX_DETECTIONS) {
            break;
        }

        bool suppressed = false;

        for (int j = 0; j < out->count; j++) {
            if (Iou(c, out->items[j]) > ctx->iouThresh) {
                suppressed = true;
                break;
            }
        }
        if (!suppressed) {
            out->items[out->count++] = c;
        }
    }

    return true;
}

void WtTrtPose_Deinit(WtTrtPose *ctx)
{
    if (ctx == nullptr) {
        return;
    }
    if (ctx->dInput != nullptr) {
        cudaFree(ctx->dInput);
    }
    if (ctx->dOutput != nullptr) {
        cudaFree(ctx->dOutput);
    }
    if (ctx->stream != nullptr) {
        cudaStreamDestroy(ctx->stream);
    }
    delete ctx;
}

}  // extern "C"
