/**
 * @file wt_trt_pose.cpp
 * @brief TensorRT YOLO 推理实现。
 *
 * 用 C++ 是因为 TensorRT 的 API 是 C++ 的。对外仍导出 C 接口
 * （见 `wt_trt_pose.h`），这样调用方（PSDK 应用，C）不受影响。
 *
 * ## 实现要点
 *
 * 1. **预处理在 CPU 上做**：1440×1080 RGB → letterbox 到 engine 输入尺寸 →
 *    float32。640 输入实测约 2 ms，相比 8.7 ms 的推理不算瓶颈。行人模型是
 *    1280 输入 + 双线性，开销约为其数倍（未在设备上实测）。上 GPU 做能再省，
 *    但引入 CUDA kernel 后复杂度陡增，当前不值得。
 *
 * 2. **letterbox 与解码在 `wt_yolo_decode.c`**（纯 C，桌面可测）。本文件只剩
 *    「读 engine、搬显存、执行」这些只有设备上才有的事。
 *
 * 3. **NMS 自己实现**，不依赖 OpenCV 的 `dnn::NMSBoxes` —— 后者要链接
 *    opencv_dnn，而设备上的 OpenCV 版本不统一（系统 4.2 / 本地 4.5 并存）。
 */

#include "wt_trt_pose.h"

#include <NvInfer.h>

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

void PrintDims(const char *what, const nvinfer1::Dims &d)
{
    std::fprintf(stderr, "[trt] %s 形状 (", what);
    for (int i = 0; i < d.nbDims; i++) {
        std::fprintf(stderr, "%s%d", i ? "," : "", (int)d.d[i]);
    }
    std::fprintf(stderr, ")\n");
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
    std::vector<float> hostInput;
    std::vector<float> hostOutput;
    std::vector<WtYoloCand> work;
    WtYoloLayout layout{};
    WtYoloFilter filter{};
    E_WtYoloResize resize = WT_YOLO_RESIZE_NEAREST;
    int inputW = 0;
    int inputH = 0;
    int channels = 0;
    int numCand = 0;
};

namespace {

/**
 * 读出 engine 的输入/输出形状并与调用方声明的布局核对。
 *
 * 用的是 TensorRT 8.5 起的按名字查询接口（getNbIOTensors / getTensorShape），
 * 不用已标记弃用的 binding 系列。执行仍走 enqueueV2，绑定顺序沿用拆分前已
 * 上机验证的「0 = 输入、1 = 输出」—— 这里先核对 IO 模式确认这个前提成立。
 */
bool ReadShapes(WtTrtPose *ctx, const WtYoloLayout *want)
{
    const nvinfer1::ICudaEngine *e = ctx->engine.get();

    if (e->getNbIOTensors() != 2) {
        std::fprintf(stderr, "[trt] engine 有 %d 个输入输出张量，期望 2 个"
                             "（一个 images、一个 output0）\n",
                     (int)e->getNbIOTensors());
        return false;
    }

    const char *inName = e->getIOTensorName(0);
    const char *outName = e->getIOTensorName(1);

    if (e->getTensorIOMode(inName) != nvinfer1::TensorIOMode::kINPUT ||
        e->getTensorIOMode(outName) != nvinfer1::TensorIOMode::kOUTPUT) {
        std::fprintf(stderr, "[trt] 张量顺序不是「0 = 输入、1 = 输出」，"
                             "与 enqueueV2 的绑定约定不符\n");
        return false;
    }
    if (e->getTensorDataType(inName) != nvinfer1::DataType::kFLOAT ||
        e->getTensorDataType(outName) != nvinfer1::DataType::kFLOAT) {
        std::fprintf(stderr, "[trt] 输入/输出不是 float32（构建时别加 "
                             "--inputIOFormats/--outputIOFormats=fp16）\n");
        return false;
    }

    const nvinfer1::Dims in = e->getTensorShape(inName);
    const nvinfer1::Dims out = e->getTensorShape(outName);

    PrintDims(inName, in);
    PrintDims(outName, out);

    if (in.nbDims != 4 || in.d[0] != 1 || in.d[1] != 3 || in.d[2] <= 0 || in.d[3] <= 0) {
        std::fprintf(stderr, "[trt] 输入应为固定形状 1×3×H×W（导出时 dynamic=False）\n");
        return false;
    }
    if (in.d[2] > WT_YOLO_MAX_INPUT || in.d[3] > WT_YOLO_MAX_INPUT) {
        std::fprintf(stderr, "[trt] 输入边长超过 %d\n", WT_YOLO_MAX_INPUT);
        return false;
    }
    if (out.nbDims != 3 || out.d[0] != 1 || out.d[1] <= 4 || out.d[2] <= 0) {
        std::fprintf(stderr, "[trt] 输出应为 1×C×N（C > 4）\n");
        return false;
    }

    ctx->inputH = (int)in.d[2];
    ctx->inputW = (int)in.d[3];
    ctx->channels = (int)out.d[1];
    ctx->numCand = (int)out.d[2];

    if (want->numClasses == 0) {
        if (!WtYolo_LayoutFromChannels(ctx->channels, want->kptCount, want->kptDims,
                                       &ctx->layout)) {
            std::fprintf(stderr, "[trt] 输出 %d 通道与「%d 个关键点 × %d 维」的约定对不上"
                                 "（装错 engine 了？）\n",
                         ctx->channels, want->kptCount, want->kptDims);
            return false;
        }
    } else {
        if (WtYolo_Channels(want) != ctx->channels) {
            std::fprintf(stderr, "[trt] 输出 %d 通道，而声明的布局（%d 类、%d 个关键点 × %d 维）"
                                 "应为 %d 通道 —— 装错 engine 了？\n",
                         ctx->channels, want->numClasses, want->kptCount, want->kptDims,
                         WtYolo_Channels(want));
            return false;
        }
        ctx->layout = *want;
    }

    return true;
}

}  // namespace

extern "C" {

WtTrtPose *WtTrtPose_Init(const char *enginePath, const WtYoloLayout *layout,
                          const WtYoloFilter *filter, E_WtYoloResize resize)
{
    if (enginePath == nullptr || layout == nullptr || filter == nullptr) {
        return nullptr;
    }

    auto *ctx = new WtTrtPose();
    ctx->filter = *filter;
    ctx->resize = resize;

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

    if (size <= 0) {
        std::fclose(f);
        delete ctx;
        return nullptr;
    }

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

    if (!ReadShapes(ctx, layout)) {
        delete ctx;
        return nullptr;
    }

    ctx->context.reset(ctx->engine->createExecutionContext());
    if (!ctx->context) {
        delete ctx;
        return nullptr;
    }

    const size_t inCount = (size_t)3 * (size_t)ctx->inputW * (size_t)ctx->inputH;
    const size_t outCount = (size_t)ctx->channels * (size_t)ctx->numCand;

    if (cudaMalloc(&ctx->dInput, inCount * sizeof(float)) != cudaSuccess ||
        cudaMalloc(&ctx->dOutput, outCount * sizeof(float)) != cudaSuccess) {
        std::fprintf(stderr, "[trt] cudaMalloc 失败\n");
        WtTrtPose_Deinit(ctx);
        return nullptr;
    }

    if (cudaStreamCreate(&ctx->stream) != cudaSuccess) {
        std::fprintf(stderr, "[trt] cudaStreamCreate 失败\n");
        ctx->stream = nullptr;
        WtTrtPose_Deinit(ctx);
        return nullptr;
    }

    ctx->hostInput.resize(inCount);
    ctx->hostOutput.resize(outCount);
    ctx->work.resize((size_t)ctx->numCand);

    return ctx;
}

void WtTrtPose_GetShape(const WtTrtPose *ctx, WtYoloLayout *layout,
                        int *inputW, int *inputH, int *numCand)
{
    if (ctx == nullptr) {
        return;
    }
    if (layout != nullptr) {
        *layout = ctx->layout;
    }
    if (inputW != nullptr) {
        *inputW = ctx->inputW;
    }
    if (inputH != nullptr) {
        *inputH = ctx->inputH;
    }
    if (numCand != nullptr) {
        *numCand = ctx->numCand;
    }
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
    WtYoloLetterbox lb;

    if (!WtYolo_Letterbox(rgb, width, height, ctx->inputW, ctx->inputH, ctx->resize,
                          ctx->hostInput.data(), &lb)) {
        return false;
    }

    const size_t inBytes = ctx->hostInput.size() * sizeof(float);
    const size_t outBytes = ctx->hostOutput.size() * sizeof(float);

    if (cudaMemcpyAsync(ctx->dInput, ctx->hostInput.data(), inBytes,
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
    if (cudaStreamSynchronize(ctx->stream) != cudaSuccess) {
        return false;
    }

    /* ---- 解码 + NMS（wt_yolo_decode.c，桌面可测） ---- */
    return WtYolo_Decode(ctx->hostOutput.data(), ctx->channels, ctx->numCand,
                         &ctx->layout, &lb, &ctx->filter,
                         ctx->work.data(), (int)ctx->work.size(), out);
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
