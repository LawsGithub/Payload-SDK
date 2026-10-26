/**
 * @file wt_yolo_decode.h
 * @brief YOLO（ultralytics v8/11 系导出）的预处理与输出解码：letterbox、
 *        框/类别/关键点解码、NMS。纯 C，不依赖 TensorRT，桌面可测。
 *
 * ## 为什么从 wt_trt_pose.cpp 里拆出来
 *
 * 推理本身（TensorRT）只能在妙算3 上跑，但「张量怎么排、坐标怎么还原」是
 * 纯算术 —— 而且恰恰是**算错了不报错**的那一类：通道下标错一位，框照样
 * 画得出来，只是落在别的地方；letterbox 漏还原，目标会系统性偏移，偏移量
 * 还随长宽比变化。留在 .cpp 里就只能上机靠肉眼看，拆出来桌面就能钉住
 * （`wt_test_yolo_decode`），并能拿 ultralytics 自己的解码结果做对照
 * （`tools/ped_model/crosscheck_decode.py`）。
 *
 * ## 输出张量布局（ultralytics 导出，通道优先）
 *
 * ```
 * 1 × C × N，  C = 4 + numClasses + kptCount × kptDims
 *   通道 0..3              cx, cy, w, h      （letterbox 后的输入像素）
 *   通道 4 .. 4+nc-1        各类别得分        （导出时已过 sigmoid）
 *   之后每 kptDims 个通道    一个关键点 x, y[, 可见度]
 * ```
 *
 * | 模型 | nc | 关键点 | C |
 * |---|---|---|---|
 * | 风机整机（RWTH，YOLO11-pose） | 1 | 7 × 2 | 19 |
 * | VisDrone 航拍检测（dronefreak/visdrone-yolo11s） | 11 | 0 | 15 |
 *
 * ⚠️ 这份 VisDrone 权重是 **11 类**（VisDrone 的 10 类 + `others`），不是
 * ultralytics VisDrone.yaml 的 10 类 —— 2026-10-24 导出时实测 `1×15×33600`。
 * 这正是类别数必须从 engine 反推、不能写死的原因。
 *
 * N 由输入边长决定（三个尺度的格点数之和）：640 → 8400，1280 → 33600。
 * 两者都从 engine 读出，不写死。
 */

#ifndef WT_YOLO_DECODE_H
#define WT_YOLO_DECODE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 关键点数组的容量。等于风机模型的 7 个；纯检测模型（kptCount = 0）不用。
 * 布局里 kptCount 超过它的模型会被 WtYolo_Decode 拒绝，而不是越界写。
 */
#define WT_TRT_KPT_COUNT 7

/**
 * 单帧最多返回的目标数（NMS 之后）。
 *
 * 航拍画面里行人常常成群（VisDrone 单图上百个），32 会把置信度排在后面的
 * 目标截掉 —— 正在跟踪的那个人若恰好排在第 33 位，跟踪会无故丢失。
 */
#define WT_TRT_MAX_DETECTIONS 64

/** letterbox 列下标表的上限（输入边长不能超过它） */
#define WT_YOLO_MAX_INPUT 2048

/**
 * 一个检测结果（像素坐标，已还原到原图）。
 *
 * 类型名沿用最初只在 TensorRT 封装里用时的叫法（`wt_ai_bridge` 与测试都
 * 依赖它），定义挪到这里是因为它就是解码器的产物。
 */
typedef struct {
    float bboxX1;
    float bboxY1;
    float bboxX2;
    float bboxY2;
    float conf;      /*!< 被采用的那个类别的得分 */
    int classId;     /*!< 得分最高的类别（argmax，与 ultralytics 的取法一致） */
    /* 关键点像素坐标；纯检测模型时不填 */
    float kptX[WT_TRT_KPT_COUNT];
    float kptY[WT_TRT_KPT_COUNT];
} WtTrtDetection;

/** 一次推理的结果集，items 按 conf 降序 */
typedef struct {
    int count;
    WtTrtDetection items[WT_TRT_MAX_DETECTIONS];
} WtTrtResult;

/** 输出张量布局 */
typedef struct {
    int numClasses;  /*!< 类别数，≥ 1 */
    int kptCount;    /*!< 每个目标的关键点数，0 = 纯检测 */
    int kptDims;     /*!< 每个关键点的维数：2 = (x,y)，3 = (x,y,可见度)；kptCount = 0 时不看 */
} WtYoloLayout;

/** letterbox 参数：把模型输入坐标还原到原图要用 */
typedef struct {
    float scale;
    float padX;
    float padY;
} WtYoloLetterbox;

/** 解码时的筛选条件 */
typedef struct {
    float confThresh;
    float iouThresh;
    /*
     * bit i = 接受类别 i；0 = 全部接受。只能表示前 32 个类别。
     *
     * 判法与 ultralytics `classes=` 参数相同：**先在全部类别上取 argmax，
     * 再看它在不在接受集里**。不是"只在接受的类别里取最大"—— 后者会把一个
     * 明显更像自行车的框也当成人。
     */
    uint32_t classMask;
} WtYoloFilter;

/** NMS 前的候选（解码器的工作区元素，由调用方分配） */
typedef struct {
    float score;
    int index;
    int classId;
} WtYoloCand;

/**
 * @brief 布局对应的输出通道数。
 * @return 通道数；布局非法（nc < 1、kptCount 越界、kptDims < 2）时返回 0
 */
int WtYolo_Channels(const WtYoloLayout *layout);

/**
 * @brief 由 engine 实际的输出通道数反推类别数。
 *
 * 检测模型换成别的数据集时类别数会变，写死就会在新模型上读错通道 ——
 * 所以 nc 从 engine 的输出形状推，只把关键点约定交给调用方。
 *
 * @return false 表示该通道数与给定的关键点约定对不上
 */
bool WtYolo_LayoutFromChannels(int channels, int kptCount, int kptDims,
                               WtYoloLayout *out);

/** letterbox 的缩放方式 */
typedef enum {
    /*
     * 最近邻：风机模型沿用（拆分前就是它，已上机验证）。目标是几百像素的
     * 整机，取样方式无关紧要。
     */
    WT_YOLO_RESIZE_NEAREST = 0,
    /*
     * 双线性，下标映射与 OpenCV INTER_LINEAR 相同（像素中心对齐）——
     * ultralytics 训练与预测用的就是它。行人模型用这个：目标只有十几个
     * 像素，最近邻缩小时会整列跳过源像素。2026-10-24 用同一个 ONNX 实测，
     * 1440×1080 的人群图上双线性检出 45 人、最近邻 39 人。
     */
    WT_YOLO_RESIZE_BILINEAR
} E_WtYoloResize;

/**
 * @brief RGB 交错图 → 等比缩放并居中填灰边 → NCHW float32 /255。
 *
 * 灰边取 114/255（YOLO 训练时的值）。缩放与填充量写进 lb，解码时据此还原。
 *
 * @param dst 至少 3 × dstW × dstH 个 float
 * @return false 表示参数非法（含 dstW 超过 WT_YOLO_MAX_INPUT）
 */
bool WtYolo_Letterbox(const unsigned char *rgb, int srcW, int srcH,
                      int dstW, int dstH, E_WtYoloResize resize,
                      float *dst, WtYoloLetterbox *lb);

/**
 * @brief 解码一帧输出：阈值 → 类别筛选 → 按得分排序 → 贪心 NMS → 还原到原图。
 *
 * NMS **不分类别**（class-agnostic）：同一个人被同时报成 pedestrian 与
 * people 时只留一个，这正是瞄准要的。
 *
 * @param data    输出张量，channels × numCand，通道优先
 * @param work    工作区，至少 numCand 个元素（避免每帧分配）
 * @param workCap 工作区容量
 * @return false 表示参数非法（通道数与布局不符、工作区不够等）；
 *         true 时 out->count 可以是 0
 */
bool WtYolo_Decode(const float *data, int channels, int numCand,
                   const WtYoloLayout *layout, const WtYoloLetterbox *lb,
                   const WtYoloFilter *filter,
                   WtYoloCand *work, int workCap, WtTrtResult *out);

#ifdef __cplusplus
}
#endif

#endif /* WT_YOLO_DECODE_H */
