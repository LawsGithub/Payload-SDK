/**
 * @file wt_vision.c
 * @brief 推理线程实现。
 */

#include "wt_vision.h"

#include "wt_ai_bridge.h"
#include "wt_aim.h"
#include "wt_frame_pump.h"
#include "wt_trt_pose.h"

#include <dji_logger.h>
#include <dji_platform.h>

#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static pthread_t s_thread;
static pthread_mutex_t s_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile bool s_running = false;
static bool s_threadStarted = false;

static WtTrtPose *s_pose = NULL;
static uint8_t *s_frameBuf = NULL;
static uint32_t s_intervalMs = 200;
static E_WtVisionModel s_model = WT_VISION_MODEL_TURBINE;

static WtVisionSnapshot s_snapshot;
static bool s_haveSnapshot = false;
/*
 * 快照对应帧的采集时刻（绝对值）。年龄必须在**读取时**算，不能在存储时
 * 算死成一个数字 —— 那样调用方拿到的永远是 0，而"这帧有多旧"正是它唯一
 * 能用来判断该不该据此下发云台角的依据。
 */
static uint32_t s_snapshotAtMs = 0;

static volatile uint32_t s_inferCount = 0;
static volatile uint32_t s_failCount = 0;
/* 快照序号（只在持锁时改） */
static uint32_t s_snapshotSeq = 0;

static uint32_t WtVision_NowMs(void)
{
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    uint32_t ms = 0;

    if (osal != NULL && osal->GetTimeMs != NULL) {
        osal->GetTimeMs(&ms);
    }

    return ms;
}

/**
 * 行人模型：每次推理都存快照，**空结果也存** —— 跟踪要分得清「这一帧没人」
 * （新帧号、空结果）与「没出新帧」（帧号不变）。
 */
static void WtVision_StorePedestrian(const WtTrtResult *trt, const WtFrameInfo *info)
{
    WtVisionSnapshot snap;

    memset(&snap, 0, sizeof(snap));
    snap.raw = *trt;
    snap.imageWidthPx = info->widthPx;
    snap.imageHeightPx = info->heightPx;
    snap.frameId = info->frameId;
    snap.capturedAtMs = info->capturedAtMs;
    snap.rawTargetCount = trt->count;
    snap.conf = (trt->count > 0) ? trt->items[0].conf : 0.0f;

    /* 与取流同一个教训：运行中周期性打，Stop 时打的那行永远看不到 */
    if (s_inferCount <= 3u || (s_inferCount % 25u) == 0u) {
        USER_LOG_INFO("视觉：第 %u 次推理 —— 检出 %d 人%s",
                      (unsigned)s_inferCount, trt->count,
                      trt->count > 0 ? "" : "（画面里没有行人？）");
        if (trt->count > 0) {
            const WtTrtDetection *b = &trt->items[0];

            USER_LOG_INFO("视觉：  最可信 conf %.2f 类别 %d，框 %.0f×%.0f px @ (%.0f, %.0f)",
                          (double)b->conf, b->classId,
                          (double)(b->bboxX2 - b->bboxX1), (double)(b->bboxY2 - b->bboxY1),
                          (double)(0.5f * (b->bboxX1 + b->bboxX2)),
                          (double)(0.5f * (b->bboxY1 + b->bboxY2)));
        }
    }

    pthread_mutex_lock(&s_lock);
    snap.seq = ++s_snapshotSeq;
    s_snapshot = snap;
    s_snapshotAtMs = info->capturedAtMs;
    s_haveSnapshot = true;
    pthread_mutex_unlock(&s_lock);
}

static void *WtVision_ThreadMain(void *arg)
{
    WtFrameInfo info;
    WtTrtResult trt;

    (void)arg;

    while (s_running) {
        if (!WtFramePump_TakeLatest(s_frameBuf, WT_FRAME_MAX_BYTES, &info)) {
            /* 暂无新帧 —— 睡一小会儿再看，不要空转占满一个核 */
            usleep(10 * 1000);
            continue;
        }

        if (!WtTrtPose_Infer(s_pose, s_frameBuf, info.widthPx, info.heightPx, &trt)) {
            s_failCount++;
            continue;
        }

        s_inferCount++;

        if (s_model == WT_VISION_MODEL_PEDESTRIAN) {
            WtVision_StorePedestrian(&trt, &info);
        } else {
            WtVisionSnapshot snap;

            memset(&snap, 0, sizeof(snap));
            snap.imageWidthPx = info.widthPx;
            snap.imageHeightPx = info.heightPx;
            snap.frameId = info.frameId;
            snap.capturedAtMs = info.capturedAtMs;
            snap.rawTargetCount = trt.count;

            if (trt.count > 0) {
                snap.conf = trt.items[0].conf;
            }

            /*
             * 运行中的推理诊断。
             *
             * 和取流那边同一个教训（2026-10-22 实测）：`dji_app_ctl stop`
             * 直接杀进程，只在 Stop 里打累计计数 = 永远看不到。而"推理在跑
             * 但检不出目标"和"推理根本没跑"在外部看起来一模一样，必须能分辨。
             *
             * 每 25 次推理（约 5 秒 @200ms）打一行，报检出数与最高 conf。
             */
            if (s_inferCount <= 3u || (s_inferCount % 25u) == 0u) {
                if (trt.count > 0) {
                    USER_LOG_INFO("视觉：第 %u 次推理 —— 检出 %d 个目标，"
                                  "最高 conf %.2f，轮毂像素 (%.0f, %.0f)",
                                  (unsigned)s_inferCount, trt.count,
                                  (double)trt.items[0].conf,
                                  (double)trt.items[0].kptX[3],
                                  (double)trt.items[0].kptY[3]);
                } else {
                    USER_LOG_INFO("视觉：第 %u 次推理 —— 未检出目标"
                                  "（画面里没有风机？）", (unsigned)s_inferCount);
                }
            }

            if (WtAiBridge_FromTrtResult(&trt, &snap.det)) {
                /*
                 * 排序必须在存快照之前。模型的三个叶尖输出是无序的，
                 * 采集编号又建立在"第几片"上 —— 不排序的话操作员采的 NO.1
                 * 和程序记的 NO.1 会对不上，且全程无报错。
                 */
                (void)WtAim_SortTipsByImageAngle(&snap.det, NULL);

                pthread_mutex_lock(&s_lock);
                snap.seq = ++s_snapshotSeq;
                s_snapshot = snap;
                s_snapshotAtMs = info.capturedAtMs;
                s_haveSnapshot = true;
                pthread_mutex_unlock(&s_lock);
            }
            /*
             * 没有检出目标时**不更新快照** —— 保留上一次的结果并让它自然变旧，
             * 比把快照清空更有用：调用方能从 ageMs 看出"多久没看到风机了"，
             * 而清空只会让它分不清"从没见过"和"刚刚丢了"。
             */
        }

        /* 采集场景不需要 30fps 推理，按配置的间隔节流 */
        if (s_intervalMs > 0) {
            usleep(s_intervalMs * 1000);
        }
    }

    return NULL;
}

bool WtVision_Start(const char *enginePath, E_WtVisionModel model,
                    float confThresh, float iouThresh, uint32_t intervalMs)
{
    WtYoloLayout layout;
    WtYoloFilter filter;
    E_WtYoloResize resize;

    if (s_running) {
        return true;
    }
    if (enginePath == NULL || enginePath[0] == '\0') {
        USER_LOG_ERROR("视觉：engine 路径为空");
        return false;
    }

    filter.confThresh = confThresh;
    filter.iouThresh = iouThresh;

    if (model == WT_VISION_MODEL_PEDESTRIAN) {
        /*
         * 类别数从 engine 反推（这份 VisDrone 权重实际是 11 类，不是标准的 10 类）；
         * 只收类别 0/1 = pedestrian/people。双线性缩放：行人只有十几个像素。
         */
        layout.numClasses = 0;
        layout.kptCount = 0;
        layout.kptDims = 0;
        filter.classMask = (1u << 0) | (1u << 1);
        resize = WT_YOLO_RESIZE_BILINEAR;
    } else {
        /* 风机：1 类 + 7 关键点 × 2，最近邻 —— 与拆分前完全相同 */
        layout.numClasses = 1;
        layout.kptCount = WT_TRT_KPT_COUNT;
        layout.kptDims = 2;
        filter.classMask = 0u;
        resize = WT_YOLO_RESIZE_NEAREST;
    }

    s_frameBuf = (uint8_t *)malloc(WT_FRAME_MAX_BYTES);
    if (s_frameBuf == NULL) {
        USER_LOG_ERROR("视觉：帧缓冲分配失败");
        return false;
    }

    s_pose = WtTrtPose_Init(enginePath, &layout, &filter, resize);
    if (s_pose == NULL) {
        USER_LOG_ERROR("视觉：engine 加载失败 %s（架构/TensorRT 版本不匹配？"
                       "或装错了 engine —— 原因见 stderr 的 [trt] 行）", enginePath);
        free(s_frameBuf);
        s_frameBuf = NULL;
        return false;
    }

    {
        WtYoloLayout real;
        int inW = 0;
        int inH = 0;
        int nCand = 0;

        WtTrtPose_GetShape(s_pose, &real, &inW, &inH, &nCand);
        USER_LOG_INFO("视觉：%s 模型，输入 %d×%d，输出 %d 类 + %d 关键点 × %d 候选，%s 缩放",
                      model == WT_VISION_MODEL_PEDESTRIAN ? "行人" : "风机",
                      inW, inH, real.numClasses, real.kptCount, nCand,
                      resize == WT_YOLO_RESIZE_BILINEAR ? "双线性" : "最近邻");
    }

    s_model = model;
    s_intervalMs = intervalMs;
    s_haveSnapshot = false;
    s_inferCount = 0;
    s_failCount = 0;
    s_running = true;

    if (pthread_create(&s_thread, NULL, WtVision_ThreadMain, NULL) != 0) {
        USER_LOG_ERROR("视觉：推理线程创建失败");
        s_running = false;
        WtTrtPose_Deinit(s_pose);
        s_pose = NULL;
        free(s_frameBuf);
        s_frameBuf = NULL;
        return false;
    }
    s_threadStarted = true;

    USER_LOG_INFO("视觉：推理线程已启动（engine=%s, 间隔 %u ms）",
                  enginePath, (unsigned)intervalMs);

    return true;
}

void WtVision_Stop(void)
{
    if (!s_running && !s_threadStarted) {
        return;
    }

    s_running = false;

    if (s_threadStarted) {
        pthread_join(s_thread, NULL);
        s_threadStarted = false;
    }

    if (s_pose != NULL) {
        WtTrtPose_Deinit(s_pose);
        s_pose = NULL;
    }

    free(s_frameBuf);
    s_frameBuf = NULL;

    USER_LOG_INFO("视觉：已停止（推理 %u 帧，失败 %u 次）",
                  (unsigned)s_inferCount, (unsigned)s_failCount);
}

bool WtVision_GetSnapshot(WtVisionSnapshot *out)
{
    bool got = false;

    if (out == NULL) {
        return false;
    }

    pthread_mutex_lock(&s_lock);
    if (s_haveSnapshot) {
        const uint32_t now = WtVision_NowMs();

        *out = s_snapshot;
        /* 无符号回绕（OSAL 的 GetTimeMs 是 uint32，约 49.7 天溢出）下这个
         * 减法依然给出正确的差值，所以不另做保护。 */
        out->ageMs = now - s_snapshotAtMs;
        got = true;
    }
    pthread_mutex_unlock(&s_lock);

    return got;
}

uint32_t WtVision_InferCount(void)
{
    return s_inferCount;
}

uint32_t WtVision_FailCount(void)
{
    return s_failCount;
}
