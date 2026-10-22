/**
 * @file wt_ai_stream_probe.c
 * @brief 最小取流探针：验证 DjiLiveview_StartImageStream 能否拿到相机帧。
 *
 * ## 为什么单独一个文件
 *
 * 2026-10-21 实测：AI 叠加（`SendAiMetaToPilot`）返回 SUCCESS 但 Pilot 2
 * 不渲染。要区分"通道不通"和"Pilot 2 渲染条件不满足"，最直接的办法是
 * 看**图像流本身能不能拿到**。
 *
 * ## 实测结论（2026-10-21，M4T + 外挂妙算3）
 *
 * ```
 * StartImageStream 成功，446 ms 后收到第一帧
 * 1440x1080  PIXFMT_RGB_PACKED  4,665,600 字节/帧  约 30 fps
 * ```
 *
 * ⇒ **妙算3 能拿到 M4T 的实时画面**，「持续跟踪」方案的前提成立。
 *    这是整个自研识别链路的入口。
 *
 * ## 一个必须记住的坑
 *
 * 必须先调 `DjiLiveview_Init()`。漏掉它的后果极具误导性：直接调
 * `RegUserAiTargetLableList` 会返回 0xE0，日志打
 * `Ai target Lable only support on manifold3` —— 看起来像硬件不支持，
 * 实际只是模块没初始化（SDK 内部缓存的 `s_mountPositionType` 还是 0）。
 * 反汇编确认该变量正是在 `DjiLiveview_Init` 里由
 * `DjiAccessAdapter_GetMountPositionType()` 填充的。
 *
 * ## 用法
 *
 * `[app] stream_probe = true`，跑 N 秒看日志里的帧计数。
 */

#include "wt_ai_stream_probe.h"

#include <dji_liveview.h>
#include <dji_logger.h>
#include <dji_platform.h>

#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* 探针参数                                                            */
/* ------------------------------------------------------------------ */

/*
 * M4T 的可见光相机：位置用默认（内置相机），source 用 M4T_VIS。
 * 若收不到帧，可改试 M4T_4K（source=3）。
 */
#define WT_PROBE_POSITION DJI_LIVEVIEW_CAMERA_POSITION_NO_1
#define WT_PROBE_SOURCE   DJI_LIVEVIEW_CAMERA_SOURCE_M4T_VIS
#define WT_PROBE_PIXFMT   PIXFMT_RGB_PACKED

/* ------------------------------------------------------------------ */
/* 状态                                                                */
/* ------------------------------------------------------------------ */

static volatile uint32_t s_frameCount = 0;
static volatile uint32_t s_lastWidth = 0;
static volatile uint32_t s_lastHeight = 0;
static volatile uint32_t s_lastFrameBytes = 0;
static bool s_liveviewInited = false;

/* ------------------------------------------------------------------ */
/* 回调                                                                */
/* ------------------------------------------------------------------ */

static void WtProbe_ImageCallback(E_DjiLiveViewCameraPosition position,
                                  const uint8_t *buf, uint32_t len,
                                  T_DjiLiveviewImageInfo imageInfo)
{
    const uint32_t n = ++s_frameCount;

    (void)position; /* 只订阅了一个源，位置无需区分 */
    (void)buf;      /* 暂不处理像素，只看有没有帧 */

    s_lastWidth = imageInfo.width;
    s_lastHeight = imageInfo.height;
    s_lastFrameBytes = len;

    /*
     * 只在前几帧和之后每 30 帧打一行 —— 30fps 下每帧打会淹没日志，
     * 但完全不打又看不出"到底有没有在收"。
     */
    if (n <= 3 || (n % 30) == 0) {
        USER_LOG_INFO("取流探针：第 %u 帧 %ux%u pixFmt=%d bytes=%u",
                      (unsigned)n, (unsigned)imageInfo.width,
                      (unsigned)imageInfo.height, (int)imageInfo.pixFmt,
                      (unsigned)len);
    }
}

/* ------------------------------------------------------------------ */
/* 实现                                                                */
/* ------------------------------------------------------------------ */

bool WtAiStreamProbe_Start(bool enabled)
{
    T_DjiReturnCode rc;

    if (!enabled) {
        return false;
    }

    rc = DjiLiveview_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("取流探针：DjiLiveview_Init 失败 rc=0x%08X", (unsigned)rc);
        return false;
    }
    s_liveviewInited = true;

    rc = DjiLiveview_StartImageStream(WT_PROBE_POSITION, WT_PROBE_SOURCE,
                                      WT_PROBE_PIXFMT, WtProbe_ImageCallback);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("取流探针：StartImageStream 失败 rc=0x%08X", (unsigned)rc);
        return false;
    }

    USER_LOG_INFO("取流探针：StartImageStream 成功，等待帧…");
    return true;
}

void WtAiStreamProbe_Stop(void)
{
    T_DjiReturnCode rc;

    rc = DjiLiveview_StopImageStream(WT_PROBE_POSITION, WT_PROBE_SOURCE);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_WARN("取流探针：StopImageStream rc=0x%08X", (unsigned)rc);
    }

    if (s_liveviewInited) {
        (void)DjiLiveview_Deinit();
        s_liveviewInited = false;
    }

    USER_LOG_INFO("取流探针：结束，共收 %u 帧（最后一帧 %ux%u，%u 字节）",
                  (unsigned)s_frameCount, (unsigned)s_lastWidth,
                  (unsigned)s_lastHeight, (unsigned)s_lastFrameBytes);
}

uint32_t WtAiStreamProbe_FrameCount(void)
{
    return s_frameCount;
}
