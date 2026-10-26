/**
 * @file wt_frame_pump.c
 * @brief 取流与帧缓冲实现。
 */

#include "wt_frame_pump.h"

#include <dji_liveview.h>
#include <dji_logger.h>
#include <dji_platform.h>

#include <pthread.h>
#include <stdlib.h>
#include <string.h>

/*
 * 相机位置与源：与 wt_ai_stream_probe.c 保持一致（那条路已实测通过
 * 1440×1080 RGB_PACKED @30fps）。改这里之前先确认探针能收到帧。
 */
#define WT_PUMP_POSITION DJI_LIVEVIEW_CAMERA_POSITION_NO_1
#define WT_PUMP_SOURCE   DJI_LIVEVIEW_CAMERA_SOURCE_M4T_VIS
#define WT_PUMP_PIXFMT   PIXFMT_RGB_PACKED

static pthread_mutex_t s_lock = PTHREAD_MUTEX_INITIALIZER;
static uint8_t *s_slot = NULL;
static WtFrameInfo s_slotInfo;
static bool s_slotReady = false;
static bool s_started = false;
static bool s_liveviewInited = false;

static volatile uint32_t s_received = 0;
static volatile uint32_t s_dropped = 0;

static volatile WtFrameTap s_tap = NULL;

static uint32_t WtPump_NowMs(void)
{
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    uint32_t ms = 0;

    if (osal != NULL && osal->GetTimeMs != NULL) {
        osal->GetTimeMs(&ms);
    }

    return ms;
}

static void WtPump_ImageCallback(E_DjiLiveViewCameraPosition position,
                                 const uint8_t *buf, uint32_t len,
                                 T_DjiLiveviewImageInfo imageInfo)
{
    WtFrameTap tap;

    (void)position; /* 只订阅一个源 */

    s_received++;

    if (buf == NULL || len == 0) {
        s_dropped++;
        return;
    }
    if (len > WT_FRAME_MAX_BYTES) {
        /*
         * 超出缓冲上限：丢帧而不是截断。截断得到的是一张下半部分是垃圾的图，
         * 推理会给出"看起来合理"的错误关键点 —— 比没有结果危险得多。
         */
        s_dropped++;
        USER_LOG_WARN("取流：帧长 %u 超出缓冲上限 %u，已丢弃",
                      (unsigned)len, (unsigned)WT_FRAME_MAX_BYTES);
        return;
    }

    /* 帧旁路（AI 叠加）：先交给编码器，再拷进推理缓冲 —— 见 WtFramePump_SetTap */
    tap = s_tap;
    if (tap != NULL) {
        tap(buf, len, (int)imageInfo.width, (int)imageInfo.height,
            (int)imageInfo.pixFmt, imageInfo.frameId);
    }

    /*
     * trylock：抢不到就丢帧，绝不在 SDK 回调线程里等锁。
     * 消费者持锁的时间只有一次 memcpy，正常情况抢不到的概率很低。
     */
    if (pthread_mutex_trylock(&s_lock) != 0) {
        s_dropped++;
        return;
    }

    if (s_slot == NULL) {
        pthread_mutex_unlock(&s_lock);
        s_dropped++;
        return;
    }

    /* 未被取走的旧帧直接被覆盖 —— latest-frame-wins */
    if (s_slotReady) {
        s_dropped++;
    }

    memcpy(s_slot, buf, len);
    s_slotInfo.widthPx = (int)imageInfo.width;
    s_slotInfo.heightPx = (int)imageInfo.height;
    s_slotInfo.frameId = imageInfo.frameId;
    s_slotInfo.byteCount = (size_t)len;
    s_slotInfo.capturedAtMs = WtPump_NowMs();
    s_slotReady = true;

    pthread_mutex_unlock(&s_lock);

    /*
     * 前几帧 + 之后每 150 帧（5 秒 @30fps）打一行。
     *
     * 为什么必须打：上一版只在 Stop 里打累计计数，而 `dji_app_ctl stop`
     * 是**直接杀进程**、不走收尾路径（2026-10-22 实测：收尾日志一行都没有）
     * ⇒ 那个计数永远看不到，于是"取流到底有没有收到帧"无法判断。
     * 诊断信息必须在**运行中**就能看到，不能只在退出时打。
     */
    if (s_received <= 3u || (s_received % 150u) == 0u) {
        USER_LOG_INFO("取流：第 %u 帧 %ux%u pixFmt=%d %u 字节（累计丢 %u）",
                      (unsigned)s_received, (unsigned)imageInfo.width,
                      (unsigned)imageInfo.height, (int)imageInfo.pixFmt,
                      (unsigned)len, (unsigned)s_dropped);
    }
}

bool WtFramePump_Start(void)
{
    T_DjiReturnCode rc;

    if (s_started) {
        return true;
    }

    s_slot = (uint8_t *)malloc(WT_FRAME_MAX_BYTES);
    if (s_slot == NULL) {
        USER_LOG_ERROR("取流：帧缓冲分配失败（%u 字节）",
                       (unsigned)WT_FRAME_MAX_BYTES);
        return false;
    }
    memset(&s_slotInfo, 0, sizeof(s_slotInfo));
    s_slotReady = false;
    s_received = 0;
    s_dropped = 0;

    rc = DjiLiveview_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        /*
         * 漏掉 Init 的后果极具误导性（见 wt_ai_stream_probe.c 的记录）：
         * 后续调用会报 "only support on manifold3"，看起来像硬件不支持。
         */
        USER_LOG_ERROR("取流：DjiLiveview_Init 失败 rc=0x%08X", (unsigned)rc);
        free(s_slot);
        s_slot = NULL;
        return false;
    }
    s_liveviewInited = true;

    rc = DjiLiveview_StartImageStream(WT_PUMP_POSITION, WT_PUMP_SOURCE,
                                      WT_PUMP_PIXFMT, WtPump_ImageCallback);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("取流：StartImageStream 失败 rc=0x%08X", (unsigned)rc);
        (void)DjiLiveview_Deinit();
        s_liveviewInited = false;
        free(s_slot);
        s_slot = NULL;
        return false;
    }

    s_started = true;
    USER_LOG_INFO("取流已启动，等待首帧（实测约 450 ms 出首帧）");

    return true;
}

void WtFramePump_Stop(void)
{
    if (!s_started) {
        return;
    }

    s_tap = NULL;
    (void)DjiLiveview_StopImageStream(WT_PUMP_POSITION, WT_PUMP_SOURCE);

    if (s_liveviewInited) {
        (void)DjiLiveview_Deinit();
        s_liveviewInited = false;
    }

    pthread_mutex_lock(&s_lock);
    free(s_slot);
    s_slot = NULL;
    s_slotReady = false;
    pthread_mutex_unlock(&s_lock);

    s_started = false;

    USER_LOG_INFO("取流已停止：收 %u 帧，丢 %u 帧",
                  (unsigned)s_received, (unsigned)s_dropped);
}

bool WtFramePump_TakeLatest(uint8_t *dst, size_t dstCap, WtFrameInfo *outInfo)
{
    bool got = false;

    if (dst == NULL || outInfo == NULL) {
        return false;
    }

    pthread_mutex_lock(&s_lock);

    if (s_slotReady && s_slot != NULL && s_slotInfo.byteCount <= dstCap) {
        memcpy(dst, s_slot, s_slotInfo.byteCount);
        *outInfo = s_slotInfo;
        s_slotReady = false;
        got = true;
    }

    pthread_mutex_unlock(&s_lock);

    return got;
}

uint32_t WtFramePump_ReceivedCount(void)
{
    return s_received;
}

uint32_t WtFramePump_DroppedCount(void)
{
    return s_dropped;
}

void WtFramePump_SetTap(WtFrameTap tap)
{
    s_tap = tap;
}
