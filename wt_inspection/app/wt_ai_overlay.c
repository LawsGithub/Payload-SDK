/**
 * @file wt_ai_overlay.c
 * @brief AI 叠加验证实现：行人框随帧编码，推给 Pilot。
 *
 * ## 为什么重写（2026-10-26）
 *
 * 2026-10-21 版只单独调 `DjiLiveview_SendAiMetaToPilot`：连续 36 次返回 SUCCESS，
 * Pilot 2 上一个框都没有，当时归因为「Pilot 2 的 AI 识别与 PSDK 应用互斥 ⇒ 死路」。
 *
 * 但官方样例 `dji_liveview_object_detection.cpp`（相机源选项专门列了
 * "M4 serials 4K"）比我们多两步：
 *
 *   取图 → 识别 → DjiLiveview_EncodeAFrameToH264(帧, 框)     把框随帧编进 H.264
 *                → 编码回调里 DjiPayloadCamera_SendVideoStream  推给 Pilot
 *
 * 框画在妙算3 推上去的**这一路视频**上，不在飞机主相机的画面上。旧版没有推流，
 * Pilot 没有可以画框的画面 —— 「互斥」未必是原因。本版照样例补齐。
 *
 * ## 挂在哪
 *
 * 不自己取流，也不自己 DjiLiveview_Init / Deinit：同一相机源不能开两路图像流，
 * 而旧版 Stop 里的 Deinit 会连带关掉 wt_frame_pump 的取流。这里通过
 * `WtFramePump_SetTap` 拿到每一帧，在 SDK 回调线程里同步交给编码器。
 * 框来自行人链路的推理快照（`WtVision_GetSnapshot`，只读、不取走）。
 * 其中「拨开关会锁谁 / 正在跟谁」的那个标成 `target`，由 `WtPed_TargetIndex`
 * 给出 —— 判据与决策层同一个，这里不另写。
 *
 * 2026-10-26 上机：Pilot 2 多出一路妙算3 的视频源，画面上能看到框 —— 这条路通了。
 *
 * ## 原文约束
 *
 * - EncodeAFrameToH264 必须在 RegEncoderCallback 之后调用（dji_liveview.h:309）
 * - SendVideoStream 单次 ≤ 65000 字节（dji_payload_camera.h:843）；官方负载相机
 *   样例按 60000 分片发（test_payload_cam_emu_media.c），这里照做
 * - 实际码率超过视频流通道的实时带宽会被丢（dji_payload_camera.h:837）——
 *   所以运行中定期打印 GetVideoStreamState
 * - 带宽比例照样例设 {10, 60, 30}。头文件说要先注册网络 HAL 才生效，本应用只注册了
 *   USB bulk —— 返回码照打，失败不阻断
 * - 样例没有调 DjiPayloadCamera_Init，这里也不调（上机确认不调也能显示）
 * - GetVideoStreamState 在这条路上始终报 0（Pilot 2 看得到画面时也是），只作参考
 */

#include "wt_ai_overlay.h"

#include "wt_frame_pump.h"
#if defined(WT_WITH_TRT)
#include "wt_ped.h"
#include "wt_vision.h"
#endif

#include <dji_high_speed_data_channel.h>
#include <dji_liveview.h>
#include <dji_logger.h>
#include <dji_payload_camera.h>
#include <dji_platform.h>

#include <string.h>

/* ------------------------------------------------------------------ */
/* 固定参数                                                            */
/* ------------------------------------------------------------------ */

/*
 * 标签下标即框的 type（官方样例从 0 起）。用 ASCII：Pilot 能否渲染中文没验证过，
 * 别让字体问题混进这次的结论。
 */
static const char *s_labels[] = {
    "pedestrian",   /* 0：行人模型类别 0 */
    "people",       /* 1：行人模型类别 1（坐着、骑车的人） */
    "target",       /* 2：拨「行人瞄准」会锁定的人 / 会话中正在跟的人 */
};
#define WT_OVERLAY_LABEL_COUNT ((uint8_t)(sizeof(s_labels) / sizeof(s_labels[0])))
#define WT_OVERLAY_TYPE_PEOPLE 1u
#define WT_OVERLAY_TYPE_TARGET 2u

/* 坐标单位是万分之一画面宽/高（0..10000），见 dji_liveview.h:150-153 */
#define WT_OVERLAY_COORD_FULL 10000.0f

/* 一帧最多带几个框。超出的按推理给出的顺序截掉（target 一定在前面就不会被截：见 RefreshMeta）。 */
#define WT_OVERLAY_MAX_BOXES 32

/* SendVideoStream 单次上限 65000 字节；官方负载相机样例用 60000 */
#define WT_OVERLAY_SEND_CHUNK 60000u

/* 框多久刷新一次：推理约 6 次/秒，没必要每帧（30 fps）都去拿推理线程的锁 */
#define WT_OVERLAY_BOX_REFRESH_MS 100u

/* 快照比这更旧就不画人框 —— 推理停了或卡住时，别让旧框一直挂在画面上 */
#define WT_OVERLAY_BOX_MAX_AGE_MS 1000u

/* 每多少帧打一行统计（5 秒 @30fps） */
#define WT_OVERLAY_LOG_EVERY_FRAMES 150u

/* 高速通道带宽比例：样例值，与头文件写明的默认值（停止或启动失败时还原） */
static const T_DjiDataChannelBandwidthProportionOfHighspeedChannel s_bandwidthOverlay = {10, 60, 30};
static const T_DjiDataChannelBandwidthProportionOfHighspeedChannel s_bandwidthDefault = {33, 33, 34};

/* boxData 是变长数组（`boxData[1]`），按实际容量自己声明，布局与 SDK 的一致 */
typedef struct {
    uint8_t boxCount;
    T_DjiLiveViewBoundingBox boxData[WT_OVERLAY_MAX_BOXES];
} __attribute__((packed)) WtOverlayMeta;

/* ------------------------------------------------------------------ */
/* 状态                                                                */
/* ------------------------------------------------------------------ */

static bool s_started = false;
static bool s_labelsRegistered = false;
static bool s_encoderRegistered = false;

/* 以下只在 SDK 图像回调线程里读写 */
static WtOverlayMeta s_meta;
static bool s_metaValid = false;
static uint32_t s_metaAtMs = 0;
#if defined(WT_WITH_TRT)
static WtVisionSnapshot s_snap;   /* 约 5 KB，不放 SDK 线程的栈上 */
#endif
static int s_lastTarget = -1;     /* 最近一次刷新时 target 在快照里的下标，只用于打印 */
static uint32_t s_frames = 0;
static uint32_t s_encodeFail = 0;
static uint32_t s_metaFail = 0;
static uint32_t s_encodeSumMs = 0;
static uint32_t s_encodeMaxMs = 0;
static T_DjiReturnCode s_lastEncodeRc = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

/* 编码回调线程写、图像回调线程读 —— 只用于打印，读到略旧的值无妨 */
static volatile uint32_t s_encOut = 0;
static volatile uint32_t s_encBytes = 0;
static volatile uint32_t s_sendFail = 0;
static volatile T_DjiReturnCode s_lastSendRc = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

/* ------------------------------------------------------------------ */
/* 实现                                                                */
/* ------------------------------------------------------------------ */

static uint32_t WtAiOverlay_NowMs(void)
{
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    uint32_t ms = 0;

    if (osal != NULL && osal->GetTimeMs != NULL) {
        osal->GetTimeMs(&ms);
    }

    return ms;
}

#if defined(WT_WITH_TRT)
/* 画面比例（0..1）→ 万分比，夹到 [0, 10000]；NaN 当 0 */
static uint16_t WtAiOverlay_Coord(float frac)
{
    const float v = frac * WT_OVERLAY_COORD_FULL;

    if (!(v > 0.0f)) {
        return 0;
    }
    if (v >= WT_OVERLAY_COORD_FULL) {
        return (uint16_t)WT_OVERLAY_COORD_FULL;
    }
    return (uint16_t)(v + 0.5f);
}
#endif

#if defined(WT_WITH_TRT)
/* 往 s_meta 追加 s_snap 里的一个框（原图像素 → 万分比）；满了或退化框不加。返回新的框数 */
static uint8_t WtAiOverlay_AddBox(uint8_t n, int index, uint8_t type)
{
    const WtTrtDetection *d = &s_snap.raw.items[index];
    const float w = (float)s_snap.imageWidthPx;
    const float h = (float)s_snap.imageHeightPx;
    const uint16_t bw = WtAiOverlay_Coord((d->bboxX2 - d->bboxX1) / w);
    const uint16_t bh = WtAiOverlay_Coord((d->bboxY2 - d->bboxY1) / h);

    if (n >= WT_OVERLAY_MAX_BOXES || bw == 0 || bh == 0) {
        return n;
    }
    s_meta.boxData[n].id = (uint16_t)(index + 1);
    s_meta.boxData[n].type = type;
    s_meta.boxData[n].state = DJI_LIVEVIEW_OBJ_STATE_TRACKED;
    s_meta.boxData[n].box.cx = WtAiOverlay_Coord((d->bboxX1 + d->bboxX2) * 0.5f / w);
    s_meta.boxData[n].box.cy = WtAiOverlay_Coord((d->bboxY1 + d->bboxY2) * 0.5f / h);
    s_meta.boxData[n].box.w = bw;
    s_meta.boxData[n].box.h = bh;

    return (uint8_t)(n + 1);
}
#endif

/* 重建 s_meta：最近一次推理的行人框，其中「目标」标成 target */
static void WtAiOverlay_RefreshMeta(void)
{
    uint8_t n = 0;

    memset(&s_meta, 0, sizeof(s_meta));
    s_lastTarget = -1;

#if defined(WT_WITH_TRT)
    if (WtVision_GetSnapshot(&s_snap) && s_snap.ageMs <= WT_OVERLAY_BOX_MAX_AGE_MS &&
        s_snap.imageWidthPx > 0 && s_snap.imageHeightPx > 0) {
        const int target = WtPed_TargetIndex(&s_snap);
        int i;

        /* target 先放：框数超过上限时，被截掉的只会是别人 */
        if (target >= 0 && target < s_snap.raw.count) {
            n = WtAiOverlay_AddBox(n, target, (uint8_t)WT_OVERLAY_TYPE_TARGET);
            s_lastTarget = target;
        }
        for (i = 0; i < s_snap.raw.count && n < WT_OVERLAY_MAX_BOXES; i++) {
            if (i == target) {
                continue;
            }
            n = WtAiOverlay_AddBox(n, i,
                                   (s_snap.raw.items[i].classId == (int)WT_OVERLAY_TYPE_PEOPLE)
                                       ? (uint8_t)WT_OVERLAY_TYPE_PEOPLE : 0u);
        }
    }
#endif

    s_meta.boxCount = n;
}

/*
 * 编码输出 → Pilot。跑在编码器的回调线程上。
 * 一帧要是有一片发失败，剩下的就不发了：半帧数据只会让解码花屏。
 */
static void WtAiOverlay_EncoderCallback(const uint8_t *buf, uint32_t len)
{
    uint32_t sent = 0;

    s_encOut++;
    s_encBytes += len;

    if (buf == NULL) {
        return;
    }

    while (sent < len) {
        const uint32_t chunk = (len - sent > WT_OVERLAY_SEND_CHUNK) ? WT_OVERLAY_SEND_CHUNK
                                                                     : (len - sent);
        const T_DjiReturnCode rc = DjiPayloadCamera_SendVideoStream(buf + sent, chunk);

        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            s_sendFail++;
            s_lastSendRc = rc;
            return;
        }
        sent += chunk;
    }
}

static void WtAiOverlay_LogStats(void)
{
    T_DjiDataChannelState st;
    T_DjiReturnCode rc;

    USER_LOG_INFO("AI 叠加：第 %u 帧，%u 个框（target #%d）；编码 均 %u ms / 最大 %u ms，失败 %u（末次 0x%08X）；"
                  "编码输出 %u 段 / %u KB，推流失败 %u（末次 0x%08X）；发框失败 %u",
                  (unsigned)s_frames, (unsigned)s_meta.boxCount, s_lastTarget,
                  (unsigned)(s_frames > 0 ? s_encodeSumMs / s_frames : 0),
                  (unsigned)s_encodeMaxMs, (unsigned)s_encodeFail, (unsigned)s_lastEncodeRc,
                  (unsigned)s_encOut, (unsigned)(s_encBytes / 1024u),
                  (unsigned)s_sendFail, (unsigned)s_lastSendRc, (unsigned)s_metaFail);

    memset(&st, 0, sizeof(st));
    rc = DjiPayloadCamera_GetVideoStreamState(&st);
    USER_LOG_INFO("AI 叠加：视频流通道 rc=0x%08X 限额 %d B/s，流控前 %d B/s，流控后 %d B/s，busy=%d",
                  (unsigned)rc, (int)st.realtimeBandwidthLimit,
                  (int)st.realtimeBandwidthBeforeFlowController,
                  (int)st.realtimeBandwidthAfterFlowController, (int)st.busyState);
}

/* 帧旁路：跑在 SDK 图像回调线程上，必须快 */
static void WtAiOverlay_OnFrame(const uint8_t *buf, uint32_t len, int widthPx, int heightPx,
                                int pixFmt, uint32_t frameId)
{
    T_DjiLiveviewImageInfo info;
    T_DjiReturnCode rc;
    uint32_t t0;
    uint32_t dt;
    const uint32_t now = WtAiOverlay_NowMs();

    if (!s_metaValid || (uint32_t)(now - s_metaAtMs) >= WT_OVERLAY_BOX_REFRESH_MS) {
        WtAiOverlay_RefreshMeta();
        s_metaAtMs = now;
        s_metaValid = true;
    }

    /* 照官方样例（无 OpenCV 分支）：每帧既单独发框，也把框随帧编码 */
    rc = DjiLiveview_SendAiMetaToPilot((T_DjiLiveViewStandardMetaData *)&s_meta);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        s_metaFail++;
    }

    memset(&info, 0, sizeof(info));
    info.pixFmt = (E_DjiLiveViewPixFormate)pixFmt;
    info.width = (uint16_t)widthPx;
    info.height = (uint16_t)heightPx;
    info.frameId = frameId;

    t0 = WtAiOverlay_NowMs();
    rc = DjiLiveview_EncodeAFrameToH264(buf, len, info, (T_DjiLiveViewStandardMetaData *)&s_meta);
    dt = WtAiOverlay_NowMs() - t0;

    s_frames++;
    s_encodeSumMs += dt;
    if (dt > s_encodeMaxMs) {
        s_encodeMaxMs = dt;
    }
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        s_encodeFail++;
        s_lastEncodeRc = rc;
    }

    /* 前几帧 + 之后每 5 秒：诊断必须在运行中可见（dji_app_ctl stop 不走收尾） */
    if (s_frames <= 3u || (s_frames % WT_OVERLAY_LOG_EVERY_FRAMES) == 0u) {
        WtAiOverlay_LogStats();
    }
}

bool WtAiOverlay_Start(bool enabled)
{
    T_DjiReturnCode rc;

    if (!enabled) {
        USER_LOG_INFO("AI 叠加验证：配置未启用，跳过");
        return false;
    }
    if (s_started) {
        return true;
    }

    /* 样例顺序：带宽比例 → 标签 → 编码回调 → 取流（已由 wt_frame_pump 启动）→ 每帧编码 */
    rc = DjiHighSpeedDataChannel_SetBandwidthProportion(s_bandwidthOverlay);
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_INFO("AI 叠加：高速通道带宽比例设为 数据 10 / 视频 60 / 下载 30");
    } else {
        USER_LOG_WARN("AI 叠加：设带宽比例失败 rc=0x%08X（按默认 33/33/34 继续）", (unsigned)rc);
    }

    /*
     * 必须在 DjiLiveview_Init 之后 —— 漏掉时返回 0xE0 并打
     * "Ai target Lable only support on manifold3"，看起来像硬件不支持
     * （2026-10-21 实测）。这里的 Init 由 wt_frame_pump 做过了。
     */
    rc = DjiLiveview_RegUserAiTargetLableList(WT_OVERLAY_LABEL_COUNT, s_labels);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("AI 叠加：注册标签失败 rc=0x%08X", (unsigned)rc);
        (void)DjiHighSpeedDataChannel_SetBandwidthProportion(s_bandwidthDefault);
        return false;
    }
    s_labelsRegistered = true;

    rc = DjiLiveview_RegEncoderCallback(WtAiOverlay_EncoderCallback);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("AI 叠加：注册编码回调失败 rc=0x%08X", (unsigned)rc);
        (void)DjiLiveview_UnregUserAiTargetLableList();
        s_labelsRegistered = false;
        (void)DjiHighSpeedDataChannel_SetBandwidthProportion(s_bandwidthDefault);
        return false;
    }
    s_encoderRegistered = true;

    s_metaValid = false;
    s_frames = 0;
    s_encodeFail = 0;
    s_metaFail = 0;
    s_encodeSumMs = 0;
    s_encodeMaxMs = 0;
    s_encOut = 0;
    s_encBytes = 0;
    s_sendFail = 0;

    WtFramePump_SetTap(WtAiOverlay_OnFrame);
    s_started = true;
    USER_LOG_INFO("AI 叠加：已挂到取流上 —— 每帧随框编码推给 Pilot（标签 pedestrian / people / target）");

    return true;
}

void WtAiOverlay_Stop(void)
{
    T_DjiOsalHandler *osal;

    if (!s_started) {
        return;
    }

    WtFramePump_SetTap(NULL);

    /* 图像回调可能正在编码这一帧：给它一帧多的时间再注销 */
    osal = DjiPlatform_GetOsalHandler();
    if (osal != NULL) {
        osal->TaskSleepMs(100);
    }

    if (s_encoderRegistered) {
        (void)DjiLiveview_UnregEncoderCallback();
        s_encoderRegistered = false;
    }
    if (s_labelsRegistered) {
        const T_DjiReturnCode rc = DjiLiveview_UnregUserAiTargetLableList();

        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_WARN("AI 叠加：注销标签失败 rc=0x%08X", (unsigned)rc);
        }
        s_labelsRegistered = false;
    }
    (void)DjiHighSpeedDataChannel_SetBandwidthProportion(s_bandwidthDefault);

    s_started = false;
    USER_LOG_INFO("AI 叠加已停止：共 %u 帧，编码输出 %u 段，推流失败 %u",
                  (unsigned)s_frames, (unsigned)s_encOut, (unsigned)s_sendFail);
}
