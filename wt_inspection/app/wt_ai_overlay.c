/**
 * @file wt_ai_overlay.c
 * @brief PSDK 目标识别叠加验证实现。
 *
 * 只做一件事：按固定节奏往 Pilot 2 发几个框，用来确认 API 能用、
 * 以及自定义标签索引从几开始。**不是生产代码**，验证完就关。
 *
 * ## 实测结论（2026-10-21，M4T + 外挂妙算3）
 *
 * API 链路完全可用：
 *
 *   DjiLiveview_Init()                    成功（日志 `Agent stream init success`）
 *   DjiLiveview_RegUserAiTargetLableList  成功
 *   DjiLiveview_SendAiMetaToPilot         连续 36 次返回 SUCCESS
 *
 * **但 Pilot 2 不渲染这些框。** 原因不是 API 问题 —— 是 Pilot 2 的
 * 「AI 识别」功能与妙算3 上的 PSDK 应用**互斥**：开着 DJI AI 时 Pilot 2
 * 不接收外部框；关掉 DJI AI 后它又不在 AI 模式，同样不渲染。
 *
 * ⇒ **这条显示路径对"自己内嵌识别"的场景是死路。**
 *    识别结果要自己用（反投影算坐标），不要指望画给操作员看。
 *
 * 对照：`DjiLiveview_StartImageStream` 同样可用（30fps 1440×1080 RGB），
 * 见 `wt_ai_stream_probe.c` —— **取流才是正道，叠加不是**。
 */

#include "wt_ai_overlay.h"

#include <dji_liveview.h>
#include <dji_logger.h>
#include <dji_platform.h>

#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* 验证用的固定参数                                                     */
/* ------------------------------------------------------------------ */

/*
 * 三个自定义标签。名字本身不重要，重要的是**它们出现在框旁边时，
 * 能反推出 type 索引的起点**：
 *
 *   - 若"左/中/右"三个位置分别显示"标签0/标签1/标签2" → 索引从 0 起
 *   - 若显示错位（比如左位置显示"标签1"）→ 索引从 1 起，或映射另有规则
 *   - 若某位置不显示标签 → 该 type 值非法
 *
 * 用 char * 而非 const char *：SDK 的签名是 `const char *labels[]`
 * （即 char 数组指针的指针，元素本身非 const），传 const 数组会丢限定符。
 */
static const char *s_labels[] = {
    "标签0-裂纹",
    "标签1-油污",
    "标签2-雷击",
};
#define WT_AI_LABEL_COUNT ((uint8_t)(sizeof(s_labels) / sizeof(s_labels[0])))

/* 坐标单位是万分之一屏幕宽/高（0..10000），见 dji_liveview.h:150-154 */
#define WT_AI_COORD_FULL 10000u

/* 每个框：宽 12% 屏宽、高 18% 屏高，居中于给定 cx */
#define WT_AI_BOX_W 1200u
#define WT_AI_BOX_H 1800u

/* 三个框的水平位置：左 1/4、中 1/2、右 3/4 */
static const uint16_t s_box_cx[3] = {
    WT_AI_COORD_FULL / 4,
    WT_AI_COORD_FULL / 2,
    WT_AI_COORD_FULL * 3 / 4,
};

/* 发送节奏：1 Hz。太快没有额外信息，只刷屏。 */
#define WT_AI_SEND_INTERVAL_MS 1000

/* ------------------------------------------------------------------ */
/* 状态                                                                */
/* ------------------------------------------------------------------ */

static bool s_started = false;
static bool s_liveviewInited = false;
static bool s_labelsRegistered = false;
static T_DjiTaskHandle s_thread = NULL;
static volatile bool s_stopRequested = false;

/* ------------------------------------------------------------------ */
/* 实现                                                                */
/* ------------------------------------------------------------------ */

static void *WtAiOverlay_Thread(void *arg)
{
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    uint32_t tick = 0;

    (void)arg;

    USER_LOG_INFO("AI 叠加验证：线程启动，每秒发 3 个框（type 0/1/2，位置左/中/右）");

    while (!s_stopRequested) {
        /*
         * boxData 是变长数组（`T_DjiLiveViewBoundingBox boxData[1]`），
         * 声明了 3 个就必须按 3 个算大小，不能只传 sizeof(结构体)。
         */
        struct {
            uint8_t boxCount;
            T_DjiLiveViewBoundingBox boxData[WT_AI_LABEL_COUNT];
        } __attribute__((packed)) meta;
        T_DjiReturnCode rc;
        uint8_t i;

        memset(&meta, 0, sizeof(meta));
        meta.boxCount = WT_AI_LABEL_COUNT;

        for (i = 0; i < WT_AI_LABEL_COUNT; i++) {
            meta.boxData[i].id = (uint16_t)(i + 1);
            meta.boxData[i].type = i;                     /* 0/1/2 —— 验证点 */
            meta.boxData[i].state = DJI_LIVEVIEW_OBJ_STATE_TRACKED;
            meta.boxData[i].box.cx = s_box_cx[i];
            meta.boxData[i].box.cy = WT_AI_COORD_FULL / 2;
            meta.boxData[i].box.w = WT_AI_BOX_W;
            meta.boxData[i].box.h = WT_AI_BOX_H;
            meta.boxData[i].box.distance = 15000;         /* 15 m，占位值 */
        }

        rc = DjiLiveview_SendAiMetaToPilot(
            (T_DjiLiveViewStandardMetaData *)&meta);

        /*
         * 每秒打一行，带上 tick 便于和 Pilot 界面截图对时间。
         * 失败也打 —— 错误码本身就是结论（比如"没起视频流"会返回非 SUCCESS）。
         */
        if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_INFO("AI 叠加 tick=%u 发送成功（3 框）", tick);
        } else {
            USER_LOG_WARN("AI 叠加 tick=%u 发送失败 rc=0x%08X", tick, (unsigned)rc);
        }
        tick++;

        osal->TaskSleepMs(WT_AI_SEND_INTERVAL_MS);
    }

    USER_LOG_INFO("AI 叠加验证：线程退出");
    return NULL;
}

bool WtAiOverlay_Start(bool enabled)
{
    T_DjiOsalHandler *osal;
    T_DjiReturnCode rc;

    if (!enabled) {
        USER_LOG_INFO("AI 叠加验证：配置未启用，跳过");
        return false;
    }
    if (s_started) {
        return true;
    }

    /*
     * liveview 模块必须先初始化。
     *
     * ⚠️ 这一步漏掉的后果极具误导性（2026-10-21 实测）：直接调
     * `DjiLiveview_RegUserAiTargetLableList` 会返回 0xE0，日志打
     * `dji_liveview.c:813 Ai target Lable only support on manifold3` ——
     * 看起来像"硬件不支持"，实际是模块没初始化，SDK 内部缓存的
     * `s_mountPositionType` 还是 0，过不了它自己的 `!= MANIFOLD3_ONBOARD`
     * 判定。反汇编确认该变量正是在 `DjiLiveview_Init` 里由
     * `DjiAccessAdapter_GetMountPositionType()` 填充的。
     *
     * 头文件要求：`Must be called after DjiCore_Init.`
     */
    rc = DjiLiveview_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("AI 叠加：DjiLiveview_Init 失败 rc=0x%08X", (unsigned)rc);
        return false;
    }
    s_liveviewInited = true;
    USER_LOG_INFO("AI 叠加：DjiLiveview_Init 成功");

    /*
     * 注册标签。必须在发框之前 —— 原文：
     * "注册后，标签与索引的键值对会记录在 Pilot 上，用于解析 metaData
     *  并在渲染目标框时显示标签。"
     */
    rc = DjiLiveview_RegUserAiTargetLableList(WT_AI_LABEL_COUNT, s_labels);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("AI 叠加：注册标签失败 rc=0x%08X（API 可能不支持本机型）",
                       (unsigned)rc);
        return false;
    }
    s_labelsRegistered = true;
    USER_LOG_INFO("AI 叠加：已注册 %u 个标签", (unsigned)WT_AI_LABEL_COUNT);

    osal = DjiPlatform_GetOsalHandler();
    if (osal == NULL) {
        USER_LOG_ERROR("AI 叠加：拿不到 osal handler");
        return false;
    }

    s_stopRequested = false;
    rc = osal->TaskCreate("wt_ai_overlay", WtAiOverlay_Thread, 4096, NULL, &s_thread);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("AI 叠加：创建线程失败 rc=0x%08X", (unsigned)rc);
        return false;
    }

    s_started = true;
    return true;
}

void WtAiOverlay_Stop(void)
{
    T_DjiOsalHandler *osal;

    if (!s_started) {
        return;
    }

    s_stopRequested = true;

    osal = DjiPlatform_GetOsalHandler();
    if (osal != NULL && s_thread != NULL) {
        osal->TaskSleepMs(WT_AI_SEND_INTERVAL_MS + 200);
        osal->TaskDestroy(s_thread);
        s_thread = NULL;
    }

    if (s_labelsRegistered) {
        const T_DjiReturnCode rc = DjiLiveview_UnregUserAiTargetLableList();

        if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_INFO("AI 叠加：已注销标签");
        } else {
            USER_LOG_WARN("AI 叠加：注销标签失败 rc=0x%08X", (unsigned)rc);
        }
        s_labelsRegistered = false;
    }

    if (s_liveviewInited) {
        (void)DjiLiveview_Deinit();
        s_liveviewInited = false;
    }

    s_started = false;
}
