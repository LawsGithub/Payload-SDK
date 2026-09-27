/**
 * @file lz_ai_probe.c
 * @brief 探针：查 M4T + 妙算3 能否把识别结果推到 Pilot 2 上画框。
 *
 * ## ⚠️ 2026-09-27 实测结论：M4T 不支持
 *
 * ```
 *   RegUserAiTargetLableList(1, {"红旗"})   rc=0x000000E0  ❌ NONSUPPORT
 *   RegEncoderCallback                      rc=0x00000100  ❌ NOT_FOUND
 *   SendAiMetaToPilot                       rc=0x00000000  ✅ 100/100 成功
 *   EncodeAFrameToH264                      rc=0x000000EC  ❌ SYSTEM_ERROR
 * ```
 *
 * 解码用的是 `dji_error.h` 的 `DJI_ERROR_SYSTEM_MODULE_RAW_CODE_*`：
 * **`0x0E0` = NONSUPPORT**、`0x100` = NOT_FOUND、`0xEC` = SYSTEM_ERROR。
 *
 * ⇒ **官方那句"当前仅在 Matrice 400 + H30 + 妙算3 场景下支持"是真的**，
 * M4T 内置相机不在内。**注册自定义类别这一环直接返回"不支持"**，
 * 所以"把目标类型设成红旗"在 M4T 上做不到。
 *
 * ⚠️ `SendAiMetaToPilot` 返回 SUCCESS 是**假阳性**：它把数据交出去了，
 * 但 Pilot 端没有注册表可查、也没有编码器可附，画面上不会出现框。
 * **返回成功 ≠ 生效** —— 这与本项目「静默的失败等于假装成功」
 * 是同一类问题的镜像：**静默的成功更要命**。
 *
 * ## 那这个探针还留着干什么
 *
 * ① 它是"外设支持性"的**可复现判据** —— 换个机型/固件/Liveview 版本时重跑
 *    一次就知道。返回码是硬证据，比文档可靠。
 * ② `--builtin` 模式试的是**另一条路**：不注册自定义标签，直接用 SDK 内置的
 *    `E_DjiLiveViewTargetObjectType`（人/车/船…）。内置类型不需要注册，
 *    所以它可能绕开 `NONSUPPORT` 那一环 —— **值得一试，代价一分钟**。
 *    代价是画面上显示的标签会是内置名（如"Person"），不是"红旗"。
 *
 * ## 要回答的问题
 *
 * 「码流 AI 识别」这条高级功能的**官方声明只覆盖 Matrice 400 挂 H30 相机**：
 *
 * > 当前仅在 Matrice 400 无人机挂载 H30 相机和 DJI 妙算3 场景下支持该功能。
 *
 * 而我们是 **M4T + 内置相机**（GitHub issue #428 有人问的正是这件事，
 * issue 已关闭但看不到官方回复）。文档没承诺 ⇒ **实测才知道**。
 *
 * 本探针按官方文档的完整流程走一遍，**每一步都打印返回码**：
 *
 *   ① `DjiLiveview_RegUserAiTargetLableList`  —— 注册「红旗」这个类别
 *   ② `DjiLiveview_Init` + `StartImageStream` —— 开图（已实测通）
 *   ③ `DjiLiveview_RegEncoderCallback`        —— 初始化硬件编码器
 *   ④ 每帧调 `EncodeAFrameToH264`（带 meta）与 `SendAiMetaToPilot`
 *
 * ⚠️ **返回码要给足信息**：这几个接口失败时可能只回一个笼统的
 * "not support"。所以把 `%llX` 的完整 64 位码打出来（高 32 位是模块号，
 * 用 `(unsigned)` 截断会丢掉它 —— 本项目踩过）。
 *
 * ## 怎么判成败
 *
 * | 判据 | 含义 |
 * |---|---|
 * | ①②③ 返回 SUCCESS | 接口在 M4T 上至少"接受"了调用 |
 * | ④ 返回 SUCCESS | 推送被接受 |
 * | **Pilot 2 画面上出现一个标着「红旗」的框** | **唯一的真正判据** |
 *
 * ⚠️ 前三条都可能返回 SUCCESS 而画面上什么都不出现 ——
 * `SendAiMetaToPilot` 的渲染在 Pilot 侧，它可以选择忽略。
 * **所以必须有人在旁边看画面**，返回码不足以定案。
 *
 * ## 用法
 *
 *   ./lz_ai_probe            # 推 10 秒，框在画面里缓慢移动
 *   ./lz_ai_probe 30         # 推 30 秒
 *
 * 框会沿画面中心绕圈移动 —— 这样"是不是真的在画"一眼可见
 * （静止的框可能被误认为界面元素）。
 *
 * ⚠️ 前提：飞机通电、`Smart3DExplore` 已停、Pilot 2 在飞行界面。
 */

#include <dji_core.h>
#include <dji_liveview.h>
#include <dji_logger.h>
#include <dji_platform.h>
#include <dji_typedef.h>

#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "lz_vision_source.h"
#include "platform/lz_platform.h"
#include "platform/lz_user_info.h"

/* 自定义类别：只注册一个 —— 越多越容易看出"是不是我们注册的那个" */
static const char *kLabels[] = { "红旗" };

/* ------------------------------------------------------------------ */

static void say(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    fflush(stdout);   /* 管道/文件下 stdio 全缓冲，卡住前的输出会丢 */
}

/** 打印一个 PSDK 返回码 —— **完整 64 位**，高 32 位是模块号 */
static void say_rc(const char *what, T_DjiReturnCode rc)
{
    say("  %-46s rc=0x%08llX %s\n", what, (unsigned long long)rc,
        (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? "✅" : "❌");
}

/** 编码回调：硬件编码器的输出，我们不消费，但必须注册（官方流程要求） */
static void onEncoded(const uint8_t *buf, uint32_t len)
{
    (void)buf;
    (void)len;
    /* 什么都不做 —— 本探针只关心"能不能推 AI 结果"，不关心码流本身。
     * ⚠️ 但**不能**在这里做耗时操作（官方注明编解码回调里禁止阻塞）。 */
}

/* 取帧缓冲。**不自己订阅** —— 用 `lz_vision_source` 那一层（它的设计用途
 * 就是这个：回调里只拷贝、检测在锁外）。本探针只按 100 ms 取最新一帧。
 *
 * 顺带这也是它的第一次真实使用：能验证"回调拷贝 + 锁外取帧"这条路
 * 在连续取帧时是否稳定（`lz_liveview_probe` 只取一帧，测不出这一点）。 */
static uint8_t s_frame[LZ_VISION_SOURCE_MAX_PIXELS * 3];

/* ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
    int seconds = 10;
    bool builtin = false;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--builtin") == 0) {
            builtin = true;
        } else {
            const int v = atoi(argv[i]);
            if (v >= 1 && v <= 600) {
                seconds = v;
            }
        }
    }

    say("[lz_ai_probe] 探针：能否把自定义类别「红旗」推到 Pilot 2\n");
    say("⚠️ 请在 Pilot 2 的飞行界面上盯着画面 —— 返回码成功不等于画面上有框。\n\n");

    if (LzPlatform_Prepare() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        say("平台层注册失败\n");
        return 1;
    }
    T_DjiUserInfo userInfo;
    if (LzUserInfo_Fill(&userInfo) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        say("凭据填充失败\n");
        LzPlatform_Deinit();
        return 1;
    }
    say("初始化 PSDK（阻塞 2-4 秒，需飞机通电）...\n");
    if (DjiCore_Init(&userInfo) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        say("DjiCore_Init 失败\n");
        LzPlatform_Deinit();
        return 1;
    }

    /* ⚠️ **不要在这里自己调 `DjiLiveview_Init()`** ——
     * `LzVisionSource_Init()` 内部已经调了（还要调 `StartImageStream`）。
     * 我第一版两处都调，结果第二次报
     * `media tool manager create error, already exist`（rc=0x00000103），
     * 而那个错误码的文案完全看不出是"重复初始化"。
     * 这个模块的初始化**只有一处**：`LzVisionSource_Init()`。 */
    /* ---- ① 注册自定义类别 ---- */
    uint8_t boxType = 1;   /* meta 里要填的 type 值 */
    if (builtin) {
        say("\n① 【--builtin 模式】跳过注册，直接用 SDK 内置类型\n");
        /* 内置类型不需要注册 —— 这正是这个模式要绕开 NONSUPPORT 的原因。
         * 用 MOVING_TARGET(34)：语义上最接近"任意一个移动/待观察目标"，
         * 而且不像 PERSON/CAR 那样会被 Pilot 用固件模型去复核。 */
        boxType = (uint8_t)DJI_LIVEVIEW_OBJ_TYPE_MOVING_TARGET;
        say("   type = MOVING_TARGET(%d)，Pilot 上标签会显示内置名\n", boxType);
    } else {
        say("\n① 注册自定义类别\n");
        const T_DjiReturnCode rcLab =
            DjiLiveview_RegUserAiTargetLableList(1, kLabels);
        say_rc("RegUserAiTargetLableList(1, {\"红旗\"})", rcLab);
        if (rcLab != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            say("   ⇒ %s\n",
                (rcLab == DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT)
                    ? "**NONSUPPORT：M4T 不支持自定义类别。试 --builtin**"
                    : "注册失败，原因见返回码");
        }
        boxType = 1;   /* 注册表索引 */
    }

    /* ---- ③ 注册编码回调（官方流程要求在 EncodeAFrameToH264 之前）---- */
    say("\n③ 注册硬件编码器回调\n");
    const T_DjiReturnCode rcEnc = DjiLiveview_RegEncoderCallback(onEncoded);
    say_rc("RegEncoderCallback", rcEnc);

    /* ---- ② 开图（走 lz_vision_source）---- */
    say("\n② 开图（position=NO_1 source=1 RGB_PACKED，已实测通）\n");
    {
        const LzStatus st = LzVisionSource_Init();
        say("  %-46s %s\n", "LzVisionSource_Init",
            (st == LZ_OK) ? "✅" : "❌");
        if (st != LZ_OK) {
            goto out;
        }
    }

    /* ---- 等第一帧 ---- */
    say("\n等第一帧...\n");
    {
        const T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
        for (int i = 0; i < 60; ++i) {
            LzFrame probe;
            if (LzVisionSource_LatestFrame(&probe, s_frame, sizeof(s_frame)) == LZ_OK) {
                break;
            }
            if (osal != NULL && osal->TaskSleepMs != NULL) {
                (void)osal->TaskSleepMs(100);
            }
        }
    }
    {
        LzVisionSourceStats st;
        LzVisionSource_GetStats(&st);
        say("收到 %u 帧，尺寸 %dx%d\n", st.frames, st.lastWidth, st.lastHeight);
        if (st.frames == 0) {
            say("✗ 没收到帧 —— 后面的推送测不了（见 lz_liveview_probe 的诊断）\n");
            goto out;
        }
    }

    /* ---- ④ 每帧推一个 meta ---- */
    say("\n④ 开始推送 AI 结果（%d 秒，框绕画面中心画圈）\n", seconds);
    say("   ⚠️ 现在请看 Pilot 2 画面。\n\n");

    /* meta 结构是柔性数组：boxCount + boxData[1]，要按实际框数分配 */
    const size_t metaSize = sizeof(T_DjiLiveViewStandardMetaData);
    T_DjiLiveViewStandardMetaData *meta =
        (T_DjiLiveViewStandardMetaData *)malloc(metaSize);
    if (meta == NULL) {
        say("malloc 失败\n");
        goto out;
    }

    const T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    uint32_t okSend = 0, okEnc = 0, tried = 0, encTried = 0;
    T_DjiReturnCode lastSend = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    T_DjiReturnCode lastEnc = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

    for (int t = 0; t < seconds * 10; ++t) {
        /* 圆心绕画面中心转，半径 0.2 —— 画面上应看到框在动 */
        const double ang = (double)t * 0.15;
        const int cx = (int)(5000 + 2000 * cos(ang));   /* 1/10000 单位 */
        const int cy = (int)(5000 + 2000 * sin(ang));

        memset(meta, 0, metaSize);
        meta->boxCount = 1;
        meta->boxData[0].id = 1;
        /* type 用自定义值：注册表里的索引。官方样例用的是 0..3（它注册了 4 个
         * 无效标签），此处我们只注册了 1 个（索引应为**1**，因为 0 通常保留给
         * 内置的 INVALID）。两种都试过才知道 —— 先试 1。 */
        meta->boxData[0].type = boxType;
        meta->boxData[0].state = DJI_LIVEVIEW_OBJ_STATE_TRACKED;
        meta->boxData[0].box.cx = (uint16_t)cx;
        meta->boxData[0].box.cy = (uint16_t)cy;
        meta->boxData[0].box.w = 1200;
        meta->boxData[0].box.h = 800;
        meta->boxData[0].box.distance = 0;

        /* 取最新一帧（取不到就沿用上一帧 —— meta 的推送不依赖分量，这里
         * 只是为了给 EncodeAFrameToH264 一个有效的图像。） */
        LzFrame f;
        memset(&f, 0, sizeof(f));
        int fw = 0, fh = 0;
        if (LzVisionSource_LatestFrame(&f, s_frame, sizeof(s_frame)) == LZ_OK) {
            fw = f.width;
            fh = f.height;
        }

        tried++;
        const T_DjiReturnCode rs = DjiLiveview_SendAiMetaToPilot(meta);
        if (rs == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            okSend++;
        } else {
            lastSend = rs;
        }
        const T_DjiReturnCode re = (fw > 0)
            ? DjiLiveview_EncodeAFrameToH264(
                  s_frame, (uint32_t)(fw * fh * 3),
                  (T_DjiLiveviewImageInfo){ .pixFmt = PIXFMT_RGB_PACKED,
                                            .width = (uint16_t)fw,
                                            .height = (uint16_t)fh,
                                            .frameId = (uint32_t)t },
                  meta)
            : DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;   /* 没帧就跳过编码 */
        if (fw > 0) {
            encTried++;
        }
        if (fw > 0) {
            if (re == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                okEnc++;
            } else {
                lastEnc = re;
            }
        }

        if ((t + 1) % 10 == 0) {
            say("  [%2ds] SendAiMeta 成功 %u/%u    EncodeFrame 成功 %u/%u\n",
                (t + 1) / 10, okSend, tried, okEnc, encTried);
        }
        if (osal != NULL && osal->TaskSleepMs != NULL) {
            (void)osal->TaskSleepMs(100);
        }
    }

    say("\n=== 结果 ===\n");
    say("  SendAiMetaToPilot : %u/%u 成功", okSend, tried);
    if (okSend < tried) {
        say("  最后一个失败码 0x%08llX", (unsigned long long)lastSend);
    }
    say("\n  EncodeAFrameToH264: %u/%u 成功", okEnc, encTried);
    if (okEnc < tried) {
        say("  最后一个失败码 0x%08llX", (unsigned long long)lastEnc);
    }
    say("\n");
    free(meta);

out:
    say("\n清理...\n");
    /* ⚠️ `LzVisionSource_Stop()` 内部已经做了 StopImageStream + Deinit，
     * 此处**不能**再调一遍 —— 重复 Deinit 会让 SDK 报一串
     * "Do not find command receive handle list"（本项目见过那种噪声）。 */
    LzVisionSource_Stop();
    (void)DjiLiveview_UnregUserAiTargetLableList();
    (void)DjiLiveview_UnregEncoderCallback();
    DjiCore_DeInit();
    LzPlatform_Deinit();
    return 0;
}
