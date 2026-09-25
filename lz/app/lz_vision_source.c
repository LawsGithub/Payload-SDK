/**
 * @file lz_vision_source.c
 * @brief 取图的实现：`DjiLiveview_StartImageStream()` → 内部缓冲 → `LzFrame`。
 *
 * 设计依据、全部待定项与硬约束见 `doc/VISION-GIMBAL-PITCH.md` §3①、§6.1；
 * 本文件顶部只重复最要紧的两条。
 *
 * ## 为什么用 `StartImageStream` 而不是 `StartH264Stream`
 *
 * 前者直接给**解码后的像素帧**，头文件 `@note This interface support on
 * DJI manifold3` —— 正是我们的机型。后者给的是编码流，得自己解码
 * （引入解码器 = 引入构建依赖 + 引入"解出来是花屏"这类新故障）。
 *
 * ## 为什么拷贝在锁内、检测在锁外
 *
 * 回调跑在 **PSDK 的工作线程**上。若在回调里做检测（几百毫秒），等于把
 * SDK 的线程占住 —— 与本项目踩过的"控件回调里调阻塞接口导致进程闪退"
 * 是**同一条纪律**（日志表现为 `semaphore wait timeout` +
 * `send msg to queue error` 刷屏后死掉）。
 *
 * 所以：
 *
 *     回调（SDK 线程）                     调用方（我们的线程）
 *     ────────────────                     ──────────────────
 *     锁 → memcpy → 解锁（~1 MB，几 ms）   锁 → 拷出 → 解锁 → 在外面慢慢算
 *
 * 两次拷贝看着浪费，但**第一次是必须的**（`buf` 回调返回后失效），
 * 第二次买到的是"检测不占 SDK 线程"。1 MB 的 memcpy 在这个尺度上
 * 是微秒到毫秒级，而一次检测是几十毫秒级。
 */

#include "lz_vision_source.h"

#include <dji_logger.h>
#include <dji_platform.h>

#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* 状态                                                                */
/* ------------------------------------------------------------------ */

/*
 * 静态缓冲而非动态分配：SDK 回调线程里做 malloc/free 会把分配器卷进来，
 * 而那种故障（堆损坏）极难定位。单帧上限见头文件。
 */
static uint8_t s_frameBuf[LZ_VISION_SOURCE_MAX_PIXELS * 3];

static T_DjiMutexHandle s_lock = NULL;
static bool s_ready = false;             /* 已开流（不代表收到过帧） */
static LzVisionSourceConfig s_cfg;
static LzVisionSourceStats s_stats;

/* 已收到的帧（受 s_lock 保护） */
static bool s_hasFrame = false;
static size_t s_frameLen = 0;
static int s_w = 0, s_h = 0;
static bool s_isBgr = false;
static uint32_t s_frameId = 0;

/* ------------------------------------------------------------------ */
/* 回调（PSDK 工作线程）                                                */
/* ------------------------------------------------------------------ */

/**
 * @brief 收到一帧解码图像
 *
 * ⚠️ 本函数**只做校验与拷贝**。任何耗时操作都会拖住 SDK 线程。
 *
 * @param buf  像素数据。**回调返回后即失效**，必须当场拷走。
 * @param len  字节数
 * @param info 像素格式 + 尺寸 + 帧号
 */
static void LzVisionSource_OnImage(E_DjiLiveViewCameraPosition position,
                                   const uint8_t *buf, uint32_t len,
                                   T_DjiLiveviewImageInfo info)
{
    (void)position;

    if (buf == NULL || info.width == 0 || info.height == 0) {
        return;
    }

    /* 闸门 1：像素格式。
     *
     * ⚠️ **选错 pixFmt 不会报错，只会给花屏** —— 所以这里对"收到的"格式
     * 做校验。不符就丢帧并计数，而不是把垃圾当图用：后者会让上层报
     * "检测不到红色目标"，而病因在格式上 —— **文案把排查方向带反**。 */
    if (info.pixFmt != PIXFMT_RGB_PACKED) {
        s_stats.droppedFmt++;
        return;
    }

    const uint32_t pixels = (uint32_t)info.width * (uint32_t)info.height;
    if (pixels > LZ_VISION_SOURCE_MAX_PIXELS) {
        s_stats.droppedTooBig++;
        return;
    }

    /* 闸门 2：打包格式下每行字节数应恰好 = w*3。
     * 不等说明它不是我们以为的"紧密打包"，按 stride 拷会拷进错位的行。 */
    const size_t need = (size_t)pixels * 3u;
    if (len < need) {
        s_stats.droppedStride++;
        return;
    }

    if (s_lock == NULL) {
        return;   /* 还没初始化完就来了帧 —— 丢掉，不是错误 */
    }
    const T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    if (osal == NULL || osal->MutexLock == NULL) {
        return;
    }

    (void)osal->MutexLock(s_lock);
    memcpy(s_frameBuf, buf, need);
    s_frameLen = need;
    s_w = (int)info.width;
    s_h = (int)info.height;
    s_frameId = info.frameId;
    s_hasFrame = true;
    s_stats.frames++;
    s_stats.lastWidth = s_w;
    s_stats.lastHeight = s_h;
    s_stats.lastFrameId = info.frameId;
    (void)osal->MutexUnlock(s_lock);
}

/* ------------------------------------------------------------------ */
/* 生命周期                                                            */
/* ------------------------------------------------------------------ */

LzVisionSourceConfig LzVisionSource_DefaultConfig(void)
{
    LzVisionSourceConfig c;
    /* 位置 1：M4T 原生相机实测挂在这里（`[V]` 2026-09-19，与激光同路） */
    c.position = DJI_LIVEVIEW_CAMERA_POSITION_NO_1;
    /* ⚠️ 猜的。M4T 有 M4T_VIS=1 / IR=2 / 4K=3，而镜头是 82°/35°/15°，
     * 对不上 —— 由探针去试。见头文件与 §9 #1。 */
    c.source = DJI_LIVEVIEW_CAMERA_SOURCE_M4T_VIS;
    /* 照官方样例（dji_liveview_object_detection.cpp:461） */
    c.pixFmt = PIXFMT_RGB_PACKED;
    return c;
}

LzStatus LzVisionSource_Init(void)
{
    const LzVisionSourceConfig cfg = LzVisionSource_DefaultConfig();
    return LzVisionSource_InitWith(&cfg);
}

LzStatus LzVisionSource_InitWith(const LzVisionSourceConfig *cfg)
{
    if (cfg == NULL) {
        return LZ_ERR_PARAM;
    }
    if (s_ready) {
        return LZ_OK;   /* 幂等 */
    }

    const T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    if (osal == NULL || osal->MutexCreate == NULL) {
        USER_LOG_ERROR("取图：平台层未就绪（osal 缺失），无法取图");
        return LZ_ERR_NOT_READY;
    }

    if (s_lock == NULL) {
        if (osal->MutexCreate(&s_lock) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("取图：创建互斥量失败");
            s_lock = NULL;
            return LZ_ERR_IO;
        }
    }

    memset(&s_stats, 0, sizeof(s_stats));
    s_hasFrame = false;
    s_cfg = *cfg;

    /* ⚠️ 头文件 @note：`DjiLiveview_Init` 必须在 `DjiCore_Init` 之后。
     * 这一条是调用方的责任，本函数只管从"核心已初始化"这个前提出发。 */
    T_DjiReturnCode rc = DjiLiveview_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("取图：DjiLiveview_Init 失败 rc=0x%08llX",
                       (unsigned long long)rc);
        return LZ_ERR_UNSUPPORTED;
    }

    /* ⚠️ 回调**必须传**，不能传 NULL —— 我们靠它拿帧。 */
    rc = DjiLiveview_StartImageStream(cfg->position, cfg->source, cfg->pixFmt,
                                      LzVisionSource_OnImage);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        /* 错误码用 %llX 看 —— `(unsigned)` 会丢掉高 32 位的模块号。 */
        USER_LOG_ERROR("取图：StartImageStream 失败 rc=0x%08llX（%s）",
                       (unsigned long long)rc, LzVisionSource_DescribeAssumptions());
        return LZ_ERR_UNSUPPORTED;
    }

    s_ready = true;
    USER_LOG_INFO("取图：已开流（%s）", LzVisionSource_DescribeAssumptions());
    return LZ_OK;
}

void LzVisionSource_Stop(void)
{
    if (!s_ready) {
        return;
    }
    const T_DjiReturnCode rc = DjiLiveview_StopImageStream(s_cfg.position, s_cfg.source);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_WARN("取图：StopImageStream 失败 rc=0x%08llX",
                      (unsigned long long)rc);
    }
    (void)DjiLiveview_Deinit();
    if (s_lock != NULL) {
        const T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
        if (osal != NULL && osal->MutexDestroy != NULL) {
            (void)osal->MutexDestroy(s_lock);
        }
        s_lock = NULL;
    }
    s_ready = false;
    s_hasFrame = false;
}

bool LzVisionSource_IsReady(void)
{
    return s_ready;
}

LzStatus LzVisionSource_LatestFrame(LzFrame *frame, uint8_t *dst, size_t dstCapacity)
{
    if (frame == NULL || dst == NULL) {
        return LZ_ERR_PARAM;
    }
    if (s_lock == NULL) {
        return LZ_ERR_NOT_READY;
    }

    const T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    if (osal == NULL || osal->MutexLock == NULL) {
        return LZ_ERR_NOT_READY;
    }

    (void)osal->MutexLock(s_lock);
    if (!s_hasFrame) {
        (void)osal->MutexUnlock(s_lock);
        return LZ_ERR_NOT_READY;   /* 还没收到过帧 */
    }
    if (dstCapacity < s_frameLen) {
        (void)osal->MutexUnlock(s_lock);
        return LZ_ERR_PARAM;       /* 调用方的缓冲不够 —— 是编程错误 */
    }
    /* 拷贝在锁内，检测在外面 —— 见文件头 */
    memcpy(dst, s_frameBuf, s_frameLen);

    memset(frame, 0, sizeof(*frame));
    frame->data = dst;
    frame->width = s_w;
    frame->height = s_h;
    frame->channels = 3;              /* pixFmt 已校验为 RGB_PACKED */
    frame->stride = s_w * 3;          /* 已校验为紧密打包 */
    frame->isBgr = s_isBgr;
    frame->frameId = s_frameId;
    (void)osal->MutexUnlock(s_lock);

    return LZ_OK;
}

void LzVisionSource_GetStats(LzVisionSourceStats *out)
{
    if (out == NULL) {
        return;
    }
    *out = s_stats;
    out->lastIsBgr = s_isBgr;
}

const char *LzVisionSource_DescribeAssumptions(void)
{
    /* 静态缓冲 + 轮转：这个函数会被反复调用（日志、探针），
     * 返回 malloc 的内存会让调用方不知道该不该 free。
     * 两个槽轮转，够覆盖"一行日志里调两次"的用法。 */
    static char buf[2][192];
    static unsigned turn = 0;
    char *b = buf[turn++ & 1u];

    const char *fmtName = "?";
    switch (s_cfg.pixFmt) {
    case PIXFMT_NV12:       fmtName = "NV12(**视觉层不支持,会花屏**)"; break;
    case PIXFMT_RGB_PLANAR: fmtName = "RGB_PLANAR(**平面,当打包读会花屏**)"; break;
    case PIXFMT_RGB_PACKED: fmtName = "RGB_PACKED"; break;
    default:                fmtName = "未知"; break;
    }

    /* 子相机源与机型强相关，且同名值在不同机型下含义不同
     * （M4T_VIS=1 / H20_WIDE=1 / M3E_VIS=1 …），所以连机型一起打出来。 */
    const char *srcName = "?";
    switch ((int)s_cfg.source) {
    case 0: srcName = "DEFAULT(0)"; break;
    case 1: srcName = "1(对 M4T 是 VIS)"; break;
    case 2: srcName = "2(对 M4T 是 IR)"; break;
    case 3: srcName = "3(对 M4T 是 4K)"; break;
    default: srcName = "其他"; break;
    }

    snprintf(b, sizeof(buf[0]),
             "position=NO_1(%d) source=%s pixFmt=%s",
             (int)s_cfg.position, srcName, fmtName);
    return b;
}
