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
static bool s_ready = false;                      /* 已开流（不代表收到过帧） */
/** `DjiLiveview_Init()` 是否调过 —— 只调一次的守卫，理由见 `LzVisionSource_InitWith()` */
static bool s_liveviewInited = false;
static LzVisionSourceConfig s_cfg;
static LzVisionSourceStats s_stats;

/**
 * @brief 记一条丢弃：计数 + 被丢那一帧的自述
 *
 * **计数只说明"有东西被拒"，排查需要的是"拒掉的是什么"** —— 这是
 * 2026-09-27 上机实测的教训（见 `LzVisionSourceStats` 的注释）。
 *
 * 宏而不是函数：`info.pixFmt` 是 SDK 的结构体字段，写成函数要多传三个参数，
 * 而这三处调用点长得完全一样。这也是本项目少数几个用宏的场合之一。
 *
 * ⚠️ **宏体里不能写 `//` 注释** —— 展开成一行后后面整段都成了注释的一部分。
 * 这是本项目踩过的坑。
 */
#define LZ_SOURCE_RECORD_DROP(why, info, len)                                 \
    do {                                                                      \
        switch (why) {                                                        \
        case LZ_VISION_DROP_FMT:       s_stats.droppedFmt++;       break;     \
        case LZ_VISION_DROP_TOO_BIG:   s_stats.droppedTooBig++;    break;     \
        case LZ_VISION_DROP_STRIDE:    s_stats.droppedStride++;    break;     \
        case LZ_VISION_DROP_NOT_READY: s_stats.droppedNotReady++;  break;     \
        default:                                                       break;  \
        }                                                                     \
        s_stats.lastDropWidth  = (int)(info).width;                           \
        s_stats.lastDropHeight = (int)(info).height;                          \
        s_stats.lastDropLen    = (uint32_t)(len);                             \
        s_stats.lastDropReason = (why);                                       \
    } while (0)

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

    /* 先记「见过的最大帧」—— 必须在任何闸门之前。
     *
     * ⚠️ 2026-09-27 上机实测逼出来的：`source=3` 一路全被 `droppedTooBig`
     * 拒掉（那说明它是 4K），但**当时记不出它到底多大**，于是"取图不工作"
     * 变成了不可诊断的状态。最大尺寸跨所有帧（含被拒的），就是为了
     * 回答"拒掉的到底是什么"。 */
    const uint32_t pix = (uint32_t)info.width * (uint32_t)info.height;
    if (info.width > s_stats.maxSeenWidth)  { s_stats.maxSeenWidth  = (int)info.width; }
    if (info.height > s_stats.maxSeenHeight) { s_stats.maxSeenHeight = (int)info.height; }

    /* 闸门 1：像素格式。
     *
     * ⚠️ **选错 pixFmt 不会报错，只会给花屏** —— 所以这里对"收到的"格式
     * 做校验。不符就丢帧并计数，而不是把垃圾当图用：后者会让上层报
     * "检测不到红色目标"，而病因在格式上 —— **文案把排查方向带反**。 */
    if (info.pixFmt != PIXFMT_RGB_PACKED) {
        LZ_SOURCE_RECORD_DROP(LZ_VISION_DROP_FMT, info, len);
        return;
    }

    if (pix > LZ_VISION_SOURCE_MAX_PIXELS) {
        LZ_SOURCE_RECORD_DROP(LZ_VISION_DROP_TOO_BIG, info, len);
        return;
    }

    /* 闸门 2：打包格式下每行字节数应恰好 = w*3。
     * 不等说明它不是我们以为的"紧密打包"，按 stride 拷会拷进错位的行。 */
    const size_t need = (size_t)pix * 3u;
    if (len < need) {
        LZ_SOURCE_RECORD_DROP(LZ_VISION_DROP_STRIDE, info, len);
        return;
    }

    if (s_lock == NULL) {
        /* 还没初始化完就来了帧 —— 丢掉，不是错误，但要记下来：
         * 若这一路真的在初始化前就有流，说明"开流"的时序假设不成立。 */
        LZ_SOURCE_RECORD_DROP(LZ_VISION_DROP_NOT_READY, info, len);
        return;
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
     * 这一条是调用方的责任，本函数只管从"核心已初始化"这个前提出发。
     *
     * ⚠️ **只调一次** —— 因为 `LzVisionSource_Stop()` 刻意不再配对地调
     * `DjiLiveview_Deinit()`（理由见那里：Deinit 会吃掉 Pilot 的飞行画面）。
     * 于是本函数可能被反复调用（照准可反复触发），必须自己守卫。
     * **重复 Init 的语义没有文档保证** —— 头文件只写"必须在 DjiCore_Init
     * 之后"，没说可以调几次。没有文档保证的事不做假设。 */
    T_DjiReturnCode rc = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    if (!s_liveviewInited) {
        rc = DjiLiveview_Init();
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("取图：DjiLiveview_Init 失败 rc=0x%08llX",
                           (unsigned long long)rc);
            return LZ_ERR_UNSUPPORTED;
        }
        s_liveviewInited = true;
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

    /* ⚠️ **刻意不调 `DjiLiveview_Deinit()`。**
     *
     * 这里原先调了它（官方样例也调，见 `test_liveview.c:224`），
     * 结果是 **Pilot 的飞行画面变黑** —— 2026-09-28 实测：
     *
     * ```text
     * 01:51:49.004  request start agent liveview  + 取图：已开流
     * 01:51:50.157  request stop agent liveview   ← 照准被自己停掉，流关了
     *                ↑ 之后 Pilot 画面全黑
     * ```
     *
     * `DjiLiveview_Deinit()` 反初始化的是**整个 liveview 模块**，而妙算3
     * 转发给 Pilot 的图传正走这个模块。⇒ 只要开过一次流再 Deinit，
     * 画面就没了，而且**不会自己恢复**（实测：`systemctl restart dji_sdk_agent`
     * 需要 root，dji 用户做不了；只能重启设备）。
     *
     * **为什么样例那样写没问题**：样例是一次性工具，跑完就退出进程 ——
     * Deinit 与进程退出相隔几毫秒，用户看不到画面黑。而我们是**常驻应用**，
     * 照准是可反复触发的动作，于是"跑一次照准黑一次画面"。
     *
     * ⇒ 取舍：不 Deinit，模块级资源留着不释放（`StopImageStream` 已经
     * 把流停了，帧回调不会再触发）。**代价是几十 KB 的常驻，
     * 收益是操作员的画面不会被我们的动作吃掉。**
     *
     * ⚠️ 由此带来一个约束：`LzVisionSource_Init()` 里的 `DjiLiveview_Init()`
     * 也不能重复调用（Deinit 不再配对）。若将来要支持"开→关→再开"，
     * 必须在这里加一个 `s_liveviewInited` 守卫，**不能**靠加回 Deinit 解决。 */
    USER_LOG_INFO("取图：仅停流，**不**反初始化 liveview 模块（保 Pilot 画面）");
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

const char *LzVisionSource_DropReasonName(LzVisionDropReason reason)
{
    switch (reason) {
    case LZ_VISION_DROP_FMT:       return "像素格式不是 RGB_PACKED";
    case LZ_VISION_DROP_TOO_BIG:   return "超出单帧像素上限";
    case LZ_VISION_DROP_STRIDE:    return "字节数不足（不是紧密打包）";
    case LZ_VISION_DROP_NOT_READY: return "开流尚未完成";
    case LZ_VISION_DROP_NONE:
    default:                       return "没丢过";
    }
}

const char *LzVisionSource_DescribeAssumptions(void)
{
    /* 静态缓冲 + 轮转：这个函数会被反复调用（日志、探针），
     * 返回 malloc 的内存会让调用方不知道该不该 free。
     * 两个槽轮转，够覆盖"一行日志里调两次"的用法。 */
    static char buf[2][224];
    static unsigned turn = 0;
    char *b = buf[turn++ & 1u];

    /* ⚠️ **未初始化时不能报"当前配置"** —— 那时 `s_cfg` 还是零值，
     * 打印出来的是 `position=0 source=0 pixFmt=未知`，看着像一份真实的
     * 配置，实际是零。2026-09-27 上机实测撞到：探针在
     * `LzVisionSource_InitWith()` 之前调本函数，输出的 position/source
     * 都是 0，而实际用的是 1/1 —— **比崩溃更危险的一类错误：
     * 它给出一份看起来可信、实际与运行状态无关的"事实"**。
     *
     * 所以未开流时显式说明"还没开流"，只报**参数默认值**（那个是确定的），
     * 不报运行状态。 */
    if (!s_ready) {
        const LzVisionSourceConfig d = LzVisionSource_DefaultConfig();
        snprintf(b, sizeof(buf[0]),
                 "**尚未开流**（下面这些是 DefaultConfig 的值，不是运行状态）: "
                 "position=NO_1(%d) source=%d pixFmt=%d",
                 (int)d.position, (int)d.source, (int)d.pixFmt);
        return b;
    }

    const char *fmtName = "?";
    switch (s_cfg.pixFmt) {
    case PIXFMT_NV12:       fmtName = "NV12(**视觉层不支持,会花屏**)"; break;
    case PIXFMT_RGB_PLANAR: fmtName = "RGB_PLANAR(**平面,当打包读会花屏**)"; break;
    case PIXFMT_RGB_PACKED: fmtName = "RGB_PACKED"; break;
    default:                fmtName = "未知"; break;
    }

    /* 子相机源与机型强相关，且同名值在不同机型下含义不同
     * （M4T_VIS=1 / H20_WIDE=1 / M3E_VIS=1 …），所以连机型一起打出来。
     *
     * ⚠️ 下面的名字来自 **2026-09-27 M4T 实测**（探针 scan 模式）：
     *   source=0 → 1440x1080 彩色        source=1 → 1440x1080 彩色（与 0 同一路）
     *   source=2 → 1280x1024 **灰度**    source=3 → 帧被拒（4K，超出单帧上限）
     * 判据见 `doc/VISION-GIMBAL-PITCH.md` §9 #1。 */
    const char *srcName = "?";
    switch ((int)s_cfg.source) {
    case 0: srcName = "DEFAULT(0)，实测=1440x1080 彩色"; break;
    case 1: srcName = "1，实测=1440x1080 彩色（与 0 同一路）"; break;
    case 2: srcName = "2，实测=1280x1024 **灰度**（热成像）"; break;
    case 3: srcName = "3，实测=4K（超出单帧上限，被丢）"; break;
    default: srcName = "其他（未实测）"; break;
    }

    snprintf(b, sizeof(buf[0]),
             "position=NO_1(%d) source=%s pixFmt=%s",
             (int)s_cfg.position, srcName, fmtName);
    return b;
}
