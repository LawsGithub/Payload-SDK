/**
 * @file lz_liveview_probe.c
 * @brief 探针：查 M4T 的图像流能不能取到、取到的是哪个镜头的画面。
 *
 * ## 为什么必须写这个探针
 *
 * `doc/VISION-GIMBAL-PITCH.md` §9 的头号待定项是
 * **「`DjiLiveview_StartImageStream` 取到的是广角 82° / 中长焦 35° / 长焦 15°
 * 里的哪一个」** —— 它直接决定"像素偏差 → 角度"换算的分母，三个值差 5 倍。
 *
 * 而 `E_DjiLiveViewCameraSource` 里 M4T 只给了 `M4T_VIS=1 / IR=2 / 4K=3`
 * 三个名字，**与三个焦距对不上**。文档、头文件、官方样例都没有说明。
 *
 * 所以只能**上机把每一路都开一遍，把帧存成 PPM 拿回本机看**：
 * 画面内容（视场宽窄、是不是红外灰度、是不是那条杆）会直接告诉我们是哪一个。
 *
 * 同一次上机顺带答掉三个问题（都是 `VISION-GIMBAL-PITCH.md` §9 的条目）：
 *
 * | 问题 | 本探针怎么答 |
 * |---|---|
 * | 取到哪个镜头 | 存下的 PPM 用肉眼/工具看视场 |
 * | 通道顺序 RGB 还是 BGR | 红旗在 PPM 里是红的还是蓝的 |
 * | 帧率与稳定性 | `Stats` 行的 fps 与丢帧计数 |
 *
 * ## 为什么写成独立探针，而不是直接接进主应用
 *
 * 与 `lz_rangefinder_probe` / `lz_mission_probe` 同一个理由：**把不确定性
 * 隔离在一次上机试验里**。取图的正确参数（position/source/pixFmt）现在
 * 是猜的，猜错的表现是"花屏"或"收不到帧"而不是报错 —— 在主应用里排查
 * 这件事，要连带着任务状态机一起看，成本高得多。
 *
 * ## 用法
 *
 *   ./lz_liveview_probe                     # 默认组合，跑 10 秒，存一帧
 *   ./lz_liveview_probe 1 1 5               # position=1 source=1(PIXFMT_RGB_PACKED) 跑 10 秒
 *   ./lz_liveview_probe scan                # ★ 依次试 source = 0,1,2,3，各存一帧
 *   ./lz_liveview_probe 1 1 5 30            # 第 4 个参数 = 跑多少秒
 *
 * 存的帧在**当前目录** `lz_frame_<pos>_<src>.ppm`（P6 二进制，零依赖格式
 * —— 本工程读 PPM 只要 20 行，PNG 得引 zlib）。拷回本机直接能看。
 *
 * ⚠️ 需要飞机通电并连接；且运行前要让出 PSDK 通道：
 *   pgrep -x Smart3DExplore >/dev/null || /system/bin/dji_app_ctl stop Smart3DExplore
 *
 * ⚠️ 需要**飞机在飞行界面**才有码流（Pilot 2 不接飞机时相机不出流）。
 */

#include <dji_core.h>
#include <dji_logger.h>
#include <dji_platform.h>
#include <dji_typedef.h>

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lz_vision_source.h"
#include "platform/lz_platform.h"
#include "platform/lz_user_info.h"

/* 单帧缓冲：与 lz_vision_source.c 里的大小一致 —— 超出上限的帧它已经丢了 */
static uint8_t s_frame[LZ_VISION_SOURCE_MAX_PIXELS * 3];

/**
 * @brief 打印并立刻刷出去
 *
 * ⚠️ **这不是多此一举，是本项目踩过两次的坑**（见 CLAUDE.md
 * 「输出重定向到管道/文件时 stdio 是全缓冲」）：
 *
 * 探针的输出被 `>` 到文件、或经 sshd 转发时，stdio 变成**全缓冲**，
 * 攒够 4 KB 才可见。而探针最需要输出的恰恰是**卡住之前**那几行
 * （"正在初始化 PSDK"、"开流失败：…"）—— 一旦 `DjiCore_Init` 卡住
 * （飞机关机时就是这样，实测卡满 45 秒超时），缓冲区里的内容
 * **全部丢失**，现场看到的是一个没有任何输出的空文件。
 *
 * 实测（2026-09-26，飞机未通电）：不加 flush 时输出仅有一行来自 SDK
 * 自己的日志重定向，我们打的全丢了 —— 那会让"探针卡在哪一步"
 * 变成无法回答的问题。
 */
static void say(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    fflush(stdout);
}

/* ------------------------------------------------------------------ */

/** 把一帧写成 P6 PPM。失败返回 false。 */
static bool save_ppm(const char *path, const LzFrame *f)
{
    FILE *fp = fopen(path, "wb");
    if (fp == NULL) {
        say("    ✗ 打不开 %s\n", path);
        return false;
    }
    fprintf(fp, "P6\n%d %d\n255\n", f->width, f->height);
    const size_t wrote = fwrite(f->data, 1, (size_t)f->width * (size_t)f->height * 3u, fp);
    fclose(fp);
    if (wrote != (size_t)f->width * (size_t)f->height * 3u) {
        say("    ✗ 写 %s 只写了 %zu 字节\n", path, wrote);
        return false;
    }
    return true;
}

/**
 * @brief 跑一个组合：开流 → 等 → 取一帧 → 存 PPM → 停流
 * @return true 表示收到过帧
 */
static bool run_one(E_DjiLiveViewCameraPosition pos, E_DjiLiveViewCameraSource src,
                    int seconds)
{
    LzVisionSourceConfig cfg;
    cfg.position = pos;
    cfg.source = src;
    cfg.pixFmt = PIXFMT_RGB_PACKED;

    say("\n── position=%d source=%d pixFmt=RGB_PACKED ──\n", (int)pos, (int)src);

    const LzStatus st = LzVisionSource_InitWith(&cfg);
    if (st != LZ_OK) {
        say("  开流失败：%s\n", LzStatus_Str(st));
        say("  （`DjiLiveview_Init` 要在 `DjiCore_Init` 之后；"
               "错误码用 %%llX 看，别用 %%X —— 高 32 位是模块号）\n");
        LzVisionSource_Stop();
        return false;
    }

    const T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    uint32_t lastFrames = 0;
    for (int i = 0; i < seconds; ++i) {
        if (osal != NULL && osal->TaskSleepMs != NULL) {
            (void)osal->TaskSleepMs(1000);
        }
        LzVisionSourceStats s;
        LzVisionSource_GetStats(&s);
        say("  [%2ds] 帧数=%-5u fps≈%-4u 丢(格式/过大/行宽)=%u/%u/%u "
               "尺寸=%dx%d frameId=%u\n",
               i + 1, s.frames, s.frames - lastFrames,
               s.droppedFmt, s.droppedTooBig, s.droppedStride,
               s.lastWidth, s.lastHeight, s.lastFrameId);
        lastFrames = s.frames;
    }

    LzFrame f;
    memset(&f, 0, sizeof(f));
    const LzStatus gst = LzVisionSource_LatestFrame(&f, s_frame, sizeof(s_frame));
    bool got = false;
    if (gst == LZ_OK) {
        char path[128];
        snprintf(path, sizeof(path), "lz_frame_%d_%d.ppm", (int)pos, (int)src);
        if (save_ppm(path, &f)) {
            say("  ✓ 已存 %s（%dx%d，%s）\n"
                   "    拷回本机看一下：画面里是宽视场还是窄视场？"
                   "红色旗是红的还是蓝的？\n",
                   path, f.width, f.height, f.isBgr ? "标为 BGR" : "标为 RGB");
            got = true;
        }
    } else {
        say("  ✗ 没取到帧（%s）—— 可能是：\n"
               "    ① source 这一路不存在；② 飞机不在飞行界面、相机没出流；\n"
               "    ③ 需要先注册编码回调（本探针刻意没注册，见 §9 #7）\n",
               LzStatus_Str(gst));
    }

    LzVisionSource_Stop();
    return got;
}

/** @brief 打印当前位置/子源组合的实际取值（不量测，只是让日志自证） */
static void print_assumptions(void)
{
    const LzVisionSourceConfig d = LzVisionSource_DefaultConfig();
    say("默认配置（**未经上机验证，是当前最优猜测**）：\n");
    say("  position = %d\n", (int)d.position);
    say("  source   = %d\n", (int)d.source);
    say("  pixFmt   = %d (PIXFMT_RGB_PACKED=%d)\n", (int)d.pixFmt, (int)PIXFMT_RGB_PACKED);
    say("  自述     = %s\n", LzVisionSource_DescribeAssumptions());
}

int main(int argc, char **argv)
{
    say("[lz_liveview_probe] 探针：M4T 图像流取图\n");

    /* ---- 平台层 ---- */
    if (LzPlatform_Prepare() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        say("平台层注册失败\n");
        return 1;
    }

    T_DjiUserInfo userInfo;
    if (LzUserInfo_Fill(&userInfo) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        say("凭据填充失败 —— 见上方日志\n");
        LzPlatform_Deinit();
        return 1;
    }

    say("初始化 PSDK（阻塞 2-4 秒，需飞机通电并连接）...\n");
    if (DjiCore_Init(&userInfo) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        say("DjiCore_Init 失败\n");
        LzPlatform_Deinit();
        return 1;
    }
    /* 调度器必须在取流之前启动 —— 与 `DjiFcSubscription_Init` 一样，
     * 那些回调都跑在 SDK 自己的线程上。 */
    (void)DjiCore_ApplicationStart();

    print_assumptions();

    /* ---- 参数解析 ---- */
    bool scan = (argc >= 2 && strcmp(argv[1], "scan") == 0);

    if (scan) {
        /* 依次试 source = 0..3。**单进程内反复 Init/Stop** —— 这是探针，
         * 不是主链路，失败了也能从输出看出是哪一步。 */
        static const E_DjiLiveViewCameraSource kSources[] = {0, 1, 2, 3};
        int gotCount = 0;
        for (size_t i = 0; i < sizeof(kSources) / sizeof(kSources[0]); ++i) {
            if (run_one(DJI_LIVEVIEW_CAMERA_POSITION_NO_1, kSources[i], 4)) {
                gotCount++;
            }
        }
        say("\n扫描完毕：4 路里 %d 路取到了帧。\n", gotCount);
        say("把取到帧的那几个 PPM 拷回本机，按下面的判据认镜头：\n");
        say("  · 视场最宽、能看到大范围地面   → 广角 82°（等效 24mm）\n");
        say("  · 视场中等、目标被放大         → 中长焦 35°（等效 70mm）\n");
        say("  · 视场最窄、细节最大           → 长焦 15°（等效 168mm）\n");
        say("  · 灰度、无彩色                 → 红外（不该用这一路做红色检测）\n");
        say("  · 红旗在 PPM 里显示为蓝色      → 通道是 BGR，要在 LzFrame.isBgr 标出来\n");
        say("结论记进 doc/VISION-GIMBAL-PITCH.md §9 #1，并同步 lz_vision_source.c 的默认值。\n");
    } else {
        /* 默认组合；也可显式给三个参数 */
        const LzVisionSourceConfig d = LzVisionSource_DefaultConfig();
        int pos = (int)d.position;
        int src = (int)d.source;
        int secs = 10;
        if (argc >= 3) { pos = atoi(argv[1]); }
        if (argc >= 4) { src = atoi(argv[2]); }
        if (argc >= 5) { secs = atoi(argv[3]); }
        /* ⚠️ `-1` 会满足 `<=` 类判断 —— 用显式范围检查，别用哨兵 */
        if (secs < 1 || secs > 600) {
            secs = 10;
        }
        const bool got = run_one((E_DjiLiveViewCameraPosition)pos,
                                 (E_DjiLiveViewCameraSource)src, secs);
        say("\n%s\n", got ? "取图成功。" : "取图失败 —— 试 `scan` 模式逐个换 source。");
    }

    DjiCore_DeInit();
    LzPlatform_Deinit();
    return 0;
}
