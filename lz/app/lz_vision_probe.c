/**
 * @file lz_vision_probe.c
 * @brief 探针：把**真正的视觉检测**跑在**实时的机载图像流**上。
 *
 * ## 这是"视觉接进 PSDK"的第一块可运行的东西
 *
 * 在此之前，`lz_vision`（HSV 阈值 + 连通域 + 杆列滤波）只在桌面上对着
 * 6 张存下来的照片跑过；而取图那一层（`lz_vision_source`）只在探针里
 * 取过帧、从没喂给检测器。本探针把两者接起来 ——
 *
 *     取图（PSDK 实时流） → LzVision_Detect() → 打印结果
 *
 * 于是第一次可以回答这些只能上机答的问题：
 *
 * | 问题 | 判据 |
 * |---|---|
 * | 检测器在真实场景下认不认得出红目标 | `命中率 = 命中帧数 / 总帧数` |
 * | 检测耗得起吗（会不会拖死 SDK 线程） | 每帧耗时 ms + 实际帧率 |
 * | 杆列稳定吗（同静止场景下应几乎不动） | 杆列 u 的标准差 |
 * | 置信度落在什么量级（桌面上 0.5 的门槛合适吗） | 置信度分布 |
 * | 「画面中心在旗上吗」现在是什么状态 | 由杆/旗的 v 算出**云台需要转多少度** |
 *
 * ⚠️ **检测在锁外做** —— 取图那一层已经把"回调里只拷贝"做对了，
 * 本探针只负责在**自己的线程**里取帧再算。这是那条纪律的完整闭环。
 *
 * ## 为什么不用现成的 `lz_test_vision` 改
 *
 * 那条是**回归测试**：输入是固定照片、输出与黄金值比对、必须确定性。
 * 本探针的输入是活画面（每次都不一样），判据是统计量。
 * 两者混在一起会让测试变成概率性的 —— 那是"测不出东西的用例"的另一种形态。
 *
 * ## 用法
 *
 *   ./lz_vision_probe              # 跑 10 秒，每秒一行统计
 *   ./lz_vision_probe 30           # 跑 30 秒
 *   ./lz_vision_probe 30 -v        # 每帧都打印（看抖动）
 *
 * ⚠️ 需要飞机通电、`Smart3DExplore` 已停（通道）。
 * ⚠️ **只在 `-DLZ_VISION_BACKEND=hsv` 下有意义** —— stub 后端刻意不看画面。
 */

#include <dji_logger.h>
#include <dji_platform.h>
#include <dji_typedef.h>

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lz_vision.h"
#include "lz_vision_source.h"
#include "platform/lz_platform.h"
#include "platform/lz_user_info.h"

/** 取帧缓冲。静态：`LZ_VISION_SOURCE_MAX_PIXELS` 见 lz_vision_source.h */
static uint8_t s_frame[LZ_VISION_SOURCE_MAX_PIXELS * 3];

/** 检测器内部的中间缓冲在 LzVision 里，这里只要一个上下文 */
static LzVision *s_vision = NULL;
/** 记下配置：未命中时要用"放宽门槛"的副本来区分两种失败原因 */
static LzVisionConfig s_visionCfg;

static void say(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    fflush(stdout);   /* 管道/文件下全缓冲，卡住前的输出会丢 */
}

/**
 * @brief 跑一次检测，返回是否命中，并回填关键量
 *
 * @param outU         杆列的归一化横坐标
 * @param outV         用于瞄准的归一化纵坐标（**旗面框中点** —— 见下方说明）
 * @param outConf      置信度
 * @param outVfovDeg   由实测分辨率与 82° DFOV 算出的垂直视场角
 */
/**
 * @brief 未命中时**为什么** —— 只报"没命中"是不够的
 *
 * ⚠️ 这是本项目反复踩的同一个坑的又一次应用：**只报结论、不报原因**，
 * 会把"检测器没工作"和"场景里本来就没有目标"变成同一件事。
 * 2026-09-27 实测就是这个情形 —— 探针报了 12 秒「未命中」，
 * 而画面里确实没有国旗竿（只有角落里一小块橙红色）。
 * **检测器是对的，但输出让人以为它坏了。**
 */
typedef enum {
    LZ_MISS_NONE = 0,
    LZ_MISS_NO_FRAME,
    LZ_MISS_ERR_PARAM,
    LZ_MISS_NO_RED,      /* 画面里连一个够大的红块都没有 */
    LZ_MISS_LOW_CONF,    /* 有红块但置信度不够（杆的证据不足） */
    LZ_MISS_OTHER,
} LzMissReason;

static LzMissReason s_lastMiss = LZ_MISS_NONE;
static double s_lastMissConf = 0.0;    /* 被判低置信度时的实际值 */

static const char *miss_name(LzMissReason r)
{
    switch (r) {
    case LZ_MISS_NO_FRAME:  return "还没取到帧";
    case LZ_MISS_ERR_PARAM: return "入参非法（不该发生）";
    case LZ_MISS_NO_RED:    return "画面里没有够大的红色块";
    case LZ_MISS_LOW_CONF:  return "有红块但置信度不足（杆的证据不够）";
    case LZ_MISS_OTHER:     return "其他（见日志）";
    case LZ_MISS_NONE:
    default:                return "—";
    }
}

static bool detect_once(double *outU, double *outV, double *outConf,
                        double *outVfovDeg)
{
    LzFrame frame;
    memset(&frame, 0, sizeof(frame));
    if (LzVisionSource_LatestFrame(&frame, s_frame, sizeof(s_frame)) != LZ_OK) {
        s_lastMiss = LZ_MISS_NO_FRAME;
        return false;
    }

    LzTargetList list;
    LzTargetList_Init(&list);

    const LzStatus st = LzVision_Detect(s_vision, &frame, &list);

    bool hit = false;
    if (st != LZ_OK) {
        /* `LZ_ERR_NO_TARGET` 覆盖"没红块"与"置信度不够"两种 ——
         * 而这两种对操作员的含义完全不同（前者是场景问题、
         * 后者是阈值问题），所以要分开说。区分办法：置信度门槛之上
         * 有没有东西，靠 `LzVision_Detect` 的返回码分不出来，
         * 那就把门槛临时降为 0 再跑一次 —— 代价是一次额外检测，
         * 只在未命中时发生。 */
        if (st == LZ_ERR_NO_TARGET) {
            LzVisionConfig loose = s_visionCfg;
            loose.minConfidence = 0.0;      /* 只看"有没有红块" */
            LzVision *probe = NULL;
            LzTargetList probeList;
            LzTargetList_Init(&probeList);
            bool hadRed = false;
            double conf = 0.0;
            if (LzVision_Init(&loose, &probe) == LZ_OK) {
                if (LzVision_Detect(probe, &frame, &probeList) == LZ_OK &&
                    probeList.count > 0) {
                    hadRed = true;
                    conf = probeList.items[0].confidence;
                }
                LzVision_Deinit(probe);
            }
            LzTargetList_Free(&probeList);
            s_lastMiss = hadRed ? LZ_MISS_LOW_CONF : LZ_MISS_NO_RED;
            s_lastMissConf = conf;
        } else {
            s_lastMiss = LZ_MISS_OTHER;
        }
    } else if (list.count == 0) {
        s_lastMiss = LZ_MISS_OTHER;
    } else {
        s_lastMiss = LZ_MISS_NONE;
    }

    if (st == LZ_OK && list.count > 0) {
        const LzTarget *t = &list.items[0];
        *outU = t->pixel.u;
        /* ⚠️ **瞄准点暂时用旗面框中点**，不是"杆的中点"。
         *
         * 为什么不是杆中点：杆的上下端要 `lz_pole_extent` 的结果，而它
         * **还没验证过**（现有 6 张测试图的裁剪窗口恰好等于整幅图，
         * 量到的是图边而不是杆端 —— 见 CLAUDE.md「步骤 3 卡在测试数据上」）。
         * 在拿到现场杆的近景照片之前，用旗中心是**唯一有依据**的选择；
         * 但它是**降级方案**，所以这里显式打出来，不假装等价。 */
        *outV = (t->pixel.topV + t->pixel.bottomV) * 0.5;
        *outConf = t->confidence;
        hit = true;
    }

    /* 垂直 FOV 由**实测分辨率**与广角 DFOV 82° 现算（`[V]` 2026-09-27 探针
     * 已确认取到的是广角）。用实测 W/H 而不是规格的 4:3 —— 两者未必一致。 */
    *outVfovDeg = LzVision_VerticalFovDeg(frame.width, frame.height, 82.0);

    LzTargetList_Free(&list);
    return hit;
}

int main(int argc, char **argv)
{
    int seconds = 10;
    bool verbose = false;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-v") == 0) {
            verbose = true;
        } else {
            const int v = atoi(argv[i]);
            if (v >= 1 && v <= 600) {
                seconds = v;
            }
        }
    }

    say("[lz_vision_probe] 视觉检测跑在实时图像流上（后端：%s）\n",
        LzVision_BackendName());
    if (strcmp(LzVision_BackendName(), "stub") == 0) {
        say("⚠️ 当前是 **stub 后端** —— 它刻意不看画面，结果与图像内容无关。\n");
        say("   要看真算法请用 -DLZ_VISION_BACKEND=hsv 重编。\n");
    }

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

    /* ---- 取图 ---- */
    const LzStatus srcSt = LzVisionSource_Init();
    say("取图初始化：%s\n", (srcSt == LZ_OK) ? "✅" : "❌");
    if (srcSt != LZ_OK) {
        goto out;
    }

    /* ---- 检测器 ---- */
    const LzVisionConfig cfg = LzVision_DefaultConfig();
    s_visionCfg = cfg;
    if (LzVision_Init(&cfg, &s_vision) != LZ_OK) {
        say("检测器初始化失败\n");
        goto out;
    }
    say("检测器就绪（minConfidence=%.2f，minBlobArea=%d）\n\n",
        cfg.minConfidence, cfg.minBlobArea);

    /* 等第一帧 */
    {
        const T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
        for (int i = 0; i < 80; ++i) {
            LzFrame f;
            if (LzVisionSource_LatestFrame(&f, s_frame, sizeof(s_frame)) == LZ_OK) {
                break;
            }
            if (osal != NULL && osal->TaskSleepMs != NULL) {
                (void)osal->TaskSleepMs(100);
            }
        }
    }

    /* ---- 主循环：按 100 ms 取最新帧做检测 ---- */
    say("跑 %d 秒（每 100 ms 检测最新一帧）...\n", seconds);
    say("%-5s %-8s %-9s %-8s %-9s %s\n",
        "sec", "命中/总", "命中率", "置信度", "杆列u", "云台需转(度)");

    uint32_t total = 0, hit = 0;
    double sumConf = 0.0, sumU = 0.0, sumU2 = 0.0, sumDelta = 0.0;
    uint32_t nU = 0, nDelta = 0;
    uint32_t lastTotal = 0, lastHit = 0;
    double lastVfov = 0.0;

    const T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    const int ticks = seconds * 10;

    for (int t = 0; t < ticks; ++t) {
        double u = 0.0, v = 0.0, conf = 0.0, vfov = 0.0;
        total++;
        const bool got = detect_once(&u, &v, &conf, &vfov);
        if (got) {
            hit++;
            sumConf += conf;
            sumU += u;
            sumU2 += u * u;
            nU++;
            lastVfov = vfov;

            /* 云台需要转多少度才能让 v 落到 0.5（画面中心）
             * —— 这就是方案 §7 步骤 4 的那个换算，第一次跑在活画面上。 */
            const double d = LzVision_PixelOffsetToDeg(v, vfov);
            sumDelta += d;
            nDelta++;

            if (verbose) {
                say("  帧 %4u  u=%.4f v=%.4f conf=%.3f  Δθ=%+.2f°\n",
                    total, u, v, conf, d);
            }
        }

        if ((t + 1) % 10 == 0) {
            const uint32_t dTotal = total - lastTotal;
            const uint32_t dHit = hit - lastHit;
            const double rate = (dTotal > 0) ? (100.0 * dHit / dTotal) : 0.0;
            say("%-5d %-8u %-8.0f%% ", (t + 1) / 10, hit, 100.0 * hit / total);
            if (hit > lastHit) {
                say("%-9.3f %-9.4f %+.2f°\n",
                    sumConf / (double)nU, sumU / (double)nU,
                    (nDelta > 0) ? (sumDelta / (double)nDelta) : 0.0);
            } else {
                say("%-9s %s\n", "(本秒未命中)", miss_name(s_lastMiss));
                if (s_lastMiss == LZ_MISS_LOW_CONF) {
                    say("        ↳ 检测到红块但置信度只有 %.3f（阈值 %.2f）"
                        "—— 多半是「杆」的证据不足：没看到够长的竖线\n",
                        s_lastMissConf, cfg.minConfidence);
                }
            }
            (void)rate;
            lastTotal = total;
            lastHit = hit;
        }

        if (osal != NULL && osal->TaskSleepMs != NULL) {
            (void)osal->TaskSleepMs(100);
        }
    }

    /* ---- 汇总 ---- */
    say("\n=== 汇总 ===\n");
    LzVisionSourceStats sst;
    LzVisionSource_GetStats(&sst);
    say("  图像流：收到 %u 帧，尺寸 %dx%d，丢帧 %u/%u/%u/%u（格式/过大/行宽/未就绪）\n",
        sst.frames, sst.lastWidth, sst.lastHeight,
        sst.droppedFmt, sst.droppedTooBig, sst.droppedStride, sst.droppedNotReady);
    say("  检测  ：%u/%u 命中（%.1f%%）\n", hit, total,
        (total > 0) ? (100.0 * hit / total) : 0.0);

    if (nU > 0) {
        const double meanU = sumU / (double)nU;
        const double varU = (sumU2 / (double)nU) - meanU * meanU;
        const double sdU = (varU > 0.0) ? sqrt(varU) : 0.0;
        say("  杆列 u：均值 %.4f，标准差 %.4f（静止场景下应很小）\n",
            meanU, sdU);
        say("  置信度：均值 %.3f（阈值 %.2f）\n", sumConf / (double)nU,
            cfg.minConfidence);
        say("  云台需转：均值 %+.2f°（让旗面中点落到画面中心）\n",
            (nDelta > 0) ? (sumDelta / (double)nDelta) : 0.0);
        say("     ⚠️ 这是按**旗面框中点**算的（降级方案，理由见源码注释）；\n");
        say("        \"杆的中点\"要等现场照片验证 `lz_pole_extent` 之后才能用。\n");
        say("  垂直 FOV：%.2f°（由实测 %dx%d 与广角 DFOV 82° 现算）\n",
            lastVfov, sst.lastWidth, sst.lastHeight);
    } else {
        say("  一帧都没命中：%s\n", miss_name(s_lastMiss));
        if (s_lastMiss == LZ_MISS_NO_RED) {
            say("    ⇒ **检测器在工作**，是画面里没有够大的红色块 ——\n");
            say("      室内对着桌椅拍本来就不该命中。要看真效果得对着红旗。\n");
        } else if (s_lastMiss == LZ_MISS_LOW_CONF) {
            say("    ⇒ 检测到红块（置信度 %.3f，阈值 %.2f）但被判不可用。\n",
                s_lastMissConf, cfg.minConfidence);
            say("      多半是「杆」的证据不足（那一段竖线不够长）——\n");
            say("      若现场确实有杆，说明 poleContrastThreshold 等参数需要按现场调。\n");
        }
        return 0;   /* 不是错误：没目标是合法结果 */
    }

out:
    say("\n清理...\n");
    if (s_vision != NULL) {
        LzVision_Deinit(s_vision);
        s_vision = NULL;
    }
    LzVisionSource_Stop();
    DjiCore_DeInit();
    LzPlatform_Deinit();
    return 0;
}
