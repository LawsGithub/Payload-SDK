/**
 * @file lz_mission_probe.c
 * @brief 探针：复现"上传 KMZ 成功后启动航点任务"并抓崩溃点。
 *
 * ## 它要回答什么
 *
 * 主应用 `lz_app` 在拨开关后跑了这条路径：
 *
 *     生成 KMZ → 上传（MD5 校验通过）→ DjiWaypointV3_Action(START) →
 *
 * 在**上传完成、START 之前**进程收到 SIGSEGV（2026-09-20 实测，日志
 * `DJI_0007` 停在 index 40，没有出现 `Check kmz file md5sum success`，
 * 也没有出现我自己打的那行"KMZ 上传完成"）。
 *
 * 这一步到底是"SDK 内部崩"还是"我们的诊断代码崩"，主应用里分不清 ——
 * 因为主应用同时跑着控件线程、状态推送线程、启动诊断订阅。本探针把
 * 变量收干净：
 *
 *   - 只调 WaypointV3 + FcSubscription 两个模块
 *   - **不注册控件、不建线程**
 *   - 每一步都先 `fflush(stdout)` 再往下走
 *
 * 于是崩溃点就落在那行没打出来的"下一步"上。
 *
 * ## 三种模式（命令行参数）
 *
 *   ./lz_mission_probe 0           只上传，不启动（基线）
 *   ./lz_mission_probe 1           上传 + 订阅诊断 + 启动
 *   ./lz_mission_probe 2           上传 + 订阅诊断 + 启动 + 读话题（含最后那步）
 *   ./lz_mission_probe 3           **只订阅 + 读话题**，不碰 KMZ / 不启动
 *
 * 模式 0 与 1 的差 = 订阅那两块是不是凶手；1 与 2 的差 = 读话题是不是；
 * 模式 3 单独回答"读话题本身是否就崩" —— 把"上传/启动失败留下的坏状态"
 * 这个嫌疑排除掉。
 *
 * ⚠️ 前提：飞机通电连接，且已让出 PSDK 通道：
 *      pgrep -x Smart3DExplore >/dev/null || /system/bin/dji_app_ctl stop Smart3DExplore
 */

#include <dji_core.h>
#include <dji_fc_subscription.h>
#include <dji_logger.h>
#include <dji_typedef.h>
#include <dji_waypoint_v3.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "platform/lz_platform.h"
#include "platform/lz_user_info.h"

/* 每一步之前都刷一次 stdout，让"崩在哪一步"从日志里直接可读。
 * 输出重定向到管道/文件时 stdio 是全缓冲 —— 不刷的话崩溃会吞掉
 * 最后几行，而那几行恰恰是定位崩溃点的关键。 */
#define STEP(msg)                     \
    do {                              \
        printf("[STEP] %s\n", (msg)); \
        fflush(stdout);               \
    } while (0)

/* 读一个话题并打印；返回是否读成功。
 *
 * ️ 关键：**在读之前先打印并 flush**。主应用在这里崩过，而崩溃会把
 * stdio 缓冲区里还没刷出去的内容一起吞掉 —— 只在读之后打印，等于
 * 崩溃时什么都看不到。先打印才知道死在哪一个话题上。 */
/* 话题数据回调。官方样例订阅时**总是**传一个回调（test_fc_subscription.c:63），
 * 而我们之前传的是 NULL —— 头文件写 "If the callback function is not needed,
 * this item can be set as NULL"，但 SDK 内部很可能仍建了与回调相关的链表/节点，
 * 读的时候空指针解引用。gdb 抓到崩溃在 DjiDataSubscriptionDds_v3_GetLastValueOfTopic
 * 内部，不是我们的代码 —— 与这个猜测吻合。 */
/* 话题缓存（回调写入，主线程读）。
 *
 * ️ 不用 DjiFcSubscription_GetLatestValueOfTopic —— 实测（模式 0/3/4）
 * 它必崩，栈在 DjiDataSubscriptionDds_v3_GetLastValueOfTopic 内部。
 * 回调能收到数据（实证 10 秒 518 次），所以改为回调里自己存。 */
static volatile int g_cbTotal = 0;
static volatile bool g_gotFlight = false;
static volatile uint8_t g_flight = 0;
static volatile bool g_gotRc = false;
static volatile T_DjiFcSubscriptionRC g_rc = {0};
static volatile bool g_gotGps = false;
static volatile T_DjiFcSubscriptionGpsDetails g_gps = {0};
static volatile bool g_gotFused = false;
static volatile T_DjiFcSubscriptionPositionFused g_fused = {0};
static volatile bool g_gotHome = false;

/* 5 个专用回调：签名里没有 topic，所以一个话题一个函数 */
#define DEF_CB(name, dst, flag, type)                                     \
    static T_DjiReturnCode name(const uint8_t *d, uint16_t sz,            \
                                const T_DjiDataTimestamp *ts)             \
    {                                                                     \
        (void)ts;                                                         \
        if (d != NULL && sz >= sizeof(type)) {                            \
            memcpy((void *)&(dst), d, sizeof(type));                      \
            (flag) = true;                                                \
            g_cbTotal++;                                                  \
        }                                                                 \
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;                      \
    }
DEF_CB(cb_flight, g_flight, g_gotFlight, T_DjiFcSubscriptionFlightStatus)
DEF_CB(cb_rc, g_rc, g_gotRc, T_DjiFcSubscriptionRC)
DEF_CB(cb_gps, g_gps, g_gotGps, T_DjiFcSubscriptionGpsDetails)
DEF_CB(cb_fused, g_fused, g_gotFused, T_DjiFcSubscriptionPositionFused)
DEF_CB(cb_home, g_flight, g_gotHome, T_DjiFcSubscriptionHomePointSetStatus)

int main(int argc, char **argv)
{
    int mode = 1;
    if (argc >= 2) {
        mode = atoi(argv[1]);
    }
    if (mode < 0 || mode > 4) {
        mode = 1;
    }

    printf("=== lz_mission_probe  模式 %d ===\n", mode);
    printf("  0 = 只上传\n  1 = 上传 + 订阅 + 启动\n  2 = 上传 + 订阅 + 启动 + 读话题\n"
           "  3 = 只订阅 + 读话题（不碰 KMZ）\n  4 = 同 3，但先等 5 秒看回调有没有数据\n");
    fflush(stdout);

    /* ---- 平台层 ---- */
    if (LzPlatform_Prepare() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("平台层注册失败\n");
        return 1;
    }
    T_DjiUserInfo userInfo;
    if (LzUserInfo_Fill(&userInfo) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("凭据填充失败\n");
        LzPlatform_Deinit();
        return 1;
    }

    STEP("DjiCore_Init（阻塞 2-4 秒）");
    if (DjiCore_Init(&userInfo) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("DjiCore_Init 失败\n");
        LzPlatform_Deinit();
        return 1;
    }

    STEP("DjiWaypointV3_Init");
    T_DjiReturnCode rc = DjiWaypointV3_Init();
    printf("      rc=0x%08X\n", (unsigned)rc);
    fflush(stdout);

    STEP("DjiCore_ApplicationStart");
    rc = DjiCore_ApplicationStart();
    printf("      rc=0x%08X\n", (unsigned)rc);
    fflush(stdout);

    /* ️ 顺序要求：FcSubscription_Init 必须在 ApplicationStart **之后**
     * （官方文档原文："请勿在 main() 函数中调用本接口……启动调度器后，
     *   该接口将正常运行"。放在前面会返回 SUCCESS 但随后 SIGSEGV —— 实测 2026-09-20） */
    if (mode >= 1) {
        STEP("等 3 秒，让 ApplicationStart 的调度器彻底就绪");
        sleep(3);

        STEP("DjiFcSubscription_Init（ApplicationStart 之后）");
        rc = DjiFcSubscription_Init();
        printf("      rc=0x%08X\n", (unsigned)rc);
        fflush(stdout);

        STEP("订阅 5 个话题");
        static const E_DjiFcSubscriptionTopic topics[] = {
            DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
            DJI_FC_SUBSCRIPTION_TOPIC_RC,
            DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
            DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
            DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_SET_STATUS,
        };
        DjiReceiveDataOfTopicCallback cbs[] = {cb_flight, cb_rc, cb_gps, cb_fused, cb_home};
        for (size_t i = 0; i < sizeof(topics) / sizeof(topics[0]); ++i) {
            rc = DjiFcSubscription_SubscribeTopic(topics[i], DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                  cbs[i]);
            printf("      话题 0x%08X rc=0x%08X %s\n", (unsigned)topics[i], (unsigned)rc,
                   (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? "OK" : "失败");
            fflush(stdout);
        }
    }

    if (mode == 3 || mode == 4) {
        /* 只验证"读话题本身"，与 KMZ/启动作业无关 */
        STEP("跳过 KMZ 与启动，直接读话题");
        goto read_topics;
    }

    /* ---- 读 KMZ（用主应用生成的那份；没有就用内置的失败）---- */
    const char *kmzPath = "data/orbit.kmz";
    STEP("读 KMZ 文件");
    FILE *fp = fopen(kmzPath, "rb");
    if (fp == NULL) {
        printf("      打不开 %s —— 先用 lz_app 生成一次，或改这里的路径\n", kmzPath);
        fflush(stdout);
        return 1;
    }
    fseek(fp, 0, SEEK_END);
    const long size = ftell(fp);
    rewind(fp);
    printf("      大小 %ld 字节\n", size);
    fflush(stdout);
    if (size <= 0) {
        fclose(fp);
        printf("      文件为空\n");
        return 1;
    }
    uint8_t *buf = malloc((size_t)size);
    if (buf == NULL || fread(buf, 1, (size_t)size, fp) != (size_t)size) {
        fclose(fp);
        free(buf);
        printf("      读入失败\n");
        return 1;
    }
    fclose(fp);

    STEP("DjiWaypointV3_UploadKmzFile（上传，含 MD5 校验）");
    rc = DjiWaypointV3_UploadKmzFile(buf, (uint32_t)size);
    free(buf);
    printf("      ★ 上传返回 rc=0x%08X\n", (unsigned)rc);
    fflush(stdout);

    if (mode == 0) {
        STEP("模式 0 结束 —— 不启动");
        goto cleanup;
    }

    STEP("DjiWaypointV3_Action(START)");
    rc = DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_START);
    printf("      ★ 启动返回 rc=0x%08X %s\n", (unsigned)rc,
           (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? "（启动了！）" : "（被拒）");
    fflush(stdout);

    if (mode == 2 || mode == 3) {
read_topics:
        /* 官方样例在读之前先 sleep 一拍（test_fc_subscription.c:173 的
         * TaskSleepMs 在 GetLatestValueOfTopic **之前**）。先补上这一拍，
         * 排除"订阅后立刻读、缓冲区还没有数据"这个可能。 */
        STEP("读话题前先 sleep 3 秒");
        sleep(3);
        STEP("等 3 秒收数据（回调写入缓存），然后打印缓存");
        sleep(3);

        printf("     回调累计 %d 次\n", g_cbTotal);
        printf("     飞行状态   : %s\n", g_gotFlight ? "有数据" : "无数据");
        if (g_gotFlight) printf("        值 = %u\n", (unsigned)g_flight);
        printf("     RC         : %s\n", g_gotRc ? "有数据" : "无数据");
        if (g_gotRc) printf("        mode = %d\n", g_rc.mode);
        printf("     GPS 详情   : %s\n", g_gotGps ? "有数据" : "无数据");
        if (g_gotGps) printf("        fixState = %.0f，卫星 = %u\n",
                             g_gps.fixState, (unsigned)g_gps.totalSatelliteNumberUsed);
        printf("     融合位置   : %s\n", g_gotFused ? "有数据" : "无数据");
        if (g_gotFused) printf("        lat = %.7f，lon = %.7f，卫星 = %u\n",
                               g_fused.latitude * 180.0 / M_PI,
                               g_fused.longitude * 180.0 / M_PI,
                               (unsigned)g_fused.visibleSatelliteNumber);
        fflush(stdout);
    }

    STEP("全部完成 —— 没有崩溃");
    printf("\n结论：把上面每个 [STEP] 是否都打出来了记下来。\n"
           "      最后一个打出来的 [STEP] = 崩溃前走到的最后一步；\n"
           "      下一个没打出来的 [STEP] = 凶手。\n");
    fflush(stdout);

cleanup:
    STEP("收尾");
    DjiWaypointV3_DeInit();
    DjiCore_DeInit();
    LzPlatform_Deinit();
    printf("退出\n");
    return 0;
}