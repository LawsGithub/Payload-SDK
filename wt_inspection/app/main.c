/**
 * @file main.c
 * @brief 风机叶片巡检 —— PSDK 机载应用入口（M4T + 妙算3）
 *
 * 平台注册部分刻意与官方 manifold3 样例保持一致：OSAL / USB Bulk /
 * Socket / FileSystem 四个处理器 + 控制台与文件两路日志。这样做的原因
 * 是平台层一旦出问题，现象往往是"飞机连不上"，很难定位；保持与官方
 * 完全一致可以把这类问题排除在外，把注意力留给业务逻辑。
 */

#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <dji_aircraft_info.h>
#include <dji_core.h>
#include <dji_logger.h>
#include <dji_platform.h>
#include <dji_typedef.h>

#include "dji_sdk_app_info.h"
#include "hal_usb_bulk.h"
#include "osal/osal.h"
#include "osal/osal_fs.h"
#include "osal/osal_socket.h"
#include "wt_runner.h"
#include "wt_ai_overlay.h"
#include "wt_ai_stream_probe.h"
#include "wt_frame_pump.h"
#include "wt_laser.h"
#include "wt_mission_watch.h"
#include "wt_ped_square.h"
#include "wt_square_mission.h"
#include "wt_widget.h"
/*
 * 采集链路（推理 + 编排）只在开了 TensorRT 时才编进来。
 *
 * 为什么是条件编译而不是运行时开关：engine 与 TensorRT 库只在妙算3 上有，
 * 而仓库级 CLAUDE.md 硬约束 3 禁止交叉编译 —— 本机构建必须仍然可用，
 * 否则连 x86_64 链接自检这条路都断了。
 */
#if defined(WT_WITH_TRT)
#include "wt_ped.h"
#include "wt_survey.h"
#include "wt_vision.h"
#endif

/*
 * 默认配置路径：**相对应用目录**，不是绝对路径。
 *
 * 应用以 uid=1000（dji）运行，/data/ 不可写，绝对路径写在那里必然失败。
 * 官方打包脚本 build_dpk.sh 会在包内建好 data/logs 与 data/media，属主
 * dji:dji 可写 —— 这就是官方的「相对应用目录」约定，配置也放这里。
 * 该常量同时是 WtAppConfig 里默认 outputDir 的基准（见 wt_app_config.c）。
 */
#define WT_DEFAULT_CONFIG_PATH "data/wt_config.ini"
#define WT_LOG_PATH            "data/logs/WT"
#define WT_LOG_PATH_MAX_SIZE   128

static WtRunner s_runner;
static volatile sig_atomic_t s_stopRequested = 0;
static FILE *s_logFile = NULL;
/*
 * 配置是否就绪。**它不是启动门槛，只是作业门槛** —— 配置缺失时进程仍要
 * 活着走完 SDK 身份校验，否则 .dpk 装不上（见 main 里步骤 4 的说明）。
 */
static bool s_configReady = false;

/*
 * 本次运行里最近一次行人打点的结果 —— 「绕点飞行」的基点。
 *
 * 只认本进程里打出来的点，**不读 ped_points.csv**：CSV 里可能是昨天、别处
 * 打的点，拿它当基点等于让飞机低空飞去一个旧坐标。进程重启就得重新打点。
 *
 * 而且只认**最近一轮**：每拨上一次「行人瞄准」就先作废旧点（WtApp_RunPed 开头），
 * 只有这一轮打点成功才重新写入。否则「先打中 A、再瞄 B 失败」之后按绕点飞行，
 * 飞机会绕着 A 的旧坐标飞，而操作员以为绕的是 B。
 */
static bool s_havePedPoint = false;
static WtGeo s_pedPoint;
static uint32_t s_pedPointMs = 0;

static uint32_t WtApp_NowMs(void)
{
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    uint32_t now = 0;

    if (osal != NULL && osal->GetTimeMs != NULL) {
        osal->GetTimeMs(&now);
    }
    return now;
}
#if defined(WT_WITH_TRT)
/* 采集链路（取流 + 推理 + 激光/云台）是否已成功就绪 */
static bool s_surveyReady = false;
/* 本次运行是否已采集过 —— 叶片没动，重复采没有意义 */
static bool s_surveyDone = false;
/* 行人瞄准链路（取流 + 行人检测推理 + 激光/云台）是否已就绪 */
static bool s_pedReady = false;

/**
 * @brief 云台 + 激光自检。**不需要风机在画面里。**
 *
 * 验的是整条对准闭环的两块基石，而它们到现在都还没在本项目的代码路径上
 * 验证过：
 *
 *   ① **云台 pan 能不能被 PSDK 转。** DJI 官方规格页把 M4T 的 Pan 标成
 *      "Not controllable"，但同一页又写着 yaw 轴 MSDK 可控、且机械范围有
 *      ±65°。姊妹项目实测 PSDK 能转。整条对准闭环都压在这条上，所以必须
 *      自己验一次。
 *   ② **激光能不能读到有效回波。**
 *
 * 做法是「下发 → 读回」而不是只看返回码：Payload-SDK issue #555/#563 的
 * 教训就是「下发了 ≠ 执行了」（#563：实际速度只有下发的 10–20%）。
 * 只有读回角度才算闭环。
 *
 * 安全性：pitch 取 −30/−45 两个温和值，yaw 只相对机头偏 ±20°，都在 M4T
 * 可控范围（pitch −90~70、pan 相对机头 ±60）内，飞机在地面时安全。结束后
 * 归还模式。
 */
static void WtApp_RunGimbalLaserSelfTest(void)
{
    WtTelemetry *tele = &s_runner.telemetry;
    double baseYaw = 0.0;
    double baseBodyYaw = 0.0;
    int i;

    USER_LOG_INFO("==== 云台 + 激光自检开始（不需要风机）====");

    if (!WtLaser_BorrowFreeMode()) {
        USER_LOG_ERROR("自检：借 FREE 模式失败，后续角度可能被机头运动带偏");
    }

    /*
     * 先拿一次基准姿态。
     *
     * 每秒报一次进度，而不是等够时间后一次报结果 —— 这样
     * 「等得不够」与「根本不推」才能分开。姊妹项目的原话：
     * 「第一轮只等 600 ms 就下了'读不到'的结论，那是错的；
     *   第二轮改成每秒报累计条数，1 秒就收到 50 条」。
     * 只有时间能把这两件事分开。
     */
    for (i = 0; i < 10; i++) {
        WtTelemetry_Poll(tele);
        if (tele->gimbal.valid && tele->aircraft.validAttitude) {
            break;
        }
        USER_LOG_INFO("自检：等姿态… %d/10（云台%s 机头%s）",
                      i + 1, tele->gimbal.valid ? "✓" : "✗",
                      tele->aircraft.validAttitude ? "✓" : "✗");
        usleep(1000 * 1000);
    }
    if (!tele->gimbal.valid) {
        /*
         * 云台角度话题在 M4T 上订不上（0xE0），这是硬限制：
         * 2026-10-22 实测，换 GIMBAL_ANGLES_ON_POS_NO1、把
         * DjiGimbalManager_Init 提前到订阅之前，**都无效**；而同一次
         * Init 里 QUATERNION 正常推送 ⇒ 不是订阅机制/时序/话题编号问题。
         *
         * 但**这不阻断目标**：整条对准闭环只需要"下发目标角"，
         * 不需要读回。所以降级为「只验下发是否被接受」——
         * 这同样能回答"pan 到底能不能控"（被拒 vs 接受）。
         *
         * 代价：无法闭环校验到位（issue #563 说实速只有下发的 10–20%，
         * 所以"下发了"不等于"转到了"）。现场需目视确认云台有没有动。
         */
        USER_LOG_WARN("自检：云台角度话题不可订（M4T 硬限制），"
                      "降级为「只验下发」—— 请目视观察云台是否真的转动");
        /*
         * 读不到云台角 ⇒ 拿不到"当前 yaw"。此时**绝不能填 0** 下发：
         * 绝对角模式下 0 是"转到正北"，机头不朝北就够不到，整条命令被拒
         * （2026-10-22 本项目自检实测：pitch 下发失败正是这个原因）。
         * 用**机头朝向**兜底 —— pan 相对机头 ±60° 内，机头朝向一定够得到。
         */
        tele->gimbal.valid = false;
        baseYaw = tele->aircraft.validAttitude ? tele->aircraft.yawDeg : 0.0;
        tele->gimbal.yawDeg = baseYaw;
        tele->gimbal.pitchDeg = 0.0;
        baseBodyYaw = baseYaw;
    } else {
        baseYaw = tele->gimbal.yawDeg;
        baseBodyYaw = tele->aircraft.validAttitude ? tele->aircraft.yawDeg : baseYaw;
        USER_LOG_INFO("自检：基准 —— 云台 pitch %.1f° yaw %.1f°，机头 %.1f°",
                      tele->gimbal.pitchDeg, baseYaw, baseBodyYaw);
    }

    /* ---- ① pitch：下发 → 读回 ---- */
    {
        const double targets[] = {-30.0, -45.0};
        size_t k;

        for (k = 0; k < sizeof(targets) / sizeof(targets[0]); k++) {
            /* yaw 原样回填当前值 —— 填 0 会被当成"转到正北"而整条被拒 */
            if (!WtLaser_SetGimbalAbs(targets[k], tele->gimbal.yawDeg)) {
                USER_LOG_ERROR("自检：pitch → %.1f° 下发失败", targets[k]);
                continue;
            }
            for (i = 0; i < 15; i++) {
                usleep(200 * 1000);
                WtTelemetry_Poll(tele);
            }
            if (tele->gimbal.valid) {
                USER_LOG_INFO("自检：pitch 目标 %.1f° → 读回 %.1f°（偏差 %+.1f°）%s",
                              targets[k], tele->gimbal.pitchDeg,
                              tele->gimbal.pitchDeg - targets[k],
                              (fabs(tele->gimbal.pitchDeg - targets[k]) < 3.0)
                                  ? "  ✓ 到位" : "  ⚠ 未到位");
            } else {
                USER_LOG_INFO("自检：pitch → %.1f° 已被接受（无读回，请目视）",
                              targets[k]);
            }
        }
    }

    /* ---- ② pan：这是关键的一条 ---- */
    {
        const double relTargets[] = {20.0, -20.0, 0.0};
        size_t k;

        for (k = 0; k < sizeof(relTargets) / sizeof(relTargets[0]); k++) {
            const double targetYaw = Wt_Wrap360(baseBodyYaw + relTargets[k]);

            if (!WtLaser_SetGimbalAbs(tele->gimbal.pitchDeg, targetYaw)) {
                USER_LOG_ERROR("自检：yaw → %.1f°（相对机头 %+.0f°）下发失败"
                               " —— 若是 0x600000004 则为超限，0x600000006 为权限",
                               targetYaw, relTargets[k]);
                continue;
            }
            for (i = 0; i < 15; i++) {
                usleep(200 * 1000);
                WtTelemetry_Poll(tele);
            }
            if (tele->gimbal.valid) {
                const double err = Wt_AngleDiff(tele->gimbal.yawDeg, targetYaw);

                USER_LOG_INFO("自检：yaw 目标 %.1f°（相对机头 %+.0f°）→ 读回 %.1f°"
                              "（偏差 %+.1f°）%s",
                              targetYaw, relTargets[k], tele->gimbal.yawDeg, err,
                              (fabs(err) < 5.0) ? "  ✓ pan 确实能转"
                                                : "  ⚠ 没转到（pan 不可控？）");
            } else {
                USER_LOG_INFO("自检：yaw → %.1f°（相对机头 %+.0f°）已被接受 "
                              "⇒ PSDK 能下发 pan（无读回，请目视确认云台水平转动）",
                              targetYaw, relTargets[k]);
            }
        }
    }

    /*
     * ---- ②b 相对角大幅转动：验证云台到底动不动 ----
     *
     * 绝对角模式下，如果读不到当前角，我们连"目标角是否真的不同于当前角"
     * 都无法确定（云台可能本来就在那个位置）。相对角模式不需要这个前提 ——
     * 直接"在当前基础上转 40°"，若云台真能动，肉眼一定看得见。
     *
     * 这是对「返回 SUCCESS ≠ 真的转了」最直接的检验（issue #555/#563）。
     */
    {
        USER_LOG_WARN("自检：即将用**相对角**大幅转动云台 —— 请盯着云台看");
        USER_LOG_WARN("自检：  ① 先向下 40°（俯仰）");
        (void)WtLaser_RotateRel(-40.0, 0.0);
        sleep(3);

        USER_LOG_WARN("自检：  ② 再向左 40°（偏航）");
        (void)WtLaser_RotateRel(0.0, -40.0);
        sleep(3);

        USER_LOG_WARN("自检：  ③ 回中（右 40°、上 40°）");
        (void)WtLaser_RotateRel(40.0, 40.0);
        sleep(3);
        USER_LOG_WARN("自检：相对转动结束 —— 云台动了吗？");
    }

    /* ---- ③ 激光 ---- */
    {
        WtLaserSample sample;
        int ok = 0;

        for (i = 0; i < 5; i++) {
            if (WtLaser_Read(&sample) && sample.valid) {
                ok++;
                USER_LOG_INFO("自检：激光第 %d 次 —— 距离 %.1f m，"
                              "打中点 (%.7f, %.7f, %.1f m)，落点屏幕 (%.1f%%, %.1f%%)",
                              i + 1, sample.distanceM, sample.point.lat,
                              sample.point.lon, sample.point.alt,
                              sample.screenXPct, sample.screenYPct);
            } else {
                USER_LOG_WARN("自检：激光第 %d 次无有效回波（exception=%u，"
                              "对着天空或超量程时正常）", i + 1,
                              (unsigned)sample.exception);
            }
            usleep(400 * 1000); /* > 250ms，避开 5Hz 限频拿到旧值 */
        }
        USER_LOG_INFO("自检：激光 %d/5 次有效", ok);
    }

    WtLaser_ReturnFollowMode();
    USER_LOG_INFO("==== 自检结束（云台模式已归还）====");
}

/**
 * @brief 跑一次采集会话，直到结束或被中止。
 *
 * ## 为什么要自己一个循环，不挂在待命循环上
 *
 * 待命循环是 `sleep(1)` 的节奏，而对准是个闭环：转云台 → 等稳 → 再看一帧 →
 * 微调。1 Hz 下这个闭环会慢到没法用。
 *
 * 这里取 **5 Hz** —— 与 `DjiCameraManager_GetLaserRangingInfo` 的
 * 「最大更新频率 5Hz」对齐。再快也拿不到新的激光值，只是白转。
 *
 * ## 这个循环会阻塞
 *
 * `WtSurvey_Tick()` 内部调激光读取，那是同步阻塞调用（实测最多 1.2 s）。
 * 所以它只能跑在主循环里，**绝不能放进 PSDK 回调线程**。
 */
static void WtApp_RunSurvey(void)
{
    WtSurvey survey;

    if (!s_surveyReady) {
        WtWidget_PostMessage("采集链路未就绪（见启动日志）");
        return;
    }
    if (s_runner.config.turbineCount <= 0) {
        WtWidget_PostMessage("配置里没有风机，无法建立坐标系");
        return;
    }

    /*
     * 台账提供坐标系原点与比对基准 —— 实测只修正盘面姿态与相位，
     * 不反推塔基（那不可解，见 wt_rotor_fit.h）。
     */
    if (!WtSurvey_Begin(&survey, &s_runner.config.turbines[0].spec, NULL)) {
        WtWidget_PostMessage("采集启动失败：台账非法");
        return;
    }

    while (!WtSurvey_IsFinished(&survey) && !s_stopRequested) {
        char progress[128];

        if (WtWidget_TakeAbortRequest()) {
            WtSurvey_Abort(&survey);
            break;
        }

        /*
         * 先刷遥测再 Tick：决策层要用这一帧的云台姿态与机头朝向，
         * 用上一帧的会让云台按过时的姿态去算目标角。
         */
        WtTelemetry_Poll(&s_runner.telemetry);
        WtSurvey_Tick(&survey, &s_runner.telemetry);

        WtSurvey_FormatProgress(&survey, progress, sizeof(progress));
        WtWidget_PostMessage("%s", progress);

        usleep(200 * 1000); /* 5 Hz，与激光的更新上限一致 */
    }

    /*
     * 把实测停用角写回配置，让既有规划流程能用上。
     *
     * 这是本次接入唯一与作业流程的交集，也是采集最直接的价值：
     * `WtRunner_PlanTurbine` 在停用角未知时**无条件拒绝规划**，而台账
     * 给不出它（叶片每次停的位置都不同）。
     *
     * ⚠️ 盘面朝向（rotAxis）的实测结果**暂未注入** —— 那要改
     * `WtRunner_PlanTurbine` 里 `WtTurbine_BuildRotorFrame()` 那一行，
     * 属于动核心流程，等整条链路上机验证通过再做。
     */
    /*
     * 归还云台模式 —— **必须在这里还，不能等进程退出**。
     *
     * 2026-10-22 实测：`dji_app_ctl stop` 直接杀进程，收尾路径一行都不执行，
     * 所以放在 DeInit 里归还等于永远不还，FREE 就留在飞机上了（之后机头
     * 连续转动会顶 pan 限位、报云台电机异常）。
     *
     * 放在成败判断**之前**，因为失败路径同样借过模式。
     */
    WtLaser_ReturnFollowMode();

    if (survey.fitAccepted) {
        s_runner.config.turbines[0].parkPhaseDeg = survey.fit.parkPhaseDeg;
        WtWidget_SetParkPhaseInitial(survey.fit.parkPhaseDeg);
        USER_LOG_INFO("采集：实测停用角 %.1f° 已写回配置（半径 %.1f m，"
                      "散布 %.2f%%，法向与台账差 %.1f°）",
                      survey.fit.parkPhaseDeg, survey.fit.rotorRadiusM,
                      survey.fit.radiusSpreadPct, survey.fit.axisAngleErrDeg);
        WtWidget_PostMessage("采集完成 停用角%.0f°", survey.fit.parkPhaseDeg);
    } else {
        USER_LOG_ERROR("采集未成功，停用角保持原值");
        WtWidget_PostMessage("采集失败 %s",
                             survey.logic.message ? survey.logic.message : "");
    }
}

/**
 * @brief 跑一次行人瞄准打点会话，直到完成、失败、被中止或开关拨回。
 *
 * ## 为什么 10 Hz
 *
 * 对准是闭环：转云台 → 读回停稳 → 看停稳后的新一帧 → 还差就再转。节奏越快，
 * 「停稳」被发现得越早、锁得越快。激光只在锁定确认之后连读 3 次（间隔 300 ms，
 * 避开 5 Hz 限频），平时不读。
 *
 * ## 这个循环会阻塞
 *
 * 打点那一拍要连读 3 次激光（每次同步、实测最多 1.2 s）。所以只能跑在主循环里，
 * **绝不能放进 PSDK 回调线程**。
 */
static void WtApp_RunPed(void)
{
    WtPedSession ped;
    WtPedConfig cfg;
    char progress[128];
    char lastPosted[128] = "";
    uint32_t lastPostMs = 0;
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();

    /*
     * 拨上一次就先作废旧基点 —— 这一轮要是失败（或链路没就绪），「绕点飞行」
     * 不能拿上一轮、可能是另一个人的点去飞。成功了会在会话结束处重新写入。
     */
    if (s_havePedPoint) {
        USER_LOG_INFO("行人瞄准：新一轮开始，作废上一轮的基点 %.8f, %.8f",
                      s_pedPoint.lat, s_pedPoint.lon);
        s_havePedPoint = false;
    }

    if (!s_pedReady) {
        WtWidget_PostMessage("行人瞄准链路未就绪（见启动日志）");
        WtWidget_ClearPedRequest();
        return;
    }

    /* 配置是这两个量的唯一来源，经 ConfigFrom 构造，不另传 */
    cfg = WtPedLogic_ConfigFrom(s_runner.config.pedDwellMs, s_runner.config.pedConf);
    if (!WtPed_Begin(&ped, &cfg, s_runner.config.outputDir)) {
        WtWidget_PostMessage("行人瞄准启动失败");
        WtWidget_ClearPedRequest();
        return;
    }

    while (!WtPed_IsFinished(&ped) && !s_stopRequested) {
        uint32_t now = 0;

        if (WtWidget_TakeAbortRequest()) {
            WtPed_Abort(&ped, "操作员按了中止");
            break;
        }
        if (!WtWidget_IsPedRequested()) {
            WtPed_Abort(&ped, "开关已拨回");
            break;
        }

        /* 先刷遥测再 Tick：决策层要用这一拍的云台读回判停稳 */
        WtTelemetry_Poll(&s_runner.telemetry);
        WtPed_Tick(&ped, &s_runner.telemetry);

        /*
         * 浮窗只在文字变了、且距上次 ≥ 500 ms 时推：PostMessage 每次都直接发
         * 并写一行日志，10 Hz 全推既刷屏又挤占 2 KB/s 的浮窗带宽。
         */
        WtPed_FormatProgress(&ped, progress, sizeof(progress));
        if (osal != NULL && osal->GetTimeMs != NULL) {
            osal->GetTimeMs(&now);
        }
        if (strcmp(progress, lastPosted) != 0 && (now - lastPostMs) >= 500u) {
            WtWidget_PostMessage("%s", progress);
            snprintf(lastPosted, sizeof(lastPosted), "%s", progress);
            lastPostMs = now;
        }

        usleep(100 * 1000);
    }

    if (!WtPed_IsFinished(&ped)) {
        WtPed_Abort(&ped, "进程退出");
    }

    /*
     * 归还云台模式 —— **必须在这里还，不能等进程退出**（`dji_app_ctl stop`
     * 直接杀进程，DeInit 不会被执行；FREE 残留会让之后绕飞顶 pan 限位）。
     * 放在成败判断之前：失败与中止路径同样借过模式。
     */
    WtLaser_ReturnFollowMode();

    /* 打到点了：记下来当「绕点飞行」的基点 */
    if (ped.havePoint) {
        s_pedPoint = ped.sample.point;
        s_havePedPoint = true;
        s_pedPointMs = WtApp_NowMs();
        USER_LOG_INFO("绕点飞行的基点已更新：%.8f, %.8f（本次行人打点）",
                      s_pedPoint.lat, s_pedPoint.lon);
    }

    WtPed_FormatProgress(&ped, progress, sizeof(progress));
    WtWidget_PostMessage("%s；要再来一次请把「行人瞄准」拨回再拨上", progress);
    WtWidget_ClearPedRequest();
}
#endif /* WT_WITH_TRT */

/**
 * @brief 待命循环里查一下「行人瞄准」开关，拨上了就跑一次会话。
 *
 * 两个待命循环（开机待命、作业被拒后的待命）都要调它 —— 只在一处响应，
 * 另一处拨开关就会"没有任何反应"，从外面看和卡死一样。
 */
static void WtApp_ServicePedRequest(void)
{
    if (!WtWidget_IsPedRequested()) {
        return;
    }
#if defined(WT_WITH_TRT)
    USER_LOG_INFO("收到「行人瞄准」—— 开始瞄准打点会话");
    WtApp_RunPed();
#else
    WtWidget_PostMessage("行人瞄准不可用：本程序编译时未开 TensorRT 链路（-DWT_BUILD_TRT=ON）");
    WtWidget_ClearPedRequest();
#endif
}

/* 绕点飞行：等飞机进入航线状态的上限，与整趟任务的上限 */
#define WT_SQUARE_START_TIMEOUT_MS   30000u
#define WT_SQUARE_MISSION_TIMEOUT_MS (10u * 60u * 1000u)

/**
 * @brief 跑一次「绕点飞行」：以最近一次行人打点为基点飞正方形。
 *
 * 规划与 KMZ 在 wt_core（wt_ped_square + WtBridge_ExportPedSquareKmz），
 * 这里只做条件检查、上传、监控和中止。
 *
 * ## 起飞前的几道闸
 *
 * - 有最近一轮「行人瞄准」成功打到的点（每拨一次都会先作废旧点，见 s_havePedPoint）
 * - 飞机位置有效、**飞机在空中** —— 在地面按下会直接起飞，离人十来米；
 *   宁可拒绝，让操作员先起飞
 * - 云台没有残留在 FREE 模式（机头要连续转一圈，FREE 会顶 pan 限位）
 * - 基点离飞机不超过上限（基点多半打错了）
 *
 * ## 中止
 *
 * 1 号「中止作业」按钮 → STOP：飞机退出航线、原地悬停，由操作员接管。
 * 不像风机作业那样「暂停 + 返航」：这条航线贴着人、高度只有 10–20 m，
 * 返航要先爬高再飞走，路线不受控；原地悬停最可预期。
 */
static void WtApp_RunSquare(void)
{
    WtPedSquareConfig cfg = WtPedSquare_DefaultConfig();
    WtPedSquarePlan plan;
    E_WtPedSquareResult r;
    WtMissionWatch watch;
    E_WtMissionWatchPhase lastPhase = WT_MW_WAITING;
    int lastIndex = -2;
    const WtAircraftState *ac = &s_runner.telemetry.aircraft;
    const char *dir = (s_runner.config.outputDir[0] != '\0') ? s_runner.config.outputDir : "data";
    char kmzPath[WT_CONFIG_MAX_PATH + 32];
    bool dryRun;
    uint32_t startMs;
    double ageS;
    int i;

    if (!s_havePedPoint) {
        WtWidget_PostMessage("绕点飞行：还没有行人坐标，先用「行人瞄准」打一个点");
        return;
    }

    WtTelemetry_Poll(&s_runner.telemetry);
    if (!ac->validPosition) {
        WtWidget_PostMessage("绕点飞行：飞机位置无效，不启动");
        return;
    }
    if (ac->flightStatus != 2) {
        /* 2 = 空中（DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR）。没收到过该话题时也是 0 */
        WtWidget_PostMessage("绕点飞行：飞机不在空中（状态 %u），不启动；请先起飞",
                             (unsigned)ac->flightStatus);
        return;
    }
    if (WtLaser_IsFreeModeBorrowed()) {
        WtLaser_ReturnFollowMode();
        if (WtLaser_IsFreeModeBorrowed()) {
            WtWidget_PostMessage("绕点飞行：云台还在 FREE 模式，不启动（拨一下云台轮再试）");
            return;
        }
    }

    r = WtPedSquare_Build(&cfg, &s_pedPoint, &ac->position, &plan);
    if (r == WT_PED_SQUARE_ERR_TOO_FAR) {
        WtWidget_PostMessage("绕点飞行：基点离飞机 %.0f m，超过上限 %.0f m（点打错了？），不启动",
                             plan.aircraftToBaseM, cfg.maxBaseDistanceM);
        return;
    }
    if (r != WT_PED_SQUARE_OK) {
        WtWidget_PostMessage("绕点飞行：%s，不启动", WtPedSquare_ResultText(r));
        return;
    }

    ageS = (double)(WtApp_NowMs() - s_pedPointMs) / 1000.0;
    USER_LOG_INFO("绕点飞行：基点 %.8f, %.8f（%.0f s 前打的点）；飞机到基点 %.1f m、到首航点 %.1f m",
                  plan.base.lat, plan.base.lon, ageS, plan.aircraftToBaseM, plan.transitM);
    USER_LOG_INFO("绕点飞行：边长 %.0f m，高度 %.0f/%.0f m（相对起飞点），速度 %.1f m/s，"
                  "航线 %.0f m，至少 %.0f s",
                  cfg.sideM, cfg.lowHeightM, cfg.highHeightM, cfg.speedMs, plan.pathLengthM,
                  plan.durationS);
    for (i = 0; i < plan.count; i++) {
        const WtPedSquareWaypoint *w = &plan.wp[i];

        USER_LOG_INFO("  航点 %2d %s V%d  %.8f, %.8f  %.1f m  朝基点 %.1f°", i,
                      WtPedSquare_StepName(w->step), w->corner + 1, w->geo.lat, w->geo.lon,
                      w->relHeightM, w->headingToBaseDeg);
    }

    (void)WtRunner_MakeDirs(dir);
    snprintf(kmzPath, sizeof(kmzPath), "%s/ped_square.kmz", dir);
    if (WtBridge_ExportPedSquareKmz(&plan, &cfg, kmzPath) != WT_PLAN_OK) {
        WtWidget_PostMessage("绕点飞行：导出 KMZ 失败（%s）", kmzPath);
        return;
    }

    /* 干跑在上传前一刻实时读控件 —— 与风机作业同一个理由（见 WtRunner_ExecuteTurbine） */
    dryRun = WtWidget_IsDryRun() || s_runner.config.dryRun;
    if (!WtSquareMission_Begin(kmzPath, !dryRun)) {
        WtWidget_PostMessage("绕点飞行：%s失败，见日志", dryRun ? "上传" : "上传或启动");
        return;
    }
    /*
     * 浮窗带上「基点离飞机多远、是多久前打的」：现场一眼就能核对绕的是不是
     * 刚才那个人 —— 远得离谱或时间对不上，立刻按「中止作业」。
     */
    if (dryRun) {
        WtWidget_PostMessage("干跑：绕点航线已上传，未启动；基点离飞机 %.0f m（%.0f s 前打的点）",
                             plan.aircraftToBaseM, ageS);
        WtSquareMission_End();
        return;
    }

    WtWidget_PostMessage("绕点飞行已启动：基点离飞机 %.0f m（%.0f s 前打的点），先飞往 V1；"
                         "按「中止作业」原地悬停",
                         plan.aircraftToBaseM, ageS);
    startMs = WtApp_NowMs();
    WtMissionWatch_Begin(&watch, startMs, WT_SQUARE_START_TIMEOUT_MS);

    while (!s_stopRequested) {
        const uint32_t now = WtApp_NowMs();
        E_WtMissionWatchPhase phase;

        if (WtWidget_TakeAbortRequest()) {
            (void)WtSquareMission_Stop();
            WtWidget_PostMessage("绕点飞行已中止：飞机原地悬停，请接管");
            break;
        }

        WtSquareMission_Poll(&watch, now);
        phase = WtMissionWatch_Tick(&watch, now);
        if (phase != lastPhase || watch.waypointIndex != lastIndex) {
            if (phase == WT_MW_RUNNING && watch.waypointIndex >= 0) {
                WtWidget_PostMessage("绕点飞行：航点 %d / %d", watch.waypointIndex, plan.count);
            } else {
                WtWidget_PostMessage("绕点飞行：%s", WtMissionWatch_PhaseName(phase));
            }
            lastPhase = phase;
            lastIndex = watch.waypointIndex;
        }

        if (phase == WT_MW_FINISHED) {
            WtWidget_PostMessage("绕点飞行结束（最后报告航点 %d / %d），飞机悬停待接管",
                                 watch.maxWaypointIndex, plan.count);
            break;
        }
        if (phase == WT_MW_NOT_STARTED) {
            (void)WtSquareMission_Stop();
            WtWidget_PostMessage("绕点飞行：%u s 内飞机没有进入航线，已放弃",
                                 WT_SQUARE_START_TIMEOUT_MS / 1000u);
            break;
        }
        if ((uint32_t)(now - startMs) > WT_SQUARE_MISSION_TIMEOUT_MS) {
            (void)WtSquareMission_Stop();
            WtWidget_PostMessage("绕点飞行超时（%u 分钟），已 STOP，飞机悬停",
                                 WT_SQUARE_MISSION_TIMEOUT_MS / 60000u);
            break;
        }

        usleep(100 * 1000);
    }

    /* 进程要退出时任务还没完：停下来，别留一条没人监控的航线 */
    if (s_stopRequested && !WtMissionWatch_IsOver(&watch)) {
        (void)WtSquareMission_Stop();
    }
    WtSquareMission_End();
}

/** @brief 待命循环里查一下「绕点飞行」按钮，按了就跑一次 */
static void WtApp_ServiceSquareRequest(void)
{
    if (!WtWidget_TakeSquareRequest()) {
        return;
    }
    USER_LOG_INFO("收到「绕点飞行」—— 检查条件并规划航线");
    WtApp_RunSquare();
}

/** 停掉推理链路（先停推理再停取流 —— 反过来推理线程会读到已释放的帧缓冲） */
static void WtApp_StopVisionChain(void)
{
#if defined(WT_WITH_TRT)
    if (s_surveyReady || s_pedReady) {
        WtVision_Stop();
        WtFramePump_Stop();
        /* 兜底归还云台模式；正常路径上会话结束时已经还过 */
        WtLaser_DeInit();
        s_surveyReady = false;
        s_pedReady = false;
    }
#endif
}

static T_DjiReturnCode WtApp_PrintConsole(const uint8_t *data, uint16_t dataLen)
{
    printf("%.*s", dataLen, (char *)data);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode WtApp_LocalWrite(const uint8_t *data, uint16_t dataLen)
{
    if (s_logFile == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    if (fwrite(data, 1, dataLen, s_logFile) != dataLen) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    fflush(s_logFile);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode WtApp_LocalWriteFsInit(const char *path)
{
    char filePath[WT_LOG_PATH_MAX_SIZE];
    char directory[WT_LOG_PATH_MAX_SIZE];
    char *p;

    snprintf(directory, sizeof(directory), "%.*s", (int)sizeof(directory) - 1, path);

    /* 逐级创建日志目录，避免日志系统因目录缺失而静默失效 */
    for (p = directory + 1; *p != '\0'; p++) {
        if (*p == '/') {
            *p = '\0';
            mkdir(directory, 0755);
            *p = '/';
        }
    }
    mkdir(directory, 0755);

    snprintf(filePath, sizeof(filePath), "%s/log_%d.txt", path, getpid());
    s_logFile = fopen(filePath, "w");
    if (s_logFile == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode WtApp_PrepareSystemEnvironment(void)
{
    T_DjiReturnCode rc;

    T_DjiOsalHandler osalHandler = {
        .TaskCreate = Osal_TaskCreate,
        .TaskDestroy = Osal_TaskDestroy,
        .TaskSleepMs = Osal_TaskSleepMs,
        .MutexCreate = Osal_MutexCreate,
        .MutexDestroy = Osal_MutexDestroy,
        .MutexLock = Osal_MutexLock,
        .MutexUnlock = Osal_MutexUnlock,
        .SemaphoreCreate = Osal_SemaphoreCreate,
        .SemaphoreDestroy = Osal_SemaphoreDestroy,
        .SemaphoreWait = Osal_SemaphoreWait,
        .SemaphoreTimedWait = Osal_SemaphoreTimedWait,
        .SemaphorePost = Osal_SemaphorePost,
        .Malloc = Osal_Malloc,
        .Free = Osal_Free,
        .GetRandomNum = Osal_GetRandomNum,
        .GetTimeMs = Osal_GetTimeMs,
        .GetTimeUs = Osal_GetTimeUs,
    };

    T_DjiHalUsbBulkHandler usbBulkHandler = {
        .UsbBulkInit = HalUsbBulk_Init,
        .UsbBulkDeInit = HalUsbBulk_DeInit,
        .UsbBulkWriteData = HalUsbBulk_WriteData,
        .UsbBulkReadData = HalUsbBulk_ReadData,
        .UsbBulkGetDeviceInfo = HalUsbBulk_GetDeviceInfo,
    };

    T_DjiFileSystemHandler fileSystemHandler = {
        .FileOpen = Osal_FileOpen,
        .FileClose = Osal_FileClose,
        .FileWrite = Osal_FileWrite,
        .FileRead = Osal_FileRead,
        .FileSync = Osal_FileSync,
        .FileSeek = Osal_FileSeek,
        .DirOpen = Osal_DirOpen,
        .DirClose = Osal_DirClose,
        .DirRead = Osal_DirRead,
        .Mkdir = Osal_Mkdir,
        .Unlink = Osal_Unlink,
        .Rename = Osal_Rename,
        .Stat = Osal_Stat,
    };

    T_DjiSocketHandler socketHandler = {
        .Socket = Osal_Socket,
        .Bind = Osal_Bind,
        .Close = Osal_Close,
        .UdpSendData = Osal_UdpSendData,
        .UdpRecvData = Osal_UdpRecvData,
        .TcpListen = Osal_TcpListen,
        .TcpAccept = Osal_TcpAccept,
        .TcpConnect = Osal_TcpConnect,
        .TcpSendData = Osal_TcpSendData,
        .TcpRecvData = Osal_TcpRecvData,
    };

    T_DjiLoggerConsole printConsole = {
        .func = WtApp_PrintConsole,
        .consoleLevel = DJI_LOGGER_CONSOLE_LOG_LEVEL_INFO,
        .isSupportColor = true,
    };

    T_DjiLoggerConsole fileConsole = {
        .func = WtApp_LocalWrite,
        .consoleLevel = DJI_LOGGER_CONSOLE_LOG_LEVEL_DEBUG,
        .isSupportColor = false,
    };

    rc = DjiPlatform_RegOsalHandler(&osalHandler);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("注册 OSAL 处理器失败\n");
        return rc;
    }

    if (WtApp_LocalWriteFsInit(WT_LOG_PATH) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("日志目录初始化失败\n");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    DjiLogger_AddConsole(&printConsole);
    DjiLogger_AddConsole(&fileConsole);

    rc = DjiPlatform_RegHalUsbBulkHandler(&usbBulkHandler);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("注册 USB Bulk 处理器失败\n");
        return rc;
    }

    /* 低速数据通道（接收地面站下发的停用角、启停指令）依赖 socket */
    rc = DjiPlatform_RegSocketHandler(&socketHandler);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("注册 Socket 处理器失败\n");
        return rc;
    }

    rc = DjiPlatform_RegFileSystemHandler(&fileSystemHandler);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("注册文件系统处理器失败\n");
        return rc;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode WtApp_FillUserInfo(T_DjiUserInfo *userInfo)
{
    if (userInfo == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    memset(userInfo, 0, sizeof(*userInfo));

    if (strlen(USER_APP_NAME) >= sizeof(userInfo->appName) ||
        strlen(USER_APP_ID) >= sizeof(userInfo->appId) ||
        strlen(USER_APP_KEY) >= sizeof(userInfo->appKey) ||
        strlen(USER_APP_LICENSE) >= sizeof(userInfo->appLicense) ||
        strlen(USER_DEVELOPER_ACCOUNT) >= sizeof(userInfo->developerAccount)) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    /* 未填写真实信息时直接拒绝启动：带占位符去连飞机只会得到一个含糊的失败 */
    if (strcmp(USER_APP_ID, "your_app_id") == 0) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    /*
     * 用 snprintf 而不是 strncpy：这里拷的都是编译期常量，长度已经在上面的
     * 分支里卡过一遍，但 strncpy 在「源串长度恰好等于 n-1」时不会补结尾的
     * '\0' —— 正确性要靠前面那句 memset 兜着。换成 snprintf 后这个不变量
     * 由函数本身保证，不再依赖调用顺序。
     */
    snprintf(userInfo->appName, sizeof(userInfo->appName), "%s", USER_APP_NAME);
    snprintf(userInfo->appId, sizeof(userInfo->appId), "%s", USER_APP_ID);
    snprintf(userInfo->appKey, sizeof(userInfo->appKey), "%s", USER_APP_KEY);
    snprintf(userInfo->appLicense, sizeof(userInfo->appLicense), "%s", USER_APP_LICENSE);
    snprintf(userInfo->developerAccount, sizeof(userInfo->developerAccount), "%s",
             USER_DEVELOPER_ACCOUNT);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static void WtApp_SignalHandler(int signalNum)
{
    (void)signalNum;

    /*
     * 信号处理函数里只允许做异步信号安全的操作：置位。
     * 返航、暂停航线这些调用都放到主循环里执行，否则可能死锁。
     */
    s_stopRequested = 1;
    WtRunner_RequestAbort(&s_runner);
}

int main(int argc, char **argv)
{
    const char *configPath = (argc > 1) ? argv[1] : WT_DEFAULT_CONFIG_PATH;
    T_DjiUserInfo userInfo;
    T_DjiAircraftInfoBaseInfo baseInfo;
    T_DjiFirmwareVersion firmwareVersion = {1, 0, 0, 0};
    T_DjiReturnCode rc;
    int exitCode = 0;

    signal(SIGINT, WtApp_SignalHandler);
    signal(SIGTERM, WtApp_SignalHandler);

    printf("==================================================\n");
    printf("  风机叶片巡检机载应用   M4T + 妙算3\n");
    printf("  作业模式：粗模环绕 -> 精细巡检（两阶段）\n");
    printf("==================================================\n");

    /* 步骤 1：平台环境 */
    if (WtApp_PrepareSystemEnvironment() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return 1;
    }

    /* 步骤 2：应用信息 */
    if (WtApp_FillUserInfo(&userInfo) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("应用信息无效，请填写 dji_sdk_app_info.h");
        return 1;
    }

    /* 步骤 3：PSDK 核心 */
    rc = DjiCore_Init(&userInfo);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiCore_Init 失败: 0x%08X", rc);
        return 1;
    }

    /*
     * 机型自检放在最前面：本方案依赖 M4T 系列的内置三摄与 E-Port 载荷。
     * 挂错机型却一路初始化到航点下发才报错，是最浪费时间的一种失败。
     */
    rc = DjiAircraftInfo_GetBaseInfo(&baseInfo);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("获取机型信息失败，无法确认硬件是否匹配");
        DjiCore_DeInit();
        return 1;
    }
    if (baseInfo.aircraftType != DJI_AIRCRAFT_TYPE_M4T &&
        baseInfo.aircraftType != DJI_AIRCRAFT_TYPE_M4E &&
        baseInfo.aircraftType != DJI_AIRCRAFT_TYPE_M4TD) {
        USER_LOG_ERROR("当前机型 %d 不在支持范围（M4T / M4E / M4TD）",
                       (int)baseInfo.aircraftType);
        DjiCore_DeInit();
        return 1;
    }
    USER_LOG_INFO("机型检查通过，aircraftType = %d", (int)baseInfo.aircraftType);

    DjiCore_SetAlias("WT_BLADE_INSPECT");
    DjiCore_SetFirmwareVersion(firmwareVersion);
    DjiCore_SetSerialNumber("WTINSPECT000001");

    /*
     * Pilot 2 控件：操作员在遥控器上控制作业，不必 SSH 改配置。
     *
     * ⚠️ **必须在这里注册** —— 早于 `DjiCore_ApplicationStart()`，
     * 否则 Pilot 拉不到控件列表。也早于配置载入，所以停用角的初值
     * 要等 `WtRunner_Init` 之后再用 `WtWidget_SetParkPhaseInitial` 补。
     *
     * 注册失败**不阻断启动**：控件是操作便利性，不是作业前提。
     * 装包时安装器要试运行应用，这里 fail-closed 会让包装不上 ——
     * 那正是"安装器报 user_app_id 错误"这类误导性失败的来源。
     */
    if (WtWidget_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_WARN("控件初始化失败，作业仍可通过配置文件进行（无 Pilot 控件）");
    }

    rc = DjiCore_ApplicationStart();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiCore_ApplicationStart 失败: 0x%08X", rc);
        DjiCore_DeInit();
        return 1;
    }

    /*
     * 步骤 4：作业配置。
     *
     * ⚠️ 这一段**不能退出进程**，哪怕配置缺失或非法。
     *
     * `dji_app_ctl install` 会在安装过程中试运行应用，并要求它走完 SDK 身份
     * 校验。若应用在 DjiCore_ApplicationStart 之后、校验完成之前退出，安装
     * 就判失败，而报的是一句误导性的 `verify app user_app_id or version info
     * error` —— 真正的死因（"进程没了"）安装器看不到。
     *
     * 所以 fail-closed 的边界划在「作业开始」，不是「进程启动」：
     * 这里只把问题记下来，让 SDK 起来、把校验走完，等到真正要执行作业时
     * 再拒绝（见下方 s_configReady 的用法）。
     */
    {
        FILE *probe = fopen(configPath, "r");

        if (probe == NULL) {
            char dir[WT_CONFIG_MAX_PATH];

            if (WtRunner_DirName(configPath, dir, sizeof(dir))) {
                WtRunner_MakeDirs(dir);
            }

            if (WtAppConfig_WriteTemplate(configPath)) {
                USER_LOG_WARN("配置文件 %s 不存在，已生成模板。"
                              "填写后重启即可作业；本次不执行飞行。", configPath);
            } else {
                USER_LOG_ERROR("配置文件 %s 不存在，且模板生成失败（目录不可写？）",
                               configPath);
            }
            s_configReady = false;
        } else {
            fclose(probe);

            if (WtRunner_Init(&s_runner, configPath) != WT_RUN_OK) {
                USER_LOG_ERROR("运行器初始化失败，本次不执行飞行（配置问题，非启动失败）");
                s_configReady = false;
            } else {
                /*
                 * 相机/云台模块必须**先于遥测订阅**初始化。
                 *
                 * 2026-10-22 上机实测的时序：
                 *   00:16:37.4  WtTelemetry_Init  订阅 GIMBAL_ANGLES → 0xE0
                 *   00:16:42.5  WtLaser_Init      DjiGimbalManager_Init()
                 * 订阅比模块初始化早了 5 秒，于是拿到 `0xE0 NONSUPPORT`。
                 *
                 * **0xE0 在这里是"模块未初始化"的伪装**，不是"机型不支持" ——
                 * 这与 wt_ai_stream_probe.c 里记的 liveview 教训**同一形状**：
                 * 漏掉 DjiLiveview_Init() 时 RegUserAiTargetLableList 也返回
                 * 0xE0，日志还打"only support on manifold3"，看起来像硬件不支持。
                 *
                 * 这也可能解释 HANDOFF 里「TOPIC_GIMBAL_STATUS 在 M4T 上订不上」
                 * 那条 —— 很可能是同一个时序问题，而非机型限制。
                 *
                 * 失败不阻断：遥测订阅那边会再次告警，只是云台角/俯仰闭环不可用。
                 */
                if (s_runner.config.surveyEnabled || s_runner.config.pedEnabled) {
                    (void)WtLaser_Init();
                }

                if (!WtTelemetry_Init(&s_runner.telemetry,
                                      s_runner.config.turbineCount > 0
                                          ? &s_runner.config.turbines[0].spec
                                          : NULL)) {
                    USER_LOG_ERROR("遥测初始化失败，本次不执行飞行");
                    WtRunner_DeInit(&s_runner);
                    s_configReady = false;
                } else {
                    s_configReady = true;

                    /*
                     * 把配置文件里的值灌进控件作初值。
                     *
                     * 不做这一步的话，操作员打开 Payload Settings 看到停用角是 0，
                     * 而程序实际用配置里的 45 —— 两边对不上，且他一旦点一下输入框
                     * 就会把 45 覆盖成 0。控件初值必须与配置一致。
                     */
                    if (s_runner.config.turbineCount > 0) {
                        WtWidget_SetParkPhaseInitial(
                            s_runner.config.turbines[0].parkPhaseDeg);
                    }
                    WtWidget_SetDryRunInitial(s_runner.config.dryRun);
                }
            }

            /*
             * AI 叠加验证（诊断用，默认关）。
             *
             * 放在配置载入之后 —— 它要读 config.aiOverlay。也放在
             * ApplicationStart 之后 —— liveview 模块依赖 SDK 已起来。
             * 失败不阻断：这是验证功能，不是作业前提。
             */
            if (WtAiOverlay_Start(s_runner.config.aiOverlay)) {
                USER_LOG_INFO("AI 叠加验证已启动，请在 Pilot 2 画面上观察框与标签");
            }

            /*
             * 取流探针（诊断用，默认关）：验证能否拿到相机实时画面。
             * 与 AI 叠加独立 —— 两者可分别开启，用于区分"通道不通"与
             * "Pilot 2 渲染条件不满足"。
             */
            if (WtAiStreamProbe_Start(s_runner.config.streamProbe)) {
                USER_LOG_INFO("取流探针已启动，等待相机帧…");
            }

#if defined(WT_WITH_TRT)
            /*
             * 激光采集链路（默认关）。
             *
             * ⚠️ 与取流探针**互斥**：两者都 DjiLiveview_Init() 并启动同一个
             * 相机源的图像流，同时开会互相抢。这里显式拦住，而不是让它们
             * 在设备上打成一团 —— 那种现象（帧忽有忽无）极难定位。
             */
            if (s_runner.config.surveyEnabled && s_runner.config.streamProbe) {
                USER_LOG_ERROR("survey_enabled 与 stream_probe 互斥（同一相机源），"
                               "本次不启动采集链路");
            } else if (s_runner.config.surveyEnabled) {
                /*
                 * 失败不阻断启动 —— 与控件、AI 叠加同一原则：装包时安装器
                 * 要试运行应用，这里 fail-closed 会让包装不上。
                 */
                if (!WtFramePump_Start()) {
                    USER_LOG_ERROR("采集：取流启动失败，本次无法采集盘面");
                } else if (!WtVision_Start(s_runner.config.surveyEnginePath,
                                           WT_VISION_MODEL_TURBINE, 0.25f, 0.45f,
                                           s_runner.config.surveyInferIntervalMs > 0
                                               ? (uint32_t)s_runner.config.surveyInferIntervalMs
                                               : 200u)) {
                    USER_LOG_ERROR("采集：推理线程启动失败（engine 路径？架构不匹配？），"
                                   "本次无法采集盘面");
                    WtFramePump_Stop();
                } else if (!WtLaser_Init()) {
                    USER_LOG_ERROR("采集：相机/云台模块初始化失败，本次无法采集盘面");
                    WtVision_Stop();
                    WtFramePump_Stop();
                } else {
                    s_surveyReady = true;
                    USER_LOG_INFO("采集链路已就绪：拨「开始作业」时会先采集盘面"
                                  "（需飞机在空中、画面里有风机）");

                    /*
                     * 自检放在这里（链路就绪之后、待命之前）：它要用云台与
                     * 激光，而那两个模块刚初始化完。放到待命循环里反而会和
                     * 「开始作业」的触发抢时序。
                     */
                    if (s_runner.config.surveySelfTest) {
                        WtApp_RunGimbalLaserSelfTest();
                    }
                }
            }

            /*
             * 高空行人瞄准打点（默认关）。
             *
             * ⚠️ 与激光采集、取流探针**互斥**：三者都要独占同一个相机源的图像流，
             * 推理线程也只能装一个模型。显式拦住，而不是让它们在设备上抢成一团。
             *
             * 推理间隔 100 ms：瞄准闭环要等「停稳之后的新一帧」，帧来得越勤
             * 锁得越快；1280 输入单帧推理约几十毫秒，10 Hz 留有余量。
             * 失败不阻断启动（装包时安装器要试运行应用）。
             */
            if (s_runner.config.pedEnabled) {
                if (s_runner.config.surveyEnabled || s_runner.config.streamProbe) {
                    USER_LOG_ERROR("ped_enabled 与 survey_enabled / stream_probe 互斥"
                                   "（同一相机源、同一推理线程），本次不启动行人瞄准");
                } else if (!WtFramePump_Start()) {
                    USER_LOG_ERROR("行人瞄准：取流启动失败");
                } else if (!WtVision_Start(s_runner.config.pedEnginePath,
                                           WT_VISION_MODEL_PEDESTRIAN, 0.25f, 0.45f, 100u)) {
                    USER_LOG_ERROR("行人瞄准：推理线程启动失败（ped_engine 路径？"
                                   "装错 engine？架构不匹配？）");
                    WtFramePump_Stop();
                } else if (!WtLaser_Init()) {
                    USER_LOG_ERROR("行人瞄准：相机/云台模块初始化失败");
                    WtVision_Stop();
                    WtFramePump_Stop();
                } else {
                    s_pedReady = true;
                    USER_LOG_INFO("行人瞄准链路已就绪：在 Pilot 2 拨「行人瞄准」开关即可开始"
                                  "（锁定后连续 3 帧确认、再停 %.1f s 自动打点）",
                                  (double)s_runner.config.pedDwellMs / 1000.0);
                }
            }
#else
            if (s_runner.config.pedEnabled) {
                USER_LOG_ERROR("ped_enabled = true，但本程序编译时未开 TensorRT 链路"
                               "（-DWT_BUILD_TRT=ON），行人瞄准不可用");
            }
#endif /* WT_WITH_TRT */

            /*
             * geoid 偏移注入。不填时只有融合定位可用的场景下 ENU 天向分量
             * 会被标记为不可信（水平仍可用），见 WtTelemetry_RefreshEnu。
             */
            if (s_runner.config.geoidOffsetM != 0.0) {
                WtTelemetry_SetGeoidOffset(&s_runner.telemetry,
                                           s_runner.config.geoidOffsetM);
                USER_LOG_INFO("已注入 geoid 偏移 %.1f m（MSL − 椭球高）",
                              s_runner.config.geoidOffsetM);
            }
        }
    }

    /*
     * 配置就绪才作业；否则保持进程存活，等安装器（或操作员）决定何时退出。
     * 这里的等待是**有意**的：进程活着 = SDK 身份校验有机会走完 = 装得上包。
     */
    if (!s_configReady) {
        USER_LOG_WARN("配置未就绪，应用保持运行但不执行飞行。"
                      "请填好配置后重启应用。");

        while (!s_stopRequested) {
            sleep(1);
        }

        DjiCore_DeInit();
        if (s_logFile != NULL) {
            fclose(s_logFile);
        }
        printf("[app] 配置未就绪，退出，返回码 0\n");

        return 0;
    }

    /*
     * 步骤 5：作业闸门。
     *
     * 两条启动路径，**都通**：
     *   ① 配置文件 `auto_start = true`  → 开机即作业（老路径，保留兼容）
     *   ② Pilot 2 控件「开始作业」拨到 ON → 操作员随时启动（新路径）
     *
     * 默认都关着。为什么：安装器会试运行应用并要求它走完身份校验。
     * 若默认就开始作业，进程会在上传航线后长时间等待飞控状态（飞机不在
     * 空中时这个等待不会结束），安装器超时 → 装不上包。而且「装上就自动
     * 起飞」本身也是危险默认。
     */
    if (!s_runner.config.autoStart) {
        USER_LOG_WARN("auto_start = false：配置已就绪，**不自动开始作业**。");
        USER_LOG_WARN("  应用保持运行并已走完 SDK 身份校验，等待指令。");
        USER_LOG_WARN("  在 Pilot 2 的 PSDK 菜单里把「开始作业」拨到 ON 即可开始；");
        USER_LOG_WARN("  停用角在 Payload Settings 里填，飞行中也可改（影响下一台）。");

        /* 待命循环：等控件请求、或进程被要求退出。 */
        {
            /*
             * ⚠️ 这里**不调 `WtTelemetry_Poll()`**。
             *
             * 2026-10-20 实测：在待命循环里每秒调一次 Poll，进程进入循环后
             * 3 秒内稳定 SEGV/ABRT（`journalctl` 里 `code=dumped, status=11`）。
             * 同一函数在执行循环里调用是安全的 —— 差别在于待命阶段飞机在地面，
             * 部分话题订阅本就失败（`subscribe topic 11/26 failed`），
             * 且遥测尚未经历稳定期。
             *
             * 断连检测因此不用遥测做判据：待命阶段本来就没有"必须收到位置"
             * 的业务需求，用它会引入一个新的崩溃面。真要判链路存活，
             * 交给 SDK 自己的错误日志与 systemd 的进程监管即可。
             */
            while (!s_stopRequested) {
                if (WtWidget_TakeAbortRequest()) {
                    USER_LOG_WARN("待命状态下收到中止请求，已清除开始请求");
                    WtWidget_ClearStartRequest();
                }
                WtApp_ServicePedRequest();
                WtApp_ServiceSquareRequest();
                if (WtWidget_IsStartRequested()) {
                    /*
                     * 采集先于规划。
                     *
                     * 顺序是刻意的：采集产出的**停用角**正是
                     * `WtRunner_PlanTurbine` 在未知时无条件拒绝规划的那个
                     * 前提。先采后规划，那条拒绝路径才走得通；反过来则规划
                     * 必然被拒，采集也就白做了。
                     *
                     * 只在第一次「开始作业」时采。重复采没有意义（叶片没动），
                     * 而且每次都采会让"改参数重试"变得很慢。
                     */
#if defined(WT_WITH_TRT)
                    if (s_surveyReady && !s_surveyDone) {
                        USER_LOG_INFO("收到「开始作业」—— 先采集盘面几何");
                        WtApp_RunSurvey();
                        s_surveyDone = true;
                    }
#endif
                    USER_LOG_INFO("收到控件「开始作业」请求，进入作业流程");
                    break;
                }
                sleep(1);
            }
        }

        /* 若不是因为"开始作业"而退出循环，走收尾路径。 */
        if (s_stopRequested || !WtWidget_IsStartRequested()) {
            WtTelemetry_DeInit(&s_runner.telemetry);
            WtApp_StopVisionChain();
            WtWidget_Stop();
            WtRunner_DeInit(&s_runner);
            DjiCore_DeInit();
            if (s_logFile != NULL) {
                fclose(s_logFile);
            }
            printf("[app] 待命状态退出，返回码 0\n");

            return 0;
        }
    }

    /* 步骤 6：逐台风机作业 */
    {
        int i;

        for (i = 0; i < s_runner.config.turbineCount && !s_stopRequested; i++) {
            const char *name = s_runner.config.turbines[i].name;
            WtRunResult rr;

            /*
             * 停用角与干跑开关都在 `WtRunner_RunTurbine` 内部、**规划前一刻**
             * 才取，不在这里提前快照。
             *
             * 2026-10-20 实测的时序缺口：原先在这里取一次，而操作员是在
             * 作业已经开始之后才填停用角的 —— 那次输入被完全忽略。
             * 取值越靠近使用点，操作员的有效窗口越大。
             */
            USER_LOG_INFO("---- 开始作业：%s ----", name);
            rr = WtRunner_RunTurbine(&s_runner, name);

            if (rr != WT_RUN_OK) {
                USER_LOG_ERROR("风机 %s 作业未完成，结果码 %d", name, (int)rr);
                exitCode = (int)rr;

                /*
                 * 被拒绝（尤其是"停用角未知"这类**操作员当场就能修好**的
                 * 原因）不该让整个应用退出。
                 *
                 * 2026-10-20 实测：操作员拨了「开始作业」→ 因停用角未设被
                 * 拒绝 → 应用退出重启 → Payload Settings 界面消失，
                 * 操作员**没有机会把角度填进去**，陷入死循环。
                 *
                 * 现在的行为：回到待命，清掉开始请求，让操作员改完再拨。
                 * 只有执行阶段的失败（已上传、已在飞）才真的退出 ——
                 * 那种情况需要人工介入，继续待命没有意义。
                 */
                if (rr == WT_RUN_ERR_SAFETY || rr == WT_RUN_ERR_CONFIG) {
                    USER_LOG_WARN("作业被拒绝（可在界面上修正后重试）："
                                  "清空「开始作业」开关，请修正参数后再拨一次。");
                    WtWidget_ClearStartRequest();
                    exitCode = 0;
                    break;   /* 跳出风机循环，走下面的待命等待 */
                }
                break; /* 执行阶段出错：交给人工介入，退出 */
            }
        }
    }

    /*
     * 被拒绝后回到待命，而不是退出。
     *
     * 这样操作员能在 Pilot 2 里改停用角/干跑、拨回开关、再拨一次 ——
     * 不用重启应用、不用重新拉控件列表。
     *
     * ⚠️ 这个循环**必须响应「开始作业」**，不能只等退出信号。
     * 2026-10-20 实测的缺陷：初版这里只有 `sleep(1)`，操作员拨了开关后
     * 日志停在「收到开始请求」就再无下文，进程活着但毫无反应 ——
     * 从外面看和"卡死"完全一样。回到待命的意义就是"能再试一次"，
     * 不会响应请求等于没回。
     */
    while (exitCode == 0 && !s_stopRequested) {
        USER_LOG_WARN("作业未执行，应用回到待命状态。修正参数后可再次拨「开始作业」。");

        /* 等一次新请求。先清掉上一轮残留，否则会立刻重入。 */
        WtWidget_ClearStartRequest();

        while (!s_stopRequested) {
            if (WtWidget_TakeAbortRequest()) {
                USER_LOG_WARN("待命状态下收到中止请求");
                WtWidget_ClearStartRequest();
            }
            WtApp_ServicePedRequest();
            WtApp_ServiceSquareRequest();
            if (WtWidget_IsStartRequested()) {
                break;
            }
            sleep(1);
        }

        if (s_stopRequested) {
            break;
        }

        /* 再跑一轮作业。失败会重新落到这里，形成"改参数 → 重试"的循环。 */
        USER_LOG_INFO("收到控件「开始作业」请求，重新进入作业流程");
        exitCode = 0;
        {
            int i;

            for (i = 0; i < s_runner.config.turbineCount && !s_stopRequested; i++) {
                const char *name = s_runner.config.turbines[i].name;
                WtRunResult rr;

                USER_LOG_INFO("---- 开始作业：%s ----", name);
                rr = WtRunner_RunTurbine(&s_runner, name);

                if (rr != WT_RUN_OK) {
                    USER_LOG_ERROR("风机 %s 作业未完成，结果码 %d", name, (int)rr);
                    if (rr == WT_RUN_ERR_SAFETY || rr == WT_RUN_ERR_CONFIG) {
                        WtWidget_ClearStartRequest();
                        break;   /* 可修正：回到外层待命 */
                    }
                    exitCode = (int)rr;
                    break;       /* 执行阶段出错：退出 */
                }
            }
        }
    }

    WtTelemetry_DeInit(&s_runner.telemetry);
    /*
     * 采集 / 行人瞄准链路的收尾：先停推理（它在消费帧），再停取流，最后收云台。
     * WtLaser_DeInit() 还会兜底把云台模式还成 YAW_FOLLOW —— FREE 残留在
     * 飞机上会让后续绕飞顶 pan 限位（姊妹项目踩过的事故）。
     */
    WtApp_StopVisionChain();
    WtAiStreamProbe_Stop();
    WtAiOverlay_Stop();
    WtWidget_Stop();
    WtRunner_DeInit(&s_runner);
    DjiCore_DeInit();

    if (s_logFile != NULL) {
        fclose(s_logFile);
    }

    printf("[app] 退出，返回码 %d\n", exitCode);

    return exitCode;
}