/**
 * @file lz_mission.c
 * @brief 绕飞作业的执行编排。
 *
 * ## 为什么要有状态机，而不是"回调里直接开飞"
 *
 * 上传 KMZ、启动航点、处理结束事件，这三件事发生在**不同线程**上：
 *   - 控件回调在 PSDK 工作线程
 *   - 航点状态回调在 PSDK 的工作线程
 *   - 主循环在 main 线程
 * 用一个显式状态机在单一位置（LzMission_Tick）串起来，
 * 比让三个线程各自改共享变量可靠得多。
 *
 * ## 与 lz_core 的分工
 *
 * lz_core 负责**算**（航线、KMZ），本模块负责**做**（上传、启动、收尾）。
 * 算的部分能在桌面上测，做的部分只能上机 —— 所以边界划得越干净越好。
 */

#include "lz_mission.h"

#include <dji_gimbal_manager.h>
#include <dji_logger.h>
#include <dji_platform.h>
#include <dji_waypoint_v3.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "lz_bridge.h"
#include "lz_bridge_psdk.h"
#include "lz_geo.h"
#include "lz_gimbal_status.h"   /* LZ_GIMBAL_STATUS_BUF —— 缓冲区尺寸的唯一真值 */
#include "lz_plan.h"
#include "lz_align.h"
#include "lz_pole_source.h"
#include "lz_visual_align.h"
#include "lz_widget.h"

typedef enum {
    LZ_MISSION_STATE_IDLE = 0,
    LZ_MISSION_STATE_RUNNING,
} LzMissionState;

static LzMissionState s_state = LZ_MISSION_STATE_IDLE;
static volatile bool s_missionEnded = false;
static char s_endReason[64];

/* 停止被拒的告警只报一次的闸。
 *
 * 为什么需要：`LzMission_Tick()` 由主循环每 100 ms 调一次，而拨 OFF 后
 * 开关会一直是 OFF —— 不加闸的话，STOP 每被拒一次就发一条浮窗，
 * 每秒 10 条。那既刷屏，也可能把 SDK 的浮窗带宽（2 KB/s）吃光，
 * 反而盖掉别的消息。
 *
 * 复位时机：只在"操作员重新拨 ON 又拨 OFF"后才复位。
 * 不复位的话，第二次失败会被当成重复而静默 ——
 * 而重复失败恰恰说明重试也没用，更需要报。 */
static bool s_stopRejectReported = false;
/** "绕飞因照准在跑而推迟"的浮窗闸 —— 理由同 s_stopRejectReported：
 *  Tick 是 100 ms 一拍，不加闸会每秒刷 10 条。 */
static bool s_orbitDeferredReported = false;

/**
 * 上一次报过的云台状态串。空串 = 还没报过。
 *
 * ## 为什么要有它（2026-10-01 上机实测）
 *
 * 现场报的是「无人机在绕飞的时候云台显示**偏航角达到限位**，然后显示
 * **云台电机异常**，结束绕飞航线后又正常了」。
 *
 * 那两件事飞机一直在报（`TOPIC_GIMBAL_STATUS` 的 `yawLimited` /
 * `escYawStatus` 位），但**我们的日志里此前一个字都没有** —— 判据在飞机上、
 * 而我们没订阅它，排查只能靠操作员口述现象，而"偏航顶限位"与"电机异常"
 * 是两个不同的位，口述里分不开。
 *
 * ⇒ 每拍查一次（`LzBridge_GimbalStatusStr`），**只在内容变了时报**：
 * 浮窗带宽 2 KB/s，每拍一条会把它灌满（本项目已因刷屏踩过两次）。
 */
/* ⚠️ 尺寸取 `LZ_GIMBAL_STATUS_BUF`，**不写死数字** —— 七项全报警时最长的
 * 一行是 108 字节 + NUL，缓冲区常量改小了这里要跟着红（见
 * `tests/lz_test_gimbal.c` 的 E1），两处各写一份数字就会漂移。 */
static char s_lastGimbalStatus[LZ_GIMBAL_STATUS_BUF] = "";

/* ------------------------------------------------------------------ */
/* 航点状态回调（PSDK 工作线程）                                        */
/* ------------------------------------------------------------------ */

/**
 * 上一次报过的航点号。
 *
 * ## ⚠️ 这个变量修的是一处**实测到的"控件点不动"**（2026-09-28）
 *
 * 现象：绕飞飞行中，Pilot 上的控件全部无响应；绕飞结束后也不恢复。
 *
 * 根因：这个回调**每次 SDK 推航点状态就发一条浮窗消息**，而 SDK 推得很密。
 * 实测一次 3 分钟的绕飞发了 **716 条**「绕飞中：航点 N」：
 *
 * ```text
 *   16:00 →  26 条
 *   16:01 → 260 条
 *   16:02 → 430 条
 * ```
 *
 * 而浮窗的带宽上限是 **2 KB/s**（PSDK 头文件明写）。716 条把这个通道灌满，
 * 于是**同一时期的控件回执与状态推送全被挤掉** —— 操作员看到的就是
 * "点了没反应"。**不是命令没发出去（`0x3C1A` 有 180 条），是应用侧那条
 * 反馈通道被自己的刷屏堵死了。**
 *
 * ⇒ 只在**航点号真的变了**时发。这是唯一有信息量的时刻：
 * 「绕飞中：航点 3」重复 400 次，对操作员零信息量，对通道却是纯负担。
 *
 * ⚠️ 用 `int` 而不是 `uint32_t`：初值要给一个**不可能的哨兵**（-1），
 * 否则第一个航点（0 或 1）会被误判成"没变过"而漏报。
 */
static int s_lastReportedWaypoint = -1;

static T_DjiReturnCode LzMission_OnWaypointState(T_DjiWaypointV3MissionState state)
{
    /* 只做"记下事件"，动作交给主循环 —— 回调里不能做耗时操作 */
    switch (state.state) {
    case DJI_WAYPOINT_V3_MISSION_STATE_MISSION: {
        /* 只在航点号变化时发 —— 理由见 `s_lastReportedWaypoint`。
         * ⚠️ 这个回调**在 PSDK 工作线程上**，而这里只比一个整数 + 一次
         * 定长拷贝，不是阻塞调用，符合"回调不做耗时动作"的纪律。 */
        const int idx = (int)state.currentWaypointIndex;
        if (idx != s_lastReportedWaypoint) {
            s_lastReportedWaypoint = idx;
            LzWidget_PostMessage("绕飞中：航点 %d", idx);
        }
        break;
    }

    case DJI_WAYPOINT_V3_MISSION_STATE_IDLE:
        /* 回到空闲 = 任务结束（正常跑完或被打断）。
         * 注意这个回调在 RUNNING 期间也可能收到 IDLE，所以用标志位
         * 而不是直接改状态 —— 由主循环统一裁决。 */
        if (s_state == LZ_MISSION_STATE_RUNNING) {
            s_missionEnded = true;
            snprintf(s_endReason, sizeof(s_endReason), "已结束（回到 IDLE）");
        }
        break;

    default:
        break;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* ------------------------------------------------------------------ */
/* 生命周期                                                            */
/* ------------------------------------------------------------------ */

T_DjiReturnCode LzMission_Init(void)
{
    const T_DjiReturnCode rc = DjiWaypointV3_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiWaypointV3_Init 失败 rc=0x%08X", (unsigned)rc);
        return rc;
    }

    const T_DjiReturnCode rc2 = DjiWaypointV3_RegMissionStateCallback(LzMission_OnWaypointState);
    if (rc2 != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("注册航点状态回调失败 rc=0x%08X", (unsigned)rc2);
        return rc2;
    }

    /* 启动诊断**不能**在这里订阅 —— 见 LzMission_StartPostApp() 的说明。
     * 这里只做注册航点回调，那是确实要在 ApplicationStart 之前完成的。 */

    s_state = LZ_MISSION_STATE_IDLE;
    s_missionEnded = false;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode LzMission_StartPostApp(void)
{
    /* 订阅飞机状态话题，供"启动被拒"时排除法定位。
     *
     * ⚠️ 官方文档（.psdk-apiref/docs/cn/20.basic-function/50.fc-subscription.md）：
     *   "请勿在 main() 函数中调用本接口，请在用户线程中调用本接口，
     *    启动调度器后，该接口将正常运行。"
     * 实测 2026-09-20：在 ApplicationStart 之前调它 → 返回 SUCCESS，
     * 但随后读话题时 SIGSEGV。**返回成功不代表调用合法。**
     *
     * 失败不阻断启动 —— 诊断只服务于可观测性，不该拖累主流程。 */
    const LzStatus st = LzBridge_InitStartDiagnostics();
    if (st != LZ_OK) {
        USER_LOG_WARN("启动诊断订阅失败（%s）—— 不影响作业，仅日志信息会少", LzStatus_Str(st));
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode LzMission_DeInit(void)
{
    if (s_state == LZ_MISSION_STATE_RUNNING) {
        (void)LzBridge_StopMissionV3();
    }
    s_state = LZ_MISSION_STATE_IDLE;
    return DjiWaypointV3_DeInit();
}

/* ------------------------------------------------------------------ */
/* 一次完整的绕飞：规划 → 生成 → 上传 → 启动                            */
/* ------------------------------------------------------------------ */

/**
 * @brief 启动绕飞前，把云台工作模式确保为 `YAW_FOLLOW`
 *
 * ## 为什么要在**这里**再设一次（2026-10-01 上机实测的缺陷）
 *
 * 「无人机在绕飞的时候云台显示偏航角达到限位，然后显示云台电机异常，
 *   结束绕飞航线后又正常了。」
 *
 * 成因：**照准**（`LzVisualAlign`）为了横向闭环会把云台设成 `FREE`
 * （"在地面坐标系里固定云台姿态，忽略机身运动"）。那是**飞机上的全局状态**，
 * 不是我们进程里的变量 —— 只要有一次没还回去（进程被杀、或者跑的是还没修
 * 这个缺陷的旧版本），它就一直留着。
 *
 * 而 `FREE` 恰好是绕飞的**唯一死穴**：绕飞靠 `towardPOI` 让机头绕杆连续转
 * 360°，而 `FREE` 要求云台保持地面姿态 ⇒ pan 关节必须反向补偿这 360°，
 * 而 M4T 的 pan 是**相对机头**的 ±60° 软限位 ⇒ 绕出去 60° 就顶到机械限位、
 * 电机持续给力 ⇒ 报"云台电机异常"。航线一结束、遥控器拿回控制权恢复跟随，
 * 关节松回来 ⇒ "结束后又正常了"。三个现象全部对上。
 *
 * ## 为什么放在启动绕飞这一刻，而不是启动应用时
 *
 * 1. 绕飞是 `FREE` **唯一**会出事的场景（照准自己静止观测，`FREE` 是对的），
 *    所以在这一刻纠正它，病因与处置在同一个地方。
 * 2. `DjiGimbalManager_*` 必须在 `DjiCore_Init` **之后**才能用，而这里
 *    一定满足（主循环里、`ApplicationStart` 之后）。
 * 3. **不在 `main()` 启动路径上加阻塞调用** —— `dji_app_ctl install` 会试运行
 *    应用并要求走完 SDK 身份校验（CLAUDE.md 硬规则），启动路径越干净越好。
 *
 * ## 刻意**不**阻断启动
 *
 * 恢复失败只记日志、照常起飞：这可能是一架本来就好的飞机（从没跑过照准），
 * 那时 `Init` 失败不该拦下一次作业。**但日志必须留下** —— 否则真出问题
 * （飞机上仍留着 FREE）时现场没有任何线索。
 */
static void lz_mission_ensure_gimbal_yaw_follow(void)
{
    if (DjiGimbalManager_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_WARN("绕飞前：云台模块初始化失败，跳过模式纠正 —— "
                      "若飞机上残留着 FREE，绕飞时云台偏航会撞 ±60° 限位");
        return;
    }

    const T_DjiReturnCode rc =
        DjiGimbalManager_SetMode(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1,
                                 DJI_GIMBAL_MODE_YAW_FOLLOW);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_WARN("绕飞前：云台模式设 YAW_FOLLOW 被拒 rc=0x%08llX —— "
                      "若当前是 FREE，绕飞时机头绕杆转一圈会把云台偏航顶到限位",
                      (unsigned long long)rc);
    } else {
        USER_LOG_INFO("绕飞前：云台模式已确认为 YAW_FOLLOW（FREE 会撞 pan 限位）");
    }

    (void)DjiGimbalManager_Deinit();
}

/**
 * @brief 按控件上的设定生成航线并上传启动
 * @return 成功返回 true
 */
static bool lz_mission_start_orbit(void)
{
    /* ★ 先把云台模式纠正回来 —— 理由见上面那个函数。
     * 放在**最前面**：取圆心、规划、生成 KMZ 都可能失败并提前返回，
     * 而"飞机上残留 FREE"这件事与那些步骤无关，不该被它们连带跳过。 */
    lz_mission_ensure_gimbal_yaw_follow();

    /* 先取圆心 —— 这是整条链路上唯一的外部输入。取不到就不起飞。 */
    LzTarget pole;
    LzStatus st = LzPole_Acquire(&pole);
    if (st != LZ_OK) {
        /* 未记录杆位是本项目**唯一**一种"操作员还没做前置动作"的失败，
         * 所以单独给一条能照着做的提示 —— 光说"前置条件未就绪"操作员
         * 不知道下一步该按哪里。其余失败仍走通用文案。 */
        if (st == LZ_ERR_NOT_READY) {
            LzWidget_PostMessage("尚未记录绕飞圆心 —— 请先在 PSDK 控件里按"
                                 "「记录飞机位」或「记录激光点」，再拨开关");
        } else {
            LzWidget_PostMessage("取圆心失败（%s）：%s",
                                 LzPole_SourceName(), LzStatus_Str(st));
        }
        return false;
    }

    LzOrbitProfile profile = {
        .radiusM = LzWidget_GetRadiusM(),
        .altitudeM = LzWidget_GetAltitudeM(),
        .speedMs = 3.0,
        /* 航点数由操作员在 Pilot 2 的输入框里填，这里只取当前值。
         * 取值合法性由 `LzPlan_ClampWaypointCount` 保证（getter 内已夹），
         * 包线复核由下面的 `LzPlan_Validate` 做。 */
        .waypointCount = LzWidget_GetWaypointCount(),
        .startBearingDeg = 0.0,
        .clockwise = true,
        /* 云台俯仰由几何反算（`autoGimbalPitch`）。
         *
         * ⚠️ 这里原先写死 -15°。实算下来它只在"低高度 + 大半径"下碰巧对：
         * 默认滑杆值（半径 12.5 m、高度 80.9 m、杆高 15 m）需要的是 **-81°**，
         * 差 66° —— 相机根本没对着目标，而画面上看不出异常。
         * 下面这个字段现在只在 autoGimbalPitch=false 时才有意义。 */
        .gimbalPitchDeg = -15.0,
        .autoGimbalPitch = true,
        /* 走曲线段（近似圆弧）而不是直线段（内接多边形）。
         *
         * 用户 2026-09-22 明确要求"用物理圆，无人机走弧线"：
         * 直线模式下 8 点半径 20 m 的轨迹边心距只有 18.48 m ——
         * 飞机实际比设定半径**近 1.5 m**。曲线段在航点附近被抹圆，
         * 轨迹贴近真圆，同时 8 个点就够（不必靠堆到 16 点去逼近）。
         *
         * 配套的提前转弯截距由 `LzWpml_Build` 从真实段长反算，
         * 这里不用管。 */
        .turnMode = LZ_TURN_PASS_WITH_CURVE,
    };

    const LzGeo takeoff = pole.geo;   /* 相对高度的参考点 */

    /* ---- 1. 算：纯算法，能在桌面上测 ---- */
    LzRoute route;
    LzRoute_Init(&route);
    st = LzPlan_BuildOrbit(&pole, &takeoff, &profile, &route);
    if (st != LZ_OK) {
        LzWidget_PostMessage("规划失败：%s", LzStatus_Str(st));
        LzRoute_Free(&route);
        return false;
    }

    st = LzPlan_Validate(&route, &profile);
    if (st != LZ_OK) {
        /* 安全校验不通过 —— 宁可不起飞，也不要带着已知问题起飞 */
        LzWidget_PostMessage("安全校验不通过：%s，已中止", LzStatus_Str(st));
        LzRoute_Free(&route);
        return false;
    }

    /* ---- 2. 生成 KMZ 到应用目录下的 data/ ---- */
    const char *kmzPath = "data/orbit.kmz";
    st = LzBridge_ExportKmz(&route, &pole, &profile, kmzPath);
    LzRoute_Free(&route);
    if (st != LZ_OK) {
        LzWidget_PostMessage("生成航线文件失败：%s", LzStatus_Str(st));
        return false;
    }

    /* ---- 3. 做：上传并启动（只能上机验证）---- */
    st = LzBridge_UploadKmzV3(kmzPath, true);
    if (st != LZ_OK) {
        /* 上传失败与启动失败要分开说 —— 两者的排查方向完全不同。
         * 早期版本这里只有一句"上传航线失败：文件读写失败"，
         * 而实测（2026-09-20）真实死因是"上传成功、启动被拒"。 */
        if (st == LZ_ERR_START) {
            /* 浮窗带宽上限 2KB/s，且操作员在室外看不到 SDK 日志 ——
             * 把最关键的几项塞进一条短消息里。 */
            LzWidget_PostMessage("启动被拒：%s", LzBridge_StartDiagSummary());
        } else if (st == LZ_ERR_UPLOAD) {
            LzWidget_PostMessage("上传被拒：%s", LzStatus_Str(st));
        } else {
            LzWidget_PostMessage("航线文件处理失败：%s", LzStatus_Str(st));
        }
        return false;
    }

    LzWidget_PostMessage("绕飞已启动（杆位取自%s）：半径 %.1f m，高度 %.1f m，"
                         "%d 个航点（%s）",
                         LzPole_SourceName(),
                         profile.radiusM, profile.altitudeM, profile.waypointCount,
                         (profile.turnMode == LZ_TURN_PASS_WITH_CURVE)
                             ? "弧线" : "直线段");
    return true;
}

/* ------------------------------------------------------------------ */
/* 记录绕飞圆心（由主循环代操作员执行）                                  */
/* ------------------------------------------------------------------ */

/**
 * @brief 处理操作员的"记录圆心"请求
 *
 * ## 为什么这段逻辑在主循环里，而不在控件回调里
 *
 * 控件回调跑在 **PSDK 的工作线程**上，不能做阻塞操作。而"记录激光点"要调
 * `DjiCameraManager_GetLaserRangingInfo()` —— 同步阻塞，最多 1.2 秒。
 * 在回调里调它会把 SDK 的链路线程卡死，实测（2026-09-22）导致进程闪退，
 * 日志里 `semaphore wait timeout` + `send msg to queue error` 刷屏后死掉。
 *
 * 所以回调只置标志，这里（我们自己的线程）才真正干活 ——
 * 与"上传 KMZ 不在回调里做"是同一个模式。
 *
 * ## 两种记录的差别只在一处
 *
 * | 来源 | 取数 | 是否阻塞 |
 * |---|---|---|
 * | 飞机位 | `LzBridge_GetCurrentPosition()` 读订阅缓存 | 否 |
 * | 激光点 | `LzPole_RecordLaser()` 调相机接口 | **是** |
 *
 * 记录成功/失败后的回执、落盘、状态更新完全相同，所以合并在这里。
 */
static void lz_mission_handle_record(void)
{
    LzPoleRecordKind kind;
    if (!LzWidget_TakeRecordRequest(&kind)) {
        return;   /* 操作员没按 */
    }

    LzStatus st;
    LzGeo recorded;

    if (kind == LZ_POLE_RECORD_AIRCRAFT) {
        LzGeo cur;
        st = LzBridge_GetCurrentPosition(&cur);
        if (st == LZ_OK) {
            st = LzPole_RecordAircraft(&cur);
        }
        if (st == LZ_OK) {
            LzWidget_PostMessage("✓ 已记录飞机位为圆心：%.7f, %.7f",
                                 cur.latitudeDeg, cur.longitudeDeg);
        } else if (st == LZ_ERR_NOT_READY) {
            LzWidget_PostMessage("✗ 记录失败：还没有飞机定位数据，请稍候再按");
        } else if (st == LZ_ERR_NO_TARGET) {
            LzWidget_PostMessage("✗ 记录失败：当前没有定位（等 GPS 锁定后再按）");
        } else {
            LzWidget_PostMessage("✗ 记录失败：%s", LzStatus_Str(st));
        }
        return;
    }

    /* ---- 激光点 ---- */
    st = LzPole_RecordLaser();
    if (st == LZ_OK && LzPole_GetRecorded(&recorded) == LZ_OK) {
        /* ⚠️ **把目标高度报出来**（2026-09-29）。
         *
         * 它直接进俯仰公式的 `-h/2` 那一项，而操作员**看不见**它 ——
         * 不报的话，"激光打在旗面上"与"打在旗后面的地面上"在界面上
         * 长得一模一样，而两者的俯仰能差 5° 以上（20 m 外 1.76 m，
         * 画面里 118 px）。用户指出的正是这个风险。
         *
         * 措辞给出**判据**而不只是数值："打旗面"与"打地面"是操作员
         * 自己能分辨的两件事，报出高度后他一眼就知道刚才瞄的是什么。 */
        LzTarget tgt;
        if (LzPole_Acquire(&tgt) == LZ_OK && tgt.heightM > 0.0) {
            LzWidget_PostMessage("✓ 已记录激光点为圆心：%.7f, %.7f（目标高 %.1f m "
                                 "—— 俯仰将瞄它的中点）",
                                 recorded.latitudeDeg, recorded.longitudeDeg,
                                 tgt.heightM);
        } else {
            LzWidget_PostMessage("✓ 已记录激光点为圆心：%.7f, %.7f（点目标，"
                                 "俯仰瞄它自身）",
                                 recorded.latitudeDeg, recorded.longitudeDeg);
        }
    } else if (st == LZ_OK) {
        LzWidget_PostMessage("✓ 已记录激光点");
    } else if (st == LZ_ERR_UNSUPPORTED) {
        /* 措辞面向**操作员**，不是开发者 —— 现场不需要知道什么编译开关 */
        LzWidget_PostMessage("✗ 本包未启用激光记录，请改用「记录飞机位」");
    } else if (st == LZ_ERR_NO_TARGET) {
        /* ⚠️ 这里**不能**只说"无回波" —— `LZ_ERR_NO_TARGET` 覆盖三种成因：
         *   ① 测不到距离（真·无回波）
         *   ② 坐标非法
         *   ③ **坐标是零解**（飞机自身没定位，瞄准点解算退化）
         *
         * 早先只写"激光无回波"，而实测出现过「距离 4.5~14.2 m 明明有效、
         * 飞机没定位导致坐标是零解」的情况 —— 文案把病因指反了，
         * 操作员去调瞄准，实际该做的是等 GPS。
         *
         * 三者里对操作员**可操作**的区分是：要不要等定位。
         * 用飞机当前位置是否有定位来判断该说哪句。 */
        LzGeo cur;
        if (LzBridge_GetCurrentPosition(&cur) == LZ_OK) {
            LzWidget_PostMessage("✗ 记录失败：激光没测到距离，请对准目标再按");
        } else {
            LzWidget_PostMessage("✗ 记录失败：飞机自身没有定位，"
                                 "激光点算不出来 —— 请到室外等 GPS 锁定");
        }
    } else if (st == LZ_ERR_IO) {
        LzWidget_PostMessage("✗ 记录失败：读激光数据出错，详见日志");
    } else {
        LzWidget_PostMessage("✗ 记录失败：%s", LzStatus_Str(st));
    }
}

/* ------------------------------------------------------------------ */
/* 状态机                                                              */
/* ------------------------------------------------------------------ */

/**
 * @brief 云台状态变化时打一条日志（含浮窗）
 *
 * ⚠️ 只在**内容变了**时报，理由见 `s_lastGimbalStatus`。
 * ⚠️ 用 `USER_LOG_ERROR` 而不是 INFO：这条只会在出问题时出现，
 *    而现场是**看不到终端**的（浮窗是唯一反馈通道），所以两边都要留。
 *    它也**不是**"每拍都打"的噪声 —— 变化才打。
 */
static void lz_mission_watch_gimbal(void)
{
    char now[LZ_GIMBAL_STATUS_BUF];   /* 与 s_lastGimbalStatus 同源，见那里 */
    if (!LzBridge_GimbalStatusStr(now, sizeof(now))) {
        return;   /* 还没收到过云台状态 —— 不是"云台正常" */
    }
    if (strcmp(now, s_lastGimbalStatus) == 0) {
        return;
    }
    snprintf(s_lastGimbalStatus, sizeof(s_lastGimbalStatus), "%s", now);

    /* ⚠️ 措辞要能直接对上现场看到的那句话。操作员报的是「偏航角达到限位」
     * 与「云台电机异常」，这里就把这两个词原样用上 —— 他才能确认是同一件事。
     *
     * ⚠️ **两条判据分开、不合并**：它们的处置完全不同（限位是姿态问题，
     * 电机异常是硬件报警），而本项目反复踩过的形状正是"光看失败了会混"。
     * `LzBridge_GimbalStatusStr` 的输出格式（一项一个短词）就是为这个
     * 匹配服务的 —— 定长表格里"偏航"会出现两次，两件事会撞在一起。
     *
     * ⚠️ 下面 `strstr` 匹配的那几个词，**真值在 `src/lz_gimbal_status.c`**
     * （`kYawLimitName` 等）。改那边的措辞就必须同步改这里，否则这两条
     * 针对性浮窗会**静默失效**（退化成下面的通用那条）—— 与控件索引、
     * 图标文件名那类"两边各写一份、对不上也不报错"同一个形状。 */
    /* ⚠️ 日志级别跟着内容走：`正常` 是**好事**，打成 ERROR 会让日志里
     * 出现"错误"字样而实际一切正常 —— 那是把排查方向带反的经典形状
     * （本项目在激光 `exception` 白名单、`LzVisionMiss` 文案上都踩过）。 */
    if (strcmp(now, "正常") == 0) {
        USER_LOG_INFO("云台状态：%s", now);
    } else {
        USER_LOG_ERROR("云台状态：%s", now);
    }

    if (strstr(now, "偏航限位") != NULL) {
        LzWidget_PostMessage("⚠ 云台偏航顶到限位 —— %s", now);
    } else if (strstr(now, "偏航电机异常") != NULL) {
        LzWidget_PostMessage("⚠ 云台偏航电机异常 —— %s", now);
    } else {
        LzWidget_PostMessage("云台状态变化：%s", now);
    }
}

void LzMission_Tick(void)
{
    /* 云台状态**最先查**：它可能在绕飞中途跳变（现场正是这个现象），
     * 而状态机那几步会提前 return / break —— 放在后面会被跳过。 */
    lz_mission_watch_gimbal();

    /* 记录请求**优先于**作业状态机处理：它与绕飞是否在跑无关
     * （操作员可以在任何时候记录圆心），而且它可能耗时 1.2 秒 ——
     * 放在状态机之前，避免被绕飞的状态转移耽误。 */
    lz_mission_handle_record();

    /* 「识别目标」按钮：回调只置标志，这里把它转成照准模块的请求。
     *
     * ⚠️ 只转发，**不做任何照准动作** —— 真正的照准由 `LzVisualAlign_Tick()`
     * 推进（在 main 的循环里，紧跟着本函数）。把十几秒的工作塞进这里会
     * 卡住整个任务状态机，操作员这期间拨绕飞开关都没有响应。
     *
     * ⚠️ **照准与绕飞互斥，且两个方向都由 `LzAlign_CheckConflict()` 判**
     * （本处是"照准"方向，下面 `lz_mission_start_orbit()` 是"绕飞"方向）。
     * 早先只有本处这一个方向查了绕飞，反方向没查 —— 于是照准正转着云台时
     * 拨开关能启动航线，航线里的 `gimbalRotate` 与手动控制抢同一个云台。
     * 判据写在一处（`lz_align.c`，含"两个都在跑时绕飞优先"的用例），
     * 两处调用，不各写一份。 */
    if (LzWidget_TakeAlignRequest()) {
        const LzConflict c = LzAlign_CheckConflict(
            LzMission_IsRunning(), LzVisualAlign_State() == LZ_ALIGN_RUNNING);
        if (c != LZ_CONFLICT_NONE && c != LZ_CONFLICT_ALIGN_ACTIVE) {
            /* 绕飞在跑 ⇒ 不启动照准。注意「照准已在跑」时**放行**：
             * 那个请求是"再按一次 = 停止"，由 LzVisualAlign_Tick 处理。 */
            LzWidget_PostMessage("%s", LzAlign_ConflictStr(c));
        } else {
            LzVisualAlign_RequestToggle();
        }
    }

    switch (s_state) {
    case LZ_MISSION_STATE_IDLE: {
        if (!LzWidget_IsOrbitRequested()) {
            break;   /* 操作员没请求，什么都不做 */
        }
        /* ⚠️ **互斥的反方向**（上面那条是"照准"方向）：
         * 照准正在转云台时不许启动航线 —— 航线里的 `gimbalRotate` 与
         * 手动云台控制会抢同一个云台，而"抢"的表现是命令被拒或姿态诡异，
         * 都不指向真实病因。
         *
         * 刻意**不在这里把开关清掉**（不调 `LzWidget_ReportOrbitFinished`）：
         * 操作员的意思很可能是"先识别、再绕飞"，此刻清掉开关会让他在照准
         * 结束后**还得再拨一次 ON** —— 而他记得自己已经拨过了。
         * 所以状态停在 IDLE、开关保持 ON：照准一结束，下一拍就自然启动。 */
        const LzConflict c = LzAlign_CheckConflict(
            false, LzVisualAlign_State() == LZ_ALIGN_RUNNING);
        if (c == LZ_CONFLICT_ALIGN_ACTIVE) {
            if (!s_orbitDeferredReported) {
                s_orbitDeferredReported = true;
                LzWidget_PostMessage("%s（开关保持 ON，照准结束后会自动启动）",
                                     LzAlign_ConflictStr(c));
            }
            break;
        }
        s_orbitDeferredReported = false;
        if (lz_mission_start_orbit()) {
            s_state = LZ_MISSION_STATE_RUNNING;
            s_missionEnded = false;
            s_stopRejectReported = false;   /* 新一轮作业，告警闸归零 */
            s_lastReportedWaypoint = -1;    /* 新一轮的第一个航点必须报 */
        } else {
            /* 启动失败：清本地意图并发结束消息。
             * ⚠️ 注意这**不会**把 Pilot 上的开关拨回去（PSDK 没有那个接口，
             * 详见 lz_widget.h 的 LzWidget_ReportOrbitFinished 说明）——
             * 实际是"开关保持 ON 而浮窗说启动失败"，所以消息里带上
             * "请手动拨回"，不假装界面已经一致了。 */
            LzWidget_ReportOrbitFinished("启动失败");
        }
        break;
    }

    case LZ_MISSION_STATE_RUNNING: {
        /* 操作员中途拨 OFF → 立即停（见 lz_widget.h 的安全语义） */
        if (!LzWidget_IsOrbitRequested()) {
            /* ⚠️ 这里**不能**丢弃返回值。
             *
             * 早先写成 `(void)LzBridge_StopMissionV3();` 然后无条件置 IDLE、
             * 无条件报"绕飞结束：操作员停止" —— 于是 STOP 被拒时，
             * 飞机还在杆旁边绕，而日志与界面都说已经停了。
             * **静默的失败等于假装成功**，而在飞控语境里这直接关系到安全。
             *
             * 停止失败时**保持 RUNNING**：让状态与事实一致
             * （任务确实还在跑），操作员再拨一次 OFF 就能重试。
             * 若在这里置 IDLE，开关还在 ON 位而状态机认为空闲 ——
             * 那个组合会让下一次 Tick 把它当成"新的绕飞请求"重新上传启动。 */
            const LzStatus st = LzBridge_StopMissionV3();
            if (st != LZ_OK) {
                /* 只报一次：Tick 是 100 ms 一拍，而开关会一直保持 OFF，
                 * 不加闸就会每秒刷 10 条浮窗。 */
                if (!s_stopRejectReported) {
                    s_stopRejectReported = true;
                    /* 措辞要准确：此刻开关**已经在 OFF 位**，只拨 OFF 不会再
                     * 触发回调（值没变化）。要重试必须走 OFF→ON→OFF，
                     * 让控件值产生变化 —— 所以提示里要写清楚。 */
                    LzWidget_PostMessage("停止指令被拒（%s）—— 任务可能仍在执行。"
                                         "重试需把开关拨回 ON 再拨 OFF，"
                                         "或直接用遥控器接管",
                                         LzStatus_Str(st));
                }
                break;   /* 状态不动，留在 RUNNING */
            }
            s_stopRejectReported = false;
            s_state = LZ_MISSION_STATE_IDLE;
            LzWidget_ReportOrbitFinished("操作员停止");
            break;
        }
        /* 飞机侧报告结束 */
        if (s_missionEnded) {
            s_missionEnded = false;
            s_state = LZ_MISSION_STATE_IDLE;
            LzWidget_ReportOrbitFinished(s_endReason);
        }
        break;
    }
    }
}

bool LzMission_IsRunning(void)
{
    return s_state == LZ_MISSION_STATE_RUNNING;
}
