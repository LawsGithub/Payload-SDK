/**
 * @file lz_gimbal_probe.c
 * @brief 探针：M4T 自带云台能不能被程序控制、能不能真的转到指定角度。
 *
 * ## 为什么这一步必须先做
 *
 * 视觉调俯仰的完整链路是「识别 → 算该转多少度 → **转云台** → 确认到位」。
 * 前三环都已实现或已实测，**只有"转云台"这一环从未在 M4T 上验过**，
 * 而它有两个已知的坏消息：
 *
 * | 来源 | 说法 |
 * |---|---|
 * | Payload-SDK issue **#555** | M4T 云台**没有自由模式**，只有跟随模式；且**没有单独的 yaw 控制** |
 * | Payload-SDK issue **#563** | `DjiGimbalManager_Rotate` 的**实际速度只有下发的 10–20%**，且差距"非常大" |
 *
 * 两个 issue 都**没有官方回复**（#555 至今 open）。所以只能实测。
 *
 * ## 本探针按顺序回答四个问题
 *
 * 1. **`DjiGimbalManager_Init()` 能不能成功** —— 不能就全完了
 * 2. **云台角度读得回来吗** —— 订阅 `GIMBAL_ANGLES` 走**回调缓存**
 *    （⚠️ `DjiFcSubscription_GetLatestValueOfTopic` 在本 SDK 版本上**必崩**，
 *    见 CLAUDE.md；官方样例用的正是那个会崩的接口，**照抄样例会崩**）
 * 3. **`SetMode` 接受哪些模式** —— 逐个试 FREE / YAW_FOLLOW，各自读回
 * 4. **`Rotate` 转到指定角度要多久、准不准** —— 下发后用**轮询读回**等它收敛，
 *    报告「实际到位值 / 耗时 / 是否稳定」。⚠️ 因为 #563 说速度对不上，
 *    **不能假设"等一下就好"**，必须实测收敛过程
 *
 * ## 安全性
 *
 * 只动**云台俯仰**（pitch），**不碰 yaw/roll**，也不动飞控。
 * 角度序列是几个温和的固定值（−30° / −60° / −90° / 回中），
 * 都在 M4T 的可控范围内（−90°~70°，见 `lz_plan.h` 的常量）。
 * 结束后**回中**，把云台留在自然位置。
 *
 * ## 用法
 *
 *   ./lz_gimbal_probe            # 默认序列，可停在需确认处
 *   ./lz_gimbal_probe 0          # 只用 RELATIVE 模式测（更保守）
 *
 * ⚠️ 需要飞机通电、`Smart3DExplore` 与已装的 `liangzhourenwu` 都停掉
 * （它们都会占 PSDK 通道）。⚠️ **不要同时开 Pilot 里别的云台控制界面** ——
 * 两个控制源会抢权限（`DJI_ERROR_GIMBAL_MODULE_CODE_NON_CONTROL_AUTHORITY`）。
 */

#include <dji_core.h>
#include <dji_fc_subscription.h>
#include <dji_gimbal_manager.h>
#include <dji_logger.h>
#include <dji_platform.h>
#include <dji_typedef.h>

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "platform/lz_platform.h"
#include "platform/lz_user_info.h"

/* M4T 原生相机（含激光、含云台）实测在位置 1 —— 见 CLAUDE.md */
#define LZ_GIMBAL_MOUNT DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1

static void say(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    fflush(stdout);   /* 管道/文件下全缓冲，卡住前的输出会丢 */
}

static void say_rc(const char *what, T_DjiReturnCode rc)
{
    say("  %-42s rc=0x%08llX %s\n", what, (unsigned long long)rc,
        (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? "✅" : "❌");
}

/* ------------------------------------------------------------------ */
/* 云台角：订阅 + 回调缓存                                             */
/* ------------------------------------------------------------------ */

static volatile T_DjiVector3f s_angles = {0};
static volatile bool s_gotAngles = false;
static volatile uint32_t s_angleMsgCount = 0;

/**
 * @brief GIMBAL_ANGLES 话题回调
 *
 * ⚠️ **必须走回调，不能用 `DjiFcSubscription_GetLatestValueOfTopic`** ——
 * 后者在 PSDK 3.16.0-beta 上**必崩**（SIGSEGV，栈在
 * `DjiDataSubscriptionDds_v3_GetLastValueOfTopic` 内部），已实测。
 * ⚠️ 官方样例 `test_gimbal_manager.c` 用的正是那个会崩的接口 ——
 * **照抄样例会崩**，这是本项目的一条硬教训（文档/样例都不等于能用）。
 */
static T_DjiReturnCode on_gimbal_angles(const uint8_t *data, uint16_t size,
                                        const T_DjiDataTimestamp *ts)
{
    (void)ts;
    if (data == NULL || size < sizeof(T_DjiVector3f)) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    memcpy((void *)&s_angles, data, sizeof(T_DjiVector3f));
    s_gotAngles = true;
    s_angleMsgCount++;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/** 读回当前俯仰（度）。读不到返回 NAN。 */
static double read_pitch(void)
{
    if (!s_gotAngles) {
        return NAN;
    }
    return (double)s_angles.x;   /* x = pitch（NED 参考系，单位 deg） */
}

/** 睡 ms —— 用 SDK 的 osal，与 SDK 内部同一套时钟 */
static void sleep_ms(uint32_t ms)
{
    const T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    if (osal != NULL && osal->TaskSleepMs != NULL) {
        (void)osal->TaskSleepMs(ms);
    }
}

/**
 * @brief 下发一个绝对俯仰角，然后**轮询读回**直到收敛或超时
 *
 * ⚠️ **不能"下发完睡一下就当到位了"** —— issue #563 报告实际速度只有下发的
 * 10–20%，"等一下"到底是等多久没有依据。所以这里实测收敛过程：
 * 每 200 ms 读一次，连续 3 次变化 < 0.2° 就算稳定。
 *
 * @param targetDeg 目标俯仰角（度，向下为负）
 * @param outSettledDeg [out] 最终稳定值
 * @param outElapsedMs  [out] 从下发到稳定的耗时
 * @return true = 稳定了
 */
static bool rotate_and_wait(double targetDeg, float *outSettledDeg,
                            uint32_t *outElapsedMs)
{
    T_DjiGimbalManagerRotation rot;
    memset(&rot, 0, sizeof(rot));
    rot.rotationMode = DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE;
    rot.pitch = (dji_f32_t)targetDeg;
    rot.roll = 0.0f;
    rot.yaw = 0.0f;
    rot.time = 0.0;   /* 0 = 让云台自己决定速度（不出手催它） */

    const T_DjiReturnCode rc = DjiGimbalManager_Rotate(LZ_GIMBAL_MOUNT, rot);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        say_rc("  Rotate(absolute)", rc);
        /* 常见失败码的中文含义 —— 直接把可能的病因列出来，省一次查表 */
        if (rc == DJI_ERROR_GIMBAL_MODULE_CODE_NON_CONTROL_AUTHORITY) {
            say("    ⇒ NON_CONTROL_AUTHORITY：有别的控制源占着云台"
                "（Pilot 界面？另一个应用？）\n");
        }
        return false;
    }

    /* 轮询等收敛 */
    const uint32_t stepMs = 200;
    const uint32_t maxMs = 8000;      /* 8 秒还没稳就算失败 —— 足够宽容 */
    uint32_t elapsed = 0;
    int stableCount = 0;
    double prev = read_pitch();
    double now = prev;

    while (elapsed < maxMs) {
        sleep_ms(stepMs);
        elapsed += stepMs;
        now = read_pitch();
        if (isnan(now) || isnan(prev)) {
            prev = now;
            continue;
        }
        if (fabs(now - prev) < 0.2) {
            stableCount++;
            if (stableCount >= 3) {
                break;
            }
        } else {
            stableCount = 0;
        }
        prev = now;
    }

    if (elapsed >= maxMs) {
        say("    ⚠️ %u ms 内未收敛（可能还在转，或读回不更新）\n", maxMs);
    }
    *outSettledDeg = (float)now;
    *outElapsedMs = elapsed;
    return (elapsed < maxMs);
}

/* ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
    int relOnly = 0;
    if (argc >= 2) {
        relOnly = atoi(argv[1]);
    }

    say("[lz_gimbal_probe] 探针：M4T 自带云台可不可控、准不准\n");
    say("⚠️ 只动 pitch，不动 yaw/roll，不动飞控。结束后回中。\n");
    say("⚠️ 请确保 Pilot 2 上**没有别的云台控制界面开着**（会抢权限）。\n\n");

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
    (void)DjiCore_ApplicationStart();

    /* ---- 1) 订阅云台角（必须先订阅，否则读不到）---- */
    say("\n① 订阅 GIMBAL_ANGLES（**走回调缓存**，不用会崩的 getter）\n");
    const T_DjiReturnCode rcSub = DjiFcSubscription_Init();
    say_rc("DjiFcSubscription_Init", rcSub);
    {
        /* ⚠️ 50 Hz 是官方样例用的频率，且 GIMBAL_ANGLES 本身支持到 50 Hz。
         * 与现有 5 个 10 Hz 话题**不同频率** ⇒ 不占它们那 242 字节的额度
         * （实测：现有 5 个合计 72 字节，加 12 字节也远没到上限）。 */
        const T_DjiReturnCode r = DjiFcSubscription_SubscribeTopic(
            DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
            DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, on_gimbal_angles);
        say_rc("SubscribeTopic(GIMBAL_ANGLES, 50Hz)", r);
        if (r != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            say("  ⇒ 订阅失败就**读不回来**，后面的测都做不了\n");
            goto out;
        }
    }

    /* ⚠️ **等久一点，并且每秒报一次消息数** —— 只等 600 ms 就下结论
     * 「读不到」是错的：那条话题的推送可能比订阅返回晚得多，
     * 也可能是"根本不会推"（那才是真结论）。两者的区别只有时间能分开。 */
    {
        const T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
        for (int i = 0; i < 8; ++i) {
            sleep_ms(1000);
            say("  [%ds] 云台角消息累计 %u 条", i + 1, s_angleMsgCount);
            if (s_gotAngles) {
                say("，pitch=%.2f° roll=%.2f yaw=%.2f\n",
                    (double)s_angles.x, (double)s_angles.y, (double)s_angles.z);
            } else {
                say("（还没收到）\n");
            }
            if (s_angleMsgCount > 20) {
                break;   /* 在推就够了，不必等满 */
            }
        }
        (void)osal;
    }
    const double pitch0 = read_pitch();
    if (isnan(pitch0)) {
        say("  ⇒ **8 秒里一条都没收到**：这条话题在 M4T 内置云台上不推送，\n"
            "     或者需要额外的开关。**云台角读不回来** ⇒ 闭环的「确认到位」\n"
            "     这一环没有依据 —— 这本身就是一个要记录的结论。\n");
    } else {
        say("  ⇒ 云台角可读（pitch=%.2f°）\n", pitch0);
    }

    /* ---- 2) Init + SetMode ---- */
    say("\n② DjiGimbalManager_Init 与 SetMode\n");
    const T_DjiReturnCode rcInit = DjiGimbalManager_Init();
    say_rc("DjiGimbalManager_Init", rcInit);
    if (rcInit != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        goto out;
    }

    /* issue #555 说 M4T 没有 FREE 模式、只有跟随模式。逐个试，各自读回，
     * 看**报错**还是**被接受但行为不符**（后者更难发现）。 */
    {
        struct { E_DjiGimbalMode m; const char *name; } modes[] = {
            { DJI_GIMBAL_MODE_FREE,       "FREE(0)" },
            { DJI_GIMBAL_MODE_YAW_FOLLOW, "YAW_FOLLOW(2)" },
        };
        for (size_t i = 0; i < sizeof(modes) / sizeof(modes[0]); ++i) {
            const T_DjiReturnCode r =
                DjiGimbalManager_SetMode(LZ_GIMBAL_MOUNT, modes[i].m);
            say_rc(modes[i].name, r);
            sleep_ms(400);
            const double p = read_pitch();
            say("      ↳ 读回 pitch = %s%.2f°\n",
                isnan(p) ? "读不到 " : "", isnan(p) ? 0.0 : p);
        }
        say("  ⚠️ 两个都返回 SUCCESS 也**不等于**模式生效 —— issue #555 正是"
            "\"设了 FREE 但云台仍在跟随\"。\n"
            "     真正的判据是：**飞机 yaw 转动时画面跟不跟着转**（需人观察）。\n");
    }

    /* ---- 2.5) 控制权诊断 ----
     *
     * ⚠️ 2026-09-27 首次实测：`Rotate` 报 `NON_CONTROL_AUTHORITY`
     * （`0x600000006` = 云台模块 + raw 0x006），而**同一时刻 `Reset` 却返回
     * SUCCESS** —— 说明云台被认出来了、只是"写"操作没权限。
     *
     * 而 issue #563 的报告者**能转**（只是慢）。他多做了两件事：
     * `SetControllerMaxSpeedPercentage` 与 `SetControllerSmoothFactor`。
     * 所以这里把它们补上，逐个试并记录返回码 ——
     * **如果"设置型"调用全被拒，说明整个写通道没权限；
     * 如果只有 Rotate 被拒，那病因就窄得多。** 这两种结论的后续完全不同。 */
    say("\n②.5 控制权诊断：把「设置型」调用逐个试一遍\n");
    say("   （Rotate 被拒而 Reset 成功 ⇒ 不是「云台不存在」，是「写」没权限；\n");
    say("     这里试的每一个能不能成功，决定了病因有多宽）\n");
    {
        say_rc("SetPitchRangeExtensionEnabled(true)",
               DjiGimbalManager_SetPitchRangeExtensionEnabled(LZ_GIMBAL_MOUNT, true));
        say_rc("SetControllerMaxSpeedPercentage(pitch,100)",
               DjiGimbalManager_SetControllerMaxSpeedPercentage(
                   LZ_GIMBAL_MOUNT, DJI_GIMBAL_AXIS_PITCH, 100));
        say_rc("SetControllerSmoothFactor(pitch,2)",
               DjiGimbalManager_SetControllerSmoothFactor(
                   LZ_GIMBAL_MOUNT, DJI_GIMBAL_AXIS_PITCH, 2));
        say_rc("SetMode(YAW_FOLLOW) 再试一次",
               DjiGimbalManager_SetMode(LZ_GIMBAL_MOUNT, DJI_GIMBAL_MODE_YAW_FOLLOW));

        /* 设完速度/平滑度**再试一次 Rotate** —— issue #563 的报告者就是这个配置 */
        say("   ↳ 配好速度/平滑度后立刻重试 Rotate(relative -5°)：\n");
        T_DjiGimbalManagerRotation r5;
        memset(&r5, 0, sizeof(r5));
        r5.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;
        r5.pitch = -5.0f;
        r5.time = 0.0;
        const T_DjiReturnCode rcRetry =
            DjiGimbalManager_Rotate(LZ_GIMBAL_MOUNT, r5);
        say_rc("Rotate(relative -5°) 重试", rcRetry);
        if (rcRetry != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            say("      ⇒ **配了速度/平滑度也没用** ⇒ 与 #563 的差异不在这些设置上，\n");
            say("        更像「控制权从一开始就不在我们这边」。下一步要试的是\n");
            say("        **在 Pilot 2 上退出相机界面 / 不碰云台**，让遥控器释放控制权。\n");
        } else {
            say("      ⇒ 配了速度/平滑度后 Rotate 通过！与 #563 的做法一致。\n");
        }
    }

    /* ---- 3) Rotate：绝对角，逐个读回 ---- */
    say("\n③ Rotate 绝对俯仰角，逐个等收敛\n");
    say("   （issue #563 说实速只有下发的 10–20%，所以**实测收敛时间**，"
        "不假设\"等一下就好\"）\n\n");

    if (!relOnly) {
        const double targets[] = { -30.0, -60.0, -90.0, 0.0 };
        say("  %-10s %-12s %-10s %-8s %s\n",
            "目标", "到位", "误差", "耗时", "判定");
        for (size_t i = 0; i < sizeof(targets) / sizeof(targets[0]); ++i) {
            float settled = 0.0f;
            uint32_t ms = 0;
            const bool ok = rotate_and_wait(targets[i], &settled, &ms);
            const double err = (double)settled - targets[i];
            say("  %-10.1f %-12.2f %-10.2f %-8u %s\n",
                targets[i], (double)settled, err, ms,
                ok ? (fabs(err) < 1.0 ? "✅ 到位" : "⚠️ 到了但偏差大")
                   : "⚠️ 未收敛");
        }
    }

    /* ---- 4) 相对模式（更保守的对照）---- */
    say("\n④ RELATIVE 模式对照（从当前位置转 −10°）\n");
    {
        const double before = read_pitch();
        T_DjiGimbalManagerRotation rot;
        memset(&rot, 0, sizeof(rot));
        rot.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;
        rot.pitch = -10.0f;
        rot.time = 0.0;
        const T_DjiReturnCode r = DjiGimbalManager_Rotate(LZ_GIMBAL_MOUNT, rot);
        say_rc("Rotate(relative -10°)", r);
        sleep_ms(2500);
        const double after = read_pitch();
        if (!isnan(before) && !isnan(after)) {
            say("      ↳ %.2f° → %.2f°（实际转了 %.2f°，目标是 −10°）\n",
                before, after, after - before);
        }
    }

    /* ---- 5) 回中，把云台留在自然位置 ---- */
    say("\n⑤ 回中（复位俯仰与偏航）\n");
    {
        const T_DjiReturnCode r =
            DjiGimbalManager_Reset(LZ_GIMBAL_MOUNT,
                                   DJI_GIMBAL_RESET_MODE_PITCH_AND_YAW);
        say_rc("DjiGimbalManager_Reset(PITCH_AND_YAW)", r);
        sleep_ms(1500);
        const double p = read_pitch();
        say("      ↳ 回中后 pitch = %s%.2f°\n",
            isnan(p) ? "读不到 " : "", isnan(p) ? 0.0 : p);
    }

    say("\n=== 结论怎么看 ===\n");
    say("  · ① 订阅成功 + ② 读回有数 ⇒ **云台角可读**，视觉闭环的前提成立\n");
    say("  · ③ 有「✅ 到位」的行 ⇒ **云台可被程序控制**，且角度准\n");
    say("  · ③ 全是「⚠️ 到了但偏差大」⇒ 像 #563 那样速度/精度有问题，"
        "闭环要改成\"多轮逼近\"而不是\"一次到位\"\n");
    say("  · ③ 全失败且报 NON_CONTROL_AUTHORITY ⇒ 有别的控制源，先关掉它\n");
    say("  · ② 两个模式都 SUCCESS ⇒ 仍需人观察飞机 yaw 时画面是否跟随"
        "（#555 的判据）\n");

out:
    say("\n清理...\n");
    (void)DjiGimbalManager_Deinit();
    (void)DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES);
    DjiCore_DeInit();
    LzPlatform_Deinit();
    return 0;
}
