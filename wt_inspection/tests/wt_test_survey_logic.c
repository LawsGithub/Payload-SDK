/**
 * @file wt_test_survey_logic.c
 * @brief 激光采集决策层的回归测试。
 *
 * 这些判据一行写错都不会报错，只会让采集卡死或采到错误的点：
 *
 *   - 已对准后又下发一次云台 → 云台重新微动 → 永远等不到停稳
 *   - 用「距上次调用」而不是「距进入状态」判停稳 → 立刻就读激光
 *   - 停稳超时用加法比较 → uint32 回绕时永久卡住
 *   - 快照过旧仍据此下发 → 按目标已经不在的位置转云台
 *   - 重试不计数 → 对不准时无限循环
 *
 * 决策层抽出来的唯一目的就是让这些能在桌面上穷举。
 */

#include "wt_survey_logic.h"

#include "wt_camera.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int s_failures = 0;

#define CHECK(cond, fmt, ...)                                              \
    do {                                                                   \
        if (!(cond)) {                                                     \
            printf("  [FAIL] " fmt "\n", ##__VA_ARGS__);                   \
            s_failures++;                                                  \
        }                                                                  \
    } while (0)

#define IMG_W 1440
#define IMG_H 1080

/** 构造一个"目标就在画面中心"的观测 —— 即已对准 */
static WtSurveyObs MakeObsCentered(uint32_t nowMs)
{
    WtCameraGeometry g = WtCamera_ComputeGeometry(&WT_CAMERA_M4T_WIDE);
    WtSurveyObs obs;

    memset(&obs, 0, sizeof(obs));
    obs.nowMs = nowMs;
    obs.haveGimbal = true;
    obs.gimbal.yawDeg = 30.0;
    obs.gimbal.pitchDeg = -20.0;
    obs.haveBodyYaw = true;
    obs.bodyYawDeg = 30.0;   /* 机头与云台同向 → 目标在可达范围内 */
    obs.haveDetection = true;
    obs.detectionAgeMs = 50;
    obs.targetVisible = true;
    obs.targetU = IMG_W * 0.5;
    obs.targetV = IMG_H * 0.5;
    obs.imageWidthPx = IMG_W;
    obs.imageHeightPx = IMG_H;
    obs.fovHorizDeg = g.fovHorizDeg;
    obs.fovVertDeg = g.fovVertDeg;

    return obs;
}

/** 目标在画面角落 —— 远未对准 */
static WtSurveyObs MakeObsOffCenter(uint32_t nowMs)
{
    WtSurveyObs obs = MakeObsCentered(nowMs);

    obs.targetU = (double)IMG_W;
    obs.targetV = (double)IMG_H;

    return obs;
}

/**
 * 已对准时**不应再下发云台**，而是直接进停稳等待。
 *
 * 多下发一次会让云台重新微动，"刚对准"又变成"正在动"，永远等不到稳定。
 */
static void Test_OnTarget_DoesNotRotate(void)
{
    WtSurveyLogic s;
    WtSurveyObs obs = MakeObsCentered(1000);
    WtSurveyDecision d;

    printf("Test_OnTarget_DoesNotRotate\n");

    WtSurveyLogic_Begin(&s, NULL, 1000);
    CHECK(s.state == WT_SURVEY_AIMING, "初始应在对准状态");

    d = WtSurveyLogic_Step(&s, &obs);

    CHECK(d.action != WT_SURVEY_ACT_ROTATE_GIMBAL,
          "已对准时不应下发云台");
    CHECK(s.state == WT_SURVEY_SETTLING, "应进入停稳等待");
    CHECK(s.attempts == 0, "已对准不应消耗重试次数（得到 %d）", s.attempts);
    CHECK(fabs(d.angularErrorDeg) < 0.01, "中心像素的夹角应接近 0");
}

/** 未对准时下发云台，并消耗一次重试 */
static void Test_OffTarget_Rotates(void)
{
    WtSurveyLogic s;
    WtSurveyObs obs = MakeObsOffCenter(1000);
    WtSurveyDecision d;

    printf("Test_OffTarget_Rotates\n");

    WtSurveyLogic_Begin(&s, NULL, 1000);
    d = WtSurveyLogic_Step(&s, &obs);

    CHECK(d.action == WT_SURVEY_ACT_ROTATE_GIMBAL, "未对准应下发云台");
    CHECK(s.attempts == 1, "应消耗一次重试（得到 %d）", s.attempts);
    CHECK(d.angularErrorDeg > 10.0, "角落目标夹角应很大（得到 %.1f）",
          d.angularErrorDeg);
    /* 水平误差要报出来：±60° 内由云台承担，超出则需转机头 */
    CHECK(fabs(d.yawErrorDeg) > 1.0, "应报出机头需要转的角度");
}

/** 停稳等待必须真的等够时间，不能立刻读激光 */
static void Test_Settling_WaitsFullDuration(void)
{
    WtSurveyLogic s;
    WtSurveyConfig cfg = WtSurveyLogic_DefaultConfig();
    WtSurveyObs obs;
    WtSurveyDecision d;

    printf("Test_Settling_WaitsFullDuration\n");

    WtSurveyLogic_Begin(&s, &cfg, 1000);

    /* 先对准，进入 SETTLING */
    obs = MakeObsCentered(1000);
    (void)WtSurveyLogic_Step(&s, &obs);
    CHECK(s.state == WT_SURVEY_SETTLING, "应在停稳等待");

    /* 还差 1 ms */
    obs = MakeObsCentered(1000 + cfg.settleMs - 1);
    d = WtSurveyLogic_Step(&s, &obs);
    CHECK(d.action == WT_SURVEY_ACT_WAIT, "未等够时应继续等待");
    CHECK(s.state == WT_SURVEY_SETTLING, "状态不应提前推进");

    /* 刚好等够 */
    obs = MakeObsCentered(1000 + cfg.settleMs);
    d = WtSurveyLogic_Step(&s, &obs);
    CHECK(d.action == WT_SURVEY_ACT_READ_LASER, "等够后应读激光");
    CHECK(s.state == WT_SURVEY_LASING, "应进入读激光状态");
}

/**
 * 停稳超时的时间比较必须抗 uint32 回绕。
 *
 * 用 `now > entered + settle` 写法时，entered 接近 UINT32_MAX 会让右边溢出
 * 成一个小数，于是"还没到"被判成"已经到了"（或反之，永久卡住）。
 * 减法写法不受影响。
 */
static void Test_Settling_SurvivesWraparound(void)
{
    WtSurveyLogic s;
    WtSurveyConfig cfg = WtSurveyLogic_DefaultConfig();
    const uint32_t nearMax = 0xFFFFFF00u; /* 距回绕还有 255 ms */
    WtSurveyObs obs;
    WtSurveyDecision d;

    printf("Test_Settling_SurvivesWraparound\n");

    WtSurveyLogic_Begin(&s, &cfg, nearMax);

    obs = MakeObsCentered(nearMax);
    (void)WtSurveyLogic_Step(&s, &obs);
    CHECK(s.state == WT_SURVEY_SETTLING, "应在停稳等待");

    /* 时间回绕过去了：now 变成一个很小的数，但实际已过 settleMs */
    obs = MakeObsCentered((uint32_t)(nearMax + cfg.settleMs)); /* 自然回绕 */
    d = WtSurveyLogic_Step(&s, &obs);

    CHECK(d.action == WT_SURVEY_ACT_READ_LASER,
          "回绕后仍应正确判定已等够（否则采集永久卡住）");
}

/** 快照过旧时必须等新帧，不能据此下发云台 */
static void Test_StaleSnapshot_Waits(void)
{
    WtSurveyLogic s;
    WtSurveyConfig cfg = WtSurveyLogic_DefaultConfig();
    WtSurveyObs obs = MakeObsOffCenter(1000);
    WtSurveyDecision d;

    printf("Test_StaleSnapshot_Waits\n");

    WtSurveyLogic_Begin(&s, &cfg, 1000);
    obs.detectionAgeMs = cfg.maxSnapshotAgeMs + 1;

    d = WtSurveyLogic_Step(&s, &obs);

    CHECK(d.action == WT_SURVEY_ACT_WAIT, "快照过旧应等待");
    CHECK(s.attempts == 0, "等待不应消耗重试次数");
    CHECK(s.state == WT_SURVEY_AIMING, "应留在对准状态");
}

/** 缺少云台姿态 / 目标不可见 / 还没识别过 —— 都只能等 */
static void Test_MissingInputs_Wait(void)
{
    WtSurveyLogic s;
    WtSurveyObs obs;
    WtSurveyDecision d;

    printf("Test_MissingInputs_Wait\n");

    WtSurveyLogic_Begin(&s, NULL, 1000);

    obs = MakeObsOffCenter(1000);
    obs.haveGimbal = false;
    d = WtSurveyLogic_Step(&s, &obs);
    CHECK(d.action == WT_SURVEY_ACT_WAIT, "无云台姿态应等待");

    obs = MakeObsOffCenter(1000);
    obs.haveDetection = false;
    d = WtSurveyLogic_Step(&s, &obs);
    CHECK(d.action == WT_SURVEY_ACT_WAIT, "无识别结果应等待");

    obs = MakeObsOffCenter(1000);
    obs.targetVisible = false;
    d = WtSurveyLogic_Step(&s, &obs);
    CHECK(d.action == WT_SURVEY_ACT_WAIT, "目标不可见应等待");

    CHECK(s.attempts == 0, "以上都不应消耗重试次数（得到 %d）", s.attempts);
}

/** 对不准时必须在限定次数后放弃，不能无限循环 */
static void Test_AimRetry_GivesUp(void)
{
    WtSurveyLogic s;
    WtSurveyConfig cfg = WtSurveyLogic_DefaultConfig();
    uint32_t now = 1000;
    int i;

    printf("Test_AimRetry_GivesUp\n");

    cfg.maxAttemptsPerTarget = 3;
    WtSurveyLogic_Begin(&s, &cfg, now);

    /* 反复喂"对不准"的观测，每轮走完 AIMING → SETTLING → AIMING */
    for (i = 0; i < 20 && !WtSurveyLogic_IsFinished(&s); i++) {
        WtSurveyObs obs = MakeObsOffCenter(now);

        (void)WtSurveyLogic_Step(&s, &obs);
        now += cfg.settleMs;

        if (s.state == WT_SURVEY_SETTLING) {
            obs = MakeObsOffCenter(now);
            (void)WtSurveyLogic_Step(&s, &obs);
        }
        if (s.state == WT_SURVEY_LASING) {
            /* 模拟激光也打不中 */
            WtSurveyLogic_LaserFailed(&s, now);
        }
        now += 100;
    }

    CHECK(WtSurveyLogic_IsFinished(&s), "应在限定次数后结束，而不是无限循环");
    CHECK(s.state == WT_SURVEY_FAILED, "应判为失败");
}

/** 四个点依次采齐后进入拟合，顺序就是编号 */
static void Test_FullSequence_ReachesFit(void)
{
    WtSurveyLogic s;
    uint32_t now = 1000;
    int target;

    printf("Test_FullSequence_ReachesFit\n");

    WtSurveyLogic_Begin(&s, NULL, now);

    for (target = 0; target < WT_SURVEY_TARGET_COUNT; target++) {
        WtSurveyObs obs;
        WtSurveyDecision d;

        CHECK(s.targetIndex == target, "应在采第 %d 个目标（得到 %d）",
              target, s.targetIndex);

        /* 对准 */
        obs = MakeObsCentered(now);
        (void)WtSurveyLogic_Step(&s, &obs);

        /* 等停稳 */
        now += 2000;
        obs = MakeObsCentered(now);
        d = WtSurveyLogic_Step(&s, &obs);
        CHECK(d.action == WT_SURVEY_ACT_READ_LASER, "应到读激光");

        /* 激光成功 */
        WtSurveyLogic_RecordPoint(&s, now);
        now += 100;
    }

    CHECK(s.state == WT_SURVEY_FITTING, "四点采齐后应进入拟合");
    {
        WtSurveyObs obs = MakeObsCentered(now);
        WtSurveyDecision d = WtSurveyLogic_Step(&s, &obs);

        CHECK(d.action == WT_SURVEY_ACT_FIT, "应要求 app 层做拟合");
    }

    /* 拟合通过 → DONE */
    WtSurveyLogic_FitDone(&s, true, NULL);
    CHECK(s.state == WT_SURVEY_DONE, "拟合通过应完成");
    CHECK(WtSurveyLogic_IsFinished(&s), "应判为已结束");
}

/** 拟合未通过阈值 → 失败，且原因要带出来 */
static void Test_FitRejected_Fails(void)
{
    WtSurveyLogic s;

    printf("Test_FitRejected_Fails\n");

    WtSurveyLogic_Begin(&s, NULL, 1000);
    WtSurveyLogic_FitDone(&s, false, "radius spread too large");

    CHECK(s.state == WT_SURVEY_FAILED, "应判为失败");
    CHECK(s.message != NULL && strstr(s.message, "radius spread") != NULL,
          "失败原因应被保留（得到 %s）", s.message ? s.message : "(null)");
}

/** 激光失败应退回对准（指向可能已偏），而不是原地重读 */
static void Test_LaserFail_ReturnsToAiming(void)
{
    WtSurveyLogic s;
    WtSurveyConfig cfg = WtSurveyLogic_DefaultConfig();

    printf("Test_LaserFail_ReturnsToAiming\n");

    cfg.maxAttemptsPerTarget = 5;
    WtSurveyLogic_Begin(&s, &cfg, 1000);
    s.state = WT_SURVEY_LASING;

    WtSurveyLogic_LaserFailed(&s, 2000);

    CHECK(s.state == WT_SURVEY_AIMING, "应退回对准");
    CHECK(s.attempts == 1, "应消耗一次重试");
    CHECK(s.targetIndex == WT_SURVEY_TARGET_HUB, "目标不应前进");
}

/** 中止与结束后再 Step 不应有动作 */
static void Test_AbortAndFinished(void)
{
    WtSurveyLogic s;
    WtSurveyObs obs = MakeObsCentered(1000);
    WtSurveyDecision d;

    printf("Test_AbortAndFinished\n");

    WtSurveyLogic_Begin(&s, NULL, 1000);
    WtSurveyLogic_Abort(&s, "操作员中止");
    CHECK(s.state == WT_SURVEY_FAILED, "中止应判为失败");

    d = WtSurveyLogic_Step(&s, &obs);
    CHECK(d.action == WT_SURVEY_ACT_FINISH, "结束后应只报 FINISH");

    /* 结束后的回调不应把状态改回去 */
    WtSurveyLogic_RecordPoint(&s, 2000);
    CHECK(s.state == WT_SURVEY_FAILED, "结束后 RecordPoint 不应复活会话");
    WtSurveyLogic_LaserFailed(&s, 2000);
    CHECK(s.state == WT_SURVEY_FAILED, "结束后 LaserFailed 不应复活会话");

    CHECK(WtSurveyLogic_Step(NULL, &obs).action == WT_SURVEY_ACT_NONE,
          "NULL 状态应安全返回");
    CHECK(WtSurveyLogic_Step(&s, NULL).action == WT_SURVEY_ACT_NONE,
          "NULL 观测应安全返回");
}

/**
 * 拿不到机头朝向时，下发的 yaw 必须是**当前云台 yaw**，绝不能是 0。
 *
 * 这是整条链路最致命的一个点：绝对角模式下 yaw=0 意为"转到正北"，而 M4T
 * 的 pan 只有相对机头 ±60° —— 机头不朝北时正北够不到，**整条命令被拒**
 * （0x600000004 YAW_REACH_POSITIVE_LIMIT），连 pitch 都不会转到位。
 * 姊妹项目 2026-09-27 实测踩过。
 */
static void Test_NoBodyYaw_KeepsCurrentYaw(void)
{
    WtSurveyLogic s;
    WtSurveyObs obs = MakeObsOffCenter(1000);
    WtSurveyDecision d;

    printf("Test_NoBodyYaw_KeepsCurrentYaw\n");

    WtSurveyLogic_Begin(&s, NULL, 1000);

    /* 模拟拿不到机头朝向（姿态话题还没来） */
    obs.haveBodyYaw = false;
    obs.gimbal.yawDeg = 137.0;

    d = WtSurveyLogic_Step(&s, &obs);

    CHECK(d.action == WT_SURVEY_ACT_ROTATE_GIMBAL, "仍应下发（只调 pitch）");
    CHECK(fabs(d.gimbalYawDeg - 137.0) < 1e-9,
          "yaw 必须保持当前云台值 137°，而不是 0（得到 %.1f）", d.gimbalYawDeg);
    CHECK(fabs(d.gimbalYawDeg) > 1e-6,
          "yaw 绝不能是 0 —— 那是'转到正北'，会让整条命令被拒");
}

/**
 * 目标超出云台相对机头 ±60° → 要求转机头，且**不消耗重试次数**。
 *
 * 人转机头需要时间，按对准重试计数会在几秒内判死。
 */
static void Test_OutOfRange_AsksForBodyYaw(void)
{
    WtSurveyLogic s;
    WtSurveyObs obs;
    WtSurveyDecision d;
    int i;

    printf("Test_OutOfRange_AsksForBodyYaw\n");

    WtSurveyLogic_Begin(&s, NULL, 1000);

    /* 云台已指向 100°，而机头朝 0° → 相对角 100° > 60° */
    obs = MakeObsCentered(1000);
    obs.gimbal.yawDeg = 100.0;
    obs.bodyYawDeg = 0.0;

    d = WtSurveyLogic_Step(&s, &obs);

    CHECK(d.action == WT_SURVEY_ACT_NEED_BODY_YAW, "应要求转机头");
    CHECK(s.attempts == 0, "等人转机头不应消耗重试次数（得到 %d）", s.attempts);
    CHECK(s.state == WT_SURVEY_AIMING, "应留在对准状态");

    /* 持续超范围但未超时 → 一直要求转机头，不判死 */
    for (i = 0; i < 20; i++) {
        obs = MakeObsCentered(1000 + (uint32_t)i * 1000);
        obs.gimbal.yawDeg = 100.0;
        obs.bodyYawDeg = 0.0;
        d = WtSurveyLogic_Step(&s, &obs);
    }
    CHECK(!WtSurveyLogic_IsFinished(&s), "20 秒内不应判死");

    /* 超过 outOfRangeTimeoutMs → 判死，避免永久挂住 */
    obs = MakeObsCentered(1000 + s.cfg.outOfRangeTimeoutMs + 1);
    obs.gimbal.yawDeg = 100.0;
    obs.bodyYawDeg = 0.0;
    d = WtSurveyLogic_Step(&s, &obs);

    CHECK(d.action == WT_SURVEY_ACT_FINISH, "超时后应结束");
    CHECK(s.state == WT_SURVEY_FAILED, "应判为失败");
}

/** 机头转过来之后，超范围计时要清零，能正常继续 */
static void Test_OutOfRange_RecoversAfterBodyTurn(void)
{
    WtSurveyLogic s;
    WtSurveyObs obs;
    WtSurveyDecision d;

    printf("Test_OutOfRange_RecoversAfterBodyTurn\n");

    WtSurveyLogic_Begin(&s, NULL, 1000);

    obs = MakeObsCentered(1000);
    obs.gimbal.yawDeg = 100.0;
    obs.bodyYawDeg = 0.0;
    d = WtSurveyLogic_Step(&s, &obs);
    CHECK(d.action == WT_SURVEY_ACT_NEED_BODY_YAW, "先应要求转机头");
    CHECK(s.outOfRangeSinceMs != 0, "应开始计时");

    /* 操作员把机头转到 100° → 相对角归零，可达 */
    obs = MakeObsCentered(5000);
    obs.gimbal.yawDeg = 100.0;
    obs.bodyYawDeg = 100.0;
    d = WtSurveyLogic_Step(&s, &obs);

    CHECK(s.outOfRangeSinceMs == 0, "可达后计时应清零");
    CHECK(d.action != WT_SURVEY_ACT_NEED_BODY_YAW, "不应再要求转机头");
}

int main(void)
{
    printf("=== 采集决策层测试 ===\n");

    Test_OnTarget_DoesNotRotate();
    Test_OffTarget_Rotates();
    Test_Settling_WaitsFullDuration();
    Test_Settling_SurvivesWraparound();
    Test_StaleSnapshot_Waits();
    Test_MissingInputs_Wait();
    Test_AimRetry_GivesUp();
    Test_FullSequence_ReachesFit();
    Test_FitRejected_Fails();
    Test_LaserFail_ReturnsToAiming();
    Test_NoBodyYaw_KeepsCurrentYaw();
    Test_OutOfRange_AsksForBodyYaw();
    Test_OutOfRange_RecoversAfterBodyTurn();
    Test_AbortAndFinished();

    if (s_failures == 0) {
        printf("全部通过\n");
        return 0;
    }
    printf("%d 项失败\n", s_failures);
    return 1;
}
