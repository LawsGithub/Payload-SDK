/**
 * @file wt_test_ped_logic.c
 * @brief 高空行人瞄准决策层的回归测试。
 *
 * 用一个小仿真代替飞机：云台按有限角速度转向下发的角度（且收到命令后
 * 有响应延迟），画面按**曝光时刻**的云台姿态成像、晚 150 ms 才到达 ——
 * 和真机一样存在取流延迟。在这个前提下钉住：
 *
 *   - 锁定后连续 3 帧（停稳后拍到、在容差内）确认就打；中途有一帧偏出要重数；
 *     同一帧重复给到不算数；配了 dwell（例如 3000）则还要满时长
 *   - 激光几次读数：取距离居中那次；不够 2 次或离散度过大判失败
 *   - 已对准就不再下发（否则云台永远在微动）
 *   - 云台没停稳、或停稳前拍的帧，一律不采信
 *   - 多人时按绝对方向关联、跟丢超时重搜；捕获要更高置信度
 *   - 偏出锁定范围要重新确认、重新计时；激光没打成重打、超次数判失败
 *   - 超出 pan 范围报"转机头"且不耗对准次数；没有机头朝向时 yaw 绝不填 0
 *   - 计时抗 uint32 回绕
 */

#include "wt_ped_logic.h"

#include "wt_camera.h"
#include "wt_test.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define IMG_W 1440
#define IMG_H 1080
#define STEP_MS 100u
#define LATENCY_MS 150u

/* ------------------------------------------------------------------ */
/* 仿真                                                                */
/* ------------------------------------------------------------------ */

typedef struct {
    double az;
    double el;
    double widthPx;
    double heightPx;
    double conf;
    bool visible;
} Person;

typedef struct {
    double yaw;
    double pitch;
    double cmdYaw;
    double cmdPitch;
    bool haveCmd;
    uint32_t cmdAt;
    uint32_t lastT;
    double speedDegS;
    uint32_t responseDelayMs;

    struct {
        uint32_t t;
        double yaw;
        double pitch;
    } hist[512];
    int histN;

    double fovH;
    double fovV;
    double bodyYaw;
    bool haveBodyYaw;
    uint32_t frameId;
} Sim;

static void SimInit(Sim *sim, uint32_t now, double yaw, double pitch)
{
    WtCameraGeometry g = WtCamera_ComputeGeometry(&WT_CAMERA_M4T_WIDE);

    memset(sim, 0, sizeof(*sim));
    sim->yaw = yaw;
    sim->pitch = pitch;
    sim->lastT = now;
    sim->speedDegS = 30.0;
    sim->responseDelayMs = 100;
    sim->fovH = g.fovHorizDeg;
    sim->fovV = g.fovVertDeg;
    sim->bodyYaw = yaw;
    sim->haveBodyYaw = true;
    sim->hist[0].t = now;
    sim->hist[0].yaw = yaw;
    sim->hist[0].pitch = pitch;
    sim->histN = 1;
}

static void SimPushHist(Sim *sim, uint32_t now)
{
    if (sim->histN < (int)(sizeof(sim->hist) / sizeof(sim->hist[0]))) {
        sim->hist[sim->histN].t = now;
        sim->hist[sim->histN].yaw = sim->yaw;
        sim->hist[sim->histN].pitch = sim->pitch;
        sim->histN++;
    } else {
        memmove(&sim->hist[0], &sim->hist[1], sizeof(sim->hist[0]) * (size_t)(sim->histN - 1));
        sim->hist[sim->histN - 1].t = now;
        sim->hist[sim->histN - 1].yaw = sim->yaw;
        sim->hist[sim->histN - 1].pitch = sim->pitch;
    }
}

/** 云台朝下发的角度匀速转（两轴各自限速），推进到 now */
static void SimAdvance(Sim *sim, uint32_t now)
{
    const uint32_t dt = now - sim->lastT;

    if (sim->haveCmd && (now - sim->cmdAt) >= sim->responseDelayMs && dt > 0) {
        const double maxStep = sim->speedDegS * (double)dt / 1000.0;
        double dy = Wt_AngleDiff(sim->cmdYaw, sim->yaw);
        double dp = sim->cmdPitch - sim->pitch;

        dy = Wt_Clamp(dy, -maxStep, maxStep);
        dp = Wt_Clamp(dp, -maxStep, maxStep);
        sim->yaw = Wt_Wrap360(sim->yaw + dy);
        sim->pitch += dp;
    }
    sim->lastT = now;
    SimPushHist(sim, now);
}

static void SimCommand(Sim *sim, uint32_t now, double pitch, double yaw)
{
    sim->haveCmd = true;
    sim->cmdAt = now;
    sim->cmdPitch = pitch;
    sim->cmdYaw = yaw;
}

static WtCameraPose PoseAt(const Sim *sim, uint32_t t)
{
    WtCameraPose p;
    int i;

    memset(&p, 0, sizeof(p));
    p.yawDeg = sim->hist[0].yaw;
    p.pitchDeg = sim->hist[0].pitch;
    for (i = 0; i < sim->histN; i++) {
        if ((int32_t)(t - sim->hist[i].t) >= 0) {
            p.yawDeg = sim->hist[i].yaw;
            p.pitchDeg = sim->hist[i].pitch;
        }
    }

    return p;
}

/**
 * 世界方向 → 像素：对 WtAim_FromPixel 做牛顿迭代求逆。
 * 用被测代码自己的正向换算求逆，测试就不必另写一套相机约定（两套必然漂移）。
 */
static bool PixelFor(const WtCameraPose *pose, double fovH, double fovV,
                     double az, double el, double *outU, double *outV)
{
    double u = IMG_W * 0.5;
    double v = IMG_H * 0.5;
    int it;

    for (it = 0; it < 40; it++) {
        WtAimResult a0;
        WtAimResult au;
        WtAimResult av;
        const double h = 0.5;
        double ex;
        double ey;
        double j11;
        double j12;
        double j21;
        double j22;
        double det;

        if (!WtAim_FromPixel(pose, u, v, IMG_W, IMG_H, fovH, fovV, &a0)) {
            return false;
        }
        ex = Wt_AngleDiff(az, a0.targetYawDeg);
        ey = el - a0.targetPitchDeg;
        if (fabs(ex) < 1e-10 && fabs(ey) < 1e-10) {
            break;
        }
        if (!WtAim_FromPixel(pose, u + h, v, IMG_W, IMG_H, fovH, fovV, &au) ||
            !WtAim_FromPixel(pose, u, v + h, IMG_W, IMG_H, fovH, fovV, &av)) {
            return false;
        }
        j11 = Wt_AngleDiff(au.targetYawDeg, a0.targetYawDeg) / h;
        j12 = Wt_AngleDiff(av.targetYawDeg, a0.targetYawDeg) / h;
        j21 = (au.targetPitchDeg - a0.targetPitchDeg) / h;
        j22 = (av.targetPitchDeg - a0.targetPitchDeg) / h;
        det = j11 * j22 - j12 * j21;
        if (fabs(det) < 1e-15) {
            return false;
        }
        u += (j22 * ex - j12 * ey) / det;
        v += (-j21 * ex + j11 * ey) / det;
    }

    *outU = u;
    *outV = v;

    return u >= 0.0 && u < IMG_W && v >= 0.0 && v < IMG_H;
}

/**
 * 组一帧观测：读回 = 现在的云台姿态；画面 = 曝光时刻（now − 延迟）的姿态下
 * 各人的像素位置，到达时刻 = now。
 */
static void MakeObs(WtPedObs *o, Sim *sim, uint32_t now, const Person *ps, int n,
                    bool newFrame)
{
    const uint32_t exposedAt = now - LATENCY_MS;
    const WtCameraPose poseThen = PoseAt(sim, exposedAt);
    int i;

    memset(o, 0, sizeof(*o));
    o->nowMs = now;
    o->haveGimbal = true;
    o->gimbal.yawDeg = sim->yaw;
    o->gimbal.pitchDeg = sim->pitch;
    o->haveBodyYaw = sim->haveBodyYaw;
    o->bodyYawDeg = sim->bodyYaw;
    o->imageWidthPx = IMG_W;
    o->imageHeightPx = IMG_H;
    o->fovHorizDeg = sim->fovH;
    o->fovVertDeg = sim->fovV;

    if (newFrame) {
        sim->frameId++;
    }
    o->haveFrame = true;
    o->frameId = sim->frameId;
    o->frameCapturedMs = now;

    for (i = 0; i < n && o->candidateCount < WT_PED_MAX_CANDIDATES; i++) {
        double u;
        double v;

        if (!ps[i].visible) {
            continue;
        }
        if (!PixelFor(&poseThen, sim->fovH, sim->fovV, ps[i].az, ps[i].el, &u, &v)) {
            continue;
        }
        o->cand[o->candidateCount].u = u;
        o->cand[o->candidateCount].v = v;
        o->cand[o->candidateCount].widthPx = ps[i].widthPx;
        o->cand[o->candidateCount].heightPx = ps[i].heightPx;
        o->cand[o->candidateCount].conf = ps[i].conf;
        o->candidateCount++;
    }
}

static Person MakePerson(double az, double el)
{
    Person p;

    p.az = az;
    p.el = el;
    p.widthPx = 24.0;   /* 半宽 ≈ 0.6°，半高 ≈ 1.25° → 锁定容差 0.3° */
    p.heightPx = 48.0;
    p.conf = 0.8;
    p.visible = true;

    return p;
}

/** 推进一步：仿真 → 观测 → 决策 → 执行下发 */
static WtPedDecision Tick(WtPedLogic *s, Sim *sim, uint32_t now, const Person *ps, int n)
{
    WtPedObs obs;
    WtPedDecision d;

    SimAdvance(sim, now);
    MakeObs(&obs, sim, now, ps, n, true);
    d = WtPedLogic_Step(s, &obs);
    if (d.action == WT_PED_ACT_ROTATE_GIMBAL) {
        SimCommand(sim, now, d.gimbalPitchDeg, d.gimbalYawDeg);
    }

    return d;
}

/**
 * 只测跟踪 / 关联、不想让它中途开打的用例用这个：停顿拉到 30 s。
 * （默认参数下锁定后约 0.2 s 就会打激光，进了「测距」就不再处理画面。）
 */
static WtPedConfig NoFireConfig(void)
{
    return WtPedLogic_ConfigFrom(30000, 0.0);
}

/* ------------------------------------------------------------------ */
/* 用例                                                                */
/* ------------------------------------------------------------------ */

static void TestDefaults(void)
{
    WtPedConfig c = WtPedLogic_DefaultConfig();

    WT_CASE("默认参数：锁定后连续 3 帧确认就打，不额外停顿（2026-10-25 用户同意）");

    WT_CHECK_EQ_INT(c.confirmFrames, 3);
    WT_CHECK_EQ_INT(c.dwellMs, 0);
    /* 重打要等下一组读数：比激光 5 Hz 限频（250 ms）长 */
    WT_CHECK(c.retryDwellMs > 250u);
    WT_CHECK(c.breakFactor > 1.0);
    WT_CHECK(c.lockMinDeg > 0.0 && c.lockMinDeg < c.lockMaxDeg);
    /* 停稳窗口要覆盖取流延迟，否则转动中拍的帧会被当成停稳后的 */
    WT_CHECK(c.stableWindowMs >= 200u);
}

/**
 * 配置里写的锁定时长必须真的生效 —— 从配置两项构造的参数跑一遍，
 * 打激光的时刻跟着变。只测 ConfigFrom 的字段抓不住"构造了却没用上"。
 */
static void TestConfigFromReachesBehavior(void)
{
    WtPedConfig c = WtPedLogic_ConfigFrom(2500, 0.6);
    WtPedConfig d = WtPedLogic_ConfigFrom(0, 0.0);
    WtPedConfig e = WtPedLogic_ConfigFrom(-5, 0.0);
    WtPedLogic s;
    Sim sim;
    Person p = MakePerson(100.0, -30.0);
    uint32_t now = 1000;
    uint32_t lockAt = 0;
    uint32_t fireAt = 0;
    int i;

    WT_CASE("配置的 ped_dwell_ms / ped_conf 经 ConfigFrom 生效到行为上");

    WT_CHECK_EQ_INT(c.dwellMs, 2500);
    WT_CHECK_NEAR(c.acquireConf, 0.6, 1e-12);
    /* 0 是合法值（不额外停顿）；负值不该出现（解析已拦），回落默认 */
    WT_CHECK_EQ_INT(d.dwellMs, 0);
    WT_CHECK_EQ_INT(e.dwellMs, WtPedLogic_DefaultConfig().dwellMs);
    WT_CHECK_NEAR(d.acquireConf, WtPedLogic_DefaultConfig().acquireConf, 1e-12);

    WtPedLogic_Begin(&s, &c, now);
    SimInit(&sim, now, 100.0, -30.0);
    for (i = 0; i < 100 && fireAt == 0; i++) {
        WtPedDecision dd;

        now += STEP_MS;
        dd = Tick(&s, &sim, now, &p, 1);
        if (lockAt == 0 && s.state == WT_PED_LOCKED) {
            lockAt = now;
        }
        if (dd.action == WT_PED_ACT_READ_LASER) {
            fireAt = now;
        }
    }
    WT_CHECK(lockAt != 0 && fireAt != 0);
    WT_CHECK((uint32_t)(fireAt - lockAt) >= 2500u);
    WT_CHECK((uint32_t)(fireAt - lockAt) <= 2500u + STEP_MS);
}

static void TestLockTolerance(void)
{
    WtPedConfig c = WtPedLogic_DefaultConfig();
    WtCameraGeometry g = WtCamera_ComputeGeometry(&WT_CAMERA_M4T_WIDE);
    const double degPerPxH = g.fovHorizDeg / IMG_W;

    WT_CASE("锁定容差：框半尺寸 × 0.5，夹在 [0.2°, 1.0°]");

    /* 40×80 px：半宽 20 px ≈ 0.994°，较小的是半宽 → 0.497° */
    WT_CHECK_NEAR(WtPedLogic_LockToleranceDeg(&c, 40, 80, IMG_W, IMG_H,
                                              g.fovHorizDeg, g.fovVertDeg),
                  0.5 * 20.0 * degPerPxH, 1e-9);
    /* 高空小人 12×24 px：0.149° → 夹到下限 */
    WT_CHECK_NEAR(WtPedLogic_LockToleranceDeg(&c, 12, 24, IMG_W, IMG_H,
                                              g.fovHorizDeg, g.fovVertDeg),
                  c.lockMinDeg, 1e-12);
    /* 近处大框：夹到上限 */
    WT_CHECK_NEAR(WtPedLogic_LockToleranceDeg(&c, 300, 600, IMG_W, IMG_H,
                                              g.fovHorizDeg, g.fovVertDeg),
                  c.lockMaxDeg, 1e-12);
    /* 取较小的半尺寸：横躺的人（宽 > 高）由半高决定 */
    WT_CHECK_NEAR(WtPedLogic_LockToleranceDeg(&c, 80, 30, IMG_W, IMG_H,
                                              g.fovHorizDeg, g.fovVertDeg),
                  0.5 * 15.0 * g.fovVertDeg / IMG_H, 1e-9);
    WT_CHECK_NEAR(WtPedLogic_LockToleranceDeg(&c, 0, 0, IMG_W, IMG_H,
                                              g.fovHorizDeg, g.fovVertDeg),
                  c.lockMinDeg, 1e-12);
}

static void TestZoomedGeometry(void)
{
    WtCameraGeometry g1 = WtCamera_ComputeGeometry(&WT_CAMERA_M4T_WIDE);
    WtCameraGeometry z1 = WtCamera_ComputeZoomedGeometry(&WT_CAMERA_M4T_WIDE, 1.0);
    WtCameraGeometry z3 = WtCamera_ComputeZoomedGeometry(&WT_CAMERA_M4T_WIDE, 3.0);
    WtCameraGeometry z0 = WtCamera_ComputeZoomedGeometry(&WT_CAMERA_M4T_WIDE, 0.0);
    WtCameraGeometry zn = WtCamera_ComputeZoomedGeometry(&WT_CAMERA_M4T_WIDE, NAN);
    const double expect3 =
        2.0 * atan(tan(Wt_DegToRad(g1.fovHorizDeg) * 0.5) / 3.0) * WT_RAD2DEG;

    WT_CASE("变焦视场：tan(半视场) 按倍率缩小；读失败的 0 / NaN 按 1×");

    WT_CHECK_NEAR(z1.fovHorizDeg, g1.fovHorizDeg, 1e-12);
    WT_CHECK_NEAR(g1.fovHorizDeg, 71.59, 0.01);
    WT_CHECK_NEAR(z3.fovHorizDeg, expect3, 1e-9);
    WT_CHECK(z3.fovHorizDeg < g1.fovHorizDeg / 2.5);   /* 不是简单地除以 3 */
    WT_CHECK_NEAR(z0.fovHorizDeg, g1.fovHorizDeg, 1e-12);
    WT_CHECK_NEAR(zn.fovHorizDeg, g1.fovHorizDeg, 1e-12);
}

/**
 * 目标在右下 4°/3°：先转、停稳确认、锁定，然后按参数打激光。
 * 打的时刻落在 [lockAt + minMs, lockAt + maxMs]。
 */
static void TestHappyPath(uint32_t t0, const WtPedConfig *cfg, uint32_t minMs, uint32_t maxMs)
{
    WtPedLogic s;
    Sim sim;
    Person p = MakePerson(104.0, -33.0);
    uint32_t now = t0;
    uint32_t lockAt = 0;
    uint32_t fireAt = 0;
    bool locked = false;
    bool fired = false;
    int rotations = 0;
    int i;

    WtPedLogic_Begin(&s, cfg, now);
    SimInit(&sim, now, 100.0, -30.0);

    for (i = 0; i < 300 && !fired; i++) {
        WtPedDecision d;

        now += STEP_MS;
        d = Tick(&s, &sim, now, &p, 1);

        if (d.action == WT_PED_ACT_ROTATE_GIMBAL) {
            rotations++;
            /* 下发必须指向目标本身（绝对角），不是增量 */
            WT_CHECK(WtPedLogic_AngleBetweenDeg(d.gimbalYawDeg, d.gimbalPitchDeg,
                                                p.az, p.el) < 0.5);
        }
        if (!locked && s.state == WT_PED_LOCKED) {
            locked = true;
            lockAt = now;
        }
        if (d.action == WT_PED_ACT_READ_LASER) {
            fired = true;
            fireAt = now;
        }
        /* 锁定期间绝不下发云台 */
        if (locked && !fired) {
            WT_CHECK(d.action != WT_PED_ACT_ROTATE_GIMBAL);
        }
    }

    WT_CHECK(locked);
    WT_CHECK(fired);
    WT_CHECK(rotations >= 1 && rotations <= 3);
    /* 不早一拍，也不拖延 */
    WT_CHECK((uint32_t)(fireAt - lockAt) >= minMs);
    WT_CHECK((uint32_t)(fireAt - lockAt) <= maxMs);
    /* 打的时候光轴确实在目标上 */
    WT_CHECK(WtPedLogic_AngleBetweenDeg(sim.yaw, sim.pitch, p.az, p.el) <= s.track.lockTolDeg);

    WtPedLogic_RecordPoint(&s, now);
    WT_CHECK(s.state == WT_PED_DONE);
    WT_CHECK(WtPedLogic_IsFinished(&s));
}

static void TestHappyPathCases(void)
{
    const WtPedConfig def = WtPedLogic_DefaultConfig();
    const WtPedConfig dwell3s = WtPedLogic_ConfigFrom(3000, 0.0);
    /* 每拍一帧新画面：锁定那一帧算第 1 帧，第 confirmFrames 帧那一拍打 */
    const uint32_t confirmMs = (uint32_t)(def.confirmFrames - 1) * STEP_MS;

    WT_CASE("正常流程：转 → 停稳 → 锁定 → 连续 3 帧确认就打（约 0.2 s，不是 3 s）");
    TestHappyPath(1000u, NULL, confirmMs, confirmMs);

    WT_CASE("正常流程跨 uint32 回绕仍按帧确认");
    TestHappyPath(0xFFFFF000u, NULL, confirmMs, confirmMs);

    WT_CASE("配了 ped_dwell_ms = 3000：确认之外还要满 3 s（旧的「3 s 后打」）");
    TestHappyPath(1000u, &dwell3s, 3000u, 3000u + STEP_MS);

    WT_CASE("配了 3000 跨 uint32 回绕仍按 3 s 计");
    TestHappyPath(0xFFFFF000u, &dwell3s, 3000u, 3000u + STEP_MS);
}

/**
 * 确认帧必须是**新的**画面：推理没出新结果时，同一帧会被反复给到决策层，
 * 不能每给一次就数一次 —— 那等于拿一帧当三帧用。
 */
static void TestConfirmNeedsNewFrames(void)
{
    WtPedLogic s;
    Sim sim;
    Person p = MakePerson(100.0, -30.0);
    WtPedObs obs;
    WtPedDecision d;
    uint32_t now = 1000;
    int i;

    WT_CASE("确认帧：同一帧重复给到不算数，新帧到了才算");

    WtPedLogic_Begin(&s, NULL, now);
    SimInit(&sim, now, 100.0, -30.0);
    for (i = 0; i < 30 && s.state != WT_PED_LOCKED; i++) {
        now += STEP_MS;
        (void)Tick(&s, &sim, now, &p, 1);
    }
    WT_CHECK(s.state == WT_PED_LOCKED);
    WT_CHECK_EQ_INT(s.confirmCount, 1);

    /* 十拍都是同一帧（frameId 不变）：不能开打 */
    for (i = 0; i < 10; i++) {
        now += STEP_MS;
        SimAdvance(&sim, now);
        MakeObs(&obs, &sim, now, &p, 1, false);
        d = WtPedLogic_Step(&s, &obs);
        WT_CHECK(d.action != WT_PED_ACT_READ_LASER);
    }
    WT_CHECK_EQ_INT(s.confirmCount, 1);

    /* 新帧来了两张：第 2 张那一拍打 */
    now += STEP_MS;
    d = Tick(&s, &sim, now, &p, 1);
    WT_CHECK(d.action != WT_PED_ACT_READ_LASER);
    WT_CHECK_EQ_INT(d.confirmCount, 2);
    now += STEP_MS;
    d = Tick(&s, &sim, now, &p, 1);
    WT_CHECK(d.action == WT_PED_ACT_READ_LASER);
}

/**
 * 「连续」：确认到一半，有一帧落在锁定容差外（但没到脱锁线）—— 从头数，
 * 不能把前后几帧拼起来凑够。容差 0.3°、脱锁线 0.48°，取 0.4°。
 */
static void TestConfirmStreakResets(void)
{
    WtPedLogic s;
    Sim sim;
    Person p = MakePerson(100.0, -30.0);
    uint32_t now = 1000;
    int framesAfterLock = 0;
    int fireFrame = -1;
    int i;

    WT_CASE("确认帧：中途一帧偏出容差（未脱锁）要从头数");

    WtPedLogic_Begin(&s, NULL, now);
    SimInit(&sim, now, 100.0, -30.0);
    for (i = 0; i < 30 && s.state != WT_PED_LOCKED; i++) {
        now += STEP_MS;
        (void)Tick(&s, &sim, now, &p, 1);
    }
    WT_CHECK(s.state == WT_PED_LOCKED);

    for (i = 0; i < 20 && fireFrame < 0; i++) {
        WtPedDecision d;

        /* 锁定后第 2 帧：人偏 0.4°；之后回到原位 */
        p.az = (framesAfterLock == 1) ? 100.4 : 100.0;
        now += STEP_MS;
        d = Tick(&s, &sim, now, &p, 1);
        framesAfterLock++;
        WT_CHECK(s.state == WT_PED_LOCKED || s.state == WT_PED_LASING);
        if (framesAfterLock == 1) {
            WT_CHECK_EQ_INT(s.confirmCount, 2);
        }
        if (framesAfterLock == 2) {
            WT_CHECK_EQ_INT(s.confirmCount, 0);   /* 偏出的那一帧把计数清零 */
        }
        if (d.action == WT_PED_ACT_READ_LASER) {
            fireFrame = framesAfterLock;
        }
    }
    /* 清零后要再攒 3 帧：锁定后第 5 帧才打（不清零的话第 3 帧就打了） */
    WT_CHECK_EQ_INT(fireFrame, 5);
}

/** 锁定后马上被挡住：确认帧凑不齐，不开打，最后按丢失重搜 */
static void TestNoConfirmWhenOccluded(void)
{
    WtPedLogic s;
    Sim sim;
    Person p = MakePerson(100.0, -30.0);
    uint32_t now = 1000;
    bool locked = false;
    int i;

    WT_CASE("锁定后立刻被遮挡：凑不齐确认帧就不打，超时回到搜索");

    WtPedLogic_Begin(&s, NULL, now);
    SimInit(&sim, now, 100.0, -30.0);
    for (i = 0; i < 100; i++) {
        WtPedDecision d;

        now += STEP_MS;
        if (s.state == WT_PED_LOCKED) {
            locked = true;
            p.visible = false;
        }
        d = Tick(&s, &sim, now, &p, 1);
        WT_CHECK(d.action != WT_PED_ACT_READ_LASER);
        if (locked && s.state == WT_PED_SEARCHING) {
            break;
        }
    }
    WT_CHECK(locked);
    WT_CHECK(s.state == WT_PED_SEARCHING);
}

/** 几次激光读数里挑代表：居中、较近、离散度、样本数 */
static void TestPickLaserSample(void)
{
    double spread = -1.0;

    WT_CASE("激光读数：3 次取距离居中那次；2 次取较近那次");
    {
        const double a[3] = {31.2, 30.9, 31.0};
        const double b[3] = {30.0, 30.4, 30.2};
        const double c[2] = {25.3, 25.1};

        WT_CHECK_EQ_INT(WtPedLogic_PickLaserSample(a, 3, &spread), 2);
        WT_CHECK_NEAR(spread, 0.3, 1e-9);
        WT_CHECK_EQ_INT(WtPedLogic_PickLaserSample(b, 3, NULL), 2);
        WT_CHECK_EQ_INT(WtPedLogic_PickLaserSample(c, 2, &spread), 1);
        WT_CHECK_NEAR(spread, 0.2, 1e-9);
    }

    WT_CASE("激光读数：离散度超限（一会儿打到人一会儿打到地面）判不可用");
    {
        const double edge[3] = {30.0, 31.6, 30.1};      /* 差 1.6 m */
        const double near60[3] = {60.0, 60.55, 60.2};   /* 0.55 m ≤ 60 × 1% */
        const double near20[3] = {20.0, 20.55, 20.2};   /* 0.55 m > max(0.5, 0.2) */
        const double far200[3] = {200.0, 201.9, 200.5}; /* 1.9 m ≤ 200 × 1% */

        WT_CHECK_EQ_INT(WtPedLogic_PickLaserSample(edge, 3, &spread), -1);
        WT_CHECK_NEAR(spread, 1.6, 1e-9);
        WT_CHECK_EQ_INT(WtPedLogic_PickLaserSample(near60, 3, NULL), 2);
        WT_CHECK_EQ_INT(WtPedLogic_PickLaserSample(near20, 3, NULL), -1);
        WT_CHECK_EQ_INT(WtPedLogic_PickLaserSample(far200, 3, NULL), 2);
    }

    WT_CASE("激光读数：不足 2 次、非法值都判不可用");
    {
        const double one[1] = {30.0};
        const double bad[3] = {30.0, NAN, 30.1};
        const double zero[2] = {0.0, 0.1};

        WT_CHECK_EQ_INT(WtPedLogic_PickLaserSample(one, 1, NULL), -1);
        WT_CHECK_EQ_INT(WtPedLogic_PickLaserSample(NULL, 3, NULL), -1);
        WT_CHECK_EQ_INT(WtPedLogic_PickLaserSample(bad, 3, NULL), -1);
        WT_CHECK_EQ_INT(WtPedLogic_PickLaserSample(zero, 2, NULL), -1);
    }
}

/** 目标已在画面中心：不下发，直接锁定 */
static void TestCenteredNoRotate(void)
{
    WtPedLogic s;
    Sim sim;
    Person p = MakePerson(100.0, -30.0);
    uint32_t now = 5000;
    bool fired = false;
    int i;

    WT_CASE("已对准时不下发云台，直接锁定计时");

    WtPedLogic_Begin(&s, NULL, now);
    SimInit(&sim, now, 100.0, -30.0);

    for (i = 0; i < 60 && !fired; i++) {
        WtPedDecision d;

        now += STEP_MS;
        d = Tick(&s, &sim, now, &p, 1);
        WT_CHECK(d.action != WT_PED_ACT_ROTATE_GIMBAL);
        fired = (d.action == WT_PED_ACT_READ_LASER);
    }
    WT_CHECK(fired);
}

/**
 * 停稳之前拍的帧不采信：下发后云台在动，这期间的画面一律不用；
 * 停稳后迟到的旧帧（采集时刻早于停稳）也不能改写目标方向。
 */
static void TestPreSettleFramesIgnored(void)
{
    WtPedLogic s;
    Sim sim;
    Person p = MakePerson(110.0, -30.0);   /* 偏右 10° */
    WtPedObs obs;
    WtPedDecision d;
    uint32_t now = 1000;
    double azBefore;
    int i;
    int rotations = 0;

    WT_CASE("云台转动中与停稳前拍的帧不采信");

    WtPedLogic_Begin(&s, NULL, now);
    SimInit(&sim, now, 100.0, -30.0);

    /* 等到第一次下发 */
    for (i = 0; i < 20 && rotations == 0; i++) {
        now += STEP_MS;
        d = Tick(&s, &sim, now, &p, 1);
        if (d.action == WT_PED_ACT_ROTATE_GIMBAL) {
            rotations++;
        }
    }
    WT_CHECK_EQ_INT(rotations, 1);

    /* 转动期间（30°/s 转 10° 约 330 ms + 响应延迟）：不得再下发 */
    for (i = 0; i < 4; i++) {
        now += STEP_MS;
        d = Tick(&s, &sim, now, &p, 1);
        WT_CHECK(d.action != WT_PED_ACT_ROTATE_GIMBAL);
        WT_CHECK(!s.settled);
    }

    /* 推进到停稳 */
    for (i = 0; i < 20 && !s.settled; i++) {
        now += STEP_MS;
        SimAdvance(&sim, now);
        MakeObs(&obs, &sim, now, &p, 0, false);   /* 不给新帧 */
        (void)WtPedLogic_Step(&s, &obs);
    }
    WT_CHECK(s.settled);

    /* 一帧迟到的旧画面：采集时刻早于停稳，里面的人在"别处" */
    azBefore = s.track.azDeg;
    SimAdvance(&sim, now + 10);
    MakeObs(&obs, &sim, now + 10, &p, 1, true);
    obs.frameCapturedMs = s.settledAtMs - 1;
    obs.cand[0].u += 200.0;
    d = WtPedLogic_Step(&s, &obs);
    WT_CHECK_NEAR(s.track.azDeg, azBefore, 1e-12);
    WT_CHECK(d.action != WT_PED_ACT_ROTATE_GIMBAL);
}

/** 捕获要 0.4，跟踪中 0.3 也认 */
static void TestConfidenceHysteresis(void)
{
    WtPedLogic s;
    Sim sim;
    Person p = MakePerson(100.0, -30.0);
    uint32_t now = 1000;
    uint32_t seenBefore;
    int i;

    WT_CASE("捕获需要更高置信度；跟上之后低一点的帧照样认");

    WtPedLogic_Begin(&s, NULL, now);
    SimInit(&sim, now, 100.0, -30.0);

    p.conf = 0.30;
    for (i = 0; i < 10; i++) {
        now += STEP_MS;
        (void)Tick(&s, &sim, now, &p, 1);
    }
    WT_CHECK(s.state == WT_PED_SEARCHING);
    WT_CHECK(!s.track.active);

    p.conf = 0.50;
    now += STEP_MS;
    (void)Tick(&s, &sim, now, &p, 1);
    WT_CHECK(s.track.active);

    p.conf = 0.30;
    seenBefore = s.track.seenAtMs;
    now += STEP_MS;
    (void)Tick(&s, &sim, now, &p, 1);
    WT_CHECK(s.track.active);
    WT_CHECK(s.track.seenAtMs != seenBefore);
}

/** 多人：跟的是方向最接近的那个；门限外的人不顶替；跟丢超时重搜 */
static void TestAssociationAndLoss(void)
{
    WtPedLogic s;
    Sim sim;
    Person ps[2];
    uint32_t now = 1000;
    int i;

    WT_CASE("关联：门限内取方向最近的；门限外不顶替；停稳后久不见则重搜");

    {
        const WtPedConfig c = NoFireConfig();

        WtPedLogic_Begin(&s, &c, now);
    }
    SimInit(&sim, now, 100.0, -30.0);

    /* 只有 A 在画面中心附近，先锁上 A */
    ps[0] = MakePerson(100.1, -30.05);
    ps[1] = MakePerson(130.0, -30.0);     /* B 在 30° 外 */
    for (i = 0; i < 6; i++) {
        now += STEP_MS;
        (void)Tick(&s, &sim, now, ps, 2);
    }
    WT_CHECK(s.track.active);
    WT_CHECK_NEAR(s.track.azDeg, 100.1, 0.05);

    /* A 往右挪 0.5°，B 挪到离 A 原位 2° 处：应继续跟 A */
    ps[0].az = 100.6;
    ps[1].az = 102.1;
    now += STEP_MS;
    (void)Tick(&s, &sim, now, ps, 2);
    WT_CHECK_NEAR(s.track.azDeg, 100.6, 0.05);

    /*
     * 区分「按方向关联」与「按离画面中心最近」：A 再往右走到 101.0（离上次
     * 0.4°、离光轴 1.0°），B 走到 99.6（离 A 上次 1.0°、离光轴 0.4°）。
     * 跟的必须还是 A —— 按离中心最近会换成 B。
     */
    ps[0].az = 101.0;
    ps[1].az = 99.6;
    now += STEP_MS;
    (void)Tick(&s, &sim, now, ps, 2);
    WT_CHECK_NEAR(s.track.azDeg, 101.0, 0.05);

    /* A 消失，只剩 7.6° 外的 C（超出 5° 门限）：不顶替 */
    ps[0].visible = false;
    ps[1].az = 108.6;
    now += STEP_MS;
    (void)Tick(&s, &sim, now, ps, 2);
    WT_CHECK(s.track.active);
    WT_CHECK_NEAR(s.track.azDeg, 101.0, 0.05);

    /* 继续看不到 A，超过 2.5 s → 放弃，回到搜索 */
    for (i = 0; i < 30 && s.state != WT_PED_SEARCHING; i++) {
        now += STEP_MS;
        ps[1].visible = false;
        (void)Tick(&s, &sim, now, ps, 2);
    }
    WT_CHECK(s.state == WT_PED_SEARCHING);
    WT_CHECK(!s.track.active);
}

/** 锁定中目标偏出 1.6× 容差：回到对准，重新锁上后要重新计满 3 s（配了 3000 时） */
static void TestLockBreakResetsDwell(void)
{
    const WtPedConfig c = WtPedLogic_ConfigFrom(3000, 0.0);
    WtPedLogic s;
    Sim sim;
    Person p = MakePerson(100.0, -30.0);
    uint32_t now = 1000;
    uint32_t relockAt = 0;
    uint32_t fireAt = 0;
    bool broke = false;
    int i;

    WT_CASE("偏出锁定范围要重新计满 3 s，不能接着上一次的计时打");

    WtPedLogic_Begin(&s, &c, now);
    SimInit(&sim, now, 100.0, -30.0);

    for (i = 0; i < 400 && fireAt == 0; i++) {
        WtPedDecision d;
        const E_WtPedState before = s.state;

        now += STEP_MS;
        /* 锁定 2 s 后人走了 1°（容差 0.3°，脱锁线 0.48°） */
        if (!broke && s.state == WT_PED_LOCKED && (now - s.lockSinceMs) >= 2000u) {
            p.az += 1.0;
            broke = true;
        }
        d = Tick(&s, &sim, now, &p, 1);
        if (broke && before != WT_PED_LOCKED && s.state == WT_PED_LOCKED) {
            relockAt = now;
        }
        if (d.action == WT_PED_ACT_READ_LASER) {
            fireAt = now;
        }
    }

    WT_CHECK(broke);
    WT_CHECK(relockAt != 0);
    WT_CHECK(fireAt != 0);
    WT_CHECK((uint32_t)(fireAt - relockAt) >= 3000u);
}

/** 激光无回波：1 s 后重打；三次都没有则判失败 */
static void TestLaserRetryThenFail(void)
{
    WtPedLogic s;
    Sim sim;
    Person p = MakePerson(100.0, -30.0);
    uint32_t now = 1000;
    uint32_t lastFire = 0;
    int fires = 0;
    int i;

    WT_CASE("激光无回波：短暂再确认后重打，超次数判失败");

    WtPedLogic_Begin(&s, NULL, now);
    SimInit(&sim, now, 100.0, -30.0);

    for (i = 0; i < 300 && !WtPedLogic_IsFinished(&s); i++) {
        WtPedDecision d;

        now += STEP_MS;
        d = Tick(&s, &sim, now, &p, 1);
        if (d.action == WT_PED_ACT_READ_LASER) {
            if (fires > 0) {
                /* 重打间隔 = 再确认时间，不是又一个 3 s */
                WT_CHECK((uint32_t)(now - lastFire) >= 1000u);
                WT_CHECK((uint32_t)(now - lastFire) < 3000u);
            }
            fires++;
            lastFire = now;
            WtPedLogic_LaserFailed(&s, now);
        }
    }

    WT_CHECK_EQ_INT(fires, 3);
    WT_CHECK(s.state == WT_PED_FAILED);
}

/**
 * 激光没打成后的重打凭的是**之后**拍到的画面：目标短暂被挡（不到丢失超时），
 * 重新出现后要再攒满确认帧，不能拿打之前那几帧的计数直接开打。
 */
static void TestRetryNeedsFreshConfirm(void)
{
    WtPedLogic s;
    Sim sim;
    Person p = MakePerson(100.0, -30.0);
    uint32_t now = 1000;
    uint32_t failAt = 0;
    uint32_t backAt = 0;
    uint32_t fireAt = 0;
    int i;

    WT_CASE("激光没打成：目标被挡一下再出现，要重新攒满确认帧才重打");

    WtPedLogic_Begin(&s, NULL, now);
    SimInit(&sim, now, 100.0, -30.0);
    for (i = 0; i < 300 && fireAt == 0; i++) {
        WtPedDecision d;

        now += STEP_MS;
        if (failAt != 0) {
            /* 失败后 1.2 s 内看不到他（短于 2.5 s 的丢失超时），之后重新出现 */
            p.visible = (uint32_t)(now - failAt) > 1200u;
            if (p.visible && backAt == 0) {
                backAt = now;
            }
        }
        d = Tick(&s, &sim, now, &p, 1);
        if (d.action == WT_PED_ACT_READ_LASER) {
            if (failAt == 0) {
                failAt = now;
                WtPedLogic_LaserFailed(&s, now);
            } else {
                fireAt = now;
            }
        }
    }
    WT_CHECK(failAt != 0 && backAt != 0 && fireAt != 0);
    /* 重新出现那一帧算第 1 帧，第 3 帧那一拍重打 */
    WT_CHECK_EQ_INT((uint32_t)(fireAt - backAt), 2u * STEP_MS);
}

/** 目标在机头右侧 80°：超出 pan ±60° → 请转机头，且不耗对准次数 */
static void TestOutOfRangeAsksBodyYaw(void)
{
    WtPedLogic s;
    Sim sim;
    Person p = MakePerson(100.0, -30.0);
    uint32_t now = 1000;
    bool asked = false;
    int i;

    WT_CASE("超出云台 pan 范围：报「转机头」及角度，不下发、不耗对准次数");

    WtPedLogic_Begin(&s, NULL, now);
    SimInit(&sim, now, 100.0, -30.0);
    sim.bodyYaw = 20.0;    /* 云台已偏到机头右侧 80° 的位置上看着他 */

    for (i = 0; i < 20; i++) {
        WtPedDecision d;

        now += STEP_MS;
        p.az = 100.6;      /* 稍偏一点，需要修正 */
        d = Tick(&s, &sim, now, &p, 1);
        WT_CHECK(d.action != WT_PED_ACT_ROTATE_GIMBAL);
        if (d.action == WT_PED_ACT_NEED_BODY_YAW) {
            asked = true;
            WT_CHECK_NEAR(d.yawErrorDeg, 80.6, 0.2);
        }
    }
    WT_CHECK(asked);
    WT_CHECK_EQ_INT(s.aimAttempts, 0);
}

/** 没有机头朝向：只调 pitch，yaw 保持当前云台值 —— 绝不填 0 */
static void TestNoBodyYawKeepsCurrentYaw(void)
{
    WtPedLogic s;
    Sim sim;
    Person p = MakePerson(250.0, -40.0);
    uint32_t now = 1000;
    bool rotated = false;
    int i;

    WT_CASE("拿不到机头朝向：yaw 回填当前云台值，绝不是 0");

    WtPedLogic_Begin(&s, NULL, now);
    SimInit(&sim, now, 250.0, -30.0);
    sim.haveBodyYaw = false;

    for (i = 0; i < 20 && !rotated; i++) {
        WtPedDecision d;

        now += STEP_MS;
        d = Tick(&s, &sim, now, &p, 1);
        if (d.action == WT_PED_ACT_ROTATE_GIMBAL) {
            rotated = true;
            WT_CHECK_NEAR(d.gimbalYawDeg, 250.0, 1e-9);
            WT_CHECK_NEAR(d.gimbalPitchDeg, -40.0, 0.05);
        }
    }
    WT_CHECK(rotated);
}

/** 目标一直在跳（每帧挪 2°，门限内）：修正次数用完判失败，而不是无限追 */
static void TestMaxAttemptsFails(void)
{
    WtPedLogic s;
    Sim sim;
    Person p = MakePerson(103.0, -30.0);
    uint32_t now = 1000;
    int rotations = 0;
    int i;

    WT_CASE("追不上（目标持续移动）：修正次数用完判失败");

    WtPedLogic_Begin(&s, NULL, now);
    SimInit(&sim, now, 100.0, -30.0);

    for (i = 0; i < 2000 && !WtPedLogic_IsFinished(&s); i++) {
        WtPedDecision d;

        now += STEP_MS;
        p.az += 0.4;       /* 4°/s，远超能锁住的速度 */
        d = Tick(&s, &sim, now, &p, 1);
        if (d.action == WT_PED_ACT_ROTATE_GIMBAL) {
            rotations++;
        }
        WT_CHECK(d.action != WT_PED_ACT_READ_LASER);
    }

    WT_CHECK(s.state == WT_PED_FAILED);
    WT_CHECK_EQ_INT(rotations, s.cfg.maxAimAttempts);
}

/** 画面里一直没人：会话时限到了判失败，不永久挂住 */
static void TestSessionTimeout(void)
{
    WtPedLogic s;
    Sim sim;
    uint32_t now = 1000;
    int i;

    WT_CASE("一直没有目标：会话超时判失败");

    WtPedLogic_Begin(&s, NULL, now);
    SimInit(&sim, now, 100.0, -30.0);

    for (i = 0; i < 1300 && !WtPedLogic_IsFinished(&s); i++) {
        now += STEP_MS;
        (void)Tick(&s, &sim, now, NULL, 0);
    }
    WT_CHECK(s.state == WT_PED_FAILED);
    WT_CHECK((uint32_t)(now - 1000u) > s.cfg.sessionTimeoutMs);
    WT_CHECK((uint32_t)(now - 1000u) <= s.cfg.sessionTimeoutMs + 2 * STEP_MS);
}

/** 锁定中操作员拨了云台轮：读回在动 → 退回对准 */
static void TestGimbalDisturbedDuringLock(void)
{
    WtPedLogic s;
    Sim sim;
    Person p = MakePerson(100.0, -30.0);
    uint32_t now = 1000;
    int i;

    WT_CASE("锁定中云台被外力转动：退回对准，不在动的时候开打");

    WtPedLogic_Begin(&s, NULL, now);
    SimInit(&sim, now, 100.0, -30.0);

    for (i = 0; i < 30 && s.state != WT_PED_LOCKED; i++) {
        now += STEP_MS;
        (void)Tick(&s, &sim, now, &p, 1);
    }
    WT_CHECK(s.state == WT_PED_LOCKED);

    /* 外力：每 100 ms 转 1°（10°/s） */
    sim.yaw += 1.0;
    now += STEP_MS;
    {
        WtPedDecision d = Tick(&s, &sim, now, &p, 1);

        WT_CHECK(d.action != WT_PED_ACT_READ_LASER);
    }
    WT_CHECK(s.state == WT_PED_AIMING);
}

/** 下发后云台还没开始动（响应延迟）：读回静止也不能当成已停稳 */
static void TestMinSettleAfterCommand(void)
{
    WtPedLogic s;
    Sim sim;
    Person p = MakePerson(110.0, -30.0);
    uint32_t now = 1000;
    uint32_t cmdAt = 0;
    int i;

    WT_CASE("下发后至少等 minSettleMs：响应延迟期间的「静止」不算停稳");

    WtPedLogic_Begin(&s, NULL, now);
    SimInit(&sim, now, 100.0, -30.0);
    sim.responseDelayMs = 10000;   /* 云台迟迟不动 */

    for (i = 0; i < 20 && cmdAt == 0; i++) {
        WtPedDecision d;

        now += STEP_MS;
        d = Tick(&s, &sim, now, &p, 1);
        if (d.action == WT_PED_ACT_ROTATE_GIMBAL) {
            cmdAt = now;
        }
    }
    WT_CHECK(cmdAt != 0);

    while ((uint32_t)(now - cmdAt) + STEP_MS < s.cfg.minSettleMs) {
        now += STEP_MS;
        (void)Tick(&s, &sim, now, &p, 1);
        WT_CHECK(!s.settled);
    }
}

/**
 * 云台慢（3°/s）、一次要转 10°：转动 3 秒多，期间不采信画面。
 * 这段时间不能算作"看不到目标"—— 否则一停稳就判丢失、换人重捕。
 */
static void TestSlowSlewNotLost(void)
{
    WtPedLogic s;
    Sim sim;
    Person p = MakePerson(110.0, -30.0);
    uint32_t now = 1000;
    bool acquired = false;
    bool fired = false;
    int i;

    WT_CASE("慢速长转动期间看不到目标，不算丢失");

    WtPedLogic_Begin(&s, NULL, now);
    SimInit(&sim, now, 100.0, -30.0);
    sim.speedDegS = 3.0;

    for (i = 0; i < 400 && !fired; i++) {
        WtPedDecision d;

        now += STEP_MS;
        d = Tick(&s, &sim, now, &p, 1);
        if (s.track.active) {
            acquired = true;
        }
        if (acquired) {
            WT_CHECK(s.state != WT_PED_SEARCHING);
        }
        fired = (d.action == WT_PED_ACT_READ_LASER);
    }
    WT_CHECK(fired);
}

/**
 * 锁定 1.5 s 后检测不到他了（被树挡住）：计时到点时最近一次观测已是 1.5 s
 * 前 —— 不能凭旧印象开打；再等下去就按丢失重搜。
 */
static void TestNoFireOnStaleObservation(void)
{
    const WtPedConfig c = WtPedLogic_ConfigFrom(3000, 0.0);
    WtPedLogic s;
    Sim sim;
    Person p = MakePerson(100.0, -30.0);
    uint32_t now = 1000;
    bool locked = false;
    int i;

    WT_CASE("锁定中目标被遮挡：计时到了也不凭旧观测开打（配了 3000 时）");

    WtPedLogic_Begin(&s, &c, now);
    SimInit(&sim, now, 100.0, -30.0);

    /* 会话本来就从 SEARCHING 起步 —— 要以"锁定过、又回到搜索"为终点 */
    for (i = 0; i < 300 && !WtPedLogic_IsFinished(&s); i++) {
        WtPedDecision d;

        now += STEP_MS;
        if (s.state == WT_PED_LOCKED) {
            locked = true;
            if ((now - s.lockSinceMs) >= 1500u) {
                p.visible = false;
            }
        }
        d = Tick(&s, &sim, now, &p, 1);
        WT_CHECK(d.action != WT_PED_ACT_READ_LASER);
        if (locked && s.state == WT_PED_SEARCHING) {
            break;
        }
    }
    WT_CHECK(locked);
    WT_CHECK(s.state == WT_PED_SEARCHING);
}

/** 一个静止候选（像素直接给，不走仿真） */
static WtPedCandidate Cand(double u, double v, double conf)
{
    WtPedCandidate c;

    c.u = u;
    c.v = v;
    c.widthPx = 20.0;
    c.heightPx = 40.0;
    c.conf = conf;

    return c;
}

/*
 * 捕获判据单独暴露给 AI 叠加（标出「拨开关会锁谁」）。钉住：置信度门槛（含等于）、
 * 取离光轴最近、夹角相同取靠前、与云台姿态无关。
 */
static void TestPickCapture(void)
{
    const WtPedConfig c = WtPedLogic_DefaultConfig();
    const WtCameraGeometry g = WtCamera_ComputeZoomedGeometry(&WT_CAMERA_M4T_WIDE, 1.0);
    const WtCameraPose poses[3] = {
        {.yawDeg = 0.0, .pitchDeg = 0.0},
        {.yawDeg = 123.0, .pitchDeg = -45.0},
        {.yawDeg = -170.0, .pitchDeg = -80.0, .rollDeg = 2.0},
    };
    WtPedObs o;
    WtAimResult aim;
    int i;

    WT_CASE("捕获判据：够格的里离光轴最近；门槛含等于；同夹角取靠前；与云台姿态无关");

    memset(&o, 0, sizeof(o));
    o.imageWidthPx = IMG_W;
    o.imageHeightPx = IMG_H;
    o.fovHorizDeg = g.fovHorizDeg;
    o.fovVertDeg = g.fovVertDeg;
    o.candidateCount = 3;
    o.cand[0] = Cand(1300.0, 900.0, 0.90);     /* 离中心远 */
    o.cand[1] = Cand(735.0, 548.0, 0.30);      /* 最近，但置信度不够 */
    o.cand[2] = Cand(800.0, 600.0, 0.50);      /* 次近，够格 */

    WT_CHECK_EQ_INT(WtPedLogic_PickCapture(&c, &o, &aim), 2);
    WT_CHECK(aim.angularErrorDeg > 0.0);

    o.cand[1].conf = c.acquireConf;            /* 恰好等于门槛：够格 */
    WT_CHECK_EQ_INT(WtPedLogic_PickCapture(&c, &o, NULL), 1);

    for (i = 0; i < 3; i++) {
        o.gimbal = poses[i];
        WT_CHECK_EQ_INT(WtPedLogic_PickCapture(&c, &o, NULL), 1);
    }

    /*
     * 夹角完全相同时取下标小的。用同一位置的两个框造平局 —— 左右对称的两点在
     * 旋转后会差一个浮点舍入，那不是平局。
     */
    memset(&o.gimbal, 0, sizeof(o.gimbal));
    o.candidateCount = 2;
    o.cand[0] = Cand(900.0, 400.0, 0.9);
    o.cand[1] = Cand(900.0, 400.0, 0.9);
    WT_CHECK_EQ_INT(WtPedLogic_PickCapture(&c, &o, NULL), 0);

    o.cand[0].conf = 0.1;
    o.cand[1].conf = 0.1;
    WT_CHECK_EQ_INT(WtPedLogic_PickCapture(&c, &o, NULL), -1);
    WT_CHECK_EQ_INT(WtPedLogic_PickCapture(NULL, &o, NULL), -1);
}

static void TestNearestCandidate(void)
{
    WtPedObs o;

    WT_CASE("就近匹配：门限内最近的；门限含等于；门限外、NaN、负门限不算");

    memset(&o, 0, sizeof(o));
    o.candidateCount = 3;
    o.cand[0] = Cand(100.0, 100.0, 0.9);
    o.cand[1] = Cand(130.0, 100.0, 0.9);
    o.cand[2] = Cand(500.0, 500.0, 0.9);

    WT_CHECK_EQ_INT(WtPedLogic_NearestCandidate(&o, 125.0, 100.0, 40.0), 1);
    WT_CHECK_EQ_INT(WtPedLogic_NearestCandidate(&o, 110.0, 100.0, 40.0), 0);
    WT_CHECK_EQ_INT(WtPedLogic_NearestCandidate(&o, 300.0, 300.0, 40.0), -1);
    WT_CHECK_EQ_INT(WtPedLogic_NearestCandidate(&o, 500.0, 540.0, 40.0), 2);
    WT_CHECK_EQ_INT(WtPedLogic_NearestCandidate(&o, NAN, 100.0, 40.0), -1);
    WT_CHECK_EQ_INT(WtPedLogic_NearestCandidate(&o, 100.0, 100.0, -1.0), -1);

    o.cand[1].u = NAN;                         /* 坏候选跳过，不影响别人 */
    WT_CHECK_EQ_INT(WtPedLogic_NearestCandidate(&o, 125.0, 100.0, 40.0), 0);
}

/*
 * 会话里真正捕获的人 = 那一帧 PickCapture 选的人 —— AI 叠加标的 target 就不会和
 * 拨开关后实际锁的人对不上。三个人：最远的（够格）排第一、居中的（够格）、
 * 最近的（置信度不够）。「取第一个够格的」或「不看置信度」都会选错。
 */
static void TestSessionCapturesPickCapture(void)
{
    WtPedLogic s;
    Sim sim;
    const WtPedConfig c = NoFireConfig();
    Person ps[3];
    uint32_t now = 5000;
    int captured = -1;
    int i;

    WT_CASE("会话捕获的就是 PickCapture 在那一帧选的人");

    WtPedLogic_Begin(&s, &c, now);
    SimInit(&sim, now, 100.0, -30.0);
    ps[0] = MakePerson(104.0, -32.0);
    ps[1] = MakePerson(101.5, -30.8);
    ps[2] = MakePerson(100.3, -30.2);
    ps[2].conf = 0.30;

    for (i = 0; i < 100 && !s.track.active; i++) {
        WtPedObs o;
        WtPedDecision d;
        int pick;

        now += STEP_MS;
        SimAdvance(&sim, now);
        MakeObs(&o, &sim, now, ps, 3, true);
        pick = WtPedLogic_PickCapture(&s.cfg, &o, NULL);
        d = WtPedLogic_Step(&s, &o);
        if (d.action == WT_PED_ACT_ROTATE_GIMBAL) {
            SimCommand(&sim, now, d.gimbalPitchDeg, d.gimbalYawDeg);
        }
        if (s.track.active) {
            WT_CHECK_EQ_INT(o.candidateCount, 3);
            WT_CHECK(pick >= 0);
            if (pick >= 0) {
                WT_CHECK_NEAR(s.track.u, o.cand[pick].u, 1e-9);
                WT_CHECK_NEAR(s.track.v, o.cand[pick].v, 1e-9);
            }
            captured = pick;
        }
    }

    WT_CHECK(s.track.active);
    WT_CHECK_EQ_INT(captured, 1);
}

int main(void)
{
    TestDefaults();
    TestConfigFromReachesBehavior();
    TestSlowSlewNotLost();
    TestNoFireOnStaleObservation();
    TestLockTolerance();
    TestZoomedGeometry();
    TestHappyPathCases();
    TestConfirmNeedsNewFrames();
    TestConfirmStreakResets();
    TestNoConfirmWhenOccluded();
    TestPickLaserSample();
    TestCenteredNoRotate();
    TestPreSettleFramesIgnored();
    TestConfidenceHysteresis();
    TestAssociationAndLoss();
    TestLockBreakResetsDwell();
    TestLaserRetryThenFail();
    TestRetryNeedsFreshConfirm();
    TestOutOfRangeAsksBodyYaw();
    TestNoBodyYawKeepsCurrentYaw();
    TestMaxAttemptsFails();
    TestSessionTimeout();
    TestGimbalDisturbedDuringLock();
    TestMinSettleAfterCommand();
    TestPickCapture();
    TestNearestCandidate();
    TestSessionCapturesPickCapture();

    return WT_TEST_SUMMARY();
}
