/**
 * @file wt_ped.c
 * @brief 行人瞄准打点的接线层：采观测 → 调决策层 → 执行动作 → 记结果。
 *
 * 判据与状态转移都在 `wt_ped_logic`（wt_core），本文件只做副作用。
 */

#include "wt_ped.h"

#include "wt_camera.h"
#include "wt_runner.h"
#include "wt_vision.h"

#include <dji_logger.h>
#include <dji_platform.h>

#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* 决策层一帧收的候选不能少于推理封装一帧给的，否则人群里会被截掉 */
typedef char WtPed_CandidateCapacityCheck[(WT_PED_MAX_CANDIDATES >= WT_TRT_MAX_DETECTIONS) ? 1 : -1];

/* 变焦倍率多久读一次：同步调用，没必要每拍都读 */
#define WT_PED_ZOOM_POLL_MS 2000u
/* 状态没变时，多久打一行进度 */
#define WT_PED_STATUS_LOG_MS 2000u
/*
 * 粗估距离用的身高。**只用于日志里和激光读数对照**（激光打到远处背景时，
 * 读数会远大于这个估值），不参与任何判据 —— 俯视时框高被压缩，估值偏大。
 */
#define WT_PED_ASSUMED_HEIGHT_M 1.7
/*
 * 连读激光的间隔 ms。WtLaser_Read 按 5 Hz 限频，间隔不足 250 ms 拿到的是
 * 上一次的值（看起来读了 3 次，其实是同一个数）—— 留出余量取 300 ms。
 */
#define WT_PED_LASER_GAP_MS 300u

/*
 * 发布给 AI 叠加的会话状态：会话跑在主循环线程，叠加在 SDK 图像回调线程里读。
 * 只存标「目标」要用的几个量，持锁只做结构体拷贝。
 */
typedef struct {
    bool haveCfg;
    WtPedConfig cfg;
    double zoomFactor;      /*!< 最近一次会话读到的变焦倍率（还没有会话过按 1） */
    bool sessionActive;
    bool trackActive;
    double trackU;          /*!< 跟踪目标最近一次关联上时的框（原图像素） */
    double trackV;
    double trackW;
    double trackH;
} WtPedPublished;

static pthread_mutex_t s_pubLock = PTHREAD_MUTEX_INITIALIZER;
static WtPedPublished s_pub = {.zoomFactor = 1.0};

static uint32_t WtPed_NowMs(void)
{
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    uint32_t ms = 0;

    if (osal != NULL && osal->GetTimeMs != NULL) {
        osal->GetTimeMs(&ms);
    }

    return ms;
}

static const char *WtPed_StateName(E_WtPedState st)
{
    switch (st) {
    case WT_PED_SEARCHING:
        return "搜索";
    case WT_PED_AIMING:
        return "对准";
    case WT_PED_LOCKED:
        return "锁定";
    case WT_PED_LASING:
        return "测距";
    case WT_PED_DONE:
        return "完成";
    case WT_PED_FAILED:
        return "失败";
    case WT_PED_IDLE:
    default:
        return "空闲";
    }
}

/** 读变焦倍率；变了就打一行，让现场能核对"视场按几倍算的" */
static void WtPed_PollZoom(WtPedSession *s, uint32_t now, bool force)
{
    double z = 0.0;

    if (!force && s->zoomEverRead && (now - s->lastZoomPollMs) < WT_PED_ZOOM_POLL_MS) {
        return;
    }
    s->lastZoomPollMs = now;

    if (!WtLaser_GetOpticalZoom(&z) || !isfinite(z) || z <= 0.0) {
        if (!s->zoomEverRead) {
            USER_LOG_WARN("行人瞄准：读不到变焦倍率，视场按 %.1f× 计 —— 会话中请不要变焦",
                          s->zoomFactor);
        }
        s->zoomEverRead = true;
        return;
    }

    if (!s->zoomEverRead || fabs(z - s->zoomFactor) > 0.05) {
        const WtCameraGeometry g = WtCamera_ComputeZoomedGeometry(&WT_CAMERA_M4T_WIDE, z);

        USER_LOG_INFO("行人瞄准：变焦 %.2f× → 水平视场 %.2f°（按广角 24 mm × 倍率换算，"
                      "取流画面是否随变焦尚未上机核实）", z, g.fovHorizDeg);
    }
    s->zoomFactor = z;
    s->zoomEverRead = true;
}

/** 把会话状态发布给 AI 叠加（WtPed_TargetIndex） */
static void WtPed_Publish(const WtPedSession *s)
{
    pthread_mutex_lock(&s_pubLock);
    s_pub.haveCfg = true;
    s_pub.cfg = s->logic.cfg;
    s_pub.zoomFactor = s->zoomFactor;
    s_pub.sessionActive = !WtPedLogic_IsFinished(&s->logic);
    s_pub.trackActive = s->logic.track.active;
    s_pub.trackU = s->logic.track.u;
    s_pub.trackV = s->logic.track.v;
    s_pub.trackW = s->logic.track.widthPx;
    s_pub.trackH = s->logic.track.heightPx;
    pthread_mutex_unlock(&s_pubLock);
}

/** 快照 → 观测里的画面部分（候选框、画面尺寸、帧序号）。会话与 AI 叠加的预览共用这一份 */
static void WtPed_FillFrame(const WtVisionSnapshot *snap, WtPedObs *obs)
{
    int n = snap->raw.count;
    int i;

    if (n > WT_PED_MAX_CANDIDATES) {
        n = WT_PED_MAX_CANDIDATES;
    }

    obs->haveFrame = true;
    obs->frameId = snap->seq;              /* 用快照序号判新帧，不用相机帧号 */
    obs->frameCapturedMs = snap->capturedAtMs;
    obs->imageWidthPx = snap->imageWidthPx;
    obs->imageHeightPx = snap->imageHeightPx;

    for (i = 0; i < n; i++) {
        const WtTrtDetection *b = &snap->raw.items[i];
        WtPedCandidate *c = &obs->cand[i];

        /* 瞄框中心：俯视时它就落在人身上（头肩一带） */
        c->u = 0.5 * ((double)b->bboxX1 + (double)b->bboxX2);
        c->v = 0.5 * ((double)b->bboxY1 + (double)b->bboxY2);
        c->widthPx = (double)(b->bboxX2 - b->bboxX1);
        c->heightPx = (double)(b->bboxY2 - b->bboxY1);
        c->conf = (double)b->conf;
    }
    obs->candidateCount = n;
}

/** 组装决策层要的观测 */
static void WtPed_BuildObs(const WtPedSession *s, const WtTelemetry *tele, uint32_t now,
                           WtPedObs *obs)
{
    WtVisionSnapshot snap;
    WtCameraGeometry g;

    memset(obs, 0, sizeof(*obs));
    obs->nowMs = now;

    if (tele != NULL && tele->gimbal.valid) {
        obs->haveGimbal = true;
        obs->gimbal.yawDeg = tele->gimbal.yawDeg;
        obs->gimbal.pitchDeg = tele->gimbal.pitchDeg;
        obs->gimbal.rollDeg = tele->gimbal.rollDeg;
    }
    /* 机头朝向：判 pan 可达性必须有它（限位是相对机头的 ±60°） */
    if (tele != NULL && tele->aircraft.validAttitude) {
        obs->haveBodyYaw = true;
        obs->bodyYawDeg = tele->aircraft.yawDeg;
    }

    /*
     * FOV 由相机模型 × 变焦倍率算出，**不写字面量**。DJI 规格页的 84° 是
     * 对角线 FOV，水平是 71.59° —— 把 84 当水平会让云台每次多转约 17%。
     */
    g = WtCamera_ComputeZoomedGeometry(&WT_CAMERA_M4T_WIDE, s->zoomFactor);
    obs->fovHorizDeg = g.fovHorizDeg;
    obs->fovVertDeg = g.fovVertDeg;

    if (WtVision_GetSnapshot(&snap)) {
        WtPed_FillFrame(&snap, obs);
    }
}

/** 结果追加到 CSV；新文件先写表头 */
static void WtPed_AppendCsv(const WtPedSession *s, uint32_t now)
{
    FILE *fp = fopen(s->csvPath, "a");

    if (fp == NULL) {
        USER_LOG_WARN("行人瞄准：结果文件 %s 打不开，结果只在日志里", s->csvPath);
        return;
    }

    fseek(fp, 0, SEEK_END);
    if (ftell(fp) == 0) {
        /*
         * wall_time 取设备时钟：妙算3 的 RTC 没有电池，掉电回 1970、平时也可能
         * 落后几十小时（进一次 Pilot 2 飞行界面会校时）。按行顺序认先后，别信它。
         */
        fprintf(fp, "wall_time,osal_ms,lat,lon,laser_alt_m,distance_m,screen_x_pct,"
                    "screen_y_pct,gimbal_pitch_deg,gimbal_yaw_deg,body_yaw_deg,conf,"
                    "box_w_px,box_h_px,err_deg,zoom,laser_n,laser_spread_m\n");
    }

    fprintf(fp, "%ld,%u,%.8f,%.8f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.3f,%.0f,%.0f,%.3f,%.2f,"
                "%d,%.2f\n",
            (long)time(NULL), (unsigned)now, s->sample.point.lat, s->sample.point.lon,
            s->sample.point.alt, s->sample.distanceM, s->sample.screenXPct,
            s->sample.screenYPct, s->fireGimbalPitchDeg, s->fireGimbalYawDeg,
            s->fireBodyYawDeg, s->fireTrack.conf, s->fireTrack.widthPx,
            s->fireTrack.heightPx, s->fireTrack.errDeg, s->zoomFactor, s->laserCount,
            s->laserSpreadM);
    fclose(fp);
}

/**
 * 连读 WT_PED_LASER_SAMPLES 次激光，挑出代表那次（判据在 WtPedLogic_PickLaserSample）。
 *
 * 阻塞：每次读最多约 1.2 s，加上间隔，最坏 4 s 左右。云台此时处于 FREE 模式、
 * 姿态锁定地面，几次读数打的是同一个方向。
 */
static void WtPed_DoLaser(WtPedSession *s, const WtPedObs *obs, uint32_t now)
{
    WtLaserSample got[WT_PED_LASER_SAMPLES];
    double dist[WT_PED_LASER_SAMPLES];
    WtLaserSample sample;
    double spread = 0.0;
    int n = 0;
    int pick;
    int k;

    memset(&sample, 0, sizeof(sample));
    for (k = 0; k < WT_PED_LASER_SAMPLES; k++) {
        if (k > 0) {
            usleep(WT_PED_LASER_GAP_MS * 1000u);
        }
        if (WtLaser_Read(&sample) && sample.valid) {
            got[n] = sample;
            dist[n] = sample.distanceM;
            n++;
        } else {
            USER_LOG_WARN("行人瞄准：第 %d 轮第 %d 次激光无有效回波（exception=%u）",
                          s->logic.laserAttempts + 1, k + 1, (unsigned)sample.exception);
        }
    }

    pick = WtPedLogic_PickLaserSample(dist, n, &spread);
    if (pick < 0) {
        USER_LOG_WARN("行人瞄准：第 %d 轮激光不可用 —— 有效 %d/%d 次，离散 %.2f m"
                      "（光斑可能压在人边缘，一会儿打到人一会儿打到地面）",
                      s->logic.laserAttempts + 1, n, WT_PED_LASER_SAMPLES, spread);
        for (k = 0; k < n; k++) {
            USER_LOG_WARN("行人瞄准：  读数 %d：%.2f m", k + 1, dist[k]);
        }
        WtPedLogic_LaserFailed(&s->logic, WtPed_NowMs());
        return;
    }
    sample = got[pick];

    s->havePoint = true;
    s->sample = sample;
    s->laserCount = n;
    s->laserSpreadM = spread;
    s->fireTrack = s->logic.track;
    s->fireGimbalPitchDeg = obs->gimbal.pitchDeg;
    s->fireGimbalYawDeg = obs->gimbal.yawDeg;
    s->fireBodyYawDeg = obs->bodyYawDeg;

    WtPedLogic_RecordPoint(&s->logic, now);

    {
        const double hDeg = s->fireTrack.heightPx * obs->fovVertDeg /
                            (obs->imageHeightPx > 0 ? (double)obs->imageHeightPx : 1.0);
        const double est = (hDeg > 0.01) ? WT_PED_ASSUMED_HEIGHT_M / tan(Wt_DegToRad(hDeg)) : 0.0;

        USER_LOG_INFO("行人瞄准：打点完成 —— 距离 %.1f m（%d 次读数离散 %.2f m，取中位），"
                      "落点 (%.7f, %.7f) 海拔读数 %.1f m（基准未文档化，只看相对量）",
                      sample.distanceM, n, spread, sample.point.lat, sample.point.lon,
                      sample.point.alt);
        USER_LOG_INFO("行人瞄准：  激光落在画面 (%.1f%%, %.1f%%)；目标框中心 (%.1f%%, %.1f%%) "
                      "%.0f×%.0f px conf %.2f；按身高 %.1f m 粗估 %.0f m（仅供核对）",
                      sample.screenXPct, sample.screenYPct,
                      obs->imageWidthPx > 0 ? 100.0 * s->fireTrack.u / obs->imageWidthPx : 0.0,
                      obs->imageHeightPx > 0 ? 100.0 * s->fireTrack.v / obs->imageHeightPx : 0.0,
                      s->fireTrack.widthPx, s->fireTrack.heightPx, s->fireTrack.conf,
                      WT_PED_ASSUMED_HEIGHT_M, est);
    }

    WtPed_AppendCsv(s, now);
}

bool WtPed_Begin(WtPedSession *s, const WtPedConfig *cfg, const char *outputDir)
{
    const char *dir = (outputDir != NULL && outputDir[0] != '\0') ? outputDir : "data";
    uint32_t now;

    if (s == NULL) {
        return false;
    }

    memset(s, 0, sizeof(*s));
    now = WtPed_NowMs();
    s->zoomFactor = 1.0;

    WtPedLogic_Begin(&s->logic, cfg, now);
    s->loggedState = s->logic.state;
    s->lastStatusLogMs = now;

    snprintf(s->csvPath, sizeof(s->csvPath), "%s/ped_points.csv", dir);
    (void)WtRunner_MakeDirs(dir);

    /*
     * 借云台 FREE 模式 —— 作用域是**这次会话**。失败不阻断：飞机晃动会带偏
     * 光轴，但锁定判据看的是画面，偏了会重新对准（而且失败本身已告警）。
     */
    (void)WtLaser_BorrowFreeMode();

    WtPed_PollZoom(s, now, true);
    WtPed_Publish(s);

    USER_LOG_INFO("行人瞄准：会话开始 —— 锁定后连续 %d 帧确认、再停 %.1f s 即自动激光打点"
                  "（连读 %d 次取中位），结果写入 %s",
                  s->logic.cfg.confirmFrames, (double)s->logic.cfg.dwellMs / 1000.0,
                  WT_PED_LASER_SAMPLES, s->csvPath);

    return true;
}

void WtPed_Tick(WtPedSession *s, const WtTelemetry *tele)
{
    WtPedObs obs;
    WtPedDecision d;
    uint32_t now;

    if (s == NULL || WtPed_IsFinished(s)) {
        return;
    }

    now = WtPed_NowMs();
    WtPed_PollZoom(s, now, false);
    WtPed_BuildObs(s, tele, now, &obs);

    d = WtPedLogic_Step(&s->logic, &obs);
    s->last = d;

    switch (d.action) {
    case WT_PED_ACT_ROTATE_GIMBAL:
        /*
         * 绝对角下发。yaw 由决策层给：可达时是目标方位，拿不到机头朝向时是
         * 当前云台 yaw（绝不是 0，否则整条命令被拒、连 pitch 都不转）。
         */
        (void)WtLaser_SetGimbalAbs(d.gimbalPitchDeg, d.gimbalYawDeg);
        USER_LOG_INFO("行人瞄准：第 %d 次修正 —— 云台 pitch → %.2f°、yaw → %.2f°"
                      "（偏差 %.2f°，锁定容差 %.2f°；目标框 %.0f×%.0f px conf %.2f）",
                      s->logic.aimAttempts, d.gimbalPitchDeg, d.gimbalYawDeg,
                      d.angularErrorDeg, d.lockTolDeg, s->logic.track.widthPx,
                      s->logic.track.heightPx, s->logic.track.conf);
        break;

    case WT_PED_ACT_NEED_BODY_YAW:
        /* 只提示，**不操纵飞机** —— 转机身要飞控控制权，风险与收益不成比例 */
        if ((now - s->lastStatusLogMs) >= WT_PED_STATUS_LOG_MS) {
            USER_LOG_WARN("行人瞄准：目标超出云台范围 —— 请把机头转 %+.0f°（%s）",
                          d.yawErrorDeg, d.message != NULL ? d.message : "");
            s->lastStatusLogMs = now;
        }
        break;

    case WT_PED_ACT_READ_LASER:
        WtPed_DoLaser(s, &obs, now);
        break;

    case WT_PED_ACT_WAIT:
    case WT_PED_ACT_FINISH:
    case WT_PED_ACT_NONE:
    default:
        break;
    }

    /* 状态变了就打一行；没变时每 2 s 打一行进度 —— 10 Hz 全打会淹没日志 */
    if (s->logic.state != s->loggedState) {
        USER_LOG_INFO("行人瞄准：%s → %s（%s）", WtPed_StateName(s->loggedState),
                      WtPed_StateName(s->logic.state),
                      s->logic.message != NULL ? s->logic.message : "");
        s->loggedState = s->logic.state;
        s->lastStatusLogMs = now;
    } else if ((now - s->lastStatusLogMs) >= WT_PED_STATUS_LOG_MS) {
        USER_LOG_INFO("行人瞄准：%s —— 画面 %d 人，偏差 %.2f°（容差 %.2f°），%s",
                      WtPed_StateName(s->logic.state), d.candidateCount,
                      d.angularErrorDeg, d.lockTolDeg, d.message != NULL ? d.message : "");
        s->lastStatusLogMs = now;
    }

    WtPed_Publish(s);
}

void WtPed_Abort(WtPedSession *s, const char *why)
{
    if (s == NULL) {
        return;
    }
    WtPedLogic_Abort(&s->logic, why);
    WtPed_Publish(s);
}

void WtPed_SetPreviewConfig(const WtPedConfig *cfg)
{
    if (cfg == NULL) {
        return;
    }
    pthread_mutex_lock(&s_pubLock);
    s_pub.cfg = *cfg;
    s_pub.haveCfg = true;
    pthread_mutex_unlock(&s_pubLock);
}

int WtPed_TargetIndex(const WtVisionSnapshot *snap)
{
    WtPedPublished pub;
    WtPedObs obs;
    WtCameraGeometry g;

    if (snap == NULL) {
        return -1;
    }

    /* 调用方在 SDK 回调线程里：抢不到锁就这一轮不标，绝不等（同 wt_frame_pump 的原则） */
    if (pthread_mutex_trylock(&s_pubLock) != 0) {
        return -1;
    }
    pub = s_pub;
    pthread_mutex_unlock(&s_pubLock);

    memset(&obs, 0, sizeof(obs));
    WtPed_FillFrame(snap, &obs);

    if (pub.sessionActive && pub.trackActive) {
        const double gate = (pub.trackW > pub.trackH) ? pub.trackW : pub.trackH;

        return WtPedLogic_NearestCandidate(&obs, pub.trackU, pub.trackV, gate);
    }
    if (!pub.haveCfg) {
        return -1;
    }

    /* 视场按最近一次会话读到的倍率；obs.gimbal 留零位 —— 捕获判据与云台姿态无关 */
    g = WtCamera_ComputeZoomedGeometry(&WT_CAMERA_M4T_WIDE, pub.zoomFactor);
    obs.fovHorizDeg = g.fovHorizDeg;
    obs.fovVertDeg = g.fovVertDeg;

    return WtPedLogic_PickCapture(&pub.cfg, &obs, NULL);
}

bool WtPed_IsFinished(const WtPedSession *s)
{
    return s != NULL && WtPedLogic_IsFinished(&s->logic);
}

void WtPed_FormatProgress(const WtPedSession *s, char *buf, size_t bufLen)
{
    if (buf == NULL || bufLen == 0) {
        return;
    }
    if (s == NULL) {
        snprintf(buf, bufLen, "行人瞄准：(空)");
        return;
    }

    switch (s->logic.state) {
    case WT_PED_DONE:
        snprintf(buf, bufLen, "打点完成 %.1fm (%.6f,%.6f)", s->sample.distanceM,
                 s->sample.point.lat, s->sample.point.lon);
        break;
    case WT_PED_FAILED:
        snprintf(buf, bufLen, "行人瞄准失败 %s",
                 s->logic.message != NULL ? s->logic.message : "");
        break;
    case WT_PED_SEARCHING:
        snprintf(buf, bufLen, "搜索行人 画面%d人", s->last.candidateCount);
        break;
    case WT_PED_LOCKED:
        if (s->last.dwellRequiredMs > 0) {
            snprintf(buf, bufLen, "锁定 确认%d/%d %.1f/%.1fs 偏差%.2f°", s->last.confirmCount,
                     s->last.confirmRequired, (double)s->last.dwellElapsedMs / 1000.0,
                     (double)s->last.dwellRequiredMs / 1000.0, s->last.angularErrorDeg);
        } else {
            snprintf(buf, bufLen, "锁定 确认%d/%d 偏差%.2f°", s->last.confirmCount,
                     s->last.confirmRequired, s->last.angularErrorDeg);
        }
        break;
    case WT_PED_LASING:
        snprintf(buf, bufLen, "激光测距中");
        break;
    case WT_PED_AIMING:
    default:
        /* 要操作员动手的量放最前面：浮窗带宽有限，排后面容易被截断 */
        if (s->last.action == WT_PED_ACT_NEED_BODY_YAW) {
            snprintf(buf, bufLen, "请把机头转%+.0f° 目标超出云台范围", s->last.yawErrorDeg);
        } else {
            snprintf(buf, bufLen, "对准行人 偏差%.2f° 第%d次", s->last.angularErrorDeg,
                     s->logic.aimAttempts);
        }
        break;
    }
}
