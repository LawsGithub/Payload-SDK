/**
 * @file wt_square_mission.c
 * @brief 绕行人正方形航线的 PSDK 执行层实现
 */

#include "wt_square_mission.h"

#include <stdio.h>
#include <stdlib.h>

#include <dji_logger.h>
#include <dji_typedef.h>
#include <dji_waypoint_v3.h>

/* KMZ 只有几十 KB；设个上限，读到离谱的大小直接拒绝 */
#define WT_SQUARE_KMZ_MAX_BYTES (4 * 1024 * 1024)

/* wt_core 里重抄的状态值必须与 SDK 一致（见 wt_mission_watch.h） */
typedef char WtSquareMission_StateCheck[
    (DJI_WAYPOINT_V3_MISSION_STATE_IDLE == WT_MW_STATE_IDLE &&
     DJI_WAYPOINT_V3_MISSION_STATE_PREPARE == WT_MW_STATE_PREPARE &&
     DJI_WAYPOINT_V3_MISSION_STATE_TRANS_MISSION == WT_MW_STATE_TRANS_MISSION &&
     DJI_WAYPOINT_V3_MISSION_STATE_MISSION == WT_MW_STATE_MISSION &&
     DJI_WAYPOINT_V3_MISSION_STATE_BREAK == WT_MW_STATE_BREAK &&
     DJI_WAYPOINT_V3_MISSION_STATE_RESUME == WT_MW_STATE_RESUME &&
     DJI_WAYPOINT_V3_MISSION_STATE_RETURN_FIRSTPOINT == WT_MW_STATE_RETURN_FIRSTPOINT)
        ? 1 : -1];

/*
 * 任务状态回调的缓存。回调跑在 PSDK 线程上，只写这三个量；
 * 判定与日志都在主循环里做（WtSquareMission_Poll）。
 */
static volatile int s_cbState = -1;
static volatile int s_cbIndex = -1;
static volatile uint32_t s_cbSeq = 0;
static uint32_t s_seenSeq = 0;
static int s_loggedState = -2;
static int s_loggedIndex = -2;
static bool s_inited = false;

static T_DjiReturnCode WtSquareMission_OnState(T_DjiWaypointV3MissionState state)
{
    s_cbState = (int)state.state;
    s_cbIndex = (int)state.currentWaypointIndex;
    s_cbSeq++;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static bool WtSquareMission_ReadFile(const char *path, uint8_t **outBuf, uint32_t *outLen)
{
    FILE *fp = fopen(path, "rb");
    long size;
    uint8_t *buf;

    if (fp == NULL) {
        USER_LOG_ERROR("绕点飞行：打不开 KMZ %s", path);
        return false;
    }
    if (fseek(fp, 0, SEEK_END) != 0 || (size = ftell(fp)) <= 0 ||
        size > WT_SQUARE_KMZ_MAX_BYTES || fseek(fp, 0, SEEK_SET) != 0) {
        USER_LOG_ERROR("绕点飞行：KMZ 大小异常 %s", path);
        fclose(fp);
        return false;
    }

    buf = malloc((size_t)size);
    if (buf == NULL || fread(buf, 1, (size_t)size, fp) != (size_t)size) {
        USER_LOG_ERROR("绕点飞行：KMZ 读取不完整 %s", path);
        free(buf);
        fclose(fp);
        return false;
    }

    fclose(fp);
    *outBuf = buf;
    *outLen = (uint32_t)size;

    return true;
}

bool WtSquareMission_Begin(const char *kmzPath, bool start)
{
    T_DjiReturnCode rc;
    uint8_t *buf = NULL;
    uint32_t len = 0;

    if (!WtSquareMission_ReadFile(kmzPath, &buf, &len)) {
        return false;
    }

    rc = DjiWaypointV3_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("绕点飞行：DjiWaypointV3_Init 失败 rc=0x%08llX", (unsigned long long)rc);
        free(buf);
        return false;
    }
    s_inited = true;

    /* 回调要在上传之前注册：任务可能在 START 的瞬间就开始推状态 */
    s_cbState = -1;
    s_cbIndex = -1;
    s_seenSeq = s_cbSeq;
    s_loggedState = -2;
    s_loggedIndex = -2;
    rc = DjiWaypointV3_RegMissionStateCallback(WtSquareMission_OnState);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && start) {
        /*
         * 要起飞就必须有状态回调：监控循环靠它判断「进入航线了没有」，收不到
         * 任何状态会在 30 s 后判为没启动、下发 STOP —— 等于把一条正常飞行中的
         * 航线停在半空。干跑不起飞，不受影响。
         */
        USER_LOG_ERROR("绕点飞行：注册任务状态回调失败 rc=0x%08llX，无法监控，不启动",
                       (unsigned long long)rc);
        free(buf);
        WtSquareMission_End();
        return false;
    }

    rc = DjiWaypointV3_UploadKmzFile(buf, len);
    free(buf);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("绕点飞行：飞机拒收 KMZ rc=0x%08llX（上传阶段）", (unsigned long long)rc);
        WtSquareMission_End();
        return false;
    }
    USER_LOG_INFO("绕点飞行：KMZ 已上传（%u 字节，MD5 由 PSDK 内部校验）", (unsigned)len);

    if (!start) {
        return true;
    }

    rc = DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_START);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        /*
         * 上传成功、START 被拒：问题在启动条件（档位、定位、起飞点……）或文件内容 ——
         * 内容校验发生在 START 时（姊妹项目的结论）。真正原因在 SDK 日志里那行
         * `error_code`，T_DjiReturnCode 是 64 位，模块号在高 32 位，用 %llX 打全。
         */
        USER_LOG_ERROR("绕点飞行：START 被拒 rc=0x%08llX —— 去 SDK 日志里找 error_code",
                       (unsigned long long)rc);
        WtSquareMission_End();
        return false;
    }
    USER_LOG_INFO("绕点飞行：航点任务已启动");

    return true;
}

void WtSquareMission_Poll(WtMissionWatch *watch, uint32_t nowMs)
{
    uint32_t seq1;
    uint32_t seq2;
    int state;
    int index;

    seq1 = s_cbSeq;
    if (seq1 == s_seenSeq) {
        return;
    }
    state = s_cbState;
    index = s_cbIndex;
    seq2 = s_cbSeq;
    if (seq1 != seq2) {
        /* 读的过程中回调又写了一次：状态与航点号可能不配套，下一拍再取 */
        return;
    }
    s_seenSeq = seq1;

    if (state != s_loggedState || index != s_loggedIndex) {
        USER_LOG_INFO("绕点飞行：任务状态 %d，航点号 %d", state, index);
        s_loggedState = state;
        s_loggedIndex = index;
    }
    WtMissionWatch_OnState(watch, state, index, nowMs);
}

bool WtSquareMission_Stop(void)
{
    T_DjiReturnCode rc = DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_STOP);

    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        /*
         * 失败必须让人看见（姊妹项目的教训：STOP 被拒却没人看，操作员以为停了）。
         * 任务已经不在跑时也会返回失败 —— 区分不了，如实上报。
         */
        USER_LOG_ERROR("绕点飞行：STOP 被拒 rc=0x%08llX —— 请用遥控器暂停键或 Pilot 2 接管",
                       (unsigned long long)rc);
        return false;
    }
    USER_LOG_WARN("绕点飞行：已下发 STOP，飞机退出航线、原地悬停");

    return true;
}

void WtSquareMission_End(void)
{
    if (!s_inited) {
        return;
    }
    (void)DjiWaypointV3_DeInit();
    s_inited = false;
}
