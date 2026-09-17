/**
 * @file wt_bridge.c
 * @brief 规划器 -> PSDK 航点的桥接层实现
 */

#include "wt_bridge.h"

#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define WT_BRIDGE_INIT_CAPACITY 64
#define WT_BRIDGE_PAYLOAD_INDEX 0 /*!< M4T 只有一路云台，载荷位置索引固定为 0 */

/* ================================================================== */
/* 动作与飞行参数                                                      */
/* ================================================================== */

void WtBridge_InitPlan(WtMissionPlan *plan)
{
    memset(plan, 0, sizeof(*plan));
}

void WtBridge_FreePlan(WtMissionPlan *plan)
{
    free(plan->actions.items);
    free(plan->flightParams);
    memset(plan, 0, sizeof(*plan));
}

static WtPlanResult WtBridge_AppendAction(WtActionList *list, const WtWaypointAction *action)
{
    if (list->count == list->capacity) {
        size_t newCap = (list->capacity == 0) ? WT_BRIDGE_INIT_CAPACITY : list->capacity * 2;
        WtWaypointAction *buf = realloc(list->items, newCap * sizeof(WtWaypointAction));

        if (buf == NULL) {
            return WT_PLAN_ERR_NOMEM;
        }
        list->items = buf;
        list->capacity = newCap;
    }

    list->items[list->count] = *action;
    list->count++;

    return WT_PLAN_OK;
}

double WtBridge_HeadingAt(const WtMission *mission, size_t index)
{
    size_t i;

    /* 取包含该点的第一个非退化航段的方位角 */
    for (i = index; i + 1 < mission->count; i++) {
        WtEnu d = WtEnu_Sub(mission->points[i + 1].enu, mission->points[i].enu);

        if (WtEnu_Length(d) > 0.5) {
            return WtEnu_Azimuth(d);
        }
    }
    for (i = index; i > 0; i--) {
        WtEnu d = WtEnu_Sub(mission->points[i].enu, mission->points[i - 1].enu);

        if (WtEnu_Length(d) > 0.5) {
            return WtEnu_Azimuth(d);
        }
    }

    return 0.0;
}

double WtBridge_RelativeAltitudeAt(const WtMission *mission, const WtGeo *takeoffGeo, size_t index)
{
    return mission->points[index].geo.alt - takeoffGeo->alt;
}

WtPlanResult WtBridge_BuildActions(const WtMission *mission,
                                   WtGimbalMode mode,
                                   WtMissionPlan *plan)
{
    size_t i;

    for (i = 0; i < mission->count; i++) {
        const WtPlanPoint *p = &mission->points[i];
        WtWaypointAction a;
        double heading;

        memset(&a, 0, sizeof(a));
        a.waypointIndex = (int)i;
        a.mode = mode;

        /*
         * 云台偏航一律按「绝对方位角」下发（gimbalHeadingYawBase = north）。
         * 若改为相对机头模式，则此处换算成偏角；换算本身简单，但会把
         * 机头朝向的误差耦合进相机指向，故默认不做这层耦合。
         */
        heading = WtBridge_HeadingAt(mission, i);
        a.setGimbal = true;
        a.gimbalYawDeg = p->gimbalYawDeg;
        a.gimbalPitchDeg = p->gimbalPitchDeg;
        (void)heading;

        switch (p->purpose) {
        case WT_PHOTO_DEFECT:
        case WT_PHOTO_COARSE_MODEL:
            a.camera = WT_ACTION_TAKE_PHOTO;
            break;
        case WT_PHOTO_NONE:
        default:
            a.camera = WT_ACTION_NONE;
            break;
        }

        if (WtBridge_AppendAction(&plan->actions, &a) != WT_PLAN_OK) {
            return WT_PLAN_ERR_NOMEM;
        }
    }

    return WT_PLAN_OK;
}

WtPlanResult WtBridge_BuildFlightParams(const WtMission *mission,
                                        const WtGeo *takeoffGeo,
                                        const WtInspectionProfile *profile,
                                        WtMissionPlan *plan)
{
    size_t i;

    plan->flightParams = calloc(mission->count, sizeof(WtWaypointFlightParam));
    if (plan->flightParams == NULL && mission->count > 0) {
        return WT_PLAN_ERR_NOMEM;
    }
    plan->flightParamCount = mission->count;

    for (i = 0; i < mission->count; i++) {
        const WtPlanPoint *p = &mission->points[i];

        /*
         * wpml 的 waypointSpeed 语义是「飞到该航点的速度」，也就是按入边
         * 判定；这里必须与 WtMission_ComputeStats 用同一份判据，否则报告上
         * 的预计耗时与飞机真正怎么飞会对不上。首点没有入边，按巡航速度。
         */
        bool inspecting = (i > 0) &&
                          WtPlan_SegmentUsesInspectSpeed(&mission->points[i - 1], p);

        plan->flightParams[i].waypointIndex = (int)i;
        plan->flightParams[i].relativeAltitudeM = WtBridge_RelativeAltitudeAt(mission, takeoffGeo, i);
        plan->flightParams[i].speedMs = inspecting ? profile->inspectSpeedMs
                                                   : profile->cruiseSpeedMs;
    }

    return WT_PLAN_OK;
}

/* ================================================================== */
/* 最小 ZIP 写入器（store 方式，KMZ 只是换了扩展名的 zip）             */
/* ================================================================== */

typedef struct {
    uint32_t crc;
    uint32_t size;
    uint32_t offset;
    char name[128];
} WtZipEntry;

static uint32_t WtZip_Crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    size_t i;
    int k;

    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (k = 0; k < 8; k++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
        }
    }

    return crc ^ 0xFFFFFFFFu;
}

static void WtZip_PutU16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void WtZip_PutU32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static bool WtZip_WriteEntry(FILE *fp, WtZipEntry *entry, const char *name,
                             const char *content, uint32_t offset)
{
    size_t nameLen = strlen(name);
    size_t dataLen = strlen(content);
    uint8_t hdr[30];

    if (nameLen >= sizeof(entry->name)) {
        return false;
    }

    memset(hdr, 0, sizeof(hdr));
    WtZip_PutU32(hdr + 0, 0x04034b50u); /* 本地文件头签名 */
    WtZip_PutU16(hdr + 4, 20);         /* 解压所需版本 */
    WtZip_PutU16(hdr + 6, 0);          /* 通用标志 */
    WtZip_PutU16(hdr + 8, 0);          /* 压缩方法 0 = store */
    WtZip_PutU16(hdr + 10, 0);         /* 修改时间 */
    WtZip_PutU16(hdr + 12, 0);         /* 修改日期 */
    WtZip_PutU32(hdr + 14, WtZip_Crc32((const uint8_t *)content, dataLen));
    WtZip_PutU32(hdr + 18, (uint32_t)dataLen);
    WtZip_PutU32(hdr + 22, (uint32_t)dataLen);
    WtZip_PutU16(hdr + 26, (uint16_t)nameLen);
    WtZip_PutU16(hdr + 28, 0); /* 扩展字段长度 */

    if (fwrite(hdr, 1, sizeof(hdr), fp) != sizeof(hdr)) {
        return false;
    }
    if (fwrite(name, 1, nameLen, fp) != nameLen) {
        return false;
    }
    if (fwrite(content, 1, dataLen, fp) != dataLen) {
        return false;
    }

    entry->crc = WtZip_Crc32((const uint8_t *)content, dataLen);
    entry->size = (uint32_t)dataLen;
    entry->offset = offset;
    strncpy(entry->name, name, sizeof(entry->name) - 1);
    entry->name[sizeof(entry->name) - 1] = '\0';

    return true;
}

static bool WtZip_WriteCentralDirectory(FILE *fp, const WtZipEntry *entries, int count,
                                        uint32_t cdOffset)
{
    int i;
    uint8_t rec[46];
    uint32_t cdSize = 0;

    for (i = 0; i < count; i++) {
        size_t nameLen = strlen(entries[i].name);

        memset(rec, 0, sizeof(rec));
        WtZip_PutU32(rec + 0, 0x02014b50u); /* 中央目录签名 */
        WtZip_PutU16(rec + 4, 20);          /* 创建版本 */
        WtZip_PutU16(rec + 6, 20);          /* 解压版本 */
        WtZip_PutU16(rec + 8, 0);
        WtZip_PutU16(rec + 10, 0); /* store */
        WtZip_PutU16(rec + 12, 0);
        WtZip_PutU16(rec + 14, 0);
        WtZip_PutU32(rec + 16, entries[i].crc);
        WtZip_PutU32(rec + 20, entries[i].size);
        WtZip_PutU32(rec + 24, entries[i].size);
        WtZip_PutU16(rec + 28, (uint16_t)nameLen);
        WtZip_PutU16(rec + 30, 0);
        WtZip_PutU16(rec + 32, 0);
        WtZip_PutU16(rec + 34, 0);
        WtZip_PutU16(rec + 36, 0);
        WtZip_PutU32(rec + 38, 0);
        WtZip_PutU32(rec + 42, entries[i].offset);

        if (fwrite(rec, 1, sizeof(rec), fp) != sizeof(rec)) {
            return false;
        }
        if (fwrite(entries[i].name, 1, nameLen, fp) != nameLen) {
            return false;
        }
        cdSize += (uint32_t)sizeof(rec) + (uint32_t)nameLen;
    }

    {
        uint8_t eocd[22];

        memset(eocd, 0, sizeof(eocd));
        WtZip_PutU32(eocd + 0, 0x06054b50u); /* EOCD 签名 */
        WtZip_PutU16(eocd + 4, 0);
        WtZip_PutU16(eocd + 6, 0);
        WtZip_PutU16(eocd + 8, (uint16_t)count);
        WtZip_PutU16(eocd + 10, (uint16_t)count);
        WtZip_PutU32(eocd + 12, cdSize);
        WtZip_PutU32(eocd + 16, cdOffset);
        WtZip_PutU16(eocd + 20, 0);

        if (fwrite(eocd, 1, sizeof(eocd), fp) != sizeof(eocd)) {
            return false;
        }
    }

    return true;
}

/* ================================================================== */
/* KMZ / wpml 生成                                                     */
/* ================================================================== */

/** 动态字符串缓冲，避免为 XML 预先估算长度 */
typedef struct {
    char *buf;
    size_t len;
    size_t cap;
} WtStr;

static bool WtStr_Reserve(WtStr *s, size_t extra)
{
    if (s->len + extra + 1 <= s->cap) {
        return true;
    }

    {
        size_t newCap = (s->cap == 0) ? 8192 : s->cap;
        char *nb;

        while (s->len + extra + 1 > newCap) {
            newCap *= 2;
        }
        nb = realloc(s->buf, newCap);
        if (nb == NULL) {
            return false;
        }
        s->buf = nb;
        s->cap = newCap;
    }

    return true;
}

static bool WtStr_Append(WtStr *s, const char *fmt, ...)
{
    va_list ap;
    int n;

    if (!WtStr_Reserve(s, 512)) {
        return false;
    }

    va_start(ap, fmt);
    n = vsnprintf(s->buf + s->len, s->cap - s->len, fmt, ap);
    va_end(ap);

    if (n < 0) {
        return false;
    }
    if ((size_t)n >= s->cap - s->len) {
        if (!WtStr_Reserve(s, (size_t)n + 1)) {
            return false;
        }
        va_start(ap, fmt);
        n = vsnprintf(s->buf + s->len, s->cap - s->len, fmt, ap);
        va_end(ap);
        if (n < 0) {
            return false;
        }
    }

    s->len += (size_t)n;

    return true;
}

static void WtStr_Free(WtStr *s)
{
    free(s->buf);
    memset(s, 0, sizeof(*s));
}

/** 生成一个 Placemark（两种 wpml 文件共用） */
static bool WtBridge_WritePlacemark(WtStr *s, const WtMission *mission,
                                    const WtMissionPlan *plan, size_t idx)
{
    const WtPlanPoint *p = &mission->points[idx];
    const WtWaypointAction *a = &plan->actions.items[idx];
    const WtWaypointFlightParam *fp = (idx < plan->flightParamCount)
                                          ? &plan->flightParams[idx]
                                          : NULL;
    double speed = (fp != NULL) ? fp->speedMs : 2.0;
    double relAlt = (fp != NULL) ? fp->relativeAltitudeM : 0.0;
    int actionId = 0;

    if (!WtStr_Append(s, "      <Placemark>\n"))
        return false;
    if (!WtStr_Append(s, "        <Point><coordinates>%.8f,%.8f</coordinates></Point>\n",
                      p->geo.lon, p->geo.lat))
        return false;
    if (!WtStr_Append(s, "        <wpml:index>%zu</wpml:index>\n", idx))
        return false;
    if (!WtStr_Append(s, "        <wpml:executeHeight>%.2f</wpml:executeHeight>\n", relAlt))
        return false;
    if (!WtStr_Append(s, "        <wpml:waypointSpeed>%.2f</wpml:waypointSpeed>\n", speed))
        return false;
    if (!WtStr_Append(s,
                      "        <wpml:waypointHeadingParam>\n"
                      "          <wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>\n"
                      "          <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>\n"
                      "          <wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>\n"
                      "          <wpml:waypointHeadingPathMode>followBadArc</wpml:waypointHeadingPathMode>\n"
                      "        </wpml:waypointHeadingParam>\n"))
        return false;
    if (!WtStr_Append(s,
                      "        <wpml:waypointTurnParam>\n"
                      "          <wpml:waypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:waypointTurnMode>\n"
                      "          <wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist>\n"
                      "        </wpml:waypointTurnParam>\n"
                      "        <wpml:useStraightLine>1</wpml:useStraightLine>\n"))
        return false;

    /* 云台动作组：先摆云台，再按需拍照，顺序不可颠倒 */
    if (a->setGimbal || a->camera != WT_ACTION_NONE) {
        if (!WtStr_Append(s,
                          "        <wpml:actionGroup>\n"
                          "          <wpml:actionGroupId>%zu</wpml:actionGroupId>\n"
                          "          <wpml:actionGroupStartIndex>%zu</wpml:actionGroupStartIndex>\n"
                          "          <wpml:actionGroupEndIndex>%zu</wpml:actionGroupEndIndex>\n"
                          "          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>\n"
                          "          <wpml:actionTrigger>\n"
                          "            <wpml:actionTriggerType>reachPoint</wpml:actionTriggerType>\n"
                          "          </wpml:actionTrigger>\n",
                          idx, idx, idx))
            return false;

        if (a->setGimbal) {
            if (!WtStr_Append(s,
                              "          <wpml:action>\n"
                              "            <wpml:actionId>%d</wpml:actionId>\n"
                              "            <wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>\n"
                              "            <wpml:actionActuatorFuncParam>\n"
                              "              <wpml:gimbalHeadingYawBase>north</wpml:gimbalHeadingYawBase>\n"
                              "              <wpml:gimbalRotateMode>absoluteAngle</wpml:gimbalRotateMode>\n"
                              "              <wpml:gimbalPitchRotateEnable>1</wpml:gimbalPitchRotateEnable>\n"
                              "              <wpml:gimbalPitchRotateAngle>%.2f</wpml:gimbalPitchRotateAngle>\n"
                              "              <wpml:gimbalYawRotateEnable>1</wpml:gimbalYawRotateEnable>\n"
                              "              <wpml:gimbalYawRotateAngle>%.2f</wpml:gimbalYawRotateAngle>\n"
                              "              <wpml:gimbalRotateTimeEnable>0</wpml:gimbalRotateTimeEnable>\n"
                              "              <wpml:payloadPositionIndex>%d</wpml:payloadPositionIndex>\n"
                              "            </wpml:actionActuatorFuncParam>\n"
                              "          </wpml:action>\n",
                              actionId++, a->gimbalPitchDeg, a->gimbalYawDeg,
                              WT_BRIDGE_PAYLOAD_INDEX))
                return false;
        }

        if (a->camera == WT_ACTION_TAKE_PHOTO) {
            const char *suffix = WtPlanPoint_IsBladeInspection(p) ? "blade" : "survey";

            if (!WtStr_Append(s,
                              "          <wpml:action>\n"
                              "            <wpml:actionId>%d</wpml:actionId>\n"
                              "            <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>\n"
                              "            <wpml:actionActuatorFuncParam>\n"
                              "              <wpml:payloadPositionIndex>%d</wpml:payloadPositionIndex>\n"
                              "              <wpml:fileSuffix>%s_%zu</wpml:fileSuffix>\n"
                              "              <wpml:useGlobalPayloadLensIndex>1</wpml:useGlobalPayloadLensIndex>\n"
                              "            </wpml:actionActuatorFuncParam>\n"
                              "          </wpml:action>\n",
                              actionId++, WT_BRIDGE_PAYLOAD_INDEX, suffix, idx))
                return false;
        }

        if (!WtStr_Append(s, "        </wpml:actionGroup>\n"))
            return false;
    }

    return WtStr_Append(s, "      </Placemark>\n");
}

/** 生成一份完整的 wpml 文档 */
static bool WtBridge_BuildWpml(const WtMission *mission, const WtMissionPlan *plan,
                               const WtInspectionProfile *profile, WtStr *out)
{
    size_t i;

    if (!WtStr_Append(out,
                      "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                      "<kml xmlns=\"http://www.opengis.net/kml/2.2\" "
                      "xmlns:wpml=\"http://www.dji.com/wpmz/1.0.2\">\n"
                      "  <Document>\n"
                      "    <wpml:missionConfig>\n"
                      "      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>\n"
                      "      <wpml:finishAction>goHome</wpml:finishAction>\n"
                      "      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>\n"
                      "      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>\n"
                      "      <wpml:takeOffSecurityHeight>%.1f</wpml:takeOffSecurityHeight>\n"
                      "      <wpml:globalTransitionalSpeed>%.1f</wpml:globalTransitionalSpeed>\n"
                      "      <wpml:droneInfo>\n"
                      "        <wpml:droneEnumValue>99</wpml:droneEnumValue>\n"
                      "        <wpml:droneSubEnumValue>0</wpml:droneSubEnumValue>\n"
                      "      </wpml:droneInfo>\n"
                      "    </wpml:missionConfig>\n"
                      "    <Folder>\n"
                      "      <wpml:templateType>waypoint</wpml:templateType>\n"
                      "      <wpml:templateId>0</wpml:templateId>\n"
                      "      <wpml:waylineCoordinateSysParam>\n"
                      "        <wpml:coordinateMode>WGS84</wpml:coordinateMode>\n"
                      "        <wpml:heightMode>relativeToStartPoint</wpml:heightMode>\n"
                      "      </wpml:waylineCoordinateSysParam>\n"
                      "      <wpml:autoFlightSpeed>%.1f</wpml:autoFlightSpeed>\n"
                      "      <wpml:gimbalPitchMode>usePointSetting</wpml:gimbalPitchMode>\n",
                      profile->groundClearanceM, profile->cruiseSpeedMs,
                      profile->inspectSpeedMs))
        return false;

    for (i = 0; i < mission->count; i++) {
        if (!WtBridge_WritePlacemark(out, mission, plan, i)) {
            return false;
        }
    }

    return WtStr_Append(out, "    </Folder>\n  </Document>\n</kml>\n");
}

WtPlanResult WtBridge_ExportKmz(const WtMission *mission,
                                const WtMissionPlan *plan,
                                const WtInspectionProfile *profile,
                                const WtGeo *takeoffGeo,
                                const char *outPath)
{
    WtStr wpml = {0};
    WtZipEntry entries[2];
    FILE *fp;
    uint32_t offset = 0;
    uint32_t cdOffset;
    bool ok;

    (void)takeoffGeo;

    if (!WtBridge_BuildWpml(mission, plan, profile, &wpml)) {
        WtStr_Free(&wpml);
        return WT_PLAN_ERR_NOMEM;
    }

    fp = fopen(outPath, "wb");
    if (fp == NULL) {
        WtStr_Free(&wpml);
        return WT_PLAN_ERR_PARAM;
    }

    /* 两个条目共用同一份 wpml 正文，只是路径不同：模板与可执行航线 */
    ok = WtZip_WriteEntry(fp, &entries[0], "wpmz/template.kml", wpml.buf, offset);
    offset += (uint32_t)(30 + strlen("wpmz/template.kml") + wpml.len);

    ok = ok && WtZip_WriteEntry(fp, &entries[1], "wpmz/waylines.wpml", wpml.buf, offset);
    cdOffset = offset + (uint32_t)(30 + strlen("wpmz/waylines.wpml") + wpml.len);

    if (!ok || !WtZip_WriteCentralDirectory(fp, entries, 2, cdOffset)) {
        fclose(fp);
        WtStr_Free(&wpml);
        return WT_PLAN_ERR_PARAM;
    }

    fclose(fp);
    WtStr_Free(&wpml);

    return WT_PLAN_OK;
}

WtPlanResult WtBridge_ExportActionCsv(const WtMission *mission,
                                      const WtMissionPlan *plan,
                                      const char *path)
{
    FILE *fp = fopen(path, "w");
    size_t i;

    if (fp == NULL) {
        return WT_PLAN_ERR_PARAM;
    }

    fprintf(fp, "index,tag,purpose,rel_alt_m,speed_ms,set_gimbal,"
                "gimbal_yaw_deg,gimbal_pitch_deg,camera_action\n");

    for (i = 0; i < mission->count; i++) {
        const WtPlanPoint *p = &mission->points[i];
        const WtWaypointAction *a = &plan->actions.items[i];
        const WtWaypointFlightParam *f = (i < plan->flightParamCount)
                                             ? &plan->flightParams[i]
                                             : NULL;
        const char *cam = "none";

        if (a->camera == WT_ACTION_TAKE_PHOTO) {
            cam = "takePhoto";
        } else if (a->camera == WT_ACTION_START_RECORD) {
            cam = "startRecord";
        } else if (a->camera == WT_ACTION_STOP_RECORD) {
            cam = "stopRecord";
        }

        fprintf(fp, "%zu,%s,%d,%.2f,%.2f,%d,%.2f,%.2f,%s\n",
                i, p->tag ? p->tag : "", (int)p->purpose,
                f ? f->relativeAltitudeM : 0.0,
                f ? f->speedMs : 0.0,
                a->setGimbal ? 1 : 0,
                a->gimbalYawDeg, a->gimbalPitchDeg, cam);
    }

    fclose(fp);

    return WT_PLAN_OK;
}