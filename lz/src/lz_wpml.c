/**
 * @file lz_wpml.c
 * @brief 生成 wpml 航点文件（template.kml + waylines.wpml）。
 *
 * 格式依据：**解包官方样例 KMZ 得到的真实结构**（见 lz/doc/）。
 * 官方样例：samples/sample_c/module_sample/waypoint_v3/waypoint_file/
 *           waypoint_v3_test_file.kmz
 *
 * ## 绕飞的关键：gimbalRotate + absoluteAngle + Yaw
 *
 * `[V]` 解包实测，wpml 的云台动作参数长这样：
 *
 *     <wpml:gimbalRotateMode>absoluteAngle</wpml:gimbalRotateMode>
 *     <wpml:gimbalYawRotateEnable>1</wpml:gimbalYawRotateEnable>
 *     <wpml:gimbalYawRotateAngle>-45</wpml:gimbalYawRotateAngle>
 *
 * 官方样例里 `gimbalYawRotateEnable` 是 **0**（它只需要俯仰）。
 * 绕飞要的正是 yaw —— 把它置 1、角度填绝对方位角，光轴就指向杆心。
 *
 * ## 两个文件的差别
 *
 * - `template.kml`：模板，带 `templateType/waylineCoordinateSysParam` 等
 * - `waylines.wpml`：**可执行航线**，用 `executeHeight` 而非 `height`，
 *   且每个点自带 `waypointSpeed` —— 这是飞机实际读的那份
 *
 * 两份都要写，内容高度重复但字段名不同。这是 wpml 规范的要求，
 * 不能只写一份。
 */

#include "lz_wpml.h"
#include "lz_geo.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 单份 XML 的预估大小。
 * ⚠️ 这个值是**估计**，不够时会返回 LZ_ERR_RANGE（见 lz_str_addf 的说明）。
 * 实测 8 航点约需 16 KB / 份，这里按 4 KB/航点留了 2 倍余量。
 * 真实航点数由配置决定，超出时会明确报错而不是写出截断的 XML。 */
#define LZ_XML_PER_WP_BYTES 4096
#define LZ_XML_BASE_BYTES   4096

typedef struct {
    char *buf;
    size_t len;
    size_t cap;
    bool overflow;
} LzStr;

static void lz_str_init(LzStr *s, size_t cap)
{
    s->buf = malloc(cap);
    s->len = 0;
    s->cap = cap;
    s->overflow = (s->buf == NULL);
    if (s->buf != NULL) {
        s->buf[0] = '\0';
    }
}

static void lz_str_addf(LzStr *s, const char *fmt, ...)
{
    if (s->overflow || s->buf == NULL) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(s->buf + s->len, s->cap - s->len, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= s->cap - s->len) {
        /* 宁可整份作废也不要写出被截断的 XML —— 截断的 XML 交给飞机
         * 只会得到一个含糊的失败，排查成本远高于这里直接报错 */
        s->overflow = true;
        return;
    }
    s->len += (size_t)n;
}

LzStatus LzWpml_Build(const LzRoute *route,
                      const LzTarget *pole,
                      const LzOrbitProfile *profile,
                      LzWpmlFiles *out)
{
    if (route == NULL || profile == NULL || out == NULL) {
        return LZ_ERR_PARAM;
    }
    /* pole 目前不参与 XML 生成（云台角已由 lz_plan 算好）；
     * 保留参数是为了将来若要在文件里写入兴趣点坐标时不必改签名 */
    (void)pole;
    if (route->count == 0) {
        return LZ_ERR_NO_TARGET;
    }

    out->templateKml = NULL;
    out->waylinesWpml = NULL;

    const size_t cap = LZ_XML_BASE_BYTES + route->count * LZ_XML_PER_WP_BYTES;
    LzStr t, w;
    lz_str_init(&t, cap);
    lz_str_init(&w, cap);
    if (t.overflow || w.overflow) {
        free(t.buf); free(w.buf);
        return LZ_ERR_IO;
    }

    /* 机型/负载枚举必须与实际匹配，否则飞机可能拒绝执行。
     * 由调用方通过 LzWpmlIdentity 指定，默认给 M4T。 */
    const int droneEnum = out->identity.droneEnumValue;
    const int droneSub  = out->identity.droneSubEnumValue;
    const int payloadEnum = out->identity.payloadEnumValue;
    const int payloadSub  = out->identity.payloadSubEnumValue;

    /* ================= template.kml ================= */
    lz_str_addf(&t, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    lz_str_addf(&t, "<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:wpml=\"http://www.dji.com/wpmz/1.0.3\">\n");
    lz_str_addf(&t, "  <Document>\n");
    lz_str_addf(&t, "    <wpml:missionConfig>\n");
    lz_str_addf(&t, "      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>\n");
    lz_str_addf(&t, "      <wpml:finishAction>goHome</wpml:finishAction>\n");
    lz_str_addf(&t, "      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>\n");
    lz_str_addf(&t, "      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>\n");
    lz_str_addf(&t, "      <wpml:takeOffSecurityHeight>%d</wpml:takeOffSecurityHeight>\n",
                LZ_WPML_TAKEOFF_SECURITY_HEIGHT);
    lz_str_addf(&t, "      <wpml:globalTransitionalSpeed>%.1f</wpml:globalTransitionalSpeed>\n",
                profile->speedMs);
    lz_str_addf(&t, "      <wpml:droneInfo>\n");
    lz_str_addf(&t, "        <wpml:droneEnumValue>%d</wpml:droneEnumValue>\n", droneEnum);
    lz_str_addf(&t, "        <wpml:droneSubEnumValue>%d</wpml:droneSubEnumValue>\n", droneSub);
    lz_str_addf(&t, "      </wpml:droneInfo>\n");
    lz_str_addf(&t, "      <wpml:payloadInfo>\n");
    lz_str_addf(&t, "        <wpml:payloadEnumValue>%d</wpml:payloadEnumValue>\n", payloadEnum);
    lz_str_addf(&t, "        <wpml:payloadSubEnumValue>%d</wpml:payloadSubEnumValue>\n", payloadSub);
    lz_str_addf(&t, "        <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>\n");
    lz_str_addf(&t, "      </wpml:payloadInfo>\n");
    lz_str_addf(&t, "    </wpml:missionConfig>\n");
    lz_str_addf(&t, "    <Folder>\n");
    lz_str_addf(&t, "      <wpml:templateType>waypoint</wpml:templateType>\n");
    lz_str_addf(&t, "      <wpml:templateId>0</wpml:templateId>\n");
    lz_str_addf(&t, "      <wpml:waylineCoordinateSysParam>\n");
    lz_str_addf(&t, "        <wpml:coordinateMode>WGS84</wpml:coordinateMode>\n");
    lz_str_addf(&t, "        <wpml:heightMode>relativeToStartPoint</wpml:heightMode>\n");
    lz_str_addf(&t, "        <wpml:positioningType>GPS</wpml:positioningType>\n");
    lz_str_addf(&t, "      </wpml:waylineCoordinateSysParam>\n");
    lz_str_addf(&t, "      <wpml:autoFlightSpeed>%.1f</wpml:autoFlightSpeed>\n", profile->speedMs);
    lz_str_addf(&t, "      <wpml:globalHeight>%.1f</wpml:globalHeight>\n", profile->altitudeM);
    lz_str_addf(&t, "      <wpml:caliFlightEnable>0</wpml:caliFlightEnable>\n");
    /* gimbalPitchMode=usePointSetting：让每个航点用自己的云台设定，
     * 而不是全局固定值 —— 绕飞必须逐点不同 */
    lz_str_addf(&t, "      <wpml:gimbalPitchMode>usePointSetting</wpml:gimbalPitchMode>\n");
    lz_str_addf(&t, "      <wpml:globalWaypointHeadingParam>\n");
    lz_str_addf(&t, "        <wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>\n");
    lz_str_addf(&t, "        <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>\n");
    lz_str_addf(&t, "        <wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>\n");
    lz_str_addf(&t, "        <wpml:waypointHeadingPoiIndex>0</wpml:waypointHeadingPoiIndex>\n");
    lz_str_addf(&t, "      </wpml:globalWaypointHeadingParam>\n");
    lz_str_addf(&t, "      <wpml:globalWaypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:globalWaypointTurnMode>\n");
    lz_str_addf(&t, "      <wpml:globalUseStraightLine>1</wpml:globalUseStraightLine>\n");

    for (size_t i = 0; i < route->count; ++i) {
        const LzWaypoint *wp = &route->points[i];
        lz_str_addf(&t, "      <Placemark>\n");
        lz_str_addf(&t, "        <Point>\n          <coordinates>%.10f,%.10f</coordinates>\n        </Point>\n",
                    wp->geo.longitudeDeg, wp->geo.latitudeDeg);
        lz_str_addf(&t, "        <wpml:index>%zu</wpml:index>\n", i);
        lz_str_addf(&t, "        <wpml:ellipsoidHeight>%.1f</wpml:ellipsoidHeight>\n", wp->relativeAltM);
        lz_str_addf(&t, "        <wpml:height>%.1f</wpml:height>\n", wp->relativeAltM);
        /* 逐点用自己的高度/速度/云台 —— 三个 useGlobal* 必须为 0 */
        lz_str_addf(&t, "        <wpml:useGlobalHeight>0</wpml:useGlobalHeight>\n");
        lz_str_addf(&t, "        <wpml:useGlobalSpeed>0</wpml:useGlobalSpeed>\n");
        lz_str_addf(&t, "        <wpml:useGlobalHeadingParam>1</wpml:useGlobalHeadingParam>\n");
        lz_str_addf(&t, "        <wpml:useGlobalTurnParam>1</wpml:useGlobalTurnParam>\n");
        lz_str_addf(&t, "        <wpml:useStraightLine>0</wpml:useStraightLine>\n");
        lz_str_addf(&t, "        <wpml:actionGroup>\n");
        lz_str_addf(&t, "          <wpml:actionGroupId>%zu</wpml:actionGroupId>\n", i);
        lz_str_addf(&t, "          <wpml:actionGroupStartIndex>%zu</wpml:actionGroupStartIndex>\n", i);
        lz_str_addf(&t, "          <wpml:actionGroupEndIndex>%zu</wpml:actionGroupEndIndex>\n", i);
        lz_str_addf(&t, "          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>\n");
        lz_str_addf(&t, "          <wpml:actionTrigger>\n");
        lz_str_addf(&t, "            <wpml:actionTriggerType>reachPoint</wpml:actionTriggerType>\n");
        lz_str_addf(&t, "          </wpml:actionTrigger>\n");
        lz_str_addf(&t, "          <wpml:action>\n");
        lz_str_addf(&t, "            <wpml:actionId>0</wpml:actionId>\n");
        lz_str_addf(&t, "            <wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>\n");
        lz_str_addf(&t, "            <wpml:actionActuatorFuncParam>\n");
        lz_str_addf(&t, "              <wpml:gimbalRotateMode>absoluteAngle</wpml:gimbalRotateMode>\n");
        lz_str_addf(&t, "              <wpml:gimbalPitchRotateEnable>1</wpml:gimbalPitchRotateEnable>\n");
        lz_str_addf(&t, "              <wpml:gimbalPitchRotateAngle>%.1f</wpml:gimbalPitchRotateAngle>\n", wp->gimbalPitchDeg);
        lz_str_addf(&t, "              <wpml:gimbalRollRotateEnable>0</wpml:gimbalRollRotateEnable>\n");
        lz_str_addf(&t, "              <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>\n");
        /* ★ 绕飞核心：yaw 使能 + 绝对方位角 */
        lz_str_addf(&t, "              <wpml:gimbalYawRotateEnable>1</wpml:gimbalYawRotateEnable>\n");
        lz_str_addf(&t, "              <wpml:gimbalYawRotateAngle>%.1f</wpml:gimbalYawRotateAngle>\n", wp->gimbalYawDeg);
        lz_str_addf(&t, "              <wpml:gimbalRotateTimeEnable>0</wpml:gimbalRotateTimeEnable>\n");
        lz_str_addf(&t, "              <wpml:gimbalRotateTime>0</wpml:gimbalRotateTime>\n");
        lz_str_addf(&t, "              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>\n");
        lz_str_addf(&t, "            </wpml:actionActuatorFuncParam>\n");
        lz_str_addf(&t, "          </wpml:action>\n");
        lz_str_addf(&t, "        </wpml:actionGroup>\n");
        lz_str_addf(&t, "      </Placemark>\n");
    }

    lz_str_addf(&t, "    </Folder>\n  </Document>\n</kml>\n");

    /* ================= waylines.wpml ================= */
    lz_str_addf(&w, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    lz_str_addf(&w, "<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:wpml=\"http://www.dji.com/wpmz/1.0.3\">\n");
    lz_str_addf(&w, "  <Document>\n");
    lz_str_addf(&w, "    <wpml:missionConfig>\n");
    lz_str_addf(&w, "      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>\n");
    lz_str_addf(&w, "      <wpml:finishAction>goHome</wpml:finishAction>\n");
    lz_str_addf(&w, "      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>\n");
    lz_str_addf(&w, "      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>\n");
    lz_str_addf(&w, "      <wpml:takeOffSecurityHeight>%d</wpml:takeOffSecurityHeight>\n",
                LZ_WPML_TAKEOFF_SECURITY_HEIGHT);
    lz_str_addf(&w, "      <wpml:globalTransitionalSpeed>%.1f</wpml:globalTransitionalSpeed>\n", profile->speedMs);
    lz_str_addf(&w, "      <wpml:droneInfo>\n");
    lz_str_addf(&w, "        <wpml:droneEnumValue>%d</wpml:droneEnumValue>\n", droneEnum);
    lz_str_addf(&w, "        <wpml:droneSubEnumValue>%d</wpml:droneSubEnumValue>\n", droneSub);
    lz_str_addf(&w, "      </wpml:droneInfo>\n");
    lz_str_addf(&w, "      <wpml:payloadInfo>\n");
    lz_str_addf(&w, "        <wpml:payloadEnumValue>%d</wpml:payloadEnumValue>\n", payloadEnum);
    lz_str_addf(&w, "        <wpml:payloadSubEnumValue>%d</wpml:payloadSubEnumValue>\n", payloadSub);
    lz_str_addf(&w, "        <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>\n");
    lz_str_addf(&w, "      </wpml:payloadInfo>\n");
    lz_str_addf(&w, "    </wpml:missionConfig>\n");
    lz_str_addf(&w, "    <Folder>\n");
    lz_str_addf(&w, "      <wpml:templateId>0</wpml:templateId>\n");
    lz_str_addf(&w, "      <wpml:executeHeightMode>relativeToStartPoint</wpml:executeHeightMode>\n");
    lz_str_addf(&w, "      <wpml:waylineId>0</wpml:waylineId>\n");
    lz_str_addf(&w, "      <wpml:autoFlightSpeed>%.1f</wpml:autoFlightSpeed>\n", profile->speedMs);
    /* distance/duration 是这条航线的**实际**长度与耗时，供 Pilot 显示用。
     *
     * ️ 不是整圈周长！飞机只飞 n 个点之间**弦**的 (n-1) 段 ——
     * 它依次经过 p0, p1, ..., p_{n-1} 就结束了（最后一个点的
     * actionGroup 是 reachPoint，到点后按 finishAction 返航）。
     * 从 p_{n-1} 回到 p0 那一段**不存在**，所以不能按 2πr 算。
     *
     * 与 lz_plan.c 里"覆盖 (n-1)/n 圈而不是整圈"是同一件事的两种体现，
     * 两边必须一致 —— 改成闭合航线时，这里也要跟着改成整圈。 */
    double pathLen = 0.0;
    for (size_t i = 1; i < route->count; ++i) {
        pathLen += LzGeo_DistanceM(&route->points[i - 1].geo, &route->points[i].geo);
    }
    lz_str_addf(&w, "      <wpml:distance>%.3f</wpml:distance>\n", pathLen);
    lz_str_addf(&w, "      <wpml:duration>%.3f</wpml:duration>\n",
                (profile->speedMs > 0.0) ? (pathLen / profile->speedMs) : 0.0);

    for (size_t i = 0; i < route->count; ++i) {
        const LzWaypoint *wp = &route->points[i];
        lz_str_addf(&w, "      <Placemark>\n");
        lz_str_addf(&w, "        <Point>\n          <coordinates>%.10f,%.10f</coordinates>\n        </Point>\n",
                    wp->geo.longitudeDeg, wp->geo.latitudeDeg);
        lz_str_addf(&w, "        <wpml:index>%zu</wpml:index>\n", i);
        lz_str_addf(&w, "        <wpml:executeHeight>%.1f</wpml:executeHeight>\n", wp->relativeAltM);
        lz_str_addf(&w, "        <wpml:waypointSpeed>%.1f</wpml:waypointSpeed>\n", wp->speedMs);
        lz_str_addf(&w, "        <wpml:waypointHeadingParam>\n");
        lz_str_addf(&w, "          <wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>\n");
        lz_str_addf(&w, "          <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>\n");
        lz_str_addf(&w, "          <wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>\n");
        lz_str_addf(&w, "          <wpml:waypointHeadingAngleEnable>0</wpml:waypointHeadingAngleEnable>\n");
        lz_str_addf(&w, "          <wpml:waypointHeadingPoiIndex>0</wpml:waypointHeadingPoiIndex>\n");
        lz_str_addf(&w, "        </wpml:waypointHeadingParam>\n");
        lz_str_addf(&w, "        <wpml:waypointTurnParam>\n");
        lz_str_addf(&w, "          <wpml:waypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:waypointTurnMode>\n");
        lz_str_addf(&w, "          <wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist>\n");
        lz_str_addf(&w, "        </wpml:waypointTurnParam>\n");
        lz_str_addf(&w, "        <wpml:useStraightLine>1</wpml:useStraightLine>\n");
        lz_str_addf(&w, "        <wpml:actionGroup>\n");
        lz_str_addf(&w, "          <wpml:actionGroupId>%zu</wpml:actionGroupId>\n", i);
        lz_str_addf(&w, "          <wpml:actionGroupStartIndex>%zu</wpml:actionGroupStartIndex>\n", i);
        lz_str_addf(&w, "          <wpml:actionGroupEndIndex>%zu</wpml:actionGroupEndIndex>\n", i);
        lz_str_addf(&w, "          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>\n");
        lz_str_addf(&w, "          <wpml:actionTrigger>\n");
        lz_str_addf(&w, "            <wpml:actionTriggerType>reachPoint</wpml:actionTriggerType>\n");
        lz_str_addf(&w, "          </wpml:actionTrigger>\n");
        lz_str_addf(&w, "          <wpml:action>\n");
        lz_str_addf(&w, "            <wpml:actionId>0</wpml:actionId>\n");
        lz_str_addf(&w, "            <wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>\n");
        lz_str_addf(&w, "            <wpml:actionActuatorFuncParam>\n");
        lz_str_addf(&w, "              <wpml:gimbalRotateMode>absoluteAngle</wpml:gimbalRotateMode>\n");
        lz_str_addf(&w, "              <wpml:gimbalPitchRotateEnable>1</wpml:gimbalPitchRotateEnable>\n");
        lz_str_addf(&w, "              <wpml:gimbalPitchRotateAngle>%.1f</wpml:gimbalPitchRotateAngle>\n", wp->gimbalPitchDeg);
        lz_str_addf(&w, "              <wpml:gimbalRollRotateEnable>0</wpml:gimbalRollRotateEnable>\n");
        lz_str_addf(&w, "              <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>\n");
        lz_str_addf(&w, "              <wpml:gimbalYawRotateEnable>1</wpml:gimbalYawRotateEnable>\n");
        lz_str_addf(&w, "              <wpml:gimbalYawRotateAngle>%.1f</wpml:gimbalYawRotateAngle>\n", wp->gimbalYawDeg);
        lz_str_addf(&w, "              <wpml:gimbalRotateTimeEnable>0</wpml:gimbalRotateTimeEnable>\n");
        lz_str_addf(&w, "              <wpml:gimbalRotateTime>0</wpml:gimbalRotateTime>\n");
        lz_str_addf(&w, "              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>\n");
        lz_str_addf(&w, "            </wpml:actionActuatorFuncParam>\n");
        lz_str_addf(&w, "          </wpml:action>\n");
        lz_str_addf(&w, "        </wpml:actionGroup>\n");
        lz_str_addf(&w, "      </Placemark>\n");
    }

    lz_str_addf(&w, "    </Folder>\n  </Document>\n</kml>\n");

    if (t.overflow || w.overflow) {
        free(t.buf); free(w.buf);
        return LZ_ERR_RANGE;   /* 缓冲区不够：航点太多，传入的 cap 估算不足 */
    }

    out->templateKml = t.buf;
    out->waylinesWpml = w.buf;
    return LZ_OK;
}

LzWpmlIdentity LzWpml_DefaultIdentity(void)
{
    /* 核实自 psdk_lib/include/dji_typedef.h：
     *   DJI_AIRCRAFT_TYPE_M4T = 99    （机型）
     *   DJI_CAMERA_TYPE_M4T   = 89    （负载）
     * 子类型取 0/1：M4T 是 M4 系列里的第 2 个变体（M4E=0, M4T=1） */
    LzWpmlIdentity id = {
        .droneEnumValue = 99,
        .droneSubEnumValue = 1,
        .payloadEnumValue = 89,
        .payloadSubEnumValue = 0,
    };
    return id;
}

void LzWpml_Free(LzWpmlFiles *files)
{
    if (files == NULL) {
        return;
    }
    free(files->templateKml);
    free(files->waylinesWpml);
    files->templateKml = NULL;
    files->waylinesWpml = NULL;
}
