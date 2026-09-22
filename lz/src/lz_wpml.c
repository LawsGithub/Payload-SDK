/**
 * @file lz_wpml.c
 * @brief 生成 wpml 航点文件（template.kml + waylines.wpml）。
 *
 * 格式依据：**解包官方样例 KMZ 得到的真实结构**（见 lz/doc/）。
 * 官方样例：samples/sample_c/module_sample/waypoint_v3/waypoint_file/
 *           waypoint_v3_test_file.kmz
 *
 * ## 绕飞的关键：towardPOI（机头对准杆心）
 *
 * 每个航点的 `waypointHeadingMode` 置 `towardPOI`、`waypointPoiPoint`
 * 填杆的经纬度，飞机就**侧飞**绕圈：机头（因而光轴）始终指向圆心。
 *
 * ## ⚠️ 为什么不用 gimbalRotate 的 absoluteAngle yaw —— 在 M4T 上做不到
 *
 * 曾经的实现是逐点写：
 *
 *     <wpml:gimbalRotateMode>absoluteAngle</wpml:gimbalRotateMode>
 *     <wpml:gimbalYawRotateEnable>1</wpml:gimbalYawRotateEnable>
 *     <wpml:gimbalYawRotateAngle>-45</wpml:gimbalYawRotateAngle>
 *
 * 同时 `waypointHeadingMode=followWayline`（机头沿航线）。**这个组合在 M4T
 * 上非法**，理由是两条独立证据：
 *
 * 1. 规范说云台 yaw 必须跟着机头（`40.common-element.md` 的
 *    gimbalRotate / orientedShoot / rotateYaw 三处都标）：
 *
 *        wpml:gimbalYawRotateAngle 与 wpml:aircraftHeading 需保持一致
 *        适用机型：M3E/M3T，M3D/M3TD，M4D/M4TD，**M4E/M4T**
 *
 * 2. M4T 规格页（enterprise.dji.com/zh-tw/matrice-4-series/specs）：
 *
 *        Controllable Rotation Range — Pan: Not controllable
 *        Yaw Axis — Manual operation is uncontrollable;
 *                   The MSDK interface program is controllable.
 *        机械软限位 Pan: -60° ~ +60°
 *
 * 合起来即：M4T 的云台 yaw **不能独立于机头偏转**（只有 MSDK 程序能驱动
 * 机械限位内的那一小段）。而我们要求光轴转 45°、机头不跟 —— 做不到。
 * 飞机给的回应就是"角度过大无法转向"。
 *
 * ⇒ 让机头承担这个偏转让，云台只做俯仰。这就是 `towardPOI` 的语义，
 *   也正是 DJI Pilot 2 里「兴趣点环绕」的做法。
 *
 * ️ 但**官方样例不能作为"哪些元素必需"的权威**：实测它自身缺
 * `wpml:globalRTHHeight`（官方标为必需元素），照样能飞。所以本文件的
 * 必需元素清单以 Cloud API 的 wpml 规范为准，不以样例为准。依据：
 * `~/projects/.psdk-apiref/docs` 之外的 Cloud-API-Doc 仓库，
 * `docs/cn/60.api-reference/00.dji-wpml/{20.template-kml,30.waylines-wpml,40.common-element}.md`。
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

/**
 * @brief 把方位角折算到 wpml 要求的 [-180, 180]
 *
 * `LzGeo_BearingDeg()` 给的是 [0, 360)。`gimbalYawRotateAngle` 的取值域
 * 在官方文档里写的是 `[-180, 180]`（见 Cloud API 文档
 * `00.dji-wpml/40.common-element.md` 的 gimbalRotate 一节），
 * **[0, 360) 的值落在域外**。180.0 映射到 -180.0，
 * 两者指同一方向（正南），但域内的那个才是合法的。
 *
 * 为什么放在这一层而不是 lz_core 的 LzGeo：`LzGeo_NormalizeDeg` 是几何量的
 * 规范区间（[0,360) 左闭右开），wpml 的 [-180,180] 是**文件格式**的取值域。
 * 混在一起会让测试里那条"极小负数加 360 舍入成 360.0"的边界断言失去意义。
 */
static double lz_yaw_to_signed(double deg)
{
    double d = LzGeo_NormalizeDeg(deg);
    if (d > 180.0) {
        d -= 360.0;
    }
    return d;
}

/**
 * @brief 把 LzTurnMode 翻成 wpml 的枚举字符串
 *
 * 放在这一层而不是 lz_plan：`LzTurnMode` 是**规划层的意图**
 * （走直线还是走曲线），wpml 的枚举名是**文件格式的词汇** ——
 * 与 `lz_yaw_to_signed` 同一个理由，格式细节不污染 lz_core。
 */
static const char *lz_turn_mode_str(LzTurnMode mode)
{
    switch (mode) {
    case LZ_TURN_PASS_WITH_CURVE:
        return "toPointAndPassWithContinuityCurvature";
    case LZ_TURN_TO_POINT_AND_STOP:
    default:
        return "toPointAndStopWithDiscontinuityCurvature";
    }
}

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
    /* pole 参与 XML 生成：它的经纬度就是 waypointPoiPoint（兴趣点）。
     * 曾经这里是 `(void)pole;` —— 那时云台角已由 lz_plan 算好、坐标无用；
     * 改成 towardPOI 后，飞机需要**自己**知道圆心在哪，所以必须写进去。 */
    /* 兴趣点坐标必须是真定位。零解（无定位时的浮点残差）能过
     * LzGeo_IsValid 却毫无意义 —— 机头会朝几内亚湾转，绕飞彻底失效。 */
    if (pole == NULL || !LzGeo_IsValid(&pole->geo) ||
        LzGeo_IsNullSolution(&pole->geo)) {
        return LZ_ERR_NO_TARGET;
    }
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

    /* 返航高度：取航线高度与下限的较大者。
     *
     * ⚠️ 这条推理的历史与现状要分清（2026-09-23 更新）：
     *
     * 原实现 `finishAction=goHome`，完成任务后爬升到 globalRTHHeight 再返航 ——
     * 若它低于航线高度，返航前半段是下降，而此刻飞机就在杆的上方。
     * 这是"取 max"的**原初理由**。
     *
     * 现在 `finishAction=gotoFirstWaypoint`（见 lz_wpml.h），**不再返航**，
     * 所以这个理由在本流程里已不成立。**但取值不改、常量保留**：
     * `globalRTHHeight` 是必需元素，且操作员手动按返航、或触发失控返航时，
     * 它仍是飞机实际使用的返航高度 —— 该值低于航线高度那个隐患依然存在。 */
    double rthHeight = profile->altitudeM;
    if (rthHeight < (double)LZ_WPML_RTH_HEIGHT_FLOOR_M) {
        rthHeight = (double)LZ_WPML_RTH_HEIGHT_FLOOR_M;
    }

    /* 提前转弯截距：由**真实航段长度**反算，不接调用方给的值 ——
     * 规范的两条约束都是相对于段长的，只有这里能量准。见 lz_plan.c。 */
    const double dampingM =
        (profile->turnMode == LZ_TURN_PASS_WITH_CURVE)
            ? LzPlan_SuggestDampingM(route)
            : 0.0;

    /* ================= template.kml ================= */
    lz_str_addf(&t, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    lz_str_addf(&t, "<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:wpml=\"http://www.dji.com/wpmz/1.0.3\">\n");
    lz_str_addf(&t, "  <Document>\n");
    lz_str_addf(&t, "    <wpml:missionConfig>\n");
    lz_str_addf(&t, "      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>\n");
    lz_str_addf(&t, "      <wpml:finishAction>%s</wpml:finishAction>\n", LZ_WPML_FINISH_ACTION);
    lz_str_addf(&t, "      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>\n");
    lz_str_addf(&t, "      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>\n");
    lz_str_addf(&t, "      <wpml:takeOffSecurityHeight>%d</wpml:takeOffSecurityHeight>\n",
                LZ_WPML_TAKEOFF_SECURITY_HEIGHT);
    lz_str_addf(&t, "      <wpml:globalTransitionalSpeed>%.1f</wpml:globalTransitionalSpeed>\n",
                profile->speedMs);
    /* globalRTHHeight 是**必需元素**（官方文档 30.waylines-wpml.md:135 /
     * 20.template-kml.md:164 均标"必需元素"），原实现漏了它。
     * 取航线高度与下限的较大者：返航高度低于航线高度会变成"先下降再返航"。 */
    lz_str_addf(&t, "      <wpml:globalRTHHeight>%.1f</wpml:globalRTHHeight>\n", rthHeight);
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
    lz_str_addf(&t, "        <wpml:waypointHeadingMode>towardPOI</wpml:waypointHeadingMode>\n");
    lz_str_addf(&t, "        <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>\n");
    /* ★ 绕飞核心：机头始终朝向杆心。高度置 0 —— 规范明说
     * "目前不支持Z方向朝向兴趣点，高度可设置为0"，写航点高度会被忽略。 */
    lz_str_addf(&t, "        <wpml:waypointPoiPoint>%.7f,%.7f,0.000000</wpml:waypointPoiPoint>\n",
                pole->geo.latitudeDeg, pole->geo.longitudeDeg);
    /* waypointHeadingPathMode 是**必需元素**（40.common-element.md 的
     * `<wpml:waypointHeadingParam> & <wpml:globalWaypointHeadingParam>` 一节）。
     * 取值跟绕行方向走：机头要主动去追杆，必须明确告诉它朝哪边转 ——
     * 顺时针绕飞时机头也该顺时针转。
     * （曾经取 followBadArc，那时机头沿航线、转向是被动的。
     *   clockwise/counterClockwise 正是给"指定目标航向、需要选一条路转过去"
     *   的场景用的，现在正是这个场景。） */
    lz_str_addf(&t, "        <wpml:waypointHeadingPathMode>%s</wpml:waypointHeadingPathMode>\n",
                profile->clockwise ? "clockwise" : "counterClockwise");
    lz_str_addf(&t, "        <wpml:waypointHeadingPoiIndex>0</wpml:waypointHeadingPoiIndex>\n");
    lz_str_addf(&t, "      </wpml:globalWaypointHeadingParam>\n");
    /* 转弯模式：决定轨迹是内接多边形还是近似圆弧 —— 见 lz_plan.h 的 LzTurnMode。
     * globalUseStraightLine=1 与 curve 模式搭配正是 Pilot 2 里
     * 「平滑过点，提前转弯」的设置方法（规范 waypointTurnMode 一行有注解）。 */
    lz_str_addf(&t, "      <wpml:globalWaypointTurnMode>%s</wpml:globalWaypointTurnMode>\n",
                lz_turn_mode_str(profile->turnMode));

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
        /* useStraightLine：规范里它只在「曲线到点停」/「曲线过点不停」两种模式下
         * 必需，含义是"航段轨迹尽量贴合两点连线"。Pilot 2 的「平滑过点，提前
         * 转弯」正是 curve 模式 + 本值=1。 */
        lz_str_addf(&t, "        <wpml:useStraightLine>1</wpml:useStraightLine>\n");
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
        /* ★ 绕飞核心：先声明 yaw 的绝对基准，再给绝对方位角
         *
         * gimbalHeadingYawBase 是 gimbalRotate 一节里的**必需元素**
         * （40.common-element.md），声明 yaw 角"相对什么"。
         * ️ 官方样例里也没有这个元素 —— 但官方样例的
         * `gimbalYawRotateEnable` 是 **0**（它只用俯仰），所以缺它对样例无影响。
         * **我们的 yaw 是使能的**，`absoluteAngle` 正是依赖"相对正北"这个基准，
         * 声明基准的元素不能省。样例的"缺了也能飞"不能用来给我们开脱。 */
        lz_str_addf(&t, "              <wpml:gimbalHeadingYawBase>north</wpml:gimbalHeadingYawBase>\n");
        lz_str_addf(&t, "              <wpml:gimbalRotateMode>absoluteAngle</wpml:gimbalRotateMode>\n");
        lz_str_addf(&t, "              <wpml:gimbalPitchRotateEnable>1</wpml:gimbalPitchRotateEnable>\n");
        lz_str_addf(&t, "              <wpml:gimbalPitchRotateAngle>%.1f</wpml:gimbalPitchRotateAngle>\n", wp->gimbalPitchDeg);
        lz_str_addf(&t, "              <wpml:gimbalRollRotateEnable>0</wpml:gimbalRollRotateEnable>\n");
        lz_str_addf(&t, "              <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>\n");
        /* 云台 yaw：数值与机头目标角一致（规范对 M4T 的硬要求）。
         *
         * 机头由 waypointHeadingMode=towardPOI 指向杆心，其目标偏航角就是
         * 「该点看向杆心」的方位角 —— 与 wp->gimbalYawDeg 是同一个角。
         * 所以这里写的值与飞机自己算出的 aircraftHeading 天然一致，
         * 满足规范的 "gimbalYawRotateAngle 与 aircraftHeading 需保持一致"。
         *
         * ⚠️ 这不是"云台独立偏转"：光轴对准杆**靠的是机头**，云台只是
         * 跟着（M4T 的 pan 轴本身不可独立控制，见文件头的说明）。
         * 曾经的实现让云台转 45° 而机头不转 —— 那个组合在 M4T 上非法。 */
        lz_str_addf(&t, "              <wpml:gimbalYawRotateEnable>1</wpml:gimbalYawRotateEnable>\n");
        lz_str_addf(&t, "              <wpml:gimbalYawRotateAngle>%.1f</wpml:gimbalYawRotateAngle>\n",
                    lz_yaw_to_signed(wp->gimbalYawDeg));
        lz_str_addf(&t, "              <wpml:gimbalRotateTimeEnable>0</wpml:gimbalRotateTimeEnable>\n");
        lz_str_addf(&t, "              <wpml:gimbalRotateTime>0</wpml:gimbalRotateTime>\n");
        lz_str_addf(&t, "              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>\n");
        lz_str_addf(&t, "            </wpml:actionActuatorFuncParam>\n");
        lz_str_addf(&t, "          </wpml:action>\n");
        lz_str_addf(&t, "        </wpml:actionGroup>\n");
        lz_str_addf(&t, "      </Placemark>\n");
    }

    /* payloadParam：template.kml 的 Folder 尾部元素。官方样例有它、我们没有。
     * 规范（20.template-kml.md 的 `<wpml:payloadParam>` 一节）里它承载的是
     * 负载对焦/测光/畸变等相机参数，而本项目不做拍照 —— 所以只写
     * payloadPositionIndex（负载挂载位置），**不加 imageFormat** 等相机项。
     * 与每个 action 里各写一遍 payloadPositionIndex 不重复：
     * 那个是"动作作用于哪个负载"，这个是"整条航线默认作用于哪个负载"。 */
    lz_str_addf(&t, "      <wpml:payloadParam>\n");
    lz_str_addf(&t, "        <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>\n");
    lz_str_addf(&t, "      </wpml:payloadParam>\n");
    lz_str_addf(&t, "    </Folder>\n  </Document>\n</kml>\n");

    /* ================= waylines.wpml ================= */
    lz_str_addf(&w, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    lz_str_addf(&w, "<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:wpml=\"http://www.dji.com/wpmz/1.0.3\">\n");
    lz_str_addf(&w, "  <Document>\n");
    lz_str_addf(&w, "    <wpml:missionConfig>\n");
    lz_str_addf(&w, "      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>\n");
    lz_str_addf(&w, "      <wpml:finishAction>%s</wpml:finishAction>\n", LZ_WPML_FINISH_ACTION);
    lz_str_addf(&w, "      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>\n");
    lz_str_addf(&w, "      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>\n");
    lz_str_addf(&w, "      <wpml:takeOffSecurityHeight>%d</wpml:takeOffSecurityHeight>\n",
                LZ_WPML_TAKEOFF_SECURITY_HEIGHT);
    lz_str_addf(&w, "      <wpml:globalTransitionalSpeed>%.1f</wpml:globalTransitionalSpeed>\n", profile->speedMs);
    lz_str_addf(&w, "      <wpml:globalRTHHeight>%.1f</wpml:globalRTHHeight>\n", rthHeight);
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
     * `LzPlan_BuildOrbit` 现在会补一个与首点重合的收尾点，所以航线是
     * **闭合**的、总长对应整圈。走直线段时总长 = n 段弦之和
     * （≈ 2πr·sinc(π/n)，比真圆略短）；走曲线段时实际轨迹更接近真圆，
     * 而这里量的是**航点间的直线距离**，是个下界 —— Pilot 显示的里程会
     * 略小于实飞距离。这个偏差在 n≥8 时小于 2%，可以接受；
     * 不做弧长估算是因为那需要知道飞机的实际转弯半径，我们没有。 */
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
        /* 逐点也写 towardPOI（飞机实际读的是这份）。
         * 每点写同一份兴趣点在协议上冗余，但 waylines.wpml 才是可执行航线，
         * 不依赖"全局值会不会被继承"这种未经验证的假设。 */
        lz_str_addf(&w, "          <wpml:waypointHeadingMode>towardPOI</wpml:waypointHeadingMode>\n");
        lz_str_addf(&w, "          <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>\n");
        lz_str_addf(&w, "          <wpml:waypointPoiPoint>%.7f,%.7f,0.000000</wpml:waypointPoiPoint>\n",
                    pole->geo.latitudeDeg, pole->geo.longitudeDeg);
        /* waypointHeadingPathMode 是 waypointHeadingParam 的必需子元素，
         * 取值跟绕行方向走 —— 理由见 template.kml 那处。 */
        lz_str_addf(&w, "          <wpml:waypointHeadingPathMode>%s</wpml:waypointHeadingPathMode>\n",
                    profile->clockwise ? "clockwise" : "counterClockwise");
        lz_str_addf(&w, "          <wpml:waypointHeadingAngleEnable>0</wpml:waypointHeadingAngleEnable>\n");
        lz_str_addf(&w, "          <wpml:waypointHeadingPoiIndex>0</wpml:waypointHeadingPoiIndex>\n");
        lz_str_addf(&w, "        </wpml:waypointHeadingParam>\n");
        lz_str_addf(&w, "        <wpml:waypointTurnParam>\n");
        lz_str_addf(&w, "          <wpml:waypointTurnMode>%s</wpml:waypointTurnMode>\n",
                    lz_turn_mode_str(profile->turnMode));
        /* 转弯截距：只对 LZ_TURN_PASS_WITH_CURVE 有意义（规范注明该元素仅在
         * coordinateTurn / toPointAndPassWithContinuityCurvature 且
         * useStraightLine=1 时必需）。走直线段时写 0。 */
        /* ⚠️ 精度用 %.2f 而不是 %.1f：规范要求该值落在 **(0, 航段最大长度]**
         * —— 是个开区间下端。半径 5 m、64 点时建议截距只有 0.22 m，
         * %.1f 会写出 "0.2"（勉强合法），再小一点就会舍成 "0.0"，
         * 直接违反开区间。%.2f 让这个裕度大得多。 */
        lz_str_addf(&w, "          <wpml:waypointTurnDampingDist>%.2f</wpml:waypointTurnDampingDist>\n",
                    dampingM);
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
        lz_str_addf(&w, "              <wpml:gimbalHeadingYawBase>north</wpml:gimbalHeadingYawBase>\n");
        lz_str_addf(&w, "              <wpml:gimbalRotateMode>absoluteAngle</wpml:gimbalRotateMode>\n");
        lz_str_addf(&w, "              <wpml:gimbalPitchRotateEnable>1</wpml:gimbalPitchRotateEnable>\n");
        lz_str_addf(&w, "              <wpml:gimbalPitchRotateAngle>%.1f</wpml:gimbalPitchRotateAngle>\n", wp->gimbalPitchDeg);
        lz_str_addf(&w, "              <wpml:gimbalRollRotateEnable>0</wpml:gimbalRollRotateEnable>\n");
        lz_str_addf(&w, "              <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>\n");
        /* 数值须与机头目标角一致 —— 理由见 template.kml 那处。 */
        lz_str_addf(&w, "              <wpml:gimbalYawRotateEnable>1</wpml:gimbalYawRotateEnable>\n");
        lz_str_addf(&w, "              <wpml:gimbalYawRotateAngle>%.1f</wpml:gimbalYawRotateAngle>\n",
                    lz_yaw_to_signed(wp->gimbalYawDeg));
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
