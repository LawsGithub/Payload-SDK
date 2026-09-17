/**
 * @file wt_plan.c
 * @brief 航线规划引擎实现
 */

#include "wt_plan.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define WT_PLAN_INIT_CAPACITY 64
#define WT_PLAN_MAX_POINTS 4096
#define WT_PLAN_MIN_SAMPLES 3

/* ------------------------------------------------------------------ */
/* 内部工具                                                            */
/* ------------------------------------------------------------------ */

/** 规划上下文：把反复用到的换算量打包，避免在每个采样点上重算 */
typedef struct {
    const WtTurbineSpec *spec;
    const WtInspectionProfile *profile;
    WtLocalFrame frame; /*!< 以塔基为原点的局部切平面 */
} WtPlanCtx;

/** 点 P 到线段 AB 的最近距离 */
static double WtPlan_PointToSegment(WtEnu p, WtEnu a, WtEnu b)
{
    WtEnu ab = WtEnu_Sub(b, a);
    WtEnu ap = WtEnu_Sub(p, a);
    double ab2 = WtEnu_Dot(ab, ab);
    double t;

    if (ab2 < 1e-12) {
        return WtEnu_Distance(p, a);
    }

    t = Wt_Clamp(WtEnu_Dot(ap, ab) / ab2, 0.0, 1.0);

    return WtEnu_Distance(p, WtEnu_Add(a, WtEnu_Scale(ab, t)));
}

/** 到第 bladeIndex 片叶片轴线的最近距离 */
static double WtPlan_DistToBladeAxis(const WtPlanCtx *ctx, const WtRotorFrame *frame,
                                     WtEnu p, int bladeIndex)
{
    WtEnu root = WtTurbine_BladePoint(frame, ctx->spec, bladeIndex, 0.0);
    WtEnu tip = WtTurbine_BladeTip(frame, ctx->spec, bladeIndex);

    return WtPlan_PointToSegment(p, root, tip);
}

/** 到所有叶片轴线的最小距离 */
static double WtPlan_MinDistToBlades(const WtPlanCtx *ctx, const WtRotorFrame *frame, WtEnu p)
{
    double best = 1e18;
    int i;

    for (i = 0; i < ctx->spec->bladeCount; i++) {
        double d = WtPlan_DistToBladeAxis(ctx, frame, p, i);

        if (d < best) {
            best = d;
        }
    }

    return best;
}

/**
 * @brief 判断某点是否落入叶片扫掠危险区
 *
 * 两种判据，按「风轮是否可能转动」选择：
 *
 *   A. 叶片静止（停机受控巡检）：只避开**当前姿态下**的叶片实体，
 *      即点到任一叶片轴线的距离必须大于安全值。这是作业必需的贴近。
 *
 *   B. 风轮可能转动（叶尖追踪 / 未受控）：「叶片当前在哪」毫无意义，
 *      因为任意一片叶片随时会扫过来。此时唯一安全的判据是保持在整个
 *      叶轮圆盘之外——点到旋转轴的轴向分量必须大于安全余量。
 *
 * 混用这两种判据是风机巡检航线最常见的错误：用 A 判据规划 B 场景，
 * 会得到一条看起来贴着叶片、实际却位于下一片叶片必经之路上的航线。
 */
static bool WtPlan_BladeHazard(const WtPlanCtx *ctx, const WtRotorFrame *frame, WtEnu p)
{
    if (ctx->profile->rotorMayRotate) {
        WtEnu v = WtEnu_Sub(p, frame->rotorCenter);
        double along = WtEnu_Dot(v, frame->rotAxis);
        WtEnu perp = WtEnu_Sub(v, WtEnu_Scale(frame->rotAxis, along));

        /* 落在叶轮圆内，且离盘面的距离不足以躲开叶片：危险 */
        return (WtEnu_Length(perp) < ctx->spec->rotorDiameter * 0.5) &&
               (fabs(along) < ctx->profile->minSafeDistM);
    }

    return WtPlan_MinDistToBlades(ctx, frame, p) < ctx->profile->minSafeDistM;
}

/**
 * @brief 判断一段航线是否穿越叶片扫掠危险区
 *
 * 沿航段密集采样（步长取安全距离的一半，保证不漏掉窄的危险区），
 * 逐点套用与单点相同的判据。相比解析求交，采样法对两种判据都适用，
 * 且能自然覆盖「斜穿叶轮圆」这类解析上不直观的情形。
 */
static bool WtPlan_SegmentIsHazardous(const WtPlanCtx *ctx, const WtRotorFrame *frame,
                                      WtEnu a, WtEnu b)
{
    double len = WtEnu_Distance(a, b);
    double step = ctx->profile->minSafeDistM * 0.5;
    int steps;
    int i;

    if (len < 1e-6) {
        return false;
    }

    steps = (int)ceil(len / step);
    if (steps > 2000) {
        steps = 2000; /* 超长航段按 2000 段采样，精度仍远高于必要 */
    }

    for (i = 1; i < steps; i++) {
        double t = (double)i / (double)steps;
        WtEnu q = WtEnu_Add(a, WtEnu_Scale(WtEnu_Sub(b, a), t));

        if (WtPlan_BladeHazard(ctx, frame, q)) {
            return true;
        }
    }

    return false;
}

/**
 * @brief 生成一个「站位点」，自动填好云台指向、斜距与 GSD
 *
 * 云台偏航取「站位 -> 目标」的水平方位角，俯仰取该方向的俯仰角
 * （目标低于站位时为负），这样相机光轴恰好指向目标。
 */
static WtPlanPoint WtPlan_MakePoint(const WtPlanCtx *ctx, WtEnu standoff, WtEnu target,
                                    const char *tag, int bladeIndex, double radialFrac, int side)
{
    WtPlanPoint p;
    WtEnu look = WtEnu_Sub(target, standoff);

    memset(&p, 0, sizeof(p));
    p.enu = standoff;
    p.geo = WtEnu_ToGeo(&ctx->frame, &standoff);
    p.bladeIndex = bladeIndex;
    p.radialFrac = radialFrac;
    p.side = side;
    p.gimbalYawDeg = WtEnu_Azimuth(look);
    p.gimbalPitchDeg = WtEnu_Elevation(look);
    p.distanceToTargetM = WtEnu_Length(look);
    p.gsdMmPerPx = WtCamera_GsdAtDistance(&ctx->profile->camera, p.distanceToTargetM);
    p.purpose = WT_PHOTO_DEFECT;
    p.tag = tag;

    return p;
}

/**
 * @brief 生成一个「转场点」：不拍照、不归属任何叶片面
 *
 * 抽出来的理由是这个构造的默认值方向原本是反的 —— WtPlan_MakePoint 默认给
 * WT_PHOTO_DEFECT，再由每个调用点各自覆盖成 WT_PHOTO_NONE。一旦哪个新增的
 * 规划或修补阶段忘了覆盖，安全圆柱面上就会多出一个触发快门的点，而它会安静地
 * 进入缺陷用图计数、GSD 判定与 KMZ 的 takePhoto 动作。让本函数成为转场点的
 * 唯一入口，这类错误就写不出来。
 */
static WtPlanPoint WtPlan_MakeTransitPoint(const WtPlanCtx *ctx, WtEnu standoff,
                                           WtEnu target, const char *tag)
{
    WtPlanPoint p = WtPlan_MakePoint(ctx, standoff, target, tag, -1, -1.0, -1);

    p.purpose = WT_PHOTO_NONE;

    return p;
}

bool WtPlanPoint_TakesPhoto(const WtPlanPoint *point)
{
    return point->purpose != WT_PHOTO_NONE;
}

bool WtPlanPoint_IsBladeInspection(const WtPlanPoint *point)
{
    if (point == NULL) {
        return false;
    }

    return point->bladeIndex >= 0 && point->side >= 0 &&
           point->purpose == WT_PHOTO_DEFECT;
}

bool WtPlan_SegmentUsesInspectSpeed(const WtPlanPoint *a, const WtPlanPoint *b)
{
    if (a == NULL || b == NULL) {
        return false; /* 首点没有入边，一律按巡航速度 */
    }

    return WtPlanPoint_IsBladeInspection(a) && WtPlanPoint_IsBladeInspection(b) &&
           a->bladeIndex == b->bladeIndex && a->side == b->side;
}

/**
 * @brief GSD 反算距离的安全系数
 *
 * 由目标 GSD 反算得到的是**上限**距离。实际站位会因 RTK 定位误差、
 * 云台指向误差、以及叶片表面不完全垂直于光轴而产生正的 GSD 偏差，
 * 因此要按系数收紧，留出余量而不是恰好卡在临界值上。
 */
#define WT_PLAN_STANDOFF_MARGIN 0.95

/**
 * @brief 把配置里的轴向偏移量收敛成一个可用比例
 *
 * 正值原样使用，负值与零同义（不做斜视），超出上界则削到上界 —— 填反了
 * 或填错量纲都不至于生成出格的航线。上界存在的意义见
 * WtPlan_ResolveAxialShift。
 *
 * 上界削顶本身是静默的，但调用方已在两处负责把这件事说出来：
 * WtInspectionProfile_Validate 直接拒绝超上界的配置，WtPlan_BuildMission
 * 对绕过校验的调用方打印一次性告警。此处保持纯函数，一行日志都不加 ——
 * 它会被报告与 demo 各调一次，在这里打印必然重复。
 *
 * @return 0 表示不做斜视；否则为 (0, 0.5] 内的比例
 */
static double WtPlan_AxialShiftFrac(const WtInspectionProfile *profile)
{
    double frac = profile->bladeAxialShiftM;

    if (frac <= 0.0) {
        return 0.0;
    }
    if (frac > WT_PLAN_AXIAL_SHIFT_MAX_FRAC) {
        frac = WT_PLAN_AXIAL_SHIFT_MAX_FRAC;
    }

    return frac;
}

double WtPlan_BladeSkewAngleDeg(const WtInspectionProfile *profile)
{
    return atan(WtPlan_AxialShiftFrac(profile)) * WT_RAD2DEG;
}

/**
 * @brief 解析拍摄距离：显式值优先，否则由目标 GSD 反算
 *
 * @param configuredM 显式指定的距离 m，0 表示自动反算
 * @param skewFrac    沿叶片轴向的偏移比例（仅叶片站位有斜视角，塔筒为 0）
 */
static double WtPlan_ResolveStandoff(const WtPlanCtx *ctx, double configuredM, double skewFrac)
{
    double limit;

    if (configuredM > 0.0) {
        return configuredM;
    }

    limit = WtCamera_MaxDistanceForGsd(&ctx->profile->camera,
                                       ctx->profile->targetGsdMmPerPx) * WT_PLAN_STANDOFF_MARGIN;

    /*
     * 斜视分量与拍照距离正交，它带来的斜距增量会把 GSD 一起推高。反算时
     * 先按 limit/√(1+f²) 取垂距，叠加斜视分量后斜距恰好回到 limit，GSD
     * 上限得以精确守住（不补偿的话，余量只剩 5%，约 0.33 的比例就会超限）。
     *
     * 仅叶片站位有斜视分量：塔筒段没有这个设计，调用方传 skewFrac = 0，
     * 否则会平白把镜头推近、白白收窄塔筒段的覆盖宽度。
     *
     * 也只对「自动反算」生效：显式给了 blade_standoff 的用户说的是「斜距就
     * 要这么多」，此时若再压缩垂距，等于悄悄把镜头推近，破坏这个明确意图。
     *
     * 注意 limit 是「斜距」上限，而返回的是「垂距」，它同时也是站位到拍摄
     * 目标（叶片轴线/塔壁）的距离。两者在正视时相等，有斜视角时不再相等 ——
     * 这是刻意的：安全下限 min_safe_dist 校验的是垂距，GSD 约束的是斜距。
     */
    if (skewFrac > 0.0) {
        limit /= sqrt(1.0 + skewFrac * skewFrac);
    }

    return limit;
}

/** 依据 standoff 与重叠率推算展向站位间距 */
static double WtPlan_BladeStepM(const WtPlanCtx *ctx, double standoffM)
{
    const WtInspectionProfile *pr = ctx->profile;
    double step;

    if (pr->bladeSpacingM > 0.0) {
        step = pr->bladeSpacingM;
    } else {
        step = WtCamera_SpacingForOverlap(&pr->camera, standoffM, pr->overlapPct, false);
    }

    return step > 0.5 ? step : 0.5;
}

/**
 * @brief 叶片站位的轴向偏移量（米），用于换取斜视角
 *
 * 偏移量按「面法向垂距」的比例给出，而不是取固定米数。原因是斜视角只由
 * 「轴向偏移 / 面法向垂距」这个比值决定，与绝对距离无关：按比例给，能让
 * 不同目标 GSD、不同机型下的斜视角保持一致，换机型时不必重新调参。
 *
 * 上界取 0.5（约 27°）是成像上的考虑：偏移量是斜边的一个分量，占比越大
 * 相机越滑向叶片侧面，前缘与叶面的夹角越接近掠射，中央区域反而先被压扁，
 * 「露出前缘」的收益被「叶面变斜」的损失吃掉。
 *
 * 注意量纲是**比例**（0.3 = 垂距的 30%），不是米。
 */
static double WtPlan_ResolveAxialShift(const WtInspectionProfile *profile, double standoffM)
{
    return standoffM * WtPlan_AxialShiftFrac(profile);
}

/**
 * @brief 推导叶片站位的采样参数（内部核心，供规划器与公开接口共用）
 *
 * 这里的每一行都同时决定三件事：站位间距、实际达成的重叠率、以及点位数。
 * 正因如此，PC 自检程序绝不能再自己算一遍 —— 少乘一个 WT_PLAN_STANDOFF_MARGIN
 * 就足以让打印出来的「29 个站位」与规划器真正的 30 个对不上，而调参的人
 * 看到的是那张打印表。
 */
static void WtPlan_ResolveSamplingCore(const WtPlanCtx *ctx, WtBladeSampling *out)
{
    const WtInspectionProfile *profile = ctx->profile;
    double span = ctx->spec->rotorDiameter * 0.5 - ctx->spec->hubRadius;
    double standoff;

    memset(out, 0, sizeof(*out));

    standoff = WtPlan_ResolveStandoff(ctx, profile->bladeStandoffM,
                                      WtPlan_AxialShiftFrac(profile));

    /* 转动模式下偏移量同时是离盘面的安全距离，见 WtPlan_BladeSamples */
    if (profile->rotorMayRotate && standoff < profile->minSafeDistM) {
        standoff = profile->minSafeDistM;
    }

    out->standoffM = standoff;
    out->axialShiftM = WtPlan_ResolveAxialShift(profile, standoff);

    /*
     * 展向步长取「重叠率推导值」与「上限裁剪值」的较大者：
     *   idealStepM —— 按画面覆盖高度与重叠率算出的理想间距；
     *   spanStepM  —— 把总点数压到 maxSamplesPerBlade 以内所需的间距。
     * 取大者保证点数不超限，同时不会因为参数误配而生成上万个航点。
     */
    out->coverageM = WtCamera_CoverageHeightAt(&profile->camera, standoff);
    out->idealStepM = WtPlan_BladeStepM(ctx, standoff);
    out->spanStepM = (profile->maxSamplesPerBlade > 1)
                         ? span / (double)(profile->maxSamplesPerBlade - 1)
                         : span;
    out->stepM = (out->idealStepM > out->spanStepM) ? out->idealStepM : out->spanStepM;

    out->actualOverlapPct =
        WtCamera_OverlapForSpacing(&profile->camera, standoff, out->stepM, false);

    out->samplesPerBlade = (int)floor(span / out->stepM) + 1;
    if (out->samplesPerBlade < WT_PLAN_MIN_SAMPLES) {
        out->samplesPerBlade = WT_PLAN_MIN_SAMPLES;
    }
}

WtPlanResult WtPlan_ResolveBladeSampling(const WtTurbineSpec *spec,
                                        const WtInspectionProfile *profile,
                                        WtBladeSampling *out)
{
    WtPlanCtx ctx;

    if (spec == NULL || profile == NULL || out == NULL) {
        return WT_PLAN_ERR_PARAM;
    }

    ctx.spec = spec;
    ctx.profile = profile;
    ctx.frame = WtLocalFrame_Init(&spec->base);

    WtPlan_ResolveSamplingCore(&ctx, out);

    return WT_PLAN_OK;
}

void WtBladeSampling_Format(const WtBladeSampling *s, char *buf, size_t bufLen)
{
    /* 斜视角只由「轴向偏移 / 垂距」这个比值决定，与绝对距离无关 */
    double skewDeg = (s != NULL && s->standoffM > 0.0)
                         ? atan(s->axialShiftM / s->standoffM) * WT_RAD2DEG
                         : 0.0;

    if (s == NULL || buf == NULL) {
        return;
    }

    snprintf(buf, bufLen,
             "采样密度\n"
             "  拍摄距离 (standoff)            : %.2f m\n"
             "  展向画面覆盖高度 @该距离       : %.2f m\n"
             "  重叠率约束的理想间距           : %.3f m\n"
             "  点数上限约束的最小间距         : %.3f m\n"
             "  -> 采用间距 %.3f m，达成重叠率 %.1f%%\n"
             "  -> 单片叶片单面站位 %d 个\n"
             "  轴向偏移                       : %.2f m (斜视角 %.1f°)\n",
             s->standoffM, s->coverageM, s->idealStepM, s->spanStepM, s->stepM,
             s->actualOverlapPct, s->samplesPerBlade, s->axialShiftM, skewDeg);
}

/* ------------------------------------------------------------------ */
/* 剖面                                                                */
/* ------------------------------------------------------------------ */

WtInspectionProfile WtInspectionProfile_Default(void)
{
    WtInspectionProfile p;

    memset(&p, 0, sizeof(p));

    p.mode = WT_MODE_BLADE_PRECISE;
    p.camera = WT_CAMERA_M4T_MID;
    p.targetGsdMmPerPx = 1.5;

    p.bladeStandoffM = 0.0; /* 0 = 由目标 GSD 反算拍摄距离 */
    p.towerStandoffM = 0.0;
    p.minSafeDistM = 3.0;
    p.groundClearanceM = 15.0;
    p.rotorMayRotate = false; /* 停机精细巡检：叶片静止，用当前姿态做包络 */

    p.bladeSide = WT_BLADE_SIDE_UPSTREAM;
    p.overlapPct = 70.0;
    p.bladeSpacingM = 0.0; /* 0 = 由重叠率推算 */
    p.maxSamplesPerBlade = 30;

    p.towerRingCount = 4;
    p.towerPerRingCount = 6;
    p.includeTower = true;

    p.coarseRadiusM = 0.0; /* 0 = 由叶轮半径自动推算 */
    p.coarsePhotoCount = 16;
    p.coarsePitchDeg = -45.0;

    p.bladeAxialShiftM = 0.0;

    p.cruiseSpeedMs = 8.0;
    p.inspectSpeedMs = 2.0;
    p.photoDwellSec = 1.5;

    p.doCoarseSurvey = true;

    return p;
}

WtValidateResult WtInspectionProfile_Validate(const WtInspectionProfile *profile)
{
    WtValidateResult r = {true, "ok"};

    if (profile == NULL) {
        r.ok = false;
        r.message = "profile is null";
        return r;
    }
    if (profile->camera.eflMm <= 0.0 || profile->camera.imageWidthPx <= 0) {
        r.ok = false;
        r.message = "camera model invalid";
        return r;
    }
    if (profile->targetGsdMmPerPx <= 0.0) {
        r.ok = false;
        r.message = "targetGsdMmPerPx must be positive";
        return r;
    }
    if (profile->bladeStandoffM < 0.0 || profile->towerStandoffM < 0.0) {
        r.ok = false;
        r.message = "standoff must not be negative (0 = auto from GSD)";
        return r;
    }
    if (profile->minSafeDistM <= 0.0) {
        r.ok = false;
        r.message = "minSafeDistM must be positive";
        return r;
    }
    /* 显式指定拍摄距离时，硬下限必须严格小于它，否则航线无解 */
    if (profile->bladeStandoffM > 0.0 && profile->minSafeDistM >= profile->bladeStandoffM) {
        r.ok = false;
        r.message = "minSafeDistM must be below bladeStandoffM";
        return r;
    }
    if (profile->overlapPct < 0.0 || profile->overlapPct >= 95.0) {
        r.ok = false;
        r.message = "overlapPct out of range [0, 95)";
        return r;
    }
    if (profile->maxSamplesPerBlade < WT_PLAN_MIN_SAMPLES) {
        r.ok = false;
        r.message = "maxSamplesPerBlade too small";
        return r;
    }

    /*
     * 轴向偏移是**比例**不是米，超上界会被静默削到 0.5。这正是现场最容易
     * 踩的量纲错误：想表达「偏移 5 米」而填了 5，程序照飞不误，只是斜视角
     * 从设想的 ~78° 变成 26.6°，前缘又回到被压扁的拍法。这种「文件说一套、
     * 程序做一套」必须在解析期就拦下来，而不是靠操作员翻文档。
     */
    if (profile->bladeAxialShiftM > WT_PLAN_AXIAL_SHIFT_MAX_FRAC) {
        r.ok = false;
        r.message = "bladeAxialShiftM is a ratio (of standoff), not meters; "
                    "must not exceed WT_PLAN_AXIAL_SHIFT_MAX_FRAC (0.5, ~26.6 deg)";
        return r;
    }
    if (profile->cruiseSpeedMs <= 0.0 || profile->inspectSpeedMs <= 0.0) {
        r.ok = false;
        r.message = "speed must be positive";
        return r;
    }
    if (profile->doCoarseSurvey && profile->coarsePhotoCount < 4) {
        r.ok = false;
        r.message = "coarsePhotoCount too small";
        return r;
    }

    return r;
}

/* ------------------------------------------------------------------ */
/* 任务容器                                                            */
/* ------------------------------------------------------------------ */

void WtMission_Init(WtMission *mission)
{
    memset(mission, 0, sizeof(*mission));
}

void WtMission_Free(WtMission *mission)
{
    free(mission->points);
    memset(mission, 0, sizeof(*mission));
}

WtPlanResult WtMission_Append(WtMission *mission, const WtPlanPoint *point)
{
    if (mission->count >= WT_PLAN_MAX_POINTS) {
        return WT_PLAN_ERR_OVERFLOW;
    }

    if (mission->count == mission->capacity) {
        size_t newCap = (mission->capacity == 0) ? WT_PLAN_INIT_CAPACITY : mission->capacity * 2;
        WtPlanPoint *newBuf = realloc(mission->points, newCap * sizeof(WtPlanPoint));

        if (newBuf == NULL) {
            return WT_PLAN_ERR_NOMEM;
        }
        mission->points = newBuf;
        mission->capacity = newCap;
    }

    mission->points[mission->count] = *point;
    mission->count++;

    return WT_PLAN_OK;
}

void WtMission_ComputeStats(WtMission *mission, const WtInspectionProfile *profile)
{
    size_t i;
    double length = 0.0;
    double duration = 0.0;
    int defectPhotos = 0;
    int modelPhotos = 0;

    for (i = 0; i < mission->count; i++) {
        const WtPlanPoint *p = &mission->points[i];

        if (p->purpose == WT_PHOTO_DEFECT) {
            defectPhotos++;
        } else if (p->purpose == WT_PHOTO_COARSE_MODEL) {
            modelPhotos++;
        }
        if (WtPlanPoint_TakesPhoto(p)) {
            duration += profile->photoDwellSec;
        }

        if (i > 0) {
            double d = WtEnu_Distance(mission->points[i - 1].enu, p->enu);
            double speed = WtPlan_SegmentUsesInspectSpeed(&mission->points[i - 1], p)
                               ? profile->inspectSpeedMs
                               : profile->cruiseSpeedMs;

            length += d;
            duration += d / speed;
        }
    }

    mission->pathLengthM = length;
    mission->durationSec = duration;
    mission->defectPhotoCount = defectPhotos;
    mission->modelPhotoCount = modelPhotos;
}

/* ------------------------------------------------------------------ */
/* 规划器                                                              */
/* ------------------------------------------------------------------ */

/**
 * @brief 粗模环绕的半径
 *
 * 环绕圆几乎与风轮盘面共面，因此半径必须大于叶轮半径，否则环上会有一段
 * 直接落在叶片扫掠的圆内。
 */
static double WtPlan_CoarseRadius(const WtTurbineSpec *spec, const WtInspectionProfile *profile)
{
    return (profile->coarseRadiusM > 0.0) ? profile->coarseRadiusM
                                          : spec->rotorDiameter * 0.5 + 12.0;
}

WtPlanResult WtPlan_CoarseSurvey(const WtTurbineSpec *spec,
                                 const WtInspectionProfile *profile,
                                 WtMission *mission)
{
    WtPlanCtx ctx;
    WtRotorFrame frame;
    WtEnu center;
    double radius;
    double startAz;
    double ringHeight;
    int i;

    ctx.spec = spec;
    ctx.profile = profile;
    ctx.frame = WtLocalFrame_Init(&spec->base);

    /* 粗模阶段相位未知，先用 0 建立参考系；环绕航线本身不依赖相位 */
    frame = WtTurbine_BuildRotorFrame(spec, 0.0);
    center = frame.rotorCenter;

    radius = WtPlan_CoarseRadius(spec, profile);

    /* 起拍方位取机舱正后方，保证第一张正对机舱背部，便于后续定向 */
    startAz = spec->headingDeg + 180.0;
    /* 环绕高度略高于风轮中心，俯视可同时覆盖机舱顶面与上叶 */
    ringHeight = center.u + spec->rotorDiameter * 0.1;

    for (i = 0; i < profile->coarsePhotoCount; i++) {
        double az = startAz + (double)i * 360.0 / (double)profile->coarsePhotoCount;
        WtEnu pos = {0.0, 0.0, 0.0};
        WtEnu target;
        WtPlanPoint pt;

        pos.e = center.e + sin(az * WT_DEG2RAD) * radius;
        pos.n = center.n + cos(az * WT_DEG2RAD) * radius;
        pos.u = ringHeight;

        /* 环绕阶段瞄准风轮中心；俯仰角交给云台，站位高度已预留俯视余量 */
        target = center;
        pt = WtPlan_MakePoint(&ctx, pos, target, "coarse_survey", -1, -1.0, -1);
        pt.gimbalPitchDeg = profile->coarsePitchDeg;
        pt.purpose = WT_PHOTO_COARSE_MODEL; /* 粗模用图不参与 GSD 质量判定 */

        if (WtMission_Append(mission, &pt) != WT_PLAN_OK) {
            return WT_PLAN_ERR_OVERFLOW;
        }
    }

    return WT_PLAN_OK;
}

WtPlanResult WtPlan_BladeSamples(const WtTurbineSpec *spec,
                                 const WtRotorFrame *frame,
                                 int bladeIndex,
                                 WtBladeSide side,
                                 const WtInspectionProfile *profile,
                                 WtPlanPoint *outPoints,
                                 size_t maxPoints,
                                 size_t *outCount)
{
    WtPlanCtx ctx;
    WtBladeSampling sampling;
    WtEnu axisDir;
    WtEnu sideNormal;
    double span;
    double standoff;
    double axialShift;
    size_t n;
    size_t i;

    if (bladeIndex < 0 || bladeIndex >= spec->bladeCount) {
        return WT_PLAN_ERR_PARAM;
    }
    if (side != WT_BLADE_SIDE_UPSTREAM && side != WT_BLADE_SIDE_DOWNSTREAM) {
        return WT_PLAN_ERR_PARAM;
    }

    ctx.spec = spec;
    ctx.profile = profile;
    ctx.frame = WtLocalFrame_Init(&spec->base);

    axisDir = WtTurbine_BladeAxisDir(frame, bladeIndex, spec);
    span = spec->rotorDiameter * 0.5 - spec->hubRadius;

    /*
     * 站位沿「巡检面法向」等距偏移，偏移量由 WtPlan_ResolveSamplingCore 统一
     * 推导 —— 那个函数同时被 PC 自检与公开接口使用，本函数只消费其结果，
     * 避免出现「规划用一种偏移、打印用另一种」的偏差。
     *
     * 面法向即旋转轴 ±，它与叶片径向严格正交，因此「站位到叶片轴线」的
     * 垂距恰好等于 standoff —— 这给了一个与展向位置无关的恒定成像距离，
     * GSD 沿整支叶片保持一致，站位也随叶片的预弯/锥角自然跟随。
     */
    WtPlan_ResolveSamplingCore(&ctx, &sampling);
    standoff = sampling.standoffM;
    axialShift = sampling.axialShiftM;
    n = (size_t)sampling.samplesPerBlade;
    sideNormal = WtEnu_Scale(frame->rotAxis,
                             (side == WT_BLADE_SIDE_UPSTREAM) ? -1.0 : 1.0);

    if (n > maxPoints) {
        return WT_PLAN_ERR_OVERFLOW;
    }

    for (i = 0; i < n; i++) {
        double s = (i == n - 1) ? span : (double)i * sampling.stepM;
        double radialFrac = (spec->hubRadius + s) / (spec->rotorDiameter * 0.5);
        WtEnu bladePoint = WtTurbine_BladePoint(frame, spec, bladeIndex, radialFrac);
        WtEnu standoffPos;

        /*
         * 站位 = 叶片点 + 面法向 × standoff + 叶片轴向 × axialShift。
         *
         * 三个分量里只有第一项是「拍摄距离」；轴向分量与叶片轴线平行，
         * 近乎不改变站位到轴线的垂距（残余量来自锥角，见上方说明），
         * 因此可以自由用来换斜视角，不必重新验算安全下限。
         */
        standoffPos = WtEnu_Add(bladePoint, WtEnu_Scale(sideNormal, standoff));
        standoffPos = WtEnu_Add(standoffPos, WtEnu_Scale(axisDir, axialShift));

        /* 相机以 bladePoint 为目标点，光轴方向即真实的观察方向 */
        outPoints[i] = WtPlan_MakePoint(&ctx, standoffPos, bladePoint, "blade", bladeIndex,
                                        radialFrac, (int)side);
    }

    *outCount = n;

    return WT_PLAN_OK;
}

/** 塔筒环绕圈数固定开销：顶部高度需低于最低叶片 */
static double WtPlan_TowerTopLimit(const WtPlanCtx *ctx, const WtRotorFrame *frame)
{
    /*
     * 塔筒段的贴塔环绕必须避开叶片扫掠区。最稳妥的上限取「当前相位下
     * 最低叶片在塔筒轴线附近的高度」减去一段安装距余量：叶片在靠近轮毂
     * 处与塔筒轴线仍有自锥角/仰角带来的平面外张距，该高度即为其裕度。
     */
    double bladeMountU = 1e18;
    int i;

    for (i = 0; i < ctx->spec->bladeCount; i++) {
        WtEnu root = WtTurbine_BladePoint(frame, ctx->spec, i, 0.0);
        double horiz = sqrt(root.e * root.e + root.n * root.n);

        /* 只统计投影落在塔筒附近的叶片，远离塔身的叶片不构成威胁 */
        if (horiz < ctx->spec->hubRadius + 8.0 && root.u < bladeMountU) {
            bladeMountU = root.u;
        }
    }

    if (bladeMountU > 1e17) {
        return ctx->spec->hubHeight * 0.95;
    }

    return bladeMountU - ctx->spec->hubRadius - 2.0;
}

WtPlanResult WtPlan_Tower(const WtTurbineSpec *spec,
                          const WtRotorFrame *frame,
                          const WtInspectionProfile *profile,
                          WtMission *mission)
{
    WtPlanCtx ctx;
    double zLow = profile->groundClearanceM;
    double zHigh;
    double standoff;
    int ring;
    int k;

    ctx.spec = spec;
    ctx.profile = profile;
    ctx.frame = WtLocalFrame_Init(&spec->base);

    standoff = WtPlan_ResolveStandoff(&ctx, profile->towerStandoffM, 0.0);

    /*
     * 风轮可能转动时不做塔筒巡检。
     *
     * 塔筒环绕必然运行在叶轮圆盘之内（塔壁到轴心的距离只有几米，远小于
     * 叶轮半径），而转动中的叶片会扫过盘内每一点 —— 这不是靠抬高或加大
     * 间距能规避的，唯一安全的选择就是不做。塔筒巡检只在停机受控时进行。
     */
    if (profile->rotorMayRotate) {
        return WT_PLAN_OK;
    }

    zHigh = WtPlan_TowerTopLimit(&ctx, frame);
    if (zHigh <= zLow) {
        return WT_PLAN_OK; /* 安全高度区间为空，安全起见直接跳过塔筒段 */
    }

    for (ring = 0; ring < profile->towerRingCount; ring++) {
        double t = (profile->towerRingCount == 1)
                       ? 0.0
                       : (double)ring / (double)(profile->towerRingCount - 1);
        double z = Wt_Lerp(zLow, zHigh, t);
        double towerR = WtTurbine_TowerRadiusAt(spec, z);
        double horizOffset = towerR + standoff;

        for (k = 0; k < profile->towerPerRingCount; k++) {
            /* 每升高一圈相位错开，形成螺旋而非竖直对齐的重复站位 */
            double az = (double)k * 360.0 / (double)profile->towerPerRingCount +
                        (double)ring * (360.0 / (double)profile->towerPerRingCount) /
                            (double)profile->towerRingCount;
            WtEnu pos = {0.0, 0.0, 0.0};
            WtEnu target = {0.0, 0.0, 0.0};
            WtPlanPoint pt;

            pos.e = sin(az * WT_DEG2RAD) * horizOffset;
            pos.n = cos(az * WT_DEG2RAD) * horizOffset;
            pos.u = z;

            /*
             * 目标取塔壁上的点而非塔筒轴线：两者对云台指向的影响只差
             * 不到 15°，但决定了「斜距」这个量的含义——取塔壁点时斜距
             * 恰好等于 towerStandoff，GSD 校验才有确定的上限。
             */
            target.e = sin(az * WT_DEG2RAD) * towerR;
            target.n = cos(az * WT_DEG2RAD) * towerR;
            target.u = z;

            pt = WtPlan_MakePoint(&ctx, pos, target, "tower", -1, -1.0, -1);

            if (WtMission_Append(mission, &pt) != WT_PLAN_OK) {
                return WT_PLAN_ERR_OVERFLOW;
            }
        }
    }

    return WT_PLAN_OK;
}

/**
 * @brief 把一点分解为「沿旋转轴的轴向高度」与「盘面内的径向距离」
 *
 * 方位角刻意在风轮平面自身的基（upRef, rotRef）里度量，而不是用 ENU 方位角。
 * 有主轴仰角时，风轮平面并不水平，两者会系统性差一个 tilt 角；如果分解用
 * 一套基、重建用另一套基，点位就会带上几米量级的误差。
 */
static void WtPlan_Cylindrical(const WtRotorFrame *frame, WtEnu p,
                              double *axial, double *radial, double *azimuthDeg)
{
    WtEnu v = WtEnu_Sub(p, frame->rotorCenter);
    double a = WtEnu_Dot(v, frame->rotAxis);
    WtEnu perp = WtEnu_Sub(v, WtEnu_Scale(frame->rotAxis, a));

    *axial = a;
    *radial = WtEnu_Length(perp);
    /* atan2 的第二/第一自变量对应 rotRef/upRef，即方位角的定义方向 */
    *azimuthDeg = Wt_Wrap360(atan2(WtEnu_Dot(perp, frame->rotRef),
                                   WtEnu_Dot(perp, frame->upRef)) * WT_RAD2DEG);
}

/**
 * @brief 在给定轴向高度、给定方位的安全圆柱面上取一点
 *
 * 用风轮平面的正交基直接构造径向分量，这样轴向高度与半径都能精确命中，
 * 不会像"用水平单位矢量乘半径"那样把 tilt 分量混进轴向里。
 */
static WtEnu WtPlan_ShellPoint(const WtRotorFrame *frame, double azimuthDeg,
                              double shellRadius, double axial)
{
    double rad = azimuthDeg * WT_DEG2RAD;
    WtEnu inPlane = WtEnu_Add(WtEnu_Scale(frame->upRef, cos(rad)),
                              WtEnu_Scale(frame->rotRef, sin(rad)));

    return WtEnu_Add(WtEnu_Add(frame->rotorCenter, WtEnu_Scale(inPlane, shellRadius)),
                     WtEnu_Scale(frame->rotAxis, axial));
}

/**
 * @brief 生成单段航程的绕行点
 *
 * 把「从 A 直接飞到 B」替换为「贴着安全圆柱面绕过去」，共四步：
 *
 *   1. 在 A 的方位上径向出壳（轴向保持 A 的高度）
 *   2. 沿圆柱面转到 B 的方位（轴向仍是 A 的高度）
 *   3. 在 B 的方位上轴向平移到 B 的高度（仍在壳上）
 *   4. 由 B 自己完成径向入壳（轴向已是 B 的高度）
 *
 * 每一步的安全依据都是同一条性质：**轴向高度恒定的段，只要该高度本身在
 * 危险带之外，全程就都在危险带之外**；而壳上的段半径大于危险半径。危险带
 * 是「轴向近零 且 径向小于叶轮半径」的圆盘，于是绕行的本质就是：先在固定
 * 高度上离开圆盘的径向范围，再在圆盘之外换高度。
 *
 * 方位角的退化要专门处理：叶根附近的站位径向距离只有几米，方位角在数值上
 * 没有意义。此时改用另一端的方位 —— 反正出壳段是等高度的，换用哪个方位
 * 都不影响安全性，只影响绕行的长短。
 */
static WtPlanResult WtPlan_AppendDetour(WtMission *out, const WtTurbineSpec *spec,
                                        const WtRotorFrame *frame,
                                        const WtInspectionProfile *profile,
                                        WtEnu a, WtEnu b)
{
    WtPlanCtx ctx;
    double axialA, axialB, radialA, radialB, azA, azB;
    double shellR;
    double sweep;
    double arcStepDeg;
    int arcCount;
    int k;

    ctx.spec = spec;
    ctx.profile = profile;
    ctx.frame = WtLocalFrame_Init(&spec->base);

    /*
     * 安全半径取「叶轮半径 + 安全距离」，而不是叶轮半径本身：壳就是航线与
     * 叶片之间的余量，余量必须显式给足，不能靠"擦着叶尖过"。
     */
    shellR = spec->rotorDiameter * 0.5 + profile->minSafeDistM;

    WtPlan_Cylindrical(frame, a, &axialA, &radialA, &azA);
    WtPlan_Cylindrical(frame, b, &axialB, &radialB, &azB);

    /* 方位角退化：叶根附近的站位径向距离太小，方位角不可用 */
    {
        const double azUsableRadius = spec->hubRadius + 10.0;

        if (radialA < azUsableRadius && radialB >= azUsableRadius) {
            azA = azB;
        } else if (radialB < azUsableRadius && radialA >= azUsableRadius) {
            azB = azA;
        } else if (radialA < azUsableRadius && radialB < azUsableRadius) {
            /*
             * 两端方位都不可用（例如叶片内段之间的转移）。此时两点都在近轴
             * 圆筒内，直接连线必然穿过危险带；接管方位角走一次绕行，虽然
             * 绕远，但保证安全。取 A 的方位作为两端的公共方位即可，
             * 出壳/入壳段等高度，弧段半径足够。
             */
            azA = 0.0;
            azB = 0.0;
        }
    }

    sweep = Wt_AngleDiff(azB, azA);

    /*
     * 圆弧分段：弦到圆心最近距离为 shellR·cos(Δθ/2)，必须不小于危险半径，
     * 否则弧的中间会重新落回危险区。留 20% 余量。
     */
    {
        double cosLimit = (spec->rotorDiameter * 0.5) / shellR;

        if (cosLimit >= 1.0) {
            arcStepDeg = 30.0;
        } else {
            double maxHalf = acos(Wt_Clamp(cosLimit, -1.0, 1.0)) * WT_RAD2DEG;

            arcStepDeg = Wt_Clamp(maxHalf * 2.0 * 0.8, 10.0, 60.0);
        }
    }
    arcCount = (int)ceil(fabs(sweep) / arcStepDeg);

    /* 步骤 1：出壳点 */
    if (radialA < shellR - 0.5) {
        WtEnu pos = WtPlan_ShellPoint(frame, azA, shellR, axialA);
        WtPlanPoint pt = WtPlan_MakeTransitPoint(&ctx, pos, frame->rotorCenter, "detour");

        if (WtMission_Append(out, &pt) != WT_PLAN_OK) {
            return WT_PLAN_ERR_OVERFLOW;
        }
    }

    /* 步骤 2：弧上中间点（不含终点，终点由步骤 3 或飞行终点覆盖） */
    for (k = 1; k < arcCount; k++) {
        double az = azA + sweep * ((double)k / (double)arcCount);
        WtEnu pos = WtPlan_ShellPoint(frame, az, shellR, axialA);
        WtPlanPoint pt = WtPlan_MakeTransitPoint(&ctx, pos, frame->rotorCenter, "detour");

        if (WtMission_Append(out, &pt) != WT_PLAN_OK) {
            return WT_PLAN_ERR_OVERFLOW;
        }
    }

    /* 步骤 3：换高度点，位于 B 的方位、B 的高度、壳上 */
    if (fabs(axialB - axialA) > 0.5 || arcCount > 0) {
        WtEnu pos = WtPlan_ShellPoint(frame, azB, shellR, axialB);
        WtPlanPoint pt = WtPlan_MakeTransitPoint(&ctx, pos, frame->rotorCenter, "detour");

        if (WtMission_Append(out, &pt) != WT_PLAN_OK) {
            return WT_PLAN_ERR_OVERFLOW;
        }
    }

    return WT_PLAN_OK;
}

/**
 * @brief 对整条航线做「危险航段修补」
 *
 * 规划器逐段生成时只保证每个航点自身安全，但两个安全航点之间的直线未必
 * 安全 —— 这正是风机巡检航线最常见的失误。这里统一在事后扫描全部航段，
 * 凡是有问题的段就地插入绕行点，直到全航线无危险段为止。
 *
 * 放在最后统一处理而不是在每处规划时各写一遍，是因为危险航段的成因五花
 * 八门（换面、叶片间转移、进出场），而绕行办法只有这一种；集中处理既避免
 * 重复实现，也保证任何新增的规划阶段自动获得同样的保护。
 */
static WtPlanResult WtPlan_RepairMission(const WtTurbineSpec *spec,
                                         const WtRotorFrame *frame,
                                         const WtInspectionProfile *profile,
                                         WtMission *mission)
{
    WtPlanCtx ctx;
    WtMission repaired;
    size_t i;
    WtPlanResult rc = WT_PLAN_OK;

    ctx.spec = spec;
    ctx.profile = profile;
    ctx.frame = WtLocalFrame_Init(&spec->base);

    WtMission_Init(&repaired);

    for (i = 0; i < mission->count; i++) {
        if (WtMission_Append(&repaired, &mission->points[i]) != WT_PLAN_OK) {
            rc = WT_PLAN_ERR_OVERFLOW;
            goto done;
        }

        if (i + 1 < mission->count) {
            WtEnu a = mission->points[i].enu;
            WtEnu b = mission->points[i + 1].enu;

            if (WtPlan_SegmentIsHazardous(&ctx, frame, a, b)) {
                if (WtPlan_AppendDetour(&repaired, spec, frame, profile, a, b) != WT_PLAN_OK) {
                    rc = WT_PLAN_ERR_OVERFLOW;
                    goto done;
                }
            }
        }
    }

done:
    WtMission_Free(mission);
    *mission = repaired;

    return rc;
}

/**
 * @brief 组装一条完整的单机巡检航线
 */
WtPlanResult WtPlan_BuildMission(const WtTurbineSpec *spec,
                                 const WtRotorFrame *frame,
                                 const WtInspectionProfile *profile,
                                 WtMission *mission)
{
    WtPlanCtx ctx;
    WtPlanPoint buf[WT_PLAN_MAX_POINTS];
    int blade;
    int sideIdx;
    int sideCount;
    WtBladeSide sides[2];

    ctx.spec = spec;
    ctx.profile = profile;
    ctx.frame = WtLocalFrame_Init(&spec->base);

    /*
     * 上界削顶在 WtPlan_AxialShiftFrac 里是静默的，且该函数被报告与 demo
     * 各调一次，在那里打日志必然重复。这里对「绕过 WtInspectionProfile_Validate
     * 直接调用规划器」的调用方补一次性告警 —— 程序化构造剖面的场景不至于
     * 无声地飞一个与预期不同的斜视角。
     */
    if (profile->bladeAxialShiftM > WT_PLAN_AXIAL_SHIFT_MAX_FRAC) {
        fprintf(stderr,
                "[plan] 警告：blade_axial_shift = %.3f 超出上界 %.2f（该参数是"
                "「拍摄距离的比例」不是米），本次按 %.2f 执行，斜视角 %.1f°。\n",
                profile->bladeAxialShiftM, (double)WT_PLAN_AXIAL_SHIFT_MAX_FRAC,
                (double)WT_PLAN_AXIAL_SHIFT_MAX_FRAC,
                atan(WT_PLAN_AXIAL_SHIFT_MAX_FRAC) * WT_RAD2DEG);
    }

    /* 阶段一：起始悬停点。抬到叶尖之上再进场，避免低空穿越叶轮扫掠面 */
    {
        WtEnu entry = {0.0, 0.0, 0.0};
        WtEnu target = frame->rotorCenter;
        WtPlanPoint pt;

        entry.n = -(spec->rotorDiameter * 0.5 + 30.0);
        entry.u = spec->hubHeight + spec->rotorDiameter * 0.5 + 20.0;

        pt = WtPlan_MakeTransitPoint(&ctx, entry, target, "entry");

        if (WtMission_Append(mission, &pt) != WT_PLAN_OK) {
            return WT_PLAN_ERR_OVERFLOW;
        }
    }

    /* 阶段二：粗模环绕建模 */
    if (profile->doCoarseSurvey) {
        if (WtPlan_CoarseSurvey(spec, profile, mission) != WT_PLAN_OK) {
            return WT_PLAN_ERR_OVERFLOW;
        }
    }

    /* 阶段三：逐叶片精细巡检 */
    if (profile->bladeSide == WT_BLADE_SIDE_BOTH) {
        sides[0] = WT_BLADE_SIDE_UPSTREAM;
        sides[1] = WT_BLADE_SIDE_DOWNSTREAM;
        sideCount = 2;
    } else {
        sides[0] = profile->bladeSide;
        sideCount = 1;
    }

    for (blade = 0; blade < spec->bladeCount; blade++) {
        for (sideIdx = 0; sideIdx < sideCount; sideIdx++) {
            size_t n = 0;
            size_t i;
            WtPlanResult rc = WtPlan_BladeSamples(spec, frame, blade, sides[sideIdx], profile,
                                                  buf, WT_PLAN_MAX_POINTS, &n);

            if (rc != WT_PLAN_OK) {
                return rc;
            }

            for (i = 0; i < n; i++) {
                if (WtMission_Append(mission, &buf[i]) != WT_PLAN_OK) {
                    return WT_PLAN_ERR_OVERFLOW;
                }
            }
        }
    }

    /* 阶段四：塔筒垂直巡检。风轮可能转动时该段会被整体跳过 */
    if (profile->includeTower) {
        size_t before = mission->count;

        if (WtPlan_Tower(spec, frame, profile, mission) != WT_PLAN_OK) {
            return WT_PLAN_ERR_OVERFLOW;
        }
        if (mission->count == before) {
            fprintf(stderr,
                    "[plan] 警告：风轮可能转动，塔筒巡检段已跳过。\n"
                    "[plan]       塔筒环绕必然位于叶轮圆盘之内，转动中的叶片会\n"
                    "[plan]       扫过盘内每一点，该段无法通过加大间距规避。\n"
                    "[plan]       如需巡检塔筒，请改为停机受控作业。\n");
        }
    }

    /* 阶段五：退场点，先侧向脱离再抬高返航，避免从叶轮正下方穿过 */
    {
        WtEnu exitPos = {0.0, 0.0, 0.0};
        WtEnu target = frame->rotorCenter;
        WtPlanPoint pt;

        exitPos.e = spec->rotorDiameter * 0.5 + 30.0;
        exitPos.u = spec->hubHeight + spec->rotorDiameter * 0.5 + 20.0;

        pt = WtPlan_MakeTransitPoint(&ctx, exitPos, target, "exit");

        if (WtMission_Append(mission, &pt) != WT_PLAN_OK) {
            return WT_PLAN_ERR_OVERFLOW;
        }
    }

    /*
     * 收尾：扫描全部航段，把穿越叶片扫掠区的航段替换为绕行路径。
     * 必须在算统计之前做，否则航程与耗时会把绕行段漏掉。
     */
    {
        WtPlanResult rrc = WtPlan_RepairMission(spec, frame, profile, mission);

        if (rrc != WT_PLAN_OK) {
            return rrc;
        }
    }

    WtMission_ComputeStats(mission, profile);

    return WT_PLAN_OK;
}

/* ------------------------------------------------------------------ */
/* 校验                                                                */
/* ------------------------------------------------------------------ */

WtSafetyReport WtPlan_ValidateMission(const WtTurbineSpec *spec,
                                      const WtRotorFrame *frame,
                                      const WtInspectionProfile *profile,
                                      const WtMission *mission)
{
    WtPlanCtx ctx;
    WtSafetyReport rep;
    size_t i;
    double sumDefectGsd = 0.0;
    double sumModelGsd = 0.0;
    int defectCount = 0;
    int modelCount = 0;

    memset(&rep, 0, sizeof(rep));
    rep.ok = true;
    rep.requiredSafeDistM = profile->minSafeDistM;
    rep.targetGsdMmPerPx = profile->targetGsdMmPerPx;
    rep.requiredGsdM = WtCamera_MaxDistanceForGsd(&profile->camera, profile->targetGsdMmPerPx);
    rep.minSafeDistM = 1e18;

    ctx.spec = spec;
    ctx.profile = profile;
    ctx.frame = WtLocalFrame_Init(&spec->base);

    for (i = 0; i < mission->count; i++) {
        const WtPlanPoint *p = &mission->points[i];
        double dBlade = WtPlan_MinDistToBlades(&ctx, frame, p->enu);

        if (dBlade < rep.minSafeDistM) {
            rep.minSafeDistM = dBlade;
        }
        if (dBlade < profile->minSafeDistM) {
            rep.minSafeDistViolations++;
            rep.ok = false;
        }

        /* 离地高度以塔基坐标为基准 */
        if (p->enu.u < profile->groundClearanceM) {
            rep.groundViolations++;
            rep.ok = false;
        }

        /*
         * 叶片扫掠危险区检查。采用与规划器同源的判据，保证「规划通过」
         * 与「校验通过」不会因为两套逻辑而不一致。
         */
        if (WtPlan_BladeHazard(&ctx, frame, p->enu)) {
            rep.bladeHazardViolations++;
            rep.ok = false;
        }

        /*
         * 航段检查：危险区往往落在两个航点之间（例如从盘正面越到背面）。
         * 只查航点会漏掉整段贴着叶片飞过去的路径。
         */
        if (i > 0 && WtPlan_SegmentIsHazardous(&ctx, frame,
                                               mission->points[i - 1].enu, p->enu)) {
            rep.bladeHazardViolations++;
            rep.ok = false;
        }

        /*
         * GSD 只对「叶片精细巡检」环节有意义：它决定缺陷能否被分辨。
         * 粗模环绕与塔筒照片不以分辨率为指标，混入统计会掩盖真实问题。
         */
        if (p->purpose == WT_PHOTO_DEFECT) {
            sumDefectGsd += p->gsdMmPerPx;
            defectCount++;
            if (p->gsdMmPerPx > rep.maxDefectGsdMmPerPx) {
                rep.maxDefectGsdMmPerPx = p->gsdMmPerPx;
            }
            if (p->distanceToTargetM > rep.maxDefectDistanceM) {
                rep.maxDefectDistanceM = p->distanceToTargetM;
            }
            if (p->gsdMmPerPx > profile->targetGsdMmPerPx * 1.001) {
                rep.gsdViolations++;
                rep.ok = false;
            }
        } else if (p->purpose == WT_PHOTO_COARSE_MODEL) {
            sumModelGsd += p->gsdMmPerPx;
            modelCount++;
        }
    }

    rep.avgDefectGsdMmPerPx = (defectCount > 0) ? sumDefectGsd / (double)defectCount : 0.0;
    rep.avgModelGsdMmPerPx = (modelCount > 0) ? sumModelGsd / (double)modelCount : 0.0;
    if (mission->count == 0) {
        rep.minSafeDistM = 0.0;
        rep.ok = false;
    }

    return rep;
}

void WtSafetyReport_Format(const WtSafetyReport *report, char *buf, size_t bufLen)
{
    snprintf(buf, bufLen,
             "safety     : %s\n"
             "  min dist to blade axis  : %.2f m (limit %.2f m)\n"
             "  violations : blade-dist %d, ground %d, blade-zone %d, GSD %d\n"
             "  defect GSD : avg %.3f mm/px, worst %.3f mm/px (target %.3f)\n"
             "  defect dist: max %.2f m (limit %.2f m for target GSD)\n"
             "  model  GSD : avg %.3f mm/px (仅记录，不作判定)\n",
             report->ok ? "PASS" : "FAIL",
             report->minSafeDistM,
             report->requiredSafeDistM,
             report->minSafeDistViolations,
             report->groundViolations,
             report->bladeHazardViolations,
             report->gsdViolations,
             report->avgDefectGsdMmPerPx,
             report->maxDefectGsdMmPerPx,
             report->targetGsdMmPerPx,
             report->maxDefectDistanceM,
             report->requiredGsdM,
             report->avgModelGsdMmPerPx);
}

WtPlanResult WtMission_ExportCsv(const WtMission *mission, const char *path)
{
    FILE *fp = fopen(path, "w");
    size_t i;

    if (fp == NULL) {
        return WT_PLAN_ERR_PARAM;
    }

    fprintf(fp, "index,tag,blade_index,radial_frac,side,lat,lon,alt,"
                "east,north,up,gimbal_yaw_deg,gimbal_pitch_deg,"
                "distance_m,gsd_mm_per_px,purpose\n");

    for (i = 0; i < mission->count; i++) {
        const WtPlanPoint *p = &mission->points[i];

        fprintf(fp, "%zu,%s,%d,%.4f,%d,%.9f,%.9f,%.3f,%.3f,%.3f,%.3f,"
                    "%.2f,%.2f,%.3f,%.4f,%d\n",
                i, p->tag ? p->tag : "", p->bladeIndex, p->radialFrac, p->side,
                p->geo.lat, p->geo.lon, p->geo.alt,
                p->enu.e, p->enu.n, p->enu.u,
                p->gimbalYawDeg, p->gimbalPitchDeg,
                p->distanceToTargetM, p->gsdMmPerPx, (int)p->purpose);
    }

    fclose(fp);

    return WT_PLAN_OK;
}