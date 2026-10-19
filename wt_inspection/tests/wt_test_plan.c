/**
 * @file wt_test_plan.c
 * @brief 规划器回归测试：航段速度判据、采样密度、安全校验
 *
 * 重点是那条曾经出错的规则：哪些航段算「巡检段」。
 * 曾经用「两端 bladeIndex >= 0」判定，结果把叶尖到叶根的叶片间转移段
 * （实测 80.3 m）也按 2 m/s 计费，报告上的预计耗时因此偏高，而 KMZ 里
 * 飞机真正的飞法又是另一套判据。下面的用例把这条规则钉死。
 */

#include <math.h>
#include <string.h>

#include "wt_bridge.h"
#include "wt_plan.h"
#include "wt_test.h"
#include "wt_test_fixture.h"

/* ------------------------------------------------------------------ */
/* 判据真值表                                                          */
/* ------------------------------------------------------------------ */

static void TestBladeInspectionPredicate(void)
{
    WtPlanPoint blade = WtTest_MakePoint(0, 0, 0, 0, 0.5, 0, WT_PHOTO_DEFECT);
    WtPlanPoint otherSide = WtTest_MakePoint(0, 0, 0, 0, 0.5, 1, WT_PHOTO_DEFECT);
    WtPlanPoint transit = WtTest_MakePoint(0, 0, 0, -1, -1.0, -1, WT_PHOTO_NONE);
    WtPlanPoint tower = WtTest_MakePoint(0, 0, 0, -1, -1.0, -1, WT_PHOTO_DEFECT);
    WtPlanPoint coarse = WtTest_MakePoint(0, 0, 0, -1, -1.0, -1, WT_PHOTO_COARSE_MODEL);
    WtPlanPoint noSide = WtTest_MakePoint(0, 0, 0, 0, 0.5, -1, WT_PHOTO_DEFECT);

    WT_CASE("WtPlanPoint_IsBladeInspection 真值表");
    WT_CHECK(WtPlanPoint_IsBladeInspection(&blade));
    WT_CHECK(WtPlanPoint_IsBladeInspection(&otherSide));
    WT_CHECK(!WtPlanPoint_IsBladeInspection(&transit));
    WT_CHECK(!WtPlanPoint_IsBladeInspection(&tower));
    WT_CHECK(!WtPlanPoint_IsBladeInspection(&coarse));
    WT_CHECK(!WtPlanPoint_IsBladeInspection(&noSide));
    WT_CHECK(!WtPlanPoint_IsBladeInspection(NULL));
}

static void TestSegmentSpeedPredicate(void)
{
    WtPlanPoint b0a = WtTest_MakePoint(0, 0, 0, 0, 0.2, 0, WT_PHOTO_DEFECT);
    WtPlanPoint b0b = WtTest_MakePoint(2.5, 0, 0, 0, 0.3, 0, WT_PHOTO_DEFECT);
    WtPlanPoint b0otherSide = WtTest_MakePoint(2.5, 0, 0, 0, 0.3, 1, WT_PHOTO_DEFECT);
    WtPlanPoint b1 = WtTest_MakePoint(80, 0, 0, 1, 0.2, 0, WT_PHOTO_DEFECT);
    WtPlanPoint transit = WtTest_MakePoint(40, 0, 0, -1, -1.0, -1, WT_PHOTO_NONE);

    WT_CASE("WtPlan_SegmentUsesInspectSpeed 真值表");
    WT_CHECK(WtPlan_SegmentUsesInspectSpeed(&b0a, &b0b));       /* 同片同面相邻 */
    WT_CHECK(!WtPlan_SegmentUsesInspectSpeed(&b0b, &b1));       /* 叶片间转移 */
    WT_CHECK(!WtPlan_SegmentUsesInspectSpeed(&b0a, &b0otherSide)); /* 同片换面 */
    WT_CHECK(!WtPlan_SegmentUsesInspectSpeed(&b0b, &transit));  /* 进出转场点 */
    WT_CHECK(!WtPlan_SegmentUsesInspectSpeed(&transit, &b0b));
    WT_CHECK(!WtPlan_SegmentUsesInspectSpeed(&transit, &transit));
    WT_CHECK(!WtPlan_SegmentUsesInspectSpeed(NULL, &b0a));      /* 首点无入边 */
    WT_CHECK(!WtPlan_SegmentUsesInspectSpeed(&b0a, NULL));
}

/* ------------------------------------------------------------------ */
/* 耗时核算：用人工构造的航点做精确断言                                */
/* ------------------------------------------------------------------ */

/**
 * 造一条「巡检段 + 跨叶片转移」的航线，分别用两种判据核算。
 *
 * 四点等距在 x 轴上，段长 10 / 100 / 10：
 *   A(-0,0) --10m--> B(10,0) --100m--> C(110,0) --10m--> D(120,0)
 * A→B、C→D 是同一片叶片的相邻站位；B→C 是叶片 0 到叶片 1 的转移。
 */
static void TestComputeStatsSegmentRule(void)
{
    WtInspectionProfile p = WtTest_ProfileTiny();
    WtMission m;

    WT_CASE("ComputeStats：叶片间转移不按巡检速度计费");

    WtMission_Init(&m);
    {
        WtPlanPoint a = WtTest_MakePoint(0, 0, 0, 0, 0.2, 0, WT_PHOTO_DEFECT);
        WtPlanPoint b = WtTest_MakePoint(10, 0, 0, 0, 0.3, 0, WT_PHOTO_DEFECT);
        WtPlanPoint c = WtTest_MakePoint(110, 0, 0, 1, 0.2, 0, WT_PHOTO_DEFECT);
        WtPlanPoint d = WtTest_MakePoint(120, 0, 0, 1, 0.3, 0, WT_PHOTO_DEFECT);

        WT_CHECK_EQ_INT(WtMission_Append(&m, &a), WT_PLAN_OK);
        WT_CHECK_EQ_INT(WtMission_Append(&m, &b), WT_PLAN_OK);
        WT_CHECK_EQ_INT(WtMission_Append(&m, &c), WT_PLAN_OK);
        WT_CHECK_EQ_INT(WtMission_Append(&m, &d), WT_PLAN_OK);
    }

    WtMission_ComputeStats(&m, &p);

    WT_CHECK_NEAR(m.pathLengthM, 120.0, 1e-9);
    /* 4×1.5 悬停 + 10/2 巡检 + 100/8 转移 + 10/2 巡检 = 6 + 5 + 12.5 + 5 */
    WT_CHECK_NEAR(m.durationSec, 28.5, 1e-9);
    WT_CHECK_EQ_INT(m.defectPhotoCount, 4);
    WT_CHECK_EQ_INT(m.modelPhotoCount, 0);

    WtMission_Free(&m);
}

static void TestComputeStatsAllInspection(void)
{
    WtInspectionProfile p = WtTest_ProfileTiny();
    WtMission m;

    WT_CASE("ComputeStats：全程同片同面时全部按巡检速度");

    WtMission_Init(&m);
    {
        WtPlanPoint a = WtTest_MakePoint(0, 0, 0, 0, 0.2, 0, WT_PHOTO_DEFECT);
        WtPlanPoint b = WtTest_MakePoint(10, 0, 0, 0, 0.3, 0, WT_PHOTO_DEFECT);
        WtPlanPoint c = WtTest_MakePoint(110, 0, 0, 0, 0.4, 0, WT_PHOTO_DEFECT);
        WtPlanPoint d = WtTest_MakePoint(120, 0, 0, 0, 0.5, 0, WT_PHOTO_DEFECT);

        WtMission_Append(&m, &a);
        WtMission_Append(&m, &b);
        WtMission_Append(&m, &c);
        WtMission_Append(&m, &d);
    }

    WtMission_ComputeStats(&m, &p);

    /* 4×1.5 + 120/2 = 6 + 60。这条用例证明判据不是「永远返回 false」 */
    WT_CHECK_NEAR(m.durationSec, 66.0, 1e-9);

    WtMission_Free(&m);
}

/* ------------------------------------------------------------------ */
/* 端到端：默认剖面                                                    */
/* ------------------------------------------------------------------ */

/** 断言「被判为巡检速度的航段里不存在大跨度转移」——本文件的核心守卫 */
static void CheckNoLongInspectionSegment(const WtMission *m, double maxM)
{
    size_t i;
    double worst = 0.0;

    for (i = 1; i < m->count; i++) {
        if (WtPlan_SegmentUsesInspectSpeed(&m->points[i - 1], &m->points[i])) {
            double d = WtEnu_Distance(m->points[i - 1].enu, m->points[i].enu);

            if (d > worst) {
                worst = d;
            }
        }
    }

    WT_CHECK_BETWEEN(worst, 0.0, maxM);
}

static void TestDefaultMission155(void)
{
    WtTurbineSpec spec = WtTest_Spec155();
    WtInspectionProfile profile = WtInspectionProfile_Default();
    WtRotorFrame frame = WtTurbine_BuildRotorFrame(&spec, 0.0);
    WtBladeSampling sampling;
    WtMission m;
    WtSafetyReport rep;

    WT_CASE("155m/100m 默认剖面：规划结果与基准值一致");

    WtMission_Init(&m);
    WT_CHECK_EQ_INT(WtPlan_BuildMission(&spec, &frame, &profile, &m), WT_PLAN_OK);

    WT_CHECK_EQ_INT(m.count, 132);
    WT_CHECK_EQ_INT(m.defectPhotoCount, 114);
    WT_CHECK_EQ_INT(m.modelPhotoCount, 16);
    WT_CHECK_BETWEEN(m.pathLengthM, 1994.0, 1999.0);

    /* 巡检段最长 2.52m；改动前叶片间转移段 80.3m 会在这里被判为巡检段 */
    CheckNoLongInspectionSegment(&m, 6.0);

    rep = WtPlan_ValidateMission(&spec, &frame, &profile, &m);
    WT_CHECK(rep.ok);
    WT_CHECK_EQ_INT(rep.bladeHazardViolations, 0);
    WT_CHECK_EQ_INT(rep.minSafeDistViolations, 0);
    WT_CHECK_EQ_INT(rep.groundViolations, 0);
    WT_CHECK_EQ_INT(rep.gsdViolations, 0);
    WT_CHECK_BETWEEN(rep.minSafeDistM, 7.0, 7.5);

    WT_CHECK_EQ_INT(WtPlan_ResolveBladeSampling(&spec, &profile, &sampling), WT_PLAN_OK);
    WT_CHECK_EQ_INT(sampling.samplesPerBlade, 30);
    WT_CHECK_BETWEEN(sampling.standoffM, 23.0, 23.1);
    WT_CHECK_BETWEEN(sampling.coverageM, 8.5, 8.6);
    /* 采用间距受点数上限约束，达成重叠率略低于配置的 70% */
    WT_CHECK_BETWEEN(sampling.stepM, 2.58, 2.59);
    WT_CHECK_BETWEEN(sampling.actualOverlapPct, 69.0, 70.0);

    WtMission_Free(&m);
}

static void TestDefaultMission191(void)
{
    WtTurbineSpec spec = WtTest_Spec191();
    WtInspectionProfile profile = WtInspectionProfile_Default();
    WtRotorFrame frame = WtTurbine_BuildRotorFrame(&spec, 0.0);
    WtBladeSampling sampling;
    WtMission m;

    WT_CASE("191m/110m 默认剖面：点数上限把站位钉在 30 个");

    WtMission_Init(&m);
    WT_CHECK_EQ_INT(WtPlan_BuildMission(&spec, &frame, &profile, &m), WT_PLAN_OK);

    /* 展向更长，但 maxSamplesPerBlade=30 生效，站位数与 155m 相同 */
    WT_CHECK_EQ_INT(WtPlan_ResolveBladeSampling(&spec, &profile, &sampling), WT_PLAN_OK);
    WT_CHECK_EQ_INT(sampling.samplesPerBlade, 30);
    WT_CHECK_EQ_INT(m.count, 132);
    CheckNoLongInspectionSegment(&m, 6.0);

    WtMission_Free(&m);
}

static void TestBothSidesMission(void)
{
    WtTurbineSpec spec = WtTest_Spec155();
    WtInspectionProfile profile = WtInspectionProfile_Default();
    WtRotorFrame frame = WtTurbine_BuildRotorFrame(&spec, 0.0);
    WtMission m;
    WtSafetyReport rep;

    WT_CASE("双面剖面：换面段不按巡检速度计费");

    profile.bladeSide = WT_BLADE_SIDE_BOTH;

    WtMission_Init(&m);
    WT_CHECK_EQ_INT(WtPlan_BuildMission(&spec, &frame, &profile, &m), WT_PLAN_OK);

    WT_CHECK_EQ_INT(m.count, 232);
    /* 换面段是同一片叶片、不同 side 之间约 48m 的横跨；判据漏掉 side
       相等这一项时，这里会出现长航段。 */
    CheckNoLongInspectionSegment(&m, 6.0);

    rep = WtPlan_ValidateMission(&spec, &frame, &profile, &m);
    WT_CHECK(rep.ok);

    WtMission_Free(&m);
}

static void TestRotatingMode(void)
{
    WtTurbineSpec spec = WtTest_Spec155();
    WtInspectionProfile profile = WtInspectionProfile_Default();
    WtRotorFrame frame = WtTurbine_BuildRotorFrame(&spec, 0.0);
    WtMission m;
    WtSafetyReport rep;
    size_t i;
    int towerPoints = 0;

    WT_CASE("转动模式：塔筒段整段跳过，安全校验仍通过");

    profile.mode = WT_MODE_BLADE_ROTATING;
    profile.rotorMayRotate = true;

    WtMission_Init(&m);
    WT_CHECK_EQ_INT(WtPlan_BuildMission(&spec, &frame, &profile, &m), WT_PLAN_OK);

    for (i = 0; i < m.count; i++) {
        if (m.points[i].tag != NULL && strcmp(m.points[i].tag, "tower") == 0) {
            towerPoints++;
        }
    }
    WT_CHECK_EQ_INT(towerPoints, 0);

    rep = WtPlan_ValidateMission(&spec, &frame, &profile, &m);
    WT_CHECK(rep.ok);
    CheckNoLongInspectionSegment(&m, 6.0);

    WtMission_Free(&m);
}

/* ------------------------------------------------------------------ */
/* 绕行修补点的语义                                                    */
/* ------------------------------------------------------------------ */

static void TestDetourPointsAreTransit(void)
{
    WtTurbineSpec spec = WtTest_Spec155();
    WtInspectionProfile profile = WtInspectionProfile_Default();
    WtRotorFrame frame = WtTurbine_BuildRotorFrame(&spec, 0.0);
    WtMission m;
    size_t i;
    int detours = 0;

    WT_CASE("绕行修补点：不拍照、不归属叶片、不按巡检速度计费");

    /* 双面剖面必然会触发换面段的绕行修补 */
    profile.bladeSide = WT_BLADE_SIDE_BOTH;

    WtMission_Init(&m);
    WT_CHECK_EQ_INT(WtPlan_BuildMission(&spec, &frame, &profile, &m), WT_PLAN_OK);

    for (i = 0; i < m.count; i++) {
        const WtPlanPoint *p = &m.points[i];

        if (p->tag != NULL && strcmp(p->tag, "detour") == 0) {
            detours++;
            /* 安全圆柱面上的点绝不能触发快门，也不能被当成叶片工位 */
            WT_CHECK_EQ_INT(p->purpose, WT_PHOTO_NONE);
            WT_CHECK(!WtPlanPoint_TakesPhoto(p));
            WT_CHECK(!WtPlanPoint_IsBladeInspection(p));
            WT_CHECK_EQ_INT(p->bladeIndex, -1);
        }
        if (i > 0) {
            const WtPlanPoint *q = &m.points[i - 1];

            if ((q->tag != NULL && strcmp(q->tag, "detour") == 0) ||
                (p->tag != NULL && strcmp(p->tag, "detour") == 0)) {
                WT_CHECK(!WtPlan_SegmentUsesInspectSpeed(q, p));
            }
        }
    }

    WT_CHECK(detours > 0);

    WtMission_Free(&m);
}

/* ------------------------------------------------------------------ */
/* 剖面校验：轴向偏移的比例语义                                        */
/* ------------------------------------------------------------------ */

static void TestAxialShiftValidation(void)
{
    WtInspectionProfile p = WtInspectionProfile_Default();

    WT_CASE("blade_axial_shift 是比例：超上界必须被拒绝而非静默削顶");

    p.bladeAxialShiftM = WT_PLAN_AXIAL_SHIFT_MAX_FRAC;
    WT_CHECK(WtInspectionProfile_Validate(&p).ok);

    p.bladeAxialShiftM = WT_PLAN_AXIAL_SHIFT_MAX_FRAC + 0.0001;
    WT_CHECK(!WtInspectionProfile_Validate(&p).ok);

    /* 现场最容易踩的坑：想表达「偏移 5 米」而填了 5 */
    p.bladeAxialShiftM = 5.0;
    WT_CHECK(!WtInspectionProfile_Validate(&p).ok);

    p.bladeAxialShiftM = 0.5;
    WT_CHECK_NEAR(WtPlan_BladeSkewAngleDeg(&p), atan(0.5) * WT_RAD2DEG, 1e-9);
}

/* ------------------------------------------------------------------ */
/* 沿中线飞行（trace）模式                                             */
/* ------------------------------------------------------------------ */

/** 统计某类 tag 的点数 */
static int CountTag(const WtMission *m, const char *tag)
{
    int n = 0;
    size_t i;

    for (i = 0; i < m->count; i++) {
        if (m->points[i].tag != NULL && strcmp(m->points[i].tag, tag) == 0) {
            n++;
        }
    }

    return n;
}

/**
 * 拓扑：每片叶片的往返必须在同一叶片内连续，叶片之间经轮毂转场。
 *
 * 这条用例钉死的是一个曾经真实出现过的错误：可视化第一版按「段类型」
 * 分组画线，把 1 号叶尖与 2 号叶尖连成了一条 141.7 m 的假航段。
 * 只要生成器出现跨叶片直连，本用例的相邻间距检查就会红。
 */
static void TestTraceTopology(void)
{
    WtTurbineSpec spec = WtTest_Spec155();
    WtTraceProfile tp = WtTraceProfile_Default();
    WtMission m;
    WtRotorFrame f;
    size_t i;
    double maxGap = 0.0;

    WtMission_Init(&m);
    f = WtTurbine_BuildRotorFrame(&spec, 0.0);

    WT_CASE("trace：航线拓扑与段连续性");
    WT_CHECK_EQ_INT((int)WtPlan_BladeTrace(&spec, &f, &tp, &m), (int)WT_PLAN_OK);

    /* 三段的数量：去程/回程各 nStep+1 点，掉头 arcSegments-1 点 */
    WT_CHECK_EQ_INT(CountTag(&m, "trace_capture"), 1);
    WT_CHECK_EQ_INT(CountTag(&m, "trace_end"), 1);
    WT_CHECK_EQ_INT(CountTag(&m, "trace_hub"), 4);   /* 进入 + 每叶片收尾 + 末次 */

    /* 相邻点间距：轮毂转场段最长（approach - offset = 20 m），
     * 任何超过它的间距都意味着跨叶片直连 */
    for (i = 1; i < m.count; i++) {
        double d = WtEnu_Distance(m.points[i].enu, m.points[i - 1].enu);

        if (d > maxGap) {
            maxGap = d;
        }
    }
    WT_CHECK(maxGap <= 21.0);

    /* 掉头段必须归属某片叶片（towardPOI 的兴趣点要靠它定位叶尖） */
    for (i = 0; i < m.count; i++) {
        if (strcmp(m.points[i].tag, "trace_arc") == 0) {
            WT_CHECK(m.points[i].bladeIndex >= 0);
        }
    }

    /* 叶尖坐标已记录，供 KMZ 的 towardPOI 使用 */
    WT_CHECK_EQ_INT((int)m.bladeTipCount, spec.bladeCount);
    WT_CHECK(m.bladeTips != NULL);

    WtMission_Free(&m);
}

/** 去程与回程必须分居盘面两侧 —— 这正是「一趟覆盖两面」的判据 */
static void TestTraceCoversBothSides(void)
{
    WtTurbineSpec spec = WtTest_Spec155();
    WtTraceProfile tp = WtTraceProfile_Default();
    WtMission m;
    WtRotorFrame f;
    int blade;

    WtMission_Init(&m);
    f = WtTurbine_BuildRotorFrame(&spec, 0.0);

    WT_CASE("trace：每片叶片的去程/回程分居盘面两侧");
    WT_CHECK_EQ_INT((int)WtPlan_BladeTrace(&spec, &f, &tp, &m), (int)WT_PLAN_OK);

    for (blade = 0; blade < spec.bladeCount; blade++) {
        double outSum = 0.0, backSum = 0.0;
        int outN = 0, backN = 0;
        size_t i;

        for (i = 0; i < m.count; i++) {
            const WtPlanPoint *p = &m.points[i];
            double along;

            if (p->bladeIndex != blade) {
                continue;
            }
            along = WtEnu_Dot(WtEnu_Sub(p->enu, f.rotorCenter), f.rotAxis);

            if (strcmp(p->tag, "trace_out") == 0) {
                outSum += along;
                outN++;
            } else if (strcmp(p->tag, "trace_back") == 0) {
                backSum += along;
                backN++;
            }
        }
        WT_CHECK(outN > 0 && backN > 0);
        /* 去程与回程的轴向均值必须异号，否则等于只拍了一面 */
        WT_CHECK(outSum * backSum < 0.0);
    }

    WtMission_Free(&m);
}

/** 偏置量必须真的体现在航点到叶片轴线的距离上 */
static void TestTraceOffsetRespected(void)
{
    WtTurbineSpec spec = WtTest_Spec155();
    WtTraceProfile tp = WtTraceProfile_Default();
    WtMission m;
    WtRotorFrame f;
    double minD = 1e18;
    size_t i;

    WtMission_Init(&m);
    f = WtTurbine_BuildRotorFrame(&spec, 0.0);
    WT_CASE("trace：航点到叶片轴线的距离 ≈ offset");
    WT_CHECK_EQ_INT((int)WtPlan_BladeTrace(&spec, &f, &tp, &m), (int)WT_PLAN_OK);

    for (i = 0; i < m.count; i++) {
        const WtPlanPoint *p = &m.points[i];
        double d;
        WtEnu root, tip, ab, ap, c;
        double t;

        if (p->bladeIndex < 0 || p->side < 0) {
            continue;   /* 只查叶片巡检点，掉头/转场点不在此列 */
        }
        root = WtTurbine_BladePoint(&f, &spec, p->bladeIndex, 0.0);
        tip = WtTurbine_BladeTip(&f, &spec, p->bladeIndex);
        ab = WtEnu_Sub(tip, root);
        ap = WtEnu_Sub(p->enu, root);
        t = Wt_Clamp(WtEnu_Dot(ap, ab) / WtEnu_Dot(ab, ab), 0.0, 1.0);
        c = WtEnu_Add(root, WtEnu_Scale(ab, t));
        d = WtEnu_Distance(p->enu, c);
        if (d < minD) {
            minD = d;
        }
    }
    /* 锥角会让偏置略大于设定值（实测 9.20~10.8），故给一个区间而不是等值 */
    WT_CHECK_BETWEEN(minD, tp.offsetM * 0.9, tp.offsetM * 1.2);

    WtMission_Free(&m);
}

/** 剖面参数校验 */
static void TestTraceProfileValidation(void)
{
    WtTraceProfile tp = WtTraceProfile_Default();

    WT_CASE("trace：剖面参数校验");
    WT_CHECK(WtTraceProfile_Validate(&tp).ok);

    tp.offsetM = 0.0;
    WT_CHECK(!WtTraceProfile_Validate(&tp).ok);
    tp = WtTraceProfile_Default();

    tp.arcSegments = 1;   /* 1 段退化成直线，等于没有掉头 */
    WT_CHECK(!WtTraceProfile_Validate(&tp).ok);
    tp = WtTraceProfile_Default();

    tp.stepM = -1.0;
    WT_CHECK(!WtTraceProfile_Validate(&tp).ok);
    tp = WtTraceProfile_Default();

    WT_CHECK(!WtTraceProfile_Validate(NULL).ok);
}

/**
 * 地面余量：叶片指向下方时叶尖接近地面，安全校验必须拒绝。
 *
 * 这不是缺陷而是本模式固有的物理约束 —— 沿叶片全长飞，叶片朝下时
 * 航点必然低。用例把它钉住，免得日后有人把 ground 判据放宽。
 */
static void TestTraceGroundClearanceRejected(void)
{
    WtTurbineSpec spec = WtTest_Spec155();
    WtTraceProfile tp = WtTraceProfile_Default();
    WtInspectionProfile prof = WtInspectionProfile_Default();
    WtMission m;
    WtRotorFrame f;
    WtSafetyReport rep;

    WT_CASE("trace：叶片朝下时地面余量判据生效");
    prof.groundClearanceM = 15.0;

    /* 相位 180°：一片叶片正指下方 */
    WtMission_Init(&m);
    f = WtTurbine_BuildRotorFrame(&spec, 180.0);
    WT_CHECK_EQ_INT((int)WtPlan_BladeTrace(&spec, &f, &tp, &m), (int)WT_PLAN_OK);
    rep = WtPlan_ValidateMission(&spec, &f, &prof, &m);
    WT_CHECK(!rep.ok);
    WT_CHECK(rep.groundViolations > 0);
    WtMission_Free(&m);

    /* 相位 0°：叶片在 12 点/4 点/8 点方向，全部远离地面 */
    WtMission_Init(&m);
    f = WtTurbine_BuildRotorFrame(&spec, 0.0);
    WT_CHECK_EQ_INT((int)WtPlan_BladeTrace(&spec, &f, &tp, &m), (int)WT_PLAN_OK);
    rep = WtPlan_ValidateMission(&spec, &f, &prof, &m);
    WT_CHECK(rep.ok);
    WtMission_Free(&m);
}

/* ------------------------------------------------------------------ */

int main(void)
{
    TestBladeInspectionPredicate();
    TestSegmentSpeedPredicate();
    TestComputeStatsSegmentRule();
    TestComputeStatsAllInspection();
    TestDefaultMission155();
    TestDefaultMission191();
    TestBothSidesMission();
    TestRotatingMode();
    TestDetourPointsAreTransit();
    TestAxialShiftValidation();
    TestTraceTopology();
    TestTraceCoversBothSides();
    TestTraceOffsetRespected();
    TestTraceProfileValidation();
    TestTraceGroundClearanceRejected();

    return WT_TEST_SUMMARY();
}