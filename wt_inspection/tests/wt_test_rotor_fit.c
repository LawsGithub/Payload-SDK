/**
 * @file wt_test_rotor_fit.c
 * @brief 激光实测点 → 叶轮盘面几何的回归测试。
 *
 * 构造方法与 `wt_test_ai_solve.c` 同一思路：用 `WtTurbine_BladeTip()`
 * **正向**生成三个叶尖的真实三维位置（当作激光测到的点），再喂给拟合器，
 * 看能不能把生成时用的参数反解回来。
 *
 * 几个期望值不是直觉值，先把推导写下来，否则测试里的常数会变成魔法数字：
 *
 *   tip = center + (radial·cos θ − axis·sin θ)·R − axis·prebend     （θ = 锥角）
 *
 *   ⇒ 盘面内径向距离 = R·cos θ      **不是 R** —— 斜距含锥角分量
 *   ⇒ 轴向偏移      = −(R·sin θ + prebend)   锥角与预弯把叶尖拉向上游，故为负
 *   ⇒ 叶尖 i 的相位 = φ + i·(360/叶片数)
 */

#include "wt_rotor_fit.h"

#include "wt_turbine.h"

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

#define CHECK_NEAR(a, b, tol, fmt, ...)                                    \
    do {                                                                   \
        const double _a = (a);                                             \
        const double _b = (b);                                             \
        if (fabs(_a - _b) > (tol)) {                                       \
            printf("  [FAIL] " fmt " (got %.4f, want %.4f)\n",             \
                   ##__VA_ARGS__, _a, _b);                                 \
            s_failures++;                                                  \
        }                                                                  \
    } while (0)

#define PARK_PHASE 37.0

/** 一台典型 3 叶风机 */
static WtTurbineSpec MakeSpec(void)
{
    WtTurbineSpec spec;

    memset(&spec, 0, sizeof(spec));
    spec.base.lat = 40.0;
    spec.base.lon = 116.0;
    spec.base.alt = 50.0;
    spec.hubHeight = 100.0;
    spec.rotorDiameter = 80.0;   /* R = 40 m */
    spec.hubRadius = 2.0;
    spec.towerBottomDia = 5.0;
    spec.towerTopDia = 3.0;
    spec.nacelleOffset = 4.5;
    spec.coneAngleDeg = 3.0;
    spec.tiltDeg = 5.0;
    spec.headingDeg = 210.0;
    spec.prebendM = 2.0;
    spec.bladeCount = 3;

    return spec;
}

/** 按台账正向生成「激光会测到的」轮毂与叶尖位置 */
static void MakeTruth(const WtTurbineSpec *spec, double phaseDeg,
                      WtEnu *outHub, WtEnu *outTips, int tipCount)
{
    WtRotorFrame frame = WtTurbine_BuildRotorFrame(spec, phaseDeg);
    int i;

    *outHub = frame.rotorCenter;
    for (i = 0; i < tipCount; i++) {
        outTips[i] = WtTurbine_BladeTip(&frame, spec, i);
    }
}

/* 由台账推出的期望值，集中在一处，避免散落的魔法数字 */
static double ExpectedRadial(const WtTurbineSpec *spec)
{
    return spec->rotorDiameter * 0.5 * cos(Wt_DegToRad(spec->coneAngleDeg));
}

static double ExpectedAxialOffset(const WtTurbineSpec *spec)
{
    return -(spec->rotorDiameter * 0.5 * sin(Wt_DegToRad(spec->coneAngleDeg)) +
             spec->prebendM);
}

/**
 * 三叶正常采集：所有量都应回到生成时的真值。
 */
static void Test_ThreeTips_RecoversTruth(void)
{
    WtTurbineSpec spec = MakeSpec();
    WtRotorFrame truth = WtTurbine_BuildRotorFrame(&spec, PARK_PHASE);
    WtEnu hub;
    WtEnu tips[3];
    WtRotorFitResult fit;
    const char *reason = NULL;

    printf("Test_ThreeTips_RecoversTruth\n");

    MakeTruth(&spec, PARK_PHASE, &hub, tips, 3);

    CHECK(WtRotorFit_FromPoints(&spec, hub, tips, 3, &fit), "拟合应成功");
    CHECK(fit.ok, "ok 应为 true（message=%s）", fit.message);

    /* 法向应与真值重合 —— 点积接近 1 */
    CHECK_NEAR(WtEnu_Dot(fit.rotAxis, truth.rotAxis), 1.0, 1e-6,
               "法向应与真值同向");
    CHECK_NEAR(fit.axisAngleErrDeg, 0.0, 0.05,
               "与台账法向夹角应接近 0（台账相位不影响法向）");

    /* 中心直接取激光点，应与真值完全一致 */
    CHECK_NEAR(WtEnu_Distance(fit.rotorCenter, truth.rotorCenter), 0.0, 1e-9,
               "中心应等于轮毂实测点");

    /* 半径是 R·cos(锥角)，不是 R */
    CHECK_NEAR(fit.rotorRadiusM, ExpectedRadial(&spec), 1e-6,
               "径向距离应为 R·cos(锥角)");
    CHECK_NEAR(fit.radiusSpreadPct, 0.0, 1e-6, "三片等长，散布应为 0");

    /* 轴向偏移为负：锥角与预弯把叶尖拉向上游 */
    CHECK_NEAR(fit.axialOffsetM, ExpectedAxialOffset(&spec), 1e-6,
               "轴向偏移应为 -(R·sin(锥角) + 预弯)");
    CHECK(fit.axialOffsetM < 0.0, "轴向偏移应为负值");

    /* 停用角 = NO.1 叶尖相位 = 生成时用的相位 */
    CHECK_NEAR(fit.parkPhaseDeg, PARK_PHASE, 1e-6, "停用角应回到生成值");
    CHECK_NEAR(fit.tipPhaseDeg[1], Wt_Wrap360(PARK_PHASE + 120.0), 1e-6,
               "NO.2 相位应为 +120°");
    CHECK_NEAR(fit.tipPhaseDeg[2], Wt_Wrap360(PARK_PHASE + 240.0), 1e-6,
               "NO.3 相位应为 +240°");
    CHECK_NEAR(fit.gapDevDeg, 0.0, 1e-6, "等间隔，间隔偏差应为 0");

    {
        WtRotorFitLimits limits = WtRotorFit_DefaultLimits();

        CHECK(WtRotorFit_CheckLimits(&fit, &limits, &reason),
              "应通过采信阈值（reason=%s）", reason);
    }

    /* 转出的 frame 要能直接喂给规划链 */
    {
        WtRotorFrame out = WtRotorFit_ToRotorFrame(&fit);

        CHECK_NEAR(out.phaseDeg, PARK_PHASE, 1e-6, "frame 相位应为停用角");
        CHECK_NEAR(WtEnu_Dot(out.rotAxis, truth.rotAxis), 1.0, 1e-6,
                   "frame 法向应与真值同向");
        CHECK_NEAR(out.rpm, 0.0, 1e-12, "实测前提是停机，rpm 应为 0");
    }
}

/**
 * 采集顺序颠倒：叉积会指向上游，定向逻辑必须把它翻回来。
 *
 * 这条用例盯的是一个"符号反了但看起来对"的错误 —— 法向反向后，
 * 盘面本身没变，radius/spread 全部照常，只有 rotAxis 的朝向错了，
 * 而那会让后续航线规划把飞机放到风机的另一侧。
 */
static void Test_ReversedOrder_FixesNormalDirection(void)
{
    WtTurbineSpec spec = MakeSpec();
    WtRotorFrame truth = WtTurbine_BuildRotorFrame(&spec, PARK_PHASE);
    WtEnu hub;
    WtEnu tips[3];
    WtEnu reversed[3];
    WtRotorFitResult fit;

    printf("Test_ReversedOrder_FixesNormalDirection\n");

    MakeTruth(&spec, PARK_PHASE, &hub, tips, 3);

    /* 按 NO.3 → NO.2 → NO.1 的顺序喂进去 */
    reversed[0] = tips[2];
    reversed[1] = tips[1];
    reversed[2] = tips[0];

    CHECK(WtRotorFit_FromPoints(&spec, hub, reversed, 3, &fit), "拟合应成功");
    CHECK_NEAR(WtEnu_Dot(fit.rotAxis, truth.rotAxis), 1.0, 1e-6,
               "顺序颠倒后法向仍应指向下游");

    /* 但停用角会变成被当成 NO.1 的那片的相位 —— 这是采集顺序的语义 */
    CHECK_NEAR(fit.parkPhaseDeg, Wt_Wrap360(PARK_PHASE + 240.0), 1e-6,
               "停用角取第一个采集点的相位");
}

/**
 * 漏测一片：剩下两片间隔 240°，间隔偏差必须仍为 0。
 *
 * 这条直接盯 wt_blade_id.c 里那个坑的反面 —— 那边用 `360/实际检出数`
 * 当理想间隔，两片时 ideal=180°，拿 240° 去比会得出 60° 偏差，
 * 把一次正常的降级采集判成质量极差。本模块按「理想间隔的整数倍」比。
 */
static void Test_MissingBlade_GapStillZero(void)
{
    WtTurbineSpec spec = MakeSpec();
    WtEnu hub;
    WtEnu tips[3];
    WtEnu twoTips[2];
    WtRotorFitResult fit;

    printf("Test_MissingBlade_GapStillZero\n");

    MakeTruth(&spec, PARK_PHASE, &hub, tips, 3);

    /* 只采到 NO.1 与 NO.3 —— 相位差 240° */
    twoTips[0] = tips[0];
    twoTips[1] = tips[2];

    CHECK(WtRotorFit_FromPoints(&spec, hub, twoTips, 2, &fit), "两叶应能解出");
    CHECK(fit.ok, "ok 应为 true");

    /*
     * 判据的要点：**远小于 60°**。
     *
     * 若按 `360/实际检出数` 算理想间隔（= 180°），240° 会得出整整 60° 的偏差。
     * 这里只剩约 1°，且那 1° 来自下面要验的法向偏斜，不是间隔算错。
     */
    CHECK(fit.gapDevDeg < 5.0,
          "240° 应被识别为 2×120°（若用 360/2 当理想值会得到约 60°，"
          "实得 %.2f°）", fit.gapDevDeg);

    CHECK(strstr(fit.message, "cone-angle bias") != NULL,
          "两叶结果应标注偏差来源（message=%s）", fit.message);

    /*
     * 两叶法向的偏角有闭式解，这里按公式算期望值而不是写死一个观测到的数。
     *
     * 法向取 (tip0−hub)×(tip1−hub)，而 hub 不在叶尖平面上（偏移
     * c = R·sinθ + prebend）。把 tip−hub 分解成盘面内分量 r·u 与轴向分量 −c·a：
     *
     *   (r u0 − c a) × (r u1 − c a) = r²·sinΔφ·a  +  r·c·[a × (u0 − u1)]
     *                                 └── 沿 a ──┘   └──── 垂直于 a ────┘
     *
     *   ⇒ 偏角 = atan( c·|u0−u1| / (r·|sinΔφ|) ),  |u0−u1| = 2·|sin(Δφ/2)|
     *
     * 对 Δφ=240°、本台账参数，结果约 11.6° —— **不是锥角的 3°**。
     * 这个反直觉的放大倍数正是两叶降级路径的代价，值得测试把它钉住。
     */
    {
        const double R = spec.rotorDiameter * 0.5;
        const double theta = Wt_DegToRad(spec.coneAngleDeg);
        const double c = R * sin(theta) + spec.prebendM;
        const double r = R * cos(theta);
        const double dPhi = Wt_DegToRad(240.0);
        const double chord = 2.0 * fabs(sin(dPhi * 0.5));
        const double expectDeg = Wt_RadToDeg(atan(c * chord / (r * fabs(sin(dPhi)))));

        CHECK_NEAR(fit.axisAngleErrDeg, expectDeg, 0.05,
                   "两叶法向偏角应符合闭式解");
        CHECK(expectDeg > 3.0 * 2.0,
              "该偏角应显著大于锥角本身（锥角 %.1f°，偏角 %.1f°）",
              spec.coneAngleDeg, expectDeg);
    }
}

/**
 * 某片叶尖打偏：径向散布必须把它抓出来。
 */
static void Test_MisAimedTip_Rejected(void)
{
    WtTurbineSpec spec = MakeSpec();
    WtEnu hub;
    WtEnu tips[3];
    WtRotorFitResult fit;
    WtRotorFitLimits limits = WtRotorFit_DefaultLimits();
    const char *reason = NULL;

    printf("Test_MisAimedTip_Rejected\n");

    MakeTruth(&spec, PARK_PHASE, &hub, tips, 3);

    /* 把 NO.2 的测点沿径向拉回 30%（激光打到了叶片中段而不是叶尖） */
    tips[1] = WtEnu_Add(hub, WtEnu_Scale(WtEnu_Sub(tips[1], hub), 0.7));

    CHECK(WtRotorFit_FromPoints(&spec, hub, tips, 3, &fit),
          "几何仍应解出（输入不退化）");
    CHECK(fit.radiusSpreadPct > 20.0, "散布应显著（得到 %.1f%%）",
          fit.radiusSpreadPct);
    CHECK(!WtRotorFit_CheckLimits(&fit, &limits, &reason),
          "应被采信阈值拒绝");
    CHECK(reason != NULL && strstr(reason, "radius spread") != NULL,
          "拒绝原因应指向径向散布（reason=%s）", reason ? reason : "(null)");
}

/**
 * 退化输入必须拒绝，而不是返回一个看起来合理的结果。
 */
static void Test_DegenerateInput_Rejected(void)
{
    WtTurbineSpec spec = MakeSpec();
    WtEnu hub;
    WtEnu tips[3];
    WtRotorFitResult fit;

    printf("Test_DegenerateInput_Rejected\n");

    MakeTruth(&spec, PARK_PHASE, &hub, tips, 3);

    CHECK(!WtRotorFit_FromPoints(&spec, hub, tips, 1, &fit),
          "单个叶尖时平面不唯一，应拒绝");
    CHECK(!WtRotorFit_FromPoints(&spec, hub, tips, 0, &fit), "0 个叶尖应拒绝");
    CHECK(!WtRotorFit_FromPoints(NULL, hub, tips, 3, &fit), "NULL spec 应拒绝");
    CHECK(!WtRotorFit_FromPoints(&spec, hub, NULL, 3, &fit), "NULL tips 应拒绝");
    CHECK(!WtRotorFit_FromPoints(&spec, hub, tips, 3, NULL), "NULL out 应拒绝");

    /* 三点重合 */
    {
        WtEnu same[3];

        same[0] = tips[0];
        same[1] = tips[0];
        same[2] = tips[0];
        CHECK(!WtRotorFit_FromPoints(&spec, hub, same, 3, &fit),
              "三点重合应拒绝");
    }

    /* 三点共线：沿同一径向取三个不同半径的点 */
    {
        WtEnu collinear[3];
        WtEnu dir = WtEnu_Sub(tips[0], hub);

        collinear[0] = WtEnu_Add(hub, WtEnu_Scale(dir, 0.5));
        collinear[1] = WtEnu_Add(hub, WtEnu_Scale(dir, 0.75));
        collinear[2] = WtEnu_Add(hub, WtEnu_Scale(dir, 1.0));
        CHECK(!WtRotorFit_FromPoints(&spec, hub, collinear, 3, &fit),
              "三点共线应拒绝");
    }
}

/**
 * 整体高度基准偏移不应影响拟合 —— 这是"只用相对量"的直接验证。
 *
 * 激光返回的海拔基准没有文档承诺（姊妹项目实测与气压高差约 38 m）。
 * 把全部 4 个点同时抬高 38 m，法向/半径/相位/间隔都必须一字不变，
 * 只有与台账比对的垂直偏差会变。
 */
static void Test_AltitudeDatumShift_Immune(void)
{
    WtTurbineSpec spec = MakeSpec();
    WtEnu hub;
    WtEnu tips[3];
    WtRotorFitResult base;
    WtRotorFitResult shifted;
    const WtEnu lift = {0.0, 0.0, 38.0};
    WtEnu shiftedHub;
    WtEnu shiftedTips[3];
    int i;

    printf("Test_AltitudeDatumShift_Immune\n");

    MakeTruth(&spec, PARK_PHASE, &hub, tips, 3);

    CHECK(WtRotorFit_FromPoints(&spec, hub, tips, 3, &base), "基准拟合应成功");

    shiftedHub = WtEnu_Add(hub, lift);
    for (i = 0; i < 3; i++) {
        shiftedTips[i] = WtEnu_Add(tips[i], lift);
    }

    CHECK(WtRotorFit_FromPoints(&spec, shiftedHub, shiftedTips, 3, &shifted),
          "整体抬高后仍应成功");

    CHECK_NEAR(shifted.rotorRadiusM, base.rotorRadiusM, 1e-9,
               "半径不应受整体平移影响");
    CHECK_NEAR(shifted.parkPhaseDeg, base.parkPhaseDeg, 1e-9,
               "停用角不应受整体平移影响");
    CHECK_NEAR(shifted.gapDevDeg, base.gapDevDeg, 1e-9,
               "间隔偏差不应受整体平移影响");
    CHECK_NEAR(WtEnu_Dot(shifted.rotAxis, base.rotAxis), 1.0, 1e-12,
               "法向不应受整体平移影响");
    CHECK_NEAR(shifted.centerOffsetHorizM, base.centerOffsetHorizM, 1e-9,
               "水平偏差不应受垂直平移影响");

    /* 唯一该变的就是垂直偏差，且正好是平移量 */
    CHECK_NEAR(shifted.centerOffsetVerticalM - base.centerOffsetVerticalM,
               38.0, 1e-9, "垂直偏差应正好反映基准平移");
}

int main(void)
{
    printf("=== 叶轮盘面实测拟合测试 ===\n");

    Test_ThreeTips_RecoversTruth();
    Test_ReversedOrder_FixesNormalDirection();
    Test_MissingBlade_GapStillZero();
    Test_MisAimedTip_Rejected();
    Test_DegenerateInput_Rejected();
    Test_AltitudeDatumShift_Immune();

    if (s_failures == 0) {
        printf("全部通过\n");
        return 0;
    }
    printf("%d 项失败\n", s_failures);
    return 1;
}
