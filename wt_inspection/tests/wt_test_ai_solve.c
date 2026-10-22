/**
 * @file wt_test_ai_solve.c
 * @brief 端到端解算的回归测试。
 *
 * 核心要验证的是：**反投影能把透视造成的径向差修正掉**。
 *
 * 构造方法：给定盘面上三片等长叶片的三维位置（真实几何），用相机模型
 * **正向投影**得到像素坐标，再喂给解算器。若解算正确，反投影回来的
 * 径向距离应重新一致 —— 这是"透视被抵消"的直接证据。
 */

#include "wt_ai_solve.h"

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

#define IMG_W 1440
#define IMG_H 1080
#define FOV_H 84.0
#define FOV_V 66.0

/* 叶片半径：从轮毂中心到叶尖 */
#define BLADE_R 40.0

/**
 * 正向投影：盘面三维点 → 像素。
 *
 * 用于构造测试输入。实现方式是把反投影的每一步反过来 ——
 * 世界 → 相机系（用 WtProjection_CameraToEnu 的正交性做内积），
 * 再按小孔模型映射到像素。
 */
static bool ProjectToPixel(const WtCameraPose *pose,
                           WtEnu worldPt,
                           int w, int h,
                           double fovH, double fovV,
                           double *outU, double *outV)
{
    double tanH = tan(Wt_DegToRad(fovH) * 0.5);
    double tanV = tan(Wt_DegToRad(fovV) * 0.5);
    WtEnu rel;
    WtEnu dirEnu;
    WtEnu right;
    WtEnu down;
    WtEnu forward;
    const double yaw = Wt_DegToRad(pose->yawDeg);
    const double pitch = Wt_DegToRad(pose->pitchDeg);
    const double roll = Wt_DegToRad(pose->rollDeg);
    const double cp = cos(pitch);
    const double sp = sin(pitch);
    const double cy = cos(yaw);
    const double sy = sin(yaw);
    double xc;
    double yc;
    double zc;

    /* 构造与 WtProjection_CameraToEnu 完全一致的基矢量 */
    forward.e = sy * cp;
    forward.n = cy * cp;
    forward.u = sp;
    right.e = cy;
    right.n = -sy;
    right.u = 0.0;
    down = WtEnu_Cross(forward, right);
    if (fabs(roll) > 1e-12) {
        right = WtEnu_RotateAround(right, forward, roll);
        down = WtEnu_RotateAround(down, forward, roll);
    }

    rel = WtEnu_Sub(worldPt, pose->position);
    dirEnu = WtEnu_Normalize(rel);

    /* 内积得到相机系分量 */
    xc = WtEnu_Dot(dirEnu, right);
    yc = WtEnu_Dot(dirEnu, down);
    zc = WtEnu_Dot(dirEnu, forward);

    if (zc <= 1e-9) {
        return false; /* 点在相机背后 */
    }

    *outU = (xc / zc) / tanH * ((double)w * 0.5) + (double)w * 0.5;
    *outV = (yc / zc) / tanV * ((double)h * 0.5) + (double)h * 0.5;

    return true;
}

/* 构造一个标准的盘面：中心在原点，法向朝南，12 点钟朝上 */
static void MakeFrame(WtRotorFrame *f)
{
    memset(f, 0, sizeof(*f));
    f->rotorCenter.e = 0.0;
    f->rotorCenter.n = 0.0;
    f->rotorCenter.u = 0.0;
    f->rotAxis.e = 0.0;
    f->rotAxis.n = -1.0;
    f->rotAxis.u = 0.0;
    f->upRef.e = 0.0;
    f->upRef.n = 0.0;
    f->upRef.u = 1.0;
    f->rotRef.e = 1.0;
    f->rotRef.n = 0.0;
    f->rotRef.u = 0.0;
}

/* 盘面内 (径向, 相位) → ENU 点 */
static WtEnu BladePoint(const WtRotorFrame *f, double radial, double phaseDeg)
{
    const double a = Wt_DegToRad(phaseDeg);
    WtEnu p = f->rotorCenter;

    p = WtEnu_Add(p, WtEnu_Scale(f->upRef, radial * cos(a)));
    p = WtEnu_Add(p, WtEnu_Scale(f->rotRef, radial * sin(a)));

    return p;
}

/**
 * 完美正对：相机在正南 100 m，三片叶片 0/120/240。
 * 反投影后径向应完全一致，编号应正确。
 */
static void Test_FrontalView(void)
{
    WtAiSolveContext ctx;
    WtAiDetection det;
    WtAiSolveResult res;
    const double phases[3] = {0.0, 120.0, 240.0};
    int i;

    printf("Test_FrontalView\n");

    memset(&ctx, 0, sizeof(ctx));
    MakeFrame(&ctx.rotorFrame);
    ctx.pose.position.e = 0.0;
    ctx.pose.position.n = -100.0;
    ctx.pose.position.u = 0.0;
    ctx.pose.yawDeg = 0.0;
    ctx.pose.pitchDeg = 0.0;
    ctx.pose.rollDeg = 0.0;
    ctx.imageWidthPx = IMG_W;
    ctx.imageHeightPx = IMG_H;
    ctx.fovHorizDeg = FOV_H;
    ctx.fovVertDeg = FOV_V;
    ctx.parkPhaseDeg = 0.0;

    memset(&det, 0, sizeof(det));
    det.hubValid = true;
    CHECK(ProjectToPixel(&ctx.pose, ctx.rotorFrame.rotorCenter, IMG_W, IMG_H,
                         FOV_H, FOV_V, &det.hubU, &det.hubV),
          "轮毂应能投影到像素");

    det.tipCount = 3;
    for (i = 0; i < 3; i++) {
        const WtEnu p = BladePoint(&ctx.rotorFrame, BLADE_R, phases[i]);

        det.tipValid[i] = ProjectToPixel(&ctx.pose, p, IMG_W, IMG_H,
                                         FOV_H, FOV_V, &det.tipU[i], &det.tipV[i]);
        CHECK(det.tipValid[i], "叶尖 %d 应能投影", i);
    }

    CHECK(WtAiSolve_FromDetection(&ctx, &det, &res), "解算应成功");
    CHECK(res.bladeCount == 3, "应解出 3 片（得到 %d）", res.bladeCount);
    CHECK(res.hubHit, "轮毂应命中");

    /* 完美正对 → 径向散布应接近 0 */
    CHECK(res.radialSpreadPct < 0.5,
          "正对时径向散布应 <0.5%%（得到 %.3f%%）", res.radialSpreadPct);

    /* 编号：相位 0 的那片应为 NO.1 */
    for (i = 0; i < 3; i++) {
        const int expected = (i == 0) ? 1 : (i == 1 ? 2 : 3);

        CHECK(res.blades[i].bladeNo == expected,
              "叶尖 %d(相位 %.0f°) 应为 NO.%d（得到 %d）",
              i, phases[i], expected, res.blades[i].bladeNo);
        CHECK_NEAR(res.blades[i].radialM, BLADE_R, 0.5,
                   "叶尖 %d 径向应约 %.0f m", i, BLADE_R);
    }

    CHECK(res.ok, "正对场景应判为可信");
}

/**
 * 斜视：相机偏到侧面 30°，叶片在图像里的投影长度会明显不同。
 *
 * **这是本测试的重点** —— 若直接按图像极角编号会偏，反投影后应重新一致。
 */
static void Test_ObliqueView(void)
{
    WtAiSolveContext ctx;
    WtAiDetection det;
    WtAiSolveResult res;
    const double phases[3] = {0.0, 120.0, 240.0};
    double imgRadials[3];
    int i;

    printf("Test_ObliqueView\n");

    memset(&ctx, 0, sizeof(ctx));
    MakeFrame(&ctx.rotorFrame);
    /* 相机在东南方向：方位角 30°，距离 100 m，高度与轮毂平齐 */
    ctx.pose.position.e = 100.0 * sin(Wt_DegToRad(30.0));
    ctx.pose.position.n = -100.0 * cos(Wt_DegToRad(30.0));
    ctx.pose.position.u = 0.0;
    /* 光轴仍指向盘面中心 */
    ctx.pose.yawDeg = Wt_RadToDeg(atan2(-ctx.pose.position.e, -ctx.pose.position.n));
    ctx.pose.pitchDeg = 0.0;
    ctx.pose.rollDeg = 0.0;
    ctx.imageWidthPx = IMG_W;
    ctx.imageHeightPx = IMG_H;
    ctx.fovHorizDeg = FOV_H;
    ctx.fovVertDeg = FOV_V;
    ctx.parkPhaseDeg = 0.0;

    memset(&det, 0, sizeof(det));
    det.hubValid = true;
    CHECK(ProjectToPixel(&ctx.pose, ctx.rotorFrame.rotorCenter, IMG_W, IMG_H,
                         FOV_H, FOV_V, &det.hubU, &det.hubV),
          "轮毂应能投影");

    det.tipCount = 3;
    for (i = 0; i < 3; i++) {
        const WtEnu p = BladePoint(&ctx.rotorFrame, BLADE_R, phases[i]);

        det.tipValid[i] = ProjectToPixel(&ctx.pose, p, IMG_W, IMG_H,
                                         FOV_H, FOV_V, &det.tipU[i], &det.tipV[i]);
        CHECK(det.tipValid[i], "叶尖 %d 应能投影", i);

        imgRadials[i] = hypot(det.tipU[i] - det.hubU, det.tipV[i] - det.hubV);
    }

    /* 先确认这个视角**确实**产生了明显的图像径向差 —— 否则测试没意义 */
    {
        double imin = imgRadials[0];
        double imax = imgRadials[0];

        for (i = 1; i < 3; i++) {
            if (imgRadials[i] < imin) {
                imin = imgRadials[i];
            }
            if (imgRadials[i] > imax) {
                imax = imgRadials[i];
            }
        }
        printf("  图像径向: %.1f / %.1f / %.1f px（散布 %.1f%%）\n",
               imgRadials[0], imgRadials[1], imgRadials[2],
               (imax - imin) / imax * 100.0);
        CHECK((imax - imin) / imax > 0.05,
              "斜视应产生 >5%% 的图像径向差（得到 %.1f%%），否则本测试无效",
              (imax - imin) / imax * 100.0);
    }

    CHECK(WtAiSolve_FromDetection(&ctx, &det, &res), "解算应成功");
    CHECK(res.bladeCount == 3, "应解出 3 片（得到 %d）", res.bladeCount);

    /*
     * 关键断言：反投影后径向散布应显著小于图像上的散布。
     * 这里是"反投影修正了透视"的直接证据。
     */
    printf("  反投影后径向散布: %.2f%%\n", res.radialSpreadPct);
    CHECK(res.radialSpreadPct < 1.0,
          "斜视反投影后径向散布应 <1%%（得到 %.3f%%）", res.radialSpreadPct);

    for (i = 0; i < 3; i++) {
        CHECK_NEAR(res.blades[i].radialM, BLADE_R, 0.5,
                   "斜视下叶尖 %d 径向应仍约 %.0f m", i, BLADE_R);
    }

    /* 编号仍应正确 */
    CHECK(res.blades[0].bladeNo == 1, "相位 0° 应为 NO.1（得到 %d）",
          res.blades[0].bladeNo);
}

/**
 * 轮毂未检出时，仍应能给出叶片编号（编号只依赖角度）。
 */
static void Test_NoHub(void)
{
    WtAiSolveContext ctx;
    WtAiDetection det;
    WtAiSolveResult res;
    const double phases[3] = {0.0, 120.0, 240.0};
    int i;

    printf("Test_NoHub\n");

    memset(&ctx, 0, sizeof(ctx));
    MakeFrame(&ctx.rotorFrame);
    ctx.pose.position.e = 0.0;
    ctx.pose.position.n = -100.0;
    ctx.pose.position.u = 0.0;
    ctx.pose.yawDeg = 0.0;
    ctx.pose.pitchDeg = 0.0;
    ctx.pose.rollDeg = 0.0;
    ctx.imageWidthPx = IMG_W;
    ctx.imageHeightPx = IMG_H;
    ctx.fovHorizDeg = FOV_H;
    ctx.fovVertDeg = FOV_V;
    ctx.parkPhaseDeg = 0.0;

    memset(&det, 0, sizeof(det));
    det.hubValid = false; /* 故意不给轮毂 */

    det.tipCount = 3;
    for (i = 0; i < 3; i++) {
        const WtEnu p = BladePoint(&ctx.rotorFrame, BLADE_R, phases[i]);

        det.tipValid[i] = ProjectToPixel(&ctx.pose, p, IMG_W, IMG_H,
                                         FOV_H, FOV_V, &det.tipU[i], &det.tipV[i]);
    }

    CHECK(WtAiSolve_FromDetection(&ctx, &det, &res), "解算应成功");
    CHECK(res.bladeCount == 3, "应解出 3 片（得到 %d）", res.bladeCount);
    CHECK(!res.hubHit, "轮毂未给，hubHit 应为 false");
    CHECK(!res.ok, "缺轮毂时应判为不可信");
    CHECK(res.blades[0].bladeNo == 1, "编号仍应正确（得到 %d）",
          res.blades[0].bladeNo);
}

/**
 * 非法输入必须拒绝。
 */
static void Test_RejectInvalid(void)
{
    WtAiSolveContext ctx;
    WtAiDetection det;
    WtAiSolveResult res;

    printf("Test_RejectInvalid\n");

    memset(&ctx, 0, sizeof(ctx));
    memset(&det, 0, sizeof(det));
    MakeFrame(&ctx.rotorFrame);
    ctx.imageWidthPx = IMG_W;
    ctx.imageHeightPx = IMG_H;
    ctx.fovHorizDeg = FOV_H;
    ctx.fovVertDeg = FOV_V;
    det.tipCount = 0;

    CHECK(!WtAiSolve_FromDetection(&ctx, &det, &res), "无叶片时应返回 false");
    CHECK(!WtAiSolve_FromDetection(NULL, &det, &res), "NULL ctx 应被拒绝");
    CHECK(!WtAiSolve_FromDetection(&ctx, NULL, &res), "NULL det 应被拒绝");
}

/* ------------------------------------------------------------------ */

int main(void)
{
    printf("=== 端到端解算测试 ===\n");

    Test_FrontalView();
    Test_ObliqueView();
    Test_NoHub();
    Test_RejectInvalid();

    if (s_failures == 0) {
        printf("全部通过\n");
        return 0;
    }

    printf("%d 项失败\n", s_failures);
    return 1;
}
