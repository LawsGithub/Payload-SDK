/**
 * @file wt_test_aim.c
 * @brief 像素 → 云台绝对指向的回归测试。
 *
 * 两条核心性质：
 *
 *   1. **画面内偏角在旋转下不变。** 目标相对光轴的夹角只由它在画面里的位置
 *      决定，云台怎么转都不改变这个夹角（旋转是保角变换）。
 *
 *   2. **但两轴误差会被俯仰角放大。** 同一个像素，云台平视时方位误差 35.8°，
 *      俯视 60° 时变成 55.3° —— 这是球面效应，不是 bug。
 *
 * 第 2 条正是「不能用 sqrt(yawErr² + pitchErr²) 当对准判据」的理由，
 * 下面有一条用例把这个高估量直接算出来钉住。
 */

#include "wt_aim.h"

#include "wt_camera.h"

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

static WtCameraPose MakePose(double yawDeg, double pitchDeg)
{
    WtCameraPose p;

    memset(&p, 0, sizeof(p));
    p.yawDeg = yawDeg;
    p.pitchDeg = pitchDeg;
    p.rollDeg = 0.0;

    return p;
}

/**
 * FOV 必须取自相机模型，不能手填。
 *
 * 这条用例顺带钉住一件事：DJI 规格页标的 84° 是 M4T 广角的**对角线** FOV，
 * 水平值是 71.59°。现有 `tests/wt_test_ai_solve.c` 与 `wt_test_projection.c`
 * 把 84 当水平 FOV 用（硬编码 `FOV_H 84.0`），本模块不沿用。
 */
static void Test_FovComesFromCameraModel(void)
{
    WtCameraGeometry g = WtCamera_ComputeGeometry(&WT_CAMERA_M4T_WIDE);

    printf("Test_FovComesFromCameraModel\n");

    CHECK_NEAR(g.fovDiagDeg, 84.06, 0.05, "对角线 FOV");
    CHECK_NEAR(g.fovHorizDeg, 71.59, 0.05, "水平 FOV");
    CHECK(g.fovHorizDeg < g.fovDiagDeg - 10.0,
          "水平 FOV 应显著小于对角线 FOV（把 84 当水平用会放大视场）");
}

/** 画面中心的像素：目标角就是当前云台角，误差为 0。 */
static void Test_CenterPixel_ZeroError(void)
{
    WtCameraGeometry g = WtCamera_ComputeGeometry(&WT_CAMERA_M4T_WIDE);
    WtCameraPose pose = MakePose(137.0, -25.0);
    WtAimResult aim;

    printf("Test_CenterPixel_ZeroError\n");

    CHECK(WtAim_FromPixel(&pose, IMG_W * 0.5, IMG_H * 0.5, IMG_W, IMG_H,
                          g.fovHorizDeg, g.fovVertDeg, &aim),
          "解算应成功");
    CHECK_NEAR(aim.targetYawDeg, 137.0, 1e-6, "目标方位应等于云台方位");
    CHECK_NEAR(aim.targetPitchDeg, -25.0, 1e-6, "目标俯仰应等于云台俯仰");
    CHECK_NEAR(aim.yawErrorDeg, 0.0, 1e-6, "方位误差应为 0");
    CHECK_NEAR(aim.pitchErrorDeg, 0.0, 1e-6, "俯仰误差应为 0");
    CHECK_NEAR(aim.angularErrorDeg, 0.0, 1e-6, "总夹角应为 0");
    CHECK(WtAim_IsOnTarget(&aim, 2.0), "应判为已对准");
}

/** 右边缘中点：平视时方位偏角应正好是水平半视场。 */
static void Test_RightEdge_HalfFov(void)
{
    WtCameraGeometry g = WtCamera_ComputeGeometry(&WT_CAMERA_M4T_WIDE);
    WtCameraPose pose = MakePose(0.0, 0.0);
    WtAimResult aim;

    printf("Test_RightEdge_HalfFov\n");

    CHECK(WtAim_FromPixel(&pose, (double)IMG_W, IMG_H * 0.5, IMG_W, IMG_H,
                          g.fovHorizDeg, g.fovVertDeg, &aim),
          "解算应成功");
    CHECK_NEAR(aim.yawErrorDeg, g.fovHorizDeg * 0.5, 0.01,
               "右边缘的方位偏角应为水平半视场");
    CHECK_NEAR(aim.pitchErrorDeg, 0.0, 1e-6, "中间高度的俯仰偏角应为 0");
    CHECK(!WtAim_IsOnTarget(&aim, 2.0), "偏 35° 不应判为已对准");
}

/** 下边缘中点：俯仰偏角应为负的垂直半视场（+v 向下）。 */
static void Test_BottomEdge_NegativePitch(void)
{
    WtCameraGeometry g = WtCamera_ComputeGeometry(&WT_CAMERA_M4T_WIDE);
    WtCameraPose pose = MakePose(0.0, 0.0);
    WtAimResult aim;

    printf("Test_BottomEdge_NegativePitch\n");

    CHECK(WtAim_FromPixel(&pose, IMG_W * 0.5, (double)IMG_H, IMG_W, IMG_H,
                          g.fovHorizDeg, g.fovVertDeg, &aim),
          "解算应成功");
    CHECK(aim.pitchErrorDeg < 0.0, "画面下方的目标应需要云台**俯**下去");
    CHECK_NEAR(aim.pitchErrorDeg, -g.fovVertDeg * 0.5, 0.01,
               "下边缘的俯仰偏角应为负的垂直半视场");
}

/**
 * 核心用例：画面内夹角在旋转下不变，而两轴误差被俯仰角放大。
 *
 * 这条同时证明两件事：点积算夹角是对的；用欧氏距离当判据是错的。
 */
static void Test_AngularError_RotationInvariant(void)
{
    WtCameraGeometry g = WtCamera_ComputeGeometry(&WT_CAMERA_M4T_WIDE);
    const double u = (double)IMG_W;      /* 右边缘 */
    const double v = IMG_H * 0.5;
    WtAimResult level;
    WtAimResult steep;
    double euclid;

    printf("Test_AngularError_RotationInvariant\n");

    {
        WtCameraPose pose = MakePose(0.0, 0.0);

        CHECK(WtAim_FromPixel(&pose, u, v, IMG_W, IMG_H,
                              g.fovHorizDeg, g.fovVertDeg, &level),
              "平视解算应成功");
    }
    {
        WtCameraPose pose = MakePose(0.0, -60.0);

        CHECK(WtAim_FromPixel(&pose, u, v, IMG_W, IMG_H,
                              g.fovHorizDeg, g.fovVertDeg, &steep),
              "俯视解算应成功");
    }

    /* ① 夹角不变 —— 旋转是保角变换 */
    CHECK_NEAR(steep.angularErrorDeg, level.angularErrorDeg, 1e-6,
               "总夹角不应随云台俯仰改变");
    CHECK_NEAR(level.angularErrorDeg, g.fovHorizDeg * 0.5, 0.01,
               "夹角应等于水平半视场");

    /* ② 但方位误差被放大了 —— 球面效应 */
    CHECK(steep.yawErrorDeg > level.yawErrorDeg * 1.3,
          "俯视 60° 时方位误差应被显著放大（平视 %.2f° → 俯视 %.2f°）",
          level.yawErrorDeg, steep.yawErrorDeg);

    /*
     * ③ 于是欧氏距离会严重高估。
     *
     * 俯视 60° 下：yawErr≈55.3°、pitchErr≈15.4°，欧氏距离 ≈57.4°，
     * 而真实夹角只有 35.8° —— 高估约 60%。若把容差设在 40°，
     * 目标其实已在画面中心线上，却会被判为未对准、重试到超次数。
     */
    euclid = sqrt(steep.yawErrorDeg * steep.yawErrorDeg +
                  steep.pitchErrorDeg * steep.pitchErrorDeg);
    CHECK(euclid > steep.angularErrorDeg * 1.4,
          "欧氏距离应明显高估真实夹角（欧氏 %.2f° vs 真实 %.2f°）",
          euclid, steep.angularErrorDeg);
}

/**
 * 闭环一致性：把解出的目标角当成新的云台指向，目标应落回画面中心。
 *
 * 这是对整条变换链最强的约束 —— 任何一处符号或轴序写反，往返都闭不上。
 */
static void Test_RoundTrip_LandsOnCenter(void)
{
    WtCameraGeometry g = WtCamera_ComputeGeometry(&WT_CAMERA_M4T_WIDE);
    WtCameraPose pose = MakePose(73.0, -18.0);
    WtAimResult first;
    WtAimResult second;

    printf("Test_RoundTrip_LandsOnCenter\n");

    /* 取一个偏离中心的像素（画面右下区域） */
    CHECK(WtAim_FromPixel(&pose, IMG_W * 0.8, IMG_H * 0.7, IMG_W, IMG_H,
                          g.fovHorizDeg, g.fovVertDeg, &first),
          "首次解算应成功");

    /* 云台转到解出的目标角 */
    {
        WtCameraPose aimed = MakePose(first.targetYawDeg, first.targetPitchDeg);

        /* 此时目标应该在画面中心，所以中心像素解出的角就是刚才那个角 */
        CHECK(WtAim_FromPixel(&aimed, IMG_W * 0.5, IMG_H * 0.5, IMG_W, IMG_H,
                              g.fovHorizDeg, g.fovVertDeg, &second),
              "二次解算应成功");
        CHECK_NEAR(second.targetYawDeg, first.targetYawDeg, 1e-6,
                   "转到位后中心像素应指向同一方位");
        CHECK_NEAR(second.targetPitchDeg, first.targetPitchDeg, 1e-6,
                   "转到位后中心像素应指向同一俯仰");
        CHECK(WtAim_IsOnTarget(&second, 0.5), "转到位后应判为已对准");
    }
}

/** 非法输入必须拒绝。 */
static void Test_RejectInvalid(void)
{
    WtCameraPose pose = MakePose(0.0, 0.0);
    WtAimResult aim;

    printf("Test_RejectInvalid\n");

    CHECK(!WtAim_FromPixel(NULL, 0.0, 0.0, IMG_W, IMG_H, 71.6, 56.8, &aim),
          "NULL pose 应拒绝");
    CHECK(!WtAim_FromPixel(&pose, 0.0, 0.0, IMG_W, IMG_H, 71.6, 56.8, NULL),
          "NULL out 应拒绝");
    CHECK(!WtAim_FromPixel(&pose, 0.0, 0.0, 0, IMG_H, 71.6, 56.8, &aim),
          "宽度为 0 应拒绝");
    CHECK(!WtAim_FromPixel(&pose, 0.0, 0.0, IMG_W, IMG_H, 0.0, 56.8, &aim),
          "FOV 为 0 应拒绝");
    CHECK(!WtAim_FromPixel(&pose, 0.0, 0.0, IMG_W, IMG_H, 180.0, 56.8, &aim),
          "FOV 达 180° 应拒绝");

    memset(&aim, 0, sizeof(aim));
    CHECK(!WtAim_IsOnTarget(&aim, 2.0), "ok=false 的结果不应判为已对准");
    CHECK(!WtAim_IsOnTarget(NULL, 2.0), "NULL 不应判为已对准");
}

/**
 * 叶尖排序必须**稳定**：同一组点无论模型以什么顺序输出，排完都一样。
 *
 * 这是整条采集链的编号基石。模型用排列不变损失训练，三个叶尖的输出顺序
 * 逐帧可变；若排序不稳定，操作员采的"NO.1"和程序记的"NO.1"会对不上，
 * 而全程没有任何报错，最终 parkPhase 偏 120°。
 */
static void Test_TipSort_IsStable(void)
{
    /* 轮毂在画面中心，三个叶尖分别在上、右下、左下（相差 120°） */
    const double hubU = 720.0;
    const double hubV = 540.0;
    const double r = 300.0;
    WtAiDetection a;
    WtAiDetection b;
    double anglesA[WT_BLADE_ID_MAX];
    double anglesB[WT_BLADE_ID_MAX];
    int i;

    printf("Test_TipSort_IsStable\n");

    memset(&a, 0, sizeof(a));
    a.hubValid = true;
    a.hubU = hubU;
    a.hubV = hubV;
    a.tipCount = 3;

    /* 顺序 ①：上(0°) → 右下(120°) → 左下(240°) */
    a.tipU[0] = hubU;                      a.tipV[0] = hubV - r;
    a.tipU[1] = hubU + r * sin(M_PI * 2.0 / 3.0);
    a.tipV[1] = hubV - r * cos(M_PI * 2.0 / 3.0);
    a.tipU[2] = hubU + r * sin(M_PI * 4.0 / 3.0);
    a.tipV[2] = hubV - r * cos(M_PI * 4.0 / 3.0);
    for (i = 0; i < 3; i++) {
        a.tipValid[i] = true;
    }

    /* 顺序 ②：同样三个点，但打乱成 左下 → 上 → 右下 */
    b = a;
    b.tipU[0] = a.tipU[2]; b.tipV[0] = a.tipV[2];
    b.tipU[1] = a.tipU[0]; b.tipV[1] = a.tipV[0];
    b.tipU[2] = a.tipU[1]; b.tipV[2] = a.tipV[1];

    CHECK(WtAim_SortTipsByImageAngle(&a, anglesA), "排序 A 应成功");
    CHECK(WtAim_SortTipsByImageAngle(&b, anglesB), "排序 B 应成功");

    for (i = 0; i < 3; i++) {
        CHECK_NEAR(a.tipU[i], b.tipU[i], 1e-9,
                   "打乱输入顺序后第 %d 片的 u 应一致", i);
        CHECK_NEAR(a.tipV[i], b.tipV[i], 1e-9,
                   "打乱输入顺序后第 %d 片的 v 应一致", i);
        CHECK_NEAR(anglesA[i], anglesB[i], 1e-9,
                   "第 %d 片的图像极角应一致", i);
    }

    /* 排序后应为升序，且第一片在图像正上方 */
    CHECK_NEAR(anglesA[0], 0.0, 1e-6, "第一片应在图像正上方（0°）");
    CHECK_NEAR(anglesA[1], 120.0, 1e-6, "第二片应在 120°");
    CHECK_NEAR(anglesA[2], 240.0, 1e-6, "第三片应在 240°");
    CHECK_NEAR(a.tipV[0], hubV - r, 1e-9, "第一片应是最上方那个点");

    CHECK(!WtAim_SortTipsByImageAngle(NULL, NULL), "NULL 应拒绝");
    {
        WtAiDetection one = a;

        one.tipCount = 1;
        CHECK(!WtAim_SortTipsByImageAngle(&one, NULL),
              "只有 1 片时无需排序，应返回 false");
    }
}

/**
 * 云台可达性：pitch 比绝对限位，pan 比**相对机头**的 ±60°。
 *
 * 为什么必须先判再下发：绝对角模式下只要 yaw 够不到，**整条命令被拒**
 * （0x600000004），连 pitch 都不会转到位。不先判就会把"参数超限"
 * 表现成"云台莫名不动"。
 */
static void Test_Reachable(void)
{
    WtCameraGeometry g = WtCamera_ComputeGeometry(&WT_CAMERA_M4T_WIDE);
    WtAimResult aim;
    const char *why = NULL;

    printf("Test_Reachable\n");

    /* 机头朝北、目标就在正前方中心 → 可达 */
    {
        WtCameraPose pose = MakePose(0.0, -20.0);

        CHECK(WtAim_FromPixel(&pose, IMG_W * 0.5, IMG_H * 0.5, IMG_W, IMG_H,
                              g.fovHorizDeg, g.fovVertDeg, &aim), "解算应成功");
        CHECK(WtAim_IsReachable(&aim, 0.0, &why), "正前方应可达（%s）", why);
    }

    /* pitch 超上限 +70° */
    {
        WtCameraPose pose = MakePose(0.0, 85.0);

        (void)WtAim_FromPixel(&pose, IMG_W * 0.5, IMG_H * 0.5, IMG_W, IMG_H,
                              g.fovHorizDeg, g.fovVertDeg, &aim);
        CHECK(!WtAim_IsReachable(&aim, 0.0, &why), "pitch 85° 应超出 +70° 上限");
        CHECK(why != NULL && strstr(why, "上限") != NULL,
              "原因应指向俯仰上限（%s）", why ? why : "(null)");
    }

    /* yaw 相对机头 80° > 60° → 不可达，需要转机头 */
    {
        WtCameraPose pose = MakePose(80.0, 0.0);

        (void)WtAim_FromPixel(&pose, IMG_W * 0.5, IMG_H * 0.5, IMG_W, IMG_H,
                              g.fovHorizDeg, g.fovVertDeg, &aim);
        CHECK(!WtAim_IsReachable(&aim, 0.0, &why),
              "目标方位 80° / 机头 0° 相差 80°，应超出 ±60°");
        CHECK(why != NULL && strstr(why, "机头") != NULL,
              "原因应提示转机头（%s）", why ? why : "(null)");
    }

    /*
     * 跨正北不能误判。
     *
     * 目标方位 10°、机头 350° —— 相对角只有 20°，完全够得到。
     * 但若直接相减会得到 10 − 350 = −340°，被判成严重超限。
     * 这正是必须用 AngleDiff 折到 [-180,180] 的理由。
     */
    {
        WtCameraPose pose = MakePose(10.0, 0.0);

        (void)WtAim_FromPixel(&pose, IMG_W * 0.5, IMG_H * 0.5, IMG_W, IMG_H,
                              g.fovHorizDeg, g.fovVertDeg, &aim);
        CHECK(WtAim_IsReachable(&aim, 350.0, &why),
              "目标 10° / 机头 350° 相对角仅 20°，应可达（%s）", why);
    }

    CHECK(!WtAim_IsReachable(NULL, 0.0, &why), "NULL 应判为不可达");
}

int main(void)
{
    printf("=== 照准解算测试 ===\n");

    Test_FovComesFromCameraModel();
    Test_CenterPixel_ZeroError();
    Test_RightEdge_HalfFov();
    Test_BottomEdge_NegativePitch();
    Test_AngularError_RotationInvariant();
    Test_RoundTrip_LandsOnCenter();
    Test_Reachable();
    Test_TipSort_IsStable();
    Test_RejectInvalid();

    if (s_failures == 0) {
        printf("全部通过\n");
        return 0;
    }
    printf("%d 项失败\n", s_failures);
    return 1;
}
