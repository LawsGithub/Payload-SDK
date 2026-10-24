/**
 * @file wt_test_projection.c
 * @brief 反投影的回归测试。
 *
 * 这类几何代码的典型失败是「差一个符号」或「轴搞反了」—— 编译通过、
 * 运行不崩，但算出来的位置在错误的一侧。所以用例要构造**已知答案**的
 * 场景，逐条对答案。
 */

#include "wt_projection.h"

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

/*
 * FOV 由相机模型算出，**不写字面量**。原先是 `84.0 / 66.0` ——
 * 84° 是 DJI 标注的**对角线** FOV，水平值是 71.59°。详见
 * `wt_test_ai_solve.c` 里同一处的说明。
 */
static WtCameraGeometry TestCamGeom(void)
{
    return WtCamera_ComputeGeometry(&WT_CAMERA_M4T_WIDE);
}

#define FOV_H (TestCamGeom().fovHorizDeg)
#define FOV_V (TestCamGeom().fovVertDeg)

/* ------------------------------------------------------------------ */

/**
 * 图像正中心 → 射线应沿光轴（相机系 (0,0,1)）。
 */
static void Test_CenterRay(void)
{
    double tanH = 0.0;
    double tanV = 0.0;
    WtEnu dir;

    printf("Test_CenterRay\n");

    CHECK(WtProjection_SetupIntrinsics(IMG_W, IMG_H, FOV_H, FOV_V, &tanH, &tanV),
          "内参设置应成功");
    CHECK(WtProjection_PixelToCameraRay(IMG_W / 2.0, IMG_H / 2.0,
                                        IMG_W, IMG_H, tanH, tanV, &dir),
          "中心像素应有射线");

    CHECK_NEAR(dir.e, 0.0, 1e-9, "中心射线 x 分量应为 0");
    CHECK_NEAR(dir.n, 0.0, 1e-9, "中心射线 y 分量应为 0");
    CHECK_NEAR(dir.u, 1.0, 1e-9, "中心射线 z 分量应为 1");
}

/**
 * 图像右边缘 → 射线向右偏，偏角约等于水平半视场角。
 */
static void Test_RightEdgeRay(void)
{
    double tanH = 0.0;
    double tanV = 0.0;
    WtEnu dir;
    double halfFovH;

    printf("Test_RightEdgeRay\n");

    WtProjection_SetupIntrinsics(IMG_W, IMG_H, FOV_H, FOV_V, &tanH, &tanV);
    CHECK(WtProjection_PixelToCameraRay((double)IMG_W, IMG_H / 2.0,
                                        IMG_W, IMG_H, tanH, tanV, &dir),
          "右边缘像素应有射线");

    halfFovH = FOV_H * 0.5;
    /* 射线与光轴的夹角应等于半视场角 */
    CHECK_NEAR(Wt_RadToDeg(atan2(dir.e, dir.u)), halfFovH, 0.01,
               "右边缘射线偏角应等于水平半视场角");
    CHECK_NEAR(dir.n, 0.0, 1e-9, "右边缘射线 y 分量应为 0");
}

/**
 * 图像下边缘 → 相机系 y 为正（y 轴向下）。
 */
static void Test_BottomEdgeRay(void)
{
    double tanH = 0.0;
    double tanV = 0.0;
    WtEnu dir;

    printf("Test_BottomEdgeRay\n");

    WtProjection_SetupIntrinsics(IMG_W, IMG_H, FOV_H, FOV_V, &tanH, &tanV);
    CHECK(WtProjection_PixelToCameraRay(IMG_W / 2.0, (double)IMG_H,
                                        IMG_W, IMG_H, tanH, tanV, &dir),
          "下边缘像素应有射线");

    CHECK(dir.n > 0.0, "下边缘射线 y 分量应为正（y 向下），得到 %.4f", dir.n);
    CHECK_NEAR(Wt_RadToDeg(atan2(dir.n, dir.u)), FOV_V * 0.5, 0.01,
               "下边缘射线偏角应等于垂直半视场角");
}

/**
 * 相机正北、水平朝前 → 光轴射线应指向正北（ENU 的 n 分量为 1）。
 */
static void Test_CameraToEnu_North(void)
{
    WtCameraPose pose;
    WtEnu dirCam = {0.0, 0.0, 1.0};
    WtEnu out;

    printf("Test_CameraToEnu_North\n");

    memset(&pose, 0, sizeof(pose));
    pose.yawDeg = 0.0;   /* 正北 */
    pose.pitchDeg = 0.0; /* 水平 */
    pose.rollDeg = 0.0;

    out = WtProjection_CameraToEnu(&pose, dirCam);

    CHECK_NEAR(out.e, 0.0, 1e-9, "正北光轴应无东向分量");
    CHECK_NEAR(out.n, 1.0, 1e-9, "正北光轴北向分量应为 1");
    CHECK_NEAR(out.u, 0.0, 1e-9, "水平光轴应无天向分量");
}

/**
 * 相机正东、水平 → 光轴指向正东（e=1）。
 */
static void Test_CameraToEnu_East(void)
{
    WtCameraPose pose;
    WtEnu dirCam = {0.0, 0.0, 1.0};
    WtEnu out;

    printf("Test_CameraToEnu_East\n");

    memset(&pose, 0, sizeof(pose));
    pose.yawDeg = 90.0; /* 正东 */

    out = WtProjection_CameraToEnu(&pose, dirCam);

    CHECK_NEAR(out.e, 1.0, 1e-9, "正东光轴东向分量应为 1");
    CHECK_NEAR(out.n, 0.0, 1e-9, "正东光轴应无北向分量");
}

/**
 * 相机正北、俯仰 30° 上仰 → 光轴应有天向分量 sin(30)=0.5。
 */
static void Test_CameraToEnu_Pitch(void)
{
    WtCameraPose pose;
    WtEnu dirCam = {0.0, 0.0, 1.0};
    WtEnu out;

    printf("Test_CameraToEnu_Pitch\n");

    memset(&pose, 0, sizeof(pose));
    pose.yawDeg = 0.0;
    pose.pitchDeg = 30.0;

    out = WtProjection_CameraToEnu(&pose, dirCam);

    CHECK_NEAR(out.u, 0.5, 1e-9, "30° 上仰的天向分量应为 0.5");
    CHECK_NEAR(out.n, cos(Wt_DegToRad(30.0)), 1e-9, "北向分量应为 cos(30°)");
}

/**
 * 射线与平面求交：从原点沿 +北 打，平面在正北 100 m 处。
 * 交点应在 (0,100,0)，距离 100。
 */
static void Test_RayPlaneIntersect(void)
{
    WtEnu origin = {0.0, 0.0, 0.0};
    WtEnu dir = {0.0, 1.0, 0.0};
    WtEnu planePoint = {0.0, 100.0, 0.0};
    WtEnu planeNormal = {0.0, 1.0, 0.0};
    WtProjectionResult res;

    printf("Test_RayPlaneIntersect\n");

    CHECK(WtProjection_RayPlaneIntersect(origin, dir, planePoint, planeNormal, &res),
          "求交应成功");
    CHECK(res.hit, "应命中");
    CHECK_NEAR(res.point.n, 100.0, 1e-9, "交点北向应为 100");
    CHECK_NEAR(res.distanceM, 100.0, 1e-9, "斜距应为 100");
}

/**
 * 射线与平面平行 → 必须返回 false，不能给出一个假交点。
 */
static void Test_RayPlaneParallel(void)
{
    WtEnu origin = {0.0, 0.0, 0.0};
    WtEnu dir = {0.0, 1.0, 0.0};      /* 朝北 */
    WtEnu planePoint = {0.0, 0.0, 50.0};
    WtEnu planeNormal = {0.0, 0.0, 1.0}; /* 水平面 → 与射线平行 */
    WtProjectionResult res;

    printf("Test_RayPlaneParallel\n");

    CHECK(!WtProjection_RayPlaneIntersect(origin, dir, planePoint, planeNormal, &res),
          "平行时应返回 false");
    CHECK(!res.hit, "平行时 hit 应为 false");
}

/**
 * 平面在射线反方向 → 视为未命中（目标在相机背后）。
 */
static void Test_RayPlaneBehind(void)
{
    WtEnu origin = {0.0, 0.0, 0.0};
    WtEnu dir = {0.0, 1.0, 0.0};       /* 朝北 */
    WtEnu planePoint = {0.0, -100.0, 0.0}; /* 平面在南边 */
    WtEnu planeNormal = {0.0, 1.0, 0.0};
    WtProjectionResult res;

    printf("Test_RayPlaneBehind\n");

    CHECK(!WtProjection_RayPlaneIntersect(origin, dir, planePoint, planeNormal, &res),
          "平面在背后时应返回 false");
}

/**
 * 端到端：相机在盘面正前方 100 m、正对盘面中心，拍图像中心。
 * 反投影结果应正好落在轮毂中心。
 */
static void Test_PixelToRotorPlane_Center(void)
{
    WtRotorFrame frame;
    WtCameraPose pose;
    WtProjectionResult res;

    printf("Test_PixelToRotorPlane_Center\n");

    /* 盘面：中心在原点，法向朝南（相机在南侧看） */
    memset(&frame, 0, sizeof(frame));
    frame.rotorCenter.e = 0.0;
    frame.rotorCenter.n = 0.0;
    frame.rotorCenter.u = 0.0;
    frame.rotAxis.e = 0.0;
    frame.rotAxis.n = -1.0; /* 指向南（下游），相机在南边 */
    frame.rotAxis.u = 0.0;
    frame.upRef.e = 0.0;
    frame.upRef.n = 0.0;
    frame.upRef.u = 1.0;
    frame.rotRef.e = 1.0;
    frame.rotRef.n = 0.0;
    frame.rotRef.u = 0.0;

    /* 相机在正南 100 m，朝北看 */
    memset(&pose, 0, sizeof(pose));
    pose.position.e = 0.0;
    pose.position.n = -100.0;
    pose.position.u = 0.0;
    pose.yawDeg = 0.0; /* 朝北 */
    pose.pitchDeg = 0.0;
    pose.rollDeg = 0.0;

    CHECK(WtProjection_PixelToRotorPlane(&pose, IMG_W / 2.0, IMG_H / 2.0,
                                         IMG_W, IMG_H, FOV_H, FOV_V,
                                         &frame, &res),
          "中心像素应命中盘面");
    CHECK(res.hit, "应命中");

    /* 交点应在轮毂中心（0,0,0），距离 100 */
    CHECK_NEAR(res.point.e, 0.0, 1e-6, "交点东向应为 0");
    CHECK_NEAR(res.point.n, 0.0, 1e-6, "交点北向应为 0");
    CHECK_NEAR(res.point.u, 0.0, 1e-6, "交点天向应为 0");
    CHECK_NEAR(res.distanceM, 100.0, 1e-6, "斜距应为 100 m");
}

/**
 * 盘面内坐标：盘面上正上方（upRef 方向）30 m 处的点，
 * 径向距离 30，相位角 0。
 */
static void Test_PointToRotorCoords(void)
{
    WtRotorFrame frame;
    WtEnu pt;
    double radial = 0.0;
    double angle = 0.0;

    printf("Test_PointToRotorCoords\n");

    memset(&frame, 0, sizeof(frame));
    frame.rotorCenter.e = 0.0;
    frame.rotorCenter.n = 0.0;
    frame.rotorCenter.u = 0.0;
    frame.upRef.e = 0.0;
    frame.upRef.n = 0.0;
    frame.upRef.u = 1.0;
    frame.rotRef.e = 1.0;
    frame.rotRef.n = 0.0;
    frame.rotRef.u = 0.0;

    /* 正上方 30 m */
    pt.e = 0.0;
    pt.n = 0.0;
    pt.u = 30.0;

    CHECK(WtProjection_PointToRotorCoords(&frame, pt, &radial, &angle),
          "盘面坐标解算应成功");
    CHECK_NEAR(radial, 30.0, 1e-9, "径向距离应为 30");
    CHECK_NEAR(angle, 0.0, 1e-9, "正上方相位角应为 0");

    /* 3 点钟方向（rotRef）40 m */
    pt.e = 40.0;
    pt.n = 0.0;
    pt.u = 0.0;
    CHECK(WtProjection_PointToRotorCoords(&frame, pt, &radial, &angle),
          "3 点钟方向解算应成功");
    CHECK_NEAR(radial, 40.0, 1e-9, "径向距离应为 40");
    CHECK_NEAR(angle, 90.0, 1e-9, "3 点钟方向相位角应为 90");
}

/* ------------------------------------------------------------------ */

int main(void)
{
    printf("=== 反投影测试 ===\n");

    Test_CenterRay();
    Test_RightEdgeRay();
    Test_BottomEdgeRay();
    Test_CameraToEnu_North();
    Test_CameraToEnu_East();
    Test_CameraToEnu_Pitch();
    Test_RayPlaneIntersect();
    Test_RayPlaneParallel();
    Test_RayPlaneBehind();
    Test_PixelToRotorPlane_Center();
    Test_PointToRotorCoords();

    if (s_failures == 0) {
        printf("全部通过\n");
        return 0;
    }

    printf("%d 项失败\n", s_failures);
    return 1;
}
