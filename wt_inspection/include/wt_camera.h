/**
 * @file wt_camera.h
 * @brief 相机成像模型 —— 地面采样距离(GSD)、视场角与拍摄间距换算
 *
 * 采用「等效 35mm 画幅」模型。大疆官方标注的等效焦距按画幅对角线定义，
 * 因此只要知道等效焦距与像素数，就能直接推出对角线 GSD 与视场角，
 * 无需关心实际传感器尺寸。以 M4T 长焦为例：
 *
 *   FOV_diag = 2·atan(43.267 / (2 × 168)) = 14.68°  —— 与官方 15° 吻合
 *   GSD@12m  = 4.3267µm × 12 / 0.168    = 0.31 mm/px
 *
 * 模型自洽后，航线规划的「分辨率指标」就不再是经验值，而是可核算的约束。
 */

#ifndef WT_CAMERA_H
#define WT_CAMERA_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 等效 35mm 画幅对角线长度 mm */
#define WT_CAMERA_FF_DIAG_MM 43.2666153

/** 相机成像模型 */
typedef struct {
    const char *name;      /*!< 相机名称，便于日志与报告中标注 */
    double eflMm;          /*!< 等效 35mm 画幅焦距 mm */
    int imageWidthPx;      /*!< 照片宽度像素 */
    int imageHeightPx;     /*!< 照片高度像素 */
} WtCameraModel;

/** 由相机模型推导出的几何量 */
typedef struct {
    double sensorDiagMm; /*!< 等效画幅对角线 mm（恒为 43.267） */
    double sensorWidthMm; /*!< 等效画幅宽度 mm */
    double sensorHeightMm;/*!< 等效画幅高度 mm */
    double fovDiagDeg;   /*!< 对角线视场角 度 */
    double fovHorizDeg;  /*!< 水平视场角 度 */
    double fovVertDeg;   /*!< 垂直视场角 度 */
    double pixelPitchUm; /*!< 等效像元尺寸 µm（沿对角线） */
} WtCameraGeometry;

/** M4T 内置三摄与红外热像仪参数（来源：DJI Matrice 4 系列规格页） */
extern const WtCameraModel WT_CAMERA_M4T_WIDE;   /* 广角 24mm  48MP */
extern const WtCameraModel WT_CAMERA_M4T_MID;    /* 中焦 70mm  48MP */
extern const WtCameraModel WT_CAMERA_M4T_TELE;   /* 长焦 168mm 48MP */

/**
 * @brief 由相机模型推导视场角、像元尺寸等几何量
 * @return 几何量；若模型非法则各字段为 0
 */
WtCameraGeometry WtCamera_ComputeGeometry(const WtCameraModel *cam);

/**
 * @brief 变焦后的视场几何：等效焦距 × 倍率，像幅比例不变。
 *
 * 实时画面跟着 Pilot 2 的变焦走时，像素 → 角度的换算必须跟着变：按广角
 * 视场去算 3× 下的画面，每像素角度被高估 3 倍，云台每次都转过头、越转越远。
 * 反过来（倍率给大了）只会少转、多迭代几次，仍然收敛 —— 所以拿不准时宁可
 * 按读到的倍率算。
 *
 * @param zoomFactor 相对 base 这颗镜头的倍率。< 1 或非有限值按 1 处理
 *        （读变焦失败时传进来的 0 不该得到一个 180° 的视场）
 */
WtCameraGeometry WtCamera_ComputeZoomedGeometry(const WtCameraModel *base, double zoomFactor);

/**
 * @brief 指定拍摄距离处的地面采样距离
 * @param distanceM 相机光心到目标的斜距 m
 * @return GSD，单位 mm/pixel
 */
double WtCamera_GsdAtDistance(const WtCameraModel *cam, double distanceM);

/**
 * @brief 在给定拍摄距离下，满足目标 GSD 所需的最大拍摄距离
 * @param targetGsdMmPerPx 目标 GSD，单位 mm/pixel
 */
double WtCamera_MaxDistanceForGsd(const WtCameraModel *cam, double targetGsdMmPerPx);

/**
 * @brief 在给定拍摄距离下，画面沿「垂直方向」的实际覆盖高度
 */
double WtCamera_CoverageHeightAt(const WtCameraModel *cam, double distanceM);

/**
 * @brief 在给定拍摄距离下，画面沿「水平方向」的实际覆盖宽度
 */
double WtCamera_CoverageWidthAt(const WtCameraModel *cam, double distanceM);

/**
 * @brief 由重叠率要求反推相邻拍摄点间距（核心换算）
 *
 * @param cam        相机模型
 * @param distanceM  拍摄斜距 m
 * @param overlapPct 期望重叠率，0~95（%）
 * @param alongHorizontal true=沿画面水平方向排布，false=沿垂直方向排布
 * @return 相邻拍摄点间距 m；参数非法时返回 0
 */
double WtCamera_SpacingForOverlap(const WtCameraModel *cam, double distanceM,
                                  double overlapPct, bool alongHorizontal);

/**
 * @brief 由期望的展向点间距反算实际重叠率（%）
 */
double WtCamera_OverlapForSpacing(const WtCameraModel *cam, double distanceM,
                                  double spacingM, bool alongHorizontal);

#ifdef __cplusplus
}
#endif

#endif /* WT_CAMERA_H */