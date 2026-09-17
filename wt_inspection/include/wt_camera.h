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