/**
 * @file wt_laser.h
 * @brief 激光测距读取与云台俯仰下发。
 *
 * ## 激光打的是光轴，不能指定方向
 *
 * `T_DjiCameraManagerLaserRangingInfo` 里的 `screenX/screenY` 是**报告**
 * 激光落在画面的哪个位置，不是让你指定打哪。所以要测某个叶尖，唯一办法是
 * 把那个叶尖转到光轴上 —— 这就是 `wt_aim` 存在的原因。
 *
 * ## 返回的是经纬度，不只是距离
 *
 * 这一点容易被名字误导（"RangingInfo" 听起来只有距离）。它实际直接给出
 * 打中点的 **WGS84 经纬度 + 海拔**，所以不需要自己做反投影 —— 这是整个
 * 采集方案可行的前提。
 *
 * ⚠️ 但 `altitude` 的**基准没有文档承诺**。姊妹项目实测它与起飞点气压高
 * 差约 38 m（因此才有了"标定地面"按钮）。所以：
 *   - 不要把激光海拔与飞机 `position.alt` 直接相减
 *   - 拟合只用**点与点之间的相对量**（`wt_rotor_fit` 正是这么设计的，
 *     整体平移对结果免疫）
 *
 * ## 频率上限 5Hz
 *
 * 头文件原文：`@note Maximum data update frequency: 5Hz`。本模块按 250 ms
 * 限频，更密的调用拿到的是同一个值 —— 那会让"多次采样取中位数"变成
 * 自欺欺人（看起来采了 5 次，其实是同一个数重复 5 遍）。
 *
 * ## 这些调用会阻塞，不要放进回调线程
 *
 * `GetLaserRangingInfo` 是同步阻塞的（姊妹项目实测最多 1.2 s）。在 PSDK
 * 回调线程里调会把链路拖死（日志刷 `semaphore wait timeout` 后进程崩）。
 * 只能在采集线程或主循环里调。
 */

#ifndef WT_LASER_H
#define WT_LASER_H

#include <stdbool.h>
#include <stdint.h>

#include "wt_geometry.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 一次激光测距的结果 */
typedef struct {
    bool valid;
    WtGeo point;          /*!< 打中点 WGS84。⚠️ alt 基准未文档化，只用相对量 */
    double distanceM;
    double screenXPct;    /*!< 激光落点在画面的横向位置 0~100% */
    double screenYPct;    /*!< 纵向位置 0~100% */
    bool lidarEnabled;
    uint8_t exception;    /*!< 非 0 表示测距异常（打空、超量程、被遮挡等） */
} WtLaserSample;

/**
 * @brief 初始化相机与云台管理模块。
 *
 * @warning 本函数会把云台设为 `DJI_GIMBAL_MODE_FREE`（姿态锁定地面坐标系，
 *          忽略飞机运动）—— 这是"对准一个固定目标"唯一合理的模式。
 *
 *          但 PSDK **没有** `GetGimbalMode` 接口，所以**无法读回原模式、
 *          也就无法真正归还**。这违反了"设置型调用借了必须还"的原则，
 *          属于已知的、无法规避的副作用：
 *            - 本模块在 DeInit 时调 `DjiGimbalManager_Reset()` 做复位，
 *              这是能做到的最接近"归还"的动作
 *            - 作业结束后操作员在 Pilot 2 里看到的云台模式可能已被改变
 *          把它写在这里，是为了下次有人发现"云台模式莫名变了"时能立刻
 *          找到源头，而不是去翻整个工程。
 */
bool WtLaser_Init(void);

/** @brief 复位云台并反初始化 */
void WtLaser_DeInit(void);

/**
 * @brief 读一次激光测距。
 *
 * 内部按 5Hz 上限限频：距上次读取不足 250 ms 时返回上一次的结果
 * （`out->valid` 仍为 true，但它不是新值）。需要独立样本请自行间隔调用。
 *
 * @return true 表示 SDK 调用成功且 `exception == 0`
 */
bool WtLaser_Read(WtLaserSample *out);

/**
 * @brief 下发云台俯仰绝对角。
 *
 * **只下发 pitch。** M4T 的云台 Pan 轴标着 `Not controllable`，下发 yaw
 * 不会生效 —— 而现象是"云台动了但画面没对准水平偏差"，极难定位。
 * 水平对准必须靠机头（航点任务的 heading，或操作员手动）。
 *
 * @param pitchDeg 绝对俯仰角（度，水平 0，向下为负）
 * @param seconds  期望的执行时长。太短会让云台猛冲过冲，太长则采集变慢
 */
bool WtLaser_SetGimbalPitchAbs(double pitchDeg, double seconds);

#ifdef __cplusplus
}
#endif

#endif /* WT_LASER_H */
