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
 * @warning 本函数会把云台设为 `DJI_GIMBAL_MODE_FREE`（姿态锁定地面坐标系、
 *          忽略飞机运动）—— 静止观测一个固定目标需要它，否则 `YAW_FOLLOW`
 *          下云台 yaw 跟着机身转，会和我们下发的绝对方位角互相抢。
 *
 *          **`WtLaser_DeInit()` 必须被调用，它会把模式还成 `YAW_FOLLOW`。**
 *          这不是礼节问题：`FREE` 残留在飞机上，之后一旦飞机头连续转动
 *          （比如绕飞航线），pan 关节为"保持地面姿态"要反向补偿，而它只有
 *          ±60° ⇒ 顶机械限位、电机持续给力 ⇒ 现场报「偏航顶限位 + 云台
 *          电机异常」。姊妹项目为这个踩过一次实打实的事故。
 *
 *          PSDK 没有 `GetGimbalMode`，读不回原值，所以只能还成 YAW_FOLLOW
 *          （M4T 的自然状态）而不是"原来那个"。
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
 * @brief 下发云台绝对角（pitch + yaw）。
 *
 * ## yaw 必须填一个**有意义**的值，不能填 0
 *
 * 绝对角模式下 `yaw = 0` 的意思是「转到正北」。而 M4T 的 pan 只有相对机头
 * ±60°，机头不朝北时正北就够不到 ⇒ **整条命令被拒**，报
 * `0x600000004 YAW_REACH_POSITIVE_LIMIT`，**连 pitch 都不会转到位**
 * （可能部分执行：先动了 pitch 才在 yaw 上撞限位）。
 *
 * 姊妹项目 2026-09-27 实测踩过这个坑。官方样例 `test_gimbal_manager.c`
 * 的做法是绝对角模式下**先读出当前 yaw 再原样传回** —— 即「不想动 yaw」
 * 要表达成「yaw 保持当前值」，而不是「yaw = 0」。
 *
 * 所以：
 *   - 要转 yaw → 传目标方位角
 *   - 不想转 yaw → **传当前云台 yaw**
 *
 * @param pitchDeg 绝对俯仰角（度，水平 0，向下为负）。应先用
 *                 `WtAim_IsReachable()` 确认在 [-90, +70] 内
 * @param yawDeg   绝对方位角（度，自正北顺时针）。见上面的说明
 * @return false 表示被拒；日志里会分辨「控制权被占」与「参数超限」两种病因
 */
bool WtLaser_SetGimbalAbs(double pitchDeg, double yawDeg);

#ifdef __cplusplus
}
#endif

#endif /* WT_LASER_H */
