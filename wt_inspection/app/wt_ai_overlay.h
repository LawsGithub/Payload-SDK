/**
 * @file wt_ai_overlay.h
 * @brief AI 叠加：把行人框随帧编码成 H.264，由妙算3 推给 Pilot（操作员辅助，默认关）。
 *
 * ## 来历
 *
 * 2026-10-21 只单独调 `DjiLiveview_SendAiMetaToPilot`：返回 SUCCESS，但 Pilot 2
 * 上没有框。当时的待验证项第 2 条正是「是否必须先启动视频流才生效」—— 那次没起流。
 *
 * 官方样例 `samples/sample_c++/module_sample/liveview/dji_liveview_object_detection.cpp`
 * 的完整流程是：取图 → 识别 → `DjiLiveview_EncodeAFrameToH264`（帧 + 框）→
 * 编码回调里 `DjiPayloadCamera_SendVideoStream` 推给 Pilot。本模块照它补齐后，
 * 2026-10-26 上机：Pilot 2 多出一路**妙算3 的视频源**，画面上有框。框只在这一路上，
 * 主相机画面上没有。
 *
 * ## 怎么用
 *
 * 配置 `[app]` 段 `ai_overlay = true`，且 `ped_enabled = true`（叠加挂在行人链路的
 * 取流上，用它的帧和识别框）。Pilot 2 里先关掉 DJI 自带的 AI 识别（两者互斥）。
 *
 * - 识别到的人画成 `pedestrian` / `people` 框
 * - 其中一个标成 `target`：拨「行人瞄准」会锁定的那个人（会话中则是正在跟的那个），
 *   判据见 `WtPed_TargetIndex`。画面里没人时只有视频、没有框
 * - 运行中每 5 秒打一行编码与推流统计（`dji_app_ctl stop` 不走收尾，只能看运行中）
 *
 * 代价：每帧编码约 5 ms（在 SDK 取流回调线程里）、多一路约 6 Mbps 的视频，Pilot 还会
 * 来要媒体文件列表（日志里 dji_downloader 的 0xE3 报错，未见影响）。不需要时关掉。
 */

#ifndef WT_AI_OVERLAY_H
#define WT_AI_OVERLAY_H

#include <stdbool.h>

/**
 * @brief 注册标签与编码回调，挂到 wt_frame_pump 的帧旁路上。
 *
 * @pre 取流已启动（`WtFramePump_Start` 成功）—— 本模块不自己 `DjiLiveview_Init`。
 *
 * 失败**不阻断启动** —— 与控件模块同一原则：这是验证/增强功能，
 * 不是作业前提。装包时安装器会试运行应用，这里 fail-closed 会让包装不上。
 *
 * @param enabled 是否启用（来自配置）
 * @return true 表示已挂上
 */
bool WtAiOverlay_Start(bool enabled);

/**
 * @brief 摘下帧旁路，注销编码回调与标签，带宽比例还原为默认。
 *
 * @note 必须在 `WtFramePump_Stop` 之前调用（之后 liveview 已 Deinit，注销会失败）。
 */
void WtAiOverlay_Stop(void);

#endif /* WT_AI_OVERLAY_H */
