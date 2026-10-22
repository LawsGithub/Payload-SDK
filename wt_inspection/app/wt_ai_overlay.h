/**
 * @file wt_ai_overlay.h
 * @brief PSDK 目标识别叠加验证（Pilot 2 画面画框）。
 *
 * ## 要验证什么
 *
 * PSDK 提供了三个 API 把第三方应用的识别结果画到 Pilot 2 画面上：
 *
 *   DjiLiveview_RegUserAiTargetLableList()   注册自定义标签
 *   DjiLiveview_SendAiMetaToPilot()          发送框信息
 *   DjiLiveview_UnregUserAiTargetLableList() 注销
 *
 * 头文件（`psdk_lib/include/dji_liveview.h:324/331/339`）确认它们存在，
 * 官方文档确认它们**仅 Manifold 3 支持** —— 正是本项目的设备。
 *
 * 但有两件事**静态查不出来，只能上机试**：
 *
 *   1. 自定义标签的 `type` 索引从 0 还是从 1 开始
 *   2. 是否必须先启动视频流，`SendAiMetaToPilot` 才生效
 *
 * 本模块用一个固定框（画面正中）验证这两点。
 *
 * ## 怎么用
 *
 * 配置项 `[ai_overlay] enabled = true` 打开（默认 false）。开启后：
 *
 *   - 启动时注册 3 个标签
 *   - 每秒发一个框，`type` 轮换 0/1/2，框位置依次在左中右
 *   - Pilot 2 上应看到框 + 标签文字
 *
 * 观察结论：
 *
 *   - 三个框都显示 → 索引从 0 开始，且三种 type 都被接受
 *   - 只有某个位置显示 → 该位置的 type 才是合法索引起点
 *   - 一个都不显示 → 要么索引规则不同，要么必须先起视频流
 *
 * 验证完记得关掉（`enabled = false`），不要留在生产配置里。
 */

#ifndef WT_AI_OVERLAY_H
#define WT_AI_OVERLAY_H

#include <stdbool.h>

/**
 * @brief 初始化并启动叠加线程。
 *
 * 失败**不阻断启动** —— 与控件模块同一原则：这是验证/增强功能，
 * 不是作业前提。装包时安装器会试运行应用，这里 fail-closed 会让包装不上。
 *
 * @param enabled 是否启用（来自配置）
 * @return true 表示线程已启动
 */
bool WtAiOverlay_Start(bool enabled);

/**
 * @brief 停止线程并注销标签。
 */
void WtAiOverlay_Stop(void);

#endif /* WT_AI_OVERLAY_H */
