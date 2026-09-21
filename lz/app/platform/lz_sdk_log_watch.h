/**
 * @file lz_sdk_log_watch.h
 * @brief 从 SDK 自己的日志流里捞关键错误行。
 *
 * ## 为什么需要它 —— PSDK 不把真正的错误码给我们
 *
 * `DjiWaypointV3_Action(START)` 失败时返回 `0x000000FF`
 * （= `DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN`，system 模块的"未知错误"），
 * **它不携带任何定位信息**。飞机真正给的原因是 `dji_waypoint_v3.c` 自己
 * 打的另一行日志：
 *
 *     dji_waypoint_v3.c:497  Start waypoint v3 mission failed, index:0, error_code: 770, wayline_id: 0.
 *     dji_waypoint_v3.c:499  Failed to start at current rc mode.
 *     dji_waypoint_v3.c:252  Execute waypoint v3 action 0 failed, error: 0x000000FF.
 *
 * `770 = 0x302` 是 `DJI_ERROR_WAYPOINT_V3_MODULE_CODE_CANNOT_START_AT_CURRENT_RC_MODE`
 * —— 有明确的操作员补救动作。**但没有任何 API 能取到它** `[V]` 已核实：
 *
 *   - `dji_waypoint_v3.h` 全部 6 个导出函数，没有错误码查询接口
 *   - 状态回调 `T_DjiWaypointV3MissionState` 只有 state / wayLineId / index
 *   - `dji_error.h` 没有"错误码→字符串"的运行时函数（只有编译期宏表；
 *     库内部倒是有 `DjiError_GetErrorMsgElements`，但未在头文件导出）
 *
 * ## 但它就在我们手上路过
 *
 * 我们自己注册了 logger console（`lz_platform.c` 的 `LzPlatform_PrintConsole`
 * 与 `LzPlatform_LogWrite`）—— **SDK 打出的每一行都从这里过**。
 * 所以不必发明接口，只要看。
 *
 * 代价要写清楚：**这是日志抓取，不是结构化接口**。日志格式变了就失效。
 * 因此设计上：抓不到时返回 NULL，调用方必须容忍"没有这条信息"，
 * **不能把"抓不到"当成"没失败"**。
 */

#ifndef LZ_SDK_LOG_WATCH_H
#define LZ_SDK_LOG_WATCH_H

#include <stdint.h>

/**
 * @brief 喂一段 SDK 日志字节流（由 logger console 调用）
 *
 * 内部自己拼行 —— console 回调拿到的可能是半行，不能假设它按行切好。
 */
void LzSdkLogWatch_Feed(const uint8_t *data, uint16_t len);

/**
 * @brief 取最近一次"航点启动被拒"的原始日志行
 * @return 静态缓冲的字符串（已去掉行尾 CR/LF），没有则返回 NULL
 *
 * 形如：`Start waypoint v3 mission failed, index:0, error_code: 770, wayline_id: 0.`
 */
const char *LzSdkLogWatch_LastStartReject(void);

/**
 * @brief 取最近一次 SDK 内部报的 PSDK 返回值
 * @return 形如 `Execute waypoint v3 action 0 failed, error: 0x000000FF.`；没有则 NULL
 */
const char *LzSdkLogWatch_LastActionError(void);

#endif /* LZ_SDK_LOG_WATCH_H */
