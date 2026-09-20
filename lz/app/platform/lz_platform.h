/**
 * @file lz_platform.h
 * @brief PSDK 平台层注册（console / OSAL / HAL / 文件系统 / socket）。
 *
 * 本文件是**唯一**被移植进来的官方代码路径：它把
 * `samples/sample_c/platform/linux/manifold3/application/main.c` 里的
 * `DjiUser_PrepareSystemEnvironment()` 拆出来单独成模块。
 *
 * 为什么要拆：这段代码有近 150 行样板（5 个 handler 结构体的字段逐一赋值），
 * 而**每个 PSDK 应用都必须原样照做一遍**。拆出来之后，探针程序与主应用
 * 共用同一份，改一处即改全部 —— 否则早晚会出现"探针能跑、主应用不行"
 * 这类由样板差异引起的怪问题。
 *
 * 移植时**未改任何逻辑**，只换掉日志路径与函数名前缀。
 */

#ifndef LZ_PLATFORM_H
#define LZ_PLATFORM_H

#include "dji_typedef.h"

/** 日志目录（相对应用目录，与仓库约定一致） */
#define LZ_LOG_DIR "data/logs"

/**
 * @brief 注册 PSDK 所需的全部平台层 handler
 *
 * 调用顺序即 PSDK 的硬约束：console / OSAL / HAL / 文件系统 / socket
 * 必须在 `DjiCore_Init()` **之前**全部注册完。
 */
T_DjiReturnCode LzPlatform_Prepare(void);

/** @brief 关闭日志文件 */
void LzPlatform_Deinit(void);

#endif /* LZ_PLATFORM_H */
