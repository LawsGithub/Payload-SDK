/**
 * @file lz_app_config.h
 * @brief 机载应用的运行配置。
 */

#ifndef LZ_APP_CONFIG_H
#define LZ_APP_CONFIG_H

#include "lz_plan.h"
#include "lz_vision.h"

typedef struct {
    char outputDir[256];    /*!< 航线与日志输出目录，相对应用目录 */
    char configPath[256];   /*!< 作业参数文件路径 */
    LzOrbitProfile orbit;   /*!< 绕飞剖面 */
    LzVisionConfig vision;  /*!< 视觉参数 */
} LzAppConfig;

/** @brief 载入配置；文件不存在时用默认值并返回 LZ_ERR_IO（调用方**不退出**） */
LzStatus LzAppConfig_Load(const char *path, LzAppConfig *out);

#endif /* LZ_APP_CONFIG_H */
