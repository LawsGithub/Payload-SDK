/**
 * @file dji_logger.h（测试替身）
 * @brief 让 `app/lz_pole_source.c` 能脱离 PSDK 编译，只提供 `USER_LOG_*` 宏。
 *
 * ## 这个替身替换了什么，没替换什么
 *
 * **只替换日志宏** —— 把 `USER_LOG_INFO/WARN/ERROR` 变成 `printf`。
 * `lz_pole_source.c` 本身是**原封不动**编译进来的，被测逻辑一行都没改。
 *
 * ⚠️ 所以这个替身**不能**用来测那些依赖 PSDK 行为的东西
 * （话题订阅、激光测距的真实返回值）。它只服务于一件事：
 * "记录 → 写文件 → 读回"这段与 PSDK 无关的逻辑能在桌面上验。
 *
 * ## 为什么值得这么做
 *
 * 落盘往返是**现场最容易出问题、桌面最容易验**的一段：格式写错、字段读丢、
 * 半个坐标被当成有效值 —— 这些错误在设备上表现为"记录的圆心是个奇怪的坐标"，
 * 而排查要接飞机。放到桌面上，一秒就能跑完 9 个用例。
 *
 * 如果哪天有人把落盘逻辑挪进 PSDK 依赖里，这个测试会**编译失败** ——
 * 那正是提醒"这段逻辑变得只能在设备上验了"。
 */

#ifndef LZ_TEST_STUB_DJI_LOGGER_H
#define LZ_TEST_STUB_DJI_LOGGER_H

#include <stdio.h>

#define USER_LOG_INFO(...)                                                     \
    do {                                                                       \
        printf("[INFO] ");                                                     \
        printf(__VA_ARGS__);                                                   \
        printf("\n");                                                          \
    } while (0)

#define USER_LOG_WARN(...)                                                     \
    do {                                                                       \
        printf("[WARN] ");                                                     \
        printf(__VA_ARGS__);                                                   \
        printf("\n");                                                          \
    } while (0)

#define USER_LOG_ERROR(...)                                                    \
    do {                                                                       \
        printf("[ERRO] ");                                                     \
        printf(__VA_ARGS__);                                                   \
        printf("\n");                                                          \
    } while (0)

#endif /* LZ_TEST_STUB_DJI_LOGGER_H */
