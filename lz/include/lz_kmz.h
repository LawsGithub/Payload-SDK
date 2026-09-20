/**
 * @file lz_kmz.h
 * @brief KMZ 容器写入（zip，store 模式，不压缩）。
 *
 * `[V]` `.kmz` 就是一个 zip（`file` 命令会报 `Zip archive data`），
 * 里面固定两个 XML：`wpmz/template.kml` 与 `wpmz/waylines.wpml`。
 *
 * ## 为什么不引 zlib
 *
 * 航点文件只有几 KB，压不压缩对传输毫无影响；而引 zlib 会让 lz_core
 * 多一个外部依赖，破坏"桌面上 clone 下来就能编"的性质。
 * 所以这里只用 zip 的 **store（compression method 0）** 模式 ——
 * 结构简单到可以手写，且完全符合规范，DJI 侧照常解析。
 *
 * ## 为什么放在 lz_core 而不是 PSDK 层
 *
 * 生成 KMZ 不依赖任何 PSDK 类型 —— 它纯粹是文件格式。
 * 放在这里意味着**能在桌面上生成、解开、逐字节核对**，不必上飞机。
 * 只有最后 DjiWaypointV3_UploadKmzFile() 那一步才需要 PSDK。
 */

#ifndef LZ_KMZ_H
#define LZ_KMZ_H

#include <stddef.h>
#include <stdint.h>

#include "lz_types.h"

/** zip 里的一项（名字 + 内容），内容为 NULL 表示目录项 */
typedef struct {
    const char *name;   /*!< 包内路径，例如 "wpmz/template.kml" */
    const char *data;   /*!< 内容，以 '\0' 结尾 */
    size_t size;        /*!< 内容字节数（不含结尾 '\0'） */
} LzKmzEntry;

/**
 * @brief 把若干条目打成 zip 写到文件
 *
 * @param entries 条目数组
 * @param count   条目数
 * @param outPath 输出 .kmz 路径
 */
LzStatus LzKmz_Write(const LzKmzEntry *entries, size_t count, const char *outPath);

/**
 * @brief 把若干条目打成 zip 到内存
 *
 * 给 `DjiWaypointV3_UploadKmzFile(data, len)` 用 —— 它要的是内存缓冲，
 * 不必先落盘再读回来。
 *
 * @param outData  [out] 由本函数 malloc，调用方负责 free
 * @param outSize  [out] 字节数
 */
LzStatus LzKmz_Build(const LzKmzEntry *entries, size_t count,
                     uint8_t **outData, size_t *outSize);

/** @brief 标准 CRC-32（zip 用的那个），单独暴露便于测试 */
uint32_t LzKmz_Crc32(const uint8_t *data, size_t len);

#endif /* LZ_KMZ_H */
