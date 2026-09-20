/**
 * @file lz_kmz.c
 * @brief KMZ（zip，store 模式）写入实现。
 *
 * 只实现 zip 的最小子集：本地文件头 + 数据 + 中央目录 + 结尾记录。
 * 不做压缩、不做 zip64、不加密 —— 航点文件小到几百 KB 都不会越界，
 * zip64 是多余的复杂度。
 *
 * 全文件小端序（zip 规范）。
 */

#include "lz_kmz.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- CRC-32（IEEE 802.3，zip 用的就是这个）---- */

uint32_t LzKmz_Crc32(const uint8_t *data, size_t len)
{
    if (data == NULL && len != 0) {
        return 0;
    }
    /* 表驱动：惰性初始化一次，之后 O(1) 查表 */
    static uint32_t table[256];
    static int ready = 0;
    if (!ready) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) {
                c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            }
            table[i] = c;
        }
        ready = 1;
    }

    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc = table[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

/* ---- 小端写入 ---- */

typedef struct {
    uint8_t *buf;
    size_t len;
    size_t cap;
} LzByteBuf;

static bool lz_buf_ensure(LzByteBuf *b, size_t extra)
{
    if (b->len + extra <= b->cap) {
        return true;
    }
    size_t newCap = (b->cap == 0) ? 4096 : b->cap;
    while (newCap < b->len + extra) {
        newCap *= 2;
    }
    uint8_t *grown = realloc(b->buf, newCap);
    if (grown == NULL) {
        return false;
    }
    b->buf = grown;
    b->cap = newCap;
    return true;
}

static bool lz_buf_u16(LzByteBuf *b, uint16_t v)
{
    if (!lz_buf_ensure(b, 2)) {
        return false;
    }
    b->buf[b->len++] = (uint8_t)(v & 0xFFu);
    b->buf[b->len++] = (uint8_t)((v >> 8) & 0xFFu);
    return true;
}

static bool lz_buf_u32(LzByteBuf *b, uint32_t v)
{
    if (!lz_buf_ensure(b, 4)) {
        return false;
    }
    b->buf[b->len++] = (uint8_t)(v & 0xFFu);
    b->buf[b->len++] = (uint8_t)((v >> 8) & 0xFFu);
    b->buf[b->len++] = (uint8_t)((v >> 16) & 0xFFu);
    b->buf[b->len++] = (uint8_t)((v >> 24) & 0xFFu);
    return true;
}

static bool lz_buf_bytes(LzByteBuf *b, const uint8_t *p, size_t n)
{
    if (!lz_buf_ensure(b, n)) {
        return false;
    }
    memcpy(b->buf + b->len, p, n);
    b->len += n;
    return true;
}

/* ---- 打包 ---- */

/* zip 的时间/日期字段用 DOS 格式；这里固定一个合法值而不取当前时间 ——
 * 设备 RTC 无电池、掉电回 1970，取当前时间会得到无意义的值。
 * 1980-01-01 00:00:00 是 DOS 格式的合法下界。 */
#define LZ_ZIP_DOS_TIME 0u
#define LZ_ZIP_DOS_DATE 33u   /* 1980-01-01 */

LzStatus LzKmz_Build(const LzKmzEntry *entries, size_t count,
                     uint8_t **outData, size_t *outSize)
{
    if (entries == NULL || outData == NULL || outSize == NULL) {
        return LZ_ERR_PARAM;
    }
    if (count == 0) {
        return LZ_ERR_PARAM;
    }
    /* zip 的中央目录里条目数、偏移都是 16/32 位，条目数超了就得 zip64，
     * 本项目不会用到；直接挡住比写错强。 */
    if (count > 0xFFFFu) {
        return LZ_ERR_RANGE;
    }

    LzByteBuf b = { NULL, 0, 0 };
    uint32_t *crcs = calloc(count, sizeof(*crcs));
    uint32_t *offsets = calloc(count, sizeof(*offsets));
    uint32_t *sizes = calloc(count, sizeof(*sizes));
    uint16_t *nameLens = calloc(count, sizeof(*nameLens));
    if (crcs == NULL || offsets == NULL || sizes == NULL || nameLens == NULL) {
        free(crcs); free(offsets); free(sizes); free(nameLens);
        return LZ_ERR_IO;
    }

    LzStatus st = LZ_OK;

    /* ---- 本地文件头 + 数据 ---- */
    for (size_t i = 0; i < count && st == LZ_OK; ++i) {
        const char *name = entries[i].name;
        if (name == NULL || strlen(name) == 0 || strlen(name) > 0xFFFFu) {
            st = LZ_ERR_PARAM;
            break;
        }
        const size_t nameLen = strlen(name);
        const uint32_t dataLen = (uint32_t)entries[i].size;
        const uint8_t *data = (const uint8_t *)entries[i].data;
        const uint32_t crc = LzKmz_Crc32(data, entries[i].size);

        offsets[i] = (uint32_t)b.len;
        crcs[i] = crc;
        sizes[i] = dataLen;
        nameLens[i] = (uint16_t)nameLen;

        bool ok = lz_buf_u32(&b, 0x04034b50u)   /* 本地文件头签名 */
               && lz_buf_u16(&b, 20)            /* 解压所需版本 2.0 */
               && lz_buf_u16(&b, 0x0800u)       /* 通用标志：bit11 = 文件名为 UTF-8 */
               && lz_buf_u16(&b, 0)             /* 压缩方法 0 = store */
               && lz_buf_u16(&b, LZ_ZIP_DOS_TIME)
               && lz_buf_u16(&b, LZ_ZIP_DOS_DATE)
               && lz_buf_u32(&b, crc)
               && lz_buf_u32(&b, dataLen)       /* 压缩后大小 == 原始大小 */
               && lz_buf_u32(&b, dataLen)
               && lz_buf_u16(&b, (uint16_t)nameLen)
               && lz_buf_u16(&b, 0)             /* 扩展字段长度 */
               && lz_buf_bytes(&b, (const uint8_t *)name, nameLen)
               && lz_buf_bytes(&b, data, dataLen);
        if (!ok) {
            st = LZ_ERR_IO;
        }
    }

    /* ---- 中央目录 ---- */
    const uint32_t centralStart = (uint32_t)b.len;
    for (size_t i = 0; i < count && st == LZ_OK; ++i) {
        bool ok = lz_buf_u32(&b, 0x02014b50u)   /* 中央目录头签名 */
               && lz_buf_u16(&b, 20)            /* 创建版本 */
               && lz_buf_u16(&b, 20)            /* 解压所需版本 */
               && lz_buf_u16(&b, 0x0800u)
               && lz_buf_u16(&b, 0)
               && lz_buf_u16(&b, LZ_ZIP_DOS_TIME)
               && lz_buf_u16(&b, LZ_ZIP_DOS_DATE)
               && lz_buf_u32(&b, crcs[i])
               && lz_buf_u32(&b, sizes[i])
               && lz_buf_u32(&b, sizes[i])
               && lz_buf_u16(&b, nameLens[i])
               && lz_buf_u16(&b, 0)             /* 扩展字段 */
               && lz_buf_u16(&b, 0)             /* 注释 */
               && lz_buf_u16(&b, 0)             /* 起始磁盘号 */
               && lz_buf_u16(&b, 0)             /* 内部属性 */
               && lz_buf_u32(&b, 0)             /* 外部属性 */
               && lz_buf_u32(&b, offsets[i])
               && lz_buf_bytes(&b, (const uint8_t *)entries[i].name, nameLens[i]);
        if (!ok) {
            st = LZ_ERR_IO;
        }
    }
    const uint32_t centralSize = (uint32_t)b.len - centralStart;

    /* ---- 中央目录结尾 ---- */
    if (st == LZ_OK) {
        bool ok = lz_buf_u32(&b, 0x06054b50u)   /* EOCD 签名 */
               && lz_buf_u16(&b, 0)             /* 本磁盘号 */
               && lz_buf_u16(&b, 0)             /* 中央目录起始磁盘号 */
               && lz_buf_u16(&b, (uint16_t)count)
               && lz_buf_u16(&b, (uint16_t)count)
               && lz_buf_u32(&b, centralSize)
               && lz_buf_u32(&b, centralStart)
               && lz_buf_u16(&b, 0);            /* 注释长度 */
        if (!ok) {
            st = LZ_ERR_IO;
        }
    }

    free(crcs); free(offsets); free(sizes); free(nameLens);

    if (st != LZ_OK) {
        free(b.buf);
        return st;
    }

    *outData = b.buf;
    *outSize = b.len;
    return LZ_OK;
}

LzStatus LzKmz_Write(const LzKmzEntry *entries, size_t count, const char *outPath)
{
    if (outPath == NULL) {
        return LZ_ERR_PARAM;
    }

    uint8_t *data = NULL;
    size_t size = 0;
    LzStatus st = LzKmz_Build(entries, count, &data, &size);
    if (st != LZ_OK) {
        return st;
    }

    FILE *fp = fopen(outPath, "wb");
    if (fp == NULL) {
        free(data);
        return LZ_ERR_IO;
    }
    const size_t written = fwrite(data, 1, size, fp);
    fclose(fp);
    free(data);

    return (written == size) ? LZ_OK : LZ_ERR_IO;
}
