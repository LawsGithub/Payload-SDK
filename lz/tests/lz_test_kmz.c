/**
 * @file lz_test_kmz.c
 * @brief KMZ 容器与 wpml 生成的回归测试。
 *
 * zip 是二进制格式，最容易错的是 CRC 与中央目录偏移。这里做两层验证：
 *   1. CRC-32 对标准测试向量（"123456789" → 0xCBF43926，公认值）
 *   2. zip 结构标记齐全（本地头 / 中央目录 / EOCD 三大签名）
 *
 * ⚠️ 这**不能**替代独立解包器验证 —— 自己检查自己写的结构，
 * 只能证明"没写崩"，不能证明"别人能读懂"。真正的独立验证是：
 *     python3 -c "import zipfile;z=zipfile.ZipFile(p);print(z.testzip())"
 * 见 tools/lz_kmz_demo.c 的注释。
 */

#include "lz_kmz.h"
#include "lz_bridge.h"
#include "lz_plan.h"
#include "lz_test.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 在小端机器上按字节读 u32 */
static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static bool contains(const uint8_t *buf, size_t len, const char *needle)
{
    const size_t n = strlen(needle);
    if (n == 0 || len < n) {
        return false;
    }
    for (size_t i = 0; i + n <= len; ++i) {
        if (memcmp(buf + i, needle, n) == 0) {
            return true;
        }
    }
    return false;
}

int main(void)
{
    LZ_CASE("CRC-32 对标准测试向量");
    {
        /* "123456789" 的 CRC-32 是公认值 0xCBF43926 */
        const uint8_t v[] = "123456789";
        LZ_CHECK(LzKmz_Crc32(v, 9) == 0xCBF43926u);
        LZ_CHECK(LzKmz_Crc32((const uint8_t *)"", 0) == 0x00000000u);
    }

    LZ_CASE("zip 结构：三大签名与条目名齐全");
    {
        const LzKmzEntry entries[] = {
            { "wpmz/template.kml",  "<a/>", 4 },
            { "wpmz/waylines.wpml", "<b/>", 4 },
        };
        uint8_t *data = NULL;
        size_t size = 0;
        LZ_CHECK(LzKmz_Build(entries, 2, &data, &size) == LZ_OK);
        LZ_CHECK(data != NULL && size > 0);

        if (data != NULL) {
            /* 开头必须是本地文件头签名 */
            LZ_CHECK(rd32(data) == 0x04034b50u);
            /* 结尾必须是 EOCD 签名（22 字节，无注释） */
            LZ_CHECK(size >= 22);
            LZ_CHECK(rd32(data + size - 22) == 0x06054b50u);
            /* 两条目名都要出现 */
            LZ_CHECK(contains(data, size, "wpmz/template.kml"));
            LZ_CHECK(contains(data, size, "wpmz/waylines.wpml"));
            free(data);
        }
    }

    LZ_CASE("非法入参被拒绝");
    {
        uint8_t *d = NULL;
        size_t s = 0;
        const LzKmzEntry e[] = { { "a", "x", 1 } };
        LZ_CHECK(LzKmz_Build(NULL, 1, &d, &s) == LZ_ERR_PARAM);
        LZ_CHECK(LzKmz_Build(e, 0, &d, &s) == LZ_ERR_PARAM);
        LZ_CHECK(LzKmz_Build(e, 1, NULL, &s) == LZ_ERR_PARAM);
        LZ_CHECK(LzKmz_Write(e, 1, NULL) == LZ_ERR_PARAM);
        LZ_CHECK(LzKmz_Write(e, 1, "/no/such/dir/x.kmz") == LZ_ERR_IO);
    }

    LZ_CASE("KMZ 导出：含云台 yaw 且指向杆心");
    {
        LzTarget pole = {
            .id = 1,
            .geo = { .latitudeDeg = 30.5005, .longitudeDeg = 114.3003, .altitudeM = 30.0 },
            .heightM = 15.0, .radiusM = 0.1,
            .confidence = 0.9,
        };
        LzOrbitProfile profile = {
            .radiusM = 20.0, .altitudeM = 12.0, .speedMs = 3.0,
            .waypointCount = 8, .startBearingDeg = 0.0,
            .clockwise = true, .gimbalPitchDeg = -15.0,
        };

        LzRoute route;
        LzRoute_Init(&route);
        route.points = calloc(8, sizeof(LzWaypoint));
        LZ_CHECK(route.points != NULL);
        if (route.points != NULL) {
            for (int i = 0; i < 8; ++i) {
                double bearing = i * 45.0;
                LzGeo p;
                LzGeo_Destination(&pole.geo, bearing, 20.0, &p);
                route.points[i] = (LzWaypoint){
                    .geo = p, .relativeAltM = 12.0, .speedMs = 3.0,
                    .gimbalYawDeg = LzGeo_NormalizeDeg(bearing + 180.0),
                    .gimbalPitchDeg = -15.0,
                };
                route.count++;
            }

            const char *path = "lz_test_kmz_out.kmz";
            LZ_CHECK(LzBridge_ExportKmz(&route, &pole, &profile, path) == LZ_OK);

            FILE *fp = fopen(path, "rb");
            LZ_CHECK(fp != NULL);
            if (fp != NULL) {
                fseek(fp, 0, SEEK_END);
                long sz = ftell(fp);
                fseek(fp, 0, SEEK_SET);
                uint8_t *buf = malloc((size_t)sz);
                LZ_CHECK(buf != NULL);
                if (buf != NULL) {
                    LZ_CHECK(fread(buf, 1, (size_t)sz, fp) == (size_t)sz);
                    /* 绕飞的核心字段必须在包里 */
                    LZ_CHECK(contains(buf, (size_t)sz, "gimbalYawRotateEnable>1"));
                    LZ_CHECK(contains(buf, (size_t)sz, "absoluteAngle"));
                    /* 机型必须是 M4T 而不是样例的 M3E(77) */
                    LZ_CHECK(contains(buf, (size_t)sz, "droneEnumValue>99"));
                    free(buf);
                }
                fclose(fp);
            }
            remove(path);
            LzRoute_Free(&route);
        }
    }

    return LZ_TEST_SUMMARY();
}
