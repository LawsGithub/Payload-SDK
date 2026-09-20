/**
 * @file lz_test_bridge.c
 * @brief 桥接层的回归测试（只测与 PSDK 无关的部分）。
 *
 * 真正的格式转换（LzRoute → T_DjiWayPointV2MissionSettings）依赖 PSDK 头文件，
 * 只能在机载目标里编译，不在这里。本文件测的是能上桌面的那半边：
 * CSV 导出。
 */

#include "lz_bridge.h"
#include "lz_test.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(void)
{
    LZ_CASE("CSV 导出：文件应存在且列数正确");
    {
        LzRoute route;
        LzRoute_Init(&route);

        route.points = malloc(2 * sizeof(LzWaypoint));
        LZ_CHECK(route.points != NULL);
        route.points[0] = (LzWaypoint){
            .geo = { .latitudeDeg = 30.5, .longitudeDeg = 114.3, .altitudeM = 30.0 },
            .relativeAltM = 12.0, .speedMs = 3.0,
            .gimbalYawDeg = 0.0, .gimbalPitchDeg = -15.0,
        };
        route.points[1] = route.points[0];
        route.points[1].gimbalYawDeg = 180.0;
        route.count = 2;
        route.capacity = 2;

        const char *path = "lz_test_bridge_out.csv";
        LZ_CHECK(LzBridge_ExportCsv(&route, path) == LZ_OK);

        FILE *fp = fopen(path, "r");
        LZ_CHECK(fp != NULL);
        if (fp != NULL) {
            char line[512];
            int lines = 0;
            while (fgets(line, sizeof(line), fp) != NULL) {
                lines++;
            }
            fclose(fp);
            /* 1 行表头 + 2 行数据 */
            LZ_CHECK(lines == 3);

            /* 第二行应含 yaw=0.00，第三行含 yaw=180.00 */
            fp = fopen(path, "r");
            LZ_CHECK(fgets(line, sizeof(line), fp) != NULL);  /* 表头 */
            LZ_CHECK(fgets(line, sizeof(line), fp) != NULL);
            LZ_CHECK(strstr(line, ",0.00,-15.00") != NULL);
            LZ_CHECK(fgets(line, sizeof(line), fp) != NULL);
            LZ_CHECK(strstr(line, ",180.00,-15.00") != NULL);
            fclose(fp);
        }
        remove(path);
        LzRoute_Free(&route);
    }

    LZ_CASE("CSV 导出：非法入参被拒绝");
    {
        LzRoute route;
        LzRoute_Init(&route);
        LZ_CHECK(LzBridge_ExportCsv(NULL, "x.csv") == LZ_ERR_PARAM);
        LZ_CHECK(LzBridge_ExportCsv(&route, NULL) == LZ_ERR_PARAM);
        /* 目录不存在应报 IO 错误而不是崩溃 */
        LZ_CHECK(LzBridge_ExportCsv(&route, "/no/such/dir/x.csv") == LZ_ERR_IO);
        LzRoute_Free(&route);
    }

    return LZ_TEST_SUMMARY();
}
