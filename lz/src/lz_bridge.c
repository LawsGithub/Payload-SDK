/**
 * @file lz_bridge.c
 * @brief 桥接层：规划结果 → 可交付文件。
 *
 * CSV 导出已实现（用于人工核对航线，是 KMZ 出问题时的对照物）。
 * KMZ 生成留待 PSDK 侧联调时补 —— 它的正确性只能靠"上传后飞机是否
 * 照做"来验证，在桌面写再多也证明不了什么。
 */

#include "lz_bridge.h"
#include "lz_geo.h"
#include "lz_kmz.h"
#include "lz_wpml.h"

#include <stdio.h>
#include <string.h>

LzStatus LzBridge_ExportCsv(const LzRoute *route, const char *path)
{
    if (route == NULL || path == NULL) {
        return LZ_ERR_PARAM;
    }

    FILE *fp = fopen(path, "w");
    if (fp == NULL) {
        return LZ_ERR_IO;
    }

    fprintf(fp, "index,latitude,longitude,altitude_rel_m,speed_ms,gimbal_yaw_deg,gimbal_pitch_deg\n");
    for (size_t i = 0; i < route->count; ++i) {
        const LzWaypoint *wp = &route->points[i];
        fprintf(fp, "%zu,%.8f,%.8f,%.2f,%.2f,%.2f,%.2f\n",
                i, wp->geo.latitudeDeg, wp->geo.longitudeDeg,
                wp->relativeAltM, wp->speedMs,
                wp->gimbalYawDeg, wp->gimbalPitchDeg);
    }

    fclose(fp);
    return LZ_OK;
}

LzStatus LzBridge_ExportKmz(const LzRoute *route,
                            const LzTarget *pole,
                            const LzOrbitProfile *profile,
                            const char *outPath)
{
    if (route == NULL || pole == NULL || profile == NULL || outPath == NULL) {
        return LZ_ERR_PARAM;
    }

    LzWpmlFiles files;
    files.identity = LzWpml_DefaultIdentity();
    files.templateKml = NULL;
    files.waylinesWpml = NULL;

    LzStatus st = LzWpml_Build(route, pole, profile, &files);
    if (st != LZ_OK) {
        return st;
    }

    const LzKmzEntry entries[] = {
        { "wpmz/template.kml",  files.templateKml,  strlen(files.templateKml)  },
        { "wpmz/waylines.wpml", files.waylinesWpml, strlen(files.waylinesWpml) },
    };

    st = LzKmz_Write(entries, 2, outPath);

    LzWpml_Free(&files);
    return st;
}
