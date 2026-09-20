/**
 * @file lz_kmz_demo.c
 * @brief 生成一份 KMZ 供外部工具检验（unzip / Python zipfile）。
 *
 * 手写 zip 最容易错的是中央目录的偏移与 CRC。这个小工具把生成的包落盘，
 * 好让我们用**独立的**解包器验证它 —— 自己写的包自己解，什么都证明不了。
 *
 *   ./build/lz_kmz_demo /tmp/out.kmz
 *   python3 -c "import zipfile;z=zipfile.ZipFile('/tmp/out.kmz');print(z.namelist());z.testzip()"
 */

#include "lz_bridge.h"
#include "lz_plan.h"
#include "lz_geo.h"

#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "用法: %s <输出.kmz>\n", argv[0]);
        return 2;
    }

    LzGeo takeoff = { .latitudeDeg = 30.5, .longitudeDeg = 114.3, .altitudeM = 30.0 };
    LzTarget pole = {
        .id = 1,
        .geo = { .latitudeDeg = 30.500500, .longitudeDeg = 114.300300, .altitudeM = 30.0 },
        .heightM = 15.0, .radiusM = 0.1,
        .pixel = { .u = 0.5, .v = 0.6, .topV = 0.2, .bottomV = 0.6 },
        .confidence = 0.92,
    };
    LzOrbitProfile profile = {
        .radiusM = 20.0, .altitudeM = 12.0, .speedMs = 3.0,
        .waypointCount = 8, .startBearingDeg = 0.0,
        .clockwise = true, .gimbalPitchDeg = -15.0,
    };
    (void)takeoff;

    /* 手动构造一个 8 点圆 —— 绕飞算法的 TODO 还没实现，
     * 但 KMZ 生成这条链路要独立于它先验证通。 */
    LzRoute route;
    LzRoute_Init(&route);
    route.points = calloc(profile.waypointCount, sizeof(LzWaypoint));
    if (route.points == NULL) {
        return 1;
    }
    route.capacity = (size_t)profile.waypointCount;
    for (int i = 0; i < profile.waypointCount; ++i) {
        double bearing = profile.startBearingDeg + i * (360.0 / profile.waypointCount);
        LzGeo p;
        /* 注意：这里从**杆**沿 bearing 外推，得到的是站点；
         * 云台要反过来指向杆心，方位角 = bearing + 180 */
        LzGeo_Destination(&pole.geo, bearing, profile.radiusM, &p);
        route.points[i] = (LzWaypoint){
            .geo = p,
            .relativeAltM = profile.altitudeM,
            .speedMs = profile.speedMs,
            .gimbalYawDeg = LzGeo_NormalizeDeg(bearing + 180.0),
            .gimbalPitchDeg = profile.gimbalPitchDeg,
        };
        route.count++;
    }

    LzStatus st = LzBridge_ExportKmz(&route, &pole, &profile, argv[1]);
    printf("生成 KMZ: %s (%s)\n", LzStatus_Str(st), argv[1]);
    if (st == LZ_OK) {
        printf("航点数 %zu，半径 %.0fm，高度 %.0fm\n",
               route.count, profile.radiusM, profile.altitudeM);
    }

    LzRoute_Free(&route);
    return (st == LZ_OK) ? 0 : 1;
}
