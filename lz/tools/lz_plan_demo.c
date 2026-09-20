/**
 * @file lz_plan_demo.c
 * @brief 桌面演示：用写死的杆位置跑一遍绕飞规划并打印航线。
 *
 * 这是"lz_core 不依赖任何东西"这句话的证明 —— 它能在没装 OpenCV、
 * 没接飞机的机器上跑出真实航线，因此规划逻辑可以在上机之前反复验证。
 *
 * 构建：cmake -S lz -B build && cmake --build build
 * 运行：./build/lz_plan_demo
 */

#include "lz_plan.h"
#include "lz_geo.h"

#include <stdio.h>

int main(void)
{
    /* 起飞点 */
    LzGeo takeoff = { .latitudeDeg = 30.5, .longitudeDeg = 114.3, .altitudeM = 30.0 };

    /* 假装视觉+定位已经算出杆的位置 */
    LzTarget pole = {
        .id = 1,
        .geo = { .latitudeDeg = 30.500500, .longitudeDeg = 114.300300, .altitudeM = 30.0 },
        .heightM = 15.0,
        .radiusM = 0.1,
        .pixel = { .u = 0.5, .v = 0.6, .topV = 0.2, .bottomV = 0.6 },
        .confidence = 0.92,
    };

    LzOrbitProfile profile = {
        .radiusM = 20.0,
        .altitudeM = 12.0,
        .speedMs = 3.0,
        .waypointCount = 8,
        .startBearingDeg = 0.0,
        .clockwise = true,
        .gimbalPitchDeg = -15.0,
    };

    LzRoute route;
    LzRoute_Init(&route);

    LzStatus st = LzPlan_BuildOrbit(&pole, &takeoff, &profile, &route);
    printf("绕飞规划: %s\n", LzStatus_Str(st));

    if (st == LZ_OK) {
        printf("圆周: 半径 %.0fm，高度 %.0fm，%d 个航点，%s\n",
               profile.radiusM, profile.altitudeM, profile.waypointCount,
               profile.clockwise ? "顺时针" : "逆时针");
        for (size_t i = 0; i < route.count; ++i) {
            const LzWaypoint *wp = &route.points[i];
            printf("  #%zu  %.7f, %.7f  高 %.1fm  速 %.1fm/s  云台(%.1f, %.1f)\n",
                   i, wp->geo.latitudeDeg, wp->geo.longitudeDeg,
                   wp->relativeAltM, wp->speedMs,
                   wp->gimbalYawDeg, wp->gimbalPitchDeg);
        }
    }

    LzRoute_Free(&route);
    return (st == LZ_OK) ? 0 : 1;
}
