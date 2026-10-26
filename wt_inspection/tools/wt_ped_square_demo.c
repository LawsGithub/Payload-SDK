/**
 * @file wt_ped_square_demo.c
 * @brief 绕行人正方形航线的 PC 侧演示：规划 + 导出 KMZ + 打印航点表
 *
 * 不依赖 PSDK。KMZ 与机载端走的是**同一份代码**（WtPedSquare_Build +
 * WtBridge_ExportPedSquareKmz），所以这里导出的文件就是飞机会收到的文件 ——
 * 核对页（网页）也是解析它画出来的，而不是另写一套公式。
 *
 * 用法：
 *     ./wt_ped_square_demo <基点纬度> <基点经度> <飞机纬度> <飞机经度> [out.kmz]
 *     ./wt_ped_square_demo                    # 用内置样例坐标
 */

#include <stdio.h>
#include <stdlib.h>

#include "wt_bridge.h"
#include "wt_geometry.h"
#include "wt_ped_square.h"

int main(int argc, char **argv)
{
    WtPedSquareConfig cfg = WtPedSquare_DefaultConfig();
    WtPedSquarePlan plan;
    E_WtPedSquareResult r;
    /* 样例：基点取测试夹具里的示例坐标，飞机在它西南方约 35 m */
    WtGeo base = {41.5236000, 111.7461000, 0.0};
    WtGeo aircraft = {41.5233800, 111.7458000, 0.0};
    const char *kmzPath = NULL;
    int i;

    if (argc >= 5) {
        base.lat = atof(argv[1]);
        base.lon = atof(argv[2]);
        aircraft.lat = atof(argv[3]);
        aircraft.lon = atof(argv[4]);
        if (argc >= 6) {
            kmzPath = argv[5];
        }
    } else if (argc == 2) {
        kmzPath = argv[1];
    } else if (argc != 1) {
        fprintf(stderr, "用法: %s <基点纬度> <基点经度> <飞机纬度> <飞机经度> [out.kmz]\n", argv[0]);
        return 2;
    }

    r = WtPedSquare_Build(&cfg, &base, &aircraft, &plan);
    if (r != WT_PED_SQUARE_OK) {
        fprintf(stderr, "规划失败：%s（飞机到基点 %.1f m）\n", WtPedSquare_ResultText(r),
                plan.aircraftToBaseM);
        return 1;
    }

    printf("== 绕行人正方形航线 ==\n");
    printf("基点 %.8f, %.8f；边长 %.1f m；低 %.1f m / 高 %.1f m（相对起飞点）；速度 %.1f m/s\n",
           base.lat, base.lon, cfg.sideM, cfg.lowHeightM, cfg.highHeightM, cfg.speedMs);
    printf("飞机到基点 %.1f m，到首个航点 %.1f m；航线长 %.1f m，耗时下界 %.0f s；返航高度 %.0f m\n",
           plan.aircraftToBaseM, plan.transitM, plan.pathLengthM, plan.durationS, plan.rthHeightM);
    printf("顶点方位（相对基点，按飞行顺序）：%.0f° → %.0f° → %.0f° → %.0f°\n",
           plan.cornerBearingDeg[0], plan.cornerBearingDeg[1], plan.cornerBearingDeg[2],
           plan.cornerBearingDeg[3]);
    printf("%4s %-4s %4s %13s %13s %8s %8s %8s %10s\n", "idx", "动作", "顶点", "lat", "lon",
           "东(m)", "北(m)", "高(m)", "朝基点°");
    for (i = 0; i < plan.count; i++) {
        const WtPedSquareWaypoint *w = &plan.wp[i];

        printf("%4d %-4s   V%d %13.8f %13.8f %8.2f %8.2f %8.2f %10.1f\n", i,
               WtPedSquare_StepName(w->step), w->corner + 1, w->geo.lat, w->geo.lon, w->enu.e,
               w->enu.n, w->relHeightM, w->headingToBaseDeg);
    }

    if (kmzPath != NULL) {
        if (WtBridge_ExportPedSquareKmz(&plan, &cfg, kmzPath) != WT_PLAN_OK) {
            fprintf(stderr, "导出 KMZ 失败：%s\n", kmzPath);
            return 1;
        }
        printf("KMZ 已导出：%s\n", kmzPath);
    }

    return 0;
}
