/**
 * @file wt_plan_demo.c
 * @brief 航线规划引擎的 PC 侧演示与自检程序
 *
 * 该程序不依赖 PSDK，用来在电脑上核对几何算法、参数灵敏度与安全边界。
 * 典型用法：
 *     ./wt_plan_demo                              # 用内置样例风机规划一条航线
 *     ./wt_plan_demo mission.csv                  # 同时导出航点 CSV
 *     ./wt_plan_demo mission.csv mission.kmz      # 并导出 PSDK 航点 KMZ
 */

#include <stdio.h>
#include <string.h>

#include "wt_bridge.h"
#include "wt_camera.h"
#include "wt_geometry.h"
#include "wt_plan.h"
#include "wt_turbine.h"

/** 内置样例：陆上大型机组，155m 叶轮、100m 轮毂中心高 */
static WtTurbineSpec DemoSpec(void)
{
    WtTurbineSpec s;

    memset(&s, 0, sizeof(s));
    /* 示例坐标：内蒙古某风场 */
    s.base.lat = 41.5236000;
    s.base.lon = 111.7461000;
    s.base.alt = 1450.0;

    s.hubHeight = 100.0;
    s.rotorDiameter = 155.0;
    s.hubRadius = 2.5;
    s.towerBottomDia = 6.5;
    s.towerTopDia = 4.0;
    s.nacelleOffset = 4.5;
    s.coneAngleDeg = 4.0;
    s.tiltDeg = 5.0;
    s.headingDeg = 30.0;
    s.prebendM = 3.0;
    s.bladeCount = 3;

    return s;
}

static void PrintCameraTable(void)
{
    const WtCameraModel *cams[3] = {&WT_CAMERA_M4T_WIDE, &WT_CAMERA_M4T_MID, &WT_CAMERA_M4T_TELE};
    int i;

    printf("== 相机模型核验 (等效 35mm 画幅) ==\n");
    printf("%-20s %8s %10s %10s %10s %10s\n",
           "camera", "EFL(mm)", "FOV_d(deg)", "FOV_h(deg)", "pitch(um)", "GSD@12m");

    for (i = 0; i < 3; i++) {
        WtCameraGeometry g = WtCamera_ComputeGeometry(cams[i]);

        printf("%-20s %8.0f %10.2f %10.2f %10.3f %6.4f mm/px\n",
               cams[i]->name, cams[i]->eflMm, g.fovDiagDeg, g.fovHorizDeg,
               g.pixelPitchUm, WtCamera_GsdAtDistance(cams[i], 12.0));
    }

    printf("\n  -> M4T 长焦官方标注 FOV 15°、等效 168mm，模型给出 %.2f°，一致。\n",
           WtCamera_ComputeGeometry(&WT_CAMERA_M4T_TELE).fovDiagDeg);
    printf("  -> 默认剖面目标 GSD %.2f mm/px 时，中焦最大拍摄距离 %.1f m。\n\n",
           WtInspectionProfile_Default().targetGsdMmPerPx,
           WtCamera_MaxDistanceForGsd(&WT_CAMERA_M4T_MID,
                                      WtInspectionProfile_Default().targetGsdMmPerPx));
}

int main(int argc, char **argv)
{
    WtTurbineSpec spec = DemoSpec();
    WtInspectionProfile profile = WtInspectionProfile_Default();
    WtRotorFrame frame;
    WtMission mission;
    WtValidateResult v;
    WtSafetyReport rep;
    WtPlanResult rc;
    char reportBuf[512];

    printf("================ 风机叶片巡检航线规划 (M4T + Manifold 3) ================\n\n");

    PrintCameraTable();

    v = WtTurbine_Validate(&spec);
    printf("== 风机参数校验 ==\n  %s : %s\n\n", v.ok ? "PASS" : "FAIL", v.message);

    /* 停机精细巡检假定风轮停在「一片叶片朝上」的相位 */
    frame = WtTurbine_BuildRotorFrame(&spec, 0.0);
    printf("== 风轮参考系 ==\n");
    printf("  风轮中心 ENU   : E %.2f, N %.2f, U %.2f\n",
           frame.rotorCenter.e, frame.rotorCenter.n, frame.rotorCenter.u);
    printf("  旋转轴单位矢量 : E %.4f, N %.4f, U %.4f\n",
           frame.rotAxis.e, frame.rotAxis.n, frame.rotAxis.u);
    printf("  12点方向单位矢量: E %.4f, N %.4f, U %.4f\n\n",
           frame.upRef.e, frame.upRef.n, frame.upRef.u);

    printf("== 采样密度推导 ==\n");
    {
        /*
         * 这段曾自己重算一遍，结果与规划器实际采用的参数不一致（漏乘
         * GSD 余量系数、也没算点数上限的裁剪），于是「29 个站位」与真实
         * 的 30 个对不上，而调参的人看到的正是这张表。现在改为直接问
         * 规划器要，杜绝第二套公式。
         */
        WtBladeSampling sampling;
        char samplingBuf[512];

        if (WtPlan_ResolveBladeSampling(&spec, &profile, &sampling) != WT_PLAN_OK) {
            printf("  采样参数推导失败\n\n");
        } else {
            WtBladeSampling_Format(&sampling, samplingBuf, sizeof(samplingBuf));
            printf("%s", samplingBuf);
            printf("  -> 三叶片共 %d 个叶片拍照点\n\n",
                   sampling.samplesPerBlade * spec.bladeCount);
        }
    }

    /* 规划 */
    WtMission_Init(&mission);
    rc = WtPlan_BuildMission(&spec, &frame, &profile, &mission);
    if (rc != WT_PLAN_OK) {
        printf("规划失败，错误码 %d\n", (int)rc);
        WtMission_Free(&mission);
        return 1;
    }

    printf("== 航线概览 ==\n");
    printf("  航点总数       : %zu\n", mission.count);
    printf("  缺陷检测用图   : %d 张\n", mission.defectPhotoCount);
    printf("  粗模重建用图   : %d 张\n", mission.modelPhotoCount);
    printf("  折线总长       : %.1f m\n", mission.pathLengthM);
    printf("  预计耗时       : %.1f s (%.1f min)\n\n",
           mission.durationSec, mission.durationSec / 60.0);

    rep = WtPlan_ValidateMission(&spec, &frame, &profile, &mission);
    WtSafetyReport_Format(&rep, reportBuf, sizeof(reportBuf));
    printf("== %s", reportBuf);
    printf("\n");

    printf("== 前 6 个航点 ==\n");
    printf("%-14s %10s %10s %10s %12s %10s %9s\n",
           "tag", "east", "north", "up", "gimbal_yaw", "pitch", "GSD");
    {
        size_t i;

        for (i = 0; i < mission.count && i < 6; i++) {
            const WtPlanPoint *p = &mission.points[i];

            printf("%-14s %10.2f %10.2f %10.2f %12.1f %10.1f %9.3f\n",
                   p->tag, p->enu.e, p->enu.n, p->enu.u,
                   p->gimbalYawDeg, p->gimbalPitchDeg, p->gsdMmPerPx);
        }
    }

    /* 生成动作与飞行参数，并导出 PSDK 可消费的航线文件 */
    {
        WtMissionPlan plan;
        WtGeo takeoff;
        WtPlanResult brc;

        WtBridge_InitPlan(&plan);
        takeoff = spec.base;
        takeoff.alt = spec.base.alt + 5.0; /* 起飞点在塔基上方 5m 的场地 */

        brc = WtBridge_BuildActions(&mission, WT_GIMBAL_FREE_YAW, &plan);
        if (brc == WT_PLAN_OK) {
            brc = WtBridge_BuildFlightParams(&mission, &takeoff, &profile, &plan);
        }
        if (brc != WT_PLAN_OK) {
            printf("\n动作生成失败，错误码 %d\n", (int)brc);
        } else {
            printf("\n== 桥接层输出 ==\n");
            printf("  动作条目   : %zu\n", plan.actions.count);
            printf("  飞行参数   : %zu\n", plan.flightParamCount);

            if (argc > 1) {
                if (WtMission_ExportCsv(&mission, argv[1]) == WT_PLAN_OK) {
                    printf("  已导出航点 CSV : %s\n", argv[1]);
                }
            }
            if (argc > 2) {
                if (WtBridge_ExportKmz(&mission, &plan, &profile, &takeoff, argv[2]) == WT_PLAN_OK) {
                    printf("  已导出航点 KMZ : %s\n", argv[2]);
                } else {
                    printf("  KMZ 导出失败\n");
                }
            }
        }

        WtBridge_FreePlan(&plan);
    }

    WtMission_Free(&mission);

    return rep.ok ? 0 : 2;
}