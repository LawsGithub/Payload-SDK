/**
 * @file wt_runner.c
 * @brief 运行器：规划与校验部分（不依赖 PSDK）
 *
 * 执行部分（上传 KMZ、启动任务、云台闭环）在 wt_runner_psdk.c。
 * 这条分界线就是"能否在 PC 上验证"的界线：规划与安全校验是出事代价最大
 * 的环节，它们必须能在桌面上反复跑；飞行控制类调用只能在机上验证。
 */

#include "wt_runner.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

/**
 * @brief 递归创建目录（仅支持绝对路径，层级不深）
 *
 * 这是 wt_core 里唯一碰文件系统的函数，动机很具体：配置模板必须在**目标
 * 目录还不存在时**也能落下来。妙算3 是全新设备时 /data/wt_inspection/ 并不
 * 存在，而 fopen 不会替你建目录 —— 那样「首次运行自动生成模板」就成了空话，
 * 操作员只看到"配置文件不存在"，却拿不到可以填的模板。
 *
 * 纯 POSIX（mkdir/stat），不依赖 PSDK，所以留在 wt_core 里，
 * 这条路径也就能在 PC 上用回归测试覆盖。
 */
bool WtRunner_MakeDirs(const char *path)
{
    char tmp[WT_CONFIG_MAX_PATH];
    size_t len;
    size_t i;

    len = strlen(path);
    if (len == 0 || len >= sizeof(tmp)) {
        return false;
    }
    strncpy(tmp, path, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';

    for (i = 1; i < len; i++) {
        if (tmp[i] == '/') {
            tmp[i] = '\0';
            if (mkdir(tmp, 0755) != 0) {
                /* 目录已存在属正常情况，无需报错 */
                struct stat st;

                if (stat(tmp, &st) != 0 || !S_ISDIR(st.st_mode)) {
                    tmp[i] = '/';
                    return false;
                }
            }
            tmp[i] = '/';
        }
    }

    if (mkdir(tmp, 0755) != 0) {
        struct stat st;

        if (stat(tmp, &st) != 0 || !S_ISDIR(st.st_mode)) {
            return false;
        }
    }

    return true;
}

/**
 * @brief 取路径的目录部分（去掉最后一个 '/' 之后的内容）
 *
 * @return true 表示 out 已写入父目录；路径里没有 '/' 时返回 false
 */
bool WtRunner_DirName(const char *path, char *out, size_t outLen)
{
    const char *slash;
    size_t n;

    if (path == NULL || out == NULL || outLen == 0) {
        return false;
    }

    slash = strrchr(path, '/');
    if (slash == NULL) {
        return false;
    }
    if (slash == path) {
        snprintf(out, outLen, "/");
        return true;
    }

    n = (size_t)(slash - path);
    if (n >= outLen) {
        n = outLen - 1;
    }
    memcpy(out, path, n);
    out[n] = '\0';

    return true;
}

WtRunResult WtRunner_Init(WtRunner *runner, const char *configPath)
{
    memset(runner, 0, sizeof(*runner));
    WtMission_Init(&runner->job.mission);
    WtBridge_InitPlan(&runner->job.plan);

    if (!WtAppConfig_Load(configPath, &runner->config)) {
        fprintf(stderr, "[runner] 配置载入失败：%s\n", configPath);
        return WT_RUN_ERR_CONFIG;
    }

    WtAppConfig_Dump(&runner->config);

    if (!WtRunner_MakeDirs(runner->config.outputDir)) {
        fprintf(stderr, "[runner] 无法创建输出目录：%s\n", runner->config.outputDir);
        return WT_RUN_ERR_CONFIG;
    }

    return WT_RUN_OK;
}

void WtRunner_DeInit(WtRunner *runner)
{
    WtMission_Free(&runner->job.mission);
    WtBridge_FreePlan(&runner->job.plan);
    WtAppConfig_Free(&runner->config);
}

/**
 * @brief 判断停用角是否可信
 *
 * 没有停用角时，规划器只能假设叶片停在某个位置。这个假设一旦不成立，
 * 贴近叶片的航线就会直接撞上叶片。因此这里必须区分"已知"与"未知"，
 * 并把未知状态显式带到安全校验的结论里。
 */
static bool WtRunner_ParkPhaseKnown(double parkPhaseDeg)
{
    return parkPhaseDeg >= 0.0 && parkPhaseDeg < 360.0;
}

WtRunResult WtRunner_PlanTurbine(WtRunner *runner, const char *turbineName,
                                 double parkPhaseDeg)
{
    WtTurbineJob *job = &runner->job;
    WtPlanResult prc;
    char buf[512];
    bool phaseKnown = WtRunner_ParkPhaseKnown(parkPhaseDeg);

    memset(&job->mission, 0, sizeof(job->mission));
    WtMission_Init(&job->mission);
    WtBridge_FreePlan(&job->plan);
    WtBridge_InitPlan(&job->plan);

    job->turbine = WtAppConfig_FindTurbine(&runner->config, turbineName);
    if (job->turbine == NULL) {
        fprintf(stderr, "[runner] 配置中找不到风机 %s\n", turbineName);
        return WT_RUN_ERR_CONFIG;
    }

    /*
     * 相位未知时的默认假设：叶片停在 12 点方向（一片朝上）。
     * 这是最常见的停用角，也便于视觉反解时定位。但它是**假设**，
     * 下面会把风险写进报告并要求操作员确认。
     */
    job->parkPhaseDeg = phaseKnown ? Wt_Wrap360(parkPhaseDeg) : 0.0;
    job->parkPhaseKnown = phaseKnown;

    printf("\n[runner] 规划风机 %s\n", job->turbine->name);
    if (!phaseKnown) {
        printf("[runner] 警告：未提供停用角，按「叶片停在 12 点方向」假设规划。\n"
               "[runner]       执行前必须用视觉反解或现场目视确认该假设成立。\n");
    } else {
        printf("[runner] 停用角 %.1f°\n", job->parkPhaseDeg);
    }

    job->frame = WtTurbine_BuildRotorFrame(&job->turbine->spec, job->parkPhaseDeg);

    prc = WtPlan_BuildMission(&job->turbine->spec, &job->frame, &runner->config.profile,
                              &job->mission);
    if (prc != WT_PLAN_OK) {
        fprintf(stderr, "[runner] 航线规划失败，错误码 %d\n", (int)prc);
        return WT_RUN_ERR_PLAN;
    }

    job->safety = WtPlan_ValidateMission(&job->turbine->spec, &job->frame,
                                         &runner->config.profile, &job->mission);

    WtSafetyReport_Format(&job->safety, buf, sizeof(buf));
    printf("[runner] %s", buf);

    /*
     * 安全校验未通过时绝不继续。这里不是"打个警告"的场景：被拒绝的航线
     * 意味着存在与叶片相撞或分辨率不达标的实际风险，必须回到参数层面解决。
     */
    if (!job->safety.ok) {
        fprintf(stderr, "[runner] 安全校验未通过，航线拒绝下发。\n");
        return WT_RUN_ERR_SAFETY;
    }

    /* 未提供停用角时，即使几何校验通过也不能自动起飞 */
    if (!phaseKnown && runner->config.autoStart) {
        fprintf(stderr, "[runner] 停用角未知，禁止自动启动；请人工确认后手动下发。\n");
        return WT_RUN_ERR_SAFETY;
    }

    /* 生成动作与飞行参数 */
    {
        WtPlanResult brc;

        brc = WtBridge_BuildActions(&job->mission, WT_GIMBAL_FREE_YAW, &job->plan);
        if (brc == WT_PLAN_OK) {
            brc = WtBridge_BuildFlightParams(&job->mission, &runner->config.takeoff,
                                             &runner->config.profile, &job->plan);
        }
        if (brc != WT_PLAN_OK) {
            fprintf(stderr, "[runner] 动作生成失败，错误码 %d\n", (int)brc);
            return WT_RUN_ERR_PLAN;
        }
    }

    /*
     * 路径长度显式限位。目录与风机名都来自配置文件，超长时 snprintf 会
     * 静默截断，得到一个"看起来正常却指向别处"的文件名 —— 这类错误在
     * 现场极难发现，所以宁可在这里把两侧宽度都卡死。
     */
    snprintf(job->kmzPath, sizeof(job->kmzPath), "%.160s/%.56s_wayline.kmz",
             runner->config.outputDir, job->turbine->name);
    snprintf(job->csvPath, sizeof(job->csvPath), "%.160s/%.56s_actions.csv",
             runner->config.outputDir, job->turbine->name);
    snprintf(job->waypointsCsvPath, sizeof(job->waypointsCsvPath),
             "%.160s/%.56s_waypoints.csv", runner->config.outputDir, job->turbine->name);
    snprintf(job->reportPath, sizeof(job->reportPath), "%.160s/%.56s_report.txt",
             runner->config.outputDir, job->turbine->name);

    if (WtBridge_ExportKmz(&job->mission, &job->plan, &runner->config.profile,
                           &runner->config.takeoff, job->kmzPath) != WT_PLAN_OK) {
        fprintf(stderr, "[runner] KMZ 导出失败\n");
        return WT_RUN_ERR_PLAN;
    }
    WtBridge_ExportActionCsv(&job->mission, &job->plan, job->csvPath);

    /*
     * 两份 CSV 用途不同，都要出：
     *   *_waypoints.csv —— 航点外参（lat/lon/alt、云台角、GSD、blade_index、
     *                      radial_frac），是缺陷三维定位的唯一依据，见方案文档 §9；
     *   *_actions.csv   —— 动作与飞行参数（相对高度、速度、快门动作），
     *                      供人工核对 KMZ 里下发的内容。
     * 只出后者的话，AI 检测侧拿不到反投影需要的外参。
     */
    if (WtMission_ExportCsv(&job->mission, job->waypointsCsvPath) != WT_PLAN_OK) {
        fprintf(stderr, "[runner] 航点外参 CSV 导出失败：%s\n", job->waypointsCsvPath);
        return WT_RUN_ERR_PLAN;
    }

    printf("[runner] 航线文件已生成：\n  %s\n  %s\n  %s\n", job->kmzPath,
           job->csvPath, job->waypointsCsvPath);
    printf("[runner] 航点 %zu 个（缺陷用图 %d 张，粗模用图 %d 张），"
           "预计耗时 %.1f 分钟\n",
           job->mission.count, job->mission.defectPhotoCount,
           job->mission.modelPhotoCount, job->mission.durationSec / 60.0);

    return WT_RUN_OK;
}

bool WtRunner_WriteReport(const WtRunner *runner)
{
    const WtTurbineJob *job = &runner->job;
    FILE *fp;
    char buf[512];

    if (job->turbine == NULL) {
        return false;
    }

    fp = fopen(job->reportPath, "w");
    if (fp == NULL) {
        return false;
    }

    fprintf(fp, "风机叶片无人机巡检作业报告\n");
    fprintf(fp, "========================================\n\n");
    fprintf(fp, "风机编号      : %s\n", job->turbine->name);
    fprintf(fp, "塔基坐标      : %.7f, %.7f, %.1f m\n",
            job->turbine->spec.base.lat, job->turbine->spec.base.lon,
            job->turbine->spec.base.alt);
    fprintf(fp, "轮毂高度      : %.1f m\n", job->turbine->spec.hubHeight);
    fprintf(fp, "叶轮直径      : %.1f m\n", job->turbine->spec.rotorDiameter);
    fprintf(fp, "机舱朝向      : %.1f°\n", job->turbine->spec.headingDeg);
    fprintf(fp, "停用角        : %.1f°\n", job->parkPhaseDeg);
    fprintf(fp, "\n");

    fprintf(fp, "相机          : %s\n", runner->config.profile.camera.name);
    fprintf(fp, "目标 GSD      : %.2f mm/px\n", runner->config.profile.targetGsdMmPerPx);
    fprintf(fp, "巡检面        : %d\n", (int)runner->config.profile.bladeSide);
    fprintf(fp, "斜视角        : %.1f°\n",
            WtPlan_BladeSkewAngleDeg(&runner->config.profile));
    fprintf(fp, "风轮可转动    : %s\n\n",
            runner->config.profile.rotorMayRotate ? "是" : "否");

    fprintf(fp, "航点总数      : %zu\n", job->mission.count);
    fprintf(fp, "缺陷检测用图  : %d 张\n", job->mission.defectPhotoCount);
    fprintf(fp, "粗模重建用图  : %d 张\n", job->mission.modelPhotoCount);
    fprintf(fp, "折线总长      : %.1f m\n", job->mission.pathLengthM);
    fprintf(fp, "预计耗时      : %.1f s\n\n", job->mission.durationSec);

    WtSafetyReport_Format(&job->safety, buf, sizeof(buf));
    fprintf(fp, "安全与成像校验\n%s\n", buf);

    /*
     * 只有在停用角确实未知时才写这条备注。
     * 不能用 parkPhaseDeg == 0 去判断 —— 0° 本身就是一个完全合法的实测值
     * （叶片确实停在 12 点方向），把它当成"假设"会让报告误导操作员。
     */
    if (!job->parkPhaseKnown) {
        fprintf(fp, "备注：停用角依据为「叶片停在 12 点方向」假设，"
                    "需现场确认后方可执行。\n");
    }

    fclose(fp);

    return true;
}