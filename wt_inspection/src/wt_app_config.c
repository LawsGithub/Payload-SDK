/**
 * @file wt_app_config.c
 * @brief 作业配置载入实现
 */

#include "wt_app_config.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wt_camera.h"

#define WT_CONFIG_MAX_LINE 512

static char *WtConfig_Trim(char *s)
{
    char *end;

    while (*s != '\0' && isspace((unsigned char)*s)) {
        s++;
    }
    end = s + strlen(s);
    while (end > s && isspace((unsigned char)*(end - 1))) {
        end--;
    }
    *end = '\0';

    return s;
}

/** 去掉行尾注释：以 '#' 或 ';' 起始的注释，且不在引号内 */
static void WtConfig_StripComment(char *line)
{
    char *p = line;
    bool inQuote = false;

    for (; *p != '\0'; p++) {
        if (*p == '"') {
            inQuote = !inQuote;
        } else if (!inQuote && (*p == '#' || *p == ';')) {
            *p = '\0';
            return;
        }
    }
}

/* ------------------------------------------------------------------ */
/* 键名表                                                              */
/*                                                                     */
/* 这三个段落的键集是本程序自己定义的，因此段内的键名必须逐个认识：     */
/* 拼错一个字母（min_safe_distt）如果只是被丢掉，操作员会以为安全距已经 */
/* 改成了 8m，而程序仍按 3m 飞 —— 这是最危险的一类静默失败。            */
/*                                                                     */
/* 表只用于「键名是否已知」的判定与拼写建议；取值仍由各自的解析分支负责， */
/* 两者由回归测试锁住一致性。                                          */
/* ------------------------------------------------------------------ */

static const char *const WT_KEYS_FIELD[] = {
    "takeoff_lat", "takeoff_lon", "takeoff_alt", NULL,
};

static const char *const WT_KEYS_INSPECTION[] = {
    "camera", "target_gsd", "blade_standoff", "tower_standoff", "min_safe_dist",
    "ground_clearance", "rotor_may_rotate", "blade_side", "overlap", "blade_spacing",
    "max_samples_per_blade", "blade_axial_shift", "tower_ring_count", "tower_per_ring",
    "include_tower", "do_coarse_survey", "coarse_radius", "coarse_photo_count",
    "coarse_pitch", "cruise_speed", "inspect_speed", "photo_dwell",
    /* 沿中线飞行模式（trace）：与上面的站位式采样互斥，见 WtTraceProfile */
    "trace_enable", "trace_offset", "trace_step", "trace_approach",
    "trace_arc_segments", NULL,
};

static const char *const WT_KEYS_APP[] = {
    "telemetry_hz", "gimbal_track_gain", "track_deadband", "auto_start", "output_dir",
    "dry_run", "no_actions", "ai_overlay", "stream_probe",
    "survey_enabled", "survey_engine", "survey_infer_interval_ms", "geoid_offset_m",
    "survey_selftest",
    /* 高空行人瞄准打点 */
    "ped_enabled", "ped_engine", "ped_dwell_ms", "ped_conf", NULL,
};

static const char *const WT_KEYS_TURBINE[] = {
    "name", "lat", "lon", "alt", "hub_height", "rotor_diameter", "hub_radius",
    "tower_bottom_dia", "tower_top_dia", "nacelle_offset", "cone_angle", "tilt",
    "heading", "prebend", "blade_count", "park_phase", NULL,
};

const char *const *WtAppConfig_KnownKeys(WtConfigSection section, size_t *outCount)
{
    const char *const *table;

    switch (section) {
    case WT_CFG_SECTION_FIELD:
        table = WT_KEYS_FIELD;
        break;
    case WT_CFG_SECTION_INSPECTION:
        table = WT_KEYS_INSPECTION;
        break;
    case WT_CFG_SECTION_APP:
        table = WT_KEYS_APP;
        break;
    case WT_CFG_SECTION_TURBINE:
        table = WT_KEYS_TURBINE;
        break;
    default:
        if (outCount != NULL) {
            *outCount = 0;
        }
        return NULL;
    }

    if (outCount != NULL) {
        size_t n = 0;

        while (table[n] != NULL) {
            n++;
        }
        *outCount = n;
    }

    return table;
}

static bool WtConfig_KeyKnown(WtConfigSection section, const char *key)
{
    const char *const *table = WtAppConfig_KnownKeys(section, NULL);
    size_t i;

    if (table == NULL) {
        return true; /* 非本程序定义的段落，不做键名判定 */
    }

    for (i = 0; table[i] != NULL; i++) {
        if (strcmp(table[i], key) == 0) {
            return true;
        }
    }

    return false;
}

/** 有界编辑距离：超过 maxDist 立即返回 maxDist+1，避免为长键名做无谓计算 */
static int WtConfig_EditDistance(const char *a, const char *b, int maxDist)
{
    int la = (int)strlen(a);
    int lb = (int)strlen(b);
    int prev[64];
    int cur[64];
    int i;
    int j;

    if (la >= 64 || lb >= 64) {
        return maxDist + 1;
    }
    if (la - lb > maxDist || lb - la > maxDist) {
        return maxDist + 1;
    }

    for (j = 0; j <= lb; j++) {
        prev[j] = j;
    }

    for (i = 1; i <= la; i++) {
        int rowMin;

        cur[0] = i;
        rowMin = cur[0];
        for (j = 1; j <= lb; j++) {
            int cost = (a[i - 1] == b[j - 1]) ? 0 : 1;
            int v = prev[j - 1] + cost;

            if (prev[j] + 1 < v) {
                v = prev[j] + 1;
            }
            if (cur[j - 1] + 1 < v) {
                v = cur[j - 1] + 1;
            }
            cur[j] = v;
            if (v < rowMin) {
                rowMin = v;
            }
        }
        if (rowMin > maxDist) {
            return maxDist + 1;
        }
        memcpy(prev, cur, sizeof(int) * (size_t)(lb + 1));
    }

    return prev[lb];
}

const char *WtAppConfig_SuggestKey(WtConfigSection section, const char *key)
{
    const char *const *table = WtAppConfig_KnownKeys(section, NULL);
    const char *best = NULL;
    int bestDist = 3; /* 允许的最大编辑距离 + 1 */
    size_t i;

    if (table == NULL || key == NULL) {
        return NULL;
    }

    for (i = 0; table[i] != NULL; i++) {
        int d = WtConfig_EditDistance(key, table[i], 2);

        if (d < bestDist) {
            bestDist = d;
            best = table[i];
        }
    }

    return best;
}

static bool WtConfig_GetDouble(const char *value, double *out)
{
    char *end = NULL;
    double v = strtod(value, &end);

    if (end == value) {
        return false;
    }
    *out = v;

    return true;
}

static bool WtConfig_GetInt(const char *value, int *out)
{
    char *end = NULL;
    long v = strtol(value, &end, 10);

    if (end == value) {
        return false;
    }
    *out = (int)v;

    return true;
}

static bool WtConfig_GetBool(const char *value, bool *out)
{
    if (strcmp(value, "1") == 0 || strcasecmp(value, "true") == 0 ||
        strcasecmp(value, "on") == 0 || strcasecmp(value, "yes") == 0) {
        *out = true;
        return true;
    }
    if (strcmp(value, "0") == 0 || strcasecmp(value, "false") == 0 ||
        strcasecmp(value, "off") == 0 || strcasecmp(value, "no") == 0) {
        *out = false;
        return true;
    }

    return false;
}

/** 按名称选择相机型号 */
static bool WtConfig_GetCamera(const char *value, WtCameraModel *out)
{
    if (strcasecmp(value, "wide") == 0 || strcasecmp(value, "m4t_wide") == 0) {
        *out = WT_CAMERA_M4T_WIDE;
        return true;
    }
    if (strcasecmp(value, "mid") == 0 || strcasecmp(value, "m4t_mid") == 0) {
        *out = WT_CAMERA_M4T_MID;
        return true;
    }
    if (strcasecmp(value, "tele") == 0 || strcasecmp(value, "m4t_tele") == 0) {
        *out = WT_CAMERA_M4T_TELE;
        return true;
    }

    return false;
}

/** 解析 [turbine.xxx] 段落里的键值 */
static bool WtConfig_ApplyTurbineKey(WtTurbineEntry *entry, const char *key, const char *value)
{
    WtTurbineSpec *s = &entry->spec;
    int i;

    if (strcmp(key, "name") == 0) {
        snprintf(entry->name, sizeof(entry->name), "%.63s", value);
        return true;
    }
    if (strcmp(key, "lat") == 0) {
        return WtConfig_GetDouble(value, &s->base.lat);
    }
    if (strcmp(key, "lon") == 0) {
        return WtConfig_GetDouble(value, &s->base.lon);
    }
    if (strcmp(key, "alt") == 0) {
        return WtConfig_GetDouble(value, &s->base.alt);
    }
    if (strcmp(key, "hub_height") == 0) {
        return WtConfig_GetDouble(value, &s->hubHeight);
    }
    if (strcmp(key, "rotor_diameter") == 0) {
        return WtConfig_GetDouble(value, &s->rotorDiameter);
    }
    if (strcmp(key, "hub_radius") == 0) {
        return WtConfig_GetDouble(value, &s->hubRadius);
    }
    if (strcmp(key, "tower_bottom_dia") == 0) {
        return WtConfig_GetDouble(value, &s->towerBottomDia);
    }
    if (strcmp(key, "tower_top_dia") == 0) {
        return WtConfig_GetDouble(value, &s->towerTopDia);
    }
    if (strcmp(key, "nacelle_offset") == 0) {
        return WtConfig_GetDouble(value, &s->nacelleOffset);
    }
    if (strcmp(key, "cone_angle") == 0) {
        return WtConfig_GetDouble(value, &s->coneAngleDeg);
    }
    if (strcmp(key, "tilt") == 0) {
        return WtConfig_GetDouble(value, &s->tiltDeg);
    }
    if (strcmp(key, "heading") == 0) {
        return WtConfig_GetDouble(value, &s->headingDeg);
    }
    if (strcmp(key, "prebend") == 0) {
        return WtConfig_GetDouble(value, &s->prebendM);
    }
    if (strcmp(key, "blade_count") == 0) {
        if (!WtConfig_GetInt(value, &i)) {
            return false;
        }
        s->bladeCount = i;
        return true;
    }
    if (strcmp(key, "park_phase") == 0) {
        /* 停用角。负数（或未配置）表示"未知"，由运行器判风险并禁止自动启动 */
        return WtConfig_GetDouble(value, &entry->parkPhaseDeg);
    }

    /*
     * 走到这里说明键名不在本段落的键表里。取值本身没有问题（否则上面早就
     * 返回 false 了），但「值合法」恰恰是这类错误最危险的地方：操作员以为
     * 改动了参数，程序却什么也没做。调用方会用 WtAppConfig_KnownKeys 再判
     * 一次并报错，这里保持返回值语义单纯（true = 取值没有问题）。
     */
    return true;
}

static bool WtConfig_ApplyProfileKey(WtInspectionProfile *p, const char *key, const char *value)
{
    int i;
    bool b;

    if (strcmp(key, "camera") == 0) {
        return WtConfig_GetCamera(value, &p->camera);
    }
    if (strcmp(key, "target_gsd") == 0) {
        return WtConfig_GetDouble(value, &p->targetGsdMmPerPx);
    }
    if (strcmp(key, "blade_standoff") == 0) {
        return WtConfig_GetDouble(value, &p->bladeStandoffM);
    }
    if (strcmp(key, "tower_standoff") == 0) {
        return WtConfig_GetDouble(value, &p->towerStandoffM);
    }
    if (strcmp(key, "min_safe_dist") == 0) {
        return WtConfig_GetDouble(value, &p->minSafeDistM);
    }
    if (strcmp(key, "ground_clearance") == 0) {
        return WtConfig_GetDouble(value, &p->groundClearanceM);
    }
    if (strcmp(key, "rotor_may_rotate") == 0) {
        if (!WtConfig_GetBool(value, &b)) {
            return false;
        }
        p->rotorMayRotate = b;
        return true;
    }
    if (strcmp(key, "blade_side") == 0) {
        if (strcasecmp(value, "upstream") == 0) {
            p->bladeSide = WT_BLADE_SIDE_UPSTREAM;
        } else if (strcasecmp(value, "downstream") == 0) {
            p->bladeSide = WT_BLADE_SIDE_DOWNSTREAM;
        } else if (strcasecmp(value, "both") == 0) {
            p->bladeSide = WT_BLADE_SIDE_BOTH;
        } else {
            return false;
        }
        return true;
    }
    if (strcmp(key, "overlap") == 0) {
        return WtConfig_GetDouble(value, &p->overlapPct);
    }
    if (strcmp(key, "blade_spacing") == 0) {
        return WtConfig_GetDouble(value, &p->bladeSpacingM);
    }
    if (strcmp(key, "max_samples_per_blade") == 0) {
        if (!WtConfig_GetInt(value, &i)) {
            return false;
        }
        p->maxSamplesPerBlade = i;
        return true;
    }
    if (strcmp(key, "blade_axial_shift") == 0) {
        return WtConfig_GetDouble(value, &p->bladeAxialShiftM);
    }
    if (strcmp(key, "tower_ring_count") == 0) {
        if (!WtConfig_GetInt(value, &i)) {
            return false;
        }
        p->towerRingCount = i;
        return true;
    }
    if (strcmp(key, "tower_per_ring") == 0) {
        if (!WtConfig_GetInt(value, &i)) {
            return false;
        }
        p->towerPerRingCount = i;
        return true;
    }
    if (strcmp(key, "include_tower") == 0) {
        if (!WtConfig_GetBool(value, &b)) {
            return false;
        }
        p->includeTower = b;
        return true;
    }
    if (strcmp(key, "do_coarse_survey") == 0) {
        if (!WtConfig_GetBool(value, &b)) {
            return false;
        }
        p->doCoarseSurvey = b;
        return true;
    }
    if (strcmp(key, "coarse_radius") == 0) {
        return WtConfig_GetDouble(value, &p->coarseRadiusM);
    }
    if (strcmp(key, "coarse_photo_count") == 0) {
        if (!WtConfig_GetInt(value, &i)) {
            return false;
        }
        p->coarsePhotoCount = i;
        return true;
    }
    if (strcmp(key, "coarse_pitch") == 0) {
        return WtConfig_GetDouble(value, &p->coarsePitchDeg);
    }
    if (strcmp(key, "cruise_speed") == 0) {
        return WtConfig_GetDouble(value, &p->cruiseSpeedMs);
    }
    if (strcmp(key, "inspect_speed") == 0) {
        return WtConfig_GetDouble(value, &p->inspectSpeedMs);
    }
    if (strcmp(key, "photo_dwell") == 0) {
        return WtConfig_GetDouble(value, &p->photoDwellSec);
    }

    /* 见 WtConfig_ApplyTurbineKey 末尾：键名的合法性由调用方按键表判定 */
    return true;
}

/** 解析 [inspection] 段里的「沿中线飞行」模式键；未命中返回 false */
static bool WtConfig_ApplyTraceKey(WtAppConfig *config, const char *key, const char *value)
{
    WtTraceProfile *t = &config->trace;

    if (strcmp(key, "trace_enable") == 0) {
        return WtConfig_GetBool(value, &config->traceEnabled);
    }
    if (strcmp(key, "trace_offset") == 0) {
        return WtConfig_GetDouble(value, &t->offsetM);
    }
    if (strcmp(key, "trace_step") == 0) {
        return WtConfig_GetDouble(value, &t->stepM);
    }
    if (strcmp(key, "trace_approach") == 0) {
        return WtConfig_GetDouble(value, &t->approachM);
    }
    if (strcmp(key, "trace_arc_segments") == 0) {
        return WtConfig_GetInt(value, &t->arcSegments);
    }

    return false;
}

bool WtAppConfig_Load(const char *path, WtAppConfig *config)
{
    FILE *fp;
    char line[WT_CONFIG_MAX_LINE];
    char section[128] = "";
    WtTurbineEntry *curTurbine = NULL;
    int capacity = 4;
    int lineNo = 0;
    bool ok = true;
    /*
     * 「写了没有」单独记，不用哨兵值：哨兵会把 ped_conf = -0.5 这种写错的值
     * 当成"没写"、静默换成默认 —— 正是本文件要防的那类错误。
     */
    bool pedDwellSet = false;
    bool pedConfSet = false;

    memset(config, 0, sizeof(*config));
    config->profile = WtInspectionProfile_Default();
    config->trace = WtTraceProfile_Default();
    config->traceEnabled = false;

    config->turbines = calloc((size_t)capacity, sizeof(WtTurbineEntry));
    if (config->turbines == NULL) {
        return false;
    }

    fp = fopen(path, "r");
    if (fp == NULL) {
        fprintf(stderr, "[config] 无法打开 %s\n", path);
        free(config->turbines);
        config->turbines = NULL;
        return false;
    }

    while (fgets(line, sizeof(line), fp) != NULL) {
        char *s;
        char *eq;

        lineNo++;
        WtConfig_StripComment(line);
        s = WtConfig_Trim(line);
        if (*s == '\0') {
            continue;
        }

        /* 段落头 [xxx] */
        if (*s == '[') {
            char *end = strchr(s, ']');

            if (end == NULL) {
                fprintf(stderr, "[config] 第 %d 行：段落头缺少 ']'\n", lineNo);
                ok = false;
                continue;
            }
            *end = '\0';
            snprintf(section, sizeof(section), "%s", WtConfig_Trim(s + 1));

            curTurbine = NULL;
            if (strncmp(section, "turbine.", 8) == 0) {
                if (config->turbineCount >= capacity) {
                    WtTurbineEntry *nb;
                    int newCap = capacity * 2;

                    nb = realloc(config->turbines, (size_t)newCap * sizeof(WtTurbineEntry));
                    if (nb == NULL) {
                        ok = false;
                        break;
                    }
                    config->turbines = nb;
                    capacity = newCap;
                }
                curTurbine = &config->turbines[config->turbineCount];
                memset(curTurbine, 0, sizeof(*curTurbine));
                curTurbine->parkPhaseDeg = -1.0; /* 未提供 */

                /* 段落名即风机编号，缺省可用 */
                snprintf(curTurbine->name, sizeof(curTurbine->name), "%.63s", section + 8);
                curTurbine->spec.bladeCount = 3;
                curTurbine->spec.hubRadius = 2.5;
                curTurbine->spec.towerBottomDia = 6.0;
                curTurbine->spec.towerTopDia = 4.0;
                curTurbine->spec.nacelleOffset = 4.0;
                curTurbine->spec.coneAngleDeg = 0.0;
                curTurbine->spec.tiltDeg = 5.0;
                config->turbineCount++;

                if (section[8] == '\0') {
                    fprintf(stderr, "[config] 第 %d 行：段落 [turbine.] 缺少风机编号\n", lineNo);
                    ok = false;
                }
            } else if (strcmp(section, "field") != 0 && strcmp(section, "inspection") != 0 &&
                       strcmp(section, "app") != 0) {
                /*
                 * 段落名写错（[inspction]）的后果是整个段落的参数全部失效，
                 * 与键名写错同级，必须一起拦下来 —— 只报「未知键」会让
                 * 操作员以为是单个参数的问题。
                 */
                fprintf(stderr, "[config] 第 %d 行：未知段落 [%s]（本程序定义的段落只有 "
                                "[field] / [inspection] / [app] / [turbine.xxx]）\n",
                        lineNo, section);
                ok = false;
            }
            continue;
        }

        eq = strchr(s, '=');
        if (eq == NULL) {
            fprintf(stderr, "[config] 第 %d 行：缺少 '='，已忽略\n", lineNo);
            ok = false;
            continue;
        }
        *eq = '\0';

        {
            char *key = WtConfig_Trim(s);
            char *value = WtConfig_Trim(eq + 1);
            bool applied = true;
            WtConfigSection keySection = curTurbine != NULL ? WT_CFG_SECTION_TURBINE
                                                           : WT_CFG_SECTION_UNKNOWN;

            if (curTurbine != NULL) {
                applied = WtConfig_ApplyTurbineKey(curTurbine, key, value);
            } else if (strcmp(section, "field") == 0) {
                keySection = WT_CFG_SECTION_FIELD;
                if (strcmp(key, "takeoff_lat") == 0) {
                    applied = WtConfig_GetDouble(value, &config->takeoff.lat);
                } else if (strcmp(key, "takeoff_lon") == 0) {
                    applied = WtConfig_GetDouble(value, &config->takeoff.lon);
                } else if (strcmp(key, "takeoff_alt") == 0) {
                    applied = WtConfig_GetDouble(value, &config->takeoff.alt);
                }
            } else if (strcmp(section, "inspection") == 0) {
                keySection = WT_CFG_SECTION_INSPECTION;
                /*
                 * trace 与站位式采样共处 [inspection] 段：两者是同一层级的
                 * 互斥航线形态，另开段落只会让现场多一层判断。先试 trace，
                 * 未命中再落到 profile —— 两组键名无交集，顺序不影响结果。
                 */
                if (!WtConfig_ApplyTraceKey(config, key, value)) {
                    applied = WtConfig_ApplyProfileKey(&config->profile, key, value);
                }
            } else if (strcmp(section, "app") == 0) {
                keySection = WT_CFG_SECTION_APP;
                if (strcmp(key, "telemetry_hz") == 0) {
                    applied = WtConfig_GetDouble(value, &config->telemetryHz);
                } else if (strcmp(key, "gimbal_track_gain") == 0) {
                    applied = WtConfig_GetDouble(value, &config->gimbalTrackGain);
                } else if (strcmp(key, "track_deadband") == 0) {
                    applied = WtConfig_GetDouble(value, &config->trackDeadbandDeg);
                } else if (strcmp(key, "auto_start") == 0) {
                    applied = WtConfig_GetBool(value, &config->autoStart);
                } else if (strcmp(key, "output_dir") == 0) {
                    applied = (*value != '\0');
                    snprintf(config->outputDir, sizeof(config->outputDir), "%.255s", value);
                } else if (strcmp(key, "dry_run") == 0) {
                    applied = WtConfig_GetBool(value, &config->dryRun);
                } else if (strcmp(key, "no_actions") == 0) {
                    applied = WtConfig_GetBool(value, &config->noActions);
                } else if (strcmp(key, "ai_overlay") == 0) {
                    applied = WtConfig_GetBool(value, &config->aiOverlay);
                } else if (strcmp(key, "stream_probe") == 0) {
                    applied = WtConfig_GetBool(value, &config->streamProbe);
                } else if (strcmp(key, "survey_enabled") == 0) {
                    applied = WtConfig_GetBool(value, &config->surveyEnabled);
                } else if (strcmp(key, "survey_engine") == 0) {
                    applied = (*value != '\0');
                    snprintf(config->surveyEnginePath, sizeof(config->surveyEnginePath),
                             "%.255s", value);
                } else if (strcmp(key, "survey_infer_interval_ms") == 0) {
                    applied = WtConfig_GetInt(value, &config->surveyInferIntervalMs);
                } else if (strcmp(key, "geoid_offset_m") == 0) {
                    applied = WtConfig_GetDouble(value, &config->geoidOffsetM);
                } else if (strcmp(key, "survey_selftest") == 0) {
                    applied = WtConfig_GetBool(value, &config->surveySelfTest);
                } else if (strcmp(key, "ped_enabled") == 0) {
                    applied = WtConfig_GetBool(value, &config->pedEnabled);
                } else if (strcmp(key, "ped_engine") == 0) {
                    applied = (*value != '\0');
                    snprintf(config->pedEnginePath, sizeof(config->pedEnginePath),
                             "%.255s", value);
                } else if (strcmp(key, "ped_dwell_ms") == 0) {
                    applied = WtConfig_GetInt(value, &config->pedDwellMs);
                    pedDwellSet = applied;
                } else if (strcmp(key, "ped_conf") == 0) {
                    applied = WtConfig_GetDouble(value, &config->pedConf);
                    pedConfSet = applied;
                }
            }

            if (!applied) {
                fprintf(stderr, "[config] 第 %d 行：[%s] %s = %s 取值非法\n",
                        lineNo, section, key, value);
                ok = false;
            } else if (!WtConfig_KeyKnown(keySection, key)) {
                /*
                 * 键名不认识：值本身可能是合法的，正因如此才危险 ——
                 * 操作员以为 min_safe_distt = 8 已经放宽了安全距，程序却
                 * 仍按默认的 3m 飞。这里必须报错而不是告警，且给出候选键名，
                 * 否则现场只能对着模板逐字数。
                 */
                const char *suggest = WtAppConfig_SuggestKey(keySection, key);

                fprintf(stderr, "[config] 第 %d 行：未识别的键 [%s] %s%s%s\n", lineNo,
                        section, key, suggest != NULL ? "，是否想写 " : "",
                        suggest != NULL ? suggest : "");
                ok = false;
            }
        }
    }

    fclose(fp);

    /* 缺省值 */
    if (config->telemetryHz <= 0.0) {
        config->telemetryHz = 10.0;
    }
    if (config->outputDir[0] == '\0') {
        /*
         * 默认输出目录：**相对应用目录**的 data/，不是绝对路径。
         *
         * 应用以 uid=1000（dji）运行，/data/ 不可写；官方打包脚本会在包内
         * 建好 data/（属主 dji:dji 可写），这是官方的「相对应用目录」约定。
         * 写绝对路径的结果是 WtRunner_MakeDirs 静默失败、作业产物无处可落。
         */
        snprintf(config->outputDir, sizeof(config->outputDir), "%s", "data");
    }
    /*
     * 行人瞄准：不写用默认；写了就必须在范围内，否则拒绝 —— 而不是夹到边界上
     * 悄悄用一个操作员没写过的值。
     */
    if (!pedDwellSet) {
        /* 默认 0：锁定后只靠「连续 3 帧确认」就打；要停顿再配（见 wt_ped_logic.h） */
        config->pedDwellMs = 0;
    } else if (config->pedDwellMs < 0 || config->pedDwellMs > 30000) {
        fprintf(stderr, "[config] [app] ped_dwell_ms = %d 超出范围 [0, 30000]\n",
                config->pedDwellMs);
        ok = false;
    }
    if (!pedConfSet) {
        config->pedConf = 0.40;
    } else if (!(config->pedConf > 0.0 && config->pedConf < 1.0)) {
        fprintf(stderr, "[config] [app] ped_conf = %g 超出范围 (0, 1)\n", config->pedConf);
        ok = false;
    }

    if (config->takeoff.lat == 0.0 && config->takeoff.lon == 0.0 && config->turbineCount > 0) {
        config->takeoff = config->turbines[0].spec.base;
    }

    if (config->turbineCount == 0) {
        fprintf(stderr, "[config] 未定义任何风机（需要至少一个 [turbine.xxx] 段落）\n");
        ok = false;
    }

    {
        WtValidateResult vr = WtInspectionProfile_Validate(&config->profile);

        if (!vr.ok) {
            fprintf(stderr, "[config] 作业剖面非法：%s\n", vr.message);
            ok = false;
        }
    }

    for (int i = 0; i < config->turbineCount; i++) {
        WtValidateResult vr = WtTurbine_Validate(&config->turbines[i].spec);

        if (!vr.ok) {
            fprintf(stderr, "[config] 风机 %s 参数非法：%s\n",
                    config->turbines[i].name, vr.message);
            ok = false;
        }
    }

    return ok;
}

void WtAppConfig_Free(WtAppConfig *config)
{
    free(config->turbines);
    memset(config, 0, sizeof(*config));
}

const WtTurbineEntry *WtAppConfig_FindTurbine(const WtAppConfig *config, const char *name)
{
    int i;

    for (i = 0; i < config->turbineCount; i++) {
        if (strcmp(config->turbines[i].name, name) == 0) {
            return &config->turbines[i];
        }
    }

    return NULL;
}

WtTurbineEntry *WtAppConfig_FindTurbineMutable(WtAppConfig *config, const char *name)
{
    int i;

    if (config == NULL || name == NULL) {
        return NULL;
    }

    for (i = 0; i < config->turbineCount; i++) {
        if (strcmp(config->turbines[i].name, name) == 0) {
            return &config->turbines[i];
        }
    }

    return NULL;
}

void WtAppConfig_Dump(const WtAppConfig *config)
{
    int i;

    printf("---- 作业配置 ----\n");
    printf("  输出目录     : %s\n", config->outputDir);
    printf("  遥测频率     : %.1f Hz\n", config->telemetryHz);
    printf("  自动启动     : %s\n", config->autoStart ? "是" : "否");
    printf("  起飞点       : %.7f, %.7f, %.1f m\n",
           config->takeoff.lat, config->takeoff.lon, config->takeoff.alt);
    printf("  相机 / GSD   : %s / %.2f mm/px\n",
           config->profile.camera.name, config->profile.targetGsdMmPerPx);
    printf("  风轮可转动   : %s\n", config->profile.rotorMayRotate ? "是（叶尖追踪模式）" : "否（停机巡检）");
    printf("  风机数量     : %d\n", config->turbineCount);
    printf("  行人瞄准     : %s（锁定确认后再停 %.1f s 打点，捕获置信度 %.2f，engine %s）\n",
           config->pedEnabled ? "开" : "关", (double)config->pedDwellMs / 1000.0,
           config->pedConf, config->pedEnginePath[0] != '\0' ? config->pedEnginePath : "未配置");

    for (i = 0; i < config->turbineCount; i++) {
        const WtTurbineSpec *s = &config->turbines[i].spec;

        printf("    [%d] %-12s 轮毂高 %.1f m, 叶轮直径 %.1f m, 机舱朝向 %.1f°\n",
               i, config->turbines[i].name, s->hubHeight, s->rotorDiameter, s->headingDeg);
    }
    printf("------------------\n");
}

bool WtAppConfig_WriteTemplate(const char *path)
{
    FILE *fp = fopen(path, "w");

    if (fp == NULL) {
        return false;
    }

    fprintf(fp,
            "# 风机叶片无人机巡检 —— 作业配置模板\n"
            "# 所有几何参数取自风机台账/机组手册，务必核对后再执行。\n"
            "#\n"
            "# 本模板列出的键即全部合法键名；上述四个段落内出现模板之外的键会被\n"
            "# 判为错误并拒绝启动（防的是 min_safe_distt 这类拼写错误被静默丢弃）。\n"
            "\n"
            "[field]\n"
            "# 起飞点（塔基旁安全区域），WGS84\n"
            "takeoff_lat = 41.5236000\n"
            "takeoff_lon = 111.7461000\n"
            "takeoff_alt = 1455.0\n"
            "\n"
            "[inspection]\n"
            "# 相机: wide(24mm) / mid(70mm) / tele(168mm)\n"
            "camera              = mid\n"
            "# 缺陷检测目标分辨率 mm/px。粗模环绕用图不参与该判定。\n"
            "target_gsd          = 1.50\n"
            "# 叶片拍摄距离，0 = 由 target_gsd 自动反算\n"
            "blade_standoff      = 0\n"
            "tower_standoff      = 0\n"
            "# 到叶片轴线的硬下限。叶片静止时 3m 足够；风轮若可能转动，\n"
            "# 该值同时成为「离风轮盘面的最小距离」，需显著放大。\n"
            "min_safe_dist       = 3.0\n"
            "ground_clearance    = 15.0\n"
            "# 风轮是否可能转动。停机精细巡检填 false；叶尖追踪填 true。\n"
            "rotor_may_rotate    = false\n"
            "# 巡检面: upstream(迎风/前缘) / downstream(背风) / both\n"
            "blade_side          = upstream\n"
            "overlap             = 70\n"
            "blade_spacing       = 0\n"
            "max_samples_per_blade = 30\n"
            "# 斜视角：站位沿叶片轴向的偏移量，取「拍摄斜距的比例」（不是米）。\n"
            "# 0 = 正对拍摄，前缘与后缘会被压成一条线；0.3 约合 17° 斜视角。\n"
            "# 正 = 朝下游(背风侧)，迎风面巡检取正值可同时看到前缘与叶面。\n"
            "blade_axial_shift   = 0\n"
            "tower_ring_count    = 4\n"
            "tower_per_ring      = 6\n"
            "include_tower       = true\n"
            "do_coarse_survey    = true\n"
            "coarse_radius       = 0\n"
            "coarse_photo_count  = 16\n"
            "coarse_pitch        = -45\n"
            "cruise_speed        = 8.0\n"
            "inspect_speed       = 2.0\n"
            "photo_dwell         = 1.5\n"
            "\n"
            "# ---- 沿中线飞行模式（trace）------------------------------------\n"
            "# 与上面的「盘外站位斜拍」互斥的另一种航线形态：无人机沿叶片轴线\n"
            "# 平行飞过，到叶尖后以叶尖为圆心绕半圈切到另一侧，再飞回轮毂。\n"
            "# 一片叶一趟同时覆盖迎风面（去程）与背风面（回程）。\n"
            "# 启用后，上面的站位式采样参数（overlap / blade_spacing 等）不再参与。\n"
            "#\n"
            "# trace_offset 是离叶片轴线的偏置距离，必须显著大于 min_safe_dist：\n"
            "# 偏置越小成像越好，但越贴近叶片，任何定位漂移都更危险。\n"
            "trace_enable        = false\n"
            "trace_offset        = 10.0\n"
            "# 沿展向的航点间距，决定成像重叠率\n"
            "trace_step          = 5.0\n"
            "# 轮毂前方捕获相位的距离\n"
            "trace_approach      = 30.0\n"
            "# 叶尖掉头半圆的分段数，至少 2\n"
            "trace_arc_segments  = 8\n"
            "\n"
            "[app]\n"
            "telemetry_hz       = 10\n"
            "gimbal_track_gain  = 0\n"
            "track_deadband     = 2.0\n"
            "# 作业总闸门。false = 应用起来后停在待命状态，不执行作业；\n"
            "# true = 逐台风机执行「规划 → 上传 → 启动任务」。\n"
            "# 默认 false：装包时安装器要试运行应用，自动开始会挂住导致装不上。\n"
            "auto_start         = false\n"
            "# 作业产物的输出目录。相对应用目录 —— 应用以 uid=1000 运行，\n"
            "# 绝对路径 /data/... 不可写。官方包内已建好 data/（可写）。\n"
            "output_dir         = data\n"
            "# 干跑：只做「规划 → 生成 KMZ → 上传」，不启动任务、飞机不飞。\n"
            "# 用于在没有现场时验证飞控能否接受这条航线（上传失败会暴露 KMZ\n"
            "# 格式/坐标系/航向模式的问题）。验证完记得改回 false。\n"
            "dry_run            = false\n"
            "# 诊断用：生成不含云台/相机动作的航线，仅用于在 DJI Assistant 2\n"
            "# 模拟器里排查「START 被拒」（模拟器不模拟云台与相机）。\n"
            "# ⚠️ 真机作业绝不能开 —— 没有拍照动作的巡检航线毫无意义。\n"
            "# no_actions        = false\n"
            "# 诊断用：往 Pilot 2 画面发固定的识别框（PSDK liveview AI 叠加），\n"
            "# 用于验证 DjiLiveview_SendAiMetaToPilot 是否可用、自定义标签的\n"
            "# type 索引从几开始。开启后每秒发 3 个框（type 0/1/2，左/中/右）。\n"
            "# ⚠️ 不是生产功能，框的位置和标签都是写死的。验证完关掉。\n"
            "# ai_overlay        = false\n"
            "# 诊断用：启动相机图像流并统计收到的帧，判定能否拿到实时画面。\n"
            "# 用于确认「持续跟踪」方案是否可行。验证完关掉。\n"
            "# stream_probe      = false\n"
            "\n"
            "# 激光采集盘面几何：依次对轮毂与三个叶尖打激光，拟合出实测的\n"
            "# 盘面朝向与**停用角** —— 后者正是 park_phase 未知时作业被拒的前提。\n"
            "# ⚠️ 与 stream_probe 互斥（两者都会启动同一个相机源的图像流）。\n"
            "# ⚠️ engine 必须是**设备上**用 trtexec 构建的，且别放 /tmp（重启会清空）。\n"
            "# survey_enabled    = false\n"
            "# survey_engine     = /home/dji/wt-models/model-s.engine\n"
            "# survey_infer_interval_ms = 200\n"
            "# 本地 geoid 偏移（MSL − 椭球高），中国境内为负。只有在没有 RTK\n"
            "# 固定解、退化到融合定位时才需要 —— 融合定位给椭球高而台账给海拔。\n"
            "# 不填则 ENU 天向分量被标记为不可信（水平仍可用）。\n"
            "# geoid_offset_m    = -38.0\n"
            "# 启动后跑一次云台+激光自检（不需要风机在画面里）。验完关掉。\n"
            "# survey_selftest   = false\n"
            "\n"
            "# 高空行人瞄准打点：识别画面里的行人（VisDrone 航拍检测模型），云台对准\n"
            "# 画面中心附近的那个人，锁定后连续 3 帧确认（再停 ped_dwell_ms）就自动\n"
            "# 打激光（连读 3 次取中位），落点写入 <output_dir>/ped_points.csv。\n"
            "# 由 Pilot 2 的「行人瞄准」开关触发。\n"
            "# ⚠️ 与 survey_enabled、stream_probe 互斥（都要独占同一个相机源的图像流）。\n"
            "# ⚠️ engine 必须是**设备上**用 trtexec 构建的，且别放 /tmp（重启会清空）。\n"
            "# ped_enabled       = false\n"
            "# ped_engine        = /home/dji/wt-models/visdrone-yolo11s-1280.engine\n"
            "# 锁定确认后再停多久才打激光（毫秒），范围 0~30000，默认 0（不额外停顿）。\n"
            "# 需要「瞄准后停 3 s 再打」时写 3000。\n"
            "# ped_dwell_ms      = 0\n"
            "# 新捕获一个行人所需的置信度（跟上之后不再要求），范围 0~1，默认 0.40\n"
            "# ped_conf          = 0.40\n"
            "\n"
            "# 每台风机一个段落，段落名即风机编号\n"
            "[turbine.WT-A01]\n"
            "lat               = 41.5236000\n"
            "lon               = 111.7461000\n"
            "alt               = 1450.0\n"
            "hub_height        = 100.0\n"
            "rotor_diameter    = 155.0\n"
            "hub_radius        = 2.5\n"
            "tower_bottom_dia  = 6.5\n"
            "tower_top_dia     = 4.0\n"
            "nacelle_offset    = 4.5\n"
            "cone_angle        = 4.0\n"
            "tilt              = 5.0\n"
            "# 机舱朝向 = 来流来向，自正北顺时针。现场以风向标/机舱指向实测为准。\n"
            "heading           = 30.0\n"
            "prebend           = 3.0\n"
            "blade_count       = 3\n"
            "# 停用角（停机位相位），自 12 点方向起、沿叶片旋转方向为正，0~360。\n"
            "# 不写或写负数 = 现场未提供：规划按「叶片停在 12 点方向」假设，\n"
            "# 并禁止自动启动（auto_start = true 时直接拒绝执行）。\n"
            "# 规划的安全包络完全依赖该值，务必现场实测后再填。\n"
            "# 这是停用角的唯一来源（运行器直接读本条目，不接受别处另传）。\n"
            "# park_phase      = 45.0\n");

    fclose(fp);

    return true;
}