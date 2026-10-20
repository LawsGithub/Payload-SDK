/**
 * @file wt_test_config.c
 * @brief 配置解析回归测试
 *
 * 守的是同一个约定：**文件里写下的东西必须真的生效，否则必须报错**。
 * 曾经拼错一个键名（min_safe_distt = 8）什么提示都没有，程序照旧按 3m
 * 的安全距规划并起飞 —— 这类静默失败在一切正常时看不出来，在出事时
 * 解释不了。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "wt_app_config.h"
#include "wt_runner.h"
#include "wt_test.h"

#define WT_TEST_TMP_FILE "wt_test_config_tmp.ini"
#define WT_TEST_NESTED_TOP "wt_test_nested"

/** 把一段配置文本写成临时文件，返回其路径 */
static const char *WriteConfig(const char *text)
{
    FILE *fp = fopen(WT_TEST_TMP_FILE, "w");

    if (fp == NULL) {
        return NULL;
    }
    fputs(text, fp);
    fclose(fp);

    return WT_TEST_TMP_FILE;
}

/** 一份最小可用配置：给定段落正文，补上必需的风机条目 */
static const char *BuildConfig(char *buf, size_t bufLen, const char *head,
                               const char *turbineExtra)
{
    snprintf(buf, bufLen,
             "%s"
             "\n[turbine.WT-A01]\n"
             "lat               = 41.5236000\n"
             "lon               = 111.7461000\n"
             "alt               = 1450.0\n"
             "hub_height        = 100.0\n"
             "rotor_diameter    = 155.0\n"
             "%s",
             head, turbineExtra != NULL ? turbineExtra : "");

    return WriteConfig(buf);
}

/* ------------------------------------------------------------------ */

static void TestTemplateIsSelfConsistent(void)
{
    WtAppConfig cfg;

    WT_CASE("模板必须被自己的解析器零错误接受");

    if (!WtAppConfig_WriteTemplate(WT_TEST_TMP_FILE)) {
        WT_CHECK(0);
        return;
    }

    WT_CHECK(WtAppConfig_Load(WT_TEST_TMP_FILE, &cfg));
    if (WtAppConfig_Load(WT_TEST_TMP_FILE, &cfg)) {
        WT_CHECK_NEAR(cfg.profile.overlapPct, 70.0, 1e-9);
        WT_CHECK(cfg.profile.includeTower);
        WT_CHECK_NEAR(cfg.profile.inspectSpeedMs, 2.0, 1e-9);
        WT_CHECK_NEAR(cfg.profile.targetGsdMmPerPx, 1.50, 1e-9);
        WT_CHECK_EQ_INT(cfg.turbineCount, 1);
        WT_CHECK_NEAR(cfg.turbines[0].spec.rotorDiameter, 155.0, 1e-9);
        /* 模板里 park_phase 是可选的注释行，缺省即「未提供」 */
        WT_CHECK(cfg.turbines[0].parkPhaseDeg < 0.0);
        WtAppConfig_Free(&cfg);
    }
}

static void TestMisspelledKeysAreRejected(void)
{
    struct {
        const char *head;         /* 含拼错键的段落 */
        const char *turbineExtra; /* 风机段落里的额外键 */
        const char *what;         /* 用例描述 */
    } cases[] = {
        {"[inspection]\noverla = 90\n", NULL, "overla（漏字母）"},
        {"[inspection]\ninclude_towr = false\n", NULL, "include_towr（漏字母）"},
        {"[inspection]\nmin_safe_distt = 8\n", NULL, "min_safe_distt（多字母）"},
        {"[app]\nauto_star = true\n", NULL, "auto_star（漏字母）"},
        {"[field]\ntakeoff_latt = 41.5\n", NULL, "takeoff_latt（多字母）"},
        {NULL, "park_phse = 45.0\n", "park_phse（漏字母）"},
    };
    size_t i;

    WT_CASE("键名拼错必须被发现，而不是静默沿用默认值");

    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        char text[512];
        const char *path = BuildConfig(text, sizeof(text), cases[i].head,
                                       cases[i].turbineExtra);
        WtAppConfig cfg;

        if (path == NULL) {
            WT_CHECK(0);
            continue;
        }

        printf("        用例 %s\n", cases[i].what);
        WT_CHECK(!WtAppConfig_Load(path, &cfg));
    }
}

static void TestMisspelledKeyKeepsDefault(void)
{
    const char *path;
    char text[512];
    WtAppConfig cfg;

    WT_CASE("拼错 min_safe_dist 后，安全距仍是默认的 3.0m（正是危险所在）");

    path = BuildConfig(text, sizeof(text), "[inspection]\nmin_safe_distt = 8\n", NULL);
    if (path == NULL) {
        WT_CHECK(0);
        return;
    }

    /* 这条断言要说明的是「为什么必须报错」：解析失败，但内存里的字段仍是
       默认值。若只是静默忽略，程序会带着 3.0m 的安全距起飞，
       而操作员以为自己填了 8。 */
    WT_CHECK(!WtAppConfig_Load(path, &cfg));
}

static void TestUnknownSectionIsRejected(void)
{
    const char *path;
    char text[512];
    WtAppConfig cfg;

    WT_CASE("段落名拼错：整段参数失效，必须报错");

    path = BuildConfig(text, sizeof(text), "[inspction]\noverlap = 90\n", NULL);
    if (path == NULL) {
        WT_CHECK(0);
        return;
    }

    WT_CHECK(!WtAppConfig_Load(path, &cfg));
}

static void TestMissingEqualsIsRejected(void)
{
    const char *path;
    char text[512];
    WtAppConfig cfg;

    WT_CASE("缺少 '=' 的行：曾只提示不失败，现在必须拒绝");

    path = BuildConfig(text, sizeof(text), "[inspection]\nmin_safe_dist 8\n", NULL);
    if (path == NULL) {
        WT_CHECK(0);
        return;
    }

    WT_CHECK(!WtAppConfig_Load(path, &cfg));
}

static void TestBadValuesStillFail(void)
{
    struct {
        const char *head;
        const char *what;
    } cases[] = {
        {"[inspection]\noverlap = 把笔误打成中文\n", "值不是数字"},
        {"[inspection]\ninclude_tower = maybe\n", "值不是布尔"},
        {"[inspection]\noverlap = 200\n", "值超范围"},
        {"[inspection]\ntarget_gsd = -1\n", "值非正"},
    };
    size_t i;

    WT_CASE("非法取值必须让 Load 失败（回归保护）");

    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        char text[512];
        const char *path = BuildConfig(text, sizeof(text), cases[i].head, NULL);
        WtAppConfig cfg;

        if (path == NULL) {
            WT_CHECK(0);
            continue;
        }

        printf("        用例 %s\n", cases[i].what);
        WT_CHECK(!WtAppConfig_Load(path, &cfg));
    }
}

static void TestMissingKeysUseDefaults(void)
{
    const char *path;
    char text[512];
    WtAppConfig cfg;

    WT_CASE("未配置的键沿用默认值（与「拼错」区分开，不能因噎废食）");

    path = BuildConfig(text, sizeof(text), "", NULL);
    if (path == NULL) {
        WT_CHECK(0);
        return;
    }

    WT_CHECK(WtAppConfig_Load(path, &cfg));
    if (WtAppConfig_Load(path, &cfg)) {
        WT_CHECK_NEAR(cfg.profile.overlapPct, 70.0, 1e-9);
        WT_CHECK(cfg.profile.includeTower);
        WT_CHECK_NEAR(cfg.profile.minSafeDistM, 3.0, 1e-9);
        WT_CHECK_NEAR(cfg.profile.cruiseSpeedMs, 8.0, 1e-9);
        WT_CHECK_NEAR(cfg.profile.inspectSpeedMs, 2.0, 1e-9);
        WT_CHECK_EQ_INT(cfg.profile.bladeSide, WT_BLADE_SIDE_UPSTREAM);
        WT_CHECK_NEAR(cfg.telemetryHz, 10.0, 1e-9);
        /*
         * 默认输出目录必须是**相对应用目录**的 data/。
         *
         * 曾经是绝对路径 /data/wt_inspection —— 那是错的：应用以 uid=1000
         * （dji）运行，/data/ 不可写，WtRunner_MakeDirs 会静默失败，作业
         * 产物无处可落。官方打包脚本在包内建好 data/（属主 dji:dji 可写），
         * 这才是可写的落点。这里连同「不得以 / 开头」一起断言，防止日后
         * 又有人改回绝对路径。
         */
        WT_CHECK_STR_EQ(cfg.outputDir, "data");
        WT_CHECK(cfg.outputDir[0] != '/');

        /* 风机段落的硬编码默认 */
        WT_CHECK_EQ_INT(cfg.turbines[0].spec.bladeCount, 3);
        WT_CHECK_NEAR(cfg.turbines[0].spec.hubRadius, 2.5, 1e-9);
        WT_CHECK_NEAR(cfg.turbines[0].spec.coneAngleDeg, 0.0, 1e-9);
        WT_CHECK(cfg.turbines[0].parkPhaseDeg < 0.0);

        WtAppConfig_Free(&cfg);
    }
}

static void TestParkPhaseAccepted(void)
{
    const char *path;
    char text[512];
    WtAppConfig cfg;

    WT_CASE("配置里写了 park_phase 就要被读进来");

    path = BuildConfig(text, sizeof(text), "", "park_phase = 45.0\n");
    if (path == NULL) {
        WT_CHECK(0);
        return;
    }

    WT_CHECK(WtAppConfig_Load(path, &cfg));
    if (WtAppConfig_Load(path, &cfg)) {
        WT_CHECK_NEAR(cfg.turbines[0].parkPhaseDeg, 45.0, 1e-9);
        WtAppConfig_Free(&cfg);
    }
}

static void TestCommentsAndQuotes(void)
{
    const char *path;
    WtAppConfig cfg;

    WT_CASE("注释处理：';' 起始的尾注释被剥离，引号内的 '#' 保留");

    /*
     * takeoff 的 lat/lon 必须填：两者都为 0 时 Load 会把整个起飞点回退成
     * 首台风机的塔基坐标，那样 alt 也会被一起覆盖，验不出尾注释是否剥离。
     */
    path = WriteConfig("[field]\n"
                       "takeoff_lat = 41.5000\n"
                       "takeoff_lon = 111.7000\n"
                       "takeoff_alt = 1455.0 ; 尾注释\n"
                       "[app]\n"
                       "output_dir = \"/data/a#b\"\n"
                       "auto_start = true\n"
                       "[turbine.WT-A01]\n"
                       "lat = 41.5\n"
                       "lon = 111.7\n"
                       "alt = 1450.0\n"
                       "hub_height = 100.0\n"
                       "rotor_diameter = 155.0\n");
    if (path == NULL) {
        WT_CHECK(0);
        return;
    }

    WT_CHECK(WtAppConfig_Load(path, &cfg));
    if (WtAppConfig_Load(path, &cfg)) {
        WT_CHECK(cfg.autoStart);
        WT_CHECK_NEAR(cfg.takeoff.alt, 1455.0, 1e-9);
        /* 引号只是"保护 '#' 不被当作注释起始"的标记，不会从值里去掉：
           output_dir 是路径，引号原样保留不影响使用，这里是如实记录现状 */
        WT_CHECK_STR_EQ(cfg.outputDir, "\"/data/a#b\"");
        WtAppConfig_Free(&cfg);
    }
}

static void TestMissingFileFails(void)
{
    WtAppConfig cfg;

    WT_CASE("配置文件不存在时 Load 必须失败");

    WT_CHECK(!WtAppConfig_Load("wt_test_no_such_file.ini", &cfg));
}

static void TestDeploymentPaths(void)
{
    char dir[256];
    char cwd[512];
    char nested[640];
    char cfgPath[768];
    WtAppConfig cfg;
    FILE *probe;

    WT_CASE("部署路径：目录不存在时也要能把模板落下来");

    /* WtRunner_MakeDirs 只接受绝对路径（刻意的：机上路径都是绝对的，
       相对路径会让 mkdir 的语义随工作目录漂移）。测试用 getcwd 拼一个绝对
       路径，走的才是机上真正会走的那条分支。 */
    if (getcwd(cwd, sizeof(cwd)) == NULL) {
        printf("        (跳过：取不到工作目录)\n");
        return;
    }

    snprintf(nested, sizeof(nested), "%s/%s/deep", cwd, WT_TEST_NESTED_TOP);
    WT_CHECK(WtRunner_MakeDirs(nested));

    /* 全新设备上 /data/wt_inspection/ 并不存在，配置模板必须先建目录再落盘，
       否则 fopen 静默失败，「首次运行自动生成模板」就是一句空话 */
    snprintf(cfgPath, sizeof(cfgPath), "%s/wt_config.ini", nested);
    WT_CHECK(WtAppConfig_WriteTemplate(cfgPath));

    probe = fopen(cfgPath, "r");
    WT_CHECK(probe != NULL);
    if (probe != NULL) {
        fclose(probe);
    }

    WT_CHECK(WtAppConfig_Load(cfgPath, &cfg));
    if (WtAppConfig_Load(cfgPath, &cfg)) {
        WT_CHECK_NEAR(cfg.profile.targetGsdMmPerPx, 1.5, 1e-9);
        WtAppConfig_Free(&cfg);
    }
    remove(cfgPath);
    remove(nested);
    remove(WT_TEST_NESTED_TOP);

    /* 目录部分提取：这是 main.c 落模板前建目录用的入口 */
    WT_CHECK(WtRunner_DirName("/data/wt_inspection/wt_config.ini", dir, sizeof(dir)));
    WT_CHECK_STR_EQ(dir, "/data/wt_inspection");

    WT_CHECK(WtRunner_MakeDirs("/tmp"));   /* 已存在也算成功 */
    WT_CHECK(!WtRunner_DirName("bare.ini", dir, sizeof(dir)));
}

static void TestKnownKeyTables(void)
{
    static const WtConfigSection sections[] = {
        WT_CFG_SECTION_FIELD,
        WT_CFG_SECTION_INSPECTION,
        WT_CFG_SECTION_APP,
        WT_CFG_SECTION_TURBINE,
    };
    size_t s;

    WT_CASE("键名表可用，且未知段落没有键表");

    for (s = 0; s < sizeof(sections) / sizeof(sections[0]); s++) {
        size_t n = 0;
        const char *const *keys = WtAppConfig_KnownKeys(sections[s], &n);

        WT_CHECK(keys != NULL);
        WT_CHECK(n > 0);
    }

    WT_CHECK(WtAppConfig_KnownKeys(WT_CFG_SECTION_UNKNOWN, NULL) == NULL);
    WT_CHECK(WtAppConfig_SuggestKey(WT_CFG_SECTION_UNKNOWN, "whatever") == NULL);
}

static void TestSpellingSuggestions(void)
{
    WT_CASE("拼错的键名要给出正确候选");

    WT_CHECK_STR_EQ(WtAppConfig_SuggestKey(WT_CFG_SECTION_INSPECTION, "overla"),
                    "overlap");
    WT_CHECK_STR_EQ(WtAppConfig_SuggestKey(WT_CFG_SECTION_INSPECTION, "min_safe_distt"),
                    "min_safe_dist");
    WT_CHECK_STR_EQ(WtAppConfig_SuggestKey(WT_CFG_SECTION_INSPECTION, "include_towr"),
                    "include_tower");
    WT_CHECK_STR_EQ(WtAppConfig_SuggestKey(WT_CFG_SECTION_TURBINE, "park_phse"),
                    "park_phase");

    /* 差得太远就不该乱猜 */
    WT_CHECK(WtAppConfig_SuggestKey(WT_CFG_SECTION_INSPECTION, "zzzzzzzzzzzz") == NULL);
}

/* ------------------------------------------------------------------ */
/* 部署路径与启动路径                                                  */
/* ------------------------------------------------------------------ */

/**
 * 部署路径必须是相对路径，且默认输出目录可写落点正确。
 *
 * 这条用例守的是一类**会装不上包**的配置错误：应用以 uid=1000（dji）运行，
 * /data/ 不可写。若默认路径写成绝对路径 /data/...，WtRunner_MakeDirs 会
 * 静默失败，作业产物无处可落 —— 而现象只是"报告没生成"，很难追。
 */
static void TestDeployPathsAreRelative(void)
{
    WtAppConfig cfg;
    const char *tpl = "wt_test_relpath_tpl.ini";
    const char *tmp = "wt_test_relpath.ini";
    char line[512];
    FILE *in;
    FILE *out;

    WT_CASE("部署路径：默认输出目录必须是相对路径");

    /*
     * 以模板为基底，只去掉 output_dir 那一行 —— 这样其余字段都合法，
     * 能确定测的是「默认值」而不是「配置非法」。
     */
    if (!WtAppConfig_WriteTemplate(tpl)) {
        WT_CHECK(0);
        return;
    }
    in = fopen(tpl, "r");
    out = fopen(tmp, "w");
    if (in == NULL || out == NULL) {
        if (in) fclose(in);
        if (out) fclose(out);
        WT_CHECK(0);
        return;
    }
    while (fgets(line, sizeof(line), in) != NULL) {
        if (strncmp(line, "output_dir", 10) == 0) {
            continue;
        }
        fputs(line, out);
    }
    fclose(in);
    fclose(out);

    if (WtAppConfig_Load(tmp, &cfg)) {
        WT_CHECK_STR_EQ(cfg.outputDir, "data");
        WT_CHECK(cfg.outputDir[0] != '/');
        WtAppConfig_Free(&cfg);
    } else {
        WT_CHECK(0);
    }

    remove(tpl);
    remove(tmp);
}

/**
 * 模板里的 output_dir 也必须是相对路径 —— 现场照着模板填，模板写绝对
 * 路径等于把错误教给操作员。
 */
static void TestTemplateOutputDirIsRelative(void)
{
    const char *tmp = "wt_test_tpl_rel.ini";
    FILE *fp;

    WT_CASE("部署路径：模板里的 output_dir 不是绝对路径");
    WT_CHECK(WtAppConfig_WriteTemplate(tmp));

    fp = fopen(tmp, "r");
    if (fp != NULL) {
        char line[512];
        bool found = false;

        while (fgets(line, sizeof(line), fp) != NULL) {
            if (strncmp(line, "output_dir", 10) == 0) {
                found = true;
                /* 值里不得出现以 '/' 开头的绝对路径 */
                WT_CHECK(strstr(line, "= /") == NULL);
                WT_CHECK(strstr(line, "= data") != NULL);
            }
        }
        fclose(fp);
        WT_CHECK(found);
    } else {
        WT_CHECK(0);
    }

    remove(tmp);
}

/**
 * dry_run 必须是可解析的布尔，且默认 false。
 *
 * 这个开关决定「上传后是否调 Action(START)」—— 也就是**飞机飞不飞**。
 * 默认值必须是 false（安全侧）：若解析出意外为 true，作业会静默地
 * 只上传不飞，现象是"任务启动了但飞机没动"，比反过来的错误难查得多。
 */
static void TestDryRunFlag(void)
{
    WtAppConfig cfg;
    const char *tpl = "wt_test_dryrun_tpl.ini";
    const char *tmp = "wt_test_dryrun.ini";
    char line[512];

    WT_CASE("dry_run：默认 false，且可被配置打开");

    /* 以模板为基底改写，保证其余字段合法 —— 测的是这个键而不是配置合法性 */
    if (!WtAppConfig_WriteTemplate(tpl)) { WT_CHECK(0); return; }

    /* 1) 模板默认（模板里写了 dry_run = false） */
    if (WtAppConfig_Load(tpl, &cfg)) {
        WT_CHECK(!cfg.dryRun);
        WtAppConfig_Free(&cfg);
    } else {
        WT_CHECK(0);
    }

    /* 2) 改成 true 应被读进来 */
    {
        FILE *in = fopen(tpl, "r");
        FILE *out = fopen(tmp, "w");

        if (in == NULL || out == NULL) {
            if (in) fclose(in);
            if (out) fclose(out);
            WT_CHECK(0);
        } else {
            while (fgets(line, sizeof(line), in) != NULL) {
                if (strncmp(line, "dry_run", 7) == 0) {
                    fputs("dry_run            = true\n", out);
                } else {
                    fputs(line, out);
                }
            }
            fclose(in);
            fclose(out);

            if (WtAppConfig_Load(tmp, &cfg)) {
                WT_CHECK(cfg.dryRun);
                WtAppConfig_Free(&cfg);
            } else {
                WT_CHECK(0);
            }
        }
    }

    /* 3) 取值非法必须拒绝（与其它键同一套严格性） */
    {
        FILE *in = fopen(tpl, "r");
        FILE *out = fopen(tmp, "w");

        if (in == NULL || out == NULL) {
            if (in) fclose(in);
            if (out) fclose(out);
            WT_CHECK(0);
        } else {
            while (fgets(line, sizeof(line), in) != NULL) {
                if (strncmp(line, "dry_run", 7) == 0) {
                    fputs("dry_run            = 也许\n", out);
                } else {
                    fputs(line, out);
                }
            }
            fclose(in);
            fclose(out);
            WT_CHECK(!WtAppConfig_Load(tmp, &cfg));
        }
    }

    remove(tpl);
    remove(tmp);
}

/**
 * auto_start 是**作业总闸门**，默认必须是 false。
 *
 * 曾经它只控制「出错后是否继续下一台风机」，名字与行为不符 —— 配置已存在
 * 时进程会直接进作业流程并长时间等待飞控状态，导致安装器试运行超时、
 * 装不上包（2026-10-20 实测）。默认 false 是安全侧：装上后待命，不自动起飞。
 */
static void TestAutoStartGate(void)
{
    WtAppConfig cfg;

    WT_CASE("auto_start：默认 false（作业闸门关闭）");

    if (!WtAppConfig_WriteTemplate(WT_TEST_TMP_FILE)) {
        WT_CHECK(0);
        return;
    }
    if (WtAppConfig_Load(WT_TEST_TMP_FILE, &cfg)) {
        WT_CHECK(!cfg.autoStart);
        WtAppConfig_Free(&cfg);
    } else {
        WT_CHECK(0);
    }

    /* 模板里的注释必须说明它是作业闸门，而不是"出错后继续" */
    {
        FILE *fp = fopen(WT_TEST_TMP_FILE, "r");
        char line[512];
        bool sawGate = false;

        if (fp != NULL) {
            while (fgets(line, sizeof(line), fp) != NULL) {
                if (strstr(line, "作业总闸门") != NULL) {
                    sawGate = true;
                }
            }
            fclose(fp);
            WT_CHECK(sawGate);
        } else {
            WT_CHECK(0);
        }
    }
}

/* ------------------------------------------------------------------ */

int main(void)
{
    TestTemplateIsSelfConsistent();
    TestMisspelledKeysAreRejected();
    TestMisspelledKeyKeepsDefault();
    TestUnknownSectionIsRejected();
    TestMissingEqualsIsRejected();
    TestBadValuesStillFail();
    TestMissingKeysUseDefaults();
    TestParkPhaseAccepted();
    TestCommentsAndQuotes();
    TestMissingFileFails();
    TestDeploymentPaths();
    TestKnownKeyTables();
    TestSpellingSuggestions();
    TestDeployPathsAreRelative();
    TestTemplateOutputDirIsRelative();
    TestDryRunFlag();
    TestAutoStartGate();

    remove(WT_TEST_TMP_FILE);
    remove(WT_TEST_NESTED_TOP);

    return WT_TEST_SUMMARY();
}