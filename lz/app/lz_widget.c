/**
 * @file lz_widget.c
 * @brief 控件模块实现。
 *
 * 骨架取自官方样例 `samples/sample_c/module_sample/widget/test_widget.c`，
 * 砍到只剩本项目需要的三个控件。**初始化四步的顺序不能变**：
 *
 *   1. DjiWidget_Init()
 *   2. 注册 UI 配置（Linux 下用 ByDirPath；RTOS 下用 ByBinaryArray）
 *   3. DjiWidget_RegHandlerList()  ← 必须在 ApplicationStart 之前
 *   4. 建状态推送线程
 */

#include "lz_widget.h"

#include <dji_logger.h>
#include <dji_platform.h>
#include <dji_widget.h>

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

/* ------------------------------------------------------------------ */
/* 控件索引（必须与 app/widget_file 下的 widget_config.json 一致）        */
/* ------------------------------------------------------------------ */
#define LZ_WIDGET_IDX_ORBIT_SWITCH 0
#define LZ_WIDGET_IDX_RADIUS_SCALE 1
#define LZ_WIDGET_IDX_HEIGHT_SCALE 2
#define LZ_WIDGET_COUNT            3

/* 范围条是 0–100 的百分比，这里映射到实际物理量。
 *
 * 高度上限 150 m 是 2026-09-20 按场地要求定的：场地地面海拔约 50–80 m，
 * 相对起飞点 100 m 即约 150 m ASL。
 *
 * ⚠️ 这是**相对起飞点**的高度（KMZ 里 executeHeightMode=relativeToStartPoint），
 *    不是绝对海拔。要飞真正的 ASL 高度得改高度模式并处理大地水准面差距，
 *    那是另一回事。 */
#define LZ_RADIUS_MIN_M   5.0
#define LZ_RADIUS_MAX_M   30.0
#define LZ_ALTITUDE_MIN_M 5.0
#define LZ_ALTITUDE_MAX_M 150.0

#define LZ_MSG_MAX_LEN  256
#define LZ_MSG_PERIOD_MS 500    /* 浮窗消息推送周期，远低于 2 KB/s 上限 */

/* ------------------------------------------------------------------ */
/* 状态                                                                */
/* ------------------------------------------------------------------ */
static bool s_orbitRequested = false;
static int32_t s_radiusPercent = 50;   /* 默认 50% → 17.5 m，与 demo 的 20 m 接近 */
static int32_t s_heightPercent = 66;   /* 默认 66% → 100.7 m 相对高度 ≈ 150 m ASL（场地地面约 50 m） */
static T_DjiTaskHandle s_msgTask = NULL;
static volatile bool s_msgTaskRun = false;
static char s_msgBuf[LZ_MSG_MAX_LEN];

static double lz_map_percent(int32_t pct, double lo, double hi)
{
    if (pct < 0) {
        pct = 0;
    }
    if (pct > 100) {
        pct = 100;
    }
    return lo + (hi - lo) * (double)pct / 100.0;
}

/* ------------------------------------------------------------------ */
/* 回调：操作员动控件时被调用                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief 控件值被设置（操作员拨动开关/拖动范围条）
 *
 * ️ 这个回调运行在 **PSDK 的工作线程**上，不是我们的主循环。
 * 所以这里**只记录状态、不做动作** —— 真正的"上传航点并开始"由主循环
 * 消费 s_orbitRequested 后执行。理由：
 *   1. 上传 KMZ 是耗时操作（要发好几 KB），在回调里做会阻塞 PSDK 线程
 *   2. 作业决策集中在一处，比散落在回调里容易审查
 */
static T_DjiReturnCode LzWidget_SetWidgetValue(E_DjiWidgetType widgetType, uint32_t index,
                                               int32_t value, void *userData)
{
    (void)userData;

    switch (index) {
    case LZ_WIDGET_IDX_ORBIT_SWITCH:
        if (widgetType != DJI_WIDGET_TYPE_SWITCH) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
        }
        if (value == DJI_WIDGET_SWITCH_STATE_ON) {
            s_orbitRequested = true;
            LzWidget_PostMessage("收到绕飞请求：半径 %.1f m，高度 %.1f m",
                                 LzWidget_GetRadiusM(), LzWidget_GetAltitudeM());
        } else {
            s_orbitRequested = false;
            LzWidget_PostMessage("收到停止请求");
        }
        break;

    case LZ_WIDGET_IDX_RADIUS_SCALE:
        s_radiusPercent = value;
        LzWidget_PostMessage("半径设为 %.1f m", LzWidget_GetRadiusM());
        break;

    case LZ_WIDGET_IDX_HEIGHT_SCALE:
        s_heightPercent = value;
        LzWidget_PostMessage("高度设为 %.1f m", LzWidget_GetAltitudeM());
        break;

    default:
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief 控件值被读取（Pilot 2 打开界面时拉取当前值）
 *
 * **必须实现**：若返回失败，Pilot 会显示默认值，界面与实际状态不一致。
 */
static T_DjiReturnCode LzWidget_GetWidgetValue(E_DjiWidgetType widgetType, uint32_t index,
                                               int32_t *value, void *userData)
{
    (void)userData;

    if (value == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    switch (index) {
    case LZ_WIDGET_IDX_ORBIT_SWITCH:
        *value = s_orbitRequested ? DJI_WIDGET_SWITCH_STATE_ON
                                  : DJI_WIDGET_SWITCH_STATE_OFF;
        break;
    case LZ_WIDGET_IDX_RADIUS_SCALE:
        *value = s_radiusPercent;
        break;
    case LZ_WIDGET_IDX_HEIGHT_SCALE:
        *value = s_heightPercent;
        break;
    default:
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    (void)widgetType;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static const T_DjiWidgetHandlerListItem s_widgetHandlerList[LZ_WIDGET_COUNT] = {
    {LZ_WIDGET_IDX_ORBIT_SWITCH, DJI_WIDGET_TYPE_SWITCH, LzWidget_SetWidgetValue, LzWidget_GetWidgetValue, NULL},
    {LZ_WIDGET_IDX_RADIUS_SCALE, DJI_WIDGET_TYPE_SCALE,  LzWidget_SetWidgetValue, LzWidget_GetWidgetValue, NULL},
    {LZ_WIDGET_IDX_HEIGHT_SCALE, DJI_WIDGET_TYPE_SCALE,  LzWidget_SetWidgetValue, LzWidget_GetWidgetValue, NULL},
};

/* ------------------------------------------------------------------ */
/* 浮窗消息                                                            */
/* ------------------------------------------------------------------ */

void LzWidget_PostMessage(const char *fmt, ...)
{
    if (fmt == NULL) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_msgBuf, sizeof(s_msgBuf), fmt, ap);
    va_end(ap);

    /* 浮窗消息是**尽力而为**：失败不视为错误。
     * 通道未就绪（Pilot 没打开页面）时返回失败是正常的，
     * 把它当错误会让日志里全是噪音。 */
    (void)DjiWidgetFloatingWindow_ShowMessage(s_msgBuf);

    USER_LOG_INFO("[widget] %s", s_msgBuf);
}

/**
 * @brief 状态推送线程
 *
 * 为什么需要它：浮窗消息是"推"出去的，但控件回调只在操作员动手时触发。
 * 绕飞进行中需要**周期性**汇报进度（剩余航点、当前状态），
 * 所以要有自己的心跳线程。
 */
static void *LzWidget_MsgTask(void *arg)
{
    (void)arg;

    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    if (osal == NULL) {
        return NULL;
    }

    char lastMsg[LZ_MSG_MAX_LEN] = {0};
    while (s_msgTaskRun) {
        /* 只在消息变化时才发 —— 避免刷屏，也省带宽（上限 2 KB/s） */
        if (strcmp(lastMsg, s_msgBuf) != 0) {
            (void)DjiWidgetFloatingWindow_ShowMessage(s_msgBuf);
            memcpy(lastMsg, s_msgBuf, sizeof(lastMsg) - 1);
            lastMsg[sizeof(lastMsg) - 1] = '\0';
        }
        osal->TaskSleepMs(LZ_MSG_PERIOD_MS);
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* 初始化                                                              */
/* ------------------------------------------------------------------ */

/**
 * @brief 找到控件配置目录，写进 out
 *
 * 为什么不能写死一个路径：**PSDK 的路径是相对进程 CWD 解析的**，而 CWD 在两种
 * 运行方式下不同 ——
 *
 *   手工调试：`cd lz && ./build-native/bin/lz_app`      → CWD = lz/
 *   装成 dpk：`dji_app_ctl start` 把 CWD 设在**包根**  → CWD = /open_app/<app>/
 *
 * 所以两个候选都要试，顺序是"装包优先"：
 *   1. `widget_file/...`      —— dpk 布局（`app.json` 的 userconfig 把它拷到包根）
 *   2. `app/widget_file/...`  —— 源码树里从 lz/ 手工跑
 *
 * 设备上已有先例佐证 dpk 布局：预装的 `proj1_app` 二进制里字符串就是
 * `widget_file/en_big_screen`，而 `/open_app/install/widget_file/` 正在包根。
 */
static const char *lz_widget_resolve_dir(const char *lang_dir)
{
    static char resolved[256];
    struct stat st;

    const char *candidates[] = {
        "widget_file",       /* dpk 安装后：CWD = 包根 */
        "app/widget_file",   /* 源码树里从 lz/ 手工跑 */
    };

    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
        snprintf(resolved, sizeof(resolved), "%s/%s", candidates[i], lang_dir);
        if (stat(resolved, &st) == 0 && S_ISDIR(st.st_mode)) {
            USER_LOG_INFO("控件配置目录: %s（相对 CWD %s）", resolved, candidates[i]);
            return resolved;
        }
    }

    /* 都没找到：返回第一个候选，让 SDK 报出它自己的错，
     * 同时把 CWD 打进日志 —— 这类"文件找不到"没有 CWD 根本没法排查。 */
    char cwd[128] = {0};
    /* getcwd 带 warn_unused_result：显式消费返回值，失败也只是 CWD 留空，
     * 不影响诊断价值（路径本身已经打出来了） */
    if (getcwd(cwd, sizeof(cwd) - 1) == NULL) {
        snprintf(cwd, sizeof(cwd), "(读取失败)");
    }
    USER_LOG_ERROR("找不到控件配置目录 %s/*（当前 CWD = %s）", lang_dir, cwd);
    snprintf(resolved, sizeof(resolved), "%s/%s", candidates[0], lang_dir);
    return resolved;
}

T_DjiReturnCode LzWidget_Init(void)
{
    T_DjiReturnCode rc = DjiWidget_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiWidget_Init 失败 rc=0x%08X", (unsigned)rc);
        return rc;
    }

    /* UI 配置：两份（中/英），控件类型/索引/数量必须一致，否则行为未定义 */
    rc = DjiWidget_RegDefaultUiConfigByDirPath(lz_widget_resolve_dir("cn_big_screen"));
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("注册默认控件配置失败 rc=0x%08X", (unsigned)rc);
        return rc;
    }

    rc = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_CHINESE,
                                        DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN,
                                        lz_widget_resolve_dir("cn_big_screen"));
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("注册中文控件配置失败 rc=0x%08X", (unsigned)rc);
        return rc;
    }

    rc = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_ENGLISH,
                                        DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN,
                                        lz_widget_resolve_dir("en_big_screen"));
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("注册英文控件配置失败 rc=0x%08X", (unsigned)rc);
        return rc;
    }

    rc = DjiWidget_RegHandlerList(s_widgetHandlerList, LZ_WIDGET_COUNT);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("注册控件 handler 失败 rc=0x%08X", (unsigned)rc);
        return rc;
    }

    s_msgTaskRun = true;
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    if (osal == NULL) {
        s_msgTaskRun = false;
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    rc = osal->TaskCreate("lz_widget_msg", LzWidget_MsgTask, 4096, NULL, &s_msgTask);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        s_msgTaskRun = false;
        USER_LOG_ERROR("创建控件消息线程失败 rc=0x%08X", (unsigned)rc);
        return rc;
    }

    LzWidget_PostMessage("就绪：半径 %.1f m，高度 %.1f m，等待操作员拨开关",
                         LzWidget_GetRadiusM(), LzWidget_GetAltitudeM());
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void LzWidget_Stop(void)
{
    s_msgTaskRun = false;

    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    if (osal != NULL && s_msgTask != NULL) {
        osal->TaskDestroy(&s_msgTask);
        s_msgTask = NULL;
    }
    /* 不调用 DjiWidget_DeInit()：该函数不存在（已核实头文件）。
     * 控件模块没有反初始化接口，官方样例同样不注销。 */
}

/* ------------------------------------------------------------------ */
/* 供主循环查询                                                        */
/* ------------------------------------------------------------------ */

bool LzWidget_IsOrbitRequested(void)
{
    return s_orbitRequested;
}

double LzWidget_GetRadiusM(void)
{
    return lz_map_percent(s_radiusPercent, LZ_RADIUS_MIN_M, LZ_RADIUS_MAX_M);
}

double LzWidget_GetAltitudeM(void)
{
    return lz_map_percent(s_heightPercent, LZ_ALTITUDE_MIN_M, LZ_ALTITUDE_MAX_M);
}

void LzWidget_ReportOrbitFinished(const char *reason)
{
    s_orbitRequested = false;

    /* 把开关程序化拨回 OFF：不回弹的话，操作员会看到"开关 ON 但飞机停了"，
     * 分不清是"已完成"还是"卡住了"。 */
    (void)LzWidget_SetWidgetValue(DJI_WIDGET_TYPE_SWITCH, LZ_WIDGET_IDX_ORBIT_SWITCH,
                                  DJI_WIDGET_SWITCH_STATE_OFF, NULL);
    LzWidget_PostMessage("绕飞结束：%s", reason ? reason : "（未说明）");
}
