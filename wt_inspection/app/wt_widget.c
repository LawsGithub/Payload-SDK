/**
 * @file wt_widget.c
 * @brief 控件模块实现。
 *
 * 骨架取自 liangzhourenwu 项目的 `lz/app/lz_widget.c`（那边已经在 M4T + 妙算3
 * 上跑通），砍到只剩本项目需要的四个控件。
 * **初始化四步的顺序不能变**：
 *
 *   1. DjiWidget_Init()
 *   2. 注册 UI 配置（Linux 下用 ByDirPath；RTOS 下用 ByBinaryArray）
 *   3. DjiWidget_RegHandlerList()  ← 必须在 ApplicationStart 之前
 *   4. 建状态推送线程
 */

#include "wt_widget.h"

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
/*                                                                     */
/* 这 4 个索引必须与两份 json 的 widget_index 一一对应（含 config 界面）—— */
/* 把界面控件分派给 handler，对不上的后果是"按了 A 按钮却执行了 B 的       */
/* 动作"，而两边都不会报错。                                             */
/* ------------------------------------------------------------------ */
/* ---- main_interface（PSDK 菜单，飞行中操作）---- */
#define WT_WIDGET_IDX_START_SWITCH 0
#define WT_WIDGET_IDX_ABORT_BUTTON 1
#define WT_WIDGET_IDX_DRY_RUN      2

/* ---- config_interface（Payload Settings，起飞前配置）----
 *
 * ⚠️ **索引共用一个编号序列，不重叠** —— PSDK 的 handler 列表是**一个扁平的
 * 索引表**，不区分界面，所以两边的 index 必须能在同一张表里唯一对上。
 *
 * ⚠️ **停用角必须放在 config_interface**：`int_input_box` 在 main_interface
 * 里会被 SDK 收下但 Pilot 不渲染成可编辑控件（liangzhourenwu 项目 2026-09-28
 * 实测，日志里 type=5 的回调零命中）。而且停用角本来就是"起飞前定的参数"，
 * 不该在飞行中改。 */
#define WT_WIDGET_IDX_PARK_PHASE   3

#define WT_WIDGET_COUNT            4

#define WT_MSG_MAX_LEN  256
#define WT_MSG_PERIOD_MS 500    /* 浮窗消息推送周期，远低于 2 KB/s 上限 */

/* 停用角的合法范围。负值保留给"未知"语义 —— 与 wt_runner 的判据一致。 */
#define WT_PARK_PHASE_MIN 0
#define WT_PARK_PHASE_MAX 359

/* ------------------------------------------------------------------ */
/* 状态                                                                */
/* ------------------------------------------------------------------ */

/* 作业开始请求。回调只置它，真正的规划/上传由主循环消费后执行。 */
static volatile bool s_startRequested = false;

/* 中止请求。同一个模式：**回调只置它**，真正的中止由主循环调
 * `WtRunner_RequestAbort()` 完成。 */
static volatile bool s_abortPending = false;

/* 停用角：**整数度**，与控件语义一致（int_input_box 只能填整数）。
 *
 * 初值由 main.c 从配置读入后调 `WtWidget_SetParkPhaseInitial()` 设置，
 * 默认 -1 表示"未知"（配置里没写）。操作员填了就覆盖它。
 *
 * ⚠️ 它是**共享状态**：PSDK 回调线程写、主循环读。写入时夹取保证永远合法，
 * 读取时再夹一次是**防御**（不假设上游一定守规矩）。 */
static volatile int32_t s_parkPhaseDeg = -1;

/* 干跑开关。初值来自配置的 `dry_run`。 */
static volatile bool s_dryRun = false;

static T_DjiTaskHandle s_msgTask = NULL;
static volatile bool s_msgTaskRun = false;
static char s_msgBuf[WT_MSG_MAX_LEN];

static int32_t wt_clamp_park_phase(int32_t v)
{
    if (v < WT_PARK_PHASE_MIN) {
        return WT_PARK_PHASE_MIN;
    }
    if (v > WT_PARK_PHASE_MAX) {
        return WT_PARK_PHASE_MAX;
    }
    return v;
}

/* ------------------------------------------------------------------ */
/* 回调：操作员动控件时被调用                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief 控件值被设置（操作员拨开关 / 填输入框）
 *
 * ⚠️ 这个回调运行在 **PSDK 的工作线程**上，不是我们的主循环。
 * 所以这里**只记录状态、不做动作** —— 真正的"规划并上传"由主循环消费
 * s_startRequested 后执行。理由：
 *   1. 规划 + 导出 KMZ + 上传是耗时操作（数秒），在回调里做会阻塞 PSDK 线程
 *   2. 作业决策集中在一处，比散落在回调里容易审查
 */
static T_DjiReturnCode WtWidget_SetWidgetValue(E_DjiWidgetType widgetType, uint32_t index,
                                               int32_t value, void *userData)
{
    (void)userData;

    switch (index) {
    case WT_WIDGET_IDX_START_SWITCH:
        if (widgetType != DJI_WIDGET_TYPE_SWITCH) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
        }
        if (value == DJI_WIDGET_SWITCH_STATE_ON) {
            s_startRequested = true;
            WtWidget_PostMessage("收到开始请求：停用角 %s，干跑 %s",
                                 WtWidget_GetParkPhaseDeg() < 0.0 ? "未设（将拒绝自动启动）"
                                                                  : "已设",
                                 WtWidget_IsDryRun() ? "开" : "关");
        } else {
            s_startRequested = false;
            WtWidget_PostMessage("收到停止请求");
        }
        break;

    /* ---- 中止按钮 ----
     *
     * ⚠️ button 的 `value` 是 `E_DjiWidgetButtonState`：**按下与松开各触发一次**
     * 回调（`PRESS_DOWN=1` / `RELEASE_UP=0`）。只在按下时置标志 ——
     * 不判的话一次点击会触发两遍。
     *
     * ⚠️ 按下**当场**回一条消息，不是客套：中止要等主循环下一拍才真正执行，
     * 这期间浮窗安静会诱发操作员重复按。 */
    case WT_WIDGET_IDX_ABORT_BUTTON:
        if (widgetType != DJI_WIDGET_TYPE_BUTTON) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
        }
        if (value != DJI_WIDGET_BUTTON_STATE_PRESS_DOWN) {
            break;   /* 忽略松开的半次 */
        }
        s_abortPending = true;
        WtWidget_PostMessage("收到中止请求，正在停止…");
        break;

    case WT_WIDGET_IDX_DRY_RUN:
        if (widgetType != DJI_WIDGET_TYPE_SWITCH) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
        }
        s_dryRun = (value == DJI_WIDGET_SWITCH_STATE_ON);
        WtWidget_PostMessage("干跑 = %s%s", s_dryRun ? "开" : "关",
                             s_dryRun ? "（只上传航线，不启动任务）" : "");
        break;

    /* ---- 停用角输入框 ----
     *
     * ⚠️ 输入框的 value 是操作员**任意敲的整数**，可能远超合理范围
     * （甚至为负）。所以这里必须夹取，且夹取后要**回一条消息**告诉操作员
     * 实际生效值 —— 否则他填了 400、界面显示 400、而程序按 359 飞，
     * 这个差异要等到航线画出来才发现。
     *
     * 为什么不在回调里直接拒绝非法值：控件是"设置"语义，不是"提交"语义，
     * 拒绝会让输入框停在那个非法值上不动。夹取 + 告知符合操作员预期。 */
    case WT_WIDGET_IDX_PARK_PHASE: {
        const int32_t clamped = wt_clamp_park_phase(value);

        if (widgetType != DJI_WIDGET_TYPE_INT_INPUT_BOX) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
        }
        s_parkPhaseDeg = clamped;
        if (clamped != value) {
            WtWidget_PostMessage("停用角 %d 超出 %d–%d，已按 %d 使用",
                                 (int)value, WT_PARK_PHASE_MIN, WT_PARK_PHASE_MAX,
                                 (int)clamped);
        } else {
            WtWidget_PostMessage("停用角设为 %d°", (int)clamped);
        }
        break;
    }

    default:
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief 控件值被读取（Pilot 2 打开界面时拉取当前值）
 *
 * **必须实现**：若返回失败，Pilot 会显示默认值，界面与实际状态不一致 ——
 * 操作员看到停用角 0，而程序实际用配置里的 45。
 */
static T_DjiReturnCode WtWidget_GetWidgetValue(E_DjiWidgetType widgetType, uint32_t index,
                                               int32_t *value, void *userData)
{
    (void)userData;

    if (value == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    switch (index) {
    case WT_WIDGET_IDX_START_SWITCH:
        *value = s_startRequested ? DJI_WIDGET_SWITCH_STATE_ON
                                  : DJI_WIDGET_SWITCH_STATE_OFF;
        break;
    case WT_WIDGET_IDX_DRY_RUN:
        *value = s_dryRun ? DJI_WIDGET_SWITCH_STATE_ON : DJI_WIDGET_SWITCH_STATE_OFF;
        break;
    /* 按钮没有"当前值"可读 —— 它是瞬时动作，不是状态。
     * 回 `RELEASE_UP` 让界面显示为未按下，与"按一下就弹回"的物理直觉一致。 */
    case WT_WIDGET_IDX_ABORT_BUTTON:
        if (widgetType != DJI_WIDGET_TYPE_BUTTON) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
        }
        *value = DJI_WIDGET_BUTTON_STATE_RELEASE_UP;
        break;
    /* 打开界面时要把当前值推给 Pilot —— 不实现的话它会显示 0，
     * 与主循环实际用的配置值不一致。 */
    case WT_WIDGET_IDX_PARK_PHASE:
        if (widgetType != DJI_WIDGET_TYPE_INT_INPUT_BOX) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
        }
        *value = (s_parkPhaseDeg < 0) ? 0 : s_parkPhaseDeg;
        break;
    default:
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static const T_DjiWidgetHandlerListItem s_widgetHandlerList[WT_WIDGET_COUNT] = {
    /* ---- main_interface ---- */
    {WT_WIDGET_IDX_START_SWITCH, DJI_WIDGET_TYPE_SWITCH, WtWidget_SetWidgetValue,
     WtWidget_GetWidgetValue, NULL},
    {WT_WIDGET_IDX_ABORT_BUTTON, DJI_WIDGET_TYPE_BUTTON, WtWidget_SetWidgetValue,
     WtWidget_GetWidgetValue, NULL},
    {WT_WIDGET_IDX_DRY_RUN,      DJI_WIDGET_TYPE_SWITCH, WtWidget_SetWidgetValue,
     WtWidget_GetWidgetValue, NULL},

    /* ---- config_interface（Payload Settings）----
     * ⚠️ handler 列表**不区分界面**，是一张扁平表 —— 所以两边的 index
     * 必须共用一个编号序列（理由见上面的索引定义处）。
     *
     * 类型必须与 json 里的 "widget_type": "int_input_box" 一致，
     * 否则 PSDK 分派时会因类型不匹配拒绝（回调里我们对 widgetType 做了校验）。 */
    {WT_WIDGET_IDX_PARK_PHASE,   DJI_WIDGET_TYPE_INT_INPUT_BOX, WtWidget_SetWidgetValue,
     WtWidget_GetWidgetValue, NULL},
};

/* ------------------------------------------------------------------ */
/* 浮窗消息                                                            */
/* ------------------------------------------------------------------ */

void WtWidget_PostMessage(const char *fmt, ...)
{
    va_list ap;

    if (fmt == NULL) {
        return;
    }
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
 * 作业进行中需要**周期性**汇报状态，所以要有自己的心跳线程。
 */
static void *WtWidget_MsgTask(void *arg)
{
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    char lastMsg[WT_MSG_MAX_LEN] = {0};

    (void)arg;

    if (osal == NULL) {
        return NULL;
    }

    while (s_msgTaskRun) {
        /* 只在消息变化时才发 —— 避免刷屏，也省带宽（上限 2 KB/s） */
        if (strcmp(lastMsg, s_msgBuf) != 0) {
            const T_DjiReturnCode rc = DjiWidgetFloatingWindow_ShowMessage(s_msgBuf);

            /* ⚠️ 只有**发送成功**才把它记为"已发"。
             *
             * 早先的写法是发完就无条件 memcpy，于是发送失败（通道未就绪）会让
             * 这条消息**永远不再重发**：去重表认为它已经发过了。
             * 浮窗是室外唯一反馈通道，"启动被拒"丢了就等于没有反馈。 */
            if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                memcpy(lastMsg, s_msgBuf, sizeof(lastMsg) - 1);
                lastMsg[sizeof(lastMsg) - 1] = '\0';
            }
            /* 失败时不动 lastMsg，下一拍自然重试同一条。 */
        }
        osal->TaskSleepMs(WT_MSG_PERIOD_MS);
    }

    return NULL;
}

/* ------------------------------------------------------------------ */
/* 初始化                                                              */
/* ------------------------------------------------------------------ */

/**
 * @brief 找到控件配置目录，写进 out
 *
 * 为什么不能写死一个路径：**PSDK 的路径是相对进程 CWD 解析的**，而 CWD 在
 * 两种运行方式下不同 ——
 *
 *   手工调试：`cd /open_app/wt-inspection && ./bin/wt_inspection_app`
 *   装成 dpk：`dji_app_ctl start` 把 CWD 设在**包根** → /open_app/<app>/
 *
 * 所以两个候选都要试，顺序是"装包优先"：
 *   1. `widget_file/...`      —— dpk 布局（`app.json` 的 userconfig 把它拷到包根）
 *   2. `app/widget_file/...`  —— 源码树里手工跑
 */
static const char *wt_widget_resolve_dir(const char *langDir)
{
    static char resolved[256];
    const char *candidates[] = {
        "widget_file",       /* dpk 安装后：CWD = 包根 */
        "app/widget_file",   /* 源码树里手工跑 */
    };
    size_t i;
    struct stat st;

    for (i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
        snprintf(resolved, sizeof(resolved), "%s/%s", candidates[i], langDir);
        if (stat(resolved, &st) == 0 && S_ISDIR(st.st_mode)) {
            USER_LOG_INFO("控件配置目录: %s（相对 CWD %s）", resolved, candidates[i]);
            return resolved;
        }
    }

    /* 都没找到：返回第一个候选，让 SDK 报出它自己的错，
     * 同时把 CWD 打进日志 —— 这类"文件找不到"没有 CWD 根本没法排查。 */
    {
        char cwd[128] = {0};

        /* getcwd 带 warn_unused_result：显式消费返回值 */
        if (getcwd(cwd, sizeof(cwd) - 1) == NULL) {
            snprintf(cwd, sizeof(cwd), "(读取失败)");
        }
        USER_LOG_ERROR("找不到控件配置目录 %s/*（当前 CWD = %s）", langDir, cwd);
    }
    snprintf(resolved, sizeof(resolved), "%s/%s", candidates[0], langDir);

    return resolved;
}

T_DjiReturnCode WtWidget_Init(void)
{
    T_DjiReturnCode rc;
    T_DjiOsalHandler *osal;

    rc = DjiWidget_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiWidget_Init 失败 rc=0x%08X", (unsigned)rc);
        return rc;
    }

    /* UI 配置：两份（中/英），控件类型/索引/数量必须一致，否则行为未定义 */
    rc = DjiWidget_RegDefaultUiConfigByDirPath(wt_widget_resolve_dir("cn_big_screen"));
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("注册默认控件配置失败 rc=0x%08X", (unsigned)rc);
        return rc;
    }

    rc = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_CHINESE,
                                        DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN,
                                        wt_widget_resolve_dir("cn_big_screen"));
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("注册中文控件配置失败 rc=0x%08X", (unsigned)rc);
        return rc;
    }

    rc = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_ENGLISH,
                                        DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN,
                                        wt_widget_resolve_dir("en_big_screen"));
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("注册英文控件配置失败 rc=0x%08X", (unsigned)rc);
        return rc;
    }

    rc = DjiWidget_RegHandlerList(s_widgetHandlerList, WT_WIDGET_COUNT);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("注册控件 handler 失败 rc=0x%08X", (unsigned)rc);
        return rc;
    }

    s_msgTaskRun = true;
    osal = DjiPlatform_GetOsalHandler();
    if (osal == NULL) {
        s_msgTaskRun = false;
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    rc = osal->TaskCreate("wt_widget_msg", WtWidget_MsgTask, 4096, NULL, &s_msgTask);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        s_msgTaskRun = false;
        USER_LOG_ERROR("创建控件消息线程失败 rc=0x%08X", (unsigned)rc);
        return rc;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void WtWidget_Stop(void)
{
    T_DjiOsalHandler *osal;

    s_msgTaskRun = false;

    osal = DjiPlatform_GetOsalHandler();
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

bool WtWidget_IsStartRequested(void)
{
    return s_startRequested;
}

bool WtWidget_TakeAbortRequest(void)
{
    if (!s_abortPending) {
        return false;
    }
    s_abortPending = false;

    return true;
}

double WtWidget_GetParkPhaseDeg(void)
{
    /* 读时再夹一次：s_parkPhaseDeg 由回调线程写，主循环可能读到回调
     * 尚未执行时的初值。夹取是纯函数、开销可忽略，
     * 不做"上游一定守规矩"的假设。
     *
     * 负值原样返回 —— 那是"未知"语义，由运行器判风险并拒绝自动启动。 */
    if (s_parkPhaseDeg < 0) {
        return -1.0;
    }

    return (double)wt_clamp_park_phase(s_parkPhaseDeg);
}

void WtWidget_SetParkPhaseInitial(double deg)
{
    if (deg < 0.0) {
        s_parkPhaseDeg = -1;    /* 配置里未提供，保持"未知" */
        return;
    }
    s_parkPhaseDeg = wt_clamp_park_phase((int32_t)(deg + 0.5));
}

bool WtWidget_IsDryRun(void)
{
    return s_dryRun;
}

void WtWidget_SetDryRunInitial(bool dryRun)
{
    s_dryRun = dryRun;
}

void WtWidget_ClearStartRequest(void)
{
    s_startRequested = false;
}
