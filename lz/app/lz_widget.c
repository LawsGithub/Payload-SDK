/**
 * @file lz_widget.c
 * @brief 控件模块实现。
 *
 * 骨架取自官方样例 `samples/sample_c/module_sample/widget/test_widget.c`，
 * 砍到只剩本项目需要的**五个**控件。**初始化四步的顺序不能变**：
 *
 *   1. DjiWidget_Init()
 *   2. 注册 UI 配置（Linux 下用 ByDirPath；RTOS 下用 ByBinaryArray）
 *   3. DjiWidget_RegHandlerList()  ← 必须在 ApplicationStart 之前
 *   4. 建状态推送线程
 */

#include "lz_widget.h"

/* 滑杆的映射区间取自规划层 —— 见文件下方"映射区间不在这里定义"的说明。
 * 包含 lz_plan.h 而不是 lz_plan.c：常量定义在头文件里，零依赖，
 * 不破坏 lz_app 与 lz_core 的分层（方向本来就是 lz_app → lz_core）。 */
#include "lz_plan.h"
/* 两个记录按钮要用：LzBridge_GetCurrentPosition（飞机位置，含 rad→度换算）
 * 与 LzPole_RecordAircraft / LzPole_RecordLaser。 */
#include "lz_bridge_psdk.h"
#include "lz_pole_source.h"

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
/* 这 5 个索引必须与两份 json 的 widget_index 一一对应 —— PSDK 按索引     */
/* 把界面控件分派给 handler，对不上的后果是"按了 A 按钮却执行了 B 的       */
/* 动作"，而两边都不会报错。                                             */
/* ------------------------------------------------------------------ */
#define LZ_WIDGET_IDX_ORBIT_SWITCH 0
#define LZ_WIDGET_IDX_RADIUS_SCALE 1
#define LZ_WIDGET_IDX_HEIGHT_SCALE 2
#define LZ_WIDGET_IDX_RECORD_AIRCRAFT 3
#define LZ_WIDGET_IDX_RECORD_LASER    4
#define LZ_WIDGET_COUNT               5

/* 范围条是 0–100 的百分比，这里映射到实际物理量。
 *
 * ⚠️ **映射区间不在这里定义** —— 用 `lz_plan.h` 的 `LZ_PLAN_RADIUS_*` /
 * `LZ_PLAN_ALTITUDE_*`。那是规划层安全校验用的**同一组**常量。
 *
 * 为什么不各写一份：先前控件层自备 30 m / 150 m 的上限，而 `LzPlan_Validate`
 * 完全看不到它 —— 结果会出现"滑杆拨到底给 30 m，但校验按 20 m 拒绝"
 * 这种界面与校验打架的情况。它只在起飞前才暴露，操作员看到的是
 * "拨了开关但飞机不动"，无从判断是控件错了还是校验错了。
 *
 * ⚠️ 高度是**相对起飞点**（KMZ 里 executeHeightMode=relativeToStartPoint），
 *    不是绝对海拔。要飞真正的 ASL 高度得改高度模式并处理大地水准面差距，
 *    那是另一回事。 */

#define LZ_MSG_MAX_LEN  256
#define LZ_MSG_PERIOD_MS 500    /* 浮窗消息推送周期，远低于 2 KB/s 上限 */

/* ------------------------------------------------------------------ */
/* 状态                                                                */
/* ------------------------------------------------------------------ */
static bool s_orbitRequested = false;
/* 默认档位。换算见下方 LzWidget_GetRadiusM/GetAltitudeM ——
 * 区间收窄到 5–20 m / 5–120 m 后，50% → 12.5 m、66% → 约 80.9 m。
 * 这两个值**不是**"设计目标值"，只是出厂默认，操作员可拨。 */
static int32_t s_radiusPercent = 50;
static int32_t s_heightPercent = 66;
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

    /* ---- 两个"记录绕飞圆心"按钮 ----
     *
     * ⚠️ button 的 `value` 是 `E_DjiWidgetButtonState`：**按下与松开各触发一次**
     * 回调（`PRESS_DOWN=1` / `RELEASE_UP=0`）。只在按下时记录 ——
     * 不判的话一次点击会被记两遍（虽然结果相同，但日志会出现两条"已记录"，
     * 让操作员以为按了两次）。 */
    case LZ_WIDGET_IDX_RECORD_AIRCRAFT:
    case LZ_WIDGET_IDX_RECORD_LASER: {
        if (widgetType != DJI_WIDGET_TYPE_BUTTON) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
        }
        if (value != DJI_WIDGET_BUTTON_STATE_PRESS_DOWN) {
            break;   /* 忽略松开的半次 */
        }

        LzStatus st;
        if (index == LZ_WIDGET_IDX_RECORD_AIRCRAFT) {
            /* 飞机位置由桥接层给（它负责 rad→度 的换算与零解拦截）。
             * 单位换错会让记录的杆位跑到几内亚湾，而坐标看上去完全合法。 */
            LzGeo cur;
            st = LzBridge_GetCurrentPosition(&cur);
            if (st == LZ_OK) {
                st = LzPole_RecordAircraft(&cur);
            }
            if (st == LZ_OK) {
                LzWidget_PostMessage("✓ 已记录飞机位为圆心：%.7f, %.7f",
                                     cur.latitudeDeg, cur.longitudeDeg);
            } else if (st == LZ_ERR_NOT_READY) {
                LzWidget_PostMessage("✗ 记录失败：还没有飞机定位数据，请稍候再按");
            } else if (st == LZ_ERR_NO_TARGET) {
                LzWidget_PostMessage("✗ 记录失败：当前没有定位（等 GPS 锁定后再按）");
            } else {
                LzWidget_PostMessage("✗ 记录失败：%s", LzStatus_Str(st));
            }
        } else {
            st = LzPole_RecordLaser();
            if (st == LZ_OK) {
                LzGeo g;
                if (LzPole_GetRecorded(&g) == LZ_OK) {
                    LzWidget_PostMessage("✓ 已记录激光点为圆心：%.7f, %.7f",
                                         g.latitudeDeg, g.longitudeDeg);
                } else {
                    LzWidget_PostMessage("✓ 已记录激光点");
                }
            } else if (st == LZ_ERR_UNSUPPORTED) {
                /* 措辞面向**操作员**，不是开发者 —— 现场不需要知道
                 * 什么编译开关。只说"这个包没有这功能，用另一个按钮"，
                 * 操作员立刻知道该怎么办。 */
                LzWidget_PostMessage("✗ 本包未启用激光记录，请改用「记录飞机位」");
            } else if (st == LZ_ERR_NO_TARGET) {
                LzWidget_PostMessage("✗ 记录失败：激光无回波，请对准目标再按");
            } else {
                LzWidget_PostMessage("✗ 记录失败：%s", LzStatus_Str(st));
            }
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
    /* 按钮没有"当前值"可读 —— 它是瞬时动作，不是状态。
     * 回 `RELEASE_UP` 让界面显示为未按下，与"按一下就弹回"的物理直觉一致。 */
    case LZ_WIDGET_IDX_RECORD_AIRCRAFT:
    case LZ_WIDGET_IDX_RECORD_LASER:
        if (widgetType != DJI_WIDGET_TYPE_BUTTON) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
        }
        *value = DJI_WIDGET_BUTTON_STATE_RELEASE_UP;
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
    /* 两个记录按钮。**索引必须与两份 widget_config.json 里的 widget_index 对上** ——
     * PSDK 按索引把界面控件分派给这里的 handler，对不上的后果是
     * "按了 A 按钮却执行了 B 的动作"，而两边都不会报错。 */
    {LZ_WIDGET_IDX_RECORD_AIRCRAFT, DJI_WIDGET_TYPE_BUTTON, LzWidget_SetWidgetValue, LzWidget_GetWidgetValue, NULL},
    {LZ_WIDGET_IDX_RECORD_LASER,    DJI_WIDGET_TYPE_BUTTON, LzWidget_SetWidgetValue, LzWidget_GetWidgetValue, NULL},
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
            const T_DjiReturnCode rc = DjiWidgetFloatingWindow_ShowMessage(s_msgBuf);

            /* ⚠️ 只有**发送成功**才把它记为"已发"。
             *
             * 早先的写法是发完就无条件 memcpy(lastMsg, ...)，于是
             * 一次发送失败（通道未就绪 —— 头文件只说"发到浮窗"，不承诺成功）
             * 会让这条消息**永远不再重发**：去重表认为它已经发过了。
             *
             * 后果不对称，所以要特别小心：浮窗是本项目
             * **唯一**的操作员反馈通道（室外看不到 SDK 日志）。
             * 一条普通的"半径设为 15 m"丢了无所谓，但
             * "启动被拒：…" 丢了就等于没有反馈 ——
             * 而通道刚就绪的那几百毫秒恰好是最容易失败的时刻。 */
            if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                memcpy(lastMsg, s_msgBuf, sizeof(lastMsg) - 1);
                lastMsg[sizeof(lastMsg) - 1] = '\0';
            }
            /* 失败时不动 lastMsg，下一拍自然会重试同一条。
             * 不在这里 sleep 或重试 —— 本线程的周期是 500 ms，
             * 由它充当退避就够了。 */
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
    return lz_map_percent(s_radiusPercent, LZ_PLAN_RADIUS_MIN_M, LZ_PLAN_RADIUS_MAX_M);
}

double LzWidget_GetAltitudeM(void)
{
    return lz_map_percent(s_heightPercent, LZ_PLAN_ALTITUDE_MIN_M, LZ_PLAN_ALTITUDE_MAX_M);
}

/**
 * @brief 绕飞收尾：清本地意图 + 发一条结束消息
 *
 * ## ⚠️ 这里**做不到**把 Pilot 上的开关拨回去 —— 别再写成那样
 *
 * 曾经的实现是这样的（注释还写着"把开关程序化拨回 OFF"）：
 *
 *     (void)LzWidget_SetWidgetValue(DJI_WIDGET_TYPE_SWITCH, ..., OFF, NULL);
 *
 * 那是个**空头承诺**：`LzWidget_SetWidgetValue` 是**我们自己**注册给 PSDK 的
 * 回调，被 Pilot 调用时才生效；直接调它只会改本地变量、再发一条浮窗消息。
 * 它**不会**改变 Pilot 界面上的开关位置。
 *
 * 已核实 PSDK 没有反向推控件状态的接口：`dji_widget.h` 全部 9 个导出函数
 * 里没有任何 setter（Init / Reg*UiConfig* / RegHandlerList /
 * FloatingWindow_ShowMessage / FloatingWindow_GetChannelState /
 * RegSpeakerHandler）；`dji_widget_manager.h` 的 `DjiWidgetManager_SetWidgetState`
 * 目标是**机上挂载的负载**，不是本应用在 Pilot 上的 UI。
 *
 * ## 那实际会发生什么
 *
 * 操作员看到的是：**开关保持 ON**（持续可见），而浮窗飘过一行
 * "绕飞结束：…"（会被下一条消息覆盖）。两条信息互相矛盾，
 * 而持续可见的那条是**错的**。
 *
 * 所以这里改用**明确的持续措辞**：直接告诉操作员"开关仍在 ON 位，请手动拨回"。
 * 与其假装能回弹，不如把这个事实说清楚 —— 后者操作员能处理，前者会误导。
 *
 * 注意 `s_orbitRequested = false` 仍然要置：它保证状态机不会因为
 * 那个仍然 ON 的开关而**重新起飞**。也就是说开关的实际位置与本地意图
 * 从这里开始就是不一致的，这正是必须在界面上说明的原因。
 */
void LzWidget_ReportOrbitFinished(const char *reason)
{
    s_orbitRequested = false;

    /* 措辞里带上"请手动拨回"：
     *   - 操作员不拨回，下次拨 ON 时 `s_orbitRequested` 本来就是 false，
     *     回调会正常触发（值从 ON→OFF→ON，每次都有变化），功能不受影响；
     *   - 但界面显示的"ON"与"已经结束"会长期矛盾，所以必须提醒。 */
    LzWidget_PostMessage("绕飞结束：%s。⚠ 开关仍在 ON 位，请手动拨回",
                         reason ? reason : "（未说明）");
}
