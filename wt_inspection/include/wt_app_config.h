/**
 * @file wt_app_config.h
 * @brief 机载应用的作业配置载入
 *
 * 配置以纯文本 key=value 形式存放，部署在妙算3 的 /data/wt_inspection/wt_config.ini。
 * 之所以不用 JSON：机载端解析库越少越好，且该文件需要现场用记事本直接改，
 * 键值格式对运维人员最友好。
 */

#ifndef WT_APP_CONFIG_H
#define WT_APP_CONFIG_H

#include <stdbool.h>
#include <stddef.h>

#include "wt_plan.h"
#include "wt_turbine.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WT_CONFIG_MAX_PATH 256

/** 一个段落里允许出现的键名上限，供键名表使用 */
#define WT_CONFIG_MAX_KEYS 32

/** 段落分类。未知段落名与段内未知键的处理方式不同，需要先分类再判定 */
typedef enum {
    WT_CFG_SECTION_FIELD = 0, /*!< [field]     起飞点 */
    WT_CFG_SECTION_INSPECTION, /*!< [inspection] 作业剖面 */
    WT_CFG_SECTION_APP,        /*!< [app]        机载运行参数 */
    WT_CFG_SECTION_TURBINE,    /*!< [turbine.*]  风机台账 */
    WT_CFG_SECTION_UNKNOWN,    /*!< 其余段落名 */
} WtConfigSection;

/**
 * @brief 取某个段落允许出现的键名表
 *
 * 这份表是「键名是否合法」的唯一判据，也是配置模板必须与之一致的依据。
 * 把它暴露出来是为了让回归测试能锁住「表与解析实现不漂移」——
 * 一个键被从实现里删掉却留在表里（或反过来），现场就会遇到
 * 「模板里写了、程序说非法」或「拼错了却没人吭声」。
 *
 * @param section 段落分类
 * @param outCount 输出键名个数，可为 NULL
 * @return 键名数组（以 NULL 结尾的字符串数组的指针），未知段落返回 NULL
 */
const char *const *WtAppConfig_KnownKeys(WtConfigSection section, size_t *outCount);

/**
 * @brief 为一个疑似拼错的键名给出建议
 *
 * 只报「未识别的键」而不给出候选，现场仍然要对着模板逐字核对；给出最近的
 * 候选键名，绝大多数的笔误一眼就能看出来。
 *
 * @return 建议的键名；无可信建议（编辑距离过远）时返回 NULL
 */
const char *WtAppConfig_SuggestKey(WtConfigSection section, const char *key);

/** 一台风机的台账条目 */
typedef struct {
    char name[64];        /*!< 风机编号，如 WT-A01，用于照片命名与报告 */
    WtTurbineSpec spec;   /*!< 风机几何参数 */
    double parkPhaseDeg;  /*!< 停用角（停机位相位）；<0 表示现场未提供 */
} WtTurbineEntry;

/** 完整作业配置 */
typedef struct {
    /* ---- 风场 ---- */
    WtTurbineEntry *turbines;
    int turbineCount;

    /* ---- 起飞点 ---- */
    WtGeo takeoff;        /*!< 起飞点 WGS84 坐标 */

    /* ---- 作业剖面 ---- */
    WtInspectionProfile profile;

    /* ---- 沿中线飞行模式（trace）----
     *
     * 与 profile 里的「盘外站位斜拍」互斥的另一种航线形态：无人机沿叶片
     * 轴线平行飞过、叶尖绕半圈换面。traceEnabled 为 true 时规划走
     * WtPlan_BladeTrace，profile 的采样字段不再参与。
     */
    bool traceEnabled;       /*!< 是否启用沿中线飞行模式 */
    WtTraceProfile trace;    /*!< 该模式的参数 */

    /* ---- 机载运行参数 ---- */
    double telemetryHz;      /*!< 遥测订阅频率 Hz */
    double gimbalTrackGain;  /*!< 云台实时跟踪增益，0 = 不跟踪 */
    double trackDeadbandDeg; /*!< 跟踪死区，小于该角度不动作，避免云台抖动 */
    /*
     * 是否自动开始作业。**这是作业的总闸门**：
     *   false（默认）→ 进程起来、走完 SDK 身份校验、停在待命状态，不执行作业
     *   true          → 逐台风机执行「规划 → 上传 → 启动任务」
     *
     * 默认 false 有两个理由：安装器试运行应用时要它活着走完校验（自动开始
     * 作业会因等待飞控状态而挂住，导致装不上包）；以及"装上就自动起飞"
     * 本身是危险默认。
     */
    bool autoStart;
    /*
     * 干跑：只走「规划 → 生成 KMZ → 上传」，**不调 Action(START)**，飞机不飞。
     *
     * 用途是在没有现场/不方便真飞时验证飞控能否接受这条航线 —— 上传失败
     * 会把 KMZ 格式、坐标系、航向模式这类问题暴露出来，而这些恰恰是
     * 仿真器（PX4 等）验证不到的（飞控实现不同）。默认 false。
     */
    bool dryRun;
    /*
     * 诊断：生成**不含任何云台/相机动作**的航线。
     *
     * 只用于在 DJI Assistant 2 模拟器里排查「START 被拒」—— 模拟器只模拟
     * 飞控，不模拟云台和相机，带动作的航点任务会被拒
     * （实测 `Start waypoint v3 mission failed, error_code: 4`），
     * 分不清是几何问题还是动作问题。关掉动作能隔离这个变量。
     *
     * ⚠️ **真机作业绝不能开**：没有拍照动作的巡检航线毫无意义。
     * 默认 false，模板里以注释形式给出并写明这一点。
     */
    bool noActions;
    /*
     * 诊断：把固定的识别框发到 Pilot 2 画面上（PSDK liveview AI 叠加）。
     *
     * 只用于验证 `DjiLiveview_SendAiMetaToPilot` 在本机可用、以及自定义
     * 标签的 type 索引从几开始 —— 这两件事静态查不出来。开启后每秒发
     * 3 个框（type 0/1/2，位置左/中/右），在 Pilot 2 上观察哪些显示。
     *
     * ⚠️ **不是生产功能**：框的位置和标签都是写死的，跟实际识别无关。
     * 验证完必须关掉。默认 false。
     */
    bool aiOverlay;
    /*
     * 诊断：启动相机图像流并统计收到的帧（`DjiLiveview_StartImageStream`）。
     *
     * 用途是判定「持续跟踪」方案是否可行 —— 拿不到实时画面就谈不上
     * 「取流 → 推理 → 定位 → 规划」。与 `aiOverlay` 相互独立，可以只开一个。
     *
     * ⚠️ **不是生产功能**，只打日志。验证完关掉。默认 false。
     */
    bool streamProbe;
    char outputDir[WT_CONFIG_MAX_PATH]; /*!< 航线与照片索引的输出目录 */
} WtAppConfig;

/**
 * @brief 载入配置文件
 * @return true 表示解析成功且参数已通过校验
 */
bool WtAppConfig_Load(const char *path, WtAppConfig *config);

/** @brief 释放配置占用的内存 */
void WtAppConfig_Free(WtAppConfig *config);

/** @brief 按名称查找风机条目，找不到返回 NULL */
const WtTurbineEntry *WtAppConfig_FindTurbine(const WtAppConfig *config, const char *name);

/**
 * @brief 按名称查找风机条目，返回**可写**指针；找不到返回 NULL
 *
 * 与只读版并存而不是替换它：绝大多数调用点只需要读（规划、报告、校验），
 * 给它们 const 能挡住误写。只有"作业运行期由操作员改写参数"这一处
 * （控件改停用角）需要写权限，单独一个入口让意图显式。
 */
WtTurbineEntry *WtAppConfig_FindTurbineMutable(WtAppConfig *config, const char *name);

/** @brief 打印配置摘要，便于现场核对 */
void WtAppConfig_Dump(const WtAppConfig *config);

/**
 * @brief 写出配置模板
 *
 * 部署时若目标文件不存在，先落一份带注释的模板，避免"参数写错却悄悄用了默认值"。
 */
bool WtAppConfig_WriteTemplate(const char *path);

#ifdef __cplusplus
}
#endif

#endif /* WT_APP_CONFIG_H */