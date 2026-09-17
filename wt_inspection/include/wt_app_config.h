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

    /* ---- 机载运行参数 ---- */
    double telemetryHz;      /*!< 遥测订阅频率 Hz */
    double gimbalTrackGain;  /*!< 云台实时跟踪增益，0 = 不跟踪 */
    double trackDeadbandDeg; /*!< 跟踪死区，小于该角度不动作，避免云台抖动 */
    bool autoStart;          /*!< 上电后是否自动开始（false 则等待遥控器指令） */
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