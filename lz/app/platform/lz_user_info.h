/**
 * @file lz_user_info.h
 * @brief 从构建期生成的 `dji_sdk_app_info.h` 填充 `T_DjiUserInfo`。
 *
 * 移植自官方样例的 `DjiUser_FillInUserInfo()`，逻辑未改。
 *
 * ⚠️ 官方原版在凭据是占位符时 `return INVALID_PARAMETER` —— **启动即退出**。
 * 那正是"`.dpk` 安装器试运行应用"踩的坑（见仓库级 CLAUDE.md）。
 * 但因为凭据是**构建期注入**的，占位符只可能出现在"开发者没配 ini"的情况，
 * 属于开发期错误而非运行期故障，这里保留官方行为以便及早暴露。
 * 真正要防的是"配置缺失就退出" —— 那类防御必须在**作业开始**处划界，
 * 不是进程启动处。
 */

#ifndef LZ_USER_INFO_H
#define LZ_USER_INFO_H

#include "dji_core.h"

/** @brief 用构建期注入的凭据填充 userInfo；占位符或超长时返回非 SUCCESS */
T_DjiReturnCode LzUserInfo_Fill(T_DjiUserInfo *userInfo);

#endif /* LZ_USER_INFO_H */
