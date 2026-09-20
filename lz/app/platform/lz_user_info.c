/**
 * @file lz_user_info.c
 * @brief 凭据填充实现（移植自官方样例，逻辑未改）。
 */

#include "lz_user_info.h"

#include <dji_logger.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>

/* 由 CMake 在构建期从 lz_credentials.ini 生成到构建目录，
 * 并放在 include 路径最前 —— 遮蔽官方样例里的同名占位文件 */
#include "dji_sdk_app_info.h"

#define LZ_MIN(a, b) (((a) < (b)) ? (a) : (b))

T_DjiReturnCode LzUserInfo_Fill(T_DjiUserInfo *userInfo)
{
    if (userInfo == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    memset(userInfo->appName, 0, sizeof(userInfo->appName));
    memset(userInfo->appId, 0, sizeof(userInfo->appId));
    memset(userInfo->appKey, 0, sizeof(userInfo->appKey));
    memset(userInfo->appLicense, 0, sizeof(userInfo->appLicense));
    memset(userInfo->developerAccount, 0, sizeof(userInfo->developerAccount));
    memset(userInfo->baudRate, 0, sizeof(userInfo->baudRate));

    /* 长度检查：T_DjiUserInfo 的字段是定长数组，超了会被截断，
     * 截断的凭据拿去校验只会得到一个含糊的失败。
     * 注意 appName / developerAccount 是 `>=`（要留 '\0'），其余是 `>`。 */
    if (strlen(USER_APP_NAME) >= sizeof(userInfo->appName) ||
        strlen(USER_APP_ID) > sizeof(userInfo->appId) ||
        strlen(USER_APP_KEY) > sizeof(userInfo->appKey) ||
        strlen(USER_APP_LICENSE) > sizeof(userInfo->appLicense) ||
        strlen(USER_DEVELOPER_ACCOUNT) >= sizeof(userInfo->developerAccount) ||
        strlen(USER_BAUD_RATE) > sizeof(userInfo->baudRate)) {
        USER_LOG_ERROR("Length of user information string is beyond limit. Please check.");
        sleep(1);
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    /* 占位符检测：构建期没读到 lz_credentials.ini 时会是这些值 */
    if (!strcmp(USER_APP_NAME, "your_app_name") ||
        !strcmp(USER_APP_ID, "your_app_id") ||
        !strcmp(USER_APP_KEY, "your_app_key") ||
        !strcmp(USER_APP_LICENSE, "your_app_license") ||
        !strcmp(USER_DEVELOPER_ACCOUNT, "your_developer_account") ||
        !strcmp(USER_BAUD_RATE, "your_baud_rate")) {
        USER_LOG_ERROR("凭据仍是占位符 —— 请创建 lz/lz_credentials.ini 后重新构建。"
                       "（见 lz/cmake/gen_app_info.cmake 的说明）");
        sleep(1);
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    strncpy(userInfo->appName, USER_APP_NAME, sizeof(userInfo->appName) - 1);
    memcpy(userInfo->appId, USER_APP_ID, LZ_MIN(sizeof(userInfo->appId), strlen(USER_APP_ID)));
    memcpy(userInfo->appKey, USER_APP_KEY, LZ_MIN(sizeof(userInfo->appKey), strlen(USER_APP_KEY)));
    memcpy(userInfo->appLicense, USER_APP_LICENSE,
           LZ_MIN(sizeof(userInfo->appLicense), strlen(USER_APP_LICENSE)));
    memcpy(userInfo->baudRate, USER_BAUD_RATE, LZ_MIN(sizeof(userInfo->baudRate), strlen(USER_BAUD_RATE)));
    strncpy(userInfo->developerAccount, USER_DEVELOPER_ACCOUNT, sizeof(userInfo->developerAccount) - 1);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
