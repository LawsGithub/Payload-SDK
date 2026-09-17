/**
 * @file main.c
 * @brief 风机叶片巡检 —— PSDK 机载应用入口（M4T + 妙算3）
 *
 * 平台注册部分刻意与官方 manifold3 样例保持一致：OSAL / USB Bulk /
 * Socket / FileSystem 四个处理器 + 控制台与文件两路日志。这样做的原因
 * 是平台层一旦出问题，现象往往是"飞机连不上"，很难定位；保持与官方
 * 完全一致可以把这类问题排除在外，把注意力留给业务逻辑。
 */

#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <dji_aircraft_info.h>
#include <dji_core.h>
#include <dji_logger.h>
#include <dji_platform.h>
#include <dji_typedef.h>

#include "dji_sdk_app_info.h"
#include "hal_usb_bulk.h"
#include "osal/osal.h"
#include "osal/osal_fs.h"
#include "osal/osal_socket.h"
#include "wt_runner.h"

#define WT_DEFAULT_CONFIG_PATH "/data/wt_inspection/wt_config.ini"
#define WT_LOG_PATH            "data/logs/WT"
#define WT_LOG_PATH_MAX_SIZE   128

static WtRunner s_runner;
static volatile sig_atomic_t s_stopRequested = 0;
static FILE *s_logFile = NULL;

static T_DjiReturnCode WtApp_PrintConsole(const uint8_t *data, uint16_t dataLen)
{
    printf("%.*s", dataLen, (char *)data);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode WtApp_LocalWrite(const uint8_t *data, uint16_t dataLen)
{
    if (s_logFile == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    if (fwrite(data, 1, dataLen, s_logFile) != dataLen) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    fflush(s_logFile);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode WtApp_LocalWriteFsInit(const char *path)
{
    char filePath[WT_LOG_PATH_MAX_SIZE];
    char directory[WT_LOG_PATH_MAX_SIZE];
    char *p;

    snprintf(directory, sizeof(directory), "%.*s", (int)sizeof(directory) - 1, path);

    /* 逐级创建日志目录，避免日志系统因目录缺失而静默失效 */
    for (p = directory + 1; *p != '\0'; p++) {
        if (*p == '/') {
            *p = '\0';
            mkdir(directory, 0755);
            *p = '/';
        }
    }
    mkdir(directory, 0755);

    snprintf(filePath, sizeof(filePath), "%s/log_%d.txt", path, getpid());
    s_logFile = fopen(filePath, "w");
    if (s_logFile == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode WtApp_PrepareSystemEnvironment(void)
{
    T_DjiReturnCode rc;

    T_DjiOsalHandler osalHandler = {
        .TaskCreate = Osal_TaskCreate,
        .TaskDestroy = Osal_TaskDestroy,
        .TaskSleepMs = Osal_TaskSleepMs,
        .MutexCreate = Osal_MutexCreate,
        .MutexDestroy = Osal_MutexDestroy,
        .MutexLock = Osal_MutexLock,
        .MutexUnlock = Osal_MutexUnlock,
        .SemaphoreCreate = Osal_SemaphoreCreate,
        .SemaphoreDestroy = Osal_SemaphoreDestroy,
        .SemaphoreWait = Osal_SemaphoreWait,
        .SemaphoreTimedWait = Osal_SemaphoreTimedWait,
        .SemaphorePost = Osal_SemaphorePost,
        .Malloc = Osal_Malloc,
        .Free = Osal_Free,
        .GetRandomNum = Osal_GetRandomNum,
        .GetTimeMs = Osal_GetTimeMs,
        .GetTimeUs = Osal_GetTimeUs,
    };

    T_DjiHalUsbBulkHandler usbBulkHandler = {
        .UsbBulkInit = HalUsbBulk_Init,
        .UsbBulkDeInit = HalUsbBulk_DeInit,
        .UsbBulkWriteData = HalUsbBulk_WriteData,
        .UsbBulkReadData = HalUsbBulk_ReadData,
        .UsbBulkGetDeviceInfo = HalUsbBulk_GetDeviceInfo,
    };

    T_DjiFileSystemHandler fileSystemHandler = {
        .FileOpen = Osal_FileOpen,
        .FileClose = Osal_FileClose,
        .FileWrite = Osal_FileWrite,
        .FileRead = Osal_FileRead,
        .FileSync = Osal_FileSync,
        .FileSeek = Osal_FileSeek,
        .DirOpen = Osal_DirOpen,
        .DirClose = Osal_DirClose,
        .DirRead = Osal_DirRead,
        .Mkdir = Osal_Mkdir,
        .Unlink = Osal_Unlink,
        .Rename = Osal_Rename,
        .Stat = Osal_Stat,
    };

    T_DjiSocketHandler socketHandler = {
        .Socket = Osal_Socket,
        .Bind = Osal_Bind,
        .Close = Osal_Close,
        .UdpSendData = Osal_UdpSendData,
        .UdpRecvData = Osal_UdpRecvData,
        .TcpListen = Osal_TcpListen,
        .TcpAccept = Osal_TcpAccept,
        .TcpConnect = Osal_TcpConnect,
        .TcpSendData = Osal_TcpSendData,
        .TcpRecvData = Osal_TcpRecvData,
    };

    T_DjiLoggerConsole printConsole = {
        .func = WtApp_PrintConsole,
        .consoleLevel = DJI_LOGGER_CONSOLE_LOG_LEVEL_INFO,
        .isSupportColor = true,
    };

    T_DjiLoggerConsole fileConsole = {
        .func = WtApp_LocalWrite,
        .consoleLevel = DJI_LOGGER_CONSOLE_LOG_LEVEL_DEBUG,
        .isSupportColor = false,
    };

    rc = DjiPlatform_RegOsalHandler(&osalHandler);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("注册 OSAL 处理器失败\n");
        return rc;
    }

    if (WtApp_LocalWriteFsInit(WT_LOG_PATH) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("日志目录初始化失败\n");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    DjiLogger_AddConsole(&printConsole);
    DjiLogger_AddConsole(&fileConsole);

    rc = DjiPlatform_RegHalUsbBulkHandler(&usbBulkHandler);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("注册 USB Bulk 处理器失败\n");
        return rc;
    }

    /* 低速数据通道（接收地面站下发的停用角、启停指令）依赖 socket */
    rc = DjiPlatform_RegSocketHandler(&socketHandler);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("注册 Socket 处理器失败\n");
        return rc;
    }

    rc = DjiPlatform_RegFileSystemHandler(&fileSystemHandler);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("注册文件系统处理器失败\n");
        return rc;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode WtApp_FillUserInfo(T_DjiUserInfo *userInfo)
{
    if (userInfo == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    memset(userInfo, 0, sizeof(*userInfo));

    if (strlen(USER_APP_NAME) >= sizeof(userInfo->appName) ||
        strlen(USER_APP_ID) >= sizeof(userInfo->appId) ||
        strlen(USER_APP_KEY) >= sizeof(userInfo->appKey) ||
        strlen(USER_APP_LICENSE) >= sizeof(userInfo->appLicense) ||
        strlen(USER_DEVELOPER_ACCOUNT) >= sizeof(userInfo->developerAccount)) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    /* 未填写真实信息时直接拒绝启动：带占位符去连飞机只会得到一个含糊的失败 */
    if (strcmp(USER_APP_ID, "your_app_id") == 0) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    /*
     * 用 snprintf 而不是 strncpy：这里拷的都是编译期常量，长度已经在上面的
     * 分支里卡过一遍，但 strncpy 在「源串长度恰好等于 n-1」时不会补结尾的
     * '\0' —— 正确性要靠前面那句 memset 兜着。换成 snprintf 后这个不变量
     * 由函数本身保证，不再依赖调用顺序。
     */
    snprintf(userInfo->appName, sizeof(userInfo->appName), "%s", USER_APP_NAME);
    snprintf(userInfo->appId, sizeof(userInfo->appId), "%s", USER_APP_ID);
    snprintf(userInfo->appKey, sizeof(userInfo->appKey), "%s", USER_APP_KEY);
    snprintf(userInfo->appLicense, sizeof(userInfo->appLicense), "%s", USER_APP_LICENSE);
    snprintf(userInfo->developerAccount, sizeof(userInfo->developerAccount), "%s",
             USER_DEVELOPER_ACCOUNT);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static void WtApp_SignalHandler(int signalNum)
{
    (void)signalNum;

    /*
     * 信号处理函数里只允许做异步信号安全的操作：置位。
     * 返航、暂停航线这些调用都放到主循环里执行，否则可能死锁。
     */
    s_stopRequested = 1;
    WtRunner_RequestAbort(&s_runner);
}

int main(int argc, char **argv)
{
    const char *configPath = (argc > 1) ? argv[1] : WT_DEFAULT_CONFIG_PATH;
    T_DjiUserInfo userInfo;
    T_DjiAircraftInfoBaseInfo baseInfo;
    T_DjiFirmwareVersion firmwareVersion = {1, 0, 0, 0};
    T_DjiReturnCode rc;
    int exitCode = 0;

    signal(SIGINT, WtApp_SignalHandler);
    signal(SIGTERM, WtApp_SignalHandler);

    printf("==================================================\n");
    printf("  风机叶片巡检机载应用   M4T + 妙算3\n");
    printf("  作业模式：粗模环绕 -> 精细巡检（两阶段）\n");
    printf("==================================================\n");

    /* 步骤 1：平台环境 */
    if (WtApp_PrepareSystemEnvironment() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return 1;
    }

    /* 步骤 2：应用信息 */
    if (WtApp_FillUserInfo(&userInfo) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("应用信息无效，请填写 dji_sdk_app_info.h");
        return 1;
    }

    /* 步骤 3：PSDK 核心 */
    rc = DjiCore_Init(&userInfo);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiCore_Init 失败: 0x%08X", rc);
        return 1;
    }

    /*
     * 机型自检放在最前面：本方案依赖 M4T 系列的内置三摄与 E-Port 载荷。
     * 挂错机型却一路初始化到航点下发才报错，是最浪费时间的一种失败。
     */
    rc = DjiAircraftInfo_GetBaseInfo(&baseInfo);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("获取机型信息失败，无法确认硬件是否匹配");
        DjiCore_DeInit();
        return 1;
    }
    if (baseInfo.aircraftType != DJI_AIRCRAFT_TYPE_M4T &&
        baseInfo.aircraftType != DJI_AIRCRAFT_TYPE_M4E &&
        baseInfo.aircraftType != DJI_AIRCRAFT_TYPE_M4TD) {
        USER_LOG_ERROR("当前机型 %d 不在支持范围（M4T / M4E / M4TD）",
                       (int)baseInfo.aircraftType);
        DjiCore_DeInit();
        return 1;
    }
    USER_LOG_INFO("机型检查通过，aircraftType = %d", (int)baseInfo.aircraftType);

    DjiCore_SetAlias("WT_BLADE_INSPECT");
    DjiCore_SetFirmwareVersion(firmwareVersion);
    DjiCore_SetSerialNumber("WTINSPECT000001");

    rc = DjiCore_ApplicationStart();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiCore_ApplicationStart 失败: 0x%08X", rc);
        DjiCore_DeInit();
        return 1;
    }

    /*
     * 步骤 4：作业配置。缺文件时落一份模板，避免"用了默认值却不自知"。
     *
     * 全新设备上 /data/wt_inspection/ 并不存在，而 fopen 不会替你建目录 ——
     * 必须先建目录，否则 WriteTemplate 静默失败，"自动生成模板"就成了空话，
     * 操作员只看到"配置文件不存在"却拿不到可填的东西。落完模板退出，让现场
     * 填好参数再重启：带着模板里的样例坐标起飞比不飞更糟。
     */
    {
        FILE *probe = fopen(configPath, "r");

        if (probe == NULL) {
            char dir[WT_CONFIG_MAX_PATH];

            if (WtRunner_DirName(configPath, dir, sizeof(dir))) {
                WtRunner_MakeDirs(dir);
            }

            if (WtAppConfig_WriteTemplate(configPath)) {
                USER_LOG_WARN("配置文件 %s 不存在，已生成模板，请填写后重启", configPath);
            } else {
                USER_LOG_ERROR("配置文件 %s 不存在，且模板生成失败（目录不可写？）",
                               configPath);
            }
            DjiCore_DeInit();
            return 1;
        }
        fclose(probe);
    }

    if (WtRunner_Init(&s_runner, configPath) != WT_RUN_OK) {
        USER_LOG_ERROR("运行器初始化失败");
        DjiCore_DeInit();
        return 1;
    }

    if (!WtTelemetry_Init(&s_runner.telemetry,
                          s_runner.config.turbineCount > 0
                              ? &s_runner.config.turbines[0].spec
                              : NULL)) {
        USER_LOG_ERROR("遥测初始化失败");
        WtRunner_DeInit(&s_runner);
        DjiCore_DeInit();
        return 1;
    }

    /* 步骤 5：逐台风机作业 */
    {
        int i;

        for (i = 0; i < s_runner.config.turbineCount && !s_stopRequested; i++) {
            const char *name = s_runner.config.turbines[i].name;
            /*
             * 停用角正常应由地面站经低速数据通道下发；此处传负值表示"未知"，
             * 运行器会给出告警并禁止自动启动，必须人工确认后再执行。
             */
            double parkPhase = -1.0;
            WtRunResult rr;

            USER_LOG_INFO("---- 开始作业：%s ----", name);
            rr = WtRunner_RunTurbine(&s_runner, name, parkPhase);

            if (rr != WT_RUN_OK) {
                USER_LOG_ERROR("风机 %s 作业未完成，结果码 %d", name, (int)rr);
                exitCode = (int)rr;
                if (!s_runner.config.autoStart) {
                    break; /* 非自动模式下出错即停，等待人工介入 */
                }
            }
        }
    }

    WtTelemetry_DeInit(&s_runner.telemetry);
    WtRunner_DeInit(&s_runner);
    DjiCore_DeInit();

    if (s_logFile != NULL) {
        fclose(s_logFile);
    }

    printf("[app] 退出，返回码 %d\n", exitCode);

    return exitCode;
}