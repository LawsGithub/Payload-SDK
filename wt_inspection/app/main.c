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

/*
 * 默认配置路径：**相对应用目录**，不是绝对路径。
 *
 * 应用以 uid=1000（dji）运行，/data/ 不可写，绝对路径写在那里必然失败。
 * 官方打包脚本 build_dpk.sh 会在包内建好 data/logs 与 data/media，属主
 * dji:dji 可写 —— 这就是官方的「相对应用目录」约定，配置也放这里。
 * 该常量同时是 WtAppConfig 里默认 outputDir 的基准（见 wt_app_config.c）。
 */
#define WT_DEFAULT_CONFIG_PATH "data/wt_config.ini"
#define WT_LOG_PATH            "data/logs/WT"
#define WT_LOG_PATH_MAX_SIZE   128

static WtRunner s_runner;
static volatile sig_atomic_t s_stopRequested = 0;
static FILE *s_logFile = NULL;
/*
 * 配置是否就绪。**它不是启动门槛，只是作业门槛** —— 配置缺失时进程仍要
 * 活着走完 SDK 身份校验，否则 .dpk 装不上（见 main 里步骤 4 的说明）。
 */
static bool s_configReady = false;

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
     * 步骤 4：作业配置。
     *
     * ⚠️ 这一段**不能退出进程**，哪怕配置缺失或非法。
     *
     * `dji_app_ctl install` 会在安装过程中试运行应用，并要求它走完 SDK 身份
     * 校验。若应用在 DjiCore_ApplicationStart 之后、校验完成之前退出，安装
     * 就判失败，而报的是一句误导性的 `verify app user_app_id or version info
     * error` —— 真正的死因（"进程没了"）安装器看不到。
     *
     * 所以 fail-closed 的边界划在「作业开始」，不是「进程启动」：
     * 这里只把问题记下来，让 SDK 起来、把校验走完，等到真正要执行作业时
     * 再拒绝（见下方 s_configReady 的用法）。
     */
    {
        FILE *probe = fopen(configPath, "r");

        if (probe == NULL) {
            char dir[WT_CONFIG_MAX_PATH];

            if (WtRunner_DirName(configPath, dir, sizeof(dir))) {
                WtRunner_MakeDirs(dir);
            }

            if (WtAppConfig_WriteTemplate(configPath)) {
                USER_LOG_WARN("配置文件 %s 不存在，已生成模板。"
                              "填写后重启即可作业；本次不执行飞行。", configPath);
            } else {
                USER_LOG_ERROR("配置文件 %s 不存在，且模板生成失败（目录不可写？）",
                               configPath);
            }
            s_configReady = false;
        } else {
            fclose(probe);

            if (WtRunner_Init(&s_runner, configPath) != WT_RUN_OK) {
                USER_LOG_ERROR("运行器初始化失败，本次不执行飞行（配置问题，非启动失败）");
                s_configReady = false;
            } else if (!WtTelemetry_Init(&s_runner.telemetry,
                                         s_runner.config.turbineCount > 0
                                             ? &s_runner.config.turbines[0].spec
                                             : NULL)) {
                USER_LOG_ERROR("遥测初始化失败，本次不执行飞行");
                WtRunner_DeInit(&s_runner);
                s_configReady = false;
            } else {
                s_configReady = true;
            }
        }
    }

    /*
     * 配置就绪才作业；否则保持进程存活，等安装器（或操作员）决定何时退出。
     * 这里的等待是**有意**的：进程活着 = SDK 身份校验有机会走完 = 装得上包。
     */
    if (!s_configReady) {
        USER_LOG_WARN("配置未就绪，应用保持运行但不执行飞行。"
                      "请填好配置后重启应用。");

        while (!s_stopRequested) {
            sleep(1);
        }

        DjiCore_DeInit();
        if (s_logFile != NULL) {
            fclose(s_logFile);
        }
        printf("[app] 配置未就绪，退出，返回码 0\n");

        return 0;
    }

    /*
     * 步骤 5：作业闸门。
     *
     * `auto_start` 控制的是**是否开始作业**，不是「出错后是否继续」。
     * 默认 false —— 装好应用后进程起来、走完 SDK 身份校验、然后停在
     * 「待命」状态等指令；要真正执行作业必须显式把它改成 true。
     *
     * 为什么这个语义很重要：安装器会试运行应用并要求它走完身份校验。
     * 若默认就开始作业，进程会在上传航线后长时间等待飞控状态（飞机不在
     * 空中时这个等待不会结束），安装器超时 → 装不上包。而且「装上就自动
     * 起飞」本身也是危险默认。
     */
    if (!s_runner.config.autoStart) {
        USER_LOG_WARN("auto_start = false：配置已就绪，但**不自动开始作业**。");
        USER_LOG_WARN("  应用保持运行并已走完 SDK 身份校验，等待指令。");
        USER_LOG_WARN("  要执行作业，请在配置里设 auto_start = true 后重启应用。");

        while (!s_stopRequested) {
            sleep(1);
        }

        WtTelemetry_DeInit(&s_runner.telemetry);
        WtRunner_DeInit(&s_runner);
        DjiCore_DeInit();
        if (s_logFile != NULL) {
            fclose(s_logFile);
        }
        printf("[app] auto_start = false，未执行作业，退出，返回码 0\n");

        return 0;
    }

    /* 步骤 6：逐台风机作业（仅 auto_start = true 时到达这里） */
    {
        int i;

        for (i = 0; i < s_runner.config.turbineCount && !s_stopRequested; i++) {
            const char *name = s_runner.config.turbines[i].name;
            WtRunResult rr;

            /*
             * 停用角由运行器从该风机的配置条目读取（[turbine.xxx]
             * park_phase），这里不传 —— 曾经在这里硬编码 -1.0，把配置
             * 静默架空了：现场改了 park_phase 却毫无效果。
             */
            USER_LOG_INFO("---- 开始作业：%s ----", name);
            rr = WtRunner_RunTurbine(&s_runner, name);

            if (rr != WT_RUN_OK) {
                USER_LOG_ERROR("风机 %s 作业未完成，结果码 %d", name, (int)rr);
                exitCode = (int)rr;
                break; /* 出错即停，等待人工介入 */
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