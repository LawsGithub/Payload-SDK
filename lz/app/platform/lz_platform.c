/**
 * @file lz_platform.c
 * @brief 平台层注册实现。
 *
 * 移植自 `samples/sample_c/platform/linux/manifold3/application/main.c` 的
 * `DjiUser_PrepareSystemEnvironment()` —— 逻辑未改，只换了日志路径与函数名。
 * 官方原始实现见该文件，比对时以那里为准。
 */

#include "lz_platform.h"
#include "lz_sdk_log_watch.h"

#include <dji_logger.h>
#include <dji_platform.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* 这两个头文件来自官方 manifold3 平台适配层。
 * include 路径写法必须与 CMake 里加进搜索路径的目录匹配：
 *   ${LZ_M3_DIR}/hal          → "hal_usb_bulk.h"
 *   ${LZ_COMMON_DIR}          → "osal/osal.h"
 * 写成 "hal/hal_usb_bulk.h" 会找不到 —— CMake 加的是 hal/ **里面**。 */
#include "hal_usb_bulk.h"
#include "osal/osal.h"
#include "osal/osal_fs.h"
#include "osal/osal_socket.h"

#define LZ_LOG_PATH              LZ_LOG_DIR "/DJI"
#define LZ_LOG_INDEX_FILE_NAME   LZ_LOG_DIR "/index"
#define LZ_LOG_FOLDER_NAME       LZ_LOG_DIR
#define LZ_LOG_MAX_COUNT         10

static FILE *s_logFile;
static FILE *s_logIndexFile;

/* ------------------------------------------------------------------ */
/* 两个 console：一个打到 stdout，一个落文件                            */
/* ------------------------------------------------------------------ */

/* ️ 两个 console 都先把数据喂给日志抓取。
 *
 * 为什么两个都喂而不是只喂一个：不能保证"哪个 console 会拿到哪一段" ——
 * 它们是各自独立的注册项。**重复喂是无害的**（`LzSdkLogWatch_Feed` 只保留
 * 最近一次命中，同样的字节流喂两遍得到同样的状态），但漏喂会丢行。
 * 宁可做两遍也不能漏。
 *
 * 为什么必须**先喂再输出**：`printf`/`fwrite` 可能阻塞（管道满、磁盘慢），
 * 而我们要抓的那几行恰恰出现在"应用即将因为启动失败而做别的事"的时刻 ——
 * 顺序反了就可能在那之前丢行。 */
static T_DjiReturnCode LzPlatform_PrintConsole(const uint8_t *data, uint16_t dataLen)
{
    LzSdkLogWatch_Feed(data, dataLen);
    printf("%.*s", dataLen, data);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode LzPlatform_LogWrite(const uint8_t *data, uint16_t dataLen)
{
    LzSdkLogWatch_Feed(data, dataLen);
    if (s_logFile == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    fwrite(data, 1, dataLen, s_logFile);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* ------------------------------------------------------------------ */
/* 日志文件初始化                                                       */
/* ------------------------------------------------------------------ */

/**
 * 每个进程启动时新建一个带时间戳的日志文件，并把 latest.log 软链过去；
 * 超过 LZ_LOG_MAX_COUNT 个就删最旧的。
 */
static T_DjiReturnCode LzPlatform_LogFsInit(const char *path)
{
    char filePath[128];
    char systemCmd[192];
    char folderName[32];

    const time_t now = time(NULL);
    const struct tm *localTime = localtime(&now);
    if (localTime == NULL) {
        printf("Get local time error.\r\n");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    if (access(LZ_LOG_FOLDER_NAME, F_OK) != 0) {
        /* 目录不存在就建。注意设备 RTC 无电池、掉电回 1970，
         * 生成的文件名时间戳会不对，但那只影响可读性，不影响功能。 */
        snprintf(folderName, sizeof(folderName), "mkdir -p %s", LZ_LOG_FOLDER_NAME);
        if (system(folderName) != 0) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        }
    }

    uint16_t logFileIndex = 0;
    s_logIndexFile = fopen(LZ_LOG_INDEX_FILE_NAME, "rb+");
    if (s_logIndexFile == NULL) {
        s_logIndexFile = fopen(LZ_LOG_INDEX_FILE_NAME, "wb+");
        if (s_logIndexFile == NULL) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        }
    } else {
        if (fseek(s_logIndexFile, 0, SEEK_SET) != 0) {
            printf("Seek log count file error, errno: %d.\r\n", errno);
            return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        }
        if (fread(&logFileIndex, 1, sizeof(uint16_t), s_logIndexFile) != sizeof(uint16_t)) {
            printf("Read log file index error.\r\n");
        }
    }

    const uint16_t currentLogFileIndex = logFileIndex;
    logFileIndex++;

    if (fseek(s_logIndexFile, 0, SEEK_SET) != 0) {
        printf("Seek log file error, errno: %d.\r\n", errno);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    if (fwrite(&logFileIndex, 1, sizeof(uint16_t), s_logIndexFile) != sizeof(uint16_t)) {
        printf("Write log file index error.\r\n");
        fclose(s_logIndexFile);
        s_logIndexFile = NULL;
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    fclose(s_logIndexFile);
    s_logIndexFile = NULL;

    snprintf(filePath, sizeof(filePath), "%s_%04d_%04d%02d%02d_%02d-%02d-%02d.log",
             path, currentLogFileIndex,
             localTime->tm_year + 1900, localTime->tm_mon + 1, localTime->tm_mday,
             localTime->tm_hour, localTime->tm_min, localTime->tm_sec);

    s_logFile = fopen(filePath, "wb+");
    if (s_logFile == NULL) {
        printf("Open log file error: %s\r\n", filePath);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    if (logFileIndex >= LZ_LOG_MAX_COUNT) {
        snprintf(systemCmd, sizeof(systemCmd), "rm -rf %s_%04d*.log",
                 path, currentLogFileIndex - LZ_LOG_MAX_COUNT);
        if (system(systemCmd) != 0) {
            printf("Remove old log error.\r\n");
        }
    }

    snprintf(systemCmd, sizeof(systemCmd), "ln -sfrv %s " LZ_LOG_FOLDER_NAME "/latest.log", filePath);
    if (system(systemCmd) != 0) {
        printf("Create latest.log symlink failed.\r\n");
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* ------------------------------------------------------------------ */
/* 注册                                                                */
/* ------------------------------------------------------------------ */

T_DjiReturnCode LzPlatform_Prepare(void)
{
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

    T_DjiLoggerConsole printConsole = {
        .func = LzPlatform_PrintConsole,
        .consoleLevel = DJI_LOGGER_CONSOLE_LOG_LEVEL_INFO,
        .isSupportColor = true,
    };

    T_DjiLoggerConsole fileConsole = {
        .func = LzPlatform_LogWrite,
        .consoleLevel = DJI_LOGGER_CONSOLE_LOG_LEVEL_DEBUG,
        .isSupportColor = true,
    };

    const T_DjiHalUsbBulkHandler usbBulkHandler = {
        .UsbBulkInit = HalUsbBulk_Init,
        .UsbBulkDeInit = HalUsbBulk_DeInit,
        .UsbBulkWriteData = HalUsbBulk_WriteData,
        .UsbBulkReadData = HalUsbBulk_ReadData,
        .UsbBulkGetDeviceInfo = HalUsbBulk_GetDeviceInfo,
    };

    const T_DjiFileSystemHandler fsHandler = {
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

    const T_DjiSocketHandler socketHandler = {
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

    T_DjiReturnCode rc = DjiPlatform_RegOsalHandler(&osalHandler);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("register osal handler error\n");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    if (LzPlatform_LogFsInit(LZ_LOG_PATH) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("log file system init error\n");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    rc = DjiLogger_AddConsole(&printConsole);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("add printf console error\n");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    rc = DjiLogger_AddConsole(&fileConsole);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("add file console error\n");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    rc = DjiPlatform_RegHalUsbBulkHandler(&usbBulkHandler);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("register hal usb bulk handler error\n");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    rc = DjiPlatform_RegSocketHandler(&socketHandler);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("register socket handler error\n");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    rc = DjiPlatform_RegFileSystemHandler(&fsHandler);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("register filesystem handler error\n");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void LzPlatform_Deinit(void)
{
    if (s_logFile != NULL) {
        fclose(s_logFile);
        s_logFile = NULL;
    }
    if (s_logIndexFile != NULL) {
        fclose(s_logIndexFile);
        s_logIndexFile = NULL;
    }
}
