/**
 * @file main.c
 * @brief 机载应用入口。
 *
 * ⚠️ 启动路径上**不允许**出现"配置缺失就退出"这类防御。
 *    `dji_app_ctl install` 会试运行应用并要求走完 SDK 身份校验；应用若在
 *    DjiCore_Init 之后、校验完成之前退出，安装会失败，而报出的却是一句
 *    误导性的 `Error, verify user_app_id or version info error`。
 *    fail-closed 的边界划在「作业开始」，不是「进程启动」。
 *
 * ## 初始化顺序是硬约束（不遵守不会编译报错）
 *
 *   1. 注册平台层（console / OSAL / HAL / FS / socket）
 *   2. DjiCore_Init —— 阻塞 2-4 秒，等飞机与转接环就绪
 *   3. 各功能模块 Init / Reg
 *   4. DjiCore_ApplicationStart —— 必须在所有模块 register 之后
 *   5. 业务循环
 *   6. DjiCore_DeInit
 *
 * 实测日志（2026-09-19，M4T + 妙算3）可见顺序正确：
 *   dji_core.c:147  Identify device : manifold3
 *   dji_core.c:228  Identify AircraftType = Matrice 4T, ...
 *   dji_identity_verify.c:654  Update dji sdk policy file successfully
 */

#include <dji_core.h>
#include <dji_logger.h>
#include <dji_platform.h>
#ifdef LZ_POLE_SOURCE_LASER
#include <dji_camera_manager.h>
#endif

#include <stdio.h>
#include <unistd.h>

#include "lz_mission.h"
#include "lz_pole_source.h"
#include "lz_widget.h"
#include "platform/lz_platform.h"
#include "platform/lz_user_info.h"

/* 固件版本，与 app_json/app.json 的 firmware_version 对应 */
#define LZ_FIRMWARE_MAJOR 1
#define LZ_FIRMWARE_MINOR 0
#define LZ_FIRMWARE_MODIFY 0
#define LZ_FIRMWARE_DEBUG 0

/* 业务循环周期。100 ms 足够跟手，又不至于空转吃 CPU。 */
#define LZ_MAIN_LOOP_MS 100

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("[lz] 启动\n");

    /* ---- 1. 平台层 ------------------------------------------------ */
    if (LzPlatform_Prepare() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[lz] 平台层注册失败\n");
        return 1;
    }

    T_DjiUserInfo userInfo;
    if (LzUserInfo_Fill(&userInfo) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[lz] 凭据填充失败 —— 见上方日志\n");
        LzPlatform_Deinit();
        return 1;
    }

    /* ---- 2. DjiCore_Init ----------------------------------------- */
    printf("[lz] 初始化 PSDK（阻塞 2-4 秒）...\n");
    if (DjiCore_Init(&userInfo) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[lz] DjiCore_Init 失败\n");
        LzPlatform_Deinit();
        return 1;
    }

    /* 负载在 Pilot 里显示的名字。
     * 注意与 app_json/app.json 的 name 字段是两回事：这里设的是**运行时别名**，
     * 不设的话 Pilot 显示的是开发者网站上登记的 App 名。 */
    (void)DjiCore_SetAlias("liangzhourenwu");

    const T_DjiFirmwareVersion fw = {
        .majorVersion = LZ_FIRMWARE_MAJOR,
        .minorVersion = LZ_FIRMWARE_MINOR,
        .modifyVersion = LZ_FIRMWARE_MODIFY,
        .debugVersion = LZ_FIRMWARE_DEBUG,
    };
    (void)DjiCore_SetFirmwareVersion(fw);

    /* ---- 3. 功能模块 --------------------------------------------- */
#ifdef LZ_POLE_SOURCE_LASER
    /* 相机管理模块 —— "记录激光点"按钮的前提。
     *
     * ⚠️ **不初始化它，`DjiCameraManager_GetLaserRangingInfo()` 就不会工作**
     * （官方头文件原话："user should call this function before using camera
     * manager features"）。而探针 `lz_rangefinder_probe.c` 里调了它、主应用
     * 原先没有 —— 这就是"探针能读到激光、主应用读不到"的原因。
     *
     * 失败**不阻断启动**（仓库级硬规则：fail-closed 的边界在「作业开始」）。
     * 失败时激光按钮会报"读取激光失败"，其余功能不受影响。 */
    if (DjiCameraManager_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[lz] 相机模块初始化失败 —— 激光记录不可用（其余功能正常）\n");
    }
#endif

    /* 控件：操作员的入口。必须先于 ApplicationStart，否则 Pilot 拉不到控件。 */
    if (LzWidget_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[lz] 控件模块初始化失败\n");
        /* 控件失败不退出 —— 见文件头：启动路径上不做 fail-closed。
         * 飞机侧的状态推送仍可用，操作员至少能从日志知道程序在跑。 */
    }

    if (LzMission_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[lz] 任务模块初始化失败\n");
    }

    /* 读回上次记录的绕飞圆心。
     *
     * ⚠️ **失败不阻断启动** —— 首次运行本来就没有记录文件，那是正常状态
     * （返回 LZ_ERR_NOT_READY）。这与仓库级 CLAUDE.md 的硬规则一致：
     * fail-closed 的边界划在「作业开始」，不是「进程启动」。
     * 未记录时操作员拨开关会被拒，并在浮窗里看到"请先记录圆心"。
     *
     * 放在这里（ApplicationStart 之前、控件 Init 之后）的理由：
     * 只是文件读取，不依赖 SDK 调度器；早读进来，操作员一开界面就能
     * 从浮窗知道当前有没有记录。 */
    (void)LzPole_LoadRecorded();

    /* ---- 4. ApplicationStart ------------------------------------- */
    if (DjiCore_ApplicationStart() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[lz] DjiCore_ApplicationStart 失败\n");
    }

    /* ️ 顺序硬约束：**必须在 ApplicationStart 之后**。
     * 这里订阅飞机状态话题（启动诊断用），而官方文档明写
     * DjiFcSubscription_Init "请勿在 main() 函数中调用……启动调度器后，
     * 该接口将正常运行"。放在前面会返回 SUCCESS 但随后 SIGSEGV
     * （实测 2026-09-20，见 lz_mission.h 的 LzMission_StartPostApp 说明）。 */
    (void)LzMission_StartPostApp();

    /* ---- 5. 业务循环 --------------------------------------------- */
    /* 只做一件事：推进任务状态机。所有决策都在 LzMission_Tick 里，
     * 这里保持极薄 —— 循环体越简单，越容易看出"程序在干什么"。 */
    LzWidget_PostMessage("liangzhourenwu 已启动，等待操作员");

    while (true) {
        LzMission_Tick();

        const T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
        if (osal != NULL) {
            osal->TaskSleepMs(LZ_MAIN_LOOP_MS);
        } else {
            usleep(LZ_MAIN_LOOP_MS * 1000);
        }
    }

    /* ---- 6. 清理（当前循环不退出，留着以备将来加退出信号）---------- */
    LzMission_DeInit();
    LzWidget_Stop();
#ifdef LZ_POLE_SOURCE_LASER
    (void)DjiCameraManager_DeInit();
#endif
    DjiCore_DeInit();
    LzPlatform_Deinit();
    printf("[lz] 退出\n");
    return 0;
}
