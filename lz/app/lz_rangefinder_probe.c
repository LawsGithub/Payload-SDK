/**
 * @file lz_rangefinder_probe.c
 * @brief 探针：查 M4T 上哪个挂载位置带激光测距，以及它输出什么。
 *
 * ## 回答"怎么判断哪个位置有激光测距"
 *
 * 没有"列出所有相机能力"的接口，只能**逐个位置去问**。本探针用三级判据，
 * 从便宜到贵，任一级失败就跳过该位置：
 *
 *   1. `DjiCameraManager_GetCameraType(position, &type)`
 *      —— 该位置有没有相机？没有就没必要往下问。
 *   2. `DjiCameraManager_GetLaserRangingInfo(position, &info)`
 *      —— 返回值 + `info.enable_lidar`。**这是最直接的判据**。
 *   3. 连读几次看 `distance` 是否稳定（排除"接口能给但硬件没有"）
 *
 * ## 为什么先写探针而不是直接写进主程序
 *
 * 测距的可用性、频率、精度、以及"激光打中旗杆时数据长什么样"，
 * 都不是读文档能确定的。探针把不确定性隔离在一次上机试验里，
 * 拿到真实数据再决定主程序怎么用 —— 比先把架构搭好再返工便宜得多。
 *
 * ## 用法
 *
 *   ./lz_rangefinder_probe              # 扫一遍所有位置，各读 5 次
 *   ./lz_rangefinder_probe 1            # 只看位置 1
 *   ./lz_rangefinder_probe 1 20         # 位置 1 连读 20 次（看稳定性）
 *   ./lz_rangefinder_probe 1 -1         # ★ 连续监视，Ctrl-C 结束
 *
 * 连续监视模式是为**对照实验**设计的：人移动飞机改变镜头前方距离，
 * 同时盯着读数。Δ 列显示相对第一帧的变化量 —— 它动没动一眼可见。
 *
 * ⚠️ 需要飞机通电并连接；且装包/运行前要让出 PSDK 通道：
 *   pgrep -x Smart3DExplore >/dev/null || /system/bin/dji_app_ctl stop Smart3DExplore
 */

#include <dji_camera_manager.h>
#include <dji_core.h>
#include <dji_logger.h>
#include <dji_typedef.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "platform/lz_platform.h"
#include "platform/lz_user_info.h"

/* 要扫的位置：枚举里 1..8 都有定义，逐个试。
 * 注意 EXTENSION_PORT_V2_NO1/2/3 与 PAYLOAD_PORT_NO1/2/3 **是同一个值**，
 * 重复扫没有意义，所以只取去重后的值。 */
static const E_DjiMountPosition kPositions[] = {
    DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1,           /* = EXTENSION_PORT_V2_NO1 = 1 */
    DJI_MOUNT_POSITION_PAYLOAD_PORT_NO2,           /* = EXTENSION_PORT_V2_NO2 = 2 */
    DJI_MOUNT_POSITION_PAYLOAD_PORT_NO3,           /* = EXTENSION_PORT_V2_NO3 = 3 */
    DJI_MOUNT_POSITION_EXTENSION_PORT_V2_NO4,      /* 4 */
    DJI_MOUNT_POSITION_EXTENSION_PORT_V2_NO5,      /* 5 */
    DJI_MOUNT_POSITION_EXTENSION_PORT_V2_NO6,      /* 6 */
    DJI_MOUNT_POSITION_EXTENSION_PORT_V2_NO7,      /* 7 */
    DJI_MOUNT_POSITION_EXTENSION_PORT_V2_NO8,      /* 8 */
};

static const char *kPositionNames[] = {
    "PAYLOAD_PORT_NO1(E1)", "PAYLOAD_PORT_NO2(E2)", "PAYLOAD_PORT_NO3(E3)",
    "EXTENSION_PORT_V2_NO4(E4)", "EXTENSION_PORT_V2_NO5(usb hub1)",
    "EXTENSION_PORT_V2_NO6(usb hub2)", "EXTENSION_PORT_V2_NO7(usb hub3)",
    "EXTENSION_PORT_V2_NO8(usb hub4)",
};

/** 探测一个位置：返回 true 表示该位置有相机 */
static bool probe_position(E_DjiMountPosition pos, const char *name, int samples)
{
    printf("\n── 位置 %d: %s ──\n", (int)pos, name);

    /* 判据 1：这个位置有没有相机？ */
    E_DjiCameraType camType = DJI_CAMERA_TYPE_UNKNOWN;
    const T_DjiReturnCode rcType = DjiCameraManager_GetCameraType(pos, &camType);
    if (rcType != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("   相机类型查询失败 rc=0x%08X（该位置可能为空）\n", (unsigned)rcType);
        return false;
    }
    printf("   相机类型: %d\n", (int)camType);

    /* 判据 2：激光测距能不能读到 */
    T_DjiCameraManagerLaserRangingInfo info;
    memset(&info, 0, sizeof(info));
    const T_DjiReturnCode rc = DjiCameraManager_GetLaserRangingInfo(pos, &info);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("   激光测距: 不支持 rc=0x%08X\n", (unsigned)rc);
        return true;   /* 有相机，只是没测距 */
    }

    printf("   ★ 激光测距可用！enable_lidar=%d exception=%u\n",
           (int)info.enable_lidar, (unsigned)info.exception);

    const bool watchMode = (samples < 0);

    /* 注意顺序：watchMode(-1) 必须在 samples<=1 之前判断 ——
     * -1 <= 1 成立，写反了就会掉进单次分支（这个 bug 真踩过）。 */
    if (!watchMode && samples <= 1) {
        printf("     lon=%.7f lat=%.7f alt=%.1fm dist=%.1fm screen=(%.1f%%, %.1f%%)\n",
               info.longitude, info.latitude, info.altitude / 10.0,
               info.distance / 10.0, info.screenX / 10.0, info.screenY / 10.0);
        return true;
    }

    /* 判据 3：连读看稳定性。
     *
     * samples < 0 表示**连续监视模式**：一直读直到 Ctrl-C。
     * 这是为了"人动、机器读"的对照实验 —— 移动飞机改变镜头前方距离，
     * 同时盯着读数变不变。比定次数的连读有用得多：定次数时人还没走到
     * 位置，程序就读完了。 */
    if (watchMode) {
        printf("   [监视模式] 每 250ms 读一次，Ctrl-C 结束。\n");
        printf("   现在开始移动飞机/妙算3，改变镜头前方的距离，观察「距离m」与 Δ 列。\n");
    }
    printf("     序号  经度        纬度        高度m   距离m   屏幕X%%  屏幕Y%%  exc\n");
    fflush(stdout);
    double dmin = 1e9, dmax = -1e9;
    double dFirst = -1.0;
    for (int i = 0; watchMode || i < samples; ++i) {
        memset(&info, 0, sizeof(info));
        if (DjiCameraManager_GetLaserRangingInfo(pos, &info) ==
            DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            const double d = info.distance / 10.0;
            if (dFirst < 0.0) {
                dFirst = d;
            }
            if (d < dmin) dmin = d;
            if (d > dmax) dmax = d;
            if (watchMode) {
                /* 监视模式：每行都带「相对首帧的变化量」，一眼看出读数有没有动 */
                printf("     %4d  %.7f  %.7f  %6.1f  %6.1f  %6.1f  %6.1f  %3u   (Δ %+.1f)\n",
                       i, info.longitude, info.latitude, info.altitude / 10.0, d,
                       info.screenX / 10.0, info.screenY / 10.0,
                       (unsigned)info.exception, d - dFirst);
                /* 必须 flush：输出重定向到文件/管道时 stdio 是全缓冲（4KB），
                 * 不 flush 的话要攒够 4KB 才可见 —— 而监视模式的全部意义
                 * 就是"实时看到读数在动"，攒批输出等于没输出。 */
                fflush(stdout);
            } else {
                printf("     %4d  %.7f  %.7f  %6.1f  %6.1f  %6.1f  %6.1f  %3u\n",
                       i, info.longitude, info.latitude, info.altitude / 10.0, d,
                       info.screenX / 10.0, info.screenY / 10.0, (unsigned)info.exception);
            }
        } else {
            printf("     %4d  读取失败\n", i);
        }
        /* 头文件 @note: 最大更新频率 5Hz → 200ms 一次 */
        usleep(250 * 1000);
    }
    if (!watchMode && dmax >= dmin) {
        printf("   距离范围: %.1f ~ %.1f m（抖动 %.1f m）\n", dmin, dmax, dmax - dmin);
    }
    return true;
}

int main(int argc, char **argv)
{
    printf("[lz-rangefinder] 激光测距探针\n");

    /* ---- 平台层：必须在 DjiCore_Init 之前全部注册 ---- */
    if (LzPlatform_Prepare() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("平台层注册失败\n");
        return 1;
    }

    T_DjiUserInfo userInfo;
    if (LzUserInfo_Fill(&userInfo) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("凭据填充失败 —— 见上方日志\n");
        LzPlatform_Deinit();
        return 1;
    }

    printf("正在初始化 PSDK（阻塞 2-4 秒，等飞机与转接环就绪）...\n");
    if (DjiCore_Init(&userInfo) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("DjiCore_Init 失败\n");
        LzPlatform_Deinit();
        return 1;
    }

    /* 相机管理模块：观测激光测距必须初始化它 */
    const T_DjiReturnCode rcCam = DjiCameraManager_Init();
    if (rcCam != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("DjiCameraManager_Init 失败 rc=0x%08X\n", (unsigned)rcCam);
        DjiCore_DeInit();
        LzPlatform_Deinit();
        return 1;
    }

    if (DjiCore_ApplicationStart() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("DjiCore_ApplicationStart 失败\n");
    }

    /* ---- 解析参数 ---- */
    int singlePos = -1;
    int samples = 5;
    if (argc >= 2) {
        singlePos = atoi(argv[1]);
    }
    if (argc >= 3) {
        samples = atoi(argv[2]);
        /* -1 = 连续监视（直到 Ctrl-C）；正数 = 读这么多次 */
        if (samples != -1 && (samples < 1 || samples > 100000)) {
            samples = 5;
        }
    }

    printf("\n注：M4T 原生相机（可见光/红外/4K）通常挂在位置 1；\n");
    printf("    激光测距若存在，多半在同轴云台那一路。\n");

    if (singlePos >= 0) {
        bool found = false;
        for (size_t i = 0; i < sizeof(kPositions) / sizeof(kPositions[0]); ++i) {
            if ((int)kPositions[i] == singlePos) {
                probe_position(kPositions[i], kPositionNames[i], samples);
                found = true;
                break;
            }
        }
        if (!found) {
            printf("位置 %d 不在扫描列表里\n", singlePos);
        }
    } else {
        int withCamera = 0;
        for (size_t i = 0; i < sizeof(kPositions) / sizeof(kPositions[0]); ++i) {
            if (probe_position(kPositions[i], kPositionNames[i], samples)) {
                withCamera++;
            }
        }
        printf("\n扫描完毕：%zu 个位置中 %d 个有相机。\n",
               sizeof(kPositions) / sizeof(kPositions[0]), withCamera);
    }

    printf("\n结论：把上面标了「★ 激光测距可用」的位置序号记下来，\n");
    printf("      写进 CLAUDE.md / API-MAP，主程序按那个位置取值。\n");

    DjiCameraManager_DeInit();
    DjiCore_DeInit();
    LzPlatform_Deinit();
    return 0;
}
