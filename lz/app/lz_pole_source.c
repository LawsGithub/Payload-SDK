/**
 * @file lz_pole_source.c
 * @brief 绕飞圆心的取得方式。
 *
 * 默认走**固定坐标**（`LZ_POLE_SOURCE_FIXED`），因为飞机当前断联、
 * 室内也无 GPS，激光那条路取不到有效值。
 * 定义 `LZ_POLE_SOURCE_LASER` 可切到激光测距实现。
 */

#include "lz_pole_source.h"

#include <dji_logger.h>

#include <string.h>

#ifdef LZ_POLE_SOURCE_LASER
#include <dji_camera_manager.h>
#endif

/* ------------------------------------------------------------------ */
/* 方式一：固定坐标（默认）                                              */
/* ------------------------------------------------------------------ */

/* 一个用于室内联调的写死坐标。真机作业时必须换成实测值 —— 这个数字
 * 落在武汉某处，纯属占位。 */
#define LZ_FIXED_POLE_LAT 30.500500
#define LZ_FIXED_POLE_LON 114.300300
#define LZ_FIXED_POLE_ALT 30.0

/* ------------------------------------------------------------------ */
/* 方式二：激光测距                                                     */
/* ------------------------------------------------------------------ */

#ifdef LZ_POLE_SOURCE_LASER

/* `[V]` 实测：M4T 的激光测距在位置 1（E1，自带云台相机那一路） */
#define LZ_LASER_MOUNT_POSITION DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1

/* exception 取值 —— **官方文档未公开**，以下由实测对照反推 `[?]`：
 *   1 = 无回波/测不到（此时 distance=0，lat/lon 也是 0）
 *   3 = 正常读数
 *   2 = 过渡态，含义不明
 * 只认 3 为有效。若日后发现有效读数也出现别的值，回来放宽这里。 */
#define LZ_LASER_EXC_OK 3

const char *LzPole_SourceName(void)
{
    return "激光测距";
}

LzStatus LzPole_Acquire(LzTarget *out)
{
    if (out == NULL) {
        return LZ_ERR_PARAM;
    }

    T_DjiCameraManagerLaserRangingInfo info;
    memset(&info, 0, sizeof(info));

    const T_DjiReturnCode rc =
        DjiCameraManager_GetLaserRangingInfo(LZ_LASER_MOUNT_POSITION, &info);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("读取激光测距失败 rc=0x%08X", (unsigned)rc);
        return LZ_ERR_IO;
    }

    /* ⚠️ 必须判 exception —— 否则会把「无回波时的 0,0」当成真坐标，
     * 那样生成的航线会指向几内亚湾。 */
    if (info.exception != LZ_LASER_EXC_OK) {
        USER_LOG_WARN("激光无有效回波（exception=%u，distance=%.1fm）",
                      (unsigned)info.exception, info.distance / 10.0);
        return LZ_ERR_NO_TARGET;
    }

    LzGeo geo = {
        .latitudeDeg = info.latitude,
        .longitudeDeg = info.longitude,
        .altitudeM = info.altitude / 10.0,
    };
    if (!LzGeo_IsValid(&geo)) {
        USER_LOG_WARN("激光给出的坐标非法（%.7f, %.7f）", geo.latitudeDeg, geo.longitudeDeg);
        return LZ_ERR_NO_TARGET;
    }

    memset(out, 0, sizeof(*out));
    out->id = 1;
    out->geo = geo;
    out->heightM = 15.0;   /* 杆高：国旗杆常见规格，绕飞不需要精确值 */
    out->radiusM = 0.1;
    out->confidence = 1.0;

    USER_LOG_INFO("杆位取自激光：%.7f, %.7f，距离 %.1f m",
                  geo.latitudeDeg, geo.longitudeDeg, info.distance / 10.0);
    return LZ_OK;
}

#else  /* 固定坐标 */

const char *LzPole_SourceName(void)
{
    return "固定坐标（室内联调用）";
}

LzStatus LzPole_Acquire(LzTarget *out)
{
    if (out == NULL) {
        return LZ_ERR_PARAM;
    }

    memset(out, 0, sizeof(*out));
    out->id = 1;
    out->geo.latitudeDeg = LZ_FIXED_POLE_LAT;
    out->geo.longitudeDeg = LZ_FIXED_POLE_LON;
    out->geo.altitudeM = LZ_FIXED_POLE_ALT;
    out->heightM = 15.0;
    out->radiusM = 0.1;
    out->confidence = 0.9;

    USER_LOG_WARN("杆位用的是写死的固定坐标（%.7f, %.7f）—— 仅供室内联调，"
                  "真机作业前必须换实测值",
                  out->geo.latitudeDeg, out->geo.longitudeDeg);
    return LZ_OK;
}

#endif /* LZ_POLE_SOURCE_LASER */