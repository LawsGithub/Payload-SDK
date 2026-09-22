/**
 * @file lz_types.c
 * @brief 基础类型的实现。
 */

#include "lz_types.h"

#include <math.h>

const char *LzStatus_Str(LzStatus status)
{
    switch (status) {
    case LZ_OK:              return "OK";
    case LZ_ERR_PARAM:       return "参数非法";
    case LZ_ERR_RANGE:       return "数值超出可规划范围";
    case LZ_ERR_UNSAFE:      return "安全校验不通过";
    case LZ_ERR_NO_TARGET:   return "没有可用目标";
    case LZ_ERR_IO:          return "文件读写失败";
    case LZ_ERR_UNSUPPORTED: return "当前机型不支持";
    case LZ_ERR_UPLOAD:      return "上传航线被拒";
    case LZ_ERR_START:       return "任务启动被拒";
    case LZ_ERR_NOT_READY:   return "前置条件未就绪";
    }
    return "未知状态";
}

bool LzGeo_IsValid(const LzGeo *geo)
{
    if (geo == NULL) {
        return false;
    }
    /* isnan/isinf 而非简单的范围比较：NaN 参与任何比较都是 false，
     * 只写范围判断会把 NaN 悄悄放过去（它既不 > 90 也不 < -90）。 */
    if (!isfinite(geo->latitudeDeg) || !isfinite(geo->longitudeDeg) ||
        !isfinite(geo->altitudeM)) {
        return false;
    }
    return geo->latitudeDeg >= -90.0 && geo->latitudeDeg <= 90.0 &&
           geo->longitudeDeg >= -180.0 && geo->longitudeDeg <= 180.0;
}
