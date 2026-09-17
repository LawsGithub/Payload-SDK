/**
 * @file wt_telemetry_psdk.c
 * @brief 遥测层的 PSDK 胶水实现
 *
 * 只做「订阅话题 -> 填充 WtTelemetry 结构体」这一件事。所有需要动脑的
 * 解算（相位反解、滤波、外推）都在 wt_telemetry.c 里，那份代码在 PC 上
 * 可编译可单测，本文件只在妙算3 目标上编译。
 */

#include "wt_telemetry.h"

#include <math.h>
#include <string.h>

#include <dji_fc_subscription.h>
#include <dji_logger.h>

/*
 * 订阅频率的选择受两条硬约束（见 dji_fc_subscription.h 的说明）：
 *   1) 不同频率的种类不超过 4 种；
 *   2) 同一频率下所有话题的数据长度之和不超过 242 字节。
 * 这里全部话题统一用 10Hz：巡检控制环本来就是 10Hz 量级，更高频率只会
 * 增加总线负载而不改善控制效果，也让上述两条约束天然满足。
 */
#define WT_TELE_TOPIC_FREQ DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ

static bool WtTelemetryPsdk_ReadPosition(const T_DjiFcSubscriptionPositionFused *pos,
                                         WtAircraftState *state)
{
    if (pos == NULL || state == NULL) {
        return false;
    }

    /*
     * 注意单位：PSDK 的 POSITION_FUSED 经纬度是**弧度**，而全工程其余部分
     * 一律用度。这个换算只在本文件出现一次，是最容易出错也最容易核对的位置。
     */
    state->position.lat = pos->latitude * WT_RAD2DEG;
    state->position.lon = pos->longitude * WT_RAD2DEG;
    state->position.alt = pos->altitude;
    state->visibleSatellites = pos->visibleSatelliteNumber;

    /* (0,0) 是 PSDK 未收到定位时的哨兵值，不能当成真实坐标使用 */
    if (fabs(state->position.lat) < 1e-9 && fabs(state->position.lon) < 1e-9) {
        return false;
    }

    return true;
}

static void WtTelemetryPsdk_ReadRtk(const T_DjiFcSubscriptionRtkPosition *rtk,
                                    T_DjiFcSubscriptionRtkPositionInfo info,
                                    WtAircraftState *state)
{
    if (rtk == NULL || state == NULL) {
        return;
    }

    /* RTK 话题的经纬度已经是「度」，无需换算 */
    state->position.lat = rtk->latitude;
    state->position.lon = rtk->longitude;
    state->position.alt = rtk->hfsl; /* 海拔高，与规划器的椭球高在此处视为一致 */

    /*
     * 只有窄巷固定解（NARROW_INT）与 L1 整周固定解才具备厘米级精度，
     * 其余（单点解、浮点解）不能作为叶片近距巡检的位置基准。
     */
    state->rtkFixed = (info == DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_NARROW_INT) ||
                      (info == DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_L1_AMBIGUITY_INT) ||
                      (info == DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_WIDE_LANE_AMBIGUITY_INT);
}

static void WtTelemetryPsdk_ReadAttitude(const T_DjiFcSubscriptionQuaternion *q,
                                         WtAircraftState *state)
{
    double q0, q1, q2, q3;
    double sinr_cosp, cosr_cosp, sinp, siny_cosp, cosy_cosp;

    if (q == NULL || state == NULL) {
        return;
    }

    q0 = q->q0;
    q1 = q->q1;
    q2 = q->q2;
    q3 = q->q3;

    /*
     * 四元数按 Hamilton 约定，表示机体 FRD 到地面 NED 的旋转。
     * 由此推出的 yaw 已经是「自正北顺时针」，与全工程的角度约定一致，
     * 不需要再做任何翻转 —— 这是选 NED 而非 ENU 做姿态中介的原因。
     */
    siny_cosp = 2.0 * (q0 * q3 + q1 * q2);
    cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
    state->yawDeg = Wt_Wrap360(atan2(siny_cosp, cosy_cosp) * WT_RAD2DEG);

    sinp = 2.0 * (q0 * q2 - q3 * q1);
    if (fabs(sinp) >= 1.0) {
        state->pitchDeg = (sinp > 0.0 ? 1.0 : -1.0) * 90.0;
    } else {
        state->pitchDeg = asin(sinp) * WT_RAD2DEG;
    }

    sinr_cosp = 2.0 * (q0 * q1 + q2 * q3);
    cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
    state->rollDeg = atan2(sinr_cosp, cosr_cosp) * WT_RAD2DEG;

    state->validAttitude = true;
}

bool WtTelemetry_Init(WtTelemetry *telemetry, const WtTurbineSpec *spec)
{
    T_DjiReturnCode rc;

    memset(telemetry, 0, sizeof(*telemetry));

    if (spec != NULL) {
        telemetry->turbineSpec = *spec;
        telemetry->frame = WtLocalFrame_Init(&spec->base);
    }

    rc = DjiFcSubscription_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiFcSubscription_Init failed: 0x%08X", rc);
        return false;
    }

    /*
     * 只订阅巡检真正需要的话题。每多一个话题就多一份总线带宽与解析开销，
     * 机载端应与飞行安全相关的话题优先，无关的（如云台角度以外的载荷状态）
     * 一律不订。
     */
    {
        const E_DjiFcSubscriptionTopic topics[] = {
            DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
            DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
            DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
            DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
            DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_RELATIVE,
            DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
            DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
        };
        size_t i;

        for (i = 0; i < sizeof(topics) / sizeof(topics[0]); i++) {
            rc = DjiFcSubscription_SubscribeTopic(topics[i], WT_TELE_TOPIC_FREQ, NULL);
            if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_WARN("subscribe topic %d failed: 0x%08X", (int)topics[i], rc);
            }
        }
    }

    telemetry->initialized = true;

    return true;
}

void WtTelemetry_DeInit(WtTelemetry *telemetry)
{
    /*
     * 取消订阅必须按订阅顺序（队列语义）。顺序写反会导致取消失败，
     * 模块 DeInit 时资源释放不干净。
     */
    const E_DjiFcSubscriptionTopic topics[] = {
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
        DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
        DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
        DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
        DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_RELATIVE,
        DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
        DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
    };
    size_t i;

    for (i = 0; i < sizeof(topics) / sizeof(topics[0]); i++) {
        DjiFcSubscription_UnSubscribeTopic(topics[i]);
    }

    DjiFcSubscription_DeInit();
    telemetry->initialized = false;
}

void WtTelemetry_Poll(WtTelemetry *telemetry)
{
    T_DjiFcSubscriptionRtkPosition rtk;
    T_DjiFcSubscriptionRtkPositionInfo rtkInfo;
    T_DjiFcSubscriptionPositionFused pos;
    T_DjiFcSubscriptionQuaternion quat;
    T_DjiFcSubscriptionVelocity vel;
    T_DjiFcSubscriptionHeightRelative height;
    T_DjiFcSubscriptionFlightStatus flightStatus;
    T_DjiFcSubscriptionGimbalAngles gimbal;
    WtAircraftState *a = &telemetry->aircraft;

    /* 位置：优先 RTK，退化到融合定位 */
    if (DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
                                                (uint8_t *)&rtk, sizeof(rtk),
                                                NULL) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        if (DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
                                                    (uint8_t *)&rtkInfo, sizeof(rtkInfo),
                                                    NULL) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            rtkInfo = DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_NOT_AVAILABLE;
        }
        WtTelemetryPsdk_ReadRtk(&rtk, rtkInfo, a);
        a->validPosition = true;
    } else if (DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
                                                       (uint8_t *)&pos, sizeof(pos),
                                                       NULL) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        a->validPosition = WtTelemetryPsdk_ReadPosition(&pos, a);
        a->rtkFixed = false;
    } else {
        a->validPosition = false;
    }

    if (a->validPosition) {
        WtTelemetry_RefreshEnu(telemetry);
    }

    if (DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                (uint8_t *)&quat, sizeof(quat),
                                                NULL) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        WtTelemetryPsdk_ReadAttitude(&quat, a);
    }

    if (DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                (uint8_t *)&vel, sizeof(vel),
                                                NULL) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        a->velocityMs = sqrt(vel.data.x * vel.data.x +
                             vel.data.y * vel.data.y +
                             vel.data.z * vel.data.z);
    }

    if (DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_RELATIVE,
                                                (uint8_t *)&height, sizeof(height),
                                                NULL) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        a->heightAboveGroundM = height;
    }

    if (DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                (uint8_t *)&flightStatus, sizeof(flightStatus),
                                                NULL) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        a->flightStatus = flightStatus;
    }

    if (DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
                                                (uint8_t *)&gimbal, sizeof(gimbal),
                                                NULL) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        /*
         * 该话题用通用 Vector3f 承载三个角度，字段与语义的对应关系是
         * x=pitch, y=roll, z=yaw —— 按 x/y/z 的直觉顺序去读会得到完全
         * 错误的姿态，是这一层最隐蔽的坑。参考系为 NED，与规划器的云台角
         * 定义一致，可直接比较与闭环。
         */
        telemetry->gimbal.pitchDeg = gimbal.x;
        telemetry->gimbal.rollDeg = gimbal.y;
        telemetry->gimbal.yawDeg = Wt_Wrap360(gimbal.z);
        telemetry->gimbal.valid = true;
    }

    a->timestampMs += 100; /* 10Hz 固定步长；真实时基由上层 MONOTONIC 时钟覆盖 */
}