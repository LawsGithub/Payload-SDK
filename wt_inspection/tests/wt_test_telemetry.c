/**
 * @file wt_test_telemetry.c
 * @brief 遥测解算层的回归测试（高度基准换算、相位/转速估计）。
 *
 * 这一层原先没有任何测试，而它恰好是两类"静默算错"的集中地：
 *
 *   1. **高度基准**：RTK 给海拔高、融合定位给椭球高，两者都写进同一个
 *      `position.alt` 再做减法，于是走融合定位时凭空多出几十米误差。
 *   2. **转速估计**：用滤波后的相位当基准算增量，把滤波滞后算成了真实转动。
 *
 * 两者都不会报错、不会崩，只会让结果悄悄偏掉。
 *
 * 注意 `WtTelemetry_Init` 在 `wt_telemetry_psdk.c`（依赖 PSDK），桌面测不到，
 * 所以这里手工构造上下文。这也意味着 Init 里设的默认值不在本测试覆盖范围内。
 */

#include "wt_telemetry.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int s_failures = 0;

#define CHECK(cond, fmt, ...)                                              \
    do {                                                                   \
        if (!(cond)) {                                                     \
            printf("  [FAIL] " fmt "\n", ##__VA_ARGS__);                   \
            s_failures++;                                                  \
        }                                                                  \
    } while (0)

#define CHECK_NEAR(a, b, tol, fmt, ...)                                    \
    do {                                                                   \
        const double _a = (a);                                             \
        const double _b = (b);                                             \
        if (fabs(_a - _b) > (tol)) {                                       \
            printf("  [FAIL] " fmt " (got %.4f, want %.4f)\n",             \
                   ##__VA_ARGS__, _a, _b);                                 \
            s_failures++;                                                  \
        }                                                                  \
    } while (0)

/* 本地 geoid 偏移（MSL − 椭球高）。中国境内为负，取一个典型值。 */
#define GEOID_OFFSET (-38.0)

#define TOWER_ALT 50.0
#define FLIGHT_ALT_MSL 150.0

static void MakeTelemetry(WtTelemetry *t, E_WtAltDatum specDatum)
{
    memset(t, 0, sizeof(*t));

    t->turbineSpec.base.lat = 40.0;
    t->turbineSpec.base.lon = 116.0;
    t->turbineSpec.base.alt = TOWER_ALT;
    t->turbineSpec.bladeCount = 3;
    t->turbineSpec.rotorDiameter = 80.0;
    t->frame = WtLocalFrame_Init(&t->turbineSpec.base);
    t->specAltDatum = specDatum;

    t->aircraft.validPosition = true;
    t->aircraft.position.lat = 40.0;
    t->aircraft.position.lon = 116.0;
}

/**
 * 基准一致（都是 MSL）：天向分量就是单纯的高度差，且标记为可信。
 */
static void Test_Datum_Match(void)
{
    WtTelemetry t;

    printf("Test_Datum_Match\n");

    MakeTelemetry(&t, WT_ALT_DATUM_MSL);
    t.aircraft.position.alt = FLIGHT_ALT_MSL;
    t.aircraft.altDatum = WT_ALT_DATUM_MSL;

    WtTelemetry_RefreshEnu(&t);

    CHECK(t.aircraft.enuVerticalTrusted, "基准一致时天向应可信");
    CHECK_NEAR(t.aircraft.enu.u, FLIGHT_ALT_MSL - TOWER_ALT, 1e-6,
               "天向分量应为单纯高度差");
}

/**
 * 基准不一致且未注入 geoid 偏移：**必须标记为不可信**。
 *
 * 这是本次修复的核心 —— 原先这种情况完全静默，天向分量带着几十米误差
 * 被当成有效值用下去。
 */
static void Test_Datum_Mismatch_Untrusted(void)
{
    WtTelemetry t;

    printf("Test_Datum_Mismatch_Untrusted\n");

    MakeTelemetry(&t, WT_ALT_DATUM_MSL);
    /* 只有融合定位可用 → 椭球高 */
    t.aircraft.position.alt = FLIGHT_ALT_MSL - GEOID_OFFSET; /* 同一物理高度的椭球高 */
    t.aircraft.altDatum = WT_ALT_DATUM_ELLIPSOID;
    t.geoidOffsetM = 0.0; /* 未注入 */

    WtTelemetry_RefreshEnu(&t);

    CHECK(!t.aircraft.enuVerticalTrusted,
          "基准不一致且无偏移时，天向必须标记为不可信");

    /*
     * 水平分量仍然可用 —— 这是刻意的：geoid 偏移只影响高度，
     * 把整个位置判为无效会让"只有融合定位"的场景完全不可用。
     */
    CHECK_NEAR(t.aircraft.enu.e, 0.0, 1e-6, "水平分量仍应可用");
    CHECK_NEAR(t.aircraft.enu.n, 0.0, 1e-6, "水平分量仍应可用");
}

/**
 * 注入 geoid 偏移后，椭球高被校正到台账基准，结果与 RTK 路径一致。
 */
static void Test_Datum_GeoidCorrection(void)
{
    WtTelemetry t;

    printf("Test_Datum_GeoidCorrection\n");

    MakeTelemetry(&t, WT_ALT_DATUM_MSL);
    t.aircraft.position.alt = FLIGHT_ALT_MSL - GEOID_OFFSET;
    t.aircraft.altDatum = WT_ALT_DATUM_ELLIPSOID;
    WtTelemetry_SetGeoidOffset(&t, GEOID_OFFSET);

    WtTelemetry_RefreshEnu(&t);

    CHECK(t.aircraft.enuVerticalTrusted, "注入偏移后天向应可信");
    CHECK_NEAR(t.aircraft.enu.u, FLIGHT_ALT_MSL - TOWER_ALT, 1e-6,
               "校正后应与 MSL 路径得到同一个天向分量");
}

/** 基准未知时一律不可信，不猜。 */
static void Test_Datum_Unknown(void)
{
    WtTelemetry t;

    printf("Test_Datum_Unknown\n");

    MakeTelemetry(&t, WT_ALT_DATUM_MSL);
    t.aircraft.position.alt = FLIGHT_ALT_MSL;
    t.aircraft.altDatum = WT_ALT_DATUM_UNKNOWN;

    WtTelemetry_RefreshEnu(&t);

    CHECK(!t.aircraft.enuVerticalTrusted, "基准未知时不应声称可信");
}

/**
 * 转速必须由「本次测量 − 上次**测量**」算出，不能用滤波后的相位。
 *
 * 构造：相位以每秒 6° 稳定增长（= 1 rpm），采样间隔 1 s。
 * 若用滤波值当基准，增量会被滤波滞后放大，rpm 系统性偏大。
 */
static void Test_Rpm_UsesMeasurementDelta(void)
{
    WtTelemetry t;
    uint64_t now = 10000;
    int i;

    printf("Test_Rpm_UsesMeasurementDelta\n");

    MakeTelemetry(&t, WT_ALT_DATUM_MSL);

    /* 喂入稳定转动：每秒 +6°，持续若干次让转速低通收敛 */
    for (i = 0; i < 40; i++) {
        WtTelemetry_UpdatePhaseEstimate(&t, (double)i * 6.0, now);
        now += 1000;
    }

    /*
     * 6°/s = 1 rpm。容差给 0.1 —— 低通（0.7/0.3）需要若干次迭代收敛，
     * 但不应收敛到一个系统性偏大的值。
     *
     * 用滤波值算增量时，这里会稳定地偏大（滤波滞后被计入每次增量）。
     */
    CHECK_NEAR(t.rotor.rpm, 1.0, 0.1, "6°/s 应对应 1 rpm");
    CHECK(t.rotor.valid, "相位应已建立");
}

/**
 * 长时间没有有效转动 → 转速必须归零。
 *
 * 原先的判据写成「距上次**调用**超过 2 s」，而以 10Hz 调用时那个间隔恒为
 * 100 ms，分支永远进不去（死代码），残留转速会被外推下去。
 * 现在判据是「距上次**有效转动**超过 2 s」。
 */
static void Test_Rpm_ZeroesWhenStill(void)
{
    WtTelemetry t;
    uint64_t now = 10000;
    int i;

    printf("Test_Rpm_ZeroesWhenStill\n");

    MakeTelemetry(&t, WT_ALT_DATUM_MSL);

    /* 先转起来 */
    for (i = 0; i < 10; i++) {
        WtTelemetry_UpdatePhaseEstimate(&t, (double)i * 6.0, now);
        now += 1000;
    }
    CHECK(fabs(t.rotor.rpm) > 0.5, "应先有非零转速（得到 %.3f）", t.rotor.rpm);

    /*
     * 然后停住：相位不再变化，但仍以 10Hz 继续喂 —— 模拟真实调用节奏。
     * 关键在于每次调用的间隔只有 100 ms，远小于 2 s 的静止超时。
     */
    {
        const double held = 54.0; /* 停在最后一个相位 */

        for (i = 0; i < 50; i++) { /* 50 × 100ms = 5 s */
            WtTelemetry_UpdatePhaseEstimate(&t, held, now);
            now += 100;
        }
    }

    CHECK_NEAR(t.rotor.rpm, 0.0, 1e-9,
               "静止超过 2 s 后转速应归零（残留会让云台追一个不存在的转动）");
}

/** 首帧只建立相位，不产生转速。 */
static void Test_FirstSample_NoRpm(void)
{
    WtTelemetry t;

    printf("Test_FirstSample_NoRpm\n");

    MakeTelemetry(&t, WT_ALT_DATUM_MSL);
    WtTelemetry_UpdatePhaseEstimate(&t, 123.0, 5000);

    CHECK(t.rotor.valid, "首帧应建立相位");
    CHECK_NEAR(t.rotor.phaseDeg, 123.0, 1e-9, "首帧相位应直接取测量值");
    CHECK_NEAR(t.rotor.rpm, 0.0, 1e-12, "首帧不应凭一个样本算出转速");
}

int main(void)
{
    printf("=== 遥测解算测试 ===\n");

    Test_Datum_Match();
    Test_Datum_Mismatch_Untrusted();
    Test_Datum_GeoidCorrection();
    Test_Datum_Unknown();
    Test_Rpm_UsesMeasurementDelta();
    Test_Rpm_ZeroesWhenStill();
    Test_FirstSample_NoRpm();

    if (s_failures == 0) {
        printf("全部通过\n");
        return 0;
    }
    printf("%d 项失败\n", s_failures);
    return 1;
}
