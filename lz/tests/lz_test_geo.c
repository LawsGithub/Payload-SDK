/**
 * @file lz_test_geo.c
 * @brief 几何工具的回归测试。
 *
 * 这些测试**现在就该全绿** —— 几何是固定的数学，没有待决策的部分。
 * 它们的价值在后面：一旦航点生成策略改动了坐标系约定，这里会立刻报警。
 */

#include "lz_geo.h"
#include "lz_test.h"

int main(void)
{
    LZ_CASE("正北方向的距离与方位角");
    {
        LzGeo a = { .latitudeDeg = 30.0, .longitudeDeg = 114.0, .altitudeM = 0.0 };
        /* 沿同一经线向北 0.001° ≈ 111.2 m */
        LzGeo b = { .latitudeDeg = 30.001, .longitudeDeg = 114.0, .altitudeM = 0.0 };
        LZ_CHECK_NEAR(LzGeo_DistanceM(&a, &b), 111.2, 1.0);
        LZ_CHECK_ANGLE_NEAR(LzGeo_BearingDeg(&a, &b), 0.0, 0.5);
    }

    LZ_CASE("正东方向：方位角应为 90°");
    {
        LzGeo a = { .latitudeDeg = 30.0, .longitudeDeg = 114.0, .altitudeM = 0.0 };
        LzGeo b = { .latitudeDeg = 30.0, .longitudeDeg = 114.001, .altitudeM = 0.0 };
        LZ_CHECK_ANGLE_NEAR(LzGeo_BearingDeg(&a, &b), 90.0, 0.5);
    }

    LZ_CASE("Destination 与 Bearing 互为逆运算");
    {
        LzGeo from = { .latitudeDeg = 30.5, .longitudeDeg = 114.3, .altitudeM = 50.0 };
        const double bearings[] = { 0.0, 45.0, 137.5, 270.0, 359.0 };
        for (size_t i = 0; i < sizeof(bearings) / sizeof(bearings[0]); ++i) {
            LzGeo out;
            LZ_CHECK(LzGeo_Destination(&from, bearings[i], 100.0, &out) == LZ_OK);
            /* 走过去再量回来，方位角应回到原值（容差取 0.1°，球面近似的残差） */
            /* 方位角用圆周差比较 —— 正北方向处球面计算返回的是
             * 359.999999998°，与 0° 只差 2e-9°，绝对差却是 360（见 lz_test.h） */
            LZ_CHECK_ANGLE_NEAR(LzGeo_BearingDeg(&from, &out), bearings[i], 0.1);
            LZ_CHECK_NEAR(LzGeo_DistanceM(&from, &out), 100.0, 0.5);
        }
    }

    LZ_CASE("Destination 保留高度");
    {
        LzGeo from = { .latitudeDeg = 30.0, .longitudeDeg = 114.0, .altitudeM = 42.5 };
        LzGeo out;
        LZ_CHECK(LzGeo_Destination(&from, 90.0, 10.0, &out) == LZ_OK);
        LZ_CHECK_NEAR(out.altitudeM, 42.5, 1e-9);
    }

    LZ_CASE("经度越过 180° 应被归一化");
    {
        LzGeo from = { .latitudeDeg = 0.0, .longitudeDeg = 179.999, .altitudeM = 0.0 };
        LzGeo out;
        LZ_CHECK(LzGeo_Destination(&from, 90.0, 1000.0, &out) == LZ_OK);
        LZ_CHECK(out.longitudeDeg >= -180.0 && out.longitudeDeg <= 180.0);
    }

    LZ_CASE("非法输入必须被拒绝");
    {
        LzGeo bad = { .latitudeDeg = NAN, .longitudeDeg = 114.0, .altitudeM = 0.0 };
        LZ_CHECK(!LzGeo_IsValid(&bad));
        LZ_CHECK(!LzGeo_IsValid(NULL));

        LzGeo ok = { .latitudeDeg = 30.0, .longitudeDeg = 114.0, .altitudeM = 0.0 };
        LzGeo out;
        LZ_CHECK(LzGeo_Destination(&ok, 0.0, 10.0, NULL) == LZ_ERR_PARAM);
        LZ_CHECK(LzGeo_Destination(NULL, 0.0, 10.0, &out) == LZ_ERR_PARAM);
        LZ_CHECK(LzGeo_Destination(&ok, NAN, 10.0, &out) == LZ_ERR_PARAM);
    }

    LZ_CASE("方位角归一化与夹角");
    {
        LZ_CHECK_NEAR(LzGeo_NormalizeDeg(-90.0), 270.0, 1e-9);
        LZ_CHECK_NEAR(LzGeo_NormalizeDeg(450.0), 90.0, 1e-9);
        LZ_CHECK_NEAR(LzGeo_AngleDiffDeg(350.0, 10.0), -20.0, 1e-9);
        LZ_CHECK_NEAR(LzGeo_AngleDiffDeg(10.0, 350.0), 20.0, 1e-9);

        /* 归一化的契约是左闭右开 [0,360)：一个极小的负数加上 360 会因
         * 浮点舍入恰好得到 360.0，必须收回到 0。 */
        LZ_CHECK_NEAR(LzGeo_NormalizeDeg(-1e-300), 0.0, 1e-12);
        LZ_CHECK(LzGeo_NormalizeDeg(-1e-300) < 360.0);
        LZ_CHECK(LzGeo_NormalizeDeg(360.0) < 360.0);
    }

    return LZ_TEST_SUMMARY();
}
